#!/usr/bin/env python3
"""Closed-loop PIDF validation using the PIDF gains already loaded in STM32.

The test does NOT change PIDF gains.

Example:
    python pid_validate.py --port /dev/ttyAMA0 --motor WR

By default each motor is stepped to its nominal working speed:
    WR/WL = 95 RPM, BR/BL = 390 RPM, CV = 80 RPM

Use --rpm only when an explicit validation reference is required.

Sequence:
    0 RPM -> target RPM -> 0 RPM

A 100 Hz feedback log is saved to tools/data/.
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

from scrobot_protocol import (
    SerialClient,
    MOTOR_IDS,
    TYPE_FEEDBACK,
    TYPE_PID_RESPONSE,
    TYPE_ERROR,
    decode_feedback,
    decode_pid_response,
    decode_error,
    status_names,
    STATUS_ARMED,
    STATUS_ESTOP,
    STATUS_COMM_TIMEOUT,
)


RPM_LIMITS = {
    "WR": 100.0,
    "WL": 100.0,
    "BR": 400.0,
    "BL": 400.0,
    "CV": 80.0,
}

NOMINAL_RPM = {
    "WR": 95.0,
    "WL": 95.0,
    "BR": 390.0,
    "BL": 390.0,
    "CV": 80.0,
}


def default_output(motor: str, rpm: float) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rpm_tag = f"{rpm:+.1f}".replace("+", "p").replace("-", "m").replace(".", "_")
    return Path("data") / f"{motor}_pid_validation_{rpm_tag}rpm_{stamp}.csv"


def command_for_motor(motor: str, rpm: float) -> dict[str, float]:
    values = {"wr": 0.0, "wl": 0.0, "br": 0.0, "bl": 0.0, "cv": 0.0}
    values[motor.lower()] = rpm
    return values


def get_current_pid(client: SerialClient, motor: str, timeout: float = 0.75) -> dict:
    motor_id = MOTOR_IDS[motor]

    client.drain(0.05)
    client.pid_get(motor_id)

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        for frame in client.read_frames(0.05):
            if frame.msg_type == TYPE_ERROR:
                raise RuntimeError(f"STM32 ERROR: {decode_error(frame.payload)}")

            if frame.msg_type != TYPE_PID_RESPONSE:
                continue

            pid = decode_pid_response(frame.payload)
            if pid["motor_id"] == motor_id:
                return pid

    raise TimeoutError(f"Timed out reading PIDF parameters for {motor}")



def wait_for_armed(client: SerialClient, timeout: float = 0.75) -> dict:
    """Wait for FEEDBACK confirming the STM32 is actually armed."""
    deadline = time.monotonic() + timeout
    last_fb = None

    while time.monotonic() < deadline:
        for frame in client.read_frames(0.05):
            if frame.msg_type == TYPE_ERROR:
                err = decode_error(frame.payload)
                raise RuntimeError(f"STM32 ERROR while arming: {err}")

            if frame.msg_type != TYPE_FEEDBACK:
                continue

            fb = decode_feedback(frame.payload)
            last_fb = fb

            if fb["status"] & STATUS_ESTOP:
                names = ",".join(status_names(fb["status"])) or "NONE"
                raise RuntimeError(
                    f"Cannot arm: STM32 reports E-stop active; status={names}"
                )

            if fb["status"] & STATUS_ARMED:
                return fb

    if last_fb is None:
        raise TimeoutError("Timed out waiting for STM32 FEEDBACK after ARM")

    names = ",".join(status_names(last_fb["status"])) or "NONE"
    raise RuntimeError(
        f"STM32 did not enter ARMED state; last status={names}"
    )


def main():
    ap = argparse.ArgumentParser(
        description="Validate the currently loaded STM32 PIDF controller with an RPM step"
    )
    ap.add_argument("--port", required=True)
    ap.add_argument("--motor", choices=MOTOR_IDS, required=True)
    ap.add_argument(
        "--rpm",
        type=float,
        help="Validation reference RPM. Defaults to the motor nominal working speed.",
    )
    ap.add_argument("--pre", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--post", type=float, default=2.0)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--output", type=Path)

    args = ap.parse_args()

    if args.pre < 0.0 or args.duration <= 0.0 or args.post < 0.0:
        raise SystemExit("--pre/--post must be >= 0 and --duration must be > 0")

    target_rpm = args.rpm if args.rpm is not None else NOMINAL_RPM[args.motor]

    limit = RPM_LIMITS[args.motor]
    if abs(target_rpm) > limit:
        raise SystemExit(
            f"{args.motor} reference must be within +/-{limit:.0f} RPM"
        )

    output = args.output or default_output(args.motor, target_rpm)
    output.parent.mkdir(parents=True, exist_ok=True)

    client = SerialClient(args.port, args.baud)

    try:
        # Read and record the controller that is actually in the STM32.
        pid = get_current_pid(client, args.motor)

        print(
            f'{args.motor} current PIDF: '
            f'Kp={pid["kp"]:.9g}, '
            f'Ki={pid["ki"]:.9g}, '
            f'Kd={pid["kd"]:.9g}, '
            f'Tf={pid["tf"]:.9g} s'
        )

        fields = [
            "host_time_s",
            "control_tick",
            "frame_seq",
            "last_setpoint_seq",
            "motor",
            "reference_rpm",
            "measured_rpm",
            "encoder_count",
            "status",
            "kp",
            "ki",
            "kd",
            "tf_s",
        ]

        client.disarm()
        time.sleep(0.05)
        client.drain(0.05)
        client.arm()
        armed_fb = wait_for_armed(client)
        armed_names = ",".join(status_names(armed_fb["status"])) or "NONE"
        print(f"STM32 arm confirmed: status={armed_names}")

        # Send one zero setpoint immediately after arm confirmation so the
        # 200 ms firmware command watchdog is refreshed before logging starts.
        client.setpoint()
        time.sleep(0.02)

        t0 = time.monotonic()

        with output.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()

            def run_segment(reference: float, duration: float):
                command = command_for_motor(args.motor, reference)
                start = time.monotonic()
                next_command = start
                last_fb = None

                while time.monotonic() - start < duration:
                    now = time.monotonic()

                    # Firmware command timeout is 200 ms; refresh at 20 Hz.
                    if now >= next_command:
                        client.setpoint(**command)
                        next_command += 0.05

                    for frame in client.read_frames(0.01):
                        if frame.msg_type == TYPE_ERROR:
                            err = decode_error(frame.payload)
                            if last_fb is None:
                                status_text = "no recent FEEDBACK"
                            else:
                                status_text = (
                                    ",".join(status_names(last_fb["status"]))
                                    or "NONE"
                                )
                            raise RuntimeError(
                                f"STM32 ERROR: {err}; "
                                f"last feedback status={status_text}"
                            )

                        if frame.msg_type != TYPE_FEEDBACK:
                            continue

                        fb = decode_feedback(frame.payload)
                        last_fb = fb

                        if not (fb["status"] & STATUS_ARMED):
                            status_text = ",".join(status_names(fb["status"])) or "NONE"
                            if fb["status"] & STATUS_ESTOP:
                                reason = "E-stop became active"
                            elif fb["status"] & STATUS_COMM_TIMEOUT:
                                reason = "command timeout disarmed STM32"
                            else:
                                reason = "STM32 became disarmed"
                            raise RuntimeError(
                                f"{reason}; status={status_text}"
                            )

                        writer.writerow({
                            "host_time_s": time.monotonic() - t0,
                            "control_tick": fb["control_tick"],
                            "frame_seq": frame.seq,
                            "last_setpoint_seq": fb["last_setpoint_seq"],
                            "motor": args.motor,
                            "reference_rpm": reference,
                            "measured_rpm": fb["rpm"][args.motor],
                            "encoder_count": fb["counts"][args.motor],
                            "status": fb["status"],
                            "kp": pid["kp"],
                            "ki": pid["ki"],
                            "kd": pid["kd"],
                            "tf_s": pid["tf"],
                        })

            try:
                print(
                    f"Running {args.motor}: "
                    f"0 -> {target_rpm:.1f} RPM -> 0 "
                    f"({args.pre:.1f}s / {args.duration:.1f}s / {args.post:.1f}s)"
                )

                run_segment(0.0, args.pre)
                run_segment(target_rpm, args.duration)
                run_segment(0.0, args.post)

            finally:
                # Safe shutdown even if logging fails.
                try:
                    client.setpoint()
                    time.sleep(0.05)
                finally:
                    client.disarm()
                    time.sleep(0.05)

        print(f"Saved {output}")

    finally:
        client.close()


if __name__ == "__main__":
    main()
