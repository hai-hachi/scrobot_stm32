#!/usr/bin/env python3
import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

from scrobot_protocol import (
    SerialClient, MOTOR_IDS, TYPE_FEEDBACK, TYPE_PID_RESPONSE, TYPE_ERROR,
    decode_feedback, decode_pid_response, decode_error,
)


def default_output(motor: str, rpm: float) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"{rpm:+.1f}".replace("+", "p").replace("-", "m").replace(".", "_")
    return Path("data") / f"{motor}_pid_{tag}rpm_{stamp}.csv"


def command_for_motor(name: str, rpm: float):
    values = dict(WR=0.0, WL=0.0, BR=0.0, BL=0.0, CV=0.0)
    values[name] = rpm
    return values


def main():
    ap = argparse.ArgumentParser(description="Closed-loop PID step test")
    ap.add_argument("--port", required=True)
    ap.add_argument("--motor", choices=MOTOR_IDS, required=True)
    ap.add_argument("--rpm", type=float, required=True)
    ap.add_argument("--kp", type=float, required=True)
    ap.add_argument("--ki", type=float, required=True)
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--tf", type=float, default=0.01)
    ap.add_argument("--pre", type=float, default=1.0)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--post", type=float, default=1.0)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--output", type=Path)
    args = ap.parse_args()

    limits = {"WR": 100.0, "WL": 100.0, "BR": 400.0, "BL": 400.0, "CV": 80.0}
    if abs(args.rpm) > limits[args.motor]:
        raise SystemExit(f"{args.motor} limit is +/-{limits[args.motor]} RPM")

    output = args.output or default_output(args.motor, args.rpm)
    output.parent.mkdir(parents=True, exist_ok=True)

    c = SerialClient(args.port, args.baud)
    motor_id = MOTOR_IDS[args.motor]

    fields = [
        "host_time_s", "control_tick", "frame_seq", "last_setpoint_seq",
        "motor", "reference_rpm", "measured_rpm", "encoder_count", "status",
    ]

    c.pid_set(motor_id, args.kp, args.ki, args.kd, args.tf)
    c.arm()
    t0 = time.monotonic()

    def run_segment(reference, duration, writer):
        values = command_for_motor(args.motor, reference)
        start = time.monotonic()
        next_cmd = start
        while time.monotonic() - start < duration:
            now = time.monotonic()
            if now >= next_cmd:
                c.setpoint(**{k.lower(): v for k, v in values.items()})
                next_cmd += 0.05

            for frame in c.read_frames(0.01):
                if frame.msg_type == TYPE_ERROR:
                    print("STM32 ERROR:", decode_error(frame.payload))
                    continue
                if frame.msg_type == TYPE_PID_RESPONSE:
                    print("PID", decode_pid_response(frame.payload))
                    continue
                if frame.msg_type != TYPE_FEEDBACK:
                    continue

                fb = decode_feedback(frame.payload)
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
                })

    try:
        with output.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            run_segment(0.0, args.pre, writer)
            run_segment(args.rpm, args.duration, writer)
            run_segment(0.0, args.post, writer)
    finally:
        try:
            c.setpoint()
            c.disarm()
            time.sleep(0.05)
        finally:
            c.close()

    print(f"Saved {output}")


if __name__ == "__main__":
    main()
