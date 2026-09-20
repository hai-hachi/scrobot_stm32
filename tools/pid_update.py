#!/usr/bin/env python3
"""Read/update runtime PIDF parameters on the STM32 through the Raspberry Pi.

Examples
--------
Read one motor:
    python pid_update.py --port /dev/ttyAMA0 get --motor WR

Read all motors:
    python pid_update.py --port /dev/ttyAMA0 get-all

Set one motor:
    python pid_update.py --port /dev/ttyAMA0 set --motor WR \
        --kp 12.3 --ki 45.6 --kd 0.12 --tf 0.01

Load all five from MATLAB's pidf_autotune_results.csv:
    python pid_update.py --port /dev/ttyAMA0 load-csv \
        --file pidf_autotune_results.csv

PID values changed through UART are runtime-only. They revert to the
app_config.h defaults after an STM32 reset/power cycle.
"""

import argparse
import csv
from pathlib import Path
import time

from scrobot_protocol import (
    SerialClient,
    MOTOR_IDS,
    TYPE_PID_RESPONSE,
    TYPE_ERROR,
    decode_pid_response,
    decode_error,
)


def wait_pid_response(client, motor_id: int, timeout: float = 0.75):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        for frame in client.read_frames(0.05):
            if frame.msg_type == TYPE_ERROR:
                raise RuntimeError(f"STM32 ERROR: {decode_error(frame.payload)}")

            if frame.msg_type != TYPE_PID_RESPONSE:
                continue

            response = decode_pid_response(frame.payload)
            if response["motor_id"] == motor_id:
                return response

    raise TimeoutError("Timed out waiting for STM32 PID response")


def get_pid(client, motor: str):
    motor_id = MOTOR_IDS[motor]
    client.pid_get(motor_id)
    return wait_pid_response(client, motor_id)


def set_pid(client, motor: str, kp: float, ki: float, kd: float, tf: float):
    values = (kp, ki, kd, tf)
    if any(value < 0.0 for value in values):
        raise ValueError("Kp, Ki, Kd and Tf must be >= 0")

    motor_id = MOTOR_IDS[motor]

    client.pid_set(motor_id, kp, ki, kd, tf)
    response = wait_pid_response(client, motor_id)

    # Read back once more to verify what the STM32 actually holds.
    client.pid_get(motor_id)
    verified = wait_pid_response(client, motor_id)

    return response, verified


def print_pid(pid):
    print(
        f'{pid["motor"]}: '
        f'Kp={pid["kp"]:.9g}  '
        f'Ki={pid["ki"]:.9g}  '
        f'Kd={pid["kd"]:.9g}  '
        f'Tf={pid["tf"]:.9g} s'
    )


def load_csv(client, path: Path):
    required = {
        "Motor",
        "Kp_STM32",
        "Ki_STM32",
        "Kd_STM32",
        "Tf_STM32_s",
    }

    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise ValueError(f"{path} contains no PIDF rows")

    missing = required - set(rows[0])
    if missing:
        raise ValueError(
            "CSV is missing columns: " + ", ".join(sorted(missing))
        )

    print(f"Loading PIDF parameters from: {path}")

    for row in rows:
        motor = row["Motor"].strip().upper()
        if motor not in MOTOR_IDS:
            raise ValueError(f"Unknown motor in CSV: {motor}")

        kp = float(row["Kp_STM32"])
        ki = float(row["Ki_STM32"])
        kd = float(row["Kd_STM32"])
        tf = float(row["Tf_STM32_s"])

        print(f"\nSetting {motor}...")
        _, verified = set_pid(client, motor, kp, ki, kd, tf)
        print_pid(verified)

    print("\nAll CSV PIDF parameters were written and read back successfully.")


def main():
    parser = argparse.ArgumentParser(
        description="Update/read SCROBOT STM32 runtime PIDF parameters"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=1_000_000)

    sub = parser.add_subparsers(dest="command", required=True)

    get = sub.add_parser("get", help="Read PIDF parameters for one motor")
    get.add_argument("--motor", choices=MOTOR_IDS, required=True)

    sub.add_parser("get-all", help="Read PIDF parameters for all motors")

    set_cmd = sub.add_parser("set", help="Set PIDF parameters for one motor")
    set_cmd.add_argument("--motor", choices=MOTOR_IDS, required=True)
    set_cmd.add_argument("--kp", type=float, required=True)
    set_cmd.add_argument("--ki", type=float, required=True)
    set_cmd.add_argument("--kd", type=float, default=0.0)
    set_cmd.add_argument("--tf", type=float, default=0.01)

    csv_cmd = sub.add_parser(
        "load-csv",
        help="Set all PIDF parameters from MATLAB pidf_autotune_results.csv",
    )
    csv_cmd.add_argument("--file", type=Path, required=True)

    args = parser.parse_args()

    client = SerialClient(args.port, args.baud)

    try:
        # Clear any old queued feedback/response frames.
        client.drain(0.05)

        if args.command == "get":
            print_pid(get_pid(client, args.motor))

        elif args.command == "get-all":
            for motor in MOTOR_IDS:
                print_pid(get_pid(client, motor))

        elif args.command == "set":
            client.disarm()
            time.sleep(0.05)
            client.drain(0.05)

            print(f"Writing {args.motor}...")
            _, verified = set_pid(
                client,
                args.motor,
                args.kp,
                args.ki,
                args.kd,
                args.tf,
            )
            print("Verified:")
            print_pid(verified)

        else:
            client.disarm()
            time.sleep(0.05)
            client.drain(0.05)

            load_csv(client, args.file)

    finally:
        client.close()


if __name__ == "__main__":
    main()
