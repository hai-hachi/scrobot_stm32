#!/usr/bin/env python3
import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

from scrobot_protocol import (
    SerialClient, MOTOR_IDS, TYPE_SYSID_SAMPLE, TYPE_ERROR,
    decode_sysid_sample, decode_error,
)


def default_output(motor: str, label: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("data") / f"{motor}_{label}_{stamp}.csv"


def run_segments(client, motor_id, segments, output: Path):
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = ["host_time_s", "control_tick", "frame_seq", "command_seq", "motor",
              "command_duty", "rpm", "encoder_count", "status"]

    client.arm()
    t0 = time.monotonic()

    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        try:
            for duty, duration in segments:
                seg_start = time.monotonic()
                next_refresh = seg_start

                while time.monotonic() - seg_start < duration:
                    now = time.monotonic()
                    if now >= next_refresh:
                        client.sysid(motor_id, duty)
                        next_refresh += 0.05

                    for frame in client.read_frames(0.01):
                        if frame.msg_type == TYPE_ERROR:
                            print("STM32 ERROR:", decode_error(frame.payload))
                            continue
                        if frame.msg_type != TYPE_SYSID_SAMPLE:
                            continue

                        sample = decode_sysid_sample(frame.payload)
                        if sample["motor_id"] != motor_id:
                            continue

                        writer.writerow({
                            "host_time_s": time.monotonic() - t0,
                            "control_tick": sample["control_tick"],
                            "frame_seq": frame.seq,
                            "command_seq": sample["command_seq"],
                            "motor": sample["motor"],
                            "command_duty": sample["duty"],
                            "rpm": sample["rpm"],
                            "encoder_count": sample["encoder_count"],
                            "status": sample["status"],
                        })
        finally:
            client.sysid_stop()
            client.disarm()
            time.sleep(0.05)

    print(f"Saved {output}")


def make_sweep(start, stop, step):
    if step == 0:
        raise ValueError("step cannot be zero")
    if (stop - start) * step < 0:
        raise ValueError("step sign does not move from start toward stop")
    values = []
    x = start
    if step > 0:
        while x <= stop + 1e-12:
            values.append(x)
            x += step
    else:
        while x >= stop - 1e-12:
            values.append(x)
            x += step
    return values


def main():
    ap = argparse.ArgumentParser(description="SCROBOT open-loop motor system identification")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    sub = ap.add_subparsers(dest="mode", required=True)

    step = sub.add_parser("step")
    step.add_argument("--motor", choices=MOTOR_IDS, required=True)
    step.add_argument("--duty", type=float, required=True)
    step.add_argument("--pre", type=float, default=2.0)
    step.add_argument("--duration", type=float, default=5.0)
    step.add_argument("--post", type=float, default=2.0)
    step.add_argument("--output", type=Path)

    sweep = sub.add_parser("sweep")
    sweep.add_argument("--motor", choices=MOTOR_IDS, required=True)
    sweep.add_argument("--start", type=float, required=True)
    sweep.add_argument("--stop", type=float, required=True)
    sweep.add_argument("--step", type=float, required=True)
    sweep.add_argument("--hold", type=float, default=1.5)
    sweep.add_argument("--settle-zero", type=float, default=1.0)
    sweep.add_argument("--output", type=Path)

    args = ap.parse_args()
    c = SerialClient(args.port, args.baud)

    try:
        motor_id = MOTOR_IDS[args.motor]

        if args.mode == "step":
            if not -1.0 <= args.duty <= 1.0:
                raise SystemExit("duty must be between -1 and +1")
            segments = [(0.0, args.pre), (args.duty, args.duration), (0.0, args.post)]
            output = args.output or default_output(args.motor, f"step_{args.duty:+.3f}".replace("+", "p").replace("-", "m"))
        else:
            values = make_sweep(args.start, args.stop, args.step)
            if any(not -1.0 <= value <= 1.0 for value in values):
                raise SystemExit("all sweep duty values must be between -1 and +1")
            segments = [(0.0, args.settle_zero)]
            for value in values:
                segments.append((value, args.hold))
            segments.append((0.0, args.settle_zero))
            output = args.output or default_output(args.motor, "sweep")

        run_segments(c, motor_id, segments, output)
    finally:
        c.close()


if __name__ == "__main__":
    main()
