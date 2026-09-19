#!/usr/bin/env python3
import argparse
import time

from scrobot_protocol import SerialClient, TYPE_FEEDBACK, decode_feedback

CPR = {"WR": 3468.0, "WL": 3468.0, "BR": 422.4, "BL": 422.4, "CV": 440.0}


def main():
    ap = argparse.ArgumentParser(description="Check encoder counts and RPM")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    args = ap.parse_args()

    c = SerialClient(args.port, args.baud)
    baseline = None
    last = 0.0

    print("Rotate one output shaft manually. Ctrl+C to stop.")
    try:
        while True:
            for frame in c.read_frames(0.1):
                if frame.msg_type != TYPE_FEEDBACK:
                    continue
                f = decode_feedback(frame.payload)
                if baseline is None:
                    baseline = dict(f["counts"])
                now = time.monotonic()
                if now - last < 0.2:
                    continue
                last = now
                print("\nMotor      Count       Delta      Rev(est)      RPM")
                for name in ("WR", "WL", "BR", "BL", "CV"):
                    count = f["counts"][name]
                    delta = count - baseline[name]
                    print(f"{name:>3s} {count:11d} {delta:11d} {delta/CPR[name]:11.4f} {f['rpm'][name]:9.2f}")
    except KeyboardInterrupt:
        pass
    finally:
        c.close()


if __name__ == "__main__":
    main()
