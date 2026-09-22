#!/usr/bin/env python3
import argparse
import time

from scrobot_protocol import (
    SerialClient, MOTOR_IDS, TYPE_SYSID_SAMPLE, TYPE_ERROR,
    decode_sysid_sample, decode_error,
)


def main():
    ap = argparse.ArgumentParser(description="Run one motor at fixed open-loop duty")
    ap.add_argument("--port", required=True)
    ap.add_argument("--motor", choices=MOTOR_IDS, required=True)
    ap.add_argument("--duty", type=float, required=True)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--baud", type=int, default=1_000_000)
    args = ap.parse_args()

    if not -1.0 <= args.duty <= 1.0:
        raise SystemExit("duty must be between -1 and +1")

    c = SerialClient(args.port, args.baud)
    motor_id = MOTOR_IDS[args.motor]
    print(f"ARM + SYSID {args.motor}, duty={args.duty:+.3f}, duration={args.duration:.1f}s")

    c.arm()
    start = time.monotonic()
    next_refresh = start

    try:
        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            if now >= next_refresh:
                c.sysid(motor_id, args.duty)
                next_refresh += 0.05

            for frame in c.read_frames(0.01):
                if frame.msg_type == TYPE_SYSID_SAMPLE:
                    s = decode_sysid_sample(frame.payload)
                    if s["motor_id"] == motor_id:
                        print(f'tick={s["control_tick"]:8d} duty={s["duty"]:+.3f} rpm={s["rpm"]:8.2f} count={s["encoder_count"]:11d}')
                elif frame.msg_type == TYPE_ERROR:
                    print("STM32 ERROR:", decode_error(frame.payload))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            c.sysid_stop()
            c.disarm()
            time.sleep(0.05)
        finally:
            c.close()


if __name__ == "__main__":
    main()
