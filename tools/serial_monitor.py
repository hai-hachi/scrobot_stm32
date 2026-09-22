#!/usr/bin/env python3
import argparse
import time

from scrobot_protocol import (
    SerialClient, TYPE_FEEDBACK, TYPE_DIAGNOSTICS, TYPE_INFO_RESPONSE, TYPE_ERROR,
    decode_feedback, decode_diagnostics, decode_info, decode_error, status_names,
)


def main():
    ap = argparse.ArgumentParser(description="Monitor SCROBOT STM32 UART v2")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--rate", type=float, default=5.0, help="console update rate")
    args = ap.parse_args()

    c = SerialClient(args.port, args.baud)
    c.info()

    last_print = 0.0
    try:
        while True:
            for frame in c.read_frames(0.1):
                if frame.msg_type == TYPE_INFO_RESPONSE:
                    print("INFO", decode_info(frame.payload))
                elif frame.msg_type == TYPE_DIAGNOSTICS:
                    print("DIAG", decode_diagnostics(frame.payload))
                elif frame.msg_type == TYPE_ERROR:
                    print("ERROR", decode_error(frame.payload))
                elif frame.msg_type == TYPE_FEEDBACK:
                    now = time.monotonic()
                    if now - last_print >= 1.0 / args.rate:
                        f = decode_feedback(frame.payload)
                        st = ",".join(status_names(f["status"])) or "NONE"
                        print(
                            f'tick={f["control_tick"]:8d} status={st:24s} '
                            f'WR={f["rpm"]["WR"]:7.2f} WL={f["rpm"]["WL"]:7.2f} '
                            f'BR={f["rpm"]["BR"]:7.2f} BL={f["rpm"]["BL"]:7.2f} '
                            f'CV={f["rpm"]["CV"]:7.2f}'
                        )
                        last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        c.close()


if __name__ == "__main__":
    main()
