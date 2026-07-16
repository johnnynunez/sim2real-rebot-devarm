#!/usr/bin/env python3
"""Print joint positions of the real reBot Arm (sanity check).

Passive read: uses request_feedback only — motors are NEVER enabled.

Usage:
    python scripts/read_joints.py [--cycles 10] [--vendor robstride --channel can0]
"""
import argparse
import time

from rebot_vendor import add_vendor_args, make_controller_and_motors, read_positions


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    add_vendor_args(ap)
    ap.add_argument("--cycles", type=int, default=10)
    args = ap.parse_args()

    ctrl, motors = make_controller_and_motors(args)
    try:
        for _ in range(args.cycles):
            q = read_positions(args.vendor, ctrl, motors)
            line = [f"j{mid}={q[mid]:+.4f}" if mid in q else f"j{mid}=NONE" for mid in motors]
            print("  ".join(line), flush=True)
            time.sleep(0.1)
    finally:
        ctrl.shutdown()


if __name__ == "__main__":
    main()
