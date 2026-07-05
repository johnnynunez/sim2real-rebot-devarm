#!/usr/bin/env python3
"""Print joint positions of the real reBot Arm B601-DM (sanity check).

Passive read: uses request_feedback only — motors are NEVER enabled.

Usage:
    python scripts/read_joints.py [--cycles 10]
"""
import argparse
import time

from motorbridge import Controller

MODELS = {1: "4340", 2: "4340", 3: "4340", 4: "4310", 5: "4310", 6: "4310", 7: "4310"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--serial-port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--cycles", type=int, default=10)
    args = ap.parse_args()

    ctrl = Controller.from_dm_serial(serial_port=args.serial_port, baud=args.baud)
    motors = {
        mid: ctrl.add_damiao_motor(motor_id=mid, feedback_id=mid + 0x10, model=model)
        for mid, model in MODELS.items()
    }
    try:
        for _ in range(args.cycles):
            line = []
            for mid, m in motors.items():
                m.request_feedback()
                time.sleep(0.002)
                st = m.get_state()
                line.append(f"j{mid}={st.pos:+.4f}" if st else f"j{mid}=NONE")
            print("  ".join(line), flush=True)
            time.sleep(0.1)
    finally:
        ctrl.shutdown()


if __name__ == "__main__":
    main()
