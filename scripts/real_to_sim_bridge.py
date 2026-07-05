#!/usr/bin/env python3
"""Real-to-sim bridge (real side) for the reBot Arm B601-DM.

Reads joint positions from the physical arm over the Damiao serial bridge
(passive: request_feedback only, motors are NEVER enabled) and streams them
as JSON over UDP to the Isaac Sim side (isaac/02_start_mirror.py).

The arm stays fully limp — move it by hand and watch the sim mirror it.

Usage:
    python scripts/real_to_sim_bridge.py [--rate 50] [--port 5801]
"""
import argparse
import json
import socket
import time

from motorbridge import Controller

# Motor model per joint (verified via register dump on our unit: motors 1-3
# report tau_max=28 -> DM4340, motors 4-7 report vel_max=30/tau_max=10 -> DM4310).
# Motor 7 is the gripper.
MODELS = {1: "4340", 2: "4340", 3: "4340", 4: "4310", 5: "4310", 6: "4310", 7: "4310"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--serial-port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--rate", type=float, default=50.0, help="poll rate in Hz")
    ap.add_argument("--port", type=int, default=5801, help="UDP target port")
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()

    ctrl = Controller.from_dm_serial(serial_port=args.serial_port, baud=args.baud)
    motors = {
        mid: ctrl.add_damiao_motor(motor_id=mid, feedback_id=mid + 0x10, model=model)
        for mid, model in MODELS.items()
    }

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.host, args.port)
    dt = 1.0 / args.rate
    n_sent = 0
    t_report = time.monotonic()

    print(f"Streaming real arm joints -> udp://{args.host}:{args.port} at {args.rate:.0f} Hz")
    print("Motors are passive (never enabled). Move the arm by hand. Ctrl+C to stop.")
    try:
        while True:
            t0 = time.monotonic()
            q = {}
            for mid, m in motors.items():
                m.request_feedback()
            # tiny settle so replies land before reading states
            time.sleep(0.001)
            for mid, m in motors.items():
                st = m.get_state()
                if st is not None:
                    q[str(mid)] = st.pos
            if len(q) == len(MODELS):
                sock.sendto(json.dumps({"t": time.time(), "q": q}).encode(), dst)
                n_sent += 1
            if time.monotonic() - t_report >= 2.0:
                vals = "  ".join(f"j{k}={v:+.3f}" for k, v in sorted(q.items()))
                print(f"[{n_sent} pkts] {vals}", flush=True)
                t_report = time.monotonic()
            sleep = dt - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ctrl.shutdown()


if __name__ == "__main__":
    main()
