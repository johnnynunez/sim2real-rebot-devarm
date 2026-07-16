#!/usr/bin/env python3
"""Real-to-sim bridge (real side) for the reBot Arm.

Reads joint positions from the physical arm (Damiao dm-serial or RobStride
SocketCAN; passive: request_feedback only, motors are NEVER enabled) and
streams them as JSON over UDP to the Isaac Sim side (isaac/02_start_mirror.py).

The arm stays fully limp — move it by hand and watch the sim mirror it.

Usage:
    python scripts/real_to_sim_bridge.py [--rate 50] [--port 5801]
    python scripts/real_to_sim_bridge.py --vendor robstride --channel can0
"""
import argparse
import json
import socket
import time

from rebot_vendor import add_vendor_args, make_controller_and_motors, read_positions


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    add_vendor_args(ap)
    ap.add_argument("--rate", type=float, default=50.0, help="poll rate in Hz")
    ap.add_argument("--port", type=int, default=5801, help="UDP target port")
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()

    ctrl, motors = make_controller_and_motors(args)

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
            q = {str(mid): pos for mid, pos in read_positions(args.vendor, ctrl, motors).items()}
            if len(q) == len(motors):
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
