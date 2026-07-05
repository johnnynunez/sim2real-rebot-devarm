#!/usr/bin/env python3
"""Thin CLI client for the reBot Arm daemon (rebot_daemon.py).

Usage:
    rebot_client.py health | state | pose | enable | disable | estop
    rebot_client.py move-joints Q1 Q2 Q3 Q4 Q5 Q6 [--vlim 0.5]
    rebot_client.py move-pose X Y Z [--rpy R P Y] [--vlim 0.5]
    rebot_client.py gripper open|close|POS_RAD
"""
import argparse
import json
import sys
import urllib.request

BASE = "http://127.0.0.1:5810"


def call(method, path, body=None):
    req = urllib.request.Request(
        BASE + path,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"},
        method=method)
    try:
        with urllib.request.urlopen(req, timeout=40) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())
    except urllib.error.URLError as e:
        return {"error": f"daemon unreachable at {BASE}: {e.reason}"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd")
    ap.add_argument("args", nargs="*")
    ap.add_argument("--vlim", type=float, default=0.5)
    ap.add_argument("--rpy", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    a = ap.parse_args()

    simple = {"health": ("GET", "/health"), "state": ("GET", "/state"),
              "pose": ("GET", "/pose"), "enable": ("POST", "/enable"),
              "disable": ("POST", "/disable"), "estop": ("POST", "/estop")}
    if a.cmd in simple:
        m, p = simple[a.cmd]
        out = call(m, p, {} if m == "POST" else None)
    elif a.cmd == "move-joints":
        out = call("POST", "/move_joints",
                   {"q": [float(x) for x in a.args], "vlim": a.vlim})
    elif a.cmd == "move-pose":
        out = call("POST", "/move_pose",
                   {"xyz": [float(x) for x in a.args[:3]],
                    "rpy": a.rpy, "vlim": a.vlim})
    elif a.cmd == "gripper":
        arg = a.args[0]
        body = {"action": arg} if arg in ("open", "close") else {"pos": float(arg)}
        out = call("POST", "/gripper", body)
    else:
        ap.error(f"unknown command {a.cmd}")
    print(json.dumps(out, indent=2))
    sys.exit(1 if "error" in out else 0)


if __name__ == "__main__":
    main()
