#!/usr/bin/env python3
"""Minimal TCP client for the Isaac Sim `isaacsim.code_editor.python_server`.

Sends Python source to a running Isaac Sim (launched with
`--enable isaacsim.code_editor.python_server`, TCP 127.0.0.1:8226) and prints
the JSON response. All scripts share one named execution context so state
(articulation handle, mirror subscription) persists between calls.

Usage:
    python scripts/isaacsim_client.py 'print("hello")'
    python scripts/isaacsim_client.py --file isaac/02_start_mirror.py
    python scripts/isaacsim_client.py --file isaac/01_load_arm_stage.py \
        --arg usd_path=/abs/path/arm.usda --timeout 180
"""
import argparse
import asyncio
import json
import sys

CONTEXT = "sim2real"


async def send(host: str, port: int, payload: str, timeout: float) -> dict:
    reader, writer = await asyncio.wait_for(asyncio.open_connection(host, port), timeout=timeout)
    writer.write(payload.encode())
    writer.write_eof()
    data = await asyncio.wait_for(reader.read(), timeout=timeout)
    writer.close()
    return json.loads(data.decode())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("code", nargs="?", help="inline Python code")
    ap.add_argument("--file", help="send a .py file instead of inline code")
    ap.add_argument("--arg", action="append", default=[], help="inject key=value as a variable")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8226)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    if args.file:
        source = open(args.file, encoding="utf-8").read()
    elif args.code:
        source = args.code
    else:
        source = sys.stdin.read()

    if args.arg:
        inject = []
        for kv in args.arg:
            key, _, value = kv.partition("=")
            inject.append(f"{key} = {value!r}")
        source = "\n".join(inject) + "\n" + source

    envelope = json.dumps({"code": source, "context": CONTEXT})
    resp = asyncio.run(send(args.host, args.port, envelope, args.timeout))

    if resp.get("output"):
        print(resp["output"], end="")
    if resp.get("status") != "ok":
        print(f"ERROR: {resp.get('ename')}: {resp.get('evalue')}", file=sys.stderr)
        for line in resp.get("traceback", []):
            print(line, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
