# Runs INSIDE Isaac Sim via scripts/isaacsim_client.py.
# Opens the reBot DevArm USD stage. Inject `usd_path` with --arg.
#
#   python scripts/isaacsim_client.py --timeout 180 \
#       --arg usd_path=/abs/path/00-arm-rs_asm-v3.usda \
#       --file isaac/01_load_arm_stage.py

import omni.usd

assert "usd_path" in dir(), "pass --arg usd_path=/abs/path/to/arm.usda"

ctx = omni.usd.get_context()
ok = ctx.open_stage(usd_path)
assert ok, f"failed to open stage: {usd_path}"
stage = ctx.get_stage()
print(f"Opened: {usd_path}")
print(f"Prims: {len(list(stage.TraverseAll()))}")
