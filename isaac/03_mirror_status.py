# Runs INSIDE Isaac Sim via scripts/isaacsim_client.py.
# Prints mirror status: packets received, last real q, current sim q.
#
#   python scripts/isaacsim_client.py --file isaac/03_mirror_status.py

import numpy as np

print("pkts received:", state["pkts"])
print("err:", state.get("err"))
print("last real q:", state["last_q"])
cur = art.get_dof_positions().numpy()[0]
print("sim q now:", np.round(cur, 4).tolist())
print("sim tgt now:", np.round(state["tgt"], 4).tolist())
