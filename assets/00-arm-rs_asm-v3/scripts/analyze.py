"""Newton vs PhysX statistical analysis + joint curves from ts_{newton,physx}.json."""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import stats

N = json.load(open("/tmp/armtrack/ts_newton.json"))
P = json.load(open("/tmp/armtrack/ts_physx.json"))
names = N["dof_names"]
tsN, tsP = np.array(N["ts"]), np.array(P["ts"])
qN, qP = np.array(N["qs"]), np.array(P["qs"])
tgtN, tgtP = np.array(N["targets"]), np.array(P["targets"])
L = min(len(tsN), len(tsP))
tsN, qN, tgtN = tsN[:L], qN[:L], tgtN[:L]
qP, tgtP = qP[:L], tgtP[:L]
assert np.allclose(tgtN, tgtP, atol=1e-6), "targets differ between runs!"

OUT = "/tmp/armtrack/analysis"
import os; os.makedirs(OUT, exist_ok=True)
DT = N["dt"]
steady = tsN > 2.0  # skip first period (0.5 Hz)

# ---------- statistics table ----------
rows = []
for j, nm in enumerate(names):
    eN = qN[:, j] - tgtN[:, j]
    eP = qP[:, j] - tgtP[:, j]
    d = qN[:, j] - qP[:, j]
    sN_, sP_, sd = eN[steady], eP[steady], d[steady]
    r = np.corrcoef(qN[steady, j], qP[steady, j])[0, 1]
    # paired test on |error| — is one engine measurably better?
    w = stats.wilcoxon(np.abs(sN_), np.abs(sP_), zero_method="zsplit")
    rows.append({
        "joint": nm,
        "rmsN": float(np.sqrt((sN_**2).mean())), "rmsP": float(np.sqrt((sP_**2).mean())),
        "maxN": float(np.abs(sN_).max()), "maxP": float(np.abs(sP_).max()),
        "p95N": float(np.percentile(np.abs(sN_), 95)), "p95P": float(np.percentile(np.abs(sP_), 95)),
        "biasN": float(sN_.mean()), "biasP": float(sP_.mean()),
        "diff_mean": float(sd.mean()), "diff_std": float(sd.std()), "diff_max": float(np.abs(sd).max()),
        "corr": float(r), "wilcoxon_p": float(w.pvalue),
    })

with open(f"{OUT}/stats.json", "w") as f:
    json.dump(rows, f, indent=1)

hdr = f"{'joint':12} {'rmsN':>8} {'rmsP':>8} {'p95N':>8} {'p95P':>8} {'ΔN-P mean':>10} {'Δ std':>8} {'Δ max':>8} {'corr':>8} {'p':>9}"
print(hdr)
for r_ in rows:
    print(f"{r_['joint']:12} {r_['rmsN']:8.4f} {r_['rmsP']:8.4f} {r_['p95N']:8.4f} {r_['p95P']:8.4f} "
          f"{r_['diff_mean']:10.5f} {r_['diff_std']:8.5f} {r_['diff_max']:8.5f} {r_['corr']:8.5f} {r_['wilcoxon_p']:9.2e}")

# ---------- fig 1: tracking curves (8 panels) ----------
fig, axes = plt.subplots(4, 2, figsize=(14, 16), sharex=True)
for j, ax in enumerate(axes.flat):
    ax.plot(tsN, tgtN[:, j], "k--", lw=1, label="target", alpha=0.7)
    ax.plot(tsN, qN[:, j], color="tab:blue", lw=1.2, label="Newton")
    ax.plot(tsN, qP[:, j], color="tab:orange", lw=1.2, label="PhysX", alpha=0.85)
    ax.set_title(f"{names[j]}", fontsize=11)
    ax.set_ylabel("pos (rad|m)")
    if j == 0: ax.legend(loc="upper right", fontsize=9)
axes[-1, 0].set_xlabel("t (s)"); axes[-1, 1].set_xlabel("t (s)")
fig.suptitle("Sinusoid tracking 0.5 Hz — Newton vs PhysX (converter asset, PhysX gains)", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.98])
fig.savefig(f"{OUT}/fig1_tracking.png", dpi=110)
plt.close(fig)

# ---------- fig 2: tracking error curves ----------
fig, axes = plt.subplots(4, 2, figsize=(14, 16), sharex=True)
for j, ax in enumerate(axes.flat):
    ax.plot(tsN, (qN[:, j]-tgtN[:, j])*1e3, color="tab:blue", lw=1, label="Newton")
    ax.plot(tsN, (qP[:, j]-tgtP[:, j])*1e3, color="tab:orange", lw=1, label="PhysX", alpha=0.85)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title(names[j], fontsize=11); ax.set_ylabel("err (mrad|mm)")
    if j == 0: ax.legend(fontsize=9)
axes[-1, 0].set_xlabel("t (s)"); axes[-1, 1].set_xlabel("t (s)")
fig.suptitle("Tracking error — Newton vs PhysX", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.98])
fig.savefig(f"{OUT}/fig2_error.png", dpi=110)
plt.close(fig)

# ---------- fig 3: Newton minus PhysX difference ----------
fig, axes = plt.subplots(4, 2, figsize=(14, 16), sharex=True)
for j, ax in enumerate(axes.flat):
    ax.plot(tsN, (qN[:, j]-qP[:, j])*1e3, color="tab:green", lw=1)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title(names[j], fontsize=11); ax.set_ylabel("N−P (mrad|mm)")
axes[-1, 0].set_xlabel("t (s)"); axes[-1, 1].set_xlabel("t (s)")
fig.suptitle("Direct engine difference (Newton − PhysX), same targets", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.98])
fig.savefig(f"{OUT}/fig3_diff.png", dpi=110)
plt.close(fig)

# ---------- fig 4: step response joint1 ----------
stN = np.array(N["step_traj"]); stP = np.array(P["step_traj"])
Ls = min(len(stN), len(stP)); tstep = np.arange(Ls)*DT
final = None
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(tstep, stN[:Ls], color="tab:blue", lw=1.5, label=f"Newton (settle {N['step_settle_time_s']:.3f}s)")
ax.plot(tstep, stP[:Ls], color="tab:orange", lw=1.5, label=f"PhysX (settle {P['step_settle_time_s']:.3f}s)")
ax.axhline(stN[-1], color="k", ls="--", lw=0.8, label="target")
ax.set_xlim(0, 1.0); ax.set_xlabel("t (s)"); ax.set_ylabel("joint1 (rad)")
ax.set_title(f"Step response 0.5 rad — overshoot N {N['step_overshoot_pct']:.1f}% / P {P['step_overshoot_pct']:.1f}%")
ax.legend(); fig.tight_layout()
fig.savefig(f"{OUT}/fig4_step.png", dpi=110)
plt.close(fig)

# ---------- fig 5: Bland-Altman + error distributions ----------
fig, axes = plt.subplots(2, 4, figsize=(18, 8))
for j in range(8):
    ax = axes.flat[j]
    mean_ = (qN[steady, j] + qP[steady, j]) / 2
    diff_ = (qN[steady, j] - qP[steady, j]) * 1e3
    ax.scatter(mean_, diff_, s=1, alpha=0.3, color="tab:green")
    mu, sd = diff_.mean(), diff_.std()
    for y, style in ((mu, "-"), (mu+1.96*sd, "--"), (mu-1.96*sd, "--")):
        ax.axhline(y, color="r", ls=style, lw=0.8)
    ax.set_title(f"{names[j]}  μ={mu:.2f} ±{1.96*sd:.2f}", fontsize=10)
    ax.set_xlabel("mean pos"); ax.set_ylabel("N−P (mrad|mm)")
fig.suptitle("Bland–Altman: Newton vs PhysX agreement (steady state)", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(f"{OUT}/fig5_blandaltman.png", dpi=110)
plt.close(fig)

print("FIGS SAVED to", OUT)
