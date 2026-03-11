#!/usr/bin/env python3
"""
yolo_inference_plot.py
======================
Plot YOLO11 nano inference time (Model1 + Model2): Idle vs Full System
"""

import json, sys
import numpy as np
import matplotlib.pyplot as plt

FILE_IDLE = "ai_stats_log_idle.csv"
FILE_FULL = "ai_stats_log_full.csv"
OUT_FILE  = "yolo_inference_time.png"

def parse(path):
    ms1, ms2 = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('%') or not line:
                continue
            idx = line.index(',')
            try:
                d = json.loads(line[idx+1:])
                ms1.append(d['inference_ms'])
                ms2.append(d['inference_ms2'])
            except:
                pass
    return np.array(ms1), np.array(ms2)

try:
    idle_ms1, idle_ms2 = parse(FILE_IDLE)
    full_ms1, full_ms2 = parse(FILE_FULL)
except FileNotFoundError as e:
    print(f"ERROR: {e}")
    sys.exit(1)

idle_t = np.arange(len(idle_ms1)) * 10
full_t = np.arange(len(full_ms1)) * 10

C_IDLE1 = "#2166ac"   # blue  - Model1 Idle
C_FULL1 = "#d6604d"   # red   - Model1 Full
C_IDLE2 = "#4dac26"   # green - Model2 Idle
C_FULL2 = "#e08214"   # orange- Model2 Full
C_GRID  = "#dddddd"

plt.rcParams.update({
    "font.family": "serif", "font.size": 11,
    "axes.titlesize": 12, "axes.labelsize": 11,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.dpi": 150,
})

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9), facecolor="white",
                                sharex=False)
fig.suptitle("YOLO11 nano Inference Time: Idle vs Full System",
             fontsize=13, fontweight="bold", y=0.98)

for ax, idle_ms, full_ms, c_idle, c_full, model_label in [
    (ax1, idle_ms1, full_ms1, C_IDLE1, C_FULL1, "Model1 (Person/COCO)  —  skip every 30 frames"),
    (ax2, idle_ms2, full_ms2, C_IDLE2, C_FULL2, "Model2 (Door)  —  skip every 72 frames"),
]:
    idle_t_local = np.arange(len(idle_ms)) * 10
    full_t_local = np.arange(len(full_ms)) * 10

    ax.set_facecolor("white")
    ax.grid(color=C_GRID, linestyle="--", linewidth=0.7, zorder=0)
    ax.set_axisbelow(True)

    ax.plot(idle_t_local, idle_ms, color=c_idle, linewidth=1.0, alpha=0.85, label="Idle")
    ax.plot(full_t_local, full_ms, color=c_full, linewidth=1.0, alpha=0.85, label="Full System")
    ax.axhline(idle_ms.mean(), color=c_idle, linestyle="--", linewidth=1.5,
               label=f"Idle mean = {idle_ms.mean():.1f} ms  ({1000/idle_ms.mean():.2f} FPS)")
    ax.axhline(full_ms.mean(), color=c_full, linestyle="--", linewidth=1.5,
               label=f"Full mean = {full_ms.mean():.1f} ms  ({1000/full_ms.mean():.2f} FPS)")

    ax.set_title(model_label, fontweight="bold", loc="left", pad=6)
    ax.set_ylabel("Inference Time (ms)")
    ax.legend(fontsize=9, framealpha=0.9, loc="upper right")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

ax2.set_xlabel("Time (s)")
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(OUT_FILE, dpi=180, bbox_inches="tight", facecolor="white")
print(f"Saved: {OUT_FILE}")

print("\n" + "="*65)
print(f"{'Condition':<22} {'Mean (ms)':>10} {'Std (ms)':>9} {'Max':>7} {'FPS':>6}")
print("="*65)
for label, arr in [("Model1 Idle",  idle_ms1), ("Model1 Full",  full_ms1),
                   ("Model2 Idle",  idle_ms2), ("Model2 Full",  full_ms2)]:
    print(f"{label:<22} {arr.mean():>10.1f} {arr.std():>9.1f} {arr.max():>7.1f} {1000/arr.mean():>6.2f}")
print("="*65)