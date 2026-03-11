#!/usr/bin/env python3
"""
yolo_inference_plot.py
======================
Plot YOLO11 nano inference time comparison: Idle vs Full System
Data source: ai_stats_log_idle.csv, ai_stats_log_full.csv
"""

import json, sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ────────────────────────────────────────────────────────────
#  CONFIG — change filenames here if needed
# ────────────────────────────────────────────────────────────
FILE_IDLE = "ai_stats_log_idle.csv"
FILE_FULL = "ai_stats_log_full.csv"
OUT_FILE  = "yolo_inference_time.png"

# ────────────────────────────────────────────────────────────
#  PARSE
# ────────────────────────────────────────────────────────────
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
    print("Please run this script in the same directory as the CSV files.")
    sys.exit(1)

# time axis (seconds, from sample index × 10s interval)
idle_t = np.arange(len(idle_ms1)) * 10
full_t = np.arange(len(full_ms1)) * 10

# ────────────────────────────────────────────────────────────
#  STYLE
# ────────────────────────────────────────────────────────────
C_IDLE = "#2166ac"   # blue
C_FULL = "#d6604d"   # red
C_GRID = "#dddddd"

plt.rcParams.update({
    "font.family": "serif", "font.size": 10,
    "axes.titlesize": 11, "axes.labelsize": 10,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.dpi": 150,
})

def style_ax(ax, title="", xlabel="", ylabel=""):
    ax.set_facecolor("white")
    ax.grid(color=C_GRID, linestyle="--", linewidth=0.7, zorder=0)
    ax.set_axisbelow(True)
    if title:  ax.set_title(title, fontweight="bold", loc="left", pad=6)
    if xlabel: ax.set_xlabel(xlabel)
    if ylabel: ax.set_ylabel(ylabel)

# ────────────────────────────────────────────────────────────
#  FIGURE
# ────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 10), facecolor="white")
gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.52, wspace=0.35,
                        top=0.93, bottom=0.07, left=0.07, right=0.97)

fig.text(0.5, 0.97, "YOLO11 nano Inference Time: Idle vs Full System (Navigation+Streaming)",
         ha="center", va="top", fontsize=13, fontweight="bold", fontfamily="serif")
fig.text(0.5, 0.955,
         f"Model1 (Person/COCO): Idle n={len(idle_ms1)}, Full n={len(full_ms1)}  |  "
         f"Model2 (Door): Idle n={len(idle_ms2)}, Full n={len(full_ms2)}",
         ha="center", va="top", fontsize=9, color="#444444")

# ── (a) Model1 time series ───────────────────────────────
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(idle_t, idle_ms1, color=C_IDLE, linewidth=1.0, alpha=0.8, label="Idle")
ax1.plot(full_t, full_ms1, color=C_FULL, linewidth=1.0, alpha=0.8, label="Full System")
ax1.axhline(idle_ms1.mean(), color=C_IDLE, linestyle="--", linewidth=1.3)
ax1.axhline(full_ms1.mean(), color=C_FULL, linestyle="--", linewidth=1.3)
style_ax(ax1, "(a) Model1 (Person) — Inference Time over Time",
         xlabel="Time (s)", ylabel="Inference Time (ms)")
ax1.legend(fontsize=9, framealpha=0.9)

# ── (b) Model2 time series ───────────────────────────────
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(idle_t, idle_ms2, color=C_IDLE, linewidth=1.0, alpha=0.8, label="Idle")
ax2.plot(full_t, full_ms2, color=C_FULL, linewidth=1.0, alpha=0.8, label="Full System")
ax2.axhline(idle_ms2.mean(), color=C_IDLE, linestyle="--", linewidth=1.3)
ax2.axhline(full_ms2.mean(), color=C_FULL, linestyle="--", linewidth=1.3)
style_ax(ax2, "(b) Model2 (Door) — Inference Time over Time",
         xlabel="Time (s)", ylabel="Inference Time (ms)")
ax2.legend(fontsize=9, framealpha=0.9)

# ── (c) Box plot Model1 ──────────────────────────────────
ax3 = fig.add_subplot(gs[1, 0])
bp = ax3.boxplot([idle_ms1, full_ms1], patch_artist=True,
                 widths=0.5, notch=False,
                 medianprops=dict(color="black", linewidth=2))
bp['boxes'][0].set_facecolor(C_IDLE + "88")
bp['boxes'][1].set_facecolor(C_FULL + "88")
for flier in bp['fliers']:
    flier.set(marker='o', markerfacecolor='#888888', markersize=3, alpha=0.5)
ax3.set_xticks([1, 2])
ax3.set_xticklabels(["Idle", "Full System"])
style_ax(ax3, "(c) Model1 (Person) — Distribution",
         ylabel="Inference Time (ms)")

# ── (d) Box plot Model2 ──────────────────────────────────
ax4 = fig.add_subplot(gs[1, 1])
bp2 = ax4.boxplot([idle_ms2, full_ms2], patch_artist=True,
                  widths=0.5, notch=False,
                  medianprops=dict(color="black", linewidth=2))
bp2['boxes'][0].set_facecolor(C_IDLE + "88")
bp2['boxes'][1].set_facecolor(C_FULL + "88")
for flier in bp2['fliers']:
    flier.set(marker='o', markerfacecolor='#888888', markersize=3, alpha=0.5)
ax4.set_xticks([1, 2])
ax4.set_xticklabels(["Idle", "Full System"])
style_ax(ax4, "(d) Model2 (Door) — Distribution",
         ylabel="Inference Time (ms)")

# ── (e) Summary bar: mean ± std + FPS ────────────────────
ax5 = fig.add_subplot(gs[2, :])
labels   = ["Model1\nIdle", "Model1\nFull System", "Model2\nIdle", "Model2\nFull System"]
means    = [idle_ms1.mean(), full_ms1.mean(), idle_ms2.mean(), full_ms2.mean()]
stds     = [idle_ms1.std(),  full_ms1.std(),  idle_ms2.std(),  full_ms2.std()]
fps_vals = [1000/m for m in means]
colors   = [C_IDLE, C_FULL, C_IDLE, C_FULL]
x = np.arange(len(labels))

bars = ax5.bar(x, means, yerr=stds, color=colors, edgecolor="white",
               capsize=6, error_kw=dict(linewidth=1.5, capthick=1.5),
               width=0.55, zorder=3)

for bar, mean, std, fps in zip(bars, means, stds, fps_vals):
    ax5.text(bar.get_x() + bar.get_width()/2,
             mean + std + 8,
             f"{mean:.1f} ms\n({fps:.2f} FPS)",
             ha="center", va="bottom", fontsize=9, fontweight="bold")

# separator line between models
ax5.axvline(1.5, color="#aaaaaa", linestyle="--", linewidth=1.2)
ax5.text(0.75, ax5.get_ylim()[1]*0.02 if ax5.get_ylim()[1] > 0 else 10,
         "Model1 (Person/COCO)", ha="center", fontsize=9, color="#444444")
ax5.text(2.75, ax5.get_ylim()[1]*0.02 if ax5.get_ylim()[1] > 0 else 10,
         "Model2 (Door)", ha="center", fontsize=9, color="#444444")

ax5.set_xticks(x)
ax5.set_xticklabels(labels, fontsize=10)
ax5.set_ylim(0, max(means) + max(stds) + 80)
style_ax(ax5, "(e) Mean Inference Time ± 1σ and FPS Summary",
         ylabel="Inference Time (ms)")

from matplotlib.patches import Patch
ax5.legend(handles=[Patch(facecolor=C_IDLE, label="Idle"),
                    Patch(facecolor=C_FULL, label="Full System")],
           fontsize=9, loc="upper right")

plt.savefig(OUT_FILE, dpi=180, bbox_inches="tight", facecolor="white")
print(f"Saved: {OUT_FILE}")

# ── Print summary table ──────────────────────────────────
print("\n" + "="*65)
print(f"{'Condition':<20} {'Mean (ms)':>10} {'Std (ms)':>9} {'Min':>7} {'Max':>7} {'FPS':>6}")
print("="*65)
for label, arr in [("Model1 Idle", idle_ms1), ("Model1 Full", full_ms1),
                   ("Model2 Idle", idle_ms2), ("Model2 Full", full_ms2)]:
    print(f"{label:<20} {arr.mean():>10.1f} {arr.std():>9.1f} {arr.min():>7.1f} {arr.max():>7.1f} {1000/arr.mean():>6.2f}")
print("="*65)
