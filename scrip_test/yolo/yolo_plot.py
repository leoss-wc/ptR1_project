#!/usr/bin/env python3
"""
yolo_detection_plot.py
======================
วาดกราฟผลการทดสอบ YOLO Detection Accuracy
Data: Person, door_open, door_close at 3m and 7m
Lighting: Bright, Dark, Dark+Spotlight
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

# ────────────────────────────────────────────────────────────
#  DATA SECTION
# ────────────────────────────────────────────────────────────
RAW_DATA = {
    "Person": {
        "3.0m": {"Bright": (20,0,0), "Dark": (0,0,20), "Dark+Spotlight": (19,0,1)},
        "7.0m": {"Bright": (17,0,3), "Dark": (0,0,20), "Dark+Spotlight": (16,0,4)},
    },
    "door_open": {
        "3.0m": {"Bright": (20,0,0), "Dark": (0,0,20), "Dark+Spotlight": (15,0,5)},
        "7.0m": {"Bright": (12,8,0), "Dark": (0,0,20), "Dark+Spotlight": (10,10,0)},
    },
    "door_close": {
        "3.0m": {"Bright": (17,1,2), "Dark": (0,0,20), "Dark+Spotlight": (20,0,0)},
        "7.0m": {"Bright": (20,0,0), "Dark": (0,0,20), "Dark+Spotlight": (20,0,0)},
    },
}

CONDITIONS = ["Bright", "Dark", "Dark+Spotlight"]
DISTANCES  = ["3.0m", "7.0m"]
CLASSES    = ["Person", "door_open", "door_close"]

# ────────────────────────────────────────────────────────────
#  METRICS
# ────────────────────────────────────────────────────────────
def calc(tp, fp, fn):
    p  = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    r  = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1 = 2*p*r / (p+r)  if (p + r)  > 0 else 0.0
    return round(p,3), round(r,3), round(f1,3)

metrics = {}
for cls in CLASSES:
    metrics[cls] = {}
    for dist in DISTANCES:
        metrics[cls][dist] = {}
        for cond in CONDITIONS:
            tp, fp, fn = RAW_DATA[cls][dist][cond]
            metrics[cls][dist][cond] = calc(tp, fp, fn)

# ────────────────────────────────────────────────────────────
#  STYLE
# ────────────────────────────────────────────────────────────
C = {
    "Bright":        "#2166ac",
    "Dark":          "#555555",
    "Dark+Spotlight": "#e08214",
}
METRIC_COLORS = {"Precision": "#2166ac", "Recall": "#d6604d", "F1": "#4dac26"}

plt.rcParams.update({
    "font.family": "serif", "font.size": 10,
    "axes.titlesize": 11, "axes.labelsize": 10,
    "xtick.labelsize": 9, "ytick.labelsize": 9,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.dpi": 150,
})

def style_ax(ax, title="", xlabel="", ylabel=""):
    ax.set_facecolor("white")
    ax.grid(axis="y", color="#dddddd", linestyle="--", linewidth=0.7, zorder=0)
    ax.set_axisbelow(True)
    if title:  ax.set_title(title, fontweight="bold", loc="left", pad=6)
    if xlabel: ax.set_xlabel(xlabel)
    if ylabel: ax.set_ylabel(ylabel)

# ────────────────────────────────────────────────────────────
#  FIGURE 1: Precision / Recall / F1 per class per distance
# ────────────────────────────────────────────────────────────
fig1, axes = plt.subplots(3, 2, figsize=(13, 11), facecolor="white")
fig1.suptitle("YOLO11 nano Detection Performance: Precision / Recall / F1",
              fontsize=13, fontweight="bold", y=0.98)

x      = np.arange(len(CONDITIONS))
width  = 0.25
metric_keys = ["Precision", "Recall", "F1"]

for ri, cls in enumerate(CLASSES):
    for ci, dist in enumerate(DISTANCES):
        ax = axes[ri][ci]
        for mi, mk in enumerate(metric_keys):
            vals = [metrics[cls][dist][c][mi] for c in CONDITIONS]
            bars = ax.bar(x + (mi - 1) * width, vals, width,
                          label=mk, color=METRIC_COLORS[mk],
                          edgecolor="white", linewidth=0.5, zorder=3)
            for bar, v in zip(bars, vals):
                if v > 0:
                    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                            f"{v:.2f}", ha="center", va="bottom", fontsize=7.5)

        ax.set_ylim(0, 1.18)
        ax.set_xticks(x)
        ax.set_xticklabels(CONDITIONS, fontsize=9)
        style_ax(ax,
                 title=f"({chr(97 + ri*2 + ci)}) {cls} @ {dist}",
                 ylabel="Score" if ci == 0 else "")
        if ri == 0 and ci == 1:
            ax.legend(loc="upper right", fontsize=8, framealpha=0.9)

plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig("yolo_precision_recall_f1.png",
            dpi=180, bbox_inches="tight", facecolor="white")
plt.close()
print("Saved: yolo_precision_recall_f1.png")

# ────────────────────────────────────────────────────────────
#  FIGURE 2: Recall heatmap (3m vs 7m × conditions × classes)
# ────────────────────────────────────────────────────────────
fig2, axes2 = plt.subplots(1, 2, figsize=(12, 4), facecolor="white")
fig2.suptitle("YOLO11 nano Recall Heatmap by Class, Distance, and Lighting",
              fontsize=13, fontweight="bold")

for ci, dist in enumerate(DISTANCES):
    ax = axes2[ci]
    matrix = np.array([
        [metrics[cls][dist][cond][1] for cond in CONDITIONS]
        for cls in CLASSES
    ])
    im = ax.imshow(matrix, cmap="RdYlGn", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(len(CONDITIONS)))
    ax.set_xticklabels(CONDITIONS, fontsize=10)
    ax.set_yticks(range(len(CLASSES)))
    ax.set_yticklabels(CLASSES, fontsize=10)
    ax.set_title(f"Recall @ {dist}", fontweight="bold", pad=8)
    for r in range(len(CLASSES)):
        for c in range(len(CONDITIONS)):
            val = matrix[r, c]
            color = "white" if val < 0.4 else "black"
            ax.text(c, r, f"{val:.2f}", ha="center", va="center",
                    fontsize=11, fontweight="bold", color=color)

plt.colorbar(im, ax=axes2[1], label="Recall")
plt.tight_layout()
plt.savefig("yolo_recall_heatmap.png",
            dpi=180, bbox_inches="tight", facecolor="white")
plt.close()
print("Saved: yolo_recall_heatmap.png")

# ────────────────────────────────────────────────────────────
#  FIGURE 3: TP/FP/FN stacked bar
# ────────────────────────────────────────────────────────────
fig3, axes3 = plt.subplots(1, 3, figsize=(14, 5), facecolor="white")
fig3.suptitle("YOLO11 nano Detection Count: TP / FP / FN by Class and Condition",
              fontsize=13, fontweight="bold")

cond_labels = [f"{c}\n{d}" for d in DISTANCES for c in CONDITIONS]
x3 = np.arange(len(cond_labels))

for ci, cls in enumerate(CLASSES):
    ax = axes3[ci]
    tps = [RAW_DATA[cls][d][c][0] for d in DISTANCES for c in CONDITIONS]
    fps = [RAW_DATA[cls][d][c][1] for d in DISTANCES for c in CONDITIONS]
    fns = [RAW_DATA[cls][d][c][2] for d in DISTANCES for c in CONDITIONS]

    ax.bar(x3, tps, label="TP", color="#4dac26", edgecolor="white", linewidth=0.5, zorder=3)
    ax.bar(x3, fps, bottom=tps, label="FP", color="#d6604d", edgecolor="white", linewidth=0.5, zorder=3)
    ax.bar(x3, fns, bottom=[t+f for t,f in zip(tps,fps)],
           label="FN", color="#aaaaaa", edgecolor="white", linewidth=0.5, zorder=3)

    ax.axvline(2.5, color="#333333", linestyle="--", linewidth=1.2, alpha=0.5)
    ax.text(1, 21.5, "3.0 m", ha="center", fontsize=9, color="#333333")
    ax.text(4, 21.5, "7.0 m", ha="center", fontsize=9, color="#333333")

    ax.set_xticks(x3)
    ax.set_xticklabels(cond_labels, fontsize=8)
    ax.set_ylim(0, 23)
    style_ax(ax, title=f"({chr(97+ci)}) {cls}", ylabel="Count" if ci==0 else "")
    if ci == 0:
        ax.legend(fontsize=8, loc="upper right")

plt.tight_layout()
plt.savefig("yolo_tp_fp_fn.png",
            dpi=180, bbox_inches="tight", facecolor="white")
plt.close()
print("Saved: yolo_tp_fp_fn.png")

print("\nAll plots saved!")
