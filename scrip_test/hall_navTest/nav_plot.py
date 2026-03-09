#!/usr/bin/env python3
"""
Navigation Performance Analysis Plot
สำหรับทดสอบ: 10m straight, 80m straight, square path
วิธีใช้: แก้ข้อมูลในส่วน DATA SECTION แล้วรัน python3 nav_plot.py
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# =============================================================================
# DATA SECTION — แก้ตรงนี้เมื่อมีข้อมูลจริง
# =============================================================================

# --- 10m Straight Path (ใส่ข้อมูลจริงแทน) ---
data_10m = {
    "trials": list(range(1, 11)),
    "success":       [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],   # 1=สำเร็จ 0=ล้มเหลว
    "travel_time":   [18.2, 19.5, 17.8, 20.1, 18.9, 19.2, 18.5, 20.3, 19.1, 18.7],  # วินาที
    "path_length":   [10.3, 10.5, 10.2, 10.6, 10.4, 10.3, 10.5, 10.7, 10.4, 10.3],  # เมตร
    "planned_dist":  10.0,  # เมตร
    "recovery":      [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    "goal_error":    [0.08, 0.12, 0.06, 0.14, 0.09, 0.07, 0.11, 0.13, 0.08, 0.09],  # เมตร
}

# --- 80m Straight Path (ใส่ข้อมูลจริงแทน) ---
data_80m = {
    "trials": list(range(1, 11)),
    "success":       [1, 1, 0, 1, 1, 1, 1, 0, 1, 1],
    "travel_time":   [145.2, 152.1, 0, 148.7, 150.3, 147.8, 153.2, 0, 149.5, 151.0],
    "path_length":   [81.2, 82.5, 0, 81.8, 82.1, 81.5, 83.0, 0, 81.9, 82.3],
    "planned_dist":  80.0,
    "recovery":      [0, 1, 0, 0, 1, 0, 2, 0, 0, 1],
    "goal_error":    [0.09, 0.15, 0, 0.11, 0.13, 0.08, 0.16, 0, 0.10, 0.12],
}

# --- Square Path (ใส่ข้อมูลจริงแทน) ---
data_sq = {
    "trials": list(range(1, 11)),
    "success":       [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    "travel_time":   [48.5, 50.2, 47.8, 51.3, 49.7, 48.9, 50.5, 51.8, 49.2, 50.1],
    "path_length":   [8.3, 8.5, 8.2, 8.6, 8.4, 8.3, 8.5, 8.7, 8.4, 8.3],
    "planned_dist":  8.0,  # 4 sides x 2m
    "recovery":      [0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    "goal_error":    [0.16, 0.08, 0.25, 0.04, 0.17, 0.23, 0.17, 0.18, 0.17, 0.18],
}

# =============================================================================
# PLOT SECTION
# =============================================================================

COLORS = {
    "10m":  "#2E75B6",
    "80m":  "#E36C09",
    "sq":   "#375623",
    "fail": "#C00000",
    "grid": "#E8E8E8",
    "mean": "#FF0000",
}

def path_ratio(data):
    return [l / data["planned_dist"] if l > 0 else None for l in data["path_length"]]

def success_only(data, key):
    return [v for v, s in zip(data[key], data["success"]) if s == 1 and v > 0]

fig = plt.figure(figsize=(18, 14))
fig.patch.set_facecolor("#FAFAFA")
fig.suptitle("Navigation Performance Analysis\nReal-World Testing: 10m / 80m Straight & Square Path",
             fontsize=16, fontweight="bold", color="#1F3864", y=0.98)

gs = fig.add_gridspec(3, 3, hspace=0.45, wspace=0.38,
                      left=0.07, right=0.97, top=0.92, bottom=0.06)

datasets = [
    ("10m Straight", data_10m, COLORS["10m"]),
    ("80m Straight", data_80m, COLORS["80m"]),
    ("Square Path",  data_sq,  COLORS["sq"]),
]

# ---------- ROW 0: Travel Time ----------
for col, (label, data, color) in enumerate(datasets):
    ax = fig.add_subplot(gs[0, col])
    ax.set_facecolor("white")
    ax.grid(axis="y", color=COLORS["grid"], linewidth=0.8, zorder=0)

    times = data["travel_time"]
    bars = ax.bar(data["trials"], times, color=[
        color if s else COLORS["fail"] for s, t in zip(data["success"], times)
    ], edgecolor="white", linewidth=0.5, zorder=3, width=0.65)

    valid = success_only(data, "travel_time")
    if valid:
        mean_val = np.mean(valid)
        ax.axhline(mean_val, color=COLORS["mean"], linestyle="--", linewidth=1.5,
                   label=f"Mean = {mean_val:.1f}s", zorder=4)
        ax.legend(fontsize=8, loc="upper right")

    ax.set_title(f"{label}\nTravel Time", fontsize=10, fontweight="bold", color="#1F3864", pad=6)
    ax.set_xlabel("Trial", fontsize=8)
    ax.set_ylabel("Time (s)", fontsize=8)
    ax.set_xticks(data["trials"])
    ax.tick_params(labelsize=8)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)

    # mark failed
    for i, (t, s) in enumerate(zip(times, data["success"])):
        if s == 0:
            ax.text(i + 1, ax.get_ylim()[1] * 0.05, "✗", ha="center",
                    fontsize=10, color="white", fontweight="bold", zorder=5)

# ---------- ROW 1: Path Length Ratio ----------
for col, (label, data, color) in enumerate(datasets):
    ax = fig.add_subplot(gs[1, col])
    ax.set_facecolor("white")
    ax.grid(axis="y", color=COLORS["grid"], linewidth=0.8, zorder=0)

    ratios = path_ratio(data)
    valid_ratios = [r for r in ratios if r is not None]

    bar_colors = []
    for r, s in zip(ratios, data["success"]):
        if s == 0 or r is None:
            bar_colors.append(COLORS["fail"])
        else:
            bar_colors.append(color)

    plot_ratios = [r if r is not None else 0 for r in ratios]
    ax.bar(data["trials"], plot_ratios, color=bar_colors,
           edgecolor="white", linewidth=0.5, zorder=3, width=0.65)
    ax.axhline(1.0, color="#555555", linestyle=":", linewidth=1.5,
               label="Ideal = 1.0", zorder=4)

    if valid_ratios:
        mean_r = np.mean(valid_ratios)
        ax.axhline(mean_r, color=COLORS["mean"], linestyle="--", linewidth=1.5,
                   label=f"Mean = {mean_r:.3f}", zorder=4)

    ax.legend(fontsize=8, loc="upper right")
    ax.set_title(f"{label}\nPath Length Ratio", fontsize=10, fontweight="bold", color="#1F3864", pad=6)
    ax.set_xlabel("Trial", fontsize=8)
    ax.set_ylabel("Actual / Planned", fontsize=8)
    ax.set_xticks(data["trials"])
    ax.tick_params(labelsize=8)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)

# ---------- ROW 2: Goal Error + Recovery + Success Rate ----------
for col, (label, data, color) in enumerate(datasets):
    ax = fig.add_subplot(gs[2, col])
    ax.set_facecolor("white")
    ax.grid(axis="y", color=COLORS["grid"], linewidth=0.8, zorder=0)

    errors = data["goal_error"]
    recoveries = data["recovery"]

    bar_colors = [color if s else COLORS["fail"] for s in data["success"]]
    bars = ax.bar(data["trials"], errors, color=bar_colors,
                  edgecolor="white", linewidth=0.5, zorder=3, width=0.65, label="Goal Error (m)")

    ax2 = ax.twinx()
    ax2.plot(data["trials"], recoveries, "D--", color="#7030A0",
             markersize=5, linewidth=1.2, label="Recovery Count", zorder=5)
    ax2.set_ylabel("Recovery Count", fontsize=8, color="#7030A0")
    ax2.tick_params(axis="y", labelcolor="#7030A0", labelsize=8)
    ax2.set_ylim(0, max(recoveries) + 1.5 if max(recoveries) > 0 else 2)
    for spine in ["top"]:
        ax2.spines[spine].set_visible(False)

    ax.axhline(0.10, color="#FF6600", linestyle="--", linewidth=1.2,
               label="xy_goal_tolerance", zorder=4)

    success_count = sum(data["success"])
    success_rate = success_count / len(data["trials"]) * 100
    ax.set_title(f"{label}\nGoal Error & Recovery  [Success: {success_count}/{len(data['trials'])} = {success_rate:.0f}%]",
                 fontsize=9, fontweight="bold", color="#1F3864", pad=6)
    ax.set_xlabel("Trial", fontsize=8)
    ax.set_ylabel("Goal Error (m)", fontsize=8)
    ax.set_xticks(data["trials"])
    ax.tick_params(labelsize=8)

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=7, loc="upper right")

    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)

# ---------- Summary bar at bottom ----------
fail_patch = mpatches.Patch(color=COLORS["fail"], label="Failed Trial")
fig.legend(handles=[fail_patch], loc="lower right", fontsize=9,
           framealpha=0.8, bbox_to_anchor=(0.97, 0.01))

plt.savefig("/mnt/user-data/outputs/nav_performance_plot.png",
            dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
plt.close()
print("Saved: nav_performance_plot.png")

# =============================================================================
# SUMMARY TABLE
# =============================================================================
print("\n" + "="*65)
print(f"{'Metric':<28} {'10m':>10} {'80m':>10} {'Square':>10}")
print("="*65)

for label, data, _ in datasets:
    pass  # just using the loop below

def summarize(data, label):
    valid_time = success_only(data, "travel_time")
    valid_err  = [e for e, s in zip(data["goal_error"], data["success"]) if s == 1 and e > 0]
    valid_ratio = [r for r in path_ratio(data) if r is not None]
    sr = sum(data["success"]) / len(data["trials"]) * 100
    return {
        "Success Rate": f"{sr:.0f}%",
        "Mean Time (s)": f"{np.mean(valid_time):.1f}" if valid_time else "N/A",
        "Mean Goal Error (m)": f"{np.mean(valid_err):.3f}" if valid_err else "N/A",
        "Mean Path Ratio": f"{np.mean(valid_ratio):.3f}" if valid_ratio else "N/A",
        "Total Recovery": str(sum(data["recovery"])),
    }

s10 = summarize(data_10m, "10m")
s80 = summarize(data_80m, "80m")
ssq = summarize(data_sq, "Square")

for key in s10:
    print(f"{key:<28} {s10[key]:>10} {s80[key]:>10} {ssq[key]:>10}")
print("="*65)
print("\n*** ข้อมูลเป็น placeholder — แก้ใน DATA SECTION ด้วยข้อมูลจริง ***")
