#!/usr/bin/env python3
"""
plot_position_error.py
วาด Scatter plot แสดง x/y error ของ Square Path จาก 10 รอบที่วัดจริง
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ข้อมูลจากการวัดจริง (หน่วย: cm)
# บวก = เกินตำแหน่ง, ลบ = ไม่ถึงตำแหน่ง
data = [
    {'round': 1,  'x_err': -14.5, 'y_err':  13.0},
    {'round': 2,  'x_err':  -0.5, 'y_err':   8.0},
    {'round': 3,  'x_err':  11.5, 'y_err':  22.5},
    {'round': 4,  'x_err':  16.0, 'y_err':   4.0},
    {'round': 5,  'x_err':   3.2, 'y_err':   2.5},
    {'round': 6,  'x_err':  18.0, 'y_err': -15.0},
    {'round': 7,  'x_err':  -9.0, 'y_err': -14.5},
    {'round': 8,  'x_err':   8.5, 'y_err': -15.0},
    {'round': 9,  'x_err': -16.0, 'y_err':  -7.5},
    {'round': 10, 'x_err':  -6.5, 'y_err': -16.5},
]

x_err = np.array([d['x_err'] for d in data])
y_err = np.array([d['y_err'] for d in data])
rounds = [d['round'] for d in data]

# Euclidean distance error ต่อรอบ
dist_err = np.sqrt(x_err**2 + y_err**2)

fig, axes = plt.subplots(1, 2, figsize=(13, 6))
fig.suptitle('Position Error — Square Path 2.0 m  (n = 10 rounds)', fontsize=12)

# ── Plot 1: Scatter x/y error ──────────────────────────────────────────────
ax = axes[0]

# วงกลม error reference
for r in [5, 10, 15, 20, 25]:
    circle = plt.Circle((0, 0), r, color='grey', fill=False,
                         linestyle='--', linewidth=0.6, alpha=0.4)
    ax.add_patch(circle)
    ax.text(r * 0.707, r * 0.707, f'{r} cm', fontsize=6,
            color='grey', alpha=0.6, ha='left', va='bottom')

# แกน
ax.axhline(0, color='black', linewidth=0.8, alpha=0.5)
ax.axvline(0, color='black', linewidth=0.8, alpha=0.5)

# scatter จุด — สีตาม round
cmap = plt.cm.tab10
colors = [cmap(i / 10) for i in range(10)]

for i, d in enumerate(data):
    ax.scatter(d['x_err'], d['y_err'], color=colors[i], s=80, zorder=5)
    ax.annotate(f"R{d['round']}",
                xy=(d['x_err'], d['y_err']),
                xytext=(4, 4), textcoords='offset points',
                fontsize=8, color=colors[i])

# mean error point
ax.scatter(np.mean(x_err), np.mean(y_err),
           marker='D', s=100, color='red', zorder=6, label='Mean error')
ax.annotate(f"Mean\n({np.mean(x_err):.1f}, {np.mean(y_err):.1f})",
            xy=(np.mean(x_err), np.mean(y_err)),
            xytext=(6, -14), textcoords='offset points',
            fontsize=8, color='red')

ax.set_xlabel('X Error (cm)')
ax.set_ylabel('Y Error (cm)')
ax.set_title('X/Y Position Error per Round')
ax.set_aspect('equal')
lim = 30
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.grid(True, linestyle='--', alpha=0.3)
ax.legend(fontsize=8)

# Quadrant labels
ax.text( 26,  26, '+X+Y', fontsize=7, color='grey', alpha=0.5, ha='right', va='top')
ax.text(-26,  26, '-X+Y', fontsize=7, color='grey', alpha=0.5, ha='left',  va='top')
ax.text( 26, -26, '+X-Y', fontsize=7, color='grey', alpha=0.5, ha='right', va='bottom')
ax.text(-26, -26, '-X-Y', fontsize=7, color='grey', alpha=0.5, ha='left',  va='bottom')

# ── Plot 2: Euclidean distance error per round (bar) ───────────────────────
ax2 = axes[1]

bar_colors = [cmap(i / 10) for i in range(10)]
bars = ax2.bar(rounds, dist_err, color=bar_colors, alpha=0.8, edgecolor='grey', linewidth=0.5)

# ค่าบนแท่ง
for bar, val in zip(bars, dist_err):
    ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
             f'{val:.1f}', ha='center', va='bottom', fontsize=8)

ax2.axhline(np.mean(dist_err), color='red', linestyle='--',
            linewidth=1.2, label=f'Mean = {np.mean(dist_err):.1f} cm')

ax2.set_xlabel('Round')
ax2.set_ylabel('Euclidean Error (cm)')
ax2.set_title('Distance Error per Round  (√x²+y²)')
ax2.set_xticks(rounds)
ax2.set_xticklabels([f'R{r}' for r in rounds], fontsize=8)
ax2.legend(fontsize=8)
ax2.grid(axis='y', linestyle='--', alpha=0.4)
ax2.set_ylim(0, max(dist_err) * 1.2)

# สรุปสถิติ
stats_text = (
    f"X error:  mean={np.mean(x_err):.1f}  std={np.std(x_err):.1f} cm\n"
    f"Y error:  mean={np.mean(y_err):.1f}  std={np.std(y_err):.1f} cm\n"
    f"Dist err: mean={np.mean(dist_err):.1f}  max={np.max(dist_err):.1f} cm"
)
ax2.text(0.02, 0.97, stats_text, transform=ax2.transAxes,
         fontsize=8, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7))

plt.tight_layout()
plt.savefig('position_error.png', dpi=150, bbox_inches='tight')
print("บันทึกกราฟเป็น: position_error.png")
plt.show()