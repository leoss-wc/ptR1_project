import matplotlib.pyplot as plt
import numpy as np

# ข้อมูลจาก analyze_tracking2.py
data = {
    'RMSE':       [0.0669, 0.0527, 0.0606, 0.0535, 0.0682, 0.0480, 0.0570, 0.0703, 0.0611, 0.0576],
    'Max Error':  [0.1391, 0.1393, 0.1863, 0.1707, 0.1866, 0.1111, 0.1877, 0.1837, 0.1634, 0.1590],
    'Mean Error': [0.0555, 0.0415, 0.0453, 0.0427, 0.0511, 0.0387, 0.0439, 0.0563, 0.0470, 0.0440],
}

labels = list(data.keys())
values = list(data.values())

fig, ax = plt.subplots(figsize=(8, 5))

bp = ax.boxplot(values, labels=labels, patch_artist=True,
                medianprops=dict(color='black', linewidth=2),
                whiskerprops=dict(linestyle='--'),
                flierprops=dict(marker='o', markersize=5))

colors = ['#AED6F1', '#F1948A', '#A9DFBF']
for patch, color in zip(bp['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)

# เส้น threshold
ax.axhline(y=0.10, color='red', linestyle=':', linewidth=1.2, label='Threshold (0.10 m)')

# jitter dots
np.random.seed(42)
for i, vals in enumerate(values, start=1):
    jitter = np.random.uniform(-0.08, 0.08, len(vals))
    ax.plot(np.full(len(vals), i) + jitter, vals,
            'k.', alpha=0.5, markersize=5)

ax.set_ylabel('Error (m)')
ax.set_title('Path Tracking Error Distribution\nSquare Path 2.0 m  (n = 10 runs)', fontsize=11)
ax.legend(fontsize=9)
ax.grid(axis='y', linestyle='--', alpha=0.4)

plt.tight_layout()
plt.savefig('path_tracking_boxplot.png', dpi=150)
plt.show()
print("บันทึกกราฟเป็น path_tracking_boxplot.png")