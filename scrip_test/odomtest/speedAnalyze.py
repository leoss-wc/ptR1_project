

"""
rosbag record /wheel_velocities /wheel_setpoints

"""

import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# ── Load bag ──────────────────────────────────────────────────────────────────
b = bagreader('2026-03-06-03-53-29.bag')
vel = pd.read_csv(b.message_by_topic('/wheel_velocities'))
sp  = pd.read_csv(b.message_by_topic('/wheel_setpoints'))

# ── แปลง Timestamp เป็นเวลาที่ผ่านไป (วินาที) ────────────────────────────────
t0 = min(vel['Time'].iloc[0], sp['Time'].iloc[0])
vel['t'] = vel['Time'] - t0
sp['t']  = sp['Time']  - t0

# ── ชื่อล้อทั้ง 4 ─────────────────────────────────────────────────────────────
wheels = [
    ('data_0', 'FL (Front Left)'),
    ('data_1', 'FR (Front Right)'),
    ('data_2', 'RL (Rear Left)'),
    ('data_3', 'RR (Rear Right)'),
]

# ── Plot 4 กราฟ ───────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 10))
fig.suptitle('Wheel Velocity — Setpoint vs Actual (PID Response)', 
             fontsize=14, fontweight='bold', y=0.98)

for i, (col, name) in enumerate(wheels):
    ax = fig.add_subplot(2, 2, i+1)
    ax.plot(vel['t'], vel[col], label='Actual', color='steelblue', linewidth=1)
    ax.plot(sp['t'],  sp[col],  label='Setpoint', color='orangered', 
            linewidth=1.5, linestyle='--')
    
    ax.set_title(f'Wheel {name}', fontsize=11, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=10)
    ax.set_ylabel('Velocity (m/s)', fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)

plt.tight_layout()
plt.savefig('wheel_pid_response.png', dpi=150, bbox_inches='tight')
plt.show()
print("Saved: wheel_pid_response.png")