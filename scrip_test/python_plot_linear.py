import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

filename = "ptR1 data.xlsx"
sheet_name = "Odometry Trajectory linear Y" 
df = pd.read_excel(filename, sheet_name=sheet_name)
target_section = 'Y+' 

# ค้นหาแถวที่เป็นจุดเริ่มต้นของข้อมูล
start_index = df[df.iloc[:, 0] == target_section].index[0]
header_index = start_index + 1

# ดึงข้อมูลตัวเลข 10 Test
df_data = df.iloc[header_index + 1 : header_index + 11, :4].copy()
df_data.columns = ['Test', 'X Error (cm)', 'Y Error (cm)', 'time(s)']

# แปลงข้อมูลในตารางให้เป็นตัวเลข
x_error = pd.to_numeric(df_data['X Error (cm)']).tolist()
y_error = pd.to_numeric(df_data['Y Error (cm)']).tolist()
time_s  = pd.to_numeric(df_data['time(s)']).tolist()
tests   = [f"Test {i:02d}" for i in range(1, len(x_error) + 1)]

print(f"อ่านข้อมูล {target_section} สำเร็จ! จำนวนข้อมูล: {len(tests)} แถว")

plt.style.use('seaborn-v0_8-whitegrid')

# กราฟที่ 1: Scatter Plot
plt.figure(figsize=(8, 6))
plt.scatter(y_error, x_error, color='blue', alpha=0.7, s=100, label='Stop Position')
plt.scatter(0, 0, color='red', marker='X', s=200, label='Target (0,0)')

plt.title(f'2D Target Map ({target_section}): Stopping Position Accuracy\n(Target: 3m, 0.3 m/s)', fontsize=14, fontweight='bold')
plt.xlabel('Y Error / Lateral Error (cm)', fontsize=12)
plt.ylabel('X Error / Longitudinal Error (cm)', fontsize=12)
plt.axhline(0, color='black', linewidth=1, linestyle='--')
plt.axvline(0, color='black', linewidth=1, linestyle='--')
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()
plt.axis('equal')

x_mean = np.mean(x_error)
y_mean = np.mean(y_error)
plt.annotate(f'Mean X Error: {x_mean:.2f} cm\nMean Y Error: {y_mean:.2f} cm', 
             xy=(0.05, 0.05), xycoords='axes fraction', 
             bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", lw=1))

plt.tight_layout()
plt.savefig('scatter_plot.png', dpi=300)
plt.show()

# กราฟที่ 2: Box Plot
plt.figure(figsize=(7, 6))
box = plt.boxplot([x_error, y_error], labels=['X Error (cm)', 'Y Error (cm)'], patch_artist=True)

colors = ['lightblue', 'lightgreen']
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)

plt.title(f'Error Distribution Box Plot ({target_section})\n(Consistency Analysis)', fontsize=14, fontweight='bold')
plt.ylabel('Error (cm)', fontsize=12)
plt.axhline(0, color='red', linestyle='--', linewidth=1.5, label='Zero Error Line (Target)')
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.legend()

plt.tight_layout()
plt.savefig('box_plot.png', dpi=300)
plt.show()

# กราฟที่ 3: Bar Chart
plt.figure(figsize=(10, 5))
bars = plt.bar(tests, time_s, color='coral', alpha=0.8, edgecolor='black')

for bar in bars:
    yval = bar.get_height()
    plt.text(bar.get_x() + bar.get_width()/2, yval + 0.01, f'{yval:.2f}', ha='center', va='bottom', fontsize=10)

mean_time = np.mean(time_s)
plt.axhline(mean_time, color='blue', linestyle='-.', linewidth=2, label=f'Mean Time ({mean_time:.2f} s)')

plt.title(f'Time Consistency per Test Run ({target_section})\n(Command: 3m distance at 0.3 m/s)', fontsize=14, fontweight='bold')
plt.ylabel('Time (seconds)', fontsize=12)
plt.ylim(min(time_s) - 0.2, max(time_s) + 0.2)
plt.grid(axis='y', linestyle='--', alpha=0.5)
plt.legend(loc='lower right')

plt.tight_layout()
plt.savefig('time_bar_chart.png', dpi=300)
plt.show()