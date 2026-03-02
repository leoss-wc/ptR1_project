import pandas as pd
import matplotlib.pyplot as plt

excel_file = 'ptR1 data.xlsx'

# 1. ดึงชุดข้อมูล "ทวนเข็ม" (CCW)
# ใช้ skiprows=2 และบังคับดึงแค่ 10 แถวด้วย nrows=10
df_ccw = pd.read_excel(excel_file, sheet_name='Odometry Trajectory angle', skiprows=2, nrows=10)
df_ccw = df_ccw.iloc[:, :5] # ดึงแค่ 5 คอลัมน์แรก
df_ccw.columns = ['Test', 'Target_Angle', 'Actual_Angle', 'Error', 'Time']
df_ccw['Abs_Error'] = df_ccw['Error'].abs() # ทำเป็นค่าสัมบูรณ์ (Absolute)

# 2. ดึงชุดข้อมูล "ตามเข็ม" (CW)
# ใช้ skiprows=17 เพื่อข้ามไปอ่านชุดข้อมูลด้านล่าง
df_cw = pd.read_excel(excel_file, sheet_name='Odometry Trajectory angle', skiprows=17, nrows=10)
df_cw = df_cw.iloc[:, :5]
df_cw.columns = ['Test', 'Target_Angle', 'Actual_Angle', 'Error', 'Time']
df_cw['Abs_Error'] = df_cw['Error'].abs() # ทำเป็นค่าสัมบูรณ์ เพราะข้อมูลเป็นค่าติดลบ (-5, -12...)

# 3. วาดกราฟเปรียบเทียบ
plt.style.use('seaborn-v0_8-whitegrid')
plt.figure(figsize=(7, 6))

# ใส่ข้อมูลทั้ง 2 ชุดเข้าไปใน Box Plot
box = plt.boxplot([df_ccw['Abs_Error'], df_cw['Abs_Error']], 
                  labels=['Counter-Clockwise (CCW)', 'Clockwise (CW)'], 
                  patch_artist=True, widths=0.4)

# ระบายสีกล่องให้แยกกันชัดเจน
colors = ['thistle', 'lightblue']
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)

plt.title('Rotation Error Distribution Comparison\n(Target: 1800° or 5 Rotations)', fontsize=14, fontweight='bold')
plt.ylabel('Absolute Error (Degrees)', fontsize=12)
plt.grid(axis='y', linestyle='--', alpha=0.7)

# ใส่ข้อความสรุปค่าเฉลี่ย
mean_ccw = df_ccw['Abs_Error'].mean()
mean_cw = df_cw['Abs_Error'].mean()
plt.annotate(f'Mean CCW: {mean_ccw:.1f}°\nMean CW: {mean_cw:.1f}°', 
             xy=(0.05, 0.85), xycoords='axes fraction', 
             bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", lw=1), fontsize=11)

plt.tight_layout()
plt.savefig('box_angle_comparison.png', dpi=300)
plt.show()