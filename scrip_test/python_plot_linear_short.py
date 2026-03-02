import pandas as pd
import matplotlib.pyplot as plt

excel_file = 'ptR1 data.xlsx'

# --- 1. ดึงข้อมูล 4 ทิศทาง (สมมติว่าคุณเว้นบรรทัดไว้ตามนี้) ---
# เดินหน้า (X+)
df_x_plus = pd.read_excel(excel_file, sheet_name='Odometry Trajectory linear X', skiprows=2, nrows=10)
# ถอยหลัง (X-) *สมมติว่าอยู่บรรทัดที่ 17 ของหน้าเดียวกัน*
df_x_minus = pd.read_excel(excel_file, sheet_name='Odometry Trajectory linear X', skiprows=17, nrows=10)
# สไลด์ซ้าย (Y+)
df_y_plus = pd.read_excel(excel_file, sheet_name='Odometry Trajectory linear Y', skiprows=2, nrows=10)
# สไลด์ขวา (Y-) *สมมติว่าอยู่บรรทัดที่ 17 ของหน้าเดียวกัน*
df_y_minus = pd.read_excel(excel_file, sheet_name='Odometry Trajectory linear Y', skiprows=17, nrows=10)

# สร้างคอลัมน์ Absolute Error เพื่อทำ Box Plot
df_x_plus['Abs_Error'] = df_x_plus.iloc[:, 1].abs() # สมมติคอลัมน์ 1 คือ X Error
df_x_minus['Abs_Error'] = df_x_minus.iloc[:, 1].abs()
df_y_plus['Abs_Error'] = df_y_plus.iloc[:, 2].abs() # สมมติคอลัมน์ 2 คือ Y Error
df_y_minus['Abs_Error'] = df_y_minus.iloc[:, 2].abs()

plt.style.use('seaborn-v0_8-whitegrid')

# ==========================================
# กราฟที่ 1: 2D Scatter Plot รวม 4 ทิศทาง
# ==========================================
plt.figure(figsize=(8, 8))
# พล็อตจุดลงไปโดยอ้างอิง X Error และ Y Error
plt.scatter(df_x_plus.iloc[:, 2], df_x_plus.iloc[:, 1], c='blue', label='Forward (X+)', alpha=0.7, s=80)
plt.scatter(df_x_minus.iloc[:, 2], df_x_minus.iloc[:, 1], c='orange', label='Backward (X-)', alpha=0.7, s=80)
plt.scatter(df_y_plus.iloc[:, 2], df_y_plus.iloc[:, 1], c='green', label='Left (Y+)', alpha=0.7, s=80)
plt.scatter(df_y_minus.iloc[:, 2], df_y_minus.iloc[:, 1], c='red', label='Right (Y-)', alpha=0.7, s=80)

plt.scatter(0, 0, color='black', marker='X', s=200, label='Target (0,0)')
plt.title('2D Target Map: All Directions Kinematic Performance', fontsize=14, fontweight='bold')
plt.xlabel('Lateral Error (cm)', fontsize=12)
plt.ylabel('Longitudinal Error (cm)', fontsize=12)
plt.axhline(0, color='gray', linestyle='--')
plt.axvline(0, color='gray', linestyle='--')
plt.legend()
plt.axis('equal') # ล็อกสเกลให้สมจริง
plt.tight_layout()
plt.savefig('scatter_all_directions.png', dpi=300)
plt.show()

# ==========================================
# กราฟที่ 2: Box Plot เปรียบเทียบ 4 ทิศทาง
# ==========================================
plt.figure(figsize=(8, 6))
box = plt.boxplot([df_x_plus['Abs_Error'], df_x_minus['Abs_Error'], 
                   df_y_plus['Abs_Error'], df_y_minus['Abs_Error']], 
                  labels=['X+ (Fwd)', 'X- (Bwd)', 'Y+ (Left)', 'Y- (Right)'], 
                  patch_artist=True)

# ระบายสีกล่องให้ตรงกับกราฟ Scatter
colors = ['lightblue', 'bisque', 'lightgreen', 'lightcoral']
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)

plt.title('Absolute Error Distribution by Direction', fontsize=14, fontweight='bold')
plt.ylabel('Absolute Error (cm)', fontsize=12)
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig('box_all_directions.png', dpi=300)
plt.show()