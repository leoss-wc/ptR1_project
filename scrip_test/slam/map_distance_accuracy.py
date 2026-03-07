#!/usr/bin/env python3
"""
Map Distance Accuracy Tool  (ป.ตรี Edition)
วัดความถูกต้องของขนาดแผนที่เทียบกับระยะจริง

วิธีใช้:
  1. เปิด RViz แล้วโหลด Reference map
  2. ใช้ Publish Point คลิกจุดอ้างอิงแต่ละจุด จด coordinate (x, y)
    คลิกที่จุด A แล้วจด coordinate (x, y)
    คลิกที่จุด B แล้วจด coordinate (x, y)
    คำนวณระยะ = √((x2-x1)² + (y2-y1)²)

  3. วัดระยะจริงด้วยตลับเมตร
  4. ใส่ค่าในส่วน USER INPUT ด้านล่าง
  5. รัน: python3 map_distance_accuracy.py

ผลลัพธ์:
  - distance_accuracy_results.png  (กราฟ Bar + Line)
  - distance_accuracy.csv
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import csv
import os

# ===== ตั้งค่า Font สำหรับ Thesis =====
plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 12,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
})


# ============================================================
#  USER INPUT — ใส่ค่าตรงนี้
# ============================================================

# จุดอ้างอิงใน map (coordinate จาก RViz Publish Point)
# วิธีอ่านค่าจาก RViz:
#   rostopic echo /clicked_point
#   แล้วคลิกจุดที่ต้องการใน RViz จะได้ x, y มา
POINTS = {
    "A": (0.00, 0.00),    # เช่น จุดเริ่มต้น / ประตูทางเข้า
    "B": (5.10, 0.05),    # เช่น เสา / มุมผนัง
    "C": (25.30, 0.10),   # เช่น ประตูกระจก
    "D": (35.20, 0.08),   # เช่น ปลาย corridor
    # เพิ่มจุดได้ตามต้องการ เช่น
    # "E": (50.00, 0.00),
}

# ระยะที่ต้องการวัด: (จุดเริ่ม, จุดสิ้นสุด, ระยะจริง_เมตร)
MEASUREMENTS = [
    ("A", "B",  5.00),    # ระยะสั้น: จุดเริ่มต้น → เสา
    ("B", "C", 20.00),    # ระยะกลาง: เสา → ประตูกระจก
    ("C", "D", 10.00),    # ระยะปลาย: ประตูกระจก → ปลาย corridor
    ("A", "C", 25.00),    # ระยะรวมกลาง
    ("A", "D", 35.00),    # ระยะรวมทั้งหมด
]

# โฟลเดอร์ output
OUTPUT_DIR = "/home/leoss/ptR1Project/scrip_test/slam"

# ============================================================
#  คำนวณระยะใน Map
# ============================================================

def calc_map_distance(p1_name: str, p2_name: str) -> float:
    """คำนวณระยะระหว่าง 2 จุดใน map จาก coordinate"""
    x1, y1 = POINTS[p1_name]
    x2, y2 = POINTS[p2_name]
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def compute_accuracy(measurements: list) -> list:
    """คำนวณ error แต่ละระยะ"""
    results = []
    for (p1, p2, real_dist) in measurements:
        map_dist  = calc_map_distance(p1, p2)
        error_m   = abs(real_dist - map_dist)
        error_pct = error_m / real_dist * 100
        results.append({
            "name":      f"{p1}→{p2}",
            "real":      real_dist,
            "map":       map_dist,
            "error_m":   error_m,
            "error_pct": error_pct,
        })
    return results


# ============================================================
#  กราฟ: Bar + Line + Summary
# ============================================================

def plot_results(results: list, output_path: str):
    names     = [r["name"]      for r in results]
    real_d    = [r["real"]      for r in results]
    map_d     = [r["map"]       for r in results]
    error_pct = [r["error_pct"] for r in results]
    error_m   = [r["error_m"]   for r in results]

    mean_err  = np.mean(error_pct)
    std_err   = np.std(error_pct)

    fig = plt.figure(figsize=(16, 10))
    gs  = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.35)

    # ---- Plot 1: Error (%) แต่ละระยะ ----
    ax1 = fig.add_subplot(gs[0, 0])
    colors = ['#F44336' if e > 5 else '#FF9800' if e > 2 else '#4CAF50'
              for e in error_pct]
    bars = ax1.bar(names, error_pct, color=colors, alpha=0.85)
    ax1.axhline(y=2.0, color='#FF9800', linestyle='--',
                alpha=0.7, label='Acceptable = 2%')
    ax1.axhline(y=5.0, color='#F44336', linestyle='--',
                alpha=0.7, label='Poor = 5%')
    ax1.set_title('Distance Error per Measurement', fontweight='bold')
    ax1.set_ylabel('Error (%)')
    ax1.set_xticklabels(names, rotation=30, ha='right')
    ax1.legend(fontsize=9)
    ax1.grid(axis='y', alpha=0.3)
    for bar, val in zip(bars, error_pct):
        ax1.text(bar.get_x() + bar.get_width() / 2,
                 bar.get_height() + 0.05,
                 f'{val:.2f}%',
                 ha='center', va='bottom', fontsize=10, fontweight='bold')

    # ---- Plot 2: Real vs Map distance ----
    ax2 = fig.add_subplot(gs[0, 1])
    x   = np.arange(len(names))
    w   = 0.35
    b1  = ax2.bar(x - w/2, real_d, w, label='Real (m)',
                  color='#2196F3', alpha=0.85)
    b2  = ax2.bar(x + w/2, map_d,  w, label='Map (m)',
                  color='#9C27B0', alpha=0.85)
    ax2.set_title('Real Distance vs Map Distance', fontweight='bold')
    ax2.set_ylabel('Distance (m)')
    ax2.set_xticks(x)
    ax2.set_xticklabels(names, rotation=30, ha='right')
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)
    for bar, val in zip(b1, real_d):
        ax2.text(bar.get_x() + bar.get_width() / 2,
                 bar.get_height() + 0.1,
                 f'{val:.2f}', ha='center', va='bottom', fontsize=9)
    for bar, val in zip(b2, map_d):
        ax2.text(bar.get_x() + bar.get_width() / 2,
                 bar.get_height() + 0.1,
                 f'{val:.2f}', ha='center', va='bottom', fontsize=9)

    # ---- Plot 3: Drift สะสม (Error_m ตามระยะจริง) ----
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(real_d, error_m, 'o-', color='#F44336',
             linewidth=2, markersize=8, label='Error (m)')
    ax3.fill_between(real_d, error_m, alpha=0.15, color='#F44336')
    for x_val, y_val, name in zip(real_d, error_m, names):
        ax3.annotate(f'{name}\n{y_val:.3f}m',
                     (x_val, y_val),
                     textcoords='offset points',
                     xytext=(0, 10), ha='center', fontsize=9)
    ax3.set_title('Accumulated Error vs Distance\n(Drift Analysis)',
                  fontweight='bold')
    ax3.set_xlabel('Real Distance (m)')
    ax3.set_ylabel('Absolute Error (m)')
    ax3.legend()
    ax3.grid(alpha=0.3)

    # ---- Plot 4: Summary ----
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.axis('off')

    # เกณฑ์ความแม่น
    if mean_err < 2:
        grade, color = "ดีมาก ✅", '#4CAF50'
    elif mean_err < 5:
        grade, color = "ยอมรับได้ ✅", '#FF9800'
    elif mean_err < 10:
        grade, color = "พอใช้ ⚠️",  '#FF5722'
    else:
        grade, color = "มีปัญหา ❌", '#F44336'

    summary_data = [
        ['Metric', 'Value'],
        ['Mean Error', f'{mean_err:.2f}%'],
        ['Std Dev',    f'{std_err:.2f}%'],
        ['Min Error',  f'{min(error_pct):.2f}%'],
        ['Max Error',  f'{max(error_pct):.2f}%'],
        ['Assessment', grade],
    ]

    table = ax4.table(
        cellText=summary_data[1:],
        colLabels=summary_data[0],
        cellLoc='center',
        loc='center',
        bbox=[0.1, 0.2, 0.8, 0.7]
    )
    table.auto_set_font_size(False)
    table.set_fontsize(11)

    # ตกแต่ง table
    for (row, col), cell in table.get_celld().items():
        if row == 0:
            cell.set_facecolor('#2196F3')
            cell.set_text_props(color='white', fontweight='bold')
        elif row == len(summary_data) - 1:
            cell.set_facecolor(color)
            cell.set_text_props(color='white', fontweight='bold')
        else:
            cell.set_facecolor('#F5F5F5' if row % 2 == 0 else 'white')
        cell.set_edgecolor('#DDDDDD')

    ax4.set_title('Summary', fontweight='bold')

    fig.suptitle('Map Distance Accuracy Analysis',
                 fontsize=15, fontweight='bold')

    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ บันทึกกราฟ: {output_path}")
    plt.close()


# ============================================================
#  Export CSV
# ============================================================

def export_csv(results: list, output_path: str):
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['Measurement', 'Real_Distance(m)',
                         'Map_Distance(m)', 'Error(m)', 'Error(%)'])
        for r in results:
            writer.writerow([
                r['name'],
                f"{r['real']:.3f}",
                f"{r['map']:.3f}",
                f"{r['error_m']:.4f}",
                f"{r['error_pct']:.2f}",
            ])

        error_pct = [r['error_pct'] for r in results]
        writer.writerow([])
        writer.writerow(['--- Statistics ---'])
        writer.writerow(['Mean Error (%)', f'{np.mean(error_pct):.4f}'])
        writer.writerow(['Std Dev (%)',    f'{np.std(error_pct):.4f}'])
        writer.writerow(['Min Error (%)',  f'{np.min(error_pct):.4f}'])
        writer.writerow(['Max Error (%)',  f'{np.max(error_pct):.4f}'])

        # เกณฑ์
        mean = np.mean(error_pct)
        if mean < 2:
            assess = "ดีมาก (< 2%)"
        elif mean < 5:
            assess = "ยอมรับได้ (2-5%)"
        elif mean < 10:
            assess = "พอใช้ (5-10%)"
        else:
            assess = "มีปัญหา (> 10%)"
        writer.writerow(['Assessment', assess])

    print(f"✅ บันทึก CSV: {output_path}")


# ============================================================
#  พิมพ์ผลสรุปใน Terminal
# ============================================================

def print_summary(results: list):
    print("\n" + "="*55)
    print("  Map Distance Accuracy Results")
    print("="*55)
    print(f"  {'Measurement':<12} {'Real':>8} {'Map':>8} {'Error':>8} {'Error%':>8}")
    print("-"*55)
    for r in results:
        print(f"  {r['name']:<12} {r['real']:>7.3f}m {r['map']:>7.3f}m "
              f"{r['error_m']:>7.4f}m {r['error_pct']:>7.2f}%")

    error_pct = [r['error_pct'] for r in results]
    print("-"*55)
    print(f"  Mean Error : {np.mean(error_pct):.2f}% ± {np.std(error_pct):.2f}%")

    mean = np.mean(error_pct)
    if mean < 2:
        print("  Assessment : ดีมาก ✅ (< 2%)")
    elif mean < 5:
        print("  Assessment : ยอมรับได้ ✅ (2-5%)")
    elif mean < 10:
        print("  Assessment : พอใช้ ⚠️  (5-10%)")
    else:
        print("  Assessment : มีปัญหา ❌ (> 10%)")
    print("="*55)


# ============================================================
#  MAIN
# ============================================================

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("🔄 กำลังคำนวณ distance accuracy...")
    results = compute_accuracy(MEASUREMENTS)

    print_summary(results)

    print("\n📊 กำลังสร้างกราฟ...")
    plot_results(results,
                 os.path.join(OUTPUT_DIR, 'distance_accuracy_results.png'))
    export_csv(results,
               os.path.join(OUTPUT_DIR, 'distance_accuracy.csv'))

    print(f"\n✅ ไฟล์ทั้งหมดบันทึกใน: {OUTPUT_DIR}")
    print("   - distance_accuracy_results.png")
    print("   - distance_accuracy.csv")


if __name__ == '__main__':
    main()
