#!/usr/bin/env python3
"""
Detection Accuracy Analysis Tool
วิเคราะห์ความแม่นยำของ YOLO11 Nano แยกตาม ระยะ × สภาพแสง
สำหรับวิทยานิพนธ์ ป.ตรี

ผลลัพธ์:
  - detection_heatmap.png     (Heatmap Recall ระยะ × แสง)
  - detection_linechart.png   (Line chart Recall vs Distance)
  - detection_metrics.csv     (ตัวเลขทั้งหมด)

วิธีใช้:
  1. ใส่ข้อมูล TP/FP/FN ในส่วน USER INPUT
  2. python3 detection_accuracy.py

นิยามสภาพแสง:
  สว่าง    = > 200 lux  (แสงไฟปกติในอาคาร)
  ปานกลาง = 50-200 lux  (แสงน้อยลง/มุมมืด)
  มืด      = < 50 lux   (กลางคืน/ไฟดับบางส่วน)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
import csv
import os

# ===== Font สำหรับ Thesis =====
plt.rcParams.update({
    'font.family':    'DejaVu Sans',
    'font.size':      12,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
    'figure.dpi':     150,
    'savefig.dpi':    300,
    'savefig.bbox':   'tight',
})


# ============================================================
#  USER INPUT — ใส่ข้อมูลที่เก็บจากการทดสอบจริง
# ============================================================
#
#  โครงสร้าง: {lighting: {distance: (TP, FP, FN)}}
#
#  TP = ตรวจเจอถูก
#  FP = ตรวจเจอแต่ผิด (false alarm)
#  FN = มีอยู่แต่ไม่เจอ (miss)
#
#  ทดสอบ 20 ภาพต่อ cell
#  เช่น TP=18 FP=1 FN=2 จาก 20 ภาพ
# ============================================================

# --- Person Detection ---
PERSON_DATA = {
    #               1.0m          2.0m          3.0m          5.0m          7.0m
    'สว่าง':    {1.0: (19,1,1), 2.0: (18,1,2), 3.0: (17,2,3), 5.0: (14,2,6), 7.0: (10,3,10)},
    'ปานกลาง': {1.0: (18,2,2), 2.0: (16,2,4), 3.0: (14,3,6), 5.0: (11,3,9), 7.0: ( 7,4,13)},
    'มืด':     {1.0: (15,3,5), 2.0: (13,3,7), 3.0: (10,4,10), 5.0: (6,4,14), 7.0: ( 3,5,17)},
}

# --- Door_Open Detection ---
DOOR_OPEN_DATA = {
    #               1.0m          2.0m          3.0m          5.0m          7.0m
    'สว่าง':    {1.0: (19,0,1), 2.0: (18,1,2), 3.0: (17,1,3), 5.0: (15,2,5), 7.0: (11,2, 9)},
    'ปานกลาง': {1.0: (18,1,2), 2.0: (17,1,3), 3.0: (15,2,5), 5.0: (12,2,8), 7.0: ( 8,3,12)},
    'มืด':     {1.0: (16,2,4), 2.0: (14,2,6), 3.0: (11,3,9), 5.0: (7,4,13), 7.0: ( 4,4,16)},
}

# --- Door_Closed Detection ---
DOOR_CLOSED_DATA = {
    #               1.0m          2.0m          3.0m          5.0m          7.0m
    'สว่าง':    {1.0: (20,0,0), 2.0: (19,0,1), 3.0: (19,1,1), 5.0: (17,1,3), 7.0: (14,1, 6)},
    'ปานกลาง': {1.0: (19,0,1), 2.0: (19,1,1), 3.0: (18,1,2), 5.0: (15,1,5), 7.0: (11,2, 9)},
    'มืด':     {1.0: (18,1,2), 2.0: (17,1,3), 3.0: (15,2,5), 5.0: (11,3,9), 7.0: ( 7,3,13)},
}

# โฟลเดอร์ output
OUTPUT_DIR = "."

# ============================================================
#  คำนวณ Metrics
# ============================================================

DISTANCES = [1.0, 2.0, 3.0, 5.0, 7.0]
LIGHTINGS = ['สว่าง', 'ปานกลาง', 'มืด']
LIGHTING_EN = {'สว่าง': 'Bright (>200lux)',
               'ปานกลาง': 'Medium (50-200lux)',
               'มืด': 'Dark (<50lux)'}
LIGHT_COLORS = {'สว่าง': '#FF9800', 'ปานกลาง': '#2196F3', 'มืด': '#673AB7'}


def precision(tp, fp):
    return tp / (tp + fp) if (tp + fp) > 0 else 0.0


def recall(tp, fn):
    return tp / (tp + fn) if (tp + fn) > 0 else 0.0


def f1(p, r):
    return 2 * p * r / (p + r) if (p + r) > 0 else 0.0


def build_matrix(data: dict, metric_fn) -> np.ndarray:
    """สร้าง matrix (lighting × distance) จาก metric function"""
    mat = np.zeros((len(LIGHTINGS), len(DISTANCES)))
    for i, light in enumerate(LIGHTINGS):
        for j, dist in enumerate(DISTANCES):
            tp, fp, fn = data[light][dist]
            mat[i, j] = metric_fn(tp, fp, fn)
    return mat


def recall_matrix(data):
    return build_matrix(data, lambda tp, fp, fn: recall(tp, fn))


def precision_matrix(data):
    return build_matrix(data, lambda tp, fp, fn: precision(tp, fp))


# ============================================================
#  กราฟที่ 1: Heatmap (Recall)
# ============================================================

def plot_heatmap(output_path: str):
    datasets = [
        ('Person Detection', PERSON_DATA),
        ('Door Open Detection', DOOR_OPEN_DATA),
        ('Door Closed Detection', DOOR_CLOSED_DATA),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle('Detection Recall Heatmap\n(Distance × Lighting Condition)',
                 fontsize=15, fontweight='bold')

    # Colormap: แดง → เหลือง → เขียว
    cmap = LinearSegmentedColormap.from_list(
        'recall', ['#d7191c', '#ffffbf', '#1a9641'], N=256)

    for ax, (title, data) in zip(axes, datasets):
        mat = recall_matrix(data)

        im = ax.imshow(mat, cmap=cmap, vmin=0, vmax=1, aspect='auto')

        # ใส่ตัวเลขใน cell
        for i in range(len(LIGHTINGS)):
            for j in range(len(DISTANCES)):
                val = mat[i, j]
                color = 'white' if val < 0.4 or val > 0.85 else 'black'
                ax.text(j, i, f'{val:.2f}',
                        ha='center', va='center',
                        fontsize=11, fontweight='bold', color=color)

        ax.set_xticks(range(len(DISTANCES)))
        ax.set_xticklabels([f'{d}m' for d in DISTANCES])
        ax.set_yticks(range(len(LIGHTINGS)))
        ax.set_yticklabels(LIGHTINGS)
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel('Lighting')
        ax.set_title(title, fontweight='bold')

        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04,
                     label='Recall')

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ บันทึก Heatmap: {output_path}")
    plt.close()


# ============================================================
#  กราฟที่ 2: Line Chart (Recall vs Distance แยกแสง)
# ============================================================

def plot_linechart(output_path: str):
    datasets = [
        ('Person Detection', PERSON_DATA),
        ('Door Open Detection', DOOR_OPEN_DATA),
        ('Door Closed Detection', DOOR_CLOSED_DATA),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('Recall vs Distance by Lighting Condition',
                 fontsize=15, fontweight='bold')

    for ax, (title, data) in zip(axes, datasets):
        mat = recall_matrix(data)

        for i, light in enumerate(LIGHTINGS):
            ax.plot(DISTANCES, mat[i],
                    'o-', linewidth=2, markersize=8,
                    color=LIGHT_COLORS[light],
                    label=LIGHTING_EN[light])
            # ใส่ค่าเหนือจุด
            for j, (d, v) in enumerate(zip(DISTANCES, mat[i])):
                ax.annotate(f'{v:.2f}',
                            (d, v),
                            textcoords='offset points',
                            xytext=(0, 8),
                            ha='center', fontsize=9)

        # เส้นเกณฑ์ Recall ≥ 0.80
        ax.axhline(y=0.80, color='red', linestyle='--',
                   alpha=0.6, label='Threshold = 0.80')
        ax.fill_between(DISTANCES, [0.80]*len(DISTANCES), [1.0]*len(DISTANCES),
                        alpha=0.05, color='green')

        ax.set_title(title, fontweight='bold')
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel('Recall')
        ax.set_ylim(0, 1.15)
        ax.set_xticks(DISTANCES)
        ax.set_xticklabels([f'{d}m' for d in DISTANCES])
        ax.legend(fontsize=9)
        ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ บันทึก Line Chart: {output_path}")
    plt.close()


# ============================================================
#  กราฟที่ 3: Summary Bar (Precision + Recall + F1 รวม)
# ============================================================

def plot_summary(output_path: str):
    """สรุป Precision / Recall / F1 เฉลี่ยทุกสภาพ"""
    datasets = {
        'Person':      PERSON_DATA,
        'Door Open':   DOOR_OPEN_DATA,
        'Door Closed': DOOR_CLOSED_DATA,
    }

    labels, prec_vals, rec_vals, f1_vals = [], [], [], []

    for name, data in datasets.items():
        tp_all = fp_all = fn_all = 0
        for light in LIGHTINGS:
            for dist in DISTANCES:
                tp, fp, fn = data[light][dist]
                tp_all += tp; fp_all += fp; fn_all += fn
        p = precision(tp_all, fp_all)
        r = recall(tp_all, fn_all)
        labels.append(name)
        prec_vals.append(p)
        rec_vals.append(r)
        f1_vals.append(f1(p, r))

    x = np.arange(len(labels))
    w = 0.25

    fig, ax = plt.subplots(figsize=(10, 6))
    b1 = ax.bar(x - w, prec_vals, w, label='Precision',
                color='#2196F3', alpha=0.85)
    b2 = ax.bar(x,     rec_vals,  w, label='Recall',
                color='#4CAF50', alpha=0.85)
    b3 = ax.bar(x + w, f1_vals,   w, label='F1-Score',
                color='#FF9800', alpha=0.85)

    ax.axhline(y=0.80, color='red', linestyle='--',
               alpha=0.6, label='Threshold = 0.80')
    ax.set_title('Overall Detection Performance\n(Average across all distances & lighting)',
                 fontweight='bold')
    ax.set_ylabel('Score')
    ax.set_ylim(0, 1.15)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    ax.grid(axis='y', alpha=0.3)

    for bars in [b1, b2, b3]:
        for bar in bars:
            ax.text(bar.get_x() + bar.get_width()/2,
                    bar.get_height() + 0.01,
                    f'{bar.get_height():.3f}',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ บันทึก Summary: {output_path}")
    plt.close()


# ============================================================
#  Export CSV
# ============================================================

def export_csv(output_path: str):
    datasets = {
        'Person':      PERSON_DATA,
        'Door_Open':   DOOR_OPEN_DATA,
        'Door_Closed': DOOR_CLOSED_DATA,
    }

    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['Model', 'Lighting', 'Distance(m)',
                         'TP', 'FP', 'FN',
                         'Precision', 'Recall', 'F1'])

        for model_name, data in datasets.items():
            for light in LIGHTINGS:
                for dist in DISTANCES:
                    tp, fp, fn = data[light][dist]
                    p = precision(tp, fp)
                    r = recall(tp, fn)
                    writer.writerow([
                        model_name, light, dist,
                        tp, fp, fn,
                        f'{p:.4f}', f'{r:.4f}', f'{f1(p,r):.4f}'
                    ])
            writer.writerow([])

        # Overall summary
        writer.writerow(['--- Overall Summary ---'])
        writer.writerow(['Model', 'Precision', 'Recall', 'F1'])
        for model_name, data in datasets.items():
            tp_all = fp_all = fn_all = 0
            for light in LIGHTINGS:
                for dist in DISTANCES:
                    tp, fp, fn = data[light][dist]
                    tp_all += tp; fp_all += fp; fn_all += fn
            p = precision(tp_all, fp_all)
            r = recall(tp_all, fn_all)
            writer.writerow([model_name,
                             f'{p:.4f}', f'{r:.4f}', f'{f1(p,r):.4f}'])

        # Effective detection range
        writer.writerow([])
        writer.writerow(['--- Effective Detection Range (Recall >= 0.80) ---'])
        writer.writerow(['Model', 'Lighting', 'Max Distance (m)'])
        for model_name, data in datasets.items():
            for light in LIGHTINGS:
                max_dist = 0
                for dist in DISTANCES:
                    tp, fp, fn = data[light][dist]
                    if recall(tp, fn) >= 0.80:
                        max_dist = dist
                writer.writerow([model_name, light,
                                 f'{max_dist}m' if max_dist > 0 else 'N/A'])

    print(f"✅ บันทึก CSV: {output_path}")


# ============================================================
#  Print Summary Terminal
# ============================================================

def print_summary():
    datasets = {
        'Person':      PERSON_DATA,
        'Door_Open':   DOOR_OPEN_DATA,
        'Door_Closed': DOOR_CLOSED_DATA,
    }

    print("\n" + "="*60)
    print("  Detection Accuracy Summary")
    print("="*60)

    for model_name, data in datasets.items():
        print(f"\n  [{model_name}]")
        print(f"  {'Lighting':<12} {'1m':>6} {'2m':>6} "
              f"{'3m':>6} {'5m':>6} {'7m':>6}")
        print("  " + "-"*45)
        for light in LIGHTINGS:
            row = f"  {light:<12}"
            for dist in DISTANCES:
                tp, fp, fn = data[light][dist]
                r = recall(tp, fn)
                row += f" {r:>5.2f}"
            print(row)

    # Effective Range
    print("\n" + "="*60)
    print("  Effective Detection Range (Recall >= 0.80)")
    print("="*60)
    for model_name, data in datasets.items():
        print(f"\n  [{model_name}]")
        for light in LIGHTINGS:
            max_dist = 0
            for dist in DISTANCES:
                tp, fp, fn = data[light][dist]
                if recall(tp, fn) >= 0.80:
                    max_dist = dist
            result = f'{max_dist}m' if max_dist > 0 else 'N/A'
            print(f"  {light:<12} → {result}")

    print("="*60)


# ============================================================
#  MAIN
# ============================================================

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("🔄 กำลังคำนวณ metrics...")
    print_summary()

    print("\n📊 กำลังสร้างกราฟ...")
    plot_heatmap(
        os.path.join(OUTPUT_DIR, 'detection_heatmap.png'))
    plot_linechart(
        os.path.join(OUTPUT_DIR, 'detection_linechart.png'))
    plot_summary(
        os.path.join(OUTPUT_DIR, 'detection_summary.png'))
    export_csv(
        os.path.join(OUTPUT_DIR, 'detection_metrics.csv'))

    print(f"\n✅ ไฟล์ทั้งหมดบันทึกใน: {OUTPUT_DIR}")
    print("   - detection_heatmap.png    (Recall ระยะ × แสง)")
    print("   - detection_linechart.png  (Recall vs Distance)")
    print("   - detection_summary.png    (Precision/Recall/F1 รวม)")
    print("   - detection_metrics.csv    (ตัวเลขทั้งหมด)")


if __name__ == '__main__':
    main()
