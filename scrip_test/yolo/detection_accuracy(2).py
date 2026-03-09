#!/usr/bin/env python3
"""
Detection Accuracy Analysis Tool
อ่านข้อมูลจาก Excel แล้วสร้างกราฟสำหรับวิทยานิพนธ์ ป.ตรี

วิธีใช้:
  python3 detection_accuracy.py
  หรือระบุไฟล์ Excel:
  python3 detection_accuracy.py --excel detection_accuracy_template.xlsx
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import pandas as pd
import csv
import os
import argparse

plt.rcParams.update({
    'font.family':    'DejaVu Sans',
    'font.size':      12,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
    'figure.dpi':     150,
    'savefig.dpi':    300,
    'savefig.bbox':   'tight',
})

DISTANCES  = [1.0, 2.0, 3.0, 5.0, 7.0]
LIGHTINGS  = ['สว่าง', 'ปานกลาง', 'มืด', 'มืด+สปอต']
MODELS     = ['Person', 'Door_Open', 'Door_Closed']
OUTPUT_DIR = '.'

LIGHTING_EN = {
    'สว่าง':      'Bright (>200lux)',
    'ปานกลาง':   'Medium (50-200lux)',
    'มืด':       'Dark (<50lux)',
    'มืด+สปอต': 'Dark + Spotlight',
}
LIGHT_COLORS = {
    'สว่าง':      '#FF9800',
    'ปานกลาง':   '#2196F3',
    'มืด':       '#673AB7',
    'มืด+สปอต': '#E91E63',
}
LIGHT_MARKERS = {
    'สว่าง':      'o',
    'ปานกลาง':   's',
    'มืด':       '^',
    'มืด+สปอต': 'D',
}


# ============================================================
#  โหลดข้อมูลจาก Excel
# ============================================================

def load_from_excel(filepath: str) -> dict:
    """
    อ่านข้อมูล TP/FP/FN จาก Excel template
    คืนค่า dict: {model: {lighting: {distance: (TP, FP, FN)}}}

    โครงสร้าง Excel แต่ละ sheet:
      Row 4  = สว่าง     TP  (col C=1m, F=2m, I=3m, L=5m, O=7m)
      Row 5  = สว่าง     FP
      Row 6  = สว่าง     FN
      Row 7  = (blank separator)
      Row 8  = ปานกลาง  TP
      ...ต่อไปเรื่อยๆ
    """
    print(f"📂 กำลังโหลดข้อมูลจาก: {filepath}")

    LIGHT_ROW_OFFSET = {'สว่าง': 0, 'ปานกลาง': 4, 'มืด': 8, 'มืด+สปอต': 12}
    DIST_COL         = {1.0: 2, 2.0: 5, 3.0: 8, 5.0: 11, 7.0: 14}  # 0-indexed
    SUB_ROW          = {'TP': 0, 'FP': 1, 'FN': 2}
    BASE_ROW         = 3  # row index 3 = Excel row 4

    all_data = {}

    for model_name in MODELS:
        try:
            df = pd.read_excel(filepath, sheet_name=model_name,
                               header=None, engine='openpyxl')
        except Exception as e:
            print(f"  ⚠️  ไม่พบ sheet '{model_name}': {e}")
            continue

        model_data = {}
        for light in LIGHTINGS:
            light_data = {}
            offset = LIGHT_ROW_OFFSET[light]
            for dist in DISTANCES:
                col = DIST_COL[dist]
                try:
                    tp = int(df.iloc[BASE_ROW + offset + SUB_ROW['TP'], col] or 0)
                    fp = int(df.iloc[BASE_ROW + offset + SUB_ROW['FP'], col] or 0)
                    fn = int(df.iloc[BASE_ROW + offset + SUB_ROW['FN'], col] or 0)
                except (IndexError, ValueError, TypeError):
                    tp, fp, fn = 0, 0, 0
                light_data[dist] = (tp, fp, fn)
            model_data[light] = light_data

        all_data[model_name] = model_data
        print(f"  ✅ โหลด {model_name} สำเร็จ")

    return all_data


# ============================================================
#  Metrics
# ============================================================

def precision(tp, fp): return tp / (tp + fp) if (tp + fp) > 0 else 0.0
def recall(tp, fn):    return tp / (tp + fn) if (tp + fn) > 0 else 0.0
def f1(p, r):          return 2*p*r / (p+r)  if (p  + r)  > 0 else 0.0

def recall_matrix(data):
    mat = np.zeros((len(LIGHTINGS), len(DISTANCES)))
    for i, light in enumerate(LIGHTINGS):
        for j, dist in enumerate(DISTANCES):
            tp, fp, fn = data[light][dist]
            mat[i, j] = recall(tp, fn)
    return mat


# ============================================================
#  กราฟที่ 1: Heatmap
# ============================================================

def plot_heatmap(all_data, output_path):
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    fig.suptitle('Detection Recall Heatmap\n(Distance × Lighting Condition)',
                 fontsize=15, fontweight='bold')
    cmap = LinearSegmentedColormap.from_list('recall', ['#d7191c','#ffffbf','#1a9641'], N=256)

    for ax, model_name in zip(axes, MODELS):
        mat = recall_matrix(all_data[model_name])
        im  = ax.imshow(mat, cmap=cmap, vmin=0, vmax=1, aspect='auto')
        for i in range(len(LIGHTINGS)):
            for j in range(len(DISTANCES)):
                v = mat[i, j]
                ax.text(j, i, f'{v:.2f}', ha='center', va='center',
                        fontsize=11, fontweight='bold',
                        color='white' if v < 0.4 or v > 0.85 else 'black')
        ax.set_xticks(range(len(DISTANCES)))
        ax.set_xticklabels([f'{d}m' for d in DISTANCES])
        ax.set_yticks(range(len(LIGHTINGS)))
        ax.set_yticklabels(LIGHTINGS)
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel('Lighting')
        ax.set_title(model_name.replace('_',' '), fontweight='bold')
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label='Recall')

    plt.tight_layout()
    plt.savefig(output_path)
    print(f"✅ บันทึก Heatmap: {output_path}")
    plt.close()


# ============================================================
#  กราฟที่ 2: Line Chart
# ============================================================

def plot_linechart(all_data, output_path):
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    fig.suptitle('Recall vs Distance by Lighting Condition',
                 fontsize=15, fontweight='bold')

    for ax, model_name in zip(axes, MODELS):
        mat = recall_matrix(all_data[model_name])
        for i, light in enumerate(LIGHTINGS):
            ax.plot(DISTANCES, mat[i],
                    marker=LIGHT_MARKERS[light],
                    linestyle='--' if light == 'มืด+สปอต' else '-',
                    linewidth=2, markersize=8,
                    color=LIGHT_COLORS[light], label=LIGHTING_EN[light])
            for d, v in zip(DISTANCES, mat[i]):
                ax.annotate(f'{v:.2f}', (d, v), textcoords='offset points',
                            xytext=(0, 8), ha='center', fontsize=9)
        ax.axhline(y=0.80, color='red', linestyle='--', alpha=0.6, label='Threshold = 0.80')
        ax.fill_between(DISTANCES, [0.80]*5, [1.0]*5, alpha=0.05, color='green')
        ax.set_title(model_name.replace('_',' '), fontweight='bold')
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel('Recall')
        ax.set_ylim(0, 1.15)
        ax.set_xticks(DISTANCES)
        ax.legend(fontsize=9)
        ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path)
    print(f"✅ บันทึก Line Chart: {output_path}")
    plt.close()


# ============================================================
#  กราฟที่ 3: Summary Bar
# ============================================================

def plot_summary(all_data, output_path):
    labels, prec_vals, rec_vals, f1_vals = [], [], [], []
    for model_name in MODELS:
        tp_all = fp_all = fn_all = 0
        for light in LIGHTINGS:
            for dist in DISTANCES:
                tp, fp, fn = all_data[model_name][light][dist]
                tp_all += tp; fp_all += fp; fn_all += fn
        p = precision(tp_all, fp_all)
        r = recall(tp_all, fn_all)
        labels.append(model_name.replace('_',' '))
        prec_vals.append(p); rec_vals.append(r); f1_vals.append(f1(p, r))

    x, w = np.arange(len(labels)), 0.25
    fig, ax = plt.subplots(figsize=(10, 6))
    b1 = ax.bar(x-w, prec_vals, w, label='Precision', color='#2196F3', alpha=0.85)
    b2 = ax.bar(x,   rec_vals,  w, label='Recall',    color='#4CAF50', alpha=0.85)
    b3 = ax.bar(x+w, f1_vals,   w, label='F1-Score',  color='#FF9800', alpha=0.85)
    ax.axhline(y=0.80, color='red', linestyle='--', alpha=0.6, label='Threshold = 0.80')
    ax.set_title('Overall Detection Performance', fontweight='bold')
    ax.set_ylabel('Score'); ax.set_ylim(0, 1.15)
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.legend(); ax.grid(axis='y', alpha=0.3)
    for bars in [b1, b2, b3]:
        for bar in bars:
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.01,
                    f'{bar.get_height():.3f}', ha='center', va='bottom',
                    fontsize=10, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path)
    print(f"✅ บันทึก Summary: {output_path}")
    plt.close()


# ============================================================
#  กราฟที่ 4: Spotlight Impact
# ============================================================

def plot_spotlight_impact(all_data, output_path):
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('Spotlight Impact on Recall\n(Dark vs Dark + Spotlight)',
                 fontsize=15, fontweight='bold')

    for ax, model_name in zip(axes, MODELS):
        data = all_data[model_name]
        r_dark = [recall(data['มืด'][d][0], data['มืด'][d][2]) for d in DISTANCES]
        r_spot = [recall(data['มืด+สปอต'][d][0], data['มืด+สปอต'][d][2]) for d in DISTANCES]

        x, w = np.arange(len(DISTANCES)), 0.3
        b1 = ax.bar(x-w/2, r_dark, w, label='Dark',             color='#673AB7', alpha=0.8)
        b2 = ax.bar(x+w/2, r_spot, w, label='Dark + Spotlight', color='#E91E63', alpha=0.8)

        for i, (rd, rs) in enumerate(zip(r_dark, r_spot)):
            imp = rs - rd
            if imp > 0:
                ax.annotate('', xy=(i+w/2, rs), xytext=(i+w/2, rd),
                            arrowprops=dict(arrowstyle='->', color='green', lw=1.5))
                ax.text(i+w/2+0.15, (rd+rs)/2, f'+{imp:.2f}',
                        color='green', fontsize=8, fontweight='bold')

        ax.axhline(y=0.80, color='red', linestyle='--', alpha=0.6, label='Threshold = 0.80')
        ax.set_title(model_name.replace('_',' '), fontweight='bold')
        ax.set_ylabel('Recall'); ax.set_ylim(0, 1.15)
        ax.set_xticks(x); ax.set_xticklabels([f'{d}m' for d in DISTANCES])
        ax.set_xlabel('Distance (m)'); ax.legend(fontsize=9); ax.grid(axis='y', alpha=0.3)
        for bars in [b1, b2]:
            for bar in bars:
                ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.01,
                        f'{bar.get_height():.2f}', ha='center', va='bottom', fontsize=8)

    plt.tight_layout()
    plt.savefig(output_path)
    print(f"✅ บันทึก Spotlight Impact: {output_path}")
    plt.close()


# ============================================================
#  Export CSV
# ============================================================

def export_csv(all_data, output_path):
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['Model','Lighting','Distance(m)','TP','FP','FN','Precision','Recall','F1'])
        for model_name in MODELS:
            for light in LIGHTINGS:
                for dist in DISTANCES:
                    tp, fp, fn = all_data[model_name][light][dist]
                    p = precision(tp, fp); r = recall(tp, fn)
                    w.writerow([model_name, light, dist, tp, fp, fn,
                                f'{p:.4f}', f'{r:.4f}', f'{f1(p,r):.4f}'])
            w.writerow([])
        w.writerow(['--- Overall Summary ---'])
        w.writerow(['Model','Precision','Recall','F1'])
        for model_name in MODELS:
            tp_all = fp_all = fn_all = 0
            for light in LIGHTINGS:
                for dist in DISTANCES:
                    tp, fp, fn = all_data[model_name][light][dist]
                    tp_all += tp; fp_all += fp; fn_all += fn
            p = precision(tp_all, fp_all); r = recall(tp_all, fn_all)
            w.writerow([model_name, f'{p:.4f}', f'{r:.4f}', f'{f1(p,r):.4f}'])
    print(f"✅ บันทึก CSV: {output_path}")


# ============================================================
#  Terminal Summary
# ============================================================

def print_summary(all_data):
    print("\n" + "="*65)
    print("  Detection Accuracy Summary (Recall)")
    print("="*65)
    for model_name in MODELS:
        data = all_data[model_name]
        print(f"\n  [{model_name}]")
        print(f"  {'Lighting':<15} {'1m':>6} {'2m':>6} {'3m':>6} {'5m':>6} {'7m':>6}")
        print("  " + "-"*48)
        for light in LIGHTINGS:
            row = f"  {light:<15}"
            for dist in DISTANCES:
                tp, fp, fn = data[light][dist]
                row += f" {recall(tp,fn):>5.2f}"
            print(row)
    print("="*65)


# ============================================================
#  MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--excel', default='detection_accuracy_template.xlsx')
    args = parser.parse_args()

    if not os.path.exists(args.excel):
        print(f"❌ ไม่พบไฟล์: {args.excel}")
        return

    all_data = load_from_excel(args.excel)
    if not all_data:
        print("❌ โหลดข้อมูลไม่ได้")
        return

    print_summary(all_data)
    print("\n📊 กำลังสร้างกราฟ...")

    plot_heatmap(all_data,          os.path.join(OUTPUT_DIR, 'detection_heatmap.png'))
    plot_linechart(all_data,        os.path.join(OUTPUT_DIR, 'detection_linechart.png'))
    plot_summary(all_data,          os.path.join(OUTPUT_DIR, 'detection_summary.png'))
    plot_spotlight_impact(all_data, os.path.join(OUTPUT_DIR, 'detection_spotlight.png'))
    export_csv(all_data,            os.path.join(OUTPUT_DIR, 'detection_metrics.csv'))

    print(f"\n✅ เสร็จสิ้น ไฟล์ทั้งหมดอยู่ใน: {OUTPUT_DIR}")


if __name__ == '__main__':
    main()
