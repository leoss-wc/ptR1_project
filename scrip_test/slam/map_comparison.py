#!/usr/bin/env python3
"""
Map Consistency Analysis Tool
สำหรับวัดความแตกต่างของแผนที่ที่สร้างซ้ำๆ และแสดงเป็นกราฟวิทยานิพนธ์
รองรับไฟล์ .pgm (ROS map_saver format)

วิธีใช้:
  python3 map_comparison.py --maps map1.pgm map2.pgm map3.pgm --reference map1.pgm
  python3 map_comparison.py --folder ./maps/  (โหลดทุก .pgm ในโฟลเดอร์)

ผลลัพธ์:
  - map_comparison_results.png  (กราฟหลัก)
  - map_diff_heatmap.png        (Heatmap ความแตกต่าง)
  - map_metrics.csv             (ตัวเลขสำหรับใส่ตาราง thesis)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
import cv2
import os
import glob
import argparse
import csv
from pathlib import Path
from skimage.metrics import structural_similarity as ssim

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
    'savefig.dpi': 300,        # สำหรับ thesis ต้องการ 300 DPI
    'savefig.bbox': 'tight',
})

# ===== สี Theme สำหรับ Thesis =====
COLORS = ['#2196F3', '#F44336', '#4CAF50', '#FF9800', '#9C27B0',
          '#00BCD4', '#795548', '#607D8B', '#E91E63', '#3F51B5']


# ============================================================
#  SECTION 1: โหลดและ Preprocess Map
# ============================================================

def load_map(filepath: str) -> np.ndarray:
    """โหลด .pgm map และ normalize เป็น binary (0=free, 1=occupied, 0.5=unknown)"""
    img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"ไม่พบไฟล์: {filepath}")

    # ROS map convention:
    # 205 = unknown (-1), 0 = occupied (100), 254 = free (0)
    normalized = np.zeros_like(img, dtype=np.float32)
    normalized[img == 254] = 1.0    # free space → 1
    normalized[img == 0]   = 0.0    # occupied   → 0
    normalized[img == 205] = 0.5    # unknown    → 0.5
    return normalized


def align_maps(ref: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Resize target map ให้ตรงกับ reference (กรณี map ขนาดต่างกัน)"""
    if ref.shape != target.shape:
        target = cv2.resize(target, (ref.shape[1], ref.shape[0]),
                            interpolation=cv2.INTER_NEAREST)
    return target


# ============================================================
#  SECTION 2: คำนวณ Metrics
# ============================================================

def compute_metrics(ref: np.ndarray, target: np.ndarray, name: str) -> dict:
    """คำนวณ metrics ทั้งหมดระหว่าง 2 map"""
    target_aligned = align_maps(ref, target)

    # --- Mask เฉพาะพื้นที่ที่รู้จัก (ไม่รวม unknown) ---
    known_mask = (ref != 0.5) & (target_aligned != 0.5)
    ref_known     = ref[known_mask]
    target_known  = target_aligned[known_mask]

    # 1. Mean Absolute Error (MAE)
    mae = np.mean(np.abs(ref_known - target_known))

    # 2. Root Mean Square Error (RMSE)
    rmse = np.sqrt(np.mean((ref_known - target_known) ** 2))

    # 3. Structural Similarity Index (SSIM)
    # ใช้ full map รวม unknown (SSIM ต้องการ 2D array)
    ssim_score, _ = ssim(ref, target_aligned,
                         data_range=1.0, full=True)

    # 4. Map Coverage (% พื้นที่ที่ scan ได้)
    coverage_ref    = np.sum(ref != 0.5) / ref.size * 100
    coverage_target = np.sum(target_aligned != 0.5) / target_aligned.size * 100

    # 5. Occupied Cell Difference (ผนัง/สิ่งกีดขวางต่างกันกี่ %)
    occ_ref    = np.sum(ref == 0.0)
    occ_target = np.sum(target_aligned == 0.0)
    occ_diff   = abs(occ_ref - occ_target) / max(occ_ref, 1) * 100

    # 6. Pixel Accuracy (% pixel ที่ตรงกัน)
    pixel_acc = np.sum(ref_known == target_known) / len(ref_known) * 100

    # 7. Difference Map (สำหรับ Heatmap)
    diff_map = np.abs(ref.astype(float) - target_aligned.astype(float))

    return {
        'name':            name,
        'mae':             mae,
        'rmse':            rmse,
        'ssim':            ssim_score,
        'coverage_ref':    coverage_ref,
        'coverage_target': coverage_target,
        'occ_diff_pct':    occ_diff,
        'pixel_acc':       pixel_acc,
        'diff_map':        diff_map,
        'target_aligned':  target_aligned,
    }


# ============================================================
#  SECTION 3: สร้างกราฟหลัก (Figure 1)
# ============================================================

def plot_main_results(all_metrics: list, ref_name: str, output_path: str):
    """กราฟหลัก 4 กราฟรวมในหน้าเดียว สำหรับ thesis"""

    names   = [m['name']    for m in all_metrics]
    mae     = [m['mae']     for m in all_metrics]
    rmse    = [m['rmse']    for m in all_metrics]
    ssim_s  = [m['ssim']    for m in all_metrics]
    pix_acc = [m['pixel_acc'] for m in all_metrics]
    occ_d   = [m['occ_diff_pct'] for m in all_metrics]
    cov     = [m['coverage_target'] for m in all_metrics]

    x = np.arange(len(names))
    fig = plt.figure(figsize=(16, 12))
    gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # ---- Plot 1: MAE & RMSE ----
    ax1 = fig.add_subplot(gs[0, 0])
    w   = 0.35
    bars1 = ax1.bar(x - w/2, mae,  w, label='MAE',  color=COLORS[0], alpha=0.85)
    bars2 = ax1.bar(x + w/2, rmse, w, label='RMSE', color=COLORS[1], alpha=0.85)
    ax1.set_title('Map Error: MAE and RMSE', fontweight='bold')
    ax1.set_ylabel('Error (normalized pixel value)')
    ax1.set_xticks(x); ax1.set_xticklabels(names, rotation=30, ha='right')
    ax1.legend()
    ax1.grid(axis='y', alpha=0.3)
    for bar in bars1:
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                 f'{bar.get_height():.4f}', ha='center', va='bottom', fontsize=9)
    for bar in bars2:
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                 f'{bar.get_height():.4f}', ha='center', va='bottom', fontsize=9)

    # ---- Plot 2: SSIM ----
    ax2  = fig.add_subplot(gs[0, 1])
    bars = ax2.bar(x, ssim_s, color=COLORS[2], alpha=0.85)
    ax2.set_title('Structural Similarity Index (SSIM)', fontweight='bold')
    ax2.set_ylabel('SSIM Score (higher = more similar)')
    ax2.set_ylim(0, 1.1)
    ax2.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5, label='Perfect = 1.0')
    ax2.set_xticks(x); ax2.set_xticklabels(names, rotation=30, ha='right')
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)
    for bar in bars:
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                 f'{bar.get_height():.4f}', ha='center', va='bottom', fontsize=9)

    # ---- Plot 3: Pixel Accuracy ----
    ax3  = fig.add_subplot(gs[1, 0])
    bars = ax3.bar(x, pix_acc, color=COLORS[3], alpha=0.85)
    ax3.set_title('Pixel Accuracy vs Reference Map', fontweight='bold')
    ax3.set_ylabel('Accuracy (%)')
    ax3.set_ylim(0, 110)
    ax3.axhline(y=100, color='gray', linestyle='--', alpha=0.5)
    ax3.set_xticks(x); ax3.set_xticklabels(names, rotation=30, ha='right')
    ax3.grid(axis='y', alpha=0.3)
    for bar in bars:
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                 f'{bar.get_height():.2f}%', ha='center', va='bottom', fontsize=9)

    # ---- Plot 4: Occupied Cell Difference ----
    ax4  = fig.add_subplot(gs[1, 1])
    bars = ax4.bar(x, occ_d, color=COLORS[4], alpha=0.85)
    ax4.set_title('Occupied Cell Difference\n(Wall/Obstacle Change)', fontweight='bold')
    ax4.set_ylabel('Difference (%)')
    ax4.set_xticks(x); ax4.set_xticklabels(names, rotation=30, ha='right')
    ax4.grid(axis='y', alpha=0.3)
    for bar in bars:
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                 f'{bar.get_height():.2f}%', ha='center', va='bottom', fontsize=9)

    # ---- Plot 5: Map Coverage ----
    ax5  = fig.add_subplot(gs[2, :])
    ax5.plot(names, cov, 'o-', color=COLORS[0], linewidth=2,
             markersize=8, label='Map Coverage (%)')
    ref_cov = all_metrics[0]['coverage_ref']
    ax5.axhline(y=ref_cov, color=COLORS[1], linestyle='--',
                alpha=0.7, label=f'Reference Coverage = {ref_cov:.1f}%')
    ax5.fill_between(names, [ref_cov]*len(names), cov,
                     alpha=0.15, color=COLORS[0])
    ax5.set_title('Map Coverage Across Trials', fontweight='bold')
    ax5.set_ylabel('Coverage (%)')
    ax5.set_xlabel('Trial')
    ax5.set_ylim(0, 110)
    ax5.legend()
    ax5.grid(alpha=0.3)
    for i, (n, v) in enumerate(zip(names, cov)):
        ax5.annotate(f'{v:.1f}%', (n, v), textcoords='offset points',
                     xytext=(0, 10), ha='center', fontsize=9)

    fig.suptitle(f'Map Consistency Analysis\n(Reference: {ref_name})',
                 fontsize=16, fontweight='bold', y=1.01)

    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"บันทึกกราฟหลัก: {output_path}")
    plt.close()


# ============================================================
#  SECTION 4: Heatmap ความแตกต่าง (Figure 2)
# ============================================================

def plot_heatmap(ref: np.ndarray, all_metrics: list, output_path: str):
    """แสดง difference heatmap ของแต่ละ trial เทียบกับ reference"""

    n = len(all_metrics)
    cols = min(3, n)
    rows = (n + cols - 1) // cols + 1  # +1 สำหรับ reference

    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5, rows * 4.5))
    if rows == 1:
        axes = [axes] if cols == 1 else list(axes)
    else:
        axes = [ax for row in axes for ax in (row if hasattr(row, '__iter__') else [row])]

    # Custom colormap: สีเขียว=เหมือน, สีแดง=ต่าง
    cmap = LinearSegmentedColormap.from_list(
        'diff_cmap', ['#1a9641', '#ffffbf', '#d7191c'], N=256)

    # แสดง Reference map ก่อน
    axes[0].imshow(ref, cmap='gray', vmin=0, vmax=1)
    axes[0].set_title('Reference Map', fontweight='bold', color='navy')
    axes[0].axis('off')

    # แสดง Difference heatmap แต่ละ trial
    for i, m in enumerate(all_metrics):
        ax = axes[i + 1]
        im = ax.imshow(m['diff_map'], cmap=cmap, vmin=0, vmax=1)
        ax.set_title(f"{m['name']}\nSSIM={m['ssim']:.4f}  MAE={m['mae']:.4f}",
                     fontsize=10)
        ax.axis('off')
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    # ซ่อน axes ที่เหลือ
    for j in range(len(all_metrics) + 1, len(axes)):
        axes[j].axis('off')

    fig.suptitle('Map Difference Heatmap\n(Green=Similar, Red=Different)',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"บันทึก Heatmap: {output_path}")
    plt.close()


# ============================================================
#  SECTION 5: Radar Chart สรุปทุก Metric (Figure 3)
# ============================================================

def plot_radar(all_metrics: list, output_path: str):
    """Radar chart แสดง metric ทุกตัวของแต่ละ trial ในกราฟเดียว"""

    categories = ['SSIM', 'Pixel Acc\n(%/100)', 'Coverage\n(%/100)',
                  '1-MAE', '1-RMSE', '1-OccDiff\n(%/100)']
    N = len(categories)
    angles = [n / float(N) * 2 * np.pi for n in range(N)]
    angles += angles[:1]

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))

    for i, m in enumerate(all_metrics):
        values = [
            m['ssim'],
            m['pixel_acc'] / 100,
            m['coverage_target'] / 100,
            1 - m['mae'],
            1 - m['rmse'],
            1 - min(m['occ_diff_pct'] / 100, 1),
        ]
        values += values[:1]
        ax.plot(angles, values, 'o-', linewidth=2,
                color=COLORS[i % len(COLORS)], label=m['name'])
        ax.fill(angles, values, alpha=0.1, color=COLORS[i % len(COLORS)])

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(categories, size=11)
    ax.set_ylim(0, 1)
    ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_yticklabels(['0.2', '0.4', '0.6', '0.8', '1.0'], size=9)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', bbox_to_anchor=(1.35, 1.15))
    ax.set_title('Map Quality Radar Chart\n(outer = better)',
                 fontweight='bold', pad=20)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ บันทึก Radar Chart: {output_path}")
    plt.close()


# ============================================================
#  SECTION 6: Export CSV สำหรับใส่ตาราง Thesis
# ============================================================

def export_csv(all_metrics: list, ref_name: str, output_path: str):
    """Export ตัวเลขทั้งหมดเป็น CSV"""
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Trial', 'MAE', 'RMSE', 'SSIM',
            'Pixel_Accuracy(%)', 'Coverage(%)', 'Occupied_Cell_Diff(%)'
        ])
        for m in all_metrics:
            writer.writerow([
                m['name'],
                f"{m['mae']:.6f}",
                f"{m['rmse']:.6f}",
                f"{m['ssim']:.6f}",
                f"{m['pixel_acc']:.2f}",
                f"{m['coverage_target']:.2f}",
                f"{m['occ_diff_pct']:.2f}",
            ])

        # สรุปสถิติ
        writer.writerow([])
        writer.writerow(['--- Statistics ---'])
        for metric_key, label in [
            ('mae', 'MAE'), ('rmse', 'RMSE'), ('ssim', 'SSIM'),
            ('pixel_acc', 'Pixel_Accuracy(%)'),
        ]:
            vals = [m[metric_key] for m in all_metrics]
            writer.writerow([
                f'{label}_mean',  f'{np.mean(vals):.6f}',
                f'{label}_std',   f'{np.std(vals):.6f}',
                f'{label}_min',   f'{np.min(vals):.6f}',
                f'{label}_max',   f'{np.max(vals):.6f}',
            ])

    print(f"บันทึก CSV: {output_path}")


# ============================================================
#  SECTION 7: Demo Mode (ถ้าไม่มีไฟล์ map จริง)
# ============================================================

def generate_demo_maps(n: int = 5) -> list:
    """สร้าง synthetic map สำหรับ demo/test"""
    print("⚠️  ไม่พบไฟล์ map จริง → รัน Demo mode")
    np.random.seed(42)
    base = np.ones((200, 400), dtype=np.float32)

    # สร้าง corridor
    base[10:190, 10:390] = 1.0   # free
    base[0:10,   :]      = 0.0   # ผนังบน
    base[190:,   :]      = 0.0   # ผนังล่าง
    base[:,  0:10]       = 0.0   # ผนังซ้าย
    base[:, 390:]        = 0.0   # ผนังขวา

    # เพิ่มห้อง
    for (r1, r2, c1, c2) in [(30, 80, 50, 120), (100, 160, 200, 280)]:
        base[r1:r2, c1:c2] = 1.0
        base[r1, c1:c2] = 0.0
        base[r2, c1:c2] = 0.0
        base[r1:r2, c1] = 0.0
        base[r1:r2, c2] = 0.0

    maps = [base.copy()]
    for i in range(1, n):
        noisy = base.copy()
        noise_level = 0.02 * i
        mask = np.random.rand(*base.shape) < noise_level
        noisy[mask] = 1.0 - noisy[mask]
        # drift เล็กน้อย
        shift = i
        noisy = np.roll(noisy, shift, axis=1)
        maps.append(noisy)

    return maps, [f'Trial_{i+1}' for i in range(n)]


# ============================================================
#  MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Map Consistency Analysis for Thesis')
    parser.add_argument('--maps',      nargs='+', help='ไฟล์ .pgm แต่ละ trial')
    parser.add_argument('--folder',    type=str,  help='โฟลเดอร์ที่มีไฟล์ .pgm')
    parser.add_argument('--reference', type=str,  help='ไฟล์ reference map (default: ไฟล์แรก)')
    parser.add_argument('--output',    type=str,  default='.', help='โฟลเดอร์ output')
    parser.add_argument('--demo',      action='store_true', help='รัน demo ด้วย synthetic maps')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # --- โหลด Maps ---
    if args.demo or (not args.maps and not args.folder):
        map_arrays, map_names = generate_demo_maps(5)
        ref_array = map_arrays[0]
        ref_name  = map_names[0]
        trial_arrays = map_arrays[1:]
        trial_names  = map_names[1:]
    else:
        # โหลดจากไฟล์จริง
        if args.folder:
            filepaths = sorted(glob.glob(os.path.join(args.folder, '*.pgm')))
        else:
            filepaths = args.maps

        if len(filepaths) < 2:
            print("❌ ต้องการไฟล์ map อย่างน้อย 2 ไฟล์")
            return

        ref_path  = args.reference if args.reference else filepaths[0]
        ref_array = load_map(ref_path)
        ref_name  = Path(ref_path).stem

        trial_paths  = [f for f in filepaths if f != ref_path]
        trial_arrays = [load_map(f) for f in trial_paths]
        trial_names  = [Path(f).stem for f in trial_paths]

        print(f"Reference: {ref_name}")
        print(f"Trials: {', '.join(trial_names)}")

    # --- คำนวณ Metrics ---
    print("\nกำลังคำนวณ metrics...")
    all_metrics = []
    for arr, name in zip(trial_arrays, trial_names):
        m = compute_metrics(ref_array, arr, name)
        all_metrics.append(m)
        print(f"  {name}: MAE={m['mae']:.4f}  RMSE={m['rmse']:.4f}  "
              f"SSIM={m['ssim']:.4f}  PixelAcc={m['pixel_acc']:.2f}%")

    # --- สร้างกราฟ ---
    print("\nกำลังสร้างกราฟ...")
    plot_main_results(all_metrics, ref_name,
                      os.path.join(args.output, 'map_comparison_results.png'))
    plot_heatmap(ref_array, all_metrics,
                 os.path.join(args.output, 'map_diff_heatmap.png'))
    plot_radar(all_metrics,
               os.path.join(args.output, 'map_radar_chart.png'))
    export_csv(all_metrics, ref_name,
               os.path.join(args.output, 'map_metrics.csv'))

    print("\nเสร็จสิ้น! ไฟล์ทั้งหมดบันทึกใน:", args.output)
    print("   - map_comparison_results.png  (กราฟหลัก)")
    print("   - map_diff_heatmap.png        (Heatmap)")
    print("   - map_radar_chart.png         (Radar Chart)")
    print("   - map_metrics.csv             (ตาราง)")


if __name__ == '__main__':
    main()