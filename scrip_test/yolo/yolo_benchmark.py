#!/usr/bin/env python3
"""
YOLO11 Nano Performance Benchmark (ONNX Runtime Edition)
ใช้ preprocessing เดียวกับ stream_manager_node2_pi.py ทุกอย่าง
- ONNX Runtime + CPUExecutionProvider
- Input 320x320
- intra/inter_op_num_threads = 2
- NMS เดียวกัน (score=0.35, iou=0.45)

ทดสอบ 3 โหมด:
  Mode 1: Person model เดียว
  Mode 2: Person + Door รันต่อกันทุก frame (Sequential)
  Mode 3: Person ทุก frame + Door ทุก N frame (Alternating)

วิธีใช้:
  pip install onnxruntime psutil opencv-python numpy matplotlib

  # ทดสอบ model เดียว (ไม่มี camera ใช้ dummy)
  python3 yolo_benchmark.py --model1 yolo11n.onnx

  # ทดสอบ 2 models + camera จริง
  python3 yolo_benchmark.py --model1 yolo11n.onnx --model2 door.onnx --source camera

  # ปรับ interval door detection
  python3 yolo_benchmark.py --model1 yolo11n.onnx --model2 door.onnx --interval 15

ผลลัพธ์:
  - yolo_benchmark_results.png
  - yolo_benchmark.csv
"""

import time
import argparse
import csv
import os
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import psutil
import cv2

try:
    import onnxruntime as ort
except ImportError:
    print("❌ ติดตั้ง onnxruntime ก่อน: pip install onnxruntime")
    exit(1)

# ===== Font สำหรับ Thesis =====
plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 12,
    'axes.titlesize': 13,
    'axes.labelsize': 11,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
})

COLORS       = ['#2196F3', '#F44336', '#4CAF50']
WARMUP       = 10    # frame อุ่น model
MEASURE      = 100   # frame วัดจริง
INPUT_SIZE   = 320   # เหมือน stream_manager
CONF_THRESH  = 0.35  # เหมือน stream_manager
NMS_THRESH   = 0.45  # เหมือน stream_manager
NUM_THREADS  = 2     # เหมือน stream_manager


# ============================================================
#  โหลด ONNX Model (เหมือน stream_manager ทุกอย่าง)
# ============================================================

def load_onnx_model(model_path: str) -> tuple:
    """โหลด ONNX model เหมือน handle_start_stream ใน stream_manager"""
    print(f"📦 โหลด: {model_path}")
    sess_options = ort.SessionOptions()
    sess_options.intra_op_num_threads = NUM_THREADS
    sess_options.inter_op_num_threads = NUM_THREADS

    session = ort.InferenceSession(
        model_path,
        sess_options=sess_options,
        providers=['CPUExecutionProvider']
    )
    input_name = session.get_inputs()[0].name
    num_classes = session.get_outputs()[0].shape[-1] - 4  # output shape ดึง class จำนวน
    print(f"   input: {input_name}  |  classes: {num_classes}")
    return session, input_name


# ============================================================
#  Inference (เหมือน ai_worker ใน stream_manager ทุกอย่าง)
# ============================================================

def run_inference(session, input_name: str, frame: np.ndarray) -> list:
    """
    Preprocessing + Inference + Postprocessing
    Copy มาจาก ai_worker ใน stream_manager_node2_pi.py
    """
    orig_h, orig_w = frame.shape[:2]

    # 1. Pre-processing
    img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.transpose((2, 0, 1))[np.newaxis] / 255.0).astype(np.float32)

    # 2. Inference
    outputs     = session.run(None, {input_name: img})
    predictions = np.squeeze(outputs[0]).T  # (N, 4+classes)

    if len(predictions) == 0:
        return []

    # 3. Post-processing
    classes_scores = predictions[:, 4:]
    class_ids      = np.argmax(classes_scores, axis=1)
    scores         = classes_scores[np.arange(len(predictions)), class_ids]

    mask        = scores > CONF_THRESH
    predictions = predictions[mask]
    scores      = scores[mask]
    class_ids   = class_ids[mask]

    if len(predictions) == 0:
        return []

    x_scale = orig_w / INPUT_SIZE
    y_scale = orig_h / INPUT_SIZE

    cx = predictions[:, 0]
    cy = predictions[:, 1]
    w  = predictions[:, 2]
    h  = predictions[:, 3]

    x1 = ((cx - w / 2) * x_scale).astype(int)
    y1 = ((cy - h / 2) * y_scale).astype(int)
    bw = (w * x_scale).astype(int)
    bh = (h * y_scale).astype(int)

    boxes_list     = np.stack([x1, y1, bw, bh], axis=1).tolist()
    scores_list    = scores.tolist()
    class_ids_list = class_ids.tolist()

    # 4. NMS
    indices = cv2.dnn.NMSBoxes(boxes_list, scores_list, CONF_THRESH, NMS_THRESH)

    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            x, y, bw_, bh_ = boxes_list[i]
            results.append([x, y, x + bw_, y + bh_,
                            scores_list[i], class_ids_list[i]])
    return results


# ============================================================
#  System Monitor (RAM + CPU)
# ============================================================

class SystemMonitor:
    def __init__(self):
        self.ram_samples = []
        self.cpu_samples = []
        self._running    = False
        self._thread     = None

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def _loop(self):
        while self._running:
            self.ram_samples.append(psutil.virtual_memory().used / 1024**2)
            self.cpu_samples.append(psutil.cpu_percent(interval=None))
            time.sleep(0.2)

    def summary(self) -> dict:
        return {
            'ram_mean': np.mean(self.ram_samples) if self.ram_samples else 0,
            'ram_max':  np.max(self.ram_samples)  if self.ram_samples else 0,
            'cpu_mean': np.mean(self.cpu_samples)  if self.cpu_samples else 0,
            'cpu_max':  np.max(self.cpu_samples)   if self.cpu_samples else 0,
        }


# ============================================================
#  Frame Source
# ============================================================

def frame_generator(source: str):
    """Generator คืน frame ไม่หยุด"""
    if source == 'dummy':
        print("⚠️  ใช้ dummy frames (640x480)")
        while True:
            yield np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    cap = cv2.VideoCapture(0 if source == 'camera' else source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    if not cap.isOpened():
        print("⚠️  เปิด camera ไม่ได้ → ใช้ dummy frames แทน")
        while True:
            yield np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
        if ret:
            yield frame


# ============================================================
#  Benchmark Functions
# ============================================================

def _warmup(sessions: list, gen):
    """อุ่น model ก่อนวัด"""
    for i, frame in enumerate(gen):
        for sess, name in sessions:
            run_inference(sess, name, frame)
        if i >= WARMUP:
            break


def benchmark(label: str, sessions: list, gen,
              door_interval: int = 1) -> dict:
    """
    วัด inference time
    sessions = [(session, input_name), ...]
    door_interval = 1  → รันทุก frame (sequential)
    door_interval = N  → รัน door ทุก N frame (alternating)
    """
    print(f"\n🔄 Warmup: {label}")
    _warmup(sessions, gen)

    print(f"📊 Measuring: {label} ({MEASURE} frames)")
    monitor = SystemMonitor()
    monitor.start()

    times = []
    for i, frame in enumerate(gen):
        t0 = time.perf_counter()

        # session[0] = person → ทุก frame เสมอ
        run_inference(sessions[0][0], sessions[0][1], frame)

        # session[1] = door → ทุก door_interval frame
        if len(sessions) > 1 and (i % door_interval == 0):
            run_inference(sessions[1][0], sessions[1][1], frame)

        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000)

        if i >= MEASURE:
            break

    monitor.stop()
    sys = monitor.summary()

    return {
        'label':      label,
        'inf_mean':   np.mean(times),
        'inf_std':    np.std(times),
        'inf_min':    np.min(times),
        'inf_max':    np.max(times),
        'fps':        1000 / np.mean(times),
        'ram_mean':   sys['ram_mean'],
        'ram_max':    sys['ram_max'],
        'cpu_mean':   sys['cpu_mean'],
        'cpu_max':    sys['cpu_max'],
        'times':      times,
    }


# ============================================================
#  กราฟ
# ============================================================

def plot(results: list, ram_gb: float, output_path: str):
    labels   = [r['label']    for r in results]
    fps      = [r['fps']      for r in results]
    inf_mean = [r['inf_mean'] for r in results]
    inf_std  = [r['inf_std']  for r in results]
    ram_max  = [r['ram_max']  for r in results]
    cpu_mean = [r['cpu_mean'] for r in results]
    ratio    = [r['inf_mean'] / results[0]['inf_mean'] for r in results]

    x   = np.arange(len(labels))
    fig = plt.figure(figsize=(16, 10))
    gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38)

    # ---- FPS ----
    ax = fig.add_subplot(gs[0, 0])
    bars = ax.bar(x, fps, color=COLORS[:len(results)], alpha=0.85)
    ax.axhline(5, color='gray', linestyle='--', alpha=0.6, label='Min usable = 5 FPS')
    ax.set_title('FPS Comparison', fontweight='bold')
    ax.set_ylabel('Frames per Second')
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.legend(fontsize=9); ax.grid(axis='y', alpha=0.3)
    for bar, v in zip(bars, fps):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                f'{v:.2f}', ha='center', va='bottom', fontsize=10, fontweight='bold')

    # ---- Inference Time ----
    ax = fig.add_subplot(gs[0, 1])
    bars = ax.bar(x, inf_mean, yerr=inf_std,
                  color=COLORS[:len(results)], alpha=0.85, capsize=6)
    ax.set_title('Inference Time\n(mean ± std)', fontweight='bold')
    ax.set_ylabel('Time (ms)')
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.grid(axis='y', alpha=0.3)
    for bar, v in zip(bars, inf_mean):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f'{v:.1f}ms', ha='center', va='bottom', fontsize=10, fontweight='bold')

    # ---- Ratio vs Single ----
    ax = fig.add_subplot(gs[0, 2])
    bars = ax.bar(x, ratio, color=COLORS[:len(results)], alpha=0.85)
    ax.axhline(1.0, color='gray',  linestyle='--', alpha=0.5, label='1x baseline')
    ax.axhline(2.0, color='red',   linestyle='--', alpha=0.4, label='2x')
    ax.set_title('Inference Time Ratio\nvs Single Model', fontweight='bold')
    ax.set_ylabel('Ratio (x times)')
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.legend(fontsize=9); ax.grid(axis='y', alpha=0.3)
    for bar, v in zip(bars, ratio):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                f'{v:.2f}x', ha='center', va='bottom', fontsize=10, fontweight='bold')

    # ---- RAM ----
    ax = fig.add_subplot(gs[1, 0])
    bars = ax.bar(x, ram_max, color=COLORS[:len(results)], alpha=0.85)
    ax.axhline(ram_gb * 1024 * 0.8, color='red', linestyle='--',
               alpha=0.5, label=f'80% of {ram_gb:.0f}GB')
    ax.set_title('Peak RAM Usage', fontweight='bold')
    ax.set_ylabel('RAM (MB)')
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.legend(fontsize=9); ax.grid(axis='y', alpha=0.3)
    for bar, v in zip(bars, ram_max):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 5,
                f'{v:.0f}MB', ha='center', va='bottom', fontsize=10, fontweight='bold')

    # ---- CPU ----
    ax = fig.add_subplot(gs[1, 1])
    bars = ax.bar(x, cpu_mean, color=COLORS[:len(results)], alpha=0.85)
    ax.axhline(80, color='red', linestyle='--', alpha=0.5, label='80% threshold')
    ax.set_title('Mean CPU Usage', fontweight='bold')
    ax.set_ylabel('CPU (%)')
    ax.set_ylim(0, 110)
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.legend(fontsize=9); ax.grid(axis='y', alpha=0.3)
    for bar, v in zip(bars, cpu_mean):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f'{v:.1f}%', ha='center', va='bottom', fontsize=10, fontweight='bold')

    # ---- Distribution ----
    ax = fig.add_subplot(gs[1, 2])
    for i, r in enumerate(results):
        ax.hist(r['times'], bins=20, alpha=0.6,
                color=COLORS[i], label=r['label'])
    ax.set_title('Inference Time Distribution\n(wide = unstable)', fontweight='bold')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Frequency')
    ax.legend(fontsize=9); ax.grid(alpha=0.3)

    total_ram = psutil.virtual_memory().total / 1024**2
    fig.suptitle(
        f'YOLO11 Nano Benchmark — ONNX Runtime (320×320)\n'
        f'Raspberry Pi 4 {ram_gb:.0f}GB  |  Threads={NUM_THREADS}  |  '
        f'Conf={CONF_THRESH}  NMS={NMS_THRESH}',
        fontsize=13, fontweight='bold'
    )

    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ บันทึกกราฟ: {output_path}")
    plt.close()


# ============================================================
#  Export CSV
# ============================================================

def export_csv(results: list, output_path: str):
    base = results[0]['inf_mean']
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Mode', 'FPS',
            'Inference_Mean(ms)', 'Inference_Std(ms)',
            'Inference_Min(ms)',  'Inference_Max(ms)',
            'RAM_Max(MB)', 'CPU_Mean(%)', 'Ratio_vs_Single'
        ])
        for r in results:
            writer.writerow([
                r['label'],
                f"{r['fps']:.2f}",
                f"{r['inf_mean']:.2f}",
                f"{r['inf_std']:.2f}",
                f"{r['inf_min']:.2f}",
                f"{r['inf_max']:.2f}",
                f"{r['ram_max']:.0f}",
                f"{r['cpu_mean']:.1f}",
                f"{r['inf_mean']/base:.2f}x",
            ])

        writer.writerow([])
        writer.writerow(['--- Config ---'])
        writer.writerow(['Input Size',    f'{INPUT_SIZE}x{INPUT_SIZE}'])
        writer.writerow(['Conf Threshold', CONF_THRESH])
        writer.writerow(['NMS Threshold',  NMS_THRESH])
        writer.writerow(['ONNX Threads',   NUM_THREADS])
        writer.writerow(['Warmup Frames',  WARMUP])
        writer.writerow(['Measure Frames', MEASURE])

    print(f"✅ บันทึก CSV: {output_path}")


# ============================================================
#  Print Summary
# ============================================================

def print_summary(results: list):
    base = results[0]['inf_mean']
    print("\n" + "="*72)
    print("  YOLO11 Nano ONNX Benchmark — Raspberry Pi 4")
    print(f"  Input: {INPUT_SIZE}x{INPUT_SIZE}  Threads: {NUM_THREADS}  "
          f"Conf: {CONF_THRESH}  NMS: {NMS_THRESH}")
    print("="*72)
    print(f"  {'Mode':<32} {'FPS':>6} {'ms':>8} {'±':>6} "
          f"{'RAM':>8} {'CPU':>7} {'Ratio':>7}")
    print("-"*72)
    for r in results:
        print(f"  {r['label']:<32} "
              f"{r['fps']:>6.2f} "
              f"{r['inf_mean']:>7.1f}ms "
              f"{r['inf_std']:>5.1f} "
              f"{r['ram_max']:>6.0f}MB "
              f"{r['cpu_mean']:>6.1f}% "
              f"{r['inf_mean']/base:>6.2f}x")
    print("="*72)


# ============================================================
#  MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='YOLO11 Nano ONNX Benchmark for RPi4')
    parser.add_argument('--model1',   default='yolo11n.onnx',
                        help='Person model (.onnx)')
    parser.add_argument('--model2',   default=None,
                        help='Door model (.onnx) ถ้าไม่ใส่ทดสอบแค่ model เดียว')
    parser.add_argument('--source',   default='dummy',
                        help='camera / video.mp4 / dummy (default: dummy)')
    parser.add_argument('--interval', type=int, default=10,
                        help='Door detect ทุกกี่ frame (default: 10)')
    parser.add_argument('--output',   default='.',
                        help='โฟลเดอร์ output (default: .)')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # RAM ของระบบ
    ram_gb = psutil.virtual_memory().total / 1024**3

    # โหลด Models
    sess1, name1 = load_onnx_model(args.model1)
    sess2, name2 = (load_onnx_model(args.model2)
                    if args.model2 else (None, None))

    results = []

    # ---- Mode 1: Single model ----
    gen = frame_generator(args.source)
    r1  = benchmark('Single Model (Person)',
                    [(sess1, name1)], gen)
    results.append(r1)

    # ---- Mode 2 & 3: 2 Models ----
    if sess2:
        gen = frame_generator(args.source)
        r2  = benchmark('2 Models Sequential',
                        [(sess1, name1), (sess2, name2)],
                        gen, door_interval=1)
        results.append(r2)

        gen = frame_generator(args.source)
        r3  = benchmark(f'2 Models Alt (door/{args.interval}f)',
                        [(sess1, name1), (sess2, name2)],
                        gen, door_interval=args.interval)
        results.append(r3)

    # Output
    print_summary(results)
    plot(results, ram_gb,
         os.path.join(args.output, 'yolo_benchmark_results.png'))
    export_csv(results,
               os.path.join(args.output, 'yolo_benchmark.csv'))

    print(f"\n✅ บันทึกใน: {args.output}")
    print("   - yolo_benchmark_results.png")
    print("   - yolo_benchmark.csv")


if __name__ == '__main__':
    main()
