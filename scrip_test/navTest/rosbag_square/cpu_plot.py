#!/usr/bin/env python3
"""
plot_cpu_from_bag.py
อ่านข้อมูล CPU จาก rosbag topic /pi/system_profile แล้ววาดกราฟ
"""

import sys
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

try:
    import rosbag
except ImportError:
    sys.exit(1)

TOPIC = '/pi/system_profile'

# สีของแต่ละ service (stacked area)
SERVICE_COLORS = {
    'gmapping':   '#E74C3C',
    'move_base':  '#3498DB',
    'amcl':       '#2ECC71',
    'map_server': '#F39C12',
    'ydlidar':    '#9B59B6',
    'rosbridge':  '#1ABC9C',
    'tf2_web':    '#E67E22',
    'ffmpeg':     '#E91E63',
    'mediamtx':   '#00BCD4',
    'rosserial':  '#8BC34A',
    'tailscale':  '#FF5722',
    'stream_mgr': '#607D8B',
    'ros_nodes':  '#795548',
    'others':     '#BDBDBD',
}

def load_bag(bag_path):
    times, system, services = [], [], []

    print(f"กำลังอ่านไฟล์: {bag_path}")
    with rosbag.Bag(bag_path, 'r') as bag:
        t0 = None
        for _, msg, t in bag.read_messages(topics=[TOPIC]):
            try:
                data = json.loads(msg.data)
                if t0 is None:
                    t0 = t.to_sec()
                times.append(t.to_sec() - t0)
                system.append(data['system'])
                services.append(data['cpu_services'])
            except Exception as e:
                print(f"  skip: {e}")

    print(f"  โหลดได้ {len(times)} samples  ({times[-1]:.1f} วิ)")
    return np.array(times), system, services


def plot_cpu(times, system, services, save_path=None):
    cpu_total = np.array([s['cpu_total']    for s in system])
    ram       = np.array([s['ram_percent']  for s in system])
    temp      = np.array([s['temperature']  for s in system])

    # รวม service keys ที่มีจริง
    svc_keys = list(SERVICE_COLORS.keys())
    svc_keys = [k for k in svc_keys if any(k in s for s in services)]

    svc_matrix = {k: np.array([s.get(k, 0.0) for s in services]) for k in svc_keys}

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle('Raspberry Pi System Monitor\n(from /pi/system_profile)', fontsize=12)

    # --- Plot 1: CPU Total ---
    ax1 = axes[0]
    ax1.plot(times, cpu_total, color='#E74C3C', linewidth=1.2, label='CPU Total (%)')
    ax1.axhline(80, color='red', linestyle=':', linewidth=0.8, alpha=0.6, label='80% threshold')
    ax1.set_ylabel('CPU (%)')
    ax1.set_ylim(0, 105)
    ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(axis='y', linestyle='--', alpha=0.4)
    ax1.set_title('CPU Usage Total', fontsize=10)

    # --- Plot 2: CPU per Service (stacked area) ---
    ax2 = axes[1]
    stack_vals = [svc_matrix[k] for k in svc_keys]
    stack_cols = [SERVICE_COLORS[k] for k in svc_keys]
    ax2.stackplot(times, stack_vals, labels=svc_keys, colors=stack_cols, alpha=0.8)
    ax2.set_ylabel('CPU (%)')
    ax2.set_title('CPU Usage per Service (Stacked)', fontsize=10)
    ax2.grid(axis='y', linestyle='--', alpha=0.3)
    ax2.legend(fontsize=7, loc='upper right', ncol=2,
               bbox_to_anchor=(1.0, 1.0), framealpha=0.7)

    # --- Plot 3: RAM & Temperature ---
    ax3 = axes[2]
    color_ram  = '#3498DB'
    color_temp = '#E67E22'
    ax3.plot(times, ram,  color=color_ram,  linewidth=1.2, label='RAM (%)')
    ax3.set_ylabel('RAM (%)', color=color_ram)
    ax3.tick_params(axis='y', labelcolor=color_ram)
    ax3.set_ylim(0, 105)

    ax3b = ax3.twinx()
    ax3b.plot(times, temp, color=color_temp, linewidth=1.2, linestyle='--', label='Temp (°C)')
    ax3b.set_ylabel('Temperature (°C)', color=color_temp)
    ax3b.tick_params(axis='y', labelcolor=color_temp)

    lines1, lab1 = ax3.get_legend_handles_labels()
    lines2, lab2 = ax3b.get_legend_handles_labels()
    ax3.legend(lines1 + lines2, lab1 + lab2, fontsize=8, loc='upper right')
    ax3.grid(axis='y', linestyle='--', alpha=0.3)
    ax3.set_title('RAM & Temperature', fontsize=10)

    ax3.set_xlabel('Time (s)')
    ax3.xaxis.set_major_locator(ticker.AutoLocator())

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"บันทึกกราฟเป็น: {save_path}")
    else:
        plt.savefig('cpu_monitor.png', dpi=150, bbox_inches='tight')
        print("บันทึกกราฟเป็น: cpu_monitor.png")

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Plot CPU from rosbag')
    parser.add_argument('bag', help='path to .bag file')
    parser.add_argument('-o', '--output', default=None, help='output image path (optional)')
    args = parser.parse_args()

    times, system, services = load_bag(args.bag)

    if len(times) == 0:
        print(f"ERROR: ไม่พบข้อมูลใน topic {TOPIC}")
        print("  ลองเช็ค: rosbag info your_file.bag | grep pi")
        sys.exit(1)

    plot_cpu(times, system, services, save_path=args.output)


if __name__ == '__main__':
    main()