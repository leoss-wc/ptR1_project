#!/usr/bin/env python3
"""
calc_goal_error.py
==================
คำนวณ Goal Error จาก rosbag ทุกไฟล์ โดยใช้ /amcl_pose
(ไม่ต้องมี /move_base/goal ใน bag)

วิธีใช้:
  python3 calc_goal_error.py --bags_dir ./forward10m/ --planned 10.0
  python3 calc_goal_error.py --bags_dir ./forward70m/ --planned 70.0
  python3 calc_goal_error.py --bags_dir ./rectangle/  --planned 8.0

Output:
  - พิมพ์ตารางสรุปทุก trial
  - บันทึก goal_error_result.csv
"""

import os
import math
import argparse
import csv
import numpy as np

# ────────────────────────────────────────────────────────────
#  PARSE SINGLE BAG
# ────────────────────────────────────────────────────────────
def parse_amcl_pose(bag_path: str):
    """
    อ่าน /amcl_pose จาก bag แล้วคืน (start_x, start_y, end_x, end_y)
    """
    try:
        import rosbag
    except ImportError:
        raise ImportError("ต้องรันใน ROS Python environment ที่มี rosbag")

    positions = []
    with rosbag.Bag(bag_path, 'r') as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())

        # หา pose topic ที่มีใน bag
        pose_topic = None
        for t in ['/amcl_pose', '/robot_pose', '/base_pose_ground_truth']:
            if t in available:
                pose_topic = t
                break

        if pose_topic is None:
            print(f"  ⚠ ไม่พบ pose topic ใน {os.path.basename(bag_path)}")
            return None

        for _, msg, _ in bag.read_messages(topics=[pose_topic]):
            try:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
            except AttributeError:
                x = msg.pose.position.x
                y = msg.pose.position.y
            positions.append((x, y))

    if len(positions) < 2:
        return None

    return positions[0], positions[-1]


def calc_goal_error(start, end, planned_dist):
    """
    คำนวณ goal error โดยสมมติว่า goal อยู่ห่างจาก start
    ตามทิศทางที่หุ่นยนต์เดินจริง ระยะ planned_dist เมตร
    """
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    actual_dist = math.hypot(dx, dy)

    if actual_dist < 0.01:
        return None, None, None

    # unit vector ทิศทางที่เดิน
    ux = dx / actual_dist
    uy = dy / actual_dist

    # goal ที่ควรจะเป็น
    goal_x = start[0] + ux * planned_dist
    goal_y = start[1] + uy * planned_dist

    goal_error = math.hypot(end[0] - goal_x, end[1] - goal_y)
    overshoot  = actual_dist - planned_dist  # + = เลย, - = หยุดก่อน

    return round(actual_dist, 4), round(goal_error, 4), round(overshoot, 4)


# ────────────────────────────────────────────────────────────
#  MAIN
# ────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bags_dir', type=str, required=True,
                        help='โฟลเดอร์ที่เก็บ .bag ไฟล์')
    parser.add_argument('--planned',  type=float, default=10.0,
                        help='ระยะทางที่ตั้งใจเดิน (เมตร)')
    parser.add_argument('--exclude',  nargs='+', type=int, default=[],
                        help='Trial number ที่ต้องการตัดออก เช่น --exclude 1')
    parser.add_argument('--out',      type=str, default='goal_error_result.csv',
                        help='ชื่อไฟล์ CSV output')
    args = parser.parse_args()

    bag_paths = sorted([
        os.path.join(args.bags_dir, f)
        for f in os.listdir(args.bags_dir) if f.endswith('.bag')
    ])

    if not bag_paths:
        print("ไม่พบไฟล์ .bag ใน", args.bags_dir)
        return

    print(f"\n{'═'*65}")
    print(f"  Goal Error Calculator  |  Planned = {args.planned} m")
    print(f"  พบ {len(bag_paths)} bag files")
    print(f"{'═'*65}")
    print(f"  {'Trial':>5}  {'Bag File':<40}  {'Actual(m)':>9}  {'Error(m)':>8}  {'Overshoot':>9}")
    print(f"  {'-'*5}  {'-'*40}  {'-'*9}  {'-'*8}  {'-'*9}")

    results = []
    for i, path in enumerate(bag_paths):
        trial_num = i + 1
        fname = os.path.basename(path)
        excluded = trial_num in args.exclude

        if excluded:
            print(f"  {trial_num:>5}  {fname:<40}  {'[excluded]':>9}")
            continue

        pose_data = parse_amcl_pose(path)
        if pose_data is None:
            print(f"  {trial_num:>5}  {fname:<40}  {'ERROR':>9}")
            continue

        start, end = pose_data
        actual, error, overshoot = calc_goal_error(start, end, args.planned)

        if error is None:
            print(f"  {trial_num:>5}  {fname:<40}  {'NO MOVE':>9}")
            continue

        over_str = f"+{overshoot:.3f}" if overshoot >= 0 else f"{overshoot:.3f}"
        print(f"  {trial_num:>5}  {fname:<40}  {actual:>9.3f}  {error:>8.3f}  {over_str:>9}")

        results.append({
            'trial':        trial_num,
            'bag':          fname,
            'actual_dist':  actual,
            'goal_error':   error,
            'overshoot':    overshoot,
        })

    if not results:
        print("ไม่มีข้อมูลที่คำนวณได้")
        return

    # ── Summary ──────────────────────────────────────────────
    errors = [r['goal_error'] for r in results]
    actual = [r['actual_dist'] for r in results]
    over   = [r['overshoot']   for r in results]

    print(f"{'═'*65}")
    print(f"  {'Mean':>5}  {'':40}  {np.mean(actual):>9.3f}  {np.mean(errors):>8.3f}  {np.mean(over):>+9.3f}")
    print(f"  {'Std':>5}  {'':40}  {np.std(actual):>9.3f}  {np.std(errors):>8.3f}  {np.std(over):>9.3f}")
    print(f"  {'Max':>5}  {'':40}  {np.max(actual):>9.3f}  {np.max(errors):>8.3f}")
    print(f"  {'Min':>5}  {'':40}  {np.min(actual):>9.3f}  {np.min(errors):>8.3f}")
    print(f"{'═'*65}\n")

    # ── Save CSV ──────────────────────────────────────────────
    out_path = os.path.join(args.bags_dir, args.out)
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['trial','bag','actual_dist','goal_error','overshoot'])
        writer.writeheader()
        writer.writerows(results)
    print(f"  บันทึก CSV → {out_path}")
    print(f"\n  ✅ คัดลอกค่า goal_error เหล่านี้ใส่ใน nav_plot.py ได้เลย:")
    print(f"     goal_error = {[r['goal_error'] for r in results]}")


if __name__ == '__main__':
    main()