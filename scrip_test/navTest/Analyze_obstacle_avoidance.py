#!/usr/bin/env python3
"""
analyze_obstacle_avoidance.py
วิเคราะห์ประสิทธิภาพการหลบสิ่งกีดขวางจาก rosbag
Metrics: Success Rate, Replanning Count, Min Obstacle Distance,
         Extra Path Length, Navigation Time
"""

import rosbag
import os
import math

# ======================== CONFIG ========================
AMCL_WARMUP_SEC   = 3.0    # ข้ามช่วง AMCL warmup
MIN_SCAN_RANGE    = 0.05   # ตัดค่า scan ที่ใกล้เกินจริง (m)
MAX_SCAN_RANGE    = 10.0   # ตัดค่า scan ที่ไกลเกินไป (m)
GOAL_REACHED_STATE = 3     # move_base SUCCEEDED

# ========================================================

def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def path_length(coords):
    """คำนวณความยาวทั้งหมดของเส้นทาง"""
    total = 0.0
    for i in range(len(coords) - 1):
        total += euclidean(coords[i], coords[i+1])
    return total

def straight_line_distance(coords):
    """ระยะเส้นตรงจากจุดเริ่มต้นถึงจุดสิ้นสุด"""
    if len(coords) < 2:
        return 0.0
    return euclidean(coords[0], coords[-1])

def analyze_obstacle_avoidance(bag_folder):
    print(f"กำลังค้นหาไฟล์ข้อมูลใน: {bag_folder}")
    bag_files = [f for f in os.listdir(bag_folder) if f.endswith('.bag')]

    if not bag_files:
        print("❌ ไม่พบไฟล์ .bag ในโฟลเดอร์นี้")
        return

    print("=" * 95)
    print(f"{'ไฟล์ทดสอบ':<32} | {'สำเร็จ':<6} | {'Replan':<7} | {'Min Dist(m)':<12} | {'Extra Path%':<12} | {'เวลา(s)'}")
    print("=" * 95)

    summary = {
        'success': 0,
        'total': 0,
        'replan_counts': [],
        'min_distances': [],
        'extra_path_pcts': [],
        'nav_times': [],
    }

    topics = [
        '/amcl_pose',
        '/scan',
        '/move_base/TebLocalPlannerROS/global_plan',
        '/move_base/result',
    ]

    for file in sorted(bag_files):
        bag_path = os.path.join(bag_folder, file)
        bag = rosbag.Bag(bag_path)

        bag_start_time  = None
        nav_start_time  = None
        nav_end_time    = None

        actual_poses    = []   # (x, y) จาก amcl_pose
        scan_min_dist   = float('inf')
        replan_count    = 0
        prev_path_stamp = None
        goal_succeeded  = None  # True/False/None

        for topic, msg, t in bag.read_messages(topics=topics):
            t_sec = t.to_sec()

            if bag_start_time is None:
                bag_start_time = t_sec
                nav_start_time = t_sec

            # ---- amcl_pose: track เส้นทางที่หุ่นเดินจริง ----
            if topic == '/amcl_pose':
                if (t_sec - bag_start_time) < AMCL_WARMUP_SEC:
                    continue
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                actual_poses.append((x, y))

            # ---- scan: หาระยะห่าง obstacle ต่ำสุด ----
            elif topic == '/scan':
                if (t_sec - bag_start_time) < AMCL_WARMUP_SEC:
                    continue
                valid_ranges = [
                    r for r in msg.ranges
                    if MIN_SCAN_RANGE < r < MAX_SCAN_RANGE and not math.isnan(r) and not math.isinf(r)
                ]
                if valid_ranges:
                    frame_min = min(valid_ranges)
                    if frame_min < scan_min_dist:
                        scan_min_dist = frame_min

            # ---- global_plan: นับจำนวน replan ----
            elif topic == '/move_base/TebLocalPlannerROS/global_plan':
                if len(msg.poses) == 0:
                    continue
                stamp = msg.header.stamp.to_sec()
                # นับว่า replan ถ้า stamp ห่างจาก plan ก่อนหน้า > 0.5 วินาที
                if prev_path_stamp is None:
                    prev_path_stamp = stamp
                elif (stamp - prev_path_stamp) > 0.5:
                    replan_count += 1
                    prev_path_stamp = stamp

            # ---- move_base result: ดูว่าสำเร็จไหม ----
            elif topic == '/move_base/result':
                if msg.status.status == GOAL_REACHED_STATE:
                    goal_succeeded = True
                    nav_end_time = t_sec
                else:
                    goal_succeeded = False
                    nav_end_time = t_sec

        bag.close()

        # ---- คำนวณ Metrics ----
        success_str = "✅" if goal_succeeded else ("❌" if goal_succeeded is False else "❓")

        # Extra Path Length (%)
        actual_len    = path_length(actual_poses)
        straight_len  = straight_line_distance(actual_poses)
        if straight_len > 0.01:
            extra_pct = ((actual_len - straight_len) / straight_len) * 100.0
        else:
            extra_pct = 0.0

        # Navigation Time
        if nav_start_time and nav_end_time:
            nav_time = nav_end_time - nav_start_time
        else:
            nav_time = 0.0

        min_dist_str = f"{scan_min_dist:.3f}" if scan_min_dist < float('inf') else "N/A"

        short_name = file[:30] + '..' if len(file) > 30 else file
        print(f"{short_name:<32} | {success_str:<6} | {replan_count:<7} | {min_dist_str:<12} | {extra_pct:<11.1f}% | {nav_time:.2f}")

        # เก็บ summary
        summary['total'] += 1
        if goal_succeeded:
            summary['success'] += 1
        summary['replan_counts'].append(replan_count)
        if scan_min_dist < float('inf'):
            summary['min_distances'].append(scan_min_dist)
        summary['extra_path_pcts'].append(extra_pct)
        if nav_time > 0:
            summary['nav_times'].append(nav_time)

    # ---- สรุปผลรวม ----
    print("=" * 95)
    print("สรุปผลการประเมินการหลบสิ่งกีดขวาง (Obstacle Avoidance Performance):")

    n = summary['total']
    if n == 0:
        print("  ไม่มีข้อมูลเพียงพอ")
        return

    success_rate = (summary['success'] / n) * 100.0
    avg_replan   = sum(summary['replan_counts']) / n
    avg_min_dist = sum(summary['min_distances']) / len(summary['min_distances']) if summary['min_distances'] else 0
    overall_min_dist = min(summary['min_distances']) if summary['min_distances'] else 0
    avg_extra    = sum(summary['extra_path_pcts']) / n
    avg_time     = sum(summary['nav_times']) / len(summary['nav_times']) if summary['nav_times'] else 0

    print(f"  จำนวนไฟล์ที่วิเคราะห์    : {n} รอบ")
    print(f"  Success Rate           : {summary['success']}/{n} รอบ ({success_rate:.1f}%)")
    print(f"  Avg Replan Count       : {avg_replan:.1f} ครั้ง/รอบ")
    print(f"  Avg Min Obstacle Dist  : {avg_min_dist:.3f} m")
    print(f"  Overall Min Obstacle   : {overall_min_dist:.3f} m  ← ระยะใกล้สุดที่เคยเจอ")
    print(f"  Avg Extra Path Length  : {avg_extra:.1f}%  (เส้นทางจริง vs เส้นตรง)")
    print(f"  Avg Navigation Time    : {avg_time:.2f} s")
    print()
    print("  เกณฑ์อ้างอิงที่แนะนำ:")
    print("    Success Rate       ≥ 90%")
    print("    Min Obstacle Dist  ≥ 0.20 m  (ไม่ชนสิ่งกีดขวาง)")
    print("    Extra Path Length  ≤ 30%     (ไม่อ้อมมากเกินไป)")
    print("=" * 95)


if __name__ == '__main__':
    data_folder = os.path.expanduser('~/ptR1Project/scrip_test/navTest/rosbagObstacle')
    analyze_obstacle_avoidance(data_folder)