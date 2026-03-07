#!/usr/bin/env python3
import rosbag
import os
import math

AMCL_WARMUP_SEC = 3.0  # ข้ามช่วงแรกที่ AMCL ยัง converge ไม่เสร็จ

# ฟังก์ชันคณิตศาสตร์หาจุดที่ใกล้ที่สุดจากหุ่นยนต์ไปยังเส้นทาง (Point-to-Line-Segment Distance)
def closest_distance_to_path(px, py, path_coords):
    min_dist = float('inf')
    if len(path_coords) == 0:
        return 0.0
    if len(path_coords) == 1:
        return math.hypot(px - path_coords[0][0], py - path_coords[0][1])

    for i in range(len(path_coords) - 1):
        x1, y1 = path_coords[i]
        x2, y2 = path_coords[i+1]

        dx = x2 - x1
        dy = y2 - y1

        ab2 = dx*dx + dy*dy
        if ab2 == 0:
            dist = math.hypot(px - x1, py - y1)
        else:
            t = max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / ab2))
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            dist = math.hypot(px - proj_x, py - proj_y)

        if dist < min_dist:
            min_dist = dist

    return min_dist

def analyze_path_tracking(bag_folder):
    print(f"กำลังค้นหาไฟล์ข้อมูลใน: {bag_folder}")
    bag_files = [f for f in os.listdir(bag_folder) if f.endswith('.bag')]

    if not bag_files:
        print("❌ ไม่พบไฟล์ .bag ในโฟลเดอร์นี้")
        return

    print("-" * 75)
    print(f"{'ไฟล์ทดสอบ':<30} | {'RMSE (m)':<12} | {'Max Error (m)':<15} | {'Mean Error (m)'}")
    print("-" * 75)

    all_rmse = []
    all_max_error = []

    for file in sorted(bag_files):
        bag_path = os.path.join(bag_folder, file)
        bag = rosbag.Bag(bag_path)

        global_path = []
        tracking_errors = []
        bag_start_time = None

        topics = [
            '/move_base/TebLocalPlannerROS/global_plan',
            '/amcl_pose'   # ✅ เปลี่ยนจาก /odom → /amcl_pose (map frame เดียวกับ global_plan)
        ]

        for topic, msg, t in bag.read_messages(topics=topics):
            t_sec = t.to_sec()

            if bag_start_time is None:
                bag_start_time = t_sec

            # ✅ อัปเดต global_path ทุกครั้งที่ TEB replan (ไม่ใช่แค่ครั้งแรก)
            if topic == '/move_base/TebLocalPlannerROS/global_plan':
                if len(msg.poses) > 0:
                    global_path = [
                        (p.pose.position.x, p.pose.position.y)
                        for p in msg.poses
                    ]

            # ✅ ข้าม warm-up และวัด CTE จาก amcl_pose (map frame)
            elif topic == '/amcl_pose':
                if (t_sec - bag_start_time) < AMCL_WARMUP_SEC:
                    continue
                if len(global_path) == 0:
                    continue

                actual_x = msg.pose.pose.position.x
                actual_y = msg.pose.pose.position.y

                cte = closest_distance_to_path(actual_x, actual_y, global_path)
                tracking_errors.append(cte)

        bag.close()

        if len(tracking_errors) > 0:
            mean_error = sum(tracking_errors) / len(tracking_errors)
            max_error  = max(tracking_errors)
            rmse       = math.sqrt(sum(e**2 for e in tracking_errors) / len(tracking_errors))

            all_rmse.append(rmse)
            all_max_error.append(max_error)

            print(f"{file[:28]+'..':<30} | {rmse:<12.4f} | {max_error:<15.4f} | {mean_error:.4f}")
        else:
            print(f"{file[:28]+'..':<30} | {'ไม่มีข้อมูลเส้นทาง / amcl_pose':<50}")

    if len(all_rmse) > 0:
        overall_rmse = sum(all_rmse) / len(all_rmse)
        overall_max  = max(all_max_error)
        std_rmse     = math.sqrt(sum((r - overall_rmse)**2 for r in all_rmse) / len(all_rmse))

        print("-" * 75)
        print("สรุปผลการประเมินการเกาะเส้นทาง (Path Tracking Performance):")
        print(f"  จำนวนไฟล์ที่วิเคราะห์  : {len(all_rmse)} รอบ")
        print(f"  Overall RMSE          : {overall_rmse:.4f} m  ({overall_rmse*100:.2f} cm)")
        print(f"  Std Dev RMSE          : {std_rmse:.4f} m  ({std_rmse*100:.2f} cm)")
        print(f"  Absolute Max Error    : {overall_max:.4f} m  ({overall_max*100:.2f} cm)")
        print("  *(เกณฑ์ ป.โท Mecanum: RMSE ควรต่ำกว่า 0.10 m)*")
    print("-" * 75)

if __name__ == '__main__':
    #data_folder = os.path.expanduser('/home/leoss/ptR1Project/scrip_test/navTest/rosbagForward')
    data_folder = os.path.expanduser('/home/leoss/ptR1Project/scrip_test/navTest/rosbag_square')

    analyze_path_tracking(data_folder)