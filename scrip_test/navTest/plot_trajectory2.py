#!/usr/bin/env python3
import rosbag
import os
import math
import glob
import matplotlib.pyplot as plt

AMCL_WARMUP_SEC = 3.0  # ข้ามช่วงแรกที่ AMCL ยัง converge ไม่เสร็จ

# ฟังก์ชันคำนวณระยะห่างตั้งฉาก (Cross-Track Error)
def closest_distance_to_path(px, py, path_coords):
    min_dist = float('inf')
    if len(path_coords) == 0: return 0.0
    if len(path_coords) == 1: return math.hypot(px - path_coords[0][0], py - path_coords[0][1])

    for i in range(len(path_coords) - 1):
        x1, y1 = path_coords[i]
        x2, y2 = path_coords[i+1]
        dx, dy = x2 - x1, y2 - y1
        ab2 = dx*dx + dy*dy
        if ab2 == 0:
            dist = math.hypot(px - x1, py - y1)
        else:
            t = max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / ab2))
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            dist = math.hypot(px - proj_x, py - proj_y)
        if dist < min_dist: min_dist = dist
    return min_dist

def plot_latest_bag(bag_folder):
    list_of_files = glob.glob(os.path.join(bag_folder, '*.bag'))
    if not list_of_files:
        print("❌ ไม่พบไฟล์ .bag ในโฟลเดอร์")
        return
    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"กำลังวาดกราฟจากไฟล์ล่าสุด: {os.path.basename(latest_file)}")

    bag = rosbag.Bag(latest_file)

    # เก็บ global path ล่าสุด (อัปเดตทุกครั้งที่ TEB replan)
    global_path_coords = []
    first_global_path_x, first_global_path_y = [], []  # เก็บ plan แรกไว้วาดกราฟ reference

    actual_x, actual_y = [], []
    time_sec, tracking_errors = [], []

    bag_start_time = None
    start_time = None

    topics = [
        '/move_base/TebLocalPlannerROS/global_plan',
        '/amcl_pose'  # ✅ เปลี่ยนจาก /odom → /amcl_pose (map frame เดียวกับ global_plan)
    ]

    for topic, msg, t in bag.read_messages(topics=topics):
        t_sec = t.to_sec()

        if bag_start_time is None:
            bag_start_time = t_sec

        # ✅ อัปเดต global_path ทุกครั้งที่ TEB replan
        if topic == '/move_base/TebLocalPlannerROS/global_plan':
            if len(msg.poses) > 0:
                global_path_coords = [
                    (p.pose.position.x, p.pose.position.y)
                    for p in msg.poses
                ]
                # เก็บ plan แรกไว้วาดเส้น reference บนกราฟ
                if not first_global_path_x:
                    first_global_path_x = [p[0] for p in global_path_coords]
                    first_global_path_y = [p[1] for p in global_path_coords]

        # ✅ ข้าม warm-up และวัดจาก amcl_pose (map frame)
        elif topic == '/amcl_pose':
            if (t_sec - bag_start_time) < AMCL_WARMUP_SEC:
                continue

            if start_time is None:
                start_time = t_sec
            current_time = t_sec - start_time

            ax = msg.pose.pose.position.x
            ay = msg.pose.pose.position.y
            actual_x.append(ax)
            actual_y.append(ay)

            # ✅ append time_sec และ tracking_errors พร้อมกันเสมอ
            if len(global_path_coords) > 0:
                cte = closest_distance_to_path(ax, ay, global_path_coords)
                tracking_errors.append(cte)
                time_sec.append(current_time)

    bag.close()

    if not actual_x or not first_global_path_x:
        print("⚠️ ข้อมูลในไฟล์ไม่ครบถ้วน (อาจไม่มี Global Plan หรือ amcl_pose)")
        return

    # ================= วาดกราฟ =================
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"Navigation Performance Analysis\n{os.path.basename(latest_file)}",
                 fontsize=13, fontweight='bold')

    # กราฟ 1: X-Y Trajectory
    ax1.plot(first_global_path_x, first_global_path_y, 'g--', linewidth=2, label='Global Plan (Reference)')
    ax1.plot(actual_x, actual_y, 'b-', linewidth=1.5, label='Actual Path (AMCL)')
    ax1.scatter([first_global_path_x[0]], [first_global_path_y[0]],
                color='black', marker='o', s=100, zorder=5, label='Start')
    ax1.scatter([first_global_path_x[-1]], [first_global_path_y[-1]],
                color='red', marker='x', s=100, zorder=5, label='Goal')
    ax1.set_title('Robot Trajectory (X-Y Plane)', fontsize=12)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.axis('equal')
    ax1.legend()

    # กราฟ 2: Cross-Track Error vs Time
    if tracking_errors:
        rmse = math.sqrt(sum(e**2 for e in tracking_errors) / len(tracking_errors))
        mean_error = sum(tracking_errors) / len(tracking_errors)
        max_error  = max(tracking_errors)

        ax2.plot(time_sec, tracking_errors, 'r-', linewidth=1.5, label='Cross-Track Error')
        ax2.fill_between(time_sec, tracking_errors, color='red', alpha=0.1)
        ax2.axhline(y=mean_error, color='orange', linestyle='--',
                    label=f'Mean = {mean_error:.3f} m')
        ax2.axhline(y=rmse, color='purple', linestyle=':',
                    label=f'RMSE = {rmse:.3f} m')

        ax2.set_title('Path Tracking Error Over Time', fontsize=12)
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Cross-Track Error (m)')
        ax2.set_ylim(bottom=0)
        ax2.legend()

        print(f"  RMSE      : {rmse:.4f} m ({rmse*100:.2f} cm)")
        print(f"  Mean Error: {mean_error:.4f} m ({mean_error*100:.2f} cm)")
        print(f"  Max Error : {max_error:.4f} m ({max_error*100:.2f} cm)")

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    data_folder = os.path.expanduser('/home/leoss/ptR1Project/scrip_test/navTest/rosbagForward')
    plot_latest_bag(data_folder)