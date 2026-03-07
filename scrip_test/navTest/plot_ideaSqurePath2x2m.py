#!/usr/bin/env python3
import rosbag
import os
import glob
import math
import matplotlib.pyplot as plt

def closest_distance_to_path(px, py, path_coords):
    min_dist = float('inf')
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

def plot_square_test(bag_folder):
    # หาไฟล์ท่าสี่เหลี่ยมล่าสุด
    list_of_files = glob.glob(os.path.join(bag_folder, '*Square_Path*.bag'))
    if not list_of_files:
        print("❌ ไม่พบไฟล์ทดสอบท่า Square_Path")
        return
    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"กำลังวาดกราฟจากไฟล์: {os.path.basename(latest_file)}")

    bag = rosbag.Bag(latest_file)
    
    actual_x, actual_y = [], []
    time_sec, tracking_errors = [], []
    start_time = None
    
    # 1. สร้างเส้นทางสี่เหลี่ยมอุดมคติ (Theoretical Path) ขนาด 1x1 เมตร
    # เริ่มจาก (0,0) -> เดินหน้า (1,0) -> เลี้ยวซ้ายเดินหน้า (1,1) -> เลี้ยวซ้ายเดินหน้า (0,1) -> กลับมา (0,0)
    ideal_square_x = [0.0, 2.0, 2.0, 0.0, 0.0]
    ideal_square_y = [0.0, 0.0, 2.0, 2.0, 0.0]
    ideal_coords = list(zip(ideal_square_x, ideal_square_y))

    start_pose = None

    # 2. อ่านข้อมูล Odom ที่เดินจริง
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        if start_pose is None:
            start_pose = msg.pose.pose.position
            start_time = t.to_sec()
            
        current_time = t.to_sec() - start_time
        
        # Shift พิกัดเริ่มต้นให้เริ่มที่ (0,0) เสมอ เพื่อเทียบกับกราฟอุดมคติได้ง่าย
        ax = msg.pose.pose.position.x - start_pose.x
        ay = msg.pose.pose.position.y - start_pose.y
        
        actual_x.append(ax)
        actual_y.append(ay)
        time_sec.append(current_time)
        
        # คำนวณ Error เทียบกับเส้นสี่เหลี่ยมอุดมคติ
        cte = closest_distance_to_path(ax, ay, ideal_coords)
        tracking_errors.append(cte)

    bag.close()

    # ================= วาดกราฟ =================
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Square Path Performance (2x2 m)", fontsize=14, fontweight='bold')

    # กราฟ 1: X-Y Trajectory
    ax1.plot(ideal_square_x, ideal_square_y, 'g--', linewidth=2, label='Ideal Square Path')
    ax1.plot(actual_x, actual_y, 'b-', linewidth=2, label='Actual Robot Path (Odom)')
    ax1.scatter(0, 0, color='black', marker='o', s=100, label='Start / End Goal')
    
    ax1.set_title('Robot Trajectory Tracking', fontsize=12)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.axis('equal') # ล็อกสเกลให้เป็นสี่เหลี่ยมจัตุรัส ไม่เบี้ยวเป็นผืนผ้า
    ax1.set_xlim(-0.3, 2.3)
    ax1.set_ylim(-0.3, 2.3)
    ax1.legend()

    # กราฟ 2: Error
    ax2.plot(time_sec, tracking_errors, 'r-', linewidth=2, label='Cross-Track Error')
    mean_error = sum(tracking_errors)/len(tracking_errors)
    ax2.axhline(y=mean_error, color='orange', linestyle='--', label=f'Mean Error ({mean_error:.3f} m)')
    
    ax2.set_title('Deviation from Ideal Path', fontsize=12)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Error Distance (m)')
    ax2.set_ylim(bottom=0)
    ax2.legend()

    plt.tight_layout()
    save_path = latest_file.replace('.bag', '_square_plot.png')
    plt.savefig(save_path, dpi=300)
    print(f"✅ บันทึกรูปกราฟเรียบร้อยแล้วที่: {save_path}")
    plt.show()

if __name__ == '__main__':
    data_folder = os.path.expanduser('~/ptr1_test_data')
    plot_square_test(data_folder)