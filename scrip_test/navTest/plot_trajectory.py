#!/usr/bin/env python3
import rosbag
import os
import math
import glob
import matplotlib.pyplot as plt

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
    # หาไฟล์ .bag ล่าสุดในโฟลเดอร์
    list_of_files = glob.glob(os.path.join(bag_folder, '*.bag'))
    if not list_of_files:
        print("❌ ไม่พบไฟล์ .bag ในโฟลเดอร์")
        return
    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"กำลังวาดกราฟจากไฟล์ล่าสุด: {os.path.basename(latest_file)}")

    bag = rosbag.Bag(latest_file)
    
    global_path_x, global_path_y = [], []
    actual_x, actual_y = [], []
    time_sec, tracking_errors = [], []
    
    start_time = None
    global_path_coords = []

    # อ่านข้อมูล
    for topic, msg, t in bag.read_messages(topics=['/move_base/TebLocalPlannerROS/global_plan', '/odom']):
        if topic == '/move_base/TebLocalPlannerROS/global_plan':
            if len(global_path_x) == 0 and len(msg.poses) > 0:
                for p in msg.poses:
                    gx, gy = p.pose.position.x, p.pose.position.y
                    global_path_x.append(gx)
                    global_path_y.append(gy)
                    global_path_coords.append((gx, gy))
                    
        elif topic == '/odom':
            if start_time is None: start_time = t.to_sec()
            current_time = t.to_sec() - start_time
            
            ax, ay = msg.pose.pose.position.x, msg.pose.pose.position.y
            actual_x.append(ax)
            actual_y.append(ay)
            time_sec.append(current_time)
            
            if len(global_path_coords) > 0:
                cte = closest_distance_to_path(ax, ay, global_path_coords)
                tracking_errors.append(cte)

    bag.close()

    if not actual_x or not global_path_x:
        print("⚠️ ข้อมูลในไฟล์ไม่ครบถ้วน (อาจไม่มี Global Plan หรือ Odom)")
        return

    # ================= วาดกราฟ =================
    plt.style.use('seaborn-v0_8-whitegrid') # ใช้สไตล์กราฟที่ดูเป็นวิชาการ
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"Navigation Performance Analysis: {os.path.basename(latest_file)}", fontsize=14, fontweight='bold')

    # กราฟ 1: X-Y Trajectory (เส้นทางในมุมมอง Top-down)
    ax1.plot(global_path_x, global_path_y, 'g--', linewidth=2, label='Global Plan (Reference)')
    ax1.plot(actual_x, actual_y, 'b-', linewidth=2, label='Actual Path (Odom)')
    ax1.scatter([global_path_x[0]], [global_path_y[0]], color='black', marker='o', s=100, label='Start')
    ax1.scatter([global_path_x[-1]], [global_path_y[-1]], color='red', marker='x', s=100, label='Goal')
    ax1.set_title('Robot Trajectory (X-Y Plane)', fontsize=12)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.axis('equal') # บังคับสเกลแกน X และ Y ให้เท่ากัน (เส้นสี่เหลี่ยมจะได้ไม่เบี้ยว)
    ax1.legend()

    # กราฟ 2: Cross-Track Error vs Time (ความคลาดเคลื่อนเทียบเวลา)
    if tracking_errors:
        ax2.plot(time_sec, tracking_errors, 'r-', linewidth=2, label='Cross-Track Error')
        ax2.fill_between(time_sec, tracking_errors, color='red', alpha=0.1)
        mean_error = sum(tracking_errors)/len(tracking_errors)
        ax2.axhline(y=mean_error, color='orange', linestyle='--', label=f'Mean Error ({mean_error:.3f} m)')
        
        ax2.set_title('Path Tracking Error Over Time', fontsize=12)
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Error Distance (m)')
        ax2.set_ylim(bottom=0)
        ax2.legend()

    plt.tight_layout()
    
    # บันทึกรูปภาพอัตโนมัติ (เป็นไฟล์ PNG ความละเอียดสูง)
    save_path = latest_file.replace('.bag', '_plot.png')
    plt.savefig(save_path, dpi=300)
    print(f"บันทึกรูปกราฟเรียบร้อยแล้วที่: {save_path}")
    
    # แสดงหน้าต่างกราฟ
    plt.show()

if __name__ == '__main__':
    data_folder = os.path.expanduser('~/ptr1_test_data')
    plot_latest_bag(data_folder)