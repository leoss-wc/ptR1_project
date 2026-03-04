#!/usr/bin/env python3
import rosbag
import os
import math

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
        
        # Project จุดปัจจุบันลงบนเส้นทาง
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

        for topic, msg, t in bag.read_messages(topics=['/move_base/TebLocalPlannerROS/global_plan', '/odom']):
            # 1. เก็บเส้นทางอุดมคติ (Global Plan) เส้นแรกที่ถูกสร้างขึ้น
            if topic == '/move_base/TebLocalPlannerROS/global_plan':
                if len(global_path) == 0 and len(msg.poses) > 0:
                    for pose_stamped in msg.poses:
                        global_path.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
            
            # 2. เก็บตำแหน่งจริงของหุ่นยนต์ และเทียบกับเส้นทาง
            elif topic == '/odom':
                if len(global_path) > 0:
                    actual_x = msg.pose.pose.position.x
                    actual_y = msg.pose.pose.position.y
                    
                    # คำนวณระยะหลุดเส้นทาง (Cross-Track Error)
                    cte = closest_distance_to_path(actual_x, actual_y, global_path)
                    tracking_errors.append(cte)
                    
        bag.close()

        if len(tracking_errors) > 0:
            # คำนวณสถิติของไฟล์นั้นๆ
            mean_error = sum(tracking_errors) / len(tracking_errors)
            max_error = max(tracking_errors)
            
            # คำนวณ Root Mean Square Error (RMSE)
            sq_errors = [e**2 for e in tracking_errors]
            rmse = math.sqrt(sum(sq_errors) / len(sq_errors))
            
            all_rmse.append(rmse)
            all_max_error.append(max_error)

            print(f"{file[:28]+'..':<30} | {rmse:<12.4f} | {max_error:<15.4f} | {mean_error:.4f}")
        else:
            print(f"{file[:28]+'..':<30} | {'ไม่มีข้อมูลเส้นทาง':<40}")

    # --- สรุปผลรวมสำหรับนำไปเขียนวิทยานิพนธ์ ---
    if len(all_rmse) > 0:
        overall_rmse = sum(all_rmse) / len(all_rmse)
        overall_max = max(all_max_error)
        
        print("-" * 75)
        print("📊 สรุปผลการประเมินการเกาะเส้นทาง (Path Tracking Performance):")
        print(f"จำนวนไฟล์ที่วิเคราะห์: {len(all_rmse)} รอบ")
        print(f"ค่าความคลาดเคลื่อนเฉลี่ยสะสม (Overall RMSE): {overall_rmse:.4f} เมตร ({overall_rmse*100:.2f} ซม.)")
        print(f"ค่าหลุดเส้นทางสูงสุดที่พบ (Absolute Max Error): {overall_max:.4f} เมตร ({overall_max*100:.2f} ซม.)")
        print("*(คำแนะนำสำหรับเล่ม ป.โท: ค่า RMSE ควรต่ำกว่า 0.10 m หรือ 10 cm สำหรับหุ่นยนต์ล้อ Mecanum)*")
    print("-" * 75)

if __name__ == '__main__':
    data_folder = os.path.expanduser('~/ptr1_test_data')
    analyze_path_tracking(data_folder)