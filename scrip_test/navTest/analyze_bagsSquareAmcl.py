#!/usr/bin/env python3
import rosbag
import os
import math

def analyze_square_path(bag_folder):
    print(f"กำลังค้นหาไฟล์ข้อมูลใน: {bag_folder}")
    bag_files = [f for f in os.listdir(bag_folder) if f.endswith('.bag') and 'Square_Path' in f]
    
    if not bag_files:
        print("ไม่พบไฟล์ทดสอบ Square_Path ในโฟลเดอร์นี้")
        return

    total_distance_error = 0.0
    total_angular_error = 0.0
    total_time = 0.0
    valid_tests = 0

    print("-" * 50)
    print(f"{'ไฟล์ทดสอบ':<30} | {'Error ระยะ (m)':<15} | {'เวลา (s)'}")
    print("-" * 50)

    for file in bag_files:
        bag_path = os.path.join(bag_folder, file)
        bag = rosbag.Bag(bag_path)
        
        start_pose = None
        end_pose = None
        start_time = None
        end_time = None

        # อ่านข้อมูลจาก Topic /amcl_pose (หรือเปลี่ยนเป็น /odom ถ้าต้องการดูความแม่นยำล้อ)
        for topic, msg, t in bag.read_messages(topics=['/amcl_pose']):
            if start_pose is None:
                start_pose = msg.pose.pose
                start_time = t.to_sec()
            end_pose = msg.pose.pose
            end_time = t.to_sec()
            
        bag.close()

        if start_pose and end_pose:
            # คำนวณความคลาดเคลื่อน (พิกัดสุดท้าย ควรจะกลับมาทับพิกัดเริ่มต้น)
            dx = end_pose.position.x - start_pose.position.x
            dy = end_pose.position.y - start_pose.position.y
            
            # Euclidean Distance Error (ระยะกระจัดที่เบี้ยวไป)
            distance_error = math.sqrt(dx**2 + dy**2)
            
            # เวลาที่ใช้
            duration = end_time - start_time

            print(f"{file[:28]+'..':<30} | {distance_error:<15.4f} | {duration:.2f}")

            total_distance_error += distance_error
            total_time += duration
            valid_tests += 1

    print("-" * 50)
    if valid_tests > 0:
        avg_dist_error = total_distance_error / valid_tests
        avg_time = total_time / valid_tests
        
        print("📊 สรุปผลการทดสอบ (นำไปใส่วิทยานิพนธ์ได้เลย):")
        print(f"จำนวนรอบที่ทดสอบ: {valid_tests} รอบ")
        print(f"ค่าเฉลี่ยความคลาดเคลื่อนตำแหน่ง (Mean Position Error): {avg_dist_error:.4f} เมตร ({avg_dist_error*100:.2f} ซม.)")
        print(f"ค่าเฉลี่ยเวลาที่ใช้ (Mean Duration): {avg_time:.2f} วินาที")
    print("-" * 50)

if __name__ == '__main__':
    # กำหนดโฟลเดอร์ที่เก็บไฟล์ bag
    data_folder = os.path.expanduser('~/ptr1_test_data')
    analyze_square_path(data_folder)