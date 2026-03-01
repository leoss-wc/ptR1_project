#!/usr/bin/env python
import rospy
import math
import csv
import os
import datetime
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

class TriggeredRotationTester:
    def __init__(self):
        rospy.init_node('triggered_rotation_tester', anonymous=True)
        
        # Publisher & Subscribers
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/rotate_cmd', Point, self.trigger_callback)
        rospy.Subscriber('/reset_angle', Empty, self.reset_callback)
        
        self.continuous_yaw = 0.0
        self.prev_yaw = None
        
        self.target_yaw = 0.0
        self.is_rotating = False
        self.start_time = None
        
        # --- ตั้งค่าพารามิเตอร์การหมุน ---
        self.kp = 1.0       
        self.max_w = 1.0    
        self.min_w = 0.15   
        self.tolerance = math.radians(1.0) 
        
        # --- ตั้งค่าไฟล์ CSV สำหรับเก็บข้อมูล ---
        self.csv_filename = 'rotation_log.csv'
        self.init_csv_file()
        
        self.rate = rospy.Rate(50)
        
        rospy.loginfo("Waiting for /odom data...")
        while self.prev_yaw is None and not rospy.is_shutdown():
            self.rate.sleep()
            
        self.print_menu()

    def init_csv_file(self):
        # สร้างไฟล์และเขียน Header ถ้าไฟล์ยังไม่มีอยู่
        if not os.path.isfile(self.csv_filename):
            with open(self.csv_filename, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Target_Angle(deg)', 'Actual_Angle(deg)', 'Error(deg)', 'Max_Speed(rad/s)', 'Time_Elapsed(sec)'])
            rospy.loginfo(f"Created new log file: {self.csv_filename}")
        else:
            rospy.loginfo(f"Appending to existing log file: {self.csv_filename}")

    def print_menu(self):
        rospy.loginfo("==========================================")
        rospy.loginfo("Ready! Waiting for commands:")
        rospy.loginfo("1. Rotate: rostopic pub -1 /rotate_cmd geometry_msgs/Point \"{x: 90.0, y: 1.5, z: 0.0}\"")
        rospy.loginfo("   (x = Angle in degrees, y = Max speed in rad/s)")
        rospy.loginfo("2. Reset:  rostopic pub -1 /reset_angle std_msgs/Empty \"{}\"")
        rospy.loginfo("==========================================")

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        if self.prev_yaw is not None:
            delta_yaw = yaw - self.prev_yaw
            while delta_yaw > math.pi: 
                delta_yaw -= 2.0 * math.pi
            while delta_yaw < -math.pi: 
                delta_yaw += 2.0 * math.pi
            self.continuous_yaw += delta_yaw
            
        self.prev_yaw = yaw

    def reset_callback(self, msg):
        self.continuous_yaw = 0.0
        self.is_rotating = False
        
        cmd = Twist()
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)
        
        rospy.loginfo("\n>>> ANGLE RESET TO 0.0 <<<")
        rospy.loginfo("You can start a new measurement now.\n")

    def trigger_callback(self, msg):
        if not self.is_rotating:
            angle_deg = msg.x
            
            # ดึงค่าความเร็วจาก msg.y และป้องกันค่าติดลบหรือค่า 0
            req_speed = abs(msg.y)
            if req_speed > 0.0:
                self.max_w = req_speed
            else:
                self.max_w = 1.0 # ใช้ค่า 1.0 เป็น Default ถ้าใส่ y มาเป็น 0
                rospy.logwarn("Speed (y) was 0, using default speed: 1.0 rad/s")
                
            rospy.loginfo(f"--> Received command to rotate {angle_deg} degrees at max speed {self.max_w} rad/s.")
            
            self.target_yaw = self.continuous_yaw + math.radians(angle_deg)
            self.is_rotating = True
            self.start_time = rospy.Time.now()
        else:
            rospy.logwarn("Robot is already rotating! Please wait until it stops or send Reset.")

    def run(self):
        while not rospy.is_shutdown():
            if self.is_rotating:
                error = self.target_yaw - self.continuous_yaw
                
                if abs(error) < self.tolerance:
                    cmd = Twist()
                    cmd.angular.z = 0.0
                    self.pub_cmd.publish(cmd)
                    
                    self.is_rotating = False
                    
                    # คำนวณเวลาที่ใช้ไป
                    duration = 0.0
                    if self.start_time is not None:
                        duration = (rospy.Time.now() - self.start_time).to_sec()
                    
                    # แปลงค่ากลับเป็นองศาเพื่อบันทึกลงไฟล์
                    target_deg = math.degrees(self.target_yaw)
                    actual_deg = math.degrees(self.continuous_yaw)
                    error_deg = math.degrees(error)
                    
                    # บันทึกข้อมูลลง CSV
                    self.save_to_csv(target_deg, actual_deg, error_deg, self.max_w, duration)
                    
                    rospy.loginfo(f"+++ Rotation Finished! Time elapsed: {duration:.2f} seconds. +++")
                    rospy.loginfo("+++ You can measure the angle now. +++")
                else:
                    cmd = Twist()
                    angular_speed = self.kp * error
                    
                    if angular_speed > 0:
                        angular_speed = min(max(angular_speed, self.min_w), self.max_w)
                    else:
                        angular_speed = max(min(angular_speed, -self.min_w), -self.max_w)
                        
                    cmd.angular.z = angular_speed
                    self.pub_cmd.publish(cmd)
                    
            self.rate.sleep()

    def save_to_csv(self, target, actual, error, speed, duration):
        try:
            with open(self.csv_filename, mode='a') as file:
                writer = csv.writer(file)
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                writer.writerow([timestamp, round(target, 2), round(actual, 2), round(error, 2), round(speed, 2), round(duration, 3)])
        except Exception as e:
            rospy.logerr(f"Failed to write to CSV: {e}")

if __name__ == '__main__':
    try:
        tester = TriggeredRotationTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass