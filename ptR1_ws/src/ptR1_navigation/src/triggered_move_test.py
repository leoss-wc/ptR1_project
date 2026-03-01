#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

class TriggeredMoveTester:
    def __init__(self):
        rospy.init_node('triggered_move_tester', anonymous=True)
        
        # Publisher & Subscribers
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_cmd', Point, self.trigger_callback)   # รับคำสั่งระยะทาง (X, Y) และความเร็ว (Z)
        rospy.Subscriber('/reset_dist', Empty, self.reset_callback)   # รับคำสั่ง Reset
        
        # ตัวแปรเก็บตำแหน่ง
        self.current_x = 0.0
        self.current_y = 0.0
        self.odom_received = False
        
        # ตัวแปรควบคุมเป้าหมายและการทดสอบ
        self.target_x = 0.0
        self.target_y = 0.0
        self.is_moving = False
        self.move_axis = 'x'       # ตัวแปรบอกว่าตอนนี้กำลังเทสแกนไหน
        self.start_time = 0.0      # ตัวแปรสำหรับจับเวลา
        
        # --- ตั้งค่าพารามิเตอร์ P-Controller ---
        self.kp = 0.8         
        self.max_v = 0.4      
        self.min_v = 0.05     
        self.tolerance = 0.02 # ยอมรับความคลาดเคลื่อนที่ 2 เซนติเมตร (0.02 เมตร)
        
        self.rate = rospy.Rate(50) # 50 Hz loop
        
        rospy.loginfo("Waiting for /odom data...")
        while not self.odom_received and not rospy.is_shutdown():
            self.rate.sleep()
            
        self.print_menu()

    def print_menu(self):
        rospy.loginfo("==========================================")
        rospy.loginfo("   PURE 1D MOVEMENT TESTER (NO DRIFT CORRECTION)   ")
        rospy.loginfo("==========================================")
        rospy.loginfo("1. Test X-Axis (Forward 3m, Spd 0.5):")
        rospy.loginfo("   rostopic pub -1 /move_cmd geometry_msgs/Point \"{x: 3.0, y: 0.0, z: 0.5}\"")
        rospy.loginfo("2. Test Y-Axis (Slide Left 2m, Spd 0.3):")
        rospy.loginfo("   rostopic pub -1 /move_cmd geometry_msgs/Point \"{x: 0.0, y: 2.0, z: 0.3}\"")
        rospy.loginfo("3. Reset Tracker:")
        rospy.loginfo("   rostopic pub -1 /reset_dist std_msgs/Empty \"{}\"")
        rospy.loginfo("==========================================")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_received = True

    def reset_callback(self, msg):
        # ยกเลิกการเคลื่อนที่และหยุดมอเตอร์
        self.is_moving = False
        cmd = Twist()
        self.pub_cmd.publish(cmd)
        
        rospy.loginfo("\n>>> DISTANCE TRACKER RESET <<<")
        rospy.loginfo("You can send a new /move_cmd now.\n")

    def trigger_callback(self, msg):
        if not self.is_moving:
            dist_x = msg.x
            dist_y = msg.y
            target_speed = abs(msg.z) 
            
            if target_speed > 0.0:
                self.max_v = target_speed
            else:
                self.max_v = 0.4
                
            # --- วิเคราะห์ว่าจะทดสอบแกนไหน ---
            if abs(dist_x) >= abs(dist_y):
                self.move_axis = 'x'
                rospy.loginfo(f"--> [TESTING X AXIS] Move X = {dist_x}m at max speed {self.max_v} m/s")
            else:
                self.move_axis = 'y'
                rospy.loginfo(f"--> [TESTING Y AXIS] Move Y = {dist_y}m at max speed {self.max_v} m/s")
            
            # ตั้งเป้าหมายและเริ่มจับเวลา
            self.target_x = self.current_x + dist_x
            self.target_y = self.current_y + dist_y
            self.start_time = rospy.Time.now().to_sec()
            self.is_moving = True
            
        else:
            rospy.logwarn("Robot is already moving! Please wait or send Reset.")

    def clamp_velocity(self, velocity):
        # ฟังก์ชันจำกัดความเร็ว
        if abs(velocity) < 0.001:
            return 0.0
        if velocity > 0:
            return min(max(velocity, self.min_v), self.max_v)
        else:
            return max(min(velocity, -self.min_v), -self.max_v)

    def run(self):
        while not rospy.is_shutdown():
            if self.is_moving:
                # คำนวณ Error แยกแต่ละแกน
                error_x = self.target_x - self.current_x
                error_y = self.target_y - self.current_y
                
                cmd = Twist()
                
                # ==========================================
                # โหมดทดสอบแกน X (เดินหน้า-ถอยหลังเพียวๆ)
                # ==========================================
                if self.move_axis == 'x':
                    if abs(error_x) < self.tolerance:
                        self.pub_cmd.publish(cmd) # หยุดมอเตอร์
                        self.is_moving = False
                        elapsed_time = rospy.Time.now().to_sec() - self.start_time
                        rospy.loginfo("+++ X-Axis Target Reached! +++")
                        rospy.loginfo(f"Time Taken: {elapsed_time:.2f} seconds")
                    else:
                        cmd.linear.x = self.clamp_velocity(self.kp * error_x)
                        cmd.linear.y = 0.0  # บังคับ Y ให้เป็น 0 ตลอดการเดินทาง
                        self.pub_cmd.publish(cmd)
                        
                # ==========================================
                # โหมดทดสอบแกน Y (สไลด์ซ้าย-ขวาเพียวๆ)
                # ==========================================
                elif self.move_axis == 'y':
                    if abs(error_y) < self.tolerance:
                        self.pub_cmd.publish(cmd) # หยุดมอเตอร์
                        self.is_moving = False
                        elapsed_time = rospy.Time.now().to_sec() - self.start_time
                        rospy.loginfo("+++ Y-Axis Target Reached! +++")
                        rospy.loginfo(f"Time Taken: {elapsed_time:.2f} seconds")
                    else:
                        cmd.linear.x = 0.0  # บังคับ X ให้เป็น 0 ตลอดการเดินทาง
                        cmd.linear.y = self.clamp_velocity(self.kp * error_y)
                        self.pub_cmd.publish(cmd)
                    
            self.rate.sleep()

if __name__ == '__main__':
    try:
        tester = TriggeredMoveTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass