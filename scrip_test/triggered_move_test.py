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
        
        # ตัวแปรควบคุมเป้าหมาย
        self.target_x = 0.0
        self.target_y = 0.0
        self.is_moving = False
        
        # --- ตั้งค่าพารามิเตอร์ P-Controller (ปรับให้เข้ากับ PID ฝั่งบอร์ด) ---
        self.kp = 0.8         # Gain ความเร็ว 
        self.max_v = 0.4      # ความเร็วเชิงเส้นสูงสุดเริ่มต้น (m/s) 
        self.min_v = 0.05     # ความเร็วเชิงเส้นต่ำสุด (m/s) ป้องกันล้อไม่มีแรงหมุนเข้าเป้า
        self.tolerance = 0.02 # ยอมรับความคลาดเคลื่อนที่ 2 เซนติเมตร (0.02 เมตร)
        self.start_time = 0.0
        
        self.rate = rospy.Rate(50) # 50 Hz loop
        
        rospy.loginfo("Waiting for /odom data...")
        while not self.odom_received and not rospy.is_shutdown():
            self.rate.sleep()
            
        self.print_menu()

    def print_menu(self):
        rospy.loginfo("==========================================")
        rospy.loginfo("Ready! Waiting for commands:")
        rospy.loginfo("1. Move X (Forward 3m, Speed 0.5):  rostopic pub -1 /move_cmd geometry_msgs/Point \"{x: 3.0, y: 0.0, z: 0.5}\"")
        rospy.loginfo("2. Move Y (Slide Left 3m, Speed 0.2): rostopic pub -1 /move_cmd geometry_msgs/Point \"{x: 0.0, y: 3.0, z: 0.2}\"")
        rospy.loginfo("3. Reset Start Point:               rostopic pub -1 /reset_dist std_msgs/Empty \"{}\"")
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
            target_speed = abs(msg.z) # ดึงค่า z มาเป็นความเร็วเป้าหมาย
            
            # ตรวจสอบว่ามีการส่งความเร็วมาหรือไม่ ถ้าไม่ได้ใส่มาให้ใช้ค่าเริ่มต้น 0.4
            if target_speed > 0.0:
                self.max_v = target_speed
            else:
                self.max_v = 0.4
                
            rospy.loginfo(f"--> Received command to move: X = {dist_x}m, Y = {dist_y}m at max speed {self.max_v} m/s")
            
            # คำนวณเป้าหมายโดยบวกเพิ่มจากตำแหน่ง *ปัจจุบัน*
            self.target_x = self.current_x + dist_x
            self.target_y = self.current_y + dist_y
            self.is_moving = True
            self.start_time = rospy.Time.now().to_sec()
        else:
            rospy.logwarn("Robot is already moving! Please wait or send Reset.")

    def clamp_velocity(self, velocity):
        # ฟังก์ชันจำกัดความเร็วไม่ให้เกิน max_v และไม่ต่ำกว่า min_v
        if abs(velocity) < 0.02:
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
                
                # หาระยะห่างรวม (Euclidean distance)
                distance_left = math.sqrt(error_x**2 + error_y**2)
                
                cmd = Twist()
                
                # ถ้าเข้าใกล้เป้าหมายในระยะที่รับได้แล้ว ให้หยุด
                if distance_left < self.tolerance:
                    self.pub_cmd.publish(cmd) # ส่ง 0.0 ไปหยุดมอเตอร์
                    self.is_moving = False
                    rospy.loginfo("+++ Movement Finished! You can measure the distance now. +++")
                    elapsed_time = rospy.Time.now().to_sec() - self.start_time
                    rospy.loginfo(f"Elapsed time: {elapsed_time:.2f} seconds")
                else:
                    # จ่ายความเร็วตามแกน X และ Y
                    vel_x = self.kp * error_x
                    vel_y = self.kp * error_y
                    
                    # หุ่นจะเร่งความเร็วไปถึงค่า z ที่เราส่งไป และลดความเร็วเมื่อเข้าใกล้เป้าหมาย
                    cmd.linear.x = self.clamp_velocity(vel_x)
                    cmd.linear.y = self.clamp_velocity(vel_y)
                    
                    self.pub_cmd.publish(cmd)
                    
            self.rate.sleep()

if __name__ == '__main__':
    try:
        tester = TriggeredMoveTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass