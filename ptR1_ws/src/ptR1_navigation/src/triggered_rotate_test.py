#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Empty
from tf.transformations import euler_from_quaternion

class TriggeredRotationTester:
    def __init__(self):
        rospy.init_node('triggered_rotation_tester', anonymous=True)
        
        # Publisher & Subscribers
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/rotate_cmd', Float32, self.trigger_callback) # รับคำสั่งหมุน
        rospy.Subscriber('/reset_angle', Empty, self.reset_callback)    # รับคำสั่ง Reset
        
        self.continuous_yaw = 0.0
        self.prev_yaw = None
        
        self.target_yaw = 0.0
        self.is_rotating = False
        
        # --- ตั้งค่าพารามิเตอร์การหมุน ---
        self.kp = 1.0       # Gain ความเร็ว
        self.max_w = 1.0    # ความเร็วเชิงมุมสูงสุด (rad/s)
        self.min_w = 0.15   # ความเร็วเชิงมุมต่ำสุด (rad/s)
        self.tolerance = math.radians(1.0) # ยอมรับความคลาดเคลื่อน 1 องศา
        
        self.rate = rospy.Rate(50)
        
        rospy.loginfo("Waiting for /odom data...")
        while self.prev_yaw is None and not rospy.is_shutdown():
            self.rate.sleep()
            
        self.print_menu()

    def print_menu(self):
        rospy.loginfo("==========================================")
        rospy.loginfo("Ready! Waiting for commands:")
        rospy.loginfo("1. Rotate: rostopic pub -1 /rotate_cmd std_msgs/Float32 \"data: 90.0\"")
        rospy.loginfo("2. Reset:  rostopic pub -1 /reset_angle std_msgs/Empty \"{}\"")
        rospy.loginfo("==========================================")

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # แปลงให้หมุนทะลุ 180 องศาได้ (Continuous Yaw)
        if self.prev_yaw is not None:
            delta_yaw = yaw - self.prev_yaw
            while delta_yaw > math.pi: 
                delta_yaw -= 2.0 * math.pi
            while delta_yaw < -math.pi: 
                delta_yaw += 2.0 * math.pi
            self.continuous_yaw += delta_yaw
            
        self.prev_yaw = yaw

    def reset_callback(self, msg):
        # เซ็ตให้มุมปัจจุบันกลายเป็น 0 องศาใหม่
        self.continuous_yaw = 0.0
        self.is_rotating = False
        
        # สั่งหยุดมอเตอร์เพื่อความปลอดภัย
        cmd = Twist()
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)
        
        rospy.loginfo("\n>>> ANGLE RESET TO 0.0 <<<")
        rospy.loginfo("You can start a new measurement now.\n")

    def trigger_callback(self, msg):
        if not self.is_rotating:
            angle_deg = msg.data
            rospy.loginfo(f"--> Received command to rotate {angle_deg} degrees.")
            
            # คำนวณเป้าหมายใหม่โดยอ้างอิงจากมุมปัจจุบัน
            self.target_yaw = self.continuous_yaw + math.radians(angle_deg)
            self.is_rotating = True
        else:
            rospy.logwarn("Robot is already rotating! Please wait until it stops or send Reset.")

    def run(self):
        while not rospy.is_shutdown():
            if self.is_rotating:
                error = self.target_yaw - self.continuous_yaw
                
                # ตรวจสอบว่าถึงเป้าหมายหรือยัง
                if abs(error) < self.tolerance:
                    cmd = Twist()
                    cmd.angular.z = 0.0
                    self.pub_cmd.publish(cmd)
                    
                    self.is_rotating = False
                    rospy.loginfo("+++ Rotation Finished! You can measure the angle now. +++")
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

if __name__ == '__main__':
    try:
        tester = TriggeredRotationTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass