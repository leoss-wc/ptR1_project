#!/usr/bin/env python3
import rospy
import actionlib
import subprocess
import os
import signal
from datetime import datetime
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

class NavTester:
    def __init__(self):
        rospy.init_node('ptr1_nav_tester', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("กำลังรอเชื่อมต่อกับ move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("เชื่อมต่อ move_base สำเร็จแล้ว!")
        
        self.bag_dir = os.path.join(os.path.expanduser('~'), 'ptr1_test_data')
        if not os.path.exists(self.bag_dir):
            os.makedirs(self.bag_dir)

    def clear_costmaps(self):
        rospy.loginfo("กำลังล้าง Costmaps...")
        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_srv()
            rospy.loginfo("ล้าง Costmaps สำเร็จ!")
        except rospy.ServiceException as e:
            rospy.logerr(f"เรียกใช้งาน Service ล้มเหลว: {e}")

    def _start_rosbag(self, test_name):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_test_name = test_name.replace(" ", "_").replace("(", "").replace(")", "").replace("+", "plus").replace("-", "minus")
        bag_filename = os.path.join(self.bag_dir, f"ptr1_{safe_test_name}_{timestamp}.bag")

        topics_to_record = [
            "/odom", "/cmd_vel", "/amcl_pose", "/tf", "/tf_static",
            "/move_base/TebLocalPlannerROS/global_plan",
            "/move_base/TebLocalPlannerROS/local_plan","/pi/system_profile"
        ]
        rospy.loginfo(f"กำลังเริ่มบันทึกข้อมูลลงไฟล์: {bag_filename}")
        rosbag_cmd = ["rosbag", "record", "-O", bag_filename] + topics_to_record
        process = subprocess.Popen(rosbag_cmd)
        rospy.sleep(0.5) # รอให้ rosbag เริ่มทำงาน
        return process, bag_filename

    def _stop_rosbag(self, process):
        rospy.loginfo("กำลังหยุดบันทึก rosbag และเซฟไฟล์...")
        process.send_signal(signal.SIGINT)
        process.wait()

    def _send_single_goal(self, x, y, z, w):
        """ส่งคำสั่ง 1 ครั้งแบบไม่อัด rosbag ซ้ำซ้อน (ใช้เป็นฟังก์ชันลูก)"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_state()

    def send_test_goal(self, x, y, z, w, test_name):
        rosbag_process, bag_filename = self._start_rosbag(test_name)
        rospy.loginfo(f"--- 🚀 เริ่มทดสอบ: {test_name} ---")
        
        start_time = rospy.Time.now()
        state = self._send_single_goal(x, y, z, w)
        duration = (rospy.Time.now() - start_time).to_sec()

        self._stop_rosbag(rosbag_process)

        if state == 3:
            rospy.loginfo(f"✅ ถึงเป้าหมายเรียบร้อย! ใช้เวลาไป: {duration:.2f} วินาที")
        else:
            rospy.logwarn(f"❌ ไม่สามารถไปถึงเป้าหมายได้ (State: {state})")
        print("-" * 60)

    def test_square_path(self, side_length=1.0):
        test_name = f"Square_Path_{side_length}m"
        rosbag_process, bag_filename = self._start_rosbag(test_name)
        
        rospy.loginfo(f"--- 🚀 เริ่มทดสอบ: เดินเป็นสี่เหลี่ยมจัตุรัส ขนาด {side_length}x{side_length} เมตร ---")
        start_time = rospy.Time.now()

        # วนลูป 4 ครั้ง (4 ด้าน)
        for i in range(1, 5):
            rospy.loginfo(f"📍 ด้านที่ {i}/4: กำลังเดินหน้า {side_length} เมตร...")
            state_move = self._send_single_goal(side_length, 0.0, 0.0, 1.0)
            if state_move != 3:
                rospy.logwarn("⚠️ หุ่นยนต์เดินหน้าไม่สำเร็จ ยกเลิกการทดสอบกลางคัน")
                break
            
            rospy.loginfo(f"🔄 ด้านที่ {i}/4: กำลังหมุนซ้าย 90 องศา...")
            state_turn = self._send_single_goal(0.0, 0.0, 0.707, 0.707)
            if state_turn != 3:
                rospy.logwarn("⚠️ หุ่นยนต์หมุนตัวไม่สำเร็จ ยกเลิกการทดสอบกลางคัน")
                break

        duration = (rospy.Time.now() - start_time).to_sec()
        self._stop_rosbag(rosbag_process)
        
        rospy.loginfo(f"✅ จบการทดสอบเดินสี่เหลี่ยม! ใช้เวลารวม: {duration:.2f} วินาที")
        rospy.loginfo(f"📁 ข้อมูลถูกบันทึกที่: {bag_filename}")
        print("-" * 60)


def main():
    tester = NavTester()
    
    while not rospy.is_shutdown():
        print("\n=== ptR1 Navigation & Data Collection ===")
        print("1. Forward 3 m (X+)")
        print("2. Backward 3 m (X-)")
        print("3. Left 3 m (Y+)")
        print("4. Right 3 m (Y-)")
        print("5. Rotate 180")
        print("6. Rotate 90 (Left)")
        print("7. Rotate -90 (Right)")
        print("8. Clear Costmaps")
        print("9. Square Path (2x2m - Move & Turn)")
        print("0. Exit")
        
        choice = input("(0-9): ")
        
        if choice == '1': tester.send_test_goal(3.0, 0.0, 0.0, 1.0, "Forward_3m")
        elif choice == '2': tester.send_test_goal(-3.0, 0.0, 0.0, 1.0, "Backward_3m")
        elif choice == '3': tester.send_test_goal(0.0, 3.0, 0.0, 1.0, "Slide_Left_3m")
        elif choice == '4': tester.send_test_goal(0.0, -3.0, 0.0, 1.0, "Slide_Right_3m")
        elif choice == '5': tester.send_test_goal(0.0, 0.0, 1.0, 0.0, "Rotate_180")
        elif choice == '6': tester.send_test_goal(0.0, 0.0, 0.707, 0.707, "Rotate_90")
        elif choice == '7': tester.send_test_goal(0.0, 0.0, -0.707, 0.707, "Rotate_minus90")
        elif choice == '8': tester.clear_costmaps()
        elif choice == '9': tester.test_square_path(side_length=2.0)
        elif choice == '0':
            rospy.loginfo("Exiting...")
            break
        else:
            print("กรุณาเลือกตัวเลข 0-9")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass