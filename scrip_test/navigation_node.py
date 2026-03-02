#!/usr/bin/env python3
from std_msgs.msg import String
import rospy
import os
import subprocess
import json
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse 
import math

# Import Service ที่จำเป็น
from ptR1_navigation.srv import (StartAMCL, StartAMCLResponse, StopAMCL, StopAMCLResponse,
                                 StartPatrol, StartPatrolResponse, PausePatrol, PausePatrolResponse,
                                 ResumePatrol, ResumePatrolResponse, StopPatrol, StopPatrolResponse,
                                 SaveMap, SaveMapResponse)
#raspi directory
HOMES_FILE = os.path.expanduser('~/ptR1_ws/src/ptR1_navigation/config/map_homes.json')
POSE_FILE = os.path.expanduser('~/ptR1_ws/src/ptR1_navigation/config/last_pose.json')

#local directory
#POSE_FILE = os.path.expanduser('~/ptR1Project/ptR1_ws/src/ptR1_navigation/config/last_pose.json')
#HOMES_FILE = os.path.expanduser('~/ptR1Project/ptR1_ws/src/ptR1_navigation/config/map_homes.json')

class NavigationManager:
    def __init__(self):
        rospy.init_node('navigation_manager')
        rospy.loginfo("Navigation Manager Started")

        # --- State ---
        self.nav_process = None
        self.latest_pose = None
        self.auto_resume = False
        self.current_map_name = "unknown"
        
        # Patrol State
        self.goal_list = [] # ลิสต์เก็บ Goal ที่จะส่งไปในโหมด Patrol
        self.current_goal_index = 0 # ตัวแปรเก็บ index ของ Goal ที่กำลังจะส่งไป
        self.is_patrolling = False # ตัวแปรบอกสถานะว่ากำลังอยู่ในโหมด Patrol หรือเปล่า
        self.is_paused = False # ตัวแปรบอกสถานะว่ากำลังหยุดชั่วคราวอยู่หรือเปล่า
        self.should_loop = False # ตัวแปรบอกว่าหมดลิสต์แล้วจะวนใหม่หรือเปล่า
        self.is_rotating_to_goal = False # ตัวแปรบอกสถานะว่ากำลังหมุนอยู่หรือเปล่า
        
        # --- ROS Comms ---
        self.status_pub = rospy.Publisher('/nav/status', String, queue_size=10)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        rospy.Subscriber('/robot/cmd', String, self.cmd_callback)
        rospy.Subscriber('/map_manager/current_map_name', String, self.map_name_callback)
        # --- Services ---
        # Navigation (AMCL + MoveBase)
        rospy.Service('/nav/start', StartAMCL, self.handle_start_nav) 
        rospy.Service('/nav/stop', StopAMCL, self.handle_stop_nav)
        
        #Patrol
        rospy.Service('/nav/start_patrol', StartPatrol, self.handle_start_patrol)
        rospy.Service('/nav/stop_patrol', StopPatrol, self.handle_stop_patrol)
        rospy.Service('/nav/pause_patrol', PausePatrol, self.handle_pause_patrol)
        rospy.Service('/nav/resume_patrol', ResumePatrol, self.handle_resume_patrol)

        rospy.Service('/nav/set_home', SaveMap, self.handle_set_home) 
        rospy.Service('/nav/go_home', SaveMap, self.handle_go_home)
        rospy.Service('/nav/init_home', SaveMap, self.handle_init_home)
        
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Navigation Services Ready.")
    
    def cmd_callback(self, msg):
        """จัดการคำสั่งจาก /robot/cmd"""
        if not self.is_patrolling:
            return
        command = msg.data
        # กรณีได้รับ manual_on -> ให้ Pause Patrol (ถ้ากำลังเดินอยู่)
        if command == 'manual_on':
            if self.is_patrolling and not self.is_paused:
                rospy.LogInfo("Manual Mode ON: Pausing patrol...")
                self.is_paused = True
                self.move_base_client.cancel_goal() # สั่งหยุดหุ่นยนต์
                self.update_status("paused")
        
        # กรณีได้รับ manual_off -> ให้ Resume Patrol (ถ้าต้องการ)
        if self.auto_resume and command == 'manual_off':
            if self.is_patrolling and self.is_paused:
                rospy.LogInfo("Manual Mode OFF: AUTO Resuming patrol...")
                self.is_paused = False
                self.send_next_goal() # ส่ง Goal เดิมให้เดินต่อ
    def map_name_callback(self, msg):
        """อัปเดตชื่อแผนที่เมื่อ MapManager แจ้งมา"""
        self.current_map_name = msg.data
        rospy.loginfo(f"Nav node: Current map updated to: {self.current_map_name}")

    def update_status(self, status_text):
        self.status_pub.publish(status_text)
    # Pose Management ---
    def amcl_pose_callback(self, msg):
        self.latest_pose = msg

    def save_pose_to_file(self):
        """บันทึกตำแหน่งปัจจุบันลงไฟล์ JSON"""
        if self.latest_pose is None:
            rospy.logwarn("⚠️ No AMCL pose received yet. Cannot save.")
            return False
            
        try:
            pose_data = {
                "map_name": self.current_map_name,
                "position": {
                    "x": self.latest_pose.pose.pose.position.x,
                    "y": self.latest_pose.pose.pose.position.y,
                    "z": self.latest_pose.pose.pose.position.z
                },
                "orientation": {
                    "x": self.latest_pose.pose.pose.orientation.x,
                    "y": self.latest_pose.pose.pose.orientation.y,
                    "z": self.latest_pose.pose.pose.orientation.z,
                    "w": self.latest_pose.pose.pose.orientation.w
                },
                "covariance": list(self.latest_pose.pose.covariance),
                "frame_id": self.latest_pose.header.frame_id
            }
            
            os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
            with open(POSE_FILE, 'w') as f:
                json.dump(pose_data, f, indent=4)
                
            rospy.loginfo(f"💾 Saved last pose to {POSE_FILE}")
            return True
        except Exception as e:
            rospy.logerr(f"❌ Failed to save pose: {e}")
            return False

    def restore_pose(self):
        """อ่านไฟล์ JSON และ Publish ไปยัง /initialpose"""
        if not os.path.exists(POSE_FILE):
            rospy.logwarn("⚠️ No saved pose file found.")
            return False
            
        try:
            with open(POSE_FILE, 'r') as f:
                data = json.load(f)

            saved_map = data.get("map_name", "unknown")
            # ถ้าเรารู้ชื่อแผนที่ปัจจุบัน (ไม่ใช่ unknown) และมันไม่ตรงกับที่เซฟไว้
            if self.current_map_name != "unknown" and saved_map != self.current_map_name:
                rospy.logerr(f"Nav node: Map Mismatch Current: {self.current_map_name}, Saved: {saved_map}")
                rospy.logerr("Nav node: Aborting restore_pose to prevent localization errors.")
                return False
                
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = data.get("frame_id", "map")
            
            msg.pose.pose.position.x = data["position"]["x"]
            msg.pose.pose.position.y = data["position"]["y"]
            msg.pose.pose.position.z = data["position"]["z"]
            
            msg.pose.pose.orientation.x = data["orientation"]["x"]
            msg.pose.pose.orientation.y = data["orientation"]["y"]
            msg.pose.pose.orientation.z = data["orientation"]["z"]
            msg.pose.pose.orientation.w = data["orientation"]["w"]
            msg.pose.covariance = data["covariance"]
            
            # Publish ย้ำๆ เพื่อความชัวร์
            for _ in range(3):
                self.initial_pose_pub.publish(msg)
                rospy.sleep(0.2)
                
            rospy.loginfo("📍 Restored initial pose from file.")
            return True
        except Exception as e:
            rospy.logerr(f"❌ Failed to load pose: {e}")
            return False

    # Process Management ---
    def handle_start_nav(self, req):
        if self.nav_process:
            return StartAMCLResponse(True, "Navigation already running.")
        
        # รียกไฟล์ launch ที่รวม AMCL และ MoveBase ไว้
        cmd = ['roslaunch', 'ptR1_navigation', 'navigation_core.launch']
        self.nav_process = subprocess.Popen(cmd)
        
        if req.restore_pose:

            # รอระบบขึ้นสักครู่แล้วค่อย Restore Pose
            rospy.Timer(rospy.Duration(5.0), lambda e: self.restore_pose(), oneshot=True)

        return StartAMCLResponse(True, "Navigation System Started.")

    def handle_stop_nav(self, req):
        rospy.loginfo("Stopping Navigation Stack...")

        # บันทึกตำแหน่งล่าสุดก่อนปิด
        if req.save_pose:
            self.save_pose_to_file()
            
        # ปิด Process หลัก (ที่รัน launch file)
        if self.nav_process:
            self.nav_process.terminate()
            try:
                self.nav_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.nav_process.kill()
            self.nav_process = None
            rospy.loginfo("Navigation launch process terminated.")

        # สั่ง Kill Nodes ที่อาจจะค้างอยู่แบบเจาะจง (Force Kill)
        # เพื่อเคลียร์ Topic /map และ /tf ให้ว่างสำหรับ SLAM
        try:
            # ใช้ os.system เพื่อเรียกคำสั่ง rosnode kill
            os.system("rosnode kill /amcl /move_base /map_server")
            rospy.loginfo("Force killed: amcl, move_base, map_server")
        except Exception as e:
            rospy.logwarn(f"⚠️ Failed to force kill nodes: {e}")

        return StopAMCLResponse(True, "Navigation and Map Server Stopped.")

    # Patrol Logic ---
    def handle_start_patrol(self, req):
        if not req.goals:
            return StartPatrolResponse(False, "Goal list cannot be empty.")
        
        self.handle_stop_patrol(None)
        self.goal_list = req.goals
        self.should_loop = req.loop
        self.is_rotating_to_goal = False
        self.current_goal_index = 0
        self.is_patrolling = True
        self.is_paused = False
        self.update_status("active")
        rospy.loginfo(f"Starting patrol with {len(self.goal_list)} goals. Loop: {self.should_loop}")
        self.send_next_goal()
        return StartPatrolResponse(True, "Patrol started.")
    
    def handle_pause_patrol(self, req):
        if not self.is_patrolling:
            return PausePatrolResponse(False, "Not currently patrolling.")
            
        self.is_paused = True
        self.move_base_client.cancel_goal()
        self.update_status("paused")
        rospy.loginfo("Patrol paused.")
        return PausePatrolResponse(True, "Patrol paused.")

    def handle_resume_patrol(self, req):
        if not self.is_patrolling:
            return ResumePatrolResponse(False, "Not currently patrolling.")
        if not self.is_paused:
            return ResumePatrolResponse(False, "Patrol is not paused.")

        self.is_paused = False
        rospy.loginfo("Resuming patrol.")
        self.send_next_goal()
        return ResumePatrolResponse(True, "Patrol resumed.")

    def handle_stop_patrol(self, req):
        self.is_patrolling = False
        self.is_paused = False
        self.goal_list = []
        self.current_goal_index = 0
        self.move_base_client.cancel_all_goals()
        self.update_status("idle")
        if req is not None:
             rospy.loginfo("Patrol stopped.")
        return StopPatrolResponse(True, "Patrol stopped.")

    def get_quaternion_from_yaw(self, yaw):
        """แปลงมุม Yaw (radians) เป็น Quaternion (x, y, z, w)"""
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }
    
    def get_yaw_from_quaternion(q):
        """แปลง Quaternion เป็นมุม Yaw"""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def send_next_goal_with_rotation(self):
        if not self.is_patrolling or self.is_paused or not self.goal_list:
            return
        if self.latest_pose is None:
            rospy.logwarn("⚠️ Unknown robot pose. Cannot rotate. Sending direct goal.")
            self.is_rotating_to_goal = False
        # ตรวจสอบว่าถึงจุดสุดท้ายของลิสต์หรือยัง
        if self.current_goal_index >= len(self.goal_list):
            rospy.loginfo("Patrol sequence finished.")
            self.is_patrolling = False
            return
        target_pose_msg = self.goal_list[self.current_goal_index]
        # ถ้ายังไม่ได้หมุน (และรู้ตำแหน่งตัวเอง) ให้สั่งหมุนก่อน
        if not self.is_rotating_to_goal and self.latest_pose is not None:
            rospy.loginfo(f"Rotating towards Goal #{self.current_goal_index + 1}...")

            # 1. ตำแหน่งปัจจุบัน
            current_x = self.latest_pose.pose.pose.position.x
            current_y = self.latest_pose.pose.pose.position.y
            
            # 2. ตำแหน่งเป้าหมาย
            target_x = target_pose_msg.pose.position.x
            target_y = target_pose_msg.pose.position.y
            
            # 3. คำนวณมุมที่ต้องหัน (atan2)
            dx = target_x - current_x
            dy = target_y - current_y
            desired_yaw = math.atan2(dy, dx)
            
            # 4. สร้าง Rotation Goal (อยู่ที่เดิม แต่หันหน้าใหม่)
            rotation_goal = MoveBaseGoal()
            rotation_goal.target_pose.header.frame_id = "map"
            rotation_goal.target_pose.header.stamp = rospy.Time.now()
            
            # อยู่ที่เดิม
            rotation_goal.target_pose.pose.position.x = current_x
            rotation_goal.target_pose.pose.position.y = current_y
            rotation_goal.target_pose.pose.position.z = 0.0
            
            # หันหน้าใหม่
            q = self.get_quaternion_from_yaw(desired_yaw)
            rotation_goal.target_pose.pose.orientation.x = q['x']
            rotation_goal.target_pose.pose.orientation.y = q['y']
            rotation_goal.target_pose.pose.orientation.z = q['z']
            rotation_goal.target_pose.pose.orientation.w = q['w']

            # 5. ส่งคำสั่งและตั้ง State
            self.is_rotating_to_goal = True
            self.move_base_client.send_goal(rotation_goal, done_cb=self.goal_done_callback)

        else:
            # ถ้าหมุนเสร็จแล้ว (หรือหมุนไม่ได้) ให้สั่งเดินจริง
            rospy.loginfo(f"Moving to Goal #{self.current_goal_index + 1}")
            
            goal = MoveBaseGoal(target_pose=target_pose_msg)
            
            if not self.move_base_client.wait_for_server(rospy.Duration(1.0)):
                rospy.logwarn("move_base server not available.")
                self.is_paused = True
                return
            
            # ส่งคำสั่งเดินจริง
            self.update_status("active")
            self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)
    
    def send_next_goal(self):
        # ตรวจสอบความพร้อม
        if not self.is_patrolling or self.is_paused or not self.goal_list:
            return
            
        # ตรวจสอบว่าจบ Loop หรือยัง
        if self.current_goal_index >= len(self.goal_list):
            if self.should_loop:
                rospy.loginfo("Looping patrol...")
                self.current_goal_index = 0
            else:
                rospy.loginfo("Patrol sequence finished.")
                self.is_patrolling = False
                self.update_status("idle")
                return

        target_pose_msg = self.goal_list[self.current_goal_index]
        
        # --- ส่ง Goal ไปให้ MoveBase โดยตรง (ตัดส่วนหมุนตัวทิ้ง) ---
        rospy.loginfo(f"Moving to Goal #{self.current_goal_index + 1}")
        
        # สร้าง Goal Object
        goal = MoveBaseGoal()
        goal.target_pose = target_pose_msg 
        
        # ตรวจสอบ Header
        if not goal.target_pose.header.frame_id:
            goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # เช็คว่า move_base พร้อมไหม
        if not self.move_base_client.wait_for_server(rospy.Duration(1.0)):
            rospy.logwarn("move_base server not available.")
            self.is_paused = True
            return
        
        self.update_status("active")
        
        # ส่งคำสั่งเดินทันที! 
        # (DWA Planner จะคำนวณวิถีโค้งเพื่อเลี้ยวไปหาจุดต่อไปเอง โดยไม่หยุดหมุน)
        self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)

    def goal_done_callback(self, status, result):
        if not self.is_patrolling: return
        
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal #{self.current_goal_index + 1} reached.")
            self.current_goal_index += 1

            if self.current_goal_index >= len(self.goal_list):
                if self.should_loop:
                    rospy.loginfo("Looping patrol...")
                    self.current_goal_index = 0
                    rospy.Timer(rospy.Duration(0.1), lambda e: self.send_next_goal(), oneshot=True)
                else:
                    self.is_patrolling = False
            else:
                rospy.Timer(rospy.Duration(0.1), lambda e: self.send_next_goal(), oneshot=True)
        elif status == actionlib.GoalStatus.PREEMPTED and self.is_paused:
            rospy.loginfo("Goal cancelled due to pause. Waiting for resume...")
        else:
            rospy.logerr(f"Goal #{self.current_goal_index + 1} failed/aborted. Status: {status}")
            self.update_status("idle")
            self.is_patrolling = False

# --- 4. Home Management (Per Map) ---
    def _load_homes_data(self):
        """Helper อ่านไฟล์ JSON"""
        if os.path.exists(HOMES_FILE):
            try:
                with open(HOMES_FILE, 'r') as f:
                    return json.load(f)
            except:
                return {}
        return {}

    def handle_set_home(self, req):
        """บันทึกตำแหน่งปัจจุบันเป็น Home ของแผนที่ที่ระบุ"""
        map_name = req.name
        if not map_name:
            return SaveMapResponse(False, "Map name is required.")
        if self.latest_pose is None:
            return SaveMapResponse(False, "No pose received yet.")

        try:
            # 1. อ่านข้อมูลเก่า
            homes_data = self._load_homes_data()

            # 2. เตรียมข้อมูล Pose ปัจจุบัน
            pose_data = {
                "x": self.latest_pose.pose.pose.position.x,
                "y": self.latest_pose.pose.pose.position.y,
                "z": self.latest_pose.pose.pose.position.z,
                "ox": self.latest_pose.pose.pose.orientation.x,
                "oy": self.latest_pose.pose.pose.orientation.y,
                "oz": self.latest_pose.pose.pose.orientation.z,
                "ow": self.latest_pose.pose.pose.orientation.w,
                "frame_id": self.latest_pose.header.frame_id
            }

            # 3. บันทึกลง Key ชื่อแผนที่
            homes_data[map_name] = pose_data

            # 4. เขียนไฟล์กลับ
            os.makedirs(os.path.dirname(HOMES_FILE), exist_ok=True)
            with open(HOMES_FILE, 'w') as f:
                json.dump(homes_data, f, indent=4)

            rospy.loginfo(f"🏠 Home set for map '{map_name}'")
            return SaveMapResponse(True, f"Home set for {map_name}")
        except Exception as e:
            rospy.logerr(f"Failed to set home: {e}")
            return SaveMapResponse(False, str(e))

    def handle_go_home(self, req):
        """สั่งหุ่นยนต์เดินกลับ Home ของแผนที่นั้น (Navigation)"""
        map_name = req.name
        homes_data = self._load_homes_data()

        if map_name not in homes_data:
            return SaveMapResponse(False, f"No home defined for map '{map_name}'")

        # ดึงข้อมูลมาสร้าง Goal
        home = homes_data[map_name]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = home.get("frame_id", "map")
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = home["x"]
        goal.target_pose.pose.position.y = home["y"]
        goal.target_pose.pose.position.z = home["z"]
        goal.target_pose.pose.orientation.x = home["ox"]
        goal.target_pose.pose.orientation.y = home["oy"]
        goal.target_pose.pose.orientation.z = home["oz"]
        goal.target_pose.pose.orientation.w = home["ow"]

        # สั่งเดิน
        if not self.move_base_client.wait_for_server(rospy.Duration(1.0)):
             return SaveMapResponse(False, "MoveBase not ready.")
        
        self.move_base_client.send_goal(goal)
        return SaveMapResponse(True, f"Going to home of {map_name}...")

    def handle_init_home(self, req):
        """ตั้งค่า Initial Pose ไปที่จุด Home (Teleport ใน AMCL)"""
        map_name = req.name
        homes_data = self._load_homes_data()

        if map_name not in homes_data:
            return SaveMapResponse(False, f"No home defined for map '{map_name}'")

        home = homes_data[map_name]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = home.get("frame_id", "map")
        msg.pose.pose.position.x = home["x"]
        msg.pose.pose.position.y = home["y"]
        msg.pose.pose.position.z = home["z"]
        msg.pose.pose.orientation.x = home["ox"]
        msg.pose.pose.orientation.y = home["oy"]
        msg.pose.pose.orientation.z = home["oz"]
        msg.pose.pose.orientation.w = home["ow"]
        # ใส่ covariance มาตรฐาน
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]

        self.initial_pose_pub.publish(msg)
        # ส่งซ้ำเพื่อความชัวร์
        rospy.sleep(0.1)
        self.initial_pose_pub.publish(msg)
        
        return SaveMapResponse(True, f"Initial pose set to home of {map_name}")
    def cleanup(self):
        # สร้าง Dummy class ง่ายๆ เพื่อส่งค่า save_pose=True
        class DummyReq:
            save_pose = True
        self.handle_stop_nav(DummyReq())

if __name__ == '__main__':
    try:
        NavigationManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass