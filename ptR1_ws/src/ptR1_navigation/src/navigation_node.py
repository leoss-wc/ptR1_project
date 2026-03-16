#!/usr/bin/env python3
from std_msgs.msg import String
import rospy
import os
import subprocess
import json
import actionlib
import threading 
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import copy
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
        self.current_map_name = "unknown"

        self._lock = threading.Lock()

        self.auto_resume = rospy.get_param('~auto_resume', False)
        
        # Patrol State
        self.goal_list = []         # ลิสต์เก็บ Goal ที่จะส่งไปในโหมด Patrol
        self.current_goal_index = 0 # ตัวแปรเก็บ index ของ Goal ที่กำลังจะส่งไป
        self.is_patrolling = False  # ตัวแปรบอกสถานะว่ากำลังอยู่ในโหมด Patrol หรือเปล่า
        self.is_paused = False      # ตัวแปรบอกสถานะว่ากำลังหยุดชั่วคราวอยู่หรือเปล่า
        self.should_loop = False    # ตัวแปรบอกว่าหมดลิสต์แล้วจะวนใหม่หรือเปล่า
        
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
        
        # Patrol
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
        with self._lock:
            if not self.is_patrolling:
                return
            command = msg.data
            if command == 'manual_on' and not self.is_paused:
                self.is_paused = True
                should_cancel = True
            elif self.auto_resume and command == 'manual_off' and self.is_paused:
                self.is_paused = False
                should_resume = True

        # เรียก ROS calls นอก lock เสมอ
        if should_cancel:
            self.move_base_client.cancel_goal()
            self.update_status("paused")
        if should_resume:
            rospy.Timer(rospy.Duration(0.1), lambda e: self.send_next_goal(), oneshot=True)


    def map_name_callback(self, msg):
        """อัปเดตชื่อแผนที่เมื่อ MapManager แจ้งมา"""
        self.current_map_name = msg.data
        rospy.loginfo(f"Nav node: Current map updated to: {self.current_map_name}")

    def update_status(self, status_text):
        self.status_pub.publish(status_text)

    # --- Pose Management ---
    def amcl_pose_callback(self, msg):
        self.latest_pose = msg

    def save_pose_to_file(self):
        """บันทึกตำแหน่งปัจจุบันลงไฟล์ JSON"""
        if self.current_map_name == "unknown":
            rospy.logwarn("⚠️ Refusing to save pose: map name is unknown.")
            return False
        if self.latest_pose is None:
            rospy.logwarn("⚠️ No AMCL pose received yet. Cannot save.")
            return False
        pose_snapshot = self.latest_pose
        if pose_snapshot is None:
            return False
        try:
            pose_data = {
                "map_name": self.current_map_name,
                "position": {
                    "x": pose_snapshot.pose.pose.position.x,
                    "y": pose_snapshot.pose.pose.position.y,
                    "z": pose_snapshot.pose.pose.position.z
                },
                "orientation": {
                    "x": pose_snapshot.pose.pose.orientation.x,
                    "y": pose_snapshot.pose.pose.orientation.y,
                    "z": pose_snapshot.pose.pose.orientation.z,
                    "w": pose_snapshot.pose.pose.orientation.w
                },
                "covariance": list(pose_snapshot.pose.covariance),
                "frame_id": pose_snapshot.header.frame_id
            }
            
            os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
            with open(POSE_FILE, 'w') as f:
                json.dump(pose_data, f, indent=4)
                
            rospy.loginfo(f"Saved last pose to {POSE_FILE}")
            return True
        except Exception as e:
            rospy.logerr(f"❌ Failed to save pose: {e}")
            return False

    def restore_pose(self):
        """อ่านไฟล์ JSON และ Publish ไปยัง /initialpose"""
        if self.current_map_name == "unknown" or saved_map == "unknown":
            rospy.logwarn("⚠️ ❌ Cannot verify map match (unknown). Aborting restore.")
        if not os.path.exists(POSE_FILE):
            rospy.logwarn("⚠️ No saved pose file found.")
            return False
            
        try:
            with open(POSE_FILE, 'r') as f:
                data = json.load(f)

            saved_map = data.get("map_name", "unknown")
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
        
            for _ in range(3):
                self.initial_pose_pub.publish(msg)
                rospy.sleep(0.2)
                
            rospy.loginfo("📍 Restored initial pose from file.")
            return True
        except Exception as e:
            rospy.logerr(f"❌ Failed to load pose: {e}")
            return False

    # --- Process Management ---
    def handle_start_nav(self, req):
        """เริ่ม Navigation Stack (AMCL + MoveBase)"""
        if self.nav_process:
            return StartAMCLResponse(True, "Navigation already running.")
        
        cmd = ['roslaunch', 'ptR1_navigation', 'navigation_core.launch']
        self.nav_process = subprocess.Popen(cmd)
        
        if req.restore_pose:
            # รอระบบขึ้นสักครู่แล้วค่อย Restore Pose
            threading.Thread(target=self._wait_and_restore, daemon=True)

        return StartAMCLResponse(True, "Navigation System Started.")

    def _wait_and_restore(self):
        """รอจนกว่า AMCL จะพร้อมรับ initialpose จริงๆ"""
        rospy.loginfo("Waiting for AMCL to be ready...")
        
        # รอ /amcl topic มีคนส่งมาก่อน (แปลว่า AMCL ขึ้นแล้ว)
        try:
            rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=30.0)
            rospy.sleep(1.0)  # buffer เล็กน้อยหลัง AMCL ขึ้น
            self.restore_pose()
        except rospy.ROSException:
            rospy.logwarn("⚠️ Timed out waiting for AMCL. Restore pose skipped.")

    def handle_stop_nav(self, req):
        """หยุด Navigation Stack"""
        rospy.loginfo("Stopping Navigation Stack...")
        self.handle_stop_patrol(None)

        if req.save_pose:
            self.save_pose_to_file()
            
        if self.nav_process:
            self.nav_process.terminate()
            try:
                self.nav_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.nav_process.kill()
            self.nav_process = None
            rospy.loginfo("Navigation launch process terminated.")
        try:
            subprocess.Popen(["rosnode", "kill", "/amcl", "/move_base", "/map_server"])
            rospy.loginfo("Force killed: amcl, move_base, map_server")
        except Exception as e:
            rospy.logwarn(f"⚠️ Failed to force kill nodes: {e}")

        return StopAMCLResponse(True, "Navigation and Map Server Stopped.")

    # --- Patrol Logic ---
    def handle_start_patrol(self, req):
        """เริ่ม Patrol ด้วย goal list ที่กำหนด"""
        if not req.goals:
            return StartPatrolResponse(False, "Goal list cannot be empty.")
        
        self.handle_stop_patrol(None)

        with self._lock:
            self.goal_list = req.goals
            self.should_loop = req.loop
            self.current_goal_index = 0
            self.is_patrolling = True
            self.is_paused = False

        self.update_status("active")
        rospy.loginfo(f"Starting patrol with {len(self.goal_list)} goals. Loop: {self.should_loop}")
        self.send_next_goal()
        return StartPatrolResponse(True, "Patrol started.")
    
    def handle_pause_patrol(self, req):
        """หยุด Patrol ชั่วคราว"""
        with self._lock:
            if not self.is_patrolling:
                return PausePatrolResponse(False, "Not currently patrolling.")
            if self.is_paused:
                return PausePatrolResponse(False, "Patrol is already paused.")
            self.is_paused = True

        self.move_base_client.cancel_goal()
        self.update_status("paused")
        rospy.loginfo("Patrol paused.")
        return PausePatrolResponse(True, "Patrol paused.")

    def handle_resume_patrol(self, req):
        """Resume Patrol ที่หยุดอยู่"""
        with self._lock:
            if not self.is_patrolling:
                return ResumePatrolResponse(False, "Not currently patrolling.")
            if not self.is_paused:
                return ResumePatrolResponse(False, "Patrol is not paused.")
            self.is_paused = False

        rospy.loginfo("Resuming patrol.")
        rospy.Timer(rospy.Duration(0.1), lambda e: self.send_next_goal(), oneshot=True)
        return ResumePatrolResponse(True, "Patrol resumed.")

    def handle_stop_patrol(self, req):
        """หยุด Patrol สมบูรณ์"""
        with self._lock:
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

    def send_next_goal(self):
        """ส่ง Goal ถัดไปให้ move_base"""
        with self._lock:
            if not self.is_patrolling or self.is_paused or not self.goal_list:
                return
            if self.current_goal_index >= len(self.goal_list):
                rospy.logwarn("send_next_goal: index out of range, stopping.")
                self.is_patrolling = False
                self.update_status("idle")
                return
            # snapshot ค่าที่ต้องใช้ก่อนออกจาก Lock
            current_index = self.current_goal_index
            goal_list_snapshot = self.goal_list
            should_loop = self.should_loop

        target_pose_msg = copy.deepcopy(goal_list_snapshot[current_index])

        # --- LOOK-AHEAD HEADING LOGIC ---
        next_index = current_index + 1
        has_next_goal = False
        
        if next_index < len(goal_list_snapshot):
            next_pose_msg = goal_list_snapshot[next_index]
            has_next_goal = True
        elif should_loop and len(goal_list_snapshot) > 1:
            next_pose_msg = goal_list_snapshot[0]
            has_next_goal = True
            
        if has_next_goal:
            current_goal_x = target_pose_msg.pose.position.x
            current_goal_y = target_pose_msg.pose.position.y
            next_goal_x = next_pose_msg.pose.position.x
            next_goal_y = next_pose_msg.pose.position.y
            
            dist = math.hypot(next_goal_x - current_goal_x, next_goal_y - current_goal_y)
            
            if dist > 0.05:
                lookahead_yaw = math.atan2(next_goal_y - current_goal_y, next_goal_x - current_goal_x)
                q = self.get_quaternion_from_yaw(lookahead_yaw)
                target_pose_msg.pose.orientation.x = q['x']
                target_pose_msg.pose.orientation.y = q['y']
                target_pose_msg.pose.orientation.z = q['z']
                target_pose_msg.pose.orientation.w = q['w']
                
                next_id = next_index if next_index < len(goal_list_snapshot) else 0
                rospy.loginfo(f"Look-ahead Active: Pre-aligning heading towards Goal #{next_id + 1}")
        # ==========================================
        
        # --- ส่ง Goal ไปให้ MoveBase ---
        rospy.loginfo(f"Moving to Goal #{current_index + 1}")
        
        goal = MoveBaseGoal()
        goal.target_pose = target_pose_msg 
        
        if not goal.target_pose.header.frame_id:
            goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        if not self.move_base_client.wait_for_server(rospy.Duration(1.0)):
            rospy.logwarn("move_base server not available. Will retry in 3s...")
            with self._lock:
                # เช็คว่ายังควร patrol อยู่ไหมก่อน re-pause
                if not self.is_patrolling:
                    return
                self.is_paused = True
            self.update_status("paused")
            rospy.Timer(rospy.Duration(3.0), lambda e: self._retry_resume(), oneshot=True)
            return

        with self._lock:
            if not self.is_patrolling or self.is_paused:
                return

        self.update_status("active")
        self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)

    def _retry_resume(self):
        """Helper: resume อัตโนมัติหลัง move_base ไม่พร้อม"""
        with self._lock:
            if not self.is_patrolling:
                return
            self.is_paused = False
        self.send_next_goal()

    def goal_done_callback(self, status, result):
        """Callback เมื่อ MoveBase เสร็จสิ้น Goal"""
        with self._lock:
            if not self.is_patrolling:
                return
            current_index = self.current_goal_index
            goal_count = len(self.goal_list)
            should_loop = self.should_loop
        
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal #{current_index + 1} reached.")
            with self._lock:
                self.current_goal_index += 1
                new_index = self.current_goal_index

            if new_index >= goal_count:
                if should_loop:
                    rospy.loginfo("Looping patrol...")
                    with self._lock:
                        self.current_goal_index = 0
                    delay = 2.0 if goal_count == 1 else 0.1
                    rospy.Timer(rospy.Duration(delay), lambda e: self.send_next_goal(), oneshot=True)
                else:
                    with self._lock:
                        self.is_patrolling = False
                    self.update_status("idle")
                    rospy.loginfo("Patrol finished.")
            else:
                rospy.Timer(rospy.Duration(0.1), lambda e: self.send_next_goal(), oneshot=True)
        elif status == actionlib.GoalStatus.PREEMPTED:
            with self._lock:
                paused = self.is_paused
            if paused:
                rospy.loginfo("Goal cancelled due to pause. Waiting for resume...")
            else:
                rospy.logwarn(f"Goal #{current_index + 1} was preempted externally. Retrying...")
                rospy.Timer(rospy.Duration(0.5), lambda e: self.send_next_goal(), oneshot=True)
        else:
            rospy.logerr(f"Goal #{current_index + 1} failed/aborted. Status: {status}")
            with self._lock:
                self.is_patrolling = False
            self.update_status("idle")

    # --- Home Management (Per Map) ---
    def _load_homes_data(self):
        """Helper อ่านไฟล์ JSON"""
        if os.path.exists(HOMES_FILE):
            try:
                with open(HOMES_FILE, 'r') as f:
                    return json.load(f)
            except Exception as e: 
                rospy.logwarn(f"⚠️ Failed to load homes data: {e}")
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
            homes_data = self._load_homes_data()
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
            homes_data[map_name] = pose_data
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
        if self.is_patrolling:
            return SaveMapResponse(False, "Patrol is running. Stop patrol before going home.")

        map_name = req.name
        homes_data = self._load_homes_data()

        if map_name not in homes_data:
            return SaveMapResponse(False, f"No home defined for map '{map_name}'")

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
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]

        self.initial_pose_pub.publish(msg)
        rospy.sleep(0.1)
        self.initial_pose_pub.publish(msg)
        
        return SaveMapResponse(True, f"Initial pose set to home of {map_name}")

    def cleanup(self):
        """เซฟ pose และหยุด Navigation เมื่อ node ถูก shutdown"""
        if not self.save_pose_to_file():
            rospy.logwarn("⚠️ Could not save final pose during shutdown.")

        class DummyReq:
            save_pose = False  # เซฟแล้วข้างบน ไม่ต้องเซฟซ้ำ
        self.handle_stop_nav(DummyReq())

if __name__ == '__main__':
    try:
        NavigationManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass