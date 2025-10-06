#!/usr/bin/env python3
import rospy
import os
import subprocess
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

# Import services ของคุณ
from ptR1_navigation.srv import (ListMaps, ListMapsResponse, LoadMap, LoadMapResponse,
                                 GetMapFile, GetMapFileResponse, SaveMap, SaveMapResponse,
                                 StartSLAM, StartSLAMResponse, StopSLAM, StopSLAMResponse,
                                 DeleteMap, DeleteMapResponse, ResetSLAM, ResetSLAMResponse,
                                 StartPatrol, StartPatrolResponse, PausePatrol, PausePatrolResponse,
                                 ResumePatrol, ResumePatrolResponse, StopPatrol, StopPatrolResponse,
                                 ClearCostmaps)

MAP_FOLDER = os.path.expanduser('~/ptR1_ws/src/ptR1_navigation/maps')

class MapManager:
    def __init__(self):
        rospy.init_node('map_manager_pro')
        rospy.loginfo("Starting Map Manager Pro Node...")

        # --- State Variables ---
        self.running_processes = []
        self.navigation_process = None
        
        # Patrol State
        self.goal_list = []
        self.current_goal_index = 0
        self.is_patrolling = False
        self.is_paused = False
        self.should_loop = False

        # --- Action Client ---
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server(rospy.Duration(10.0))
        rospy.loginfo("move_base action server connected.")

        # --- Services ---
        # Map/SLAM Management
        rospy.Service('/map_manager/list_maps', ListMaps, self.handle_list_maps)
        rospy.Service('/map_manager/load_map', LoadMap, self.handle_load_map)
        rospy.Service('/map_manager/save_map', SaveMap, self.handle_save_map)
        rospy.Service('/map_manager/delete_map', DeleteMap, self.handle_delete_map)
        rospy.Service('/map_manager/start_slam', StartSLAM, self.handle_start_slam)
        rospy.Service('/map_manager/stop_processes', StopSLAM, self.handle_stop_processes) # ใช้ชื่อใหม่ที่สื่อกว่า
        rospy.Service('/map_manager/reset_slam', ResetSLAM, self.handle_reset_slam)
        
        # Patrol Management
        rospy.Service('/map_manager/start_patrol', StartPatrol, self.handle_start_patrol)
        rospy.Service('/map_manager/pause_patrol', PausePatrol, self.handle_pause_patrol)
        rospy.Service('/map_manager/resume_patrol', ResumePatrol, self.handle_resume_patrol)
        rospy.Service('/map_manager/stop_patrol', StopPatrol, self.handle_stop_patrol)

        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("All Map Manager services are ready.")

# ------ Patrol Logic ------
    def send_next_goal(self):
        if not self.is_patrolling or self.is_paused or not self.goal_list:
            return

        if self.current_goal_index < len(self.goal_list):
            goal_pose = self.goal_list[self.current_goal_index]
            goal = MoveBaseGoal(target_pose=goal_pose)
            
            rospy.loginfo(f"Sending goal #{self.current_goal_index + 1}/{len(self.goal_list)}")
            self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)
        else:
            rospy.loginfo("Patrol sequence finished.")
            self.is_patrolling = False

    def goal_done_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal #{self.current_goal_index + 1} reached successfully.")
            self.current_goal_index += 1

            if self.current_goal_index >= len(self.goal_list) and self.should_loop:
                rospy.loginfo("Looping patrol, starting from first goal.")
                self.current_goal_index = 0
            
            self.send_next_goal()
        else:
            rospy.logerr(f"Failed to reach goal #{self.current_goal_index + 1}. Stopping patrol. Status: {status}")
            self.is_patrolling = False

# ------ Service Handlers for Patrol ------
    def handle_start_patrol(self, req):
        if not req.goals:
            return StartPatrolResponse(False, "Goal list cannot be empty.")
        
        self.handle_stop_patrol(None) # Stop previous patrol first
        
        self.goal_list = req.goals
        self.should_loop = req.loop
        self.current_goal_index = 0
        self.is_patrolling = True
        self.is_paused = False
        
        rospy.loginfo(f"Starting patrol with {len(self.goal_list)} goals. Loop: {self.should_loop}")
        self.send_next_goal()
        return StartPatrolResponse(True, "Patrol started.")

    def handle_pause_patrol(self, req):
        """Cancels the current goal and pauses the patrol sequence."""
        if not self.is_patrolling:
            return PausePatrolResponse(False, "Not currently patrolling.")
            
        self.is_paused = True
        self.move_base_client.cancel_goal()
        rospy.loginfo("Patrol paused.")
        return PausePatrolResponse(True, "Patrol paused.")

    def handle_resume_patrol(self, req):
        if not self.is_patrolling:
            return ResumePatrolResponse(False, "Not currently patrolling.")
        if not self.is_paused:
            return ResumePatrolResponse(False, "Patrol is not paused.")

        self.is_paused = False
        rospy.loginfo("Resuming patrol.")
        self.send_next_goal() # Send the goal that was paused
        return ResumePatrolResponse(True, "Patrol resumed.")

    def handle_stop_patrol(self, req):
        self.is_patrolling = False
        self.is_paused = False
        self.goal_list = []
        self.current_goal_index = 0
        self.move_base_client.cancel_all_goals()
        if req is not None:
             rospy.loginfo("Patrol stopped by request.")
        return StopPatrolResponse(True, "Patrol stopped.")

# ------ Map/SLAM Handlers------
    def handle_list_maps(self, req):
        rospy.loginfo("Listing maps...")
        try:
            names = [os.path.splitext(f)[0] for f in os.listdir(MAP_FOLDER) if f.endswith(".yaml")]
            return ListMapsResponse(names)
        except Exception as e:
            rospy.logerr(f"Could not list maps: {e}")
            return ListMapsResponse([])

    def handle_load_map(self, req):
        map_to_load = req.name
        rospy.loginfo(f"🗺️ Loading map '{map_to_load}' and starting navigation nodes...")

        if self.navigation_process:
            rospy.loginfo("Stopping existing navigation process...")
            self.navigation_process.terminate()
            self.navigation_process.wait()
            if self.navigation_process in self.running_processes:
                self.running_processes.remove(self.navigation_process)
            self.navigation_process = None

        map_yaml_path = os.path.join(MAP_FOLDER, f"{map_to_load}.yaml")
        if not os.path.exists(map_yaml_path):
            message = f"Map '{map_to_load}' not found."
            rospy.logerr(message)
            return LoadMapResponse(False, message)

        try:
            command = ['roslaunch', 'ptR1_navigation', 'navigation_2.launch', f'map_name:={map_to_load}']
            rospy.loginfo(f"🚀 Executing: {' '.join(command)}")
            self.navigation_process = subprocess.Popen(command)
            self.running_processes.append(self.navigation_process)
            message = f"Navigation started with map '{map_to_load}'."
            rospy.loginfo(message)
            return LoadMapResponse(True, message)
        except Exception as e:
            message = f"Error starting navigation for map '{map_to_load}': {str(e)}"
            rospy.logerr(message)
            return LoadMapResponse(False, message)

    def handle_start_slam(self, req):
        rospy.loginfo("Starting SLAM using ptR1_navigation/slam.launch...")
        self.handle_stop_processes(None) 
        try:
            slam_launch_command = ['roslaunch', 'ptR1_navigation', 'slam.launch']
            process = subprocess.Popen(slam_launch_command)
            self.running_processes.append(process)
            rospy.loginfo("Started SLAM process from launch file.")
            return StartSLAMResponse(True, "SLAM started successfully via launch file.")
        except Exception as e:
            rospy.logerr(f"Failed to start SLAM launch file: {e}")
            self.handle_stop_processes(None)
            return StartSLAMResponse(False, str(e))

    def handle_stop_processes(self, req):
        if not self.running_processes:
            if req is not None: rospy.loginfo("No managed processes were running.")
            return StopSLAMResponse(True, "No managed processes were running.")
        rospy.loginfo(f"Stopping {len(self.running_processes)} managed processes...")
        try:
            # ใช้ list copy [:] เพื่อให้สามารถลบสมาชิกจาก list เดิมได้โดยไม่สับสน
            for process in self.running_processes[:]:
                process.terminate()
                process.wait(timeout=5)
                self.running_processes.remove(process)
            rospy.loginfo("All managed processes terminated.")
            # รีเซ็ต navigation_process ด้วย
            self.navigation_process = None
            return StopSLAMResponse(True, "All managed processes terminated successfully.")
        except subprocess.TimeoutExpired:
            for process in self.running_processes[:]:
                 process.kill()
                 self.running_processes.remove(process)
            self.navigation_process = None
            return StopSLAMResponse(False, "Timeout expired, processes were killed.")
        except Exception as e:
            return StopSLAMResponse(False, f"Failed to stop processes: {str(e)}")

    def handle_reset_slam(self, req):
        rospy.loginfo("🔄 Received request to reset SLAM.")
        slam_reset_service_name = '/slam_toolbox/reset'
        try:
            rospy.wait_for_service(slam_reset_service_name, timeout=2.0)
            reset_slam_service = rospy.ServiceProxy(slam_reset_service_name, Empty)
            reset_slam_service()
            return ResetSLAMResponse(True, "SLAM has been successfully reset.")
        except Exception as e:
            return ResetSLAMResponse(False, f"Failed to reset SLAM: {e}")

    def handle_clear_costmaps(self, req):
        rospy.loginfo("Received request to clear costmaps.")
        service_name = '/move_base/clear_costmaps'
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            clear_costmaps_service = rospy.ServiceProxy(service_name, Empty)
            clear_costmaps_service()
            return ClearCostmaps(True, "Costmaps cleared successfully.")
        except Exception as e:
            return ClearCostmaps(False, f"Failed to clear costmaps: {e}")

    def handle_save_map(self, req):
        name = req.name
        rospy.loginfo(f"Saving map to {name}")
        try:
            map_filepath = os.path.join(MAP_FOLDER, name)
            subprocess.check_call(['rosrun', 'map_server', 'map_saver', '-f', map_filepath, 'map:=/rb/slam/map'])
            # Convert pgm to png
            subprocess.check_call(['convert', f"{map_filepath}.pgm", f"{map_filepath}.png"])
            return SaveMapResponse(True, f"Map saved as {name}.pgm and {name}.png")
        except Exception as e:
            return SaveMapResponse(False, str(e))

    def handle_delete_map(self, req):
        map_name = req.name
        rospy.loginfo(f"🗑️ Received request to delete map: {map_name}")
        files_to_check = [os.path.join(MAP_FOLDER, f"{map_name}{ext}") for ext in ['.yaml', '.pgm', '.png']]
        deleted_count = 0
        try:
            for file_path in files_to_check:
                if os.path.exists(file_path):
                    os.remove(file_path)
                    deleted_count += 1
            if deleted_count > 0:
                return DeleteMapResponse(True, f"Map '{map_name}' and associated files deleted.")
            else:
                return DeleteMapResponse(True, f"Map '{map_name}' not found.")
        except OSError as e:
            return DeleteMapResponse(False, f"Error deleting map '{map_name}': {str(e)}")
    
    def handle_get_map_file(self,req):
        rospy.loginfo(f"Getting map file: {req.name}")
        map_name = req.name
        image_path = os.path.join(MAP_FOLDER, f"{map_name}.png")
        yaml_path = os.path.join(MAP_FOLDER, f"{map_name}.yaml")

        if not os.path.exists(image_path):
            rospy.logwarn(f"[get_map_file] Not found: {image_path}")
            return GetMapFileResponse(
                success=False,
                message=f"Map image {map_name}.png not found",
                image_data_base64="",
                yaml_data=""
            )

        try:
            with open(image_path, 'rb') as f:
                encoded_image = base64.b64encode(f.read()).decode('utf-8')
            
            with open(yaml_path, 'r') as f:
                yaml_content = f.read()

            return GetMapFileResponse(
                success=True,
                message=f"{map_name}.png loaded",
                image_data_base64=encoded_image,
                yaml_data=yaml_content
            )
        except Exception as e:
            return GetMapFileResponse(
                success=False,
                message=str(e),
                image_data_base64="",
                yaml_data=""
            )
                
    def shutdown_hook(self):
        rospy.loginfo("Shutdown request received...")
        self.handle_stop_patrol(None)
        self.handle_stop_processes(None)
        rospy.loginfo("Goodbye!")            
        

if __name__ == '__main__':
    try:
        MapManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass