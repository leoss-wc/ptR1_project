#!/usr/bin/env python3
import rospy
import os
import subprocess
import actionlib
import base64
import signal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
import json
from geometry_msgs.msg import PoseWithCovarianceStamped #สำหรับรับ/ส่ง pose
import math
import cv2
import numpy as np
import shutil # ไว้ copy file
# Import services
from ptR1_navigation.srv import (ListMaps, ListMapsResponse, SelectNavMap, SelectNavMapResponse,
                                 GetMapFile, GetMapFileResponse, SaveMap, SaveMapResponse,
                                 StartSLAM, StartSLAMResponse, StopSLAM, StopSLAMResponse,
                                 DeleteMap, DeleteMapResponse, ResetSLAM, ResetSLAMResponse,                                            
                                 ClearCostmaps, ClearCostmapsResponse,SaveEditedMap, SaveEditedMapResponse)

MAP_FOLDER = os.path.expanduser('~/ptR1_ws/src/ptR1_navigation/maps')
#MAP_FOLDER = os.path.expanduser('~/ptR1Project/ptR1_ws/src/ptR1_navigation/maps')


class MapManager:
    def __init__(self):
        rospy.init_node('map_manager_pro')
        rospy.loginfo("Starting Map Manager Pro Node...")

        if not os.path.exists(MAP_FOLDER):
            os.makedirs(MAP_FOLDER)
            rospy.loginfo(f"Created map folder at {MAP_FOLDER}")

        # --- State Variables ---
        self.running_processes = []
        self.navigation_process = None
        self.is_saving = False

        # --- Action Client ---
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.current_map_pub = rospy.Publisher('/map_manager/current_map_name', String, queue_size=1, latch=True)
        # --- Services ---
        # Map/SLAM Management
        rospy.Service('/map_manager/list_maps', ListMaps, self.handle_list_maps)
        rospy.Service('/map_manager/select_nav_map', SelectNavMap, self.handle_select_nav_map)
        rospy.Service('/map_manager/save_map', SaveMap, self.handle_save_map)
        rospy.Service('/map_manager/delete_map', DeleteMap, self.handle_delete_map)
        rospy.Service('/map_manager/start_slam', StartSLAM, self.handle_start_slam)
        rospy.Service('/map_manager/stop_processes', StopSLAM, self.handle_stop_processes)
        rospy.Service('/map_manager/reset_slam', ResetSLAM, self.handle_reset_slam)
        rospy.Service('/map_manager/get_map_file', GetMapFile, self.handle_get_map_file)
        rospy.Service('/map_manager/clear_costmaps', ClearCostmaps, self.handle_clear_costmaps)
        rospy.Service('/map_manager/save_edited_map', SaveEditedMap, self.handle_save_edited_map)
        
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("All Map Manager services are ready.")

    def handle_save_edited_map(self, req):
        try:
            map_name = req.map_name
            base64_str = req.base64_image
            yaml_content = req.yaml_content #รับเนื้อหา YAML มาจาก JS
            
            maps_dir = MAP_FOLDER
            # Save PNG & PGM (Logic เดิม)
            if "," in base64_str:
                base64_str = base64_str.split(",")[1]
            img_data = base64.b64decode(base64_str)
            np_arr = np.frombuffer(img_data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            
            # Save PNG
            cv2.imwrite(os.path.join(maps_dir, f"{map_name}.png"), image)
            
            # Save PGM (Convert to Gray)
            if len(image.shape) > 2:
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray_image = image
            cv2.imwrite(os.path.join(maps_dir, f"{map_name}.pgm"), gray_image)

            # Save YAML (เขียนไฟล์ใหม่จาก Content ที่ส่งมาเลย)
            yaml_path = os.path.join(maps_dir, f"{map_name}.yaml")
            with open(yaml_path, 'w') as f:
                f.write(yaml_content)

            return {"success": True, "message": f"Saved {map_name} (PGM, PNG, YAML)"}

        except Exception as e:
            return {"success": False, "message": str(e)}

# ------ Map/SLAM Handlers------
    def handle_list_maps(self, req):
        rospy.loginfo("Listing maps...")
        try:
            names = [os.path.splitext(f)[0] for f in os.listdir(MAP_FOLDER) if f.endswith(".yaml")]
            rospy.loginfo(f"Found {len(names)} maps: {names}")
            return ListMapsResponse(names)
        except Exception as e:
            rospy.logerr(f"Could not list maps: {e}")
            return ListMapsResponse([])

    def handle_select_nav_map(self, req):
        map_to_load = req.name
        rospy.loginfo(f"Loading map '{map_to_load}' and starting navigation nodes...")

        # 1. เช็คไฟล์ก่อนเลย ถ้าไม่มีจะได้ไม่ต้องไปสั่งหยุด process ให้เสียเวลา
        map_yaml_path = os.path.join(MAP_FOLDER, f"{map_to_load}.yaml")
        if not os.path.exists(map_yaml_path):
            return SelectNavMapResponse(False, f"Map '{map_to_load}' not found.")

        # 2. เคลียร์ process เก่า (Nav หรือ SLAM)
        self.handle_stop_processes(None)

        try:
            command = ['roslaunch', 'ptR1_navigation', 'active_map_server.launch', f'map_name:={map_to_load}']
            rospy.loginfo(f"Executing: {' '.join(command)}")
            self.navigation_process = subprocess.Popen(command)
            self.running_processes.append(self.navigation_process)
            rospy.Timer(rospy.Duration(3.0), 
                lambda e: self.current_map_pub.publish(map_to_load), 
                oneshot=True)

            rospy.loginfo(f"Navigation started with map '{map_to_load}'.")
            return SelectNavMapResponse(True, f"Navigation started with map '{map_to_load}'.")
        except Exception as e:
            return SelectNavMapResponse(False, f"Error starting navigation: {str(e)}")
        
    def handle_stop_processes(self, req):
        if not self.running_processes:
            if req is not None: rospy.loginfo("No managed processes were running.")
            return StopSLAMResponse(True, "No managed processes were running.")
        rospy.loginfo(f"Stopping {len(self.running_processes)} managed processes...")
        try:
            for process in self.running_processes[:]:
                if process.poll() is None: # ถ้ายังไม่ตาย
                    process.terminate() # ส่ง SIGTERM
                    try:
                        process.wait(timeout=3) # รอ 3 วิ
                    except subprocess.TimeoutExpired:
                        rospy.logwarn("Process did not stop, killing it.")
                        process.kill() # ส่ง SIGKILL
                self.running_processes.remove(process)
            
            self.navigation_process = None
            return StopSLAMResponse(True, "All processes stopped.")
        except Exception as e:
            return StopSLAMResponse(False, f"Failed to stop processes: {str(e)}")
        
    def handle_start_slam(self, req):
        rospy.loginfo("Starting SLAM...")
        self.handle_stop_processes(None) 
        try:
            slam_launch_command = ['roslaunch', 'ptR1_navigation', 'slam.launch']
            process = subprocess.Popen(slam_launch_command)
            self.running_processes.append(process)
            return StartSLAMResponse(True, "SLAM started successfully.")
        except Exception as e:
            self.handle_stop_processes(None)
            return StartSLAMResponse(False, str(e))

    def handle_reset_slam(self, req):
        rospy.loginfo("Resetting SLAM.")
        slam_reset_service_name = '/slam_toolbox/reset'
        try:
            # รอ Service นานหน่อยเผื่อ SLAM เพิ่งเริ่ม (5 วินาที)
            rospy.wait_for_service(slam_reset_service_name, timeout=5.0) 
            reset_slam_service = rospy.ServiceProxy(slam_reset_service_name, Empty)
            reset_slam_service()
            return ResetSLAMResponse(True, "SLAM reset successful.")
        except rospy.ROSException:
             return ResetSLAMResponse(False, "SLAM service not available (Timeout).")
        except Exception as e:
            return ResetSLAMResponse(False, f"Failed to reset SLAM: {e}")
        
    def handle_clear_costmaps(self, req):
        rospy.loginfo("Received request to clear costmaps.")
        service_name = '/move_base/clear_costmaps'
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            clear_costmaps_service = rospy.ServiceProxy(service_name, Empty)
            clear_costmaps_service()
            return ClearCostmapsResponse(True, "Costmaps cleared successfully.")
        except Exception as e:
            return ClearCostmapsResponse(False, f"Failed to clear costmaps: {e}")

    def handle_save_map(self, req):
        if self.is_saving:
            msg = "Save operation already in progress. Request ignored."
            rospy.logwarn(msg)
            return SaveMapResponse(False, msg)
        self.is_saving = True
        name = req.name
        rospy.loginfo(f"Saving map to {name}")
        try:
            map_filepath = os.path.join(MAP_FOLDER, name)

            subprocess.check_call(['rosrun', 'map_server', 'map_saver', '-f', map_filepath, 'map:=/map'])

            if os.path.exists(f"{map_filepath}.pgm"):
                img = cv2.imread(f"{map_filepath}.pgm", cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    cv2.imwrite(f"{map_filepath}.png", img)
                    return SaveMapResponse(True, f"Map saved as {name}.pgm and {name}.png")
                else:
                    return SaveMapResponse(False, "Failed to convert PGM to PNG.")
            else:
                return SaveMapResponse(False, "Map saver failed to create PGM file.")

        except Exception as e:
            rospy.logerr(f"Save map error: {e}")
            return SaveMapResponse(False, str(e))
        finally:
            self.is_saving = False  # ✅ คืนค่าเสมอไม่ว่าจะ return ทางไหน

        
              
    def handle_delete_map(self, req):
        map_name = req.name
        rospy.loginfo(f"Received request to delete map: {map_name}")
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

        if not os.path.exists(image_path) or not os.path.exists(yaml_path):
            return GetMapFileResponse(
                success=False,
                message=f"Map files for {map_name} not found",
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
                message=f"Loaded {map_name}",
                image_data_base64=encoded_image,
                yaml_data=yaml_content
            )
        except Exception as e:
            return GetMapFileResponse(False, str(e), "", "")


    def shutdown_hook(self):
        rospy.loginfo("Shutdown request received...")
        self.handle_stop_processes(None) 
        rospy.loginfo("Map Manager shutdown complete.")        
        

if __name__ == '__main__':
    try:
        MapManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass