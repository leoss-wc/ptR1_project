#!/usr/bin/env python3
import rospy
import os
import base64
import subprocess
import yaml
import shutil
import roslaunch

from ptR1_navigation.srv import ListMaps, ListMapsResponse
from ptR1_navigation.srv import LoadMap, LoadMapResponse
from ptR1_navigation.srv import GetMapFile, GetMapFileResponse
from ptR1_navigation.srv import SaveMap, SaveMapResponse
from ptR1_navigation.srv import StartSLAM, StartSLAMResponse
from ptR1_navigation.srv import StopSLAM, StopSLAMResponse

#รวมการจัดการ Process ไว้ที่นี่ที่เดียว
running_processes = []
map_server_process = None #เก็บ Popen object ของ map_server แยกไว้เพื่อ stop/start ได้ง่าย 
slam_launch_process = None

MAP_FOLDER = os.path.expanduser('~/ptR1Project/ptR1_ws/src/ptR1_navigation/maps')
ACTIVE_MAP_NAME = "active_map"


# ----------------- [LIST MAPS] -----------------
def handle_list_maps(req):
    rospy.loginfo("🗺️ Listing maps...")
    names = []
    for file in os.listdir(MAP_FOLDER):
        if file.endswith(".yaml"):
            name = os.path.splitext(file)[0]
            if name != ACTIVE_MAP_NAME:
                names.append(name)
    return ListMapsResponse(names)

# ----------------- [LOAD MAP] ------------------
def handle_load_map(req):
    global map_server_process
    rospy.loginfo(f"🗺️ Loading map: {req.name}")
    name = req.name
    src_yaml = os.path.join(MAP_FOLDER, f"{name}.yaml")
    src_pgm = os.path.join(MAP_FOLDER, f"{name}.pgm")
    dest_yaml = os.path.join(MAP_FOLDER, f"{ACTIVE_MAP_NAME}.yaml")
    dest_pgm = os.path.join(MAP_FOLDER, f"{ACTIVE_MAP_NAME}.pgm")

    if not os.path.exists(src_yaml) or not os.path.exists(src_pgm):
        rospy.logerr(f"Map '{name}' not found at specified path.")
        return LoadMapResponse(False, f"Map '{name}' not found")

    try:
        shutil.copy(src_yaml, dest_yaml)
        shutil.copy(src_pgm, dest_pgm)
        rospy.loginfo(f"Copied '{name}' to '{ACTIVE_MAP_NAME}'")

        with open(dest_yaml, 'r') as file:
            map_data = yaml.safe_load(file)

        new_image_name = f"{ACTIVE_MAP_NAME}.pgm"
        map_data['image'] = new_image_name
        rospy.loginfo(f"Updating image path in YAML to: {new_image_name}")

        with open(dest_yaml, 'w') as file:
            yaml.dump(map_data, file, default_flow_style=False)

        # ✨ แก้ไข: หยุด map_server ตัวเก่าและลบออกจาก List
        if map_server_process in running_processes:
            running_processes.remove(map_server_process)
        if map_server_process:
            rospy.loginfo("Stopping existing map_server process...")
            map_server_process.terminate()
            map_server_process.wait()
        
        map_server_command = [
            'rosrun', 'map_server', 'map_server', dest_yaml
        ]
        rospy.loginfo(f"🚀 Starting new map_server process for {ACTIVE_MAP_NAME}.yaml")
        map_server_process = subprocess.Popen(map_server_command)
        running_processes.append(map_server_process) 
        
        rospy.loginfo(f"🗺️ Loaded map: {name} and published to /map topic.")
        return LoadMapResponse(True, f"Map '{name}' loaded successfully")
        
    except Exception as e:
        rospy.logerr(f"Error loading map '{name}': {str(e)}")
        return LoadMapResponse(False, str(e))

# -------------- [GET MAP FILE] -----------------
def handle_get_map_file(req):
    rospy.loginfo(f" 🗺️ Getting map file: {req.name}")
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

# ----------------- [SAVE MAP] -----------------
def handle_save_map(req):
    name = req.name
    pgm_path = os.path.join(MAP_FOLDER, f"{name}.pgm")
    png_path = os.path.join(MAP_FOLDER, f"{name}.png")

    rospy.loginfo(f"💾 Saving map to {name}")

    try:
        subprocess.check_call([
            'rosrun', 'map_server', 'map_saver',
            '-f', os.path.join(MAP_FOLDER, name),
            'map:=/rb/slam/map'
        ])
        subprocess.check_call([
            'convert', pgm_path, png_path
        ])
        
        return SaveMapResponse(
            success=True,     
            message=f"Map saved and converted to {png_path}"
        )
    except subprocess.CalledProcessError as e:
        return SaveMapResponse(
            success=False,
            message=str(e)
        )

# ----------------- [START SLAM] -----------------
def handle_start_slam(req):
    global running_processes

    handle_stop_slam(None) 
    
    rospy.loginfo("🧭 Starting SLAM nodes directly from Python script...")

    try:
        params_to_set = {
            '/slam_toolbox/odom_frame': 'odom',
            '/slam_toolbox/map_frame': 'map',
            '/slam_toolbox/base_frame': 'base_link',
            '/slam_toolbox/scan_topic': 'scan',
            '/slam_toolbox/use_scan_matching': 'true',
            '/slam_toolbox/max_laser_range': '10.0'
        }
        for param, value in params_to_set.items():
            subprocess.run(['rosparam', 'set', param, value], check=True)
        
        rospy.loginfo(f"Set {len(params_to_set)} parameters for slam_toolbox.")

        slam_command = [
            'rosrun', 
            'slam_toolbox',
            'async_slam_toolbox_node',
            'scan:=/scan',
            'map:=/rb/slam/map'
        ]

        process = subprocess.Popen(slam_command)
        running_processes.append(process)
        rospy.loginfo("Started SLAM node (slam_toolbox).")
        return StartSLAMResponse(success=True, message="SLAM nodes started successfully.")

    except Exception as e:
        rospy.logerr(f"Failed to start SLAM nodes: {e}")
        handle_stop_slam(None)
        return StartSLAMResponse(success=False, message=str(e))

# ----------------- [STOP SLAM] -----------------
def handle_stop_slam(req):
    global running_processes

    if not running_processes:
        if req is not None:
             rospy.loginfo("No managed processes were running.")
        return StopSLAMResponse(success=True, message="No managed processes were running.")

    rospy.loginfo(f"Stopping {len(running_processes)} managed processes...")
    
    try:
        for process in running_processes:
            process.terminate()
            process.wait(timeout=5)
        
        running_processes = []
        rospy.loginfo("All managed processes terminated.")
        return StopSLAMResponse(success=True, message="All managed processes terminated successfully.")
    
    except subprocess.TimeoutExpired:
        rospy.logerr("Timeout expired while waiting for a process to terminate. Forcing kill.")
        for process in running_processes:
            process.kill()
        running_processes = []
        return StopSLAMResponse(success=False, message="Timeout expired, processes were killed.")

    except Exception as e:
        rospy.logerr(f"Failed to stop SLAM processes: {str(e)}")
        return StopSLAMResponse(success=False, message=f"Failed to stop SLAM: {str(e)}")

# -------------- [SHUTDOWN HOOK] -----------------
def shutdown_hook():
    rospy.loginfo("Shutdown request received...")
    if running_processes:
        handle_stop_slam(None)
    rospy.loginfo("Goodbye!")

# ----------------- [MAIN NODE] -----------------
def map_manager_server():
    rospy.init_node('map_manager')

    rospy.Service('/map_manager/list_maps', ListMaps, handle_list_maps)
    rospy.Service('/map_manager/load_map', LoadMap, handle_load_map)
    rospy.Service('/map_manager/get_map_file', GetMapFile, handle_get_map_file)
    rospy.Service('/map_manager/save_map', SaveMap, handle_save_map)
    rospy.Service('/map_manager/start_slam', StartSLAM, handle_start_slam)
    rospy.Service('/map_manager/stop_slam', StopSLAM, handle_stop_slam) 

    rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("Map Manager Services Ready")
    rospy.spin()

if __name__ == '__main__':
    map_manager_server()