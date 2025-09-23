#!/usr/bin/env python3
import rospy
import os
import base64
import subprocess

from ptR1_navigation.srv import ListMaps, ListMapsResponse
from ptR1_navigation.srv import LoadMap, LoadMapResponse
from ptR1_navigation.srv import GetMapFile, GetMapFileResponse
from ptR1_navigation.srv import SaveMap, SaveMapResponse
from ptR1_navigation.srv import StartSLAM, StartSLAMResponse
from ptR1_navigation.srv import StopSLAM, StopSLAMResponse

slam_process = None  # ✅ เก็บ process ไว้สำหรับหยุด



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
    rospy.loginfo(f"🗺️ Loading map: {req.name}")
    name = req.name
    src_yaml = os.path.join(MAP_FOLDER, f"{name}.yaml")
    src_pgm = os.path.join(MAP_FOLDER, f"{name}.pgm")
    dest_yaml = os.path.join(MAP_FOLDER, f"{ACTIVE_MAP_NAME}.yaml")
    dest_pgm = os.path.join(MAP_FOLDER, f"{ACTIVE_MAP_NAME}.pgm")

    if not os.path.exists(src_yaml) or not os.path.exists(src_pgm):
        return LoadMapResponse(False, f"Map '{name}' not found")

    try:
        os.system(f"cp {src_yaml} {dest_yaml}")
        os.system(f"cp {src_pgm} {dest_pgm}")
        rospy.loginfo(f"🗺️ Loaded map: {name}")
        return LoadMapResponse(True, f"Map '{name}' loaded successfully")
    except Exception as e:
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
        
         # อ่านไฟล์ yaml เป็น string
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


def handle_save_map(req):
    name = req.name
    yaml_path = os.path.join(MAP_FOLDER, f"{name}.yaml")
    pgm_path = os.path.join(MAP_FOLDER, f"{name}.pgm")
    png_path = os.path.join(MAP_FOLDER, f"{name}.png")

    rospy.loginfo(f"💾 Saving map to {name}")


    try:
        
        # ใช้ rosrun map_server map_saver
        subprocess.check_call([
            'rosrun', 'map_server', 'map_saver',
            '-f', os.path.join(MAP_FOLDER, name)
        ])

        # แปลง .pgm → .png (ใช้ ImageMagick : `sudo apt install imagemagick`)
        subprocess.check_call([
            'convert', pgm_path, png_path
        ])
        

        return SaveMapResponse(
            success=True,
            #message=f"Map saved and converted to png_path"       
            message=f"Map saved and converted to {png_path}"
        )
    except subprocess.CalledProcessError as e:
        return SaveMapResponse(
            success=False,
            message=str(e)
        )

def handle_start_slam(req):
    global slam_process

    rospy.loginfo("🧭 Starting SLAM...")

    try:
        slam_process = subprocess.Popen([
            'roslaunch', 'your_slam_pkg', 'your_slam_launch_file.launch'
        ])
        return StartSLAMResponse(success=True, message="SLAM started")
    except Exception as e:
        return StartSLAMResponse(success=False, message=str(e))

def handle_stop_slam(req):
    global slam_process

    if slam_process is None:
        return StopSLAMResponse(success=False, message="No SLAM process running")

    try:
        slam_process.terminate()
        slam_process.wait(timeout=5)
        slam_process = None
        rospy.loginfo("🛑 SLAM stopped.")
        return StopSLAMResponse(success=True, message="SLAM process terminated")
    except Exception as e:
        return StopSLAMResponse(success=False, message=f"Failed to stop SLAM: {str(e)}")

# ----------------- [MAIN NODE] -----------------
def map_manager_server():
    rospy.init_node('map_manager')

    rospy.Service('/map_manager/list_maps', ListMaps, handle_list_maps)
    rospy.Service('/map_manager/load_map', LoadMap, handle_load_map)
    rospy.Service('/map_manager/get_map_file', GetMapFile, handle_get_map_file)
    rospy.Service('/map_manager/save_map', SaveMap, handle_save_map)
    rospy.Service('/map_manager/start_slam', StartSLAM, handle_start_slam)
    rospy.Service('/map_manager/stop_slam', StopSLAM, handle_stop_slam) 


    rospy.loginfo("✅ Map Manager Services Ready")
    rospy.spin()

if __name__ == '__main__':
    map_manager_server()
