#!/usr/bin/env python3
import rospy
import os
import base64
from ptR1_navigation.srv import GetMapFile, GetMapFileResponse

MAP_FOLDER = os.path.expanduser('~/ptR1_ws/src/ptR1_navigation/maps')

def handle_get_map_file(req):
    rospy.loginfo(f" 🗺️ Getting map file: {req.name}")
    map_name = req.name
    image_path = os.path.join(MAP_FOLDER, f"{map_name}.png")

    if not os.path.exists(image_path):
        rospy.logwarn(f"[get_map_file] Not found: {image_path}")
        return GetMapFileResponse(
            success=False,
            message=f"Map image {map_name}.png not found",
            image_data_base64=""
        )

    try:
        with open(image_path, 'rb') as f:
            encoded = base64.b64encode(f.read()).decode('utf-8')
        return GetMapFileResponse(
            success=True,
            message=f"{map_name}.png loaded",
            image_data_base64=encoded
        )
    except Exception as e:
        return GetMapFileResponse(
            success=False,
            message=str(e),
            image_data_base64=""
        )

def get_map_file_server():
    rospy.init_node('get_map_file_server')
    rospy.Service('/map_manager/get_map_file', GetMapFile, handle_get_map_file)
    rospy.loginfo(" 🗺️ get_map_file_server ready")
    rospy.spin()

if __name__ == '__main__':
    get_map_file_server()
