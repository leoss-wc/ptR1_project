#!/usr/bin/env python3

import rospy
import subprocess
from std_srvs.srv import Trigger, TriggerResponse

# ตัวแปรสำหรับเก็บ FFmpeg process ที่กำลังทำงานอยู่
ffmpeg_process = None

def handle_start_stream(req):
    """
    Service handler สำหรับเริ่มการสตรีม
    """
    global ffmpeg_process
    rospy.loginfo("Received request to start FFmpeg stream.")

    # ตรวจสอบว่ามี process ทำงานอยู่แล้วหรือไม่
    if ffmpeg_process is not None and ffmpeg_process.poll() is None:
        rospy.logwarn("FFmpeg stream is already running.")
        return TriggerResponse(success=False, message="Stream is already running.")

    # --- 🔧 แก้ไขคำสั่ง FFmpeg ของคุณที่นี่ ---
    # ตัวอย่าง: สตรีมจากกล้อง USB (/dev/video0) ไปยัง MediaMTX
    ffmpeg_command = [
        'ffmpeg',
        '-f', 'v4l2',      # Source format
        '-i', '/dev/video0', # Source device (แก้เป็น path กล้องของคุณ)
        '-c:v', 'libx264', # Video codec
        '-preset', 'ultrafast',
        '-tune', 'zerolatency',
        '-f', 'rtsp',      # Output format
        'rtsp://localhost:8554/mystream' # Destination MediaMTX (แก้ path ตามของคุณ)
    ]
    
    try:
        rospy.loginfo(f"Executing command: {' '.join(ffmpeg_command)}")
        # รัน FFmpeg เป็น background process
        ffmpeg_process = subprocess.Popen(ffmpeg_command)
        return TriggerResponse(success=True, message="FFmpeg stream started successfully.")
    except Exception as e:
        rospy.logerr(f"Failed to start FFmpeg: {e}")
        return TriggerResponse(success=False, message=str(e))

def handle_stop_stream(req):
    """
    Service handler สำหรับหยุดการสตรีม
    """
    global ffmpeg_process
    rospy.loginfo("Received request to stop FFmpeg stream.")

    if ffmpeg_process is None or ffmpeg_process.poll() is not None:
        rospy.logwarn("FFmpeg stream is not running.")
        return TriggerResponse(success=False, message="Stream is not running.")

    try:
        rospy.loginfo(f"Terminating FFmpeg process with PID: {ffmpeg_process.pid}")
        ffmpeg_process.terminate() # ส่งสัญญาณให้ process หยุดทำงาน
        ffmpeg_process.wait(timeout=5) # รอ 5 วินาที
        ffmpeg_process = None
        return TriggerResponse(success=True, message="FFmpeg stream stopped successfully.")
    except subprocess.TimeoutExpired:
        rospy.logerr("FFmpeg process did not terminate in time, killing.")
        ffmpeg_process.kill()
        ffmpeg_process = None
        return TriggerResponse(success=False, message="Stream was forcefully killed.")
    except Exception as e:
        rospy.logerr(f"Failed to stop FFmpeg: {e}")
        return TriggerResponse(success=False, message=str(e))

def stream_manager_server():
    rospy.init_node('stream_manager_server')
    
    # สร้าง Service ชื่อ /stream_manager/start
    rospy.Service('/stream_manager/start', Trigger, handle_start_stream)
    
    # สร้าง Service ชื่อ /stream_manager/stop
    rospy.Service('/stream_manager/stop', Trigger, handle_stop_stream)
    
    rospy.loginfo("🚀 Stream Manager is ready.")
    rospy.spin()

if __name__ == "__main__":
    stream_manager_server()