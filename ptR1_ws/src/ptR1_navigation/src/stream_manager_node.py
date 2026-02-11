#!/usr/bin/env python3

import rospy
import subprocess
import time
import os
import signal
from std_srvs.srv import Trigger, TriggerResponse

# ตัวแปรสำหรับเก็บ FFmpeg process
ffmpeg_process = None

def cleanup():
    """ฟังก์ชันสำหรับเคลียร์ Process เมื่อปิด Node"""
    global ffmpeg_process
    if ffmpeg_process is not None and ffmpeg_process.poll() is None:
        rospy.logwarn("Shutting down ROS node, killing FFmpeg...")
        # ส่งสัญญาณ SIGINT (เหมือนกด Ctrl+C ให้ FFmpeg)
        os.kill(ffmpeg_process.pid, signal.SIGINT)
        # รอให้จบจริง
        try:
            ffmpeg_process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            ffmpeg_process.kill()

def handle_start_stream(req):
    global ffmpeg_process
    rospy.loginfo("Received request to start stream.")

    if ffmpeg_process is not None and ffmpeg_process.poll() is None:
        return TriggerResponse(success=False, message="Stream is already running.")

    # --- 🔧 ปรับปรุงคำสั่ง FFmpeg ---
    ffmpeg_command = [
        'ffmpeg',
        '-y', # Overwrite output files
        '-f', 'v4l2',
        '-input_format', 'mjpeg', # บังคับใช้ mjpeg เพื่อลด load usb
        '-framerate', '30',       # กำหนด fps ขาเข้า
        '-video_size', '640x480', # กำหนดขนาดภาพ
        '-i', '/dev/video0',
        '-c:v', 'libx264',
        '-preset', 'ultrafast',
        '-tune', 'zerolatency',
        '-b:v', '600k',           # จำกัด Bitrate ไม่ให้กิน Bandwidth เกิน
        '-f', 'rtsp',
        '-rtsp_transport', 'tcp', # สำคัญ! ใช้ TCP เพื่อความเสถียร
        'rtsp://localhost:8554/mystream'
    ]

    try:
        # redirect stdout/stderr เพื่อไม่ให้รก Terminal (หรือเอาออกถ้าอยากเห็น Log)
        # ffmpeg_process = subprocess.Popen(ffmpeg_command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # รันแบบปกติ (เห็น Log ของ FFmpeg ใน Terminal ROS)
        ffmpeg_process = subprocess.Popen(ffmpeg_command)

        # --- Safety Check: รอ 1 วินาทีเพื่อดูว่ารอดไหม ---
        time.sleep(1.0) 
        if ffmpeg_process.poll() is not None:
            # ถ้า poll() ไม่ใช่ None แปลว่า Process ตายไปแล้ว
            return TriggerResponse(success=False, message="FFmpeg started but crashed immediately (Check camera/RTSP server).")

        rospy.loginfo(f"Stream started with PID: {ffmpeg_process.pid}")
        return TriggerResponse(success=True, message="Stream started successfully.")

    except Exception as e:
        rospy.logerr(f"Failed to start: {e}")
        return TriggerResponse(success=False, message=str(e))

def handle_stop_stream(req):
    global ffmpeg_process
    rospy.loginfo("Request to stop stream.")

    if ffmpeg_process is None or ffmpeg_process.poll() is not None:
        return TriggerResponse(success=False, message="Stream is not running.")

    try:
        # ใช้ os.kill เพื่อส่ง SIGINT (นุ่มนวลกว่า terminate สำหรับ ffmpeg)
        os.kill(ffmpeg_process.pid, signal.SIGINT)
        ffmpeg_process.wait(timeout=3)
        ffmpeg_process = None
        return TriggerResponse(success=True, message="Stream stopped.")
    except subprocess.TimeoutExpired:
        rospy.logwarn("FFmpeg stuck, forcing kill.")
        ffmpeg_process.kill()
        ffmpeg_process = None
        return TriggerResponse(success=True, message="Stream killed forcefully.")
    except Exception as e:
        return TriggerResponse(success=False, message=str(e))

def stream_manager_server():
    rospy.init_node('stream_manager_server')
    
    # ลงทะเบียนฟังก์ชัน cleanup ให้ทำงานตอนปิด Node
    rospy.on_shutdown(cleanup)

    rospy.Service('/stream_manager/start', Trigger, handle_start_stream)
    rospy.Service('/stream_manager/stop', Trigger, handle_stop_stream)
    
    rospy.loginfo("Stream Manager Ready (Robust Mode)")
    rospy.spin()

if __name__ == "__main__":
    stream_manager_server()