#!/usr/bin/env python3

import rospy
import subprocess
import time
import os
import signal
import cv2
import psutil
from ultralytics import YOLO
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float32

# --- Global variables ---
ffmpeg_process = None
mediamtx_process = None
is_stream_enabled = False
is_starting = False 

# --- YOLO Settings ---
model = None
cap = None

def start_mediamtx():
    global mediamtx_process
    mediamtx_exec = rospy.get_param('~mediamtx_exec', '/home/patrolR1/MediaMtx/mediamtx')
    mediamtx_config = rospy.get_param('~mediamtx_config', '/home/patrolR1/MediaMtx/mediamtx.yml')
    
    # ตรวจสอบว่ามี process รันอยู่แล้วหรือไม่
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == 'mediamtx':
            rospy.loginfo("MediaMTX is already running.")
            return True

    cmd = [mediamtx_exec]
    if mediamtx_config: cmd.append(mediamtx_config)
        
    try:
        mediamtx_process = subprocess.Popen(cmd)
        time.sleep(2)
        return True
    except Exception as e:
        rospy.logerr(f"Error starting MediaMTX: {e}")
        return False

def launch_yolo_stream():
    global ffmpeg_process, cap, model, is_starting
    
    if is_starting: return False, "Already starting"
    is_starting = True

    try:
        # 1. โหลดโมเดล YOLO (โหลดครั้งแรกอาจใช้เวลา)
        if model is None:
            rospy.loginfo("Loading YOLO11 Nano...")
            model = YOLO('yolo11n.pt')

        # 2. เปิดกล้องด้วย OpenCV
        device = rospy.get_param('~device', '/dev/video0')
        cap = cv2.VideoCapture(device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 15)

        if not cap.isOpened():
            is_starting = False
            return False, "Cannot open camera"

        # 3. เตรียม MediaMTX
        start_mediamtx()

        # 4. ตั้งค่า FFmpeg รับค่าจาก Pipe (stdin)
        rtsp_url = rospy.get_param('~rtsp_url', 'rtsp://localhost:8554/mystream')
        bitrate = str(rospy.get_param('~bitrate', '600k'))
        
        ffmpeg_command = [
            'ffmpeg', '-y',
            '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-s', '640x480', '-pix_fmt', 'bgr24', '-r', '15',
            '-i', '-', # รับ Input จาก Python Pipe
            '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
            '-pix_fmt', 'yuv420p', '-b:v', bitrate,
            '-f', 'rtsp', '-rtsp_transport', 'tcp', rtsp_url
        ]

        ffmpeg_process = subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)
        rospy.loginfo("YOLO + FFmpeg Stream Started")
        is_starting = False
        return True, "Started"

    except Exception as e:
        is_starting = False
        return False, str(e)

def stream_loop():
    """Loop หลักที่ทำหน้าที่ดึงภาพ รัน YOLO และส่งเข้า FFmpeg"""
    global cap, model, ffmpeg_process, is_stream_enabled
    
    fps = 15
    rate = rospy.Rate(fps)
    frame_counter = 0
    skip_rate = 3 # รัน YOLO ทุกๆ 3 เฟรม (เหลือ 5 FPS) เพื่อประหยัด CPU
    cached_boxes = []

    while not rospy.is_shutdown():
        if is_stream_enabled and cap is not None and ffmpeg_process is not None:
            ret, frame = cap.read()
            if not ret: continue

            frame_counter += 1
            
            # --- AI Processing (Frame Skipping) ---
            if frame_counter % skip_rate == 0:
                results = model.predict(source=frame, conf=0.5, verbose=False)
                cached_boxes = results[0].boxes.data.cpu().numpy()

            # --- Drawing ---
            for box in cached_boxes:
                x1, y1, x2, y2, conf, cls = box
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"{model.names[int(cls)]} {conf:.2f}"
                cv2.putText(frame, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # --- Send to FFmpeg ---
            try:
                ffmpeg_process.stdin.write(frame.tobytes())
            except Exception:
                rospy.logerr("FFmpeg Pipe broken")
                is_stream_enabled = False

        rate.sleep()

def handle_start(req):
    global is_stream_enabled
    if is_stream_enabled: return TriggerResponse(success=False, message="Already running")
    
    success, msg = launch_yolo_stream()
    if success:
        is_stream_enabled = True
        return TriggerResponse(success=True, message="Stream with YOLO started")
    return TriggerResponse(success=False, message=msg)

def handle_stop(req):
    global is_stream_enabled, ffmpeg_process, cap
    is_stream_enabled = False
    if ffmpeg_process:
        ffmpeg_process.stdin.close()
        ffmpeg_process.terminate()
    if cap: cap.release()
    return TriggerResponse(success=True, message="Stream stopped")

def cleanup():
    handle_stop(None)
    if mediamtx_process: mediamtx_process.terminate()

if __name__ == "__main__":
    rospy.init_node('stream_manager_yolo')
    rospy.on_shutdown(cleanup)
    
    rospy.Service('/stream_manager/start', Trigger, handle_start)
    rospy.Service('/stream_manager/stop', Trigger, handle_stop)
    
    rospy.loginfo("Stream Manager with YOLO Ready")
    # รัน Loop การสตรีมใน Thread หลัก
    stream_loop()