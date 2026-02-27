#!/usr/bin/env python3

import rospy
import subprocess
import time
import os
import signal
import socket
from urllib.parse import urlparse
from std_srvs.srv import Trigger, TriggerResponse
import psutil
from std_msgs.msg import Float32

# --- Global variables ---
ffmpeg_process = None
mediamtx_process = None
is_stream_enabled = False
is_starting = False 
mtx_host = 'localhost'
mtx_port = 8554
cpu_pub_ffmpeg = None
cpu_pub_mediamtx = None

def cpu_monitor_loop(event):
    global ffmpeg_process, mediamtx_process, cpu_pub_ffmpeg, cpu_pub_mediamtx
    
    # 1. ดึง CPU ของ FFmpeg
    if ffmpeg_process is not None and ffmpeg_process.poll() is None:
        try:
            p_ffmpeg = psutil.Process(ffmpeg_process.pid)
            cpu_usage = p_ffmpeg.cpu_percent(interval=0.1) 
            cpu_pub_ffmpeg.publish(cpu_usage)
        except psutil.NoSuchProcess:
            pass

    # 2. ดึง CPU ของ MediaMTX
    if mediamtx_process is not None and mediamtx_process.poll() is None:
        try:
            p_mtx = psutil.Process(mediamtx_process.pid)
            cpu_usage = p_mtx.cpu_percent(interval=0.1)
            cpu_pub_mediamtx.publish(cpu_usage)
        except psutil.NoSuchProcess:
            pass

def check_socket_open(host, port, timeout=1):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False

def start_mediamtx():
    global mediamtx_process, mtx_host, mtx_port

    if check_socket_open(mtx_host, mtx_port):
        rospy.loginfo("MediaMTX is already running.")
        return True
        
    rospy.loginfo("MediaMTX is NOT running. Attempting to start...")

    # เช็ค Path ให้ดีนะครับ
    mediamtx_exec = rospy.get_param('~mediamtx_exec', '/home/patrolR1/MediaMtx/mediamtx')
    mediamtx_config = rospy.get_param('~mediamtx_config', '/home/patrolR1/MediaMtx/mediamtx.yml')
    
    cmd = [mediamtx_exec]
    if mediamtx_config:
        cmd.append(mediamtx_config)
        
    try:
        # เปิดแบบให้เห็น Log ของ MediaMTX ด้วย (ถ้าไม่อยากเห็นให้ใส่ stdout=subprocess.DEVNULL)
        mediamtx_process = subprocess.Popen(cmd)
        time.sleep(2.5)  # [Adjust] เพิ่มเวลารอ Server Boot นิดหน่อย

        if mediamtx_process.poll() is not None:
            rospy.logerr("MediaMTX process terminated immediately.")
            return False
            
        rospy.loginfo(f"MediaMTX started successfully (PID: {mediamtx_process.pid}).")
        return True

    except FileNotFoundError:
        rospy.logerr(f"MediaMTX executable not found at: {mediamtx_exec}")
        return False
    except Exception as e:
        rospy.logerr(f"Exception starting MediaMTX: {e}")
        return False

def launch_ffmpeg_stream():
    global ffmpeg_process, mtx_host, mtx_port, is_starting
    
    # ถ้ากำลัง Start อยู่แล้ว ห้ามทำงานซ้อน
    if is_starting:
        rospy.logwarn("Stream launch already in progress. Ignoring duplicate request.")
        return False, "Launch in progress"

    is_starting = True # ล็อคการทำงาน

    try:
        if ffmpeg_process is not None:
            # เรียกฟังก์ชัน stop_process ที่เราเขียนไว้แล้ว
            ffmpeg_process = stop_process(ffmpeg_process, "Old FFmpeg Instance")

        rtsp_url = rospy.get_param('~rtsp_url', 'rtsp://localhost:8554/mystream')
        
        try:
            parsed_url = urlparse(rtsp_url)
            mtx_host = parsed_url.hostname or 'localhost'
            mtx_port = parsed_url.port or 8554
        except:
            pass 

        if not start_mediamtx():
            return False, "Failed to start MediaMTX"

        if not check_socket_open(mtx_host, mtx_port):
            rospy.logerr(f"MediaMTX unreachable at {mtx_host}:{mtx_port}")
            return False, "MediaMTX unreachable"
        
        device = rospy.get_param('~device', '/dev/video0')
        bitrate = str(rospy.get_param('~bitrate', '600k')) # ครอบ str() ไว้กันเหนียวเผื่อรับค่ามาเป็นตัวเลข
        
        ffmpeg_command = [
            'ffmpeg',
            '-y',
            '-f', 'v4l2',
            '-input_format', 'mjpeg',
            '-framerate', '15',        
            '-video_size', '640x480',
            '-i', device,

            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-profile:v', 'baseline',
            '-pix_fmt', 'yuv420p',
            '-g', '15',                
            
            '-b:v', bitrate,      
            '-f', 'rtsp',
            '-rtsp_transport', 'tcp',
            rtsp_url
        ]
        rospy.loginfo(f"Executing FFmpeg: {' '.join(ffmpeg_command)}")
        
        # รัน FFmpeg โดยไม่ปิด stdout/stderr เพื่อดู Error ถ้ามี
        ffmpeg_process = subprocess.Popen(ffmpeg_command)
        
        time.sleep(1.5) # รอเช็คสถานะ
        
        if ffmpeg_process.poll() is not None:
            # ถ้าตายทันที ให้แจ้ง Error
            rospy.logerr("FFmpeg crashed immediately! Check camera connection.")
            is_starting = False
            return False, "FFmpeg crashed"
            
        rospy.loginfo("FFmpeg streaming started successfully.")
        is_starting = False # ปลดล็อค
        return True, "FFmpeg started"
        
    except Exception as e:
        is_starting = False
        rospy.logerr(f"Exception starting FFmpeg: {e}")
        return False, str(e)
    finally:
        is_starting = False

def stop_process(proc, name):
    if proc is None:
        return None
    
    rospy.loginfo(f"Stopping {name} (PID: {proc.pid})...")
    try:
        if proc.poll() is None:
            # ส่ง SIGTERM (สวยงามกว่า SIGINT ในบางกรณี)
            proc.terminate()
            try:
                # รอนานขึ้นนิดนึง (3-5 วินาที) โดยเฉพาะเมื่อ CPU โหลดสูงแบบใน htop ของคุณ
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rospy.logwarn(f"{name} did not stop in time. Force killing...")
                proc.kill() # ถ้าไม่ยอมตาย ให้ใช้ท่าไม้ตาย SIGKILL
                proc.wait()
    except Exception as e:
        rospy.logerr(f"Exception stopping {name}: {e}")    
    return None

def monitor_loop(event):
    global ffmpeg_process, mediamtx_process, is_stream_enabled, is_starting
    
    # ถ้า user สั่งเปิด AND ไม่ได้กำลัง Start อยู่
    if is_stream_enabled and not is_starting:
        
        # เช็ค FFmpeg
        if ffmpeg_process is None or ffmpeg_process.poll() is not None:
            rospy.logwarn("Stream Monitor: FFmpeg is not running. Attempting auto-restart...")        
            success, msg = launch_ffmpeg_stream()
            if not success:
                rospy.logerr(f"Stream Monitor: Auto-restart failed -> {msg}")
            else:
                rospy.loginfo("Stream Monitor: Auto-restart successful.")

        # เช็ค MediaMTX (เฉพาะถ้าเราเป็นคนเปิด)
        if mediamtx_process is not None and mediamtx_process.poll() is not None:
            rospy.logerr("Stream Monitor: MediaMTX crashed! Performing full system reset...")
            
            # ปิดทุกอย่างแล้วเริ่มใหม่จากศูนย์
            is_stream_enabled = False # หยุด monitor ชั่วคราว
            cleanup() 
            time.sleep(1)
            
            # ลองเริ่มระบบใหม่
            success, msg = launch_ffmpeg_stream()
            if success:
                is_stream_enabled = True

def handle_start_stream(req):
    global is_stream_enabled, ffmpeg_process
    rospy.loginfo("Request to START stream received.")
    
    if is_stream_enabled and ffmpeg_process is not None and ffmpeg_process.poll() is None:
        return TriggerResponse(success=False, message="Stream is already running.")
    success, message = launch_ffmpeg_stream()

    if success:
        is_stream_enabled = True # เปิดระบบ Monitor เมื่อ Start ผ่านเท่านั้น
        return TriggerResponse(success=True, message=message)
    else:
        is_stream_enabled = False
        return TriggerResponse(success=False, message=message)

def handle_stop_stream(req):
    global is_stream_enabled, ffmpeg_process
    rospy.loginfo("Request to STOP stream received.")
    is_stream_enabled = False # ปิด Monitor ทันที
    
    ffmpeg_process = stop_process(ffmpeg_process, "FFmpeg")
    return TriggerResponse(success=True, message="Stream stopped.")

def cleanup():
    global is_stream_enabled, ffmpeg_process, mediamtx_process
    is_stream_enabled = False
    ffmpeg_process = stop_process(ffmpeg_process, "FFmpeg")
    mediamtx_process = stop_process(mediamtx_process, "MediaMTX")

def stream_manager_server():
    rospy.init_node('stream_manager_server')
    rospy.on_shutdown(cleanup)

    cpu_pub_ffmpeg = rospy.Publisher('/stream_manager/cpu/ffmpeg', Float32, queue_size=10)
    cpu_pub_mediamtx = rospy.Publisher('/stream_manager/cpu/mediamtx', Float32, queue_size=10)
    rospy.Service('/stream_manager/start', Trigger, handle_start_stream)
    rospy.Service('/stream_manager/stop', Trigger, handle_stop_stream)

    rospy.Timer(rospy.Duration(5), monitor_loop)
    rospy.Timer(rospy.Duration(2), cpu_monitor_loop)
    
    rospy.loginfo("Stream Manager Ready")
    rospy.spin()

if __name__ == "__main__":
    stream_manager_server()