#!/usr/bin/env python3
import rospy
import subprocess
import time
import socket
import cv2
#from ultralytics import YOLO
from std_srvs.srv import Trigger, TriggerResponse
import threading
import queue
from ptR1_navigation.srv import UpdateDetection, UpdateDetectionResponse
from datetime import datetime
from std_msgs.msg import String
import json

import onnxruntime as ort
import numpy as np

COCO_CLASSES = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus',
    6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant',
    11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat',
    16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear',
    22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag',
    27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard',
    32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove',
    36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle',
    40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl',
    46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli',
    51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake',
    56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table',
    61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard',
    67: 'cell phone', 68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink',
    72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 76: 'scissors',
    77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'
}

alert_pub = None
last_alert_time = {}  # ป้องกัน spam ต่อ class

ai_result_lock = threading.Lock()
cached_boxes = []
frame_queue = queue.Queue(maxsize=1) 
ai_running = False  
latest_frame = None
frame_lock = threading.Lock()
camera_stop_event = threading.Event()
cam_reader_thread_ref = None

# --- Global variables ---
ffmpeg_process = None
mediamtx_process = None
is_stream_enabled = False
is_starting = False
mtx_host = 'localhost'
mtx_port = 8554
cap = None
model = None
prev_frame_gray = None

detection_enabled = False
detection_mode    = 'time'   # 'time' | 'manual'
detection_start   = 22
detection_end     = 6
detection_classes = ['person']
detection_lock    = threading.Lock()

def has_motion(frame, threshold=1000):
    global prev_frame_gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    if prev_frame_gray is None:
        prev_frame_gray = gray
        return False

    diff = cv2.absdiff(prev_frame_gray, gray)
    prev_frame_gray = gray
    changed_pixels = np.sum(diff > 25)
    return changed_pixels > threshold

def is_frame_usable(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mean_brightness = np.mean(gray)
    # มืดเกินไป < 20, สว่างเกิน (overexposed) > 240
    return 20 < mean_brightness < 240

def init_alert_publisher():
    global alert_pub
    alert_pub = rospy.Publisher('/stream_manager/alert', String, queue_size=10)

def publish_alert(class_name, conf):
    global last_alert_time
    now = time.time()
    # throttle — ส่งซ้ำได้ทุก 5 วินาทีต่อ class
    if now - last_alert_time.get(class_name, 0) < 5.0:
        return
    last_alert_time[class_name] = now

    if alert_pub is None:
        return

    payload = json.dumps({
        'class_name': class_name,
        'timestamp':  datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'mode':       detection_mode,
    })
    alert_pub.publish(String(data=payload))
    rospy.loginfo(f"ALERT published: {class_name}")

def is_night_time(start, end): 
    now = datetime.now().hour
    return now >= start or now < end

def should_alert(class_name):
    with detection_lock:
        if not detection_enabled:
            return False
        if detection_mode == 'manual':
            return class_name in detection_classes
        if detection_mode == 'time':
            return is_night_time(detection_start, detection_end) and class_name in detection_classes
    return False

def handle_update_detection(req):
    global detection_enabled, detection_mode, detection_start, detection_end, detection_classes
    with detection_lock:
        detection_enabled = req.enabled
        detection_mode    = req.mode
        detection_start   = req.time_start
        detection_end     = req.time_end
        detection_classes = list(req.classes)

    rospy.loginfo(
        f"Detection updated: enabled={req.enabled}, mode={req.mode}, "
        f"time={req.time_start}-{req.time_end}, classes={list(req.classes)}"
    )
    return UpdateDetectionResponse(success=True, message="Detection settings updated.")

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

    mediamtx_exec = rospy.get_param('~mediamtx_exec', '/home/patrolR1/MediaMtx/mediamtx')
    mediamtx_config = rospy.get_param('~mediamtx_config', '/home/patrolR1/MediaMtx/mediamtx.yml')

    cmd = [mediamtx_exec]
    if mediamtx_config:
        cmd.append(mediamtx_config)

    try:
        mediamtx_process = subprocess.Popen(cmd)
        time.sleep(2.5)

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
def ffmpeg_writer_thread():
    """Thread แยกสำหรับส่งเฟรมไป FFmpeg — ใช้แค่เฟรมล่าสุดเสมอ"""
    while not rospy.is_shutdown():
        try:
            # ✅ รอเฟรมใหม่ timeout 1 วิ ป้องกัน block ตลอดกาล
            frame = frame_queue.get(timeout=1.0)
            if ffmpeg_process and ffmpeg_process.poll() is None:
                ffmpeg_process.stdin.write(frame.tobytes())
        except queue.Empty:
            continue
        except Exception as e:
            rospy.logerr(f"FFmpeg Writer Error: {e}")
            break
def launch_ffmpeg_pipe():
    """ตั้งค่า FFmpeg ให้รอรับภาพจากท่อ (stdin) ของ Python"""
    rtsp_url = rospy.get_param('~rtsp_url', 'rtsp://localhost:8554/mystream')
    bitrate = str(rospy.get_param('~bitrate', '600k'))

    ffmpeg_command = [
    'ffmpeg', '-y',
    '-f', 'rawvideo', '-vcodec', 'rawvideo',
    '-s', '640x480', '-pix_fmt', 'bgr24', '-r', '10',
    '-i', '-',
    '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
    '-profile:v', 'baseline', '-pix_fmt', 'yuv420p',
    '-g', '5',           # ✅ ลดจาก 15 → 5 (keyframe ถี่ขึ้น)
    '-bf', '0',          # ✅ ไม่ใช้ B-frames
    '-refs', '1',        # ✅ ลด reference frames
    '-b:v', bitrate,
    '-maxrate', bitrate, # ✅ จำกัด bitrate สูงสุด
    '-bufsize', '500k',  # ✅ ลด encoder buffer
    '-f', 'rtsp',
    '-rtsp_transport', 'tcp',
    '-muxdelay', '0',    # ✅ ลด mux delay
    '-muxpreload', '0',  # ✅ ลด mux preload
    rtsp_url
]

    rospy.loginfo("Starting FFmpeg Pipe...")
    return subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)

def stop_process(proc, name):
    if proc is None:
        return None

    rospy.loginfo(f"Stopping {name} (PID: {proc.pid})...")
    try:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rospy.logwarn(f"{name} did not stop in time. Force killing...")
                proc.kill()
                proc.wait()
    except Exception as e:
        rospy.logerr(f"Exception stopping {name}: {e}")
    return None

def monitor_loop(event):
    global ffmpeg_process, mediamtx_process, is_stream_enabled, is_starting

    if is_stream_enabled and not is_starting:

        # ✅ เช็ค MediaMTX ก่อน เพราะถ้าตาย FFmpeg ก็ไม่มีประโยชน์
        if mediamtx_process is not None and mediamtx_process.poll() is not None:
            rospy.logerr("Stream Monitor: MediaMTX crashed! Performing full system reset...")
            is_stream_enabled = False
            cleanup()
            time.sleep(1)

            if start_mediamtx():
                is_starting = True  # ✅ ล็อคก่อนเสมอ
                try:
                    ffmpeg_process = launch_ffmpeg_pipe()
                finally:
                    is_starting = False
                    is_stream_enabled = True
            return  # ออกก่อน ไม่ต้องเช็ค FFmpeg ซ้ำ

        # เช็ค FFmpeg
        if ffmpeg_process is None or ffmpeg_process.poll() is not None:
            rospy.logwarn("Stream Monitor: FFmpeg is not running. Attempting auto-restart...")
            is_starting = True  # ✅ ล็อคก่อนเสมอ
            try:
                ffmpeg_process = launch_ffmpeg_pipe()
            finally:
                is_starting = False  # ✅ ปลดล็อคเสมอแม้จะเกิด exception

            if ffmpeg_process is not None:
                rospy.loginfo("Stream Monitor: Auto-restart successful.")
            else:
                rospy.logerr("Stream Monitor: Auto-restart failed.")

def camera_reader_thread():
    global latest_frame, cap
    while not rospy.is_shutdown() and not camera_stop_event.is_set():
        if cap and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                with frame_lock:
                    latest_frame = frame
        else:
            time.sleep(0.01)

    # ✅ thread release cap เองเลย ไม่ให้ใครมา release ทับ
    if cap:
        cap.release()
        cap = None
    rospy.loginfo("camera_reader_thread exited cleanly.")

def handle_start_stream(req):
    global is_stream_enabled, ffmpeg_process, cap, model, latest_frame, cam_reader_thread_ref
    rospy.loginfo("Request to START stream received.")

    if is_stream_enabled and ffmpeg_process is not None and ffmpeg_process.poll() is None:
        return TriggerResponse(success=False, message="Stream is already running.")

    if not start_mediamtx():
        return TriggerResponse(success=False, message="Failed to start MediaMTX")

    device = rospy.get_param('~device', '/dev/video0')
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 500)

    if not cap.isOpened():
        return TriggerResponse(success=False, message="Camera failed")

    if model is None:
        rospy.loginfo("Loading YOLO11 Nano (ONNX Runtime)...")
        # ใช้ ONNX Runtime โหลดโมเดล
        model_path = '/home/patrolR1/ptR1_ws/src/ptR1_navigation/model/yolo11n.onnx'
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 2  # จำกัดให้ใช้แค่ 2 Thread (2 Core)
        sess_options.inter_op_num_threads = 2  
        
        model = ort.InferenceSession(model_path, sess_options=sess_options, providers=['CPUExecutionProvider'])

    latest_frame = None
    camera_stop_event.clear()
    cam_reader_thread_ref = threading.Thread(target=camera_reader_thread)  # ✅ ตัวเดียว
    cam_reader_thread_ref.daemon = True
    cam_reader_thread_ref.start()

    ffmpeg_process = launch_ffmpeg_pipe()
    is_stream_enabled = True
    return TriggerResponse(success=True, message="Stream + YOLO Node Started")
def handle_stop_stream(req):
    global is_stream_enabled, ffmpeg_process, mediamtx_process, latest_frame
    rospy.loginfo("Request to STOP stream received.")

    is_stream_enabled = False
    camera_stop_event.set()

    if cam_reader_thread_ref is not None and cam_reader_thread_ref.is_alive():
        cam_reader_thread_ref.join(timeout=3.0)  # ✅ thread release cap เอง

    latest_frame = None

    if ffmpeg_process and ffmpeg_process.stdin:
        try:
            ffmpeg_process.stdin.close()
        except:
            pass

    ffmpeg_process = stop_process(ffmpeg_process, "FFmpeg")
    mediamtx_process = stop_process(mediamtx_process, "MediaMTX")
    return TriggerResponse(success=True, message="Stream stopped.")

def handle_toggle_ai(req):
    global detection_enabled
    detection_enabled = not detection_enabled
    msg = f"AI Detection: {'ON' if detection_enabled else 'OFF'}"
    rospy.loginfo(msg)
    return TriggerResponse(success=True, message=msg)
def ai_worker(frame):
    global cached_boxes, ai_running
    try:
        if model is None:
            return

        # 1. Pre-processing: เตรียมรูปภาพ
        input_width, input_height = 320, 320
        img = cv2.resize(frame, (input_width, input_height))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose((2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)  # Add batch dimension
        img = img.astype(np.float32) / 255.0  # Normalize to 0-1

        # 2. Inference: โยนเข้า ONNX
        input_name = model.get_inputs()[0].name
        outputs = model.run(None, {input_name: img})
        predictions = np.squeeze(outputs[0]).T  # Shape: (84, N) -> (N, 84)

        # 3. Post-processing: แกะข้อมูลกรอบ (Bounding Boxes)
        boxes = []
        scores = []
        class_ids = []
        
        # สเกลเพื่อขยายกรอบกลับไปที่ขนาดจอจริง (640x480)
        orig_h, orig_w = frame.shape[:2]
        x_scale = orig_w / input_width
        y_scale = orig_h / input_height

        for row in predictions:
            classes_scores = row[4:]
            class_id = np.argmax(classes_scores)
            score = classes_scores[class_id]

            if score > 0.35: # ค่าความมั่นใจ (Confidence)
                cx, cy, w, h = row[0:4]
                
                # แปลงจุดศูนย์กลางให้เป็น x, y มุมซ้ายบน และขยายสเกล
                x1 = int((cx - w / 2) * x_scale)
                y1 = int((cy - h / 2) * y_scale)
                width = int(w * x_scale)
                height = int(h * y_scale)

                boxes.append([x1, y1, width, height])
                scores.append(float(score))
                class_ids.append(class_id)

        # 4. Non-Maximum Suppression (ลบกรอบที่ซ้อนทับกัน)
        indices = cv2.dnn.NMSBoxes(boxes, scores, 0.35, 0.45)
        
        new_boxes = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                new_boxes.append([x, y, x + w, y + h, scores[i], class_ids[i]])

        # 5. อัปเดตกรอบให้ Stream Manager ดึงไปวาด
        with ai_result_lock:
            cached_boxes = new_boxes

    except Exception as e:
        rospy.logerr(f"AI Worker Error: {e}")
    finally:
        ai_running = False  # ปลดล็อคเสมอ


def cleanup():
    global is_stream_enabled, ffmpeg_process, mediamtx_process, cap, cached_boxes, ai_running, latest_frame
    is_stream_enabled = False
    camera_stop_event.set()

    if cam_reader_thread_ref is not None and cam_reader_thread_ref.is_alive():
        cam_reader_thread_ref.join(timeout=3.0)

    ai_running = False
    cached_boxes = []
    latest_frame = None

    if ffmpeg_process and ffmpeg_process.stdin:
        try:
            ffmpeg_process.stdin.close()
        except:
            pass

    ffmpeg_process = stop_process(ffmpeg_process, "FFmpeg")
    mediamtx_process = stop_process(mediamtx_process, "MediaMTX")
def stream_manager_server():
    global is_stream_enabled, cap, ffmpeg_process, cached_boxes, ai_running, latest_frame

    rospy.init_node('stream_manager_server')
    init_alert_publisher()
    rospy.on_shutdown(cleanup)
    rospy.Service('/stream_manager/start', Trigger, handle_start_stream)
    rospy.Service('/stream_manager/stop', Trigger, handle_stop_stream)
    rospy.Service('/stream_manager/toggle_ai', Trigger, handle_toggle_ai)
    rospy.Service('/stream_manager/update_detection', UpdateDetection, handle_update_detection)
    rospy.loginfo("Stream Manager Ready")

    writer = threading.Thread(target=ffmpeg_writer_thread)
    writer.daemon = True
    writer.start()

    rate = rospy.Rate(10)
    frame_counter = 0

    rospy.Timer(rospy.Duration(5), monitor_loop)

    try:
        while not rospy.is_shutdown():
            if is_stream_enabled and ffmpeg_process:

                # อ่านเฟรมล่าสุดจาก camera_reader_thread
                with frame_lock:
                    if latest_frame is None:
                        rate.sleep()
                        continue
                    frame = latest_frame.copy()

                # --- AI LOGIC ---
                if detection_enabled and model:
                    frame_counter += 1
                    if frame_counter % 7 == 0 and not ai_running:
                        if is_frame_usable(frame) and has_motion(frame):
                            ai_running = True
                            t = threading.Thread(target=ai_worker, args=(frame.copy(),))
                            t.daemon = True
                            t.start()
                        frame_counter = 0

                    with ai_result_lock:
                        boxes_to_draw = list(cached_boxes)

                    for box in boxes_to_draw:
                        x1, y1, x2, y2, conf, cls = box
                        class_name = COCO_CLASSES.get(int(cls), f"Unknown_{int(cls)}")
                        #ข้ามถ้า class ไม่ได้อยู่ใน detection_classes
                        with detection_lock:
                            classes_to_show = list(detection_classes)
                        if class_name not in classes_to_show:
                            continue

                        # เช็คเงื่อนไข alert
                        if should_alert(class_name):
                            color = (0, 0, 255)     # แดง — alert
                            thickness = 3
                            label = f"! {class_name} {conf:.2f}"
                            publish_alert(class_name, conf) 
                        else:
                            color = (0, 255, 0)     # เขียว — ปกติ
                            thickness = 2
                            label = f"{class_name} {conf:.2f}"

                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
                        cv2.putText(frame, label, (int(x1), int(y1) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                else:
                    with ai_result_lock:
                        cached_boxes = []

                # --- ส่งเฟรมล่าสุดไป ffmpeg_writer_thread ---
                try:
                    frame_queue.get_nowait()  # ทิ้งเฟรมเก่าถ้ามี
                except queue.Empty:
                    pass
                frame_queue.put_nowait(frame)  # ใส่เฟรมใหม่

            rate.sleep()

    except Exception as e:
        rospy.logfatal(f"FATAL ERROR in main loop: {e}")
        import traceback
        rospy.logfatal(traceback.format_exc())
        raise

if __name__ == "__main__":
    stream_manager_server()