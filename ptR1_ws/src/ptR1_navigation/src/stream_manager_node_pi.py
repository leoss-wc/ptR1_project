#!/usr/bin/env python3
import rospy
import subprocess
import time
import socket
import cv2
from std_srvs.srv import Trigger, TriggerResponse
import threading
import queue
from ptR1_navigation.srv import UpdateDetection, UpdateDetectionResponse
from datetime import datetime
from std_msgs.msg import String
import json
import os

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

DOOR_CLASSES = {
    0: 'door_close',
    1: 'door_open'
}

alert_pub = None
last_alert_time = {}

# --- Model 1 (COCO / person detection) ---
ai_result_lock = threading.Lock()
cached_boxes = []
frame_queue = queue.Queue(maxsize=1)
ai_running = threading.Event()
last_inference_ms = 0.0

# --- Model 2 (Door detection) ---
ai_result_lock2 = threading.Lock()
cached_boxes2 = []
ai_running2 = threading.Event()
last_inference_ms2 = 0.0

latest_frame = None
frame_lock = threading.Lock()
camera_stop_event = threading.Event()
cam_reader_thread_ref = None
ai_stats_pub = None

# --- Global variables ---
ffmpeg_process = None
mediamtx_process = None
is_stream_enabled = False
is_starting = False
mtx_host = 'localhost'
mtx_port = 8554
cap = None
model = None
model_input_name = None
model2 = None
model2_input_name = None
prev_frame_gray = None

detection_enabled = False
detection_mode    = 'time'
detection_start   = 22
detection_end     = 6
detection_classes = ['person']
detection_lock    = threading.Lock()

capture_burst_timer = None
capture_label = "object"
capture_save_dir = os.path.expanduser('~/dataset/raw')
capture_count = 0
capture_pub = None
os.makedirs(capture_save_dir, exist_ok=True)


def _do_capture():
    global capture_count
    with frame_lock:
        if latest_frame is None:
            rospy.logwarn("Capture: No frame available")
            return
        frame = latest_frame.copy()

    timestamp = int(time.time() * 1000)
    filename  = f"{capture_label}_{timestamp}.jpg"
    filepath  = os.path.join(capture_save_dir, filename)

    cv2.imwrite(filepath, frame)
    capture_count += 1
    rospy.loginfo(f"Captured [{capture_count}]: {filename}")

    if capture_pub:
        capture_pub.publish(json.dumps({
            'filename': filename,
            'count':    capture_count,
            'label':    capture_label,
        }))

def handle_capture_single(req):
    if not is_stream_enabled:
        return TriggerResponse(success=False, message="Stream not running")
    _do_capture()
    return TriggerResponse(success=True, message=f"Captured as '{capture_label}'")

def handle_capture_start(req):
    global capture_burst_timer, capture_label
    if not is_stream_enabled:
        return TriggerResponse(success=False, message="Stream not running")
    return TriggerResponse(success=True, message=f"Burst started: {capture_label}")

def handle_capture_stop(req):
    global capture_burst_timer
    if capture_burst_timer:
        capture_burst_timer.shutdown()
        capture_burst_timer = None
    rospy.loginfo("Burst capture stopped")
    return TriggerResponse(success=True, message=f"Stopped. Total: {capture_count} images")

def handle_capture_config(msg):
    global capture_burst_timer, capture_label
    data = msg.data.strip()

    if data == 'stop':
        if capture_burst_timer:
            capture_burst_timer.shutdown()
            capture_burst_timer = None
        rospy.loginfo("Burst stopped via topic")
        return

    if ':' in data:
        label, interval = data.split(':', 1)
        capture_label = label.strip()
        interval_sec  = float(interval.strip())

        if capture_burst_timer:
            capture_burst_timer.shutdown()

        capture_burst_timer = rospy.Timer(
            rospy.Duration(interval_sec),
            lambda event: _do_capture()
        )
        rospy.loginfo(f"Burst started: label={capture_label}, interval={interval_sec}s")


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
    return 20 < mean_brightness < 240

def init_alert_publisher():
    global alert_pub, ai_stats_pub, capture_pub
    alert_pub    = rospy.Publisher('/stream_manager/alert', String, queue_size=10)
    ai_stats_pub = rospy.Publisher('/stream_manager/ai_stats', String, queue_size=5)
    capture_pub  = rospy.Publisher('/stream_manager/captured', String, queue_size=10)

def publish_alert(class_name, conf):
    global last_alert_time
    now = time.time()
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

    mediamtx_exec   = rospy.get_param('~mediamtx_exec',   '/home/patrolR1/MediaMtx/mediamtx')
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
    while not rospy.is_shutdown():
        try:
            frame = frame_queue.get(timeout=1.0)
            if ffmpeg_process and ffmpeg_process.poll() is None:
                ffmpeg_process.stdin.write(frame.tobytes())
        except queue.Empty:
            continue
        except Exception as e:
            rospy.logerr(f"FFmpeg Writer Error: {e}")
            break

def launch_ffmpeg_pipe():
    rtsp_url = rospy.get_param('~rtsp_url', 'rtsp://localhost:8554/mystream')
    bitrate  = str(rospy.get_param('~bitrate', '600k'))
    fps      = rospy.get_param('~camera_fps', 12)

    ffmpeg_command = [
        'ffmpeg', '-y',
        '-f', 'rawvideo', '-vcodec', 'rawvideo',
        '-s', '640x480', '-pix_fmt', 'bgr24', '-r', str(fps),
        '-i', '-',
        '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
        '-profile:v', 'baseline', '-pix_fmt', 'yuv420p',
        '-g', '5', '-bf', '0', '-refs', '1',
        '-b:v', bitrate, '-maxrate', bitrate, '-bufsize', '500k',
        '-f', 'rtsp', '-rtsp_transport', 'tcp',
        '-muxdelay', '0', '-muxpreload', '0',
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
        if mediamtx_process is not None and mediamtx_process.poll() is not None:
            rospy.logerr("Stream Monitor: MediaMTX crashed! Performing full system reset...")
            is_stream_enabled = False
            cleanup()
            time.sleep(1)

            if start_mediamtx():
                is_starting = True
                try:
                    ffmpeg_process = launch_ffmpeg_pipe()
                finally:
                    is_starting = False
                    is_stream_enabled = True
            return

        if ffmpeg_process is None or ffmpeg_process.poll() is not None:
            rospy.logwarn("Stream Monitor: FFmpeg is not running. Attempting auto-restart...")
            is_starting = True
            try:
                ffmpeg_process = launch_ffmpeg_pipe()
            finally:
                is_starting = False

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

    if cap:
        cap.release()
        cap = None
    rospy.loginfo("camera_reader_thread exited cleanly.")

def handle_start_stream(req):
    global is_stream_enabled, ffmpeg_process, cap, model, model_input_name, model2, model2_input_name, latest_frame, cam_reader_thread_ref
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
    cap.set(cv2.CAP_PROP_FPS, 12)

    if not cap.isOpened():
        return TriggerResponse(success=False, message="Camera failed")

    if model is None:
        rospy.loginfo("Loading YOLO11 Nano (ONNX Runtime) — Model 1 (COCO)...")
        model_path = '/home/patrolR1/ptR1_ws/src/ptR1_navigation/model/yolo11n.onnx'
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 1
        sess_options.inter_op_num_threads = 1
        model = ort.InferenceSession(model_path, sess_options=sess_options, providers=['CPUExecutionProvider'])
        model_input_name = model.get_inputs()[0].name

    if model2 is None:
        rospy.loginfo("Loading Door Detection Model (ONNX Runtime) — Model 2 (door_open/door_close)...")
        model2_path = '/home/patrolR1/ptR1_ws/src/ptR1_navigation/model/door_detector_best.onnx'
        sess_options2 = ort.SessionOptions()
        sess_options2.intra_op_num_threads = 1
        sess_options2.inter_op_num_threads = 1
        model2 = ort.InferenceSession(model2_path, sess_options=sess_options2, providers=['CPUExecutionProvider'])
        model2_input_name = model2.get_inputs()[0].name

    latest_frame = None
    camera_stop_event.clear()
    cam_reader_thread_ref = threading.Thread(target=camera_reader_thread)
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
        cam_reader_thread_ref.join(timeout=3.0)

    latest_frame = None

    if ffmpeg_process and ffmpeg_process.stdin:
        try:
            ffmpeg_process.stdin.close()
        except:
            pass

    ffmpeg_process   = stop_process(ffmpeg_process, "FFmpeg")
    mediamtx_process = stop_process(mediamtx_process, "MediaMTX")
    return TriggerResponse(success=True, message="Stream stopped.")

def handle_toggle_ai(req):
    global detection_enabled
    with detection_lock:
        detection_enabled = not detection_enabled
    msg = f"AI Detection: {'ON' if detection_enabled else 'OFF'}"
    rospy.loginfo(msg)
    return TriggerResponse(success=True, message=msg)

def ai_worker(frame):
    global cached_boxes, ai_running, last_inference_ms
    try:
        if model is None:
            return
        t0 = time.time()
        orig_h, orig_w = frame.shape[:2]
        img = cv2.resize(frame, (320, 320))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = (img.transpose((2, 0, 1))[np.newaxis] / 255.0).astype(np.float32)

        outputs = model.run(None, {model_input_name: img})
        last_inference_ms = (time.time() - t0) * 1000
        predictions = np.squeeze(outputs[0]).T

        if len(predictions) == 0:
            with ai_result_lock:
                cached_boxes = []
            return

        classes_scores = predictions[:, 4:]
        class_ids      = np.argmax(classes_scores, axis=1)
        scores         = classes_scores[np.arange(len(predictions)), class_ids]

        mask        = scores > 0.45
        predictions = predictions[mask]
        scores      = scores[mask]
        class_ids   = class_ids[mask]

        if len(predictions) == 0:
            with ai_result_lock:
                cached_boxes = []
            return

        x_scale = orig_w / 320
        y_scale = orig_h / 320

        cx = predictions[:, 0]
        cy = predictions[:, 1]
        w  = predictions[:, 2]
        h  = predictions[:, 3]

        x1 = ((cx - w / 2) * x_scale).astype(int)
        y1 = ((cy - h / 2) * y_scale).astype(int)
        bw = (w * x_scale).astype(int)
        bh = (h * y_scale).astype(int)

        boxes_list     = np.stack([x1, y1, bw, bh], axis=1).tolist()
        scores_list    = scores.tolist()
        class_ids_list = class_ids.tolist()

        indices = cv2.dnn.NMSBoxes(boxes_list, scores_list, 0.35, 0.45)

        new_boxes = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, bw_, bh_ = boxes_list[i]
                new_boxes.append([x, y, x + bw_, y + bh_,
                                  scores_list[i], class_ids_list[i]])

        with ai_result_lock:
            cached_boxes = new_boxes

    except Exception as e:
        rospy.logerr(f"AI Worker Error: {e}")
    finally:
        ai_running.clear()

def ai_worker2(frame):
    global cached_boxes2, ai_running2, last_inference_ms2
    try:
        if model2 is None:
            return
        t0 = time.time()

        orig_h, orig_w = frame.shape[:2]
        img = cv2.resize(frame, (320, 320))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = (img.transpose((2, 0, 1))[np.newaxis] / 255.0).astype(np.float32)

        outputs = model2.run(None, {model2_input_name: img})
        last_inference_ms2 = (time.time() - t0) * 1000
        predictions = np.squeeze(outputs[0]).T

        num_door_classes = len(DOOR_CLASSES)
        if len(predictions) == 0:
            with ai_result_lock2:
                cached_boxes2 = []
            return

        classes_scores = predictions[:, 4:4 + num_door_classes]
        class_ids      = np.argmax(classes_scores, axis=1)
        scores         = classes_scores[np.arange(len(predictions)), class_ids]

        mask        = scores > 0.45
        predictions = predictions[mask]
        scores      = scores[mask]
        class_ids   = class_ids[mask]

        if len(predictions) == 0:
            with ai_result_lock2:
                cached_boxes2 = []
            return

        x_scale = orig_w / 320
        y_scale = orig_h / 320

        cx = predictions[:, 0]
        cy = predictions[:, 1]
        w  = predictions[:, 2]
        h  = predictions[:, 3]

        x1 = ((cx - w / 2) * x_scale).astype(int)
        y1 = ((cy - h / 2) * y_scale).astype(int)
        bw = (w * x_scale).astype(int)
        bh = (h * y_scale).astype(int)

        boxes_list     = np.stack([x1, y1, bw, bh], axis=1).tolist()
        scores_list    = scores.tolist()
        class_ids_list = class_ids.tolist()

        indices = cv2.dnn.NMSBoxes(boxes_list, scores_list, 0.35, 0.45)

        new_boxes2 = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, bw_, bh_ = boxes_list[i]
                new_boxes2.append([x, y, x + bw_, y + bh_,
                                   scores_list[i], class_ids_list[i]])

        with ai_result_lock2:
            cached_boxes2 = new_boxes2

        rospy.logdebug(f"[Model2] Door inference: {last_inference_ms2:.1f} ms, detections: {len(new_boxes2)}")

    except Exception as e:
        rospy.logerr(f"AI Worker2 (Door) Error: {e}")
    finally:
        ai_running2.clear()


def cleanup():
    global is_stream_enabled, ffmpeg_process, mediamtx_process, cap, cached_boxes, ai_running, latest_frame
    global cached_boxes2, ai_running2
    is_stream_enabled = False
    camera_stop_event.set()

    if cam_reader_thread_ref is not None and cam_reader_thread_ref.is_alive():
        cam_reader_thread_ref.join(timeout=3.0)

    ai_running.clear()
    ai_running2.clear()
    cached_boxes  = []
    cached_boxes2 = []
    latest_frame  = None

    if ffmpeg_process and ffmpeg_process.stdin:
        try:
            ffmpeg_process.stdin.close()
        except:
            pass

    ffmpeg_process   = stop_process(ffmpeg_process, "FFmpeg")
    mediamtx_process = stop_process(mediamtx_process, "MediaMTX")

def stream_manager_server():
    global is_stream_enabled, ffmpeg_process, cached_boxes, ai_running, latest_frame

    camera_fps          = rospy.get_param('~camera_fps', 12)
    person_interval_sec = rospy.get_param('~person_interval_sec', 2.5)
    door_interval_sec   = rospy.get_param('~door_interval_sec', 6.0)

    PERSON_SKIP = int(camera_fps * person_interval_sec)  # เช่น 30
    DOOR_SKIP   = int(camera_fps * door_interval_sec)    # เช่น 72

    rospy.loginfo(f"PERSON_SKIP={PERSON_SKIP}, DOOR_SKIP={DOOR_SKIP}")

    rospy.init_node('stream_manager_server')
    init_alert_publisher()
    rospy.on_shutdown(cleanup)
    rospy.Service('/stream_manager/start',            Trigger,          handle_start_stream)
    rospy.Service('/stream_manager/stop',             Trigger,          handle_stop_stream)
    rospy.Service('/stream_manager/toggle_ai',        Trigger,          handle_toggle_ai)
    rospy.Service('/stream_manager/update_detection', UpdateDetection,  handle_update_detection)
    rospy.Service('/stream_manager/capture',          Trigger,          handle_capture_single)
    rospy.Service('/stream_manager/capture_stop',     Trigger,          handle_capture_stop)
    rospy.Subscriber('/stream_manager/capture_config', String, handle_capture_config)
    rospy.loginfo("Stream Manager Ready")

    writer = threading.Thread(target=ffmpeg_writer_thread)
    writer.daemon = True
    writer.start()

    rate = rospy.Rate(camera_fps)
    frame_counter      = 0
    ai_stats_counter   = 0
    door_frame_counter = 0

    # threshold สำหรับ publish ai_stats (ทุก PERSON_SKIP * 10 เฟรม)
    AI_STATS_THRESHOLD = PERSON_SKIP * 10

    rospy.Timer(rospy.Duration(5), monitor_loop)

    try:
        while not rospy.is_shutdown():
            if is_stream_enabled and ffmpeg_process:

                with frame_lock:
                    if latest_frame is None:
                        rate.sleep()
                        continue
                    frame = latest_frame.copy()

                frame_usable = is_frame_usable(frame)
                frame_moving = has_motion(frame)  # [FIX #3] เรียกครั้งเดียวเก็บผลไว้ใช้

                # snapshot detection settings ครั้งเดียวต่อเฟรม
                with detection_lock:
                    _enabled = detection_enabled
                    _mode    = detection_mode
                    _start   = detection_start
                    _end     = detection_end
                    _classes = list(detection_classes)

                # คำนวณว่าควร alert ไหม (ใช้ร่วมกันทั้ง 2 โมเดล)
                should_alert = False
                if _enabled:
                    if _mode == 'manual':
                        should_alert = True
                    elif _mode == 'time':
                        should_alert = is_night_time(_start, _end)

                # ==========================================
                # โมเดล 1: COCO (person detection)
                # ==========================================
                if _enabled and model and frame_moving and frame_usable:
                    frame_counter    += 1
                    ai_stats_counter += 1

                    # [FIX #2] ย้าย ai_stats ออกนอก PERSON_SKIP block
                    if ai_stats_counter >= AI_STATS_THRESHOLD and ai_stats_pub:
                        ai_stats_counter = 0
                        ai_stats_pub.publish(json.dumps({
                            'inference_ms':      round(last_inference_ms, 1),
                            'inference_ms2':     round(last_inference_ms2, 1),
                            'detection_enabled': _enabled,
                            'mode':              _mode
                        }))

                    # [FIX #3] ไม่เช็ค is_frame_usable/has_motion ซ้ำ
                    if frame_counter % PERSON_SKIP == 0 and not ai_running.is_set():
                        frame_counter = 0
                        ai_running.set()
                        t = threading.Thread(target=ai_worker, args=(frame.copy(),))
                        t.daemon = True
                        t.start()

                    with ai_result_lock:
                        boxes_to_draw = list(cached_boxes)

                    for box in boxes_to_draw:
                        x1, y1, x2, y2, conf, cls = box
                        class_name = COCO_CLASSES.get(int(cls), f"Unknown_{int(cls)}")
                        if class_name not in _classes:
                            continue

                        if should_alert:
                            color, thickness = (0, 0, 255), 3
                            label = f"! {class_name} {conf:.2f}"
                            publish_alert(class_name, conf)
                        else:
                            color, thickness = (0, 255, 0), 2
                            label = f"{class_name} {conf:.2f}"

                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
                        cv2.putText(frame, label, (int(x1), int(y1) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                else:
                    with ai_result_lock:
                        cached_boxes = []

                # ==========================================
                # โมเดล 2: Door Detection
                # ==========================================
                if model2 and _enabled and frame_usable and frame_moving:
                    door_frame_counter += 1

                    # [FIX #3] ไม่เช็ค is_frame_usable ซ้ำ
                    if door_frame_counter % DOOR_SKIP == 0 and not ai_running2.is_set():
                        door_frame_counter = 0
                        ai_running2.set()
                        t2 = threading.Thread(target=ai_worker2, args=(frame.copy(),))
                        t2.daemon = True
                        t2.start()

                    with ai_result_lock2:
                        door_boxes_to_draw = list(cached_boxes2)

                    for box in door_boxes_to_draw:
                        x1, y1, x2, y2, conf, cls = box
                        door_class = DOOR_CLASSES.get(int(cls), f"door_{int(cls)}")

                        # [FIX #1] เช็ค should_alert เหมือน โมเดล 1
                        if should_alert:
                            color     = (0, 0, 255)
                            thickness = 3
                            label     = f"! [M2] {door_class} {conf:.2f}"
                            publish_alert(door_class, conf)
                        else:
                            # door_open → สีส้ม, door_close → สีฟ้า
                            color     = (255, 165, 0) if door_class == 'door_open' else (0, 165, 255)
                            thickness = 2
                            label     = f"[M2] {door_class} {conf:.2f}"

                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
                        cv2.putText(frame, label, (int(x1), int(y1) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # ส่งเฟรมไป ffmpeg_writer_thread
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
                frame_queue.put_nowait(frame)

            rate.sleep()

    except Exception as e:
        rospy.logfatal(f"FATAL ERROR in main loop: {e}")
        import traceback
        rospy.logfatal(traceback.format_exc())
        raise

if __name__ == "__main__":
    stream_manager_server()