# ======================[ img_ws_adaptive_fixed.py ]======================

import asyncio
import websockets
import threading
import time
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

# ------------------------[ CONFIG ]------------------------
DEFAULT_QUALITY = 80
MIN_QUALITY = 40
MAX_QUALITY = 90
TARGET_FPS = 30

# ------------------------[ GLOBALS ]------------------------
clients = {}
clients_lock = asyncio.Lock()
sending_lock = asyncio.Lock()
ws_loop = None

# ------------------------[ HELPER STRUCT ]------------------------
class ClientInfo:
    def __init__(self, websocket):
        self.ws = websocket
        self.quality = DEFAULT_QUALITY
        self.last_ping = 0
        self.latency_ms = 0
        self.token_bucket = TARGET_FPS
        self.last_frame_time = time.time()

# ------------------------[ ROS Callback ]------------------------
def image_callback(msg):
    now = time.time()

    async def broadcast():
        async with sending_lock:
            async with clients_lock:
                for cid in list(clients):
                    client = clients[cid]

                    elapsed = now - client.last_frame_time
                    tokens_to_add = int(elapsed * TARGET_FPS)
                    client.token_bucket = min(client.token_bucket + tokens_to_add, TARGET_FPS)

                    if client.token_bucket < 1:
                        continue

                    client.token_bucket -= 1
                    client.last_frame_time = now

                    try:
                        if msg.encoding != "bgr8":
                            rospy.logwarn(f"Unsupported encoding: {msg.encoding}")
                            return
                        cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                        quality = max(MIN_QUALITY, min(client.quality, MAX_QUALITY))
                        ret, jpeg = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
                        if not ret:
                            continue
                        await client.ws.send(jpeg.tobytes())
                    except Exception as e:
                        rospy.logwarn(f"Send failed to client {cid}: {e}")
                        clients.pop(cid, None)

    # 🛑 ห้ามใช้ get_event_loop() ตรงนี้  
    # ✅ ใช้ ws_loop ที่สร้างใน start_ws()
    if ws_loop and ws_loop.is_running():
        ws_loop.call_soon_threadsafe(lambda: asyncio.ensure_future(broadcast()))
    else:
        rospy.logerr("❌ WebSocket loop not running!")
# ------------------------[ WebSocket Server ]------------------------
async def handle_client(websocket, path):
    cid = id(websocket)
    async with clients_lock:
        clients[cid] = ClientInfo(websocket)
    rospy.loginfo(f"New client {cid} from {websocket.remote_address}")

    try:
        async for message in websocket:
            if message.startswith(b"ping:"):
                try:
                    sent_ts = float(message.decode().split(":")[1])
                    rtt = (time.time() - sent_ts) * 1000
                    async with clients_lock:
                        clients[cid].latency_ms = rtt
                        if rtt > 800:
                            clients[cid].quality = max(MIN_QUALITY, clients[cid].quality - 10)
                        elif rtt < 200:
                            clients[cid].quality = min(MAX_QUALITY, clients[cid].quality + 5)
                except Exception:
                    pass
    finally:
        async with clients_lock:
            clients.pop(cid, None)
        rospy.loginfo(f"Client {cid} disconnected")

# ------------------------[ Thread Starter ]------------------------
def start_ws():
    global ws_loop
    ws_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(ws_loop)

    ws_server = websockets.serve(handle_client, '0.0.0.0', 8181)
    server = ws_loop.run_until_complete(ws_server)  # <- แก้: เก็บไว้ในตัวแปรด้วย!

    rospy.loginfo("✅ WebSocket server started and ready")
    ws_loop.run_forever()


# ------------------------[ Main ]------------------------
if __name__ == "__main__":
    rospy.init_node("adaptive_image_ws_node")

    ws_thread = threading.Thread(target=start_ws, daemon=True)
    ws_thread.start()

    try:
        while ws_loop is None or not ws_loop.is_running():
            rospy.loginfo("⏳ Waiting for WebSocket server to be ready...")
            time.sleep(0.1)
    except KeyboardInterrupt:
        rospy.logwarn("🛑 Interrupted while waiting for websocket loop!")
        exit(0)

    rospy.Subscriber("/cv_camera_node/image_raw", Image, image_callback, queue_size=1)
    rospy.loginfo("✅ Subscribed to /cv_camera_node/image_raw")

    rospy.spin()

    # ✅ เมื่อ ws_loop พร้อมแล้ว ค่อย subscribe ภาพ
    rospy.Subscriber("/cv_camera_node/image_raw", Image, image_callback, queue_size=1)
    rospy.loginfo("✅ Subscribed to /cv_camera_node/image_raw")

    rospy.spin()

