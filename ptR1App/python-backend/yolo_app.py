import asyncio
import websockets
import json
import base64
import io
import cv2
import numpy as np
from PIL import Image
from ultralytics import YOLO
from concurrent.futures import ThreadPoolExecutor

# โหลดโมเดล
try:
    # แนะนำให้โหลดไปที่ GPU ถ้ามี (device='cuda') หรือ 'cpu'
    model = YOLO('yolov8n.pt') 
    print("YOLOv8 model loaded successfully.")
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

# สร้าง ThreadPool สำหรับรัน YOLO (เพื่อไม่ให้ขวาง WebSocket Loop)
executor = ThreadPoolExecutor(max_workers=1)

def run_inference(image_data):
    """ฟังก์ชันสำหรับรัน YOLO (Blocking Function)"""
    try:
        # แปลง Base64 เป็นรูปภาพ
        image = Image.open(io.BytesIO(image_data))
        
        # รัน YOLO
        results = model(image, verbose=False)
        
        # ดึงข้อมูล
        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                name = model.names[cls]
                
                detections.append({
                    "box": [int(x1), int(y1), int(x2), int(y2)],
                    "confidence": round(conf, 2),
                    "class": name
                })
        return detections
    except Exception as e:
        print(f"Inference Error: {e}")
        return []

async def handler(websocket):
    print(f"Client connected: {websocket.remote_address}")
    try:
        async for message in websocket:
            # 1. จัดการ Base64 Header (Data URI Scheme)
            # รองรับทั้งแบบมี "data:image..." และแบบไม่มี
            if isinstance(message, str):
                if "," in message:
                    _, encoded_data = message.split(",", 1)
                else:
                    encoded_data = message
            else:
                # กรณีส่งมาเป็น bytes (เผื่ออนาคต)
                encoded_data = message

            try:
                # แปลง string -> bytes
                image_bytes = base64.b64decode(encoded_data)

                # 2. ส่งงาน YOLO ไปทำใน Thread แยก (Non-blocking)
                loop = asyncio.get_running_loop()
                detections = await loop.run_in_executor(executor, run_inference, image_bytes)

                # 3. ส่งผลลัพธ์กลับ
                await websocket.send(json.dumps(detections))

            except Exception as e:
                print(f"Processing Error: {e}")
                # ส่ง array ว่างกลับไปกัน Client ค้าง
                await websocket.send(json.dumps([]))

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected.")
    except Exception as e:
        print(f"Connection Error: {e}")

async def main():
    host = "0.0.0.0" 
    port = 8765
    print(f"YOLO Server running on ws://{host}:{port}")
    
    # เพิ่ม max_size เป็น 10MB เผื่อภาพความละเอียดสูง
    async with websockets.serve(handler, host, port, max_size=10*1024*1024):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped.")