import asyncio
import websockets
import cv2
import numpy as np
import base64
from io import BytesIO
from PIL import Image
import json
from ultralytics import YOLO

# --- โหลดโมเดล YOLOv8 ของคุณ ---
# ตรวจสอบให้แน่ใจว่าไฟล์ yolov8n.pt อยู่ในโฟลเดอร์เดียวกับ app.py
try:
    model = YOLO('yolov8n.pt')
    print("YOLOv8 model loaded successfully.")
except Exception as e:
    print(f"Error loading YOLOv8 model: {e}")
    exit()


async def handler(websocket, path):
    print("Client connected!")
    try:
        async for message in websocket:
            # 1. รับข้อมูล Base64 และถอดรหัส
            if "," in message:
                header, encoded_data = message.split(",", 1)
            else:
                continue # ข้ามถ้า format ไม่ถูกต้อง

            image_data = base64.b64decode(encoded_data)
            image = Image.open(BytesIO(image_data))
            
            # 2. ประมวลผลด้วย YOLO
            results = model(image, verbose=False) # verbose=False เพื่อไม่ให้แสดง log เยอะเกินไป

            # 3. ดึงข้อมูลผลลัพธ์ที่ต้องการ
            detections = []
            for r in results:
                for box in r.boxes:
                    detections.append({
                        'class': model.names[int(box.cls)],
                        'confidence': float(box.conf),
                        'box': [int(b) for b in box.xyxy[0]] # [x1, y1, x2, y2]
                    })
            
            # 4. ส่งผลลัพธ์กลับไปให้ Electron ในรูปแบบ JSON String
            await websocket.send(json.dumps(detections))

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected.")

async def main():
    host = "localhost"
    port = 8765
    print(f"YOLO WebSocket server started at ws://{host}:{port}")
    async with websockets.serve(handler, host, port, max_size=1024 * 1024 * 2): # เพิ่ม max_size เผื่อภาพขนาดใหญ่
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server shutting down.")