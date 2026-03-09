from ultralytics import YOLO
import os

model = YOLO("door_detector_best.onnx")
test_folder = "test_images/bright_1m/"  # ใส่ภาพทดสอบ

tp = fp = fn = 0
ground_truth = "door_open"  # class จริงของภาพชุดนี้

for img_path in os.listdir(test_folder):
    results = model(f"{test_folder}/{img_path}", conf=0.5)
    
    detected = [model.names[int(c)] 
                for c in results[0].boxes.cls]
    
    if ground_truth in detected:
        tp += 1  # เจอถูก
    elif detected:
        fp += 1  # เจอแต่ผิด class
    else:
        fn += 1  # ไม่เจอเลย

print(f"TP={tp}, FP={fp}, FN={fn}")