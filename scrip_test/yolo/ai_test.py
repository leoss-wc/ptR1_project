from ultralytics import YOLO
import os

model = YOLO("door_detector_best_v9.onnx")
test_folder = "./door_open7mSpotLight/"
tp = fp = fn = 0
ground_truth = "door_open"

for img_path in os.listdir(test_folder):
    results = model(f"{test_folder}/{img_path}", conf=0.5)
    detected = [model.names[int(c)] for c in results[0].boxes.cls]
    print(f"[{img_path}] Detected: {detected if detected else 'None'}")
    
    if ground_truth in detected:
        tp += 1  # เจอถูก
    elif detected:
        fp += 1  # เจอแต่ผิด class
    else:
        fn += 1  # ไม่เจอเลย

print(f"\nTP={tp}, FP={fp}, FN={fn}")