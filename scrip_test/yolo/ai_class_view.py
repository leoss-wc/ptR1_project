from ultralytics import YOLO

model = YOLO("door_detector_best_v9.onnx")
print(model.names)