import onnx

model = onnx.load("door_detector_best.onnx")
model.ir_version = 9
onnx.save(model, "door_detector_best_v9.onnx")
print("Done! Saved as door_detector_best_v9.onnx")