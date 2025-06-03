from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("/home/victor/semilla/yolov5/yolov5n.pt")

# Export the model to ONNX format
model.export(format="onnx")  # creates 'yolo11n.onnx'

# Load the exported ONNX model
onnx_model = YOLO("yolo11n.onnx")
