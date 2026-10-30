from ultralytics import YOLO

model = YOLO('yolov8s.pt') 

print("正在导出 TensorRT 引擎，这需要几分钟，请耐心等待...")

# 导出指令
# format='engine': 导出为 TensorRT
# half=True: 开启 FP16 半精度 
# device=0: 使用 GPU
model.export(format='engine', device=0, half=True)

print("导出完成！文件位于当前目录的 yolov8s.engine")
