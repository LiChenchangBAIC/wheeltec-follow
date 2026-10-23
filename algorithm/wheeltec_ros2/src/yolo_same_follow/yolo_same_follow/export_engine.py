#!/usr/bin/env python3
"""在 Jetson 上把分割权重导出成 TensorRT FP16 engine。必须在车上执行。"""

import argparse


def main():
    parser = argparse.ArgumentParser(description='导出 yolov8n-seg TensorRT engine')
    parser.add_argument('--weights', default='yolov8n-seg.pt', help='pt 权重，不存在时 Ultralytics 会尝试下载')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.weights)
    exported = model.export(format='engine', half=True, imgsz=args.imgsz, device=args.device)
    print(exported)


if __name__ == '__main__':
    main()
