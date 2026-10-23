#!/usr/bin/env python3
"""YOLO 实例分割 + ByteTrack。只跟随用户点选的那个跟踪 ID。"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from turn_on_wheeltec_robot.msg import Position as PositionMsg

# 与色块跟踪 visualTracker 相同的半视场角正切，用来把像素换成偏角。
TAN_HORIZONTAL = float(np.tan(0.5235987755982988))
TAN_VERTICAL = float(np.tan(0.43196898986859655))

PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>点选尾随</title>
<style>
  body { margin: 0; font-family: sans-serif; background: #111; color: #eee; }
  header { padding: 12px 16px; display: flex; gap: 12px; align-items: center; flex-wrap: wrap; }
  h1 { font-size: 18px; margin: 0; font-weight: 600; }
  button { background: #eee; color: #111; border: 0; border-radius: 6px; padding: 8px 14px; font-size: 14px; }
  #status { font-size: 14px; line-height: 1.4; }
  #view { display: block; max-width: 100%; height: auto; cursor: crosshair; background: #000; }
  .hint { color: #aaa; font-size: 13px; }
</style>
</head>
<body>
<header>
  <h1>点选尾随</h1>
  <button id="unlock" type="button">取消锁定</button>
  <div id="status">正在连接…</div>
</header>
<img id="view" src="/stream.mjpg" alt="相机画面">
<p class="hint">点击要跟的物体。选定之后车子只跟这一个编号，丢失就停车，不会自己换成旁边的目标。</p>
<script>
const view = document.getElementById('view');
const statusEl = document.getElementById('status');
view.addEventListener('click', (event) => {
  const rect = view.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  const x = (event.clientX - rect.left) / rect.width;
  const y = (event.clientY - rect.top) / rect.height;
  fetch('/lock', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ x, y })
  }).then((r) => r.json()).then((data) => {
    statusEl.textContent = data.ok ? data.message : data.error;
  }).catch(() => { statusEl.textContent = '锁定请求失败'; });
});
document.getElementById('unlock').addEventListener('click', () => {
  fetch('/unlock', { method: 'POST' })
    .then((r) => r.json())
    .then((data) => { statusEl.textContent = data.message; });
});
setInterval(() => {
  fetch('/status').then((r) => r.json()).then((data) => {
    statusEl.textContent = data.message;
  }).catch(() => {});
}, 500);
</script>
</body>
</html>
"""


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = b''
        self.width = 640
        self.height = 480
        self.detections = []
        self.locked_id = None
        self.locked_class = ''
        self.mode = 'idle'
        self.fps = 0.0
        self.model_error = ''

    def message(self):
        if self.model_error:
            return self.model_error
        if self.mode == 'tracking':
            return f'正在跟随 {self.locked_class}  #{self.locked_id}    {self.fps:.1f} fps'
        if self.mode == 'nodepth':
            return f'#{self.locked_id} 还在画面里，但没有有效深度，车已停下。'
        if self.mode == 'lost':
            return f'#{self.locked_id} 暂时丢失，车已停下。同一个编号回来会继续跟；出现新编号时请再点一次。'
        return '尚未选定。点击画面中的一个物体。'


def depth_to_mm(depth, u, v):
    if depth is None or depth.size == 0:
        return 0.0
    height, width = depth.shape[:2]
    u = int(np.clip(round(u), 0, width - 1))
    v = int(np.clip(round(v), 0, height - 1))
    half = 3
    roi = depth[max(0, v - half):min(height, v + half + 1), max(0, u - half):min(width, u + half + 1)]
    if np.issubdtype(roi.dtype, np.floating):
        values = roi[np.isfinite(roi)]
        values = values[values > 0.05]
        if values.size == 0:
            return 0.0
        median = float(np.median(values))
        if median < 20.0:
            median *= 1000.0
        return median
    values = roi.astype(np.float32)
    values = values[values > 0]
    if values.size == 0:
        return 0.0
    return float(np.median(values))


def pixel_angle(center, size, tan_half_fov):
    if size <= 1:
        return 0.0
    displacement = 2.0 * float(center) / float(size) - 1.0
    return float(-np.arctan(displacement * tan_half_fov))


def pick_detection(detections, x, y):
    hits = []
    for item in detections:
        polygon = item.get('poly')
        if polygon is not None and len(polygon) >= 3:
            contour = np.asarray(polygon, dtype=np.float32)
            if cv2.pointPolygonTest(contour, (float(x), float(y)), False) >= 0:
                hits.append(item)
                continue
        x1, y1, x2, y2 = item['xyxy']
        if x1 <= x <= x2 and y1 <= y <= y2:
            hits.append(item)
    if not hits:
        return None

    def area(item):
        x1, y1, x2, y2 = item['xyxy']
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    return min(hits, key=area)


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            return

        def _send(self, code, body, content_type):
            data = body if isinstance(body, bytes) else body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/':
                self._send(200, PAGE, 'text/html; charset=utf-8')
                return
            if path == '/status':
                with state.lock:
                    payload = {
                        'mode': state.mode,
                        'locked_id': state.locked_id,
                        'locked_class': state.locked_class,
                        'fps': state.fps,
                        'message': state.message(),
                    }
                self._send(200, json.dumps(payload, ensure_ascii=False), 'application/json; charset=utf-8')
                return
            if path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    while True:
                        with state.lock:
                            frame = state.jpeg
                        if not frame:
                            time.sleep(0.05)
                            continue
                        chunk = (
                            b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                            + str(len(frame)).encode()
                            + b'\r\n\r\n'
                            + frame
                            + b'\r\n'
                        )
                        self.wfile.write(chunk)
                        self.wfile.flush()
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return
                return
            self._send(404, 'not found', 'text/plain; charset=utf-8')

        def do_POST(self):
            path = self.path.split('?', 1)[0]
            length = int(self.headers.get('Content-Length', '0') or 0)
            raw = self.rfile.read(length) if length else b''
            if path == '/unlock':
                with state.lock:
                    state.locked_id = None
                    state.locked_class = ''
                    state.mode = 'idle'
                    message = state.message()
                self._send(200, json.dumps({'ok': True, 'message': message}, ensure_ascii=False),
                           'application/json; charset=utf-8')
                return
            if path != '/lock':
                self._send(404, 'not found', 'text/plain; charset=utf-8')
                return
            try:
                body = json.loads(raw.decode('utf-8') or '{}')
                nx = float(body['x'])
                ny = float(body['y'])
            except (ValueError, KeyError, json.JSONDecodeError):
                self._send(400, json.dumps({'ok': False, 'error': '点击坐标无效'}, ensure_ascii=False),
                           'application/json; charset=utf-8')
                return
            with state.lock:
                width = state.width
                height = state.height
                detections = list(state.detections)
            chosen = pick_detection(detections, nx * width, ny * height)
            if chosen is None or chosen.get('id') is None:
                error = '没有点中带编号的目标，请再点一次'
                self._send(200, json.dumps({'ok': False, 'error': error}, ensure_ascii=False),
                           'application/json; charset=utf-8')
                return
            with state.lock:
                state.locked_id = int(chosen['id'])
                state.locked_class = chosen['cls']
                state.mode = 'tracking'
                message = f"已锁定 {state.locked_class}  #{state.locked_id}"
            self._send(200, json.dumps({'ok': True, 'message': message}, ensure_ascii=False),
                       'application/json; charset=utf-8')

    return Handler


class YoloSameFollow(Node):
    def __init__(self, state):
        super().__init__('yolo_same_follow')
        self.state = state
        self.bridge = CvBridge()
        self.declare_parameter('model', 'yolov8n-seg.engine')
        self.declare_parameter('port', 8090)
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')

        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.tracker_yaml = str(Path(__file__).with_name('bytetrack.yaml'))
        self.model = None
        self._load_model(str(self.get_parameter('model').value))

        self.position_pub = self.create_publisher(PositionMsg, '/object_tracker/current_position', 10)
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], 10, 0.08)
        self.sync.registerCallback(self.on_images)

        self._frame_count = 0
        self._fps_stamp = time.perf_counter()
        port = int(self.get_parameter('port').value)
        self._start_http(port)
        self.get_logger().info(f'点选页面 http://<车IP>:{port}/')

    def _load_model(self, model_path):
        path = Path(model_path).expanduser()
        if not path.is_file():
            self.state.model_error = f'找不到模型 {path}。先在车上运行 export_seg_engine'
            self.get_logger().error(self.state.model_error)
            return
        try:
            from ultralytics import YOLO
            self.model = YOLO(str(path), task='segment')
            self.get_logger().info(f'已加载分割模型 {path}')
        except Exception as exc:
            self.state.model_error = f'模型加载失败: {exc}'
            self.get_logger().error(self.state.model_error)

    def _start_http(self, port):
        handler = make_handler(self.state)
        self.httpd = ThreadingHTTPServer(('0.0.0.0', port), handler)
        thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        thread.start()

    def on_images(self, color_msg, depth_msg):
        try:
            bgr, depth = self._convert(color_msg, depth_msg)
            detections = self._infer(bgr)
            self._publish_locked(detections, bgr, depth)
            self._draw_and_store(bgr, detections)
        except Exception as exc:
            self.get_logger().error(f'处理图像失败: {exc}')
            self._publish_stop()

    def _convert(self, color_msg, depth_msg):
        encoding = color_msg.encoding.lower()
        if 'rgb' in encoding and 'bgr' not in encoding:
            rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        else:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        return bgr, depth

    def _infer(self, bgr):
        if self.model is None:
            return []
        results = self.model.track(
            source=bgr,
            persist=True,
            tracker=self.tracker_yaml,
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False,
        )
        if not results:
            return []
        result = results[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        classes = boxes.cls.cpu().numpy().astype(int) if boxes.cls is not None else np.zeros(len(xyxy), dtype=int)
        if boxes.id is None:
            id_list = [None] * len(xyxy)
        else:
            id_list = [int(value) for value in boxes.id.cpu().numpy().tolist()]
        polygons = [None] * len(xyxy)
        if result.masks is not None and result.masks.xy is not None:
            for index, polygon in enumerate(result.masks.xy):
                if index < len(polygons):
                    polygons[index] = np.asarray(polygon, dtype=np.float32)
        names = result.names or {}
        detections = []
        for index, box in enumerate(xyxy):
            x1, y1, x2, y2 = [float(value) for value in box]
            polygon = polygons[index]
            if polygon is not None and len(polygon) >= 3:
                center = polygon.mean(axis=0)
                cx, cy = float(center[0]), float(center[1])
            else:
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            class_id = int(classes[index])
            detections.append({
                'id': id_list[index],
                'cls': str(names.get(class_id, class_id)),
                'xyxy': (x1, y1, x2, y2),
                'poly': polygon,
                'center': (cx, cy),
            })
        return detections

    def _publish_locked(self, detections, bgr, depth):
        with self.state.lock:
            locked_id = self.state.locked_id
        if locked_id is None:
            with self.state.lock:
                self.state.mode = 'idle'
            self._publish_stop()
            return
        chosen = next((item for item in detections if item['id'] == locked_id), None)
        if chosen is None:
            with self.state.lock:
                self.state.mode = 'lost'
            self._publish_stop()
            return
        cx, cy = chosen['center']
        scale_x = depth.shape[1] / float(bgr.shape[1])
        scale_y = depth.shape[0] / float(bgr.shape[0])
        distance = depth_to_mm(depth, cx * scale_x, cy * scale_y)
        if distance <= 0.0:
            with self.state.lock:
                self.state.mode = 'nodepth'
            self._publish_stop()
            return
        message = PositionMsg()
        message.angle_x = pixel_angle(cx, bgr.shape[1], TAN_HORIZONTAL)
        message.angle_y = pixel_angle(cy, bgr.shape[0], TAN_VERTICAL)
        message.distance = float(distance)
        self.position_pub.publish(message)
        with self.state.lock:
            self.state.mode = 'tracking'
            self.state.locked_class = chosen['cls']

    def _publish_stop(self):
        message = PositionMsg()
        message.angle_x = 0.0
        message.angle_y = 0.0
        message.distance = 0.0
        self.position_pub.publish(message)

    def _draw_and_store(self, bgr, detections):
        canvas = bgr.copy()
        with self.state.lock:
            locked_id = self.state.locked_id
            mode = self.state.mode
        for item in detections:
            x1, y1, x2, y2 = [int(round(value)) for value in item['xyxy']]
            active = item['id'] is not None and item['id'] == locked_id and mode in ('tracking', 'nodepth')
            color = (40, 200, 40) if active else (180, 180, 180)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2 if active else 1)
            label = item['cls'] if item['id'] is None else f"#{item['id']} {item['cls']}"
            cv2.putText(canvas, label, (x1, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        if mode == 'lost':
            cv2.putText(canvas, 'LOST', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (40, 40, 220), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode('.jpg', canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ok:
            return
        now = time.perf_counter()
        self._frame_count += 1
        elapsed = now - self._fps_stamp
        fps = self.state.fps
        if elapsed >= 1.0:
            fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_stamp = now
        with self.state.lock:
            self.state.jpeg = encoded.tobytes()
            self.state.width = canvas.shape[1]
            self.state.height = canvas.shape[0]
            self.state.detections = detections
            self.state.fps = fps

    def destroy_node(self):
        if hasattr(self, 'httpd'):
            self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    state = SharedState()
    node = YoloSameFollow(state)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
