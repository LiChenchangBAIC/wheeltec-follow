#!/usr/bin/env python3

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO


class YoloTrtNode(Node):

    def __init__(self):
        super().__init__('yolo_trt_node')
                                                                           #或yolov8s.pt
        self.engine_path = '/home/wheeltec/wheeltec_arm/src/yolo_tensorrt_pkg/yolov8s.engine'
        self.conf_thres = 0.45
        self.depth_window_size = 5
        self.min_depth_m = 0.05
        self.max_depth_m = 5.0

        self.rgb_topic = '/camera_arm/color/image_raw'
        self.depth_topic = '/camera_arm/depth/image_raw'
        self.camera_info_topic = '/gemini_info'

        self.parent_frame = 'camera_arm_color_optical_frame'
        self.child_frame = 'yolo_object_link'

        self.get_logger().info(f'TensorRT : {self.engine_path} ...')
        self.model = YOLO(self.engine_path, task='detect')
        self.get_logger().info('TensorRT ')

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pub_img = self.create_publisher(Image, '/yolo/result', 10)

        # -------------------- ???? RGB / Depth / CameraInfo --------------------
        self.rgb_sub = Subscriber(
            self, Image, self.rgb_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.info_sub = Subscriber(
            self, CameraInfo, self.camera_info_topic, qos_profile=qos_profile_sensor_data
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.08
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info(
            f'rgb={self.rgb_topic}, depth={self.depth_topic}, camera_info={self.camera_info_topic}'
        )

    def get_depth_in_meters(self, depth_img, depth_encoding, u, v, window_size=5):
        h, w = depth_img.shape[:2]
        half = max(1, window_size // 2)

        x_min = max(0, u - half)
        x_max = min(w, u + half + 1)
        y_min = max(0, v - half)
        y_max = min(h, v + half + 1)

        roi = depth_img[y_min:y_max, x_min:x_max]
        if roi.size == 0:
            return None

        if depth_encoding == '16UC1':
            depth_values = roi.astype(np.float32) / 1000.0
        elif depth_encoding == '32FC1':
            depth_values = roi.astype(np.float32)
        else:
            self.get_logger().error(f'{depth_encoding}')
            return None

        valid = depth_values[np.isfinite(depth_values)]
        valid = valid[(valid > self.min_depth_m) & (valid < self.max_depth_m)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def publish_target_tf(self, stamp, x, y, z):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = float(z)

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)

    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f' {e}')
            return

        start_time = time.time()
        try:
            results = self.model(rgb, conf=self.conf_thres, verbose=False, device=0)
        except Exception as e:
            self.get_logger().error(f'{e}')
            return

        infer_time_ms = (time.time() - start_time) * 1000.0
        fps = 1000.0 / infer_time_ms if infer_time_ms > 0 else 0.0

        result = results[0]
        annotated = result.plot()

        if result.boxes is None or len(result.boxes) == 0:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            out_msg.header = rgb_msg.header
            self.pub_img.publish(out_msg)
            return

        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        nearest_obj = None

        for box in result.boxes:
            cls_id = int(box.cls.item())
            score = float(box.conf.item())
            class_name = self.model.names[cls_id]

            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            u = int(round((x1 + x2) * 0.5))
            v = int(round((y1 + y2) * 0.5))

            depth_m = self.get_depth_in_meters(
                depth, depth_msg.encoding, u, v, self.depth_window_size
            )

            if depth_m is None:
                continue

            x = (u - cx) * depth_m / fx
            y = (v - cy) * depth_m / fy
            z = depth_m

            if nearest_obj is None or depth_m < nearest_obj["depth"]:
                nearest_obj = {
                    "class_name": class_name,
                    "score": score,
                    "u": u,
                    "v": v,
                    "x1": int(round(x1)),
                    "y1": int(round(y1)),
                    "x2": int(round(x2)),
                    "y2": int(round(y2)),
                    "depth": depth_m,
                    "x": x,
                    "y": y,
                    "z": z,
                }

        if nearest_obj is None:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            out_msg.header = rgb_msg.header
            self.pub_img.publish(out_msg)
            # self.get_logger().warn(' TF')
            return

      
        self.publish_target_tf(
            rgb_msg.header.stamp,
            nearest_obj["x"],
            nearest_obj["y"],
            nearest_obj["z"]
        )

        # ??????
        import cv2
        cv2.rectangle(
            annotated,
            (nearest_obj["x1"], nearest_obj["y1"]),
            (nearest_obj["x2"], nearest_obj["y2"]),
            (0, 255, 0),
            2
        )
        cv2.circle(
            annotated,
            (nearest_obj["u"], nearest_obj["v"]),
            4,
            (0, 0, 255),
            -1
        )
        cv2.putText(
            annotated,
            f'nearest {nearest_obj["class_name"]} {nearest_obj["depth"]:.3f}m',
            (nearest_obj["x1"], max(0, nearest_obj["y1"] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        out_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        out_msg.header = rgb_msg.header
        self.pub_img.publish(out_msg)

        self.get_logger().info(
            f'class={nearest_obj["class_name"]}, '
            f'score={nearest_obj["score"]:.3f}, '
            f'pixel=({nearest_obj["u"]}, {nearest_obj["v"]}), '
            f'xyz=({nearest_obj["x"]:.3f}, {nearest_obj["y"]:.3f}, {nearest_obj["z"]:.3f}) m, '
            f'{infer_time_ms:.2f} ms, FPS={fps:.1f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrtNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

