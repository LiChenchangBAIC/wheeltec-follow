#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math
import yaml
import os
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data


class SimpleRelocalization(Node):
    def __init__(self):
        super().__init__('simple_relocalization')
        
        # 可配置参数
        self.declare_parameter('map_yaml_path', '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.yaml')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('initialpose_topic', '/initialpose')
        
        self.declare_parameter('max_candidates', 1)
        self.declare_parameter('score_threshold', 0.2)
        
        # 获取参数
        self.map_yaml_path = self.get_parameter('map_yaml_path').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        
        self.max_candidates = self.get_parameter('max_candidates').value
        self.score_threshold = self.get_parameter('score_threshold').value
        
        # 状态变量
        self.map_data = None
        self.map_image = None
        self.map_origin = None
        self.map_resolution = None
        self.map_width = 0
        self.map_height = 0
        
        self.current_scan = None
        self.scan_received = False
        
        self.relocalization_done = False
        
        # 创建发布者和订阅者
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data

        )
        
        # 加载地图
        self.load_map()
        
        self.get_logger().info('simple relocalization node started')
        self.get_logger().info('waiting for laser scan data...')
    
    def load_map(self):
        """加载地图文件（map.yaml和map.pgm）"""
        try:
            if not os.path.exists(self.map_yaml_path):
                self.get_logger().error(f'map yaml file not found: {self.map_yaml_path}')
                self.get_logger().info('please set map_yaml_path parameter correctly')
                return False
            
            with open(self.map_yaml_path, 'r') as f:
                map_config = yaml.safe_load(f)
            
            self.map_resolution = map_config['resolution']
            self.map_origin = map_config['origin']
            
            pgm_path = os.path.join(os.path.dirname(self.map_yaml_path), map_config['image'])
            
            if not os.path.exists(pgm_path):
                self.get_logger().error(f'map pgm file not found: {pgm_path}')
                return False
            
            self.map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if self.map_image is None:
                self.get_logger().error(f'failed to read map image: {pgm_path}')
                return False
            
            self.map_height, self.map_width = self.map_image.shape
            
            # 二值化地图（自由区域=255，障碍物=0）
            _, self.map_binary = cv2.threshold(self.map_image, 200, 255, cv2.THRESH_BINARY)
            
            # 提取地图边缘
            self.map_edges = cv2.Canny(self.map_binary, 50, 150)
            
            self.get_logger().info(f'map loaded successfully')
            self.get_logger().info(f'map size: {self.map_width}x{self.map_height}')
            self.get_logger().info(f'resolution: {self.map_resolution} m/pixel')
            self.get_logger().info(f'origin: {self.map_origin}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'error loading map: {e}')
            return False
    
    def scan_callback(self, msg):
        """接收激光扫描数据"""
        #self.get_logger().info('performing simple relocalization...')
        if self.relocalization_done:
            return
        
        self.current_scan = msg
        self.scan_received = True
        
        # 如果地图已加载，开始处理
        if self.map_image is not None:
            self.perform_relocalization()
    
    def perform_relocalization(self):
        """执行简单重定位"""
        self.get_logger().info('performing simple relocalization...')
        
        # 生成候选位姿（只取匹配分数最高的）
        candidates = self.match_scan_with_map(self.current_scan)
        
        if len(candidates) == 0:
            self.get_logger().error('no candidates found')
            return
        
        # 取分数最高的
        best_candidate = candidates[0]
        x, y, yaw_rad, yaw_deg, score = best_candidate
        
        self.get_logger().info(f'found best pose: x={x:.2f}m, y={y:.2f}m, yaw={yaw_deg}°, score={score:.3f}')
        
        # 发布这个位姿
        self.publish_pose(x, y, yaw_rad)
        
        self.relocalization_done = True
        self.get_logger().info('relocalization done!')
    
    def calculate_match_score(self, scan_msg, map_px, map_py, angle):
        """计算激光扫描在指定位置和角度的匹配分数"""
        ranges = np.array(scan_msg.ranges)
        ranges = np.clip(ranges, scan_msg.range_min, scan_msg.range_max)
        
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        angles += angle
        
        valid_mask = (ranges < scan_msg.range_max) & (ranges > scan_msg.range_min)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        matches = 0
        total = 0
        
        for r, theta in zip(valid_ranges, valid_angles):
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue
            
            # 计算激光点在地图上的像素坐标
            local_x = r * np.cos(theta)
            local_y = r * np.sin(theta)
            
            px = int(map_px + local_x / self.map_resolution)
            py = int(map_py - local_y / self.map_resolution)
            
            total += 1
            
            # 检查该点是否靠近地图边缘
            if 0 <= px < self.map_width and 0 <= py < self.map_height:
                # 检查周围区域
                check_size = 3
                region = self.map_edges[
                    max(0, py - check_size):min(self.map_height, py + check_size + 1),
                    max(0, px - check_size):min(self.map_width, px + check_size + 1)
                ]
                if np.any(region > 0):
                    matches += 1
        
        return matches / max(total, 1)
    
    def match_scan_with_map(self, scan_msg):
        """将激光扫描与地图进行匹配，找到候选位置"""
        import time
        
        candidates = []
        
        if self.map_edges is None or scan_msg is None:
            return candidates
        
        # 在地图上采样候选位置
        step = 20  # 像素步长
        angle_step_deg = 30  # 角度步长
        self.get_logger().info(f'sampling params: step={step}px, angle_step={angle_step_deg}°')
        
        # 开始计时
        start_time = time.time()
        
        for y in range(step, self.map_height - step, step):
            for x in range(step, self.map_width - step, step):
                # 检查该位置是否为自由空间
                if self.map_binary[y, x] < 200:
                    continue
                
                # 转换为地图坐标系
                map_x = (x * self.map_resolution) + self.map_origin[0]
                map_y = ((self.map_height - y - 1) * self.map_resolution) + self.map_origin[1]
                
                # 尝试多个角度
                for angle_deg in range(0, 360, angle_step_deg):
                    angle_rad = math.radians(angle_deg)
                    
                    # 计算匹配分数
                    score = self.calculate_match_score(scan_msg, x, y, angle_rad)
                    
                    if score > self.score_threshold:
                        candidates.append((map_x, map_y, angle_rad, angle_deg, score))
        
        # 结束计时
        end_time = time.time()
        total_time = end_time - start_time
        self.get_logger().info(f'total time: {total_time:.3f}s')
        
        # 按分数排序，取前N个
        candidates.sort(key=lambda c: c[4], reverse=True)
        candidates = candidates[:self.max_candidates]
        
        if len(candidates) > 0:
            self.get_logger().info(f'top {len(candidates)} candidates:')
            for i, (x, y, yaw_rad, yaw_deg, score) in enumerate(candidates):
                self.get_logger().info(f'{i+1}: x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°, score={score:.3f}')
        
        return candidates
    
    def publish_pose(self, x, y, yaw_rad):
        """发布位姿"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # 较大的协方差，让AMCL可以自由收敛
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info('pose published!')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRelocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nuser interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

