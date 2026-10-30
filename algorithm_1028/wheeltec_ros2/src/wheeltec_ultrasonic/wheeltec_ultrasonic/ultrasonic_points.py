#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import math
import random

def points_to_pointcloud2(points_xy, frame_id, stamp, z=0.0):
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    buffer = []
    for (x, y) in points_xy:
        buffer.append(struct.pack('fff', float(x), float(y), float(z)))

    data = b''.join(buffer)

    cloud = PointCloud2(
        header=header,
        height=1,
        width=len(points_xy),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=12,
        row_step=12 * len(points_xy),
        data=data
    )

    return cloud
    
class UltrasonicPointCloudNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_pointcloud_node')
        #订阅障碍物位置
        self.declare_parameter("use_rear_ultrasonic", False)

        self.declare_parameter("input_topic", "/ultrasonic/obstacles_pos")
        self.declare_parameter("output_topic", "/ultrasonic/points")

        self.declare_parameter("max_range", 1.5)      # 超声波最大可信距离
        self.declare_parameter("min_points", 2)            # 最远时的点数
        self.declare_parameter("max_points", 10)      # 最近时的点数
        self.declare_parameter("spread_radius", 0.05) # 点云扩散半径（米）

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.declare_parameter("frame_id", "obstacles_link")

        self.max_range = self.get_parameter("max_range").get_parameter_value().double_value
        self.min_points = self.get_parameter("min_points").get_parameter_value().integer_value
        self.max_points = self.get_parameter("max_points").get_parameter_value().integer_value
        self.spread_radius = self.get_parameter("spread_radius").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        
        self.use_rear_ultrasonic = self.get_parameter("use_rear_ultrasonic").get_parameter_value().bool_value
        
        self.subscriber_ = self.create_subscription(PoseArray,self.input_topic,self.obstacle_callback,10)

        self.latest_front_msg = None
        self.latest_rear_msg = None
        if self.use_rear_ultrasonic:
            self.rear_sub_ = self.create_subscription(PoseArray, "/rear_ultrasonic/obstacles_pos", self.rear_callback, 10)
            self.get_logger().info('Rear ultrasonic fusion enabled')

        #发布PointCloud2
        self.publisher_ = self.create_publisher(PointCloud2,self.output_topic,10)
        self.get_logger().info('Ultrasonic PointCloud node started')

    def _poses_to_points(self, msg):
        """将 PoseArray 转为散点列表"""
        points = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            r = math.sqrt(x * x + y * y)
            weight = max(0.0, min(1.0, (self.max_range - r) / self.max_range))
            num_points = int(self.min_points + weight * (self.max_points - self.min_points))
            for _ in range(num_points):
                dx = random.uniform(-self.spread_radius, self.spread_radius)
                dy = random.uniform(-self.spread_radius, self.spread_radius)
                points.append((x + dx, y + dy))
        return points

    def obstacle_callback(self, msg: PoseArray):
        self.latest_front_msg = msg
        self._publish_combined(msg.header.stamp)

    def rear_callback(self, msg):
        self.latest_rear_msg = msg
        self._publish_combined(msg.header.stamp)

    def _publish_combined(self, stamp):
        points = []
        if self.latest_front_msg is not None:
            points.extend(self._poses_to_points(self.latest_front_msg))
        if self.use_rear_ultrasonic and self.latest_rear_msg is not None:
            points.extend(self._poses_to_points(self.latest_rear_msg))
        if len(points) == 0:
            return
        cloud = points_to_pointcloud2(points_xy=points, frame_id=self.frame_id, stamp=stamp)
        self.publisher_.publish(cloud)
        
def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPointCloudNode()
    print('ultrasonic to pointcloud node done')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()