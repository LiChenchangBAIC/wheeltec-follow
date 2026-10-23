#!/usr/bin/env python3
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_localization_msgs.srv import InitializeLocalization


class GnssInitializer(Node):
    def __init__(self):
        super().__init__('gnss_initializer')
        self.latest_gnss_pose_ = None
        self.latest_ndt_pose_ = None       # 第一步 NDT 对齐后的位姿
        self.gnss_pose_for_step2_ = None   # 保存 GNSS 航向给第二步
        self.prev_pose_ = None             # 用于稳定性检测
        self.request_in_flight_ = False
        self.initialized_      = False   # 成功后不再重试

        gnss_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        self.gnss_sub_ = self.create_subscription(PoseWithCovarianceStamped,'/sensing/gnss/pose_with_covariance',self.gnss_callback,gnss_qos)
        self.ndt_sub_ = self.create_subscription(PoseWithCovarianceStamped,'/initialpose3d',self.ndt_callback,gnss_qos)
        self.cli_ = self.create_client(InitializeLocalization,'/localization/initialize')
        self.timer_ = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info(
            'GnssInitializer started. '
            'Will send DIRECT init as soon as valid GNSS pose and service are ready...'
        )

    def gnss_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_gnss_pose_ = msg

    def ndt_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_ndt_pose_ = msg

    def is_valid_orientation(self, ori) -> bool:
        if abs(ori.x) < 1e-6 and abs(ori.y) < 1e-6 and abs(ori.z) < 1e-6:
            self.get_logger().warn(
                'GNSS orientation is default (0,0,0,1), '
                'heading not ready yet, waiting...'
            )
            return False
        norm = math.sqrt(ori.x**2 + ori.y**2 + ori.z**2 + ori.w**2)
        if abs(norm - 1.0) > 0.01:
            self.get_logger().warn(
                f'GNSS orientation norm={norm:.4f} invalid, waiting...'
            )
            return False
        if abs(ori.x) > 0.05 or abs(ori.y) > 0.05:
            self.get_logger().warn(
                f'GNSS orientation not horizontal: x={ori.x:.4f}, y={ori.y:.4f}, '
                'waiting for valid heading...'
            )
            return False
        return True

    def is_valid_position(self, position) -> bool:
        dist = math.sqrt(position.x**2 + position.y**2)
        if dist < 1.0:
            self.get_logger().warn(f'GNSS position too close to origin: dist={dist:.2f}m')
            return False
        # Z 值异常检查（正常在 ±10m 以内）
        if abs(position.z) > 10.0:
            self.get_logger().warn(f'GNSS Z abnormal: z={position.z:.2f}m')
            return False
        return True

    def is_stable(self, new_pose) -> bool:
        # 首次接收，记录并等待下一帧确认
        if self.prev_pose_ is None:
            self.prev_pose_ = new_pose
            self.get_logger().info('First GNSS pose received, waiting for stability...')
            return False
        dx = new_pose.pose.pose.position.x - self.prev_pose_.pose.pose.position.x
        dy = new_pose.pose.pose.position.y - self.prev_pose_.pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        if dist > 3.0:
            self.prev_pose_ = new_pose
            self.get_logger().warn(f'GNSS not stable yet, delta={dist:.2f}m, waiting...')
            return False
        return True

    def timer_callback(self):
        if self.initialized_ or self.request_in_flight_:
            return

        if self.latest_gnss_pose_ is None:
            self.get_logger().info('Waiting for /sensing/gnss/pose_with_covariance...')
            return

        if not self.cli_.service_is_ready():
            self.get_logger().info('Waiting for /localization/initialize service...')
            return

        # 检查航向有效性
        ori = self.latest_gnss_pose_.pose.pose.orientation
        if not self.is_valid_orientation(ori):
            return

        # 检查位置有效性
        pos = self.latest_gnss_pose_.pose.pose.position
        if not self.is_valid_position(pos):
            return

        yaw_var = self.latest_gnss_pose_.pose.covariance[35]
        if yaw_var > 0.1:
            self.get_logger().warn(
                f'GNSS yaw variance={yaw_var:.6f} > 0.1, '
                'waiting for better GNSS fix...'
            )
            return

        if not self.is_stable(self.latest_gnss_pose_):
            return

        self.send_initialize_request(self.latest_gnss_pose_)


    def send_initialize_request(self, gnss_pose: PoseWithCovarianceStamped):
        self.gnss_pose_for_step2_ = gnss_pose  # 保存 GNSS 航向
        self.send_step1_auto(gnss_pose)

    def send_step1_auto(self, gnss_pose):
        pose_for_ndt = copy.deepcopy(gnss_pose)
        cov = pose_for_ndt.pose.covariance
        cov[0] = cov[7] = 10.0    # XY 宽松
        cov[14] = 0.1              # Z 紧
        cov[21] = cov[28] = 0.01   # roll/pitch 紧

        req = InitializeLocalization.Request()
        req.pose_with_covariance.append(pose_for_ndt)
        req.method = InitializeLocalization.Request.AUTO

        self.get_logger().info('Step 1: AUTO init (NDT position)...')
        self.request_in_flight_ = True
        future = self.cli_.call_async(req)
        future.add_done_callback(self.step1_callback)

    def step1_callback(self, future):
        self.request_in_flight_ = False
        try:
            response = future.result()
            if not response.status.success:
                self.get_logger().error(
                    f'Step 1 FAILED: code={response.status.code} msg="{response.status.message}"')
                return
        except Exception as e:
            self.get_logger().error(f'Step 1 exception: {e}')
            return

        self.get_logger().info('Step 1 OK. Step 2: DIRECT with GNSS heading...')
        self.send_step2_direct()

    def send_step2_direct(self):
        if self.latest_ndt_pose_ is None:
            self.get_logger().error('No NDT pose received, cannot apply GNSS heading')
            return

        hybrid_pose = copy.deepcopy(self.latest_ndt_pose_)
        gnss_ori = self.gnss_pose_for_step2_.pose.pose.orientation
        hybrid_pose.pose.pose.orientation = gnss_ori

        req = InitializeLocalization.Request()
        req.pose_with_covariance.append(hybrid_pose)
        req.method = InitializeLocalization.Request.DIRECT

        self.request_in_flight_ = True
        future = self.cli_.call_async(req)
        future.add_done_callback(self.step2_callback)

    def step2_callback(self, future):
        self.request_in_flight_ = False
        try:
            response = future.result()
            if response.status.success:
                self.initialized_ = True
                self.timer_.cancel()
                self.get_logger().info(
                    'Step 2 SUCCESS: NDT position + GNSS heading applied. Timer stopped.')
            else:
                self.get_logger().error(
                    f'Step 2 FAILED: code={response.status.code} msg="{response.status.message}"')
        except Exception as e:
            self.get_logger().error(f'Step 2 exception: {e}')


def main():
    rclpy.init()
    node = GnssInitializer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()