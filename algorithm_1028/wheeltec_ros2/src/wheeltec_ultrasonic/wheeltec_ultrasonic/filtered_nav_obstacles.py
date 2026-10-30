#!/usr/bin/env python3
"""Navigation-only laser and six-channel ultrasonic obstacle filtering."""
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Range
from std_msgs.msg import Header

from wheeltec_ultrasonic.ultrasonic_interlock import FrontUltrasonicInterlock


class RearLaserFilter:
    """Filter confirmed rear body returns and debounce near rear obstacles."""

    def __init__(self, self_filter_m=.40, stop_distance_m=.45, half_angle_deg=30.):
        if not .35 <= self_filter_m < stop_distance_m <= .60:
            raise ValueError('Invalid rear laser distances')
        if not 0 < half_angle_deg <= 90:
            raise ValueError('Invalid rear laser sector')
        self.self_filter_m = self_filter_m
        self.stop_distance_m = stop_distance_m
        self.half_angle = math.radians(half_angle_deg)
        self.states = {}

    def process(self, ranges, angle_min, angle_increment):
        result = list(ranges)
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            rear_angle = abs(math.atan2(math.sin(angle-math.pi), math.cos(angle-math.pi)))
            if rear_angle > self.half_angle:
                continue
            state = self.states.setdefault(i, dict(near=0, far=0, blocked=False, last=.0))
            is_self = math.isfinite(distance) and 0 < distance <= self.self_filter_m
            is_near = (math.isfinite(distance) and self.self_filter_m < distance
                       < self.stop_distance_m)
            is_far = ((math.isinf(distance) and distance > 0)
                      or (math.isfinite(distance) and distance >= self.stop_distance_m))
            if is_near:
                state['near'] += 1
                state['far'] = 0
                state['last'] = float(distance)
                if state['near'] >= 3:
                    state['blocked'] = True
                result[i] = float(distance) if state['blocked'] else float('nan')
            elif is_self or is_far:
                state['near'] = 0
                state['far'] += 1
                if state['blocked'] and state['far'] < 3:
                    result[i] = state['last']
                else:
                    state['blocked'] = False
                    result[i] = float('nan') if is_self else distance
            else:
                # Unknown data cannot confirm clearance or erase a confirmed obstacle.
                state['near'] = state['far'] = 0
                result[i] = state['last'] if state['blocked'] else float('nan')
        return result

    @property
    def blocked_count(self):
        return sum(s['blocked'] for s in self.states.values())


class UltrasonicSafety:
    """Apply the voice three-frame rules independently to each channel."""

    def __init__(self, stop_distance_m=.45):
        self.interlocks = {
            c: FrontUltrasonicInterlock(stop_distance_m, despike_frames=2,
                                       channels=c, direction='front' if c in 'ABCD' else 'rear')
            for c in 'ABCDEF'
        }

    def update(self, channel, distance, stamp, now, minimum=.02, maximum=5.3):
        self.interlocks[channel].update(channel, distance, stamp, now, minimum, maximum)

    def distance_or_veto(self, channel, now):
        status = self.interlocks[channel].status(now)
        if status['clear']:
            return None, status
        if status['reason'].endswith('_obstacle'):
            distance = status['filtered_m'].get(channel)
            if distance is None:
                distance = status['raw_latest_m'].get(channel, .02)
            return max(.02, min(.45, float(distance))), status
        # Missing, stale, invalid or not-yet-stable data fails closed.
        return .02, status


def points_to_pointcloud2(points, frame_id, stamp):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    data = b''.join(struct.pack('fff', *point) for point in points)
    return PointCloud2(header=Header(stamp=stamp, frame_id=frame_id),
                       height=1, width=len(points), is_dense=True, is_bigendian=False,
                       fields=fields, point_step=12, row_step=12*len(points), data=data)


class NavigationObstacleFilter(Node):
    def __init__(self):
        super().__init__('nav_obstacle_filter')
        parameters = {
            'input_scan': '/scan',
            'output_scan': '/nav/scan_filtered',
            'output_cloud': '/nav/ultrasonic/points',
            'cloud_frame': 'base_link',
            'rear_self_filter_m': .40,
            'rear_stop_distance_m': .45,
            'rear_sector_half_angle_deg': 30.,
            'ultrasonic_stop_distance_m': .45,
            'front_sensor_x': .10,
            'front_sensor_lateral': [-.205, -.065, .065, .205],
            'rear_sensor_x': -.30,
            'rear_sensor_lateral': [-.12, .12],
            'sensor_z': .15,
            'publish_period_s': .05,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        if value('ultrasonic_stop_distance_m') != .45:
            raise ValueError('The shared three-frame interlock requires 0.45 m')
        if len(value('front_sensor_lateral')) != 4 or len(value('rear_sensor_lateral')) != 2:
            raise ValueError('Four front and two rear ultrasonic positions are required')
        if not .02 <= value('publish_period_s') <= .2:
            raise ValueError('Invalid ultrasonic publication period')
        self.laser_filter = RearLaserFilter(value('rear_self_filter_m'),
                                            value('rear_stop_distance_m'),
                                            value('rear_sector_half_angle_deg'))
        self.ultrasonic = UltrasonicSafety(value('ultrasonic_stop_distance_m'))
        self.output_cloud = value('output_cloud')
        self.cloud_frame = value('cloud_frame')
        self.front_x = value('front_sensor_x')
        self.rear_x = value('rear_sensor_x')
        self.sensor_z = value('sensor_z')
        self.lateral = dict(zip('ABCD', value('front_sensor_lateral')))
        self.lateral.update(zip('EF', value('rear_sensor_lateral')))
        self.scan_pub = self.create_publisher(LaserScan, value('output_scan'), 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_cloud, 10)
        self.scan_sub = self.create_subscription(LaserScan, value('input_scan'),
                                                 self.scan_callback, qos_profile_sensor_data)
        self.range_subs = [
            self.create_subscription(Range, '/ultrasonic_data_'+channel,
                                     lambda m, c=channel: self.range_callback(c, m),
                                     qos_profile_sensor_data)
            for channel in 'ABCDEF'
        ]
        self.timer = self.create_timer(value('publish_period_s'), self.publish_ultrasonic)
        self.last_point_count = None
        self.get_logger().info('Navigation obstacle filter ready: laser rear 0.40/0.45 m, '
                               'six ultrasonic channels 0.45 m with three-frame confirmation')

    def range_callback(self, channel, msg):
        now = time.monotonic()
        clock = self.get_clock().now().nanoseconds / 1e9
        sensor_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = clock-sensor_time
        stamp = now-age if sensor_time and -.01 <= age <= .5 else float('nan')
        self.ultrasonic.update(channel, msg.range, stamp, now, msg.min_range, msg.max_range)

    def scan_callback(self, msg):
        msg.ranges = self.laser_filter.process(msg.ranges, msg.angle_min, msg.angle_increment)
        self.scan_pub.publish(msg)

    def ultrasonic_points(self, now):
        points = []
        for channel in 'ABCDEF':
            distance, status = self.ultrasonic.distance_or_veto(channel, now)
            if distance is None:
                continue
            x = self.front_x+distance if channel in 'ABCD' else self.rear_x-distance
            points.append((x, self.lateral[channel], self.sensor_z))
        return points

    def publish_ultrasonic(self):
        points = self.ultrasonic_points(time.monotonic())
        self.cloud_pub.publish(points_to_pointcloud2(points, self.cloud_frame,
                                                     self.get_clock().now().to_msg()))
        if len(points) != self.last_point_count:
            self.get_logger().info('Filtered ultrasonic obstacle points: %d' % len(points))
            self.last_point_count = len(points)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationObstacleFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
