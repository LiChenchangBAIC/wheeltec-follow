#!/usr/bin/env python3

import math
from tf_transformations import euler_from_quaternion


def deg2rad(deg):
    return deg * math.pi / 180.0

def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def calculate_yaw_offset(
        phone_heading_deg,
        qx, qy, qz, qw):
    roll, pitch, yaw_imu = euler_from_quaternion(
        [qx, qy, qz, qw]
    )
    heading_rad = deg2rad(phone_heading_deg)

    yaw_expected = math.pi / 2.0 - heading_rad

    yaw_expected = normalize_angle(yaw_expected)

    yaw_offset = yaw_expected - yaw_imu

    yaw_offset = normalize_angle(yaw_offset)

    return yaw_offset, yaw_imu, yaw_expected


if __name__ == "__main__":

    phone_heading_deg = float(
        input("请输入手机指南针角度(北=0 东=90): ")
    )
    print("\n请输入IMU orientation quaternion:\n")

    qx = float(input("x: "))
    qy = float(input("y: "))
    qz = float(input("z: "))
    qw = float(input("w: "))

    yaw_offset, yaw_imu, yaw_expected = calculate_yaw_offset(
        phone_heading_deg,
        qx, qy, qz, qw
    )

    print(f"IMU yaw               : {yaw_imu:.6f} rad")
    print(f"Expected ENU yaw      : {yaw_expected:.6f} rad")

    print("\n参数配置:\n")
    print(f"yaw_offset: {yaw_offset:.6f}\n")
