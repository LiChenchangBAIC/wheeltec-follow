#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseArray, Pose

class RearUltrasonicSolver(Node):

    def __init__(self):
        super().__init__("rear_ultrasonic_solver")
        # 后方两个超声波安装位置
        self.sensor_x = [0.14, -0.14]
        self.sensor_y = [0.0, 0.0]
        self.uldis = [None] * 2
        self.data_ready = [False] * 2
        self.uldis_max_range = 1.5
        self.single_sensor_penalty = 0.08

        self.declare_parameter("sensor_x", self.sensor_x)
        self.declare_parameter("sensor_y", self.sensor_y)
        self.declare_parameter("uldis_max_range", 1.8)
        self.declare_parameter("single_sensor_penalty", 0.08)

        self.sensor_x = self.get_parameter("sensor_x").value
        self.sensor_y = self.get_parameter("sensor_y").value
        self.uldis_max_range = self.get_parameter("uldis_max_range").value
        self.single_sensor_penalty = self.get_parameter("single_sensor_penalty").value

        topics = ['ultrasonic_data_E','ultrasonic_data_F',]

        for idx, topic in enumerate(topics):
            self.create_subscription(Range,topic,lambda msg, i=idx: self.range_callback(msg, i),10)

        self.pos_pub = self.create_publisher(PoseArray,"/rear_ultrasonic/obstacles_pos",10)
        self.create_timer(0.1, self.process)
        self.get_logger().info("Rear ultrasonic solver started")

    def range_callback(self, msg, idx):

        if msg.range < self.uldis_max_range:
            self.uldis[idx] = msg.range
            self.data_ready[idx] = True
        else:
            self.uldis[idx] = None

    def enumerate_candidates(self, valid_data):
        candidates = []
        if len(valid_data) == 1:
            candidates.append([valid_data])
            return candidates
        s0 = [valid_data[0]]
        s1 = [valid_data[1]]
        # 左
        candidates.append([s0])
        # 右
        candidates.append([s1])
        # 同一障碍
        candidates.append([valid_data])
        # 两个障碍
        candidates.append([s0, s1])
        return candidates

    def process(self):
        if not any(self.data_ready):
            return
        valid_data = []
        for i, r in enumerate(self.uldis):
            if r is not None:
                valid_data.append((i, r))
        if len(valid_data) == 0:
            return
        all_candidates = self.enumerate_candidates(valid_data)
        best_solution = None
        best_score = float("inf")
        for candidate in all_candidates:
            solution = []
            total_error = 0.0
            valid = True
            
            
            for cluster in candidate:
                sensors = [i for i, _ in cluster]
                pos, err = self.solve_cluster(cluster)
                if pos is None:
                    valid = False
                    break
                    
                if pos[0] > 0.0 or pos[0] < -3.0 or abs(pos[1]) > 0.3:
                    continue
                solution.append((pos[0], pos[1], cluster))
                total_error += err
                if len(cluster) == 1:
                    total_error += self.single_sensor_penalty
            if not valid:
                continue
            #self.get_logger().info( f"pos=({ pos[0]:.2f},{ pos[1]:.2f}), sensors={sensors}")
            score = self.evaluate_solution(solution, total_error)
            if score < best_score:
                best_score = score
                best_solution = solution
        if best_solution is None:
            return

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for x, y, cluster in best_solution:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pos_pub.publish(msg)

    def evaluate_solution(self, solution, total_error):
        score = total_error
        cluster_num = len(solution)
        if cluster_num > 1:
            score += 0.05 * (cluster_num - 1)
        return score

    def solve_cluster(self, cluster):
        #用最小二乘法解算一个cluster 的障碍物位置
        #cluster: [(idx, range), ...]
        #返回:
        #    pos: (x, y)
        #    error: 总误差
        A = []
        b = []
    
        for idx, r in cluster:
            xi = self.sensor_x[idx]
            # 传感器坐标：yi=0
            # 方程： (x - xi)^2 + y^2 = r^2
            # 化成线性形式：
            # x^2 + y^2 - 2*xi*x + xi^2 = r^2
            # 2*xi*x - (x^2+y^2) = xi^2 - r^2
            # 设 z = x^2 + y^2
            # 2*xi*x - z = xi^2 - r^2
            A.append([2 * xi, -1])
            b.append([xi * xi - r * r])
        A = np.array(A, dtype=np.float64)
        b = np.array(b, dtype=np.float64)
        #解最小二乘
        try:
            sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except Exception as e:
            return None, None
    
        x = sol[0, 0]
        z = sol[1, 0]
        y_sq = z - x * x

        if y_sq < 0:
            return None, None
        y = np.sqrt(y_sq)
        #计算误差
        error = 0.0
        for idx, r in cluster:
            xi = self.sensor_x[idx]
            err = abs(np.sqrt((x - xi) ** 2 + y ** 2) - r)
            error += err
        return (-y, x), error #切换坐标系

def main(args=None):
    rclpy.init(args=args)
    node = RearUltrasonicSolver()
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