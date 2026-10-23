#!/usr/bin/env python3
import rclpy
import message_filters
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Range
import math
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
import time
from itertools import combinations
import itertools
from geometry_msgs.msg import PoseArray, Pose

class UltrasonicEnumSolver(Node):
    def __init__(self):
        super().__init__('ultrasonic_enum_solver')
        queue_size = 30

        self.uldis = [10.0] * 4 #保存最近一次测距
        self.data_ready= [False] * 4
        #超声波安装位置（前方一排）
        self.declare_parameter("sensor_x",[-0.205, -0.065, 0.065, 0.205])
        self.declare_parameter("sensor_y",0.0)

        # 算法参数
        self.declare_parameter("uldis_max_range", 1.8)
        self.declare_parameter("single_sensor_penalty", 0.1)
        self.declare_parameter("cluster_count_penalty", 0.05) # 每个cluster的惩罚分，抑制过度拆分
        self.declare_parameter("merge_threshold", 0.05) # 5cm内认为是同一障碍物，触发合并
        self.declare_parameter("outlier_residual", 0.15) # 残差>0.15m判定为离群传感器


        self.sensor_x = self.get_parameter("sensor_x").value
        self.sensor_y = self.get_parameter("sensor_y").value
        self.uldis_max_range = self.get_parameter("uldis_max_range").value
        self.single_sensor_penalty = self.get_parameter("single_sensor_penalty").value
        self.cluster_count_penalty = self.get_parameter("cluster_count_penalty").value
        self.merge_threshold = self.get_parameter("merge_threshold").value
        self.outlier_residual = self.get_parameter("outlier_residual").value
        
        self.get_logger().info( f" sensor_x=({self.sensor_x})")
        self.get_logger().info( f" sensor_y=({self.sensor_y})")
        self.get_logger().info( f" uldis_max_range=({self.uldis_max_range})")
        self.get_logger().info( f" cluster_count_penalty=({self.cluster_count_penalty})")
        self.get_logger().info( f" merge_threshold=({self.merge_threshold})")
        self.get_logger().info( f" outlier_residual=({self.outlier_residual})")


        topics = ['ultrasonic_data_A','ultrasonic_data_B','ultrasonic_data_C','ultrasonic_data_D',]
        for idx,topic in enumerate(topics):
            self.create_subscription(Range,topic,lambda msg, i=idx:self.range_callback(msg, i),10)
            
        self.pos_pub = self.create_publisher( PoseArray,'/ultrasonic/obstacles_pos',10)
        self.create_timer(0.1, self.process)
        self.get_logger().info('Ultrasonic enum node started')

    def range_callback(self, msg, idx):
        if msg.range < self.uldis_max_range:
            self.uldis[idx] = msg.range
            self.data_ready[idx] = True
        else:
            self.uldis[idx] = None
    def solve_best_subset(self, cluster):
        n = len(cluster)
        if n < 2:
            return None, None, None
        # 用全量LS初算，逐传感器算残差，超过阈值的判为离群
        inlier_cluster = cluster
        if n >= 3:
            pos_all, _ = self.solve_cluster(cluster)
            if pos_all is not None:
                y_obs, x_obs = pos_all
                inlier_cluster = []
                for idx, r in cluster:
                    xi = self.sensor_x[idx]
                    pred = math.sqrt((x_obs - xi) ** 2 + y_obs ** 2)
                    residual = abs(pred - r)
                    if residual <= self.outlier_residual:
                        inlier_cluster.append((idx, r))
                if len(inlier_cluster) < 2:
                    inlier_cluster = cluster  # 剔除过多则退回原集
        # 离群已剔除，剩余传感器间的小偏差由LS加权平均自动消化
        # 传感器越多→基线越宽→x越稳定，不存在"用多了反而偏"的问题
        pos_all, err_all = self.solve_cluster(inlier_cluster)
        if pos_all is not None:
            return pos_all, err_all, inlier_cluster
        best_pos = None
        best_error = float('inf')
        best_cluster = None
        m = len(inlier_cluster)
        for length in range(2, m + 1):
            for start in range(0, m - length + 1):
                sub_cluster = inlier_cluster[start:start + length]
                pos, err = self.solve_cluster(sub_cluster)
                if pos is None:
                    continue
                if err < best_error:
                    best_error = err
                    best_pos = pos
                    best_cluster = sub_cluster
        return best_pos, best_error, best_cluster

    def is_adjacent_cluster(self, cluster):
        idx = sorted(i for i, _ in cluster)
        return (max(idx) - min(idx)) == (len(idx) - 1)

    def try_merge_close_solutions(self, solution):
        """仅合并传感器列表,不求解。求解统一在重算误差时由solve_best_subset完成"""
        if len(solution) <= 1:
            return solution
        merged = []
        used = set()
        for i, (x1, y1, cl1) in enumerate(solution):
            if i in used:
                continue
            combined = list(cl1)
            for j, (x2, y2, cl2) in enumerate(solution):
                if j <= i or j in used:
                    continue
                dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
                if dist < self.merge_threshold:
                    combined.extend(cl2)
                    used.add(j)
            if len(combined) > len(cl1):
                # 去重并按传感器编号排序
                unique = {}
                for s in combined:
                    unique[s[0]] = s
                merged_cluster = sorted(unique.values(), key=lambda s: s[0])
                # 合并后必须仍满足连续约束，否则不能判定为同一障碍物
                # 例: [S0,S1] + [S3,S4] → [S0,S1,S3,S4] 中间缺S2，拒绝合并
                if self.is_adjacent_cluster(merged_cluster):
                    merged.append((x1, y1, merged_cluster))
                else:
                    merged.append((x1, y1, cl1))  # 不连续，保留原cluster
            else:
                merged.append((x1, y1, cl1))
        return merged
    #测距分组，每组算一个障碍物坐标
    def process(self):
        if not any(self.data_ready):
            return
        valid_data = [] #(传感器编号i, 测距值r)
        for i, r in enumerate(self.uldis):
            if r is not None:
                valid_data.append((i, r))
        if len(valid_data) < 2:
            return
        #枚举所有非空分组
        all_partitions = self.enumerate_partitions(valid_data)
        best_solution = None
        best_score = float('inf')
        #对每个分组求解
        for partition in all_partitions:
            solution = []
            single_count = 0  # 记录原始分区中单传感器cluster数量
            valid_partition = True

            for cluster in partition:
                sensors = [i for i, _ in cluster]
                if len(cluster) < 2:
                    single_count += 1
                    continue
                if not self.is_adjacent_cluster(cluster):
                    valid_partition = False
                    break
                pos, err, cluster = self.solve_best_subset(cluster)
                if pos is None:
                    valid_partition = False
                    break
                # 约束：y>0在前方，x范围随y自适应放宽（远物允许更偏）
                max_x = 0.3 + 0.25 * pos[0]  
                if pos[0] <= 0.0 or pos[0] > 3.0 or abs(pos[1]) > max_x:
                    continue
                solution.append((pos[0], pos[1], cluster))
                #self.get_logger().info( f"pos=({ pos[1]:.2f},{ pos[0]:.2f}), sensors={sensors}")

            if not valid_partition:
                continue

            # 合并位置过近的解（同一障碍物被不同sensor组解出）
            solution = self.try_merge_close_solutions(solution)
            # 统一用solve_best_subset重算：既得到最优子集+位置，又得到一致误差
            total_error = 0.0
            resolved = []
            for (_, _, cl) in solution:
                pos, err, best_cluster = self.solve_best_subset(cl)
                if pos is None:
                    valid_partition = False
                    break
                max_x = 0.4 + 0.25 * pos[0]
                if pos[0] <= 0.0 or pos[0] > 3.0 or abs(pos[1]) > max_x:
                    continue  # 合并后仍不满足约束，丢弃该解
                resolved.append((pos[0], pos[1], best_cluster))
                total_error += err
            if not valid_partition:
                continue
            solution = resolved
            # 单传感器惩罚 + 聚类数惩罚
            total_error += self.single_sensor_penalty * single_count
            total_error += self.cluster_count_penalty * len(solution)

            # 选择误差最小的分组作为最优解
            if total_error < best_score:
                best_score = total_error
                best_solution = solution

        if best_solution is None:
            return

        #输出结果
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        for (x, y, cluster) in best_solution:
            if len(cluster) < 2:  # 跳过单传感器占位
                continue
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)     
        self.pos_pub.publish(msg)        
    
    def enumerate_partitions(self, data):
            n = len(data)
            partitions = []
            # 用分割点枚举：在 n-1 个间隙中选择切割位置
            for cuts in itertools.product([False, True], repeat=n - 1):
                partition = []
                cluster = [data[0]]
                for i, cut in enumerate(cuts):
                    if cut:
                        partition.append(cluster)
                        cluster = [data[i + 1]]
                    else:
                        cluster.append(data[i + 1])
                partition.append(cluster)
                partitions.append(partition)
            return partitions
            
            
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
        # self.get_logger().info( f" y_sq=({y_sq})\n")
        # self.get_logger().info( f" x=({x})\n")
        # self.get_logger().info( f" y=({y})\n")

        #计算误差
        error = 0.0
        for idx, r in cluster:
            xi = self.sensor_x[idx]
            err = abs(np.sqrt((x - xi) ** 2 + y ** 2) - r)
            error += err
        return (y, x), error #切换坐标系

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicEnumSolver()
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