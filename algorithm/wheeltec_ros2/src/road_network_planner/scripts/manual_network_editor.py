#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial import KDTree
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Bool, String, Int16MultiArray
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import json
import os
import threading
import queue
import math


def euler_to_quaternion(roll, pitch, yaw):
    """将欧拉角转换为四元数"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    
    return [qx, qy, qz, qw]


class ManualNetworkEditor(Node):

    
    def __init__(self):
        super().__init__('manual_network_editor')
        
        # ========== 参数声明 ==========
        self.declare_parameter('roadnet_file', '/home/wheeltec/wheeltec_ros2/src/road_network_planner/map/manual_roadnet.geojson')
        self.road_net_file = self.get_parameter('roadnet_file').value
        self.declare_parameter('COST_THRESHOLD', 60)
        self.COST_THRESHOLD = self.get_parameter('COST_THRESHOLD').value
        self.declare_parameter('PATH_OBSTACLES_THRESHOLD', 5)
        self.PATH_OBSTACLES_THRESHOLD = self.get_parameter('PATH_OBSTACLES_THRESHOLD').value
        
        # ========== 路网数据结构 ==========
        self.road_network = {}  # {node_id: {'x': x, 'y': y, 'name': name, 'edges': [neighbor_ids]}}
        self.node_coords = []   # 用于KDTree搜索
        self.node_ids = []      # 节点ID列表
        self.tree = None        # KDTree实例
        self.next_node_id = 0   # 下一个节点ID
        
        # ========== 编辑状态 ==========
        self.selected_node_id = None      # 当前选中的节点（用于手动创建连接）
        self.edit_mode = 'add_point'      # 当前编辑模式: 'add_point' 或 'connect'
        
        # ========== 导航相关 ==========
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.global_costmap = None
        self.nav_event_queue = queue.Queue(5)
        self.is_navigating = False
        
        # ========== ROS发布器 ==========
        self.waypoint_marker_publisher = self.create_publisher(Marker, 'road_network/waypoint_marker', 10)
        self.edge_marker_publisher = self.create_publisher(Marker, 'road_network/edge_marker', 10)
        self.path_marker_publisher = self.create_publisher(Marker, 'road_network/path_marker', 10)
        self.action_feedback_pub = self.create_publisher(String, '/action_feedback', 5)
        
        # ========== ROS订阅器 ==========
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.handle_clicked_point, 10)
        
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.handle_goal_pose, 10)
        
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, 5)
        
        self.road_net_nav_sub = self.create_subscription(
            PoseStamped, 'road_net_nav', self.nav_callback, 10)
        
        self.road_net_nav_ID_sub = self.create_subscription(
            Int16MultiArray, 'road_net_nav_id', self.id_nav_callback, 10)
        
        self.cancel_nav_sub = self.create_subscription(
            Bool, '/cancel_net_nav', self.cancel_nav_callback, 10)
        
        # ========== ROS服务 ==========
        self.save_network_srv = self.create_service(
            Trigger, 'road_network/save_network', self.handle_save_network)
        
        self.load_network_srv = self.create_service(
            Trigger, 'road_network/load_network', self.handle_load_network)
        
        self.clear_network_srv = self.create_service(
            Trigger, 'road_network/clear_network', self.handle_clear_network)
        
        # ========== 定时器 ==========
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        # ========== 启动导航事件处理线程 ==========
        handle_event_thread = threading.Thread(target=self.handle_nav_events)
        handle_event_thread.daemon = True
        handle_event_thread.start()
        
        self.get_logger().info('=== Manual Road Network Editor ===')
        self.get_logger().info('Node has been started')
        self.get_logger().info('Current mode: ADD POINT')
        self.get_logger().info('Use /road_network/set_edit_mode to switch modes')
        self.get_logger().info('Use 2D Nav Goal to set navigation target')
        self.get_logger().info('Call /road_network/save_network to save network')
        self.get_logger().info('Call /road_network/load_network to load network')

        # 切换编辑模式服务
        from std_srvs.srv import SetBool
        self.set_edit_mode_srv = self.create_service(
            SetBool, 
            'road_network/set_edit_mode', 
            self.handle_set_edit_mode
        )

    def handle_set_edit_mode(self, request, response):
        """切换编辑模式：True=连接模式，False=添加点模式"""
        if request.data:
            self.edit_mode = 'connect'
            self.selected_node_id = None
            self.get_logger().info('=== Mode: CONNECT ===')
            self.get_logger().info('Click on existing waypoints to create connections')
            self.get_logger().info('First click: select start node')
            self.get_logger().info('Second click: select end node')
        else:
            self.edit_mode = 'add_point'
            self.selected_node_id = None
            self.get_logger().info('=== Mode: ADD POINT ===')
            self.get_logger().info('Click on map to add waypoints')
        
        response.success = True
        response.message = f"Mode changed to: {self.edit_mode}"
        return response

    def handle_clicked_point(self, msg):
        """处理RViz点击事件"""
        x = msg.point.x
        y = msg.point.y
        
        if self.edit_mode == 'add_point':
            # 添加点模式：点击地图添加路标点
            node_id = self._add_node(x, y)
            self.get_logger().info(f'Added waypoint {node_id} at ({x:.2f}, {y:.2f})')
            self.get_logger().info(f'Total waypoints: {len(self.road_network)}')
            
        elif self.edit_mode == 'connect':
            # 连接模式：点击已有的路标点创建连接
            nearest_node_id, distance = self._find_nearest_node(x, y)
            
            if nearest_node_id is not None and distance < 0.5:  # 0.5m内认为是点击了路标点
                if self.selected_node_id is None:
                    # 第一次点击：选择起始节点
                    self.selected_node_id = nearest_node_id
                    node_data = self.road_network[nearest_node_id]
                    self.get_logger().info(f'Selected node {nearest_node_id} at ({node_data["x"]:.2f}, {node_data["y"]:.2f})')
                    self.get_logger().info('Click another waypoint to create connection')
                else:
                    # 第二次点击：选择目标节点并创建连接
                    if self.selected_node_id != nearest_node_id:
                        if self._add_edge(self.selected_node_id, nearest_node_id):
                            self.get_logger().info(f'Created connection: {self.selected_node_id} <-> {nearest_node_id}')
                        else:
                            self.get_logger().warn('Connection already exists!')
                    else:
                        self.get_logger().warn('Cannot connect a node to itself!')
                    
                    # 重置选择状态
                    self.selected_node_id = None
                    self.get_logger().info('Ready for next connection...')
            else:
                self.get_logger().warn('No waypoint found nearby! Click closer to a waypoint.')
        
        self._update_kdtree()
        self.publish_markers()

    def _add_node(self, x, y):
        """添加路标点"""
        node_id = self.next_node_id
        self.next_node_id += 1
        
        self.road_network[node_id] = {
            'x': x,
            'y': y,
            'name': f'WP{node_id}',
            'edges': []
        }
        
        return node_id

    def _add_edge(self, from_id, to_id):
        """添加连接线"""
        if from_id in self.road_network and to_id in self.road_network:
            if to_id not in self.road_network[from_id]['edges']:
                self.road_network[from_id]['edges'].append(to_id)
            if from_id not in self.road_network[to_id]['edges']:
                self.road_network[to_id]['edges'].append(from_id)
            return True
        return False

    def _update_kdtree(self):
        """更新KDTree用于最近节点搜索"""
        self.node_coords = []
        self.node_ids = []
        
        for node_id, data in self.road_network.items():
            self.node_coords.append([data['x'], data['y']])
            self.node_ids.append(node_id)
        
        if self.node_coords:
            self.tree = KDTree(self.node_coords)
            self.get_logger().info(f'Updated KDTree with {len(self.node_coords)} nodes')

    def _find_nearest_node(self, x, y):
        """使用KDTree查找最近节点"""
        if self.tree is None:
            return None, float('inf')
        
        distance, index = self.tree.query([x, y])
        nearest_node_id = self.node_ids[index]
        return nearest_node_id, distance

    def handle_goal_pose(self, msg):
        """处理导航目标点（RViz 2D Nav Goal）"""
        goal_pos = msg.pose.position
        self.get_logger().info(f'===== RECEIVED GOAL POSE =====')
        self.get_logger().info(f'Goal position: ({goal_pos.x:.2f}, {goal_pos.y:.2f})')
        
        if not self.road_network:
            self.get_logger().warn('Cannot navigate: No waypoints in network')
            return
        
        # 获取当前位置
        current_x = 0.0
        current_y = 0.0
        got_pose = False
        
        try:
            # 使用 TF 获取当前位姿
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=5.0)
            )
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            self.get_logger().info(f'Current robot position: ({current_x:.2f}, {current_y:.2f})')
            got_pose = True
        except Exception as e:
            self.get_logger().warn(f'Failed to get current pose: {e}')
        
        if not got_pose:
            # 如果无法获取当前位置，使用路网中距离目标点最远的节点作为起点
            # 这样可以找到一条合理的路径
            self.get_logger().warn('Using farthest node in network as start point')
            max_dist = -1
            start_node_id = None
            
            for node_id, data in self.road_network.items():
                dist = math.sqrt((data['x'] - goal_pos.x)**2 + (data['y'] - goal_pos.y)**2)
                if dist > max_dist:
                    max_dist = dist
                    start_node_id = node_id
            
            if start_node_id is not None:
                start_data = self.road_network[start_node_id]
                current_x = start_data['x']
                current_y = start_data['y']
                self.get_logger().info(f'Using farthest node {start_node_id} at ({current_x:.2f}, {current_y:.2f}) as start')
        
        # 查找最近的路标点
        start_node_id, start_dist = self._find_nearest_node(current_x, current_y)
        goal_node_id, goal_dist = self._find_nearest_node(goal_pos.x, goal_pos.y)
        
        if start_node_id is None or goal_node_id is None:
            self.get_logger().warn('Cannot find nearest waypoints')
            return
        
        self.get_logger().info(f'Start node: {start_node_id} (distance: {start_dist:.2f}m)')
        self.get_logger().info(f'Goal node: {goal_node_id} (distance: {goal_dist:.2f}m)')
        
        # 发布导航事件
        nav_msg = Int16MultiArray()
        nav_msg.data = [start_node_id, goal_node_id]
        self.nav_event_queue.put(nav_msg)

    def id_nav_callback(self, msg: Int16MultiArray):
        """ID导航回调"""
        self.nav_event_queue.put(msg)

    def nav_callback(self, msg: PoseStamped):
        """位姿导航回调"""
        self.nav_event_queue.put(msg)

    def cancel_nav_callback(self, msg: Bool):
        """取消导航回调"""
        if msg.data:
            self.navigator.cancelTask()
            self.get_logger().info("navigation task canceled via message.")

    def handle_nav_events(self):
        """处理导航事件队列"""
        while True:
            try:
                event = self.nav_event_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            
            self.navigator = BasicNavigator()
            
            if isinstance(event, Int16MultiArray):
                start_id = event.data[0]
                goal_id = event.data[1]
                self.handle_id_navigation(start_id, goal_id)
            elif isinstance(event, PoseStamped):
                self.handle_pose_navigation(event)

    def handle_id_navigation(self, start_id: int, goal_id: int):
        """路点ID导航 - 基于路网的路径规划"""
        self.get_logger().info(f'Starting ID navigation: {start_id} -> {goal_id}')
        
        # 检查路点是否存在
        if start_id not in self.road_network or goal_id not in self.road_network:
            self.get_logger().warn('Invalid node ID')
            return
        
        # 使用Dijkstra算法在路网上找最短路径
        path = self._find_shortest_path(start_id, goal_id)
        
        if not path:
            self.get_logger().warn('No path found in road network')
            return
        
        self.get_logger().info(f'Found path: {" -> ".join(map(str, path))}')
        
        # 依次导航到每个路标点
        for i, node_id in enumerate(path):
            node_data = self.road_network[node_id]
            
            # 创建导航目标位姿
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = node_data['x']
            goal_pose.pose.position.y = node_data['y']
            goal_pose.pose.position.z = 0.0
            
            # 设置朝向（面向下一个路标点）
            if i < len(path) - 1:
                next_node_data = self.road_network[path[i+1]]
                dx = next_node_data['x'] - node_data['x']
                dy = next_node_data['y'] - node_data['y']
                yaw = math.atan2(dy, dx)
                quat = euler_to_quaternion(0, 0, yaw)
                goal_pose.pose.orientation.x = quat[0]
                goal_pose.pose.orientation.y = quat[1]
                goal_pose.pose.orientation.z = quat[2]
                goal_pose.pose.orientation.w = quat[3]
            else:
                # 最后一个点保持当前朝向或设置为0
                goal_pose.pose.orientation.w = 1.0
            
            self.get_logger().info(f'Navigating to waypoint {node_id} at ({node_data["x"]:.2f}, {node_data["y"]:.2f})')
            
            # 使用Nav2导航到该路标点
            self.navigator.goToPose(goal_pose)
            
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    # 可以在这里添加障碍物检测逻辑
                    pass
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'✅ Arrived at waypoint {node_id}')
            elif result == TaskResult.FAILED:
                self.get_logger().error(f'❌ Failed to reach waypoint {node_id}')
                self.action_feedback_pub.publish(String(data="road_net_nav_failed"))
                return
        
        self.get_logger().info("✅ Navigation completed successfully!")
        self.action_feedback_pub.publish(String(data="road_net_nav_succeeded"))
    
    def _find_shortest_path(self, start_id: int, goal_id: int):
        """使用Dijkstra算法在路网上找最短路径"""
        if start_id == goal_id:
            return [start_id]
        
        # 初始化距离字典
        distances = {node_id: float('inf') for node_id in self.road_network}
        distances[start_id] = 0
        
        # 前驱节点字典
        previous = {node_id: None for node_id in self.road_network}
        
        # 使用优先队列
        import heapq
        queue = []
        heapq.heappush(queue, (0, start_id))
        
        while queue:
            current_dist, current_id = heapq.heappop(queue)
            
            if current_id == goal_id:
                break
            
            if current_dist > distances[current_id]:
                continue
            
            # 遍历相邻节点
            if current_id not in self.road_network:
                continue
            
            neighbors = self.road_network[current_id].get('edges', [])
            for neighbor_id in neighbors:
                if neighbor_id not in self.road_network:
                    continue
                
                # 计算距离
                dx = self.road_network[current_id]['x'] - self.road_network[neighbor_id]['x']
                dy = self.road_network[current_id]['y'] - self.road_network[neighbor_id]['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                new_distance = current_dist + distance
                
                if new_distance < distances[neighbor_id]:
                    distances[neighbor_id] = new_distance
                    previous[neighbor_id] = current_id
                    heapq.heappush(queue, (new_distance, neighbor_id))
        
        # 重建路径
        path = []
        current_id = goal_id
        while current_id is not None:
            path.insert(0, current_id)
            current_id = previous[current_id]
        
        if path[0] != start_id:
            return []
        
        return path

    def handle_pose_navigation(self, goal_pose: PoseStamped):
        """处理任意位姿导航 - 基于路网的点到点导航"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", 
                rclpy.time.Time(), 
                rclpy.duration.Duration(seconds=5.0)
            )
            # 获取当前位置
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            got_pose = True
        except Exception as e:
            self.get_logger().warn(f"Failed to get current pose: {e}")
            self.get_logger().warn('Using farthest node in network as start point')
            
            # 如果无法获取当前位置，使用路网中距离目标点最远的节点作为起点
            max_dist = -1
            start_node_id = None
            for node_id, data in self.road_network.items():
                dist = math.sqrt((data['x'] - goal_pose.pose.position.x)**2 + 
                                (data['y'] - goal_pose.pose.position.y)**2)
                if dist > max_dist:
                    max_dist = dist
                    start_node_id = node_id
            
            if start_node_id is not None:
                start_data = self.road_network[start_node_id]
                current_x = start_data['x']
                current_y = start_data['y']
                got_pose = True
                self.get_logger().info(f'Using farthest node {start_node_id} at ({current_x:.2f}, {current_y:.2f}) as start')
            else:
                return
        
        self.get_logger().info(f"Current position: ({current_x:.2f}, {current_y:.2f})")
        self.get_logger().info(f"Goal position: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")
        
        # 查找最近的路标点
        start_node_id, start_dist = self._find_nearest_node(current_x, current_y)
        goal_node_id, goal_dist = self._find_nearest_node(goal_pose.pose.position.x, goal_pose.pose.position.y)
        
        if start_node_id is None or goal_node_id is None:
            self.get_logger().warn('Cannot find nearest waypoints')
            return
        
        self.get_logger().info(f'Start node: {start_node_id} (distance: {start_dist:.2f}m)')
        self.get_logger().info(f'Goal node: {goal_node_id} (distance: {goal_dist:.2f}m)')
        
        # 使用路网进行点到点导航
        self.handle_id_navigation(start_node_id, goal_node_id)

    def _check_path_obstacles(self, path: Path) -> bool:
        """检查路径上是否存在障碍物"""
        if self.global_costmap is None:
            return False
        
        obstacle_count = 0
        for pose in path.poses:
            wx = pose.pose.position.x
            wy = pose.pose.position.y
            
            mx, my = self.global_costmap.worldToMapValidated(wx, wy)
            if mx is None or my is None:
                continue
            
            cost = self.global_costmap.getCostXY(int(mx), int(my))
            if cost > self.COST_THRESHOLD:
                obstacle_count += 1
        
        if obstacle_count > self.PATH_OBSTACLES_THRESHOLD:
            self.get_logger().info(f"Path contains {obstacle_count} potential obstacles.")
            return True
        return False

    def _adjust_pose(self, final_pose: PoseStamped) -> bool:
        """调整到目标姿态"""
        self.get_logger().info("Adjusting final orientation...")
        final_orientation_task = self.navigator.goToPose(final_pose)
        
        while not self.navigator.isTaskComplete(task=final_orientation_task):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        orientation_result = self.navigator.getResult()
        if orientation_result == TaskResult.SUCCEEDED:
            return True
        else:
            self.get_logger().warn("Final orientation adjustment failed.")
            return False

    def _reroute_strategy(self, last_id):
        """重规划策略：回退到上一个路点"""
        self.get_logger().info(f"Rerouting from node {last_id}")
        
        if last_id not in self.road_network:
            self.get_logger().warn("Last node ID not found")
            return
        
        goal_x = self.road_network[last_id]['x']
        goal_y = self.road_network[last_id]['y']
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        
        navigator = BasicNavigator()
        back_task = navigator.goToPose(goal_pose)
        
        while not navigator.isTaskComplete(task=back_task):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if navigator.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info("Back to last node succeeded.")
        else:
            self.get_logger().warn("Back to last node failed.")

    def global_costmap_callback(self, msg: OccupancyGrid):
        """更新全局代价地图"""
        from nav2_simple_commander.costmap_2d import PyCostmap2D
        self.global_costmap = PyCostmap2D(msg)

    def publish_markers(self):
        """发布路标点和连接线的可视化标记"""
        self.publish_waypoint_markers()
        self.publish_edge_markers()

    def publish_waypoint_markers(self):
        """发布路标点标记"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        for node_id, data in self.road_network.items():
            p = Point()
            p.x = data['x']
            p.y = data['y']
            p.z = 0.05
            marker.points.append(p)
        
        self.waypoint_marker_publisher.publish(marker)

    def publish_edge_markers(self):
        """发布连接线标记"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'edges'
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        for node_id, data in self.road_network.items():
            for neighbor_id in data['edges']:
                if neighbor_id > node_id:  # 避免重复绘制
                    p1 = Point()
                    p1.x = data['x']
                    p1.y = data['y']
                    p1.z = 0.02
                    marker.points.append(p1)
                    
                    neighbor_data = self.road_network.get(neighbor_id)
                    if neighbor_data:
                        p2 = Point()
                        p2.x = neighbor_data['x']
                        p2.y = neighbor_data['y']
                        p2.z = 0.02
                        marker.points.append(p2)
        
        self.edge_marker_publisher.publish(marker)

        # # 发布路标点ID标签
        # text_marker = Marker()
        # text_marker.header.frame_id = 'map'
        # text_marker.header.stamp = self.get_clock().now().to_msg()
        # text_marker.ns = 'waypoint_labels'
        # text_marker.id = 100
        # text_marker.type = Marker.TEXT_VIEW_FACING
        # text_marker.action = Marker.ADD
        # text_marker.scale.z = 0.3
        # text_marker.color.r = 0.0
        # text_marker.color.g = 0.0
        # text_marker.color.b = 0.0
        # text_marker.color.a = 1.0
        
        # for node_id, data in self.road_network.items():
        #     label = Marker()
        #     label.header = text_marker.header
        #     label.ns = text_marker.ns
        #     label.id = 100 + node_id
        #     label.type = Marker.TEXT_VIEW_FACING
        #     label.action = Marker.ADD
        #     label.pose.position.x = data['x'] + 0.25
        #     label.pose.position.y = data['y'] + 0.25
        #     label.pose.position.z = 0.1
        #     label.scale.z = 0.3
        #     label.color.r = 0.0
        #     label.color.g = 0.0
        #     label.color.b = 0.0
        #     label.color.a = 1.0
        #     label.text = str(node_id)
        #     self.waypoint_marker_publisher.publish(label)

    def handle_save_network(self, request, response):
        """保存路网到GeoJSON文件"""
        try:
            features = []
            
            # 添加路标点（Point类型）
            for node_id, data in self.road_network.items():
                point_feature = {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'Point',
                        'coordinates': [data['x'], data['y']]
                    },
                    'properties': {
                        'id': node_id,
                        'name': data['name']
                    }
                }
                features.append(point_feature)
            
            # 添加连接线（LineString类型）
            for node_id, data in self.road_network.items():
                for neighbor_id in data['edges']:
                    if neighbor_id > node_id:  # 避免重复
                        neighbor_data = self.road_network.get(neighbor_id)
                        if neighbor_data:
                            line_feature = {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': [
                                        [data['x'], data['y']],
                                        [neighbor_data['x'], neighbor_data['y']]
                                    ]
                                },
                                'properties': {
                                    'from_id': node_id,
                                    'to_id': neighbor_id
                                }
                            }
                            features.append(line_feature)
            
            # 创建GeoJSON对象
            geojson = {
                'type': 'FeatureCollection',
                'features': features
            }
            
            # 保存文件
            with open(self.road_net_file, 'w') as f:
                json.dump(geojson, f, indent=2)
            
            response.success = True
            response.message = f'Saved {len(self.road_network)} nodes to {self.road_net_file}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to save: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def handle_load_network(self, request, response):
        """从GeoJSON文件加载路网"""
        try:
            if not os.path.exists(self.road_net_file):
                response.success = False
                response.message = f'File not found: {self.road_net_file}'
                return response
            
            with open(self.road_net_file, 'r') as f:
                data = json.load(f)
            
            # 清空现有路网
            self.road_network = {}
            self.next_node_id = 0
            
            # 加载路标点
            node_id_map = {}  # 原始ID到新ID的映射
            for feat in data.get('features', []):
                if feat['geometry']['type'] == 'Point':
                    x, y = feat['geometry']['coordinates']
                    original_id = feat['properties'].get('id', self.next_node_id)
                    name = feat['properties'].get('name', f'WP{self.next_node_id}')
                    
                    node_id_map[original_id] = self.next_node_id
                    self.road_network[self.next_node_id] = {
                        'x': x,
                        'y': y,
                        'name': name,
                        'edges': []
                    }
                    self.next_node_id += 1
            
            # 加载连接线
            for feat in data.get('features', []):
                if feat['geometry']['type'] == 'LineString':
                    from_id = feat['properties'].get('from_id')
                    to_id = feat['properties'].get('to_id')
                    
                    if from_id in node_id_map and to_id in node_id_map:
                        new_from_id = node_id_map[from_id]
                        new_to_id = node_id_map[to_id]
                        self._add_edge(new_from_id, new_to_id)
            
            self._update_kdtree()
            self.publish_markers()
            
            response.success = True
            response.message = f'Loaded {len(self.road_network)} nodes from {self.road_net_file}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to load: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def handle_clear_network(self, request, response):
        """清空路网"""
        self.road_network = {}
        self.next_node_id = 0
        self.node_coords = []
        self.node_ids = []
        self.tree = None
        self.first_point = None
        
        self.publish_markers()
        
        response.success = True
        response.message = 'Road network cleared'
        self.get_logger().info(response.message)
        
        return response


def main():
    """主函数"""
    rclpy.init()
    node = ManualNetworkEditor()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted')
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
