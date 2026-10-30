#!/usr/bin/env python3
"""
优化版扫描-地图重定位 (距离场 / likelihood-field 匹配)

相比 simple_relocalization.py 的四个关键改动:
1. 评分函数: Canny 边缘图 + 3px 邻域投票 -> 障碍物距离场(cv2.distanceTransform)
   + 似然场核 exp(-d^2/2σ^2), 并加入射线中点"穿墙惩罚"(抑制对称环境误匹配的关键)。
2. 全部 numpy/cv2 向量化: 一次对"一整行候选位置 × 全部射线"同时评分,
   取代逐射线的 Python 循环 (旧版主要耗时点)。
3. 粗到精三级搜索: 低分辨率全局搜索(整图) -> 高分辨率局部细搜(top-K 邻域)
   -> 坐标下降亚像素精化。
4. 自适应协方差: 匹配置信度高时给 AMCL 发布较紧的初始协方差, 帮助其锁定正确模态;
   置信度低时发布大协方差, 把收敛工作交给 AMCL。
5. 可选搜索区域限制(search_x/y_min/max): 已知机器人大概区域时可大幅缩小搜索空间。
6. 多帧扫描累积(num_scans): 重定位期间让机器人动起来并采集多帧(与车型无关,
   只要求/odom 有输出: 阿克曼缓慢行驶弧线, 麦轮/全向轮平移或旋转, 差速/四驱
   原地旋转均可), 按里程计相对位姿拼接后联合评分, 可强力抑制对称/重复环境的
   错位匹配(激光扫描匹配最有效的消歧手段)。静止重定位(num_scans=1)不依赖车型。
7. 已知位姿诊断(known_pose_*): 配置真值后打印其分数, 用于判断是"地图与当前扫描
   不符"还是"环境歧义"。
8. 里程计修正(use_odom + /odom): 帧间相对位姿由里程计提供, 求解的是最后一帧
   (≈当前)位姿, 天然消除"旋转弧段平均航向"偏差; 无里程计时退化为同一位姿多帧
   模型(仅适合原地旋转或静止采集)。

预期性能(0.05 m 分辨率、2000x2000 地图、~720 线激光):
  旧版: 数十秒~数分钟;  新版: 约 1~3 秒, 位置误差一般 <10 cm, 航向 <2°。
"""

import heapq
import math
import os
import time

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class SimpleRelocalization(Node):
    def __init__(self):
        super().__init__('simple_relocalization')
        self.get_logger().info('fast relocalization v4: odometry-corrected multi-scan (chassis-agnostic)')

        # ---- 可配置参数 ----
        self.declare_parameter('map_yaml_path',
                               '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.yaml')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('robot_radius', 0.20)      # 候选位置与障碍物的最小距离(m)
        self.declare_parameter('score_threshold', 0.30)   # 低于该分数发布大协方差
        self.declare_parameter('high_score', 0.55)        # 高于该分数发布紧协方差
        self.declare_parameter('top_k', 10)               # 全局搜索保留的候选位姿数
        self.declare_parameter('coarse_scale', 4)         # 低分辨率层降采样倍率
        self.declare_parameter('coarse_step', 3)          # 低分辨率层采样步长(低分辨率像素)
        self.declare_parameter('coarse_yaw_step', 30.0)   # 全局航向步长(度)
        self.declare_parameter('max_coarse_rays', 180)    # 粗匹配最多使用的射线数
        self.declare_parameter('max_fine_rays', 1000)     # 细搜索最多使用的射线数(多帧时自动降采样)
        self.declare_parameter('sigma_px', 3.0)           # 似然场σ(高分辨率像素)
        self.declare_parameter('save_debug_image', True)  # 把最终匹配结果画到地图上存档
        # 可选: 已知机器人大概在哪个区域时, 限制搜索范围(米, -1 表示不限制)
        self.declare_parameter('search_x_min', -1.0)
        self.declare_parameter('search_x_max', -1.0)
        self.declare_parameter('search_y_min', -1.0)
        self.declare_parameter('search_y_max', -1.0)
        # 多帧扫描累积: >1 时请在采集期间让机器人动起来(车型无关, 只要求 /odom 有输出)
        self.declare_parameter('num_scans', 1)
        # 已知真值位姿诊断(仅打印分数对比, 不参与搜索): 单位米/度
        self.declare_parameter('known_pose_x', -9999.0)
        self.declare_parameter('known_pose_y', -9999.0)
        self.declare_parameter('known_pose_yaw_deg', 0.0)
        # 里程计修正(阿克曼车无法原地旋转, 需边行驶边采集): /odom 提供帧间相对位姿
        self.declare_parameter('use_odom', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('laser_offset_x', 0.0)    # 激光相对车体中心(odom 原点)的偏移(m)
        self.declare_parameter('laser_offset_y', 0.0)

        self.map_yaml_path = self.get_parameter('map_yaml_path').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.score_threshold = self.get_parameter('score_threshold').value
        self.high_score = self.get_parameter('high_score').value
        self.top_k = self.get_parameter('top_k').value
        self.coarse_scale = self.get_parameter('coarse_scale').value
        self.coarse_step = self.get_parameter('coarse_step').value
        self.coarse_yaw_step = self.get_parameter('coarse_yaw_step').value
        self.max_coarse_rays = self.get_parameter('max_coarse_rays').value
        self.max_fine_rays = self.get_parameter('max_fine_rays').value
        self.sigma_px = self.get_parameter('sigma_px').value
        self.save_debug = self.get_parameter('save_debug_image').value
        self.search_x_min = self.get_parameter('search_x_min').value
        self.search_x_max = self.get_parameter('search_x_max').value
        self.search_y_min = self.get_parameter('search_y_min').value
        self.search_y_max = self.get_parameter('search_y_max').value
        self.num_scans = max(1, int(self.get_parameter('num_scans').value))
        self.known_pose_x = self.get_parameter('known_pose_x').value
        self.known_pose_y = self.get_parameter('known_pose_y').value
        self.known_pose_yaw_deg = self.get_parameter('known_pose_yaw_deg').value
        self.use_odom = bool(self.get_parameter('use_odom').value)
        self.odom_topic = self.get_parameter('odom_topic').value
        self.laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(self.get_parameter('laser_offset_y').value)

        # ---- 状态 ----
        self.map_image = None
        self.dist = None        # 高分辨率距离场(像素)
        self.dist_low = None    # 低分辨率距离场(低分辨率像素)
        self.free_low = None    # 低分辨率自由空间掩码
        self.sigma_low = None   # 低分辨率层似然场σ
        self.scan_buffer = []   # 多帧扫描累积缓冲 [(ranges, angles, odom_pose|None), ...]
        self.latest_odom = None     # 最近一帧里程计位姿 (x, y, yaw)
        self.frame_r = None         # 各帧有效射线距离(m), _prepare_frames 后填充
        self.frame_a = None         # 各帧有效射线角度(rad, 不含帧间航向差)
        self.frame_dx = None        # 各帧激光原点相对参考帧(最后一帧)的偏移(世界系, m)
        self.frame_dy = None
        self.frame_dyaw = None      # 各帧相对参考帧的航向差(rad)
        self.fr_coarse = None       # 各帧粗匹配抽样(角度已含 frame_dyaw)
        self.fa_coarse = None
        self.fr_fine = None         # 各帧细搜索抽样(角度已含 frame_dyaw)
        self.fa_fine = None
        self.mid_margin_px = 0.0    # 射线中点"穿墙"检查容差(像素), 地图加载后计算
        self.mid_margin_low = 0.0   # 低分辨率层的对应容差
        self.known_score = 0.0      # 已知真值位姿的匹配分数(仅诊断用)
        self.relocalization_done = False

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)

        if self.load_map():
            self.get_logger().info('waiting for laser scan data...')
        else:
            self.get_logger().error('map loading failed, node is idle')

    # ---------------- 地图加载 ----------------
    def load_map(self):
        try:
            with open(self.map_yaml_path, 'r') as f:
                cfg = yaml.safe_load(f)
            self.map_resolution = float(cfg['resolution'])
            self.map_origin = cfg['origin']
            pgm = os.path.join(os.path.dirname(self.map_yaml_path), cfg['image'])

            img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
            if img is None:
                self.get_logger().error(f'failed to read pgm: {pgm}')
                return False

            self.map_height, self.map_width = img.shape
            self.map_image = img
            # 与旧版一致: >=200 视为自由空间, 其余视为障碍
            free = (img >= 200).astype(np.uint8)

            # 距离场: 每个像素到最近障碍物的距离(像素)
            # 注意 distanceTransform 计算的是"到最近零像素"的距离,
            # 所以前景必须传自由空间(free=255, 障碍=0), 得到才是到障碍物的距离
            self.dist = cv2.distanceTransform(free * 255, cv2.DIST_L2, 5)

            # 低分辨率层(粗搜索用): 对自由空间图降采样后再求距离场
            s = self.coarse_scale
            free_low_full = cv2.resize(
                free, (self.map_width // s, self.map_height // s),
                interpolation=cv2.INTER_NEAREST)
            self.dist_low = cv2.distanceTransform(free_low_full * 255, cv2.DIST_L2, 5)
            # 粗匹配容差约 50cm(换算成低分辨率像素, 至少 2px)
            self.sigma_low = max(0.5 / (self.map_resolution * s), 2.0)
            # 射线中点一致性检查的容差: 中点到障碍物距离小于该值时判定"穿墙"
            self.mid_margin_px = max(self.robot_radius / self.map_resolution, 1.0)
            self.mid_margin_low = max(self.mid_margin_px / s, 0.5)

            # 自由空间候选掩码: 腐蚀掉机器人半径, 防止位姿落在贴墙处
            margin = max(1, int(round(self.robot_radius / self.map_resolution)))
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * margin + 1, 2 * margin + 1))
            free = cv2.erode(free, kernel)
            free_low = cv2.resize(
                free, (self.map_width // s, self.map_height // s),
                interpolation=cv2.INTER_NEAREST)
            self.free_low = free_low > 0

            self.get_logger().info(
                f'map loaded: {self.map_width}x{self.map_height}px, '
                f'{self.map_resolution:.4f} m/px, '
                f'low-res {self.dist_low.shape[1]}x{self.dist_low.shape[0]}')
            return True
        except Exception as e:
            self.get_logger().error(f'error loading map: {e}')
            return False

    # ---------------- 里程计 ----------------
    def odom_callback(self, msg):
        """记录最近一帧里程计位姿(仅取 x, y, yaw)"""
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    # ---------------- 扫描数据 ----------------
    def scan_callback(self, msg):
        if self.relocalization_done or self.dist is None:
            return
        r, a = self.extract_scan(msg)
        if r is None:
            return

        # 多帧采集: 收满 num_scans 帧后统一匹配
        # 采集期间让车动起来(车型无关): 阿克曼缓慢开弧线, 麦轮/全向轮平移或旋转,
        # 差速/四驱原地慢转——帧间相对位姿由里程计修正
        if len(self.scan_buffer) < self.num_scans:
            self.scan_buffer.append((r, a, self.latest_odom))
            n = len(self.scan_buffer)
            if n < self.num_scans:
                self.get_logger().info(
                    f'scan {n}/{self.num_scans} collected '
                    f'(drive slowly / rotate between scans)')
                return
            self.get_logger().info(
                f'{self.num_scans} scans collected, matching...')

        self._prepare_frames()
        self.perform_relocalization()

    def _prepare_frames(self):
        """把多帧扫描按里程计相对位姿对齐到参考帧(最后一帧), 生成各帧评分数据"""
        n = len(self.scan_buffer)
        use_odom = self.use_odom and all(s[2] is not None for s in self.scan_buffer)
        if self.num_scans > 1 and not use_odom:
            self.get_logger().warn(
                'no odometry available: assuming all frames share one pose '
                '(only valid for in-place rotation or a stationary robot)')
        self.frame_r = [s[0] for s in self.scan_buffer]
        self.frame_a = [s[1] for s in self.scan_buffer]
        self.frame_dx = np.zeros(n, dtype=np.float64)
        self.frame_dy = np.zeros(n, dtype=np.float64)
        self.frame_dyaw = np.zeros(n, dtype=np.float64)
        if use_odom:
            xL, yL, tL = self.scan_buffer[-1][2]
            lxL = xL + self.laser_offset_x * math.cos(tL) - self.laser_offset_y * math.sin(tL)
            lyL = yL + self.laser_offset_x * math.sin(tL) + self.laser_offset_y * math.cos(tL)
            for i, s in enumerate(self.scan_buffer):
                x, y, t = s[2]
                lx = x + self.laser_offset_x * math.cos(t) - self.laser_offset_y * math.sin(t)
                ly = y + self.laser_offset_x * math.sin(t) + self.laser_offset_y * math.cos(t)
                dx, dy = lx - lxL, ly - lyL
                # 转到参考帧(最后一帧)车体坐标
                self.frame_dx[i] = math.cos(tL) * dx + math.sin(tL) * dy
                self.frame_dy[i] = -math.sin(tL) * dx + math.cos(tL) * dy
                self.frame_dyaw[i] = t - tL
        self.fr_coarse = []
        self.fa_coarse = []
        self.fr_fine = []
        self.fa_fine = []
        per_c = max(20, self.max_coarse_rays // n)
        per_f = max(25, self.max_fine_rays // n)
        for i in range(n):
            c = max(1, len(self.frame_r[i]) // per_c)
            f = max(1, len(self.frame_r[i]) // per_f)
            self.fr_coarse.append(self.frame_r[i][::c])
            self.fa_coarse.append(self.frame_a[i][::c] + self.frame_dyaw[i])
            self.fr_fine.append(self.frame_r[i][::f])
            self.fa_fine.append(self.frame_a[i][::f] + self.frame_dyaw[i])
        total = sum(len(r) for r in self.frame_r)
        self.get_logger().info(
            f'total valid rays: {total} '
            f'(coarse {sum(len(r) for r in self.fr_coarse)}, '
            f'fine {sum(len(r) for r in self.fr_fine)})')

    def extract_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = np.linspace(msg.angle_min, msg.angle_max, len(r), dtype=np.float32)
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        r = r[ok]
        a = a[ok]
        if len(r) < 30:
            self.get_logger().error(f'too few valid rays: {len(r)}')
            return None, None
        return r, a

    # ---------------- 匹配 ----------------
    def _search_bounds(self, W, H, step):
        """把世界坐标搜索范围换算成低分辨率像素范围(未设置时全图)"""
        s = self.coarse_scale
        res = self.map_resolution
        ox, oy = self.map_origin[0], self.map_origin[1]
        x_lo, x_hi = step, W - step
        y_lo, y_hi = step, H - step
        if self.search_x_min >= 0.0:
            x_lo = max(x_lo, int((self.search_x_min - ox) / res / s))
        if self.search_x_max >= 0.0:
            x_hi = min(x_hi, int((self.search_x_max - ox) / res / s) + 1)
        # 图像 y 轴向下, 与世界坐标相反
        if self.search_y_max >= 0.0:
            y_lo = max(y_lo, int((self.map_height - 1 - (self.search_y_max - oy) / res) / s))
        if self.search_y_min >= 0.0:
            y_hi = min(y_hi, int((self.map_height - 1 - (self.search_y_min - oy) / res) / s) + 1)
        return x_lo, x_hi, y_lo, y_hi

    def global_search(self):
        """低分辨率层全局搜索: 遍历整张地图的候选位置×航向, 保留 top-K"""
        s = self.coarse_scale
        res_low = self.map_resolution * s
        H, W = self.dist_low.shape
        step = self.coarse_step
        yaws = np.deg2rad(np.arange(0.0, 360.0, self.coarse_yaw_step))
        sig = self.sigma_low
        n = len(self.frame_r)
        fr = [self.fr_coarse[i] / res_low for i in range(n)]
        fdx = self.frame_dx / res_low  # 各帧原点偏移(低分辨率像素, 世界系)
        fdy = self.frame_dy / res_low

        heap = []  # 元素: (score, x_hi, y_hi, yaw)
        x_lo, x_hi, y_lo, y_hi = self._search_bounds(W, H, step)
        t0 = time.time()
        for yaw in yaws:
            cy, sy = math.cos(yaw), math.sin(yaw)
            for y in range(y_lo, y_hi, step):
                xs = np.arange(x_lo, x_hi, step, dtype=np.float32)
                cols = np.flatnonzero(self.free_low[y, xs.astype(np.int32)])
                if cols.size == 0:
                    continue
                xs = xs[cols]
                # 候选位姿是参考帧(最后一帧)的位姿; 各帧按自己的偏移分别评分后取平均
                scores = np.zeros(xs.size, dtype=np.float64)
                for i in range(n):
                    # 帧 i 激光原点相对参考帧位姿的偏移(图像坐标, 随候选航向旋转)
                    ox = fdx[i] * cy - fdy[i] * sy
                    oy = -(fdx[i] * sy + fdy[i] * cy)
                    ca = np.cos(self.fa_coarse[i] + yaw).astype(np.float32)
                    sa = np.sin(self.fa_coarse[i] + yaw).astype(np.float32)
                    dx = (fr[i] * ca).astype(np.float32)  # 射线终点偏移(低分辨率像素)
                    dy = (fr[i] * sa).astype(np.float32)
                    map_x = (xs[None, :] + ox) + dx[:, None]
                    map_y = (y + oy - dy)[:, None] * np.ones((1, xs.size), dtype=np.float32)
                    vals = cv2.remap(self.dist_low, map_x, map_y, cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=1e4)
                    # 射线中点一致性: 中点离障碍太近说明射线"穿墙", 予以惩罚
                    mid_x = (xs[None, :] + ox) + 0.5 * dx[:, None]
                    mid_y = (y + oy - 0.5 * dy)[:, None] * np.ones((1, xs.size), dtype=np.float32)
                    mid_d = cv2.remap(self.dist_low, mid_x, mid_y, cv2.INTER_LINEAR,
                                      borderMode=cv2.BORDER_CONSTANT, borderValue=1e4)
                    pen = np.clip(mid_d / self.mid_margin_low, 0.0, 1.0)
                    scores += np.mean(np.exp(-0.5 * (vals / sig) ** 2) * pen, axis=0)
                scores /= n
                for j in range(scores.size):
                    sc = float(scores[j])
                    if sc < 0.05:
                        continue
                    if len(heap) < self.top_k or sc > heap[0][0]:
                        heapq.heappush(heap, (sc, float(xs[j]) * s, float(y) * s, float(yaw)))
                        if len(heap) > self.top_k:
                            heapq.heappop(heap)
        self.get_logger().info(f'global search: {time.time() - t0:.2f}s, {len(heap)} candidates')
        return [h[1:] + (h[0],) for h in sorted(heap, key=lambda h: -h[0])]

    def refine_grid(self, hyp):
        """高分辨率层局部细搜: 在粗匹配候选附近做细网格搜索"""
        x0, y0, yaw0, _ = hyp
        win_xy = self.coarse_step * self.coarse_scale // 2 + 4  # 覆盖粗采样间隙
        yaw_win = self.coarse_yaw_step / 2.0
        xs = np.arange(x0 - win_xy, x0 + win_xy + 1, 2, dtype=np.float32)
        ys = np.arange(y0 - win_xy, y0 + win_xy + 1, 2, dtype=np.float32)
        yaws = np.deg2rad(np.rad2deg(yaw0) + np.arange(-yaw_win, yaw_win + 1, 2.0)).astype(np.float32)

        gx, gy, gyaw = np.meshgrid(xs, ys, yaws, indexing='ij')
        gx = gx.ravel()
        gy = gy.ravel()
        gyaw = gyaw.ravel()

        # 各帧按里程计偏移分别评分后取平均(多帧模式下射线已按帧降采样)
        n = len(self.frame_r)
        scores = np.zeros(gx.size, dtype=np.float64)
        for i in range(n):
            ox = (self.frame_dx[i] * np.cos(gyaw) - self.frame_dy[i] * np.sin(gyaw)) / self.map_resolution
            oy = -(self.frame_dx[i] * np.sin(gyaw) + self.frame_dy[i] * np.cos(gyaw)) / self.map_resolution
            r = (self.fr_fine[i] / self.map_resolution).astype(np.float32)
            ca = np.cos(self.fa_fine[i][:, None] + gyaw[None, :]).astype(np.float32)
            sa = np.sin(self.fa_fine[i][:, None] + gyaw[None, :]).astype(np.float32)
            map_x = (gx[None, :] + ox[None, :]) + r[:, None] * ca
            map_y = (gy[None, :] + oy[None, :]) - r[:, None] * sa
            vals = cv2.remap(self.dist, map_x, map_y, cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=1e4)
            mid_d = cv2.remap(self.dist, (gx[None, :] + ox[None, :]) + 0.5 * r[:, None] * ca,
                              (gy[None, :] + oy[None, :]) - 0.5 * r[:, None] * sa,
                              cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=1e4)
            pen = np.clip(mid_d / self.mid_margin_px, 0.0, 1.0)
            scores += np.mean(np.exp(-0.5 * (vals / self.sigma_px) ** 2) * pen, axis=0)
        scores /= n
        k = int(np.argmax(scores))
        return float(gx[k]), float(gy[k]), float(gyaw[k]), float(scores[k])

    def bilinear_dist(self, px, py):
        """距离场的双线性插值采样"""
        x0 = np.floor(px).astype(np.int32)
        y0 = np.floor(py).astype(np.int32)
        fx = px - x0
        fy = py - y0
        x0c = np.clip(x0, 0, self.map_width - 1)
        y0c = np.clip(y0, 0, self.map_height - 1)
        x1c = np.clip(x0 + 1, 0, self.map_width - 1)
        y1c = np.clip(y0 + 1, 0, self.map_height - 1)
        d = (self.dist[y0c, x0c] * (1 - fx) * (1 - fy) +
             self.dist[y0c, x1c] * fx * (1 - fy) +
             self.dist[y1c, x0c] * (1 - fx) * fy +
             self.dist[y1c, x1c] * fx * fy)
        outside = (x0 < 0) | (x0 >= self.map_width - 1) | (y0 < 0) | (y0 >= self.map_height - 1)
        return np.where(outside, 1e4, d)

    def cost(self, x, y, yaw):
        """负似然场分数(越小越好), 用于亚像素精化; 各帧按里程计偏移分别评分"""
        cy, sy = math.cos(yaw), math.sin(yaw)
        res = self.map_resolution
        n = len(self.frame_r)
        total = 0.0
        for i in range(n):
            ox = (self.frame_dx[i] * cy - self.frame_dy[i] * sy) / res
            oy = -(self.frame_dx[i] * sy + self.frame_dy[i] * cy) / res
            a = self.frame_a[i] + self.frame_dyaw[i]
            ca = np.cos(a + yaw)
            sa = np.sin(a + yaw)
            rx = self.frame_r[i] * ca / res
            ry = self.frame_r[i] * sa / res
            d_end = self.bilinear_dist(x + ox + rx, y + oy - ry)
            d_mid = self.bilinear_dist(x + ox + 0.5 * rx, y + oy - 0.5 * ry)
            pen = np.clip(d_mid / self.mid_margin_px, 0.0, 1.0)
            total += float(np.mean(np.exp(-0.5 * (d_end / self.sigma_px) ** 2) * pen))
        return -total / n

    def subpixel_refine(self, x, y, yaw):
        """坐标下降亚像素精化: 位置收敛到 <0.1px, 航向收敛到 <0.1°"""
        s_xy, s_yaw = 0.5, math.radians(0.5)
        c = self.cost(x, y, yaw)
        for _ in range(40):
            improved = False
            for dx, dy, dt in ((s_xy, 0.0, 0.0), (-s_xy, 0.0, 0.0),
                               (0.0, s_xy, 0.0), (0.0, -s_xy, 0.0),
                               (0.0, 0.0, s_yaw), (0.0, 0.0, -s_yaw)):
                c2 = self.cost(x + dx, y + dy, yaw + dt)
                if c2 < c - 1e-7:
                    x, y, yaw, c = x + dx, y + dy, yaw + dt, c2
                    improved = True
            if not improved:
                s_xy *= 0.5
                s_yaw *= 0.5
                if s_xy < 0.05:
                    break
        return x, y, yaw, -c

    def perform_relocalization(self):
        self.get_logger().info('performing relocalization...')
        t0 = time.time()

        candidates = self.global_search()
        if not candidates:
            self.get_logger().error('global search found no candidate')
            return

        # 对 top-K 逐一细搜, 去重后取最优与次优
        refined = []
        for i, hyp in enumerate(candidates):
            x, y, yaw, score = self.refine_grid(hyp)
            refined.append((x, y, yaw, score))
            self.get_logger().info(
                f'  candidate {i}: x={x:.2f}px y={y:.2f}px '
                f'yaw={math.degrees(yaw):.1f}° score={score:.3f}')
        deduped = []
        for r_ in refined:
            dup = False
            for d_ in deduped:
                dyaw = abs((r_[2] - d_[2] + math.pi) % (2 * math.pi) - math.pi)
                if abs(r_[0] - d_[0]) < 5.0 and abs(r_[1] - d_[1]) < 5.0 and dyaw < 0.05:
                    dup = True
                    break
            if not dup:
                deduped.append(r_)
        deduped.sort(key=lambda t: -t[3])
        best = deduped[0]
        second_best = deduped[1] if len(deduped) > 1 else None

        # 歧义检查: 全局或精化后的前两名分数太接近, 说明环境对称, 结果不可靠
        ambiguous = (len(candidates) > 1 and candidates[0][3] - candidates[1][3] < 0.05) or \
                    (second_best is not None and best[3] - second_best[3] < 0.03)
        if ambiguous:
            self.get_logger().warn(
                f'top candidates are ambiguous (global {candidates[0][3]:.3f} vs '
                f'{candidates[1][3] if len(candidates) > 1 else 0.0:.3f}, refined '
                f'{best[3]:.3f} vs {second_best[3] if second_best else 0.0:.3f}); '
                f'publishing LARGE covariance — drive/rotate and retry with num_scans>1')

        # 求解的位姿即参考帧(最后一帧)的位姿: 里程计修正已消除旋转弧段的平均航向
        # 偏差, 最后一帧≈当前帧, 无需再做"静止帧"航向修正
        x, y, yaw, score = self.subpixel_refine(best[0], best[1], best[2])
        map_x = x * self.map_resolution + self.map_origin[0]
        map_y = (self.map_height - 1 - y) * self.map_resolution + self.map_origin[1]

        self.get_logger().info(
            f'final pose: x={map_x:.3f}m y={map_y:.3f}m '
            f'(px={x:.1f}, py={y:.1f}) yaw={math.degrees(yaw):.2f}° '
            f'score={score:.3f} total time {time.time() - t0:.2f}s')
        self.get_logger().info('pose published!')

        # 诊断: 若配置了已知真值位姿, 打印其分数与匹配结果的对比
        if self.known_pose_x > -9000.0:
            kx = (self.known_pose_x - self.map_origin[0]) / self.map_resolution
            ky = self.map_height - 1 - (self.known_pose_y - self.map_origin[1]) / self.map_resolution
            k_score = -self.cost(kx, ky, math.radians(self.known_pose_yaw_deg))
            self.known_score = k_score
            self.get_logger().info(
                f'known pose score: {k_score:.3f} (best match: {score:.3f}) — '
                f'if known pose scores much lower, the map does not match the current '
                f'scan (dynamic obstacles?); if similar, the environment is ambiguous')

        # 自适应协方差: 歧义或低置信时发大协方差, 交给 AMCL 用运动消歧
        if ambiguous:
            sig_xy, sig_yaw = 0.50, 0.26
        elif score >= self.high_score:
            sig_xy, sig_yaw = 0.08, 0.05
        elif score >= self.score_threshold:
            sig_xy, sig_yaw = 0.15, 0.10
        else:
            sig_xy, sig_yaw = 0.50, 0.26
            self.get_logger().warn('low match confidence, publishing large covariance')

        if self.save_debug:
            self.save_debug_image(x, y, yaw, map_x, map_y)

        self.publish_pose(map_x, map_y, yaw, sig_xy, sig_yaw)
        self.relocalization_done = True
        self.get_logger().info('relocalization done!')

    def _draw_scan(self, img, px0, py0, yaw0, color, max_pts=5000):
        """按帧偏移把全部扫描帧画到地图上(px0/py0 为参考帧位姿)"""
        res = self.map_resolution
        cy, sy = math.cos(yaw0), math.sin(yaw0)
        total = sum(len(r) for r in self.frame_r)
        dec = max(1, total // max_pts)
        for i in range(len(self.frame_r)):
            ox = (self.frame_dx[i] * cy - self.frame_dy[i] * sy) / res
            oy = -(self.frame_dx[i] * sy + self.frame_dy[i] * cy) / res
            a = self.frame_a[i] + self.frame_dyaw[i]
            px = px0 + ox + self.frame_r[i] * np.cos(a + yaw0) / res
            py = py0 + oy - self.frame_r[i] * np.sin(a + yaw0) / res
            for u, v in zip(px[::dec], py[::dec]):
                cv2.circle(img, (int(u), int(v)), 1, color, -1)

    def save_debug_image(self, x, y, yaw, map_x, map_y):
        """把匹配到的扫描点画到地图上, 存 /tmp/relocalization_debug.png 便于核对"""
        try:
            img = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)
            self._draw_scan(img, x, y, yaw, (0, 0, 255))
            cv2.circle(img, (int(x), int(y)), 5, (0, 255, 0), -1)
            cv2.putText(img, f'x={map_x:.2f} y={map_y:.2f} yaw={math.degrees(yaw):.1f}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            # 已知真值位姿(黄色)对比: 黄色扫描点若散落在空白/障碍内而非贴墙,
            # 说明该位置地图与当前扫描不符(动态障碍/地图过时), 而不是算法选错
            if self.known_pose_x > -9000.0:
                kx = (self.known_pose_x - self.map_origin[0]) / self.map_resolution
                ky = self.map_height - 1 - (self.known_pose_y - self.map_origin[1]) / self.map_resolution
                kyaw = math.radians(self.known_pose_yaw_deg)
                self._draw_scan(img, kx, ky, kyaw, (0, 255, 255))
                cv2.circle(img, (int(kx), int(ky)), 5, (255, 255, 0), -1)
                cv2.putText(img, f'known {self.known_pose_x:.2f},{self.known_pose_y:.2f} '
                            f'yaw={self.known_pose_yaw_deg:.1f} score={self.known_score:.3f}',
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            path = os.path.join('/tmp', 'relocalization_debug.png')
            cv2.imwrite(path, img)
            #self.get_logger().info(f'debug image saved to {path}')
        except Exception as e:
            self.get_logger().warn(f'failed to save debug image: {e}')

    def publish_pose(self, x, y, yaw_rad, sig_xy, sig_yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = sig_xy ** 2
        msg.pose.covariance[7] = sig_xy ** 2
        msg.pose.covariance[35] = sig_yaw ** 2
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'pose published: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw_rad):.2f}°, '
            f'cov sigma_xy={sig_xy} sigma_yaw={sig_yaw}')


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
