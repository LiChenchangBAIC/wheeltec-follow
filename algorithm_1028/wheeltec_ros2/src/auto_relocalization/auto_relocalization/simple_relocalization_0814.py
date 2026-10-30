#!/usr/bin/env python3
"""
Optimized relocalization node for Nav2 / AMCL.
Key improvements over the simple version:
  1. Distance-transform scoring + coverage ratio + ray-penetration penalty
  2. Three-layer search: coarse → medium → fine (much lower miss rate)
  3. ICP-style Gauss-Newton refinement on the distance field
  4. Multi-frame consistency verification
  5. Confidence gating — reject ambiguous matches

Author: optimized version
"""

import os
import math
import time
import yaml

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# helper: Bresenham line iteration
# ---------------------------------------------------------------------------
def bresenham_line(x0, y0, x1, y1):
    """Yield every pixel coordinate along the line from (x0,y0) to (x1,y1)."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points


class SimpleRelocalization(Node):
    def __init__(self):
        super().__init__('simple_relocalization')

        # -------------------------------------------------------------------
        # Parameters
        # -------------------------------------------------------------------
        self.declare_parameter('map_yaml_path',
                               '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.yaml')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('initialpose_topic', '/initialpose')

        # Search region (meters; -1 = no limit)
        self.declare_parameter('search_x_min', -1.0)
        self.declare_parameter('search_x_max', -1.0)
        self.declare_parameter('search_y_min', -1.0)
        self.declare_parameter('search_y_max', -1.0)

        # ---- Coarse search ----
        self.declare_parameter('coarse_pixel_step', 40)
        self.declare_parameter('coarse_yaw_step_deg', 20)
        self.declare_parameter('coarse_top_k', 15)
        self.declare_parameter('coarse_scan_stride', 12)   # rough scoring, sparse points

        # ---- Medium search ----
        self.declare_parameter('medium_pixel_step', 25)
        self.declare_parameter('medium_yaw_step_deg', 12)
        self.declare_parameter('medium_radius_px', 60)
        self.declare_parameter('medium_top_k', 6)

        # ---- Fine search ----
        self.declare_parameter('fine_pixel_step', 12)
        self.declare_parameter('fine_yaw_step_deg', 8)
        self.declare_parameter('fine_radius_px', 35)

        # ---- ICP refinement ----
        self.declare_parameter('icp_max_iterations', 20)
        self.declare_parameter('icp_max_candidates', 3)

        # ---- Scan down-sampling ----
        self.declare_parameter('scan_stride', 4)

        # ---- Scoring ----
        self.declare_parameter('score_threshold', 0.25)
        self.declare_parameter('distance_sigma_px', 4.0)
        self.declare_parameter('coverage_min_ratio', 0.30)
        self.declare_parameter('penetration_max_ratio', 0.40)
        self.declare_parameter('range_sigma_m', 5.0)        # range weighting falloff

        # ---- Multi-frame ----
        self.declare_parameter('multi_frame_enabled', False)
        self.declare_parameter('multi_frame_count', 2)

        # ---- Confidence ----
        self.declare_parameter('confidence_margin', 0.05)   # best must beat 2nd by this
        self.declare_parameter('only_free_space', True)

        # -------------------------------------------------------------------
        # Get parameters
        # -------------------------------------------------------------------
        self.map_yaml_path       = self.get_parameter('map_yaml_path').value
        self.scan_topic          = self.get_parameter('scan_topic').value
        self.initialpose_topic   = self.get_parameter('initialpose_topic').value

        self.search_x_min        = self.get_parameter('search_x_min').value
        self.search_x_max        = self.get_parameter('search_x_max').value
        self.search_y_min        = self.get_parameter('search_y_min').value
        self.search_y_max        = self.get_parameter('search_y_max').value

        self.coarse_pixel_step   = int(self.get_parameter('coarse_pixel_step').value)
        self.coarse_yaw_step_deg = int(self.get_parameter('coarse_yaw_step_deg').value)
        self.coarse_top_k        = int(self.get_parameter('coarse_top_k').value)
        self.coarse_scan_stride  = int(self.get_parameter('coarse_scan_stride').value)

        self.medium_pixel_step   = int(self.get_parameter('medium_pixel_step').value)
        self.medium_yaw_step_deg = int(self.get_parameter('medium_yaw_step_deg').value)
        self.medium_radius_px    = int(self.get_parameter('medium_radius_px').value)
        self.medium_top_k        = int(self.get_parameter('medium_top_k').value)

        self.fine_pixel_step     = int(self.get_parameter('fine_pixel_step').value)
        self.fine_yaw_step_deg   = int(self.get_parameter('fine_yaw_step_deg').value)
        self.fine_radius_px      = int(self.get_parameter('fine_radius_px').value)

        self.icp_max_iterations  = int(self.get_parameter('icp_max_iterations').value)
        self.icp_max_candidates  = int(self.get_parameter('icp_max_candidates').value)

        self.scan_stride         = int(self.get_parameter('scan_stride').value)
        self.score_threshold     = float(self.get_parameter('score_threshold').value)
        self.distance_sigma_px   = float(self.get_parameter('distance_sigma_px').value)
        self.coverage_min_ratio  = float(self.get_parameter('coverage_min_ratio').value)
        self.penetration_max_ratio = float(self.get_parameter('penetration_max_ratio').value)
        self.range_sigma_m       = float(self.get_parameter('range_sigma_m').value)

        self.multi_frame_enabled = bool(self.get_parameter('multi_frame_enabled').value)
        self.multi_frame_count   = int(self.get_parameter('multi_frame_count').value)
        self.confidence_margin   = float(self.get_parameter('confidence_margin').value)
        self.only_free_space     = bool(self.get_parameter('only_free_space').value)

        # -------------------------------------------------------------------
        # State
        # -------------------------------------------------------------------
        self.map_image       = None
        self.map_binary      = None
        self.distance_map    = None
        self.distance_grad_x = None   # pre-computed gradient for ICP
        self.distance_grad_y = None

        self.map_origin      = None
        self.map_resolution  = None
        self.map_width       = 0
        self.map_height      = 0

        self.scan_buffer     = []     # multi-frame buffer
        self.relocalization_done = False
        self.failure_count      = 0      # guard against infinite retry
        self.max_failures       = 5

        # -------------------------------------------------------------------
        # Pub / Sub
        # -------------------------------------------------------------------
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)

        # -------------------------------------------------------------------
        # Load map
        # -------------------------------------------------------------------
        if not self.load_map():
            self.get_logger().error('Failed to load map – relocalization will not work.')

        self.get_logger().info('Optimized relocalization node started.')
        self.get_logger().info(f'Multi-frame: {self.multi_frame_enabled}, '
                               f'frames={self.multi_frame_count}')
        self.get_logger().info('Waiting for laser scan data...')

    # =======================================================================
    # Map loading
    # =======================================================================
    def load_map(self):
        try:
            if not os.path.exists(self.map_yaml_path):
                self.get_logger().error(f'Map YAML not found: {self.map_yaml_path}')
                return False

            with open(self.map_yaml_path, 'r') as f:
                cfg = yaml.safe_load(f)

            self.map_resolution = float(cfg['resolution'])
            self.map_origin     = cfg['origin']

            pgm_path = os.path.join(os.path.dirname(self.map_yaml_path), cfg['image'])
            if not os.path.exists(pgm_path):
                self.get_logger().error(f'PGM not found: {pgm_path}')
                return False

            self.map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if self.map_image is None:
                self.get_logger().error(f'Failed to read PGM: {pgm_path}')
                return False

            self.map_height, self.map_width = self.map_image.shape

            # Binary map: free=255, occupied/unknown=0
            _, self.map_binary = cv2.threshold(self.map_image, 200, 255, cv2.THRESH_BINARY)

            # Distance transform: distance from every free pixel to nearest obstacle
            free_mask = (self.map_binary > 200).astype(np.uint8)
            self.distance_map = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)

            # Pre-compute distance-field gradients for ICP (Sobel)
            self.distance_grad_x = cv2.Sobel(self.distance_map, cv2.CV_32F, 1, 0, ksize=3)
            self.distance_grad_y = cv2.Sobel(self.distance_map, cv2.CV_32F, 0, 1, ksize=3)

            self.get_logger().info(f'Map loaded: {self.map_width}x{self.map_height}, '
                                   f'res={self.map_resolution:.4f} m/px')
            self.get_logger().info(f'Origin: {self.map_origin}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error loading map: {e}')
            return False

    # =======================================================================
    # Coordinate helpers
    # =======================================================================
    def image_to_world(self, px, py):
        ox = self.map_origin[0]
        oy = self.map_origin[1]
        x_m = px * self.map_resolution + ox
        y_m = (self.map_height - 1 - py) * self.map_resolution + oy
        return x_m, y_m

    def world_to_image(self, x_m, y_m):
        ox = self.map_origin[0]
        oy = self.map_origin[1]
        px = int((x_m - ox) / self.map_resolution)
        py = int(self.map_height - 1 - ((y_m - oy) / self.map_resolution))
        return px, py

    def is_in_search_range_world(self, x_m, y_m):
        if self.search_x_min >= 0.0 and x_m < self.search_x_min:
            return False
        if self.search_x_max >= 0.0 and x_m > self.search_x_max:
            return False
        if self.search_y_min >= 0.0 and y_m < self.search_y_min:
            return False
        if self.search_y_max >= 0.0 and y_m > self.search_y_max:
            return False
        return True

    def is_in_bounds(self, px, py):
        return 0 <= px < self.map_width and 0 <= py < self.map_height

    # =======================================================================
    # Scan callbacks / multi-frame logic
    # =======================================================================
    def scan_callback(self, msg):
        if self.relocalization_done:
            return

        if self.map_image is None or self.distance_map is None:
            return

        self.scan_buffer.append(msg)
        needed = self.multi_frame_count if self.multi_frame_enabled else 1

        self.get_logger().info(f'Scan received ({len(self.scan_buffer)}/{needed})')

        if len(self.scan_buffer) >= needed:
            self.perform_relocalization()

    # =======================================================================
    # Main relocalization entry point
    # =======================================================================
    def perform_relocalization(self):
        t0 = time.time()
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Performing relocalization with {len(self.scan_buffer)} frame(s)')

        # ---- Phase 1: search each frame independently ----
        all_frame_results = []
        for i, scan in enumerate(self.scan_buffer):
            self.get_logger().info(f'--- Frame {i+1}/{len(self.scan_buffer)} ---')
            candidates = self.match_scan_with_map(scan)
            all_frame_results.append(candidates)

        # ---- Phase 2: merge multi-frame results ----
        if self.multi_frame_enabled and len(all_frame_results) > 1:
            merged = self._merge_multi_frame_results(all_frame_results)
        else:
            merged = all_frame_results[0]

        if len(merged) == 0:
            self.failure_count += 1
            self.get_logger().error(
                f'No valid candidates found. (failure {self.failure_count}/{self.max_failures})')
            self.scan_buffer.clear()
            if self.failure_count >= self.max_failures:
                self.get_logger().error(
                    'Too many failures — giving up. Check score_threshold and map.')
                self.relocalization_done = True
            return

        # ---- Phase 3: confidence check ----
        best = merged[0]
        best_x, best_y, best_yaw_rad, best_yaw_deg, best_score = best

        if len(merged) >= 2:
            second_score = merged[1][4]
            margin = best_score - second_score
            self.get_logger().info(f'Confidence margin: {margin:.4f} '
                                   f'(best={best_score:.4f}, second={second_score:.4f})')
            if margin < self.confidence_margin:
                self.get_logger().warn(
                    f'Ambiguous match! Margin {margin:.4f} < {self.confidence_margin}. '
                    'Consider collecting more frames or adjusting thresholds.')
                # Still proceed but with higher covariance
                ambiguous = True
            else:
                ambiguous = False
        else:
            ambiguous = False

        # ---- Phase 4: ICP refinement on best candidates ----
        n_refine = min(self.icp_max_candidates, len(merged))
        refined = []
        for i in range(n_refine):
            x_m, y_m, yaw_rad, yaw_deg, score = merged[i]
            ref_x, ref_y, ref_yaw, ref_score = self._icp_refine(
                self.scan_buffer[-1], x_m, y_m, yaw_rad)

            # Directional score for tiebreaking
            dir_score = self._directional_score(self.scan_buffer[-1], ref_x, ref_y, ref_yaw)
            combined = 0.7 * ref_score + 0.3 * dir_score

            refined.append((ref_x, ref_y, ref_yaw, math.degrees(ref_yaw), ref_score, dir_score, combined))
            self.get_logger().info(
                f'ICP refine #{i+1}: ({x_m:.3f},{y_m:.3f})→({ref_x:.3f},{ref_y:.3f}), '
                f'score {score:.4f}→{ref_score:.4f}, dir={dir_score:.3f}, comb={combined:.4f}')

        # Sort by combined score (distance + directional)
        refined.sort(key=lambda c: c[6], reverse=True)

        # ---- Phase 4.5: 180° flip check on best refined candidate ----
        best_r = refined[0]
        bx, by, byaw, byaw_deg, bscore, bdir, bcomb = best_r

        # Try the 180° yaw-flipped version
        flip_yaw = byaw + math.pi
        flip_x, flip_y, flip_yaw_final, flip_score = self._icp_refine(
            self.scan_buffer[-1], bx, by, flip_yaw, max_iters=10)
        flip_dir = self._directional_score(self.scan_buffer[-1], flip_x, flip_y, flip_yaw_final)
        flip_comb = 0.7 * flip_score + 0.3 * flip_dir

        self.get_logger().info(
            f'180° flip check: ({bx:.3f},{by:.3f},{byaw_deg:.1f}°)  '
            f'comb={bcomb:.4f}')
        self.get_logger().info(
            f'  flipped:          ({flip_x:.3f},{flip_y:.3f},{math.degrees(flip_yaw_final):.1f}°)  '
            f'score={flip_score:.4f}, dir={flip_dir:.3f}, comb={flip_comb:.4f}')

        if flip_comb > bcomb:
            self.get_logger().warn(
                '180° flipped pose scores HIGHER — using flipped result!')
            x_final, y_final, yaw_final = flip_x, flip_y, flip_yaw_final
            score_final = flip_score
            yaw_deg_final = math.degrees(flip_yaw_final)
        else:
            x_final, y_final, yaw_final = bx, by, byaw
            score_final = bscore
            yaw_deg_final = byaw_deg

        # ---- Phase 5: validate and publish ----
        if score_final < self.score_threshold:
            self.failure_count += 1
            self.get_logger().error(
                f'Best refined score {score_final:.4f} < threshold {self.score_threshold}. '
                f'(failure {self.failure_count}/{self.max_failures})')
            self.scan_buffer.clear()
            if self.failure_count >= self.max_failures:
                self.get_logger().error('Too many failures — giving up.')
                self.relocalization_done = True
            return

        covariance_scale = 2.0 if ambiguous else 1.0
        self.publish_pose(x_final, y_final, yaw_final, covariance_scale)

        self.relocalization_done = True
        elapsed = time.time() - t0
        self.get_logger().info(f'Relocalization complete in {elapsed:.2f}s')
        self.get_logger().info(f'Final pose: x={x_final:.3f}, y={y_final:.3f}, '
                               f'yaw={yaw_deg_final:.1f}°, score={score_final:.4f}')
        self.scan_buffer.clear()

    # =======================================================================
    # Multi-frame result merging
    # =======================================================================
    def _merge_multi_frame_results(self, all_frame_results):
        """
        Merge results from multiple frames.
        Strategy: score each candidate against all frames, pick the one
        with highest worst-case (minimum) score across frames.
        """
        # Collect all unique candidates across frames (dedup by position)
        all_cands = []
        for frame_cands in all_frame_results:
            for c in frame_cands:
                all_cands.append(c)

        if len(all_cands) == 0:
            return []

        # Deduplicate: 0.5m + 20°
        deduped = []
        for c in all_cands:
            x, y, yaw_rad, yaw_deg, score = c
            dup = False
            for d in deduped:
                dx = x - d[0]
                dy = y - d[1]
                dyaw = abs((yaw_deg - d[3] + 180) % 360 - 180)
                if (dx*dx + dy*dy) < 0.25 and dyaw < 20:
                    dup = True
                    break
            if not dup:
                deduped.append(c)

        # Re-score each unique candidate against ALL frames
        rescored = []
        for c in deduped:
            x_m, y_m, yaw_rad, yaw_deg, _ = c
            px, py = self.world_to_image(x_m, y_m)

            frame_scores = []
            for scan in self.scan_buffer:
                s = self.calculate_match_score(scan, px, py, yaw_rad, check_penetration=True)
                frame_scores.append(s)

            # Use minimum score across frames (worst-case) as the merged score.
            # This penalizes candidates that only look good in one frame.
            min_score = min(frame_scores)
            avg_score = sum(frame_scores) / len(frame_scores)

            # Combined score: 70% worst-case, 30% average
            combined = 0.7 * min_score + 0.3 * avg_score

            rescored.append((x_m, y_m, yaw_rad, yaw_deg, combined))

        rescored.sort(key=lambda c: c[4], reverse=True)
        return rescored

    # =======================================================================
    # Laser scan pre-processing
    # =======================================================================
    def _get_scan_samples(self, scan_msg, stride=None):
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max,
                             len(ranges), dtype=np.float32)

        # Valid: finite, within range, not max-range (no-return)
        valid = (np.isfinite(ranges) &
                 (ranges > scan_msg.range_min) &
                 (ranges < scan_msg.range_max * 0.95))

        s = stride if stride is not None else self.scan_stride
        ranges = ranges[valid][::s]
        angles = angles[valid][::s]

        return ranges, angles

    # =======================================================================
    # Ray penetration check (Bresenham)
    # =======================================================================
    def _check_ray_penetration(self, px_robot, py_robot, px_end, py_end,
                               skip_end_px=4):
        """
        Trace the ray from robot to laser endpoint.
        Return True if any intermediate pixel is occupied (= the ray
        would have been blocked before reaching the endpoint).
        The last `skip_end_px` pixels are ignored since walls have thickness.
        """
        points = bresenham_line(px_robot, py_robot, px_end, py_end)

        if len(points) <= skip_end_px + 2:
            return False  # too short to check

        for x, y in points[1:-skip_end_px]:  # skip robot pos & near-endpoint
            if self.is_in_bounds(x, y) and self.map_binary[y, x] < 200:
                return True
        return False

    # =======================================================================
    # Scoring function (the heart of the algorithm)
    # =======================================================================
    def calculate_match_score(self, scan_msg, map_px, map_py, yaw_rad,
                              check_penetration=False, stride=None):
        """
        Score a candidate pose using the distance transform.

        Returns a score in [0, 1]:
          score = alignment_score * coverage_penalty * penetration_penalty

        - alignment_score:  range-weighted avg(exp(-d²/2σ²))
                            close-range points count more — they are more
                            distinctive and less affected by yaw errors.
        - coverage_penalty:  fraction of scan points that fall inside map bounds
        - penetration_penalty: penalty for rays that pass through walls
          (uses subsampling stride to keep cost manageable)
        """
        ranges, angles = self._get_scan_samples(scan_msg, stride=stride)
        total_points = len(ranges)
        if total_points == 0:
            return 0.0

        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        score_sum = 0.0
        weight_sum = 0.0
        hits_in_map = 0
        penetration_count = 0
        checked_rays = 0

        # Subsample stride for expensive penetration check
        pen_stride = 3

        for i, (r, theta) in enumerate(zip(ranges, angles)):
            # Laser point in robot-local frame
            lx = r * math.cos(theta)
            ly = r * math.sin(theta)

            # Rotate to map frame
            gx = lx * cos_yaw - ly * sin_yaw
            gy = lx * sin_yaw + ly * cos_yaw

            px = int(map_px + gx / self.map_resolution)
            py = int(map_py - gy / self.map_resolution)

            if not self.is_in_bounds(px, py):
                continue  # outside map → counts as zero contribution

            hits_in_map += 1

            # Range-based weight: close = distinctive, far = generic
            w_range = math.exp(-r / self.range_sigma_m)

            # Distance-based score (Gaussian decay)
            dist_px = self.distance_map[py, px]
            s = math.exp(-(dist_px * dist_px) /
                         (2.0 * self.distance_sigma_px * self.distance_sigma_px))
            score_sum += s * w_range
            weight_sum += w_range

            # Optional penetration check (expensive — subsampled & only final stages)
            if check_penetration and i % pen_stride == 0:
                checked_rays += 1
                if self._check_ray_penetration(map_px, map_py, px, py):
                    penetration_count += 1

        if hits_in_map == 0 or weight_sum == 0:
            return 0.0

        # ---- Sub-scores ----
        alignment = score_sum / weight_sum  # range-weighted average
        coverage  = hits_in_map / total_points

        # Hard coverage floor
        if coverage < self.coverage_min_ratio:
            return 0.0

        # Penetration penalty — soft multiplier, never hard-filter
        if check_penetration and checked_rays > 0:
            penetration_ratio = penetration_count / checked_rays
            # Soft penalty: linear ramp from 1.0 (0% pen) to 0.3 (100% pen)
            penetration_factor = 1.0 - penetration_ratio * 0.7
        else:
            penetration_ratio = 0.0
            penetration_factor = 1.0

        # Combined score
        return alignment * coverage * penetration_factor

    def _calculate_match_score_debug(self, scan_msg, map_px, map_py, yaw_rad):
        """
        Like calculate_match_score but returns (score, penetration_ratio, coverage)
        for debug logging.
        """
        ranges, angles = self._get_scan_samples(scan_msg)  # always full stride for debug
        total_points = len(ranges)
        if total_points == 0:
            return 0.0, 0.0, 0.0

        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        pen_stride = 3

        score_sum = 0.0
        weight_sum = 0.0
        hits_in_map = 0
        penetration_count = 0
        checked_rays = 0

        for i, (r, theta) in enumerate(zip(ranges, angles)):
            lx = r * math.cos(theta)
            ly = r * math.sin(theta)
            gx = lx * cos_yaw - ly * sin_yaw
            gy = lx * sin_yaw + ly * cos_yaw
            px = int(map_px + gx / self.map_resolution)
            py = int(map_py - gy / self.map_resolution)

            if not self.is_in_bounds(px, py):
                continue
            hits_in_map += 1

            w_range = math.exp(-r / self.range_sigma_m)

            dist_px = self.distance_map[py, px]
            score_sum += math.exp(-(dist_px * dist_px) /
                                  (2.0 * self.distance_sigma_px * self.distance_sigma_px)) * w_range
            weight_sum += w_range

            if i % pen_stride == 0:
                checked_rays += 1
                if self._check_ray_penetration(map_px, map_py, px, py):
                    penetration_count += 1

        if hits_in_map == 0 or weight_sum == 0:
            return 0.0, 0.0, 0.0
            return 0.0, 0.0, 0.0

        alignment = score_sum / weight_sum
        coverage = hits_in_map / total_points
        pen_ratio = penetration_count / max(checked_rays, 1)
        pen_factor = 1.0 - pen_ratio * 0.7
        score = alignment * coverage * pen_factor
        return score, pen_ratio, coverage

    def _directional_score(self, scan_msg, x_m, y_m, yaw_rad):
        """
        Fraction of laser endpoints where the distance-map gradient points
        OPPOSITE to the laser ray direction.

        For a correct pose, laser rays go from robot TOWARD walls,
        while the distance gradient at the wall points AWAY from walls
        (into free space).  dot(ray, grad) should be < 0.

        For 180°-flipped poses this often degrades, making it a useful
        tiebreaker when distance-field scores are ambiguous.
        """
        ranges, angles = self._get_scan_samples(scan_msg)
        if len(ranges) < 10:
            return 0.5

        ox = self.map_origin[0]
        oy = self.map_origin[1]
        res = self.map_resolution
        H = self.map_height

        consistent = 0
        total = 0

        for r, theta in zip(ranges, angles):
            gx_w = x_m + r * math.cos(theta + yaw_rad)
            gy_w = y_m + r * math.sin(theta + yaw_rad)

            px = int((gx_w - ox) / res)
            py = int(H - 1 - (gy_w - oy) / res)

            if not self.is_in_bounds(px, py):
                continue

            gx_px = self.distance_grad_x[py, px]
            gy_px = self.distance_grad_y[py, px]

            # Ray direction (robot → endpoint)
            ray_dx = math.cos(theta + yaw_rad)
            ray_dy = math.sin(theta + yaw_rad)

            # dot < 0 → ray goes toward wall, gradient goes away from wall ✓
            dot = ray_dx * gx_px + ray_dy * gy_px
            if dot < 0:
                consistent += 1
            total += 1

        return consistent / max(total, 1)

    # =======================================================================
    # Three-layer search: Coarse → Medium → Fine
    # =======================================================================
    def match_scan_with_map(self, scan_msg):
        """
        Three-layer search:
          1. Coarse  — wide grid, large steps, narrows down to general area
          2. Medium  — denser search around each coarse candidate
          3. Fine    — high-resolution search around each medium candidate
        """
        if self.distance_map is None or scan_msg is None:
            return []

        t_start = time.time()

        # ================================================================
        # Layer 1: Coarse search
        # ================================================================
        self.get_logger().info(
            f'[Coarse]  step={self.coarse_pixel_step}px, '
            f'yaw_step={self.coarse_yaw_step_deg}°')
        t1 = time.time()

        coarse_cands = []
        for py in range(self.coarse_pixel_step,
                        self.map_height - self.coarse_pixel_step,
                        self.coarse_pixel_step):
            for px in range(self.coarse_pixel_step,
                            self.map_width - self.coarse_pixel_step,
                            self.coarse_pixel_step):

                x_m, y_m = self.image_to_world(px, py)
                if not self.is_in_search_range_world(x_m, y_m):
                    continue
                if self.only_free_space and self.map_binary[py, px] < 200:
                    continue

                for yaw_deg in range(0, 360, self.coarse_yaw_step_deg):
                    yaw_rad = math.radians(yaw_deg)
                    score = self.calculate_match_score(scan_msg, px, py, yaw_rad,
                                                       check_penetration=False,
                                                       stride=self.coarse_scan_stride)
                    if score > self.score_threshold:
                        coarse_cands.append(
                            (x_m, y_m, yaw_rad, yaw_deg, score, px, py))

        coarse_cands.sort(key=lambda c: c[4], reverse=True)
        coarse_cands = coarse_cands[:self.coarse_top_k]

        self.get_logger().info(
            f'[Coarse]  {len(coarse_cands)} candidates in {time.time()-t1:.2f}s')

        if len(coarse_cands) == 0:
            self.get_logger().warn('[Coarse] No candidates found – try lowering '
                                   'score_threshold or coarse_yaw_step_deg')
            return []

        # ================================================================
        # Layer 2: Medium search around each coarse candidate
        # ================================================================
        self.get_logger().info(
            f'[Medium] step={self.medium_pixel_step}px, '
            f'yaw_step={self.medium_yaw_step_deg}°, '
            f'radius={self.medium_radius_px}px')
        t2 = time.time()

        medium_cands = []
        for cand in coarse_cands:
            _, _, _, yaw_deg, _, cpx, cpy = cand

            for dy in range(-self.medium_radius_px,
                            self.medium_radius_px + 1,
                            self.medium_pixel_step):
                for dx in range(-self.medium_radius_px,
                                self.medium_radius_px + 1,
                                self.medium_pixel_step):
                    px = cpx + dx
                    py = cpy + dy

                    if not self.is_in_bounds(px, py):
                        continue
                    x_m, y_m = self.image_to_world(px, py)
                    if not self.is_in_search_range_world(x_m, y_m):
                        continue
                    if self.only_free_space and self.map_binary[py, px] < 200:
                        continue

                    yaw_range = range(max(0, yaw_deg - 30),
                                      min(360, yaw_deg + 31),
                                      self.medium_yaw_step_deg)
                    for yaw_d in yaw_range:
                        yaw_r = math.radians(yaw_d)
                        score = self.calculate_match_score(scan_msg, px, py, yaw_r,
                                                           check_penetration=False)
                        if score > self.score_threshold:
                            medium_cands.append(
                                (x_m, y_m, yaw_r, yaw_d, score, px, py))

        medium_cands.sort(key=lambda c: c[4], reverse=True)
        medium_cands = medium_cands[:self.medium_top_k]

        self.get_logger().info(
            f'[Medium] {len(medium_cands)} candidates in {time.time()-t2:.2f}s')

        if len(medium_cands) == 0:
            self.get_logger().warn('[Medium] No candidates – falling back to coarse.')
            # Fallback: use coarse candidates for fine search
            medium_cands = coarse_cands[:self.medium_top_k]

        # ================================================================
        # Layer 3: Fine search around each medium candidate
        # ================================================================
        self.get_logger().info(
            f'[Fine]   step={self.fine_pixel_step}px, '
            f'yaw_step={self.fine_yaw_step_deg}°, '
            f'radius={self.fine_radius_px}px')
        t3 = time.time()

        fine_cands = []
        for cand in medium_cands:
            _, _, _, yaw_deg, _, cpx, cpy = cand

            for dy in range(-self.fine_radius_px,
                            self.fine_radius_px + 1,
                            self.fine_pixel_step):
                for dx in range(-self.fine_radius_px,
                                self.fine_radius_px + 1,
                                self.fine_pixel_step):
                    px = cpx + dx
                    py = cpy + dy

                    if not self.is_in_bounds(px, py):
                        continue
                    x_m, y_m = self.image_to_world(px, py)
                    if not self.is_in_search_range_world(x_m, y_m):
                        continue
                    if self.only_free_space and self.map_binary[py, px] < 200:
                        continue

                    yaw_range = range(max(0, yaw_deg - 20),
                                      min(360, yaw_deg + 21),
                                      self.fine_yaw_step_deg)
                    for yaw_d in yaw_range:
                        yaw_r = math.radians(yaw_d)
                        # Fast scoring — penetration check is too expensive for
                        # the fine-search inner loop; we verify afterwards.
                        score = self.calculate_match_score(scan_msg, px, py, yaw_r,
                                                           check_penetration=False)
                        if score > self.score_threshold:
                            fine_cands.append(
                                (x_m, y_m, yaw_r, yaw_d, score))

        self.get_logger().info(
            f'[Fine]   {len(fine_cands)} candidates in {time.time()-t3:.2f}s')

        # ---- Sort & dedup (fast scores) ----
        fine_cands.sort(key=lambda c: c[4], reverse=True)

        deduped = []
        for c in fine_cands:
            x, y, yaw_rad, yaw_deg, score = c
            dup = False
            for d in deduped:
                dx = x - d[0]
                dy = y - d[1]
                dyaw = abs((yaw_deg - d[3] + 180) % 360 - 180)
                if (dx*dx + dy*dy) < 0.04 and dyaw < 5:  # 0.2m, 5°
                    dup = True
                    break
            if not dup:
                deduped.append(c)

        # Keep a larger pool before penetration verification
        pool_size = max(self.coarse_top_k, self.medium_top_k) * 3
        deduped_pool = deduped[:pool_size]

        # ---- Penetration verification (only on top pool, not inner loop) ----
        self.get_logger().info(
            f'[Verify] penetration check on top {len(deduped_pool)} candidates')
        t4 = time.time()

        verified = []
        for c in deduped_pool:
            x_m, y_m, yaw_rad, yaw_deg, fast_score = c
            px, py = self.world_to_image(x_m, y_m)
            # Re-score WITH penetration check, returning debug info
            pen_score, pen_ratio, cov_ratio = self._calculate_match_score_debug(
                scan_msg, px, py, yaw_rad)
            self.get_logger().info(
                f'  verify #{len(verified)+1}: fast={fast_score:.3f} → pen={pen_score:.3f} '
                f'(pen_ratio={pen_ratio:.2f}, cov={cov_ratio:.2f}) '
                f'pose=({x_m:.2f},{y_m:.2f},{yaw_deg}°)')
            if pen_score > self.score_threshold:
                verified.append((x_m, y_m, yaw_rad, yaw_deg, pen_score))

        verified.sort(key=lambda c: c[4], reverse=True)
        verified = verified[:max(self.coarse_top_k, self.medium_top_k)]

        self.get_logger().info(
            f'[Verify] {len(verified)} passed penetration check in {time.time()-t4:.2f}s')

        elapsed = time.time() - t_start
        self.get_logger().info(f'Search total: {elapsed:.2f}s, '
                               f'final candidates: {len(verified)}')

        for i, (x, y, yaw_rad, yaw_deg, score) in enumerate(verified):
            self.get_logger().info(
                f'  #{i+1}: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}°, score={score:.4f}')

        return verified

    # =======================================================================
    # ICP-style Gauss-Newton refinement on the distance field
    # =======================================================================
    def _icp_refine(self, scan_msg, x_init, y_init, yaw_init, max_iters=None):
        """
        Refine a candidate pose using Gauss-Newton optimization on the
        distance-transform field.

        Minimizes:  Σ d_i²(x, y, θ)
        where d_i is the distance-transform value at the i-th laser point.

        Uses pre-computed Sobel gradients of the distance map for the Jacobian.
        """
        ranges, angles = self._get_scan_samples(scan_msg)
        if len(ranges) < 10:
            return x_init, y_init, yaw_init, 0.0

        x, y, yaw = float(x_init), float(y_init), float(yaw_init)
        lam = 0.5       # damping factor (Levenberg-Marquardt style)
        prev_score = -1.0

        n_iters = max_iters if max_iters is not None else self.icp_max_iterations
        for iteration in range(n_iters):
            # Build Jacobian and residual
            J_rows = []   # list of [∂d/∂x, ∂d/∂y, ∂d/∂θ]
            residuals = []
            score_sum = 0.0
            valid_count = 0

            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            # Robot position in pixels
            px_r, py_r = self.world_to_image(x, y)

            for r, theta in zip(ranges, angles):
                # World coordinates of laser endpoint
                lx = r * math.cos(theta)
                ly = r * math.sin(theta)
                gx = x + lx * cos_yaw - ly * sin_yaw
                gy = y + lx * sin_yaw + ly * cos_yaw

                # Pixel coordinates
                px = int((gx - self.map_origin[0]) / self.map_resolution)
                py = int(self.map_height - 1 -
                         ((gy - self.map_origin[1]) / self.map_resolution))

                if not self.is_in_bounds(px, py):
                    continue

                d = self.distance_map[py, px]      # distance value (pixels)

                # Only use points reasonably close to obstacles
                if d > 30.0:
                    continue

                # Gradient of distance map at this pixel
                gx_px = self.distance_grad_x[py, px]   # ∂d/∂px
                gy_px = self.distance_grad_y[py, px]   # ∂d/∂py

                # Jacobian: ∂d/∂pose = [∂d/∂x, ∂d/∂y, ∂d/∂θ]
                # ∂d/∂x = (∂d/∂px)*(∂px/∂x_w)*(∂x_w/∂x) = gx_px * (1/res) * 1
                # ∂d/∂y = gy_px * (-1/res) * 1
                # ∂d/∂θ = gx_px * ∂px/∂θ + gy_px * ∂py/∂θ
                j_x = gx_px / self.map_resolution
                j_y = -gy_px / self.map_resolution

                # ∂px/∂θ = -r*sin(θ+yaw) / res,  ∂py/∂θ = -r*cos(θ+yaw) / res
                dpx_dt = -r * math.sin(theta + yaw) / self.map_resolution
                dpy_dt = -r * math.cos(theta + yaw) / self.map_resolution
                j_t = gx_px * dpx_dt + gy_px * dpy_dt

                J_rows.append([j_x, j_y, j_t])
                residuals.append(d)
                score_sum += math.exp(-(d*d) / (2.0 * self.distance_sigma_px * self.distance_sigma_px))
                valid_count += 1

            if valid_count < 10:
                break

            # Current score
            cur_score = score_sum / valid_count if valid_count > 0 else 0.0

            # Check convergence
            if prev_score > 0 and cur_score <= prev_score + 1e-6:
                # Not improving, reduce damping or stop
                if lam < 1e-4 or cur_score < prev_score + 1e-8:
                    break
                lam *= 0.5
                continue

            prev_score = cur_score

            # Build normal equations: (JᵀJ + λ·diag(JᵀJ)) Δ = -Jᵀr
            J = np.array(J_rows, dtype=np.float64)
            r = np.array(residuals, dtype=np.float64)

            H = J.T @ J                               # 3×3 Hessian approximation
            g_vec = J.T @ r                            # 3×1 gradient

            # Damped Hessian
            H_damped = H + lam * np.diag(np.diag(H))

            try:
                delta = -np.linalg.solve(H_damped, g_vec)
            except np.linalg.LinAlgError:
                self.get_logger().warn(f'ICP iter {iteration}: singular Hessian, breaking')
                break

            # Step-size limiting
            max_dx = 0.3   # meters
            max_dyaw = 0.2   # radians (~11°)
            delta[0] = max(-max_dx, min(max_dx, delta[0]))
            delta[1] = max(-max_dx, min(max_dx, delta[1]))
            delta[2] = max(-max_dyaw, min(max_dyaw, delta[2]))

            # Update
            x += delta[0]
            y += delta[1]
            yaw += delta[2]

            # Normalize yaw to [-π, π]
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))

            # Adapt damping
            if abs(delta[0]) < 0.005 and abs(delta[1]) < 0.005 and abs(delta[2]) < 0.002:
                # Converged
                break
            lam = max(lam * 0.7, 1e-4)

        # Final score
        px_f, py_f = self.world_to_image(x, y)
        final_score = self.calculate_match_score(scan_msg, px_f, py_f, yaw,
                                                 check_penetration=True)

        return x, y, yaw, final_score

    # =======================================================================
    # Publish
    # =======================================================================
    def publish_pose(self, x, y, yaw_rad, covariance_scale=1.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        base_cov_xy = 0.25
        base_cov_yaw = 0.0685
        msg.pose.covariance = [
            base_cov_xy * covariance_scale, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, base_cov_xy * covariance_scale, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, base_cov_yaw * covariance_scale,
        ]

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'Pose published to {self.initialpose_topic} '
                               f'(cov_scale={covariance_scale:.1f})')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRelocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('User interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

