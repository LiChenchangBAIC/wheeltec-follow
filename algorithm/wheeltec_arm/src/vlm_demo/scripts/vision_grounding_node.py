#!/usr/bin/env python3
"""批量视觉定位节点：图像 + 目标描述 -> 目标像素坐标。

对内提供 /vlm_demo/ground_targets 服务供 pick_task_manager 批量定位；
对外提供 /vlm_demo/chat_query 话题做画面对话。
本节点只输出像素坐标，3D 反投影由 geometry_resolver_node 负责。
"""

import base64
import json
import os
import threading

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vlm_demo.srv import GroundTargets


class VisionGroundingNode(Node):
    """批量视觉定位节点。

    职责边界：本节点只做"图像 + 目标描述 -> 目标像素坐标"，不碰 3D、不碰机器人。
    像素到 3D 的反投影由 geometry_resolver_node 负责，抓取/移动由 pick_task_manager 负责。

    两条业务线共用同一份相机图像缓存：
      (A) 任务定位服务 /vlm_demo/ground_targets ：被 pick_task_manager 调用，批量返回 bbox/grasp 像素。
      (B) 画面对话    /vlm_demo/chat_query        ：被 command_node 的 /chat 转发触发，自由问答。
    """

    def __init__(self):
        super().__init__('vision_grounding_node')

        # ---- 订阅与发布话题名（可由 launch 配置覆盖） ----
        self.declare_parameter('image_topic', '/camera_arm/color/image_raw')
        self.declare_parameter('api_base', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('api_key_env', 'DASHSCOPE_API_KEY')
        self.declare_parameter('model', 'qwen-vl-max-latest')
        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('result_topic', '/vlm_demo/grounding_result')
        self.declare_parameter('debug_image_topic', '/vlm_demo/grounding_debug_image')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('coordinate_mode', 'pixel')
        self.declare_parameter('chat_topic', '/vlm_demo/chat_query')
        self.declare_parameter('chat_response_topic', '/vlm_demo/chat_response')

        self.image_topic_ = self.get_parameter('image_topic').value
        self.api_base_ = self.get_parameter('api_base').value.rstrip('/')
        api_key_env = self.get_parameter('api_key_env').value
        self.model_ = self.get_parameter('model').value
        self.timeout_sec_ = float(self.get_parameter('timeout_sec').value)
        self.jpeg_quality_ = int(self.get_parameter('jpeg_quality').value)
        self.result_topic_ = self.get_parameter('result_topic').value
        self.debug_image_topic_ = self.get_parameter('debug_image_topic').value
        self.publish_debug_image_ = bool(self.get_parameter('publish_debug_image').value)
        # 坐标模式：norm_1000 = 让 VLM 吐 0~1000 归一化坐标(对齐 Qwen-VL)，本节点再映射回像素
        self.coordinate_mode_ = self._normalize_coordinate_mode(
            self.get_parameter('coordinate_mode').value)
        self.chat_topic_ = self.get_parameter('chat_topic').value
        self.chat_response_topic_ = self.get_parameter('chat_response_topic').value

        # API key 从环境变量读取；为空时仅警告，真正调用时才会失败
        self.api_key_ = os.environ.get(api_key_env, '')
        if not self.api_key_:
            self.get_logger().warn(
                f'Environment variable {api_key_env} is empty; VLM calls will fail.')

        self.bridge_ = CvBridge()
        # 最新一帧彩色图像及其 ROS header，定位/对话都从这里取图
        self.latest_bgr_ = None
        self.latest_header_ = None
        self.image_lock_ = threading.Lock()  # 订阅回调与服务回调跨线程，需加锁

        # 结果发布：定位 JSON、调试可视化图、对话回复
        self.result_pub_ = self.create_publisher(String, self.result_topic_, 10)
        self.debug_image_pub_ = self.create_publisher(Image, self.debug_image_topic_, 10)
        self.chat_response_pub_ = self.create_publisher(String, self.chat_response_topic_, 10)

        # 订阅：彩色图像流 + 对话请求
        self.image_sub_ = self.create_subscription(
            Image, self.image_topic_, self._image_cb, 10)
        self.chat_sub_ = self.create_subscription(
            String, self.chat_topic_, self._on_chat_query, 10)
        # 服务：批量视觉定位（被 pick_task_manager 调用）
        self.srv_ = self.create_service(
            GroundTargets, '/vlm_demo/ground_targets', self._handle_ground_targets)

        self.get_logger().info(
            f'Batch visual grounding node ready. model={self.model_}, service=/vlm_demo/ground_targets')
        self.get_logger().info(
            f'Visual chat ready. query={self.chat_topic_}, response={self.chat_response_topic_}')

    def _image_cb(self, msg: Image):
        """彩色图像订阅回调：转成 BGR 并缓存最新一帧，供定位/对话取用。"""
        try:
            cv_img = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
            with self.image_lock_:
                self.latest_bgr_ = cv_img.copy()
                self.latest_header_ = msg.header
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')

    def _on_chat_query(self, msg: String):
        """对话请求回调：开后台线程跑 VLM 问答，避免阻塞服务/订阅回调。"""
        question = msg.data.strip()
        if not question:
            self.chat_response_pub_.publish(String(data='请输入 /chat 后面的对话内容。'))
            return

        worker = threading.Thread(
            target=self._run_chat_query,
            args=(question,),
            daemon=True)
        worker.start()

    def _run_chat_query(self, question: str):
        """后台线程：取当前帧 -> 调 VLM 对话模型 -> 发布回复。"""
        with self.image_lock_:
            bgr = None if self.latest_bgr_ is None else self.latest_bgr_.copy()

        if bgr is None:
            self.chat_response_pub_.publish(String(data='我还没有收到相机图像，暂时看不到画面。'))
            return

        try:
            answer = self._call_chat_model(bgr, question)
        except Exception as exc:
            answer = f'视觉对话失败: {exc}'
            self.get_logger().error(answer)

        self.chat_response_pub_.publish(String(data=answer))

    def _handle_ground_targets(self, req, res):
        """批量视觉定位服务回调（核心）。

        流程：取最新帧 -> 编码 base64 -> 调 VLM -> 解析 JSON ->
              映射坐标 -> 填充响应 -> 发布结果与调试图。
        """
        queries = [str(q).strip() for q in req.queries if str(q).strip()]
        if not queries:
            res.success = False
            res.message = 'empty target query list'
            return res

        # 取当前帧（带锁拷贝，避免与订阅回调竞争）
        with self.image_lock_:
            if self.latest_bgr_ is None:
                res.success = False
                res.message = 'no color image received yet'
                return res
            bgr = self.latest_bgr_.copy()
            header = self.latest_header_

        raw = ''
        try:
            raw = self._call_model(bgr, queries)
            parsed = json.loads(self._extract_json(raw))
            items = self._extract_batch_items(parsed, queries)

            # 逐 query 解析字段并映射到像素坐标
            for i, query in enumerate(queries):
                item = items[i] if i < len(items) and isinstance(items[i], dict) else {}
                fields = self._parse_grounding_fields(
                    item, bgr.shape[1], bgr.shape[0], self.coordinate_mode_)

                res.found.append(fields['found'])
                res.ambiguous.append(fields['ambiguous'])
                res.questions.append(fields['question'])
                res.target_labels.append(fields['target_label'] or query)
                res.confidences.append(float(fields['confidence']))
                res.x1.append(int(fields['x1']))
                res.y1.append(int(fields['y1']))
                res.x2.append(int(fields['x2']))
                res.y2.append(int(fields['y2']))
                res.grasp_u.append(int(fields['grasp_u']))
                res.grasp_v.append(int(fields['grasp_v']))

            res.success = True
            res.message = f'grounded {len(queries)} target(s)'
            res.raw_json = json.dumps(parsed, ensure_ascii=False)
            # 旁路发布：结构化结果 + 可视化图，供调试/记录
            self._publish_grounding_result(
                queries, parsed, res, bgr.shape[1], bgr.shape[0], self.coordinate_mode_)
            self._publish_debug_image(bgr, header, res)
            self.get_logger().info(
                f'Batch grounding result: count={len(queries)} coord_mode={self.coordinate_mode_}')
        except Exception as exc:
            # 整批失败：回填失败信息，把原始返回也带上便于排查
            res.success = False
            res.message = f'batch visual grounding failed: {exc}'
            res.raw_json = raw
            self._publish_grounding_error(queries, res.message, raw)
            self.get_logger().error(res.message)

        return res

    def _publish_grounding_result(self, queries, parsed, res, width, height, coordinate_mode):
        """旁路发布结构化定位结果 JSON（便于外部订阅调试/记录，与服务响应同源）。"""
        targets = []
        for i, query in enumerate(queries):
            targets.append({
                'query': query,
                'found': bool(res.found[i]),
                'ambiguous': bool(res.ambiguous[i]),
                'question': res.questions[i],
                'target_label': res.target_labels[i],
                'confidence': float(res.confidences[i]),
                'bbox_2d': [res.x1[i], res.y1[i], res.x2[i], res.y2[i]],
                'grasp_point_2d': [res.grasp_u[i], res.grasp_v[i]],
            })

        payload = {
            'success': res.success,
            'queries': list(queries),
            'image_width': width,
            'image_height': height,
            'coordinate_mode': coordinate_mode,
            'targets': targets,
            'raw': parsed,
        }
        self.result_pub_.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_grounding_error(self, queries, message, raw):
        """失败时旁路发布错误 JSON（带原始返回文本，方便离线排查模型输出）。"""
        payload = {
            'success': False,
            'queries': list(queries),
            'message': message,
            'raw_text': raw,
        }
        self.result_pub_.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_debug_image(self, bgr, header, res):
        """在原图上画 bbox 框 + grasp 十字点 + 标签，发布为调试可视化图。"""
        if not self.publish_debug_image_:
            return

        debug = bgr.copy()
        # 多目标用不同颜色区分
        colors = [
            (0, 255, 0),
            (255, 0, 0),
            (0, 255, 255),
            (255, 0, 255),
            (255, 255, 0),
            (0, 128, 255),
        ]

        for i, found in enumerate(res.found):
            if not found:
                continue

            color = colors[i % len(colors)]
            x1, y1, x2, y2 = res.x1[i], res.y1[i], res.x2[i], res.y2[i]
            u, v = res.grasp_u[i], res.grasp_v[i]
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            cv2.drawMarker(
                debug, (u, v), color,
                markerType=cv2.MARKER_CROSS,
                markerSize=24,
                thickness=2)
            label = self._debug_label(res.target_labels[i], res.confidences[i])
            cv2.putText(
                debug,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA)

        msg = self.bridge_.cv2_to_imgmsg(debug, encoding='bgr8')
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_image_pub_.publish(msg)

    @staticmethod
    def _debug_label(label, confidence):
        """调试图上的文字标签：非 ASCII 名统一显示为 target，避免 cv2 乱码。"""
        if not label or not str(label).isascii():
            label = 'target'
        return f'{label} {float(confidence):.2f}'

    def _call_model(self, bgr, queries):
        """调用视觉定位大模型（批量）。
  
        把当前帧 JPEG 编码成 base64 内联进 OpenAI 兼容请求，
        prompt 强约束模型只返回严格 JSON（targets 数组，顺序与 queries 对齐）。
        """
        quality = max(20, min(self.jpeg_quality_, 100))
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError('JPEG encode failed')

        b64 = base64.b64encode(buf.tobytes()).decode('ascii')
        image_url = f'data:image/jpeg;base64,{b64}'
        height, width = bgr.shape[:2]
        prompt = self._build_prompt(queries, width, height)

        payload = {
            'model': self.model_,
            'messages': [{
                'role': 'user',
                'content': [
                    {'type': 'text', 'text': prompt},
                    {'type': 'image_url', 'image_url': {'url': image_url}},
                ],
            }],
            'temperature': 0.0,
            'stream': False,
        }
        headers = {
            'Authorization': f'Bearer {self.api_key_}',
            'Content-Type': 'application/json',
        }

        resp = requests.post(
            f'{self.api_base_}/chat/completions',
            json=payload,
            headers=headers,
            timeout=self.timeout_sec_,
        )
        if resp.status_code < 200 or resp.status_code >= 300:
            raise RuntimeError(f'model API HTTP {resp.status_code}: {resp.text}')

        data = resp.json()
        return data['choices'][0]['message']['content']

    def _call_chat_model(self, bgr, question):
        """调用画面对话模型：自由问答，prompt 约束只描述看到的内容、不承诺已执行动作。"""
        quality = max(20, min(self.jpeg_quality_, 100))
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError('JPEG encode failed')
        
        b64 = base64.b64encode(buf.tobytes()).decode('ascii')
        image_url = f'data:image/jpeg;base64,{b64}'
        height, width = bgr.shape[:2]
        prompt = (
            '你是机器人当前相机画面的中文视觉助手。'
            '请根据图像回答用户问题，回答要简短、直接。'
            '如果问题涉及是否能抓取或执行动作，只能描述你看到的物体和大致位置，'
            '不要承诺已经执行机器人动作。\n'
            f'图像尺寸: width={width}, height={height}\n'
            f'用户问题: {question}'
        )

        payload = {
            'model': self.model_,
            'messages': [{
                'role': 'user',
                'content': [
                    {'type': 'text', 'text': prompt},
                    {'type': 'image_url', 'image_url': {'url': image_url}},
                ],
            }],
            'temperature': 0.2,
            'stream': False,
        }
        headers = {
            'Authorization': f'Bearer {self.api_key_}',
            'Content-Type': 'application/json',
        }

        resp = requests.post(
            f'{self.api_base_}/chat/completions',
            json=payload,
            headers=headers,
            timeout=self.timeout_sec_,
        )
        if resp.status_code < 200 or resp.status_code >= 300:
            raise RuntimeError(f'model API HTTP {resp.status_code}: {resp.text}')

        data = resp.json()
        return data['choices'][0]['message']['content'].strip()

    def _build_prompt(self, queries, width, height):
        """构造视觉定位的 prompt：固定 JSON schema + 按坐标模式给坐标规则。

        要点：targets 数组长度/顺序必须与 queries 一致；bbox 紧贴物体；
              grasp 点在物体中心；找不到则 found=false + 空坐标；
              QR/标签块只定位可见块、不解码 id。
        """
        # norm_1000 模式要求模型吐 0~1000 归一化整数，本节点再换算回像素
        if self.coordinate_mode_ == 'norm_1000':
            coord_rule = (
                'Coordinates must be normalized integers from 0 to 1000. '
                'Top-left is [0,0], bottom-right is [1000,1000]. '
                'Do not output pixel coordinates.'
            )
        else:
            coord_rule = (
                f'Coordinates must be original pixel coordinates. '
                f'Top-left is [0,0], bottom-right is [{width - 1},{height - 1}].'
            )

        query_text = json.dumps(list(queries), ensure_ascii=False)
        return (
            'You are a robot visual grounding module. Locate every requested target '
            'in the current image. Do not plan robot actions. Do not output Markdown. '
            'Return strict JSON only.\n'
            f'Image size: width={width}, height={height}.\n'
            f'Target queries, in required output order: {query_text}\n'
            f'{coord_rule}\n'
            'Return exactly this schema:\n'
            '{"targets":['
            '{"query":"same query text",'
            '"found":true/false,'
            '"ambiguous":true/false,'
            '"question":"short clarification question if ambiguous, else empty string",'
            '"target_label":"short target name",'
            '"confidence":0.0,'
            '"bbox_2d":[x1,y1,x2,y2],'
            '"grasp_point_2d":[u,v]}'
            ']}\n'
            'Rules: targets array length must equal the number of queries and keep the same order. '
            'bbox_2d must tightly cover the visible object body, not shadows, reflections, or background. '
            'grasp_point_2d should be near the center of the target body. '
            'If a requested target is not visible, set found=false and use empty bbox/grasp arrays. '
            'For QR-code/label blocks, locate the visible block only; do not decode or infer its id.'
        )

    @staticmethod
    def _extract_json(text):
        """从模型输出中截取最外层花括号之间的内容（容忍前后多余文字/解释）。"""
        first = text.find('{')
        last = text.rfind('}')
        if first == -1 or last == -1 or last <= first:
            raise RuntimeError('model response does not contain a JSON object')
        return text[first:last + 1]

    @staticmethod
    def _extract_batch_items(parsed, queries):
        """从解析结果中取出 targets 列表，长度不足则补空对象，保证与 queries 对齐。"""
        if isinstance(parsed, dict):
            items = parsed.get('targets', [])
        elif isinstance(parsed, list):
            items = parsed
        else:
            items = []

        if not isinstance(items, list):
            items = []

        if len(items) < len(queries):
            items = items + [{} for _ in range(len(queries) - len(items))]

        return items

    @staticmethod
    def _parse_grounding_fields(parsed, width, height, coordinate_mode):
        """把单个 target 的原始 JSON 字段规整成像素坐标字段。

        统一处理：坐标模式映射、bbox 规整(clip + 大小翻转)、
        缺失 grasp_point 时退化为 bbox 中心。
        """
        found = bool(parsed.get('found', False))
        ambiguous = bool(parsed.get('ambiguous', False))
        question = str(parsed.get('question', ''))
        target_label = str(parsed.get('target_label', 'vlm_target'))
        confidence = float(parsed.get('confidence', 0.0))

        bbox = parsed.get('bbox_2d', [])
        x2_default = 1000.0 if coordinate_mode == 'norm_1000' else float(width - 1)
        y2_default = 1000.0 if coordinate_mode == 'norm_1000' else float(height - 1)

        x1 = VisionGroundingNode._coord_to_pixel(
            VisionGroundingNode._float_at(bbox, 0, 0.0), width, coordinate_mode)
        y1 = VisionGroundingNode._coord_to_pixel(
            VisionGroundingNode._float_at(bbox, 1, 0.0), height, coordinate_mode)
        x2 = VisionGroundingNode._coord_to_pixel(
            VisionGroundingNode._float_at(bbox, 2, x2_default), width, coordinate_mode)
        y2 = VisionGroundingNode._coord_to_pixel(
            VisionGroundingNode._float_at(bbox, 3, y2_default), height, coordinate_mode)

        x1 = int(np.clip(round(x1), 0, max(0, width - 1)))
        y1 = int(np.clip(round(y1), 0, max(0, height - 1)))
        x2 = int(np.clip(round(x2), 0, max(0, width - 1)))
        y2 = int(np.clip(round(y2), 0, max(0, height - 1)))
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1

        grasp = parsed.get('grasp_point_2d', [])
        if len(grasp) >= 2 and grasp[0] is not None and grasp[1] is not None:
            u = VisionGroundingNode._coord_to_pixel(
                VisionGroundingNode._float_at(grasp, 0, 0.0), width, coordinate_mode)
            v = VisionGroundingNode._coord_to_pixel(
                VisionGroundingNode._float_at(grasp, 1, 0.0), height, coordinate_mode)
        else:
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0

        return {
            'found': found,
            'ambiguous': ambiguous,
            'question': question,
            'target_label': target_label,
            'confidence': confidence,
            'x1': x1,
            'y1': y1,
            'x2': x2,
            'y2': y2,
            'grasp_u': int(np.clip(round(u), 0, max(0, width - 1))),
            'grasp_v': int(np.clip(round(v), 0, max(0, height - 1))),
        }

    @staticmethod
    def _normalize_coordinate_mode(mode):
        """把配置里的坐标模式别名归一成内部两档：norm_1000 / pixel。"""
        value = str(mode or 'pixel').lower()
        if value in ('norm_1000', 'normalized_1000', 'qwen'):
            return 'norm_1000'
        return 'pixel'

    @staticmethod
    def _coord_to_pixel(value, axis_size, coordinate_mode):
        """把模型返回的坐标换算成像素：norm_1000 模式按 value*(size-1)/1000 缩放。"""
        if coordinate_mode == 'norm_1000':
            return value * max(0, axis_size - 1) / 1000.0
        return value

    @staticmethod
    def _float_at(values, idx, default):
        """安全地从列表取第 idx 个浮点越界则返回默认值。"""
        if isinstance(values, (list, tuple)) and idx < len(values) and values[idx] is not None:
            return float(values[idx])
        return default


def main(args=None):
    rclpy.init(args=args)
    node = VisionGroundingNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
