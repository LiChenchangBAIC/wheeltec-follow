#!/usr/bin/env python3
"""Plan natural-language robot commands and execute arm/base/VLM steps.

任务管理节点：接收用户自然语言指令，调用大模型规划成 JSON 步骤序列，
再逐步执行机械臂 / 底盘导航 / 视觉定位等技能。
"""

import json
import math
import os
import queue
import re
import threading
import time
import traceback
from dataclasses import dataclass, field

import requests
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from grab_demo.srv import (
    GrabObject,
    MoveCartesian,
    MoveJoints,
    MoveNamed,
    MovePose,
    MoveRelative,
    MoveToFrame,
    PlaceObject,
    PlaceToFrame,
)
from vlm_demo.srv import GroundTargets, ResolveTargetPose


TASK_PROMPT = """
你是机器人任务规划器。用户输入中文自然语言，你只负责输出机器人可执行 JSON。
不要解释，不要输出 Markdown，不要输出代码块，只输出一个 JSON 对象。

总原则：
- 根据用户本次指令自由组合技能；步骤数量、顺序、目标名称都由用户语义决定。
- 用户说了几个连续动作，steps 就按原顺序包含几个动作，不要合并、跳过或改顺序。
- 纯机械臂移动、关节转动、等待、底盘导航，不要使用视觉定位。
- 只有需要寻找当前相机图像里的外观目标时，才使用 locate_vlm。
- 简单“夹取/抓取/拿起某目标”任务：先 locate_vlm，再直接 pick；
  不要额外插入 move_to_frame 到目标上方。只有用户明确要求“到目标上方/靠近/先移动到目标附近”时才使用 move_to_frame。

可用技能：

move_named:
  机械臂移动到 MoveIt SRDF 已有命名点。
  字段: {"skill":"move_named","target_name":"look"}

move_relative:
  机械臂末端相对移动。
  字段: {"skill":"move_relative","frame":"base_link","dx":0.0,"dy":0.0,"dz":0.0}
  单位是米。向上 dz 正，向下 dz 负；向前 dx 正，向后 dx 负；
  向左 dy 正，向右 dy 负。"一点点" 默认 0.02 米，"一些" 默认 0.05 米。

move_to_frame:
  机械臂末端移动到 TF 目标附近。
  字段: {"skill":"move_to_frame","target":"target_var","offset_x":0.0,"offset_y":0.0,"offset_z":0.08,"keep_current_orientation":true}
  到目标上方 offset_z 默认 0.08；靠近目标 offset_z 默认 0.05。

move_joints:
  控制关节。
  字段: {"skill":"move_joints","joint_names":["joint6"],"values":[-45],"degrees":true,"relative":true}
  第六关节写 joint6。顺时针通常用正角度，逆时针通常用负角度。

wait:
  等待。
  字段: {"skill":"wait","seconds":2.0}

navigate_to_pose:
  底盘导航到地图坐标。
  字段: {"skill":"navigate_to_pose","x":1.0,"y":0.0,"yaw_deg":45.0,"frame_id":"map"}
  用户没说朝向时可以省略 yaw_deg。

navigate_named:
  底盘导航到配置文件里的命名点。
  字段: {"skill":"navigate_named","target_name":"A"}
  不要自己编造命名点坐标，坐标由配置文件提供。

locate_vlm:
  根据当前相机图像寻找任意目标物体，并生成临时 TF，后续用 save_as 引用。
  字段: {"skill":"locate_vlm","query":"黄色色块","save_as":"yellow_block"}
  适用：任何可以用外观、类别、颜色、形状、位置关系描述的物体，
  例如色块、二维码块、水杯、工具、左边/右边/最近的物体等。
  query 直接填用户描述该物体的原始说法。


pick:
  夹取目标。
  字段: {"skill":"pick","target":"target_var"}
  target 可以是 locate_vlm 的 save_as，也可以是直接 TF 名。
  pick 内部会调用抓取接口完成到位、闭爪和返回，不要在 pick 前自动增加 move_to_frame。

place:
  放到机械臂默认放置点。
  字段: {"skill":"place"}
  只有用户明确说默认/固定/预设放置点时才使用。

place_to_frame:
  把当前夹持物放到某个目标 TF 上方。
  字段: {"skill":"place_to_frame","target":"target_var","offset_x":0.0,"offset_y":0.0,"offset_z":0.05}
  放到/堆到某个目标上时必须用 place_to_frame，不要用 place。

输出 JSON 格式：
{
  "intent": "short_english_intent",
  "say": "给用户看的简短中文确认",
  "steps": [
    {"skill": "..."}
  ]
}
"""


@dataclass
class StepResult:
    """单个步骤的执行结果：ok 表示成功与否，message 是描述，data 携带附加数据。"""

    ok: bool
    message: str = ''
    data: dict = field(default_factory=dict)

    @staticmethod
    def success(message='ok', **data):
        """构造一个成功结果，附加数据通过关键字参数传入。"""
        return StepResult(True, message, data)

    @staticmethod
    def failed(message='failed', **data):
        """构造一个失败结果，附加数据通过关键字参数传入。"""
        return StepResult(False, message, data)

    def to_dict(self):
        """转换为字典，便于序列化成 JSON 发布出去。"""
        return {
            'status': 'success' if self.ok else 'failed',
            'message': self.message,
            'data': self.data,
        }


class PickTaskManagerNode(Node):
    """任务管理主节点：规划并执行机械臂 / 底盘 / 视觉抓取任务。"""

    def __init__(self):
        super().__init__('pick_task_manager_node')

        # ---- 声明可配置参数（话题名） ----
        self.declare_parameter('command_topic', '/vlm_demo/user_command')
        self.declare_parameter('result_topic', '/vlm_demo/task_result')
        self.declare_parameter('chat_topic', '/vlm_demo/chat_query')

        # ---- 文本规划器（大模型）相关参数 ----
        self.declare_parameter('api_base', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('api_key_env', 'DASHSCOPE_API_KEY')
        self.declare_parameter('planner_model', 'qwen-plus-latest')
        self.declare_parameter('planner_temperature', 0.0)
        self.declare_parameter('planner_timeout_sec', 60.0)

        # ---- 视觉定位 / 几何解算服务相关参数 ----
        self.declare_parameter('ground_targets_service', '/vlm_demo/ground_targets')
        self.declare_parameter('resolve_service', '/vlm_demo/resolve_target_pose')
        self.declare_parameter('min_confidence', 0.30)
        self.declare_parameter('tf_frame_prefix', 'vlm')
        self.declare_parameter('batch_vlm_grounding', True)

        # ---- 机械臂运动 / 抓取放置服务名 ----
        self.declare_parameter('move_named_service', '/arm/move_named')
        self.declare_parameter('move_relative_service', '/arm/move_relative')
        self.declare_parameter('move_pose_service', '/arm/move_pose')
        self.declare_parameter('move_to_frame_service', '/arm/move_to_frame')
        self.declare_parameter('move_cartesian_service', '/arm/move_cartesian')
        self.declare_parameter('move_joints_service', '/arm/move_joints')
        self.declare_parameter('grab_service', '/obj_grab_service')
        self.declare_parameter('place_service', '/obj_place_service')
        self.declare_parameter('place_to_frame_service', '/obj_place_to_frame_service')

        # ---- 运动默认值与各类超时时间 ----
        self.declare_parameter('default_reference_frame', 'base_link')
        self.declare_parameter('default_velocity_scale', 0.3)
        self.declare_parameter('default_acceleration_scale', 0.3)
        self.declare_parameter('default_above_offset_z', 0.08)
        self.declare_parameter('default_stack_offset_z', 0.05)
        self.declare_parameter('service_wait_timeout_sec', 2.0)
        self.declare_parameter('service_call_timeout_sec', 120.0)
        self.declare_parameter('max_wait_sec', 60.0)

        # ---- Nav2 底盘导航相关参数 ----
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter('nav_frame_id', 'map')
        self.declare_parameter('nav_settle_delay_sec', 2.0)
        self.declare_parameter('nav_server_timeout_sec', 5.0)
        self.declare_parameter('nav_goal_timeout_sec', 180.0)
        self.declare_parameter('nav_feedback_log_interval_sec', 2.0)
        self.declare_parameter('nav_named_targets_json', '{}')

        # ---- 读取参数到成员变量 ----
        self.command_topic = self.get_parameter('command_topic').value
        self.result_topic = self.get_parameter('result_topic').value
        self.chat_topic = self.get_parameter('chat_topic').value

        self.api_base = self.get_parameter('api_base').value.rstrip('/')
        self.api_key_env = self.get_parameter('api_key_env').value
        self.planner_model = self.get_parameter('planner_model').value
        self.planner_temperature = float(self.get_parameter('planner_temperature').value)
        self.planner_timeout_sec = float(self.get_parameter('planner_timeout_sec').value)
        # 从环境变量读取 API Key，为空时警告（后续调用会失败）
        self.api_key = os.environ.get(self.api_key_env, '')
        if not self.api_key:
            self.get_logger().warn(
                f'Environment variable {self.api_key_env} is empty; planner API calls will fail.'
            )

        self.ground_targets_srv = self.get_parameter('ground_targets_service').value
        self.resolve_srv = self.get_parameter('resolve_service').value
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.tf_frame_prefix = self.get_parameter('tf_frame_prefix').value
        self.batch_vlm_grounding = bool(self.get_parameter('batch_vlm_grounding').value)

        self.default_reference_frame = self.get_parameter('default_reference_frame').value
        self.default_velocity_scale = float(self.get_parameter('default_velocity_scale').value)
        self.default_acceleration_scale = float(
            self.get_parameter('default_acceleration_scale').value)
        self.default_above_offset_z = float(self.get_parameter('default_above_offset_z').value)
        self.default_stack_offset_z = float(self.get_parameter('default_stack_offset_z').value)
        self.service_wait_timeout_sec = float(self.get_parameter('service_wait_timeout_sec').value)
        self.service_call_timeout_sec = float(self.get_parameter('service_call_timeout_sec').value)
        self.max_wait_sec = float(self.get_parameter('max_wait_sec').value)

        self.nav2_action_name = self.get_parameter('nav2_action_name').value
        self.nav_settle_delay_sec = float(self.get_parameter('nav_settle_delay_sec').value)
        self.nav_frame_id = self.get_parameter('nav_frame_id').value
        self.nav_server_timeout_sec = float(self.get_parameter('nav_server_timeout_sec').value)
        self.nav_goal_timeout_sec = float(self.get_parameter('nav_goal_timeout_sec').value)
        self.nav_feedback_log_interval_sec = float(
            self.get_parameter('nav_feedback_log_interval_sec').value)

        # 可重入回调组，允许服务/动作在多线程执行器中并发回调
        cb_group = ReentrantCallbackGroup()

        # 结果发布 / 视觉问答转发 / 用户指令订阅
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.chat_pub = self.create_publisher(String, self.chat_topic, 10)
        self.cmd_sub = self.create_subscription(
            String,
            self.command_topic,
            self.task_callback,
            10,
            callback_group=cb_group,
        )

        # 视觉定位与几何解算服务客户端
        self.ground_targets_client = self.create_client(
            GroundTargets, self.ground_targets_srv, callback_group=cb_group)
        self.resolve_client = self.create_client(
            ResolveTargetPose, self.resolve_srv, callback_group=cb_group)

        # 机械臂运动与抓取放置服务客户端
        self.move_named_client = self.create_client(
            MoveNamed, self.get_parameter('move_named_service').value, callback_group=cb_group)
        self.move_relative_client = self.create_client(
            MoveRelative, self.get_parameter('move_relative_service').value, callback_group=cb_group)
        self.move_pose_client = self.create_client(
            MovePose, self.get_parameter('move_pose_service').value, callback_group=cb_group)
        self.move_to_frame_client = self.create_client(
            MoveToFrame, self.get_parameter('move_to_frame_service').value, callback_group=cb_group)
        self.move_cartesian_client = self.create_client(
            MoveCartesian, self.get_parameter('move_cartesian_service').value, callback_group=cb_group)
        self.move_joints_client = self.create_client(
            MoveJoints, self.get_parameter('move_joints_service').value, callback_group=cb_group)
        self.grab_client = self.create_client(
            GrabObject, self.get_parameter('grab_service').value, callback_group=cb_group)
        self.place_client = self.create_client(
            PlaceObject, self.get_parameter('place_service').value, callback_group=cb_group)
        self.place_to_frame_client = self.create_client(
            PlaceToFrame, self.get_parameter('place_to_frame_service').value, callback_group=cb_group)

        # Nav2 导航动作客户端
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            self.nav2_action_name,
            callback_group=cb_group,
        )

        # 技能名 -> 执行函数 的分发表（同一函数可对应多个别名）
        self.handlers = {
            'locate_vlm': self.execute_locate_vlm,
            'resolve_vlm': self.execute_locate_vlm,
            'resolve_color': self.execute_locate_vlm,
            'move_named': self.execute_move_named,
            'move_relative': self.execute_move_relative,
            'move_pose': self.execute_move_pose,
            'move_to_frame': self.execute_move_to_frame,
            'move_cartesian': self.execute_move_cartesian,
            'move_joints': self.execute_move_joints,
            'pick': self.execute_pick,
            'grab': self.execute_pick,
            'place': self.execute_place,
            'place_to_frame': self.execute_place_to_frame,
            'place_on': self.execute_place_to_frame,
            'wait': self.execute_wait,
            'sleep': self.execute_wait,
            'delay': self.execute_wait,
            'navigate_to_pose': self.execute_navigate_to_pose,
            'nav_to_pose': self.execute_navigate_to_pose,
            'move_base_to_pose': self.execute_navigate_to_pose,
            'navigate_named': self.execute_navigate_named,
            'navigate_to_named': self.execute_navigate_named,
            'navigate_to_point': self.execute_navigate_named,
            'nav_named': self.execute_navigate_named,
        }

        self.task_queue = queue.Queue()          # 待处理任务队列
        self.task_context = {}                   # 本次任务中 save_as -> 目标信息 的上下文
        self._target_counter = 0                 # 生成唯一 TF 名的自增计数
        self._last_nav_feedback_log_time = 0.0   # 上次打印导航反馈的时间（用于限频）

        # 后台工作线程：从队列取指令并串行执行，避免阻塞回调
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'Pick task manager ready. topic={self.command_topic}, planner={self.planner_model}')

    # ------------------------------------------------------------------
    # Task input, planning, and worker loop
    # ------------------------------------------------------------------
    def task_callback(self, msg):
        """指令话题回调：区分 /chat 视觉问答与 /task 任务，任务入队交给工作线程。"""
        command = msg.data.strip()
        if not command:
            self.publish_text_result('空指令，已忽略。')
            return

        lower_command = command.lower()
        # /chat 前缀：转发到视觉问答话题，不走任务规划
        if lower_command.startswith('/chat'):
            chat_text = command[len('/chat'):].strip()
            if not chat_text:
                self.publish_text_result('请输入 /chat 后面的对话内容。')
                return

            self.chat_pub.publish(String(data=chat_text))
            self.publish_text_result('已转到视觉问答。')
            return

        # /task 前缀：去掉前缀后作为任务指令
        if lower_command.startswith('/task'):
            command = command[len('/task'):].strip()
            if not command:
                self.publish_text_result('空任务指令，已忽略。')
                return

        self.task_queue.put(command)
        self.publish_text_result('收到指令，开始规划任务。')

    def worker_loop(self):
        """后台循环：不断取出指令 -> 规划 -> 执行 -> 发布结果，异常统一捕获上报。"""
        while rclpy.ok():
            try:
                command = self.task_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                plan = self.plan_command(command)
                self.publish_text_result(plan.get('say', '规划完成，开始执行。'))
                result = self.execute_task(plan)

                payload = result.to_dict()
                payload['intent'] = plan.get('intent', '')
                if result.ok:
                    self.publish_text_result(f"任务完成: {result.message}")
                else:
                    self.publish_text_result(f"任务失败: {result.message}")
                self.publish_json_result(payload)
            except Exception:
                detail = traceback.format_exc()
                self.get_logger().error('Task crashed:\n' + detail)
                self.publish_json_result({
                    'status': 'failed',
                    'message': 'task exception',
                    'traceback': detail,
                })

    def plan_command(self, command: str):
        """把用户指令转成规范化任务计划：直接是 JSON 则解析，否则调用大模型规划。"""
        text = command.strip()
        lower_text = text.lower()
        # 去掉可能的 /pub、/task 前缀
        for prefix in ('/pub', '/task'):
            if lower_text.startswith(prefix):
                text = text[len(prefix):].strip()
                lower_text = text.lower()
                break

        # 用户直接给了 JSON，跳过大模型
        if text.startswith('{') or text.startswith('['):
            return self.normalize_plan(json.loads(text))

        # 否则调用规划大模型，从其输出中提取 JSON 并规范化
        raw = self.call_planner_model(text)
        plan = self.extract_json_object(raw)
        return self.normalize_plan(plan, source_text=text)

    def call_planner_model(self, command: str) -> str:
        """调用大模型，返回其文本输出（期望是 JSON 字符串）。"""
        url = f'{self.api_base}/chat/completions'
        prompt = self.build_task_prompt()
        payload = {
            'model': self.planner_model,
            'messages': [
                {'role': 'system', 'content': prompt},
                {'role': 'user', 'content': command},
            ],
            'temperature': self.planner_temperature,
            'stream': False,
        }
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Content-Type': 'application/json',
        }

        response = requests.post(
            url,
            json=payload,
            headers=headers,
            timeout=self.planner_timeout_sec,
        )
        if response.status_code < 200 or response.status_code >= 300:
            raise RuntimeError(f'planner API HTTP {response.status_code}: {response.text}')

        data = response.json()
        return data['choices'][0]['message']['content'].strip()

    def build_task_prompt(self) -> str:
        """拼接系统提示词，并把当前已配置的底盘命名点追加进去供模型参考。"""
        targets = self.load_nav_named_targets()
        names = ', '.join(sorted(targets.keys())) if targets else '无'
        return TASK_PROMPT + f'\n当前已配置底盘命名点: {names}\n'

    @staticmethod
    def extract_json_object(text: str):
        """从模型输出文本中提取第一个 JSON 对象，兼容 ```json 代码块包裹。"""
        text = text.strip()
        if text.startswith('```'):
            text = text.replace('```json', '').replace('```', '').strip()

        start = text.find('{')
        if start < 0:
            raise ValueError('planner output does not contain a JSON object')

        # 从第一个 '{' 开始增量解析，忽略后面多余内容
        decoder = json.JSONDecoder()
        obj, _ = decoder.raw_decode(text[start:])
        return obj

    def normalize_plan(self, plan, source_text=''):
        """校验并补全任务计划：确保存在非空 steps 列表、每步都有 skill，并清理冗余步骤。"""
        if isinstance(plan, list):
            plan = {'intent': 'robot_task', 'steps': plan}

        if not isinstance(plan, dict):
            raise ValueError('task plan must be a JSON object')

        if 'steps' not in plan or not isinstance(plan['steps'], list):
            raise ValueError('task plan must contain steps list')

        if not plan['steps']:
            raise ValueError('steps must not be empty')

        for index, step in enumerate(plan['steps'], start=1):
            if not isinstance(step, dict):
                raise ValueError(f'step {index} must be an object')
            if not step.get('skill'):
                raise ValueError(f'step {index} missing skill')

        plan.setdefault('intent', 'robot_task')
        plan.setdefault('say', '收到，开始执行任务。')
        plan['steps'] = self.remove_redundant_pre_pick_moves(plan['steps'], source_text)
        return plan

    def remove_redundant_pre_pick_moves(self, steps, source_text=''):
        """删除 pick 前多余的 move_to_frame（除非用户明确要求先移动到目标上方/附近）。"""
        # 用户明确要求过渡移动时，保留原步骤不做删除
        if self.explicit_pre_pick_move_requested(source_text):
            return steps

        cleaned = []
        for step in steps:
            skill = self.canonical_skill_name(step)
            if skill in ('pick', 'grab') and cleaned:
                previous = cleaned[-1]
                if self.is_redundant_move_to_frame_before_pick(previous, step):
                    removed = cleaned.pop()
                    self.get_logger().info(
                        'Removed redundant pre-pick move_to_frame: '
                        f'{self.raw_step_target(removed)} -> {self.raw_step_target(step)}')
            cleaned.append(step)

        return cleaned

    @staticmethod
    def explicit_pre_pick_move_requested(source_text):
        """判断用户原文是否在抓取词之前明确要求了"移动到上方/附近"等过渡动作。"""
        # 无原文（如直接给 JSON）时保守处理，视为需要保留
        if not source_text:
            return True

        lower_text = source_text.lower()
        pick_words = ('夹取', '抓取', '拿起', '取起', '夹', '抓', '拿', '取', 'pick', 'grab')
        pick_positions = [
            lower_text.find(word)
            for word in pick_words
            if lower_text.find(word) >= 0
        ]
        if not pick_positions:
            return False

        before_pick = lower_text[:min(pick_positions)]
        explicit_words = (
            '上方', '上面', '上边', '上空', '附近', '靠近', '接近',
            '移动到', '运动到', '移到', '到达', '停到', '停在', '悬停',
            'above', 'approach', 'move to',
        )
        return any(word in before_pick for word in explicit_words)

    @staticmethod
    def canonical_skill_name(step):
        """取出步骤的 skill 名并统一为小写、去空格，便于比较。"""
        return str(step.get('skill', '')).strip().lower()

    def is_redundant_move_to_frame_before_pick(self, move_step, pick_step):
        """判断某个 move_to_frame 是否是 pick 前的冗余步骤（同一目标、无横向偏移、仅竖直上方）。"""
        if self.canonical_skill_name(move_step) != 'move_to_frame':
            return False

        move_target = self.raw_step_target(move_step)
        pick_target = self.raw_step_target(pick_step)
        if not move_target or not pick_target or move_target != pick_target:
            return False

        try:
            offset_x = float(move_step.get('offset_x', move_step.get('x_offset', 0.0)))
            offset_y = float(move_step.get('offset_y', move_step.get('y_offset', 0.0)))
            offset_z = float(
                move_step.get('offset_z', move_step.get('z_offset', self.default_above_offset_z)))
        except (TypeError, ValueError):
            return False

        return abs(offset_x) < 1e-9 and abs(offset_y) < 1e-9 and offset_z >= 0.0

    @staticmethod
    def raw_step_target(step):
        """按优先级从步骤中取出目标标识字符串（obj_link/target_frame/frame_id/frame/target）。"""
        target_value = (
            step.get('obj_link')
            or step.get('target_frame')
            or step.get('frame_id')
            or step.get('frame')
            or step.get('target')
        )
        return str(target_value).strip() if target_value is not None else ''

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------
    def execute_task(self, plan):
        """按顺序执行计划中的每个步骤，任一步失败即中止并返回失败结果。"""
        self.task_context = {}      # 重置本次任务上下文
        self._target_counter = 0
        step_results = []

        steps = plan.get('steps', [])
        for index, step in enumerate(steps, start=1):
            skill = str(step.get('skill', 'unknown'))
            self.get_logger().info(f'Executing step {index}: {skill} {step}')

            # 遇到视觉定位步骤时，先尝试批量预取本段内的所有视觉目标
            if self.is_vlm_locate_step(step):
                prefetch_result = self.prefetch_vlm_targets(steps, index - 1)
                if prefetch_result is not None and not prefetch_result.ok:
                    result = prefetch_result
                else:
                    result = self.execute_step(step)
            else:
                result = self.execute_step(step)

            step_results.append({
                'index': index,
                'skill': skill,
                **result.to_dict(),
            }) 
             # 视觉定位成功后该帧刚被 geometry_resolver 发出,
            # 紧接着的 move_to_frame 会按(略早的)时刻查 TF 容易踩单点外推失败,
            # 这里停一小段等下一拍 TF 就绪,纯静态目标不受影响。
            if result.ok and self.is_vlm_locate_step(step):
                time.sleep(0.15)
            if not result.ok:
                return StepResult.failed(
                    f'step {index} failed: {result.message}',
                    failed_step=index,
                    failed_skill=skill,
                    step_results=step_results,
                )

        return StepResult.success(
            'task completed',
            step_count=len(plan.get('steps', [])),
            step_results=step_results,
        )

    def execute_step(self, step):
        """根据 skill 从分发表找到对应处理函数并执行，统一捕获异常与校验返回类型。"""
        if not isinstance(step, dict):
            return StepResult.failed('step must be an object')

        skill = str(step.get('skill', ''))
        handler = self.handlers.get(skill)
        if handler is None:
            return StepResult.failed(f'unknown skill: {skill}')

        try:
            result = handler(step)
        except Exception:
            return StepResult.failed(
                'exception while executing skill',
                skill=skill,
                traceback=traceback.format_exc(),
            )

        if isinstance(result, StepResult):
            return result
        return StepResult.failed(f'invalid handler result type: {type(result).__name__}')

    def is_vlm_locate_step(self, step):
        """判断步骤是否为视觉定位类技能（locate_vlm / resolve_vlm / resolve_color）。"""
        if not isinstance(step, dict):
            return False
        return str(step.get('skill', '')) in (
            'locate_vlm',
            'resolve_vlm',
            'resolve_color',
        )

    @staticmethod
    def is_navigation_step(step):
        """判断步骤是否为底盘导航类技能（用于批量预取时作为分段边界）。"""
        if not isinstance(step, dict):
            return False
        return str(step.get('skill', '')) in (
            'navigate_to_pose',
            'nav_to_pose',
            'move_base_to_pose',
            'navigate_named',
            'navigate_to_named',
            'navigate_to_point',
            'nav_named',
        )

    def prefetch_vlm_targets(self, steps, start_index):
        """一次视觉调用批量定位当前段内的多个目标，减少重复拍照/推理开销。

        从 start_index 起收集连续的视觉定位步骤（遇到导航步骤则分段截断），
        通过一次 GroundTargets 服务调用得到所有像素坐标，再逐个解算成 TF 存入上下文。
        返回 None 表示无需批量（关闭开关或目标数 <=1），交由单步逻辑处理。
        """
        if not self.batch_vlm_grounding:
            return None

        # 收集本段内所有待定位、且尚未缓存的目标
        entries = []
        for index in range(start_index, len(steps)):
            step = steps[index]
            # 导航会改变机器人位置，之后的视觉目标不能与当前段合并
            if index > start_index and self.is_navigation_step(step):
                break

            if not self.is_vlm_locate_step(step):
                continue

            save_as = str(step.get('save_as', step.get('name', 'target')))
            if save_as in self.task_context:
                continue

            entries.append({
                'index': index,
                'step': step,
                'query': self.vlm_query_from_step(step),
                'save_as': save_as,
            })

        # 只有 0 或 1 个目标时没有批量必要
        if len(entries) <= 1:
            return None

        if not self.ground_targets_client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
            return StepResult.failed(
                f'visual grounding service unavailable: {self.ground_targets_srv}')

        # 一次性把所有查询发给视觉定位服务
        req = GroundTargets.Request()
        req.queries = [entry['query'] for entry in entries]
        self.get_logger().info(
            f'Batch locating {len(entries)} visual targets: {req.queries}')

        res = self.ground_targets_client.call(req)
        if not res.success:
            return StepResult.failed(f'batch visual grounding failed: {res.message}')

        # 逐个校验置信度/歧义，并把像素坐标解算成 TF 目标
        for i, entry in enumerate(entries):
            if i >= len(res.found):
                return StepResult.failed('batch visual grounding response size mismatch')

            query = entry['query']
            save_as = entry['save_as']
            step = entry['step']

            if res.ambiguous[i]:
                question = res.questions[i] or '目标有歧义，请描述得更具体。'
                return StepResult.failed(f'vision target ambiguous for {query}: {question}')
            if not res.found[i]:
                return StepResult.failed(f'vision target not found: {query}')
            if float(res.confidences[i]) < self.min_confidence:
                return StepResult.failed(
                    f'vision confidence too low for {query}: '
                    f'{float(res.confidences[i]):.2f} < {self.min_confidence:.2f}')

            frame_hint = str(step.get('frame_id_hint', '')).strip()
            if not frame_hint:
                frame_hint = self.next_vlm_frame_name(save_as)

            resolve_result = self.resolve_grounded_pixel(
                query=query,
                save_as=save_as,
                target_label=res.target_labels[i] or query,
                u=int(res.grasp_u[i]),
                v=int(res.grasp_v[i]),
                confidence=float(res.confidences[i]),
                frame_hint=frame_hint,
            )
            if not resolve_result.ok:
                return resolve_result

        return StepResult.success(
            f'batch located {len(entries)} visual targets',
            targets=[entry['save_as'] for entry in entries],
        )

    def execute_locate_vlm(self, step):
        """单个视觉定位技能：调用视觉服务找到目标像素，再解算成 TF 存入上下文。"""
        query = self.vlm_query_from_step(step)
        save_as = str(step.get('save_as', step.get('name', 'target')))
        # 若批量预取已定位过该目标，直接命中缓存
        if save_as in self.task_context and self.task_context[save_as].get('kind') == 'vlm':
            frame = self.task_context[save_as].get('frame', '')
            return StepResult.success(
                f'located {query} from batch cache: {frame}',
                save_as=save_as,
                frame=frame,
            )

        frame_hint = str(step.get('frame_id_hint', '')).strip()
        if not frame_hint:
            frame_hint = self.next_vlm_frame_name(save_as)

        if not self.ground_targets_client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
            return StepResult.failed(
                f'visual grounding service unavailable: {self.ground_targets_srv}')

        ground_req = GroundTargets.Request()
        ground_req.queries = [query]
        ground_res = self.ground_targets_client.call(ground_req)

        if not ground_res.success:
            return StepResult.failed(f'vision grounding failed: {ground_res.message}')
        if not ground_res.found:
            return StepResult.failed('vision grounding returned empty result')
        if ground_res.ambiguous[0]:
            question = ground_res.questions[0] or '目标有歧义，请描述得更具体。'
            return StepResult.failed(f'vision target ambiguous: {question}')
        if not ground_res.found[0]:
            return StepResult.failed(f'vision target not found: {query}')
        if float(ground_res.confidences[0]) < self.min_confidence:
            return StepResult.failed(
                f'vision confidence too low: '
                f'{float(ground_res.confidences[0]):.2f} < {self.min_confidence:.2f}')

        return self.resolve_grounded_pixel(
            query=query,
            save_as=save_as,
            target_label=ground_res.target_labels[0] or query,
            u=ground_res.grasp_u[0],
            v=ground_res.grasp_v[0],
            confidence=float(ground_res.confidences[0]),
            frame_hint=frame_hint,
        )

    def resolve_grounded_pixel(self, query, save_as, target_label, u, v, confidence, frame_hint):
        """把视觉给出的像素坐标 (u,v) 交给几何解算服务，得到 3D 位姿并发布为 TF，存入上下文。"""
        if not self.resolve_client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
            return StepResult.failed(f'geometry resolver service unavailable: {self.resolve_srv}')

        resolve_req = ResolveTargetPose.Request()
        resolve_req.u = int(u)
        resolve_req.v = int(v)
        resolve_req.target_label = str(target_label or query)
        resolve_req.frame_id_hint = str(frame_hint)
        resolve_res = self.resolve_client.call(resolve_req)

        if not resolve_res.success:
            return StepResult.failed(f'geometry resolve failed for {query}: {resolve_res.message}')

        self.task_context[save_as] = {
            'kind': 'vlm',
            'frame': resolve_res.target_frame,
            'query': query,
            'label': target_label,
            'confidence': float(confidence),
        }

        return StepResult.success(
            f'located {query}: {resolve_res.target_frame}',
            save_as=save_as,
            frame=resolve_res.target_frame,
            confidence=float(confidence),
            depth_m=float(resolve_res.depth_m),
        )


    def execute_move_named(self, step):
        """机械臂移动到 SRDF 中预设的命名位姿（如 look、home）。"""
        target_name = step.get('target_name', step.get('name', ''))
        if not target_name:
            return StepResult.failed('move_named missing target_name')

        req = MoveNamed.Request()
        req.target_name = str(target_name)
        req.velocity_scale = self.velocity(step)
        req.acceleration_scale = self.acceleration(step)

        return self.call_service(self.move_named_client, req, '/arm/move_named')

    def execute_move_relative(self, step):
        """机械臂末端在指定参考系下做相对平移（dx/dy/dz，单位米）。"""
        req = MoveRelative.Request()
        req.reference_frame = str(
            step.get('reference_frame', step.get('frame', self.default_reference_frame)))
        req.dx = float(step.get('dx', 0.0))
        req.dy = float(step.get('dy', 0.0))
        req.dz = float(step.get('dz', 0.0))
        req.velocity_scale = self.velocity(step)
        req.acceleration_scale = self.acceleration(step)

        return self.call_service(self.move_relative_client, req, '/arm/move_relative')

    def execute_move_to_frame(self, step):
        """机械臂末端移动到某个 TF 目标附近（可带 xyz 偏移，默认停在目标上方）。"""
        target_frame = self.frame_from_step_target(step)
        if not target_frame:
            return StepResult.failed('move_to_frame missing target / target_frame')

        req = MoveToFrame.Request()
        req.target_frame = target_frame
        req.reference_frame = str(step.get('reference_frame', self.default_reference_frame))
        req.offset_x = float(step.get('offset_x', step.get('x_offset', 0.0)))
        req.offset_y = float(step.get('offset_y', step.get('y_offset', 0.0)))
        req.offset_z = float(
            step.get('offset_z', step.get('z_offset', self.default_above_offset_z)))
        req.keep_current_orientation = bool(step.get('keep_current_orientation', True))
        req.velocity_scale = self.velocity(step)
        req.acceleration_scale = self.acceleration(step)

        return self.call_service(self.move_to_frame_client, req, '/arm/move_to_frame')

    def execute_move_pose(self, step):
        """机械臂末端移动到一个绝对目标位姿（position + orientation）。"""
        pose_data = step.get('target_pose', step.get('pose', None))
        if not isinstance(pose_data, dict):
            return StepResult.failed('move_pose missing target_pose object')

        req = MovePose.Request()
        req.target_pose = self.parse_pose_stamped(pose_data)
        req.velocity_scale = self.velocity(step)
        req.acceleration_scale = self.acceleration(step)

        return self.call_service(self.move_pose_client, req, '/arm/move_pose')

    def execute_move_cartesian(self, step):
        """机械臂沿一组路径点做笛卡尔直线插补运动。"""
        waypoints_data = step.get('waypoints', [])
        if not isinstance(waypoints_data, list) or not waypoints_data:
            return StepResult.failed('move_cartesian missing waypoints')

        req = MoveCartesian.Request()
        req.waypoints = [self.parse_pose_stamped(x) for x in waypoints_data]
        req.keep_current_orientation = bool(step.get('keep_current_orientation', True))
        req.eef_step = float(step.get('eef_step', 0.01))
        req.jump_threshold = float(step.get('jump_threshold', 0.0))
        req.min_fraction = float(step.get('min_fraction', 0.8))

        return self.call_service(self.move_cartesian_client, req, '/arm/move_cartesian')

    def execute_move_joints(self, step):
        """控制指定关节转动到目标角度（支持角度/弧度、绝对/相对）。"""
        req = MoveJoints.Request()
        req.joint_names = [str(x) for x in step.get('joint_names', [])]
        req.values = [float(x) for x in step.get('values', [])]
        req.degrees = bool(step.get('degrees', True))
        req.relative = bool(step.get('relative', True))
        req.velocity_scale = self.velocity(step)
        req.acceleration_scale = self.acceleration(step)

        if not req.values:
            return StepResult.failed('move_joints missing values')

        if req.joint_names and len(req.joint_names) != len(req.values):
            return StepResult.failed('joint_names size must match values size')

        return self.call_service(self.move_joints_client, req, '/arm/move_joints')

    def execute_pick(self, step):
        """夹取目标：调用抓取服务，内部完成靠近、闭爪、抬起等动作。"""
        frame = self.frame_from_step_target(step)
        if not frame:
            return StepResult.failed('pick missing target / obj_link')

        req = GrabObject.Request()
        req.obj_link = frame

        self.get_logger().info(f'Pick object link: {frame}')
        return self.call_service(self.grab_client, req, '/obj_grab_service')

    def execute_place(self, step):
        """把当前夹持物放到机械臂预设的默认放置点。"""
        req = PlaceObject.Request()
        return self.call_service(self.place_client, req, '/obj_place_service')

    def execute_place_to_frame(self, step):
        """把当前夹持物放到指定 TF 目标上方（用于堆叠/放到某物体上）。"""
        target_frame = self.frame_from_step_target(step)
        if not target_frame:
            return StepResult.failed('place_to_frame missing target / target_frame')

        req = PlaceToFrame.Request()
        req.target_frame = target_frame
        req.offset_x = float(step.get('offset_x', 0.0))
        req.offset_y = float(step.get('offset_y', 0.0))
        req.offset_z = float(step.get('offset_z', self.default_stack_offset_z))

        self.get_logger().info(
            f'Place to frame: {req.target_frame}, '
            f'offset=({req.offset_x}, {req.offset_y}, {req.offset_z})'
        )

        return self.call_service(self.place_to_frame_client, req, '/obj_place_to_frame_service')

    def execute_wait(self, step):
        """等待指定秒数（超过 max_wait_sec 会被截断，防止卡死）。"""
        try:
            seconds = float(step.get('seconds', step.get('duration', 0.0)))
        except (TypeError, ValueError):
            return StepResult.failed('wait seconds must be a number')

        if seconds < 0.0:
            return StepResult.failed('wait seconds must be >= 0')

        if seconds > self.max_wait_sec:
            self.get_logger().warn(
                f'wait seconds {seconds:.2f} is too large, clamp to {self.max_wait_sec:.2f}')
            seconds = self.max_wait_sec

        self.get_logger().info(f'Waiting {seconds:.2f} seconds before next step...')
        time.sleep(seconds)
        return StepResult.success(f'waited {seconds:.2f} seconds')

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def execute_navigate_named(self, step):
        """导航到配置文件里的命名点：先查坐标，再复用 execute_navigate_to_pose 执行。"""
        target_name = (
            step.get('target_name')
            or step.get('name')
            or step.get('target')
            or step.get('point')
        )

        if not target_name:
            return StepResult.failed('navigate_named missing target_name')

        try:
            pose = self.resolve_nav_target_name(str(target_name))
        except Exception as exc:
            return StepResult.failed(str(exc))

        nav_step = {
            'skill': 'navigate_to_pose',
            'x': pose['x'],
            'y': pose['y'],
            'yaw': pose['yaw'],
            'frame_id': pose['frame_id'],
            'target_name': str(target_name),
        }

        self.get_logger().info(
            f"Navigate named target {str(target_name)} -> "
            f"x={pose['x']:.3f} y={pose['y']:.3f} yaw={pose['yaw']:.3f} "
            f"frame={pose['frame_id']}"
        )

        return self.execute_navigate_to_pose(nav_step)

    def execute_navigate_to_pose(self, step):
        try:
            x = float(step['x'])
            y = float(step['y'])
            yaw = self.get_yaw_from_step(step, 0.0)
        except KeyError as exc:
            return StepResult.failed(f'navigate_to_pose missing field: {exc}')
        except Exception as exc:
            return StepResult.failed(f'navigate_to_pose invalid pose: {exc}')

        frame_id = str(step.get('frame_id', self.nav_frame_id))
        target_name = step.get('target_name', '')

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        behavior_tree = step.get('behavior_tree', '')
        if behavior_tree:
            goal_msg.behavior_tree = str(behavior_tree)

        if target_name:
            self.get_logger().info(
                f'Navigate to {str(target_name)}: '
                f'frame={frame_id} x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
        else:
            self.get_logger().info(
                f'Navigate to pose: frame={frame_id} x={x:.3f} y={y:.3f} yaw={yaw:.3f}')

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=self.nav_server_timeout_sec):
            return StepResult.failed(f'Nav2 action server not available: {self.nav2_action_name}')

        def feedback_callback(feedback_msg):
            now = time.monotonic()
            if now - self._last_nav_feedback_log_time < self.nav_feedback_log_interval_sec:
                return

            self._last_nav_feedback_log_time = now
            distance = getattr(feedback_msg.feedback, 'distance_remaining', None)
            if distance is not None:
                self.get_logger().info(f'Navigation distance remaining: {distance:.3f}')

        send_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback,
        )

        if not self.wait_future_done(
            send_future,
            self.nav_server_timeout_sec,
            'send NavigateToPose goal',
        ):
            return StepResult.failed('send NavigateToPose goal timeout')

        goal_handle = send_future.result()
        if goal_handle is None:
            return StepResult.failed('NavigateToPose goal handle is None')
        if not goal_handle.accepted:
            return StepResult.failed('NavigateToPose goal rejected')

        result_future = goal_handle.get_result_async()

        if not self.wait_future_done(
            result_future,
            self.nav_goal_timeout_sec,
            'NavigateToPose result',
        ):
            try:
                cancel_future = goal_handle.cancel_goal_async()
                self.wait_future_done(cancel_future, 3.0, 'cancel NavigateToPose goal')
            except Exception as exc:
                self.get_logger().warn(f'Cancel navigation goal failed: {exc}')

            return StepResult.failed('NavigateToPose timeout and goal canceled')

        result = result_future.result()
        if result is None:
            return StepResult.failed('NavigateToPose result is None')

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self.nav_settle_delay_sec > 0.0:
                self.get_logger().info(
                    f'Navigation succeeded; waiting {self.nav_settle_delay_sec:.1f}s '
                    'for base to settle before next step')
                time.sleep(self.nav_settle_delay_sec)

            return StepResult.success(
                f'navigation succeeded: frame={frame_id}, x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')

        return StepResult.failed(f'NavigateToPose failed, status={result.status}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def call_service(self, client, request, service_label):
        if not client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
            return StepResult.failed(f'service not available: {service_label}')

        future = client.call_async(request)
        deadline = time.monotonic() + self.service_call_timeout_sec

        while rclpy.ok() and not future.done():
            if self.service_call_timeout_sec > 0.0 and time.monotonic() > deadline:
                return StepResult.failed(f'service call timeout: {service_label}')
            time.sleep(0.02)

        if not future.done():
            return StepResult.failed(f'service call interrupted: {service_label}')

        try:
            response = future.result()
        except Exception as exc:
            return StepResult.failed(f'service call exception: {service_label}: {exc}')

        success = bool(getattr(response, 'success', True))
        message = str(getattr(response, 'message', ''))
        data = {}
        if hasattr(response, 'fraction'):
            data['fraction'] = float(response.fraction)

        if success:
            return StepResult.success(message or f'{service_label} success', **data)

        return StepResult.failed(message or f'{service_label} failed', **data)

    def frame_from_step_target(self, step):
        target_value = (
            step.get('obj_link')
            or step.get('target_frame')
            or step.get('frame_id')
            or step.get('frame')
            or step.get('target')
        )

        target = self.resolve_target(target_value)
        return str(target.get('frame', ''))

    def resolve_target(self, target_value):
        if target_value is None:
            return {'kind': 'empty', 'frame': ''}

        if isinstance(target_value, dict):
            return target_value

        key = str(target_value)
        if key in self.task_context:
            return self.task_context[key]

        return {'kind': 'frame', 'frame': key, 'name': key}

    def vlm_query_from_step(self, step):
        if str(step.get('skill', '')) == 'resolve_color':
            color = step.get('color', step.get('target', ''))
            return f'{color}色块' if color else '色块'

        for key in ('query', 'command', 'target_desc', 'description', 'target_label', 'target'):
            value = step.get(key)
            if isinstance(value, str) and value.strip():
                return value.strip()
        return '目标物体'

    def next_vlm_frame_name(self, save_as):
        self._target_counter += 1
        safe = re.sub(r'[^0-9A-Za-z_]+', '_', str(save_as)).strip('_').lower()
        if not safe:
            safe = 'target'
        return f'{self.tf_frame_prefix}_{safe}_{self._target_counter}_frame'

    def velocity(self, step):
        return float(step.get('velocity_scale', self.default_velocity_scale))

    def acceleration(self, step):
        return float(step.get('acceleration_scale', self.default_acceleration_scale))

    def parse_pose_stamped(self, data):
        msg = PoseStamped()
        msg.header.frame_id = str(
            data.get('frame_id', data.get('reference_frame', self.default_reference_frame)))

        pos = data.get('position', data)
        ori = data.get('orientation', data)

        msg.pose.position.x = float(pos.get('x', 0.0))
        msg.pose.position.y = float(pos.get('y', 0.0))
        msg.pose.position.z = float(pos.get('z', 0.0))

        msg.pose.orientation.x = float(ori.get('qx', ori.get('xq', ori.get('orientation_x', 0.0))))
        msg.pose.orientation.y = float(ori.get('qy', ori.get('yq', ori.get('orientation_y', 0.0))))
        msg.pose.orientation.z = float(ori.get('qz', ori.get('zq', ori.get('orientation_z', 0.0))))
        msg.pose.orientation.w = float(ori.get('qw', ori.get('wq', ori.get('orientation_w', 1.0))))

        return msg

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    @staticmethod
    def get_yaw_from_step(step, default_value=0.0):
        if 'yaw_deg' in step:
            return math.radians(float(step['yaw_deg']))
        if 'yaw_degrees' in step:
            return math.radians(float(step['yaw_degrees']))
        if 'theta_deg' in step:
            return math.radians(float(step['theta_deg']))
        if 'theta' in step:
            return float(step['theta'])
        if 'yaw' in step:
            return float(step['yaw'])
        return float(default_value)

    def wait_future_done(self, future, timeout_sec, label):
        start_time = time.monotonic()
        while rclpy.ok() and not future.done():
            if timeout_sec is not None and time.monotonic() - start_time > timeout_sec:
                self.get_logger().error(f'{label} timeout after {timeout_sec:.1f} sec')
                return False
            time.sleep(0.05)
        return future.done()

    def load_nav_named_targets(self):
        raw = self.get_parameter('nav_named_targets_json').value
        if not raw:
            return {}

        try:
            data = json.loads(raw)
        except Exception as exc:
            self.get_logger().error(f'nav_named_targets_json parse failed: {exc}')
            return {}

        targets = {}
        for name, pose in data.items():
            try:
                x = float(pose['x'])
                y = float(pose['y'])
                if 'yaw_deg' in pose:
                    yaw = math.radians(float(pose['yaw_deg']))
                elif 'yaw_degrees' in pose:
                    yaw = math.radians(float(pose['yaw_degrees']))
                elif 'theta' in pose:
                    yaw = float(pose['theta'])
                else:
                    yaw = float(pose.get('yaw', 0.0))

                frame_id = str(pose.get('frame_id', self.nav_frame_id))
                targets[str(name)] = {
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'frame_id': frame_id,
                }
            except Exception as exc:
                self.get_logger().error(f'Invalid nav target {name}: {exc}')

        return targets

    def resolve_nav_target_name(self, target_name):
        targets = self.load_nav_named_targets()
        if target_name in targets:
            return targets[target_name]

        lower_name = target_name.lower()
        for name, pose in targets.items():
            if name.lower() == lower_name:
                return pose

        known = ', '.join(sorted(targets.keys()))
        raise ValueError(f'unknown nav target "{target_name}", known targets: {known}')

    def publish_text_result(self, text):
        self.result_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_json_result(self, payload):
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = PickTaskManagerNode()
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
