#!/usr/bin/env python3
"""Run on the robot. Reads JSON commands on stdin, writes JSON lines on stdout.

Topics follow algorithm_1028:
  /odom                         nav_msgs/Odometry        linear speed
  /PowerVoltage                 std_msgs/Float32         battery voltage
  /camera/color/image_raw       sensor_msgs/Image        RGB, JPEG if OpenCV exists
  /camera/color/image_raw/compressed                     preferred camera path
  /map                          nav_msgs/OccupancyGrid   SLAM map
  /ultrasonic_data_A            sensor_msgs/Range        chassis ultrasonic
  /voice_words                  std_msgs/String          recognized speech
  /feedback_words               std_msgs/String          text for TTS
"""

import base64
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

os.environ.setdefault("RCUTILS_LOGGING_SEVERITY", "WARN")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CompressedImage, Image, Range
from std_msgs.msg import Float32, String

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:
    Buffer = None
    TransformException = Exception
    TransformListener = None

try:
    import cv2
    from cv_bridge import CvBridge
except Exception:
    cv2 = None
    CvBridge = None

MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
CAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
CAMERA_INTERVAL = 1.0
MAP_INTERVAL = 3.0


def emit(payload):
    sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    sys.stdout.flush()


ULTRA_FRAMES = [f"ultrasonic_data_{name}" for name in "ABCDEF"]
PROC_LABELS = (
    ("建图", "rtabmap_slam/rtabmap"),
    ("图像同步", "rgbd_sync"),
    ("相机", "astra_camera"),
    ("界面", "baic-hmi/bridge.py"),
    ("底盘", "wheeltec_robot"),
    ("语音", "robot-voice-visual"),
    ("速度仲裁", "robot-voice-uwb.py"),
    ("旧仲裁", "legacy-voice-arbiter"),
    ("跟随规划", "robot-uwb-follow.py"),
    ("跟随激光", "robot-follow-scan.py"),
    ("目标跟踪", "robot-bag-tracker"),
    ("激光雷达", "lslidar_driver"),
    ("平面激光", "pointcloud_to_laserscan"),
    ("里程计", "ekf_node"),
)


def proc_label(command, comm):
    for name, needle in PROC_LABELS:
        if needle in command:
            return name
    parts = command.split()
    script = next((os.path.basename(part) for part in parts[1:] if part.endswith(".py")), "")
    if script:
        return script
    base = os.path.basename(parts[0]) if parts else comm
    return base or comm or "进程"


def yaw_of(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def pack_grid(width, height, data, resolution, origin_x, origin_y):
    step = 1
    while width // step > 1500 or height // step > 1500:
        step += 1
    packed_w = max(1, width // step)
    packed_h = max(1, height // step)
    raw = bytearray(packed_w * packed_h)
    total = len(data)
    for row in range(packed_h):
        src_row = row * step
        for col in range(packed_w):
            index = src_row * width + col * step
            value = data[index] if index < total else -1
            if value < 0:
                raw[row * packed_w + col] = 255
            else:
                raw[row * packed_w + col] = min(100, int(value))
    shift = 0.5 * resolution * (1 - step)
    return packed_w, packed_h, resolution * step, origin_x + shift, origin_y + shift, raw


class Bridge(Node):
    def __init__(self):
        super().__init__(f"baic_hmi_{os.getpid()}")
        self.speed = None
        self.voltage = None
        self.seen = {}
        self.have_compressed = False
        self.last_cam = 0.0
        self.last_map = 0.0
        self.cam_sub = None
        self.raw_sub = None
        self.map_sub = None
        self.camera_armed_at = 0.0
        self.map_stamp = None
        self.ranges = {}
        self.map_wh = None
        self.pose_state = None
        self.host_cpu = None
        self.host_proc = {}
        self.host_proc_at = None
        self.pending = []
        self.lock = threading.Lock()
        self.cv_bridge = CvBridge() if CvBridge else None
        self.tf_buffer = Buffer() if Buffer else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if TransformListener else None

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Float32, "/PowerVoltage", self.on_voltage, 10)
        self.create_subscription(String, "/voice_words", self.on_voice, 10)
        self.create_subscription(String, "/feedback_words", self.on_feedback, 10)
        for frame in ULTRA_FRAMES:
            self.create_subscription(Range, f"/{frame}", self.on_range, 10)
        self.hear_pub = self.create_publisher(String, "/voice_words", 10)
        self.say_pub = self.create_publisher(String, "/feedback_words", 10)
        self.create_timer(1.0, self.tick)
        self.create_timer(0.2, self.publish_pose)

    def mark(self, key):
        self.seen[key] = time.time()

    def arm_camera(self):
        if self.cam_sub is not None:
            return
        self.camera_armed_at = time.time()
        self.cam_sub = self.create_subscription(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            self.on_compressed,
            CAM_QOS,
        )

    def arm_raw_camera(self):
        if self.raw_sub is not None or self.cam_sub is None or self.have_compressed or self.cv_bridge is None:
            return
        if time.time() - self.camera_armed_at < 3.0:
            return
        self.raw_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.on_raw,
            CAM_QOS,
        )

    def arm_map(self):
        if self.map_sub is not None:
            return
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, MAP_QOS)

    def disarm_camera(self):
        if self.cam_sub is not None:
            self.destroy_subscription(self.cam_sub)
            self.cam_sub = None
        if self.raw_sub is not None:
            self.destroy_subscription(self.raw_sub)
            self.raw_sub = None
        self.have_compressed = False
        self.last_cam = 0.0

    def disarm_map(self):
        if self.map_sub is not None:
            self.destroy_subscription(self.map_sub)
            self.map_sub = None
        self.map_stamp = None
        self.last_map = 0.0

    def on_odom(self, msg):
        linear = msg.twist.twist.linear
        self.speed = math.hypot(linear.x, linear.y)
        self.mark("odom")

    def on_voltage(self, msg):
        self.voltage = float(msg.data)
        self.mark("voltage")

    def on_voice(self, msg):
        text = (msg.data or "").strip()
        if text:
            emit({"type": "voice", "text": text})
            self.mark("voice")

    def on_feedback(self, msg):
        text = (msg.data or "").strip()
        if text:
            emit({"type": "feedback", "text": text})
            self.mark("voice")

    def send_jpeg(self, blob):
        now = time.time()
        if now - self.last_cam < CAMERA_INTERVAL:
            return
        if not blob or len(blob) > 500000:
            return
        self.last_cam = now
        self.mark("camera")
        emit({"type": "camera", "jpeg": base64.b64encode(bytes(blob)).decode("ascii")})

    def on_compressed(self, msg):
        self.have_compressed = True
        self.send_jpeg(msg.data)

    def on_raw(self, msg):
        if self.have_compressed or cv2 is None or self.cv_bridge is None:
            return
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        except Exception:
            return
        if ok:
            self.send_jpeg(encoded.tobytes())

    def on_range(self, msg):
        frame = msg.header.frame_id
        if frame not in ULTRA_FRAMES:
            return
        reading = float(msg.range)
        if not math.isfinite(reading) or reading < msg.min_range or reading > msg.max_range:
            self.ranges.pop(frame, None)
            return
        self.ranges[frame] = reading
        self.mark("ultra")

    def lookup(self, target, source):
        if self.tf_buffer is None:
            return None
        try:
            return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except TransformException:
            return None

    def publish_pose(self):
        transform = self.lookup("map", "base_footprint")
        if transform is None:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        beams = []
        for frame, reading in list(self.ranges.items()):
            sensor = self.lookup("base_footprint", frame)
            if sensor is None:
                continue
            place = sensor.transform.translation
            beams.append({
                "r": round(reading, 3),
                "x": round(place.x, 3),
                "y": round(place.y, 3),
                "yaw": round(yaw_of(sensor.transform.rotation), 4),
            })
        self.pose_state = (
            round(translation.x, 3),
            round(translation.y, 3),
            round(yaw_of(rotation), 4),
        )
        emit({
            "type": "pose",
            "x": self.pose_state[0],
            "y": self.pose_state[1],
            "yaw": self.pose_state[2],
            "beams": beams,
        })

    def on_map(self, msg):
        now = time.time()
        info = msg.info
        stamp = (info.width, info.height, msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.map_stamp and now - self.last_map < MAP_INTERVAL:
            self.mark("map")
            return
        if now - self.last_map < MAP_INTERVAL:
            self.mark("map")
            return
        try:
            width, height, resolution, origin_x, origin_y, raw = pack_grid(
                info.width,
                info.height,
                msg.data,
                info.resolution,
                info.origin.position.x,
                info.origin.position.y,
            )
        except Exception as exc:
            emit({"type": "error", "message": f"地图编码失败: {exc}"})
            return
        self.map_wh = (width, height, resolution)
        self.map_stamp = stamp
        self.last_map = now
        self.mark("map")
        emit({
            "type": "map",
            "w": width,
            "h": height,
            "res": resolution,
            "ox": origin_x,
            "oy": origin_y,
            "oyaw": yaw_of(info.origin.orientation),
            "grid": base64.b64encode(raw).decode("ascii"),
        })

    def voice_ready(self):
        # The live recognizer does not publish /voice_words. It is healthy when
        # robot-voice-visual.py is the recognition-only process and its log
        # still shows a fresh microphone block.
        try:
            pid = int(open("/home/wheeltec/robot_voice/recognition.pid", encoding="utf-8").read().strip())
            command = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode("utf-8", "replace")
        except (OSError, ValueError):
            return False
        if "robot-voice-visual.py" not in command or " --execute" in f" {command} ":
            return False
        try:
            with open("/home/wheeltec/robot_voice/voice-stable-live.log", "rb") as handle:
                handle.seek(0, os.SEEK_END)
                handle.seek(max(0, handle.tell() - 12000))
                tail = handle.read().decode("utf-8", "replace")
        except OSError:
            return False
        now = time.time()
        for line in reversed(tail.splitlines()):
            if "audio_age_s" not in line:
                continue
            try:
                item = json.loads(line)
                stamp = datetime.fromisoformat(item["utc"]).timestamp()
                audio = item.get("audio_age_s")
                blocks = item.get("audio_blocks")
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                continue
            age = now - stamp
            return (
                0 <= age < 8
                and item.get("execute") is False
                and isinstance(audio, (int, float))
                and math.isfinite(audio)
                and 0 <= audio < 1
                and isinstance(blocks, int)
                and blocks > 0
            )
        return False

    def others(self, topic):
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception:
            return 0
        return sum(1 for info in infos if info.node_name != self.get_name())

    def missing(self, topic, key, stale):
        if self.others(topic) <= 0:
            return True
        if stale is None:
            return False
        last = self.seen.get(key, 0)
        return last == 0 or time.time() - last > stale

    def tick(self):
        with self.lock:
            queued = self.pending
            self.pending = []
        for item in queued:
            if item.get("op") in ("watch", "unwatch"):
                target = item.get("target")
                if item.get("op") == "watch" and target == "camera":
                    self.arm_camera()
                elif item.get("op") == "watch" and target == "map":
                    self.arm_map()
                elif item.get("op") == "unwatch" and target == "camera":
                    self.disarm_camera()
                elif item.get("op") == "unwatch" and target == "map":
                    self.disarm_map()
                continue
            text = str(item.get("text") or "").strip()
            if not text:
                continue
            message = String()
            message.data = text
            if item.get("op") == "hear":
                self.hear_pub.publish(message)
            elif item.get("op") == "say":
                self.say_pub.publish(message)
        self.arm_raw_camera()

        fault = ""
        checks = (
            ("底盘", "/odom", "odom", 2.5),
            ("相机", "/camera/color/image_raw", "camera", 3.0),
            ("超声波", "/ultrasonic_data_A", "ultra", None),
            ("SLAM", "/map", "map", 8.0),
            ("语音", "", "voice", None),
        )
        for label, topic, key, stale in checks:
            if label == "语音":
                bad = not self.voice_ready()
            elif label == "相机" and self.cam_sub is None:
                bad = False
            elif label == "SLAM" and self.map_sub is None:
                bad = False
            elif label == "SLAM":
                bad = self.others("/map") <= 0 or self.seen.get("map", 0) == 0
            else:
                bad = self.missing(topic, key, stale)
            if bad:
                fault = label
                break
        emit({
            "type": "status",
            "speed": None if self.speed is None else round(self.speed, 3),
            "voltage": None if self.voltage is None else round(self.voltage, 2),
            "fault": fault,
        })
        try:
            emit(self.sample_host())
        except Exception:
            pass

    def sample_host(self):
        parts = open("/proc/stat", encoding="ascii").readline().split()
        nums = [int(item) for item in parts[1:8]]
        idle = nums[3] + nums[4]
        total = sum(nums)
        cpu = None
        if self.host_cpu and total > self.host_cpu[1]:
            cpu = 100.0 * (1.0 - (idle - self.host_cpu[0]) / (total - self.host_cpu[1]))
            cpu = round(max(0.0, min(100.0, cpu)), 1)
        self.host_cpu = (idle, total)

        mem = {}
        for line in open("/proc/meminfo", encoding="ascii"):
            key, value = line.split(":", 1)
            mem[key] = int(value.split()[0])
        total_mib = mem["MemTotal"] / 1024.0
        avail_mib = mem.get("MemAvailable", mem.get("MemFree", 0)) / 1024.0
        load = [float(item) for item in open("/proc/loadavg", encoding="ascii").read().split()[:3]]
        uptime = int(float(open("/proc/uptime", encoding="ascii").read().split()[0]))
        temp = None
        try:
            temp = int(open("/sys/class/thermal/thermal_zone0/temp", encoding="ascii").read()) / 1000.0
        except OSError:
            temp = None

        now = time.time()
        elapsed = None if self.host_proc_at is None else now - self.host_proc_at
        clk = os.sysconf("SC_CLK_TCK")
        cores = max(1, os.cpu_count() or 1)
        page = os.sysconf("SC_PAGE_SIZE") // 1024
        found = {}
        ticks_now = {}
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                stat = open(f"/proc/{pid}/stat", encoding="ascii", errors="replace").read()
                rest = stat[stat.rfind(")") + 2:].split()
                ticks = int(rest[11]) + int(rest[12])
            except (OSError, IndexError, ValueError):
                continue
            ticks_now[pid] = ticks
            previous = self.host_proc.get(pid)
            if previous is None or not elapsed or elapsed <= 0 or clk <= 0:
                continue
            share = (ticks - previous) / clk / elapsed * 100.0 / cores
            if share < 0.4:
                continue
            try:
                command = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode("utf-8", "replace")
                rss_pages = int(open(f"/proc/{pid}/statm", encoding="ascii").read().split()[1])
            except (OSError, IndexError, ValueError):
                command, rss_pages = "", 0
            comm = stat[stat.find("(") + 1:stat.rfind(")")]
            label = proc_label(command, comm)
            bucket = found.setdefault(label, {"name": label, "cpu": 0.0, "rss": 0})
            bucket["cpu"] += share
            bucket["rss"] += rss_pages * page
        self.host_proc = ticks_now
        self.host_proc_at = now
        procs = sorted(found.values(), key=lambda item: item["cpu"], reverse=True)[:12]
        for item in procs:
            item["cpu"] = round(item["cpu"], 1)
            item["rss"] = round(item["rss"] / 1024.0)
        if cpu is not None and procs:
            rest = round(max(0.0, cpu - sum(item["cpu"] for item in procs)), 1)
            if rest >= 0.4:
                procs.append({"name": "其余", "cpu": rest, "rss": None})
        pose = None
        if self.pose_state:
            pose = {"x": self.pose_state[0], "y": self.pose_state[1], "yaw": self.pose_state[2]}
        grid = None
        if self.map_wh:
            grid = {"w": self.map_wh[0], "h": self.map_wh[1], "res": self.map_wh[2]}
        return {
            "type": "host",
            "cpu": cpu,
            "cores": os.cpu_count() or 0,
            "load": load,
            "mem_used": round(total_mib - avail_mib),
            "mem_total": round(total_mib),
            "uptime": uptime,
            "temp": None if temp is None else round(temp, 1),
            "procs": procs,
            "pose": pose,
            "map": grid,
        }

    def enqueue(self, item):
        with self.lock:
            self.pending.append(item)


def stdin_loop(node):
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if item.get("op") in ("watch", "unwatch", "hear", "say"):
            node.enqueue(item)


def main():
    rclpy.init(args=["--ros-args", "--log-level", "warn"])
    node = Bridge()
    threading.Thread(target=stdin_loop, args=(node,), daemon=True).start()
    emit({"type": "ready"})
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
