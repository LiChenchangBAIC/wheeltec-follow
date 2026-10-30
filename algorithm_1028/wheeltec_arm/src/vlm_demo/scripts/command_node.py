#!/usr/bin/env python3
import locale
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 确保标准输入输出使用 UTF-8，避免中文 Backspace 残留乱码
os.environ.setdefault("PYTHONIOENCODING", "utf-8")
try:
    locale.setlocale(locale.LC_ALL, "C.UTF-8")
except locale.Error:
    try:
        locale.setlocale(locale.LC_ALL, "en_US.UTF-8")
    except locale.Error:
        pass  # 若系统没有对应 locale，则跳过，仍尝试后续设置

if hasattr(sys.stdin, "reconfigure"):
    sys.stdin.reconfigure(encoding="utf-8")
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

try:
    import readline  # noqa: F401  # 显式加载 GNU readline，Linux 下用于正确处理多字节字符行编辑
except ImportError:
    readline = None

try:
    import termios
except ImportError:
    termios = None  # 非 POSIX 终端（不会在 Ubuntu 上发生），做个兜底


def _save_terminal_settings():
    """保存当前 tty 设置，供退出时恢复，防止异常路径导致终端状态损坏。"""
    if termios is None or not sys.stdin.isatty():
        return None
    try:
        return termios.tcgetattr(sys.stdin.fileno())
    except Exception:
        return None


def _restore_terminal_settings(saved):
    """程序退出前恢复 tty 设置。"""
    if termios is None or saved is None:
        return
    try:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, saved)
    except Exception:
        pass


class CommandNode(Node):
    """Read terminal commands and publish them as ROS strings."""

    def __init__(self):
        super().__init__("command_node")
        self.command_topic = (
            self.declare_parameter("command_topic", "/vlm_demo/user_command")
            .get_parameter_value()
            .string_value
        )
        self.chat_topic = (
            self.declare_parameter("chat_topic", "/vlm_demo/chat_query")
            .get_parameter_value()
            .string_value
        )
        self.chat_response_topic = (
            self.declare_parameter("chat_response_topic", "/vlm_demo/chat_response")
            .get_parameter_value()
            .string_value
        )
        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.chat_pub = self.create_publisher(String, self.chat_topic, 10)
        self.chat_response_sub = self.create_subscription(
            String,
            self.chat_response_topic,
            self._chat_response_cb,
            10,
        )
        self._stopping = threading.Event()
        self._input_thread = threading.Thread(target=self._terminal_loop, daemon=True)
        self._input_thread.start()
        # 用 stderr 打印启动提示，避免和 input() 抢占 stdout 行编辑状态
        print(
            f"Command terminal ready. task={self.command_topic}, chat={self.chat_topic}",
            file=sys.stderr,
        )

    def destroy_node(self):
        self._stopping.set()
        return super().destroy_node()

    def _chat_response_cb(self, msg):
        print(f"\nchat> {msg.data}", file=sys.stderr)

    def _terminal_loop(self):
        print(
            "\n==== VLM pick terminal ====\n"
            "Type a natural-language robot command.\n"
            "Use '/chat ...' for visual Q&A, '/task ...' for an explicit robot task,\n"
            "or 'exit'/'quit' to stop.\n"
        )

        while rclpy.ok() and not self._stopping.is_set():
            try:
                # 保留 input("cmd> ") 原样，让 readline 完整接管 prompt/编辑/换行，
                # 不要手动拆成 sys.stdout.write + 裸 input()，否则会破坏 tty 状态。
                line = input("cmd> ").strip()
            except (EOFError, KeyboardInterrupt):
                self._stopping.set()
                if rclpy.ok():
                    rclpy.shutdown()
                break

            if not line:
                continue
            if line in ("exit", "quit"):
                self._stopping.set()
                if rclpy.ok():
                    rclpy.shutdown()
                break

            msg = String()
            if line.startswith("/chat"):
                chat_text = line[len("/chat"):].strip()
                if not chat_text:
                    print("请输入聊天内容，例如：/chat 视角里能看到什么", file=sys.stderr)
                    continue
                msg.data = chat_text
                self.chat_pub.publish(msg)
                print(f"Published chat: {chat_text}", file=sys.stderr)
                continue

            if line.startswith("/task"):
                line = line[len("/task"):].strip()
                if not line:
                    print("请输入任务内容，例如：/task 夹取绿色色块", file=sys.stderr)
                    continue

            msg.data = line
            self.command_pub.publish(msg)
            # 日志走 stderr，不与 input() 的 stdout 行编辑抢占同一个流，
            # 这是解决"需要多按一次回车"的根本方式（而不是加锁强行同步）。
            print(f"Published task: {line}", file=sys.stderr)


def main(args=None):
    saved_tty = _save_terminal_settings()
    rclpy.init(args=args)
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # 无论正常退出还是异常退出，都强制恢复 tty 设置，
        # 避免 Ctrl+C 或线程被杀死时终端残留在坏状态（无法输入/回显错乱）。
        _restore_terminal_settings(saved_tty)


if __name__ == "__main__":
    main()
