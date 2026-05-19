"""
C_S.2 — Rolling Recorder: 5 秒滚动缓冲 + 急停写盘

机器人运行时，始终在内存里保留最近 5 秒的传感器与系统数据。
收到 /part3/safety/estop_event 时，立即把当前窗口的快照写成 rosbag 文件，
供离线回放和事故分析（PDF Task 6 要求）。

设计要点：
  - 每条消息收到时立即序列化为 CDR bytes，存入 deque，减少快照时的序列化开销。
  - 写盘在独立守护线程中完成，不阻塞 spin。
  - _saving 标志防止同一次急停的多条 event 触发并发写盘。
  - 依赖 rosbag2_py（ROS2 Jazzy 内置，无需额外安装）。
"""

import os
import threading
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

import rosbag2_py


# 话题名 → (ROS2 类型字符串, 消息类) 映射
# 只缓存对事故回放最有价值的话题；/camera 数据量大，如内存受限可注释掉
_RECORD_TOPICS: dict[str, tuple[str, type]] = {
    '/scan':                     ('sensor_msgs/msg/LaserScan', LaserScan),
    '/camera':                   ('sensor_msgs/msg/Image',     Image),
    '/odometry/filtered':        ('nav_msgs/msg/Odometry',     Odometry),
    '/tf':                       ('tf2_msgs/msg/TFMessage',    TFMessage),
    '/part3/system/state':       ('std_msgs/msg/String',       String),
    '/part3/safety/estop_event': ('std_msgs/msg/String',       String),
}


class RollingRecorder(Node):
    """
    滚动缓冲记录器：5 秒窗口，急停时写盘。

    缓冲元素：(topic_name, type_str, cdr_bytes, timestamp_ns)
    写盘格式：rosbag2 sqlite3，路径 <bag_save_dir>/estop_<timestamp>/
    """

    def __init__(self):
        super().__init__('part3_rolling_recorder')

        # ── 参数（来自 config/safety.yaml）──────────────────────────────────────
        self.declare_parameter('buffer_duration_sec', 5.0)
        self.declare_parameter('bag_save_dir', 'artifacts/bags')

        self._buf_dur  = float(self.get_parameter('buffer_duration_sec').value)
        self._save_dir = str(self.get_parameter('bag_save_dir').value)

        # ── 滚动缓冲 ─────────────────────────────────────────────────────────────
        # 元素：(topic_name, type_str, cdr_bytes, timestamp_ns)
        self._buffer: deque[tuple[str, str, bytes, int]] = deque()
        self._lock    = threading.Lock() # lock 保护 buffer 和 _saving 标志，确保线程安全
        self._saving  = False   # 防止同一次急停重复写盘

        # ── 订阅所有需要缓存的话题 ───────────────────────────────────────────────
        for topic, (type_str, msg_cls) in _RECORD_TOPICS.items():
            self.create_subscription(
                msg_cls, topic,
                # default-arg capture 确保 topic/type_str 按当前循环值绑定
                lambda msg, t=topic, ts=type_str: self._on_msg(t, ts, msg),
                10,
            )

        # 每秒清理过期条目（避免频繁 lock 争用）
        self.create_timer(1.0, self._trim_buffer)

        os.makedirs(self._save_dir, exist_ok=True)

        self.get_logger().info(
            f'RollingRecorder ready — '
            f'buffer={self._buf_dur}s  save_dir={self._save_dir}  '
            f'topics={len(_RECORD_TOPICS)}'
        )

    # ── 消息收集 ──────────────────────────────────────────────────────────────────

    def _on_msg(self, topic: str, type_str: str, msg) -> None:
        ts_ns = self.get_clock().now().nanoseconds
        # sim clock 未初始化时返回 0，跳过这类消息避免污染缓冲区
        # （这类消息后续会被 _trim_buffer 清掉，但提前过滤更干净）
        if ts_ns == 0:
            return
        cdr_bytes = serialize_message(msg)

        with self._lock:
            self._buffer.append((topic, type_str, cdr_bytes, ts_ns))

        # estop_event 到来时触发写盘（lock 已释放，避免死锁）
        if topic == '/part3/safety/estop_event':
            self._trigger_save()

    def _trim_buffer(self) -> None:
        """清理超过 buffer_duration_sec 的旧条目。"""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            return  # clock 未就绪，不修剪（防止 cutoff 变负数清掉有效消息）
        cutoff_ns = now_ns - int(self._buf_dur * 1e9)
        with self._lock:
            while self._buffer and self._buffer[0][3] < cutoff_ns:
                self._buffer.popleft()

    # ── 写盘触发 ──────────────────────────────────────────────────────────────────

    def _trigger_save(self) -> None:
        """拷贝当前缓冲快照并在守护线程中写盘；_saving 标志防并发。"""
        if self._saving:
            return

        with self._lock:
            snapshot = list(self._buffer)

        if not snapshot:
            return

        self._saving = True
        threading.Thread(
            target=self._write_bag,
            args=(snapshot,),
            daemon=True,
        ).start()

    def _write_bag(self, snapshot: list[tuple[str, str, bytes, int]]) -> None:
        """在独立线程把 snapshot 写成 rosbag2 sqlite3 文件。"""
        # 过滤 ts=0 的无效条目，并按时间戳升序排列
        # rosbag2 SQLite 后端要求消息时间戳单调递增，否则 write() 抛异常
        valid = sorted(
            [(t, ts, b, ns) for t, ts, b, ns in snapshot if ns > 0],
            key=lambda x: x[3],
        )
        if not valid:
            self.get_logger().warn('[RollingRecorder] 快照为空或全部时间戳无效，跳过写盘')
            self._saving = False
            return

        ts_sec   = valid[-1][3] / 1e9
        bag_path = os.path.join(self._save_dir, f'estop_{ts_sec:.3f}')

        try:
            storage_opts   = rosbag2_py.StorageOptions(
                uri=bag_path, storage_id='sqlite3'
            )
            converter_opts = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            )
            writer = rosbag2_py.SequentialWriter()
            writer.open(storage_opts, converter_opts)

            # 只为快照中实际出现的话题注册元数据
            # Jazzy 的 TopicMetadata 要求显式传入 id（从 0 递增）
            registered: set[str] = set()
            for topic, type_str, _, _ in valid:
                if topic not in registered:
                    writer.create_topic(rosbag2_py.TopicMetadata(
                        id=len(registered),
                        name=topic,
                        type=type_str,
                        serialization_format='cdr',
                    ))
                    registered.add(topic)

            for topic, _, cdr_bytes, ts_ns in valid:
                writer.write(topic, cdr_bytes, ts_ns)

            # CPython 引用计数保证 del 触发 __del__ → flush + close
            del writer

            duration_s = (valid[-1][3] - valid[0][3]) / 1e9
            self.get_logger().info(
                f'[RollingRecorder] 已保存 estop bag: {bag_path}  '
                f'({len(valid)} msgs, {duration_s:.1f}s)'
            )
        except Exception as exc:
            self.get_logger().error(f'[RollingRecorder] 写盘失败: {exc}')
        finally:
            self._saving = False


def main(args=None):
    rclpy.init(args=args)
    node = RollingRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
