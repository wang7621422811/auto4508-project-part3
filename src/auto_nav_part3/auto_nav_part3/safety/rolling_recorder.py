"""
C_S.2 — Rolling Recorder: 5-second rolling buffer + e-stop bag dump

While the robot runs, the most recent 5 seconds of sensor and system data are kept in
memory at all times. When /part3/safety/estop_event is received, the current window
snapshot is immediately written to a rosbag file for offline replay and incident analysis
(PDF Task 6 requirement).

Design notes:
  - Each incoming message is serialised to CDR bytes immediately and stored in a deque,
    reducing serialisation overhead at snapshot time.
  - Disk writing happens in a dedicated daemon thread so spin is never blocked.
  - The _saving flag prevents concurrent writes from multiple event messages in the same e-stop.
  - Depends on rosbag2_py (built into ROS 2 Jazzy, no extra installation needed).
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


# topic name → (ROS 2 type string, message class) mapping
# only buffer the topics most valuable for incident replay; /camera/image is large — comment it out if memory is constrained
_RECORD_TOPICS: dict[str, tuple[str, type]] = {
    '/scan':                     ('sensor_msgs/msg/LaserScan', LaserScan),
    '/camera/image':             ('sensor_msgs/msg/Image',     Image),
    '/odometry/filtered':        ('nav_msgs/msg/Odometry',     Odometry),
    '/tf':                       ('tf2_msgs/msg/TFMessage',    TFMessage),
    '/part3/system/state':       ('std_msgs/msg/String',       String),
    '/part3/safety/estop_event': ('std_msgs/msg/String',       String),
}


class RollingRecorder(Node):
    """
    Rolling buffer recorder: 5-second window, dumps to disk on e-stop.

    Buffer element: (topic_name, type_str, cdr_bytes, timestamp_ns)
    Output format: rosbag2 sqlite3, path <bag_save_dir>/estop_<timestamp>/
    """

    def __init__(self):
        super().__init__('part3_rolling_recorder')

        # ── parameters (from config/safety.yaml) ──────────────────────────────────
        self.declare_parameter('buffer_duration_sec', 5.0)
        self.declare_parameter('bag_save_dir', 'artifacts/bags')

        self._buf_dur  = float(self.get_parameter('buffer_duration_sec').value)
        self._save_dir = str(self.get_parameter('bag_save_dir').value)

        # ── rolling buffer ────────────────────────────────────────────────────────
        # element: (topic_name, type_str, cdr_bytes, timestamp_ns)
        self._buffer: deque[tuple[str, str, bytes, int]] = deque()
        self._lock    = threading.Lock() # lock protects buffer and _saving flag for thread safety
        self._saving  = False   # prevents duplicate writes for multiple events in the same e-stop

        # ── subscribe to all topics that should be buffered ───────────────────────
        for topic, (type_str, msg_cls) in _RECORD_TOPICS.items():
            self.create_subscription(
                msg_cls, topic,
                # default-arg capture ensures topic/type_str are bound to the current loop values
                lambda msg, t=topic, ts=type_str: self._on_msg(t, ts, msg),
                10,
            )

        # trim expired entries once per second (avoids frequent lock contention)
        self.create_timer(1.0, self._trim_buffer)

        os.makedirs(self._save_dir, exist_ok=True)

        self.get_logger().info(
            f'RollingRecorder ready — '
            f'buffer={self._buf_dur}s  save_dir={self._save_dir}  '
            f'topics={len(_RECORD_TOPICS)}'
        )

    # ── message collection ────────────────────────────────────────────────────────

    def _on_msg(self, topic: str, type_str: str, msg) -> None:
        ts_ns = self.get_clock().now().nanoseconds
        # sim clock returns 0 before initialisation; skip to avoid polluting the buffer
        # (_trim_buffer would remove them anyway, but filtering here is cleaner)
        if ts_ns == 0:
            return
        cdr_bytes = serialize_message(msg)

        with self._lock:
            self._buffer.append((topic, type_str, cdr_bytes, ts_ns))

        # trigger disk write when an estop_event arrives (lock already released to avoid deadlock)
        if topic == '/part3/safety/estop_event':
            self._trigger_save()

    def _trim_buffer(self) -> None:
        """Remove entries older than buffer_duration_sec."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            return  # clock not yet ready; skip trim to avoid a negative cutoff deleting valid messages
        cutoff_ns = now_ns - int(self._buf_dur * 1e9)
        with self._lock:
            while self._buffer and self._buffer[0][3] < cutoff_ns:
                self._buffer.popleft()

    # ── disk write trigger ────────────────────────────────────────────────────────

    def _trigger_save(self) -> None:
        """Copy the current buffer snapshot and write it to disk in a daemon thread; _saving prevents concurrent writes."""
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
        """Write snapshot to a rosbag2 sqlite3 file in a separate thread."""
        # filter out ts=0 entries and sort by timestamp ascending
        # rosbag2 SQLite backend requires monotonically increasing timestamps; write() raises otherwise
        valid = sorted(
            [(t, ts, b, ns) for t, ts, b, ns in snapshot if ns > 0],
            key=lambda x: x[3],
        )
        if not valid:
            self.get_logger().warn('[RollingRecorder] snapshot empty or all timestamps invalid, skipping write')
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

            # register metadata only for topics that actually appear in the snapshot
            # Jazzy's TopicMetadata requires an explicit id (incrementing from 0)
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

            # CPython reference counting guarantees del triggers __del__ → flush + close
            del writer

            duration_s = (valid[-1][3] - valid[0][3]) / 1e9
            self.get_logger().info(
                f'[RollingRecorder] saved estop bag: {bag_path}  '
                f'({len(valid)} msgs, {duration_s:.1f}s)'
            )
        except Exception as exc:
            self.get_logger().error(f'[RollingRecorder] failed to write bag: {exc}')
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
