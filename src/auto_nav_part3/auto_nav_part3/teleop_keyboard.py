#!/usr/bin/env python3
"""
teleop_keyboard.py
Keyboard teleoperation for Pioneer 3-AT.
Reads keystrokes and publishes geometry_msgs/Twist to /cmd_vel.

Flow:
  keyboard → /cmd_vel (ROS) → ros_gz_bridge → Gazebo diff-drive
           → wheels turn → /odom + /tf (odom→base_link) updated

Key bindings
────────────
  W / ↑   Forward           Q   Linear speed  +0.05 m/s
  S / ↓   Backward          E   Linear speed  −0.05 m/s
  A / ←   Turn left         Z   Angular speed +0.10 rad/s
  D / →   Turn right        X   Angular speed −0.10 rad/s
  Space   Stop (zero vel)   Ctrl+C  Quit
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


BANNER = """\
┌──────────────────────────────────────────────────────┐
│          Pioneer 3-AT  Keyboard Teleop               │
├───────────────────────┬──────────────────────────────┤
│  W / ↑   Forward      │  Q   Linear speed  +0.05 m/s │
│  S / ↓   Backward     │  E   Linear speed  −0.05 m/s │
│  A / ←   Turn left    │  Z   Angular speed +0.10 r/s │
│  D / →   Turn right   │  X   Angular speed −0.10 r/s │
│  Space   Stop         │  Ctrl+C  Quit                │
└───────────────────────┴──────────────────────────────┘
"""

LIN_STEP = 0.05   # m/s per Q/E press
ANG_STEP = 0.10   # rad/s per Z/X press
LIN_MAX  = 1.00
LIN_MIN  = 0.05
ANG_MAX  = 2.00
ANG_MIN  = 0.10

_ARROW = {'\x1b[A': 'w', '\x1b[B': 's', '\x1b[D': 'a', '\x1b[C': 'd'}


def _read_key(saved: list) -> str:
    """
    Block until a key is pressed, then return it as a string.
    Arrow-key escape sequences (\x1b[A/B/C/D) are mapped to w/s/d/a.
    """
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        # Collect the rest of a potential escape sequence with a short timeout
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch += sys.stdin.read(1)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch += sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
    return _ARROW.get(ch, ch)


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._lin = 0.20   # current linear speed magnitude (m/s)
        self._ang = 0.50   # current angular speed magnitude (rad/s)
        self.get_logger().info(
            f'Publishing to /cmd_vel  '
            f'(linear={self._lin:.2f} m/s  angular={self._ang:.2f} rad/s)'
        )

    # ── internal ──────────────────────────────────────────────────────────────

    def _send(self, lx: float, az: float) -> None:
        msg = Twist()
        msg.linear.x  = lx
        msg.angular.z = az
        self._pub.publish(msg)

    def _status(self, label: str, lx: float, az: float) -> None:
        print(
            f'\r  {label:<13s} │ '
            f'cmd_vel  lin={lx:+.2f} m/s  ang={az:+.2f} rad/s  '
            f'│ speed [{self._lin:.2f} m/s / {self._ang:.2f} rad/s]   ',
            end='', flush=True,
        )

    # ── public ────────────────────────────────────────────────────────────────

    def stop(self) -> None:
        self._send(0.0, 0.0)

    def handle(self, key: str) -> bool:
        """
        Process one key event.
        Returns False when the user requests shutdown (Ctrl+C).
        """
        key = key.lower()

        if key == '\x03':   # Ctrl+C
            return False

        # ── speed adjustments (no motion command sent) ─────────────────────
        if key == 'q':
            self._lin = min(self._lin + LIN_STEP, LIN_MAX)
            print(f'\r  [Q] Linear speed  → {self._lin:.2f} m/s              ', end='', flush=True)
            return True
        if key == 'e':
            self._lin = max(self._lin - LIN_STEP, LIN_MIN)
            print(f'\r  [E] Linear speed  → {self._lin:.2f} m/s              ', end='', flush=True)
            return True
        if key == 'z':
            self._ang = min(self._ang + ANG_STEP, ANG_MAX)
            print(f'\r  [Z] Angular speed → {self._ang:.2f} rad/s            ', end='', flush=True)
            return True
        if key == 'x':
            self._ang = max(self._ang - ANG_STEP, ANG_MIN)
            print(f'\r  [X] Angular speed → {self._ang:.2f} rad/s            ', end='', flush=True)
            return True

        # ── motion commands ───────────────────────────────────────────────
        lx = az = 0.0
        label = 'Stop'

        if   key == 'w':  lx =  self._lin; label = 'Forward'
        elif key == 's':  lx = -self._lin; label = 'Backward'
        elif key == 'a':  az =  self._ang; label = 'Turn Left'
        elif key == 'd':  az = -self._ang; label = 'Turn Right'
        elif key == ' ':  pass             # stop
        else:             return True      # unknown key — ignore

        self._send(lx, az)
        self._status(label, lx, az)
        return True


def main(args=None):
    # termios requires a real TTY on stdin.  ros2 launch does not forward the
    # terminal's stdin to child processes, so keyboard reading would fail with
    # "Inappropriate ioctl for device".  Detect this early and give a clear hint.
    if not sys.stdin.isatty():
        print(
            '[teleop_keyboard] ERROR: stdin is not a terminal.\n'
            'Teleop must run in a foreground terminal, not via ros2 launch.\n'
            'Use one of:\n'
            '  ./scripts/launch.sh start teleop\n'
            '  ros2 run auto_nav_part3 teleop_keyboard',
            file=sys.stderr,
        )
        sys.exit(1)

    rclpy.init(args=args)
    node = TeleopKeyboard()
    saved = termios.tcgetattr(sys.stdin)
    print(BANNER)
    try:
        while rclpy.ok():
            key = _read_key(saved)
            if not node.handle(key):
                break
    finally:
        print('\n  Stopping robot...')
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
