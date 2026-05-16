#!/usr/bin/env python3
"""
Pioneer 手柄控制器 - AUTO4508 Part 2
============================================

模式:
  O 键 → 手动模式: L1 + 摇杆直接控制车
  X 键 → 自动模式: Nav2 控制车
  方块键 → 死人开关切换 (自动模式下)

关键设计:
  手动模式: joy_controller 发布 /cmd_vel
  自动模式+死人开关开: joy_controller 不发任何 /cmd_vel, Nav2 直接控制
  自动模式+死人开关关: joy_controller 持续发零速度覆盖 Nav2 (100Hz)
  任何模式下 L1+摇杆 → 手动覆盖

发布: /cmd_vel, /robot_mode, /auto_enabled
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Bool


class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        # 按钮映射
        self.declare_parameter('btn_auto', 1)        # X
        self.declare_parameter('btn_manual', 2)      # O
        self.declare_parameter('btn_enable', 9)      # L1
        self.declare_parameter('btn_deadman', 3)     # 方块
        self.declare_parameter('axis_linear', 1)     # 左摇杆上下
        self.declare_parameter('axis_angular', 0)    # 左摇杆左右

        # 速度限制
        self.declare_parameter('manual_linear_max', 0.3)
        self.declare_parameter('manual_angular_max', 0.8)

        # 状态
        self.mode = 'manual'
        self.deadman_on = False
        self.deadman_btn_prev = 0
        self.manual_override = False
        self.last_joy_time = self.get_clock().now()
        self.last_joy_msg = None

        # 发布
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.auto_pub = self.create_publisher(Bool, '/auto_enabled', 10)

        # 订阅
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # 100Hz 控制循环 (关键: 持续覆盖 Nav2 指令)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        self.mode_timer = self.create_timer(1.0, self.publish_mode)

        self.get_logger().info('=== Pioneer 手柄控制器 ===')
        self.get_logger().info('X = 自动模式 | O = 手动模式 | 方块 = 死人开关')
        self.get_logger().info('自动模式: Nav2 直接控制 /cmd_vel')

    def joy_cb(self, msg):
        """只处理按钮事件和保存最新数据"""
        self.last_joy_time = self.get_clock().now()
        self.last_joy_msg = msg

        btn_auto = self.get_parameter('btn_auto').value
        btn_manual = self.get_parameter('btn_manual').value
        btn_deadman = self.get_parameter('btn_deadman').value

        # --- 模式切换 ---
        if len(msg.buttons) > btn_auto and msg.buttons[btn_auto] == 1:
            if self.mode != 'auto':
                self.mode = 'auto'
                self.deadman_on = False
                self.get_logger().info('>>> 自动模式 (按方块键启用Nav2)')

        if len(msg.buttons) > btn_manual and msg.buttons[btn_manual] == 1:
            if self.mode != 'manual':
                self.mode = 'manual'
                self.deadman_on = False
                self.get_logger().info('>>> 手动模式 (L1 + 摇杆)')

        # --- 方块键切换死人开关 ---
        if len(msg.buttons) > btn_deadman:
            btn_now = msg.buttons[btn_deadman]
            if btn_now == 1 and self.deadman_btn_prev == 0:
                self.deadman_on = not self.deadman_on
                if self.deadman_on:
                    self.get_logger().info('死人开关: 开启 (Nav2 接管)')
                else:
                    self.get_logger().info('死人开关: 关闭 (停车)')
            self.deadman_btn_prev = btn_now

    def control_loop(self):
        """100Hz 控制循环, 持续发送速度或让 Nav2 控制"""
        # 手柄超时 → 强制停车
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > 7.0:
            self.cmd_pub.publish(Twist())
            self.deadman_on = False
            self.manual_override = False
            return

        if self.last_joy_msg is None:
            return

        btn_enable = self.get_parameter('btn_enable').value
        axis_lin = self.get_parameter('axis_linear').value
        axis_ang = self.get_parameter('axis_angular').value

        msg = self.last_joy_msg
        l1_pressed = (len(msg.buttons) > btn_enable and msg.buttons[btn_enable] == 1)

        if self.mode == 'manual':
            cmd = Twist()
            if l1_pressed:
                max_lin = self.get_parameter('manual_linear_max').value
                max_ang = self.get_parameter('manual_angular_max').value
                if len(msg.axes) > max(axis_lin, axis_ang):
                    cmd.linear.x = msg.axes[axis_lin] * max_lin
                    cmd.angular.z = msg.axes[axis_ang] * max_ang
            self.cmd_pub.publish(cmd)
            self.manual_override = False

        elif self.mode == 'auto':
            if l1_pressed:
                cmd = Twist()
                max_lin = self.get_parameter('manual_linear_max').value
                max_ang = self.get_parameter('manual_angular_max').value
                if len(msg.axes) > max(axis_lin, axis_ang):
                    cmd.linear.x = msg.axes[axis_lin] * max_lin
                    cmd.angular.z = msg.axes[axis_ang] * max_ang
                self.cmd_pub.publish(cmd)
                self.manual_override = True
            else:
                self.manual_override = False
                if self.deadman_on:
                    # Nav2 接管, 不发任何东西
                    pass
                else:
                    # 死人开关关 → 持续发零速度覆盖 Nav2 (100Hz)
                    self.cmd_pub.publish(Twist())

        auto_msg = Bool()
        auto_msg.data = (self.mode == 'auto' and self.deadman_on)
        self.auto_pub.publish(auto_msg)

    def publish_mode(self):
        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

        status = f'模式: {self.mode}'
        if self.mode == 'auto':
            status += f' | 死人开关: {"开启" if self.deadman_on else "关闭"}'
            if self.manual_override:
                status += ' | L1覆盖'
        self.get_logger().info(status, throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
