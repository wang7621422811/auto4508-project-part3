#!/usr/bin/env python3
"""
pub_image.py — 把本地图片以指定频率发布到 ROS2 话题，用于测试感知节点。

用法:
  python3 scripts/pub_image.py <图片路径> [话题名] [频率Hz]

示例:
  python3 scripts/pub_image.py src/auto_nav_part3/resource/greek_number/alpha.png
  python3 scripts/pub_image.py src/auto_nav_part3/resource/greek_number/beta.png /oak/rgb/image_raw 10
"""

import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    img_path  = sys.argv[1] if len(sys.argv) > 1 else 'alpha.png'
    topic     = sys.argv[2] if len(sys.argv) > 2 else '/oak/rgb/image_raw'
    rate_hz   = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    img = cv2.imread(img_path)
    if img is None:
        print(f'[ERROR] 无法读取图片: {img_path}')
        sys.exit(1)
    # 统一 resize 到检测器期望的尺寸
    img = cv2.resize(img, (640, 480))

    rclpy.init()
    node   = Node('pub_image')
    pub    = node.create_publisher(Image, topic, 10)
    bridge = CvBridge()
    msg    = bridge.cv2_to_imgmsg(img, encoding='bgr8')

    def publish():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    node.create_timer(1.0 / rate_hz, publish)
    print(f'[pub_image] 发布 {img_path} → {topic}  @ {rate_hz}Hz  (Ctrl+C 停止)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
