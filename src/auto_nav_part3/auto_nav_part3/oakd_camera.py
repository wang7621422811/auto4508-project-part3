#!/usr/bin/env python3
"""
OAK-D ROS2 camera node — runs INSIDE Docker container.
Uses depthai directly to capture RGB + depth and publishes as ROS Image topics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import depthai as dai
import threading
import time


class OakDCamera(Node):
    def __init__(self):
        super().__init__('oakd_camera')
        self.rgb_pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oak/stereo/depth', 10)

        self.device = None
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        self.get_logger().info('OAK-D camera node started')

    def _build_pipeline(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(15)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.preview.link(xout_rgb.input)

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera('left')
        mono_left.setFps(15)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setCamera('right')
        mono_right.setFps(15)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName('depth')
        stereo.depth.link(xout_depth.input)

        return pipeline

    def _capture_loop(self):
        while rclpy.ok():
            try:
                pipeline = self._build_pipeline()
                self.get_logger().info('Connecting to OAK-D...')
                with dai.Device(pipeline) as device:
                    self.device = device
                    self.get_logger().info(f'OAK-D connected: {device.getMxId()}')
                    q_rgb = device.getOutputQueue('rgb', maxSize=4, blocking=False)
                    q_depth = device.getOutputQueue('depth', maxSize=4, blocking=False)

                    while rclpy.ok():
                        rgb = q_rgb.tryGet()
                        if rgb is not None:
                            frame = rgb.getCvFrame()
                            msg = Image()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = 'oak_rgb'
                            msg.height, msg.width = frame.shape[:2]
                            msg.encoding = 'bgr8'
                            msg.step = msg.width * 3
                            msg.data = frame.tobytes()
                            self.rgb_pub.publish(msg)

                        depth = q_depth.tryGet()
                        if depth is not None:
                            frame = depth.getCvFrame()
                            msg = Image()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = 'oak_depth'
                            msg.height, msg.width = frame.shape[:2]
                            msg.encoding = 'mono16'
                            msg.step = msg.width * 2
                            msg.data = frame.tobytes()
                            self.depth_pub.publish(msg)

                        time.sleep(0.01)

            except Exception as e:
                self.get_logger().warn(f'OAK-D error: {e}. Retrying in 3s...')
                time.sleep(3)

    def destroy_node(self):
        self.device = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakDCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
OAK-D ROS2 camera node — runs INSIDE Docker container.
Uses depthai directly to capture RGB + depth and publishes as ROS Image topics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import depthai as dai
import threading
import time


class OakDCamera(Node):
    def __init__(self):
        super().__init__('oakd_camera')
        self.rgb_pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oak/stereo/depth', 10)

        self.device = None
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        self.get_logger().info('OAK-D camera node started')

    def _build_pipeline(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(15)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.preview.link(xout_rgb.input)

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera('left')
        mono_left.setFps(15)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setCamera('right')
        mono_right.setFps(15)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName('depth')
        stereo.depth.link(xout_depth.input)

        return pipeline

    def _capture_loop(self):
        while rclpy.ok():
            try:
                pipeline = self._build_pipeline()
                self.get_logger().info('Connecting to OAK-D...')
                with dai.Device(pipeline) as device:
                    self.device = device
                    self.get_logger().info(f'OAK-D connected: {device.getMxId()}')
                    q_rgb = device.getOutputQueue('rgb', maxSize=4, blocking=False)
                    q_depth = device.getOutputQueue('depth', maxSize=4, blocking=False)

                    while rclpy.ok():
                        rgb = q_rgb.tryGet()
                        if rgb is not None:
                            frame = rgb.getCvFrame()
                            msg = Image()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = 'oak_rgb'
                            msg.height, msg.width = frame.shape[:2]
                            msg.encoding = 'bgr8'
                            msg.step = msg.width * 3
                            msg.data = frame.tobytes()
                            self.rgb_pub.publish(msg)

                        depth = q_depth.tryGet()
                        if depth is not None:
                            frame = depth.getCvFrame()
                            msg = Image()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = 'oak_depth'
                            msg.height, msg.width = frame.shape[:2]
                            msg.encoding = 'mono16'
                            msg.step = msg.width * 2
                            msg.data = frame.tobytes()
                            self.depth_pub.publish(msg)

                        time.sleep(0.01)

            except Exception as e:
                self.get_logger().warn(f'OAK-D error: {e}. Retrying in 3s...')
                time.sleep(3)

    def destroy_node(self):
        self.device = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakDCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
