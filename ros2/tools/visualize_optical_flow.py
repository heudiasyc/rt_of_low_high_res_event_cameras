#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class OpticalFlowVisualizer(Node):

    def __init__(self):
        super().__init__('optical_flow_visualizer')

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            '/optical_flow_visualization',
            10
        )

        self.subscription = self.create_subscription(
            Image,
            '/optical_flow',
            self.callback,
            10
        )

        self.frame_count = 0

        self.get_logger().info(
            'Optical flow visualization started'
        )

    def callback(self, msg):

        flow = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        if flow.ndim != 3 or flow.shape[2] != 2:
            self.get_logger().error(
                f'Unexpected flow shape: {flow.shape}'
            )
            return

        flow = np.nan_to_num(
            flow,
            nan=0.0,
            posinf=0.0,
            neginf=0.0
        )

        fx = flow[:, :, 0]
        fy = flow[:, :, 1]

        magnitude, angle = cv2.cartToPolar(
            fx,
            fy,
            angleInDegrees=True
        )

        hsv = np.zeros(
            (flow.shape[0], flow.shape[1], 3),
            dtype=np.uint8
        )

        # 颜色表示运动方向
        hsv[:, :, 0] = (angle / 2).astype(np.uint8)

        # 饱和度
        hsv[:, :, 1] = 255

        # 亮度表示运动速度
        hsv[:, :, 2] = np.clip(
            magnitude * 100,
            0,
            255
        ).astype(np.uint8)

        bgr = cv2.cvtColor(
            hsv,
            cv2.COLOR_HSV2BGR
        )

        output = self.bridge.cv2_to_imgmsg(
            bgr,
            encoding='bgr8'
        )

        output.header = msg.header

        self.publisher.publish(output)

        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Visualized {self.frame_count} flow frames'
            )


def main():
    rclpy.init()

    node = OpticalFlowVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
