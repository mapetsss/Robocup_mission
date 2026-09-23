#!/usr/bin/env python3
"""从 ROS 2 图像话题保存图片，替代未安装的 image_view/image_saver。"""

import argparse
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageSaver:
    def __init__(self, topic, filename_format, count):
        self.bridge = CvBridge()
        self.filename_format = filename_format
        self.count = count
        self.saved = 0
        self.node = rclpy.create_node("detect_ws_image_saver")
        self.sub = self.node.create_subscription(
            Image,
            topic,
            self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.node.get_logger().error(f"图像转换失败: {exc}")
            return

        filename = self.make_filename(self.saved)
        os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)

        if not cv2.imwrite(filename, frame):
            self.node.get_logger().error(f"保存失败: {filename}")
            return

        self.saved += 1
        self.node.get_logger().info(f"已保存: {filename}")

    def make_filename(self, index):
        try:
            return self.filename_format % index
        except TypeError:
            root, ext = os.path.splitext(self.filename_format)
            return f"{root}_{index:04d}{ext or '.jpg'}"


def main():
    parser = argparse.ArgumentParser(description="保存 ROS 2 图像话题中的图片")
    parser.add_argument("--topic", default="/camera/image_raw", help="图像话题名")
    parser.add_argument(
        "--output",
        default="/home/banana/detect_ws/camera_frame_%04i.jpg",
        help="输出文件格式，例如 /home/banana/detect_ws/camera_frame_%04i.jpg",
    )
    parser.add_argument("--count", type=int, default=1, help="保存图片数量")
    parser.add_argument("--timeout", type=float, default=5.0, help="最长等待时间，单位秒")
    args = parser.parse_args()

    if args.count < 1:
        print("--count 必须大于等于 1", file=sys.stderr)
        return 2

    rclpy.init()
    saver = ImageSaver(args.topic, args.output, args.count)
    saver.node.get_logger().info(f"等待图像话题: {args.topic}")

    start = time.time()
    try:
        while rclpy.ok() and saver.saved < args.count:
            rclpy.spin_once(saver.node, timeout_sec=0.1)
            if time.time() - start > args.timeout:
                saver.node.get_logger().error(
                    f"{args.timeout:.1f}s 内没有保存到足够图片，请检查话题是否在发布"
                )
                return 1
    finally:
        saver.node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
