#!/usr/bin/env python3
import math
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


class ServoWaypointTask(Node):
    """Trigger one servo service when local pose reaches each configured point."""

    def __init__(self) -> None:
        super().__init__("servo_waypoint_task")

        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("reach_tolerance_m", 0.15)
        self.declare_parameter("big_point", [6.0, 1.0])
        self.declare_parameter("left_point", [3.6, -1.6])
        self.declare_parameter("right_point", [1.8, 1.6])
        self.declare_parameter("big_service", "/servo/big_home")
        self.declare_parameter("left_service", "/servo/left_home")
        self.declare_parameter("right_service", "/servo/right_home")
        self.declare_parameter("enable_big", True)
        self.declare_parameter("enable_left", True)
        self.declare_parameter("enable_right", True)

        self._tolerance = float(self.get_parameter("reach_tolerance_m").value)

        self._targets = {
            "big": self._load_target("big"),
            "left": self._load_target("left"),
            "right": self._load_target("right"),
        }
        self._enabled = {
            "big": bool(self.get_parameter("enable_big").value),
            "left": bool(self.get_parameter("enable_left").value),
            "right": bool(self.get_parameter("enable_right").value),
        }
        self._triggered = {"big": False, "left": False, "right": False}
        self._pending = {"big": False, "left": False, "right": False}
        self._servo_clients = {
            "big": self.create_client(Trigger, str(self.get_parameter("big_service").value)),
            "left": self.create_client(Trigger, str(self.get_parameter("left_service").value)),
            "right": self.create_client(Trigger, str(self.get_parameter("right_service").value)),
        }

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        pose_topic = str(self.get_parameter("pose_topic").value)
        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, qos)

        self.get_logger().info(
            f"坐标触发节点已启动: pose_topic={pose_topic}, tolerance={self._tolerance:.2f}m"
        )
        for name, point in self._targets.items():
            if self._enabled[name]:
                self.get_logger().info(f"{name} 触发点: {point}")

    def _load_target(self, name: str) -> List[float]:
        value = list(self.get_parameter(f"{name}_point").value)
        if len(value) != 2:
            raise ValueError(f"{name}_point 必须是 [x, y] 两个数")
        return [float(value[0]), float(value[1])]

    def _distance_to(self, msg: PoseStamped, point: List[float]) -> float:
        dx = msg.pose.position.x - point[0]
        dy = msg.pose.position.y - point[1]
        return math.sqrt(dx * dx + dy * dy)

    def _pose_cb(self, msg: PoseStamped) -> None:
        for name, point in self._targets.items():
            if not self._enabled[name] or self._triggered[name] or self._pending[name]:
                continue

            distance = self._distance_to(msg, point)
            if distance <= self._tolerance:
                self._call_servo(name, distance)

    def _call_servo(self, name: str, distance: float) -> None:
        client = self._servo_clients[name]
        if not client.service_is_ready():
            self.get_logger().warn(
                f"{name} 已到点但服务未就绪: {client.srv_name}"
            )
            return

        self._pending[name] = True
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done_future, servo_name=name: self._handle_response(
                servo_name, done_future
            )
        )
        self.get_logger().info(
            f"{name} 到达触发点，distance={distance:.3f}m，已调用 {client.srv_name}"
        )

    def _handle_response(self, name: str, future) -> None:
        self._pending[name] = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"{name} 舵机服务调用失败: {exc}")
            return

        if response.success:
            self._triggered[name] = True
            self.get_logger().info(f"{name} 舵机任务完成: {response.message}")
        else:
            self.get_logger().error(f"{name} 舵机任务失败: {response.message}")


def main() -> None:
    rclpy.init()
    node = ServoWaypointTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
