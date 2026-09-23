#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServoDriverNode(Node):
    """PCA9685/ServoKit driver exposed through simple Trigger services."""

    def __init__(self) -> None:
        super().__init__("servo_driver_node")

        self.declare_parameter("dry_run", False)
        self.declare_parameter("i2c_bus", 2)
        self.declare_parameter("frequency_hz", 50)
        self.declare_parameter("min_pulse_us", 500)
        self.declare_parameter("max_pulse_us", 2500)

        self.declare_parameter("left_channel", 0)
        self.declare_parameter("right_channel", 1)
        self.declare_parameter("big_channel", 2)

        self.declare_parameter("left_home_angle", 0.0)
        self.declare_parameter("right_home_angle", 0.0)
        self.declare_parameter("big_home_angle", 0.0)
        self.declare_parameter("left_trigger_angle", 0.0)
        self.declare_parameter("right_trigger_angle", 0.0)
        self.declare_parameter("big_trigger_angle", 0.0)

        self.declare_parameter("move_delay_s", 0.5)
        self.declare_parameter("reset_after_trigger", False)
        self.declare_parameter("reset_delay_s", 0.8)
        self.declare_parameter("set_home_on_start", True)
        self.declare_parameter("startup_move_all_on_start", True)
        self.declare_parameter("startup_angle", 90.0)
        self.declare_parameter("startup_move_delay_s", 0.5)

        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._kit = None

        self._servos = {
            "left": {
                "channel": int(self.get_parameter("left_channel").value),
                "home": float(self.get_parameter("left_home_angle").value),
                "trigger": float(self.get_parameter("left_trigger_angle").value),
            },
            "right": {
                "channel": int(self.get_parameter("right_channel").value),
                "home": float(self.get_parameter("right_home_angle").value),
                "trigger": float(self.get_parameter("right_trigger_angle").value),
            },
            "big": {
                "channel": int(self.get_parameter("big_channel").value),
                "home": float(self.get_parameter("big_home_angle").value),
                "trigger": float(self.get_parameter("big_trigger_angle").value),
            },
        }

        self._init_hardware()

        if bool(self.get_parameter("set_home_on_start").value):
            self._set_all_home()

        if bool(self.get_parameter("startup_move_all_on_start").value):
            self._startup_move_all()

        self.create_service(Trigger, "/servo/left_90", self._make_trigger_cb("left"))
        self.create_service(Trigger, "/servo/right_90", self._make_trigger_cb("right"))
        self.create_service(Trigger, "/servo/big_90", self._make_trigger_cb("big"))
        self.create_service(Trigger, "/servo/left_home", self._make_home_cb("left"))
        self.create_service(Trigger, "/servo/right_home", self._make_home_cb("right"))
        self.create_service(Trigger, "/servo/big_home", self._make_home_cb("big"))
        self.create_service(Trigger, "/servo/home_all", self._home_all_cb)

        self.get_logger().info(
            "舵机服务已启动: /servo/*_home /servo/*_90 /servo/home_all"
        )

    def _init_hardware(self) -> None:
        if self._dry_run:
            self.get_logger().warn("dry_run=true，只打印动作，不写入 I2C/PWM")
            return

        i2c_bus = int(self.get_parameter("i2c_bus").value)
        frequency_hz = int(self.get_parameter("frequency_hz").value)
        min_pulse = int(self.get_parameter("min_pulse_us").value)
        max_pulse = int(self.get_parameter("max_pulse_us").value)

        try:
            from adafruit_extended_bus import ExtendedI2C as I2C
            from adafruit_servokit import ServoKit
        except ImportError as exc:
            raise RuntimeError(
                "缺少舵机硬件依赖，请安装 adafruit-circuitpython-servokit "
                "和 adafruit-extended-bus，或用 dry_run:=true 启动"
            ) from exc

        i2c = I2C(i2c_bus)
        self._kit = ServoKit(i2c=i2c, channels=16, frequency=frequency_hz)

        for config in self._servos.values():
            self._kit.servo[config["channel"]].set_pulse_width_range(
                min_pulse, max_pulse
            )

        self.get_logger().info(
            f"ServoKit 初始化完成: i2c_bus={i2c_bus}, frequency={frequency_hz}Hz"
        )

    def _set_angle(self, name: str, angle: float) -> None:
        config = self._servos[name]
        clamped_angle = max(0.0, min(180.0, float(angle)))
        channel = int(config["channel"])

        if self._dry_run:
            self.get_logger().info(
                f"[dry_run] {name} channel={channel} -> {clamped_angle:.1f} deg"
            )
            return

        self._kit.servo[channel].angle = clamped_angle

    def _trigger_servo(self, name: str) -> None:
        config = self._servos[name]
        self._set_angle(name, float(config["trigger"]))
        time.sleep(float(self.get_parameter("move_delay_s").value))

        if bool(self.get_parameter("reset_after_trigger").value):
            time.sleep(float(self.get_parameter("reset_delay_s").value))
            self._set_angle(name, float(config["home"]))

    def _set_all_home(self) -> None:
        for name, config in self._servos.items():
            self._set_angle(name, float(config["home"]))
        self.get_logger().info("三个舵机已设置到 home_angle")

    def _startup_move_all(self) -> None:
        startup_angle = float(self.get_parameter("startup_angle").value)
        for name in self._servos:
            self._set_angle(name, startup_angle)
        time.sleep(float(self.get_parameter("startup_move_delay_s").value))
        self.get_logger().info(f"前置动作完成: 三个舵机已转到 {startup_angle:.1f} deg")

    def _make_trigger_cb(self, name: str):
        def callback(_request: Trigger.Request, response: Trigger.Response):
            try:
                self._trigger_servo(name)
                response.success = True
                response.message = (
                    f"{name} servo moved to "
                    f"{self._servos[name]['trigger']:.1f} deg"
                )
                self.get_logger().info(response.message)
            except Exception as exc:  # noqa: BLE001
                response.success = False
                response.message = f"{name} servo failed: {exc}"
                self.get_logger().error(response.message)
            return response

        return callback

    def _make_home_cb(self, name: str):
        def callback(_request: Trigger.Request, response: Trigger.Response):
            try:
                home_angle = float(self._servos[name]["home"])
                self._set_angle(name, home_angle)
                response.success = True
                response.message = f"{name} servo moved home to {home_angle:.1f} deg"
                self.get_logger().info(response.message)
            except Exception as exc:  # noqa: BLE001
                response.success = False
                response.message = f"{name} servo home failed: {exc}"
                self.get_logger().error(response.message)
            return response

        return callback

    def _home_all_cb(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            self._set_all_home()
            response.success = True
            response.message = "all servos moved to home_angle"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"home_all failed: {exc}"
            self.get_logger().error(response.message)
        return response


def main() -> None:
    rclpy.init()
    node = ServoDriverNode()
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
