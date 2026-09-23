# servo_controller

独立舵机验证包，不接入 `mavros_offboard`，不发布飞控 setpoint，不切换模式，不解锁。

## 节点

- `servo_driver_node`: 初始化 PCA9685/ServoKit，并提供三个舵机服务。
- `servo_waypoint_task`: 订阅 `/mavros/local_position/pose`，到达配置坐标后调用对应舵机服务。

`servo_driver_node` 启动后默认会先执行一次前置动作：三个舵机全部转到 `startup_angle`，默认 `90.0` 度。
后续到达坐标触发时，对应舵机转到 `*_trigger_angle`，默认 `0.0` 度。

## 服务

```bash
/servo/left_home
/servo/right_home
/servo/big_home
/servo/left_90
/servo/right_90
/servo/big_90
/servo/home_all
```

手动测试：

```bash
ros2 service call /servo/big_home std_srvs/srv/Trigger {}
ros2 service call /servo/big_90 std_srvs/srv/Trigger {}
ros2 service call /servo/left_90 std_srvs/srv/Trigger {}
ros2 service call /servo/right_90 std_srvs/srv/Trigger {}
```

## 启动

```bash
source /opt/ros/humble/setup.bash
source /home/banana/detect_ws/install/setup.bash
ros2 launch servo_controller servo_standalone.launch.py
```

无硬件 dry-run 测试：

```bash
ros2 launch servo_controller servo_standalone.launch.py dry_run:=true
```

## 改坐标

坐标在 `config/servo_waypoints.yaml`：

```yaml
big_point: [0.20, 0.0]
left_point: [0.20, 0.20]
right_point: [0.0, 0.20]
```

这些坐标使用 `/mavros/local_position/pose` 的 EKF2 local 平面坐标系，只按 XY 距离触发，不判断 Z 高度。改 YAML 后重新启动 launch 即可，不需要重新 build。

启动前置动作也在同一个配置文件里：

```yaml
startup_move_all_on_start: true
startup_angle: 90.0
startup_move_delay_s: 0.5

left_trigger_angle: 0.0
right_trigger_angle: 0.0
big_trigger_angle: 0.0
```

注意：默认 launch 读取的是安装后的配置文件。如果直接修改 `src/servo_controller/config/servo_waypoints.yaml`，需要重新执行 `colcon build --packages-select servo_controller`，或者启动时显式传入源码配置路径：

```bash
ros2 launch servo_controller servo_standalone.launch.py config:=/home/banana/detect_ws/src/servo_controller/config/servo_waypoints.yaml
```

## 硬件依赖

运行真实舵机需要 Python 包：

```bash
pip install adafruit-circuitpython-servokit adafruit-extended-bus
```
