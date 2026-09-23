# mavros_offboard

ROS 2 MAVROS offboard mission nodes for this workspace. This package only starts one offboard controller at a time. Do not run it together with `px4_offboard_mission` or any other node publishing `/mavros/setpoint_position/local`.

## Environment

Use it with the FAST-LIO/PX4 bridge from `/home/banana/fastlio_ws`:

```bash
source /opt/ros/humble/setup.bash
source /home/banana/fastlio_ws/install/setup.bash
source /home/banana/detect_ws/install/setup.bash
```

The expected localization chain is:

```text
FAST-LIO /Odometry -> drone_bridge -> /mavros/odometry/out -> PX4 EKF2 -> /mavros/local_position/pose
```

Start the localization and MAVROS chain first. A typical sequence is:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py rviz:=false
ros2 launch drone_bridge fastlio_to_px4_odom.launch.py
```

Start MAVROS with your flight-controller link as well, for example `/dev/ttyACM0:115200` if that is the active port. After `/mavros/state` is connected and `/mavros/local_position/pose` is fresh, start exactly one offboard mission.

## Robocup Mission

```bash
ros2 launch mavros_offboard robocup_v1.launch.py
```

`robocup_v1` 的任务点在 `config/robocup_v1_params.yaml` 中配置。节点启动并收到
`/mavros/local_position/pose` 后会锁定当前位姿为任务原点，之后所有 XY/Z 目标都按该
原点的相对坐标发布给 `/mavros/setpoint_position/local`，不会再飞向 EKF2 全局绝对
`(0,0)`。

主要参数：

- `param3`: home side, 1 for left and 2 for right.
- `target_fly_z`: task flight altitude relative to the locked origin.
- `target_drop_z`: drop altitude relative to the locked origin.
- `*_target_x`, `*_target_y`: target XY positions relative to the locked origin.
- `ring_center_x`, `ring_center_y`, `ring_center_z`: ring center relative to the locked origin.
- `pose_timeout_s`: stop advancing the state machine if `/mavros/local_position/pose` is stale.

The node requests `OFFBOARD`, but it never sends an arming command. Arm manually from the RC after OFFBOARD is accepted.

## Offboard + Servo Mission

这个流程由 `mavros_offboard_node` 统一控制飞行和舵机触发：

```text
起飞到原点悬停点 -> /servo/big_home -> 左飞到指定点 -> /servo/left_home -> /servo/right_home -> AUTO.LAND
```

启动前先保证定位、MAVROS 和 PX4 EKF2 local position 正常。然后启动舵机驱动和 offboard 节点：

```bash
ros2 launch mavros_offboard mavros_offboard_servo.launch.py
```

无硬件验证时使用 dry-run：

```bash
ros2 launch mavros_offboard mavros_offboard_servo.launch.py dry_run:=true
```

常用参数：

- `hover_x hover_y hover_z`: 起飞悬停点，相对启动后锁定的任务原点，默认 `0.0 0.0 0.5`。
- `left_x left_y left_z`: 左飞指定点，相对启动后锁定的任务原点，默认 `0.0 -0.5 0.5`。
- `hover_duration_s`: 到达起飞悬停点后等待多久再执行 BIG，默认 `1.0`。
- `reach_tolerance_m`: 到点判定半径，默认 `0.1`。
- `pose_timeout_s`: 位姿超时后暂停推进任务，默认 `1.0`。
- `dry_run`: `true` 时舵机只打印动作，不写 I2C/PWM。

示例：把左飞指定点改成 `0.0 -0.8 0.5`：

```bash
ros2 launch mavros_offboard mavros_offboard_servo.launch.py left_y:=-0.8
```

注意：这个 launch 不会启动 `servo_waypoint_task`。执行本任务时不要再另外启动 `servo_controller servo_standalone.launch.py`，否则 `servo_waypoint_task` 会按坐标独立触发舵机，和 offboard 状态机重复控制。
