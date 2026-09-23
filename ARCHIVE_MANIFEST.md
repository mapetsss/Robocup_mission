# RoboCup 投放与穿环任务归档清单

## 来源

- Git 提交：`eaff6693935a5d0f5e5889d829d463732ff00a06`
- 对应标签：`archive/robocup-eaff669`
- 选择原因：这是 `mavros_offboard` 最后一次直接修改后的提交；后续共享视觉代码已转向 H 题。

## 包和资产

- `mavros_offboard`：飞行、投放、穿环、返航和降落状态机。
- `servo_controller`：PCA9685/ServoKit 三路舵机服务。
- `mono_camera_capture`：当时版本的相机采集包。
- `ros2_yolos_cpp`：当时版本的检测包及 `robocup.world`。
- `models/best.onnx` 与 `models/classes.txt`：旧版单类别 `robocup` 模型。
- `support/`：当时版本的工具和文档。

## 主要入口

```text
ros2 launch mavros_offboard robocup_v1.launch.py
ros2 launch mavros_offboard mavros_offboard_servo.launch.py
ros2 launch servo_controller servo_standalone.launch.py
```

`mavros_offboard` 和 `servo_standalone` 中的坐标触发节点不应在同一任务中重复控制舵机。

## 关键接口

- 发布：`/mavros/setpoint_position/local`
- 订阅：`/mavros/state`、`/mavros/local_position/pose`
- 模式服务：`/mavros/set_mode`
- 舵机服务：`/servo/big_home`、`/servo/left_home`、`/servo/right_home`、`/servo/*_90`

## 模型

- 文件：`ros2_ws/src/models/best.onnx`
- 类别：`robocup`
- SHA-256：`CD6C194894C8643429B373B8F83D3859AEF7F86E3BC4A49734BAB72805A70818`
- 原仓库没有提供完整模型导出清单，训练来源和输入尺寸仍需人工补录。

## 验证状态

- 已确认文件来自指定 Git 提交。
- 尚未执行 ROS 构建、仿真或实机飞行测试。
- 未发现任务级自动化测试；恢复时需依赖现场参数、场地图和人工联调。

