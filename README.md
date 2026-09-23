# RoboCup Drop and Ring ROS 2

RoboCup 无人机投放、穿环和舵机控制任务的独立归档仓库。

## 来源

- 原仓库：`mapetsss/Detect_ws`
- 提取提交：`eaff6693935a5d0f5e5889d829d463732ff00a06`
- 归档原因：该提交是 RoboCup 专属代码最后一次直接修改后的历史状态；后续共享视觉代码已用于其他任务。

## 目录

- `ros2_ws/src/`：ROS 2 包和旧版 RoboCup 模型。
- `support/`：当时版本的相机、标定和测试资料。
- `ARCHIVE_MANIFEST.md`：接口、模型和验证状态清单。
- `SOURCE_README.md`：原提交中的工作空间说明。

构建时进入 `ros2_ws/`，不要与另外两套任务控制器放在同一 ROS graph 中运行。

本仓库尚未在 ROS 2 Humble、PX4/MAVROS 或实机环境重新构建验证。

