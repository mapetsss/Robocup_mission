# detect_ws 使用说明

`detect_ws` 是后续视觉识别任务使用的 ROS 2 工作空间。目前其中包含一个
`mono_camera_capture` 包，用于从单目 USB 摄像头采集图像，并发布给后续
YOLO 检测节点使用。

本工作空间的设计原则：

- 只读取 `/dev/video*` 摄像头设备，不修改摄像头驱动、udev、系统硬件配置。
- 不修改网络接口，不修改 netplan、NetworkManager 或其他系统网络配置。
- 不依赖 `fastlio_ws` 的源码，不改动其他工作空间。
- 图像采集参数放在 YAML 文件中，方便后续调试。

## 目录结构

```text
detect_ws/
  README.md
  docs/
    camera_usage_zh.md
    camera_parameters_zh.md
  src/
    mono_camera_capture/
      config/
        mono_camera.yaml
      launch/
        mono_camera.launch.py
      src/
        mono_camera_node.cpp
        mono_camera_node_main.cpp
      include/
        mono_camera_capture/
          mono_camera_node.hpp
      README.md
      CMakeLists.txt
      package.xml
```

## 当前图像采集程序

包名：

```text
mono_camera_capture
```

节点名：

```text
mono_camera_node
```

默认发布话题：

```text
/camera/image_raw
/camera/camera_info
```

默认摄像头参数：

```text
设备：/dev/video0
图像格式：MJPG
分辨率：1280x720
帧率：30 fps
ROS 图像编码：bgr8
```

## 推荐阅读顺序

1. 先看 [docs/camera_usage_zh.md](docs/camera_usage_zh.md)，了解如何编译、启动、查看图像话题。
2. 再看 [docs/camera_parameters_zh.md](docs/camera_parameters_zh.md)，了解如何调整分辨率、帧率、图像格式。
3. 最后看 [src/mono_camera_capture/README.md](src/mono_camera_capture/README.md)，确认和 `ros2_yolos_cpp` 的接入方式。

## 后续接入 YOLO 的核心逻辑

图像采集链路：

```text
USB 单目摄像头
  -> mono_camera_capture
  -> /camera/image_raw
  -> ros2_yolos_cpp 检测节点
  -> 检测框 / 偏移量 / 距离估计
```

后续启动 `ros2_yolos_cpp` 时，应关闭它自己的自动相机启动，让它订阅本工作空间发布的图像：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  start_camera:=false \
  image_topic:=/camera/image_raw \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/labels.txt
```

## 重要提醒

当前文档只说明如何使用 `detect_ws`。执行前请先审查参数文件：

```text
detect_ws/src/mono_camera_capture/config/mono_camera.yaml
```

如果你还没有确认模型路径、类别文件和 YOLO 包是否已经放入对应工作空间，
先不要启动检测节点，先只验证 `/camera/image_raw` 是否稳定发布。
