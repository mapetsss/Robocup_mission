# mono_camera_capture 包说明

`mono_camera_capture` 是 `detect_ws` 中的单目 USB 摄像头采集包。

它的职责很单一：

```text
打开 USB 摄像头
  -> 设置采集格式、分辨率、帧率
  -> 读取图像帧
  -> 发布 sensor_msgs/msg/Image
  -> 供后续 YOLO 节点订阅
```

本包不会修改系统硬件配置、驱动配置、udev 规则、网络接口或其他工作空间。

## 节点信息

节点名：

```text
mono_camera_node
```

可执行文件：

```text
mono_camera_node
```

默认发布话题：

```text
/camera/image_raw
/camera/camera_info
```

默认图像编码：

```text
bgr8
```

## 参数文件

默认参数文件：

```text
config/mono_camera.yaml
```

主要参数：

```yaml
device_path: "/dev/video0"
fourcc: "MJPG"
width: 1280
height: 720
fps: 30.0
output_encoding: "bgr8"
```

后续调试摄像头时，优先修改这个 YAML 文件，不需要改 C++ 源码。

## 编译

在工作空间根目录执行：

```bash
cd ~/detect_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mono_camera_capture
source install/setup.bash
```

## 启动

```bash
ros2 launch mono_camera_capture mono_camera.launch.py
```

启动后查看图像话题频率：

```bash
ros2 topic hz /camera/image_raw
```

## 修改启动话题

如果需要修改发布话题：

```bash
ros2 launch mono_camera_capture mono_camera.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

默认已经是 `/camera/image_raw`，通常不需要改。

## 后续接入 ros2_yolos_cpp

后续接入 `ros2_yolos_cpp` 时，不要让 YOLO 包再启动相机，避免抢占 `/dev/video0`：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  start_camera:=false \
  image_topic:=/camera/image_raw \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/labels.txt
```

## 调试建议

先单独启动本采集节点，确认：

```text
/camera/image_raw 存在
ros2 topic hz /camera/image_raw 接近设定帧率
图像没有明显卡顿或丢帧
```

确认采集稳定后，再启动 YOLO 检测节点。
