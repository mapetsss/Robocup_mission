# 单目 USB 摄像头采集程序使用说明

本文说明如何在 `detect_ws` 中使用 `mono_camera_capture` 图像采集程序。

## 1. 程序作用

`mono_camera_capture` 负责完成一件事：

```text
从 /dev/video0 读取 USB 摄像头画面，并发布成 ROS 2 图像话题。
```

默认发布：

```text
/camera/image_raw
/camera/camera_info
```

后续 YOLO 节点只需要订阅 `/camera/image_raw` 即可获得图像。

## 2. 使用前检查

确认摄像头设备存在：

```bash
ls -l /dev/video0 /dev/video1
```

正常情况下应该能看到类似：

```text
/dev/video0
/dev/video1
```

如果看不到 `/dev/video0`，先不要运行本节点，需要先确认摄像头是否接入主机或容器是否映射了视频设备。

## 3. 编译工作空间

进入工作空间：

```bash
cd ~/detect_ws
```

如果当前终端还没有加载 ROS 2 环境，先执行：

```bash
source /opt/ros/humble/setup.bash
```

编译图像采集包：

```bash
colcon build --packages-select mono_camera_capture
```

编译完成后加载本工作空间：

```bash
source install/setup.bash
```

## 4. 启动图像采集节点

使用默认参数启动：

```bash
ros2 launch mono_camera_capture mono_camera.launch.py
```

默认参数来自：

```text
~/detect_ws/src/mono_camera_capture/config/mono_camera.yaml
```

默认会发布：

```text
/camera/image_raw
/camera/camera_info
```

## 5. 查看话题是否发布

查看话题列表：

```bash
ros2 topic list
```

确认存在：

```text
/camera/image_raw
/camera/camera_info
```

查看图像发布频率：

```bash
ros2 topic hz /camera/image_raw
```

如果参数是 `fps: 30.0`，正常情况下频率应接近 30 Hz。实际值会受机载算力、USB 带宽和系统负载影响。

## 6. 查看图像内容

如果系统安装了 `rqt_image_view`：

```bash
rqt_image_view
```

然后选择：

```text
/camera/image_raw
```

如果没有图形界面，可以只用 `ros2 topic hz` 判断图像是否稳定发布。

如果没有安装 `image_view` 或 `rqt_image_view`，可以使用本工作空间提供的保存脚本：

```bash
cd ~/detect_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/save_image_once.py \
  --topic /camera/image_raw \
  --output /home/banana/detect_ws/camera_frame_%04i.jpg \
  --count 1 \
  --timeout 5
```

保存成功后查看文件：

```bash
ls -lh ~/detect_ws/camera_frame_*.jpg
```

这个脚本只订阅图像话题并保存图片，不需要安装 `ros-humble-image-view`。

## 7. 临时修改启动话题

默认图像话题是 `/camera/image_raw`。如果后续想改成其他话题，可以通过 launch 参数修改：

```bash
ros2 launch mono_camera_capture mono_camera.launch.py \
  image_topic:=/my_camera/image_raw \
  camera_info_topic:=/my_camera/camera_info
```

注意：如果后续 YOLO 仍然订阅 `/camera/image_raw`，这里就不要改。

## 8. 和 ros2_yolos_cpp 接入

本采集节点启动后，再启动 YOLO 检测节点，并让 YOLO 订阅 `/camera/image_raw`：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  start_camera:=false \
  image_topic:=/camera/image_raw \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/labels.txt
```

其中：

```text
start_camera:=false
```

表示不要让 `ros2_yolos_cpp` 再启动自己的相机节点，避免两个程序同时抢占 `/dev/video0`。

## 9. 推荐执行顺序

建议按这个顺序推进：

```text
确认 /dev/video0 存在
  -> 编译 detect_ws
  -> 启动 mono_camera_capture
  -> ros2 topic hz /camera/image_raw
  -> 确认图像频率稳定
  -> 再接入 ros2_yolos_cpp
```

不要一开始就同时启动相机和 YOLO。先把图像采集链路单独跑稳，后面问题会好定位很多。

## 10. 停止程序

在启动节点的终端中按：

```text
Ctrl+C
```

即可停止图像采集。
