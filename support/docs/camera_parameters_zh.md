# 图像采集参数说明

本文说明 `mono_camera_capture` 的参数含义，以及后续调试时应该如何修改。

参数文件位置：

```text
~/detect_ws/src/mono_camera_capture/config/mono_camera.yaml
```

## 1. 默认参数

```yaml
mono_camera_node:
  ros__parameters:
    device_path: "/dev/video0"
    fourcc: "MJPG"
    width: 1280
    height: 720
    fps: 30.0
    buffer_size: 2
    output_encoding: "bgr8"
    frame_id: "camera_link"
    publish_camera_info: true
    log_fps: true
    reconnect_on_failure: true
    reconnect_period_ms: 1000
```

## 2. 参数逐项说明

`device_path`

摄像头设备路径。当前 USB 摄像头默认使用：

```yaml
device_path: "/dev/video0"
```

如果后续插入多个摄像头，设备编号可能变化，需要重新确认 `/dev/video*`。

`fourcc`

摄像头采集格式，常用：

```yaml
fourcc: "MJPG"
```

或者：

```yaml
fourcc: "YUYV"
```

你当前摄像头已确认支持：

```text
MJPG 1280x720 @ 30 fps
YUYV 640x480  @ 30 fps
```

建议优先使用 `MJPG`，因为它在 `1280x720` 下也能到 30 fps。

`width` / `height`

图像宽高。例如：

```yaml
width: 1280
height: 720
```

如果 YOLO 推理速度不够，可以改为：

```yaml
width: 640
height: 480
```

`fps`

采集帧率。例如：

```yaml
fps: 30.0
```

如果机载算力不足，可以先降到：

```yaml
fps: 15.0
```

但需要注意：摄像头不一定支持任意帧率，实际帧率以节点启动日志和 `ros2 topic hz` 为准。

`buffer_size`

OpenCV/V4L2 的采集缓冲数量：

```yaml
buffer_size: 2
```

数值较小可以减少延迟；数值较大可能更稳定，但延迟会增加。视觉控制任务建议保持 1 到 2。

`output_encoding`

发布到 ROS 2 的图像编码。当前支持：

```text
bgr8
rgb8
mono8
```

后续接 `ros2_yolos_cpp` 建议保持：

```yaml
output_encoding: "bgr8"
```

因为检测节点内部按 `bgr8` 转换图像。

`frame_id`

图像消息头里的坐标系名称：

```yaml
frame_id: "camera_link"
```

如果后续做相机外参、TF 或与机体坐标系关联，需要保证这里和 TF 树一致。

`publish_camera_info`

是否发布 `/camera/camera_info`：

```yaml
publish_camera_info: true
```

当前 `camera_info` 中没有真实标定内参，只是占位信息。后续完成相机标定后，应再填入真实内参。

`log_fps`

是否在节点日志中打印发布帧率：

```yaml
log_fps: true
```

调试阶段建议打开，正式运行时可以关闭以减少日志输出。

`reconnect_on_failure`

读取失败时是否尝试重新打开摄像头：

```yaml
reconnect_on_failure: true
```

机载环境建议打开，这样 USB 摄像头短暂异常时节点会尝试恢复。

`reconnect_period_ms`

重连日志和重连尝试的节流周期：

```yaml
reconnect_period_ms: 1000
```

单位是毫秒。

## 3. 推荐配置

画质优先，适合先做识别效果验证：

```yaml
fourcc: "MJPG"
width: 1280
height: 720
fps: 30.0
output_encoding: "bgr8"
```

实时性优先，适合机载算力不足时：

```yaml
fourcc: "YUYV"
width: 640
height: 480
fps: 30.0
output_encoding: "bgr8"
```

低负载调试：

```yaml
fourcc: "MJPG"
width: 640
height: 480
fps: 15.0
output_encoding: "bgr8"
```

## 4. 运行时临时改参数

节点运行后，可以查看参数：

```bash
ros2 param list /mono_camera_node
```

查看当前分辨率参数：

```bash
ros2 param get /mono_camera_node width
ros2 param get /mono_camera_node height
```

临时修改分辨率：

```bash
ros2 param set /mono_camera_node width 640
ros2 param set /mono_camera_node height 480
```

临时修改格式：

```bash
ros2 param set /mono_camera_node fourcc YUYV
```

临时修改帧率：

```bash
ros2 param set /mono_camera_node fps 15.0
```

修改 `device_path`、`fourcc`、`width`、`height`、`fps`、`buffer_size` 后，节点会尝试重新打开摄像头。

## 5. 调试判断标准

启动后重点看三件事：

```text
1. 节点是否成功打开 /dev/video0
2. 日志中的 effective 分辨率、格式、帧率是否符合预期
3. ros2 topic hz /camera/image_raw 是否稳定
```

如果 `ros2 topic hz` 明显低于设定值，先降低分辨率，再降低帧率，最后再考虑换模型或优化 YOLO 推理。
