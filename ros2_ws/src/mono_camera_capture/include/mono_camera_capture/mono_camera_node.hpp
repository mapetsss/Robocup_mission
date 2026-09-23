#ifndef MONO_CAMERA_CAPTURE__MONO_CAMERA_NODE_HPP_
#define MONO_CAMERA_CAPTURE__MONO_CAMERA_NODE_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace mono_camera_capture
{

class MonoCameraNode : public rclcpp::Node
{
public:
  explicit MonoCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MonoCameraNode() override;

private:
  struct CameraConfig
  {
    std::string device_path{"/dev/video0"};
    std::string fourcc{"MJPG"};
    std::string output_encoding{"bgr8"};
    std::string frame_id{"camera_link"};
    int width{1280};
    int height{720};
    double fps{30.0};
    int buffer_size{2};
    bool publish_camera_info{true};
    bool log_fps{true};
    bool reconnect_on_failure{true};
    int reconnect_period_ms{1000};
  };

  void declareParameters();
  CameraConfig loadConfigFromParameters() const;
  bool openCameraLocked();
  void closeCameraLocked();
  void captureOnce();
  bool publishFrame(
    const cv::Mat & frame,
    const rclcpp::Time & stamp,
    const CameraConfig & config);
  sensor_msgs::msg::CameraInfo makeCameraInfo(
    const rclcpp::Time & stamp,
    const CameraConfig & config) const;
  cv::Mat convertFrameForEncoding(const cv::Mat & frame, const std::string & encoding) const;
  void logEffectiveCameraSettingsLocked() const;
  void updateFpsStats(const rclcpp::Time & stamp, bool log_fps);
  bool isSupportedOutputEncoding(const std::string & encoding) const;
  bool isValidFourcc(const std::string & fourcc) const;

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  CameraConfig config_;
  mutable std::mutex camera_mutex_;
  cv::VideoCapture capture_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::atomic<bool> reopening_{false};
  rclcpp::Time fps_window_start_;
  int fps_frame_count_{0};
};

}  // namespace mono_camera_capture

#endif  // MONO_CAMERA_CAPTURE__MONO_CAMERA_NODE_HPP_
