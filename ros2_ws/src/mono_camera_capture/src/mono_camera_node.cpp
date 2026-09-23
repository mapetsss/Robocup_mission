#include "mono_camera_capture/mono_camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace mono_camera_capture
{
namespace
{

constexpr int kFourccLength = 4;

int makeFourcc(const std::string & fourcc)
{
  return cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
}

}  // namespace

MonoCameraNode::MonoCameraNode(const rclcpp::NodeOptions & options)
: Node("mono_camera_node", options)
{
  declareParameters();
  config_ = loadConfigFromParameters();

  const auto qos = rclcpp::SensorDataQoS();
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);

  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    openCameraLocked();
  }

  const auto period_ms = static_cast<int>(
    std::max(1.0, std::round(1000.0 / std::max(config_.fps, 1.0))));
  capture_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&MonoCameraNode::captureOnce, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&MonoCameraNode::onParameters, this, std::placeholders::_1));
}

MonoCameraNode::~MonoCameraNode()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  closeCameraLocked();
}

void MonoCameraNode::declareParameters()
{
  declare_parameter<std::string>("device_path", "/dev/video0");
  declare_parameter<std::string>("fourcc", "MJPG");
  declare_parameter<int>("width", 1280);
  declare_parameter<int>("height", 720);
  declare_parameter<double>("fps", 30.0);
  declare_parameter<int>("buffer_size", 2);
  declare_parameter<std::string>("output_encoding", "bgr8");
  declare_parameter<std::string>("frame_id", "camera_link");
  declare_parameter<bool>("publish_camera_info", true);
  declare_parameter<bool>("log_fps", true);
  declare_parameter<bool>("reconnect_on_failure", true);
  declare_parameter<int>("reconnect_period_ms", 1000);
}

MonoCameraNode::CameraConfig MonoCameraNode::loadConfigFromParameters() const
{
  CameraConfig config;
  config.device_path = get_parameter("device_path").as_string();
  config.fourcc = get_parameter("fourcc").as_string();
  config.width = static_cast<int>(get_parameter("width").as_int());
  config.height = static_cast<int>(get_parameter("height").as_int());
  config.fps = get_parameter("fps").as_double();
  config.buffer_size = static_cast<int>(get_parameter("buffer_size").as_int());
  config.output_encoding = get_parameter("output_encoding").as_string();
  config.frame_id = get_parameter("frame_id").as_string();
  config.publish_camera_info = get_parameter("publish_camera_info").as_bool();
  config.log_fps = get_parameter("log_fps").as_bool();
  config.reconnect_on_failure = get_parameter("reconnect_on_failure").as_bool();
  config.reconnect_period_ms = static_cast<int>(get_parameter("reconnect_period_ms").as_int());
  return config;
}

bool MonoCameraNode::openCameraLocked()
{
  closeCameraLocked();

  if (!isValidFourcc(config_.fourcc)) {
    RCLCPP_ERROR(get_logger(), "Invalid fourcc '%s'. It must be a 4-character code.", config_.fourcc.c_str());
    return false;
  }

  capture_.open(config_.device_path, cv::CAP_V4L2);
  if (!capture_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Failed to open camera device: %s", config_.device_path.c_str());
    return false;
  }

  capture_.set(cv::CAP_PROP_FOURCC, makeFourcc(config_.fourcc));
  capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  capture_.set(cv::CAP_PROP_FPS, config_.fps);
  capture_.set(cv::CAP_PROP_BUFFERSIZE, config_.buffer_size);

  logEffectiveCameraSettingsLocked();
  return true;
}

void MonoCameraNode::closeCameraLocked()
{
  if (capture_.isOpened()) {
    capture_.release();
  }
}

void MonoCameraNode::captureOnce()
{
  cv::Mat frame;
  CameraConfig config_snapshot;
  const auto stamp = now();

  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    config_snapshot = config_;
    if (!capture_.isOpened()) {
      if (!config_snapshot.reconnect_on_failure || reopening_.exchange(true)) {
        return;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), config_snapshot.reconnect_period_ms,
        "Camera is not open, trying to reconnect to %s", config_snapshot.device_path.c_str());
      openCameraLocked();
      reopening_ = false;
      return;
    }

    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), config_snapshot.reconnect_period_ms,
        "Failed to read frame from %s", config_snapshot.device_path.c_str());
      if (config_snapshot.reconnect_on_failure) {
        closeCameraLocked();
      }
      return;
    }
  }

  if (publishFrame(frame, stamp, config_snapshot)) {
    updateFpsStats(stamp, config_snapshot.log_fps);
  }
}

bool MonoCameraNode::publishFrame(
  const cv::Mat & frame,
  const rclcpp::Time & stamp,
  const CameraConfig & config)
{
  cv::Mat output_frame;
  try {
    output_frame = convertFrameForEncoding(frame, config.output_encoding);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Frame conversion failed: %s", e.what());
    return false;
  }

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = config.frame_id;

  auto image_msg = cv_bridge::CvImage(header, config.output_encoding, output_frame).toImageMsg();
  image_pub_->publish(*image_msg);

  if (config.publish_camera_info) {
    camera_info_pub_->publish(makeCameraInfo(stamp, config));
  }

  return true;
}

sensor_msgs::msg::CameraInfo MonoCameraNode::makeCameraInfo(
  const rclcpp::Time & stamp,
  const CameraConfig & config) const
{
  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = config.frame_id;
  info.width = static_cast<uint32_t>(config.width);
  info.height = static_cast<uint32_t>(config.height);
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0};
  info.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  info.p = {
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return info;
}

cv::Mat MonoCameraNode::convertFrameForEncoding(
  const cv::Mat & frame,
  const std::string & encoding) const
{
  if (!isSupportedOutputEncoding(encoding)) {
    throw std::runtime_error("unsupported output_encoding: " + encoding);
  }

  if (encoding == sensor_msgs::image_encodings::BGR8) {
    return frame;
  }

  cv::Mat converted;
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    cv::cvtColor(frame, converted, cv::COLOR_BGR2RGB);
    return converted;
  }

  if (encoding == sensor_msgs::image_encodings::MONO8) {
    cv::cvtColor(frame, converted, cv::COLOR_BGR2GRAY);
    return converted;
  }

  return frame;
}

void MonoCameraNode::logEffectiveCameraSettingsLocked() const
{
  const int fourcc_value = static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC));
  std::string effective_fourcc;
  effective_fourcc.push_back(static_cast<char>(fourcc_value & 0xFF));
  effective_fourcc.push_back(static_cast<char>((fourcc_value >> 8) & 0xFF));
  effective_fourcc.push_back(static_cast<char>((fourcc_value >> 16) & 0xFF));
  effective_fourcc.push_back(static_cast<char>((fourcc_value >> 24) & 0xFF));

  RCLCPP_INFO(
    get_logger(),
    "Opened %s: requested %dx%d %.2f fps %s, effective %.0fx%.0f %.2f fps %s, output %s",
    config_.device_path.c_str(),
    config_.width,
    config_.height,
    config_.fps,
    config_.fourcc.c_str(),
    capture_.get(cv::CAP_PROP_FRAME_WIDTH),
    capture_.get(cv::CAP_PROP_FRAME_HEIGHT),
    capture_.get(cv::CAP_PROP_FPS),
    effective_fourcc.c_str(),
    config_.output_encoding.c_str());
}

void MonoCameraNode::updateFpsStats(const rclcpp::Time & stamp, bool log_fps)
{
  if (!log_fps) {
    return;
  }

  if (fps_window_start_.nanoseconds() == 0) {
    fps_window_start_ = stamp;
    fps_frame_count_ = 0;
  }

  ++fps_frame_count_;
  const double elapsed = (stamp - fps_window_start_).seconds();
  if (elapsed >= 1.0) {
    RCLCPP_INFO(get_logger(), "Publish FPS: %.2f", static_cast<double>(fps_frame_count_) / elapsed);
    fps_window_start_ = stamp;
    fps_frame_count_ = 0;
  }
}

bool MonoCameraNode::isSupportedOutputEncoding(const std::string & encoding) const
{
  return encoding == sensor_msgs::image_encodings::BGR8 ||
         encoding == sensor_msgs::image_encodings::RGB8 ||
         encoding == sensor_msgs::image_encodings::MONO8;
}

bool MonoCameraNode::isValidFourcc(const std::string & fourcc) const
{
  return fourcc.size() == kFourccLength;
}

rcl_interfaces::msg::SetParametersResult MonoCameraNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto next_config = config_;
  bool reopen_required = false;
  bool timer_required = false;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "device_path") {
      next_config.device_path = parameter.as_string();
      reopen_required = true;
    } else if (name == "fourcc") {
      next_config.fourcc = parameter.as_string();
      reopen_required = true;
    } else if (name == "width") {
      next_config.width = static_cast<int>(parameter.as_int());
      reopen_required = true;
    } else if (name == "height") {
      next_config.height = static_cast<int>(parameter.as_int());
      reopen_required = true;
    } else if (name == "fps") {
      next_config.fps = parameter.as_double();
      reopen_required = true;
      timer_required = true;
    } else if (name == "buffer_size") {
      next_config.buffer_size = static_cast<int>(parameter.as_int());
      reopen_required = true;
    } else if (name == "output_encoding") {
      next_config.output_encoding = parameter.as_string();
    } else if (name == "frame_id") {
      next_config.frame_id = parameter.as_string();
    } else if (name == "publish_camera_info") {
      next_config.publish_camera_info = parameter.as_bool();
    } else if (name == "log_fps") {
      next_config.log_fps = parameter.as_bool();
    } else if (name == "reconnect_on_failure") {
      next_config.reconnect_on_failure = parameter.as_bool();
    } else if (name == "reconnect_period_ms") {
      next_config.reconnect_period_ms = static_cast<int>(parameter.as_int());
    }
  }

  if (next_config.width <= 0 || next_config.height <= 0) {
    result.successful = false;
    result.reason = "width and height must be positive";
    return result;
  }

  if (next_config.fps <= 0.0) {
    result.successful = false;
    result.reason = "fps must be positive";
    return result;
  }

  if (next_config.buffer_size < 1) {
    result.successful = false;
    result.reason = "buffer_size must be >= 1";
    return result;
  }

  if (next_config.reconnect_period_ms < 100) {
    result.successful = false;
    result.reason = "reconnect_period_ms must be >= 100";
    return result;
  }

  if (!isValidFourcc(next_config.fourcc)) {
    result.successful = false;
    result.reason = "fourcc must be a 4-character code such as MJPG or YUYV";
    return result;
  }

  if (!isSupportedOutputEncoding(next_config.output_encoding)) {
    result.successful = false;
    result.reason = "output_encoding must be bgr8, rgb8, or mono8";
    return result;
  }

  if (timer_required && capture_timer_) {
    const auto period_ms = static_cast<int>(
      std::max(1.0, std::round(1000.0 / std::max(next_config.fps, 1.0))));
    capture_timer_->cancel();
    capture_timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&MonoCameraNode::captureOnce, this));
  }

  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    config_ = next_config;
    fps_window_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    fps_frame_count_ = 0;

    if (reopen_required && !openCameraLocked()) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter update accepted, but camera did not reopen. Check device and format support.");
    }
  }

  return result;
}

}  // namespace mono_camera_capture
