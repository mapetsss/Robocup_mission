//位置控制
//飞行高度和投放高度默认由 config/robocup_v1_params.yaml 控制
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <sstream>
#include <vector>
#include <cctype>
#include <utility>

#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

// 任务状态机
enum class MissionState {

    WAITING,           // 等待连接与位置
    STREAMING,         // 持续发送期望值
    WAIT_ARM,          // 切 OFFBOARD 后等待手动解锁
    TAKEOFF,          // 起飞到任务飞行高度
    HOVERING1,        // 到达后悬停

    MOVE_TO_QRCODE,      // 先飞往二维码
    HOVERING2,           // 到达后悬停

    MOVE_TO_POINT2,     // 飞往特殊靶点
    DROP1,              //下降至投放高度 xy不变
    PUT_1,              // 特殊靶点舵机投放
    RISE1,              //上升回飞行高度 xy不变

    MOVE_TO_PICTURE1,   // 飞往第二个靶点
    DROP2,              //下降至投放高度 xy不变
    PUT_2,              // 第二个靶点舵机投放
    RISE2,              //上升回飞行高度 xy不变

    MOVE_TO_PICTURE2,   // 飞往第一个靶点
    DROP3,              //下降至投放高度 xy不变
    PUT_3,              // 第一个靶点舵机投放
    RISE3,              //上升回飞行高度 xy不变

    MOVE_TO_DETECT,     // 飞到输入圆环中心点推导出的入口点
    PASSING_RING,       // 穿环

    RETURN_HOME,        // 回到 left 降落点
    FINAL_HOVER,        // 回到降落点后的悬停
    LANDING,            // 降落
    FINISHED            // 任务结束
};

int home_num; //1为左 2为右

class MavrosOffboard : public rclcpp::Node
{
public:
    MavrosOffboard() : Node("mavros_offboard")
    {
        // QoS 匹配 mavros
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // 订阅
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&MavrosOffboard::state_cb, this, std::placeholders::_1));

        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos,
            std::bind(&MavrosOffboard::local_pose_cb, this, std::placeholders::_1));

        // 发布
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        // 服务客户端
        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");


        // 主定时器 20Hz
        timer_ = this->create_wall_timer(50ms, std::bind(&MavrosOffboard::timer_cb, this));

        last_request_ = this->now();
        const auto param3 = static_cast<int>(this->declare_parameter<int>("param3", 1));
        qrcode_result_ = this->declare_parameter<std::string>("qrcode_result", qrcode_result_);
        qrcode_hover_s_ = std::max(this->declare_parameter<double>("qrcode_hover_s", qrcode_hover_s_), 0.0);
        field_forward_mavros_x_ = this->declare_parameter<double>("field_forward_mavros_x", field_forward_mavros_x_);
        field_forward_mavros_y_ = this->declare_parameter<double>("field_forward_mavros_y", field_forward_mavros_y_);
        field_left_mavros_x_ = this->declare_parameter<double>("field_left_mavros_x", field_left_mavros_x_);
        field_left_mavros_y_ = this->declare_parameter<double>("field_left_mavros_y", field_left_mavros_y_);
        ring_center_x_ = this->declare_parameter<double>("ring_center_x", ring_center_x_);
        ring_center_y_ = this->declare_parameter<double>("ring_center_y", ring_center_y_);
        ring_center_z_ = this->declare_parameter<double>("ring_center_z", ring_center_z_);
        ring_entry_offset_y_ = std::max(this->declare_parameter<double>("ring_entry_offset_y", ring_entry_offset_y_), 0.0);
        ring_exit_offset_y_ = std::max(this->declare_parameter<double>("ring_exit_offset_y", ring_exit_offset_y_), 0.0);
        const auto ring_direction_param = this->declare_parameter<int>("ring_pass_direction_y", ring_pass_direction_y_);
        big_target_x_ = this->declare_parameter<double>("big_target_x", big_target_x_);
        big_target_y_ = this->declare_parameter<double>("big_target_y", big_target_y_);
        left_target_x_ = this->declare_parameter<double>("left_target_x", left_target_x_);
        left_target_y_ = this->declare_parameter<double>("left_target_y", left_target_y_);
        right_target_x_ = this->declare_parameter<double>("right_target_x", right_target_x_);
        right_target_y_ = this->declare_parameter<double>("right_target_y", right_target_y_);
        target_fly_z_ = this->declare_parameter<double>("target_fly_z", target_fly_z_);
        target_drop_z_ = this->declare_parameter<double>("target_drop_z", target_drop_z_);
        drop_duration_s_ = std::max(this->declare_parameter<double>("drop_duration_s", drop_duration_s_), 0.1);
        rise_duration_s_ = std::max(this->declare_parameter<double>("rise_duration_s", rise_duration_s_), 0.1);
        pre_servo_hold_s_ = std::max(this->declare_parameter<double>("pre_servo_hold_s", pre_servo_hold_s_), 0.0);
        servo_hold_s_ = std::max(this->declare_parameter<double>("servo_hold_s", servo_hold_s_), 0.0);
        const auto big_service = this->declare_parameter<std::string>("big_service", "/servo/big_home");
        const auto left_service = this->declare_parameter<std::string>("left_service", "/servo/left_home");
        const auto right_service = this->declare_parameter<std::string>("right_service", "/servo/right_home");
        servo_clients_["big"] = this->create_client<std_srvs::srv::Trigger>(big_service);
        servo_clients_["left"] = this->create_client<std_srvs::srv::Trigger>(left_service);
        servo_clients_["right"] = this->create_client<std_srvs::srv::Trigger>(right_service);
        for (const auto& item : servo_clients_) {
            servo_done_[item.first] = false;
            servo_pending_[item.first] = false;
        }
        home_num = std::clamp(param3, 1, 2);
        ring_pass_direction_y_ = ring_direction_param >= 0 ? 1 : -1;
        pose_timeout_s_ = std::max(this->declare_parameter<double>("pose_timeout_s", 1.0), 0.2);
        RCLCPP_INFO(
            this->get_logger(),
            "任务参数: home=%d pose_timeout=%.1fs qrcode=\"%s\" qrcode_hover=%.1fs ring_field=(%.2f,%.2f,%.2f) entry_left_offset=%.2f exit_left_offset=%.2f direction_left=%d",
            home_num, pose_timeout_s_,
            qrcode_result_.c_str(), qrcode_hover_s_,
            ring_center_x_, ring_center_y_, ring_center_z_,
            ring_entry_offset_y_, ring_exit_offset_y_, ring_pass_direction_y_);
        RCLCPP_INFO(
            this->get_logger(),
            "场地坐标轴: F->mavros=(%.3f,%.3f), L->mavros=(%.3f,%.3f). 后续 x/y 参数解释为遥控器坐标 F/L。",
            field_forward_mavros_x_, field_forward_mavros_y_,
            field_left_mavros_x_, field_left_mavros_y_);
        RCLCPP_INFO(
            this->get_logger(),
            "舵机投放场地坐标: big=(F%.2f,L%.2f), left=(F%.2f,L%.2f), right=(F%.2f,L%.2f), fly_z=%.2f drop_z=%.2f drop_time=%.1fs pre_hold=%.1fs rise_time=%.1fs hold=%.1fs",
            big_target_x_, big_target_y_, left_target_x_, left_target_y_, right_target_x_, right_target_y_,
            target_fly_z_, target_drop_z_, drop_duration_s_, pre_servo_hold_s_, rise_duration_s_, servo_hold_s_);
        RCLCPP_INFO(this->get_logger(), "✅ offboard节点已启动");
    }

private:
    // 状态机
    MissionState mission_state_ = MissionState::WAITING;

    // 飞控状态
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_received_ = false;
    geometry_msgs::msg::Quaternion mission_orientation_;
    bool mission_orientation_locked_ = false;
    bool mission_origin_locked_ = false;
    double mission_origin_x_ = 0.0;
    double mission_origin_y_ = 0.0;
    double mission_origin_z_ = 0.0;

    // 悬停计时
    rclcpp::Time hover_start_time_;
    rclcpp::Time last_request_;
    rclcpp::Time last_pose_time_;
    double pose_timeout_s_ = 1.0;

    // 安全停止
    bool stop_auto_ = false;

    std::pair<double, double> field_to_mavros_xy(double forward_m, double left_m) const
    {
        return {
            field_forward_mavros_x_ * forward_m + field_left_mavros_x_ * left_m,
            field_forward_mavros_y_ * forward_m + field_left_mavros_y_ * left_m
        };
    }

    bool set_target_pose(geometry_msgs::msg::PoseStamped& target_pose, double forward_m, double left_m, double z)
    {
        if (!mission_origin_locked_) {
            return false;
        }
        const auto [mavros_x, mavros_y] = field_to_mavros_xy(forward_m, left_m);
        target_pose.pose.position.x = mission_origin_x_ + mavros_x;
        target_pose.pose.position.y = mission_origin_y_ + mavros_y;
        target_pose.pose.position.z = mission_origin_z_ + z;
        return true;
    }

    void lock_mission_origin()
    {
        if (mission_origin_locked_) {
            return;
        }
        mission_origin_x_ = current_pose_.pose.position.x;
        mission_origin_y_ = current_pose_.pose.position.y;
        mission_origin_z_ = current_pose_.pose.position.z;
        mission_origin_locked_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "任务原点已锁定: map=(%.3f, %.3f, %.3f). 目标点先从场地 F/L 坐标转换到 MAVROS local，再相对该原点发布。",
            mission_origin_x_, mission_origin_y_, mission_origin_z_);
    }

    double ring_entry_y() const
    {
        return ring_center_y_ - static_cast<double>(ring_pass_direction_y_) * ring_entry_offset_y_;
    }

    double ring_exit_y() const
    {
        return ring_center_y_ + static_cast<double>(ring_pass_direction_y_) * ring_exit_offset_y_;
    }

    // 订阅回调
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        if(current_state_.mode == "STABILIZED")
            stop_auto_ = true;
    }

    void local_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        last_pose_time_ = this->now();
        pose_received_ = true;
    }

    geometry_msgs::msg::Quaternion target_orientation() const
    {
        return mission_orientation_locked_ ? mission_orientation_ : current_pose_.pose.orientation;
    }

    // 判断是否到达目标点（0.1米误差）
    bool is_reached(double target_forward_m, double target_left_m, double tz)
    {
        if (!mission_origin_locked_) {
            return false;
        }
        const auto [mavros_x, mavros_y] = field_to_mavros_xy(target_forward_m, target_left_m);
        double dx = current_pose_.pose.position.x - (mission_origin_x_ + mavros_x);
        double dy = current_pose_.pose.position.y - (mission_origin_y_ + mavros_y);
        double dz = current_pose_.pose.position.z - (mission_origin_z_ + tz);
        return sqrt(dx*dx + dy*dy + dz*dz) < 0.1;
    }

    bool request_mode(const std::string& mode)
    {
        if (!mode_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "等待 /mavros/set_mode 服务");
            return false;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->base_mode = 0;
        request->custom_mode = mode;
        mode_client_->async_send_request(
            request,
            [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                const auto response = future.get();
                if (response->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "模式请求已发送: %s", mode.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "模式请求被拒绝: %s", mode.c_str());
                }
            });
        return true;
    }

    bool tick_servo(const std::string& name)
    {
        if (servo_done_[name]) {
            return true;
        }
        if (servo_pending_[name]) {
            return false;
        }

        auto client_iter = servo_clients_.find(name);
        if (client_iter == servo_clients_.end()) {
            RCLCPP_ERROR(this->get_logger(), "未配置 %s 舵机服务客户端", name.c_str());
            return false;
        }

        auto client = client_iter->second;
        if (!client->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "%s 舵机服务未就绪: %s",
                name.c_str(), client->get_service_name());
            return false;
        }

        servo_pending_[name] = true;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client->async_send_request(
            request,
            [this, name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                servo_pending_[name] = false;
                try {
                    const auto response = future.get();
                    if (response->success) {
                        servo_done_[name] = true;
                        servo_done_time_[name] = this->now();
                        RCLCPP_INFO(
                            this->get_logger(),
                            "%s 舵机动作完成: %s",
                            name.c_str(), response->message.c_str());
                    } else {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "%s 舵机动作失败: %s",
                            name.c_str(), response->message.c_str());
                    }
                } catch (const std::exception& exc) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "%s 舵机服务调用异常: %s",
                        name.c_str(), exc.what());
                }
            });
        RCLCPP_INFO(this->get_logger(), "已调用 %s 舵机服务: %s", name.c_str(), client->get_service_name());
        return false;
    }

    bool servo_hold_done(const std::string& name) const
    {
        const auto done_iter = servo_done_.find(name);
        const auto time_iter = servo_done_time_.find(name);
        if (done_iter == servo_done_.end() || time_iter == servo_done_time_.end()) {
            return false;
        }
        if (!done_iter->second) {
            return false;
        }
        return (this->now() - time_iter->second).seconds() >= servo_hold_s_;
    }

    static std::string trim_copy(const std::string& input)
    {
        const auto begin = std::find_if_not(input.begin(), input.end(), [](unsigned char ch) {
            return std::isspace(ch);
        });
        const auto end = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char ch) {
            return std::isspace(ch);
        }).base();
        if (begin >= end) {
            return "";
        }
        return std::string(begin, end);
    }

    static std::string lower_copy(std::string input)
    {
        std::transform(input.begin(), input.end(), input.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        return input;
    }

    std::vector<std::string> split_csv(const std::string& input) const
    {
        std::vector<std::string> parts;
        std::stringstream stream(input);
        std::string item;
        while (std::getline(stream, item, ',')) {
            parts.push_back(trim_copy(item));
        }
        return parts;
    }

    void apply_qrcode_result()
    {
        if (qrcode_result_applied_) {
            return;
        }

        const auto parts = split_csv(qrcode_result_);
        if (parts.size() >= 1) {
            qrcode_item_1_ = parts[0];
        }
        if (parts.size() >= 2) {
            qrcode_item_2_ = parts[1];
        }
        if (parts.size() >= 3) {
            qrcode_home_side_ = lower_copy(parts[2]);
        }

        if (qrcode_home_side_ == "left") {
            home_num = 1;
        } else if (qrcode_home_side_ == "right") {
            home_num = 2;
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "二维码返航方向无效: \"%s\"，保持当前 home_num=%d",
                qrcode_home_side_.c_str(), home_num);
        }

        qrcode_result_applied_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "二维码回传数据: %s -> item1=%s item2=%s home=%s home_num=%d",
            qrcode_result_.c_str(), qrcode_item_1_.c_str(), qrcode_item_2_.c_str(),
            qrcode_home_side_.c_str(), home_num);
    }

    void start_vertical_motion(double target_z, double duration_s)
    {
        vertical_motion_start_time_ = this->now();
        vertical_motion_start_z_ = current_pose_.pose.position.z - mission_origin_z_;
        vertical_motion_target_z_ = target_z;
        vertical_motion_duration_s_ = std::max(duration_s, 0.1);
    }

    double vertical_motion_z() const
    {
        const double elapsed_s = (this->now() - vertical_motion_start_time_).seconds();
        const double ratio = std::clamp(elapsed_s / vertical_motion_duration_s_, 0.0, 1.0);
        return vertical_motion_start_z_ + (vertical_motion_target_z_ - vertical_motion_start_z_) * ratio;
    }

    bool vertical_motion_time_done() const
    {
        return (this->now() - vertical_motion_start_time_).seconds() >= vertical_motion_duration_s_;
    }

    void run_move_to_target(
        geometry_msgs::msg::PoseStamped& target_pose,
        bool& target_set,
        const char* state_name,
        const char* target_name,
        double x,
        double y,
        MissionState drop_state)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[状态] %s → 飞往 %s 靶点 (%.2f,%.2f,%.2f)",
            state_name, target_name, x, y, target_fly_z_);
        target_set = set_target_pose(target_pose, x, y, target_fly_z_);

        if (is_reached(x, y, target_fly_z_)) {
            RCLCPP_INFO(this->get_logger(), "[状态] %s → 到达 %s 靶点，开始下降", state_name, target_name);
            start_vertical_motion(target_drop_z_, drop_duration_s_);
            mission_state_ = drop_state;
        }
    }

    void run_drop_target(
        geometry_msgs::msg::PoseStamped& target_pose,
        bool& target_set,
        const char* state_name,
        const char* target_name,
        double x,
        double y,
        MissionState put_state)
    {
        const double z = vertical_motion_z();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[状态] %s → %s %.1fs 下降至投放高度 %.2fm",
            state_name, target_name, drop_duration_s_, target_drop_z_);
        target_set = set_target_pose(target_pose, x, y, z);

        if (vertical_motion_time_done() && is_reached(x, y, target_drop_z_)) {
            RCLCPP_INFO(this->get_logger(), "[状态] %s → 到达 %s 投放高度，调用 %s 舵机", state_name, target_name, target_name);
            mission_state_ = put_state;
            hover_start_time_ = this->now();
        }
    }

    void run_put_target(
        geometry_msgs::msg::PoseStamped& target_pose,
        bool& target_set,
        const char* state_name,
        const char* target_name,
        const char* servo_name,
        double x,
        double y,
        MissionState rise_state,
        const char* next_action)
    {
        target_set = set_target_pose(target_pose, x, y, target_drop_z_);

        const double pre_hold_elapsed_s = (this->now() - hover_start_time_).seconds();
        if (pre_hold_elapsed_s < pre_servo_hold_s_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "[状态] %s → %s 投放高度稳定中 %.1f/%.1fs",
                state_name, target_name, pre_hold_elapsed_s, pre_servo_hold_s_);
            return;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[状态] %s → %s 舵机投放中",
            state_name, target_name);

        if (tick_servo(servo_name) && servo_hold_done(servo_name)) {
            RCLCPP_INFO(this->get_logger(), "[状态] %s → %s 投放完成，%s", state_name, target_name, next_action);
            start_vertical_motion(target_fly_z_, rise_duration_s_);
            mission_state_ = rise_state;
        }
    }

    void run_rise_target(
        geometry_msgs::msg::PoseStamped& target_pose,
        bool& target_set,
        const char* state_name,
        const char* target_name,
        double x,
        double y,
        MissionState next_state,
        const char* next_action)
    {
        const double z = vertical_motion_z();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[状态] %s → %.1fs 上升回安全高度 %.2fm",
            state_name, rise_duration_s_, target_fly_z_);
        target_set = set_target_pose(target_pose, x, y, z);

        if (vertical_motion_time_done() && is_reached(x, y, target_fly_z_)) {
            RCLCPP_INFO(this->get_logger(), "[状态] %s → %s 投放完成，%s", state_name, target_name, next_action);
            mission_state_ = next_state;
        }
    }

    bool qrcode_result_applied_ = false;
    std::string qrcode_result_ = "apple,motorcycle,left";
    std::string qrcode_item_1_ = "apple";
    std::string qrcode_item_2_ = "motorcycle";
    std::string qrcode_home_side_ = "left";
    double qrcode_hover_s_ = 1.0;

    // ===== 坐标转换，现场快捷修改区 =====
    // x/y 参数统一解释为遥控器/场地坐标：x=前方 F，y=左方 L。
    // 以下四个数来自 2026-06-05 现场手持实测，只修正 offboard 节点任务坐标，不改 FAST-LIO/PX4。
    double field_forward_mavros_x_ = -0.930; // 场地前方 +1m 在 MAVROS local x 的变化
    double field_forward_mavros_y_ = 0.366;  // 场地前方 +1m 在 MAVROS local y 的变化
    double field_left_mavros_x_ = -0.232;    // 场地左方 +1m 在 MAVROS local x 的变化
    double field_left_mavros_y_ = -0.973;    // 场地左方 +1m 在 MAVROS local y 的变化

    // ===== 圆环穿越坐标，现场快捷修改区 =====
    // 默认 direction_y=1：入口在圆环 -L 侧，穿到圆环 +L 侧。
    // 需要从反方向穿时，把 ring_pass_direction_y_ 改成 -1，或 launch 传 ring_pass_direction_y:=-1。
    double ring_center_x_ = 6.0;       // 圆环中心前方 F
    double ring_center_y_ = -1.6;      // 圆环中心左方 L
    double ring_center_z_ = 1.6;       // 圆环中心高度 Z
    double ring_entry_offset_y_ = 1.0; // 入口前位置距离圆环中心的 L 向距离
    double ring_exit_offset_y_ = 1.0;  // 穿环后位置距离圆环中心的 L 向距离
    int ring_pass_direction_y_ = -1;   // 1: -L -> +L, -1: +L -> -L

    // ===== 投放靶点坐标，现场快捷修改区 =====
    // 也可以在 launch 命令里传 big_target_x:=... 这类参数，不需要改代码。
    // 状态机顺序: 第一个靶点 -> 第二个靶点 -> 特殊靶。
    // 舵机接口仍沿用原服务名: 特殊靶=big，第二个=left，第一个=right。
    // 坐标为遥控器/场地坐标：x=前方 F，y=左方 L。
    double big_target_x_ = 6.0;
    double big_target_y_ = 1.0;
    double left_target_x_ = 3.6;
    double left_target_y_ = 1.6;
    double right_target_x_ = 1.8;
    double right_target_y_ = 1.6;
    double target_fly_z_ = 1.2;
    double target_drop_z_ = 0.5;
    double drop_duration_s_ = 1.5;
    double rise_duration_s_ = 1.5;
    double pre_servo_hold_s_ = 2.0;
    double servo_hold_s_ = 0.5;
    rclcpp::Time vertical_motion_start_time_;
    double vertical_motion_start_z_ = 1.2;
    double vertical_motion_target_z_ = 1.2;
    double vertical_motion_duration_s_ = 1.5;

    std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> servo_clients_;
    std::unordered_map<std::string, bool> servo_done_;
    std::unordered_map<std::string, bool> servo_pending_;
    std::unordered_map<std::string, rclcpp::Time> servo_done_time_;

// 主循环
void timer_cb()
{
    if(stop_auto_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "🛑 已手动切STABILIZED，自动控制已停止");
        return;
    }
    if (!current_state_.connected || !pose_received_) {
        return;
    }
    lock_mission_origin();
    if ((this->now() - last_pose_time_).seconds() > pose_timeout_s_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "等待新的 /mavros/local_position/pose，暂停推进任务");
        return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = this->now();
    target_pose.header.frame_id = "map";
    target_pose.pose.orientation = target_orientation();
    bool target_set = false;

    // ====================== 完整状态机 ======================
    switch (mission_state_)
    {
        // 初始化
        case MissionState::WAITING:
            RCLCPP_INFO(this->get_logger(), "[状态] WAITING → 开始初始化控制流");
            mission_state_ = MissionState::STREAMING;
            break;

        case MissionState::STREAMING:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[状态] STREAMING → 预占位 (0,0,%.2f)", target_fly_z_);
            target_set = set_target_pose(target_pose, 0.0, 0.0, target_fly_z_);

            static int cnt = 0;
            if (++cnt > 100) {
                RCLCPP_INFO(this->get_logger(), "[状态] STREAMING → 准备切换 OFFBOARD，随后等待手动解锁");
                mission_state_ = MissionState::WAIT_ARM;
            }
            break;

        case MissionState::WAIT_ARM:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[状态] WAIT_ARM → 等待 OFFBOARD 与手动解锁");
            target_set = set_target_pose(target_pose, 0.0, 0.0, target_fly_z_);

            if (this->now() - last_request_ > 2s)
            {
                if (current_state_.mode != "OFFBOARD") {
                    RCLCPP_INFO(this->get_logger(), "[状态] 请求切换到 OFFBOARD 模式");
                    if (request_mode("OFFBOARD")) {
                        last_request_ = this->now();
                    }
                } else if (!current_state_.armed) {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "[状态] WAIT_ARM → 等待遥控器手动解锁");
                }
            }

            if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
                mission_orientation_ = current_pose_.pose.orientation;
                mission_orientation_locked_ = true;
                RCLCPP_INFO(this->get_logger(), "[状态] WAIT_ARM → 检测到手动解锁，开始起飞");
                mission_state_ = MissionState::TAKEOFF;
            }
            break;

        case MissionState::TAKEOFF:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[状态] TAKEOFF → 飞往起飞点 (0,0,%.2f)", target_fly_z_);
            target_set = set_target_pose(target_pose, 0.0, 0.0, target_fly_z_);

            if (is_reached(0.0, 0.0, target_fly_z_)) {
                RCLCPP_INFO(this->get_logger(), "[状态] TAKEOFF → 已到达起飞点");
                mission_state_ = MissionState::HOVERING1;
                hover_start_time_ = this->now();
            }
            break;

        case MissionState::HOVERING1:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[状态] HOVERING1 → 起飞点悬停中");
            target_set = set_target_pose(target_pose, 0.0, 0.0, target_fly_z_);

            if (this->now() - hover_start_time_ > 3s) {
                RCLCPP_INFO(this->get_logger(), "[状态] HOVERING1 → 悬停结束，先前往二维码");
                mission_state_ = MissionState::MOVE_TO_QRCODE;
            }
            break;

        // 二维码
        case MissionState::MOVE_TO_QRCODE:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[状态] MOVE_TO_QRCODE → 飞往二维码 (1.8,0,%.2f)", target_fly_z_);
            target_set = set_target_pose(target_pose, 1.8, 0.0, target_fly_z_);

            if (is_reached(1.8, 0.0, target_fly_z_)) {
                RCLCPP_INFO(this->get_logger(), "[状态] MOVE_TO_QRCODE → 已到达二维码位置");
                mission_state_ = MissionState::HOVERING2;
                hover_start_time_ = this->now();
            }
            break;

        case MissionState::HOVERING2:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[状态] HOVERING2 → 二维码上方悬停 %.1fs，等待回传", qrcode_hover_s_);
            target_set = set_target_pose(target_pose, 1.8, 0.0, target_fly_z_);

            if ((this->now() - hover_start_time_).seconds() >= qrcode_hover_s_) {
                apply_qrcode_result();
                RCLCPP_INFO(this->get_logger(), "[状态] HOVERING2 → 二维码回传完成，开始按 第一个靶点 → 第二个靶点 → 特殊靶 顺序投放");
                mission_state_ = MissionState::MOVE_TO_PICTURE2;
            }
            break;

        case MissionState::MOVE_TO_POINT2:
            run_move_to_target(target_pose, target_set, "MOVE_TO_POINT2", "特殊靶",
                               big_target_x_, big_target_y_, MissionState::DROP1);
            break;

        case MissionState::DROP1:
            run_drop_target(target_pose, target_set, "DROP1", "特殊靶",
                            big_target_x_, big_target_y_, MissionState::PUT_1);
            break;

        case MissionState::PUT_1:
            run_put_target(target_pose, target_set, "PUT_1", "特殊靶", "big",
                           big_target_x_, big_target_y_, MissionState::RISE1, "开始上升");
            break;

        case MissionState::RISE1:
            run_rise_target(target_pose, target_set, "RISE1", "特殊靶",
                            big_target_x_, big_target_y_, MissionState::MOVE_TO_DETECT,
                            "直接前往圆环入口");
            break;

        case MissionState::MOVE_TO_PICTURE1:
            run_move_to_target(target_pose, target_set, "MOVE_TO_PICTURE1", "第二个靶点",
                               left_target_x_, left_target_y_, MissionState::DROP2);
            break;

        case MissionState::DROP2:
            run_drop_target(target_pose, target_set, "DROP2", "第二个靶点",
                            left_target_x_, left_target_y_, MissionState::PUT_2);
            break;

        case MissionState::PUT_2:
            run_put_target(target_pose, target_set, "PUT_2", "第二个靶点", "left",
                           left_target_x_, left_target_y_, MissionState::RISE2, "上升");
            break;

        case MissionState::RISE2:
            run_rise_target(target_pose, target_set, "RISE2", "第二个靶点",
                            left_target_x_, left_target_y_, MissionState::MOVE_TO_POINT2,
                            "前往特殊靶");
            break;

        case MissionState::MOVE_TO_PICTURE2:
            run_move_to_target(target_pose, target_set, "MOVE_TO_PICTURE2", "第一个靶点",
                               right_target_x_, right_target_y_, MissionState::DROP3);
            break;

        case MissionState::DROP3:
            run_drop_target(target_pose, target_set, "DROP3", "第一个靶点",
                            right_target_x_, right_target_y_, MissionState::PUT_3);
            break;

        case MissionState::PUT_3:
            run_put_target(target_pose, target_set, "PUT_3", "第一个靶点", "right",
                           right_target_x_, right_target_y_, MissionState::RISE3, "准备上升");
            break;

        case MissionState::RISE3:
            run_rise_target(target_pose, target_set, "RISE3", "第一个靶点",
                            right_target_x_, right_target_y_, MissionState::MOVE_TO_PICTURE1,
                            "前往第二个靶点");
            break;

        case MissionState::MOVE_TO_DETECT:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[状态] MOVE_TO_DETECT → 飞往圆环入口点 (%.2f,%.2f,%.2f)",
                                 ring_center_x_, ring_entry_y(), ring_center_z_);
            target_set = set_target_pose(target_pose, ring_center_x_, ring_entry_y(), ring_center_z_);
            if (is_reached(ring_center_x_, ring_entry_y(), ring_center_z_)) {
                RCLCPP_INFO(this->get_logger(), "[状态] MOVE_TO_DETECT → 已到达圆环入口点，准备穿环");
                hover_start_time_ = this->now();
                mission_state_ = MissionState::PASSING_RING;
            }
            break;

        case MissionState::PASSING_RING:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[状态] PASSING_RING → 正在穿环 (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                                 ring_center_x_, ring_entry_y(), ring_center_z_,
                                 ring_center_x_, ring_exit_y(), ring_center_z_);
            
            target_set = set_target_pose(target_pose, ring_center_x_, ring_exit_y(), ring_center_z_);

            if (is_reached(ring_center_x_, ring_exit_y(), ring_center_z_)) {
                RCLCPP_INFO(this->get_logger(), "[状态] 穿环完成，直接返回 left 降落点");
                home_num = 1;
                mission_state_ = MissionState::RETURN_HOME;
            }
            break;

        case MissionState::RETURN_HOME:
                    target_set = set_target_pose(target_pose, 0.0, HOME_POS[home_num], target_fly_z_);
                    if (is_reached(0.0, HOME_POS[home_num], target_fly_z_)) {
                        RCLCPP_INFO(this->get_logger(), "[状态] RETURN_HOME → 已回到起点");
                        mission_state_ = MissionState::FINAL_HOVER; // 修改跳转
                        hover_start_time_ = this->now();
                    }
                    break;

        case MissionState::FINAL_HOVER: // 专用结尾悬停
                    target_set = set_target_pose(target_pose, 0.0, HOME_POS[home_num], target_fly_z_);
                    if (this->now() - hover_start_time_ > 2s) {
                        RCLCPP_INFO(this->get_logger(), "[状态] 悬停结束，开始降落");
                        mission_state_ = MissionState::LANDING;
                    }
                    break;

        // 降落结束
        case MissionState::LANDING:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[状态] LANDING → 自动降落中");
            if (current_state_.mode != "AUTO.LAND" && this->now() - last_request_ > 2s)
            {
                if (request_mode("AUTO.LAND")) {
                    last_request_ = this->now();
                }
            }

            if (!current_state_.armed) {
                RCLCPP_INFO(this->get_logger(), "[状态] LANDING → 降落完成，任务即将结束");
                mission_state_ = MissionState::FINISHED;
            }
            break;

        case MissionState::FINISHED:
            RCLCPP_INFO(this->get_logger(), "🏁 全部任务圆满完成！");
            rclcpp::shutdown();
            return;
    }

    if (target_set) {
        pose_pub_->publish(target_pose);
    }
}
















    // 成员变量
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    const float HOME_POS[3] = { 0, 1.6, -1.6 };//第一个不要：1=左侧 L+，2=右侧 L-

};


// ====================== main 函数 ======================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto drone_node = std::make_shared<MavrosOffboard>();
    rclcpp::spin(drone_node);
    rclcpp::shutdown();
    return 0;
}
