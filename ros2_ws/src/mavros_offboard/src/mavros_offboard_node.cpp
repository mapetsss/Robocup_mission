#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>

using namespace std::chrono_literals;

// 任务状态机 —— 只在这里添加了新状态
enum class MissionState {
    WAITING,        // 等待连接与位置
    STREAMING,      // 持续发送期望值
    WAIT_ARM,       // 切 OFFBOARD 后等待手动解锁
    FLY_TO_HOVER,   // 起飞悬停 0 0 0.5
    HOVER_START,    // 初始悬停

    CALL_BIG_HOME,  // 原点悬停后 BIG 舵机回原点
    FLY_TO_LEFT,    // 向左飞到指定点
    CALL_LEFT_HOME, // 左侧到点后 left 舵机回原点
    CALL_RIGHT_HOME,// left 完成后 right 舵机回原点

    LANDING,        // 降落
    FINISHED        // 任务结束
};

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
        pose_timeout_s_ = std::max(this->declare_parameter<double>("pose_timeout_s", 1.0), 0.2);
        reach_tolerance_m_ = std::max(this->declare_parameter<double>("reach_tolerance_m", 0.1), 0.02);
        hover_duration_s_ = std::max(this->declare_parameter<double>("hover_duration_s", 1.0), 0.0);

        hover_x_ = this->declare_parameter<double>("hover_x", 0.0);
        hover_y_ = this->declare_parameter<double>("hover_y", 0.0);
        hover_z_ = this->declare_parameter<double>("hover_z", 0.5);
        left_x_ = this->declare_parameter<double>("left_x", 0.0);
        left_y_ = this->declare_parameter<double>("left_y", -0.5);
        left_z_ = this->declare_parameter<double>("left_z", 0.5);

        const auto big_service = this->declare_parameter<std::string>("big_home_service", "/servo/big_home");
        const auto left_service = this->declare_parameter<std::string>("left_home_service", "/servo/left_home");
        const auto right_service = this->declare_parameter<std::string>("right_home_service", "/servo/right_home");
        servo_clients_["big"] = this->create_client<std_srvs::srv::Trigger>(big_service);
        servo_clients_["left"] = this->create_client<std_srvs::srv::Trigger>(left_service);
        servo_clients_["right"] = this->create_client<std_srvs::srv::Trigger>(right_service);
        for (const auto& item : servo_clients_) {
            servo_done_[item.first] = false;
            servo_pending_[item.first] = false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "✅ 起飞原点触发 BIG，左点依次触发 left/right 后降落: hover=(%.2f, %.2f, %.2f), left=(%.2f, %.2f, %.2f)",
            hover_x_, hover_y_, hover_z_, left_x_, left_y_, left_z_);
    }

private:
    // 状态机
    MissionState mission_state_ = MissionState::WAITING;

    // 飞控状态
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_received_ = false;
    bool mission_origin_locked_ = false;
    double mission_origin_x_ = 0.0;
    double mission_origin_y_ = 0.0;
    double mission_origin_z_ = 0.0;

    // 悬停计时
    rclcpp::Time hover_start_time_;
    rclcpp::Time last_request_;
    rclcpp::Time last_pose_time_;
    double pose_timeout_s_ = 1.0;
    double reach_tolerance_m_ = 0.1;
    double hover_duration_s_ = 1.0;

    double hover_x_ = 0.0;
    double hover_y_ = 0.0;
    double hover_z_ = 0.5;
    double left_x_ = 0.0;
    double left_y_ = -0.5;
    double left_z_ = 0.5;

    std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> servo_clients_;
    std::unordered_map<std::string, bool> servo_done_;
    std::unordered_map<std::string, bool> servo_pending_;

    // 安全停止
    bool stop_auto_ = false;

    bool set_target_pose(geometry_msgs::msg::PoseStamped& target_pose, double x, double y, double z)
    {
        if (!mission_origin_locked_) {
            return false;
        }
        target_pose.pose.position.x = mission_origin_x_ + x;
        target_pose.pose.position.y = mission_origin_y_ + y;
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
            "任务原点已锁定: map=(%.3f, %.3f, %.3f). hover/left 目标点按相对该原点发布。",
            mission_origin_x_, mission_origin_y_, mission_origin_z_);
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

    // 判断是否到达目标点
    bool is_reached(double tx, double ty, double tz)
    {
        if (!mission_origin_locked_) {
            return false;
        }
        double dx = current_pose_.pose.position.x - (mission_origin_x_ + tx);
        double dy = current_pose_.pose.position.y - (mission_origin_y_ + ty);
        double dz = current_pose_.pose.position.z - (mission_origin_z_ + tz);
        return sqrt(dx*dx + dy*dy + dz*dz) < reach_tolerance_m_;
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

    bool tick_servo_home(const std::string& name)
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
                        RCLCPP_INFO(
                            this->get_logger(),
                            "%s 舵机回原点完成: %s",
                            name.c_str(), response->message.c_str());
                    } else {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "%s 舵机回原点失败: %s",
                            name.c_str(), response->message.c_str());
                    }
                } catch (const std::exception& exc) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "%s 舵机服务调用异常: %s",
                        name.c_str(), exc.what());
                }
            });
        RCLCPP_INFO(this->get_logger(), "已调用 %s 舵机回原点服务: %s", name.c_str(), client->get_service_name());
        return false;
    }

    // 主循环 —— 只修改这里的状态机
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
        target_pose.pose.orientation.w = 1.0;
        bool target_set = false;

        // ====================== 状态机 已修改 ======================
        switch (mission_state_)
        {
            case MissionState::WAITING:
                mission_state_ = MissionState::STREAMING;
                RCLCPP_INFO(this->get_logger(), "📶 开始发送控制流");
                break;

            case MissionState::STREAMING:
                target_set = set_target_pose(target_pose, hover_x_, hover_y_, hover_z_);

                static int cnt = 0;
                if (++cnt > 100) {
                    mission_state_ = MissionState::WAIT_ARM;
                    RCLCPP_INFO(this->get_logger(), "🔐 准备切 OFFBOARD，随后等待手动解锁");
                }
                break;

            case MissionState::WAIT_ARM:
                target_set = set_target_pose(target_pose, hover_x_, hover_y_, hover_z_);

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
                            "[状态] 已进入 OFFBOARD，等待你手动解锁");
                    }
                }

                if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
                    mission_state_ = MissionState::FLY_TO_HOVER;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "✈️ 起飞至原点悬停点 %.2f %.2f %.2f",
                        hover_x_, hover_y_, hover_z_);
                }
                break;

            // 起飞到原点
            case MissionState::FLY_TO_HOVER:
                target_set = set_target_pose(target_pose, hover_x_, hover_y_, hover_z_);
                if (is_reached(hover_x_, hover_y_, hover_z_)) {
                    mission_state_ = MissionState::HOVER_START;
                    hover_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "✅ 已到达起飞点");
                }
                break;

            // 初始悬停 → BIG 舵机回原点
            case MissionState::HOVER_START:
                target_set = set_target_pose(target_pose, hover_x_, hover_y_, hover_z_);
                if ((this->now() - hover_start_time_).seconds() > hover_duration_s_) {
                    mission_state_ = MissionState::CALL_BIG_HOME;
                    RCLCPP_INFO(this->get_logger(), "开始执行 BIG 舵机回原点");
                }
                break;

            // BIG 回原点完成 → 左飞
            case MissionState::CALL_BIG_HOME:
                target_set = set_target_pose(target_pose, hover_x_, hover_y_, hover_z_);
                if (tick_servo_home("big")) {
                    mission_state_ = MissionState::FLY_TO_LEFT;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "⬅️ 开始向左飞到指定点 %.2f %.2f %.2f",
                        left_x_, left_y_, left_z_);
                }
                break;

            // 向左飞到指定点
            case MissionState::FLY_TO_LEFT:
                target_set = set_target_pose(target_pose, left_x_, left_y_, left_z_);
                if (is_reached(left_x_, left_y_, left_z_)) {
                    mission_state_ = MissionState::CALL_LEFT_HOME;
                    hover_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "✅ 已到达左侧指定点，开始执行 left 舵机回原点");
                }
                break;

            // left 回原点完成 → right 回原点
            case MissionState::CALL_LEFT_HOME:
                target_set = set_target_pose(target_pose, left_x_, left_y_, left_z_);
                if (tick_servo_home("left")) {
                    mission_state_ = MissionState::CALL_RIGHT_HOME;
                    RCLCPP_INFO(this->get_logger(), "开始执行 right 舵机回原点");
                }
                break;

            // right 回原点完成 → 降落
            case MissionState::CALL_RIGHT_HOME:
                target_set = set_target_pose(target_pose, left_x_, left_y_, left_z_);
                if (tick_servo_home("right")) {
                    mission_state_ = MissionState::LANDING;
                    RCLCPP_INFO(this->get_logger(), "⬇️ left/right 舵机已依次回原点，开始自动降落");
                }
                break;

            case MissionState::LANDING:
                if (current_state_.mode != "AUTO.LAND" && this->now() - last_request_ > 2s)
                {
                    if (request_mode("AUTO.LAND")) {
                        last_request_ = this->now();
                    }
                }

                if (!current_state_.armed) {
                    mission_state_ = MissionState::FINISHED;
                }
                break;

            case MissionState::FINISHED:
                RCLCPP_INFO(this->get_logger(), "🏁 自动降落完成，任务结束！");
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosOffboard>());
    rclcpp::shutdown();
    return 0;
}
