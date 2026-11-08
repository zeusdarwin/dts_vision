#pragma once

#include "tasks/base/packet_serial.hpp"
#include <point_msgs/msg/message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <mutex>

namespace dts_vision::tools::rcl {

inline std::optional<ReceiveRobotData> navigation_feedback(const point_msgs::msg::Message& msg) {
    if (msg.cmd_id != ReceiveRobotData::ID
        || msg.head != std::array<uint8_t, 3>{'D', 'T', 'S'}
        || !std::isfinite(msg.yaw) || !std::isfinite(msg.pitch)
        || !std::isfinite(msg.roll) || !std::isfinite(msg.bullet_speed)
        || !std::isfinite(msg.big_yaw_in_world)
        || msg.detect_color > 1 || msg.mode > 1) {
        return std::nullopt;
    }
    ReceiveRobotData data{};
    data.time_stamp_pc = msg.time_stamp_pc;
    data.time_stamp_receive_micro = msg.time_stamp_receive_micro;
    data.time_stamp_send_micro = msg.time_stamp_send_micro;
    data.yaw = msg.yaw;
    data.pitch = msg.pitch;
    data.roll = msg.roll;
    data.bullet_speed = msg.bullet_speed;
    data.detect_color = msg.detect_color;
    data.bullet_count = msg.bullet_count;
    data.operator_yaw_offset = msg.operator_yaw_offset;
    data.operator_pitch_offset = msg.operator_pitch_offset;
    data.mode = msg.mode;
    return data;
}

inline point_msgs::msg::Message navigation_plan(const SendRobotCmdData& data) {
    point_msgs::msg::Message msg;
    msg.cmd_id = data.cmd_ID;
    std::copy(std::begin(data.head), std::end(data.head), msg.head.begin());
    msg.time_stamp = data.time_stamp;
    msg.appear = data.appear;
    msg.pitch = data.pitch;
    msg.yaw = data.yaw;
    msg.target_yaw = data.target_yaw;
    msg.target_pitch = data.target_pitch;
    msg.enable_yaw_diff = data.enable_yaw_diff;
    msg.enable_pitch_diff = data.enable_pitch_diff;
    msg.v_yaw = data.v_yaw;
    msg.v_pitch = data.v_pitch;
    msg.a_yaw = data.a_yaw;
    msg.a_pitch = data.a_pitch;
    msg.detect_color = data.detect_color;
    return msg;
}

class NavigationBridge {
public:
    struct SubscriptionLog {
        uint64_t received = 0;
        uint64_t accepted = 0;
        uint64_t rejected = 0;
        std::chrono::steady_clock::time_point last_received{};
        std::optional<point_msgs::msg::Message> last_message;
    };

    SubscriptionLog subscription_log() {
        std::lock_guard lock(log_mutex_);
        return subscription_log_;
    }

    explicit NavigationBridge(
        std::function<void(const ReceiveRobotData&, const SentryJointState&)> receive) {
        // A private context leaves the application's signal handling intact.
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr);
        rclcpp::NodeOptions options;
        options.context(context_);
        node_ = std::make_shared<rclcpp::Node>("dts_auto_aim_navigation", options);
        publisher_ = node_->create_publisher<point_msgs::msg::Message>("/auto_aim_plan", 10);
        subscription_ = node_->create_subscription<point_msgs::msg::Message>(
            "/target_point", rclcpp::QoS(1).best_effort(),
            [this, receive = std::move(receive)](point_msgs::msg::Message::ConstSharedPtr msg) {
                auto data = navigation_feedback(*msg);
                {
                    std::lock_guard lock(log_mutex_);
                    ++subscription_log_.received;
                    if (data) ++subscription_log_.accepted;
                    else ++subscription_log_.rejected;
                    subscription_log_.last_received = std::chrono::steady_clock::now();
                    subscription_log_.last_message = *msg;
                }
                if (data) {
                    SentryJointState joint{};
                    joint.big_yaw_in_world = msg->big_yaw_in_world;
                    receive(*data, joint);
                }
            });
        rclcpp::ExecutorOptions executor_options;
        executor_options.context = context_;
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
        executor_->add_node(node_);
    }

    ~NavigationBridge() {
        executor_->cancel();
        context_->shutdown("navigation bridge stopped");
    }

    void poll() {
        std::lock_guard lock(spin_mutex_);
        executor_->spin_some(std::chrono::milliseconds(1));
    }

    void publish(const SendRobotCmdData& data) {
        publisher_->publish(navigation_plan(data));
    }

private:
    std::mutex log_mutex_;
    SubscriptionLog subscription_log_;
    std::shared_ptr<rclcpp::Context> context_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<point_msgs::msg::Message>::SharedPtr publisher_;
    rclcpp::Subscription<point_msgs::msg::Message>::SharedPtr subscription_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::mutex spin_mutex_;
};

} // namespace dts_vision::tools::rcl
