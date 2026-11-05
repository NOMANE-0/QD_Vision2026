// Created by Chengfu Zou on 2023.7.1
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_serial_driver/serial_driver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
// std
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>
// ros2
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_serial_driver/uart_transporter.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace qd::serial_driver {
SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options):
    Node("serial_driver", options),
    state_machine_(qd::utils::VisionStateMachine::getInstance()) {
    FYT_REGISTER_LOGGER("serial_driver", "qd2026-log", INFO);

    // Task thread
    listen_thread_ = std::make_unique<std::thread>(&SerialDriverNode::listenLoop, this);
}

void SerialDriverNode::init() {
    FYT_INFO("serial_driver", "Initializing SerialDriverNode!");
    // Init
    target_frame_ = this->declare_parameter("target_frame", "odom");
    child_frame_id_ = this->declare_parameter("child_frame_id", "gimbal_link");
    std::string port_name = this->declare_parameter("port_name", "/dev/ttyUSB0");
    std::string protocol_type = this->declare_parameter("protocol", "infantry");
    bool enable_data_print = this->declare_parameter("enable_data_print", false);
    int baud_rate = this->declare_parameter("baud_rate", 115200);
    // Create Protocol
    protocol_ =
        ProtocolFactory::createProtocol(protocol_type, port_name, baud_rate, enable_data_print);
    if (protocol_ == nullptr) {
        FYT_FATAL("serial_driver", "Failed to create protocol with type: {}", protocol_type);
        rclcpp::shutdown();
        return;
    }
    FYT_INFO(
        "serial_driver",
        "Protocol has been created with type: {}, port: {}",
        protocol_type,
        port_name
    );

    // Subscriptions
    subscriptions_ = protocol_->getSubscriptions(this->shared_from_this());
    for (auto sub: subscriptions_) {
        FYT_INFO("serial_driver", "Subscribe to topic: {}", sub->get_topic_name());
    }
    // Publisher
    serial_receive_data_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
        "serial/receive",
        rclcpp::SensorDataQoS()
    );
    yaw_data_age_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "serial/yaw_data_age_ms", rclcpp::SensorDataQoS()
    );

    // TF broadcaster
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Param client
    for (auto client: protocol_->getClients(this->shared_from_this())) {
        std::string name = client->get_service_name();
        set_mode_clients_.emplace(name, client);
        FYT_INFO("serial_driver", "Create client for service: {}", name);
    }

    // Heartbeat
    heartbeat_ = HeartBeatPublisher::create(this);

    FYT_INFO("serial_driver", "SerialDriverNode has been initialized!");
}

SerialDriverNode::~SerialDriverNode() {
    FYT_INFO("serial_driver", "Destroy SerialDriverNode!");
    rclcpp::shutdown();
    if (listen_thread_ != nullptr) {
        listen_thread_->join();
    }
}

void SerialDriverNode::listenLoop() {
    if (protocol_ == nullptr) {
        // Lazy init because shared_from_this() is not available in constructor
        init();
    }

    while (rclcpp::ok()) {
        rm_interfaces::msg::SerialReceiveData receive_data;
        if (protocol_->receive(receive_data)) {
            const uint8_t timing_byte = protocol_->get_yaw_timing_byte();
            const auto yaw_age = decode_yaw_data_age(timing_byte);
            // 获取当前 PC 时间和 MCU 时间（转换为秒）
            rclcpp::Time pc_now = this->now();
            rclcpp::Time real_gimbal_time = pc_now; // 默认使用 PC 时间，后续会通过 PLL 同步调整
            /*             double mcu_time_sec = receive_data.mcu_timestamp / 1000.0; // 假设下位机发来的是毫秒

            // 计算当前包的时间偏移量 (Offset)
            double current_offset = pc_now.seconds() - mcu_time_sec;

            // 滑动窗口寻找最小偏移量（过滤掉 USB 延迟峰值）
            offset_window_.push_back(current_offset);
            if (offset_window_.size() > 100) { // 维持约 1 秒的窗口 (假设 100Hz)
                offset_window_.pop_front();
            }
            double min_offset = *std::min_element(offset_window_.begin(), offset_window_.end());

            // 计算无抖动的真实云台采样时间
            rclcpp::Time real_gimbal_time(( mcu_time_sec + min_offset) * 1e9, pc_now.get_clock_type()); // 转换为纳秒构造 rclcpp::Time
 */
            // 扔进 PLL 滤波器，获得剃除了所有调度毛刺的完美绝对时间
            if (receive_data.mcu_timestamp > 0 || yaw_age.supported) {
                double sync_time_sec =
                    time_sync_.sync(receive_data.mcu_timestamp, pc_now.seconds(), false);

                // 将 double 秒级时间无损转换为 rclcpp::Time (通过纳秒构造)
                real_gimbal_time = rclcpp::Time(
                    static_cast<int64_t>(sync_time_sec * 1e9),
                    pc_now.get_clock_type()
                );

                timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                real_gimbal_time += rclcpp::Duration::from_seconds(timestamp_offset_);
            }

            receive_data.header.stamp = real_gimbal_time;
            receive_data.header.frame_id = target_frame_;

            std_msgs::msg::Float64 age_msg;
            // NaN 表示旧电控没有上报年龄；127 表示新电控姿态缺失或过期。
            age_msg.data = yaw_age.supported ? static_cast<double>(yaw_age.milliseconds)
                                             : std::numeric_limits<double>::quiet_NaN();
            yaw_data_age_pub_->publish(age_msg);
            const auto pose = feedback_pose_aligner_.update(
                real_gimbal_time.nanoseconds(),
                receive_data.mcu_timestamp,
                receive_data.roll,
                receive_data.pitch,
                receive_data.yaw,
                timing_byte
            );

            state_machine_.setVisionMode(
                static_cast<VisionMode>(receive_data.mode)
            ); // 状态机设置识别模式
            for (auto& [service_name, client]: set_mode_clients_) {
                if (client.mode.load() != receive_data.mode && !client.on_waiting.load()) {
                    setMode(client, receive_data.mode);
                }
            }

            if (pose) {
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = rclcpp::Time(pose->stamp_ns, pc_now.get_clock_type());
                t.header.frame_id = target_frame_;
                t.child_frame_id = child_frame_id_;
                auto roll = pose->roll_deg * M_PI / 180.0;
                auto pitch = -pose->pitch_deg * M_PI / 180.0;
                auto yaw = pose->yaw_deg * M_PI / 180.0;
                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);
                t.transform.rotation = tf2::toMsg(q);
                tf_broadcaster_->sendTransform(t);

                // odom_rectify: 转了roll角后的坐标系
                Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
                Eigen::Vector3d rpy = utils::getRPY(q_eigen.toRotationMatrix());
                q.setRPY(rpy[0], 0, 0);
                t.header.frame_id = target_frame_;
                t.child_frame_id = target_frame_ + "_rectify";
                tf_broadcaster_->sendTransform(t);
            }

            serial_receive_data_pub_->publish(
                std::make_unique<rm_interfaces::msg::SerialReceiveData>(std::move(receive_data))
            );

            heartbeat_->publish();
        } else {
            auto error_message = protocol_->getErrorMessage();
            error_message = error_message.empty() ? "unknown" : error_message;
            FYT_WARN("serial_driver", "Failed to reveive packet! error message :{}", error_message);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

void SerialDriverNode::setMode(SetModeClient& client, const uint8_t mode) {
    using namespace std::chrono_literals;

    std::string service_name = client.ptr->get_service_name();
    // Wait for service
    while (!client.ptr->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            FYT_ERROR(
                "serial_driver",
                "Interrupted while waiting for the service {}. Exiting.",
                service_name
            );
            return;
        }
        FYT_INFO("serial_driver", "Service {} not available, waiting again...", service_name);
    }
    if (!client.ptr->service_is_ready()) {
        FYT_WARN("serial_driver", "Service: {} is not available!", service_name);
        return;
    }
    // Send request
    auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
    req->mode = mode;

    client.on_waiting.store(true);
    auto result = client.ptr->async_send_request(
        req,
        [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
            client.on_waiting.store(false);
            if (result.get()->success) {
                client.mode.store(mode);
            }
        }
    );
}

} // namespace qd::serial_driver

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(qd::serial_driver::SerialDriverNode)
