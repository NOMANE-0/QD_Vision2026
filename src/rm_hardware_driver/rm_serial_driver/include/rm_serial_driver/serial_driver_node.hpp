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

#ifndef SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_
#define SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_

// std
#include <memory>
#include <thread>
// ros2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "rm_utils/heartbeat.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/feedback_pose_aligner.hpp"
#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/protocol_factory.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "rm_utils/vision_state_machine.hpp"
#include "rm_utils/time_synchronizer.hpp"

namespace qd::serial_driver {

// Node wrapper for SerialDriver
// Implementing secondary development through the Protocol class
class SerialDriverNode : public rclcpp::Node {
public:
  explicit SerialDriverNode(const rclcpp::NodeOptions &options);

  ~SerialDriverNode();

  void listenLoop();

  void init();

  // Param client to set detect_color
  struct SetModeClient {
    SetModeClient(rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr p) : ptr(p) {}
    std::atomic<bool> on_waiting = false;
    std::atomic<int> mode = 0;
    rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr ptr;
  };
  std::unordered_map<std::string, SetModeClient> set_mode_clients_;
  void setMode(SetModeClient &client, const uint8_t mode);

private:
  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;

  std::unique_ptr<std::thread> listen_thread_;
  // Protocol
  std::unique_ptr<protocol::Protocol> protocol_;

  std::string target_frame_;
  std::string child_frame_id_;
  // Subscriptions
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  // Publisher
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_data_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_data_age_pub_;
  FeedbackPoseAligner feedback_pose_aligner_;

  // Broadcast tf from odom to gimbal_link
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 状态机
  qd::utils::VisionStateMachine& state_machine_;

  // 软同步时间戳
  std::deque<double> offset_window_;
  utils::TimeSynchronizer time_sync_;
};

}  // namespace qd::serial_driver

#endif  // SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_
