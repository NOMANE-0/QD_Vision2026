// Created by Chengfu Zou
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

#include "rm_serial_driver/protocol/default_protocol.hpp"

namespace qd::serial_driver::protocol {

DefaultProtocol::DefaultProtocol(std::string_view port_name, int speed, bool enable_data_print) {
    auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name), speed);
    packet_tool_ = std::make_shared<FixedPacketTool<16>>(uart_transporter);
    packet_tool_->enbaleDataPrint(enable_data_print);
}

std::vector<rclcpp::SubscriptionBase::SharedPtr>
DefaultProtocol::getSubscriptions(rclcpp::Node::SharedPtr node) {
    return { node->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "armor_solver/cmd_gimbal",
        rclcpp::SensorDataQoS(),
        [this](const rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg) { this->send(*msg); }
    )

    };
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
DefaultProtocol::getClients(rclcpp::Node::SharedPtr node) const {
    return { node->create_client<rm_interfaces::srv::SetMode>(
        "armor_detector/set_mode",
        rmw_qos_profile_services_default
    ) };
}

void DefaultProtocol::send(const rm_interfaces::msg::GimbalCmd& data) {
    FixedPacket<16> packet;
    SendPacket pkt;
    // 填充数据并处理大小端转换
    pkt.fire_advice = data.fire_advice ? FireState::Fire : FireState::NotFire;
    pkt.yaw = swap_bytes(static_cast<int16_t>(data.yaw * 100));
    pkt.pitch = swap_bytes(static_cast<int16_t>(data.pitch * 100));
    pkt.distance = swap_bytes(static_cast<int16_t>(data.distance * 100));
    pkt.id = armor_id_to_uint8(data.id);
    pkt.target_v_yaw = swap_bytes(static_cast<int16_t>(data.target_v_yaw * 100));
    
    packet.loadData<SendPacket>(pkt, 0);

    packet_tool_->sendPacket(packet);
}

bool DefaultProtocol::receive(rm_interfaces::msg::SerialReceiveData& data) {
    FixedPacket<16> packet;
    if (!packet_tool_->recvPacket(packet)) {
        return false;
    }

    const auto* raw_ptr = packet.buffer();
    ReceivePacket pkt;
    std::memcpy(&pkt, raw_ptr, sizeof(ReceivePacket));

    // 电控反馈使用小端 int16（度×100）和 uint32（毫秒）。
    // 当前支持的 amd64/arm64 均为小端；pitch 保持电控抬头为正的符号约定。
    data.mode = pkt.mode;
    data.roll = pkt.roll / 100.0f;
    data.pitch = pkt.pitch / 100.0f;
    data.yaw = pkt.yaw / 100.0f;
    data.bullet_speed = pkt.bullet_speed / 100.0f;
    data.mcu_timestamp = pkt.mcu_timestamp;
    // QD 帧尚未启用校验，第 14 字节由新电控携带顶板 yaw 数据年龄。
    yaw_timing_byte_ = pkt.check_byte;

    return true;
}

} // namespace qd::serial_driver::protocol
