// Created by Chengfu Zou on 2023.7.6
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

#ifndef SERIAL_DRIVER_PROTOCOL_HPP_
#define SERIAL_DRIVER_PROTOCOL_HPP_

// std
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_interfaces/msg/chassis_cmd.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/uart_transporter.hpp"

namespace qd::serial_driver::protocol {
typedef enum : unsigned char { Fire = 0x01, NotFire = 0x00 } FireState;

// Protocol interface
class Protocol {
public:
    virtual ~Protocol() = default;

    // Send gimbal command
    virtual void send(const rm_interfaces::msg::GimbalCmd& data) = 0;

    // Receive data from serial port
    virtual bool receive(rm_interfaces::msg::SerialReceiveData& data) = 0;

    /**
     * @brief 获取最近一次成功接收的 QD yaw 数据年龄编码。
     * @return 第 14 字节编码；不支持该扩展的协议返回 0，保持旧协议行为。
     */
    virtual uint8_t get_yaw_timing_byte() const {
        return 0;
    }

    // Create subscriptions for SerialDriverNode
    virtual std::vector<rclcpp::SubscriptionBase::SharedPtr>
    getSubscriptions(rclcpp::Node::SharedPtr node) = 0;

    // Cretate setMode client for SerialDriverNode
    virtual std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
    getClients(rclcpp::Node::SharedPtr node) const = 0;

    virtual std::string getErrorMessage() = 0;

    // 字节交换辅助函数
    inline int16_t swap_bytes(int16_t val) {
        return (val << 8) | ((val >> 8) & 0xFF);
    }
    inline uint32_t swap_bytes(uint32_t val) {
        return ((val << 24) & 0xFF000000) | ((val << 8) & 0x00FF0000) | ((val >> 8) & 0x0000FF00)
            | ((val >> 24) & 0x000000FF);
    }

// 确保结构体按1字节对齐，禁止编译器填充
#pragma pack(push, 1)
    struct SendPacket {
        int8_t header_pad = 0xff;       // 帧头 1
        int8_t fire_advice;             // 开火建议 2
        int16_t yaw;                    // 3 4
        int16_t pitch;                  // 5 6
        int16_t distance;               // 7 8
        uint8_t id;                     // 正在跟踪的装甲板类型 9
        int16_t target_v_yaw;           // EKF 目标自转角速度，定点 *100 10 11
        uint8_t reserved3;              // 保留位 12
        uint8_t reserved4;              // 保留位 13
        uint8_t reserved5;              // 保留位 14
        uint8_t check_byte;             // 校验位 15
        uint8_t tail_byte = 0x0d;       // 帧尾 16
    };

    struct ReceivePacket {
        uint8_t header = 0xff;          // 帧头 1
        uint8_t mode;                   // 模式 2
        int16_t roll;                   // 3 4
        int16_t pitch;                  // 5 6
        int16_t yaw;                    // 7 8
        int16_t bullet_speed;           // 9 10
        uint32_t mcu_timestamp;         // 下位机时间戳，单位毫秒 11 12 13 14
        uint8_t check_byte;             // 校验位 15
        uint8_t tail_byte = 0x0d;       // 帧尾 16
    };
#pragma pack(pop)

    uint8_t armor_id_to_uint8(const std::string& id) {
        const auto it = id_unit8_map.find(id);
        if (it != id_unit8_map.end()) {
            return it->second;
        }
        if (warned_unknown_ids_.insert(id).second) {
            FYT_WARN("serial_driver", "Unknown armor id '{}', fallback to 0", id);
        }
        return 0;
    }

    const std::map<std::string, uint8_t> id_unit8_map { { "1", 1 },      { "2", 2 },
                                                         { "3", 3 },      { "4", 4 },
                                                         { "5", 5 },      { "outpost", 6 },
                                                         { "sentry", 7 }, { "base", 8 },
                                                         { "negative", 9 },
                                                         { "0", 0 } };

private:
    std::set<std::string> warned_unknown_ids_;
};

} // namespace qd::serial_driver::protocol

#endif // SERIAL_DRIVER_PROTOCOLS_HPP_
