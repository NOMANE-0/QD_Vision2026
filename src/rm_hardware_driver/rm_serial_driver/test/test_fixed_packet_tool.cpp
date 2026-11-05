// Copyright 2021 RoboMaster-OSS
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

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dummy_transporter.hpp"
#include "gtest/gtest.h"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"

using namespace qd;
TEST(FixedPacketTool, construct_with_nullptr) {
  EXPECT_THROW(serial_driver::FixedPacketTool<32>(nullptr), std::invalid_argument);
}

TEST(FixedPacketTool, send_and_recv) {
  auto factory = std::make_shared<TransporterFactory>();
  auto transporter1 = factory->get_transporter1();
  auto transporter2 = factory->get_transporter2();
  auto packet_tool1 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter1);
  auto packet_tool2 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter2);
  serial_driver::FixedPacket<32> packet1, packet2;
  // send
  int a = 10;
  packet1.loadData(a, 10);
  bool send_ret = packet_tool1->sendPacket(packet1);
  ASSERT_TRUE(send_ret);
  // recv
  int b;
  bool recv_ret = packet_tool2->recvPacket(packet2);
  ASSERT_TRUE(recv_ret);
  packet2.unloadData<int>(b, 10);
  EXPECT_EQ(a, b);
}

TEST(FixedPacketTool, realtime_send) {
  auto factory = std::make_shared<TransporterFactory>();
  auto transporter1 = factory->get_transporter1();
  auto transporter2 = factory->get_transporter2();
  auto packet_tool1 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter1);
  auto packet_tool2 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter2);
  packet_tool1->enbaleRealtimeSend(true);
  serial_driver::FixedPacket<32> packet1, packet2;
  // recv
  auto t = std::thread([&]() {
    int b;
    for (int i = 0; i < 10; i++) {
      bool recv_ret = packet_tool2->recvPacket(packet2);
      ASSERT_TRUE(recv_ret);
      packet2.unloadData<int>(b, 10);
      EXPECT_EQ(i, b);
    }
  });
  // send
  for (int i = 0; i < 10; i++) {
    packet1.loadData(i, 10);
    bool send_ret = packet_tool1->sendPacket(packet1);
    ASSERT_TRUE(send_ret);
  }
  t.join();
}

namespace {

class ChunkedTransporter: public serial_driver::TransporterInterface {
public:
    explicit ChunkedTransporter(std::vector<std::vector<uint8_t>> chunks)
        : chunks_(std::move(chunks)) {}

    bool open() override { return true; }
    void close() override {}
    bool isOpen() override { return true; }

    /**
     * @brief 按预设分段模拟串口读取，空分段模拟传输中断。
     * @param buffer 输出字节缓冲区。
     * @param len 本次允许读取的最大字节数。
     * @return 读取字节数；中断时返回 0，数据耗尽时返回 -1。
     */
    int read(void* buffer, size_t len) override {
        if (chunk_index_ == chunks_.size()) {
            return -1;
        }
        const auto& chunk = chunks_[chunk_index_];
        const size_t count = std::min(len, chunk.size() - offset_);
        if (count > 0) {
            std::memcpy(buffer, chunk.data() + offset_, count);
        }
        offset_ += count;
        if (offset_ == chunk.size()) {
            ++chunk_index_;
            offset_ = 0;
        }
        return static_cast<int>(count);
    }

    int write(const void*, size_t len) override { return static_cast<int>(len); }
    std::string errorMessage() override { return "scripted stream ended"; }

private:
    std::vector<std::vector<uint8_t>> chunks_;
    size_t chunk_index_ = 0;
    size_t offset_ = 0;
};

/**
 * @brief 提供与电控发送代码一致的小端反馈帧，pitch 为 -6.01 度、yaw 为 19.17 度。
 * @return 16 字节帧，弹速为 28 米每秒，时间戳为 123456 毫秒。
 */
std::vector<uint8_t> feedback_frame() {
    return {0xff, 0x01, 0x00, 0x00, 0xa7, 0xfd, 0x7d, 0x07,
            0xf0, 0x0a, 0x40, 0xe2, 0x01, 0x00, 0x00, 0x0d};
}

} // namespace

TEST(FixedPacketTool, receive_every_frame_split) {
    const auto frame = feedback_frame();
    for (size_t split = 1; split < frame.size(); ++split) {
        SCOPED_TRACE(split);
        auto transporter = std::make_shared<ChunkedTransporter>(
            std::vector<std::vector<uint8_t>> {
                {frame.begin(), frame.begin() + split},
                {frame.begin() + split, frame.end()}});
        serial_driver::FixedPacketTool<16> tool(transporter);
        serial_driver::FixedPacket<16> packet;
        ASSERT_TRUE(tool.recvPacket(packet));
        EXPECT_EQ(std::memcmp(packet.buffer(), frame.data(), frame.size()), 0);
    }
}

TEST(FixedPacketTool, receive_one_byte_at_a_time_with_frame_markers_in_payload) {
    auto frame = feedback_frame();
    frame[4] = 0xff;
    frame[5] = 0x0d;
    frame[11] = 0xff;
    std::vector<std::vector<uint8_t>> chunks;
    for (const auto byte : frame) {
        chunks.push_back({byte});
    }
    auto transporter = std::make_shared<ChunkedTransporter>(std::move(chunks));
    serial_driver::FixedPacketTool<16> tool(transporter);
    serial_driver::FixedPacket<16> packet;
    ASSERT_TRUE(tool.recvPacket(packet));
    EXPECT_EQ(std::memcmp(packet.buffer(), frame.data(), frame.size()), 0);
}

TEST(FixedPacketTool, resynchronize_after_noise_and_bad_tail) {
    const auto frame = feedback_frame();
    auto bad_frame = frame;
    bad_frame.back() = 0x42;
    std::vector<uint8_t> stream {0x0d, 0x11, 0x22};
    stream.insert(stream.end(), bad_frame.begin(), bad_frame.end());
    stream.insert(stream.end(), frame.begin(), frame.end());
    auto transporter = std::make_shared<ChunkedTransporter>(
        std::vector<std::vector<uint8_t>> {stream});
    serial_driver::FixedPacketTool<16> tool(transporter);
    serial_driver::FixedPacket<16> packet;
    ASSERT_TRUE(tool.recvPacket(packet));
    EXPECT_EQ(std::memcmp(packet.buffer(), frame.data(), frame.size()), 0);
}

TEST(FixedPacketTool, preserve_order_across_partial_and_back_to_back_frames) {
    const auto first = feedback_frame();
    auto second = first;
    second[1] = 0x02;
    auto third = first;
    third[1] = 0x03;
    std::vector<uint8_t> remainder(first.begin() + 8, first.end());
    remainder.insert(remainder.end(), second.begin(), second.end());
    remainder.insert(remainder.end(), third.begin(), third.end());
    auto transporter = std::make_shared<ChunkedTransporter>(
        std::vector<std::vector<uint8_t>> {{first.begin(), first.begin() + 8}, remainder});
    serial_driver::FixedPacketTool<16> tool(transporter);
    serial_driver::FixedPacket<16> packet;
    for (const auto& expected : {first, second, third}) {
        ASSERT_TRUE(tool.recvPacket(packet));
        EXPECT_EQ(std::memcmp(packet.buffer(), expected.data(), expected.size()), 0);
    }
}

TEST(FixedPacketTool, discard_partial_frame_after_transport_failure) {
    const auto frame = feedback_frame();
    auto transporter = std::make_shared<ChunkedTransporter>(
        std::vector<std::vector<uint8_t>> {{frame.begin(), frame.begin() + 8}, {}, frame});
    serial_driver::FixedPacketTool<16> tool(transporter);
    serial_driver::FixedPacket<16> packet;
    packet.loadData<uint8_t>(0x55, 1);
    ASSERT_FALSE(tool.recvPacket(packet));
    EXPECT_EQ(packet.buffer()[1], 0x55);
    ASSERT_TRUE(tool.recvPacket(packet));
    EXPECT_EQ(std::memcmp(packet.buffer(), frame.data(), frame.size()), 0);
}
