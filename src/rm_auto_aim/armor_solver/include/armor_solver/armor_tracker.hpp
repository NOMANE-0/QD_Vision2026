// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
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

#ifndef ARMOR_SOLVER_TRACKER_HPP_
#define ARMOR_SOLVER_TRACKER_HPP_

// std
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>
// ros2
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <Eigen/Eigen>
// project
#include "armor_solver/motion_model.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/extended_kalman_filter.hpp"
#include "rm_utils/math/utils.hpp"
#include "rm_utils/vision_state_machine.hpp"
#include <rm_utils/math/math.hpp>

namespace qd::auto_aim {

constexpr double OUTPOST_SPEED = 0.8 * M_PI;
constexpr double OUTPOST_RADIUS = 553. / 2. * 1e-3; // 553mm 直径
constexpr double MAX_AREA_RATIO = 0.7; // 最大面积比

enum ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker {
public:
    Tracker(double max_match_distance, double max_match_yaw);

    using Armors = rm_interfaces::msg::Armors;
    using Armor = rm_interfaces::msg::Armor;

    void init(const Armors::SharedPtr& armors_msg) noexcept;

    void update(const Armors::SharedPtr& armors_msg) noexcept;

    TrackerState tracker_state;

    std::unique_ptr<RobotStateEKF> ekf;
    std::unique_ptr<ArmorStateEKF> ekf_point;

    int tracking_thres; // frame
    int lost_thres; // second

    Armor tracked_armor;
    std::string tracked_id;
    ArmorsNum tracked_armors_num;
    Eigen::VectorXd measurement;
    Eigen::VectorXd target_state;
    Eigen::VectorXd armor_state;

    // 从状态量推导的装甲板参数（每次 update 后重新计算）
    std::vector<double> dz_list_;
    std::vector<double> radius_list_;

    double choose_yaw_diff;
    double choose_position_diff;

private:
    void initEKF(const Armor& a) noexcept;

    void updateArmorStateEKF(const Armors::SharedPtr& armors_msg) noexcept;

    void computeArmorParams() noexcept;

    double orientationToYaw(const geometry_msgs::msg::Quaternion& q) noexcept;

    void updateTracker(bool matched) noexcept;

    double max_match_distance_;
    double max_match_yaw_diff_;

    int detect_count_;
    int lost_count_;
    std::int64_t last_matched_stamp_ns_ = 0;

    // 状态机
    qd::utils::VisionStateMachine& state_machine_;
};

} // namespace qd::auto_aim

#endif // ARMOR_SOLVER_ARMOR_TRACKER_HPP_
