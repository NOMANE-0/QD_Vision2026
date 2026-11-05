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

#include "armor_solver/armor_tracker.hpp"

#include "armor_solver/armor_measurement_gate.hpp"

namespace qd::auto_aim {
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff):
    tracker_state(TrackerState::LOST),
    tracked_id(std::string("")),
    measurement(Eigen::VectorXd::Zero(Z_N)),
    target_state(Eigen::VectorXd::Zero(X_N)),
    armor_state(Eigen::VectorXd::Zero(X_N_)),
    choose_yaw_diff(DBL_MAX),
    choose_position_diff(DBL_MAX),
    max_match_distance_(max_match_distance),
    max_match_yaw_diff_(max_match_yaw_diff),
    detect_count_(0),
    lost_count_(0),
    state_machine_(qd::utils::VisionStateMachine::getInstance()) {}

void Tracker::init(const Armors::SharedPtr& armors_msg) noexcept {
    if (armors_msg->armors.empty()) {
        return;
    }

    double min_distance = DBL_MAX;
    tracked_armor = armors_msg->armors[0];
    for (const auto& armor: armors_msg->armors) {
        if (armor.distance_to_image_center < min_distance) {
            min_distance = armor.distance_to_image_center;
            tracked_armor = armor;
        }
    }

    initEKF(tracked_armor);
    last_matched_stamp_ns_ = rclcpp::Time(armors_msg->header.stamp).nanoseconds();
    FYT_INFO("armor_solver", "Init EKF!");

    tracked_id = tracked_armor.number;
    tracker_state = TrackerState::DETECTING;
    state_machine_.setTrackerState(tracker_state);

    if (tracked_armor.type == "large"
        && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
        tracked_armors_num = ArmorsNum::BALANCE_2;
    } else if (tracked_id == "outpost") {
        tracked_armors_num = ArmorsNum::OUTPOST_3;
    } else {
        tracked_armors_num = ArmorsNum::NORMAL_4;
    }

    computeArmorParams();
}

void Tracker::update(const Armors::SharedPtr& armors_msg) noexcept {
    if (armors_msg->armors.empty()) {
        updateTracker(false);
        return;
    }

    utils::AutoTimer timer("armor_solver", "Tracker::update");
    updateArmorStateEKF(armors_msg);

    // EKF predict
    Eigen::VectorXd ekf_prediction = ekf->predict();
    target_state = ekf_prediction;

    int armors_num_int = static_cast<int>(tracked_armors_num);

    // 匹配并更新所有同 id 装甲板（用马氏距离选择对应装甲板）
    bool matched = false;
    double best_area = 0;
    Armor best_armor;
    double best_yaw_diff = DBL_MAX;
    const auto frame_stamp_ns = rclcpp::Time(armors_msg->header.stamp).nanoseconds();

    for (const auto& armor: armors_msg->armors) {
        if (armor.number != tracked_id) {
            continue;
        }

        double yaw = orientationToYaw(armor.pose.orientation);

        auto p = armor.pose.position;
        if (!std::isfinite(yaw) || !std::isfinite(p.x) || !std::isfinite(p.y)
            || !std::isfinite(p.z))
        {
            continue;
        }
        if (is_implausible_same_armor_orientation(
                tracked_armor.pose.position,
                orientationToYaw(tracked_armor.pose.orientation),
                last_matched_stamp_ns_,
                p,
                yaw,
                frame_stamp_ns
            ))
        {
            FYT_WARN("armor_solver", "Reject implausible armor yaw jump for {}", armor.number);
            continue;
        }
        double center_yaw = std::atan2(p.y, p.x);
        double pitch = std::atan2(p.z, std::sqrt(p.x * p.x + p.y * p.y));
        double distance = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        Eigen::Vector4d measurement_candidate(center_yaw, pitch, distance, yaw);

        // 遍历所有候选装甲板位，选择马氏距离最小的对应关系
        int best_offset = -1;
        double min_mahalanobis = DBL_MAX;
        double min_yaw_diff = DBL_MAX;
        for (int i = 0; i < armors_num_int; ++i) {
            Measure h(i, armors_num_int);
            this->ekf->setMeasureFunc(h);

            double mahalanobis = this->ekf->calculateMahalanobisDistance(measurement_candidate);
            double predicted_yaw = reduced_angle(target_state(6) + i * 2.0 * M_PI / armors_num_int);
            double yaw_diff = std::abs(reduced_angle(predicted_yaw - yaw));

            if (yaw_diff > max_match_yaw_diff_) {
                continue;
            }

            if (mahalanobis < min_mahalanobis) {
                min_mahalanobis = mahalanobis;
                min_yaw_diff = yaw_diff;
                best_offset = i;
            }
        }

        // 所有板位都与本帧观测朝向不匹配。
        if (best_offset < 0) {
            FYT_WARN(
                "armor_solver",
                "Armor {} best match yaw diff {:.3f} rad > {:.3f} rad, md {:.3f}, ignore this armor",
                armor.number,
                min_yaw_diff,
                max_match_yaw_diff_,
                min_mahalanobis
            );
            continue;
        }

        this->measurement = measurement_candidate;

        // 设置对应装甲板的观测模型
        Measure h(best_offset, armors_num_int);
        this->ekf->setMeasureFunc(h);

        target_state = this->ekf->update(measurement);

        matched = true;

        // 记录面积最大的装甲板用于可视化与决策
        if (armor.area > best_area) {
            best_area = armor.area;
            best_armor = armor;
            best_yaw_diff = min_yaw_diff;
        }
    }

    updateTracker(matched);
    if (!matched) {
        return;
    }

    tracked_armor = best_armor;
    last_matched_stamp_ns_ = frame_stamp_ns;
    choose_yaw_diff = best_yaw_diff;

    // 状态约束
    target_state(5) = 0.0; // v_zc 始终为 0
    target_state(6) = reduced_angle(target_state(6)); // yaw
    target_state(8) = std::clamp(target_state(8), 0.1, 0.4); // r
    target_state(9) = std::clamp(target_state(9), -0.2, 0.2); // dr
    target_state(10) = std::clamp(target_state(10), -0.3, 0.3); // dz1
    target_state(11) = std::clamp(target_state(11), -0.3, 0.3); // dz2

    // 前哨站特殊处理
    if (tracked_armors_num == ArmorsNum::OUTPOST_3) {
        auto v_yaw = target_state(7);
        if (std::abs(v_yaw) > 2.0) {
            target_state(7) = (v_yaw > 0) ? OUTPOST_SPEED : -OUTPOST_SPEED;
        }

        target_state(8) = OUTPOST_RADIUS; // 固定半径
        target_state(9) = 0.0; // 前哨战半径相等，无 dr
    }

    ekf->setState(target_state);
    computeArmorParams();

    // 更新装甲板数量类型
    if (tracked_armor.type == "large"
        && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
        tracked_armors_num = ArmorsNum::BALANCE_2;
    } else if (tracked_id == "outpost") {
        tracked_armors_num = ArmorsNum::OUTPOST_3;
    } else {
        tracked_armors_num = ArmorsNum::NORMAL_4;
    }
}

void Tracker::initEKF(const Armor& a) noexcept {
    double xa = a.pose.position.x;
    double ya = a.pose.position.y;
    double za = a.pose.position.z;

    double yaw = orientationToYaw(a.pose.orientation);

    target_state = Eigen::VectorXd::Zero(X_N);
    double r = 0.26;
    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    double zc = za;

    // xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, dr, dz1, dz2
    target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, 0, 0, 0;

    ekf->init(target_state);

    armor_state << xa, 0, ya, 0, za, 0;
    ekf_point->init(armor_state);
}

/**
 * @brief 基于 EKF 状态量更新 dz_list 和 radius_list
 * 
 */
void Tracker::computeArmorParams() noexcept {
    int n = static_cast<int>(tracked_armors_num);
    dz_list_.resize(n);
    radius_list_.resize(n);
    for (int i = 0; i < n; ++i) {
        // NORMAL_4 对称半径、高度
        // 循环嵌套有点多了
        if (tracked_armors_num == ArmorsNum::NORMAL_4) {
            if (i == 1 || i == 3) {
                dz_list_[i] = target_state(4) + target_state(10);
                radius_list_[i] = target_state(8) + target_state(9);
            } else {
                dz_list_[i] = target_state(4);
                radius_list_[i] = target_state(8);
            }
        }

        if (tracked_armors_num == ArmorsNum::OUTPOST_3) {
            // 前哨战完全不等高
            if (i == 1) {
                dz_list_[i] = target_state(4) + target_state(10);
            } else if (i == 2) {
                dz_list_[i] = target_state(4) + target_state(11);
            } else {
                dz_list_[i] = target_state(4);
            }

            // 半径相等
            radius_list_[i] = target_state(8);
        }
    }
}

// 平移运动模型更新
void Tracker::updateArmorStateEKF(const Armors::SharedPtr& armors_msg) noexcept {
    if (armors_msg->armors.empty()) {
        return;
    }
    Eigen::VectorXd ekf_prediction = ekf_point->predict();
    auto predicted_position =
        Eigen::Vector3d(ekf_prediction(0), ekf_prediction(2), ekf_prediction(4));

    bool matched = false;
    double min_distance = DBL_MAX;
    Armor closest_armor;
    Armor max_area_armor;
    double max_area = 0.0;

    // 找最近的一块目标装甲板，并记录最大面积的装甲板
    for (const auto& armor: armors_msg->armors) {
        if (armor.number != tracked_id) {
            continue;
        }

        // 记录最大面积
        if (armor.area > max_area) {
            max_area = armor.area;
            max_area_armor = armor;
        }

        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < min_distance) {
            min_distance = position_diff;
            closest_armor = armor;
            matched = true;
        }
    }

    // 没同号装甲板的话返回
    if (!matched) {
        choose_position_diff = DBL_MAX;
        return;
    }

    choose_position_diff = min_distance;

    Armor best_armor;
    bool is_jumped = false;
    // 如果预测跟踪的最近板面积小于最大面积的 MAX_AREA_RATIO 倍，则发生切板
    if (closest_armor.area < MAX_AREA_RATIO * max_area) {
        best_armor = max_area_armor;
        is_jumped = true;
    } else {
        best_armor = closest_armor;
    }

    auto p = best_armor.pose.position;
    Eigen::Vector3d measurement_point(
        std::atan2(p.y, p.x),
        std::atan2(p.z, std::sqrt(p.x * p.x + p.y * p.y)),
        std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
    );
    if (!is_jumped && min_distance < max_match_distance_) {
        // 如果没有发生切板，并且匹配距离小于阈值，就进行平滑更新
        armor_state = ekf_point->update(measurement_point);
    } else {
        // 发生切板或匹配距离过大，强制重置滤波器参数
        armor_state(0) = p.x;
        armor_state(1) = 0;
        armor_state(2) = p.y;
        armor_state(3) = 0;
        armor_state(4) = p.z;
        armor_state(5) = 0;
        ekf_point->setState(armor_state);
    }
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion& q) noexcept {
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    yaw = reduced_angle(yaw);
    return yaw;
}

/**
 * @brief 更新跟踪器状态机
 * 
 * @param matched 时候匹配到装甲板
 */
void Tracker::updateTracker(bool matched) noexcept {
    if (tracker_state == TrackerState::DETECTING) {
        if (matched) {
            detect_count_++;
            if (detect_count_ > tracking_thres) {
                detect_count_ = 0;
                tracker_state = TrackerState::TRACKING;
                FYT_INFO("armor_solver", "Tracker state: TRACKING {}", tracked_id);
            }
        } else {
            detect_count_ = 0;
            tracker_state = TrackerState::LOST;
            FYT_INFO("armor_solver", "Tracker state: LOST {}", tracked_id);
        }
    } else if (tracker_state == TrackerState::TRACKING) {
        if (!matched) {
            tracker_state = TrackerState::TEMP_LOST;
            lost_count_++;
            FYT_INFO("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
        }
    } else if (tracker_state == TrackerState::TEMP_LOST) {
        if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_thres) {
                lost_count_ = 0;
                tracker_state = TrackerState::LOST;
                FYT_INFO("armor_solver", "Tracker state: LOST {}", tracked_id);
            }
        } else {
            tracker_state = TrackerState::TRACKING;
            lost_count_ = 0;
            FYT_INFO("armor_solver", "Tracker state: TRACKING {}", tracked_id);
        }
    }

    state_machine_.setTrackerState(tracker_state);
}

} // namespace qd::auto_aim
