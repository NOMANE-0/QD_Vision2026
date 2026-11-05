#include "armor_solver/armor_measurement_gate.hpp"
#include "armor_solver/armor_solver.hpp"
#include "armor_solver/armor_yaw_solver.hpp"
#include "armor_solver/yaw_command_filter.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace qd::auto_aim {
namespace {
    TEST(ArmorMeasurementGateTest, RejectsFloorReflectionAndKeepsStaggeredArmor) {
        const std::vector<double> predicted_heights { -0.30, -0.52, -0.30, -0.52 };
        EXPECT_TRUE(is_armor_height_consistent(-0.31, predicted_heights, 0.30));
        EXPECT_TRUE(is_armor_height_consistent(-0.54, predicted_heights, 0.30));
        EXPECT_FALSE(is_armor_height_consistent(-0.90, predicted_heights, 0.30));
    }

    TEST(ArmorMeasurementGateTest, RejectsOrientationFlipAtSamePositionButAllowsPlateSwitch) {
        geometry_msgs::msg::Point previous_position;
        previous_position.x = 1.0;
        geometry_msgs::msg::Point same_position = previous_position;
        same_position.y = 0.01;
        constexpr std::int64_t kPreviousStampNs = 1000000000;
        constexpr std::int64_t kNextStampNs = 1020000000;

        EXPECT_TRUE(is_implausible_same_armor_orientation(
            previous_position, 20.0 * M_PI / 180.0, kPreviousStampNs,
            same_position, -49.0 * M_PI / 180.0, kNextStampNs
        ));
        EXPECT_FALSE(is_implausible_same_armor_orientation(
            previous_position, 20.0 * M_PI / 180.0, kPreviousStampNs,
            same_position, 9.0 * M_PI / 180.0, kNextStampNs
        ));

        geometry_msgs::msg::Point next_plate_position = previous_position;
        next_plate_position.y = 0.3;
        EXPECT_FALSE(is_implausible_same_armor_orientation(
            previous_position, 20.0 * M_PI / 180.0, kPreviousStampNs,
            next_plate_position, -70.0 * M_PI / 180.0, kNextStampNs
        ));
        EXPECT_FALSE(is_implausible_same_armor_orientation(
            previous_position, 20.0 * M_PI / 180.0, kPreviousStampNs,
            same_position, -49.0 * M_PI / 180.0, kPreviousStampNs + 200000000
        ));
    }

    TEST(YawCommandFilterTest, SmoothsCorrectionsOncePerImageAndKeepsSmallDeadband) {
        YawCommandFilter filter(0.25, 0.15);
        EXPECT_DOUBLE_EQ(filter.update(-14.0, 1, "3"), -14.0);
        EXPECT_DOUBLE_EQ(filter.update(-22.0, 2, "3"), -16.0);
        EXPECT_DOUBLE_EQ(filter.update(-22.0, 2, "3"), -16.0);
        EXPECT_DOUBLE_EQ(filter.update(-10.0, 3, "3"), -14.5);
        EXPECT_DOUBLE_EQ(filter.update(-14.6, 4, "3"), -14.5);
        EXPECT_DOUBLE_EQ(filter.update(-14.6, 5, "3"), -14.5);
        EXPECT_DOUBLE_EQ(filter.update(-15.0, 6, "4"), -15.0);
        filter.reset();
        EXPECT_DOUBLE_EQ(filter.update(20.0, 7, "4"), 20.0);
    }

    TEST(YawCommandFilterTest, UsesShortestAngleAcrossWraparound) {
        YawCommandFilter filter(0.25, 0.0);
        EXPECT_DOUBLE_EQ(filter.update(179.0, 1, "3"), 179.0);
        EXPECT_DOUBLE_EQ(filter.update(-179.0, 2, "3"), 179.5);
        EXPECT_DOUBLE_EQ(filter.update(-179.0, 3, "3"), 179.875);
    }

    class YawStabilityTest: public testing::Test {
    protected:
        static void SetUpTestSuite() {
            rclcpp::init(0, nullptr);
            FYT_REGISTER_LOGGER("armor_solver", "/tmp/armor_solver_yaw_tests", ERROR);
        }

        static void TearDownTestSuite() {
            rclcpp::shutdown();
        }

        void SetUp() override {
            node_ = std::make_shared<rclcpp::Node>("yaw_stability_test");
            node_->declare_parameter("target_frame", "odom");
            node_->declare_parameter("gimble_frame", "gimbal_link");
            buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            buffer_->setUsingDedicatedThread(true);
            set_gimbal(10, 0.0);
        }

        /**
         * @brief 向独立 TF 缓冲区写入模拟云台姿态。
         * @param seconds ROS 时间，单位 s。
         * @param yaw 云台在 odom 中的 yaw，单位 rad。
         */
        void set_gimbal(int seconds, double yaw) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.frame_id = "odom";
            transform.header.stamp = rclcpp::Time(seconds, 0);
            transform.child_frame_id = "gimbal_link";
            tf2::Quaternion orientation;
            orientation.setRPY(0.0, 0.0, yaw);
            transform.transform.rotation = tf2::toMsg(orientation);
            ASSERT_TRUE(buffer_->setTransform(transform, "test", false));
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<tf2_ros::Buffer> buffer_;
    };

    TEST_F(YawStabilityTest, YawPnPUsesImagePoseWhenLatestGimbalMoves) {
        geometry_msgs::msg::TransformStamped optical;
        optical.header.frame_id = "gimbal_link";
        optical.child_frame_id = "camera_optical_frame";
        optical.transform.rotation.x = 0.5;
        optical.transform.rotation.y = -0.5;
        optical.transform.rotation.z = 0.5;
        optical.transform.rotation.w = -0.5;
        ASSERT_TRUE(buffer_->setTransform(optical, "test", true));

        const double plate_yaw = 70.0 * M_PI / 180.0;
        auto armors = std::make_shared<rm_interfaces::msg::Armors>();
        rm_interfaces::msg::Armor armor;
        armor.number = "3";
        armor.type = "small";
        armor.pose.position.x = 2.0;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, ANGLE_UP_15, plate_yaw);
        armor.pose.orientation = tf2::toMsg(orientation);

        const cv::Mat camera_matrix =
            (cv::Mat_<double>(3, 3) << 1348, 0, 804, 0, 1348, 631, 0, 0, 1);
        const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
        CoordConverter projector;
        projector.set_camera_params(camera_matrix, distortion);
        projector.set_gimbal_rpy(Eigen::Vector3d::Zero());
        const Eigen::Quaterniond camera_to_world(-0.5, 0.5, -0.5, 0.5);
        projector.set_world2camera(
            camera_to_world.toRotationMatrix().transpose(),
            Eigen::Vector3d::Zero()
        );
        ArmorData data(armor);
        projector.update_projected_points(data, plate_yaw);
        for (std::size_t i = 0; i < armor.image_points.size(); ++i) {
            armor.image_points[i].x = data.projected_points[i].x;
            armor.image_points[i].y = data.projected_points[i].y;
        }
        armors->armors.push_back(armor);

        std_msgs::msg::Header image_header;
        image_header.frame_id = "camera_optical_frame";
        image_header.stamp = rclcpp::Time(10, 0);
        SolveYawPnP yaw_solver(node_);
        yaw_solver.set_camera_params(camera_matrix, distortion);
        yaw_solver.solve(armors, image_header, plate_yaw, "3", buffer_);
        const auto baseline_orientation = armors->armors.front().pose.orientation;

        // 图像与固定目标不变，仅模拟处理期间云台已转到另一个方向。
        armors->armors.front() = armor;
        set_gimbal(11, -60.0 * M_PI / 180.0);
        yaw_solver.solve(armors, image_header, plate_yaw, "3", buffer_);
        const auto moving_orientation = armors->armors.front().pose.orientation;
        EXPECT_NEAR(moving_orientation.x, baseline_orientation.x, 1e-12);
        EXPECT_NEAR(moving_orientation.y, baseline_orientation.y, 1e-12);
        EXPECT_NEAR(moving_orientation.z, baseline_orientation.z, 1e-12);
        EXPECT_NEAR(moving_orientation.w, baseline_orientation.w, 1e-12);
    }

    TEST_F(YawStabilityTest, OrdinaryTrackingKeepsFixedPointDespiteGimbalAndOrientationChanges) {
        Solver solver(node_);
        node_->set_parameter(rclcpp::Parameter("solver.use_armor_top", true));
        rm_interfaces::msg::Target target;
        target.header.frame_id = "odom";
        target.header.stamp = rclcpp::Time(10, 0);
        target.tracking = true;
        target.id = "3";
        target.armors_num = 4;
        target.position.x = 2.26;
        target.radius_list = { 0.26, 0.26, 0.26, 0.26 };
        target.dz_list = { 0.0, 0.0, 0.0, 0.0 };
        target.position_armor.x = 2.0;
        target.position_armor.y = 0.2;
        const double expected_yaw = std::atan2(0.2, 2.0) * 180.0 / M_PI;
        for (int i = -5; i <= 5; ++i) {
            set_gimbal(17 + i, i * 6.0 * M_PI / 180.0);
            target.yaw = i * 8.0 * M_PI / 180.0;
            target.v_yaw = i * 0.1;
            const auto command =
                solver.solve(target, rclcpp::Time(10, 40000000, RCL_ROS_TIME), buffer_);
            EXPECT_NEAR(command.yaw, expected_yaw, 1e-9);
            EXPECT_EQ(target.top_level, 0);
            EXPECT_DOUBLE_EQ(command.target_v_yaw, 0.0);
            EXPECT_GT(command.distance, 0.0);
        }

        // 平移跟随仍保留对运动目标的飞行时间预测。
        target.velocity_armor.y = 0.5;
        const auto moving = solver.solve(target, rclcpp::Time(10, 40000000, RCL_ROS_TIME), buffer_);
        EXPECT_GT(moving.yaw, expected_yaw + 1.0);
        EXPECT_LT(moving.yaw, expected_yaw + 10.0);
    }

    TEST_F(YawStabilityTest, RotatingModeRequiresDistinctConsistentImageFrames) {
        Solver solver(node_);
        node_->set_parameter(rclcpp::Parameter("solver.use_armor_top", true));
        rm_interfaces::msg::Target target;
        target.header.frame_id = "odom";
        target.tracking = true;
        target.id = "3";
        target.armors_num = 4;
        target.position.x = 2.26;
        target.position_armor.x = 2.0;
        target.radius_list = { 0.26, 0.26, 0.26, 0.26 };
        target.dz_list = { 0.0, 0.0, 0.0, 0.0 };

        auto solve_frame = [&](int frame, double v_yaw) {
            const auto nanoseconds = static_cast<uint32_t>(frame * 20000000);
            target.header.stamp = rclcpp::Time(10, nanoseconds);
            target.v_yaw = v_yaw;
            solver.solve(
                target,
                rclcpp::Time(10, nanoseconds + 40000000, RCL_ROS_TIME),
                buffer_
            );
        };

        solve_frame(0, 2.0);
        for (int i = 0; i < 10; ++i) {
            solve_frame(0, 2.0);
        }
        EXPECT_EQ(solver.get_top_level(), 0);

        solve_frame(1, -2.0);
        for (int frame = 2; frame <= 4; ++frame) {
            solve_frame(frame, 2.0);
            EXPECT_EQ(solver.get_top_level(), 0);
        }
        solve_frame(5, 2.0);
        EXPECT_EQ(solver.get_top_level(), 1);

        solver.reset_top_level();
        EXPECT_EQ(solver.get_top_level(), 0);
        solve_frame(6, 2.0);
        EXPECT_EQ(solver.get_top_level(), 0);
    }
    TEST_F(YawStabilityTest, MotionPredictionCanBeDisabledAndRestoredWithoutChangingTarget) {
        Solver solver(node_);
        rm_interfaces::msg::Target target;
        target.header.frame_id = "odom";
        target.header.stamp = rclcpp::Time(10, 0);
        target.tracking = true;
        target.id = "3";
        target.armors_num = 4;
        target.position.x = 2.26;
        target.position.y = 0.2;
        target.position_armor.x = 2.0;
        target.position_armor.y = 0.2;
        target.radius_list = { 0.26, 0.26, 0.26, 0.26 };
        target.dz_list = { 0.0, 0.0, 0.0, 0.0 };
        const auto current_time = rclcpp::Time(10, 40000000, RCL_ROS_TIME);

        // 同时验证装甲板位置跟随和整车模型，开关不应丢失弹道补偿或诊断速度。
        for (const bool use_armor_top: { true, false }) {
            node_->set_parameter(rclcpp::Parameter("solver.use_armor_top", use_armor_top));
            target.velocity_armor.y = 0.0;
            target.velocity.y = 0.0;
            const auto baseline = solver.solve(target, current_time, buffer_);

            target.velocity_armor.y = 0.5;
            target.velocity.y = 0.5;
            node_->set_parameter(rclcpp::Parameter("solver.additional_prediction_time", 0.2));
            node_->set_parameter(rclcpp::Parameter("solver.enable_motion_prediction", false));
            const auto disabled = solver.solve(target, current_time, buffer_);
            EXPECT_NEAR(disabled.yaw, baseline.yaw, 1e-9);
            EXPECT_NEAR(disabled.pitch, baseline.pitch, 1e-9);
            EXPECT_NEAR(disabled.distance, baseline.distance, 1e-9);
            EXPECT_GT(disabled.yaw_vel, 1.0);
            EXPECT_DOUBLE_EQ(target.velocity_armor.y, 0.5);
            EXPECT_DOUBLE_EQ(target.velocity.y, 0.5);

            node_->set_parameter(rclcpp::Parameter("solver.enable_motion_prediction", true));
            const auto restored = solver.solve(target, current_time, buffer_);
            EXPECT_GT(restored.yaw, baseline.yaw + 1.0);
            node_->set_parameter(rclcpp::Parameter("solver.additional_prediction_time", 0.0));
        }
    }
} // namespace
} // namespace qd::auto_aim
