#include <memory>

#include "armor_solver/armor_solver.hpp"
#include "gtest/gtest.h"

namespace qd::auto_aim {
class SolverMotionSwitches : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
        FYT_REGISTER_LOGGER("armor_solver", "/tmp/qyg-mpc-test-log", INFO);
    }
    virtual std::string mode() const { return "angle_command"; }
    void SetUp() override {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("solver.mpc.mode", mode()),
            rclcpp::Parameter("solver.use_armor_top", true),
            rclcpp::Parameter("solver.use_ballistic_compensation", false),
            rclcpp::Parameter("solver.compensator_type", "ideal")});
        node_ = std::make_shared<rclcpp::Node>("solver_mpc_test", options);
        node_->declare_parameter("gimble_frame", "gimbal_link");
        solver_ = std::make_unique<Solver>(node_);
        buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now_;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "gimbal_link";
        transform.transform.rotation.w = 1;
        ASSERT_TRUE(buffer_->setTransform(transform, "test", true));
        target_.header = transform.header;
        target_.tracking = true;
        target_.id = "3";
        target_.armors_num = 4;
        target_.position.x = 5.2;
        target_.position_armor.x = 5;
        target_.radius_list = {0.2, 0.2, 0.2, 0.2};
        target_.dz_list = {0, 0, 0, 0};
        feedback_.header = target_.header;
        feedback_.mode = 0;
        feedback_.bullet_speed = 16;
    }
    rm_interfaces::msg::GimbalCmd solve() {
        return solver_->solve(target_, now_, buffer_);
    }
    rclcpp::Time now_{100, 0, RCL_ROS_TIME};
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<Solver> solver_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    rm_interfaces::msg::Target target_;
    rm_interfaces::msg::SerialReceiveData feedback_;
};

TEST_F(SolverMotionSwitches, requiresFeedbackThenPublishesCompleteReference) {
    auto command = solve();
    EXPECT_FALSE(command.reference_valid);
    EXPECT_EQ(command.reference_status, "no_feedback");
    solver_->update_feedback(feedback_);
    command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_EQ(command.reference_status, "valid");
    EXPECT_NEAR(command.yaw, 0, 1e-8);
    EXPECT_NEAR(command.pitch, 0, 1e-8);
    EXPECT_NEAR(command.yaw_vel_deg_s, 0, 1e-8);
    EXPECT_NEAR(command.pitch_acc_deg_s2, 0, 1e-8);
    EXPECT_EQ(command.target_v_yaw, 0);
}

TEST_F(SolverMotionSwitches, plansBallisticPitchWithoutDerivativeFeedforward) {
    node_->set_parameter(rclcpp::Parameter("solver.use_ballistic_compensation", true));
    solver_->update_bullet_speed(16);
    solver_->update_feedback(feedback_);
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_LT(command.exp_pitch, 0);
    EXPECT_LT(command.pitch, 0);
    EXPECT_DOUBLE_EQ(command.pitch_vel_deg_s, 0);
    EXPECT_DOUBLE_EQ(command.pitch_acc_deg_s2, 0);
}

TEST_F(SolverMotionSwitches, targetSwitchUsesFreshOptimizationWithoutDifferencingJump) {
    solver_->update_feedback(feedback_);
    target_.position_armor.y = 1;
    const auto left = solve();
    ASSERT_TRUE(left.reference_valid) << left.reference_status;
    EXPECT_GT(left.yaw, 0);
    target_.position_armor.y = -1;
    const auto right = solve();
    ASSERT_TRUE(right.reference_valid) << right.reference_status;
    EXPECT_LT(right.yaw, 0);
    EXPECT_DOUBLE_EQ(right.yaw_acc_deg_s2, 0);
    EXPECT_DOUBLE_EQ(right.yaw_vel_deg_s, 0);
}

TEST_F(SolverMotionSwitches, repeatsDoNotRenewFeedbackAndStaleTargetIsRejected) {
    solver_->update_feedback(feedback_);
    now_ += rclcpp::Duration::from_seconds(0.2);
    target_.header.stamp = now_;
    solver_->update_feedback(feedback_);
    auto command = solve();
    EXPECT_FALSE(command.reference_valid);
    EXPECT_EQ(command.reference_status, "stale_feedback");
    feedback_.header.stamp = now_;
    solver_->update_feedback(feedback_);
    ASSERT_TRUE(solve().reference_valid);
    now_ += rclcpp::Duration::from_seconds(0.4);
    feedback_.header.stamp = now_;
    solver_->update_feedback(feedback_);
    command = solve();
    EXPECT_FALSE(command.reference_valid);
    EXPECT_EQ(command.reference_status, "stale_target");
    EXPECT_FALSE(command.fire_advice);
}

TEST_F(SolverMotionSwitches, rejectsEnergyModeAndFrameMismatch) {
    feedback_.mode = 2;
    solver_->update_feedback(feedback_);
    EXPECT_EQ(solve().reference_status, "not_armor_mode");
    feedback_.mode = 0;
    feedback_.header.frame_id = "other_frame";
    solver_->update_feedback(feedback_);
    const auto command = solve();
    EXPECT_EQ(command.reference_status, "frame_mismatch");
    EXPECT_FALSE(command.reference_valid);
}
class SolverAngleCommand : public SolverMotionSwitches {
protected:
    std::string mode() const override { return "angle_command"; }
};

TEST_F(SolverAngleCommand, publishesAngleWithZeroFeedforwardAndChecksActualAim) {
    solver_->update_feedback(feedback_);
    target_.position_armor.y = 1;
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_GT(command.yaw, 1.0);
    EXPECT_LE(command.yaw, 10.0);
    EXPECT_DOUBLE_EQ(command.yaw_vel_deg_s, 0);
    EXPECT_DOUBLE_EQ(command.pitch_vel_deg_s, 0);
    EXPECT_DOUBLE_EQ(command.yaw_acc_deg_s2, 0);
    EXPECT_DOUBLE_EQ(command.pitch_acc_deg_s2, 0);
    EXPECT_FALSE(command.fire_advice);
}

class SolverDirectAngle : public SolverMotionSwitches {
protected:
    std::string mode() const override { return "direct"; }
};

TEST_F(SolverDirectAngle, publishesValidDirectReference) {
    solver_->update_feedback(feedback_);
    target_.position_armor.y = 1;
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_EQ(command.reference_status, "valid_direct");
    EXPECT_GT(command.yaw, 1.0);
    EXPECT_DOUBLE_EQ(command.yaw_vel_deg_s, 0);
    EXPECT_DOUBLE_EQ(command.yaw_acc_deg_s2, 0);
}
} // namespace qd::auto_aim
