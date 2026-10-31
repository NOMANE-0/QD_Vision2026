#include <cmath>
#include <limits>
#include <vector>

#include "armor_solver/axis_mpc.hpp"
#include "gtest/gtest.h"

namespace qd::auto_aim {
TEST(AngleCommandMpc, holdsAndCommandsAnAngleInsteadOfFourMillisecondPosition) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    const auto hold = mpc.plan(10, std::vector<double>(21, 10));
    ASSERT_TRUE(hold);
    EXPECT_NEAR(hold->position, 10, 1e-6);
    const auto step = mpc.plan(0, std::vector<double>(21, 10));
    ASSERT_TRUE(step);
    EXPECT_GT(step->position, 1.0);
    EXPECT_LE(step->position, config.max_command_lead);
    EXPECT_DOUBLE_EQ(step->velocity, 0);
    EXPECT_DOUBLE_EQ(step->acceleration, 0);
}

TEST(AngleCommandMpc, boundsBothDirectionsAndCrossesSeam) {
    AxisMpc::Config config;
    config.max_command_lead = 5;
    AxisMpc mpc(config);
    for (double goal : {-30.0, 30.0}) {
        const auto step = mpc.plan(0, std::vector<double>(21, goal));
        ASSERT_TRUE(step);
        EXPECT_GT(step->position * goal, 0);
        EXPECT_LE(std::abs(step->position), 5);
    }
    const auto seam = mpc.plan(179, std::vector<double>(21, 181));
    ASSERT_TRUE(seam);
    EXPECT_GT(seam->position, 179);
    EXPECT_LE(seam->position, 184);
}

TEST(AngleCommandMpc, settlesThroughPositionControllerModel) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    double actual = 0;
    const double alpha = -std::expm1(-0.004 / config.response_time);
    for (int i = 0; i < 250; ++i) {
        const auto step = mpc.plan(actual, std::vector<double>(21, 10));
        ASSERT_TRUE(step) << i;
        EXPECT_LE(std::abs(step->position - actual), config.max_command_lead);
        actual += alpha * (step->position - actual);
    }
    EXPECT_NEAR(actual, 10, 0.01);
}

TEST(AngleCommandMpc, rejectsInvalidResponseAndExhaustedBudget) {
    AxisMpc::Config config;
    config.response_time = 0;
    AxisMpc invalid(config);
    EXPECT_FALSE(invalid.plan(0, std::vector<double>(21, 10)));
    config.response_time = 0.06;
    config.max_iterations = 1;
    AxisMpc limited(config);
    EXPECT_FALSE(limited.plan(0, std::vector<double>(21, 30)));
}
TEST(AngleCommandMpc, comparesAngleOnlyPlantAgainstDirect) {
    for (int scenario = 0; scenario < 3; ++scenario) {
        double errors[2] = {};
        double final_angles[2] = {};
        double first_commands[2] = {};
        for (int mode = 0; mode < 2; ++mode) {
            AxisMpc::Config config;
            AxisMpc mpc(config);
            double actual = 0;
            const double period = 0.004;
            const double alpha = -std::expm1(-period / 0.06);
            for (int i = 0; i < 250; ++i) {
                const auto goal = [scenario](double t) {
                    if (scenario == 0) return 10.0;
                    if (scenario == 1) return 20.0 * t;
                    return 10.0 * std::sin(2.0 * 3.141592653589793 * t);
                };
                std::vector<double> reference(21);
                for (int k = 0; k <= 20; ++k)
                    reference[k] = goal(i * period + k * config.dt);
                double command = reference[0];
                if (mode != 0) {
                    const auto step = mpc.plan(actual, reference);
                    ASSERT_TRUE(step) << scenario << " " << mode << " " << i;
                    command = step->position;
                }
                if (i == 0) first_commands[mode] = command;
                actual += alpha * (command - actual);
                const double error = actual - goal((i + 1) * period);
                errors[mode] += error * error;
            }
            final_angles[mode] = actual;
        }
        std::cout << "ANGLE_COMPARISON scenario=" << scenario
                  << " direct/angle RMSE=";
        for (double error : errors) std::cout << std::sqrt(error / 250) << " ";
        std::cout << " final=";
        for (double angle : final_angles) std::cout << angle << " ";
        std::cout << " first_command=";
        for (double command : first_commands) std::cout << command << " ";
        std::cout << std::endl;
    }
}
TEST(AngleCommandMpc, rejectsInvalidInputsAndConfiguration) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    EXPECT_FALSE(mpc.plan(0, {}));
    auto reference = std::vector<double>(21, 0);
    reference[3] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(mpc.plan(0, reference));
    EXPECT_FALSE(mpc.plan(std::numeric_limits<double>::infinity(), std::vector<double>(21, 0)));
    config.dt = 0;
    AxisMpc invalid(config);
    EXPECT_FALSE(invalid.plan(0, std::vector<double>(21, 0)));
}
} // namespace qd::auto_aim
