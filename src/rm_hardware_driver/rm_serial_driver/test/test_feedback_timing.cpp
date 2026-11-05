#include <cmath>
#include <cstdint>
#include <limits>

#include "rm_serial_driver/feedback_pose_aligner.hpp"
#include "rm_utils/time_synchronizer.hpp"
#include "gtest/gtest.h"

using qd::serial_driver::decode_yaw_data_age;
using qd::serial_driver::FeedbackPoseAligner;
using qd::utils::TimeSynchronizer;

TEST(FeedbackTiming, SupportsLegacyAndTaggedAge) {
    const auto legacy = decode_yaw_data_age(0);
    EXPECT_FALSE(legacy.supported);
    EXPECT_TRUE(legacy.valid);
    const auto fresh = decode_yaw_data_age(0x80);
    EXPECT_TRUE(fresh.supported);
    EXPECT_TRUE(fresh.valid);
    EXPECT_EQ(fresh.milliseconds, 0);
    const auto delayed = decode_yaw_data_age(0x93);
    EXPECT_TRUE(delayed.valid);
    EXPECT_EQ(delayed.milliseconds, 19);
    EXPECT_TRUE(decode_yaw_data_age(0xfe).valid);
    EXPECT_FALSE(decode_yaw_data_age(0xff).valid);
}

TEST(FeedbackTiming, PreservesLegacyPose) {
    FeedbackPoseAligner aligner;
    const auto pose = aligner.update(1000000000LL, 100, 2.0, 3.0, 4.0, 0);
    ASSERT_TRUE(pose);
    EXPECT_EQ(pose->stamp_ns, 1000000000LL);
    EXPECT_DOUBLE_EQ(pose->roll_deg, 2.0);
    EXPECT_DOUBLE_EQ(pose->pitch_deg, 3.0);
    EXPECT_DOUBLE_EQ(pose->yaw_deg, 4.0);
}

TEST(FeedbackTiming, AlignsDelayedYawWithHistoricalPitch) {
    FeedbackPoseAligner aligner;
    ASSERT_TRUE(aligner.update(1000000000LL, 100, 0.0, 0.0, 4.0, 0x80));
    // pitch 在 20 ms 内从 0 到 20 度；yaw 对应中点，而非当前 pitch。
    const auto pose = aligner.update(1020000000LL, 120, 10.0, 20.0, 8.0, 0x8a);
    ASSERT_TRUE(pose);
    EXPECT_EQ(pose->stamp_ns, 1010000000LL);
    EXPECT_DOUBLE_EQ(pose->roll_deg, 5.0);
    EXPECT_DOUBLE_EQ(pose->pitch_deg, 10.0);
    EXPECT_DOUBLE_EQ(pose->yaw_deg, 8.0);
}

TEST(FeedbackTiming, DoesNotRestampRepeatedYaw) {
    FeedbackPoseAligner aligner;
    ASSERT_TRUE(aligner.update(1000000000LL, 100, 0.0, 0.0, 4.0, 0x80));
    for (uint32_t age = 2; age <= 20; age += 2) {
        EXPECT_FALSE(aligner.update(
            1000000000LL + age * 1000000LL,
            100 + age,
            0.0,
            1.0,
            4.0,
            static_cast<uint8_t>(0x80U | age)
        ));
    }
    ASSERT_TRUE(aligner.update(1022000000LL, 122, 0.0, 2.0, 6.0, 0x80));
}

TEST(FeedbackTiming, RejectsInvalidAgeAndMissingHistory) {
    FeedbackPoseAligner aligner;
    EXPECT_FALSE(aligner.update(1000000000LL, 100, 0.0, 0.0, 4.0, 0xff));
    EXPECT_FALSE(aligner.update(1002000000LL, 102, 0.0, 2.0, 4.0, 0x93));
    const auto pose = aligner.update(1004000000LL, 104, 0.0, 4.0, 5.0, 0x82);
    ASSERT_TRUE(pose);
    EXPECT_EQ(pose->stamp_ns, 1002000000LL);
    EXPECT_DOUBLE_EQ(pose->pitch_deg, 2.0);
}

TEST(FeedbackTiming, HandlesMcuWrapAndRestart) {
    FeedbackPoseAligner aligner;
    ASSERT_TRUE(aligner.update(1000000000LL, 0xfffffffeU, 0.0, 1.0, 2.0, 0x80));
    ASSERT_TRUE(aligner.update(1004000000LL, 2, 0.0, 2.0, 3.0, 0x80));
    ASSERT_TRUE(aligner.update(2000000000LL, 0, 0.0, 3.0, 4.0, 0x80));
}

TEST(FeedbackTiming, InterpolatesAcrossAngleWrap) {
    FeedbackPoseAligner aligner;
    ASSERT_TRUE(aligner.update(1000000000LL, 100, 179.0, 0.0, 0.0, 0x80));
    const auto pose = aligner.update(1004000000LL, 104, -179.0, 0.0, 1.0, 0x82);
    ASSERT_TRUE(pose);
    EXPECT_DOUBLE_EQ(pose->roll_deg, 180.0);
}

TEST(FeedbackTiming, KeepsHistoryBoundedAndRejectsOlderSamples) {
    FeedbackPoseAligner aligner;
    for (uint32_t i = 0; i < 500; ++i) {
        ASSERT_TRUE(aligner.update(1000000000LL + i * 2000000LL, 100 + i * 2, 0, 0, 0, 0));
    }
    EXPECT_FALSE(aligner.update(2000000000LL, 1100, 0, 0, 0, 0xfe));
    ASSERT_TRUE(aligner.update(2002000000LL, 1102, 0, 0, 0, 0x80));
}

TEST(TimeSynchronizer, RepeatedInitialStampHasInitializedTime) {
    TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(100, 1000.0), 1000.0);
    EXPECT_DOUBLE_EQ(sync.sync(100, 1000.02), 1000.0);
    sync.reset();
    EXPECT_DOUBLE_EQ(sync.sync(200, 2000.0), 2000.0);
    EXPECT_DOUBLE_EQ(sync.sync(200, 2000.02), 2000.0);
}

TEST(TimeSynchronizer, ReanchorsAfterMcuRestart) {
    TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(50000, 1000.0), 1000.0);
    EXPECT_NEAR(sync.sync(50002, 1000.002), 1000.002, 1e-9);
    EXPECT_DOUBLE_EQ(sync.sync(1, 1001.0), 1001.0);
    EXPECT_NEAR(sync.sync(3, 1001.002), 1001.002, 1e-6);
}

TEST(TimeSynchronizer, PreservesNormalCounterWrap) {
    TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(0xfffffffeU, 1000.0), 1000.0);
    EXPECT_NEAR(sync.sync(2, 1000.004), 1000.004, 1e-9);
    EXPECT_DOUBLE_EQ(sync.sync(2, 1000.1), 1000.004);
}
