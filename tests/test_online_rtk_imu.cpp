#include <gtest/gtest.h>
#include <libgnss++/fusion/online_rtk_imu.hpp>
#include <limits>
#include <stdexcept>

using namespace libgnss;
namespace {
OnlineRtkImuProcessor::Config configuration() {
    OnlineRtkImuProcessor::Config config;
    config.base_position_ecef = Vector3d(6378137.0, 0.0, 0.0);
    config.fusion.align_static_window_s = 0.02;
    config.fusion.zupt_enable = false;
    return config;
}
GNSSTime time(double tow) { return GNSSTime(2200, tow); }
ObservationData epoch(double tow) { return ObservationData(time(tow)); }
ImuSample imu(double tow) {
    ImuSample sample;
    sample.time = time(tow);
    sample.accel_raw = Vector3d(0.0, 0.0, 9.80665);
    return sample;
}
TEST(OnlineRtkImuTest, FutureSourcesAndBackwardReceptionAreRejectedWithoutQueueMutation) {
    OnlineRtkImuProcessor processor(configuration());
    processor.pushBase(epoch(10.0), time(10.0));
    EXPECT_THROW(processor.pushBase(epoch(11.0), time(10.0)), std::invalid_argument);
    EXPECT_THROW(processor.pushImu(imu(11.0), time(10.0)), std::invalid_argument);
    EXPECT_THROW(processor.processRover(epoch(11.0), time(10.0)), std::invalid_argument);
    EXPECT_THROW(processor.pushNavigation(NavigationData{}, time(9.0)), std::invalid_argument);
    EXPECT_EQ(processor.pendingBase(), 1U);
    EXPECT_EQ(processor.pendingImu(), 0U);
    EXPECT_EQ(processor.diagnostics().rover_epochs, 0U);
}
TEST(OnlineRtkImuTest, ReceivedFutureBaseCannotInterpolateEarlierRover) {
    OnlineRtkImuProcessor processor(configuration());
    processor.pushBase(epoch(10.0), time(10.0));
    processor.pushBase(epoch(11.0), time(11.0));
    const auto first = processor.processRover(epoch(10.2), time(11.0));
    EXPECT_FALSE(first.exact_base_available);
    EXPECT_EQ(first.reason, "missing_exact_base");
    EXPECT_EQ(processor.pendingBase(), 1U);
    EXPECT_EQ(processor.diagnostics().expired_base_epochs, 1U);
    const auto second = processor.processRover(epoch(11.0), time(11.0));
    EXPECT_TRUE(second.exact_base_available);
    EXPECT_EQ(processor.pendingBase(), 0U);
}
TEST(OnlineRtkImuTest, FutureImuStaysQueuedUntilItsEpoch) {
    OnlineRtkImuProcessor processor(configuration());
    for (double t : {10.0, 10.01, 10.02, 10.03}) processor.pushImu(imu(t), time(10.03));
    const auto first = processor.processRover(epoch(10.02), time(10.03));
    EXPECT_EQ(first.imu_consumed, 3U);
    EXPECT_EQ(processor.pendingImu(), 1U);
    EXPECT_FALSE(first.fused.isValid());
    const auto second = processor.processRover(epoch(10.03), time(10.03));
    EXPECT_EQ(second.imu_consumed, 1U);
    EXPECT_EQ(processor.pendingImu(), 0U);
}
TEST(OnlineRtkImuTest, LateBaseAndImuCannotReviseAnEmittedEpoch) {
    OnlineRtkImuProcessor processor(configuration());
    const auto first = processor.processRover(epoch(10.0), time(10.1));
    processor.pushBase(epoch(10.0), time(10.2));
    EXPECT_FALSE(processor.pushImu(imu(10.0), time(10.2)));
    EXPECT_EQ(processor.pendingBase(), 0U);
    EXPECT_EQ(processor.pendingImu(), 0U);
    EXPECT_EQ(processor.diagnostics().expired_base_epochs, 1U);
    EXPECT_EQ(processor.diagnostics().late_imu_dropped, 1U);
    EXPECT_FALSE(first.exact_base_available);
    EXPECT_DOUBLE_EQ(first.input_age_s, 10.1 - 10.0);
    EXPECT_THROW(processor.processRover(epoch(10.0), time(10.2)), std::invalid_argument);
}
TEST(OnlineRtkImuTest, GapReinitializationDoesNotReuseAlignmentOrTimeUpdate) {
    OnlineRtkImuProcessor processor(configuration());
    for (double t : {10.0, 10.01, 10.02, 10.03}) processor.pushImu(imu(t), time(t));
    processor.processRover(epoch(10.03), time(10.03));
    processor.pushImu(imu(10.5), time(10.5));
    const auto after_gap = processor.processRover(epoch(10.5), time(10.5));
    EXPECT_EQ(after_gap.reset_generation, 1U);
    EXPECT_EQ(after_gap.reason, "imu_gap_reset");
    EXPECT_FALSE(after_gap.fusion_initialized);
    EXPECT_FALSE(after_gap.fused.isValid());
    EXPECT_FALSE(after_gap.tight_time_update_supplied);
    EXPECT_EQ(processor.diagnostics().imu_gap_resets, 1U);
}
TEST(OnlineRtkImuTest, StaleImuAndRoverOutageResetOncePerDiscontinuity) {
    OnlineRtkImuProcessor processor(configuration());
    processor.pushImu(imu(10.0), time(10.0));
    processor.processRover(epoch(10.0), time(10.0));
    const auto stale = processor.processRover(epoch(10.2), time(10.2));
    EXPECT_EQ(stale.reason, "imu_stale_reset");
    EXPECT_EQ(stale.reset_generation, 1U);
    processor.processRover(epoch(10.4), time(10.4));
    EXPECT_EQ(processor.diagnostics().imu_gap_resets, 1U);
    const auto outage = processor.processRover(epoch(14.0), time(14.5));
    EXPECT_EQ(outage.reason, "rover_gap_reset");
    EXPECT_EQ(outage.reset_generation, 2U);
    EXPECT_EQ(processor.diagnostics().rover_gap_resets, 1U);
    EXPECT_DOUBLE_EQ(outage.input_age_s, 0.5);
}
TEST(OnlineRtkImuTest, CapacityAndInvalidImuFailBeforeAcceptance) {
    auto config = configuration();
    config.max_pending_imu = 1;
    config.max_pending_base = 1;
    OnlineRtkImuProcessor processor(config);
    auto invalid = imu(10.0);
    invalid.gyro_raw_radps.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(processor.pushImu(invalid, time(10.0)), std::invalid_argument);
    EXPECT_EQ(processor.pendingImu(), 0U);
    processor.pushImu(imu(10.0), time(10.0));
    EXPECT_THROW(processor.pushImu(imu(10.01), time(10.01)), std::length_error);
    processor.pushBase(epoch(10.0), time(10.0));
    EXPECT_THROW(processor.pushBase(epoch(10.0), time(10.0)), std::invalid_argument);
    EXPECT_THROW(processor.pushBase(epoch(10.01), time(10.01)), std::length_error);
}
TEST(OnlineRtkImuTest, ResetClearsPendingDataAndPreservesReceptionWatermark) {
    OnlineRtkImuProcessor processor(configuration());
    processor.pushImu(imu(10.0), time(10.0));
    processor.pushBase(epoch(10.0), time(10.0));
    processor.reset(time(10.1));
    EXPECT_EQ(processor.pendingBase(), 0U);
    EXPECT_EQ(processor.pendingImu(), 0U);
    EXPECT_EQ(processor.diagnostics().reset_generation, 1U);
    EXPECT_THROW(processor.pushBase(epoch(10.0), time(10.0)), std::invalid_argument);
    processor.pushBase(epoch(10.0), time(10.1));
    EXPECT_TRUE(processor.processRover(epoch(10.0), time(10.1)).exact_base_available);
}
TEST(OnlineRtkImuTest, SamePrefixIsUnaffectedByDifferentSuffixes) {
    OnlineRtkImuProcessor left(configuration()), right(configuration());
    for (double t : {10.0, 10.01, 10.02}) {
        left.pushImu(imu(t), time(t));
        right.pushImu(imu(t), time(t));
    }
    const auto a = left.processRover(epoch(10.02), time(10.04));
    const auto b = right.processRover(epoch(10.02), time(10.04));
    left.pushBase(epoch(11.0), time(11.0));
    right.reset(time(11.0));
    EXPECT_EQ(a.imu_consumed, b.imu_consumed);
    EXPECT_EQ(a.reset_generation, b.reset_generation);
    EXPECT_EQ(a.exact_base_available, b.exact_base_available);
    EXPECT_EQ(a.rtk.status, b.rtk.status);
    EXPECT_EQ(a.fused.status, b.fused.status);
    EXPECT_EQ(a.reason, b.reason);
    EXPECT_DOUBLE_EQ(a.input_age_s, b.input_age_s);
    // Position parity on usable solutions additionally needs raw PPC replay.
}
}
