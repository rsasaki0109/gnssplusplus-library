#include <gtest/gtest.h>

#include <libgnss++/fusion/tight_coupling_processor.hpp>

namespace libgnss {
namespace {

FusionState bootstrapState() {
    FusionState state;
    state.nominal.time = GNSSTime(2200, 100.0);
    state.nominal.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    state.nominal.accel_bias = {0.01, -0.02, 0.03};
    state.nominal.gyro_bias = {0.001, -0.002, 0.003};
    state.covariance.setIdentity();
    return state;
}

ImuSample stationarySample(double tow) {
    ImuSample sample;
    sample.time = GNSSTime(2200, tow);
    sample.accel_raw = {0.01, -0.02, kStandardGravityMps2 + 0.03};
    sample.gyro_raw_radps = {0.001, -0.002, 0.003};
    return sample;
}

TEST(TightCouplingProcessorTest, ProducesZeroStationaryAntennaIncrement) {
    TightCouplingProcessor::Config config;
    config.lever_arm_body = {0.3, 0.0, 0.5};
    config.zupt_enable = false;
    TightCouplingProcessor processor(config);
    const Eigen::Vector3d antenna{-3961765.0, 3349009.0, 3698311.0};
    const FusionState bootstrap = bootstrapState();
    ASSERT_TRUE(processor.reanchor(antenna, Eigen::Matrix3d::Identity(),
                                   Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(),
                                   bootstrap.nominal.time, &bootstrap));
    for (int i = 1; i <= 20; ++i) processor.processImuSample(stationarySample(100.0 + i * 0.01));
    const auto update = processor.prepareTimeUpdate();
    ASSERT_TRUE(update.valid);
    EXPECT_LT(update.antenna_delta_ecef.norm(), 1e-8);
    EXPECT_LT(update.antenna_velocity_ecef.norm(), 1e-8);
    EXPECT_TRUE(update.process_noise_ecef.allFinite());
    EXPECT_TRUE(update.position_velocity_process_noise_ecef.allFinite());
    EXPECT_TRUE(update.velocity_covariance_ecef.allFinite());
}

TEST(TightCouplingProcessorTest, LaterReanchorRetainsPrivateBiasState) {
    TightCouplingProcessor::Config config;
    config.zupt_enable = false;
    TightCouplingProcessor processor(config);
    const Eigen::Vector3d antenna{-3961765.0, 3349009.0, 3698311.0};
    FusionState bootstrap = bootstrapState();
    ASSERT_TRUE(processor.reanchor(antenna, Eigen::Matrix3d::Identity(),
                                   Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(),
                                   bootstrap.nominal.time, &bootstrap));
    processor.processImuSample(stationarySample(100.01));
    ASSERT_TRUE(processor.prepareTimeUpdate().valid);
    FusionState hostile = bootstrap;
    hostile.nominal.accel_bias.setConstant(9.0);
    hostile.nominal.gyro_bias.setConstant(8.0);
    ASSERT_TRUE(processor.reanchor(antenna, Eigen::Matrix3d::Identity(),
                                   Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(),
                                   GNSSTime(2200, 100.01), &hostile));
    EXPECT_TRUE(processor.state().nominal.accel_bias.isApprox(bootstrap.nominal.accel_bias));
    EXPECT_TRUE(processor.state().nominal.gyro_bias.isApprox(bootstrap.nominal.gyro_bias));
}

TEST(TightCouplingProcessorTest, GapInvalidatesWholeInterval) {
    TightCouplingProcessor processor;
    const Eigen::Vector3d antenna{-3961765.0, 3349009.0, 3698311.0};
    const FusionState bootstrap = bootstrapState();
    ASSERT_TRUE(processor.reanchor(antenna, Eigen::Matrix3d::Identity(),
                                   Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(),
                                   bootstrap.nominal.time, &bootstrap));
    processor.processImuSample(stationarySample(100.2));
    EXPECT_FALSE(processor.prepareTimeUpdate().valid);
    EXPECT_EQ(processor.diagnostics().invalid_intervals, 1u);
}

TEST(TightCouplingProcessorTest, OwnsZuptAndNhcConstraintUpdates) {
    TightCouplingProcessor::Config config;
    config.nhc_enable = true;
    TightCouplingProcessor processor(config);
    const Eigen::Vector3d antenna{-3961765.0, 3349009.0, 3698311.0};
    const FusionState bootstrap = bootstrapState();
    ASSERT_TRUE(processor.reanchor(antenna, Eigen::Matrix3d::Identity(),
                                   Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(),
                                   bootstrap.nominal.time, &bootstrap));
    for (int i = 1; i <= 20; ++i) processor.processImuSample(stationarySample(100.0 + i * 0.01));
    ASSERT_TRUE(processor.prepareTimeUpdate().valid);
    EXPECT_EQ(processor.diagnostics().zupt_updates, 1u);
    EXPECT_EQ(processor.diagnostics().nhc_updates, 1u);
}

}  // namespace
}  // namespace libgnss
