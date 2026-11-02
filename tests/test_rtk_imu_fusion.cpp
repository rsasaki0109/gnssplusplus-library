#include <gtest/gtest.h>

#include <libgnss++/fusion/rtk_imu_fusion.hpp>

namespace libgnss {
namespace {

const Eigen::Vector3d kBasePositionEcef{-3961904.9530, 3348993.7578, 3698211.7553};

TEST(RtkImuFusionProcessorTest, DefaultConfigLeavesTheRtkFilterUncoupled) {
    RTKProcessor::RTKConfig rtk_config;
    // Caller values of the coupling fields are replaced; the rest is kept.
    rtk_config.use_external_position_prior = true;
    rtk_config.enable_cp_pr_fixed_gate = true;
    rtk_config.ratio_threshold = 2.4;
    const RtkImuFusionProcessor processor(RtkImuFusionProcessor::Config{}, rtk_config,
                                          kBasePositionEcef);
    const auto& applied = processor.rtk().getRTKConfig();
    EXPECT_FALSE(applied.use_external_position_prior);
    EXPECT_FALSE(applied.use_external_position_time_update);
    EXPECT_FALSE(applied.enable_velocity_states);
    EXPECT_FALSE(applied.enable_tdcp_diagnostics);
    EXPECT_FALSE(applied.enable_cp_pr_fixed_gate);
    EXPECT_DOUBLE_EQ(applied.ratio_threshold, 2.4);
}

TEST(RtkImuFusionProcessorTest, ClosedLoopConfigCouplesTheRtkFilter) {
    RtkImuFusionProcessor::Config config;
    config.tc_closed_loop = true;
    config.tc_velocity_states = true;
    config.tc_tdcp_diagnostics = true;
    const RtkImuFusionProcessor processor(config, RTKProcessor::RTKConfig{},
                                          kBasePositionEcef);
    const auto& applied = processor.rtk().getRTKConfig();
    EXPECT_FALSE(applied.use_external_position_prior);
    EXPECT_TRUE(applied.use_external_position_time_update);
    EXPECT_TRUE(applied.enable_velocity_states);
    EXPECT_TRUE(applied.enable_tdcp_diagnostics);
    EXPECT_TRUE(applied.enable_cp_pr_fixed_gate);
}

TEST(RtkImuFusionProcessorTest, InsPriorAndTimeUpdateSetTheirOwnRtkFields) {
    RtkImuFusionProcessor::Config prior_config;
    prior_config.rtk_ins_prior = true;
    const RtkImuFusionProcessor prior(prior_config, RTKProcessor::RTKConfig{},
                                      kBasePositionEcef);
    EXPECT_TRUE(prior.rtk().getRTKConfig().use_external_position_prior);
    EXPECT_FALSE(prior.rtk().getRTKConfig().use_external_position_time_update);

    RtkImuFusionProcessor::Config time_update_config;
    time_update_config.tc_ins_time_update = true;
    const RtkImuFusionProcessor time_update(time_update_config, RTKProcessor::RTKConfig{},
                                            kBasePositionEcef);
    EXPECT_FALSE(time_update.rtk().getRTKConfig().use_external_position_prior);
    EXPECT_TRUE(time_update.rtk().getRTKConfig().use_external_position_time_update);
    EXPECT_FALSE(time_update.rtk().getRTKConfig().enable_cp_pr_fixed_gate);
}

TEST(RtkImuFusionProcessorTest, EpochWithoutObservationsIsInvalidAndHasNoOutput) {
    RtkImuFusionProcessor::Config config;
    config.tc_ins_time_update = true;
    config.tc_closed_loop = true;
    config.independent_output_filter = true;
    RtkImuFusionProcessor processor(config, RTKProcessor::RTKConfig{}, kBasePositionEcef);
    ObservationData rover;
    rover.time = GNSSTime(2200, 100.0);
    ObservationData base;
    base.time = rover.time;
    const NavigationData nav;
    const auto result = processor.processEpoch(rover, base, nav);
    EXPECT_FALSE(result.rtk_valid);
    EXPECT_FALSE(result.rtk.isValid());
    EXPECT_FALSE(result.has_output);
    EXPECT_EQ(processor.counters().tc_update_supplied_count, 0);
    EXPECT_EQ(processor.counters().tc_anchor_count, 0);
    EXPECT_FALSE(processor.fusion().isInitialized());
    EXPECT_FALSE(processor.outputFilter().isOriginSet());
    EXPECT_EQ(processor.closedLoop().diagnostics().anchors, 0u);
}

}  // namespace
}  // namespace libgnss
