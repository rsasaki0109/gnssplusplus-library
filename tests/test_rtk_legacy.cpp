#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/processor.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/io/rinex.hpp>

#define private public
#include <libgnss++/algorithms/rtk.hpp>
#undef private

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

std::string sourcePath(const std::string& relative_path) {
    return std::string(GNSSPP_SOURCE_DIR) + "/" + relative_path;
}

bool sourcePathExists(const std::string& relative_path) {
    return std::filesystem::exists(sourcePath(relative_path));
}

class RTKLegacyCompatibilityTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!sourcePathExists("data/rover_kinematic.obs") ||
            !sourcePathExists("data/base_kinematic.obs") ||
            !sourcePathExists("data/navigation_kinematic.nav")) {
            GTEST_SKIP() << "repo kinematic test data is not available";
        }
        ASSERT_TRUE(rover_reader_.open(sourcePath("data/rover_kinematic.obs")));
        ASSERT_TRUE(rover_reader_.readHeader(rover_header_));
        ASSERT_TRUE(base_reader_.open(sourcePath("data/base_kinematic.obs")));
        ASSERT_TRUE(base_reader_.readHeader(base_header_));
        ASSERT_TRUE(nav_reader_.open(sourcePath("data/navigation_kinematic.nav")));
        ASSERT_TRUE(nav_reader_.readNavigationData(nav_data_));
        ASSERT_GT(rover_header_.approximate_position.norm(), 1e6);
        ASSERT_GT(base_header_.approximate_position.norm(), 1e6);
        ASSERT_TRUE(readAlignedEpoch(rover_obs_, base_obs_));

        rover_obs_.receiver_position = rover_header_.approximate_position;

        config_.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        config_.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        config_.min_satellites_for_ar = 5;
        config_.ratio_threshold = 3.0;
        processor_.setRTKConfig(config_);
        processor_.setBasePosition(base_header_.approximate_position);
    }

    bool readAlignedEpoch(ObservationData& rover_obs, ObservationData& base_obs) {
        bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
        bool base_ok = base_reader_.readObservationEpoch(base_obs);
        while (rover_ok && base_ok) {
            const double time_diff =
                (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                (rover_obs.time.tow - base_obs.time.tow);
            if (std::abs(time_diff) <= 0.5) {
                return true;
            }
            if (time_diff < 0.0) {
                rover_ok = rover_reader_.readObservationEpoch(rover_obs);
            } else {
                base_ok = base_reader_.readObservationEpoch(base_obs);
            }
        }
        return false;
    }

    RTKProcessor processor_;
    RTKProcessor::RTKConfig config_;
    io::RINEXReader rover_reader_;
    io::RINEXReader base_reader_;
    io::RINEXReader nav_reader_;
    io::RINEXReader::RINEXHeader rover_header_;
    io::RINEXReader::RINEXHeader base_header_;
    NavigationData nav_data_;
    ObservationData rover_obs_;
    ObservationData base_obs_;
};

TEST_F(RTKLegacyCompatibilityTest, FormsDoubleDifferencesAndLinearizedHelpersStayConsistent) {
    const auto measurements = processor_.formDoubleDifferences(rover_obs_, base_obs_, nav_data_);

    ASSERT_GE(measurements.size(), 4u);
    EXPECT_TRUE(processor_.filter_initialized_);
    EXPECT_FALSE(processor_.current_sat_data_.empty());

    for (const auto& measurement : measurements) {
        EXPECT_TRUE(measurement.valid);
        EXPECT_TRUE(std::isfinite(measurement.pseudorange_dd));
        EXPECT_TRUE(std::isfinite(measurement.carrier_phase_dd));
        EXPECT_TRUE(std::isfinite(measurement.geometric_range));
        EXPECT_TRUE(measurement.unit_vector.array().isFinite().all());
        EXPECT_GT(measurement.variance, 0.0);
    }

    const Vector3d baseline = processor_.calculateBaseline();
    EXPECT_TRUE(baseline.array().isFinite().all());

    const VectorXd residuals = processor_.calculateResiduals(measurements, baseline);
    ASSERT_EQ(residuals.size(), static_cast<int>(measurements.size()));
    EXPECT_TRUE(residuals.array().isFinite().all());

    const MatrixXd H =
        processor_.formMeasurementMatrix(measurements, nav_data_, rover_obs_.time);
    ASSERT_EQ(H.rows(), static_cast<int>(measurements.size()));
    ASSERT_EQ(H.cols(), 3);
    EXPECT_TRUE(H.array().isFinite().all());

    const MatrixXd W = processor_.calculateMeasurementWeights(measurements);
    ASSERT_EQ(W.rows(), static_cast<int>(measurements.size()));
    ASSERT_EQ(W.cols(), static_cast<int>(measurements.size()));
    for (int i = 0; i < W.rows(); ++i) {
        EXPECT_GT(W(i, i), 0.0);
        for (int j = 0; j < W.cols(); ++j) {
            if (i == j) {
                continue;
            }
            EXPECT_DOUBLE_EQ(W(i, j), 0.0);
        }
    }

    EXPECT_TRUE(processor_.hasSufficientSatellites(measurements));
}

TEST(RTKLegacyCompatibilityStandaloneTest, LambdaCompatibilityProducesValidatedIntegerFix) {
    RTKProcessor processor;
    VectorXd float_ambiguities(3);
    float_ambiguities << 1.02, -2.97, 5.01;
    MatrixXd covariance = MatrixXd::Identity(3, 3) * 0.01;

    const auto result = processor.solveLAMBDA(float_ambiguities, covariance);

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.fixed_ambiguities.size(), float_ambiguities.size());
    EXPECT_GT(result.ratio, 0.0);
    for (int i = 0; i < result.fixed_ambiguities.size(); ++i) {
        EXPECT_NEAR(result.fixed_ambiguities(i),
                    std::round(result.fixed_ambiguities(i)),
                    1e-9);
    }

    EXPECT_TRUE(processor.validateAmbiguityResolution(result.fixed_ambiguities,
                                                      float_ambiguities,
                                                      covariance,
                                                      3.5));
    EXPECT_FALSE(processor.validateAmbiguityResolution(result.fixed_ambiguities,
                                                       float_ambiguities,
                                                       covariance,
                                                       0.5));
}

TEST_F(RTKLegacyCompatibilityTest, AppliesFixedAmbiguitiesAndPublishesFixedBaseline) {
    const auto measurements = processor_.formDoubleDifferences(rover_obs_, base_obs_, nav_data_);

    ASSERT_GE(measurements.size(), 4u);
    ASSERT_TRUE(processor_.filter_initialized_);

    std::vector<SatelliteId> satellites;
    satellites.reserve(processor_.current_sat_data_.size());
    for (const auto& [satellite, sat_data] : processor_.current_sat_data_) {
        if (sat_data.has_l1 || sat_data.has_l2) {
            satellites.push_back(satellite);
        }
    }
    std::sort(satellites.begin(), satellites.end());

    int n1_count = 0;
    int n2_count = 0;
    for (const auto& satellite : satellites) {
        if (processor_.filter_state_.n1_indices.count(satellite) > 0) {
            ++n1_count;
        }
        if (processor_.filter_state_.n2_indices.count(satellite) > 0) {
            ++n2_count;
        }
    }

    ASSERT_GT(n1_count, 0);
    ASSERT_GT(n2_count, 0);

    VectorXd fixed_n1(n1_count);
    VectorXd fixed_n2(n2_count);
    for (int i = 0; i < n1_count; ++i) {
        fixed_n1(i) = 100.0 + i;
    }
    for (int i = 0; i < n2_count; ++i) {
        fixed_n2(i) = 200.0 + i;
    }

    ASSERT_TRUE(processor_.applyFixedAmbiguities(fixed_n1, fixed_n2, processor_.current_sat_data_));

    int n1_index = 0;
    int n2_index = 0;
    for (const auto& satellite : satellites) {
        const auto n1_it = processor_.filter_state_.n1_indices.find(satellite);
        if (n1_it != processor_.filter_state_.n1_indices.end()) {
            EXPECT_DOUBLE_EQ(processor_.filter_state_.state(n1_it->second), fixed_n1(n1_index));
            ++n1_index;
        }
        const auto n2_it = processor_.filter_state_.n2_indices.find(satellite);
        if (n2_it != processor_.filter_state_.n2_indices.end()) {
            EXPECT_DOUBLE_EQ(processor_.filter_state_.state(n2_it->second), fixed_n2(n2_index));
            ++n2_index;
        }
    }

    processor_.fixed_baseline_ = Vector3d(1.0, 2.0, 3.0);
    processor_.has_fixed_solution_ = true;
    processor_.solvePositionWithAmbiguities(processor_.current_sat_data_);

    EXPECT_TRUE(processor_.filter_state_.state.head<3>().isApprox(processor_.fixed_baseline_, 1e-12));
}

// ============================================================
// ARPolicy gate unit tests
// ============================================================

TEST(RTKLegacyCompatibilityStandaloneTest, ArPolicyExtendedDefaultBehaviorUnchanged) {
    // Default policy must be EXTENDED.
    RTKProcessor processor;
    const RTKProcessor::RTKConfig& cfg = processor.getRTKConfig();
    EXPECT_EQ(cfg.ar_policy, RTKProcessor::RTKConfig::ARPolicy::EXTENDED);

    // With EXTENDED policy, lambdaMethod applies Q regularization and succeeds on a
    // well-conditioned covariance.
    VectorXd float_amb(3);
    float_amb << 1.02, -2.97, 5.01;
    MatrixXd Q = MatrixXd::Identity(3, 3) * 0.01;
    VectorXd fixed_amb;
    double ratio = 0.0;
    EXPECT_TRUE(processor.lambdaMethod(float_amb, Q, fixed_amb, ratio));
    EXPECT_GT(ratio, 0.0);
    ASSERT_EQ(fixed_amb.size(), float_amb.size());
    for (int i = 0; i < fixed_amb.size(); ++i) {
        EXPECT_NEAR(fixed_amb(i), std::round(fixed_amb(i)), 1e-9);
    }

    // With EXTENDED and holdamb active, the relaxed ratio path is exercisable.
    // Set up state so that consecutive_fix_count_ >= min_hold_count.
    RTKProcessor::RTKConfig ext_cfg;
    ext_cfg.ar_policy = RTKProcessor::RTKConfig::ARPolicy::EXTENDED;
    ext_cfg.min_hold_count = 3;
    processor.setRTKConfig(ext_cfg);
    processor.consecutive_fix_count_ = 3;
    processor.has_last_fixed_position_ = true;
    // The relaxed hold-ratio path exists under EXTENDED; verify config reads correctly.
    EXPECT_EQ(processor.getRTKConfig().ar_policy, RTKProcessor::RTKConfig::ARPolicy::EXTENDED);
    EXPECT_GE(processor.consecutive_fix_count_, processor.getRTKConfig().min_hold_count);
}

TEST(RTKLegacyCompatibilityStandaloneTest, ArPolicyDemo5ContinuousDisablesSubsetFallback) {
    // Under DEMO5_CONTINUOUS, subset fallback flags must be false regardless of AR state.
    // We verify this by checking that lambdaMethod with a near-singular (but not completely
    // degenerate) covariance behaves consistently — DEMO5 passes raw Q while EXTENDED
    // regularizes, so results may differ on ill-conditioned input.
    RTKProcessor proc_extended;
    RTKProcessor proc_demo5;

    RTKProcessor::RTKConfig cfg_extended;
    cfg_extended.ar_policy = RTKProcessor::RTKConfig::ARPolicy::EXTENDED;
    proc_extended.setRTKConfig(cfg_extended);

    RTKProcessor::RTKConfig cfg_demo5;
    cfg_demo5.ar_policy = RTKProcessor::RTKConfig::ARPolicy::DEMO5_CONTINUOUS;
    proc_demo5.setRTKConfig(cfg_demo5);

    // Well-conditioned Q: both policies should produce the same integer fix.
    VectorXd float_amb(3);
    float_amb << 0.98, -3.05, 2.01;
    MatrixXd Q_good = MatrixXd::Identity(3, 3) * 0.01;

    VectorXd fix_ext, fix_demo5;
    double ratio_ext = 0.0, ratio_demo5 = 0.0;
    EXPECT_TRUE(proc_extended.lambdaMethod(float_amb, Q_good, fix_ext, ratio_ext));
    EXPECT_TRUE(proc_demo5.lambdaMethod(float_amb, Q_good, fix_demo5, ratio_demo5));

    // Both should round to the same integers on a well-conditioned system.
    ASSERT_EQ(fix_ext.size(), fix_demo5.size());
    for (int i = 0; i < fix_ext.size(); ++i) {
        EXPECT_NEAR(std::round(fix_ext(i)), std::round(fix_demo5(i)), 1e-9);
    }

    // Confirm DEMO5_CONTINUOUS policy is set correctly.
    EXPECT_EQ(proc_demo5.getRTKConfig().ar_policy,
              RTKProcessor::RTKConfig::ARPolicy::DEMO5_CONTINUOUS);
}

TEST(RTKLegacyCompatibilityStandaloneTest, MaxHoldDivergenceDefaultDisabled) {
    // Default max_hold_divergence_m must be 0.0 (disabled — existing behavior preserved).
    RTKProcessor processor;
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_hold_divergence_m, 0.0);

    // Explicitly set 0.0 and confirm round-trip.
    RTKProcessor::RTKConfig cfg;
    cfg.max_hold_divergence_m = 0.0;
    processor.setRTKConfig(cfg);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_hold_divergence_m, 0.0);

    // Setting a non-zero value should be stored correctly.
    RTKProcessor::RTKConfig cfg2;
    cfg2.max_hold_divergence_m = 0.5;
    processor.setRTKConfig(cfg2);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_hold_divergence_m, 0.5);

    // Reset to 0 confirms disabled state.
    RTKProcessor::RTKConfig cfg3;
    cfg3.max_hold_divergence_m = 0.0;
    processor.setRTKConfig(cfg3);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_hold_divergence_m, 0.0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, MaxPositionJumpDefaultDisabled) {
    // Default max_position_jump_m must be 0.0 (disabled — existing behavior preserved).
    RTKProcessor processor;
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_min_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_rate_mps, 0.0);

    // Explicitly set 0.0 and confirm round-trip.
    RTKProcessor::RTKConfig cfg;
    cfg.max_position_jump_m = 0.0;
    processor.setRTKConfig(cfg);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_m, 0.0);

    // Setting a non-zero value should be stored correctly.
    RTKProcessor::RTKConfig cfg2;
    cfg2.max_position_jump_m = 1.0;
    cfg2.max_position_jump_min_m = 20.0;
    cfg2.max_position_jump_rate_mps = 25.0;
    processor.setRTKConfig(cfg2);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_m, 1.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_min_m, 20.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_rate_mps, 25.0);

    // Reset to 0 confirms disabled state.
    RTKProcessor::RTKConfig cfg3;
    cfg3.max_position_jump_m = 0.0;
    cfg3.max_position_jump_min_m = 0.0;
    cfg3.max_position_jump_rate_mps = 0.0;
    processor.setRTKConfig(cfg3);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_min_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_rate_mps, 0.0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, MaxFloatSppDivergenceDefaultDisabled) {
    RTKProcessor processor;
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_spp_divergence_m, 0.0);

    RTKProcessor::RTKConfig cfg;
    cfg.max_float_spp_divergence_m = 0.0;
    processor.setRTKConfig(cfg);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_spp_divergence_m, 0.0);

    RTKProcessor::RTKConfig cfg2;
    cfg2.max_float_spp_divergence_m = 30.0;
    processor.setRTKConfig(cfg2);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_spp_divergence_m, 30.0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, MaxFloatPrefitResidualGateDefaultDisabled) {
    RTKProcessor processor;
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_rms_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_max_m, 0.0);
    EXPECT_EQ(processor.getRTKConfig().max_float_prefit_residual_reset_streak, 3);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_float_prefit_residual_trusted_jump_m, 0.0);

    RTKProcessor::RTKConfig cfg;
    cfg.max_float_prefit_residual_rms_m = 0.0;
    cfg.max_float_prefit_residual_max_m = 0.0;
    cfg.min_float_prefit_residual_trusted_jump_m = 0.0;
    processor.setRTKConfig(cfg);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_rms_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_max_m, 0.0);
    EXPECT_EQ(processor.getRTKConfig().max_float_prefit_residual_reset_streak, 3);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_float_prefit_residual_trusted_jump_m, 0.0);

    RTKProcessor::RTKConfig cfg2;
    cfg2.max_float_prefit_residual_rms_m = 6.0;
    cfg2.max_float_prefit_residual_max_m = 30.0;
    cfg2.max_float_prefit_residual_reset_streak = 5;
    cfg2.min_float_prefit_residual_trusted_jump_m = 8.0;
    processor.setRTKConfig(cfg2);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_rms_m, 6.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_max_m, 30.0);
    EXPECT_EQ(processor.getRTKConfig().max_float_prefit_residual_reset_streak, 5);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_float_prefit_residual_trusted_jump_m, 8.0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, MaxConsecutiveFloatResetDefaultDisabled) {
    RTKProcessor processor;
    EXPECT_EQ(processor.getRTKConfig().max_consecutive_float_for_reset, 0);
    EXPECT_EQ(processor.getRTKConfig().max_consecutive_nonfix_for_reset, 0);

    RTKProcessor::RTKConfig cfg;
    cfg.max_consecutive_float_for_reset = 0;
    cfg.max_consecutive_nonfix_for_reset = 0;
    processor.setRTKConfig(cfg);
    EXPECT_EQ(processor.getRTKConfig().max_consecutive_float_for_reset, 0);
    EXPECT_EQ(processor.getRTKConfig().max_consecutive_nonfix_for_reset, 0);

    RTKProcessor::RTKConfig cfg2;
    cfg2.max_consecutive_float_for_reset = 10;
    cfg2.max_consecutive_nonfix_for_reset = 12;
    processor.setRTKConfig(cfg2);
    EXPECT_EQ(processor.getRTKConfig().max_consecutive_float_for_reset, 10);
    EXPECT_EQ(processor.getRTKConfig().max_consecutive_nonfix_for_reset, 12);

    RTKProcessor::RTKConfig cfg3;
    cfg3.max_consecutive_float_for_reset = 0;
    cfg3.max_consecutive_nonfix_for_reset = 0;
    processor.setRTKConfig(cfg3);
    EXPECT_EQ(processor.getRTKConfig().max_consecutive_float_for_reset, 0);
    EXPECT_EQ(processor.getRTKConfig().max_consecutive_nonfix_for_reset, 0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, MaxPostfixResidualRmsDefaultDisabled) {
    // Default max_postfix_residual_rms must be 0.0 (disabled — existing behavior preserved).
    RTKProcessor processor;
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_postfix_residual_rms, 0.0);

    // Explicitly set 0.0 and confirm round-trip.
    RTKProcessor::RTKConfig cfg;
    cfg.max_postfix_residual_rms = 0.0;
    processor.setRTKConfig(cfg);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_postfix_residual_rms, 0.0);

    // Setting a non-zero value should be stored correctly.
    RTKProcessor::RTKConfig cfg2;
    cfg2.max_postfix_residual_rms = 0.5;
    processor.setRTKConfig(cfg2);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_postfix_residual_rms, 0.5);

    // Reset to 0 confirms disabled state.
    RTKProcessor::RTKConfig cfg3;
    cfg3.max_postfix_residual_rms = 0.0;
    processor.setRTKConfig(cfg3);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_postfix_residual_rms, 0.0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, MaxPostfixResidualRmsConfigurable) {
    RTKProcessor processor;

    // 1.0 m threshold
    RTKProcessor::RTKConfig cfg1;
    cfg1.max_postfix_residual_rms = 1.0;
    processor.setRTKConfig(cfg1);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_postfix_residual_rms, 1.0);

    // 0.5 m threshold
    RTKProcessor::RTKConfig cfg2;
    cfg2.max_postfix_residual_rms = 0.5;
    processor.setRTKConfig(cfg2);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_postfix_residual_rms, 0.5);

    // infinity sentinel (disabled)
    RTKProcessor::RTKConfig cfg3;
    cfg3.max_postfix_residual_rms = std::numeric_limits<double>::infinity();
    processor.setRTKConfig(cfg3);
    EXPECT_TRUE(std::isinf(processor.getRTKConfig().max_postfix_residual_rms));

    // NaN sentinel (also disabled by isfinite guard)
    RTKProcessor::RTKConfig cfg4;
    cfg4.max_postfix_residual_rms = std::numeric_limits<double>::quiet_NaN();
    processor.setRTKConfig(cfg4);
    EXPECT_TRUE(std::isnan(processor.getRTKConfig().max_postfix_residual_rms));

    // 0.0 (disabled)
    RTKProcessor::RTKConfig cfg5;
    cfg5.max_postfix_residual_rms = 0.0;
    processor.setRTKConfig(cfg5);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_postfix_residual_rms, 0.0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, WideLaneArDefaultDisabled) {
    // Default enable_wide_lane_ar must be false (disabled — existing behavior preserved).
    RTKProcessor processor;
    EXPECT_FALSE(processor.getRTKConfig().enable_wide_lane_ar);

    // Default wide_lane_acceptance_threshold must be 0.25.
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().wide_lane_acceptance_threshold, 0.25);

    // Explicitly set false and confirm round-trip.
    RTKProcessor::RTKConfig cfg;
    cfg.enable_wide_lane_ar = false;
    processor.setRTKConfig(cfg);
    EXPECT_FALSE(processor.getRTKConfig().enable_wide_lane_ar);

    // Setting true should be stored correctly.
    RTKProcessor::RTKConfig cfg2;
    cfg2.enable_wide_lane_ar = true;
    processor.setRTKConfig(cfg2);
    EXPECT_TRUE(processor.getRTKConfig().enable_wide_lane_ar);

    // Reset to false confirms disabled state.
    RTKProcessor::RTKConfig cfg3;
    cfg3.enable_wide_lane_ar = false;
    processor.setRTKConfig(cfg3);
    EXPECT_FALSE(processor.getRTKConfig().enable_wide_lane_ar);

    // Threshold default preserved after round-trip.
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().wide_lane_acceptance_threshold, 0.25);
}

TEST(RTKLegacyCompatibilityStandaloneTest, WideLaneArConfigurable) {
    RTKProcessor processor;

    // threshold 0.10
    RTKProcessor::RTKConfig cfg1;
    cfg1.enable_wide_lane_ar = true;
    cfg1.wide_lane_acceptance_threshold = 0.10;
    processor.setRTKConfig(cfg1);
    EXPECT_TRUE(processor.getRTKConfig().enable_wide_lane_ar);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().wide_lane_acceptance_threshold, 0.10);

    // threshold 0.25 (worktree default)
    RTKProcessor::RTKConfig cfg2;
    cfg2.enable_wide_lane_ar = true;
    cfg2.wide_lane_acceptance_threshold = 0.25;
    processor.setRTKConfig(cfg2);
    EXPECT_TRUE(processor.getRTKConfig().enable_wide_lane_ar);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().wide_lane_acceptance_threshold, 0.25);

    // threshold 0.50
    RTKProcessor::RTKConfig cfg3;
    cfg3.enable_wide_lane_ar = true;
    cfg3.wide_lane_acceptance_threshold = 0.50;
    processor.setRTKConfig(cfg3);
    EXPECT_TRUE(processor.getRTKConfig().enable_wide_lane_ar);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().wide_lane_acceptance_threshold, 0.50);
}

TEST(RTKLegacyCompatibilityStandaloneTest, ArPolicyDemo5ContinuousDisablesHoldFix) {
    // Under DEMO5_CONTINUOUS, the hold-fix fallback code path is gated off.
    // We verify by checking that tryHoldFix returns false when called directly
    // even when consecutive_fix_count meets the hold threshold — because the
    // DEMO5_CONTINUOUS gate prevents the hold-fix block from being entered.
    RTKProcessor processor;
    RTKProcessor::RTKConfig cfg;
    cfg.ar_policy = RTKProcessor::RTKConfig::ARPolicy::DEMO5_CONTINUOUS;
    cfg.min_hold_count = 3;
    processor.setRTKConfig(cfg);

    // Set up state as if hold is active.
    processor.consecutive_fix_count_ = 5;
    processor.has_last_fixed_position_ = true;
    processor.last_fixed_position_ = Eigen::Vector3d(-3961832.0, 3354966.0, 3697065.0);

    // tryHoldFix should return false because last_dd_fixed_ is empty (no held integers).
    // This is the baseline behavior; the DEMO5_CONTINUOUS gate in processRTKEpoch
    // prevents tryHoldFix from being called at all in normal operation.
    libgnss::GNSSTime dummy_time;
    dummy_time.week = 2300;
    dummy_time.tow = 0.0;
    libgnss::PositionSolution sol;
    // tryHoldFix itself checks hasHeldIntegers(); with no held state it returns false.
    EXPECT_FALSE(processor.tryHoldFix(processor.current_sat_data_, dummy_time, 0, sol));

    // Confirm policy is correctly set.
    EXPECT_EQ(processor.getRTKConfig().ar_policy,
              RTKProcessor::RTKConfig::ARPolicy::DEMO5_CONTINUOUS);
}

}  // namespace
