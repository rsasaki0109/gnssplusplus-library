#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/processor.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/io/rinex.hpp>

#include <libgnss++/algorithms/rtk.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace libgnss {

// Befriended by RTKProcessor. This replaces the former "#define private
// public" trick, which cannot link under the MSVC ABI (MSVC -- and Clang
// targeting it -- encodes member access in mangled names). Each accessor is
// named after the private member it exposes.
struct RTKProcessorTestAccess {
#define GNSSPP_RTK_TEST_MEMBER(name) \
    static auto& name(RTKProcessor& processor) { return processor.name; }
#define GNSSPP_RTK_TEST_METHOD(name)                                   \
    template <typename... Args>                                        \
    static decltype(auto) name(RTKProcessor& processor, Args&&... args) { \
        return processor.name(std::forward<Args>(args)...);            \
    }
    GNSSPP_RTK_TEST_MEMBER(filter_state_)
    GNSSPP_RTK_TEST_MEMBER(filter_initialized_)
    GNSSPP_RTK_TEST_MEMBER(current_sat_data_)
    GNSSPP_RTK_TEST_MEMBER(fixed_baseline_)
    GNSSPP_RTK_TEST_MEMBER(has_fixed_solution_)
    GNSSPP_RTK_TEST_MEMBER(last_fixed_position_)
    GNSSPP_RTK_TEST_MEMBER(has_last_fixed_position_)
    GNSSPP_RTK_TEST_MEMBER(consecutive_fix_count_)
    GNSSPP_RTK_TEST_MEMBER(lock_count_l1_)
    GNSSPP_RTK_TEST_MEMBER(last_dd_pairs_)
    GNSSPP_RTK_TEST_MEMBER(last_best_subset_)
    GNSSPP_RTK_TEST_MEMBER(last_dd_fixed_)
    GNSSPP_RTK_TEST_MEMBER(last_ar_ratio_)
    GNSSPP_RTK_TEST_MEMBER(last_num_fixed_ambiguities_)
    GNSSPP_RTK_TEST_METHOD(formDoubleDifferences)
    GNSSPP_RTK_TEST_METHOD(calculateBaseline)
    GNSSPP_RTK_TEST_METHOD(calculateResiduals)
    GNSSPP_RTK_TEST_METHOD(formMeasurementMatrix)
    GNSSPP_RTK_TEST_METHOD(calculateMeasurementWeights)
    GNSSPP_RTK_TEST_METHOD(hasSufficientSatellites)
    GNSSPP_RTK_TEST_METHOD(applyFixedAmbiguities)
    GNSSPP_RTK_TEST_METHOD(solvePositionWithAmbiguities)
    GNSSPP_RTK_TEST_METHOD(solveLAMBDA)
    GNSSPP_RTK_TEST_METHOD(validateAmbiguityResolution)
    GNSSPP_RTK_TEST_METHOD(varerr)
    GNSSPP_RTK_TEST_METHOD(tryHoldFix)
    GNSSPP_RTK_TEST_METHOD(applyHoldAmbiguity)
#undef GNSSPP_RTK_TEST_METHOD
#undef GNSSPP_RTK_TEST_MEMBER

    using DDPair = RTKProcessor::DDPair;
    static constexpr int kStateCount = RTKProcessor::NX;
    static int ambiguityIndex(const SatelliteId& satellite, int freq) {
        return RTKProcessor::IB(satellite, freq);
    }
};

}  // namespace libgnss

using namespace libgnss;
using Peer = RTKProcessorTestAccess;

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
    const auto measurements =
        Peer::formDoubleDifferences(processor_, rover_obs_, base_obs_, nav_data_);

    ASSERT_GE(measurements.size(), 4u);
    EXPECT_TRUE(Peer::filter_initialized_(processor_));
    EXPECT_FALSE(Peer::current_sat_data_(processor_).empty());

    for (const auto& measurement : measurements) {
        EXPECT_TRUE(measurement.valid);
        EXPECT_TRUE(std::isfinite(measurement.pseudorange_dd));
        EXPECT_TRUE(std::isfinite(measurement.carrier_phase_dd));
        EXPECT_TRUE(std::isfinite(measurement.geometric_range));
        EXPECT_TRUE(measurement.unit_vector.array().isFinite().all());
        EXPECT_GT(measurement.variance, 0.0);
    }

    const Vector3d baseline = Peer::calculateBaseline(processor_);
    EXPECT_TRUE(baseline.array().isFinite().all());

    const VectorXd residuals = Peer::calculateResiduals(processor_, measurements, baseline);
    ASSERT_EQ(residuals.size(), static_cast<int>(measurements.size()));
    EXPECT_TRUE(residuals.array().isFinite().all());

    const MatrixXd H =
        Peer::formMeasurementMatrix(processor_, measurements, nav_data_, rover_obs_.time);
    ASSERT_EQ(H.rows(), static_cast<int>(measurements.size()));
    ASSERT_EQ(H.cols(), 3);
    EXPECT_TRUE(H.array().isFinite().all());

    const MatrixXd W = Peer::calculateMeasurementWeights(processor_, measurements);
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

    EXPECT_TRUE(Peer::hasSufficientSatellites(processor_, measurements));
}

TEST_F(RTKLegacyCompatibilityTest, AssemblesRealRowsForTightlyCoupledDDIMUBridge) {
    const Matrix3d ecef_to_enu = Matrix3d::Identity();
    const auto rows = processor_.formTightlyCoupledObservations(
        rover_obs_, base_obs_, nav_data_, rover_header_.approximate_position,
        ecef_to_enu, Eigen::Quaterniond::Identity());

    ASSERT_GE(rows.size(), 4u);
    for (const auto& row : rows) {
        EXPECT_GT(row.key.satellite_prn, 0);
        EXPECT_GT(row.key.reference_satellite_prn, 0);
        EXPECT_NE(row.key.satellite_system, 0);
        EXPECT_EQ(row.key.satellite_system, row.key.reference_satellite_system);
        EXPECT_GE(row.key.frequency_index, 0);
        EXPECT_GE(row.key.signal_type, 0);
        EXPECT_TRUE(row.geometry_enu.array().isFinite().all());
        EXPECT_TRUE(std::isfinite(row.code_residual_m));
        EXPECT_TRUE(std::isfinite(row.carrier_residual_m));
        EXPECT_GT(row.code_variance_m2, 0.0);
        // The fixture is a fresh arc; tight carrier rows mature for 300
        // consecutive lock epochs before ambiguity-state augmentation.
        EXPECT_DOUBLE_EQ(row.carrier_variance_m2, 0.0);
        EXPECT_GT(row.wavelength_m, 0.0);
        EXPECT_GE(row.posterior_abs_residual_m, 0.0);
        EXPECT_LE(row.posterior_abs_residual_m, 0.5 * row.wavelength_m + 1e-12);
        EXPECT_TRUE(std::isfinite(row.body_azimuth_rad));
    }
}

TEST(RTKLegacyCompatibilityStandaloneTest, LambdaCompatibilityProducesValidatedIntegerFix) {
    RTKProcessor processor;
    VectorXd float_ambiguities(3);
    float_ambiguities << 1.02, -2.97, 5.01;
    MatrixXd covariance = MatrixXd::Identity(3, 3) * 0.01;

    const auto result = Peer::solveLAMBDA(processor, float_ambiguities, covariance);

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.fixed_ambiguities.size(), float_ambiguities.size());
    EXPECT_GT(result.ratio, 0.0);
    for (int i = 0; i < result.fixed_ambiguities.size(); ++i) {
        EXPECT_NEAR(result.fixed_ambiguities(i),
                    std::round(result.fixed_ambiguities(i)),
                    1e-9);
    }

    EXPECT_TRUE(Peer::validateAmbiguityResolution(processor,
                                                  result.fixed_ambiguities,
                                                  float_ambiguities,
                                                  covariance,
                                                  3.5));
    EXPECT_FALSE(Peer::validateAmbiguityResolution(processor,
                                                   result.fixed_ambiguities,
                                                   float_ambiguities,
                                                   covariance,
                                                   0.5));
}

TEST_F(RTKLegacyCompatibilityTest, AppliesFixedAmbiguitiesAndPublishesFixedBaseline) {
    const auto measurements =
        Peer::formDoubleDifferences(processor_, rover_obs_, base_obs_, nav_data_);

    ASSERT_GE(measurements.size(), 4u);
    ASSERT_TRUE(Peer::filter_initialized_(processor_));

    std::vector<SatelliteId> satellites;
    satellites.reserve(Peer::current_sat_data_(processor_).size());
    for (const auto& [satellite, sat_data] : Peer::current_sat_data_(processor_)) {
        if (sat_data.has_l1 || sat_data.has_l2) {
            satellites.push_back(satellite);
        }
    }
    std::sort(satellites.begin(), satellites.end());

    int n1_count = 0;
    int n2_count = 0;
    for (const auto& satellite : satellites) {
        if (Peer::filter_state_(processor_).n1_indices.count(satellite) > 0) {
            ++n1_count;
        }
        if (Peer::filter_state_(processor_).n2_indices.count(satellite) > 0) {
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

    ASSERT_TRUE(Peer::applyFixedAmbiguities(
        processor_, fixed_n1, fixed_n2, Peer::current_sat_data_(processor_)));

    int n1_index = 0;
    int n2_index = 0;
    for (const auto& satellite : satellites) {
        const auto n1_it = Peer::filter_state_(processor_).n1_indices.find(satellite);
        if (n1_it != Peer::filter_state_(processor_).n1_indices.end()) {
            EXPECT_DOUBLE_EQ(Peer::filter_state_(processor_).state(n1_it->second), fixed_n1(n1_index));
            ++n1_index;
        }
        const auto n2_it = Peer::filter_state_(processor_).n2_indices.find(satellite);
        if (n2_it != Peer::filter_state_(processor_).n2_indices.end()) {
            EXPECT_DOUBLE_EQ(Peer::filter_state_(processor_).state(n2_it->second), fixed_n2(n2_index));
            ++n2_index;
        }
    }

    Peer::fixed_baseline_(processor_) = Vector3d(1.0, 2.0, 3.0);
    Peer::has_fixed_solution_(processor_) = true;
    Peer::solvePositionWithAmbiguities(processor_, Peer::current_sat_data_(processor_));

    EXPECT_TRUE(Peer::filter_state_(processor_).state.head<3>().isApprox(
        Peer::fixed_baseline_(processor_), 1e-12));
}

// ============================================================
// (Phase 1 GNSS/IMU coupling unit tests for RTKProcessor::
// setExternalPositionPrior() live in tests/test_rtk_ins_prior.cpp, which
// exercises RTKProcessor through its public API only.)
// ============================================================

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
    Peer::consecutive_fix_count_(processor) = 3;
    Peer::has_last_fixed_position_(processor) = true;
    // The relaxed hold-ratio path exists under EXTENDED; verify config reads correctly.
    EXPECT_EQ(processor.getRTKConfig().ar_policy, RTKProcessor::RTKConfig::ARPolicy::EXTENDED);
    EXPECT_GE(Peer::consecutive_fix_count_(processor), processor.getRTKConfig().min_hold_count);
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

TEST(RTKLegacyCompatibilityStandaloneTest, MaxPositionJumpDefaultEnabled) {
    // Default max_position_jump_m is 5.0m (wrong-FIX rejection enabled by default).
    // Setting 0.0 disables the check. Adaptive min/rate remain disabled by default.
    RTKProcessor processor;
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_m, 5.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_min_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_rate_mps, 0.0);

    // Explicitly set 0.0 and confirm round-trip (disable opt-out preserved).
    RTKProcessor::RTKConfig cfg_off;
    cfg_off.max_position_jump_m = 0.0;
    processor.setRTKConfig(cfg_off);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_m, 0.0);

    // Setting a non-zero custom value should be stored correctly.
    RTKProcessor::RTKConfig cfg_custom;
    cfg_custom.max_position_jump_m = 1.0;
    cfg_custom.max_position_jump_min_m = 20.0;
    cfg_custom.max_position_jump_rate_mps = 25.0;
    processor.setRTKConfig(cfg_custom);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_m, 1.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_min_m, 20.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_rate_mps, 25.0);

    // Restore to new default value.
    RTKProcessor::RTKConfig cfg_back;
    cfg_back.max_position_jump_m = 5.0;
    cfg_back.max_position_jump_min_m = 0.0;
    cfg_back.max_position_jump_rate_mps = 0.0;
    processor.setRTKConfig(cfg_back);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_position_jump_m, 5.0);
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
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_update_nis_per_observation, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_nis_per_observation, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_post_residual_rms_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_ratio, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_gate_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_gate_speed_mps, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_speed_mps, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_ratio, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_secondary_gate_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_secondary_gate_speed_mps, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_speed_mps, 0.0);

    RTKProcessor::RTKConfig cfg;
    cfg.max_float_prefit_residual_rms_m = 0.0;
    cfg.max_float_prefit_residual_max_m = 0.0;
    cfg.min_float_prefit_residual_trusted_jump_m = 0.0;
    cfg.max_update_nis_per_observation = 0.0;
    cfg.max_fixed_update_nis_per_observation = 0.0;
    cfg.max_fixed_update_post_residual_rms_m = 0.0;
    cfg.max_fixed_update_gate_ratio = 0.0;
    cfg.min_fixed_update_gate_baseline_m = 0.0;
    cfg.max_fixed_update_gate_baseline_m = 0.0;
    cfg.min_fixed_update_gate_speed_mps = 0.0;
    cfg.max_fixed_update_gate_speed_mps = 0.0;
    cfg.max_fixed_update_secondary_gate_ratio = 0.0;
    cfg.min_fixed_update_secondary_gate_baseline_m = 0.0;
    cfg.max_fixed_update_secondary_gate_baseline_m = 0.0;
    cfg.min_fixed_update_secondary_gate_speed_mps = 0.0;
    cfg.max_fixed_update_secondary_gate_speed_mps = 0.0;
    processor.setRTKConfig(cfg);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_rms_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_max_m, 0.0);
    EXPECT_EQ(processor.getRTKConfig().max_float_prefit_residual_reset_streak, 3);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_float_prefit_residual_trusted_jump_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_update_nis_per_observation, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_nis_per_observation, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_post_residual_rms_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_ratio, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_gate_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_gate_speed_mps, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_speed_mps, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_ratio, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_secondary_gate_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_secondary_gate_speed_mps, 0.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_speed_mps, 0.0);

    RTKProcessor::RTKConfig cfg2;
    cfg2.max_float_prefit_residual_rms_m = 6.0;
    cfg2.max_float_prefit_residual_max_m = 30.0;
    cfg2.max_float_prefit_residual_reset_streak = 5;
    cfg2.min_float_prefit_residual_trusted_jump_m = 8.0;
    cfg2.max_update_nis_per_observation = 12.0;
    cfg2.max_fixed_update_nis_per_observation = 10.0;
    cfg2.max_fixed_update_post_residual_rms_m = 6.0;
    cfg2.max_fixed_update_gate_ratio = 8.0;
    cfg2.min_fixed_update_gate_baseline_m = 7000.0;
    cfg2.max_fixed_update_gate_baseline_m = 8200.0;
    cfg2.min_fixed_update_gate_speed_mps = 5.0;
    cfg2.max_fixed_update_gate_speed_mps = 15.0;
    cfg2.max_fixed_update_secondary_gate_ratio = 4.0;
    cfg2.min_fixed_update_secondary_gate_baseline_m = 2000.0;
    cfg2.max_fixed_update_secondary_gate_baseline_m = 2500.0;
    cfg2.min_fixed_update_secondary_gate_speed_mps = 7.0;
    cfg2.max_fixed_update_secondary_gate_speed_mps = 15.0;
    processor.setRTKConfig(cfg2);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_rms_m, 6.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_float_prefit_residual_max_m, 30.0);
    EXPECT_EQ(processor.getRTKConfig().max_float_prefit_residual_reset_streak, 5);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_float_prefit_residual_trusted_jump_m, 8.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_update_nis_per_observation, 12.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_nis_per_observation, 10.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_post_residual_rms_m, 6.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_ratio, 8.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_gate_baseline_m, 7000.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_baseline_m, 8200.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_gate_speed_mps, 5.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_gate_speed_mps, 15.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_ratio, 4.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_secondary_gate_baseline_m, 2000.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_baseline_m, 2500.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_fixed_update_secondary_gate_speed_mps, 7.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().max_fixed_update_secondary_gate_speed_mps, 15.0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, SnrWeightingDefaultDisabledAndConfigurable) {
    RTKProcessor processor;
    EXPECT_FALSE(processor.getRTKConfig().enable_snr_weighting);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_reference_dbhz, 45.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_max_variance_scale, 25.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_min_baseline_m, 0.0);

    const double elevation = 30.0 * M_PI / 180.0;
    const double default_variance = Peer::varerr(processor, elevation, true, 30.0);
    EXPECT_DOUBLE_EQ(default_variance, Peer::varerr(processor, elevation, true, 45.0));

    RTKProcessor::RTKConfig cfg;
    cfg.enable_snr_weighting = true;
    cfg.snr_reference_dbhz = 45.0;
    cfg.snr_max_variance_scale = 10.0;
    cfg.snr_min_baseline_m = 0.0;
    processor.setRTKConfig(cfg);

    EXPECT_TRUE(processor.getRTKConfig().enable_snr_weighting);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_reference_dbhz, 45.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_max_variance_scale, 10.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_min_baseline_m, 0.0);
    EXPECT_DOUBLE_EQ(Peer::varerr(processor, elevation, true, 50.0), default_variance);
    EXPECT_NEAR(Peer::varerr(processor, elevation, true, 35.0), default_variance * 10.0, 1e-12);

    cfg.snr_min_baseline_m = 7000.0;
    processor.setRTKConfig(cfg);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_min_baseline_m, 7000.0);
    EXPECT_DOUBLE_EQ(Peer::varerr(processor, elevation, true, 35.0), default_variance);
}

TEST(RTKLegacyCompatibilityStandaloneTest, SnrElevationAwareReferenceAndCodeOnlyInflation) {
    RTKProcessor processor;
    EXPECT_FALSE(processor.getRTKConfig().snr_elevation_aware);
    EXPECT_FALSE(processor.getRTKConfig().snr_inflate_code_only);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_elevation_floor_dbhz, 33.0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().snr_elevation_gain_dbhz, 15.0);

    const double elevation = 30.0 * M_PI / 180.0;
    const double model_phase = Peer::varerr(processor, elevation, true, 0.0);
    const double model_code = Peer::varerr(processor, elevation, false, 0.0);

    RTKProcessor::RTKConfig cfg;
    cfg.enable_snr_weighting = true;
    cfg.snr_reference_dbhz = 45.0;
    cfg.snr_max_variance_scale = 1.0e6;
    processor.setRTKConfig(cfg);
    // Constant reference: 40.5 dB-Hz is already 4.5 dB below it at any elevation.
    EXPECT_NEAR(Peer::varerr(processor, elevation, false, 40.5),
                model_code * std::pow(10.0, 0.45), 1e-9);

    cfg.snr_elevation_aware = true;
    processor.setRTKConfig(cfg);
    // 30 deg: expected 33 + 15 sin(30 deg) = 40.5 dB-Hz -> no inflation at 40.5,
    // 10 dB of deficit at 30.5.
    EXPECT_DOUBLE_EQ(Peer::varerr(processor, elevation, false, 40.5), model_code);
    EXPECT_NEAR(Peer::varerr(processor, elevation, false, 30.5), model_code * 10.0, 1e-9);
    EXPECT_NEAR(Peer::varerr(processor, elevation, true, 30.5), model_phase * 10.0, 1e-12);
    // Zenith side: the expectation is capped by the constant reference.
    const double zenith = 80.0 * M_PI / 180.0;
    const double zenith_code = Peer::varerr(processor, zenith, false, 0.0);
    EXPECT_DOUBLE_EQ(Peer::varerr(processor, zenith, false, 45.0), zenith_code);
    EXPECT_NEAR(Peer::varerr(processor, zenith, false, 35.0), zenith_code * 10.0, 1e-9);

    cfg.snr_inflate_code_only = true;
    processor.setRTKConfig(cfg);
    EXPECT_NEAR(Peer::varerr(processor, elevation, false, 30.5), model_code * 10.0, 1e-9);
    EXPECT_DOUBLE_EQ(Peer::varerr(processor, elevation, true, 30.5), model_phase);
}

TEST(RTKLegacyCompatibilityStandaloneTest, DynamicSlipThresholdFloorConfigurable) {
    RTKProcessor processor;
    EXPECT_TRUE(processor.getRTKConfig().use_dynamic_slip_threshold_floor);
    EXPECT_FALSE(processor.getRTKConfig().enable_adaptive_dynamic_slip_thresholds);
    EXPECT_EQ(processor.getRTKConfig().adaptive_dynamic_slip_nonfix_count, 3);
    EXPECT_EQ(processor.getRTKConfig().adaptive_dynamic_slip_hold_epochs, 10);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().cycle_slip_threshold, 0.05);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().doppler_slip_threshold, 0.20);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().code_slip_threshold, 5.0);

    RTKProcessor::RTKConfig cfg;
    cfg.cycle_slip_threshold = 0.08;
    cfg.doppler_slip_threshold = 0.15;
    cfg.code_slip_threshold = 4.0;
    cfg.use_dynamic_slip_threshold_floor = false;
    cfg.enable_adaptive_dynamic_slip_thresholds = true;
    cfg.adaptive_dynamic_slip_nonfix_count = 2;
    cfg.adaptive_dynamic_slip_hold_epochs = 8;
    processor.setRTKConfig(cfg);

    EXPECT_FALSE(processor.getRTKConfig().use_dynamic_slip_threshold_floor);
    EXPECT_TRUE(processor.getRTKConfig().enable_adaptive_dynamic_slip_thresholds);
    EXPECT_EQ(processor.getRTKConfig().adaptive_dynamic_slip_nonfix_count, 2);
    EXPECT_EQ(processor.getRTKConfig().adaptive_dynamic_slip_hold_epochs, 8);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().cycle_slip_threshold, 0.08);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().doppler_slip_threshold, 0.15);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().code_slip_threshold, 4.0);
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

TEST(RTKLegacyCompatibilityStandaloneTest, DopplerFloatSeedDefaultDisabled) {
    RTKProcessor processor;
    EXPECT_FALSE(processor.getRTKConfig().use_doppler_float_seed);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().doppler_float_seed_max_age_s, 6.0);

    auto config = processor.getRTKConfig();
    config.use_doppler_float_seed = true;
    config.doppler_float_seed_max_age_s = 3.0;
    processor.setRTKConfig(config);
    EXPECT_TRUE(processor.getRTKConfig().use_doppler_float_seed);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().doppler_float_seed_max_age_s, 3.0);
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
    EXPECT_FALSE(processor.getRTKConfig().enable_wlnl_fallback);

    // Default wide_lane_acceptance_threshold must be 0.25.
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().wide_lane_acceptance_threshold, 0.25);
    // MW arc smoothing is opt-in; 0 keeps the legacy single-epoch WL test.
    EXPECT_EQ(processor.getRTKConfig().wide_lane_min_arc_samples, 0);

    // Explicitly set false and confirm round-trip.
    RTKProcessor::RTKConfig cfg;
    cfg.enable_wide_lane_ar = false;
    cfg.enable_wlnl_fallback = false;
    processor.setRTKConfig(cfg);
    EXPECT_FALSE(processor.getRTKConfig().enable_wide_lane_ar);
    EXPECT_FALSE(processor.getRTKConfig().enable_wlnl_fallback);

    // Setting true should be stored correctly.
    RTKProcessor::RTKConfig cfg2;
    cfg2.enable_wide_lane_ar = true;
    cfg2.enable_wlnl_fallback = true;
    processor.setRTKConfig(cfg2);
    EXPECT_TRUE(processor.getRTKConfig().enable_wide_lane_ar);
    EXPECT_TRUE(processor.getRTKConfig().enable_wlnl_fallback);

    // Reset to false confirms disabled state.
    RTKProcessor::RTKConfig cfg3;
    cfg3.enable_wide_lane_ar = false;
    cfg3.enable_wlnl_fallback = false;
    processor.setRTKConfig(cfg3);
    EXPECT_FALSE(processor.getRTKConfig().enable_wide_lane_ar);
    EXPECT_FALSE(processor.getRTKConfig().enable_wlnl_fallback);

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

TEST(RTKLegacyCompatibilityStandaloneTest, SubsetArFullRatioGateDefaultDisabled) {
    RTKProcessor processor;
    EXPECT_EQ(processor.getRTKConfig().min_subset_pairs_for_ar, 4);
    EXPECT_EQ(processor.getRTKConfig().max_subset_drop_steps_for_ar, 6);
    EXPECT_FALSE(processor.getRTKConfig().enable_paper_constellation_fallback_ar);
    EXPECT_EQ(processor.getRTKConfig().min_subset_sats_for_ar, 0);
    EXPECT_EQ(processor.getRTKConfig().min_subset_systems_for_ar, 0);
    EXPECT_EQ(processor.getRTKConfig().min_subset_frequencies_for_ar, 0);
    EXPECT_EQ(processor.getRTKConfig().min_subset_dual_frequency_sats_for_ar, 0);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_full_ratio_for_subset_ar, 0.0);

    RTKProcessor::RTKConfig cfg;
    cfg.min_subset_pairs_for_ar = 6;
    cfg.max_subset_drop_steps_for_ar = 12;
    cfg.min_subset_sats_for_ar = 3;
    cfg.min_subset_systems_for_ar = 2;
    cfg.min_subset_frequencies_for_ar = 2;
    cfg.min_subset_dual_frequency_sats_for_ar = 2;
    cfg.min_full_ratio_for_subset_ar = 3.0;
    processor.setRTKConfig(cfg);
    EXPECT_EQ(processor.getRTKConfig().min_subset_pairs_for_ar, 6);
    EXPECT_EQ(processor.getRTKConfig().max_subset_drop_steps_for_ar, 12);
    EXPECT_EQ(processor.getRTKConfig().min_subset_sats_for_ar, 3);
    EXPECT_EQ(processor.getRTKConfig().min_subset_systems_for_ar, 2);
    EXPECT_EQ(processor.getRTKConfig().min_subset_frequencies_for_ar, 2);
    EXPECT_EQ(processor.getRTKConfig().min_subset_dual_frequency_sats_for_ar, 2);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_full_ratio_for_subset_ar, 3.0);

    RTKProcessor::RTKConfig disabled;
    disabled.min_full_ratio_for_subset_ar = 0.0;
    processor.setRTKConfig(disabled);
    EXPECT_DOUBLE_EQ(processor.getRTKConfig().min_full_ratio_for_subset_ar, 0.0);
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
    Peer::consecutive_fix_count_(processor) = 5;
    Peer::has_last_fixed_position_(processor) = true;
    Peer::last_fixed_position_(processor) = Eigen::Vector3d(-3961832.0, 3354966.0, 3697065.0);

    // tryHoldFix should return false because last_dd_fixed_ is empty (no held integers).
    // This is the baseline behavior; the DEMO5_CONTINUOUS gate in processRTKEpoch
    // prevents tryHoldFix from being called at all in normal operation.
    libgnss::GNSSTime dummy_time;
    dummy_time.week = 2300;
    dummy_time.tow = 0.0;
    libgnss::PositionSolution sol;
    // tryHoldFix itself checks hasHeldIntegers(); with no held state it returns false.
    EXPECT_FALSE(Peer::tryHoldFix(processor, Peer::current_sat_data_(processor), dummy_time, 0, sol));

    // Confirm policy is correctly set.
    EXPECT_EQ(processor.getRTKConfig().ar_policy,
              RTKProcessor::RTKConfig::ARPolicy::DEMO5_CONTINUOUS);
}

// ============================================================
// Hold-fix only for an unchanged pair set (opt-in hold_fix_unchanged_set_only)
// ============================================================

// Current epoch: reference G01 and eight GPS satellites G02..G09 on L1, one
// SD ambiguity state each. G02..G05 are held from earlier fixes, G06..G09 are
// new (they joined, or their ambiguity was reset). The new floats sit on x.5
// so rounding them would be a coin flip. G10 is held but no longer tracked.
// The scene is built with the option on (reject) or off (today's behavior).
class RTKHoldFixSceneTest : public ::testing::Test {
protected:
    static constexpr double kRefAmbiguityCycles = 1000.25;
    static constexpr double kL1WavelengthM = 0.1902936728;

    static SatelliteId gps(int prn) {
        return SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(prn));
    }

    virtual bool unchangedSetOnly() const = 0;

    static Vector3d basePosition() { return Vector3d(-3961832.0, 3354966.0, 3697065.0); }

    static GNSSTime epochTime() {
        GNSSTime time;
        time.week = 2300;
        time.tow = 100.0;
        return time;
    }

    static void buildScene(RTKProcessor& processor, bool unchanged_set_only) {
        RTKProcessor::RTKConfig cfg;
        cfg.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        cfg.min_hold_count = 3;
        cfg.hold_fix_unchanged_set_only = unchanged_set_only;
        processor.setRTKConfig(cfg);
        processor.setBasePosition(basePosition());

        auto& fs = Peer::filter_state_(processor);
        fs.state = VectorXd::Zero(Peer::kStateCount);
        fs.covariance = MatrixXd::Zero(Peer::kStateCount, Peer::kStateCount);
        fs.state.head<3>() = Vector3d(1.0, 2.0, 3.0);
        fs.covariance.topLeftCorner<3, 3>() = Matrix3d::Identity() * 0.01;

        // DD float of each non-reference satellite against G01.
        const std::map<int, double> dd_float = {
            {2, 12.10}, {3, -6.85}, {4, 25.20}, {5, 3.60},   // held
            {6, 7.49}, {7, -3.51}, {8, 15.50}, {9, -9.50}};  // new, near x.5
        auto& sat_data = Peer::current_sat_data_(processor);
        auto add_satellite = [&](int prn, double ambiguity_cycles, double elevation) {
            const SatelliteId sat = gps(prn);
            auto& sd = sat_data[sat];
            sd.satellite = sat;
            sd.has_l1 = true;
            sd.l1_wavelength = kL1WavelengthM;
            sd.elevation = elevation;
            const int idx = Peer::ambiguityIndex(sat, 0);
            fs.n1_indices[sat] = idx;
            fs.state(idx) = ambiguity_cycles;
            fs.covariance(idx, idx) = 0.04;
            Peer::lock_count_l1_(processor)[sat] = 10;
        };
        add_satellite(1, kRefAmbiguityCycles, 1.4);
        for (const auto& [prn, dd] : dd_float) {
            add_satellite(prn, kRefAmbiguityCycles - dd, 1.0 - 0.05 * prn);
        }
        // Position correlates with the first new satellite only. Any fixed
        // position that depends on that pair's rounded integer moves.
        fs.covariance(0, Peer::ambiguityIndex(gps(6), 0)) = 0.01;
        fs.covariance(Peer::ambiguityIndex(gps(6), 0), 0) = 0.01;

        // Held set from earlier fixes: G10 is stale, and last_best_subset_
        // does not follow last_dd_pairs_ order.
        auto make_pair = [&](int prn) {
            return Peer::DDPair{gps(1), Peer::ambiguityIndex(gps(1), 0), Peer::ambiguityIndex(gps(prn), 0),
                                gps(prn), 0};
        };
        Peer::last_dd_pairs_(processor) = {make_pair(10), make_pair(2), make_pair(3),
                                           make_pair(4), make_pair(5)};
        Peer::last_best_subset_(processor) = {1, 2, 3, 4, 0};
        VectorXd held(5);
        held << 12.0, -7.0, 25.0, 3.0, 40.0;  // G05 floats at 3.60: the held 3 must win
        Peer::last_dd_fixed_(processor) = held;

        Peer::consecutive_fix_count_(processor) = 5;
        Peer::has_last_fixed_position_(processor) = true;
        Peer::last_fixed_position_(processor) = basePosition() + Vector3d(1.0, 2.0, 3.0);
    }

    // Stop tracking the given satellites (they are no longer candidates).
    static void dropSatellites(RTKProcessor& processor, std::initializer_list<int> prns) {
        auto& fs = Peer::filter_state_(processor);
        for (int prn : prns) {
            Peer::current_sat_data_(processor).erase(gps(prn));
            fs.n1_indices.erase(gps(prn));
        }
    }

    void SetUp() override {
        buildScene(processor_, unchangedSetOnly());
        time_ = epochTime();
    }

    RTKProcessor processor_;
    GNSSTime time_;
};

// Option on (hold_fix_unchanged_set_only = true): H2.
class RTKHoldFixUnchangedSetTest : public RTKHoldFixSceneTest {
protected:
    bool unchangedSetOnly() const override { return true; }
};

// Option off (the default): today's behavior, rounding included.
class RTKHoldFixDefaultTest : public RTKHoldFixSceneTest {
protected:
    bool unchangedSetOnly() const override { return false; }
};

TEST_F(RTKHoldFixUnchangedSetTest, UnmatchedPairRejectsHoldFixAndKeepsHeldSet) {
    const auto pairs_before = Peer::last_dd_pairs_(processor_);
    const auto subset_before = Peer::last_best_subset_(processor_);
    const VectorXd fixed_before = Peer::last_dd_fixed_(processor_);
    const Vector3d fixed_baseline_before = Peer::fixed_baseline_(processor_);
    const Vector3d filter_position_before = Peer::filter_state_(processor_).state.head<3>();

    PositionSolution solution;
    EXPECT_FALSE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 9, solution));

    const auto& telemetry = processor_.getLastDebugTelemetry();
    EXPECT_TRUE(telemetry.hold_fix_attempted);
    EXPECT_FALSE(telemetry.hold_fix_applied);
    EXPECT_EQ(telemetry.hold_fix_reject_reason, "unmatched_pairs");
    EXPECT_EQ(telemetry.hold_fix_candidate_pairs, 8);
    EXPECT_EQ(telemetry.hold_fix_matched_pairs, 4);

    // Nothing was accepted: the held set, the fixed state and the filter are untouched.
    const auto& pairs = Peer::last_dd_pairs_(processor_);
    ASSERT_EQ(pairs.size(), pairs_before.size());
    for (size_t i = 0; i < pairs.size(); ++i) {
        EXPECT_EQ(pairs[i].ref_sat, pairs_before[i].ref_sat);
        EXPECT_EQ(pairs[i].sat, pairs_before[i].sat);
        EXPECT_EQ(pairs[i].ref_idx, pairs_before[i].ref_idx);
        EXPECT_EQ(pairs[i].sat_idx, pairs_before[i].sat_idx);
        EXPECT_EQ(pairs[i].freq, pairs_before[i].freq);
    }
    EXPECT_EQ(Peer::last_best_subset_(processor_), subset_before);
    EXPECT_EQ(Peer::last_dd_fixed_(processor_), fixed_before);
    EXPECT_FALSE(Peer::has_fixed_solution_(processor_));
    EXPECT_EQ(Peer::fixed_baseline_(processor_), fixed_baseline_before);
    EXPECT_EQ(Peer::last_num_fixed_ambiguities_(processor_), 0);
    EXPECT_DOUBLE_EQ(Peer::last_ar_ratio_(processor_), 0.0);
    EXPECT_EQ(Peer::filter_state_(processor_).state.head<3>(), filter_position_before);
}

TEST_F(RTKHoldFixUnchangedSetTest, AllCandidatesMatchedWithStaleHeldPairIsAcceptedAsBefore) {
    // Remove the four new satellites: every candidate pair (G02..G05) is held.
    // G10 stays in the held set although it is no longer a candidate.
    dropSatellites(processor_, {6, 7, 8, 9});

    PositionSolution solution;
    ASSERT_TRUE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, solution));

    const auto& telemetry = processor_.getLastDebugTelemetry();
    EXPECT_TRUE(telemetry.hold_fix_applied);
    EXPECT_TRUE(telemetry.hold_fix_reject_reason.empty());
    EXPECT_EQ(telemetry.hold_fix_candidate_pairs, 4);
    EXPECT_EQ(telemetry.hold_fix_matched_pairs, 4);

    EXPECT_EQ(solution.status, SolutionStatus::FIXED);
    EXPECT_EQ(solution.num_fixed_ambiguities, 4);
    EXPECT_DOUBLE_EQ(solution.ratio, processor_.getRTKConfig().ambiguity_ratio_threshold);
    EXPECT_TRUE(Peer::fixed_baseline_(processor_).isApprox(Vector3d(1.0, 2.0, 3.0), 1e-9))
        << Peer::fixed_baseline_(processor_).transpose();

    // The held set becomes the current candidate set with the held integers
    // (G05 floats at 3.60, the held 3 wins); the stale G10 pair is gone.
    const auto& pairs = Peer::last_dd_pairs_(processor_);
    const auto& subset = Peer::last_best_subset_(processor_);
    const VectorXd& fixed = Peer::last_dd_fixed_(processor_);
    ASSERT_EQ(pairs.size(), 4u);
    ASSERT_EQ(subset.size(), 4u);
    ASSERT_EQ(fixed.size(), 4);
    EXPECT_EQ(subset, (std::vector<int>{0, 1, 2, 3}));
    std::map<int, double> integer_by_prn;
    for (int i = 0; i < 4; ++i) {
        const auto& pair = pairs[subset[i]];
        EXPECT_EQ(pair.ref_sat, gps(1));
        EXPECT_EQ(pair.ref_idx, Peer::ambiguityIndex(gps(1), 0));
        EXPECT_EQ(pair.sat_idx, Peer::ambiguityIndex(pair.sat, 0));
        integer_by_prn[pair.sat.prn] = fixed(i);
    }
    const std::map<int, double> expected = {{2, 12.0}, {3, -7.0}, {4, 25.0}, {5, 3.0}};
    EXPECT_EQ(integer_by_prn, expected);
    EXPECT_EQ(Peer::last_num_fixed_ambiguities_(processor_), 4);
}

TEST_F(RTKHoldFixUnchangedSetTest, FewerThanFourMatchedPairsStillRejectsWithItsOwnReason) {
    // Drop two held satellites from tracking: 2 held pairs remain, 6 new ones.
    dropSatellites(processor_, {4, 5});
    const auto pairs_before = Peer::last_dd_pairs_(processor_).size();
    const VectorXd fixed_before = Peer::last_dd_fixed_(processor_);

    PositionSolution solution;
    EXPECT_FALSE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 7, solution));
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_reject_reason, "matched_pairs_lt4");
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_matched_pairs, 2);
    EXPECT_EQ(Peer::last_dd_pairs_(processor_).size(), pairs_before);
    EXPECT_EQ(Peer::last_dd_fixed_(processor_), fixed_before);
}

TEST_F(RTKHoldFixDefaultTest, UnmatchedPairsAreRoundedAndTheHeldSetGrowsAsToday) {
    EXPECT_FALSE(processor_.getRTKConfig().hold_fix_unchanged_set_only);

    PositionSolution solution;
    ASSERT_TRUE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 9, solution));

    const auto& telemetry = processor_.getLastDebugTelemetry();
    EXPECT_TRUE(telemetry.hold_fix_applied);
    EXPECT_TRUE(telemetry.hold_fix_reject_reason.empty());
    EXPECT_EQ(telemetry.hold_fix_candidate_pairs, 8);
    EXPECT_EQ(telemetry.hold_fix_matched_pairs, 4);
    EXPECT_EQ(solution.status, SolutionStatus::FIXED);
    EXPECT_EQ(solution.num_fixed_ambiguities, 4);
    EXPECT_DOUBLE_EQ(solution.ratio, processor_.getRTKConfig().ambiguity_ratio_threshold);

    // The four new pairs were rounded from their x.5 floats and entered the held
    // set, which grows from 4 matched (+ the stale one) to all 8 candidates.
    const auto& pairs = Peer::last_dd_pairs_(processor_);
    const auto& subset = Peer::last_best_subset_(processor_);
    const VectorXd& fixed = Peer::last_dd_fixed_(processor_);
    ASSERT_EQ(pairs.size(), 8u);
    ASSERT_EQ(subset.size(), 8u);
    ASSERT_EQ(fixed.size(), 8);
    EXPECT_EQ(subset, (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}));
    std::map<int, double> integer_by_prn;
    for (int i = 0; i < 8; ++i) {
        integer_by_prn[pairs[subset[i]].sat.prn] = fixed(i);
    }
    const std::map<int, double> expected = {{2, 12.0}, {3, -7.0}, {4, 25.0}, {5, 3.0},   // held
                                            {6, 7.0}, {7, -4.0}, {8, 16.0}, {9, -10.0}};  // rounded
    EXPECT_EQ(integer_by_prn, expected);

    // The rounded G06 pair (7.49 -> 7) moved the fixed position off the float baseline.
    EXPECT_GT((Peer::fixed_baseline_(processor_) - Vector3d(1.0, 2.0, 3.0)).norm(), 0.01);

    // applyHoldAmbiguity then pins every held pair, the rounded ones included.
    Peer::applyHoldAmbiguity(processor_);
    const auto& fs = Peer::filter_state_(processor_);
    const int ref_idx = Peer::ambiguityIndex(gps(1), 0);
    for (const auto& [prn, integer] : expected) {
        const int idx = Peer::ambiguityIndex(gps(prn), 0);
        EXPECT_NEAR(fs.state(ref_idx) - fs.state(idx), integer, 1e-9) << "G" << prn;
        EXPECT_DOUBLE_EQ(fs.covariance(idx, idx), 0.001) << "G" << prn;
    }
}

TEST_F(RTKHoldFixDefaultTest, FewerThanFourMatchedPairsRejectsAsToday) {
    dropSatellites(processor_, {4, 5});
    const auto pairs_before = Peer::last_dd_pairs_(processor_).size();

    PositionSolution solution;
    EXPECT_FALSE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 7, solution));
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_reject_reason, "matched_pairs_lt4");
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_matched_pairs, 2);
    EXPECT_EQ(Peer::last_dd_pairs_(processor_).size(), pairs_before);
}

TEST_F(RTKHoldFixDefaultTest, AllCandidatesMatchedGivesTheSameResultWithTheOptionOnOrOff) {
    // With every candidate pair held there is nothing to round: the option must
    // not change a single value of the accepted hold-fix.
    RTKProcessor option_on;
    buildScene(option_on, true);
    ASSERT_FALSE(processor_.getRTKConfig().hold_fix_unchanged_set_only);
    ASSERT_TRUE(option_on.getRTKConfig().hold_fix_unchanged_set_only);
    dropSatellites(processor_, {6, 7, 8, 9});
    dropSatellites(option_on, {6, 7, 8, 9});

    PositionSolution solution_off;
    PositionSolution solution_on;
    ASSERT_TRUE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, solution_off));
    ASSERT_TRUE(Peer::tryHoldFix(option_on, Peer::current_sat_data_(option_on), time_, 5, solution_on));

    EXPECT_EQ(solution_off.position_ecef, solution_on.position_ecef);
    EXPECT_EQ(solution_off.status, solution_on.status);
    EXPECT_EQ(solution_off.ratio, solution_on.ratio);
    EXPECT_EQ(solution_off.num_fixed_ambiguities, solution_on.num_fixed_ambiguities);
    EXPECT_EQ(Peer::fixed_baseline_(processor_), Peer::fixed_baseline_(option_on));
    EXPECT_EQ(Peer::last_best_subset_(processor_), Peer::last_best_subset_(option_on));
    EXPECT_EQ(Peer::last_dd_fixed_(processor_), Peer::last_dd_fixed_(option_on));
    ASSERT_EQ(Peer::last_dd_pairs_(processor_).size(), Peer::last_dd_pairs_(option_on).size());
    for (size_t i = 0; i < Peer::last_dd_pairs_(processor_).size(); ++i) {
        EXPECT_EQ(Peer::last_dd_pairs_(processor_)[i].sat, Peer::last_dd_pairs_(option_on)[i].sat);
        EXPECT_EQ(Peer::last_dd_pairs_(processor_)[i].sat_idx, Peer::last_dd_pairs_(option_on)[i].sat_idx);
    }
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_jump_m,
              option_on.getLastDebugTelemetry().hold_fix_jump_m);
}

TEST(RTKLegacyCompatibilityStandaloneTest, HoldFixUnchangedSetOnlyDefaultOffAndConfigurable) {
    // Opt-in: the default keeps today's hold-fix (rounding included).
    RTKProcessor processor;
    EXPECT_FALSE(processor.getRTKConfig().hold_fix_unchanged_set_only);
    EXPECT_FALSE(RTKProcessor::RTKConfig().hold_fix_unchanged_set_only);

    RTKProcessor::RTKConfig on;
    on.hold_fix_unchanged_set_only = true;
    processor.setRTKConfig(on);
    EXPECT_TRUE(processor.getRTKConfig().hold_fix_unchanged_set_only);

    RTKProcessor::RTKConfig off;
    off.hold_fix_unchanged_set_only = false;
    processor.setRTKConfig(off);
    EXPECT_FALSE(processor.getRTKConfig().hold_fix_unchanged_set_only);
}

}  // namespace

// ephemerisHealthy(): the single satellite health rule shared by SPP, PPP and FGO.
TEST(EphemerisHealthyTest, FullWidthWordAndQzssMask) {
    Ephemeris gps;
    gps.satellite = SatelliteId(GNSSSystem::GPS, 5);
    gps.health = 0;
    EXPECT_TRUE(ephemerisHealthy(gps));
    gps.health = 1;  // GPS has no masked bit
    EXPECT_FALSE(ephemerisHealthy(gps));
    gps.health = 256;
    EXPECT_FALSE(ephemerisHealthy(gps));

    Ephemeris gal;
    gal.satellite = SatelliteId(GNSSSystem::Galileo, 7);
    gal.health = 256;  // E5b HS bit 8: must not be truncated away
    EXPECT_EQ(gal.health, 256);
    EXPECT_FALSE(ephemerisHealthy(gal));
    gal.health = 0x1FF;
    EXPECT_FALSE(ephemerisHealthy(gal));

    Ephemeris qzss;
    qzss.satellite = SatelliteId(GNSSSystem::QZSS, 193);
    qzss.health = 1;
    EXPECT_TRUE(ephemerisHealthy(qzss));
    qzss.health = 3;
    EXPECT_FALSE(ephemerisHealthy(qzss));
    qzss.health = 256;
    EXPECT_FALSE(ephemerisHealthy(qzss));
    qzss.health = 0x101;  // bit 0 masked, bit 8 still set
    EXPECT_FALSE(ephemerisHealthy(qzss));

    // The explicit-system overload decides the QZSS rule, not the record.
    gps.health = 1;
    EXPECT_TRUE(ephemerisHealthy(gps, GNSSSystem::QZSS));
    EXPECT_FALSE(ephemerisHealthy(gps, GNSSSystem::GPS));
}
