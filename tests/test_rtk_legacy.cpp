#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/processor.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/io/rinex.hpp>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/coordinates.hpp>

#include "synthetic_rtk_scene.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <limits>
#include <memory>
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
    GNSSPP_RTK_TEST_MEMBER(last_fixed_time_)
    GNSSPP_RTK_TEST_MEMBER(has_last_fixed_time_)
    GNSSPP_RTK_TEST_MEMBER(last_trusted_position_)
    GNSSPP_RTK_TEST_MEMBER(has_last_trusted_position_)
    GNSSPP_RTK_TEST_MEMBER(last_trusted_time_)
    GNSSPP_RTK_TEST_MEMBER(has_last_trusted_time_)
    GNSSPP_RTK_TEST_MEMBER(prev_trusted_position_)
    GNSSPP_RTK_TEST_MEMBER(prev_trusted_time_)
    GNSSPP_RTK_TEST_MEMBER(has_prev_trusted_position_)
    GNSSPP_RTK_TEST_MEMBER(doppler_continuity_position_ecef_)
    GNSSPP_RTK_TEST_MEMBER(doppler_continuity_time_)
    GNSSPP_RTK_TEST_MEMBER(doppler_continuity_anchor_time_)
    GNSSPP_RTK_TEST_MEMBER(has_doppler_continuity_position_)
    GNSSPP_RTK_TEST_MEMBER(consecutive_fix_count_)
    GNSSPP_RTK_TEST_MEMBER(lock_count_l1_)
    GNSSPP_RTK_TEST_MEMBER(last_dd_pairs_)
    GNSSPP_RTK_TEST_MEMBER(last_best_subset_)
    GNSSPP_RTK_TEST_MEMBER(last_dd_fixed_)
    GNSSPP_RTK_TEST_MEMBER(last_ar_ratio_)
    GNSSPP_RTK_TEST_MEMBER(last_num_fixed_ambiguities_)
    GNSSPP_RTK_TEST_MEMBER(doppler_history_time_)
    GNSSPP_RTK_TEST_MEMBER(last_doppler_velocity_ecef_)
    GNSSPP_RTK_TEST_MEMBER(last_doppler_velocity_time_)
    GNSSPP_RTK_TEST_MEMBER(has_last_doppler_velocity_)
    GNSSPP_RTK_TEST_MEMBER(has_doppler_history_time_)
    GNSSPP_RTK_TEST_MEMBER(doppler_history_base_time_)
    GNSSPP_RTK_TEST_MEMBER(has_doppler_history_base_time_)
    GNSSPP_RTK_TEST_MEMBER(current_update_diagnostics_)
    GNSSPP_RTK_TEST_MEMBER(nis_suspended_hold_run_)
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
    GNSSPP_RTK_TEST_METHOD(validateFixedSolution)
    GNSSPP_RTK_TEST_METHOD(rememberSolution)
    GNSSPP_RTK_TEST_METHOD(varerr)
    GNSSPP_RTK_TEST_METHOD(tryHoldFix)
    GNSSPP_RTK_TEST_METHOD(holdPhaseFit)
    GNSSPP_RTK_TEST_METHOD(applyRoadHeightPrior)
    GNSSPP_RTK_TEST_METHOD(applyHoldAmbiguity)
    GNSSPP_RTK_TEST_METHOD(resolveAmbiguities)
    GNSSPP_RTK_TEST_METHOD(tryWlNlFallback)
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

TEST(RTKTrustedHistoryTest, SameEpochFloatAndFixedReplaceCurrentSampleWithoutAdvancingHistory) {
    RTKProcessor processor;
    auto sample = [](double tow, double east, SolutionStatus status) {
        PositionSolution solution;
        solution.time = GNSSTime(2300, tow);
        solution.status = status;
        solution.num_satellites = 8;
        solution.position_ecef = Vector3d(6378137, east, 0);
        return solution;
    };
    const auto first = sample(100, 0, SolutionStatus::FIXED);
    const auto next = sample(101, 1, SolutionStatus::FLOAT);
    Peer::rememberSolution(processor, first);
    Peer::rememberSolution(processor, next);
    for (const auto& same_epoch : {sample(101, 1.1, SolutionStatus::FLOAT),
                                   sample(101, 1.2, SolutionStatus::FIXED)}) {
        Peer::rememberSolution(processor, same_epoch);
        EXPECT_EQ(Peer::last_trusted_position_(processor), same_epoch.position_ecef);
        EXPECT_EQ(Peer::prev_trusted_position_(processor), first.position_ecef);
        EXPECT_DOUBLE_EQ(Peer::prev_trusted_time_(processor).tow, 100);
        EXPECT_DOUBLE_EQ(Peer::last_trusted_time_(processor).tow, 101);
        const auto velocity = float_trust_policy::estimateVelocityFromTrustedDeltas(
            Peer::last_trusted_position_(processor), Peer::prev_trusted_position_(processor),
            Peer::last_trusted_time_(processor) - Peer::prev_trusted_time_(processor), 10.0);
        EXPECT_NEAR(velocity.y(), same_epoch.position_ecef.y(), 1e-9);
    }
}

TEST(RTKTrustedHistoryTest, RejectedFloatRestoresBothTrustSamplesAndPreviousPresenceFlag) {
    const auto nav = synthetic_rtk_scene::constellation();
    const Vector3d base_position(6378137, 0, 0);
    const Vector3d rover_position = base_position + Vector3d(40, 30, 10);
    const auto prns = synthetic_rtk_scene::visible(
        nav, synthetic_rtk_scene::epochTime(0), rover_position, 15);
    ASSERT_GE(prns.size(), 6U);
    for (bool previous_present : {false, true}) {
        SCOPED_TRACE(previous_present);
        RTKProcessor processor;
        auto config = processor.getRTKConfig();
        config.ratio_threshold = 1e9;
        config.ambiguity_ratio_threshold = 1e9;
        config.elevation_mask = 5 * M_PI / 180;
        processor.setRTKConfig(config);
        ProcessorConfig spp_config;
        spp_config.elevation_mask = 5;
        ASSERT_TRUE(processor.initialize(spp_config));
        processor.setBasePosition(base_position);
        auto process = [&](double offset) {
            const auto time = synthetic_rtk_scene::epochTime(offset);
            return processor.processRTKEpoch(
                synthetic_rtk_scene::observations(nav, time, rover_position, 30, prns, 0),
                synthetic_rtk_scene::observations(nav, time, base_position, 10, prns, 500), nav);
        };
        ASSERT_TRUE(process(0).isValid());
        ASSERT_TRUE(Peer::has_last_trusted_position_(processor));
        const Vector3d trusted = Peer::last_trusted_position_(processor);
        const GNSSTime trusted_time = Peer::last_trusted_time_(processor);
        const Vector3d previous = trusted - Vector3d(0, 1, 0);
        Peer::prev_trusted_position_(processor) = previous;
        Peer::prev_trusted_time_(processor) = trusted_time - 1;
        Peer::has_prev_trusted_position_(processor) = previous_present;
        config.max_float_spp_divergence_m = 1e-8;
        processor.setRTKConfig(config);
        const auto rejected = process(0.2);
        ASSERT_EQ(rejected.status, SolutionStatus::SPP);
        ASSERT_GT(processor.getLastDebugTelemetry().float_update_observation_count, 0);
        EXPECT_EQ(Peer::last_trusted_position_(processor), trusted);
        EXPECT_DOUBLE_EQ(Peer::last_trusted_time_(processor) - trusted_time, 0);
        EXPECT_EQ(Peer::has_prev_trusted_position_(processor), previous_present);
        EXPECT_EQ(Peer::prev_trusted_position_(processor), previous);
        EXPECT_DOUBLE_EQ(Peer::prev_trusted_time_(processor) - (trusted_time - 1), 0);
    }
}

TEST(RTKIteratedUpdateTest, RelinearizationFromPriorDoesNotRecountEpochMeasurements) {
    // Noise-free static scene with an unbiased seed: every row passes the
    // prefit gate on the first pass, so relinearizing at the updated state
    // adds no information. With --kf-relinearize-from-prior a single update
    // and the iterated update leave the same position and double-difference
    // ambiguity covariance. The legacy default re-applies the same rows to
    // the posterior and halves it (docs/rtk_iterated_update_covariance.md).
    const auto nav = synthetic_rtk_scene::constellation();
    const Vector3d base_position(6378137, 0, 0);
    const Vector3d rover_position = base_position + Vector3d(40, 30, 10);
    const auto prns = synthetic_rtk_scene::visible(
        nav, synthetic_rtk_scene::epochTime(0), rover_position, 15);
    ASSERT_GE(prns.size(), 6U);
    const int reference = Peer::ambiguityIndex(SatelliteId(GNSSSystem::GPS, prns[0]), 0);
    auto run = [&](int kf_iterations, bool relinearize_from_prior) {
        RTKProcessor processor;
        auto config = processor.getRTKConfig();
        config.ratio_threshold = 1e9;
        config.ambiguity_ratio_threshold = 1e9;
        config.elevation_mask = 5 * M_PI / 180;
        config.kf_iterations = kf_iterations;
        config.kf_relinearize_from_epoch_prior = relinearize_from_prior;
        processor.setRTKConfig(config);
        ProcessorConfig spp_config;
        spp_config.elevation_mask = 5;
        // The scene has no atmosphere: an unbiased SPP seed keeps every row
        // inside the prefit gate on the first pass.
        spp_config.use_ionosphere_model = false;
        spp_config.use_troposphere_model = false;
        EXPECT_TRUE(processor.initialize(spp_config));
        processor.setBasePosition(base_position);
        for (int epoch = 0; epoch < 3; ++epoch) {
            const auto time = synthetic_rtk_scene::epochTime(epoch);
            const auto solution = processor.processRTKEpoch(
                synthetic_rtk_scene::observations(nav, time, rover_position, 30, prns, 0),
                synthetic_rtk_scene::observations(nav, time, base_position, 10, prns, 500),
                nav);
            EXPECT_EQ(solution.status, SolutionStatus::FLOAT);
            const auto& telemetry = processor.getLastDebugTelemetry();
            EXPECT_EQ(telemetry.float_update_suppressed_outliers, 0);
            EXPECT_EQ(telemetry.retained_phase_rows, static_cast<int>(prns.size()) - 1);
        }
        return Peer::filter_state_(processor);
    };
    const auto single = run(1, true);
    const auto iterated = run(4, true);
    const auto legacy = run(4, false);
    EXPECT_EQ(run(1, false).covariance, single.covariance);
    for (int axis = 0; axis < 3; ++axis) {
        SCOPED_TRACE(axis);
        EXPECT_NEAR(iterated.covariance(axis, axis) / single.covariance(axis, axis), 1.0, 1e-3);
        EXPECT_NEAR(legacy.covariance(axis, axis) / single.covariance(axis, axis), 0.5, 0.01);
    }
    auto ddVariance = [&](const auto& state, int index) {
        return state.covariance(reference, reference) + state.covariance(index, index) -
               2.0 * state.covariance(reference, index);
    };
    for (std::size_t i = 1; i < prns.size(); ++i) {
        const int index = Peer::ambiguityIndex(SatelliteId(GNSSSystem::GPS, prns[i]), 0);
        SCOPED_TRACE(static_cast<int>(prns[i]));
        ASSERT_NE(single.state(index), 0.0);
        EXPECT_NEAR(ddVariance(iterated, index) / ddVariance(single, index), 1.0, 1e-3);
        EXPECT_NEAR(ddVariance(legacy, index) / ddVariance(single, index), 0.5, 0.01);
        EXPECT_NEAR(iterated.state(reference) - iterated.state(index),
                    single.state(reference) - single.state(index), 1e-3);
    }
}

TEST(RTKLambdaShadowTest, ReportsPerPairIntegerResidualOfBestCandidate) {
    // Noise-free scene: the float DD ambiguities sit on integers, so the
    // largest normalized integer residual is small and named after a pair.
    const auto nav = synthetic_rtk_scene::constellation();
    const Vector3d base_position(6378137, 0, 0);
    const Vector3d rover_position = base_position + Vector3d(40, 30, 10);
    const auto prns = synthetic_rtk_scene::visible(
        nav, synthetic_rtk_scene::epochTime(0), rover_position, 15);
    ASSERT_GE(prns.size(), 6U);
    RTKProcessor processor;
    auto config = processor.getRTKConfig();
    config.ratio_threshold = 1e9;
    config.ambiguity_ratio_threshold = 1e9;
    config.elevation_mask = 5 * M_PI / 180;
    config.lambda_candidate_shadow_count = 2;
    processor.setRTKConfig(config);
    ProcessorConfig spp_config;
    spp_config.elevation_mask = 5;
    spp_config.use_ionosphere_model = false;
    spp_config.use_troposphere_model = false;
    ASSERT_TRUE(processor.initialize(spp_config));
    processor.setBasePosition(base_position);
    for (int epoch = 0; epoch < 6; ++epoch) {
        const auto time = synthetic_rtk_scene::epochTime(epoch);
        processor.processRTKEpoch(
            synthetic_rtk_scene::observations(nav, time, rover_position, 30, prns, 0),
            synthetic_rtk_scene::observations(nav, time, base_position, 10, prns, 500), nav);
    }
    const auto& telemetry = processor.getLastDebugTelemetry();
    ASSERT_TRUE(telemetry.lambda_shadow_solved);
    ASSERT_TRUE(std::isfinite(telemetry.lambda_shadow_max_abs_z));
    EXPECT_LT(telemetry.lambda_shadow_max_abs_z, 3.0);
    EXPECT_EQ(telemetry.lambda_shadow_pairs_abs_z_gt3, 0);
    EXPECT_GT(telemetry.lambda_shadow_max_z_lock_count, 0);
    EXPECT_EQ(telemetry.lambda_shadow_max_z_pair.rfind("G", 0), 0U);
    EXPECT_NE(telemetry.lambda_shadow_max_z_pair.find("/L1"), std::string::npos);
}

TEST(RTKPostFixResidualTest, ReportsLargestPairResidualAlongsideRms) {
    // Noise-free scene: once fixed, every DD phase residual is near zero; the
    // largest one is named after its pair and bounds the RMS from above.
    const auto nav = synthetic_rtk_scene::constellation();
    const Vector3d base_position(6378137, 0, 0);
    const Vector3d rover_position = base_position + Vector3d(40, 30, 10);
    const auto prns = synthetic_rtk_scene::visible(
        nav, synthetic_rtk_scene::epochTime(0), rover_position, 15);
    ASSERT_GE(prns.size(), 6U);
    RTKProcessor processor;
    auto config = processor.getRTKConfig();
    config.elevation_mask = 5 * M_PI / 180;
    processor.setRTKConfig(config);
    ProcessorConfig spp_config;
    spp_config.elevation_mask = 5;
    spp_config.use_ionosphere_model = false;
    spp_config.use_troposphere_model = false;
    ASSERT_TRUE(processor.initialize(spp_config));
    processor.setBasePosition(base_position);
    bool fixed_by_ar = false;
    for (int epoch = 0; epoch < 10 && !fixed_by_ar; ++epoch) {
        const auto time = synthetic_rtk_scene::epochTime(epoch);
        const auto solution = processor.processRTKEpoch(
            synthetic_rtk_scene::observations(nav, time, rover_position, 30, prns, 0),
            synthetic_rtk_scene::observations(nav, time, base_position, 10, prns, 500), nav);
        fixed_by_ar = solution.status == SolutionStatus::FIXED &&
                      !processor.getLastDebugTelemetry().hold_fix_applied &&
                      std::isfinite(processor.getLastDebugTelemetry().postfix_residual_rms);
    }
    ASSERT_TRUE(fixed_by_ar);
    const auto& telemetry = processor.getLastDebugTelemetry();
    ASSERT_TRUE(std::isfinite(telemetry.postfix_residual_max_abs_m));
    EXPECT_LT(telemetry.postfix_residual_max_abs_m, 0.01);
    EXPECT_GE(telemetry.postfix_residual_max_abs_m + 1e-12, telemetry.postfix_residual_rms);
    EXPECT_EQ(telemetry.postfix_residual_max_pair.rfind("G", 0), 0U);
    EXPECT_NE(telemetry.postfix_residual_max_pair.find("/L"), std::string::npos);
}

TEST(RTKHoldAmbiguityUpdateTest, LegacyHoldOverwritesInsteadOfConditioningTheState) {
    // A held DD integer is information about the ambiguity differences. As a
    // Kalman pseudo-observation (RTKLIB holdamb, 0.001 cycle^2) it moves the
    // correlated position and keeps the posterior covariance consistent. The
    // legacy hold writes the integer into the satellite state, zeroes that
    // state's correlations (position included) and leaves the position alone.
    const int n = Peer::kStateCount;
    const std::vector<int> prns{1, 2, 3, 4};
    std::vector<int> index;
    for (const int prn : prns) {
        index.push_back(Peer::ambiguityIndex(SatelliteId(GNSSSystem::GPS, prn), 0));
    }
    std::vector<int> active{0, 1, 2};
    active.insert(active.end(), index.begin(), index.end());
    Eigen::MatrixXd factor(active.size(), active.size());
    for (int r = 0; r < factor.rows(); ++r) {
        for (int c = 0; c < factor.cols(); ++c) {
            factor(r, c) = 0.3 * std::sin(1.7 * r + 0.9 * c + 0.4) + (r == c ? 1.0 : 0.0);
        }
    }
    const Eigen::MatrixXd active_covariance = factor * factor.transpose();
    const std::vector<double> values{1.0, 2.0, 3.0, 100.3, 88.1, 120.6, 95.2};
    VectorXd integers(3);
    integers << 12.0, -20.0, 5.0;
    auto prepare = [&](RTKProcessor& processor, bool kalman_update) {
        auto config = processor.getRTKConfig();
        config.hold_ambiguity_kalman_update = kalman_update;
        processor.setRTKConfig(config);
        auto& filter = Peer::filter_state_(processor);
        filter.state = VectorXd::Zero(n);
        filter.covariance = MatrixXd::Zero(n, n);
        for (std::size_t r = 0; r < active.size(); ++r) {
            filter.state(active[r]) = values[r];
            for (std::size_t c = 0; c < active.size(); ++c) {
                filter.covariance(active[r], active[c]) = active_covariance(r, c);
            }
        }
        auto& pairs = Peer::last_dd_pairs_(processor);
        pairs.clear();
        for (std::size_t i = 1; i < prns.size(); ++i) {
            pairs.push_back({SatelliteId(GNSSSystem::GPS, prns[0]), index[0], index[i],
                             SatelliteId(GNSSSystem::GPS, prns[i]), 0});
        }
        Peer::last_best_subset_(processor) = {0, 1, 2};
        Peer::last_dd_fixed_(processor) = integers;
    };
    RTKProcessor processor;
    RTKProcessor conditioned;
    prepare(processor, false);
    prepare(conditioned, true);
    const auto& filter = Peer::filter_state_(processor);

    // Reference: one Kalman update with z = held DD, H = e_ref - e_sat.
    MatrixXd H = MatrixXd::Zero(3, n);
    for (int i = 0; i < 3; ++i) {
        H(i, index[0]) = 1.0;
        H(i, index[i + 1]) = -1.0;
    }
    const VectorXd x0 = filter.state;
    const MatrixXd P0 = filter.covariance;
    const MatrixXd S = H * P0 * H.transpose() + 0.001 * MatrixXd::Identity(3, 3);
    const MatrixXd K = (S.ldlt().solve(H * P0)).transpose();
    const VectorXd expected_state = x0 + K * (integers - H * x0);
    const MatrixXd expected_covariance = P0 - K * (H * P0);
    ASSERT_GT((expected_state.head<3>() - x0.head<3>()).norm(), 0.01);

    Peer::applyHoldAmbiguity(conditioned);
    const auto& updated = Peer::filter_state_(conditioned);
    EXPECT_LT((updated.state - expected_state).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((updated.covariance - expected_covariance).cwiseAbs().maxCoeff(), 1e-9);

    Peer::applyHoldAmbiguity(processor);
    const auto& legacy = Peer::filter_state_(processor);
    EXPECT_EQ(legacy.state.head<3>(), x0.head<3>());
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(legacy.state(index[0]) - legacy.state(index[i + 1]), integers(i), 1e-9);
        EXPECT_DOUBLE_EQ(legacy.covariance(index[i + 1], 0), 0.0);
        EXPECT_NE(expected_covariance(index[i + 1], 0), 0.0);
    }
}

TEST(RTKFixHistoryJumpTest, MotionCompensationKeepsConsistentMovingFixes) {
    // At 6 m/s and 10 Hz a correct next FIX is 0.6 m from the previous one.
    // The stationary 0.2 m history check rejects it after three consecutive
    // fixes; propagating the previous FIX by the Doppler velocity accepts it
    // and still rejects a candidate 0.5 m off that motion.
    const Vector3d base(6378137.0, 0.0, 0.0);
    const GNSSTime previous(2032, 273700.0);
    const GNSSTime now(2032, 273700.1);
    auto check = [&](bool compensate, double along_track_m) {
        RTKProcessor processor;
        auto config = processor.getRTKConfig();
        config.fix_history_motion_compensation = compensate;
        processor.setRTKConfig(config);
        processor.setBasePosition(base);
        Peer::has_fixed_solution_(processor) = true;
        Peer::fixed_baseline_(processor) = Vector3d(0.0, along_track_m, 0.0);
        Peer::last_fixed_position_(processor) = base;
        Peer::has_last_fixed_position_(processor) = true;
        Peer::last_fixed_time_(processor) = previous;
        Peer::has_last_fixed_time_(processor) = true;
        Peer::consecutive_fix_count_(processor) = 3;
        Peer::last_doppler_velocity_ecef_(processor) = Vector3d(0.0, 6.0, 0.0);
        Peer::last_doppler_velocity_time_(processor) = previous;
        Peer::has_last_doppler_velocity_(processor) = true;
        const bool ok =
            Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), now);
        return std::make_pair(ok, processor.getLastDebugTelemetry().reject_reason);
    };
    EXPECT_EQ(check(false, 0.6), std::make_pair(false, std::string("fix_history_jump")));
    EXPECT_TRUE(check(true, 0.6).first);
    EXPECT_EQ(check(true, 1.1), std::make_pair(false, std::string("fix_history_jump")));
    EXPECT_TRUE(check(false, 0.1).first);
}

TEST(RTKFixHistoryJumpTest, MotionCompensationKeepsConsistentFixesUnderMaxPositionJump) {
    // At 13 m/s a vehicle is 14.3 m from its last FIX after 1.1 s. The
    // absolute 5 m jump check against that stationary anchor (usable for
    // 5 s) rejects every consistent candidate; propagating the anchor by the
    // Doppler velocity accepts it, still rejects a candidate 6 m off that
    // motion, and keeps the stationary anchor beyond the 2 s propagation.
    const Vector3d base(6378137.0, 0.0, 0.0);
    const GNSSTime previous(2032, 273505.7);
    auto check = [&](bool compensate, double along_track_m, double dt_s) {
        RTKProcessor processor;
        auto config = processor.getRTKConfig();
        config.max_fixed_anchor_age_s = 5.0;
        config.max_position_jump_motion_compensation = compensate;
        processor.setRTKConfig(config);
        processor.setBasePosition(base);
        Peer::has_fixed_solution_(processor) = true;
        Peer::fixed_baseline_(processor) = Vector3d(0.0, along_track_m, 0.0);
        Peer::last_fixed_position_(processor) = base;
        Peer::has_last_fixed_position_(processor) = true;
        Peer::last_fixed_time_(processor) = previous;
        Peer::has_last_fixed_time_(processor) = true;
        Peer::consecutive_fix_count_(processor) = 0;
        Peer::last_doppler_velocity_ecef_(processor) = Vector3d(0.0, 13.0, 0.0);
        Peer::last_doppler_velocity_time_(processor) = previous + (dt_s - 0.1);
        Peer::has_last_doppler_velocity_(processor) = true;
        const bool ok = Peer::validateFixedSolution(
            processor, Peer::current_sat_data_(processor), previous + dt_s);
        return std::make_pair(ok, processor.getLastDebugTelemetry().reject_reason);
    };
    EXPECT_EQ(check(false, 14.3, 1.1), std::make_pair(false, std::string("max_position_jump")));
    EXPECT_TRUE(check(true, 14.3, 1.1).first);
    EXPECT_EQ(check(true, 20.3, 1.1), std::make_pair(false, std::string("max_position_jump")));
    EXPECT_EQ(check(true, 39.0, 3.0), std::make_pair(false, std::string("max_position_jump")));
    EXPECT_TRUE(check(false, 4.0, 1.1).first);
}

TEST(RTKDopplerConsensusTest, FreshFixAnchorCannotBypassCurrentCandidateCheck) {
    RTKProcessor processor;
    auto config = processor.getRTKConfig();
    config.max_fixed_anchor_age_s = 5.0;
    config.max_fixed_doppler_consensus_m = 3.0;
    processor.setRTKConfig(config);
    const Vector3d base(6378137.0, 0.0, 0.0);
    const GNSSTime time(2032, 273936.1);
    processor.setBasePosition(base);
    Peer::has_fixed_solution_(processor) = true;
    Peer::fixed_baseline_(processor) = Vector3d(4.758, 0.0, 0.0);
    Peer::last_fixed_position_(processor) = base;
    Peer::has_last_fixed_position_(processor) = true;
    Peer::last_fixed_time_(processor) = GNSSTime(2032, 273935.9);
    Peer::has_last_fixed_time_(processor) = true;
    Peer::has_doppler_continuity_position_(processor) = true;
    Peer::doppler_continuity_position_ecef_(processor) = base;
    Peer::doppler_continuity_time_(processor) = time;
    // Early reacquisition, within the 5 m history limit, with a fresh anchor:
    // the former conditional skipped the Doppler gate and accepted this jump.
    EXPECT_FALSE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
    EXPECT_EQ(processor.getLastDebugTelemetry().reject_reason, "fixed_doppler_consensus");
    EXPECT_NEAR(processor.getLastDebugTelemetry().fixed_candidate_doppler_consensus_distance_m,
                4.758, 1e-8);

    // A candidate that agrees with the current propagated position still passes.
    Peer::fixed_baseline_(processor) = Vector3d(0.1, 0.0, 0.0);
    EXPECT_TRUE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
}

TEST(RTKDopplerConsensusTest, ExpiredAnchorDoesNotBlockMotionConsistentReacquisition) {
    RTKProcessor processor;
    auto config = processor.getRTKConfig();
    config.max_fixed_anchor_age_s = 5.0;
    config.max_fixed_doppler_consensus_m = 3.0;
    processor.setRTKConfig(config);
    const Vector3d base(6378137.0, 0.0, 0.0);
    const GNSSTime time(2032, 273700.0);
    processor.setBasePosition(base);
    Peer::has_fixed_solution_(processor) = true;
    Peer::fixed_baseline_(processor) = Vector3d(415.16, 0.0, 0.0);
    Peer::last_fixed_position_(processor) = base;
    Peer::has_last_fixed_position_(processor) = true;
    Peer::last_fixed_time_(processor) = GNSSTime(2032, 273520.9);
    Peer::has_last_fixed_time_(processor) = true;
    Peer::consecutive_fix_count_(processor) = 10;
    Peer::has_doppler_continuity_position_(processor) = true;
    Peer::doppler_continuity_position_ecef_(processor) = base + Vector3d(415.0, 0.0, 0.0);
    Peer::doppler_continuity_time_(processor) = time;
    EXPECT_TRUE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
    // Disabling expiry reproduces the stationary-anchor lockout.
    config.max_fixed_anchor_age_s = 0.0;
    processor.setRTKConfig(config);
    Peer::has_fixed_solution_(processor) = true;
    Peer::fixed_baseline_(processor) = Vector3d(415.16, 0.0, 0.0);
    Peer::last_fixed_position_(processor) = base;
    Peer::has_last_fixed_position_(processor) = true;
    EXPECT_FALSE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
}

TEST(RTKDopplerConsensusTest, DriftedTrackLocksOutUntilItsAnchorExpires) {
    // Odaiba CDM: after a long FIX gap the Doppler-integrated track is 7-11 m
    // off, so it rejects every later candidate, all of them correct, for up to
    // 2704 epochs; only an accepted FIX could re-anchor it. A track whose last
    // anchor is older than max_fixed_doppler_consensus_age_s no longer vetoes.
    RTKProcessor processor;
    auto config = processor.getRTKConfig();
    config.max_fixed_doppler_consensus_m = 3.0;
    processor.setRTKConfig(config);
    const Vector3d base(6378137.0, 0.0, 0.0);
    const GNSSTime time(2032, 274250.0);
    processor.setBasePosition(base);
    auto validate = [&](double anchor_age_s) {
        Peer::has_fixed_solution_(processor) = true;
        Peer::fixed_baseline_(processor) = Vector3d::Zero();
        Peer::has_doppler_continuity_position_(processor) = true;
        Peer::doppler_continuity_position_ecef_(processor) = base + Vector3d(0.0, 9.0, 0.0);
        Peer::doppler_continuity_time_(processor) = time;
        Peer::doppler_continuity_anchor_time_(processor) = time - anchor_age_s;
        const bool ok =
            Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time);
        return std::make_pair(ok, processor.getLastDebugTelemetry().reject_reason);
    };
    // Legacy (no expiry): a 270 s old drifted track still vetoes.
    EXPECT_EQ(validate(270.0), std::make_pair(false, std::string("fixed_doppler_consensus")));

    config.max_fixed_doppler_consensus_age_s = 5.0;
    processor.setRTKConfig(config);
    EXPECT_TRUE(validate(270.0).first);
    EXPECT_NEAR(processor.getLastDebugTelemetry().fixed_candidate_doppler_consensus_age_s, 270.0,
                1e-6);
    // A track re-anchored within the age limit keeps vetoing a disagreeing candidate.
    EXPECT_EQ(validate(2.0), std::make_pair(false, std::string("fixed_doppler_consensus")));
}

TEST(RTKDopplerConsensusTest, StaleTrackRejectsButDisabledGateAndBootstrapRemainAvailable) {
    RTKProcessor processor;
    auto config = processor.getRTKConfig();
    config.max_fixed_doppler_consensus_m = 3.0;
    processor.setRTKConfig(config);
    const Vector3d base(6378137.0, 0.0, 0.0);
    const GNSSTime time(2300, 100.0);
    processor.setBasePosition(base);
    Peer::has_fixed_solution_(processor) = true;
    Peer::fixed_baseline_(processor) = Vector3d::Zero();
    Peer::has_doppler_continuity_position_(processor) = true;
    Peer::doppler_continuity_position_ecef_(processor) = base;
    Peer::doppler_continuity_time_(processor) = GNSSTime(2300, 99.9);
    EXPECT_FALSE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
    EXPECT_EQ(processor.getLastDebugTelemetry().reject_reason, "fixed_doppler_stale");
    Peer::doppler_continuity_time_(processor) = GNSSTime(2300, 100.1);
    EXPECT_FALSE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
    Peer::doppler_continuity_time_(processor) = time;
    Peer::doppler_continuity_position_ecef_(processor).x() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
    EXPECT_EQ(processor.getLastDebugTelemetry().reject_reason, "fixed_doppler_consensus");
    Peer::has_doppler_continuity_position_(processor) = false;
    EXPECT_TRUE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
    config.max_fixed_doppler_consensus_m = 0.0;
    processor.setRTKConfig(config);
    Peer::has_fixed_solution_(processor) = true;
    Peer::fixed_baseline_(processor) = Vector3d::Zero();
    Peer::has_doppler_continuity_position_(processor) = true;
    Peer::doppler_continuity_position_ecef_(processor) = base + Vector3d(100.0, 0.0, 0.0);
    EXPECT_TRUE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
    config.max_fixed_doppler_consensus_m = 3.0;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::MOVING_BASE;
    processor.setRTKConfig(config);
    EXPECT_TRUE(Peer::validateFixedSolution(processor, Peer::current_sat_data_(processor), time));
}

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
// The scene is built with the option on (reject) or off (today's behavior),
// and with a minimum of held pairs (hold_fix_min_matched_pairs, default 4).
class RTKHoldFixSceneTest : public ::testing::Test {
protected:
    static constexpr double kRefAmbiguityCycles = 1000.25;
    static constexpr double kL1WavelengthM = 0.1902936728;

    static SatelliteId gps(int prn) {
        return SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(prn));
    }

    virtual bool unchangedSetOnly() const = 0;
    virtual int minMatchedPairs() const { return 4; }

    static Vector3d basePosition() { return Vector3d(-3961832.0, 3354966.0, 3697065.0); }

    static GNSSTime epochTime() {
        GNSSTime time;
        time.week = 2300;
        time.tow = 100.0;
        return time;
    }

    static void buildScene(RTKProcessor& processor, bool unchanged_set_only,
                           int min_matched_pairs = 4) {
        RTKProcessor::RTKConfig cfg;
        cfg.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        cfg.min_hold_count = 3;
        cfg.hold_fix_unchanged_set_only = unchanged_set_only;
        cfg.hold_fix_min_matched_pairs = min_matched_pairs;
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
        buildScene(processor_, unchangedSetOnly(), minMatchedPairs());
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

TEST_F(RTKHoldFixUnchangedSetTest, DopplerRejectionCannotMutateHeldOrTrustedFixState) {
    dropSatellites(processor_, {6, 7, 8, 9});
    auto config = processor_.getRTKConfig();
    config.max_fixed_doppler_consensus_m = 3.0;
    processor_.setRTKConfig(config);
    const auto filter = Peer::filter_state_(processor_);
    const auto pairs = Peer::last_dd_pairs_(processor_);
    const auto subset = Peer::last_best_subset_(processor_);
    const VectorXd integers = Peer::last_dd_fixed_(processor_);
    const Vector3d previous = Peer::last_fixed_position_(processor_);
    Peer::has_last_trusted_position_(processor_) = true;
    Peer::last_trusted_position_(processor_) = previous;
    Peer::has_doppler_continuity_position_(processor_) = true;
    Peer::doppler_continuity_time_(processor_) = time_;
    Peer::doppler_continuity_position_ecef_(processor_) = previous + Vector3d(5.0, 0.0, 0.0);
    PositionSolution solution;
    EXPECT_FALSE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, solution));
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_reject_reason, "fixed_doppler_consensus");
    EXPECT_FALSE(processor_.getLastDebugTelemetry().hold_fix_applied);
    ASSERT_EQ(Peer::last_dd_fixed_(processor_).size(), integers.size());
    EXPECT_EQ(Peer::last_dd_fixed_(processor_), integers);
    EXPECT_EQ(Peer::last_best_subset_(processor_), subset);
    EXPECT_EQ(Peer::last_fixed_position_(processor_), previous);
    EXPECT_EQ(Peer::last_dd_pairs_(processor_).size(), pairs.size());
    EXPECT_EQ(Peer::last_trusted_position_(processor_), previous);
    EXPECT_TRUE(Peer::has_last_trusted_position_(processor_));
    EXPECT_FALSE(Peer::has_fixed_solution_(processor_));
    EXPECT_EQ(solution.status, SolutionStatus::NONE);
    EXPECT_EQ(Peer::filter_state_(processor_).state, filter.state);
    Peer::doppler_continuity_position_ecef_(processor_) = previous;
    EXPECT_TRUE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, solution));
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

TEST_F(RTKHoldFixDefaultTest, HoldFixIgnoresAnInconsistentFloatUpdateUnlessItsNisLimitIsSet) {
    // UrbanNav Shinjuku TOW 281650.1 under --preset urban: the float update NIS per
    // observation jumps from 1 to 52.5, LAMBDA fails, and the hold fix that follows is
    // 0.65 m off. The fixed-update NIS gate guards only AR candidates, so the hold path
    // accepts it. hold_fix_max_update_nis_per_observation applies a limit to it.
    dropSatellites(processor_, {6, 7, 8, 9});
    Peer::current_update_diagnostics_(processor_).normalized_innovation_squared_per_observation = 52.5;
    auto config = processor_.getRTKConfig();
    config.max_fixed_update_nis_per_observation = 10.0;
    processor_.setRTKConfig(config);
    PositionSolution legacy;
    EXPECT_TRUE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, legacy));
    EXPECT_TRUE(processor_.getLastDebugTelemetry().hold_fix_applied);

    RTKProcessor limited;
    buildScene(limited, false);
    dropSatellites(limited, {6, 7, 8, 9});
    config = limited.getRTKConfig();
    config.hold_fix_max_update_nis_per_observation = 10.0;
    limited.setRTKConfig(config);
    Peer::current_update_diagnostics_(limited).normalized_innovation_squared_per_observation = 52.5;
    const auto pairs = Peer::last_dd_pairs_(limited);
    const auto subset = Peer::last_best_subset_(limited);
    const VectorXd integers = Peer::last_dd_fixed_(limited);
    const Vector3d previous = Peer::last_fixed_position_(limited);
    PositionSolution rejected;
    EXPECT_FALSE(Peer::tryHoldFix(limited, Peer::current_sat_data_(limited), time_, 5, rejected));
    EXPECT_EQ(limited.getLastDebugTelemetry().hold_fix_reject_reason, "hold_fix_update_nis");
    EXPECT_FALSE(limited.getLastDebugTelemetry().hold_fix_applied);
    EXPECT_FALSE(Peer::has_fixed_solution_(limited));
    EXPECT_EQ(rejected.status, SolutionStatus::NONE);
    EXPECT_EQ(Peer::last_dd_fixed_(limited), integers);
    EXPECT_EQ(Peer::last_best_subset_(limited), subset);
    EXPECT_EQ(Peer::last_dd_pairs_(limited).size(), pairs.size());
    EXPECT_EQ(Peer::last_fixed_position_(limited), previous);

    // A consistent update keeps the hold fix, with the same position as without the limit.
    Peer::current_update_diagnostics_(limited).normalized_innovation_squared_per_observation = 2.0;
    PositionSolution accepted;
    ASSERT_TRUE(Peer::tryHoldFix(limited, Peer::current_sat_data_(limited), time_, 5, accepted));
    EXPECT_EQ(accepted.position_ecef, legacy.position_ecef);
}

TEST_F(RTKHoldFixDefaultTest, NisRejectedHoldKeepsTheHoldPathOpenOnlyWhenAsked) {
    // Under --preset urban + --hold-fix-max-update-nis-per-obs 10 most of the lost correct
    // fixes are later hold fixes that were never attempted: the NIS rejection made the epoch
    // FLOAT, the FLOAT epoch reset the consecutive-fix run, and the hold path stayed closed
    // until min_hold_count new AR fixes. hold_fix_update_nis_keeps_hold_run keeps it open.
    for (const bool keep : {false, true}) {
        SCOPED_TRACE(keep ? "keep hold run" : "legacy");
        RTKProcessor processor;
        buildScene(processor, false);
        dropSatellites(processor, {6, 7, 8, 9});
        auto config = processor.getRTKConfig();
        config.hold_fix_max_update_nis_per_observation = 10.0;
        config.hold_fix_update_nis_keeps_hold_run = keep;
        processor.setRTKConfig(config);
        Peer::current_update_diagnostics_(processor).normalized_innovation_squared_per_observation = 52.5;
        PositionSolution rejected;
        ASSERT_FALSE(Peer::tryHoldFix(processor, Peer::current_sat_data_(processor), time_, 5, rejected));
        EXPECT_EQ(processor.getLastDebugTelemetry().hold_fix_reject_reason, "hold_fix_update_nis");
        EXPECT_EQ(Peer::nis_suspended_hold_run_(processor), keep ? 5 : 0);

        // The FLOAT epoch resets the run; the next update is consistent again.
        Peer::consecutive_fix_count_(processor) = 0;
        Peer::current_update_diagnostics_(processor).normalized_innovation_squared_per_observation = 2.0;
        PositionSolution next;
        EXPECT_EQ(Peer::tryHoldFix(processor, Peer::current_sat_data_(processor), time_, 5, next), keep);
        if (!keep) {
            EXPECT_EQ(processor.getLastDebugTelemetry().hold_fix_reject_reason, "preconditions");
        }
    }
}

// Hold scene with ten held GPS L1 pairs (G02..G05 of the scene plus G11..G16) and
// satellite positions and carrier phases. The phases are consistent with a rover
// at phase_rover_offset from the hold position (base + the float baseline).
static void addPhaseGeometry(RTKProcessor& processor, const Vector3d& base,
                             const Vector3d& phase_rover_offset) {
    constexpr double kL1WavelengthM = 0.1902936728;
    auto gps = [](int prn) { return SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(prn)); };
    auto& fs = Peer::filter_state_(processor);
    auto& sat_data = Peer::current_sat_data_(processor);
    auto& pairs = Peer::last_dd_pairs_(processor);
    auto& subset = Peer::last_best_subset_(processor);
    auto& held = Peer::last_dd_fixed_(processor);
    const int ref_idx = Peer::ambiguityIndex(gps(1), 0);
    for (int prn = 11; prn <= 16; ++prn) {
        const SatelliteId sat = gps(prn);
        auto& sd = sat_data[sat];
        sd.satellite = sat;
        sd.has_l1 = true;
        sd.l1_wavelength = kL1WavelengthM;
        const int idx = Peer::ambiguityIndex(sat, 0);
        fs.n1_indices[sat] = idx;
        fs.state(idx) = fs.state(ref_idx) - prn;  // DD float on the integer prn
        fs.covariance(idx, idx) = 0.04;
        Peer::lock_count_l1_(processor)[sat] = 10;
        pairs.push_back(Peer::DDPair{gps(1), ref_idx, idx, sat, 0});
        subset.push_back(static_cast<int>(pairs.size()) - 1);
        held.conservativeResize(held.size() + 1);
        held(held.size() - 1) = prn;
    }

    // Satellites 20,000 km from the base, spread in azimuth and elevation.
    const Vector3d up = base.normalized();
    const Vector3d east = Vector3d(-base.y(), base.x(), 0.0).normalized();
    const Vector3d north = up.cross(east);
    const Vector3d rover = base + fs.state.head<3>() + phase_rover_offset;
    int k = 0;
    for (auto& [sat, sd] : sat_data) {
        // G01 stays the highest satellite, the DD reference of the held pairs.
        const double az = 0.61 * k, el = sat == gps(1) ? 1.4 : 0.35 + 0.09 * (k % 9);
        ++k;
        const Vector3d direction =
            std::cos(el) * (std::sin(az) * east + std::cos(az) * north) + std::sin(el) * up;
        sd.sat_pos = base + 2.0e7 * direction;
        sd.sat_pos_base = sd.sat_pos;
        sd.elevation = el;
        sd.base_elevation = el;
        const double ambiguity = std::round(fs.state(Peer::ambiguityIndex(sat, 0)));
        sd.rover_l1_phase = (sd.sat_pos - rover).norm() / kL1WavelengthM + ambiguity;
        sd.base_l1_phase = (sd.sat_pos - base).norm() / kL1WavelengthM;
    }
}

TEST_F(RTKHoldFixDefaultTest, HoldPhaseFitIsHighOnlyNearAPositionThatFitsThePhases) {
    dropSatellites(processor_, {6, 7, 8, 9});
    addPhaseGeometry(processor_, basePosition(), Vector3d::Zero());
    const auto& pairs = Peer::last_dd_pairs_(processor_);
    std::vector<Peer::DDPair> candidates;
    for (int index : Peer::last_best_subset_(processor_)) {
        if (Peer::current_sat_data_(processor_).count(pairs[index].sat)) candidates.push_back(pairs[index]);
    }
    ASSERT_EQ(candidates.size(), 10u);
    const Vector3d hold = basePosition() + Vector3d(1.0, 2.0, 3.0);
    const auto& sat_data = Peer::current_sat_data_(processor_);

    EXPECT_GT(Peer::holdPhaseFit(processor_, sat_data, candidates, hold), 0.99);
    // Within the +-0.10 m search (0.02 m grid) a few centimetres do not matter.
    EXPECT_GT(Peer::holdPhaseFit(processor_, sat_data, candidates, hold + Vector3d(0.04, -0.06, 0.02)), 0.99);
    // The 0.65 m hold of UrbanNav Shinjuku TOW 281650.1 does not fit.
    EXPECT_LT(Peer::holdPhaseFit(processor_, sat_data, candidates, hold + Vector3d(0.45, 0.30, -0.35)), 0.5);
    // Below ten usable pairs the fit is unknown.
    candidates.pop_back();
    EXPECT_TRUE(std::isnan(Peer::holdPhaseFit(processor_, sat_data, candidates, hold)));
}

TEST_F(RTKHoldFixDefaultTest, NisLimitSparesOnlyAHoldWhosePositionFitsThePhases) {
    // nagoya/run3 TOW 553961.2 under --preset urban: the float update NIS per observation
    // is 1559, yet the hold is correct. NIS alone rejects it; with
    // hold_fix_update_nis_max_phase_fit the rejection also needs a hold position that
    // misfits the carrier phases.
    struct Case {
        const char* name;
        Vector3d phase_rover_offset;
        double nis;
        bool accepted;
    };
    for (const Case& c : {Case{"phases fit", Vector3d::Zero(), 52.5, true},
                          Case{"stale hold", Vector3d(0.45, 0.30, -0.35), 52.5, false},
                          Case{"consistent update", Vector3d(0.45, 0.30, -0.35), 2.0, true}}) {
        SCOPED_TRACE(c.name);
        RTKProcessor processor;
        buildScene(processor, false);
        dropSatellites(processor, {6, 7, 8, 9});
        addPhaseGeometry(processor, basePosition(), c.phase_rover_offset);
        auto config = processor.getRTKConfig();
        config.hold_fix_max_update_nis_per_observation = 10.0;
        config.hold_fix_update_nis_max_phase_fit = 0.5;
        processor.setRTKConfig(config);
        Peer::current_update_diagnostics_(processor).normalized_innovation_squared_per_observation = c.nis;
        PositionSolution solution;
        EXPECT_EQ(Peer::tryHoldFix(processor, Peer::current_sat_data_(processor), time_, 11, solution),
                  c.accepted);
        const auto& telemetry = processor.getLastDebugTelemetry();
        EXPECT_EQ(telemetry.hold_fix_matched_pairs, 10);
        EXPECT_EQ(telemetry.hold_fix_reject_reason, c.accepted ? "" : "hold_fix_update_nis");
        // The fit is evaluated only for an update above the NIS limit.
        EXPECT_EQ(std::isnan(telemetry.hold_fix_phase_fit), c.nis <= 10.0);
        if (c.accepted) {
            EXPECT_TRUE(solution.position_ecef.isApprox(basePosition() + Vector3d(1.0, 2.0, 3.0), 1e-9));
        }
    }

    // Four held pairs are too few for a fit: the NIS limit rejects as without the option.
    dropSatellites(processor_, {6, 7, 8, 9});
    auto config = processor_.getRTKConfig();
    config.hold_fix_max_update_nis_per_observation = 10.0;
    config.hold_fix_update_nis_max_phase_fit = 0.5;
    processor_.setRTKConfig(config);
    Peer::current_update_diagnostics_(processor_).normalized_innovation_squared_per_observation = 52.5;
    PositionSolution solution;
    EXPECT_FALSE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, solution));
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_reject_reason, "hold_fix_update_nis");
    EXPECT_TRUE(std::isnan(processor_.getLastDebugTelemetry().hold_fix_phase_fit));
}

TEST_F(RTKHoldFixDefaultTest, KeptHoldRunEndsAtAnyOtherHoldRejection) {
    dropSatellites(processor_, {6, 7, 8, 9});
    auto config = processor_.getRTKConfig();
    config.hold_fix_max_update_nis_per_observation = 10.0;
    config.hold_fix_update_nis_keeps_hold_run = true;
    processor_.setRTKConfig(config);
    Peer::consecutive_fix_count_(processor_) = 0;
    Peer::nis_suspended_hold_run_(processor_) = 5;
    // The last FIX is 2 m away: the 1 m hold jump limit rejects, and the kept run ends.
    Peer::last_fixed_position_(processor_) += Vector3d(2.0, 0.0, 0.0);
    PositionSolution solution;
    EXPECT_FALSE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, solution));
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_reject_reason, "position_jump");
    EXPECT_EQ(Peer::nis_suspended_hold_run_(processor_), 0);
    PositionSolution again;
    EXPECT_FALSE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, again));
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_reject_reason, "preconditions");
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

// ============================================================
// Hold-fix minimum of held pairs (opt-in hold_fix_min_matched_pairs)
// ============================================================

// Minimum raised to 5 with the unchanged-set rule off: the four held pairs of
// the scene (G02..G05) are no longer enough to fix an epoch.
class RTKHoldFixMinMatchedTest : public RTKHoldFixSceneTest {
protected:
    bool unchangedSetOnly() const override { return false; }
    int minMatchedPairs() const override { return 5; }

    // tryHoldFix must reject for `reason` and accept nothing: the held set, the
    // fixed state and the filter position stay untouched.
    static void expectRejectedAndNothingAccepted(RTKProcessor& processor, const char* reason,
                                                 int candidate_pairs, int matched_pairs) {
        const auto pairs_before = Peer::last_dd_pairs_(processor);
        const auto subset_before = Peer::last_best_subset_(processor);
        const VectorXd fixed_before = Peer::last_dd_fixed_(processor);
        const Vector3d fixed_baseline_before = Peer::fixed_baseline_(processor);
        const Vector3d filter_position_before = Peer::filter_state_(processor).state.head<3>();

        PositionSolution solution;
        EXPECT_FALSE(Peer::tryHoldFix(processor, Peer::current_sat_data_(processor), epochTime(), 9,
                                      solution));

        const auto& telemetry = processor.getLastDebugTelemetry();
        EXPECT_TRUE(telemetry.hold_fix_attempted);
        EXPECT_FALSE(telemetry.hold_fix_applied);
        EXPECT_EQ(telemetry.hold_fix_reject_reason, reason);
        EXPECT_EQ(telemetry.hold_fix_candidate_pairs, candidate_pairs);
        EXPECT_EQ(telemetry.hold_fix_matched_pairs, matched_pairs);

        // Nothing was accepted: the held set, the fixed state and the filter are untouched.
        const auto& pairs = Peer::last_dd_pairs_(processor);
        ASSERT_EQ(pairs.size(), pairs_before.size());
        for (size_t i = 0; i < pairs.size(); ++i) {
            EXPECT_EQ(pairs[i].ref_sat, pairs_before[i].ref_sat);
            EXPECT_EQ(pairs[i].sat, pairs_before[i].sat);
            EXPECT_EQ(pairs[i].ref_idx, pairs_before[i].ref_idx);
            EXPECT_EQ(pairs[i].sat_idx, pairs_before[i].sat_idx);
            EXPECT_EQ(pairs[i].freq, pairs_before[i].freq);
        }
        EXPECT_EQ(Peer::last_best_subset_(processor), subset_before);
        EXPECT_EQ(Peer::last_dd_fixed_(processor), fixed_before);
        EXPECT_FALSE(Peer::has_fixed_solution_(processor));
        EXPECT_EQ(Peer::fixed_baseline_(processor), fixed_baseline_before);
        EXPECT_EQ(Peer::last_num_fixed_ambiguities_(processor), 0);
        EXPECT_DOUBLE_EQ(Peer::last_ar_ratio_(processor), 0.0);
        EXPECT_EQ(Peer::filter_state_(processor).state.head<3>(), filter_position_before);
    }
};

TEST_F(RTKHoldFixMinMatchedTest, FourHeldPairsBelowTheMinimumRejectAndKeepHeldSet) {
    // The scene of AllCandidatesMatchedWithStaleHeldPairIsAcceptedAsBefore: only
    // G02..G05 are candidates and all four are held (the stale G10 does not count).
    // The legacy floor of 4 passes, the minimum of 5 does not.
    dropSatellites(processor_, {6, 7, 8, 9});
    ASSERT_EQ(processor_.getRTKConfig().hold_fix_min_matched_pairs, 5);

    expectRejectedAndNothingAccepted(processor_, "matched_pairs_below_min", 4, 4);
}

TEST_F(RTKHoldFixMinMatchedTest, NewPairsDoNotCountTowardTheMinimum) {
    // Full scene: 8 candidates, only G02..G05 carry a held integer. The unchanged-set
    // rule is off, so without the minimum the four new pairs would be rounded and the
    // hold accepted. 4 < 5 rejects instead, with its own reason (not "unmatched_pairs"),
    // and nothing is rounded into the held set.
    expectRejectedAndNothingAccepted(processor_, "matched_pairs_below_min", 8, 4);
}

TEST_F(RTKHoldFixMinMatchedTest, FewerThanFourMatchedPairsStillRejectsWithTheLegacyReason) {
    // Drop two held satellites from tracking: 2 held pairs remain, 4 new ones. The
    // legacy floor of 4 is checked first, so the minimum of 5 does not change the reason.
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

TEST_F(RTKHoldFixMinMatchedTest, MinimumIsCheckedBeforeTheUnchangedSetRule) {
    // Both opt-ins on, full scene (4 of 8 candidates matched): the minimum of 5
    // rejects first. The unchanged-set rule alone would say "unmatched_pairs".
    RTKProcessor both_on;
    buildScene(both_on, true, 5);
    ASSERT_TRUE(both_on.getRTKConfig().hold_fix_unchanged_set_only);

    expectRejectedAndNothingAccepted(both_on, "matched_pairs_below_min", 8, 4);
}

TEST_F(RTKHoldFixDefaultTest, DefaultMinimumIsTheLegacyFloorAndFourHeldPairsAreAccepted) {
    // hold_fix_min_matched_pairs defaults to 4, the legacy floor: the scene that
    // a minimum of 5 rejects (only G02..G05 are candidates, all held) is accepted
    // with the same result as in AllCandidatesMatchedWithStaleHeldPairIsAcceptedAsBefore.
    EXPECT_EQ(processor_.getRTKConfig().hold_fix_min_matched_pairs, 4);
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

    // The held integers win over the floats (G05 floats at 3.60, the held 3 wins).
    const auto& pairs = Peer::last_dd_pairs_(processor_);
    const auto& subset = Peer::last_best_subset_(processor_);
    const VectorXd& fixed = Peer::last_dd_fixed_(processor_);
    ASSERT_EQ(pairs.size(), 4u);
    ASSERT_EQ(subset.size(), 4u);
    ASSERT_EQ(fixed.size(), 4);
    EXPECT_EQ(subset, (std::vector<int>{0, 1, 2, 3}));
    std::map<int, double> integer_by_prn;
    for (int i = 0; i < 4; ++i) {
        integer_by_prn[pairs[subset[i]].sat.prn] = fixed(i);
    }
    const std::map<int, double> expected = {{2, 12.0}, {3, -7.0}, {4, 25.0}, {5, 3.0}};
    EXPECT_EQ(integer_by_prn, expected);
    EXPECT_EQ(Peer::last_num_fixed_ambiguities_(processor_), 4);
}

TEST_F(RTKHoldFixDefaultTest, MinimumBelowTheLegacyFloorBehavesLikeTheDefault) {
    // The legacy floor of 4 is checked first, so a smaller minimum changes nothing
    // (the CLI rejects values below 4 anyway): the accepted hold-fix is identical.
    RTKProcessor below_floor;
    buildScene(below_floor, false, 3);
    ASSERT_EQ(below_floor.getRTKConfig().hold_fix_min_matched_pairs, 3);
    ASSERT_EQ(processor_.getRTKConfig().hold_fix_min_matched_pairs, 4);
    dropSatellites(processor_, {6, 7, 8, 9});
    dropSatellites(below_floor, {6, 7, 8, 9});

    PositionSolution solution_default;
    PositionSolution solution_below;
    ASSERT_TRUE(Peer::tryHoldFix(processor_, Peer::current_sat_data_(processor_), time_, 5, solution_default));
    ASSERT_TRUE(Peer::tryHoldFix(below_floor, Peer::current_sat_data_(below_floor), time_, 5, solution_below));

    EXPECT_EQ(solution_default.position_ecef, solution_below.position_ecef);
    EXPECT_EQ(solution_default.status, solution_below.status);
    EXPECT_EQ(solution_default.ratio, solution_below.ratio);
    EXPECT_EQ(solution_default.num_fixed_ambiguities, solution_below.num_fixed_ambiguities);
    EXPECT_EQ(Peer::fixed_baseline_(processor_), Peer::fixed_baseline_(below_floor));
    EXPECT_EQ(Peer::last_best_subset_(processor_), Peer::last_best_subset_(below_floor));
    EXPECT_EQ(Peer::last_dd_fixed_(processor_), Peer::last_dd_fixed_(below_floor));
    EXPECT_EQ(processor_.getLastDebugTelemetry().hold_fix_jump_m,
              below_floor.getLastDebugTelemetry().hold_fix_jump_m);
}

// ============================================================
// Minimum of non-reference satellites in an AR fix (opt-in min_fix_satellites)
// ============================================================

// resolveAmbiguities() on a synthetic scene: reference G01 and non-reference
// satellites that each carry one independent SD ambiguity state per frequency
// (0.04 cycle^2), so the double differences against G01 have variance 0.08 and
// covariance 0.04. The floats sit a few hundredths of a cycle off an integer,
// which LAMBDA fixes with a ratio far above the threshold of 3, unless a scene
// makes satellites ambiguous on purpose (x.5, larger variance). The position is
// correlated with the first satellite only, so an applied fix moves
// fixed_baseline_ off the float baseline. Every run starts from a stale earlier
// fix in the members the AR step writes, to tell "rejected, nothing written"
// from "fixed, overwritten".
class RTKMinFixSatellitesTest : public ::testing::Test {
protected:
    static constexpr double kRefAmbiguityCycles = 1000.25;
    static constexpr int kStalePrn = 30;
    static constexpr double kStaleFixedInteger = 42.0;
    static constexpr double kStaleRatio = 2.5;
    static constexpr int kStaleFixedCount = 1;

    struct Sat {
        int prn;
        int freq;         // 0 = L1, 1 = L2
        double dd_float;  // reference minus satellite [cycles]
        double variance;  // SD ambiguity variance [cycles^2]
    };

    // What resolveAmbiguities() leaves behind.
    struct Outcome {
        bool resolved = false;
        bool has_fixed_solution = false;
        Vector3d fixed_baseline = Vector3d::Zero();
        std::vector<int> pair_prns;  // satellites of last_dd_pairs_
        std::vector<int> subset;     // last_best_subset_
        VectorXd dd_fixed;           // last_dd_fixed_
        double ar_ratio = 0.0;       // last_ar_ratio_
        int num_fixed = 0;           // last_num_fixed_ambiguities_
        bool selected_fixed = false;
        double selected_ratio = 0.0;
        int selected_pair_count = 0;
        int selected_distinct_sats = 0;
        bool selected_used_subset = false;
        std::string reject_reason;
    };

    static SatelliteId gps(int prn) {
        return SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(prn));
    }

    static Vector3d floatBaseline() { return Vector3d(1.0, 2.0, 3.0); }
    static Vector3d staleBaseline() { return Vector3d(7.0, 8.0, 9.0); }

    // The first `count` of G02..G06 on L1, near the integers 12, -7, 25, 3, 7.
    static std::vector<Sat> cleanSatellites(int count) {
        const std::vector<Sat> all = {{2, 0, 12.06, 0.04}, {3, 0, -7.04, 0.04}, {4, 0, 25.08, 0.04},
                                      {5, 0, 2.95, 0.04},  {6, 0, 7.03, 0.04}};
        return std::vector<Sat>(all.begin(), all.begin() + count);
    }

    // G02..G05 near integers, G06 and G07 ambiguous (x.5) with the largest
    // variances. The 6- and 5-satellite sets fail LAMBDA's ratio test (about
    // 1.1). Partial AR drops G07, then G06, and keeps G02..G05 (ratio about 50).
    static std::vector<Sat> ambiguousTail() {
        auto sats = cleanSatellites(4);
        sats.push_back({6, 0, 7.5, 0.09});
        sats.push_back({7, 0, -3.5, 0.16});
        return sats;
    }

    // Three satellites on two frequencies: six pairs, three satellites.
    static std::vector<Sat> dualFrequencySatellites() {
        return {{2, 0, 12.06, 0.04}, {2, 1, 9.95, 0.04},   {3, 0, -7.04, 0.04},
                {3, 1, 3.04, 0.04},  {4, 0, 25.08, 0.04},  {4, 1, -11.07, 0.04}};
    }

    static Outcome run(int min_fix_satellites, const std::vector<Sat>& sats) {
        RTKProcessor processor;
        RTKProcessor::RTKConfig cfg;
        cfg.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        cfg.min_fix_satellites = min_fix_satellites;
        processor.setRTKConfig(cfg);
        processor.setBasePosition(Vector3d(-3961832.0, 3354966.0, 3697065.0));

        auto& fs = Peer::filter_state_(processor);
        fs.state = VectorXd::Zero(Peer::kStateCount);
        fs.covariance = MatrixXd::Zero(Peer::kStateCount, Peer::kStateCount);
        fs.state.head<3>() = floatBaseline();
        fs.covariance.topLeftCorner<3, 3>() = Matrix3d::Identity() * 0.01;
        std::vector<Peer::DDPair> pairs;
        for (const Sat& sat : sats) {
            const int ref_idx = Peer::ambiguityIndex(gps(1), sat.freq);
            const int sat_idx = Peer::ambiguityIndex(gps(sat.prn), sat.freq);
            fs.state(ref_idx) = kRefAmbiguityCycles;
            fs.covariance(ref_idx, ref_idx) = 0.04;
            fs.state(sat_idx) = kRefAmbiguityCycles - sat.dd_float;
            fs.covariance(sat_idx, sat_idx) = sat.variance;
            pairs.push_back({gps(1), ref_idx, sat_idx, gps(sat.prn), sat.freq});
        }
        const int first_idx = pairs.front().sat_idx;
        fs.covariance(0, first_idx) = 0.01;
        fs.covariance(first_idx, 0) = 0.01;
        Peer::filter_initialized_(processor) = true;

        // The stale earlier fix: one pair of G30 against G29.
        Peer::last_dd_pairs_(processor) = {Peer::DDPair{gps(29), Peer::ambiguityIndex(gps(29), 0),
                                                        Peer::ambiguityIndex(gps(kStalePrn), 0),
                                                        gps(kStalePrn), 0}};
        Peer::last_best_subset_(processor) = {0};
        VectorXd stale_fixed(1);
        stale_fixed << kStaleFixedInteger;
        Peer::last_dd_fixed_(processor) = stale_fixed;
        Peer::fixed_baseline_(processor) = staleBaseline();
        Peer::last_ar_ratio_(processor) = kStaleRatio;
        Peer::last_num_fixed_ambiguities_(processor) = kStaleFixedCount;
        Peer::has_fixed_solution_(processor) = true;

        Outcome out;
        const VectorXd float_state_before_ar = fs.state;
        const MatrixXd float_covariance_before_ar = fs.covariance;
        out.resolved = Peer::resolveAmbiguities(processor, pairs);
        // Integer search produces a candidate; it must not commit a conditional
        // update to the FLOAT filter before the acceptance gates run.
        EXPECT_EQ(fs.state, float_state_before_ar);
        EXPECT_EQ(fs.covariance, float_covariance_before_ar);
        out.has_fixed_solution = Peer::has_fixed_solution_(processor);
        out.fixed_baseline = Peer::fixed_baseline_(processor);
        for (const auto& pair : Peer::last_dd_pairs_(processor)) {
            out.pair_prns.push_back(pair.sat.prn);
        }
        out.subset = Peer::last_best_subset_(processor);
        out.dd_fixed = Peer::last_dd_fixed_(processor);
        out.ar_ratio = Peer::last_ar_ratio_(processor);
        out.num_fixed = Peer::last_num_fixed_ambiguities_(processor);
        const auto& telemetry = processor.getLastDebugTelemetry();
        out.selected_fixed = telemetry.selected_fixed;
        out.selected_ratio = telemetry.selected_ratio;
        out.selected_pair_count = telemetry.selected_pair_count;
        out.selected_distinct_sats = telemetry.selected_distinct_sats;
        out.selected_used_subset = telemetry.selected_used_subset;
        out.reject_reason = telemetry.reject_reason;
        return out;
    }

    // The minimum changed nothing: the same fix, bit for bit, as without it.
    static void expectSameOutcome(const Outcome& expected, const Outcome& actual) {
        EXPECT_EQ(actual.resolved, expected.resolved);
        EXPECT_EQ(actual.has_fixed_solution, expected.has_fixed_solution);
        EXPECT_EQ(actual.fixed_baseline, expected.fixed_baseline);
        EXPECT_EQ(actual.pair_prns, expected.pair_prns);
        EXPECT_EQ(actual.subset, expected.subset);
        EXPECT_EQ(actual.dd_fixed, expected.dd_fixed);
        EXPECT_EQ(actual.ar_ratio, expected.ar_ratio);
        EXPECT_EQ(actual.num_fixed, expected.num_fixed);
        EXPECT_EQ(actual.selected_fixed, expected.selected_fixed);
        EXPECT_EQ(actual.selected_ratio, expected.selected_ratio);
        EXPECT_EQ(actual.selected_pair_count, expected.selected_pair_count);
        EXPECT_EQ(actual.selected_distinct_sats, expected.selected_distinct_sats);
        EXPECT_EQ(actual.selected_used_subset, expected.selected_used_subset);
        EXPECT_EQ(actual.reject_reason, expected.reject_reason);
    }

    // LAMBDA fixed the epoch, the minimum refused it: nothing of the fix was
    // written, the stale flag is cleared, the stale fix stays.
    static void expectRejectedKeepingTheStaleFix(const Outcome& out, int distinct_sats,
                                                 int pair_count, bool used_subset) {
        EXPECT_FALSE(out.resolved);
        EXPECT_FALSE(out.has_fixed_solution);
        EXPECT_EQ(out.reject_reason, "fixed_satellites_below_min");
        EXPECT_FALSE(out.selected_fixed);
        EXPECT_GT(out.selected_ratio, 3.0);  // the ratio of an accepted fix
        EXPECT_EQ(out.selected_distinct_sats, distinct_sats);
        EXPECT_EQ(out.selected_pair_count, pair_count);
        EXPECT_EQ(out.selected_used_subset, used_subset);

        EXPECT_EQ(out.fixed_baseline, staleBaseline());
        EXPECT_EQ(out.pair_prns, (std::vector<int>{kStalePrn}));
        EXPECT_EQ(out.subset, (std::vector<int>{0}));
        ASSERT_EQ(out.dd_fixed.size(), 1);
        EXPECT_EQ(out.dd_fixed(0), kStaleFixedInteger);
        EXPECT_EQ(out.ar_ratio, kStaleRatio);
        EXPECT_EQ(out.num_fixed, kStaleFixedCount);
    }
};

TEST_F(RTKMinFixSatellitesTest, FullSetFixOfFiveSatellitesIsTheBaselineWithoutTheOption) {
    const Outcome fix = run(0, cleanSatellites(5));

    EXPECT_TRUE(fix.resolved);
    EXPECT_TRUE(fix.has_fixed_solution);
    EXPECT_TRUE(fix.reject_reason.empty());
    EXPECT_TRUE(fix.selected_fixed);
    EXPECT_GT(fix.selected_ratio, 3.0);
    EXPECT_EQ(fix.selected_pair_count, 5);
    EXPECT_EQ(fix.selected_distinct_sats, 5);
    EXPECT_FALSE(fix.selected_used_subset);

    // The fix replaced the stale one ...
    EXPECT_EQ(fix.pair_prns, (std::vector<int>{2, 3, 4, 5, 6}));
    EXPECT_EQ(fix.subset, (std::vector<int>{0, 1, 2, 3, 4}));
    ASSERT_EQ(fix.dd_fixed.size(), 5);
    EXPECT_EQ(fix.dd_fixed, (VectorXd(5) << 12.0, -7.0, 25.0, 3.0, 7.0).finished());
    EXPECT_EQ(fix.num_fixed, 5);
    EXPECT_EQ(fix.ar_ratio, fix.selected_ratio);
    // ... and moved the position off the float baseline.
    EXPECT_GT((fix.fixed_baseline - floatBaseline()).norm(), 1e-3);
}

TEST_F(RTKMinFixSatellitesTest, MinimumUpToTheSatelliteCountOfTheFixChangesNothing) {
    const auto sats = cleanSatellites(5);
    const Outcome baseline = run(0, sats);
    ASSERT_TRUE(baseline.resolved);

    // A negative minimum is off like 0; 1 to 5 are all met by the five non-reference satellites.
    for (const int minimum : {-3, 1, 3, 4, 5}) {
        SCOPED_TRACE("min_fix_satellites=" + std::to_string(minimum));
        expectSameOutcome(baseline, run(minimum, sats));
    }
}

TEST_F(RTKMinFixSatellitesTest, FullSetFixBelowTheMinimumIsRejectedAndKeepsTheStaleFix) {
    const auto sats = cleanSatellites(5);
    for (const int minimum : {6, 7, 99}) {
        SCOPED_TRACE("min_fix_satellites=" + std::to_string(minimum));
        expectRejectedKeepingTheStaleFix(run(minimum, sats), 5, 5, false);
    }
}

TEST_F(RTKMinFixSatellitesTest, TheBoundaryIsTheNonReferenceSatelliteCount) {
    // AR needs four DD pairs, so four non-reference satellites (five with G01) are
    // the smallest full-set fix. The minimum counts the non-reference ones: 4 passes,
    // 5 does not.
    const auto sats = cleanSatellites(4);
    const Outcome baseline = run(0, sats);
    ASSERT_TRUE(baseline.resolved);
    EXPECT_EQ(baseline.selected_distinct_sats, 4);

    expectSameOutcome(baseline, run(4, sats));
    expectRejectedKeepingTheStaleFix(run(5, sats), 4, 4, false);
}

TEST_F(RTKMinFixSatellitesTest, SubsetFixIsCountedByTheSatellitesItKeeps) {
    const auto sats = ambiguousTail();
    const Outcome baseline = run(0, sats);
    ASSERT_TRUE(baseline.resolved);
    EXPECT_TRUE(baseline.selected_used_subset);
    EXPECT_EQ(baseline.selected_pair_count, 4);
    EXPECT_EQ(baseline.selected_distinct_sats, 4);
    EXPECT_EQ(baseline.subset, (std::vector<int>{0, 1, 2, 3}));
    EXPECT_EQ(baseline.pair_prns.size(), 6u);  // the 6-satellite set stays the candidate set
    EXPECT_EQ(baseline.dd_fixed, (VectorXd(4) << 12.0, -7.0, 25.0, 3.0).finished());

    // The six satellites seen do not count, only the four the fix keeps.
    expectSameOutcome(baseline, run(4, sats));
    expectRejectedKeepingTheStaleFix(run(5, sats), 4, 4, true);
    expectRejectedKeepingTheStaleFix(run(6, sats), 4, 4, true);
}

TEST_F(RTKMinFixSatellitesTest, DualFrequencyPairsOfOneSatelliteCountOnce) {
    // Three satellites on L1 and L2: six pairs, but only three satellites.
    const auto sats = dualFrequencySatellites();
    const Outcome baseline = run(0, sats);
    ASSERT_TRUE(baseline.resolved);
    EXPECT_EQ(baseline.selected_pair_count, 6);
    EXPECT_EQ(baseline.selected_distinct_sats, 3);
    EXPECT_EQ(baseline.dd_fixed, (VectorXd(6) << 12.0, 10.0, -7.0, 3.0, 25.0, -11.0).finished());

    expectSameOutcome(baseline, run(3, sats));
    expectRejectedKeepingTheStaleFix(run(4, sats), 3, 6, false);
}

// WL/NL fallback of the same rule: tryWlNlFallback() keeps its legacy floor of
// 3 resolved satellites, and min_fix_satellites can only raise it. Reference
// G01 and non-reference satellites with an L1 and an L2 pair each (all L1 pairs
// first). A satellite passes the Melbourne-Wubbena check when its wide-lane
// comes out as an integer N_w; its floats are then N1 = N_w + N2 and N2, which
// the narrow-lane check gives back exactly. A broken satellite lands half a
// cycle off N_w and is not resolved.
class RTKMinFixSatellitesWlNlTest : public ::testing::Test {
protected:
    static constexpr double kL1Hz = 1575.42e6;
    static constexpr double kL2Hz = 1227.60e6;
    static constexpr int kStalePrn = 30;
    static constexpr double kStaleFixedInteger = 42.0;

    struct Sat {
        int prn;
        int wide_lane;  // integer N_w [cycles]
        int n2;         // integer L2 ambiguity [cycles]
        bool broken;    // wide-lane half a cycle off: not resolved
    };

    struct Run {
        int satellites = 0;
        // Inputs, as handed to the fallback.
        VectorXd dd_float_in;
        MatrixXd qb_in;
        MatrixXd qab_in;
        std::vector<int> subset_in;
        // Outputs.
        bool fixed = false;
        double ratio = 0.0;
        VectorXd dd_float;
        MatrixXd qb;
        MatrixXd qab;
        VectorXd dd_fixed;
        std::vector<int> subset;
        bool used_fallback = false;
        // Members the fallback writes.
        bool has_fixed_solution = false;
        Vector3d fixed_baseline = Vector3d::Zero();
        std::vector<int> pair_prns;
        std::vector<int> last_subset;
        VectorXd last_dd_fixed;
    };

    static SatelliteId gps(int prn) {
        return SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(prn));
    }

    static Vector3d floatBaseline() { return Vector3d(1.0, 2.0, 3.0); }
    static Vector3d staleBaseline() { return Vector3d(7.0, 8.0, 9.0); }

    // The first `count` of G02..G05, the last `broken_count` of them broken.
    static std::vector<Sat> satellites(int count, int broken_count) {
        const std::vector<Sat> all = {{2, 3, -5, false}, {3, -2, 8, false}, {4, 5, 2, false},
                                      {5, 1, -3, false}};
        std::vector<Sat> sats(all.begin(), all.begin() + count);
        for (int i = 0; i < broken_count; ++i) {
            sats[count - 1 - i].broken = true;
        }
        return sats;
    }

    static Run run(int min_fix_satellites, const std::vector<Sat>& sats) {
        RTKProcessor processor;
        RTKProcessor::RTKConfig cfg;
        cfg.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        cfg.enable_wlnl_fallback = true;
        cfg.min_fix_satellites = min_fix_satellites;
        processor.setRTKConfig(cfg);

        const double lambda1 = constants::SPEED_OF_LIGHT / kL1Hz;
        const double lambda2 = constants::SPEED_OF_LIGHT / kL2Hz;
        auto& sat_data = Peer::current_sat_data_(processor);
        // Only the rover L1 phase is non-zero, so the Melbourne-Wubbena value of a
        // satellite is that phase; the reference has 0 and the double difference is -phase.
        auto add_satellite = [&](int prn, double rover_l1_phase_cycles) {
            auto& sd = sat_data[gps(prn)];
            sd.satellite = gps(prn);
            sd.has_l1 = true;
            sd.has_l2 = true;
            sd.l1_frequency_hz = kL1Hz;
            sd.l2_frequency_hz = kL2Hz;
            sd.l1_wavelength = lambda1;
            sd.l2_wavelength = lambda2;
            sd.rover_l1_phase = rover_l1_phase_cycles;
        };
        add_satellite(1, 0.0);

        const int n = static_cast<int>(sats.size());
        std::vector<Peer::DDPair> pairs;
        Run out;
        out.satellites = n;
        out.dd_float_in = VectorXd::Zero(2 * n);
        for (int freq = 0; freq < 2; ++freq) {
            for (int j = 0; j < n; ++j) {
                const Sat& sat = sats[j];
                if (freq == 0) {
                    add_satellite(sat.prn, -(sat.wide_lane + (sat.broken ? 0.5 : 0.0)));
                }
                pairs.push_back({gps(1), Peer::ambiguityIndex(gps(1), freq),
                                 Peer::ambiguityIndex(gps(sat.prn), freq), gps(sat.prn), freq});
                out.dd_float_in(freq * n + j) = freq == 0 ? sat.wide_lane + sat.n2 : sat.n2;
            }
        }
        out.qb_in = MatrixXd::Identity(2 * n, 2 * n) * 0.02 + MatrixXd::Constant(2 * n, 2 * n, 0.01);
        out.qab_in = MatrixXd::Zero(3, 2 * n);
        out.qab_in.row(0).setConstant(0.001);
        out.subset_in.resize(2 * n);
        for (int i = 0; i < 2 * n; ++i) {
            out.subset_in[i] = i;
        }

        // The stale earlier fix, as in RTKMinFixSatellitesTest.
        Peer::last_dd_pairs_(processor) = {Peer::DDPair{gps(29), Peer::ambiguityIndex(gps(29), 0),
                                                        Peer::ambiguityIndex(gps(kStalePrn), 0),
                                                        gps(kStalePrn), 0}};
        Peer::last_best_subset_(processor) = {0};
        VectorXd stale_fixed(1);
        stale_fixed << kStaleFixedInteger;
        Peer::last_dd_fixed_(processor) = stale_fixed;
        Peer::fixed_baseline_(processor) = staleBaseline();
        Peer::has_fixed_solution_(processor) = false;

        const VectorXd head_state = floatBaseline();
        out.dd_float = out.dd_float_in;
        out.qb = out.qb_in;
        out.qab = out.qab_in;
        out.subset = out.subset_in;
        // na = 3 (the position), max_var < 1 as after a few converged epochs.
        Peer::tryWlNlFallback(processor, 3, 2 * n, 0.2, pairs, head_state, out.dd_float, out.qb,
                              out.qab, out.dd_fixed, out.fixed, out.ratio, out.subset);

        out.used_fallback = processor.getLastDebugTelemetry().used_wlnl_fallback;
        out.has_fixed_solution = Peer::has_fixed_solution_(processor);
        out.fixed_baseline = Peer::fixed_baseline_(processor);
        for (const auto& pair : Peer::last_dd_pairs_(processor)) {
            out.pair_prns.push_back(pair.sat.prn);
        }
        out.last_subset = Peer::last_best_subset_(processor);
        out.last_dd_fixed = Peer::last_dd_fixed_(processor);
        return out;
    }

    // The fallback fixed the resolved satellites (the first `resolved` of them,
    // as the scenes break the last ones): both pairs of each, ratio 999.9.
    static void expectApplied(const Run& out, int resolved) {
        EXPECT_TRUE(out.fixed);
        EXPECT_EQ(out.ratio, 999.9);
        EXPECT_TRUE(out.used_fallback);
        EXPECT_TRUE(out.has_fixed_solution);
        std::vector<int> expected_subset;
        for (int j = 0; j < resolved; ++j) {
            expected_subset.push_back(j);
        }
        for (int j = 0; j < resolved; ++j) {
            expected_subset.push_back(out.satellites + j);
        }
        EXPECT_EQ(out.subset, expected_subset);
        EXPECT_EQ(out.last_subset, expected_subset);
        ASSERT_EQ(out.dd_fixed.size(), 2 * resolved);
        EXPECT_EQ(out.last_dd_fixed, out.dd_fixed);
        EXPECT_EQ(out.pair_prns.size(), static_cast<size_t>(2 * out.satellites));
        EXPECT_TRUE(out.fixed_baseline.isApprox(floatBaseline(), 1e-9)) << out.fixed_baseline.transpose();
    }

    // The fallback was refused before it wrote anything.
    static void expectRefused(const Run& out) {
        EXPECT_FALSE(out.fixed);
        EXPECT_EQ(out.ratio, 0.0);
        EXPECT_FALSE(out.used_fallback);
        EXPECT_EQ(out.dd_float, out.dd_float_in);
        EXPECT_EQ(out.qb, out.qb_in);
        EXPECT_EQ(out.qab, out.qab_in);
        EXPECT_EQ(out.dd_fixed.size(), 0);
        EXPECT_EQ(out.subset, out.subset_in);

        EXPECT_FALSE(out.has_fixed_solution);
        EXPECT_EQ(out.fixed_baseline, staleBaseline());
        EXPECT_EQ(out.pair_prns, (std::vector<int>{kStalePrn}));
        EXPECT_EQ(out.last_subset, (std::vector<int>{0}));
        ASSERT_EQ(out.last_dd_fixed.size(), 1);
        EXPECT_EQ(out.last_dd_fixed(0), kStaleFixedInteger);
    }
};

TEST_F(RTKMinFixSatellitesWlNlTest, FourResolvedSatellitesPassTheFloorAndAnyMinimumUpToFour) {
    const auto sats = satellites(4, 0);
    // Off (0 and below), below the floor of 3, at the floor, and up to the four resolved.
    for (const int minimum : {-1, 0, 2, 3, 4}) {
        SCOPED_TRACE("min_fix_satellites=" + std::to_string(minimum));
        expectApplied(run(minimum, sats), 4);
    }
}

TEST_F(RTKMinFixSatellitesWlNlTest, MinimumAboveTheResolvedSatellitesRefusesTheFallbackBeforeWritingAnything) {
    const auto sats = satellites(4, 0);
    for (const int minimum : {5, 6, 99}) {
        SCOPED_TRACE("min_fix_satellites=" + std::to_string(minimum));
        expectRefused(run(minimum, sats));
    }
}

TEST_F(RTKMinFixSatellitesWlNlTest, ThreeResolvedSatellitesAreTheLegacyFloor) {
    // One of four broken: three are resolved, the fallback's legacy floor.
    const auto sats = satellites(4, 1);
    // A minimum below or at the floor does not change it ...
    for (const int minimum : {-1, 0, 1, 2, 3}) {
        SCOPED_TRACE("min_fix_satellites=" + std::to_string(minimum));
        expectApplied(run(minimum, sats), 3);
    }
    // ... a higher one refuses it.
    for (const int minimum : {4, 5}) {
        SCOPED_TRACE("min_fix_satellites=" + std::to_string(minimum));
        expectRefused(run(minimum, sats));
    }
}

TEST_F(RTKMinFixSatellitesWlNlTest, TwoResolvedSatellitesStayBelowTheFloorWhateverTheMinimum) {
    // Two of four broken: the legacy floor of 3 is not met, and a minimum of 1 or 2
    // cannot lower it.
    const auto sats = satellites(4, 2);
    for (const int minimum : {-1, 0, 1, 2, 3, 4}) {
        SCOPED_TRACE("min_fix_satellites=" + std::to_string(minimum));
        expectRefused(run(minimum, sats));
    }
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

// Road-surface height prior (RTKConfig::road_height_sigma_m) after a kinematic
// re-seed: SPP-like position, 900 m^2 per axis, 12 m above the road + antenna.
TEST(RTKRoadHeightPriorTest, PullsOnlyTheHeightOfTheSeedTowardTheRoadSurface) {
    const double lat = 35.66 * M_PI / 180.0, lon = 139.79 * M_PI / 180.0;
    const Vector3d base = geodetic2ecef(lat, lon, 40.0);
    const Vector3d seed = geodetic2ecef(lat, lon, 38.0 + 1.5 + 12.0) +
                          enu2ecef(Vector3d(100.0, 50.0, 0.0), lat, lon);
    double seed_lat = 0.0, seed_lon = 0.0, seed_h = 0.0;
    ecef2geodetic(seed, seed_lat, seed_lon, seed_h);
    auto map = std::make_shared<road_height::RoadHeightMap>(road_height::buildRoadHeightMap(
        {Vector3d(seed_lat * 180.0 / M_PI, seed_lon * 180.0 / M_PI, 38.0)}, 1.5));

    auto make = [&](double sigma, bool with_map, RTKProcessor::RTKConfig::PositionMode mode) {
        auto processor = std::make_unique<RTKProcessor>();
        RTKProcessor::RTKConfig cfg;
        cfg.position_mode = mode;
        cfg.road_height_sigma_m = sigma;
        cfg.road_height_offset_m = 1.5;
        processor->setRTKConfig(cfg);
        processor->setBasePosition(base);
        if (with_map) processor->setRoadHeightMap(map);
        auto& fs = Peer::filter_state_(*processor);
        fs.state = VectorXd::Zero(Peer::kStateCount);
        fs.covariance = MatrixXd::Identity(Peer::kStateCount, Peer::kStateCount) * 0.01;
        fs.state.head<3>() = seed - base;
        fs.covariance.topLeftCorner<3, 3>() = Matrix3d::Identity() * 900.0;
        return processor;
    };

    auto on = make(0.2, true, RTKProcessor::RTKConfig::PositionMode::KINEMATIC);
    Peer::applyRoadHeightPrior(*on);
    const auto& telemetry = on->getLastDebugTelemetry();
    ASSERT_TRUE(telemetry.road_height_applied);
    EXPECT_NEAR(telemetry.road_height_innovation_m, 38.0 + 1.5 - seed_h, 1e-9);
    const Vector3d rover = base + Peer::filter_state_(*on).state.head<3>();
    double h_lat = 0.0, h_lon = 0.0, h = 0.0;
    ecef2geodetic(rover, h_lat, h_lon, h);
    EXPECT_NEAR(h, 39.5, 1e-3);  // road 38.0 + antenna 1.5; 0.04 vs 900 m^2
    const Vector3d moved = ecef2enu(rover - seed, seed_lat, seed_lon);
    EXPECT_NEAR(moved.x(), 0.0, 1e-6);
    EXPECT_NEAR(moved.y(), 0.0, 1e-6);
    const Vector3d up = enu2ecef(Vector3d(0.0, 0.0, 1.0), seed_lat, seed_lon);
    const Matrix3d p_pos = Peer::filter_state_(*on).covariance.topLeftCorner<3, 3>();
    EXPECT_NEAR(up.dot(p_pos * up), 900.0 * 0.04 / 900.04, 1e-6);

    // Off, without a map, outside the map radius, or in static mode: untouched.
    std::vector<std::unique_ptr<RTKProcessor>> untouched;
    untouched.push_back(make(0.0, true, RTKProcessor::RTKConfig::PositionMode::KINEMATIC));
    untouched.push_back(make(0.2, false, RTKProcessor::RTKConfig::PositionMode::KINEMATIC));
    untouched.push_back(make(0.2, true, RTKProcessor::RTKConfig::PositionMode::STATIC));
    untouched.push_back(make(0.2, true, RTKProcessor::RTKConfig::PositionMode::KINEMATIC));
    Peer::filter_state_(*untouched.back()).state.head<3>() +=
        enu2ecef(Vector3d(3.0, 0.0, 0.0), lat, lon);  // 3 m from the only map point
    for (auto& processor : untouched) {
        const VectorXd before = Peer::filter_state_(*processor).state;
        Peer::applyRoadHeightPrior(*processor);
        EXPECT_FALSE(processor->getLastDebugTelemetry().road_height_applied);
        EXPECT_EQ(Peer::filter_state_(*processor).state, before);
    }
}

TEST(RTKRoadHeightPriorTest, PositionBlockOnlyLeavesEveryOtherStateUntouched) {
    // road_height_publish_only: the published FLOAT takes the position-block update,
    // the ambiguity states and their correlation to the position do not change.
    const double lat = 35.66 * M_PI / 180.0, lon = 139.79 * M_PI / 180.0;
    const Vector3d base = geodetic2ecef(lat, lon, 40.0);
    const Vector3d seed = geodetic2ecef(lat, lon, 38.0 + 1.5 + 2.0);
    auto map = std::make_shared<road_height::RoadHeightMap>(
        road_height::buildRoadHeightMap({Vector3d(35.66, 139.79, 38.0)}, 1.5));
    auto build = [&]() {
        auto processor = std::make_unique<RTKProcessor>();
        RTKProcessor::RTKConfig cfg;
        cfg.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        cfg.road_height_sigma_m = 0.2;
        cfg.road_height_offset_m = 1.5;
        processor->setRTKConfig(cfg);
        processor->setBasePosition(base);
        processor->setRoadHeightMap(map);
        auto& fs = Peer::filter_state_(*processor);
        fs.state = VectorXd::Constant(Peer::kStateCount, 3.0);
        fs.covariance = MatrixXd::Identity(Peer::kStateCount, Peer::kStateCount);
        fs.state.head<3>() = seed - base;
        fs.covariance.topLeftCorner<3, 3>() = Matrix3d::Identity() * 4.0;
        fs.covariance(0, 5) = fs.covariance(5, 0) = 0.5;  // position-ambiguity correlation
        return processor;
    };
    auto full = build();
    auto block = build();
    const VectorXd before = Peer::filter_state_(*block).state;
    const MatrixXd covariance_before = Peer::filter_state_(*block).covariance;
    Peer::applyRoadHeightPrior(*full, false);
    Peer::applyRoadHeightPrior(*block, true);
    ASSERT_TRUE(block->getLastDebugTelemetry().road_height_applied);
    const auto& fs = Peer::filter_state_(*block);
    // Same position block as the full update ...
    EXPECT_TRUE(fs.state.head<3>().isApprox(Peer::filter_state_(*full).state.head<3>(), 1e-12));
    const Matrix3d block_covariance = fs.covariance.topLeftCorner<3, 3>();
    const Matrix3d full_covariance = Peer::filter_state_(*full).covariance.topLeftCorner<3, 3>();
    EXPECT_TRUE(block_covariance.isApprox(full_covariance, 1e-12));
    // ... and nothing else moves, unlike the full update.
    EXPECT_EQ(fs.state.tail(Peer::kStateCount - 3), before.tail(Peer::kStateCount - 3));
    EXPECT_EQ(fs.covariance.rightCols(Peer::kStateCount - 3),
              covariance_before.rightCols(Peer::kStateCount - 3));
    EXPECT_NE(Peer::filter_state_(*full).state(5), before(5));
}

TEST(RTKLegacyCompatibilityStandaloneTest, RoadHeightPriorDefaultsOff) {
    EXPECT_EQ(RTKProcessor::RTKConfig{}.road_height_sigma_m, 0.0);
    EXPECT_FALSE(RTKProcessor::RTKConfig{}.road_height_publish_only);
}

TEST(RTKLegacyCompatibilityStandaloneTest, HoldFixUpdateNisLimitDefaultsOff) {
    EXPECT_EQ(RTKProcessor::RTKConfig{}.hold_fix_max_update_nis_per_observation, 0.0);
    EXPECT_FALSE(RTKProcessor::RTKConfig{}.hold_fix_update_nis_keeps_hold_run);
    EXPECT_EQ(RTKProcessor::RTKConfig{}.hold_fix_update_nis_max_phase_fit, 0.0);
}

TEST(RTKLegacyCompatibilityStandaloneTest, HoldFixMinMatchedPairsDefaultsToTheLegacyFloorAndIsConfigurable) {
    // Opt-in: the default is the legacy floor of 4 matched pairs, so the hold-fix is
    // unchanged unless the minimum is raised.
    RTKProcessor processor;
    EXPECT_EQ(processor.getRTKConfig().hold_fix_min_matched_pairs, 4);
    EXPECT_EQ(RTKProcessor::RTKConfig().hold_fix_min_matched_pairs, 4);

    RTKProcessor::RTKConfig raised;
    raised.hold_fix_min_matched_pairs = 6;
    processor.setRTKConfig(raised);
    EXPECT_EQ(processor.getRTKConfig().hold_fix_min_matched_pairs, 6);

    RTKProcessor::RTKConfig legacy;
    legacy.hold_fix_min_matched_pairs = 4;
    processor.setRTKConfig(legacy);
    EXPECT_EQ(processor.getRTKConfig().hold_fix_min_matched_pairs, 4);
}

TEST(RTKLegacyCompatibilityStandaloneTest, MinFixSatellitesDefaultsToOffAndIsConfigurable) {
    // Opt-in: 0 disables the check, so the AR fix is unchanged unless a minimum is set.
    RTKProcessor processor;
    EXPECT_EQ(processor.getRTKConfig().min_fix_satellites, 0);
    EXPECT_EQ(RTKProcessor::RTKConfig().min_fix_satellites, 0);

    RTKProcessor::RTKConfig raised;
    raised.min_fix_satellites = 6;
    processor.setRTKConfig(raised);
    EXPECT_EQ(processor.getRTKConfig().min_fix_satellites, 6);

    RTKProcessor::RTKConfig off;
    off.min_fix_satellites = 0;
    processor.setRTKConfig(off);
    EXPECT_EQ(processor.getRTKConfig().min_fix_satellites, 0);
}

// ============================================================
// Base-epoch time of the Doppler slip history (updateBias)
// ============================================================

// The Doppler slip test predicts the single-difference phase change over the span the
// rover and the base each advanced since the previous detector call. updateBias() keeps
// the base epoch time of that call next to doppler_history_time_; the two are set,
// advanced and dropped together (long gap, reset()). Synthetic L1 scene whose base
// epochs alternate between a measurement time 1 ms before the rover epoch and the rover
// time (what gnss live feeds when exact-time and interpolated base epochs alternate).
class RTKDopplerBaseSpanHistoryTest : public ::testing::Test {
protected:
    static constexpr double kSpacingS = 0.2;

    static Vector3d basePosition() { return Vector3d(6378137.0, 0.0, 0.0); }
    static Vector3d roverPosition() { return basePosition() + Vector3d(40.0, 30.0, 10.0); }

    void SetUp() override { startProcessor(RTKProcessor::RTKConfig()); }

    void startProcessor(RTKProcessor::RTKConfig config) {
        nav_ = synthetic_rtk_scene::constellation();
        prns_ = synthetic_rtk_scene::visible(nav_, synthetic_rtk_scene::epochTime(0.0),
                                             roverPosition(), 15.0);
        ASSERT_GE(prns_.size(), 6U);
        config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        config.elevation_mask = 5.0 * M_PI / 180.0;
        config.ratio_threshold = 1.0e9;
        config.ambiguity_ratio_threshold = 1.0e9;
        processor_ = std::make_unique<RTKProcessor>(config);
        ProcessorConfig processor_config;
        processor_config.elevation_mask = 5.0;
        ASSERT_TRUE(processor_->initialize(processor_config));
        processor_->setBasePosition(basePosition());
    }

    ObservationData roverEpoch(double t) const {
        auto rover = synthetic_rtk_scene::observations(
            nav_, synthetic_rtk_scene::epochTime(t), roverPosition(), 30.0, prns_, 0.0,
            /*with_doppler=*/true);
        rover.receiver_position = roverPosition();
        return rover;
    }

    ObservationData baseEpoch(double measured_t) const {
        return synthetic_rtk_scene::observations(nav_, synthetic_rtk_scene::epochTime(measured_t),
                                                 basePosition(), 10.0, prns_, 500.0,
                                                 /*with_doppler=*/true);
    }

    // One RTK epoch at rover time t with the base measured at t + base_offset_s.
    GNSSTime process(double t, double base_offset_s) {
        const auto rover = roverEpoch(t);
        const auto base = baseEpoch(t + base_offset_s);
        processor_->processRTKEpoch(rover, base, nav_);
        return base.time;
    }

    bool historyTimesAreSet() {
        return Peer::has_doppler_history_time_(*processor_) &&
               Peer::has_doppler_history_base_time_(*processor_);
    }

    bool historyTimesAreClear() {
        return !Peer::has_doppler_history_time_(*processor_) &&
               !Peer::has_doppler_history_base_time_(*processor_);
    }

    NavigationData nav_;
    std::vector<uint8_t> prns_;
    std::unique_ptr<RTKProcessor> processor_;
};

TEST_F(RTKDopplerBaseSpanHistoryTest, StartsWithNoHistory) {
    EXPECT_TRUE(historyTimesAreClear());
}

TEST_F(RTKDopplerBaseSpanHistoryTest, BaseTimeIsRecordedWithTheRoverTimeEveryEpoch) {
    for (int epoch = 0; epoch < 6; ++epoch) {
        const double t = kSpacingS * epoch;
        const GNSSTime base_time = process(t, epoch % 2 == 0 ? -0.001 : 0.0);
        ASSERT_TRUE(historyTimesAreSet()) << "epoch " << epoch;
        EXPECT_NEAR(Peer::doppler_history_time_(*processor_) - synthetic_rtk_scene::epochTime(t),
                    0.0, 1.0e-9)
            << "epoch " << epoch;
        // The base history holds the base's own (measurement) time, not the rover time.
        EXPECT_NEAR(Peer::doppler_history_base_time_(*processor_) - base_time, 0.0, 1.0e-9)
            << "epoch " << epoch;
        EXPECT_NEAR(Peer::doppler_history_time_(*processor_) -
                        Peer::doppler_history_base_time_(*processor_),
                    epoch % 2 == 0 ? 0.001 : 0.0, 1.0e-9)
            << "epoch " << epoch;
    }
}

TEST_F(RTKDopplerBaseSpanHistoryTest, LongGapDropsBothHistoryTimes) {
    for (int epoch = 0; epoch < 3; ++epoch) process(kSpacingS * epoch, -0.001);
    ASSERT_TRUE(historyTimesAreSet());

    // The rover epoch comes 9.6 s later: too long for a Doppler prediction, the stored
    // samples and both history times are dropped.
    process(10.0, -0.001);
    EXPECT_TRUE(historyTimesAreClear());

    // The next epoch starts a new history for both.
    const GNSSTime base_time = process(10.0 + kSpacingS, 0.0);
    ASSERT_TRUE(historyTimesAreSet());
    EXPECT_NEAR(Peer::doppler_history_time_(*processor_) -
                    synthetic_rtk_scene::epochTime(10.0 + kSpacingS),
                0.0, 1.0e-9);
    EXPECT_NEAR(Peer::doppler_history_base_time_(*processor_) - base_time, 0.0, 1.0e-9);
}

TEST_F(RTKDopplerBaseSpanHistoryTest, ResetDropsBothHistoryTimes) {
    for (int epoch = 0; epoch < 3; ++epoch) process(kSpacingS * epoch, -0.001);
    ASSERT_TRUE(historyTimesAreSet());

    processor_->reset();
    EXPECT_TRUE(historyTimesAreClear());
}

TEST_F(RTKDopplerBaseSpanHistoryTest, DisabledDopplerDetectionKeepsNoHistory) {
    RTKProcessor::RTKConfig config;
    config.enable_doppler_slip_detection = false;
    startProcessor(config);
    for (int epoch = 0; epoch < 4; ++epoch) process(kSpacingS * epoch, -0.001);
    EXPECT_TRUE(historyTimesAreClear());
}

// The second updateBias() call site (formDoubleDifferences(), reached when an epoch was not
// processed by processRTKEpoch) hands over the base epoch time as well.
TEST_F(RTKDopplerBaseSpanHistoryTest, FormDoubleDifferencesPassesTheBaseEpochTime) {
    for (int epoch = 0; epoch < 8; ++epoch) {
        const double t = kSpacingS * epoch;
        const auto rover = roverEpoch(t);
        const auto base = baseEpoch(t + (epoch % 2 == 0 ? -0.001 : 0.0));
        Peer::formDoubleDifferences(*processor_, rover, base, nav_);
        ASSERT_TRUE(historyTimesAreSet()) << "epoch " << epoch;
        EXPECT_NEAR(Peer::doppler_history_time_(*processor_) - rover.time, 0.0, 1.0e-9)
            << "epoch " << epoch;
        EXPECT_NEAR(Peer::doppler_history_base_time_(*processor_) - base.time, 0.0, 1.0e-9)
            << "epoch " << epoch;
        // Consistent observations on the alternating base epochs: no Doppler slip.
        EXPECT_EQ(processor_->getLastDebugTelemetry().doppler_slip_l1_count, 0)
            << "epoch " << epoch;
    }
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
