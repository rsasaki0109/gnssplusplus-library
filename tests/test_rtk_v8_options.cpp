// velocity_consistency_v8 RTK options (docs/online_pva_candidate_v9.md):
//   (k) RTKConfig::reject_float_seeded_at_base
//   (m) RTKConfig::spp_fallback_blank_max_anchor_age_s
//   (l) PositionSolution::float_prefit_gate_exceeded
//   (n) RTKProcessor::currentSpp()
// Synthetic observations only (no data files): the receivers and the 24-GPS
// constellation are modelled with the same geometry the processors use.
// Public-API-only, like the other RTK unit tests.

#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>

#include "synthetic_rtk_scene.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

using namespace libgnss;

namespace {

using namespace synthetic_rtk_scene;

const Vector3d kBase(6378137.0, 0.0, 0.0);
const Vector3d kRover = kBase + Vector3d(40.0, 30.0, 10.0);

RTKProcessor::RTKConfig rtkConfig() {
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.elevation_mask = 5.0 * M_PI / 180.0;
    return config;
}

ProcessorConfig processorConfig(double spp_elevation_mask_deg) {
    ProcessorConfig config;
    config.elevation_mask = spp_elevation_mask_deg;
    return config;
}

// RTK processor whose SPP sees no satellite (mask 89 deg) but whose DD filter
// accepts them (5 deg): the kinematic re-seed falls through to the base.
std::unique_ptr<RTKProcessor> makeProcessor(const RTKProcessor::RTKConfig& config,
                                            double spp_mask_deg) {
    auto processor = std::make_unique<RTKProcessor>(config);
    EXPECT_TRUE(processor->initialize(processorConfig(spp_mask_deg)));
    processor->setBasePosition(kBase);
    return processor;
}

}  // namespace

TEST(RtkV8OptionsTest, DefaultsAreOff) {
    const RTKProcessor::RTKConfig config;
    EXPECT_FALSE(config.reject_float_seeded_at_base);
    EXPECT_EQ(config.spp_fallback_blank_max_anchor_age_s, 0.0);
    EXPECT_FALSE(PositionSolution{}.float_prefit_gate_exceeded);
}

TEST(RtkV8OptionsTest, SyntheticSceneHasEnoughSatellites) {
    const auto nav = constellation();
    EXPECT_GE(visible(nav, epochTime(0.0), kBase, 15.0).size(), 6U);
    EXPECT_GE(visible(nav, epochTime(0.0), kRover, 15.0).size(), 6U);
}

// ---- (k) reject_float_seeded_at_base ------------------------------------

TEST(RtkV8OptionsTest, BaseSeededFloatIsEmittedByDefaultAndTracked) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    auto processor_owner = makeProcessor(rtkConfig(), 89.0);
    auto& processor = *processor_owner;
    const auto solution = processor.processRTKEpoch(
        observations(nav, epochTime(0.0), kRover, 30.0, prns, 0.0),
        observations(nav, epochTime(0.0), kBase, 10.0, prns, 500.0), nav);
    const auto& telemetry = processor.getLastDebugTelemetry();
    // Nothing is available but the base coordinates, and the epoch tracks it.
    EXPECT_FALSE(processor.currentSpp().isValid());
    EXPECT_TRUE(telemetry.rover_seed_from_base_fallback);
    EXPECT_FALSE(telemetry.float_seeded_at_base_rejected);
    EXPECT_EQ(solution.status, SolutionStatus::FLOAT);
    EXPECT_TRUE(solution.isValid());
}

TEST(RtkV8OptionsTest, BaseSeededFloatReturnsThroughFallbackAndFilterStaysInitialised) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    auto config = rtkConfig();
    config.reject_float_seeded_at_base = true;
    auto processor_owner = makeProcessor(config, 89.0);
    auto& processor = *processor_owner;
    const auto solution = processor.processRTKEpoch(
        observations(nav, epochTime(0.0), kRover, 30.0, prns, 0.0),
        observations(nav, epochTime(0.0), kBase, 10.0, prns, 500.0), nav);
    const auto& telemetry = processor.getLastDebugTelemetry();
    EXPECT_TRUE(telemetry.rover_seed_from_base_fallback);
    EXPECT_TRUE(telemetry.float_seeded_at_base_rejected);
    // fallback_spp: the invalid SPP yields no solution; the FLOAT is not emitted.
    EXPECT_NE(solution.status, SolutionStatus::FLOAT);
    EXPECT_FALSE(solution.isValid());
    Vector3d position;
    Matrix3d covariance;
    EXPECT_TRUE(processor.getFloatPosteriorPosition(position, covariance));
}

TEST(RtkV8OptionsTest, BaseSeedFlagClearsOnceARealPositionSeedsTheFilter) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    auto config = rtkConfig();
    config.reject_float_seeded_at_base = true;
    auto processor_owner = makeProcessor(config, 89.0);
    auto& processor = *processor_owner;
    // Epochs 0 and 1: base seed, rejected each time (nothing is remembered).
    for (int i = 0; i < 2; ++i) {
        const double t = 0.2 * i;
        const auto solution = processor.processRTKEpoch(
            observations(nav, epochTime(t), kRover, 30.0, prns, 0.0),
            observations(nav, epochTime(t), kBase, 10.0, prns, 500.0), nav);
        EXPECT_TRUE(processor.getLastDebugTelemetry().float_seeded_at_base_rejected) << i;
        EXPECT_FALSE(solution.isValid()) << i;
    }
    // Epoch 2: the rover header position is now available as a seed.
    auto rover = observations(nav, epochTime(0.4), kRover, 30.0, prns, 0.0);
    rover.receiver_position = kRover;
    const auto solution = processor.processRTKEpoch(
        rover, observations(nav, epochTime(0.4), kBase, 10.0, prns, 500.0), nav);
    EXPECT_FALSE(processor.getLastDebugTelemetry().rover_seed_from_base_fallback);
    EXPECT_FALSE(processor.getLastDebugTelemetry().float_seeded_at_base_rejected);
    EXPECT_EQ(solution.status, SolutionStatus::FLOAT);
    // The float seeded at the real position is nowhere near the base.
    EXPECT_LT((solution.position_ecef - kRover).norm(), 50.0);
}

TEST(RtkV8OptionsTest, RealSeedIsNeverRejectedEvenWithTheOptionOn) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    auto config = rtkConfig();
    config.reject_float_seeded_at_base = true;
    auto processor_owner = makeProcessor(config, 5.0);
    auto& processor = *processor_owner;  // SPP is valid
    const auto solution = processor.processRTKEpoch(
        observations(nav, epochTime(0.0), kRover, 30.0, prns, 0.0),
        observations(nav, epochTime(0.0), kBase, 10.0, prns, 500.0), nav);
    EXPECT_TRUE(processor.currentSpp().isValid());
    EXPECT_FALSE(processor.getLastDebugTelemetry().rover_seed_from_base_fallback);
    EXPECT_FALSE(processor.getLastDebugTelemetry().float_seeded_at_base_rejected);
    EXPECT_EQ(solution.status, SolutionStatus::FLOAT);
}

// ---- (n) currentSpp() ------------------------------------------------------

TEST(RtkV8OptionsTest, CurrentSppExposesTheEpochSolutionAndNeverGoesStale) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    auto with_doppler = [&](double t) {
        return observations(nav, epochTime(t), kRover, 30.0, prns, 0.0, true);
    };
    auto processor_owner = makeProcessor(rtkConfig(), 5.0);
    auto& processor = *processor_owner;
    EXPECT_FALSE(processor.currentSpp().isValid());
    processor.processRTKEpoch(with_doppler(0.0),
                              observations(nav, epochTime(0.0), kBase, 10.0, prns, 500.0), nav);
    const auto& first = processor.currentSpp();
    ASSERT_TRUE(first.isValid());
    EXPECT_NEAR((first.position_ecef - kRover).norm(), 0.0, 30.0);
    EXPECT_TRUE(first.has_velocity);
    EXPECT_TRUE(first.velocity_ecef.allFinite());
    EXPECT_TRUE(first.velocity_covariance.allFinite());
    EXPECT_GT(first.velocity_covariance.trace(), 0.0);
    // processEpoch() (the missing-base path of the online processor) sets it too.
    const auto spp = processor.processEpoch(with_doppler(0.2), nav);
    ASSERT_TRUE(spp.isValid());
    EXPECT_EQ(processor.currentSpp().time, spp.time);
    EXPECT_TRUE(processor.currentSpp().position_ecef.isApprox(spp.position_ecef, 0.0));
    // A later epoch without any usable rover observation replaces it.
    processor.processRTKEpoch(ObservationData(epochTime(0.4)),
                              observations(nav, epochTime(0.4), kBase, 10.0, prns, 500.0), nav);
    EXPECT_FALSE(processor.currentSpp().isValid());
    processor.processEpoch(ObservationData(epochTime(0.6)), nav);
    EXPECT_FALSE(processor.currentSpp().isValid());
}

// ---- (m) spp_fallback_blank_max_anchor_age_s ---------------------------------

namespace {
// Epoch 0: DD FLOAT with all satellites (sets the trusted anchor). Epoch 1 at
// `gap_s`: the rover moved 200 m and the base only matches 3 satellites, so
// the DD filter has < 4 satellites and the SPP fallback decides. The rover
// SPP uses exactly five satellites (the blanking rule's "<= 5").
struct BlankingRun {
    SolutionStatus status = SolutionStatus::NONE;
    bool age_limited = false;
    bool valid = false;
    int satellites = 0;
};
BlankingRun runBlanking(double max_anchor_age_s, double gap_s) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    auto config = rtkConfig();
    config.spp_fallback_blank_max_anchor_age_s = max_anchor_age_s;
    auto processor_owner = makeProcessor(config, 15.0);
    auto& processor = *processor_owner;
    const auto first = processor.processRTKEpoch(
        observations(nav, epochTime(0.0), kRover, 30.0, prns, 0.0),
        observations(nav, epochTime(0.0), kBase, 10.0, prns, 500.0), nav);
    EXPECT_EQ(first.status, SolutionStatus::FLOAT);
    // Highest five satellites seen by the moved rover for the SPP.
    const Vector3d moved = kRover + Vector3d(200.0, 0.0, 0.0);
    auto moved_prns = visible(nav, epochTime(gap_s), moved, 15.0);
    std::sort(moved_prns.begin(), moved_prns.end(), [&](uint8_t a, uint8_t b) {
        return geometry(nav, a, epochTime(gap_s), moved).elevation_rad >
               geometry(nav, b, epochTime(gap_s), moved).elevation_rad;
    });
    EXPECT_GE(moved_prns.size(), 6U);
    moved_prns.resize(5);
    std::vector<uint8_t> base_prns(moved_prns.begin(), moved_prns.begin() + 3);
    const auto second = processor.processRTKEpoch(
        observations(nav, epochTime(gap_s), moved, 30.0, moved_prns, 0.0),
        observations(nav, epochTime(gap_s), kBase, 10.0, base_prns, 500.0), nav);
    BlankingRun run;
    run.status = second.status;
    run.valid = second.isValid();
    run.satellites = second.num_satellites;
    run.age_limited = processor.getLastDebugTelemetry().spp_blank_age_limited;
    return run;
}
}  // namespace

TEST(RtkV8OptionsTest, SppFallbackBlankingHasNoAgeLimitByDefault) {
    const auto old_anchor = runBlanking(0.0, 10.0);
    EXPECT_FALSE(old_anchor.valid);
    EXPECT_FALSE(old_anchor.age_limited);
    const auto young_anchor = runBlanking(0.0, 2.0);
    EXPECT_FALSE(young_anchor.valid);
}

TEST(RtkV8OptionsTest, SppFallbackBlankingIsBoundedByTheAnchorAge) {
    // Anchor 10 s old, limit 3 s: the SPP is not blanked.
    const auto old_anchor = runBlanking(3.0, 10.0);
    EXPECT_TRUE(old_anchor.age_limited);
    EXPECT_TRUE(old_anchor.valid);
    EXPECT_EQ(old_anchor.status, SolutionStatus::SPP);
    EXPECT_EQ(old_anchor.satellites, 5);
    // Anchor 2 s old, limit 3 s: the rule still applies.
    const auto young_anchor = runBlanking(3.0, 2.0);
    EXPECT_FALSE(young_anchor.age_limited);
    EXPECT_FALSE(young_anchor.valid);
    // Exactly at the limit still blanks (age <= limit).
    const auto at_limit = runBlanking(10.0, 10.0);
    EXPECT_FALSE(at_limit.valid);
}

// ---- (l) float_prefit_gate_exceeded ------------------------------------------

TEST(RtkV8OptionsTest, FloatPrefitGateFlagFollowsTheConfiguredLimits) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    auto run = [&](double rms_limit, double max_limit, double rover_code_error_m) {
        auto config = rtkConfig();
        config.max_float_prefit_residual_rms_m = rms_limit;
        config.max_float_prefit_residual_max_m = max_limit;
        auto processor_owner = makeProcessor(config, 5.0);
    auto& processor = *processor_owner;
        auto rover = observations(nav, epochTime(0.0), kRover, 30.0, prns, 0.0);
        // A pseudorange error on one satellite appears in the DD prefit residual.
        rover.observations.front().pseudorange += rover_code_error_m;
        return processor.processRTKEpoch(
            rover, observations(nav, epochTime(0.0), kBase, 10.0, prns, 500.0), nav);
    };
    // Disabled limits (0): never flagged, whatever the residual.
    const auto disabled = run(0.0, 0.0, 400.0);
    ASSERT_TRUE(disabled.isValid());
    EXPECT_FALSE(disabled.float_prefit_gate_exceeded);
    EXPECT_GT(disabled.rtk_update_prefit_residual_max_m, 10.0);
    // Enabled limits: flagged exactly when the solution's own residual exceeds one.
    const auto rms_limited = run(4.0, 0.0, 400.0);
    EXPECT_EQ(rms_limited.float_prefit_gate_exceeded,
              rms_limited.rtk_update_prefit_residual_rms_m > 4.0);
    const auto max_limited = run(0.0, 10.0, 400.0);
    EXPECT_TRUE(max_limited.float_prefit_gate_exceeded);
    EXPECT_GT(max_limited.rtk_update_prefit_residual_max_m, 10.0);
    const auto clean = run(4.0, 10.0, 0.0);
    ASSERT_TRUE(clean.isValid());
    EXPECT_FALSE(clean.float_prefit_gate_exceeded);
    EXPECT_LE(clean.rtk_update_prefit_residual_rms_m, 4.0);
    EXPECT_LE(clean.rtk_update_prefit_residual_max_m, 10.0);
}
