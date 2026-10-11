// Doppler/phase slip test interval (RTKProcessor::updateBias).
//
// The test compares the single-difference phase change since the previous
// sample with SD range rate * dt. The stored sample is refreshed on every
// updateBias() call, but the generic state dt is the time since the last
// *remembered* solution; whenever an epoch is rejected after the filter update
// (float-prefit reset, innovation/jump gates) the two differ. The SD range rate
// carries the rover-minus-base receiver clock drift (tens to >100 m/s on the
// UrbanNav Trimble pair), so the mismatch rate * (dt - spacing) exceeds the
// threshold for every satellite and flags a false slip, which resets all
// ambiguities. Synthetic noise-free L1 observations with a rover clock drift;
// public API only.

#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>

#include "synthetic_rtk_scene.hpp"

#include <memory>
#include <vector>

using namespace libgnss;
using namespace synthetic_rtk_scene;

namespace {

const Vector3d kBase(6378137.0, 0.0, 0.0);
const Vector3d kRover = kBase + Vector3d(40.0, 30.0, 10.0);
constexpr double kWavelengthM = 0.19029367;
constexpr double kSpacingS = 0.2;

struct Counts {
    int doppler_slips = 0;
    int ambiguity_resets = 0;
    int epochs = 0;
};

// Rover receiver clock drifts at `drift_mps` (carrier phase and Doppler follow).
// `force_rejections` makes the float-prefit gate reject (and reset after) every
// epoch, so the last remembered solution time stops advancing.
Counts run(bool force_rejections, double drift_mps, int epochs) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    EXPECT_GE(prns.size(), 6U);

    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.elevation_mask = 5.0 * M_PI / 180.0;
    // Keep the scene in the FLOAT branch so the float-prefit gate is reached.
    config.ratio_threshold = 1.0e9;
    config.ambiguity_ratio_threshold = 1.0e9;
    if (force_rejections) {
        config.max_float_prefit_residual_rms_m = 1.0e-9;
        config.max_float_prefit_residual_reset_streak = 1;
    }
    auto processor = std::make_unique<RTKProcessor>(config);
    ProcessorConfig processor_config;
    processor_config.elevation_mask = 5.0;
    EXPECT_TRUE(processor->initialize(processor_config));
    processor->setBasePosition(kBase);

    Counts counts;
    for (int epoch = 0; epoch < epochs; ++epoch) {
        const double t = kSpacingS * epoch;
        auto rover = observations(nav, epochTime(t), kRover, 30.0 + drift_mps * t, prns, 0.0,
                                  /*with_doppler=*/true);
        auto base = observations(nav, epochTime(t), kBase, 10.0, prns, 500.0,
                                 /*with_doppler=*/true);
        rover.receiver_position = kRover;
        for (auto& obs : rover.observations) {
            obs.carrier_phase += drift_mps * t / kWavelengthM;
            obs.doppler -= drift_mps / kWavelengthM;
        }
        processor->processRTKEpoch(rover, base, nav);
        const auto& telemetry = processor->getLastDebugTelemetry();
        counts.doppler_slips += telemetry.doppler_slip_l1_count;
        counts.ambiguity_resets += telemetry.ambiguity_reset_l1_count;
        ++counts.epochs;
    }
    return counts;
}

}  // namespace

TEST(RtkDopplerSlipDtTest, ConsistentDopplerDoesNotSlipWithClockDrift) {
    // Baseline: every epoch is remembered, the interval is the sample spacing.
    const Counts counts = run(/*force_rejections=*/false, /*drift_mps=*/30.0, 12);
    EXPECT_EQ(counts.doppler_slips, 0);
}

TEST(RtkDopplerSlipDtTest, RejectedEpochsDoNotCreateFalseDopplerSlips) {
    // Every epoch is rejected after updateBias() ran, so the last remembered
    // solution stays at the first epoch while the Doppler phase history is
    // refreshed each epoch. The interval must come from the history.
    const Counts counts = run(/*force_rejections=*/true, /*drift_mps=*/30.0, 12);
    EXPECT_EQ(counts.doppler_slips, 0);
}
