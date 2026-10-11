// RTKConfig::enable_canyon_carrier_prior: TDCP displacement solver, code
// consistency test, engage/hold/release monitor and an end-to-end synthetic
// scene (stationary rover, zenith-side satellites re-acquired under a blocked
// sky: carrier continuous, code biased +10 m). Public API only; no dataset.

#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_carrier_prior.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>

#include "synthetic_rtk_scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

using namespace libgnss;
using namespace libgnss::rtk_carrier_prior;

namespace {

std::vector<TdcpRow> makeRows(const Eigen::Vector3d& displacement, double clock_m,
                              int count, double noise_m) {
    // Spread of line-of-sight vectors with a realistic vertical bias.
    const double az_deg[] = {10, 70, 130, 190, 250, 310, 40, 220, 100, 280};
    const double el_deg[] = {70, 35, 50, 25, 60, 40, 20, 80, 30, 55};
    std::vector<TdcpRow> rows;
    for (int i = 0; i < count; ++i) {
        const double az = az_deg[i % 10] * M_PI / 180.0;
        const double el = el_deg[i % 10] * M_PI / 180.0;
        TdcpRow row;
        row.los = Eigen::Vector3d(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az),
                                  std::sin(el));
        // Deterministic pseudo-noise, bounded by `noise_m`.
        const double jitter = noise_m * std::sin(1.7 * i + 0.3);
        row.delta_m = -row.los.dot(displacement) + clock_m + jitter;
        row.sigma_m = 0.005 / std::max(std::sin(el), 0.17);
        rows.push_back(row);
    }
    return rows;
}

TEST(RtkCarrierPriorTdcp, RecoversDisplacementAndClockTerm) {
    const Eigen::Vector3d truth(0.31, -0.12, 0.07);
    const auto solution = solveTdcpDisplacement(makeRows(truth, 13.4, 8, 0.002));
    ASSERT_TRUE(solution.ok);
    EXPECT_LT((solution.displacement_ecef - truth).norm(), 0.006);
    EXPECT_NEAR(solution.clock_term_m, 13.4, 0.01);
    EXPECT_EQ(solution.rows_used, 8);
    EXPECT_EQ(solution.rows_rejected, 0);
    EXPECT_LT(solution.rms_m, 0.003);
    EXPECT_TRUE(solution.covariance.allFinite());
    EXPECT_GT(solution.covariance.trace(), 0.0);
    EXPECT_LT(solution.covariance.trace(), 1e-3);
}

TEST(RtkCarrierPriorTdcp, DropsOneCycleSlipRowAndKeepsTheDisplacement) {
    const Eigen::Vector3d truth(-0.05, 0.22, 0.11);
    auto rows = makeRows(truth, -2.0, 8, 0.001);
    rows[3].delta_m += 0.1903;  // one L1 cycle
    const auto solution = solveTdcpDisplacement(rows);
    ASSERT_TRUE(solution.ok);
    EXPECT_EQ(solution.rows_rejected, 1);
    EXPECT_EQ(solution.rows_used, 7);
    EXPECT_LT((solution.displacement_ecef - truth).norm(), 0.006);
}

TEST(RtkCarrierPriorTdcp, FailsWithoutRedundancyAndRemovesRepeatedSlips) {
    const Eigen::Vector3d truth(0.1, 0.1, 0.1);
    // 4 unknowns: 4 rows leave no redundancy to validate anything.
    EXPECT_FALSE(solveTdcpDisplacement(makeRows(truth, 0.0, 4, 0.0)).ok);
    // 5 rows, one slipped: removing it would leave fewer than the minimum.
    auto five = makeRows(truth, 0.0, 5, 0.0);
    five[0].delta_m += 0.19;
    EXPECT_FALSE(solveTdcpDisplacement(five).ok);
    // 10 rows, two slipped by a cycle: both are removed one after the other.
    auto ten = makeRows(truth, 0.0, 10, 0.0);
    ten[1].delta_m += 0.19;
    ten[4].delta_m -= 0.19;
    const auto two_slips = solveTdcpDisplacement(ten);
    ASSERT_TRUE(two_slips.ok);
    EXPECT_EQ(two_slips.rows_rejected, 2);
    EXPECT_LT((two_slips.displacement_ecef - truth).norm(), 1e-6);
    // Non-finite input.
    auto bad = makeRows(truth, 0.0, 8, 0.0);
    bad[2].delta_m = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(solveTdcpDisplacement(bad).ok);
}

TEST(RtkCarrierPriorTdcp, ScalesCovarianceWhenResidualsExceedNominalNoise) {
    const Eigen::Vector3d truth(0.0, 0.0, 0.0);
    const auto quiet = solveTdcpDisplacement(makeRows(truth, 0.0, 8, 0.001));
    const auto noisy = solveTdcpDisplacement(makeRows(truth, 0.0, 8, 0.016));
    ASSERT_TRUE(quiet.ok);
    ASSERT_TRUE(noisy.ok);
    EXPECT_GT(noisy.covariance.trace(), 1.5 * quiet.covariance.trace());
}

TEST(RtkCarrierPriorCodeConsistency, CountsOnlyZenithSideOutliersAroundTheMedian) {
    const double kDeg = M_PI / 180.0;
    // Receiver clock term of -57480 m is common and must not matter.
    const std::vector<double> residuals = {-57480.2, -57479.7, -57480.5, -57479.9,
                                           -57472.0, -57480.1};
    std::vector<double> elevations = {60 * kDeg, 50 * kDeg, 30 * kDeg,
                                      20 * kDeg, 66 * kDeg, 40 * kDeg};
    auto result = evaluateCodeConsistency(residuals, elevations, 3.0, 45 * kDeg);
    EXPECT_EQ(result.rows, 6);
    EXPECT_EQ(result.inconsistent_rows, 1);  // the 66 deg satellite, +8 m

    // The same bias on a 20 deg satellite is ordinary multipath territory.
    elevations[4] = 20 * kDeg;
    result = evaluateCodeConsistency(residuals, elevations, 3.0, 45 * kDeg);
    EXPECT_EQ(result.inconsistent_rows, 0);

    // Mismatched inputs are rejected rather than guessed.
    result = evaluateCodeConsistency(residuals, {60 * kDeg}, 3.0, 45 * kDeg);
    EXPECT_EQ(result.inconsistent_rows, 0);
    EXPECT_EQ(result.rows, 0);
}

MonitorInput input(double t, bool healthy, int inconsistent, bool available = true,
                   bool bridged = false) {
    MonitorInput in;
    in.time_s = t;
    in.prior_available = available;
    in.prior_bridged = bridged;
    in.previous_epoch_healthy = healthy;
    in.inconsistent_rows = inconsistent;
    return in;
}

TEST(RtkCarrierPriorMonitor, NeedsAHealthyStreakBeforeItEngages) {
    Monitor monitor;
    // 4 s of healthy epochs, then evidence: streak too short.
    for (int i = 0; i < 40; ++i) EXPECT_FALSE(monitor.update(input(0.1 * i, true, 0)));
    EXPECT_FALSE(monitor.update(input(4.0, true, 1)));
    // Past the 5 s streak the same evidence engages.
    for (int i = 41; i < 60; ++i) EXPECT_FALSE(monitor.update(input(0.1 * i, true, 0)));
    EXPECT_TRUE(monitor.update(input(6.0, true, 1)));
    EXPECT_TRUE(monitor.engaged());
}

TEST(RtkCarrierPriorMonitor, DoesNotEngageFromBridgeOrWithoutEvidence) {
    Monitor monitor;
    for (int i = 0; i < 60; ++i) EXPECT_FALSE(monitor.update(input(0.1 * i, true, 0)));
    EXPECT_FALSE(monitor.update(input(6.0, true, 1, true, /*bridged=*/true)));
    EXPECT_FALSE(monitor.update(input(6.1, true, 1, /*available=*/false)));
    EXPECT_FALSE(monitor.engaged());
}

TEST(RtkCarrierPriorMonitor, HoldsWhileUnhealthyAndReleasesAfterFiveHealthySeconds) {
    Monitor monitor;
    double t = 0.0;
    for (; t < 6.0; t += 0.1) monitor.update(input(t, true, 0));
    ASSERT_TRUE(monitor.update(input(t, true, 2)));
    // The offending satellite drops out of tracking: no evidence, but the
    // update is still inconsistent (blocked sky). The prior must stay.
    for (int i = 0; i < 300; ++i) {
        t += 0.1;
        EXPECT_TRUE(monitor.update(input(t, false, 0))) << "epoch " << i;
    }
    // Consistent again: the prior is released only after the 5 s hold.
    bool released = false;
    double healthy_for = 0.0;
    for (int i = 0; i < 100 && !released; ++i) {
        t += 0.1;
        healthy_for += 0.1;
        if (!monitor.update(input(t, true, 0))) released = true;
    }
    ASSERT_TRUE(released);
    EXPECT_GE(healthy_for, 4.9);
    EXPECT_LE(healthy_for, 5.3);
    EXPECT_FALSE(monitor.engaged());
}

TEST(RtkCarrierPriorMonitor, LostPriorAndContradictionRestartTheStreak) {
    Monitor monitor;
    double t = 0.0;
    for (; t < 6.0; t += 0.1) monitor.update(input(t, true, 0));
    ASSERT_TRUE(monitor.update(input(t, true, 1)));
    t += 0.1;
    EXPECT_FALSE(monitor.update(input(t, true, 1, /*available=*/false)));
    EXPECT_FALSE(monitor.engaged());
    // Healthy again but without a fresh streak: no engagement on evidence.
    t += 0.1;
    EXPECT_FALSE(monitor.update(input(t, true, 1)));
    for (int i = 0; i < 60; ++i) {
        t += 0.1;
        monitor.update(input(t, true, 0));
    }
    t += 0.1;
    ASSERT_TRUE(monitor.update(input(t, true, 1)));
    monitor.reportContradiction();
    EXPECT_FALSE(monitor.engaged());
    t += 0.1;
    EXPECT_FALSE(monitor.update(input(t, true, 1)));
}

TEST(RtkCarrierPriorMonitor, EpisodeIsBoundedAndEpochGapsBreakTheStreak) {
    Monitor monitor;
    double t = 0.0;
    for (; t < 6.0; t += 0.1) monitor.update(input(t, true, 0));
    ASSERT_TRUE(monitor.update(input(t, true, 1)));
    const double start = t;
    bool ended = false;
    while (t - start < 200.0) {
        t += 0.5;
        if (!monitor.update(input(t, false, 1))) {
            ended = true;
            break;
        }
    }
    ASSERT_TRUE(ended);
    EXPECT_GE(t - start, 180.0);
    EXPECT_LE(t - start, 181.0);

    Monitor gapped;
    double g = 0.0;
    for (; g < 3.0; g += 0.1) gapped.update(input(g, true, 0));
    g += 2.0;  // skipped epochs
    for (int i = 0; i < 30; ++i, g += 0.1) gapped.update(input(g, true, 0));
    // Only ~3 s of continuous health since the gap.
    EXPECT_FALSE(gapped.update(input(g, true, 1)));
}

// ---------------------------------------------------------------------------
// End to end on the synthetic scene.
// ---------------------------------------------------------------------------

using namespace synthetic_rtk_scene;

const Vector3d kBase(6378137.0, 0.0, 0.0);
const Vector3d kRover = kBase + Vector3d(40.0, 30.0, 10.0);
constexpr double kWavelengthM = 0.19029367;
constexpr double kSpacingS = 0.2;
constexpr int kCleanEpochs = 150;   // 30 s: healthy streak > 5 s
constexpr int kBlockedEpochs = 200; // 40 s

struct SceneResult {
    double final_error_m = 0.0;
    double max_error_m = 0.0;
    RTKProcessor::CarrierPriorStats stats;
};

// `block`: from epoch kCleanEpochs the satellites above `block_min_elevation`
// are re-acquired with a cycle-slip flag, an integer cycle jump, and a +10 m
// code bias (direct signal blocked, carrier path intact and continuous
// afterwards). The rover is stationary.
SceneResult runScene(bool enable_prior, bool block) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    EXPECT_GE(prns.size(), 8U);

    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.elevation_mask = 5.0 * M_PI / 180.0;
    config.ratio_threshold = 1.0e9;  // stay in the float branch
    config.ambiguity_ratio_threshold = 1.0e9;
    config.enable_canyon_carrier_prior = enable_prior;
    auto processor = std::make_unique<RTKProcessor>(config);
    ProcessorConfig processor_config;
    processor_config.elevation_mask = 5.0;
    EXPECT_TRUE(processor->initialize(processor_config));
    processor->setBasePosition(kBase);

    std::vector<uint8_t> blocked;
    for (const uint8_t prn : prns) {
        const auto g = geometry(nav, prn, epochTime(0.0), kRover);
        if (g.elevation_rad >= 45.0 * M_PI / 180.0) blocked.push_back(prn);
    }
    EXPECT_FALSE(blocked.empty());
    EXPECT_LT(blocked.size() + 5U, prns.size());

    SceneResult result;
    for (int epoch = 0; epoch < kCleanEpochs + kBlockedEpochs; ++epoch) {
        const double t = kSpacingS * epoch;
        auto rover = observations(nav, epochTime(t), kRover, 30.0, prns, 0.0);
        auto base = observations(nav, epochTime(t), kBase, 10.0, prns, 500.0);
        rover.receiver_position = kRover;
        if (block && epoch >= kCleanEpochs) {
            for (auto& obs : rover.observations) {
                const uint8_t prn = obs.satellite.prn;
                if (std::find(blocked.begin(), blocked.end(), prn) == blocked.end()) continue;
                obs.pseudorange += 10.0;
                obs.carrier_phase += 37.0;  // integer cycle jump at re-acquisition
                if (epoch == kCleanEpochs) obs.lli = 1;
            }
        }
        const auto solution = processor->processRTKEpoch(rover, base, nav);
        if (epoch >= kCleanEpochs && solution.isValid()) {
            const double error = (solution.position_ecef - kRover).norm();
            result.max_error_m = std::max(result.max_error_m, error);
            result.final_error_m = error;
        }
    }
    result.stats = processor->getCarrierPriorStats();
    return result;
}

TEST(RtkCarrierPriorScene, DefaultIsOffAndCountsNothing) {
    RTKProcessor::RTKConfig config;
    EXPECT_FALSE(config.enable_canyon_carrier_prior);
    const SceneResult off = runScene(/*enable_prior=*/false, /*block=*/true);
    EXPECT_EQ(off.stats.epochs_evaluated, 0U);
    EXPECT_EQ(off.stats.engaged_epochs, 0U);
}

TEST(RtkCarrierPriorScene, CleanSkyNeverEngagesAndChangesNothing) {
    const SceneResult off = runScene(false, /*block=*/false);
    const SceneResult on = runScene(true, /*block=*/false);
    EXPECT_GT(on.stats.epochs_evaluated, 0U);
    EXPECT_EQ(on.stats.engaged_epochs, 0U);
    EXPECT_EQ(on.stats.phase_initialised_ambiguities, 0U);
    EXPECT_DOUBLE_EQ(on.final_error_m, off.final_error_m);
    EXPECT_DOUBLE_EQ(on.max_error_m, off.max_error_m);
}

TEST(RtkCarrierPriorScene, BlockedZenithSideCodeNoLongerDefinesThePosition) {
    const SceneResult off = runScene(false, /*block=*/true);
    const SceneResult on = runScene(true, /*block=*/true);
    // The scenario must be meaningful: the legacy filter takes the bias.
    EXPECT_GT(off.max_error_m, 2.0);
    // The prior engaged, seeded the re-acquired ambiguities from the carrier
    // and kept the position.
    EXPECT_GT(on.stats.engaged_epochs, 0U);
    EXPECT_GT(on.stats.phase_initialised_ambiguities, 0U);
    EXPECT_LT(on.max_error_m, 1.0);
    EXPECT_LT(on.max_error_m, 0.5 * off.max_error_m);
}

}  // namespace
