// Phase 2a unit tests: CMC-aware DD reference-satellite selection with
// hysteresis (RTKConfig::cmc_aware_reference_selection).
//
// The stateful algorithm (rtk_cmc_reference::CmcSuspectTracker /
// ReferenceHysteresis, include/libgnss++/algorithms/rtk_cmc_reference.hpp)
// is deliberately factored out of RTKProcessor into small, dependency-free
// classes precisely so it can be unit tested directly with synthetic
// per-epoch inputs -- no RINEX/observation machinery needed, and no
// "#define private public" trick (which does not work reliably: MSVC's ABI
// encodes member access in mangled names, and this build's clang targets
// the MSVC ABI on Windows -- see tests/test_rtk_legacy.cpp's guard). The
// last test in this file exercises RTKProcessor's public API only, to
// confirm the knob defaults off and stays a strict no-op end to end.

#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_cmc_reference.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/io/rinex.hpp>

#include <cmath>
#include <filesystem>
#include <limits>
#include <string>

using namespace libgnss;
using rtk_cmc_reference::CmcSuspectTracker;
using rtk_cmc_reference::ReferenceHysteresis;

namespace {

SatelliteId gps(int prn) {
    return SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(prn));
}

ReferenceHysteresis::Candidate makeCandidate(int prn, double elevation_rad, bool dual_frequency,
                                             bool suspect) {
    ReferenceHysteresis::Candidate c;
    c.satellite = gps(prn);
    c.elevation_rad = elevation_rad;
    c.dual_frequency = dual_frequency;
    c.suspect = suspect;
    return c;
}

}  // namespace

// --- CmcSuspectTracker -------------------------------------------------

TEST(CmcSuspectTrackerTest, FirstSampleAndWarmupNeverSuspect) {
    // warmup_epochs=3: the first classify() seeds the baseline (returns
    // false), and the next 2 average into it (also false) regardless of how
    // far the raw value is from a hypothetical "true" baseline.
    CmcSuspectTracker tracker(/*level_threshold_m=*/0.5, /*warmup_epochs=*/3);
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));
    EXPECT_FALSE(tracker.classify(gps(1), 5.0, false));  // still warming up
    EXPECT_FALSE(tracker.classify(gps(1), 5.0, false));
}

TEST(CmcSuspectTrackerTest, FlagsDeviationBeyondThresholdAfterWarmup) {
    CmcSuspectTracker tracker(/*level_threshold_m=*/0.4, /*warmup_epochs=*/2,
                              /*baseline_alpha=*/0.5);
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));  // seed
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));  // warmup epoch 2 -> baseline settles at 0
    EXPECT_FALSE(tracker.classify(gps(1), 0.1, false));  // within threshold
    EXPECT_TRUE(tracker.classify(gps(1), 1.0, false));   // 1.0 - baseline(~0.05) > 0.4 -> suspect
}

TEST(CmcSuspectTrackerTest, CleanSatelliteNeverFlaggedAcrossManyEpochs) {
    CmcSuspectTracker tracker(0.4, 3, 0.05);
    for (int i = 0; i < 50; ++i) {
        // Small jitter around zero -- never exceeds the 0.4 m threshold.
        const double noise = (i % 2 == 0) ? 0.05 : -0.05;
        EXPECT_FALSE(tracker.classify(gps(1), noise, false));
    }
}

TEST(CmcSuspectTrackerTest, ArcRestartResetsBaselineInsteadOfFlagging) {
    CmcSuspectTracker tracker(0.4, 2, 0.5);
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));  // baseline ~0
    // A cycle slip re-anchors the ambiguity: CMC jumps to 3.0, but
    // arc_restarted=true means this is treated as a fresh baseline seed,
    // not a suspect deviation.
    EXPECT_FALSE(tracker.classify(gps(1), 3.0, true));
    EXPECT_FALSE(tracker.classify(gps(1), 3.0, true));  // still warming up post-restart
    EXPECT_FALSE(tracker.classify(gps(1), 3.05, false));  // close to the new baseline
    EXPECT_TRUE(tracker.classify(gps(1), 4.0, false));    // now a real deviation from ~3.0
}

TEST(CmcSuspectTrackerTest, NonFiniteDeviationNeverFlaggedAndLeavesStateUntouched) {
    CmcSuspectTracker tracker(0.4, 2, 0.5);
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));
    EXPECT_FALSE(tracker.classify(gps(1), std::numeric_limits<double>::quiet_NaN(), false));
    // Baseline should be unaffected by the NaN sample -- a small in-range
    // value right after it is still classified against the pre-NaN baseline.
    EXPECT_FALSE(tracker.classify(gps(1), 0.1, false));
}

TEST(CmcSuspectTrackerTest, ZeroOrNegativeThresholdDisablesClassification) {
    CmcSuspectTracker tracker(0.0, 2, 0.5);
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));
    EXPECT_FALSE(tracker.classify(gps(1), 0.0, false));
    EXPECT_FALSE(tracker.classify(gps(1), 1000.0, false));  // huge deviation, still not flagged
}

TEST(CmcSuspectTrackerTest, PruneMissingDropsUnseenSatellites) {
    CmcSuspectTracker tracker(0.4, 1, 0.5);
    tracker.classify(gps(1), 0.0, false);
    tracker.classify(gps(2), 0.0, false);
    tracker.pruneMissing({gps(1)});
    // sat 2's baseline was dropped, so it re-seeds (not suspect) even with a
    // huge raw value that would have been flagged against its old baseline.
    EXPECT_FALSE(tracker.classify(gps(2), 1000.0, false));
}

// --- ReferenceHysteresis ------------------------------------------------

TEST(ReferenceHysteresisTest, EmptyCandidateListLeavesStateUntouched) {
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    EXPECT_FALSE(hysteresis.update({}, gps(1), 0.5, 3, 0.0, out_ref, switched));
    EXPECT_FALSE(hysteresis.hasReference());
    EXPECT_FALSE(switched);
}

TEST(ReferenceHysteresisTest, FirstCallBootstrapsToNaturalRefWithoutCountingAsSwitch) {
    ReferenceHysteresis hysteresis;
    const std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 0.7, true, false),
        makeCandidate(2, 0.5, true, false),
    };
    SatelliteId out_ref;
    bool switched = true;  // deliberately pre-set to a wrong value
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, 3, 0.0, out_ref, switched));
    EXPECT_EQ(out_ref.prn, 1);
    EXPECT_FALSE(switched);
    EXPECT_TRUE(hysteresis.hasReference());
    EXPECT_EQ(hysteresis.currentReference().prn, 1);
}

TEST(ReferenceHysteresisTest, DoesNotSwitchBeforeKConsecutiveSuspectEpochs) {
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 3;

    // Epoch 0: bootstrap onto sat 1 (highest elevation, not suspect yet).
    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 0.7, true, false),
        makeCandidate(2, 0.5, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
    ASSERT_EQ(out_ref.prn, 1);

    // Epochs 1..K-1: sat 1 goes suspect but hasn't hit the K-epoch streak
    // yet -- must stay on sat 1 every time.
    for (int epoch = 1; epoch < K; ++epoch) {
        candidates = {
            makeCandidate(1, 0.7, true, true),  // suspect
            makeCandidate(2, 0.5, true, false),
        };
        ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
        EXPECT_EQ(out_ref.prn, 1) << "epoch " << epoch;
        EXPECT_FALSE(switched) << "epoch " << epoch;
    }

    // Epoch K: the K-th consecutive suspect epoch for sat 1 -- must now
    // switch to the highest-elevation non-suspect candidate (sat 2).
    candidates = {
        makeCandidate(1, 0.7, true, true),
        makeCandidate(2, 0.5, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
    EXPECT_EQ(out_ref.prn, 2);
    EXPECT_TRUE(switched);
}

TEST(ReferenceHysteresisTest, SuspectStreakResetsOnAnyCleanEpoch) {
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 3;

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 0.7, true, false),
        makeCandidate(2, 0.5, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));

    // Two suspect epochs (not yet K), then one clean epoch resets the
    // streak, then two more suspect epochs -- still must not have hit K
    // CONSECUTIVE suspect epochs anywhere, so no switch.
    for (bool suspect : {true, true, false, true, true}) {
        candidates = {
            makeCandidate(1, 0.7, true, suspect),
            makeCandidate(2, 0.5, true, false),
        };
        ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
        EXPECT_EQ(out_ref.prn, 1);
        EXPECT_FALSE(switched);
    }
}

TEST(ReferenceHysteresisTest, NoSwitchWhenEveryCandidateIsSuspect) {
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 2;

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 0.7, true, false),
        makeCandidate(2, 0.5, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
    ASSERT_EQ(out_ref.prn, 1);

    // Drive sat 1's suspect streak past K, but every candidate (including
    // sat 2) is suspect this epoch -- there is nowhere clean to switch to,
    // so the reference must stay on sat 1.
    for (int epoch = 0; epoch < K + 2; ++epoch) {
        candidates = {
            makeCandidate(1, 0.7, true, true),
            makeCandidate(2, 0.5, true, true),
        };
        ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
        EXPECT_EQ(out_ref.prn, 1) << "epoch " << epoch;
        EXPECT_FALSE(switched) << "epoch " << epoch;
    }
}

TEST(ReferenceHysteresisTest, SwitchBackRequiresBothCleanStreakAndElevationMargin) {
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 2;
    const double margin_rad = 5.0 * M_PI / 180.0;

    // Bootstrap onto sat 2 directly (simulate having already switched away
    // from sat 1 previously) by making sat 2 the natural/highest-elevation
    // pick on the first call.
    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 0.30, true, false),
        makeCandidate(2, 0.35, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(2), 0.35, K, margin_rad, out_ref, switched));
    ASSERT_EQ(out_ref.prn, 2);

    // Now sat 1 rises well above sat 2 (clears the margin) and is clean,
    // but only for ONE epoch so far -- must not switch back yet.
    candidates = {
        makeCandidate(1, 0.35 + margin_rad + 0.05, true, false),
        makeCandidate(2, 0.35, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.35 + margin_rad + 0.05, K, margin_rad,
                                  out_ref, switched));
    EXPECT_EQ(out_ref.prn, 2);
    EXPECT_FALSE(switched);

    // Second consecutive clean epoch at the same elevated candidate hits
    // the K=2 streak -- switch back to sat 1 now.
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.35 + margin_rad + 0.05, K, margin_rad,
                                  out_ref, switched));
    EXPECT_EQ(out_ref.prn, 1);
    EXPECT_TRUE(switched);
}

TEST(ReferenceHysteresisTest, NoSwitchBackWhenElevationMarginNotCleared) {
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 1;
    const double margin_rad = 5.0 * M_PI / 180.0;

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 0.30, true, false),
        makeCandidate(2, 0.32, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(2), 0.32, K, margin_rad, out_ref, switched));
    ASSERT_EQ(out_ref.prn, 2);

    // sat 1 rises to become the natural (highest-elevation) pick and is
    // clean, but only ~1.1 degrees above the current reference -- far short
    // of the 5-degree margin -- so must not switch back even though K=1 (a
    // single clean epoch would otherwise be enough).
    candidates = {
        makeCandidate(1, 0.34, true, false),
        makeCandidate(2, 0.32, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.34, K, margin_rad, out_ref, switched));
    EXPECT_EQ(out_ref.prn, 2);
    EXPECT_FALSE(switched);
}

TEST(ReferenceHysteresisTest, PrefersDualFrequencyCandidateWhenSwitchingAway) {
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 1;

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 0.7, true, false),
        makeCandidate(2, 0.6, false, false),  // L1-only, higher elevation than sat 3
        makeCandidate(3, 0.5, true, false),   // dual-freq, lower elevation than sat 2
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
    ASSERT_EQ(out_ref.prn, 1);

    // sat 1 goes suspect; among the clean candidates, sat 2 has higher
    // elevation but is L1-only, sat 3 is dual-freq -- the switch-away rule
    // must prefer sat 3 (dual-freq preferred, matching
    // selectSystemReferenceSatellite's own tie-break).
    candidates = {
        makeCandidate(1, 0.7, true, true),
        makeCandidate(2, 0.6, false, false),
        makeCandidate(3, 0.5, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
    EXPECT_EQ(out_ref.prn, 3);
}

// --- Switch-away elevation-quality gate (rtk_cmc_reference.cpp refinement)

TEST(ReferenceHysteresisTest, SwitchAwayBlockedByExcessiveElevationDrop) {
    // Suspect reference at 70deg, only clean candidate at 55deg -- a 15deg
    // drop exceeds a 10deg max_elev_drop gate, so the switch-away must be
    // suppressed even though the K-epoch suspect streak is satisfied.
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 2;
    const double max_drop_rad = 10.0 * M_PI / 180.0;
    const double min_elev_rad = 0.0;  // floor disabled for this test

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 70.0 * M_PI / 180.0, true, false),
        makeCandidate(2, 55.0 * M_PI / 180.0, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 70.0 * M_PI / 180.0, K, 0.0, out_ref,
                                  switched, max_drop_rad, min_elev_rad));
    ASSERT_EQ(out_ref.prn, 1);

    for (int epoch = 0; epoch < K; ++epoch) {
        candidates = {
            makeCandidate(1, 70.0 * M_PI / 180.0, true, true),  // suspect
            makeCandidate(2, 55.0 * M_PI / 180.0, true, false),
        };
        ASSERT_TRUE(hysteresis.update(candidates, gps(1), 70.0 * M_PI / 180.0, K, 0.0, out_ref,
                                      switched, max_drop_rad, min_elev_rad));
    }
    // K-th consecutive suspect epoch reached: without the gate this would
    // switch to sat 2, but the 15deg drop blocks it -- stays on sat 1.
    EXPECT_EQ(out_ref.prn, 1);
    EXPECT_FALSE(switched);
}

TEST(ReferenceHysteresisTest, SwitchAwayBlockedByAbsoluteElevationFloor) {
    // Suspect reference at 32deg, only clean candidate at 25deg -- the drop
    // (7deg) is within a generous 10deg max-drop gate, but 25deg falls
    // below a 30deg absolute floor, so the switch-away must still be
    // suppressed.
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 1;
    const double max_drop_rad = 10.0 * M_PI / 180.0;
    const double min_elev_rad = 30.0 * M_PI / 180.0;

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 32.0 * M_PI / 180.0, true, false),
        makeCandidate(2, 25.0 * M_PI / 180.0, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 32.0 * M_PI / 180.0, K, 0.0, out_ref,
                                  switched, max_drop_rad, min_elev_rad));
    ASSERT_EQ(out_ref.prn, 1);

    candidates = {
        makeCandidate(1, 32.0 * M_PI / 180.0, true, true),  // suspect
        makeCandidate(2, 25.0 * M_PI / 180.0, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 32.0 * M_PI / 180.0, K, 0.0, out_ref,
                                  switched, max_drop_rad, min_elev_rad));
    EXPECT_EQ(out_ref.prn, 1);
    EXPECT_FALSE(switched);
}

TEST(ReferenceHysteresisTest, SwitchAwayAllowedWhenBothElevationGatesSatisfied) {
    // Suspect reference at 40deg, clean candidate at 33deg -- 7deg drop
    // (within the 10deg max-drop gate) and 33deg clears the 30deg floor, so
    // the switch-away must proceed normally.
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 1;
    const double max_drop_rad = 10.0 * M_PI / 180.0;
    const double min_elev_rad = 30.0 * M_PI / 180.0;

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 40.0 * M_PI / 180.0, true, false),
        makeCandidate(2, 33.0 * M_PI / 180.0, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 40.0 * M_PI / 180.0, K, 0.0, out_ref,
                                  switched, max_drop_rad, min_elev_rad));
    ASSERT_EQ(out_ref.prn, 1);

    candidates = {
        makeCandidate(1, 40.0 * M_PI / 180.0, true, true),  // suspect
        makeCandidate(2, 33.0 * M_PI / 180.0, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 40.0 * M_PI / 180.0, K, 0.0, out_ref,
                                  switched, max_drop_rad, min_elev_rad));
    EXPECT_EQ(out_ref.prn, 2);
    EXPECT_TRUE(switched);
}

TEST(ReferenceHysteresisTest, DefaultElevationGateParamsPreserveUngatedBehavior) {
    // Calling update() without the two new trailing arguments (as all the
    // pre-existing tests above do) must reproduce the pre-refinement
    // behavior exactly: even a huge elevation drop still switches away once
    // the K-epoch suspect streak is hit.
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 1;

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 80.0 * M_PI / 180.0, true, false),
        makeCandidate(2, 5.0 * M_PI / 180.0, true, false),  // 75deg drop, near-horizon
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 80.0 * M_PI / 180.0, K, 0.0, out_ref,
                                  switched));
    ASSERT_EQ(out_ref.prn, 1);

    candidates = {
        makeCandidate(1, 80.0 * M_PI / 180.0, true, true),  // suspect
        makeCandidate(2, 5.0 * M_PI / 180.0, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 80.0 * M_PI / 180.0, K, 0.0, out_ref,
                                  switched));
    EXPECT_EQ(out_ref.prn, 2);
    EXPECT_TRUE(switched);
}

TEST(ReferenceHysteresisTest, HardReacquisitionWhenCurrentReferenceDisappears) {
    ReferenceHysteresis hysteresis;
    SatelliteId out_ref;
    bool switched = false;
    const int K = 5;  // large K -- would never satisfy a suspect-driven switch this fast

    std::vector<ReferenceHysteresis::Candidate> candidates = {
        makeCandidate(1, 0.7, true, false),
        makeCandidate(2, 0.5, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(1), 0.7, K, 0.0, out_ref, switched));
    ASSERT_EQ(out_ref.prn, 1);

    // sat 1 drops out of the candidate set entirely (e.g. below the
    // elevation mask) -- must reacquire onto sat 2 immediately, bypassing
    // the K-epoch hysteresis entirely (it never went "suspect", it's just
    // gone).
    candidates = {
        makeCandidate(2, 0.5, true, false),
    };
    ASSERT_TRUE(hysteresis.update(candidates, gps(2), 0.5, K, 0.0, out_ref, switched));
    EXPECT_EQ(out_ref.prn, 2);
    EXPECT_TRUE(switched);
}

// --- RTKProcessor integration: knob off is default and a strict no-op --

std::string sourcePath(const std::string& relative_path) {
    return std::string(GNSSPP_SOURCE_DIR) + "/" + relative_path;
}

bool sourcePathExists(const std::string& relative_path) {
    return std::filesystem::exists(sourcePath(relative_path));
}

TEST(RTKCmcReferenceIntegrationTest, KnobOffByDefaultAndDiagnosticsStayZero) {
    RTKProcessor::RTKConfig default_config;
    EXPECT_FALSE(default_config.cmc_aware_reference_selection);
    EXPECT_DOUBLE_EQ(default_config.cmc_ref_level_m, 0.75);
    EXPECT_EQ(default_config.cmc_ref_switch_epochs, 3);
    EXPECT_DOUBLE_EQ(default_config.cmc_ref_return_min_elev_deg, 5.0);
    EXPECT_DOUBLE_EQ(default_config.cmc_ref_switch_max_elev_drop_deg, 10.0);
    EXPECT_DOUBLE_EQ(default_config.cmc_ref_switch_min_elev_deg, 30.0);

    std::string rover_path = "data/rover_kinematic.obs";
    std::string base_path = "data/base_kinematic.obs";
    std::string nav_path = "data/navigation_kinematic.nav";
    if (!sourcePathExists(rover_path) || !sourcePathExists(base_path) ||
        !sourcePathExists(nav_path)) {
        rover_path = "data/PPC-Dataset/tokyo/run1/rover.obs";
        base_path = "data/PPC-Dataset/tokyo/run1/base.obs";
        nav_path = "data/PPC-Dataset/tokyo/run1/base.nav";
    }
    if (!sourcePathExists(rover_path) || !sourcePathExists(base_path) ||
        !sourcePathExists(nav_path)) {
        GTEST_SKIP() << "no kinematic RTK test data available (neither data/*_kinematic.* "
                        "nor data/PPC-Dataset/tokyo/run1)";
    }

    io::RINEXReader rover_reader;
    io::RINEXReader base_reader;
    io::RINEXReader nav_reader;
    io::RINEXReader::RINEXHeader rover_header;
    io::RINEXReader::RINEXHeader base_header;
    NavigationData nav_data;
    ASSERT_TRUE(rover_reader.open(sourcePath(rover_path)));
    ASSERT_TRUE(rover_reader.readHeader(rover_header));
    ASSERT_TRUE(base_reader.open(sourcePath(base_path)));
    ASSERT_TRUE(base_reader.readHeader(base_header));
    ASSERT_TRUE(nav_reader.open(sourcePath(nav_path)));
    ASSERT_TRUE(nav_reader.readNavigationData(nav_data));
    ASSERT_GT(base_header.approximate_position.norm(), 1e6);

    RTKProcessor processor(default_config);
    processor.setBasePosition(base_header.approximate_position);

    ObservationData rover_obs;
    ObservationData base_obs;
    int processed = 0;
    while (processed < 10 && rover_reader.readObservationEpoch(rover_obs) &&
           base_reader.readObservationEpoch(base_obs)) {
        processor.processRTKEpoch(rover_obs, base_obs, nav_data);
        ++processed;
    }
    ASSERT_GT(processed, 0);

    // Knob is off for the whole run -- diagnostics must be untouched.
    const auto diag = processor.getCmcReferenceDiagnostics();
    EXPECT_EQ(diag.suspect_epoch_count, 0u);
    EXPECT_EQ(diag.switch_count, 0u);
}
