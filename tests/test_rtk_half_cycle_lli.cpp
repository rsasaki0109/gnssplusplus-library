// RTKConfig::use_half_cycle_lli (RINEX/receiver LLI bit1 = half-cycle
// ambiguity unresolved), modelled on RTKLIB demo5 rtkpos.c:
//   * detslp_ll: a clear->set or set->clear transition of the bit on the
//     rover or the base observation of one satellite/frequency is a slip for
//     that satellite/frequency only,
//   * ddres: +0.01 m^2 phase variance while the bit is set,
//   * resamb_LAMBDA: a satellite whose LLI has the bit set is not used for
//     ambiguity resolution.
// Synthetic noise-free L1 observations only; public API only.

#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>

#include "synthetic_rtk_scene.hpp"

#include <functional>
#include <memory>
#include <vector>

using namespace libgnss;
using namespace synthetic_rtk_scene;

namespace {

const Vector3d kBase(6378137.0, 0.0, 0.0);
const Vector3d kRover = kBase + Vector3d(40.0, 30.0, 10.0);
constexpr int kHalf = 0x02;
constexpr int kSlip = 0x01;

struct EpochRecord {
    int lli_slips = 0;
    int resets = 0;
    int input_pairs = 0;
    bool float_ok = false;
    double cov_trace = 0.0;
    Vector3d float_position = Vector3d::Zero();
};

// lli(epoch, prn, is_rover) -> LLI byte for that observation.
using LliFn = std::function<int(int, int, bool)>;

std::vector<EpochRecord> run(bool half_cycle_option, int epochs, const LliFn& lli,
                             int* flagged_prn_out = nullptr) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    EXPECT_GE(prns.size(), 6U);
    if (flagged_prn_out != nullptr) *flagged_prn_out = prns.empty() ? 0 : prns.front();

    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.elevation_mask = 5.0 * M_PI / 180.0;
    config.use_half_cycle_lli = half_cycle_option;
    auto processor = std::make_unique<RTKProcessor>(config);
    ProcessorConfig processor_config;
    processor_config.elevation_mask = 5.0;
    EXPECT_TRUE(processor->initialize(processor_config));
    processor->setBasePosition(kBase);

    std::vector<EpochRecord> records;
    for (int epoch = 0; epoch < epochs; ++epoch) {
        const double t = 0.2 * epoch;
        auto rover = observations(nav, epochTime(t), kRover, 30.0, prns, 0.0);
        auto base = observations(nav, epochTime(t), kBase, 10.0, prns, 500.0);
        rover.receiver_position = kRover;
        for (auto& obs : rover.observations) {
            obs.lli = static_cast<uint8_t>(lli(epoch, obs.satellite.prn, true));
            obs.loss_of_lock = (obs.lli & kSlip) != 0;
        }
        for (auto& obs : base.observations) {
            obs.lli = static_cast<uint8_t>(lli(epoch, obs.satellite.prn, false));
            obs.loss_of_lock = (obs.lli & kSlip) != 0;
        }
        processor->processRTKEpoch(rover, base, nav);
        const auto& telemetry = processor->getLastDebugTelemetry();
        EpochRecord record;
        record.lli_slips = telemetry.lli_slip_l1_count;
        record.resets = telemetry.ambiguity_reset_l1_count;
        record.input_pairs = telemetry.input_pair_count;
        Matrix3d covariance;
        record.float_ok =
            processor->getFloatPosteriorPosition(record.float_position, covariance);
        record.cov_trace = covariance.trace();
        records.push_back(record);
    }
    return records;
}

LliFn none() {
    return [](int, int, bool) { return 0; };
}

// `prn` carries `value` on the given receiver for epochs in [from, to).
LliFn flag(int prn, bool on_rover, int from, int to, int value = kHalf) {
    return [=](int epoch, int p, bool is_rover) {
        return (p == prn && is_rover == on_rover && epoch >= from && epoch < to) ? value : 0;
    };
}

int firstPrn() {
    const auto nav = constellation();
    return visible(nav, epochTime(0.0), kRover, 15.0).front();
}

}  // namespace

TEST(RtkHalfCycleLliTest, DefaultMatchesDemo5AndIsOn) {
    // Default ON (RTKLIB demo5 behaviour); no-op on data without LLI bit1.
    const RTKProcessor::RTKConfig config;
    EXPECT_TRUE(config.use_half_cycle_lli);
    EXPECT_DOUBLE_EQ(config.half_cycle_phase_variance_m2, 0.01);
}

TEST(RtkHalfCycleLliTest, ClearToSetTransitionIsASlipForThatSatelliteOnly) {
    const int prn = firstPrn();
    const auto records = run(true, 12, flag(prn, /*on_rover=*/true, 8, 100));
    for (int epoch = 0; epoch < 12; ++epoch) {
        if (epoch == 8) {
            EXPECT_EQ(records[epoch].lli_slips, 1) << epoch;
            EXPECT_EQ(records[epoch].resets, 1) << epoch;
        } else {
            EXPECT_EQ(records[epoch].lli_slips, 0) << epoch;
            EXPECT_EQ(records[epoch].resets, 0) << epoch;
        }
    }
}

TEST(RtkHalfCycleLliTest, SetToClearTransitionIsASlipForThatSatelliteOnly) {
    const int prn = firstPrn();
    // Set from the very first epoch (no history -> no slip), cleared at 8.
    const auto records = run(true, 12, flag(prn, /*on_rover=*/true, 0, 8));
    for (int epoch = 0; epoch < 12; ++epoch) {
        if (epoch == 8) {
            EXPECT_EQ(records[epoch].lli_slips, 1) << epoch;
            EXPECT_EQ(records[epoch].resets, 1) << epoch;
        } else {
            EXPECT_EQ(records[epoch].lli_slips, 0) << epoch;
            EXPECT_EQ(records[epoch].resets, 0) << epoch;
        }
    }
}

TEST(RtkHalfCycleLliTest, SteadySetFlagIsNotASlip) {
    const int prn = firstPrn();
    const auto records = run(true, 14, flag(prn, /*on_rover=*/true, 0, 100));
    for (int epoch = 0; epoch < 14; ++epoch) {
        EXPECT_EQ(records[epoch].lli_slips, 0) << epoch;
        EXPECT_EQ(records[epoch].resets, 0) << epoch;
    }
}

TEST(RtkHalfCycleLliTest, BaseTransitionsAreSlipsToo) {
    const int prn = firstPrn();
    const auto records = run(true, 14, flag(prn, /*on_rover=*/false, 6, 10));
    for (int epoch = 0; epoch < 14; ++epoch) {
        const int expected = (epoch == 6 || epoch == 10) ? 1 : 0;
        EXPECT_EQ(records[epoch].lli_slips, expected) << epoch;
        EXPECT_EQ(records[epoch].resets, expected) << epoch;
    }
}

TEST(RtkHalfCycleLliTest, OnlyTheTransitionedSatelliteIsResetWhenSeveralAreFlagged) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    ASSERT_GE(prns.size(), 3U);
    const int steady = prns[0];
    const int toggled = prns[1];
    const LliFn lli = [=](int epoch, int p, bool is_rover) {
        if (!is_rover) return 0;
        if (p == steady) return kHalf;                      // set the whole time
        if (p == toggled) return epoch >= 7 ? kHalf : 0;    // clear -> set at 7
        return 0;
    };
    const auto records = run(true, 10, lli);
    EXPECT_EQ(records[7].lli_slips, 1);
    EXPECT_EQ(records[7].resets, 1);
    for (int epoch : {1, 2, 3, 4, 5, 6, 8, 9}) {
        EXPECT_EQ(records[epoch].lli_slips, 0) << epoch;
    }
}

TEST(RtkHalfCycleLliTest, OptionOffIgnoresBit1AndIsBitIdenticalToNoFlags) {
    const int prn = firstPrn();
    // Transitions on both receivers plus a steady flag on another epoch range.
    const LliFn lli = [=](int epoch, int p, bool is_rover) {
        if (p != prn) return 0;
        if (is_rover) return (epoch >= 4 && epoch < 9) ? kHalf : 0;
        return (epoch >= 6 && epoch < 11) ? kHalf : 0;
    };
    const auto flagged = run(false, 14, lli);
    const auto clean = run(false, 14, none());
    ASSERT_EQ(flagged.size(), clean.size());
    for (size_t i = 0; i < clean.size(); ++i) {
        EXPECT_EQ(flagged[i].lli_slips, 0) << i;
        EXPECT_EQ(flagged[i].resets, 0) << i;
        EXPECT_EQ(flagged[i].input_pairs, clean[i].input_pairs) << i;
        EXPECT_EQ(flagged[i].float_ok, clean[i].float_ok) << i;
        EXPECT_EQ(flagged[i].cov_trace, clean[i].cov_trace) << i;  // bit-identical
        EXPECT_EQ(flagged[i].float_position, clean[i].float_position) << i;
    }
}

TEST(RtkHalfCycleLliTest, OptionOnWithoutAnyBit1IsBitIdenticalToOptionOff) {
    // bit0 slips still behave as before, bit1 never present.
    const int prn = firstPrn();
    const LliFn lli = flag(prn, true, 7, 8, kSlip);
    const auto on = run(true, 12, lli);
    const auto off = run(false, 12, lli);
    for (size_t i = 0; i < on.size(); ++i) {
        EXPECT_EQ(on[i].lli_slips, off[i].lli_slips) << i;
        EXPECT_EQ(on[i].resets, off[i].resets) << i;
        EXPECT_EQ(on[i].input_pairs, off[i].input_pairs) << i;
        EXPECT_EQ(on[i].cov_trace, off[i].cov_trace) << i;
        EXPECT_EQ(on[i].float_position, off[i].float_position) << i;
    }
    EXPECT_EQ(on[7].lli_slips, 1);
}

TEST(RtkHalfCycleLliTest, PhaseVarianceIsInflatedWhileTheFlagIsSet) {
    const int prn = firstPrn();
    // Steady flag on the rover from epoch 0 (no slip) vs no flag at all.
    const auto flagged = run(true, 10, flag(prn, true, 0, 100));
    const auto clean = run(true, 10, none());
    // The flagged satellite's phase rows are down-weighted, so the float
    // position covariance after identical geometry is larger.
    EXPECT_GT(flagged.back().cov_trace, clean.back().cov_trace);
    // Flag on the base inflates in the same way.
    const auto flagged_base = run(true, 10, flag(prn, false, 0, 100));
    EXPECT_GT(flagged_base.back().cov_trace, clean.back().cov_trace);
    // With the option off the same flag changes nothing (bit-identical).
    const auto off = run(false, 10, flag(prn, true, 0, 100));
    const auto off_clean = run(false, 10, none());
    EXPECT_EQ(off.back().cov_trace, off_clean.back().cov_trace);
    // Larger inflation => larger covariance (the knob is live).
    // (Checked through the config struct default only; value is 0.01 m^2.)
}

TEST(RtkHalfCycleLliTest, FlaggedSatelliteIsExcludedFromAmbiguityResolutionCandidates) {
    const int prn = firstPrn();
    const auto flagged = run(true, 14, flag(prn, true, 0, 100));
    const auto clean = run(true, 14, none());
    const auto off = run(false, 14, flag(prn, true, 0, 100));
    // Find an epoch where AR candidates are formed in the clean run.
    int checked = 0;
    for (size_t i = 0; i < clean.size(); ++i) {
        if (clean[i].input_pairs <= 1) continue;
        ++checked;
        EXPECT_EQ(flagged[i].input_pairs, clean[i].input_pairs - 1) << i;
        EXPECT_EQ(off[i].input_pairs, clean[i].input_pairs) << i;
    }
    EXPECT_GT(checked, 0) << "synthetic scene never formed AR candidates";
}
