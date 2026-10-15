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
//
// The second half covers the base epoch's own span. The test predicts the
// single-difference phase change as rover rate * rover span - base rate * base
// span. With base epochs that do not advance with the rover epochs (an
// exact-time base epoch keeps its measurement time a millisecond before the
// rover epoch, an interpolated one takes the rover time, and the two alternate)
// the rover span alone mispredicted by about a cycle at a Doppler of 1 kHz and
// flagged a slip on every fast satellite. Every observation in those scenes is
// consistent with its own time tag, so no epoch contains a slip.

#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>

#include "synthetic_rtk_scene.hpp"

#include <cmath>
#include <functional>
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

namespace {

// L2 and L5 share the geometry of L1: the same range and range rate, in cycles and Hz of the
// longer wavelength.
const double kL2WavelengthM = constants::GPS_L2_WAVELENGTH;
const double kL5WavelengthM = constants::GPS_L5_WAVELENGTH;

void addFrequencyFromL1(ObservationData& data, SignalType signal, double wavelength_m) {
    const std::vector<Observation> l1 = data.observations;
    const double ratio = kWavelengthM / wavelength_m;
    for (Observation obs : l1) {
        obs.signal = signal;
        obs.carrier_phase *= ratio;
        obs.doppler *= ratio;
        data.addObservation(obs);
    }
}

struct BaseSpanScenario {
    int epochs = 24;
    // Base measurement time minus the rover time, per epoch (s).
    std::function<double(int)> base_offset_s = [](int) { return 0.0; };
    bool with_l2 = false;
    bool with_l5 = false;
    double rover_drift_mps = 30.0;
    // A persistent jump of the rover phase of `slip_prn` from `slip_epoch` on, per frequency.
    int slip_epoch = -1;
    uint8_t slip_prn = 0;
    double slip_l1_cycles = 0.0;
    double slip_l2_cycles = 0.0;
    double slip_l5_cycles = 0.0;
    // Lower the Doppler slip threshold to its smallest effective value (0.10 m)
    // so that a one-cycle L1 slip (0.19 m) is detectable.
    bool smallest_threshold = false;
};

// Alternating base epochs of gnss live: one measured 1 ms before the rover epoch, one at
// the rover time.
double alternatingBaseOffsetS(int epoch) { return epoch % 2 == 0 ? -0.001 : 0.0; }

struct BaseSpanRun {
    std::vector<int> l1_slips;
    std::vector<int> l2_slips;
    std::vector<int> l5_slips;
    std::vector<bool> valid;
    std::vector<ObservationData> rover;
    std::vector<ObservationData> base;
};

ObservationData roverEpoch(const NavigationData& nav, double t, const std::vector<uint8_t>& prns,
                           const BaseSpanScenario& scenario, int epoch) {
    auto rover = observations(nav, epochTime(t), kRover, 30.0 + scenario.rover_drift_mps * t,
                              prns, 0.0, /*with_doppler=*/true);
    rover.receiver_position = kRover;
    for (auto& obs : rover.observations) {
        obs.carrier_phase += scenario.rover_drift_mps * t / kWavelengthM;
        obs.doppler -= scenario.rover_drift_mps / kWavelengthM;
    }
    if (scenario.with_l2) addFrequencyFromL1(rover, SignalType::GPS_L2C, kL2WavelengthM);
    if (scenario.with_l5) addFrequencyFromL1(rover, SignalType::GPS_L5, kL5WavelengthM);
    if (scenario.slip_epoch >= 0 && epoch >= scenario.slip_epoch) {
        for (auto& obs : rover.observations) {
            if (obs.satellite.prn != scenario.slip_prn) continue;
            if (obs.signal == SignalType::GPS_L1CA) obs.carrier_phase += scenario.slip_l1_cycles;
            if (obs.signal == SignalType::GPS_L2C) obs.carrier_phase += scenario.slip_l2_cycles;
            if (obs.signal == SignalType::GPS_L5) obs.carrier_phase += scenario.slip_l5_cycles;
        }
    }
    return rover;
}

ObservationData baseEpoch(const NavigationData& nav, double measured_t,
                          const std::vector<uint8_t>& prns, const BaseSpanScenario& scenario) {
    auto base = observations(nav, epochTime(measured_t), kBase, 10.0, prns, 500.0,
                             /*with_doppler=*/true);
    if (scenario.with_l2) addFrequencyFromL1(base, SignalType::GPS_L2C, kL2WavelengthM);
    if (scenario.with_l5) addFrequencyFromL1(base, SignalType::GPS_L5, kL5WavelengthM);
    return base;
}

BaseSpanRun runBaseSpanScenario(const BaseSpanScenario& scenario) {
    const auto nav = constellation();
    const auto prns = visible(nav, epochTime(0.0), kRover, 15.0);
    EXPECT_GE(prns.size(), 6U);

    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.elevation_mask = 5.0 * M_PI / 180.0;
    // Keep the scene in the FLOAT branch.
    config.ratio_threshold = 1.0e9;
    config.ambiguity_ratio_threshold = 1.0e9;
    config.enable_l5 = scenario.with_l5;
    if (scenario.smallest_threshold) {
        config.use_dynamic_slip_threshold_floor = false;
        config.doppler_slip_threshold = 0.10;
    }
    auto processor = std::make_unique<RTKProcessor>(config);
    ProcessorConfig processor_config;
    processor_config.elevation_mask = 5.0;
    EXPECT_TRUE(processor->initialize(processor_config));
    processor->setBasePosition(kBase);

    BaseSpanRun run;
    for (int epoch = 0; epoch < scenario.epochs; ++epoch) {
        const double t = kSpacingS * epoch;
        auto rover = roverEpoch(nav, t, prns, scenario, epoch);
        auto base = baseEpoch(nav, t + scenario.base_offset_s(epoch), prns, scenario);
        const auto solution = processor->processRTKEpoch(rover, base, nav);
        const auto& telemetry = processor->getLastDebugTelemetry();
        run.l1_slips.push_back(telemetry.doppler_slip_l1_count);
        run.l2_slips.push_back(telemetry.doppler_slip_l2_count);
        run.l5_slips.push_back(telemetry.doppler_slip_l5_count);
        run.valid.push_back(solution.isValid());
        run.rover.push_back(std::move(rover));
        run.base.push_back(std::move(base));
    }
    return run;
}

int total(const std::vector<int>& counts) {
    int sum = 0;
    for (const int count : counts) sum += count;
    return sum;
}

// What the rover-span-only test does with two consecutive epochs of the run: the
// satellites it flags although nothing slipped.
int legacyFalseSlipsBetween(const BaseSpanRun& run, int epoch, double threshold_m) {
    const double rover_span_s = run.rover[epoch].time - run.rover[epoch - 1].time;
    int flagged = 0;
    for (const auto& rover_now : run.rover[epoch].observations) {
        if (rover_now.signal != SignalType::GPS_L1CA) continue;
        const Observation* rover_before = nullptr;
        const Observation* base_now = nullptr;
        const Observation* base_before = nullptr;
        for (const auto& o : run.rover[epoch - 1].observations)
            if (o.satellite == rover_now.satellite && o.signal == SignalType::GPS_L1CA)
                rover_before = &o;
        for (const auto& o : run.base[epoch].observations)
            if (o.satellite == rover_now.satellite && o.signal == SignalType::GPS_L1CA)
                base_now = &o;
        for (const auto& o : run.base[epoch - 1].observations)
            if (o.satellite == rover_now.satellite && o.signal == SignalType::GPS_L1CA)
                base_before = &o;
        if (!rover_before || !base_now || !base_before) continue;
        const double sd_now_m = (rover_now.carrier_phase - base_now->carrier_phase) * kWavelengthM;
        const double sd_before_m =
            (rover_before->carrier_phase - base_before->carrier_phase) * kWavelengthM;
        const double rate_mps = rtk_slip_detection::singleDifferenceRangeRateMps(
            rover_now.doppler, base_now->doppler, kWavelengthM);
        if (rtk_slip_detection::detectDopplerSlip(sd_before_m, sd_now_m, rate_mps, rover_span_s,
                                                  threshold_m)) {
            ++flagged;
        }
    }
    return flagged;
}

}  // namespace

TEST(RtkDopplerSlipBaseSpanTest, AlternatingBaseEpochTimesDoNotCreateDopplerSlips) {
    BaseSpanScenario scenario;
    scenario.base_offset_s = alternatingBaseOffsetS;
    scenario.with_l2 = true;
    scenario.with_l5 = true;
    const BaseSpanRun run = runBaseSpanScenario(scenario);

    ASSERT_EQ(run.l1_slips.size(), 24U);
    for (int epoch = 0; epoch < scenario.epochs; ++epoch) {
        EXPECT_TRUE(run.valid[epoch]) << "epoch " << epoch;
        EXPECT_EQ(run.l1_slips[epoch], 0) << "epoch " << epoch;
        EXPECT_EQ(run.l2_slips[epoch], 0) << "epoch " << epoch;
        EXPECT_EQ(run.l5_slips[epoch], 0) << "epoch " << epoch;
    }
    // The base really advances 0.201 s and 0.199 s against the rover's 0.2 s, and the
    // rover-span-only test flags slips on this very data (it would reset those ambiguities
    // every epoch).
    EXPECT_NEAR(run.base[1].time - run.base[0].time, 0.201, 1.0e-9);
    EXPECT_NEAR(run.base[2].time - run.base[1].time, 0.199, 1.0e-9);
    EXPECT_NEAR(run.rover[1].time - run.rover[0].time, 0.2, 1.0e-9);
    int legacy_false_slips = 0;
    for (int epoch = 1; epoch < scenario.epochs; ++epoch) {
        legacy_false_slips += legacyFalseSlipsBetween(run, epoch, 0.20);
    }
    EXPECT_GT(legacy_false_slips, 40);
}

TEST(RtkDopplerSlipBaseSpanTest, TimeMatchedBaseEpochsStayCleanOnAllFrequencies) {
    BaseSpanScenario scenario;  // base epochs at the rover time
    scenario.with_l2 = true;
    scenario.with_l5 = true;
    const BaseSpanRun run = runBaseSpanScenario(scenario);
    for (int epoch = 0; epoch < scenario.epochs; ++epoch) {
        EXPECT_TRUE(run.valid[epoch]) << "epoch " << epoch;
        EXPECT_EQ(run.l1_slips[epoch], 0) << "epoch " << epoch;
        EXPECT_EQ(run.l2_slips[epoch], 0) << "epoch " << epoch;
        EXPECT_EQ(run.l5_slips[epoch], 0) << "epoch " << epoch;
    }
    for (int epoch = 1; epoch < scenario.epochs; ++epoch) {
        EXPECT_EQ(legacyFalseSlipsBetween(run, epoch, 0.20), 0) << "epoch " << epoch;
    }
}

TEST(RtkDopplerSlipBaseSpanTest, BaseEpochsTenMillisecondsEarlyEveryThirdEpochAreHandledToo) {
    // A larger offset with a different alternation: every third base epoch is early.
    BaseSpanScenario scenario;
    scenario.with_l2 = true;
    scenario.with_l5 = true;
    scenario.epochs = 30;
    scenario.base_offset_s = [](int epoch) { return epoch % 3 == 0 ? -0.010 : 0.0; };
    const BaseSpanRun run = runBaseSpanScenario(scenario);
    for (int epoch = 0; epoch < scenario.epochs; ++epoch) {
        EXPECT_EQ(run.l1_slips[epoch], 0) << "epoch " << epoch;
        EXPECT_EQ(run.l2_slips[epoch], 0) << "epoch " << epoch;
        EXPECT_EQ(run.l5_slips[epoch], 0) << "epoch " << epoch;
    }
}

TEST(RtkDopplerSlipBaseSpanTest, BaseEpochsAfterTheRoverEpochAreHandledToo) {
    // The sign of the offset does not matter: a base epoch measured 1 ms after the rover epoch,
    // alternating with one at the rover time.
    BaseSpanScenario scenario;
    scenario.with_l2 = true;
    scenario.with_l5 = true;
    scenario.base_offset_s = [](int epoch) { return epoch % 2 == 0 ? 0.001 : 0.0; };
    const BaseSpanRun run = runBaseSpanScenario(scenario);
    EXPECT_NEAR(run.base[1].time - run.base[0].time, 0.199, 1.0e-9);
    for (int epoch = 0; epoch < scenario.epochs; ++epoch) {
        EXPECT_EQ(run.l1_slips[epoch], 0) << "epoch " << epoch;
        EXPECT_EQ(run.l2_slips[epoch], 0) << "epoch " << epoch;
        EXPECT_EQ(run.l5_slips[epoch], 0) << "epoch " << epoch;
    }
}

TEST(RtkDopplerSlipBaseSpanTest, RealSlipIsStillDetectedWithAlternatingBaseEpochs) {
    // One cycle on one satellite and frequency at a time (L1 0.19 m, L2 0.24 m, L5 0.25 m),
    // at the smallest threshold the detector applies.
    BaseSpanScenario clean;
    clean.base_offset_s = alternatingBaseOffsetS;
    clean.with_l2 = true;
    clean.with_l5 = true;
    clean.smallest_threshold = true;
    const BaseSpanRun reference = runBaseSpanScenario(clean);
    ASSERT_GE(reference.rover[0].observations.size(), 12U);
    // The observations list every satellite on L1 first; pick three of them.
    const uint8_t l1_prn = reference.rover[0].observations.at(0).satellite.prn;
    const uint8_t l2_prn = reference.rover[0].observations.at(3).satellite.prn;
    const uint8_t l5_prn = reference.rover[0].observations.at(5).satellite.prn;
    ASSERT_NE(l1_prn, l2_prn);
    ASSERT_NE(l1_prn, l5_prn);
    ASSERT_NE(l2_prn, l5_prn);

    // A run without any slip is clean at this threshold too.
    EXPECT_EQ(total(reference.l1_slips), 0);
    EXPECT_EQ(total(reference.l2_slips), 0);
    EXPECT_EQ(total(reference.l5_slips), 0);

    struct Case {
        const char* name;
        uint8_t prn;
        int epoch;
        double l1, l2, l5;
    };
    const Case cases[] = {
        {"L1", l1_prn, 8, 1.0, 0.0, 0.0},
        {"L2", l2_prn, 11, 0.0, -1.0, 0.0},
        {"L5", l5_prn, 14, 0.0, 0.0, 1.0},
    };
    for (const Case& c : cases) {
        BaseSpanScenario slipped = clean;
        slipped.slip_epoch = c.epoch;
        slipped.slip_prn = c.prn;
        slipped.slip_l1_cycles = c.l1;
        slipped.slip_l2_cycles = c.l2;
        slipped.slip_l5_cycles = c.l5;
        const BaseSpanRun run = runBaseSpanScenario(slipped);
        for (int epoch = 0; epoch < slipped.epochs; ++epoch) {
            const bool at_slip = epoch == c.epoch;
            EXPECT_EQ(run.l1_slips[epoch], at_slip && c.l1 != 0.0 ? 1 : 0)
                << c.name << " slip, epoch " << epoch;
            EXPECT_EQ(run.l2_slips[epoch], at_slip && c.l2 != 0.0 ? 1 : 0)
                << c.name << " slip, epoch " << epoch;
            EXPECT_EQ(run.l5_slips[epoch], at_slip && c.l5 != 0.0 ? 1 : 0)
                << c.name << " slip, epoch " << epoch;
        }
    }
}
