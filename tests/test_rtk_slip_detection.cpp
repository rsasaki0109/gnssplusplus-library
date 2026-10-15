#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_slip_detection.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace {

using libgnss::rtk_slip_detection::detectDopplerSlip;
using libgnss::rtk_slip_detection::detectDopplerSlipWithBaseSpan;
using libgnss::rtk_slip_detection::detectCodeSlip;
using libgnss::rtk_slip_detection::dopplerSpansDiffer;
using libgnss::rtk_slip_detection::predictedSingleDifferencePhaseChangeM;
using libgnss::rtk_slip_detection::singleDifferenceCodeMinusPhaseM;
using libgnss::rtk_slip_detection::singleDifferenceRangeRateMps;

TEST(RTKSlipDetectionTest, SingleDifferenceRangeRateMatchesDopplerConvention) {
    const double wavelength_m = 0.190293672798365;
    const double rover_doppler_hz = -1000.0;
    const double base_doppler_hz = -998.0;
    const double rate_mps =
        singleDifferenceRangeRateMps(rover_doppler_hz, base_doppler_hz, wavelength_m);
    EXPECT_NEAR(rate_mps, 2.0 * wavelength_m, 1e-12);
}

TEST(RTKSlipDetectionTest, DopplerConsistentPhaseChangeDoesNotTriggerSlip) {
    const double previous_sd_phase_m = 10.0;
    const double current_sd_phase_m = 10.15;
    const double sd_range_rate_mps = 0.15;
    EXPECT_FALSE(
        detectDopplerSlip(previous_sd_phase_m, current_sd_phase_m, sd_range_rate_mps, 1.0, 0.2));
}

TEST(RTKSlipDetectionTest, LargeDopplerResidualTriggersSlip) {
    const double previous_sd_phase_m = 10.0;
    const double current_sd_phase_m = 10.40;
    const double sd_range_rate_mps = 0.05;
    EXPECT_TRUE(
        detectDopplerSlip(previous_sd_phase_m, current_sd_phase_m, sd_range_rate_mps, 1.0, 0.2));
}

TEST(RTKSlipDetectionTest, SingleDifferenceCodeMinusPhaseCombinesCodeAndCarrier) {
    const double wavelength_m = 0.190293672798365;
    const double rover_code_m = 20200000.40;
    const double base_code_m = 20199998.10;
    const double rover_phase_cycles = 106151720.0;
    const double base_phase_cycles = 106151708.0;
    const double sd_code_minus_phase_m =
        singleDifferenceCodeMinusPhaseM(
            rover_code_m, base_code_m, rover_phase_cycles, base_phase_cycles, wavelength_m);
    const double expected =
        (rover_code_m - base_code_m) -
        ((rover_phase_cycles - base_phase_cycles) * wavelength_m);
    EXPECT_NEAR(sd_code_minus_phase_m, expected, 1e-12);
}

TEST(RTKSlipDetectionTest, SmallCodeMinusPhaseChangeDoesNotTriggerSlip) {
    EXPECT_FALSE(detectCodeSlip(1.50, 3.90, 3.0));
}

TEST(RTKSlipDetectionTest, LargeCodeMinusPhaseJumpTriggersSlip) {
    EXPECT_TRUE(detectCodeSlip(1.50, 6.75, 3.0));
}

TEST(RTKSlipDetectionTest, InvalidInputsDoNotTriggerSlip) {
    EXPECT_FALSE(detectDopplerSlip(1.0, 2.0, 0.0, 0.0, 0.2));
    EXPECT_FALSE(detectDopplerSlip(1.0, 2.0, 0.0, 1.0, -0.1));
    EXPECT_FALSE(detectCodeSlip(1.0, 2.0, -0.1));
}

// ---------------------------------------------------------------------------
// Doppler slip test with the rover and the base advancing over their own spans
// (detectDopplerSlipWithBaseSpan / predictedSingleDifferencePhaseChangeM).
// ---------------------------------------------------------------------------

constexpr double kL1WavelengthM = 0.190293672798365;
constexpr double kL2WavelengthM = 0.244210213424568;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// What the receivers measure between two detector calls when nothing slips:
// each phase advances by its own range rate (-wavelength * Doppler) over its
// own span, so the single-difference phase changes by
// rover_range_rate * rover_span - base_range_rate * base_span.
double physicalSdPhaseChangeM(double rover_doppler_hz, double base_doppler_hz,
                              double wavelength_m, double rover_span_s, double base_span_s) {
    return -rover_doppler_hz * wavelength_m * rover_span_s -
           (-base_doppler_hz * wavelength_m * base_span_s);
}

// The expression RTKProcessor used before the base span was introduced.
double legacyPredictionM(double rover_doppler_hz, double base_doppler_hz,
                         double wavelength_m, double rover_span_s) {
    return singleDifferenceRangeRateMps(rover_doppler_hz, base_doppler_hz, wavelength_m) *
           rover_span_s;
}

bool legacyDecision(double previous_sd_phase_m, double current_sd_phase_m,
                    double rover_doppler_hz, double base_doppler_hz, double wavelength_m,
                    double rover_span_s, double threshold_m) {
    return detectDopplerSlip(previous_sd_phase_m, current_sd_phase_m,
                             singleDifferenceRangeRateMps(
                                 rover_doppler_hz, base_doppler_hz, wavelength_m),
                             rover_span_s, threshold_m);
}

struct DopplerPair {
    double rover_hz;
    double base_hz;
};

// Rover/base Doppler (Hz) of one satellite: mostly a few Hz apart (receiver
// clock drift and baseline), from slow to fast satellites, both signs, plus one
// far-apart pair. Values are not round on purpose so that different
// floating-point expressions differ.
const std::vector<DopplerPair>& dopplerPairs() {
    static const std::vector<DopplerPair> pairs = {
        {1003.2718, 1000.0391}, {-2487.6149, -2490.1203}, {617.3318, 614.9987},
        {3921.0467, 3917.7721}, {-1234.5679, -1237.9013}, {-987.6543, 12.3457},
        {0.7321, -2.4129},      {4210.3333, 4205.1111},
    };
    return pairs;
}

TEST(RTKSlipDetectionTest, SpansWithinOneMicrosecondAreTheSameInterval) {
    EXPECT_FALSE(dopplerSpansDiffer(0.2, 0.2));
    EXPECT_FALSE(dopplerSpansDiffer(0.2, 0.2 + 0.5e-6));
    EXPECT_FALSE(dopplerSpansDiffer(0.2, 0.2 - 0.9e-6));
    // Exactly representable offsets on both sides of the 1 us tolerance.
    EXPECT_FALSE(dopplerSpansDiffer(0.5, 0.5 + std::ldexp(1.0, -20)));  // 0.95 us
    EXPECT_TRUE(dopplerSpansDiffer(0.5, 0.5 + std::ldexp(1.0, -19)));   // 1.91 us
    EXPECT_TRUE(dopplerSpansDiffer(0.2, 0.2 + 1.1e-6));
    EXPECT_TRUE(dopplerSpansDiffer(0.2, 0.201));
    EXPECT_TRUE(dopplerSpansDiffer(0.2, 0.199));
    EXPECT_TRUE(dopplerSpansDiffer(0.2, 1.0));
}

TEST(RTKSlipDetectionTest, UnknownBaseSpanIsNeverDifferent) {
    EXPECT_FALSE(dopplerSpansDiffer(0.2, kNaN));
    EXPECT_FALSE(dopplerSpansDiffer(0.2, kInf));
    EXPECT_FALSE(dopplerSpansDiffer(0.2, 0.0));
    EXPECT_FALSE(dopplerSpansDiffer(0.2, -0.2));
    EXPECT_FALSE(dopplerSpansDiffer(kNaN, 0.2));
}

// Matching spans: the prediction is the legacy expression, bit for bit.
TEST(RTKSlipDetectionTest, MatchingSpansPredictTheLegacyValueExactly) {
    const double rover_span_s = 0.2;
    const std::vector<double> matching_base_spans = {
        rover_span_s,
        rover_span_s + 0.5e-6,
        rover_span_s - 0.9e-6,
        rover_span_s + 0.95e-6,
        rover_span_s - 0.3e-6,
    };
    int compared = 0;
    for (const double wavelength_m : {kL1WavelengthM, kL2WavelengthM}) {
        for (const auto& pair : dopplerPairs()) {
            for (const double base_span_s : matching_base_spans) {
                ASSERT_FALSE(dopplerSpansDiffer(rover_span_s, base_span_s));
                const double expected =
                    legacyPredictionM(pair.rover_hz, pair.base_hz, wavelength_m, rover_span_s);
                EXPECT_EQ(predictedSingleDifferencePhaseChangeM(
                              pair.rover_hz, pair.base_hz, wavelength_m, rover_span_s,
                              base_span_s),
                          expected)
                    << "rover " << pair.rover_hz << " Hz, base " << pair.base_hz
                    << " Hz, base span " << base_span_s;
                ++compared;
            }
        }
    }
    EXPECT_EQ(compared, 80);
}

// Spans that are not usable (first call, not finite, not positive): the prediction is
// the legacy expression with the rover span, bit for bit.
TEST(RTKSlipDetectionTest, UnknownBaseSpanPredictsTheLegacyValueExactly) {
    const double rover_span_s = 0.2;
    for (const double base_span_s : {kNaN, kInf, -kInf, 0.0, -0.2, -1.0e-9}) {
        for (const auto& pair : dopplerPairs()) {
            EXPECT_EQ(predictedSingleDifferencePhaseChangeM(
                          pair.rover_hz, pair.base_hz, kL1WavelengthM, rover_span_s, base_span_s),
                      legacyPredictionM(pair.rover_hz, pair.base_hz, kL1WavelengthM,
                                        rover_span_s))
                << "base span " << base_span_s;
        }
    }
}

// Matching or unknown spans: the decision is the legacy decision for every observed change,
// including the ones within a hair of the threshold.
TEST(RTKSlipDetectionTest, MatchingSpansDecideLikeTheLegacyTest) {
    const double previous_sd_phase_m = 12345.678901;
    const double rover_span_s = 0.2;
    const std::vector<double> base_spans = {rover_span_s, rover_span_s + 0.9e-6,
                                            rover_span_s - 0.5e-6, kNaN, 0.0, -0.2};
    int slips = 0;
    int clean = 0;
    for (const double threshold_m : {0.10, 0.20}) {
        for (const auto& pair : dopplerPairs()) {
            const double predicted_m =
                legacyPredictionM(pair.rover_hz, pair.base_hz, kL1WavelengthM, rover_span_s);
            // The residual (observed - predicted) from far below to far above the threshold,
            // finely around it, both signs.
            for (const double scale : {0.0, 0.3, 0.9, 0.999, 0.99999, 1.0, 1.00001, 1.001, 1.1,
                                       2.0, 7.0}) {
                for (const double sign : {1.0, -1.0}) {
                    const double residual_m = sign * scale * threshold_m;
                    const double current_sd_phase_m =
                        previous_sd_phase_m + predicted_m + residual_m;
                    const bool legacy = legacyDecision(previous_sd_phase_m, current_sd_phase_m,
                                                       pair.rover_hz, pair.base_hz,
                                                       kL1WavelengthM, rover_span_s, threshold_m);
                    for (const double base_span_s : base_spans) {
                        EXPECT_EQ(detectDopplerSlipWithBaseSpan(
                                      previous_sd_phase_m, current_sd_phase_m, pair.rover_hz,
                                      pair.base_hz, kL1WavelengthM, rover_span_s, base_span_s,
                                      threshold_m),
                                  legacy)
                            << "threshold " << threshold_m << " residual " << residual_m
                            << " base span " << base_span_s;
                    }
                    (legacy ? slips : clean)++;
                }
            }
        }
    }
    // The sweep crosses the threshold, so it exercises both outcomes.
    EXPECT_GT(slips, 100);
    EXPECT_GT(clean, 100);
}

// A change just under the threshold stays clean with a span offset below the
// tolerance. The offset would move the base-span expression by ~1.7e-4 m here,
// more than the 1e-5 m margin: the legacy expression must be what decides.
TEST(RTKSlipDetectionTest, ChangeJustUnderTheThresholdStaysCleanForMatchingSpans) {
    const double rover_hz = -990.0;
    const double base_hz = -1000.0;
    const double rover_span_s = 0.2;
    const double base_span_s = rover_span_s + 0.9e-6;
    const double threshold_m = 0.10;
    const double previous_sd_phase_m = 50.0;
    const double current_sd_phase_m =
        previous_sd_phase_m +
        legacyPredictionM(rover_hz, base_hz, kL1WavelengthM, rover_span_s) +
        (threshold_m - 1.0e-5);
    ASSERT_FALSE(legacyDecision(previous_sd_phase_m, current_sd_phase_m, rover_hz, base_hz,
                                kL1WavelengthM, rover_span_s, threshold_m));
    EXPECT_FALSE(detectDopplerSlipWithBaseSpan(previous_sd_phase_m, current_sd_phase_m, rover_hz,
                                               base_hz, kL1WavelengthM, rover_span_s,
                                               base_span_s, threshold_m));
    // Past the tolerance the base's own span is used: 4 us of base motion moves the
    // prediction by ~7.6e-4 m, which pushes the same change over the threshold.
    const double far_base_span_s = rover_span_s + 4.0e-6;
    ASSERT_TRUE(dopplerSpansDiffer(rover_span_s, far_base_span_s));
    EXPECT_TRUE(detectDopplerSlipWithBaseSpan(previous_sd_phase_m, current_sd_phase_m, rover_hz,
                                              base_hz, kL1WavelengthM, rover_span_s,
                                              far_base_span_s, threshold_m));
}

// Rover 0.2 s, base 0.201 s, base Doppler ~1 kHz, no slip: the legacy test flags a
// slip (the base moved a millisecond more than the rover), the new one does not.
TEST(RTKSlipDetectionTest, DifferentSpansWithoutSlipAreFlaggedByLegacyOnly) {
    const double rover_hz = 1003.2;
    const double base_hz = 1000.0;
    const double rover_span_s = 0.2;
    const double base_span_s = 0.201;
    const double threshold_m = 0.10;
    const double previous_sd_phase_m = 7.25;
    const double observed_change_m =
        physicalSdPhaseChangeM(rover_hz, base_hz, kL1WavelengthM, rover_span_s, base_span_s);
    const double current_sd_phase_m = previous_sd_phase_m + observed_change_m;

    EXPECT_TRUE(legacyDecision(previous_sd_phase_m, current_sd_phase_m, rover_hz, base_hz,
                               kL1WavelengthM, rover_span_s, threshold_m));
    // The legacy residual is the base's extra millisecond: one L1 cycle at 1 kHz.
    EXPECT_NEAR(observed_change_m - legacyPredictionM(rover_hz, base_hz, kL1WavelengthM,
                                                      rover_span_s),
                base_hz * kL1WavelengthM * (base_span_s - rover_span_s), 1.0e-9);

    EXPECT_FALSE(detectDopplerSlipWithBaseSpan(previous_sd_phase_m, current_sd_phase_m, rover_hz,
                                               base_hz, kL1WavelengthM, rover_span_s,
                                               base_span_s, threshold_m));
    EXPECT_NEAR(predictedSingleDifferencePhaseChangeM(rover_hz, base_hz, kL1WavelengthM,
                                                      rover_span_s, base_span_s),
                observed_change_m, 1.0e-9);
}

// The same on a base that is early instead of late (0.199 s), and for every Doppler pair
// and both wavelengths: the new test never flags a clean change, the legacy test does for
// the satellites fast enough that a millisecond of base motion exceeds the threshold.
TEST(RTKSlipDetectionTest, DifferentSpansWithoutSlipNeverTriggerTheNewTest) {
    const double rover_span_s = 0.2;
    const double threshold_m = 0.10;
    const double previous_sd_phase_m = -3.5;
    int legacy_false_slips = 0;
    int checked = 0;
    for (const double wavelength_m : {kL1WavelengthM, kL2WavelengthM}) {
        for (const auto& pair : dopplerPairs()) {
            for (const double base_span_s : {0.199, 0.201, 0.2005, 0.3, 1.0}) {
                const double current_sd_phase_m =
                    previous_sd_phase_m + physicalSdPhaseChangeM(pair.rover_hz, pair.base_hz,
                                                                 wavelength_m, rover_span_s,
                                                                 base_span_s);
                EXPECT_FALSE(detectDopplerSlipWithBaseSpan(
                    previous_sd_phase_m, current_sd_phase_m, pair.rover_hz, pair.base_hz,
                    wavelength_m, rover_span_s, base_span_s, threshold_m))
                    << "rover " << pair.rover_hz << " base " << pair.base_hz << " span "
                    << base_span_s;
                if (legacyDecision(previous_sd_phase_m, current_sd_phase_m, pair.rover_hz,
                                   pair.base_hz, wavelength_m, rover_span_s, threshold_m)) {
                    ++legacy_false_slips;
                }
                ++checked;
            }
        }
    }
    EXPECT_EQ(checked, 80);
    // The scene is not vacuous: the legacy test is wrong for most of it.
    EXPECT_GT(legacy_false_slips, 40);
}

// A real one-cycle slip is still detected when the spans differ, whichever way the
// base span differs and whichever the sign of the slip.
TEST(RTKSlipDetectionTest, RealOneCycleSlipIsDetectedWhenSpansDiffer) {
    const double rover_span_s = 0.2;
    const double threshold_m = 0.10;  // the smallest threshold RTKProcessor applies
    const double previous_sd_phase_m = 100.0;
    for (const auto& pair : dopplerPairs()) {
        for (const double base_span_s : {0.199, 0.201, 0.4, 1.0}) {
            const double clean_change_m = physicalSdPhaseChangeM(
                pair.rover_hz, pair.base_hz, kL1WavelengthM, rover_span_s, base_span_s);
            for (const double slip_cycles : {1.0, -1.0, 2.0, -3.0}) {
                const double current_sd_phase_m =
                    previous_sd_phase_m + clean_change_m + slip_cycles * kL1WavelengthM;
                EXPECT_TRUE(detectDopplerSlipWithBaseSpan(
                    previous_sd_phase_m, current_sd_phase_m, pair.rover_hz, pair.base_hz,
                    kL1WavelengthM, rover_span_s, base_span_s, threshold_m))
                    << "slip " << slip_cycles << " cycles, base " << pair.base_hz
                    << " Hz, base span " << base_span_s;
            }
        }
    }
    // One L2 cycle (0.244 m) exceeds the 0.20 m default threshold as well.
    const double l2_change_m = physicalSdPhaseChangeM(1003.2, 1000.0, kL2WavelengthM,
                                                      rover_span_s, 0.201);
    EXPECT_TRUE(detectDopplerSlipWithBaseSpan(previous_sd_phase_m,
                                              previous_sd_phase_m + l2_change_m + kL2WavelengthM,
                                              1003.2, 1000.0, kL2WavelengthM, rover_span_s,
                                              0.201, 0.20));
    EXPECT_FALSE(detectDopplerSlipWithBaseSpan(previous_sd_phase_m,
                                               previous_sd_phase_m + l2_change_m, 1003.2, 1000.0,
                                               kL2WavelengthM, rover_span_s, 0.201, 0.20));
}

// The legacy test can hide a real slip behind the span mismatch (a -1 cycle slip cancels
// the +1 cycle of the base's extra millisecond); the new test sees it.
TEST(RTKSlipDetectionTest, RealSlipHiddenByTheSpanMismatchIsFoundByTheNewTest) {
    const double rover_hz = 1003.2;
    const double base_hz = 1000.0;
    const double rover_span_s = 0.2;
    const double base_span_s = 0.201;
    const double previous_sd_phase_m = 0.0;
    const double current_sd_phase_m =
        physicalSdPhaseChangeM(rover_hz, base_hz, kL1WavelengthM, rover_span_s, base_span_s) -
        kL1WavelengthM;
    EXPECT_FALSE(legacyDecision(previous_sd_phase_m, current_sd_phase_m, rover_hz, base_hz,
                                kL1WavelengthM, rover_span_s, 0.10));
    EXPECT_TRUE(detectDopplerSlipWithBaseSpan(previous_sd_phase_m, current_sd_phase_m, rover_hz,
                                              base_hz, kL1WavelengthM, rover_span_s,
                                              base_span_s, 0.10));
}

TEST(RTKSlipDetectionTest, PredictionUsesEachReceiversOwnSpan) {
    // Hand-computed: range rate = -wavelength * Doppler.
    const double rover_hz = -500.0;  // +95.1468363991825 m/s
    const double base_hz = 250.0;    // -47.57341819959125 m/s
    const double expected_m = (500.0 * kL1WavelengthM) * 0.2 - (-250.0 * kL1WavelengthM) * 0.5;
    EXPECT_NEAR(predictedSingleDifferencePhaseChangeM(rover_hz, base_hz, kL1WavelengthM, 0.2, 0.5),
                expected_m, 1.0e-12);
    EXPECT_NEAR(expected_m, 95.1468363991825 * 0.2 + 47.57341819959125 * 0.5, 1.0e-9);
    // The rover alone moving, the base standing still, and the reverse.
    EXPECT_NEAR(predictedSingleDifferencePhaseChangeM(rover_hz, 0.0, kL1WavelengthM, 0.2, 0.5),
                95.1468363991825 * 0.2, 1.0e-9);
    EXPECT_NEAR(predictedSingleDifferencePhaseChangeM(0.0, base_hz, kL1WavelengthM, 0.2, 0.5),
                47.57341819959125 * 0.5, 1.0e-9);
}

TEST(RTKSlipDetectionTest, InvalidInputsDoNotTriggerSlipWhenSpansDiffer) {
    const double rover_span_s = 0.2;
    const double base_span_s = 0.201;
    ASSERT_TRUE(dopplerSpansDiffer(rover_span_s, base_span_s));
    auto detect = [&](double previous, double current, double threshold) {
        return detectDopplerSlipWithBaseSpan(previous, current, 1003.2, 1000.0, kL1WavelengthM,
                                             rover_span_s, base_span_s, threshold);
    };
    EXPECT_TRUE(detect(0.0, 5.0, 0.10));
    EXPECT_FALSE(detect(kNaN, 5.0, 0.10));
    EXPECT_FALSE(detect(0.0, kNaN, 0.10));
    EXPECT_FALSE(detect(0.0, kInf, 0.10));
    EXPECT_FALSE(detect(0.0, 5.0, 0.0));
    EXPECT_FALSE(detect(0.0, 5.0, -0.1));
    EXPECT_FALSE(detect(0.0, 5.0, kNaN));
    // A rover span that is not positive never triggers, like detectDopplerSlip().
    EXPECT_FALSE(detectDopplerSlipWithBaseSpan(0.0, 5.0, 1003.2, 1000.0, kL1WavelengthM, -0.2,
                                               0.201, 0.10));
    EXPECT_FALSE(detectDopplerSlipWithBaseSpan(0.0, 5.0, 1003.2, 1000.0, kL1WavelengthM, 0.0,
                                               0.201, 0.10));
}

TEST(RTKSlipDetectionTest, UnusableDopplerGivesNoPredictionWhenSpansDiffer) {
    // Same convention as singleDifferenceRangeRateMps(): no usable Doppler, no prediction.
    EXPECT_EQ(predictedSingleDifferencePhaseChangeM(kNaN, 1000.0, kL1WavelengthM, 0.2, 0.201),
              0.0);
    EXPECT_EQ(predictedSingleDifferencePhaseChangeM(1003.2, kInf, kL1WavelengthM, 0.2, 0.201),
              0.0);
    EXPECT_EQ(predictedSingleDifferencePhaseChangeM(1003.2, 1000.0, 0.0, 0.2, 0.201), 0.0);
    EXPECT_EQ(predictedSingleDifferencePhaseChangeM(1003.2, 1000.0, -0.19, 0.2, 0.201), 0.0);
    EXPECT_EQ(predictedSingleDifferencePhaseChangeM(1003.2, 1000.0, kNaN, 0.2, 0.201), 0.0);
}

}  // namespace
