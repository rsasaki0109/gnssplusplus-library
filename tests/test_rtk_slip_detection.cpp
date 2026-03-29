#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_slip_detection.hpp>

namespace {

using libgnss::rtk_slip_detection::detectDopplerSlip;
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

TEST(RTKSlipDetectionTest, InvalidInputsDoNotTriggerSlip) {
    EXPECT_FALSE(detectDopplerSlip(1.0, 2.0, 0.0, 0.0, 0.2));
    EXPECT_FALSE(detectDopplerSlip(1.0, 2.0, 0.0, 1.0, -0.1));
}

}  // namespace
