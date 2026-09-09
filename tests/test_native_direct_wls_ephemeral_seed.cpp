#include <gtest/gtest.h>

#include <libgnss++/algorithms/source_clock_c0d_initializer.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

using libgnss::GNSSTime;
using libgnss::Vector3d;
using libgnss::source_clock_c0d::RetainedRawEpochKey;
using libgnss::source_clock_c0d::
    DirectWlsEphemeralC7DSeedReport;
using libgnss::source_clock_c0d::
    validateAndCopyDirectWlsEphemeralC7DSeed;

struct SyntheticSeedInputs {
    std::vector<RetainedRawEpochKey> keys;
    std::vector<Vector3d> positions;
    std::vector<double> clocks;
    std::vector<double> retained_drift;
    std::vector<Vector3d> velocities;
    std::vector<GNSSTime> raw_times;
    std::vector<std::int64_t> raw_utc;
    std::vector<double> raw_drift;
};

SyntheticSeedInputs makeSeed(std::size_t count) {
    SyntheticSeedInputs input;
    const GNSSTime start(2300, 100.0);
    for (std::size_t i = 0; i < count; ++i) {
        const GNSSTime time = start + static_cast<double>(i);
        input.keys.push_back(
            {i, 1610000000000LL + static_cast<std::int64_t>(i) * 1000LL,
             time});
        input.positions.emplace_back(6378137.0 + static_cast<double>(i), 2.0,
                                     3.0);
        input.clocks.push_back(10.0 + static_cast<double>(i));
        input.retained_drift.push_back(0.25 + 0.01 * static_cast<double>(i));
        input.velocities.emplace_back(1.0, 2.0, 3.0);
        input.raw_times.push_back(time);
        input.raw_utc.push_back(input.keys.back().raw_utc_time_millis);
        input.raw_drift.push_back(input.retained_drift.back());
    }
    return input;
}

bool validate(const SyntheticSeedInputs& input,
              std::vector<std::array<double, 7>>& c_out,
              std::vector<double>& d_out,
              DirectWlsEphemeralC7DSeedReport& report) {
    return validateAndCopyDirectWlsEphemeralC7DSeed(
        input.keys, input.positions, input.clocks, input.retained_drift,
        input.velocities, input.raw_times, input.raw_utc, input.raw_drift,
        c_out, d_out, report);
}

}  // namespace

TEST(NativeDirectWlsEphemeralSeed,
     AAndLaxStyleFullRawKeysMapScalarClockToOfficialC7AndD) {
    for (const std::size_t count : {2U, 4U}) {
        const SyntheticSeedInputs input = makeSeed(count);
        std::vector<std::array<double, 7>> c_out;
        std::vector<double> d_out;
        DirectWlsEphemeralC7DSeedReport report;

        ASSERT_TRUE(validate(input, c_out, d_out, report));
        EXPECT_TRUE(report.valid);
        EXPECT_TRUE(report.full_raw_coverage);
        EXPECT_EQ(report.exact_key_count, count);
        ASSERT_EQ(c_out.size(), count);
        ASSERT_EQ(d_out.size(), count);
        for (std::size_t i = 0; i < count; ++i) {
            EXPECT_DOUBLE_EQ(c_out[i][0], input.clocks[i]);
            for (std::size_t component = 1; component < 7; ++component) {
                EXPECT_DOUBLE_EQ(c_out[i][component], 0.0);
            }
            EXPECT_DOUBLE_EQ(d_out[i], input.retained_drift[i]);
        }
    }
}

TEST(NativeDirectWlsEphemeralSeed,
     UStyleSparseRetentionFailsClosedWithoutC7OrDOutput) {
    SyntheticSeedInputs input = makeSeed(4);
    input.keys.pop_back();
    input.positions.pop_back();
    input.clocks.pop_back();
    input.retained_drift.pop_back();
    input.velocities.pop_back();

    std::vector<std::array<double, 7>> c_out;
    std::vector<double> d_out;
    DirectWlsEphemeralC7DSeedReport report;
    EXPECT_FALSE(validate(input, c_out, d_out, report));
    EXPECT_FALSE(report.full_raw_coverage);
    EXPECT_EQ(report.failure, "direct WLS seed does not cover every raw epoch");
    EXPECT_TRUE(c_out.empty());
    EXPECT_TRUE(d_out.empty());
}

TEST(NativeDirectWlsEphemeralSeed,
     HStyleMissingVelocityFailsClosedWithoutFiniteDifferenceOrZeroFill) {
    SyntheticSeedInputs input = makeSeed(3);
    input.velocities[1] =
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    std::vector<std::array<double, 7>> c_out;
    std::vector<double> d_out;
    DirectWlsEphemeralC7DSeedReport report;
    EXPECT_FALSE(validate(input, c_out, d_out, report));
    EXPECT_EQ(report.nonfinite_velocity_count, 1U);
    EXPECT_EQ(report.failure,
              "direct WLS seed contains a non-finite position, clock, D, or velocity");
    EXPECT_TRUE(c_out.empty());
    EXPECT_TRUE(d_out.empty());
}

TEST(NativeDirectWlsEphemeralSeed,
     ReorderedOrMutatedRawKeyAndDriftAreRejected) {
    SyntheticSeedInputs input = makeSeed(3);
    std::swap(input.keys[0], input.keys[1]);
    input.retained_drift[2] += 0.5;

    std::vector<std::array<double, 7>> c_out;
    std::vector<double> d_out;
    DirectWlsEphemeralC7DSeedReport report;
    EXPECT_FALSE(validate(input, c_out, d_out, report));
    EXPECT_GT(report.raw_key_order_mismatch_count, 0U);
    EXPECT_GT(report.raw_drift_mismatch_count, 0U);
    EXPECT_TRUE(c_out.empty());
    EXPECT_TRUE(d_out.empty());
}
