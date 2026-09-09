#ifdef GNSSPP_HAS_GTSAM

#include <gtest/gtest.h>

#include "../src/algorithms/fgo_gtsam_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace {

using libgnss::GNSSTime;
using libgnss::ImuSample;
using libgnss::Vector3d;

// Test-local transcription of the production loop at
// fgo_gtsam_backend.cpp:1107-1127.  Phase200 deliberately does not refactor
// that loop: this records the exact current behavior without changing solver
// code before a real-data parity gate is available.
struct NativeSegmentForAudit {
    std::size_t sample_index = 0;
    double dt_s = 0.0;
};

struct NativeScheduleForAudit {
    std::size_t sample_cursor = 0;
    std::vector<NativeSegmentForAudit> preceding_segments;
    std::optional<NativeSegmentForAudit> tail_segment;
};

NativeScheduleForAudit nativeScheduleForAudit(
    const std::vector<ImuSample>& samples, const GNSSTime& t0,
    const GNSSTime& t1, std::size_t sample_cursor) {
    NativeScheduleForAudit schedule;
    while (sample_cursor < samples.size() && samples[sample_cursor].time < t0) {
        ++sample_cursor;
    }
    schedule.sample_cursor = sample_cursor;
    std::size_t j = sample_cursor;
    GNSSTime prev_time = t0;
    std::size_t integrated = 0;
    while (j < samples.size() && samples[j].time < t1) {
        const double dt = samples[j].time - prev_time;
        if (dt > 1e-9) {
            schedule.preceding_segments.push_back(NativeSegmentForAudit{j, dt});
            ++integrated;
        }
        prev_time = samples[j].time;
        ++j;
    }
    const double dt_tail = t1 - prev_time;
    if (integrated > 0 && dt_tail > 1e-9 && j > 0) {
        schedule.tail_segment = NativeSegmentForAudit{j - 1, dt_tail};
    }
    return schedule;
}

struct SourceSegment {
    std::size_t sample_index = 0;
    double dt_s = 0.0;
};

std::vector<SourceSegment> sourceInclusiveForwardSegments(
    const std::vector<ImuSample>& samples, const GNSSTime& t0,
    const GNSSTime& t1) {
    std::vector<SourceSegment> segments;
    if (samples.size() < 2) return segments;
    const double last_dt = samples.back().time - samples[samples.size() - 2].time;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (samples[i].time < t0 || samples[i].time > t1) continue;
        const double dt = i + 1 < samples.size()
                              ? samples[i + 1].time - samples[i].time
                              : last_dt;
        segments.push_back(SourceSegment{i, dt});
    }
    return segments;
}

// A physically bounded left-hold schedule is included only as a diagnostic
// comparator.  It is not asserted to be the official source contract: the
// cached MATLAB code uses the inclusive-forward schedule above.
std::vector<SourceSegment> boundedForwardSegments(
    const std::vector<ImuSample>& samples, const GNSSTime& t0,
    const GNSSTime& t1) {
    std::vector<SourceSegment> segments;
    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        const double start = std::max<double>(samples[i].time - t0, 0.0);
        const double end = std::min<double>(samples[i + 1].time - t0,
                                            t1 - t0);
        if (end > start) {
            segments.push_back(SourceSegment{i, end - start});
        }
    }
    return segments;
}

std::vector<ImuSample> makeUnequalSamples(const GNSSTime& t0) {
    const std::vector<double> offsets = {0.0, 0.25, 0.70, 1.0, 1.40};
    std::vector<ImuSample> samples;
    for (const double offset : offsets) {
        ImuSample sample;
        sample.time = t0 + offset;
        // Time-varying inputs make the endpoint/hold choice observable.  The
        // z accelerometer component is gravity so this remains a physically
        // valid identity-attitude control for GTSAM preintegration.
        sample.accel_raw = Vector3d(1.0 + 2.0 * offset, 0.0, 9.80665);
        sample.gyro_raw_radps = Vector3d(0.0, 0.0, 0.1 + 0.3 * offset);
        samples.push_back(sample);
    }
    return samples;
}

double weightedAccel(const std::vector<ImuSample>& samples,
                     const std::vector<NativeSegmentForAudit>& segments) {
    double total = 0.0;
    for (const auto& segment : segments) {
        total += samples[segment.sample_index].accel_raw.x() * segment.dt_s;
    }
    return total;
}

double weightedGyro(const std::vector<ImuSample>& samples,
                    const std::vector<NativeSegmentForAudit>& segments) {
    double total = 0.0;
    for (const auto& segment : segments) {
        total += samples[segment.sample_index].gyro_raw_radps.z() * segment.dt_s;
    }
    return total;
}

double weightedAccel(const std::vector<ImuSample>& samples,
                     const std::vector<SourceSegment>& segments) {
    double total = 0.0;
    for (const auto& segment : segments) {
        total += samples[segment.sample_index].accel_raw.x() * segment.dt_s;
    }
    return total;
}

double weightedGyro(const std::vector<ImuSample>& samples,
                    const std::vector<SourceSegment>& segments) {
    double total = 0.0;
    for (const auto& segment : segments) {
        total += samples[segment.sample_index].gyro_raw_radps.z() * segment.dt_s;
    }
    return total;
}

double totalDt(const std::vector<NativeSegmentForAudit>& segments) {
    double total = 0.0;
    for (const auto& segment : segments) total += segment.dt_s;
    return total;
}

double totalDt(const std::vector<SourceSegment>& segments) {
    double total = 0.0;
    for (const auto& segment : segments) total += segment.dt_s;
    return total;
}

}  // namespace

TEST(GtsamImuTimeIndexingTest,
     NativeScheduleAuditDiffersFromSourceInclusiveForwardAtExactBoundary) {
    const GNSSTime t0(2300, 1000.0);
    const auto samples = makeUnequalSamples(t0);
    const NativeScheduleForAudit native =
        nativeScheduleForAudit(samples, t0, t0 + 1.0, 0);
    const auto source = sourceInclusiveForwardSegments(samples, t0, t0 + 1.0);

    ASSERT_EQ(native.preceding_segments.size(), 2U);
    ASSERT_TRUE(native.tail_segment.has_value());
    EXPECT_EQ(native.preceding_segments[0].sample_index, 1U);
    EXPECT_EQ(native.preceding_segments[1].sample_index, 2U);
    EXPECT_NEAR(native.preceding_segments[0].dt_s, 0.25, 1e-10);
    EXPECT_NEAR(native.preceding_segments[1].dt_s, 0.45, 1e-10);
    EXPECT_EQ(native.tail_segment->sample_index, 2U);
    EXPECT_NEAR(native.tail_segment->dt_s, 0.30, 1e-10);

    std::vector<NativeSegmentForAudit> native_all = native.preceding_segments;
    native_all.push_back(*native.tail_segment);
    ASSERT_EQ(source.size(), 4U);
    EXPECT_EQ(source.front().sample_index, 0U);
    EXPECT_EQ(source.back().sample_index, 3U);
    EXPECT_NEAR(totalDt(native_all), 1.0, 1e-10);
    EXPECT_NEAR(totalDt(source), 1.40, 1e-10);

    // The analytic integrals over the bounded [0, 1] interval are 2.0 m/s
    // and 0.25 rad for these polynomials.  These are diagnostics, not an
    // assertion that either historical sample-hold convention is the gold
    // model; they make the observed timing/hold discrepancy reproducible.
    EXPECT_NEAR(weightedAccel(samples, native_all), 2.175, 1e-10);
    EXPECT_NEAR(weightedGyro(samples, native_all), 0.27625, 1e-10);
    EXPECT_NEAR(weightedAccel(samples, source), 2.845, 1e-10);
    EXPECT_NEAR(weightedGyro(samples, source), 0.35675, 1e-10);
    const double analytic_accel_integral = 1.0 + 1.0;
    const double analytic_gyro_integral = 0.1 + 0.15;
    EXPECT_NEAR(analytic_accel_integral, 2.0, 1e-12);
    EXPECT_NEAR(analytic_gyro_integral, 0.25, 1e-12);
    EXPECT_GT(std::abs(weightedAccel(samples, native_all) -
                       weightedAccel(samples, source)),
              0.2);
}

TEST(GtsamImuTimeIndexingTest,
     NativeScheduleAuditDiffersWhenGnssBoundaryFallsBetweenSamples) {
    const GNSSTime t0(2300, 1000.0);
    const auto samples = makeUnequalSamples(t0);
    const NativeScheduleForAudit native =
        nativeScheduleForAudit(samples, t0, t0 + 0.90, 0);
    const auto source = sourceInclusiveForwardSegments(samples, t0, t0 + 0.90);

    ASSERT_EQ(native.preceding_segments.size(), 2U);
    ASSERT_TRUE(native.tail_segment.has_value());
    std::vector<NativeSegmentForAudit> native_all = native.preceding_segments;
    native_all.push_back(*native.tail_segment);
    ASSERT_EQ(source.size(), 3U);
    EXPECT_NEAR(totalDt(native_all), 0.90, 1e-10);
    EXPECT_NEAR(totalDt(source), 1.00, 1e-10);
    EXPECT_EQ(native.tail_segment->sample_index, 2U);
    EXPECT_NEAR(native.tail_segment->dt_s, 0.20, 1e-10);
    EXPECT_EQ(source.back().sample_index, 2U);
    EXPECT_NEAR(source.back().dt_s, 0.30, 1e-10);
}

TEST(GtsamImuTimeIndexingTest,
     ConstantInputControlSeparatesDurationFromSampleValueChoice) {
    const GNSSTime t0(2300, 1000.0);
    auto samples = makeUnequalSamples(t0);
    for (auto& sample : samples) {
        sample.accel_raw = Vector3d(2.0, 0.0, 9.80665);
        sample.gyro_raw_radps = Vector3d(0.0, 0.0, 0.4);
    }

    const NativeScheduleForAudit native_exact =
        nativeScheduleForAudit(samples, t0, t0 + 1.0, 0);
    const auto source_exact =
        sourceInclusiveForwardSegments(samples, t0, t0 + 1.0);
    const auto bounded_exact = boundedForwardSegments(samples, t0, t0 + 1.0);
    std::vector<NativeSegmentForAudit> native_exact_all =
        native_exact.preceding_segments;
    native_exact_all.push_back(*native_exact.tail_segment);
    EXPECT_NEAR(totalDt(native_exact_all), 1.0, 1e-10);
    EXPECT_NEAR(totalDt(source_exact), 1.40, 1e-10);
    EXPECT_NEAR(totalDt(bounded_exact), 1.0, 1e-10);
    EXPECT_NEAR(weightedAccel(samples, native_exact_all), 2.0, 1e-10);
    EXPECT_NEAR(weightedAccel(samples, source_exact), 2.8, 1e-10);
    EXPECT_NEAR(weightedAccel(samples, bounded_exact), 2.0, 1e-10);

    const NativeScheduleForAudit native_between =
        nativeScheduleForAudit(samples, t0, t0 + 0.90, 0);
    const auto source_between =
        sourceInclusiveForwardSegments(samples, t0, t0 + 0.90);
    const auto bounded_between =
        boundedForwardSegments(samples, t0, t0 + 0.90);
    std::vector<NativeSegmentForAudit> native_between_all =
        native_between.preceding_segments;
    native_between_all.push_back(*native_between.tail_segment);
    EXPECT_NEAR(totalDt(native_between_all), 0.90, 1e-10);
    EXPECT_NEAR(totalDt(source_between), 1.00, 1e-10);
    EXPECT_NEAR(totalDt(bounded_between), 0.90, 1e-10);

    // Exercise a start that is between samples as well.  The bounded
    // comparator keeps the partial [0.10, 0.25) interval; dropping it would
    // hide a separate boundary-indexing bug.
    const NativeScheduleForAudit native_offset =
        nativeScheduleForAudit(samples, t0 + 0.10, t0 + 0.90, 0);
    const auto source_offset =
        sourceInclusiveForwardSegments(samples, t0 + 0.10, t0 + 0.90);
    const auto bounded_offset =
        boundedForwardSegments(samples, t0 + 0.10, t0 + 0.90);
    std::vector<NativeSegmentForAudit> native_offset_all =
        native_offset.preceding_segments;
    native_offset_all.push_back(*native_offset.tail_segment);
    EXPECT_NEAR(totalDt(native_offset_all), 0.80, 1e-10);
    EXPECT_NEAR(totalDt(source_offset), 0.75, 1e-10);
    EXPECT_NEAR(totalDt(bounded_offset), 0.80, 1e-10);
}

TEST(GtsamImuTimeIndexingTest,
     NativeScheduleAuditFeedsGtsamPreintegrationWithItsOwnDuration) {
    const GNSSTime t0(2300, 1000.0);
    auto samples = makeUnequalSamples(t0);
    // Keep the GTSAM delta-V assertion one-dimensional: the source schedule
    // comparison above covers varying gyro timing, while nonzero yaw would
    // rotate the varying x acceleration inside the preintegrator.
    for (auto& sample : samples) sample.gyro_raw_radps = Vector3d::Zero();
    const NativeScheduleForAudit native =
        nativeScheduleForAudit(samples, t0, t0 + 1.0, 0);

    auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.80665);
    const gtsam::imuBias::ConstantBias bias;
    gtsam::PreintegratedCombinedMeasurements pim(params, bias);
    for (const auto& segment : native.preceding_segments) {
        const auto& sample = samples[segment.sample_index];
        pim.integrateMeasurement(gtsam::Vector3(sample.accel_raw),
                                 gtsam::Vector3(sample.gyro_raw_radps),
                                 segment.dt_s);
    }
    ASSERT_TRUE(native.tail_segment.has_value());
    const auto& tail_sample = samples[native.tail_segment->sample_index];
    pim.integrateMeasurement(gtsam::Vector3(tail_sample.accel_raw),
                             gtsam::Vector3(tail_sample.gyro_raw_radps),
                             native.tail_segment->dt_s);

    EXPECT_NEAR(pim.deltaTij(), 1.0, 1e-12);
    EXPECT_NEAR(pim.deltaVij().x(), 2.175, 1e-12);
}

TEST(GtsamImuTimeIndexingTest,
     Phase201SourceScheduleUsesInclusiveForwardDeltasAndAdjacentCursor) {
    const GNSSTime t0(2300, 1000.0);
    const auto samples = makeUnequalSamples(t0);
    const auto exact =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            samples, t0, t0 + 1.0);
    ASSERT_TRUE(exact.valid);
    ASSERT_EQ(exact.segments.size(), 4U);
    EXPECT_EQ(exact.segments[0].sample_index, 0U);
    EXPECT_EQ(exact.segments[1].sample_index, 1U);
    EXPECT_EQ(exact.segments[2].sample_index, 2U);
    EXPECT_EQ(exact.segments[3].sample_index, 3U);
    EXPECT_NEAR(exact.segments[0].dt_s, 0.25, 1e-10);
    EXPECT_NEAR(exact.segments[1].dt_s, 0.45, 1e-10);
    EXPECT_NEAR(exact.segments[2].dt_s, 0.30, 1e-10);
    EXPECT_NEAR(exact.segments[3].dt_s, 0.40, 1e-10);
    EXPECT_NEAR(exact.integrated_duration_s, 1.40, 1e-10);

    // The returned overlap cursor keeps the exact t=1 endpoint available in
    // the next adjacent interval, while avoiding an O(samples*epochs) scan.
    const auto adjacent =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            samples, t0 + 1.0, t0 + 2.0, exact.next_sample_index, false);
    ASSERT_TRUE(adjacent.valid);
    ASSERT_EQ(adjacent.segments.size(), 2U);
    EXPECT_EQ(adjacent.segments.front().sample_index, 3U);
    EXPECT_EQ(adjacent.segments.back().sample_index, 4U);
    EXPECT_NEAR(adjacent.integrated_duration_s, 0.80, 1e-10);

    // The cursor path must agree with an independently validated call.  The
    // shared t=1 sample is deliberately present in both intervals.
    const auto adjacent_full =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            samples, t0 + 1.0, t0 + 2.0, 0U, true);
    ASSERT_TRUE(adjacent_full.valid);
    ASSERT_EQ(adjacent_full.segments.size(), adjacent.segments.size());
    for (std::size_t i = 0; i < adjacent.segments.size(); ++i) {
        EXPECT_EQ(adjacent_full.segments[i].sample_index,
                  adjacent.segments[i].sample_index);
        EXPECT_NEAR(adjacent_full.segments[i].dt_s,
                    adjacent.segments[i].dt_s, 1e-12);
    }

    // A GNSS interval can begin and end between IMU samples.  The source
    // inclusive-forward contract selects the samples at .25 and .70 and
    // retains their full forward deltas, .45 + .30 = .75 s.
    const auto between =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            samples, t0 + 0.10, t0 + 0.90);
    ASSERT_TRUE(between.valid);
    ASSERT_EQ(between.segments.size(), 2U);
    EXPECT_EQ(between.segments[0].sample_index, 1U);
    EXPECT_EQ(between.segments[1].sample_index, 2U);
    EXPECT_NEAR(between.integrated_duration_s, 0.75, 1e-10);
}

TEST(GtsamImuTimeIndexingTest,
     Phase201SourceScheduleUsesExactBoundsAcrossFuzzyTimeEqualityAndWeekRollover) {
    const GNSSTime t0(2300, 1000.0);
    std::vector<ImuSample> near_boundary;
    for (const double offset : {-0.5e-6, 0.0, 1.0, 1.0 + 0.5e-6, 2.0}) {
        ImuSample sample;
        sample.time = t0 + offset;
        sample.accel_raw = Vector3d::Zero();
        sample.gyro_raw_radps = Vector3d::Zero();
        near_boundary.push_back(sample);
    }
    const auto exact =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            near_boundary, t0, t0 + 1.0);
    ASSERT_TRUE(exact.valid);
    ASSERT_EQ(exact.segments.size(), 2U);
    EXPECT_EQ(exact.segments[0].sample_index, 1U);
    EXPECT_EQ(exact.segments[1].sample_index, 2U);

    const GNSSTime rollover(2300, 604799.75);
    std::vector<ImuSample> rollover_samples;
    for (const double offset : {0.0, 0.25, 0.75, 1.25}) {
        ImuSample sample;
        sample.time = rollover + offset;
        sample.accel_raw = Vector3d::Zero();
        sample.gyro_raw_radps = Vector3d::Zero();
        rollover_samples.push_back(sample);
    }
    const auto rolled =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            rollover_samples, rollover, rollover + 1.0);
    ASSERT_TRUE(rolled.valid);
    ASSERT_EQ(rolled.segments.size(), 3U);
    EXPECT_EQ(rolled.segments.front().sample_index, 0U);
    EXPECT_EQ(rolled.segments.back().sample_index, 2U);
    EXPECT_NEAR(rolled.integrated_duration_s, 1.25, 1e-10);
}

TEST(GtsamImuTimeIndexingTest,
     Phase201SourceScheduleFailsClosedForEmptyMalformedAndInvalidDeltas) {
    const GNSSTime t0(2300, 1000.0);
    auto sample_at = [t0](double offset) {
        ImuSample sample;
        sample.time = t0 + offset;
        sample.accel_raw = Vector3d::Zero();
        sample.gyro_raw_radps = Vector3d::Zero();
        return sample;
    };
    const auto empty =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            {sample_at(-2.0), sample_at(-1.0)}, t0, t0 + 1.0);
    EXPECT_FALSE(empty.valid);
    EXPECT_TRUE(empty.empty_interval);

    auto bad_measurement = std::vector<ImuSample>{sample_at(0.0),
                                                    sample_at(1.0)};
    bad_measurement.front().accel_raw.x() =
        std::numeric_limits<double>::quiet_NaN();
    const auto invalid =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            bad_measurement, t0, t0 + 1.0);
    EXPECT_FALSE(invalid.valid);
    EXPECT_EQ(invalid.invalid_sample_count, 1U);

    auto nonfinite_delta = std::vector<ImuSample>{sample_at(0.0),
                                                    sample_at(1.0)};
    nonfinite_delta[1].time.tow =
        std::numeric_limits<double>::quiet_NaN();
    const auto nonfinite =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            nonfinite_delta, t0, t0 + 1.0, 0U, false);
    EXPECT_FALSE(nonfinite.valid);
    EXPECT_EQ(nonfinite.nonfinite_dt_count, 1U);

    const auto nonpositive =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            {sample_at(0.0), sample_at(0.0)}, t0, t0 + 1.0, 0U, false);
    EXPECT_FALSE(nonpositive.valid);
    EXPECT_EQ(nonpositive.nonpositive_dt_count, 1U);

    // Full validation rejects malformed ordering even when the malformed
    // pair is outside the selected GNSS interval.
    const auto malformed_order =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            {sample_at(0.0), sample_at(2.0), sample_at(1.0)}, t0, t0 + 0.5);
    EXPECT_FALSE(malformed_order.valid);
    EXPECT_EQ(malformed_order.nonpositive_dt_count, 1U);
}

TEST(GtsamImuTimeIndexingTest,
     Phase201SourceScheduleFeedsGtsamWithTimeVaryingAccelAndGyro) {
    const GNSSTime t0(2300, 1000.0);
    auto samples = makeUnequalSamples(t0);
    const auto schedule =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            samples, t0, t0 + 1.0);
    ASSERT_TRUE(schedule.valid);

    auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.80665);
    const gtsam::imuBias::ConstantBias bias;
    gtsam::PreintegratedCombinedMeasurements pim(params, bias);
    for (const auto& segment : schedule.segments) {
        const auto& sample = samples[segment.sample_index];
        pim.integrateMeasurement(gtsam::Vector3(sample.accel_raw),
                                 gtsam::Vector3::Zero(), segment.dt_s);
    }
    EXPECT_NEAR(pim.deltaTij(), 1.40, 1e-12);
    // The x acceleration is held at each inclusive sample for its forward
    // delta: 1*.25 + 1.5*.45 + 2.4*.30 + 3*.40 = 2.845 m/s.
    EXPECT_NEAR(pim.deltaVij().x(), 2.845, 1e-12);

    gtsam::PreintegratedCombinedMeasurements yaw_pim(params, bias);
    for (const auto& segment : schedule.segments) {
        const auto& sample = samples[segment.sample_index];
        yaw_pim.integrateMeasurement(gtsam::Vector3(0.0, 0.0, 9.80665),
                                     gtsam::Vector3(sample.gyro_raw_radps),
                                     segment.dt_s);
    }
    EXPECT_NEAR(yaw_pim.deltaTij(), 1.40, 1e-12);
    EXPECT_NEAR(yaw_pim.deltaRij().yaw(), 0.35675, 1e-12);

    // Constant acceleration is a control for the sample-value choice: the
    // inclusive-forward schedule changes duration, but not the held value.
    auto constant_samples = samples;
    for (auto& sample : constant_samples) {
        sample.accel_raw = Vector3d(2.0, 0.0, 9.80665);
        sample.gyro_raw_radps = Vector3d::Zero();
    }
    const auto constant_schedule =
        libgnss::fgo_gtsam_internal::makeSourceInclusiveForwardImuSchedule(
            constant_samples, t0, t0 + 1.0);
    ASSERT_TRUE(constant_schedule.valid);
    gtsam::PreintegratedCombinedMeasurements constant_pim(params, bias);
    for (const auto& segment : constant_schedule.segments) {
        const auto& sample = constant_samples[segment.sample_index];
        constant_pim.integrateMeasurement(gtsam::Vector3(sample.accel_raw),
                                          gtsam::Vector3::Zero(),
                                          segment.dt_s);
    }
    EXPECT_NEAR(constant_pim.deltaTij(), 1.40, 1e-12);
    EXPECT_NEAR(constant_pim.deltaVij().x(), 2.80, 1e-12);
}

#endif  // GNSSPP_HAS_GTSAM
