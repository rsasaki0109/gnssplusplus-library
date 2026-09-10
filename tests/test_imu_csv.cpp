#include <gtest/gtest.h>

#include <libgnss++/io/imu.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace libgnss {
namespace {

void writeFile(const std::filesystem::path& path, const std::string& content) {
    std::ofstream file(path, std::ios::binary);
    file << content;
}

TEST(ImuCsvTest, ParsesRealPpcHeaderVerbatim) {
    // Exact header string from data/PPC-Dataset/tokyo/run1/imu.csv, including
    // its irregular double spaces before "Ang Rate Y"/"Ang Rate Z" and the
    // single leading space after every comma. normalize_header()-style
    // whitespace stripping is mandatory for this to parse correctly.
    const std::string header =
        "GPS TOW (s), GPS Week, Acc X (m/s^2), Acc Y (m/s^2), Acc Z (m/s^2), "
        "Ang Rate X (deg/s),  Ang Rate Y (deg/s),  Ang Rate Z (deg/s)";
    const auto path = std::filesystem::temp_directory_path() / "libgnss_imu_real_header_test.csv";
    writeFile(path, header + "\n" +
                         "187470.00, 2324,  0.38587500, -0.32891250,  9.80980000, "
                         "-0.07750000,  0.25375000, -0.09000000\n");

    ImuSeries series;
    const auto result = loadImuCsv(path.string(), series);
    std::filesystem::remove(path);

    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.row_count, 1);
    ASSERT_EQ(series.samples.size(), 1u);
    EXPECT_EQ(series.samples[0].time.week, 2324);
    EXPECT_NEAR(series.samples[0].time.tow, 187470.00, 1e-9);
    EXPECT_NEAR(series.samples[0].accel_raw(0), 0.38587500, 1e-9);
    EXPECT_NEAR(series.samples[0].accel_raw(1), -0.32891250, 1e-9);
    EXPECT_NEAR(series.samples[0].accel_raw(2), 9.80980000, 1e-9);
    // Gyro converted deg/s -> rad/s at load time.
    EXPECT_NEAR(series.samples[0].gyro_raw_radps(0), -0.07750000 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(series.samples[0].gyro_raw_radps(1), 0.25375000 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(series.samples[0].gyro_raw_radps(2), -0.09000000 * M_PI / 180.0, 1e-12);
}

TEST(ImuCsvTest, ResolvesAlternateHeaderSpellings) {
    // Alternate header spellings that should resolve to the same logical
    // columns as analyze_ppc_imu_coverage.py's candidate lists would (a
    // port-parity check between the Python analysis script and this loader).
    const std::string header = "tow,week,AccX(m/s2),accel_y,az,gyrox,AngRateY(deg/s),wz";
    const auto path = std::filesystem::temp_directory_path() / "libgnss_imu_alt_header_test.csv";
    writeFile(path, header + "\n" + "100.0,2000,1.0,2.0,3.0,0.1,0.2,0.3\n");

    ImuSeries series;
    const auto result = loadImuCsv(path.string(), series);
    std::filesystem::remove(path);

    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(series.samples.size(), 1u);
    EXPECT_EQ(series.samples[0].time.week, 2000);
    EXPECT_NEAR(series.samples[0].time.tow, 100.0, 1e-9);
    EXPECT_NEAR(series.samples[0].accel_raw(0), 1.0, 1e-9);
    EXPECT_NEAR(series.samples[0].accel_raw(1), 2.0, 1e-9);
    EXPECT_NEAR(series.samples[0].accel_raw(2), 3.0, 1e-9);
    EXPECT_NEAR(series.samples[0].gyro_raw_radps(0), 0.1 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(series.samples[0].gyro_raw_radps(1), 0.2 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(series.samples[0].gyro_raw_radps(2), 0.3 * M_PI / 180.0, 1e-12);
}

TEST(ImuCsvTest, MissingRequiredColumnProducesDescriptiveError) {
    const std::string header = "tow,week,ax,ay,az,gyrox,gyroy";  // gyro Z missing
    const auto path = std::filesystem::temp_directory_path() / "libgnss_imu_missing_column_test.csv";
    writeFile(path, header + "\n" + "100.0,2000,1.0,2.0,3.0,0.1,0.2\n");

    ImuSeries series;
    const auto result = loadImuCsv(path.string(), series);
    std::filesystem::remove(path);

    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.error.empty());
    EXPECT_TRUE(series.samples.empty());
}

TEST(ImuCsvTest, MissingFileProducesError) {
    ImuSeries series;
    const auto result = loadImuCsv("this/path/does/not/exist_libgnss_imu.csv", series);
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.error.empty());
}

TEST(ImuCsvTest, RejectsMatInputsBeforeOpeningAnyFile) {
    const auto path =
        std::filesystem::temp_directory_path() / "libgnss_forbidden_input.MAT";
    // The path does not need to exist: the extension guard must fire before
    // attempting an open, and no MATLAB container is part of this contract.
    ImuSeries series;
    const auto result = loadImuCsv(path.string(), series);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("MATLAB .mat inputs are forbidden"), std::string::npos);
    EXPECT_TRUE(series.samples.empty());
}

TEST(AndroidImuCsvTest, ConvertsUtcAndAlignsAccelOnElapsedClock) {
    constexpr std::int64_t kUtc0 = 1'700'000'000'000;
    const std::string header =
        "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,"
        "MeasurementZ,BiasX,BiasY,BiasZ";
    const auto path =
        std::filesystem::temp_directory_path() / "libgnss_android_imu_alignment_test.csv";
    writeFile(
        path,
        header + "\n" +
            "UncalAccel," + std::to_string(kUtc0) + ",1000000000,1,2,3,0,0,0\n" +
            "UncalAccel," + std::to_string(kUtc0 + 20) + ",1020000000,3,4,5,0,0,0\n" +
            // MATLAB unique() retains the first row for duplicate UTC keys.
            "UncalAccel," + std::to_string(kUtc0 + 20) + ",1030000000,99,99,99,0,0,0\n" +
            "UncalGyro," + std::to_string(kUtc0 + 10) + ",1010000000,0.1,0.2,0.3,0,0,0\n" +
            "UncalGyro," + std::to_string(kUtc0 - 10) + ",990000000,0.4,0.5,0.6,0,0,0\n" +
            "UncalGyro," + std::to_string(kUtc0 + 30) + ",1030000000,0.7,0.8,0.9,0,0,0\n" +
            "UncalMag," + std::to_string(kUtc0) + ",1000000000,10,11,12,1,1,1\n");

    ImuSeries series;
    const auto result = loadAndroidImuCsv(path.string(), series);
    std::filesystem::remove(path);

    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.total_rows, 7u);
    EXPECT_EQ(result.accel_rows, 2u);
    EXPECT_EQ(result.gyro_rows, 3u);
    EXPECT_EQ(result.unsupported_rows, 1u);
    EXPECT_EQ(result.duplicate_accel_timestamps, 1u);
    EXPECT_EQ(result.paired_rows, 3u);
    EXPECT_EQ(result.interpolated_rows, 1u);
    EXPECT_EQ(result.endpoint_nearest_rows, 2u);
    EXPECT_EQ(result.omitted_rows, 0u);
    EXPECT_TRUE(result.elapsed_clock_preserved);
    EXPECT_FALSE(result.gnss_elapsed_anchor_applied);
    EXPECT_EQ(result.first_gyro_elapsed_ns, 990000000);
    EXPECT_EQ(result.last_gyro_elapsed_ns, 1030000000);
    EXPECT_NEAR(result.median_abs_pair_offset_ms, 10.0, 1e-12);
    EXPECT_NEAR(result.maximum_abs_pair_offset_ms, 10.0, 1e-12);
    ASSERT_EQ(series.samples.size(), 3u);
    EXPECT_NEAR(series.samples[0].accel_raw.x(), 1.0, 1e-12);
    EXPECT_NEAR(series.samples[0].accel_raw.y(), 2.0, 1e-12);
    EXPECT_NEAR(series.samples[0].accel_raw.z(), 3.0, 1e-12);
    EXPECT_NEAR(series.samples[1].accel_raw.x(), 2.0, 1e-12);
    EXPECT_NEAR(series.samples[1].accel_raw.y(), 3.0, 1e-12);
    EXPECT_NEAR(series.samples[1].accel_raw.z(), 4.0, 1e-12);
    EXPECT_NEAR(series.samples[2].accel_raw.x(), 3.0, 1e-12);
    EXPECT_NEAR(series.samples[2].accel_raw.y(), 4.0, 1e-12);
    EXPECT_NEAR(series.samples[2].accel_raw.z(), 5.0, 1e-12);
    // UncalGyro is already rad/s; the raw C++ adapter does not convert twice.
    EXPECT_NEAR(series.samples[1].gyro_raw_radps.x(), 0.1, 1e-12);
    EXPECT_NEAR(series.samples[1].gyro_raw_radps.y(), 0.2, 1e-12);
    EXPECT_NEAR(series.samples[1].gyro_raw_radps.z(), 0.3, 1e-12);
    EXPECT_EQ(series.samples[1].elapsed_realtime_nanos, 1010000000);
    EXPECT_LT(series.samples[0].time, series.samples[1].time);
    EXPECT_LT(series.samples[1].time, series.samples[2].time);
}

TEST(AndroidImuCsvTest, SynchronizesUtcFromGnssElapsedAnchorsAndRequiresThem) {
    const auto gnss_path =
        std::filesystem::temp_directory_path() / "libgnss_android_gnss_anchor_test.csv";
    writeFile(
        gnss_path,
        "MessageType,utcTimeMillis,ChipsetElapsedRealtimeNanos\n"
        "Raw,1700000000000,1000000000\n"
        "UncalFix,1700000000500,1500000000\n"
        "Raw,1700000001000,2000000000\n"
        // MATLAB unique(utcTimeMillis) retains the first elapsed value.
        "Raw,1700000001000,2100000000\n");

    std::vector<AndroidGnssTimeAnchor> anchors;
    const auto anchor_result = loadAndroidGnssTimeAnchors(gnss_path.string(), anchors);
    ASSERT_TRUE(anchor_result.ok) << anchor_result.error;
    EXPECT_EQ(anchor_result.input_rows, 4u);
    EXPECT_EQ(anchor_result.raw_rows, 3u);
    EXPECT_EQ(anchor_result.unsupported_rows, 1u);
    EXPECT_EQ(anchor_result.duplicate_utc_timestamps, 1u);
    ASSERT_EQ(anchors.size(), 2u);
    EXPECT_EQ(anchors.front().utc_time_ms, 1700000000000LL);
    EXPECT_EQ(anchors.back().elapsed_realtime_nanos, 2000000000LL);

    const std::string imu_header =
        "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,"
        "MeasurementZ,BiasX,BiasY,BiasZ";
    const auto imu_path =
        std::filesystem::temp_directory_path() / "libgnss_android_gnss_anchor_imu_test.csv";
    writeFile(
        imu_path,
        imu_header + "\n"
                     // The raw UTC values below are intentionally unrelated;
                     // anchored elapsed time is authoritative for inference.
                     "UncalAccel,1800000000000,1000000000,1,2,3,0,0,0\n"
                     "UncalAccel,1800000000010,1010000000,3,4,5,0,0,0\n"
                     "UncalGyro,1800000000000,1005000000,0.1,0.2,0.3,0,0,0\n"
                     "UncalGyro,1800000000010,1010000000,0.4,0.5,0.6,0,0,0\n");

    AndroidImuCsvConfig config;
    config.require_gnss_elapsed_anchor = true;
    config.imu_sync_coefficient = 0.5;
    ImuSeries series;
    const auto result = loadAndroidImuCsv(imu_path.string(), series, config, anchors);
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_TRUE(result.gnss_elapsed_anchor_applied);
    EXPECT_EQ(result.gnss_anchor_points, 2u);
    EXPECT_EQ(result.gnss_anchor_interpolated_rows, 2u);
    EXPECT_EQ(result.gnss_anchor_extrapolated_rows, 0u);
    EXPECT_DOUBLE_EQ(result.imu_sync_coefficient, 0.5);
    EXPECT_NEAR(result.first_mapped_utc_time_ms, 1700000000005.0, 1e-6);
    EXPECT_NEAR(result.last_mapped_utc_time_ms, 1700000000010.0, 1e-6);
    // GNSSTime stores seconds in a double; at a 2023 epoch the representable
    // spacing is about 0.12 microseconds, so keep the oracle tighter than the
    // sensor contract but above that unavoidable timestamp quantisation.
    EXPECT_NEAR(result.first_dt_s, 0.005, 1e-6);
    EXPECT_NEAR(result.last_dt_s, 0.005, 1e-6);
    EXPECT_TRUE(result.dt_tail_repeated);
    ASSERT_EQ(series.samples.size(), 2u);
    EXPECT_EQ(series.samples.front().elapsed_realtime_nanos, 1005000000LL);
    EXPECT_NEAR(series.samples.front().accel_raw.x(), 2.0, 1e-12);

    ImuSeries rejected_series;
    const auto rejected = loadAndroidImuCsv(imu_path.string(), rejected_series, config);
    EXPECT_FALSE(rejected.ok);
    EXPECT_NE(rejected.error.find("requires GNSS elapsed-time anchors"), std::string::npos);

    std::filesystem::remove(gnss_path);
    std::filesystem::remove(imu_path);
}

TEST(AndroidUtcGpsMappingTest, FitsRawHardwareGpsToUtcAndSynchronizesBlankImuClock) {
    constexpr std::int64_t kUtc0 = 1'700'000'000'000;
    constexpr std::int64_t kGps0 = 1'300'000'000'000'000'000LL;
    constexpr std::int64_t kFullBias = -1'299'000'000'000'000'000LL;
    const auto gnss_path =
        std::filesystem::temp_directory_path() / "libgnss_android_utc_gps_mapping_test.csv";
    const std::string gnss_header =
        "MessageType,utcTimeMillis,TimeNanos,FullBiasNanos,BiasNanos,"
        "HardwareClockDiscontinuityCount,ChipsetElapsedRealtimeNanos,Extra";
    // A 0.1 ppm positive clock drift is represented directly in the raw GPS
    // hardware time; no arrival-time or receiver-coordinate column is used.
    writeFile(
        gnss_path,
        gnss_header + "\n" +
            "Raw," + std::to_string(kUtc0) + "," +
            std::to_string(kGps0 + kFullBias) + "," +
            std::to_string(kFullBias) + ",0.0,0,,0\n" +
            "Raw," + std::to_string(kUtc0 + 1000) + "," +
            std::to_string(kGps0 + 1'000'000'100LL + kFullBias) + "," +
            std::to_string(kFullBias) + ",0.0,0,,0\n" +
            "Raw," + std::to_string(kUtc0 + 2000) + "," +
            std::to_string(kGps0 + 2'000'000'200LL + kFullBias) + "," +
            std::to_string(kFullBias) + ",0.0,0,,0\n");

    // The monotonic-anchor parser remains strict: an all-blank elapsed
    // column is not silently converted into a fabricated clock.  The
    // explicit UTC/GPS mapping below is the separate, opt-in contract for
    // this exact source shape.
    std::vector<AndroidGnssTimeAnchor> elapsed_anchors;
    const auto elapsed_result = loadAndroidGnssTimeAnchors(
        gnss_path.string(), elapsed_anchors);
    EXPECT_FALSE(elapsed_result.ok);
    EXPECT_NE(elapsed_result.error.find("timestamps must be non-negative integers"),
              std::string::npos);
    EXPECT_TRUE(elapsed_anchors.empty());

    AndroidGnssUtcGpsMapping mapping;
    const auto mapping_result =
        loadAndroidGnssUtcGpsMapping(gnss_path.string(), mapping);
    ASSERT_TRUE(mapping_result.ok) << mapping_result.error;
    EXPECT_EQ(mapping_result.raw_rows, 3u);
    EXPECT_EQ(mapping.unique_anchors, 3u);
    EXPECT_TRUE(mapping.valid);
    EXPECT_TRUE(mapping.hardware_clock_count_field_present);
    EXPECT_TRUE(mapping.hardware_clock_count_constant);
    EXPECT_NEAR(mapping.slope_nanos_per_ms, 1'000'000.1, 1e-6);
    EXPECT_NEAR(mapping.drift_ppm, 0.1, 1e-6);
    EXPECT_NEAR(mapping.maximum_fit_residual_ms, 0.0, 1e-12);
    EXPECT_NEAR(static_cast<double>(mapping.gpsNanosAtUtc(kUtc0 + 1000) -
                                    static_cast<long double>(kGps0 + 1'000'000'100LL)),
                0.0, 1e-3);

    const auto imu_path =
        std::filesystem::temp_directory_path() / "libgnss_android_utc_gps_imu_test.csv";
    const std::string imu_header =
        "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,"
        "MeasurementZ,BiasX,BiasY,BiasZ";
    // The elapsed clock is intentionally blank.  UTC is the pairing clock,
    // while the output retains -1 rather than inventing elapsed timestamps.
    writeFile(
        imu_path,
        imu_header + "\n" +
            "UncalAccel," + std::to_string(kUtc0) + ",,1,2,3,0,0,0\n" +
            "UncalAccel," + std::to_string(kUtc0 + 10) + ",,3,4,5,0,0,0\n" +
            "UncalAccel," + std::to_string(kUtc0 + 20) + ",,5,6,7,0,0,0\n" +
            "UncalGyro," + std::to_string(kUtc0) + ",,0.1,0.2,0.3,0,0,0\n" +
            "UncalGyro," + std::to_string(kUtc0 + 10) + ",,0.4,0.5,0.6,0,0,0\n" +
            "UncalGyro," + std::to_string(kUtc0 + 20) + ",,0.7,0.8,0.9,0,0,0\n");
    AndroidImuCsvConfig config;
    config.require_gnss_elapsed_anchor = true;
    config.allow_utc_wall_clock_fallback = true;
    ImuSeries series;
    const auto result = loadAndroidImuCsv(
        imu_path.string(), series, config, {}, &mapping);
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_TRUE(result.utc_wall_clock_fallback_applied);
    EXPECT_FALSE(result.gnss_elapsed_anchor_applied);
    EXPECT_FALSE(result.elapsed_clock_preserved);
    EXPECT_EQ(result.utc_mapping_anchors, 3u);
    EXPECT_NEAR(result.utc_mapping_drift_ppm, 0.1, 1e-6);
    EXPECT_EQ(result.first_gyro_elapsed_ns, -1);
    EXPECT_EQ(result.last_gyro_elapsed_ns, -1);
    ASSERT_EQ(series.samples.size(), 3u);
    EXPECT_EQ(series.samples.front().elapsed_realtime_nanos, -1);
    EXPECT_NEAR(result.first_dt_s, 0.010, 1e-6);
    EXPECT_NEAR(result.last_dt_s, 0.010, 1e-6);
    EXPECT_TRUE(result.dt_tail_repeated);

    std::filesystem::remove(gnss_path);
    std::filesystem::remove(imu_path);
}

TEST(AndroidUtcGpsMappingTest, RejectsClockBoundsAndMixedImuDomains) {
    constexpr std::int64_t kUtc0 = 1'700'000'000'000;
    constexpr std::int64_t kFullBias = -1'299'000'000'000'000'000LL;
    const std::string header =
        "MessageType,utcTimeMillis,TimeNanos,FullBiasNanos,BiasNanos,"
        "HardwareClockDiscontinuityCount";
    const auto path = std::filesystem::temp_directory_path() /
                      "libgnss_android_utc_gps_mapping_reject_test.csv";
    const auto make_rows = [&](std::int64_t gap_ms, double slope_ns_per_ms,
                               int count1) {
        const std::int64_t gps0 = 1'300'000'000'000'000'000LL;
        const auto gps_at = [&](std::int64_t delta_ms) {
            return gps0 + static_cast<std::int64_t>(
                std::llround(static_cast<double>(delta_ms) * slope_ns_per_ms));
        };
        return header + "\n" +
               "Raw," + std::to_string(kUtc0) + "," +
               std::to_string(gps_at(0) + kFullBias) + "," +
               std::to_string(kFullBias) + ",0," + std::to_string(count1) + "\n" +
               "Raw," + std::to_string(kUtc0 + gap_ms) + "," +
               std::to_string(gps_at(gap_ms) + kFullBias) + "," +
               std::to_string(kFullBias) + ",0," + std::to_string(count1) + "\n" +
               "Raw," + std::to_string(kUtc0 + 2 * gap_ms) + "," +
               std::to_string(gps_at(2 * gap_ms) + kFullBias) + "," +
               std::to_string(kFullBias) + ",0,0\n";
    };
    AndroidGnssUtcGpsMapping mapping;
    writeFile(path, make_rows(1000, 1'002'000.0, 0));
    auto result = loadAndroidGnssUtcGpsMapping(path.string(), mapping);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("drift/residual"), std::string::npos);

    writeFile(path, make_rows(5001, 1'000'000.0, 0));
    result = loadAndroidGnssUtcGpsMapping(path.string(), mapping);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("gap exceeds 5000"), std::string::npos);

    writeFile(path, make_rows(1000, 1'000'000.0, 1));
    result = loadAndroidGnssUtcGpsMapping(path.string(), mapping);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("discontinuity"), std::string::npos);
    std::filesystem::remove(path);

    const auto imu_path = std::filesystem::temp_directory_path() /
                          "libgnss_android_utc_gps_mixed_domain_test.csv";
    writeFile(
        imu_path,
        "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,"
        "MeasurementZ,BiasX,BiasY,BiasZ\n"
        "UncalAccel,1700000000000,,1,2,3,0,0,0\n"
        "UncalGyro,1700000000000,1000000000,0,0,0,0,0,0\n");
    AndroidImuCsvConfig config;
    config.require_gnss_elapsed_anchor = true;
    config.allow_utc_wall_clock_fallback = true;
    ImuSeries series;
    AndroidGnssUtcGpsMapping valid_mapping;
    valid_mapping.valid = true;
    valid_mapping.unique_anchors = 3;
    valid_mapping.slope_nanos_per_ms = 1'000'000.0;
    valid_mapping.drift_ppm = 0.0;
    const auto mixed = loadAndroidImuCsv(
        imu_path.string(), series, config, {}, &valid_mapping);
    EXPECT_FALSE(mixed.ok);
    EXPECT_NE(mixed.error.find("mixes"), std::string::npos);
    std::filesystem::remove(imu_path);
}

TEST(AndroidUtcGpsMappingTest, AppliesSourceMinusTwentyMsOnlyToFallbackMappedTime) {
    constexpr std::int64_t kUtc0 = 1'700'000'000'000;
    constexpr std::int64_t kGps0 = 1'300'000'000'000'000'000LL;
    constexpr std::int64_t kFullBias = -1'299'000'000'000'000'000LL;
    constexpr double kSlopeNanosPerMs = 1'000'500.0;
    const auto gnss_path =
        std::filesystem::temp_directory_path() / "libgnss_android_utc_offset_mapping_test.csv";
    const std::string gnss_header =
        "MessageType,utcTimeMillis,TimeNanos,FullBiasNanos,BiasNanos,"
        "HardwareClockDiscontinuityCount,ChipsetElapsedRealtimeNanos,Extra";
    const auto raw_row = [&](std::int64_t delta_ms) {
        const auto gps_nanos = kGps0 + static_cast<std::int64_t>(
            std::llround(static_cast<double>(delta_ms) * kSlopeNanosPerMs));
        return "Raw," + std::to_string(kUtc0 + delta_ms) + "," +
               std::to_string(gps_nanos + kFullBias) + "," +
               std::to_string(kFullBias) + ",0,0,,0\n";
    };
    writeFile(gnss_path, gnss_header + "\n" + raw_row(0) + raw_row(1000) + raw_row(2000));
    AndroidGnssUtcGpsMapping mapping;
    const auto mapping_result =
        loadAndroidGnssUtcGpsMapping(gnss_path.string(), mapping);
    ASSERT_TRUE(mapping_result.ok) << mapping_result.error;
    EXPECT_NEAR(mapping.slope_nanos_per_ms, kSlopeNanosPerMs, 1e-6);
    EXPECT_NEAR(mapping.drift_ppm, 500.0, 1e-6);

    const auto imu_path =
        std::filesystem::temp_directory_path() / "libgnss_android_utc_offset_imu_test.csv";
    const std::string imu_header =
        "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,"
        "MeasurementZ,BiasX,BiasY,BiasZ";
    const auto accel_row = [&](std::int64_t delta_ms, int value) {
        return "UncalAccel," + std::to_string(kUtc0 + delta_ms) +
               ",," + std::to_string(value) + ",2,3,0,0,0\n";
    };
    const auto gyro_row = [&](std::int64_t delta_ms, double value) {
        return "UncalGyro," + std::to_string(kUtc0 + delta_ms) +
               ",," + std::to_string(value) + ",0.2,0.3,0,0,0\n";
    };
    writeFile(imu_path, imu_header + "\n" + accel_row(0, 1) + accel_row(10, 2) +
                                accel_row(20, 3) + accel_row(30, 4) +
                                gyro_row(5, 0.1) + gyro_row(15, 0.2) +
                                gyro_row(25, 0.3));

    AndroidImuCsvConfig no_offset;
    no_offset.require_gnss_elapsed_anchor = true;
    no_offset.allow_utc_wall_clock_fallback = true;
    ImuSeries baseline_series;
    const auto baseline = loadAndroidImuCsv(
        imu_path.string(), baseline_series, no_offset, {}, &mapping);
    ASSERT_TRUE(baseline.ok) << baseline.error;
    EXPECT_TRUE(baseline.utc_wall_clock_fallback_applied);
    EXPECT_FALSE(baseline.utc_wall_clock_fallback_offset_requested);
    EXPECT_FALSE(baseline.utc_wall_clock_fallback_offset_applied);

    AndroidImuCsvConfig source_offset = no_offset;
    source_offset.apply_utc_wall_clock_fallback_offset = true;
    source_offset.utc_wall_clock_fallback_offset_ms = -20;
    ImuSeries offset_series;
    const auto shifted = loadAndroidImuCsv(
        imu_path.string(), offset_series, source_offset, {}, &mapping);
    ASSERT_TRUE(shifted.ok) << shifted.error;
    EXPECT_TRUE(shifted.utc_wall_clock_fallback_offset_requested);
    EXPECT_TRUE(shifted.utc_wall_clock_fallback_offset_applied);
    EXPECT_EQ(shifted.utc_wall_clock_fallback_offset_ms, -20);
    EXPECT_EQ(shifted.utc_wall_clock_fallback_effective_offset_ms, -20);
    EXPECT_EQ(shifted.paired_rows, baseline.paired_rows);
    EXPECT_EQ(shifted.interpolated_rows, baseline.interpolated_rows);
    EXPECT_EQ(shifted.endpoint_nearest_rows, baseline.endpoint_nearest_rows);
    EXPECT_EQ(shifted.omitted_rows, baseline.omitted_rows);
    EXPECT_EQ(shifted.first_gyro_utc_ms, baseline.first_gyro_utc_ms);
    EXPECT_EQ(shifted.last_gyro_utc_ms, baseline.last_gyro_utc_ms);
    // GNSSTime stores TOW as double at a 2023-scale epoch; allow its
    // sub-nanosecond representation noise while requiring identical pairing.
    EXPECT_NEAR(shifted.first_dt_s, baseline.first_dt_s, 1e-9);
    EXPECT_NEAR(shifted.last_dt_s, baseline.last_dt_s, 1e-9);
    EXPECT_EQ(shifted.interpolated_rows, 3u);
    ASSERT_EQ(offset_series.samples.size(), baseline_series.samples.size());
    for (std::size_t i = 0; i < baseline_series.samples.size(); ++i) {
        EXPECT_EQ(offset_series.samples[i].elapsed_realtime_nanos,
                  baseline_series.samples[i].elapsed_realtime_nanos);
        EXPECT_TRUE(offset_series.samples[i].accel_raw.isApprox(
            baseline_series.samples[i].accel_raw, 1e-12));
        EXPECT_TRUE(offset_series.samples[i].gyro_raw_radps.isApprox(
            baseline_series.samples[i].gyro_raw_radps, 1e-12));
        EXPECT_NEAR(offset_series.samples[i].time - baseline_series.samples[i].time,
                    -0.02001, 1e-7);
    }
    // The validated map has a non-unit slope: -20 ms maps to -20.01 ms in
    // GPST, proving that the correction is applied before the affine map.
    EXPECT_NEAR(offset_series.samples.front().time -
                    baseline_series.samples.front().time,
                -0.02001, 1e-7);

    // A monotonic elapsed-anchor path wins over the fallback.  Even when the
    // selector is requested, the -20 ms correction is not applied there.
    const auto anchored_path =
        std::filesystem::temp_directory_path() / "libgnss_android_utc_offset_anchor_imu_test.csv";
    writeFile(anchored_path, imu_header + "\n" +
                              "UncalAccel,1800000000000,1000000000,1,2,3,0,0,0\n" +
                              "UncalAccel,1800000000010,1010000000,2,2,3,0,0,0\n" +
                              "UncalGyro,1800000000000,1000000000,0.1,0.2,0.3,0,0,0\n" +
                              "UncalGyro,1800000000010,1010000000,0.2,0.2,0.3,0,0,0\n");
    std::vector<AndroidGnssTimeAnchor> anchors{{1800000000000LL, 1000000000LL},
                                                {1800000000010LL, 1010000000LL}};
    AndroidImuCsvConfig anchored_config = source_offset;
    ImuSeries anchored_series;
    const auto anchored = loadAndroidImuCsv(
        anchored_path.string(), anchored_series, anchored_config, anchors, &mapping);
    ASSERT_TRUE(anchored.ok) << anchored.error;
    AndroidImuCsvConfig anchored_no_offset = anchored_config;
    anchored_no_offset.apply_utc_wall_clock_fallback_offset = false;
    anchored_no_offset.utc_wall_clock_fallback_offset_ms = 0;
    ImuSeries anchored_baseline_series;
    const auto anchored_baseline = loadAndroidImuCsv(
        anchored_path.string(), anchored_baseline_series, anchored_no_offset,
        anchors, &mapping);
    ASSERT_TRUE(anchored_baseline.ok) << anchored_baseline.error;
    EXPECT_TRUE(anchored.gnss_elapsed_anchor_applied);
    EXPECT_FALSE(anchored.utc_wall_clock_fallback_applied);
    EXPECT_TRUE(anchored.utc_wall_clock_fallback_offset_requested);
    EXPECT_FALSE(anchored.utc_wall_clock_fallback_offset_applied);
    EXPECT_EQ(anchored.utc_wall_clock_fallback_offset_ms, -20);
    EXPECT_EQ(anchored.utc_wall_clock_fallback_effective_offset_ms, 0);
    ASSERT_EQ(anchored_series.samples.size(), anchored_baseline_series.samples.size());
    for (std::size_t i = 0; i < anchored_series.samples.size(); ++i) {
        EXPECT_EQ(anchored_series.samples[i].time,
                  anchored_baseline_series.samples[i].time);
        EXPECT_TRUE(anchored_series.samples[i].accel_raw.isApprox(
            anchored_baseline_series.samples[i].accel_raw, 1e-12));
        EXPECT_TRUE(anchored_series.samples[i].gyro_raw_radps.isApprox(
            anchored_baseline_series.samples[i].gyro_raw_radps, 1e-12));
    }

    // Correcting below the representable non-negative UTC domain fails closed
    // without changing the raw pairing stream or emitting partial samples.
    const auto underflow_path =
        std::filesystem::temp_directory_path() / "libgnss_android_utc_offset_underflow_test.csv";
    writeFile(underflow_path, imu_header + "\n" +
                              "UncalAccel,10,,1,2,3,0,0,0\n" +
                              "UncalAccel,20,,2,2,3,0,0,0\n" +
                              "UncalGyro,10,,0.1,0.2,0.3,0,0,0\n" +
                              "UncalGyro,20,,0.2,0.2,0.3,0,0,0\n");
    AndroidGnssUtcGpsMapping low_mapping = mapping;
    low_mapping.reference_utc_time_ms = 0;
    ImuSeries underflow_series;
    const auto underflow = loadAndroidImuCsv(
        underflow_path.string(), underflow_series, source_offset, {}, &low_mapping);
    EXPECT_FALSE(underflow.ok);
    EXPECT_NE(underflow.error.find("underflows timestamp"), std::string::npos);
    EXPECT_TRUE(underflow_series.samples.empty());

    AndroidGnssUtcGpsMapping invalid_mapping = mapping;
    invalid_mapping.valid = false;
    ImuSeries invalid_series;
    const auto invalid_mapping_result = loadAndroidImuCsv(
        imu_path.string(), invalid_series, source_offset, {}, &invalid_mapping);
    EXPECT_FALSE(invalid_mapping_result.ok);
    EXPECT_NE(invalid_mapping_result.error.find("validated GNSS mapping"),
              std::string::npos);

    std::filesystem::remove(gnss_path);
    std::filesystem::remove(imu_path);
    std::filesystem::remove(anchored_path);
    std::filesystem::remove(underflow_path);
}

TEST(AndroidUtcGpsMappingTest, RejectsMatBeforeOpeningFile) {
    AndroidGnssUtcGpsMapping mapping;
    const auto result = loadAndroidGnssUtcGpsMapping(
        (std::filesystem::temp_directory_path() / "mapping.mat").string(), mapping);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("MATLAB .mat inputs are forbidden"), std::string::npos);
}

TEST(AndroidImuCsvTest, AppliesPairBoundAtBoundaryAndOmitsBeyondIt) {
    const std::string header =
        "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,"
        "MeasurementZ,BiasX,BiasY,BiasZ";
    const auto path =
        std::filesystem::temp_directory_path() / "libgnss_android_imu_pair_bound_test.csv";
    writeFile(
        path,
        header + "\n"
                 "UncalAccel,1700000000000,1000000000,1,2,3,0,0,0\n"
                 "UncalAccel,1700000000100,1100000000,4,5,6,0,0,0\n"
                 // 25 ms is accepted by the frozen default bound.
                 "UncalGyro,1700000000025,1025000000,0,0,0,0,0,0\n"
                 // 26 ms is omitted, while the loader remains successful.
                 "UncalGyro,1700000000126,1126000000,0,0,0,0,0,0\n");

    ImuSeries series;
    const auto result = loadAndroidImuCsv(path.string(), series);
    std::filesystem::remove(path);

    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.gyro_rows, 2u);
    EXPECT_EQ(result.paired_rows, 1u);
    EXPECT_EQ(result.interpolated_rows, 1u);
    EXPECT_EQ(result.omitted_rows, 1u);
    ASSERT_EQ(series.samples.size(), 1u);
    EXPECT_NEAR(series.samples.front().accel_raw.x(), 1.75, 1e-12);
}

TEST(AndroidImuCsvTest, RejectsNonFiniteAndNonMonotonicSupportedRows) {
    const std::string header =
        "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,"
        "MeasurementZ,BiasX,BiasY,BiasZ";
    const auto nonfinite_path =
        std::filesystem::temp_directory_path() / "libgnss_android_imu_nonfinite_test.csv";
    writeFile(
        nonfinite_path,
        header + "\n"
                 "UncalAccel,1700000000000,1000000000,nan,2,3,0,0,0\n"
                 "UncalGyro,1700000000000,1000000000,0,0,0,0,0,0\n");
    ImuSeries series;
    auto result = loadAndroidImuCsv(nonfinite_path.string(), series);
    std::filesystem::remove(nonfinite_path);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("non-finite"), std::string::npos);
    EXPECT_TRUE(series.samples.empty());

    const auto ordering_path =
        std::filesystem::temp_directory_path() / "libgnss_android_imu_ordering_test.csv";
    writeFile(
        ordering_path,
        header + "\n"
                 "UncalAccel,1700000000000,1000000000,1,2,3,0,0,0\n"
                 "UncalAccel,1700000000010,1000000000,1,2,3,0,0,0\n"
                 "UncalGyro,1700000000000,1000000000,0,0,0,0,0,0\n");
    result = loadAndroidImuCsv(ordering_path.string(), series);
    std::filesystem::remove(ordering_path);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("strictly increasing"), std::string::npos);
    EXPECT_TRUE(series.samples.empty());
}

TEST(AndroidImuCsvTest, RejectsMatInputsBeforeOpeningAnyFile) {
    ImuSeries series;
    const auto result = loadAndroidImuCsv(
        (std::filesystem::temp_directory_path() / "android_input.mat").string(), series);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("MATLAB .mat inputs are forbidden"), std::string::npos);
    EXPECT_TRUE(series.samples.empty());
}

TEST(AndroidImuCsvTest, RejectsMatGnssAnchorInputsBeforeOpeningAnyFile) {
    std::vector<AndroidGnssTimeAnchor> anchors;
    const auto result = loadAndroidGnssTimeAnchors(
        (std::filesystem::temp_directory_path() / "gnss_anchor_input.mat").string(),
        anchors);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("MATLAB .mat inputs are forbidden"), std::string::npos);
    EXPECT_TRUE(anchors.empty());
}

TEST(ImuCsvTest, ParsesRtklibExplorerSelfFormattedCsv) {
    const auto path =
        std::filesystem::temp_directory_path() / "libgnss_imu_rtklibexplorer_test.csv";
    writeFile(
        path,
        "% UNIX time(s),    accX(g),  accY(g),  accZ(g),  gyroX(r/s),gyroY(r/s),"
        "gyroZ(r/s),magX(uT),magY(uT),magZ(uT), unused\n"
        "1752003261.8540001,0.1,-0.2,1.0,-0.01,0.02,-0.03,0,0,0,0\n");

    ImuSeries series;
    const auto result = loadRtklibExplorerImuCsv(path.string(), series);
    std::filesystem::remove(path);

    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(series.samples.size(), 1u);
    EXPECT_EQ(series.samples[0].time.week, 2374);
    EXPECT_NEAR(series.samples[0].time.tow, 243261.854, 1e-6);
    EXPECT_NEAR(series.samples[0].accel_raw.x(), 0.980665, 1e-12);
    EXPECT_NEAR(series.samples[0].accel_raw.y(), -1.96133, 1e-12);
    EXPECT_NEAR(series.samples[0].accel_raw.z(), 9.80665, 1e-12);
    EXPECT_NEAR(series.samples[0].gyro_raw_radps.x(), -0.01, 1e-12);
    EXPECT_NEAR(series.samples[0].gyro_raw_radps.y(), 0.02, 1e-12);
    EXPECT_NEAR(series.samples[0].gyro_raw_radps.z(), -0.03, 1e-12);
}

TEST(ImuAxisConventionTest, IdentityMappingIsPassthrough) {
    ImuAxisConvention convention;
    const Eigen::Vector3d raw(1.0, 2.0, 3.0);
    const Eigen::Vector3d body = convention.apply(raw);
    EXPECT_NEAR(body(0), 1.0, 1e-12);
    EXPECT_NEAR(body(1), 2.0, 1e-12);
    EXPECT_NEAR(body(2), 3.0, 1e-12);
}

TEST(ImuAxisConventionTest, RemapsAndFlipsAxes) {
    // FRD -> FLU: keep forward, flip left/up (as RTKLIB/robotics convention
    // would require for a Front-Right-Down sensor mount).
    ImuAxisConvention convention;
    convention.forward_source_axis = 0;
    convention.forward_sign = 1.0;
    convention.left_source_axis = 1;
    convention.left_sign = -1.0;
    convention.up_source_axis = 2;
    convention.up_sign = -1.0;

    const Eigen::Vector3d raw(1.0, 2.0, 3.0);
    const Eigen::Vector3d body = convention.apply(raw);
    EXPECT_NEAR(body(0), 1.0, 1e-12);
    EXPECT_NEAR(body(1), -2.0, 1e-12);
    EXPECT_NEAR(body(2), -3.0, 1e-12);
}

TEST(ImuSeriesTest, GetSamplesFiltersByInclusiveTimeRange) {
    ImuSeries series;
    for (int i = 0; i < 5; ++i) {
        ImuSample sample;
        sample.time = GNSSTime(2000, 100.0 + i);
        series.samples.push_back(sample);
    }
    const auto filtered = series.getSamples(GNSSTime(2000, 101.0), GNSSTime(2000, 103.0));
    ASSERT_EQ(filtered.size(), 3u);
    EXPECT_NEAR(filtered.front().time.tow, 101.0, 1e-9);
    EXPECT_NEAR(filtered.back().time.tow, 103.0, 1e-9);
}

TEST(ImuSeriesTest, ShiftTimeShiftsAllSamples) {
    ImuSeries series;
    for (int i = 0; i < 3; ++i) {
        ImuSample sample;
        sample.time = GNSSTime(2000, 100.0 + i);
        series.samples.push_back(sample);
    }
    series.shiftTime(0.25);
    EXPECT_NEAR(series.samples[0].time.tow, 100.25, 1e-12);
    EXPECT_NEAR(series.samples[2].time.tow, 102.25, 1e-12);
    series.shiftTime(-0.5);
    EXPECT_NEAR(series.samples[0].time.tow, 99.75, 1e-12);
    EXPECT_EQ(series.samples[0].time.week, 2000);
}

TEST(ImuSeriesTest, ShiftTimeHandlesWeekRollover) {
    ImuSeries series;
    ImuSample near_end;
    near_end.time = GNSSTime(2000, 604799.9);
    ImuSample near_start;
    near_start.time = GNSSTime(2000, 0.05);
    series.samples.push_back(near_end);
    series.samples.push_back(near_start);

    series.shiftTime(0.2);  // pushes near_end across the rollover
    EXPECT_EQ(series.samples[0].time.week, 2001);
    EXPECT_NEAR(series.samples[0].time.tow, 0.1, 1e-6);

    series.shiftTime(-0.3);  // pulls near_start (now 0.25) back across
    EXPECT_EQ(series.samples[0].time.week, 2000);
    EXPECT_NEAR(series.samples[0].time.tow, 604799.8, 1e-6);
    EXPECT_EQ(series.samples[1].time.week, 1999);
}

TEST(ImuSeriesTest, ShiftTimeZeroIsIdentity) {
    ImuSeries series;
    ImuSample sample;
    sample.time = GNSSTime(2000, 123.456);
    series.samples.push_back(sample);
    series.shiftTime(0.0);
    EXPECT_EQ(series.samples[0].time.week, 2000);
    EXPECT_DOUBLE_EQ(series.samples[0].time.tow, 123.456);
}

}  // namespace
}  // namespace libgnss
