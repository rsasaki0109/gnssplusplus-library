#include <gtest/gtest.h>

#include <libgnss++/io/imu.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>

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
