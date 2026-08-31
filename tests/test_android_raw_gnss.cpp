#include <gtest/gtest.h>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>

#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <unistd.h>

namespace libgnss::io {
namespace {

constexpr const char* kHeader =
    "MessageType,utcTimeMillis,TimeNanos,FullBiasNanos,BiasNanos,"
    "HardwareClockDiscontinuityCount,Svid,ConstellationType,"
    "ReceivedSvTimeNanos,PseudorangeRateMetersPerSecond,"
    "AccumulatedDeltaRangeState,AccumulatedDeltaRangeMeters,"
    "CarrierFrequencyHz,Cn0DbHz,SignalType,RawPseudorangeMeters\n";

std::filesystem::path fixturePath(const char* stem) {
    return std::filesystem::temp_directory_path() /
           (std::string("libgnss_android_raw_") + stem + "_" +
            std::to_string(static_cast<long long>(::getpid())) + ".csv");
}

void writeRow(std::ostream& output,
              std::int64_t utc_millis,
              std::int64_t time_nanos,
              std::int64_t full_bias_nanos,
              std::int64_t received_sv_time_nanos,
              int svid,
              int constellation,
              double pseudorange_rate,
              int adr_state,
              double adr_m,
              double frequency_hz,
              double cn0,
              const char* signal,
              double raw_pseudorange = 0.0,
              bool include_raw_pseudorange = false) {
    output << std::setprecision(17)
           << "Raw," << utc_millis << ',' << time_nanos << ',' << full_bias_nanos
           << ",0,7," << svid << ',' << constellation << ','
           << received_sv_time_nanos << ',' << pseudorange_rate << ','
           << adr_state << ',' << adr_m << ',' << frequency_hz << ',' << cn0 << ','
           << signal;
    output << ',';
    if (include_raw_pseudorange) output << raw_pseudorange;
    output << '\n';
}

TEST(AndroidRawGnssTest, ReconstructsRawClockAndObservableSigns) {
    const auto path = fixturePath("signs");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double tow = 100'000.123;
    const auto time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + tow) * 1.0e9L);
    constexpr double transmit_delay = 0.070;
    const auto transmit_nanos = static_cast<std::int64_t>(
        (tow - transmit_delay) * 1.0e9);
    const long double gps_seconds =
        (static_cast<long double>(time_nanos) -
         static_cast<long double>(full_bias)) /
        1.0e9L;
    const long double week_start =
        std::floor(gps_seconds / 604'800.0L) * 604'800.0L;
    const double pseudorange = static_cast<double>(
        (gps_seconds - week_start -
         static_cast<long double>(transmit_nanos) / 1.0e9L) *
        constants::SPEED_OF_LIGHT);

    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 3, 1, 25.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 42.0, "GPS_L1_CA", pseudorange, true);
    }

    AndroidRawGnssResult result;
    AndroidRawGnssConfig config;
    config.device_model = "sm-a205u";
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), config, result, error)) << error;
    ASSERT_EQ(result.observations.epochs.size(), 1u);
    ASSERT_EQ(result.observations.epochs.front().observations.size(), 1u);
    const auto& observation = result.observations.epochs.front().observations.front();
    const double wavelength = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;
    EXPECT_EQ(observation.satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(observation.satellite.prn, 3);
    EXPECT_NEAR(result.observations.epochs.front().time.tow, tow, 1e-9);
    EXPECT_NEAR(observation.pseudorange, pseudorange, 1e-5);
    EXPECT_NEAR(observation.doppler, -25.0 / wavelength, 1e-12);
    EXPECT_TRUE(observation.has_carrier_phase);
    EXPECT_NEAR(observation.carrier_phase, -42.0 / wavelength, 1e-12);
    EXPECT_EQ(result.diagnostics.enriched_pseudorange_checks, 1u);
    EXPECT_EQ(result.diagnostics.enriched_pseudorange_mismatches, 0u);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, KeepsGpsAndGalileoE1AndSkipsOtherSignals) {
    const auto path = fixturePath("signals");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr std::int64_t time_nanos = 30'660'000'070'000'000LL;
    constexpr std::int64_t transmit_nanos = 100'000'000'000'000LL;
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 3, 1, 0.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 5, 6, 0.0, 1, 42.0,
                 constants::GAL_E1_FREQ, 40.0, "GAL_E1_C_P");
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 7, 3, 0.0, 1, 42.0,
                 constants::GPS_L2_FREQ, 40.0, "GPS_L2C");
    }

    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{}, result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 1u);
    EXPECT_EQ(result.observations.epochs.front().observations.size(), 2u);
    EXPECT_EQ(result.diagnostics.gps_rows, 1u);
    EXPECT_EQ(result.diagnostics.galileo_e1_rows, 1u);
    EXPECT_EQ(result.diagnostics.skipped_unsupported_signal_rows, 1u);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, DuplicateSupportedRowsFailClosed) {
    const auto path = fixturePath("duplicate");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr std::int64_t time_nanos = 30'660'000'070'000'000LL;
    constexpr std::int64_t transmit_nanos = 100'000'000'000'000LL;
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        for (int index = 0; index < 2; ++index) {
            writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                     transmit_nanos, 3, 1, 0.0, 1, 42.0,
                     constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
        }
    }
    AndroidRawGnssResult result;
    std::string error;
    EXPECT_FALSE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{}, result, error));
    EXPECT_NE(error.find("duplicate"), std::string::npos);
    std::filesystem::remove(path);
}

}  // namespace
}  // namespace libgnss::io
