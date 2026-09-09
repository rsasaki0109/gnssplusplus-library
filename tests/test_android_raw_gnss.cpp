#include <gtest/gtest.h>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/algorithms/source_transmission_clock.hpp>

#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>

namespace libgnss::io {
namespace {

constexpr const char* kHeader =
    "MessageType,utcTimeMillis,TimeNanos,FullBiasNanos,BiasNanos,"
    "BiasUncertaintyNanos,TimeOffsetNanos,HardwareClockDiscontinuityCount,Svid,ConstellationType,"
    "ReceivedSvTimeNanos,PseudorangeRateMetersPerSecond,"
    "AccumulatedDeltaRangeState,AccumulatedDeltaRangeMeters,"
    "CarrierFrequencyHz,Cn0DbHz,SignalType,ReceivedSvTimeUncertaintyNanos,"
    "RawPseudorangeMeters\n";

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
              bool include_raw_pseudorange = false,
              double time_offset_nanos = 0.0,
              double bias_uncertainty_nanos = 0.0,
              double received_sv_time_uncertainty_nanos =
                  std::numeric_limits<double>::quiet_NaN()) {
    output << std::setprecision(17)
           << "Raw," << utc_millis << ',' << time_nanos << ',' << full_bias_nanos
           << ",0," << bias_uncertainty_nanos << ',' << time_offset_nanos
           << ",7," << svid << ',' << constellation << ','
           << received_sv_time_nanos << ',' << pseudorange_rate << ','
           << adr_state << ',' << adr_m << ',' << frequency_hz << ',' << cn0 << ','
           << signal;
    output << ',';
    if (std::isfinite(received_sv_time_uncertainty_nanos)) {
        output << received_sv_time_uncertainty_nanos;
    }
    output << ',';
    if (include_raw_pseudorange) output << raw_pseudorange;
    output << '\n';
}

TEST(AndroidRawGnssTest, OptionalAdrUncertaintyDoesNotChangeAdmissionOrCarrierUnits) {
    constexpr std::int64_t full_bias=-1'300'000'000'000'000'000LL;
    constexpr std::int64_t time_nanos=full_bias+(2200LL*604800+100000)*1000000000LL;
    constexpr std::int64_t transmit_nanos=100000LL*1000000000LL-70000000LL;
    for(const std::string token:{"absent","","0","-1","nan","inf","bad","0.003"}) {
      for(int adr_state:{0,1,3,5}) {
        SCOPED_TRACE(token+" ADR state="+std::to_string(adr_state));
        const auto path=fixturePath("adr_uncertainty");
        {
            std::ofstream out(path);
            std::string header=kHeader;
            if(token!="absent") header.insert(header.size()-1,",AccumulatedDeltaRangeUncertaintyMeters");
            out<<header;
            std::ostringstream row;
            writeRow(row,1700000000000LL,time_nanos,full_bias,transmit_nanos,
                     3,1,0.,adr_state,42.,constants::GPS_L1_FREQ,40.,"GPS_L1_CA");
            std::string text=row.str();
            if(token!="absent") text.insert(text.size()-1,","+token);
            out<<text;
        }
        AndroidRawGnssResult result;
        std::string error;
        ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(),AndroidRawGnssConfig{},result,error))<<token<<error;
        ASSERT_EQ(result.observations.epochs.size(),1U);
        ASSERT_EQ(result.observations.epochs[0].observations.size(),1U);
        const auto& obs=result.observations.epochs[0].observations.front();
        EXPECT_EQ(obs.has_carrier_phase,adr_state==1);
        if(adr_state==1)
            EXPECT_NEAR(std::abs(obs.carrier_phase)*constants::GPS_L1_WAVELENGTH,42.,1e-10);
        if(adr_state&6) EXPECT_NE(obs.lli,0);
        EXPECT_EQ(obs.has_source_adr_uncertainty_m,token=="0.003");
        EXPECT_DOUBLE_EQ(obs.source_adr_uncertainty_m,token=="0.003"?.003:0.);
        std::filesystem::remove(path);
      }
    }
}

TEST(AndroidRawGnssTest, FrequencyPairTimingOptInRejectsOffsetsAndClockMismatch) {
    constexpr std::int64_t full_bias=-1'300'000'000'000'000'000LL;
    constexpr std::int64_t time_nanos=full_bias+(2200LL*604800+100000)*1000000000LL;
    for (int constellation : {1, 6}) {
    for (int mode=0; mode<4; ++mode) {
        SCOPED_TRACE("constellation=" + std::to_string(constellation) +
                     " mode=" + std::to_string(mode));
        const auto path=fixturePath("paired_timing");
        {
            std::ofstream output(path);
            ASSERT_TRUE(output.is_open());
            output << kHeader;
            writeRow(output,1700000000000LL,time_nanos,full_bias,99999930000000LL,
                     3,constellation,0,1,42,constants::GPS_L1_FREQ,40,
                     constellation==1 ? "GPS_L1_CA" : "GAL_E1");
            std::ostringstream second;
            writeRow(second,1700000000000LL,time_nanos,full_bias+(mode==2 ? 1 : 0),
                     99999929000000LL,3,constellation,0,1,42,constants::GPS_L5_FREQ,40,
                     constellation==1 ? "GPS_L5" : "GAL_E5A",
                     0.,false,mode==1 ? 1. : 0.);
            std::string row = second.str();
            if (mode==3) {
                // Remove only field 6 (TimeOffsetNanos), preserving CSV columns.
                std::size_t start=0;
                for (int field=0; field<6; ++field) start=row.find(',',start)+1;
                row.erase(start,row.find(',',start)-start);
            }
            output << row;
        }
        AndroidRawGnssConfig config;
        config.verify_enriched_pseudorange=false;
        AndroidRawGnssResult baseline,result;
        std::string error;
        EXPECT_TRUE(loadAndroidRawGnssCsv(path.string(),config,baseline,error)) << error;
        config.require_frequency_pair_timing=true;
        const bool loaded=loadAndroidRawGnssCsv(path.string(),config,result,error);
        std::filesystem::remove(path);
        EXPECT_EQ(loaded,mode==0) << error;
        if (mode!=0) EXPECT_NE(error.find("paired TDCP raw timing"),std::string::npos);
        else EXPECT_EQ(result.diagnostics.selected_rows,baseline.diagnostics.selected_rows);
    }
    }
}

TEST(AndroidRawGnssTest, TransmissionSelectionConsumesParserCodeMask) {
    const auto path = fixturePath("transmission_mask");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr std::int64_t time_nanos =
        full_bias + (2200LL * 604800 + 100000) * 1000000000LL;
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        // Both codes exist in the same raw epoch. Only L1 fails exobs SNR.
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 99999930000000LL, 3, 1, 0, 1, 42,
                 constants::GPS_L1_FREQ, 19, "GPS_L1_CA");
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 99999929000000LL, 3, 1, 0, 1, 42,
                 constants::GPS_L5_FREQ, 40, "GPS_L5");
    }
    AndroidRawGnssConfig config;
    config.verify_enriched_pseudorange = false;
    AndroidRawGnssResult result;
    std::string error;
    const bool loaded = loadAndroidRawGnssCsv(path.string(), config, result, error);
    std::filesystem::remove(path);
    ASSERT_TRUE(loaded) << error;
    ASSERT_EQ(result.observations.epochs.size(), 1U);
    const auto& observations = result.observations.epochs.front().observations;
    ASSERT_EQ(observations.size(), 2U);
    std::size_t masked = 0;
    for (const auto& observation : observations) {
        if (observation.signal == SignalType::GPS_L1CA) {
            EXPECT_TRUE(observation.raw_code_masked);
            EXPECT_FALSE(observation.has_pseudorange);
            ++masked;
        }
    }
    EXPECT_EQ(masked, 1U);
    const auto selected = source_transmission_clock::selectNativeEpoch(observations);
    ASSERT_EQ(selected.size(), 1U);
    EXPECT_EQ(selected.at(SatelliteId(GNSSSystem::GPS, 3)).slot,
              source_transmission_clock::Slot::L5);
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
    ASSERT_EQ(result.epoch_utc_time_millis.size(), 1u);
    EXPECT_EQ(result.epoch_utc_time_millis.front(), 1'700'000'000'000LL);
    EXPECT_EQ(result.observations.epochs.front().raw_source_index, 0U);
    EXPECT_EQ(result.observations.epochs.front().raw_utc_time_millis,
              1'700'000'000'000LL);
    ASSERT_EQ(result.observations.epochs.front().observations.size(), 1u);
    const auto& observation = result.observations.epochs.front().observations.front();
    const double wavelength = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;
    EXPECT_EQ(observation.satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(observation.satellite.prn, 3);
    EXPECT_NEAR(result.observations.epochs.front().time.tow, tow, 1e-9);
    EXPECT_NEAR(observation.pseudorange, pseudorange, 1e-5);
    EXPECT_TRUE(observation.has_pseudorange_rate_mps);
    EXPECT_DOUBLE_EQ(observation.pseudorange_rate_mps, 25.0);
    EXPECT_TRUE(observation.has_source_carrier_frequency_hz);
    EXPECT_DOUBLE_EQ(observation.source_carrier_frequency_hz,
                     constants::GPS_L1_FREQ);
    EXPECT_NEAR(observation.doppler, -25.0 / wavelength, 1e-12);
    EXPECT_NEAR(observation.pseudorange_rate_mps,
                -observation.doppler * wavelength, 1e-12);
    EXPECT_TRUE(observation.has_carrier_phase);
    EXPECT_NEAR(observation.carrier_phase, -42.0 / wavelength, 1e-12);
    EXPECT_EQ(result.diagnostics.enriched_pseudorange_checks, 1u);
    EXPECT_EQ(result.diagnostics.enriched_pseudorange_mismatches, 0u);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, ConvertsReceivedSvTimeUncertaintyToMetresAndFailsClosed) {
    const auto path = fixturePath("sv_time_uncertainty");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double tow = 100'000.123;
    const auto time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + tow) * 1.0e9L);
    const auto transmit_nanos = static_cast<std::int64_t>((tow - 0.070) * 1.0e9);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 3, 1, 0.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA", 0.0, false,
                 0.0, 0.0, 10.0);
        writeRow(output, 1'700'000'001'000LL, time_nanos + 1'000'000LL,
                 full_bias, transmit_nanos + 1'000'000LL, 4, 1, 0.0, 1,
                 42.0, constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA", 0.0,
                 false, 0.0, 0.0, 0.0);
        writeRow(output, 1'700'000'002'000LL, time_nanos + 2'000'000LL,
                 full_bias, transmit_nanos + 2'000'000LL, 5, 1, 0.0, 1,
                 42.0, constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA", 0.0,
                 false, 0.0, 0.0, -1.0);
        // A textual non-finite value is accepted as unavailable and must not
        // make the complete raw row fail.
        output << std::setprecision(17)
               << "Raw," << 1'700'000'003'000LL << ',' << time_nanos + 3'000'000LL
               << ',' << full_bias << ",0,0,0,7,6,1,"
               << transmit_nanos + 3'000'000LL << ",0,1,42,"
               << constants::GPS_L1_FREQ << ",40,GPS_L1_CA,nan,\n";
    }
    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{},
                                      result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 4U);
    const auto& valid = result.observations.epochs[0].observations.front();
    EXPECT_TRUE(valid.has_received_sv_time_uncertainty_m);
    EXPECT_NEAR(valid.received_sv_time_uncertainty_m,
                10.0e-9 * constants::SPEED_OF_LIGHT, 1e-12);
    for (std::size_t i = 1; i < result.observations.epochs.size(); ++i) {
        const auto& unavailable = result.observations.epochs[i].observations.front();
        EXPECT_FALSE(unavailable.has_received_sv_time_uncertainty_m);
        EXPECT_DOUBLE_EQ(unavailable.received_sv_time_uncertainty_m, 0.0);
    }
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, RawClockOnlyIgnoresMalformedEnrichedPseudorange) {
    const auto path = fixturePath("raw_clock_only");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double tow = 100'000.123;
    const auto time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + tow) * 1.0e9L);
    const auto transmit_nanos = static_cast<std::int64_t>((tow - 0.070) * 1.0e9);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        std::ostringstream malformed;
        writeRow(malformed, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 3, 1, 0.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA", 1.0, true);
        std::string row = malformed.str();
        const auto comma = row.rfind(',');
        ASSERT_NE(comma, std::string::npos);
        row.replace(comma + 1U, row.size() - comma - 2U, "not-a-number");
        output << row;
    }

    AndroidRawGnssResult strict_result;
    std::string error;
    EXPECT_FALSE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{},
                                       strict_result, error));
    EXPECT_NE(error.find("RawPseudorangeMeters"), std::string::npos);

    AndroidRawGnssResult raw_only_result;
    AndroidRawGnssConfig raw_only;
    raw_only.verify_enriched_pseudorange = false;
    error.clear();
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), raw_only, raw_only_result,
                                      error))
        << error;
    ASSERT_EQ(raw_only_result.observations.epochs.size(), 1U);
    ASSERT_EQ(raw_only_result.observations.epochs.front().observations.size(), 1U);
    EXPECT_TRUE(raw_only_result.diagnostics.enriched_pseudorange_input_ignored);
    EXPECT_EQ(raw_only_result.diagnostics.enriched_pseudorange_ignored_rows, 1U);
    EXPECT_EQ(raw_only_result.diagnostics.enriched_pseudorange_checks, 0U);
    EXPECT_TRUE(std::isfinite(
        raw_only_result.observations.epochs.front().observations.front().pseudorange));
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, RawClockOnlyIgnoresDisagreeingEnrichedPseudorange) {
    const auto path = fixturePath("raw_clock_mismatch");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double tow = 100'000.123;
    const auto time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + tow) * 1.0e9L);
    const auto transmit_nanos = static_cast<std::int64_t>((tow - 0.070) * 1.0e9);
    const long double gps_seconds =
        (static_cast<long double>(time_nanos) -
         static_cast<long double>(full_bias)) /
        1.0e9L;
    const long double week_start =
        std::floor(gps_seconds / 604'800.0L) * 604'800.0L;
    const double raw_clock_pseudorange = static_cast<double>(
        (gps_seconds - week_start -
         static_cast<long double>(transmit_nanos) / 1.0e9L) *
        constants::SPEED_OF_LIGHT);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        // One metre is deliberately beyond the fixed 0.05 m strict
        // diagnostic tolerance.  It is not an estimator input.
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 3, 1, 0.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA",
                 raw_clock_pseudorange + 1.0, true);
    }

    AndroidRawGnssResult strict_result;
    std::string error;
    EXPECT_FALSE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{},
                                       strict_result, error));
    EXPECT_NE(error.find("raw-clock pseudorange disagrees"), std::string::npos);
    EXPECT_EQ(strict_result.diagnostics.enriched_pseudorange_checks, 1U);
    EXPECT_EQ(strict_result.diagnostics.enriched_pseudorange_mismatches, 1U);

    AndroidRawGnssResult raw_only_result;
    AndroidRawGnssConfig raw_only;
    raw_only.verify_enriched_pseudorange = false;
    error.clear();
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), raw_only, raw_only_result,
                                      error))
        << error;
    ASSERT_EQ(raw_only_result.observations.epochs.size(), 1U);
    ASSERT_EQ(raw_only_result.observations.epochs.front().observations.size(), 1U);
    EXPECT_TRUE(raw_only_result.diagnostics.enriched_pseudorange_input_ignored);
    EXPECT_EQ(raw_only_result.diagnostics.enriched_pseudorange_ignored_rows, 1U);
    // Raw-clock-only mode does not parse the optional enriched field at all;
    // it records the column as ignored rather than checking or comparing it.
    EXPECT_EQ(raw_only_result.diagnostics.enriched_pseudorange_checks, 0U);
    EXPECT_EQ(raw_only_result.diagnostics.enriched_pseudorange_mismatches, 0U);
    EXPECT_NEAR(raw_only_result.observations.epochs.front()
                    .observations.front()
                    .pseudorange,
                raw_clock_pseudorange, 1e-5);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, UnwrapsTransmitWeekAcrossBoundaryWithExactNanoseconds) {
    const auto path = fixturePath("week_boundary");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double first_tow = 604'799.950;
    constexpr double second_tow = 0.050;
    const auto first_time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + first_tow) * 1.0e9L);
    const auto second_time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week + 1) * 604'800.0L + second_tow) * 1.0e9L);
    const auto first_transmit_nanos =
        static_cast<std::int64_t>((first_tow - 0.070) * 1.0e9);
    // The second transmit time is in the preceding week while the receiver
    // TOW has wrapped to the start of the next week.
    const auto second_transmit_nanos =
        static_cast<std::int64_t>((604'799.980) * 1.0e9);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        writeRow(output, 1'700'000'000'000LL, first_time_nanos, full_bias,
                 first_transmit_nanos, 3, 1, 0.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
        writeRow(output, 1'700'000'001'000LL, second_time_nanos, full_bias,
                 second_transmit_nanos, 4, 1, 0.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
    }
    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{},
                                      result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 2U);
    ASSERT_EQ(result.observations.epochs[0].observations.size(), 1U);
    ASSERT_EQ(result.observations.epochs[1].observations.size(), 1U);
    const double expected = 0.070 * constants::SPEED_OF_LIGHT;
    EXPECT_NEAR(result.observations.epochs[0].observations.front().pseudorange,
                expected, 0.03);
    EXPECT_NEAR(result.observations.epochs[1].observations.front().pseudorange,
                expected, 0.03);
    EXPECT_EQ(result.observations.epochs[0].time.week, week);
    EXPECT_EQ(result.observations.epochs[1].time.week, week + 1);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, ResetsRawClockSegmentOnlyAfterStrictlyGreaterOneSecond) {
    const auto path = fixturePath("clock_segment_reset");
    constexpr std::int64_t full_bias0 = -1'300'000'000'000'000'000LL;
    constexpr std::int64_t full_bias1 = full_bias0 + 1'000;
    constexpr int week = 2200;
    constexpr double tow0 = 100'000.123;
    const auto time0 = static_cast<std::int64_t>(
        static_cast<long double>(full_bias0) +
        (static_cast<long double>(week) * 604'800.0L + tow0) * 1.0e9L);
    const auto time1 = time0 + 1'000'000'000LL;
    const auto time2 = time1 + 1'000'000'001LL;
    const auto tx0 = static_cast<std::int64_t>((tow0 - 0.070) * 1.0e9);
    const auto tx1 = static_cast<std::int64_t>((tow0 + 1.000001 - 0.070) * 1.0e9);
    const long double gps2 =
        (static_cast<long double>(time2) - static_cast<long double>(full_bias1)) /
        1.0e9L;
    const long double tow2 =
        gps2 - std::floor(gps2 / 604'800.0L) * 604'800.0L;
    const auto tx2 = static_cast<std::int64_t>((tow2 - 0.070L) * 1.0e9L);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        writeRow(output, 1'700'000'000'000LL, time0, full_bias0, tx0, 3, 1,
                 0.0, 1, 42.0, constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
        writeRow(output, 1'700'000'001'000LL, time1, full_bias1, tx1, 4, 1,
                 0.0, 1, 42.0, constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
        // The 1,000,000,001 ns gap is deliberately just above the upstream
        // strict threshold and must start a new FullBias segment.
        writeRow(output, 1'700'000'002'000LL, time2, full_bias1, tx2, 5, 1,
                 0.0, 1, 42.0, constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
    }
    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{},
                                      result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 3U);
    EXPECT_NEAR(result.observations.epochs[0].receiver_clock_bias, 0.0, 1e-15);
    EXPECT_NEAR(result.observations.epochs[1].receiver_clock_bias, 1.0e-6, 1e-15);
    EXPECT_NEAR(result.observations.epochs[2].receiver_clock_bias, 0.0, 1e-15);
    EXPECT_EQ(result.diagnostics.clock_discontinuities, 1U);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, RetainsRawReceiverClockDriftInEpoch) {
    const auto path = fixturePath("clock_drift");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double tow = 100'000.123;
    const auto time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + tow) * 1.0e9L);
    const auto transmit_nanos = static_cast<std::int64_t>(
        (tow - 0.070) * 1.0e9);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        // DriftNanosPerSecond is optional in the Android logger schema.  Keep
        // it explicit in this fixture so the adapter's m/s conversion is
        // tested at the same boundary used by the residual screen.
        output << "MessageType,utcTimeMillis,TimeNanos,FullBiasNanos,BiasNanos,"
                  "DriftNanosPerSecond,BiasUncertaintyNanos,TimeOffsetNanos,"
                  "HardwareClockDiscontinuityCount,Svid,ConstellationType,"
                  "ReceivedSvTimeNanos,PseudorangeRateMetersPerSecond,"
                  "AccumulatedDeltaRangeState,AccumulatedDeltaRangeMeters,"
                  "CarrierFrequencyHz,Cn0DbHz,SignalType,RawPseudorangeMeters\n";
        output << std::setprecision(17)
               << "Raw,1700000000000," << time_nanos << ',' << full_bias
               << ",0,38.0,0,0,7,3,1," << transmit_nanos
               << ",0,1,42," << constants::GPS_L1_FREQ
               << ",40,GPS_L1_CA,\n";
    }

    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{},
                                      result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 1u);
    EXPECT_NEAR(result.observations.epochs.front().receiver_clock_drift_mps,
                38.0 * constants::SPEED_OF_LIGHT / 1.0e9, 1.0e-12);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, DropsNonPositiveRawClockRangeLikeUpstreamCodeMask) {
    const auto path = fixturePath("non_positive_range");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double tow = 100'000.123;
    const auto time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + tow) * 1.0e9L);
    const auto valid_tx = static_cast<std::int64_t>((tow - 0.070) * 1.0e9);
    const auto invalid_tx = static_cast<std::int64_t>((tow + 0.070) * 1.0e9);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                  valid_tx, 3, 1, 0.0, 1, 42.0,
                  constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
        // The upstream exobs.m P<1e7 mask removes this row after conversion;
        // the native adapter must retain the valid epoch instead of aborting.
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                  invalid_tx, 4, 1, 0.0, 1, 42.0,
                  constants::GPS_L1_FREQ, 40.0, "GPS_L1_CA");
    }

    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{},
                                      result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 1U);
    EXPECT_EQ(result.observations.epochs.front().observations.size(), 1U);
    EXPECT_GE(result.diagnostics.skipped_invalid_quality_rows, 1U);
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

TEST(AndroidRawGnssTest, MapsUpstreamL1AndL5ConstellationSignals) {
    const auto path = fixturePath("l1_l5_mapping");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr std::int64_t time_nanos = 30'660'000'070'000'000LL;
    constexpr std::int64_t transmit_nanos = 100'000'000'000'000LL;
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 3, 1, 0.0, 1, 42.0,
                 constants::GPS_L5_FREQ, 40.0, "GPS_L5_Q");
        // For GLONASS the transmit field is a time-of-day value.  24,382 s
        // maps to the receiver's 100,000 s GPST reference in the upstream
        // glot2gpst conversion.
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 24'382'000'000'000LL, 7, 3, 0.0, 1, 42.0,
                 constants::GLO_L1_BASE_FREQ + 2.0 * constants::GLO_L1_STEP_FREQ,
                 40.0, "GLO_G1_CA");
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 5, 6, 0.0, 1, 42.0,
                 constants::GAL_E5A_FREQ, 40.0, "GAL_E5A_Q");
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos - 14'000'000'000LL, 8, 5, 0.0, 1, 42.0,
                 constants::BDS_B1I_FREQ, 40.0, "BDS_B1I");
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos - 14'000'000'000LL, 9, 5, 0.0, 1, 42.0,
                 constants::BDS_B2A_FREQ, 40.0, "BDS_B2A");
    }

    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{}, result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 1u);
    const auto& epoch = result.observations.epochs.front();
    EXPECT_NE(epoch.getObservation(SatelliteId(GNSSSystem::GPS, 3), SignalType::GPS_L5), nullptr);
    const auto* glo = epoch.getObservation(
        SatelliteId(GNSSSystem::GLONASS, 7), SignalType::GLO_L1CA);
    ASSERT_NE(glo, nullptr);
    EXPECT_TRUE(glo->has_glonass_frequency_channel);
    EXPECT_EQ(glo->glonass_frequency_channel, 2);
    EXPECT_NE(epoch.getObservation(SatelliteId(GNSSSystem::Galileo, 5), SignalType::GAL_E5A), nullptr);
    EXPECT_NE(epoch.getObservation(SatelliteId(GNSSSystem::BeiDou, 8), SignalType::BDS_B1I), nullptr);
    EXPECT_NE(epoch.getObservation(SatelliteId(GNSSSystem::BeiDou, 9), SignalType::BDS_B2A), nullptr);
    EXPECT_EQ(result.diagnostics.gps_l5_rows, 1u);
    EXPECT_EQ(result.diagnostics.glonass_l1_rows, 1u);
    EXPECT_EQ(result.diagnostics.galileo_e5_rows, 1u);
    EXPECT_EQ(result.diagnostics.beidou_l1_rows, 1u);
    EXPECT_EQ(result.diagnostics.beidou_l5_rows, 1u);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, AppliesTimeOffsetAndUpstreamBiasQualityGate) {
    const auto path = fixturePath("raw_quality");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double tow = 100'000.123;
    const auto time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + tow) * 1.0e9L);
    constexpr double transmit_delay = 0.070;
    const auto transmit_nanos = static_cast<std::int64_t>(
        (tow - transmit_delay) * 1.0e9);
    constexpr double time_offset_nanos = 1'000'000.0;
    const long double gps_seconds =
        (static_cast<long double>(time_nanos) -
         static_cast<long double>(full_bias)) / 1.0e9L;
    const long double week_start =
        std::floor(gps_seconds / 604'800.0L) * 604'800.0L;
    const double pseudorange = static_cast<double>(
        (gps_seconds - week_start -
         static_cast<long double>(transmit_nanos) / 1.0e9L -
         static_cast<long double>(time_offset_nanos) / 1.0e9L) *
        constants::SPEED_OF_LIGHT);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << kHeader;
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 3, 1, 25.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 42.0, "GPS_L1_CA", pseudorange, true,
                 time_offset_nanos, 0.0);
        writeRow(output, 1'700'000'000'000LL, time_nanos, full_bias,
                 transmit_nanos, 4, 1, 25.0, 1, 42.0,
                 constants::GPS_L1_FREQ, 42.0, "GPS_L1_CA", 0.0, false,
                 0.0, 10'001.0);
    }
    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{}, result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 1u);
    ASSERT_EQ(result.observations.epochs.front().observations.size(), 1u);
    EXPECT_NEAR(result.observations.epochs.front().observations.front().pseudorange,
                pseudorange, 1e-5);
    EXPECT_EQ(result.diagnostics.skipped_invalid_quality_rows, 1u);
    EXPECT_EQ(result.diagnostics.selected_rows, 1u);
    EXPECT_EQ(result.diagnostics.enriched_pseudorange_mismatches, 0u);
    std::filesystem::remove(path);
}

TEST(AndroidRawGnssTest, AppliesPublishedStateAndMultipathMasks) {
    const auto path = fixturePath("status_masks");
    constexpr std::int64_t full_bias = -1'300'000'000'000'000'000LL;
    constexpr int week = 2200;
    constexpr double tow = 100'000.123;
    const auto time_nanos = static_cast<std::int64_t>(
        static_cast<long double>(full_bias) +
        (static_cast<long double>(week) * 604'800.0L + tow) * 1.0e9L);
    constexpr auto transmit_nanos = static_cast<std::int64_t>(
        (tow - 0.070) * 1.0e9);
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << "MessageType,utcTimeMillis,TimeNanos,FullBiasNanos,BiasNanos,"
                  "BiasUncertaintyNanos,TimeOffsetNanos,HardwareClockDiscontinuityCount,"
                  "Svid,ConstellationType,ReceivedSvTimeNanos,PseudorangeRateMetersPerSecond,"
                  "AccumulatedDeltaRangeState,AccumulatedDeltaRangeMeters,CarrierFrequencyHz,"
                  "Cn0DbHz,SignalType,State,MultipathIndicator\n";
        auto row = [&](int svid, int state, int multipath) {
            output << "Raw,1700000000000," << time_nanos << ',' << full_bias
                   << ",0,0,0,7," << svid << ",1," << transmit_nanos
                   << ",0,1,42," << constants::GPS_L1_FREQ << ",40,GPS_L1_CA,"
                   << state << ',' << multipath << '\n';
        };
        // 0x402f has GPS code lock and TOW-valid bits from exobs.m.
        row(3, 0x402f, 0);
        row(4, 0x402f, 1);
    }

    AndroidRawGnssResult result;
    std::string error;
    ASSERT_TRUE(loadAndroidRawGnssCsv(path.string(), AndroidRawGnssConfig{}, result, error))
        << error;
    ASSERT_EQ(result.observations.epochs.size(), 1u);
    const auto* valid = result.observations.epochs.front().getObservation(
        SatelliteId(GNSSSystem::GPS, 3), SignalType::GPS_L1CA);
    const auto* multipath = result.observations.epochs.front().getObservation(
        SatelliteId(GNSSSystem::GPS, 4), SignalType::GPS_L1CA);
    ASSERT_NE(valid, nullptr);
    ASSERT_NE(multipath, nullptr);
    EXPECT_TRUE(valid->has_pseudorange);
    EXPECT_TRUE(valid->has_doppler);
    EXPECT_TRUE(valid->has_carrier_phase);
    EXPECT_FALSE(multipath->has_pseudorange);
    EXPECT_FALSE(multipath->has_doppler);
    EXPECT_FALSE(multipath->has_carrier_phase);
    EXPECT_EQ(result.diagnostics.masked_code_rows, 1u);
    EXPECT_EQ(result.diagnostics.masked_doppler_rows, 1u);
    EXPECT_EQ(result.diagnostics.masked_carrier_rows, 1u);
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

TEST(AndroidRawGnssTest, MatlabPathsFailClosedBeforeOpening) {
    AndroidRawGnssResult result;
    AndroidRawGnssConfig config;
    std::string error;
    EXPECT_FALSE(loadAndroidRawGnssCsv(
        (std::filesystem::temp_directory_path() / "poisoned.MaT").string(),
        config, result, error));
    EXPECT_NE(error.find("MATLAB"), std::string::npos);
}

TEST(AndroidRawGnssTest, IncludesFirstEpochOnlyFromExactNativeSolution) {
    const GNSSTime t0(2200, 100000.0);
    const std::vector<GNSSTime> times = {t0, t0 + 1.0};
    const std::vector<std::int64_t> keys = {1700000000000LL, 1700000001000LL};
    const Vector3d p(6378137.0, 0.0, 0.0);
    AndroidRawGnssEpochAlignment a;
    std::string error;
    ASSERT_TRUE(alignAndroidRawGnssSolutionsToUtcKeys(
        times, keys, {{t0, p}, {t0 + 1.0, p}}, 1.0, a, error, true));
    ASSERT_EQ(a.epochs.size(), 2U);
    EXPECT_EQ(a.exact_solution_epochs, 2U);
    EXPECT_EQ(a.epochs.front().utc_time_millis, keys.front());
    EXPECT_EQ(a.interpolated_epochs, 0U);
    EXPECT_EQ(a.edge_hold_epochs, 0U);
    EXPECT_FALSE(alignAndroidRawGnssSolutionsToUtcKeys(
        times, keys, {{t0 + 1.0, p}}, 1.0, a, error, true));
    EXPECT_EQ(error, "first raw epoch requires an exact native solution");
}

TEST(AndroidRawGnssTest, AlignsSolutionsToRawUtcKeysWithWarmupAndGapRules) {
    const GNSSTime t0(2200, 100'000.0);
    const std::vector<GNSSTime> raw_times = {
        t0, t0 + 1.0, t0 + 2.0, t0 + 3.0};
    const std::vector<std::int64_t> raw_utc = {
        1'700'000'000'000LL, 1'700'000'001'000LL,
        1'700'000'002'000LL, 1'700'000'003'000LL};
    const Vector3d p0(6'378'137.0, 0.0, 0.0);
    const Vector3d p2(6'378'137.0, 20.0, 0.0);
    const std::vector<AndroidRawGnssSolutionPoint> solutions = {
        {t0 + 0.0005, p0}, {t0 + 2.0, p2}};

    AndroidRawGnssEpochAlignment alignment;
    std::string error;
    ASSERT_TRUE(alignAndroidRawGnssSolutionsToUtcKeys(
        raw_times, raw_utc, solutions, 1.0, alignment, error)) << error;
    ASSERT_EQ(alignment.target_epochs, 3u);
    ASSERT_EQ(alignment.epochs.size(), 3u);
    EXPECT_EQ(alignment.exact_solution_epochs, 1u);
    EXPECT_EQ(alignment.interpolated_epochs, 1u);
    EXPECT_EQ(alignment.edge_hold_epochs, 1u);
    EXPECT_EQ(alignment.unresolved_epochs, 0u);
    EXPECT_EQ(alignment.epochs[0].utc_time_millis, raw_utc[1]);
    EXPECT_EQ(alignment.epochs[0].source,
              AndroidRawGnssEpochSource::EcefLinearInterpolation);
    EXPECT_NEAR(alignment.epochs[0].position_ecef.y(), 10.0, 1e-12);
    EXPECT_EQ(alignment.epochs[1].source,
              AndroidRawGnssEpochSource::ExactSolution);
    EXPECT_EQ(alignment.epochs[2].source,
              AndroidRawGnssEpochSource::NearestEdgeHold);
    EXPECT_EQ(alignment.epochs[2].position_ecef.y(), 20.0);
    EXPECT_DOUBLE_EQ(alignment.max_interpolation_gap_ms, 2'000.0);
    EXPECT_DOUBLE_EQ(alignment.max_edge_hold_gap_ms, 1'000.0);
}

TEST(AndroidRawGnssTest, RawUtcAlignmentRejectsCrossEpochDuplicateAndUnresolved) {
    const GNSSTime t0(2200, 100'000.0);
    const std::vector<GNSSTime> raw_times = {t0, t0 + 1.0, t0 + 2.0};
    const std::vector<std::int64_t> raw_utc = {1000, 2000, 3000};
    const Vector3d position(6'378'137.0, 0.0, 0.0);
    const std::vector<AndroidRawGnssSolutionPoint> duplicate = {
        {t0 + 1.0, position}, {t0 + 1.0001, position}};
    AndroidRawGnssEpochAlignment alignment;
    std::string error;
    EXPECT_FALSE(alignAndroidRawGnssSolutionsToUtcKeys(
        raw_times, raw_utc, duplicate, 1.0, alignment, error));
    EXPECT_NE(error.find("multiple"), std::string::npos);

    const std::vector<AndroidRawGnssSolutionPoint> no_match = {
        {t0 + 10.0, position}};
    error.clear();
    EXPECT_FALSE(alignAndroidRawGnssSolutionsToUtcKeys(
        raw_times, raw_utc, no_match, 1.0, alignment, error));
    EXPECT_NE(error.find("tolerance"), std::string::npos);
}

}  // namespace
}  // namespace libgnss::io
