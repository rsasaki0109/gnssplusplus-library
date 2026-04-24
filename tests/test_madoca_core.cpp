#include <gtest/gtest.h>

#include <libgnss++/algorithms/madoca_core.hpp>
#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/external/madocalib_oracle.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

using namespace libgnss;
namespace madoca_parity = libgnss::algorithms::madoca_parity;
namespace madocalib_oracle = libgnss::external::madocalib_oracle;
namespace qzss_l6 = libgnss::qzss_l6;

namespace {

SSROrbitClockCorrection makeCorrection(const SatelliteId& sat,
                                       const GNSSTime& time,
                                       double clock_m) {
    SSROrbitClockCorrection correction;
    correction.satellite = sat;
    correction.time = time;
    correction.orbit_correction_ecef = Vector3d(0.1, -0.2, 0.3);
    correction.clock_correction_m = clock_m;
    correction.ura_sigma_m = 0.05;
    correction.code_bias_m[1] = 0.12;
    correction.phase_bias_m[1] = -0.004;
    correction.atmos_tokens["atmos_network_id"] = "3";
    correction.orbit_valid = true;
    correction.clock_valid = true;
    correction.ura_valid = true;
    correction.code_bias_valid = true;
    correction.phase_bias_valid = true;
    correction.atmos_valid = true;
    correction.atmos_network_id = 3;
    return correction;
}


madoca_parity::GTime makeMadocaGpsTime(int week, double tow) {
    constexpr std::int64_t kGpsEpochUnixSeconds = 315964800;
    constexpr double kSecondsPerWeek = 604800.0;
    const double whole_tow = std::floor(tow);
    madoca_parity::GTime time;
    time.time = kGpsEpochUnixSeconds + static_cast<std::int64_t>(week * kSecondsPerWeek + whole_tow);
    time.sec = tow - whole_tow;
    return time;
}


void copyVector3(const Vector3d& value, double out[3]) {
    out[0] = value(0);
    out[1] = value(1);
    out[2] = value(2);
}


std::pair<std::int64_t, long long> mionoResultTimeKey(
    const madoca_parity::MionoCorrResult& result) {
    std::pair<std::int64_t, long long> key = {0, 0};
    for (int sat_index = 0; sat_index < madoca_parity::kMadocalibMaxSat; ++sat_index) {
        if (result.t0[sat_index].time == 0) {
            continue;
        }
        const std::pair<std::int64_t, long long> candidate = {
            result.t0[sat_index].time,
            static_cast<long long>(std::llround(result.t0[sat_index].sec * 1e9))};
        if (key < candidate) {
            key = candidate;
        }
    }
    return key;
}

using SsrOrbitClockKey = std::tuple<std::string, int, long long>;

long long towNanos(double tow) {
    return static_cast<long long>(std::llround(tow * 1e9));
}

SsrOrbitClockKey ssrOrbitClockKey(const SatelliteId& satellite,
                                  const GNSSTime& time) {
    return {satellite.toString(), time.week, towNanos(time.tow)};
}

SsrOrbitClockKey ssrOrbitClockKey(
    const madocalib_oracle::SsrCorrectionSnapshot& snapshot) {
    const SatelliteId satellite =
        libgnss::algorithms::madoca_core::satelliteIdFromMadocalibSat(snapshot.sat);
    const GNSSTime time =
        libgnss::algorithms::madoca_core::gnssTimeFromMadocalibTime(snapshot.t0[1]);
    return ssrOrbitClockKey(satellite, time);
}

std::string formatSsrOrbitClockKey(const SsrOrbitClockKey& key) {
    return std::get<0>(key) + "/" + std::to_string(std::get<1>(key)) + "/" +
           std::to_string(std::get<2>(key));
}

int madocalibCodeForCssrSlot(const SatelliteId& satellite, int slot) {
    if (slot < 0 || slot >= 16) {
        return madoca_parity::kCodeNone;
    }

    using Codes = std::array<int, 16>;
    static constexpr Codes kGpsCodes = {
        madoca_parity::kCodeL1C, madoca_parity::kCodeL1P,
        madoca_parity::kCodeL1W, madoca_parity::kCodeL1S,
        madoca_parity::kCodeL1L, madoca_parity::kCodeL1X,
        madoca_parity::kCodeL2S, madoca_parity::kCodeL2L,
        madoca_parity::kCodeL2X, madoca_parity::kCodeL2P,
        madoca_parity::kCodeL2W, madoca_parity::kCodeL5I,
        madoca_parity::kCodeL5Q, madoca_parity::kCodeL5X,
        madoca_parity::kCodeNone, madoca_parity::kCodeNone};
    static constexpr Codes kGloCodes = {
        madoca_parity::kCodeL1C, madoca_parity::kCodeL1P,
        madoca_parity::kCodeL2C, madoca_parity::kCodeL2P,
        madoca_parity::kCodeL4A, madoca_parity::kCodeL4B,
        madoca_parity::kCodeL4X, madoca_parity::kCodeL6A,
        madoca_parity::kCodeL6B,
        madoca_parity::kCodeL6X, madoca_parity::kCodeL3I,
        madoca_parity::kCodeL3Q, madoca_parity::kCodeL3X,
        madoca_parity::kCodeNone, madoca_parity::kCodeNone,
        madoca_parity::kCodeNone};
    static constexpr Codes kGalCodes = {
        madoca_parity::kCodeL1B, madoca_parity::kCodeL1C,
        madoca_parity::kCodeL1X, madoca_parity::kCodeL5I,
        madoca_parity::kCodeL5Q, madoca_parity::kCodeL5X,
        madoca_parity::kCodeL7I, madoca_parity::kCodeL7Q,
        madoca_parity::kCodeL7X, madoca_parity::kCodeL8I,
        madoca_parity::kCodeL8Q, madoca_parity::kCodeL8X,
        madoca_parity::kCodeL6B, madoca_parity::kCodeL6C,
        madoca_parity::kCodeNone, madoca_parity::kCodeNone};
    static constexpr Codes kBd2Codes = {
        madoca_parity::kCodeL2I, madoca_parity::kCodeL2Q,
        madoca_parity::kCodeL2X, madoca_parity::kCodeL6I,
        madoca_parity::kCodeL6Q, madoca_parity::kCodeL6X,
        madoca_parity::kCodeL7I, madoca_parity::kCodeL7Q,
        madoca_parity::kCodeL7X, madoca_parity::kCodeNone,
        madoca_parity::kCodeNone, madoca_parity::kCodeNone,
        madoca_parity::kCodeNone, madoca_parity::kCodeNone,
        madoca_parity::kCodeNone, madoca_parity::kCodeNone};
    static constexpr Codes kBd3Codes = {
        madoca_parity::kCodeL2I, madoca_parity::kCodeL2Q,
        madoca_parity::kCodeNone, madoca_parity::kCodeL6I,
        madoca_parity::kCodeL6Q, madoca_parity::kCodeNone,
        madoca_parity::kCodeL7D, madoca_parity::kCodeL7P,
        madoca_parity::kCodeL7Z, madoca_parity::kCodeL1D,
        madoca_parity::kCodeL1P, madoca_parity::kCodeL1X,
        madoca_parity::kCodeL5D, madoca_parity::kCodeL5P,
        madoca_parity::kCodeL5X, madoca_parity::kCodeNone};
    static constexpr Codes kQzsCodes = {
        madoca_parity::kCodeL1C, madoca_parity::kCodeL1S,
        madoca_parity::kCodeL1L, madoca_parity::kCodeL1X,
        madoca_parity::kCodeL2S, madoca_parity::kCodeL2L,
        madoca_parity::kCodeL2X, madoca_parity::kCodeL5I,
        madoca_parity::kCodeL5Q, madoca_parity::kCodeL5X,
        madoca_parity::kCodeL6S, madoca_parity::kCodeL6L,
        madoca_parity::kCodeL6E, madoca_parity::kCodeL1E,
        madoca_parity::kCodeNone, madoca_parity::kCodeNone};

    const Codes* codes = nullptr;
    switch (satellite.system) {
        case GNSSSystem::GPS: codes = &kGpsCodes; break;
        case GNSSSystem::GLONASS: codes = &kGloCodes; break;
        case GNSSSystem::Galileo: codes = &kGalCodes; break;
        case GNSSSystem::BeiDou:
            codes = satellite.prn >= 19 ? &kBd3Codes : &kBd2Codes;
            break;
        case GNSSSystem::QZSS: codes = &kQzsCodes; break;
        default: return madoca_parity::kCodeNone;
    }
    return (*codes)[static_cast<std::size_t>(slot)];
}

using BiasRows = std::map<int, double>;

BiasRows nativeBiasRowsForSatellite(
    const std::map<SatelliteId, std::map<int, double>>& biases,
    const SatelliteId& satellite) {
    BiasRows rows;
    const auto sat_it = biases.find(satellite);
    if (sat_it == biases.end()) {
        return rows;
    }
    for (const auto& [slot, bias_m] : sat_it->second) {
        const int code = madocalibCodeForCssrSlot(satellite, slot);
        if (code > 0) {
            rows[code - 1] = bias_m;
        }
    }
    return rows;
}

BiasRows oracleBiasRows(
    const double (&biases)[madoca_parity::kMadocalibMaxCode],
    const int (&valid_flags)[madoca_parity::kMadocalibMaxCode]) {
    BiasRows rows;
    for (int code_index = 0; code_index < madoca_parity::kMadocalibMaxCode;
         ++code_index) {
        if (valid_flags[code_index] > 0) {
            rows[code_index] = biases[code_index];
        }
    }
    return rows;
}

void expectBiasRowsNear(const BiasRows& native,
                        const BiasRows& oracle,
                        const SsrOrbitClockKey& key,
                        const char* bias_type) {
    ASSERT_EQ(native.size(), oracle.size())
        << "type=" << bias_type << " key=" << formatSsrOrbitClockKey(key);
    for (const auto& [code_index, oracle_value] : oracle) {
        const auto native_it = native.find(code_index);
        ASSERT_NE(native_it, native.end())
            << "type=" << bias_type << " code_index=" << code_index
            << " key=" << formatSsrOrbitClockKey(key);
        EXPECT_NEAR(native_it->second, oracle_value, madoca_parity::kOracleTolerance)
            << "type=" << bias_type << " code_index=" << code_index
            << " key=" << formatSsrOrbitClockKey(key);
    }
}


void writePackedBit(std::vector<std::uint8_t>& data, int bit, int value) {
    const int byte_index = bit / 8;
    if (byte_index >= static_cast<int>(data.size())) {
        data.resize(static_cast<std::size_t>(byte_index + 1), 0);
    }
    if (value != 0) {
        data[static_cast<std::size_t>(byte_index)] |=
            static_cast<std::uint8_t>(1U << (7 - (bit % 8)));
    }
}

void writeUnsignedBits(std::vector<std::uint8_t>& data,
                       int& bit,
                       std::uint64_t value,
                       int bit_count) {
    for (int i = bit_count - 1; i >= 0; --i) {
        writePackedBit(data, bit++, static_cast<int>((value >> i) & 1U));
    }
}

void writeSignedBits(std::vector<std::uint8_t>& data,
                     int& bit,
                     int value,
                     int bit_count) {
    const std::uint64_t encoded = value < 0
        ? (1ULL << bit_count) + static_cast<std::uint64_t>(value)
        : static_cast<std::uint64_t>(value);
    writeUnsignedBits(data, bit, encoded, bit_count);
}

void writeFrameBit(std::array<std::uint8_t, qzss_l6::kFrameBytes>& frame,
                   int bit,
                   int value) {
    if (value == 0) {
        return;
    }
    frame[static_cast<std::size_t>(bit / 8)] |=
        static_cast<std::uint8_t>(1U << (7 - (bit % 8)));
}

std::array<std::uint8_t, qzss_l6::kFrameBytes> makeSyntheticL6DFrame() {
    std::vector<std::uint8_t> payload((qzss_l6::kDataPartBits + 7) / 8, 0);
    int bit = 0;

    constexpr int kCorrectionBits = 103;
    writeUnsignedBits(payload, bit, 1, 12);       // coverage message number
    writeUnsignedBits(payload, bit, 0, 4);        // subtype
    writeUnsignedBits(payload, bit, 172800, 20);  // GNSS epoch TOW
    writeUnsignedBits(payload, bit, 0, 4);        // UDI
    writeUnsignedBits(payload, bit, 0, 1);        // sync
    writeUnsignedBits(payload, bit, 3, 4);        // IOD
    writeUnsignedBits(payload, bit, 11, 8);       // region ID
    writeUnsignedBits(payload, bit, 0, 1);        // region alert
    writeUnsignedBits(payload, bit, kCorrectionBits, 16);
    writeUnsignedBits(payload, bit, 1, 5);        // area count
    writeUnsignedBits(payload, bit, 4, 5);        // area number
    writeUnsignedBits(payload, bit, 0, 1);        // rectangle
    writeSignedBits(payload, bit, 356, 11);       // 35.6 deg
    writeUnsignedBits(payload, bit, 1398, 12);    // 139.8 deg
    writeUnsignedBits(payload, bit, 10, 8);       // 1.0 deg latitude span
    writeUnsignedBits(payload, bit, 10, 8);       // 1.0 deg longitude span

    writeUnsignedBits(payload, bit, 2, 12);       // correction message number
    writeUnsignedBits(payload, bit, 0, 4);        // subtype
    writeUnsignedBits(payload, bit, 30, 12);      // hourly epoch
    writeUnsignedBits(payload, bit, 0, 4);        // UDI
    writeUnsignedBits(payload, bit, 0, 1);        // sync
    writeUnsignedBits(payload, bit, 3, 4);        // IOD
    writeUnsignedBits(payload, bit, 11, 8);       // region ID
    writeUnsignedBits(payload, bit, 4, 5);        // area number
    writeUnsignedBits(payload, bit, 0, 2);        // correction type C00 only
    writeUnsignedBits(payload, bit, 1, 5);        // GPS satellite count
    writeUnsignedBits(payload, bit, 0, 5);        // GLONASS
    writeUnsignedBits(payload, bit, 0, 5);        // Galileo
    writeUnsignedBits(payload, bit, 0, 5);        // BeiDou
    writeUnsignedBits(payload, bit, 0, 5);        // QZSS
    writeUnsignedBits(payload, bit, 5, 6);        // GPS PRN 5
    writeUnsignedBits(payload, bit, 9, 6);        // SQI
    writeSignedBits(payload, bit, 245, 14);       // 12.25 TECU

    std::array<std::uint8_t, qzss_l6::kFrameBytes> frame{};
    frame[0] = 0x1A;
    frame[1] = 0xCF;
    frame[2] = 0xFC;
    frame[3] = 0x1D;
    frame[4] = 200;
    frame[5] = static_cast<std::uint8_t>((2U << 5U) | (1U << 2U) | 1U);
    for (int payload_bit = 0; payload_bit < qzss_l6::kDataPartBits; ++payload_bit) {
        const int byte_index = payload_bit / 8;
        const int bit_index = 7 - (payload_bit % 8);
        const int value =
            (payload[static_cast<std::size_t>(byte_index)] >> bit_index) & 1U;
        writeFrameBit(frame, 49 + payload_bit, value);
    }
    return frame;
}

qzss_l6::CssrEpoch makeL6EEpoch(const SatelliteId& sat, const GNSSTime& time) {
    qzss_l6::CssrEpoch epoch;
    epoch.week = time.week;
    epoch.tow = time.tow;
    epoch.clocks[sat].dclock_m = 0.42;
    epoch.has_clock = true;

    qzss_l6::CssrOrbitCorrection orbit;
    orbit.dx = 0.01;
    orbit.dy = -0.02;
    orbit.dz = 0.03;
    orbit.iode = 77;
    epoch.orbits[sat] = orbit;
    epoch.has_orbit = true;

    epoch.code_biases[sat][0] = 0.11;
    epoch.phase_biases[sat][0] = -0.005;
    epoch.ura_indices[sat] = 9;
    epoch.has_code_bias = true;
    epoch.has_phase_bias = true;
    epoch.has_ura = true;
    return epoch;
}

}  // namespace

TEST(MadocaCoreTest, NativeConfigBuildsPppSsrProfile) {
    using namespace libgnss::algorithms::madoca_core;

    NativeCoreConfig core_config;
    core_config.enable_ambiguity_resolution = true;
    core_config.correction_max_age_seconds = 45.0;

    const ppp_shared::PPPConfig ppp_config = makePPPConfig(core_config);

    EXPECT_FALSE(ppp_config.use_precise_orbits);
    EXPECT_FALSE(ppp_config.use_precise_clocks);
    EXPECT_TRUE(ppp_config.use_ssr_corrections);
    EXPECT_TRUE(ppp_config.require_ssr_orbit_clock);
    EXPECT_FALSE(ppp_config.enforce_ssr_orbit_iode);
    EXPECT_TRUE(ppp_config.use_rtklib_broadcast_selection);
    EXPECT_FALSE(ppp_config.use_clas_osr_filter);
    EXPECT_TRUE(ppp_config.use_carrier_phase_without_precise_products);
    EXPECT_NEAR(ppp_config.initial_troposphere_variance, 0.0144, 1e-12);
    EXPECT_TRUE(ppp_config.enable_ambiguity_resolution);
    EXPECT_TRUE(ppp_config.prefer_receiver_position_seed);
    EXPECT_TRUE(ppp_config.reset_clock_to_spp_each_epoch);
    EXPECT_FALSE(ppp_config.reset_kinematic_position_to_spp_each_epoch);
    EXPECT_TRUE(ppp_config.allowed_systems.empty());
    EXPECT_DOUBLE_EQ(ppp_config.clas_atmos_stale_after_seconds, 45.0);
    EXPECT_EQ(ppp_config.clas_correction_application_policy,
              ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS);
}

TEST(MadocaCoreTest, NativeConfigCopiesAllowedSystems) {
    using namespace libgnss::algorithms::madoca_core;

    NativeCoreConfig core_config;
    core_config.allowed_systems = {GNSSSystem::GPS, GNSSSystem::QZSS};

    const ppp_shared::PPPConfig ppp_config = makePPPConfig(core_config);

    EXPECT_EQ(ppp_config.allowed_systems.size(), 2U);
    EXPECT_EQ(ppp_config.allowed_systems.count(GNSSSystem::GPS), 1U);
    EXPECT_EQ(ppp_config.allowed_systems.count(GNSSSystem::BeiDou), 0U);
    EXPECT_EQ(ppp_config.allowed_systems.count(GNSSSystem::QZSS), 1U);
}

TEST(MadocaCoreTest, CorrectionStoreSummarizesAndInterpolatesNativeProducts) {
    using namespace libgnss::algorithms::madoca_core;

    NativeCorrectionStore store;
    const SatelliteId sat(GNSSSystem::GPS, 5);
    const GNSSTime time(2400, 345600.0);
    store.addCorrection(makeCorrection(sat, time, 1.25), CorrectionSource::L6E);

    ASSERT_FALSE(store.empty());
    EXPECT_EQ(store.summary().total, 1U);
    EXPECT_EQ(store.summary().satellites, 1U);
    EXPECT_EQ(store.summary().orbit, 1U);
    EXPECT_EQ(store.summary().clock, 1U);
    EXPECT_EQ(store.summary().code_bias, 1U);
    EXPECT_EQ(store.summary().phase_bias, 1U);
    EXPECT_EQ(store.summary().atmosphere, 1U);
    ASSERT_EQ(store.sourceCounts().at(CorrectionSource::L6E), 1U);

    Vector3d orbit = Vector3d::Zero();
    double clock_m = 0.0;
    double ura_m = 0.0;
    std::map<uint8_t, double> code_biases;
    std::map<uint8_t, double> phase_biases;
    std::map<std::string, std::string> atmos_tokens;

    ASSERT_TRUE(store.interpolateCorrection(sat,
                                            time,
                                            orbit,
                                            clock_m,
                                            &ura_m,
                                            &code_biases,
                                            &phase_biases,
                                            &atmos_tokens,
                                            3));
    EXPECT_DOUBLE_EQ(orbit.x(), 0.1);
    EXPECT_DOUBLE_EQ(orbit.y(), -0.2);
    EXPECT_DOUBLE_EQ(orbit.z(), 0.3);
    EXPECT_DOUBLE_EQ(clock_m, 1.25);
    EXPECT_DOUBLE_EQ(ura_m, 0.05);
    EXPECT_DOUBLE_EQ(code_biases[1], 0.12);
    EXPECT_DOUBLE_EQ(phase_biases[1], -0.004);
    EXPECT_EQ(atmos_tokens["atmos_network_id"], "3");
}

TEST(MadocaCoreTest, NativeStoreCanLoadDirectlyIntoPppProcessor) {
    using namespace libgnss::algorithms::madoca_core;

    NativeCorrectionStore store;
    const SatelliteId sat(GNSSSystem::QZSS, 193);
    const GNSSTime time(2400, 345630.0);
    store.addCorrection(makeCorrection(sat, time, -0.75), CorrectionSource::ExternalFixture);

    PPPProcessor processor(makePPPConfig(NativeCoreConfig{}));
    ASSERT_TRUE(loadCorrections(processor, store));
    EXPECT_TRUE(processor.hasLoadedSSRProducts());

    Vector3d orbit = Vector3d::Zero();
    double clock_m = 0.0;
    ASSERT_TRUE(processor.interpolateLoadedSSRCorrection(sat, time, orbit, clock_m));
    EXPECT_DOUBLE_EQ(orbit.x(), 0.1);
    EXPECT_DOUBLE_EQ(orbit.y(), -0.2);
    EXPECT_DOUBLE_EQ(orbit.z(), 0.3);
    EXPECT_DOUBLE_EQ(clock_m, -0.75);
}

TEST(MadocaCoreTest, MaterializesL6ECompactEpochsAsNativeSsrCorrections) {
    using namespace libgnss::algorithms::madoca_core;

    const SatelliteId satellite(GNSSSystem::GPS, 5);
    const GNSSTime time(2402, 333.25);
    const auto corrections = materializeL6ECorrections({makeL6EEpoch(satellite, time)}, 0);

    ASSERT_EQ(corrections.size(), 1U);
    const auto& correction = corrections.front();

    EXPECT_EQ(correction.satellite, satellite);
    EXPECT_EQ(correction.time.week, time.week);
    EXPECT_DOUBLE_EQ(correction.time.tow, time.tow);
    EXPECT_TRUE(correction.orbit_valid);
    EXPECT_TRUE(correction.clock_valid);
    EXPECT_TRUE(correction.code_bias_valid);
    EXPECT_TRUE(correction.phase_bias_valid);
    EXPECT_DOUBLE_EQ(correction.orbit_correction_ecef.x(), 0.01);
    EXPECT_DOUBLE_EQ(correction.orbit_correction_ecef.y(), -0.02);
    EXPECT_DOUBLE_EQ(correction.orbit_correction_ecef.z(), 0.03);
    EXPECT_EQ(correction.orbit_iode, 77);
    EXPECT_DOUBLE_EQ(correction.clock_correction_m, 0.42);
    EXPECT_TRUE(correction.ura_valid);
    EXPECT_NEAR(correction.ura_sigma_m, 0.00275, 1e-12);
    ASSERT_EQ(correction.code_bias_m.count(2), 1U);
    ASSERT_EQ(correction.phase_bias_m.count(2), 1U);
    EXPECT_DOUBLE_EQ(correction.code_bias_m.at(2), -0.11);
    EXPECT_DOUBLE_EQ(correction.phase_bias_m.at(2), 0.005);
}

TEST(MadocaCoreTest, MaterializesMadocaQzssPrnsAsObservationIds) {
    using namespace libgnss::algorithms::madoca_core;

    const SatelliteId raw_satellite(GNSSSystem::QZSS, 194);
    const SatelliteId observation_satellite(GNSSSystem::QZSS, 2);
    const GNSSTime time(2402, 345.0);
    const auto corrections = materializeL6ECorrections({makeL6EEpoch(raw_satellite, time)}, 0);

    ASSERT_EQ(corrections.size(), 1U);
    EXPECT_EQ(corrections.front().satellite, observation_satellite);

    NativeCorrectionStore store;
    store.addL6ECorrections({makeL6EEpoch(raw_satellite, time)});

    Vector3d orbit = Vector3d::Zero();
    double clock_m = 0.0;
    ASSERT_TRUE(store.interpolateCorrection(observation_satellite, time, orbit, clock_m));
    EXPECT_DOUBLE_EQ(orbit.x(), 0.01);
    EXPECT_DOUBLE_EQ(clock_m, 0.42);
}

TEST(MadocaCoreTest, StoreCarriesL6ECompactEpochsThroughSsrInterpolation) {
    using namespace libgnss::algorithms::madoca_core;

    NativeCorrectionStore store;
    const SatelliteId satellite(GNSSSystem::GPS, 8);
    const GNSSTime time(2402, 360.0);
    store.addL6ECorrections({makeL6EEpoch(satellite, time)});

    ASSERT_FALSE(store.empty());
    EXPECT_EQ(store.summary().total, 1U);
    EXPECT_EQ(store.summary().orbit, 1U);
    EXPECT_EQ(store.summary().clock, 1U);
    EXPECT_EQ(store.summary().code_bias, 1U);
    EXPECT_EQ(store.summary().phase_bias, 1U);
    ASSERT_EQ(store.sourceCounts().at(CorrectionSource::L6E), 1U);

    Vector3d orbit = Vector3d::Zero();
    double clock_m = 0.0;
    double ura_m = 0.0;
    std::map<uint8_t, double> code_biases;
    std::map<uint8_t, double> phase_biases;
    ASSERT_TRUE(store.interpolateCorrection(satellite,
                                            time,
                                            orbit,
                                            clock_m,
                                            &ura_m,
                                            &code_biases,
                                            &phase_biases));

    EXPECT_DOUBLE_EQ(orbit.x(), 0.01);
    EXPECT_DOUBLE_EQ(orbit.y(), -0.02);
    EXPECT_DOUBLE_EQ(orbit.z(), 0.03);
    EXPECT_DOUBLE_EQ(clock_m, 0.42);
    EXPECT_NEAR(ura_m, 0.00275, 1e-12);
    ASSERT_EQ(code_biases.count(2), 1U);
    ASSERT_EQ(phase_biases.count(2), 1U);
    EXPECT_DOUBLE_EQ(code_biases.at(2), -0.11);
    EXPECT_DOUBLE_EQ(phase_biases.at(2), 0.005);
}

TEST(MadocaCoreTest, LoadsL6ECompactEpochsDirectlyIntoPppProcessor) {
    using namespace libgnss::algorithms::madoca_core;

    PPPProcessor processor(makePPPConfig(NativeCoreConfig{}));
    const SatelliteId satellite(GNSSSystem::GPS, 9);
    const GNSSTime time(2402, 390.0);
    ASSERT_TRUE(loadL6ECorrections(processor, {makeL6EEpoch(satellite, time)}));
    EXPECT_TRUE(processor.hasLoadedSSRProducts());

    Vector3d orbit = Vector3d::Zero();
    double clock_m = 0.0;
    std::map<uint8_t, double> code_biases;
    std::map<uint8_t, double> phase_biases;
    ASSERT_TRUE(processor.interpolateLoadedSSRCorrection(satellite,
                                                         time,
                                                         orbit,
                                                         clock_m,
                                                         nullptr,
                                                         &code_biases,
                                                         &phase_biases));

    EXPECT_DOUBLE_EQ(orbit.x(), 0.01);
    EXPECT_DOUBLE_EQ(orbit.y(), -0.02);
    EXPECT_DOUBLE_EQ(orbit.z(), 0.03);
    EXPECT_DOUBLE_EQ(clock_m, 0.42);
    ASSERT_EQ(code_biases.count(2), 1U);
    ASSERT_EQ(phase_biases.count(2), 1U);
    EXPECT_DOUBLE_EQ(code_biases.at(2), -0.11);
    EXPECT_DOUBLE_EQ(phase_biases.at(2), 0.005);
}

TEST(MadocaCoreTest, EmptyL6EFileListDoesNotLoadCorrections) {
    using namespace libgnss::algorithms::madoca_core;

    EXPECT_TRUE(decodeL6EFiles({}, 2402).empty());

    PPPProcessor processor(makePPPConfig(NativeCoreConfig{}));
    EXPECT_FALSE(loadL6EFiles(processor, {}, 2402));
    EXPECT_FALSE(processor.hasLoadedSSRProducts());
}

TEST(MadocaCoreTest, MaterializesMionoCorrectionsAsPerSatelliteStecTokens) {
    using namespace libgnss::algorithms::madoca_core;

    const int sat_no = madoca_parity::satno(madoca_parity::kSysGps, 5);
    ASSERT_GT(sat_no, 0);

    const SatelliteId satellite(GNSSSystem::GPS, 5);
    const double target_stec_tecu = 12.25;
    madoca_parity::MionoCorrResult result;
    result.rid = 33;
    result.area_number = 7;
    result.t0[sat_no - 1] = makeMadocaGpsTime(2400, 345600.25);
    result.delay[sat_no - 1] = target_stec_tecu * l1StecDelayFactorMeters(satellite);
    result.std[sat_no - 1] = 0.125;

    const auto corrections = materializeMionoCorrections(result, 9);
    ASSERT_EQ(corrections.size(), 1U);
    const auto& correction = corrections.front();

    EXPECT_EQ(correction.satellite, satellite);
    EXPECT_EQ(correction.time.week, 2400);
    EXPECT_NEAR(correction.time.tow, 345600.25, 1e-9);
    EXPECT_TRUE(correction.atmos_valid);
    EXPECT_EQ(correction.atmos_network_id, 9);
    EXPECT_EQ(correction.atmos_tokens.at("madoca_region_id"), "33");
    EXPECT_EQ(correction.atmos_tokens.at("madoca_area_number"), "7");
    EXPECT_EQ(correction.atmos_tokens.at("atmos_selected_satellites"), "1");

    const double stec_tecu = ppp_atmosphere::atmosphericStecTecu(
        correction.atmos_tokens,
        satellite,
        Vector3d::Zero());
    EXPECT_NEAR(stec_tecu, target_stec_tecu, 1e-12);
}

TEST(MadocaCoreTest, MaterializesMionoCorrectionsForBds3Satellites) {
    using namespace libgnss::algorithms::madoca_core;

    const int sat_no = madoca_parity::satno(madoca_parity::kSysCmp, 28);
    ASSERT_GT(sat_no, 0);

    const SatelliteId satellite(GNSSSystem::BeiDou, 28);
    ASSERT_GT(l1StecDelayFactorMeters(satellite), 0.0);

    const double target_stec_tecu = 8.75;
    madoca_parity::MionoCorrResult result;
    result.rid = 35;
    result.area_number = 9;
    result.t0[sat_no - 1] = makeMadocaGpsTime(2400, 345660.0);
    result.delay[sat_no - 1] = target_stec_tecu * l1StecDelayFactorMeters(satellite);
    result.std[sat_no - 1] = 0.075;

    const auto corrections = materializeMionoCorrections(result, 10);
    ASSERT_EQ(corrections.size(), 1U);
    const auto& correction = corrections.front();

    EXPECT_EQ(correction.satellite, satellite);
    EXPECT_EQ(correction.atmos_network_id, 10);
    EXPECT_EQ(correction.atmos_tokens.at("atmos_stec_type:C28"), "0");
    EXPECT_TRUE(correction.atmos_tokens.count("atmos_stec_c00_tecu:C28"));

    const double stec_tecu = ppp_atmosphere::atmosphericStecTecu(
        correction.atmos_tokens,
        satellite,
        Vector3d::Zero());
    EXPECT_NEAR(stec_tecu, target_stec_tecu, 1e-12);
}

TEST(MadocaCoreTest, SkipsMionoCorrectionsForUnsupportedBds2Frequency) {
    using namespace libgnss::algorithms::madoca_core;

    const int sat_no = madoca_parity::satno(madoca_parity::kSysBd2, 6);
    ASSERT_GT(sat_no, 0);

    const SatelliteId satellite(GNSSSystem::BeiDou, 6);
    EXPECT_DOUBLE_EQ(l1StecDelayFactorMeters(satellite), 0.0);

    madoca_parity::MionoCorrResult result;
    result.rid = 34;
    result.area_number = 8;
    result.t0[sat_no - 1] = makeMadocaGpsTime(2400, 345630.0);
    result.delay[sat_no - 1] = 1.0;
    result.std[sat_no - 1] = 0.25;

    EXPECT_TRUE(materializeMionoCorrections(result, 9).empty());
}

TEST(MadocaCoreTest, DecodesSyntheticL6DFileIntoMionoAtmosphereRows) {
    using namespace libgnss::algorithms::madoca_core;

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "libgnss_madoca_l6d_synthetic.l6";
    const auto frame = makeSyntheticL6DFrame();
    {
        std::ofstream output(path, std::ios::binary);
        ASSERT_TRUE(output.good());
        output.write(reinterpret_cast<const char*>(frame.data()), frame.size());
    }

    const Vector3d receiver_position =
        geodetic2ecef(35.6 * M_PI / 180.0, 139.8 * M_PI / 180.0, 0.0);
    const auto results = decodeL6DFile(path.string(), 2360, receiver_position);
    std::filesystem::remove(path);

    ASSERT_EQ(results.size(), 1U);
    const auto& result = results.front();
    EXPECT_EQ(result.rid, 11);
    EXPECT_EQ(result.area_number, 4);

    const int sat_no = madoca_parity::satno(madoca_parity::kSysGps, 5);
    ASSERT_GT(sat_no, 0);
    const GNSSTime correction_time = gnssTimeFromMadocalibTime(result.t0[sat_no - 1]);
    EXPECT_EQ(correction_time.week, 2360);
    EXPECT_NEAR(correction_time.tow, 172830.0, 1e-9);

    const SatelliteId satellite(GNSSSystem::GPS, 5);
    const auto corrections = materializeL6DCorrections(results, 21);
    ASSERT_EQ(corrections.size(), 1U);
    EXPECT_EQ(corrections.front().satellite, satellite);
    EXPECT_EQ(corrections.front().atmos_network_id, 21);
    const double stec_tecu = ppp_atmosphere::atmosphericStecTecu(
        corrections.front().atmos_tokens,
        satellite,
        receiver_position);
    EXPECT_NEAR(stec_tecu, 12.25, 1e-12);

    NativeCorrectionStore store;
    store.addL6DCorrections(results, 21);
    ASSERT_FALSE(store.empty());
    EXPECT_EQ(store.summary().atmosphere, 1U);
    ASSERT_EQ(store.sourceCounts().at(CorrectionSource::L6D), 1U);

    Vector3d orbit = Vector3d::Zero();
    double clock_m = 0.0;
    std::map<std::string, std::string> atmos_tokens;
    ASSERT_TRUE(store.interpolateCorrection(satellite,
                                            correction_time,
                                            orbit,
                                            clock_m,
                                            nullptr,
                                            nullptr,
                                            nullptr,
                                            &atmos_tokens,
                                            21));
    EXPECT_NEAR(ppp_atmosphere::atmosphericStecTecu(atmos_tokens,
                                                    satellite,
                                                    receiver_position),
                12.25,
                1e-12);
}

TEST(MadocaCoreTest, OracleLinkedL6DFileMatchesMadocalibDecoderForMizuSample) {
    using namespace libgnss::algorithms::madoca_core;

    if (!madocalib_oracle::available()) {
        GTEST_SKIP() << "MADOCALIB oracle unavailable; configure with "
                     << "-DMADOCALIB_PARITY_LINK=ON";
    }

    const std::filesystem::path root(madocalib_oracle::rootDirectory());
    const std::filesystem::path l6d_200 =
        root / "sample_data" / "data" / "l6_is-qzss-mdc-004" / "2025" /
        "091" / "2025091A.200.l6";
    const std::filesystem::path l6d_201 =
        root / "sample_data" / "data" / "l6_is-qzss-mdc-004" / "2025" /
        "091" / "2025091A.201.l6";
    if (!std::filesystem::exists(l6d_200) || !std::filesystem::exists(l6d_201)) {
        GTEST_SKIP() << "MADOCALIB sample L6D files not found under: " << root;
    }
    const std::vector<std::string> l6d_files = {l6d_200.string(), l6d_201.string()};

    const Vector3d receiver_position(-3857167.6484, 3108694.9138, 4004041.6876);
    double rr[3] = {};
    copyVector3(receiver_position, rr);

    const auto native = decodeL6DFiles(l6d_files, 2360, receiver_position);
    const auto oracle = madocalib_oracle::decode_l6d_files(l6d_files, 2360, rr);

    ASSERT_FALSE(native.empty());
    ASSERT_FALSE(oracle.empty());

    std::map<std::pair<std::int64_t, long long>, const madoca_parity::MionoCorrResult*>
        native_final_by_time;
    for (const auto& result : native) {
        const auto key = mionoResultTimeKey(result);
        if (key.first != 0) {
            native_final_by_time[key] = &result;
        }
    }
    ASSERT_EQ(native_final_by_time.size(), oracle.size());

    for (std::size_t result_index = 0; result_index < oracle.size(); ++result_index) {
        const auto& oracle_result = oracle[result_index];
        const auto key = mionoResultTimeKey(oracle_result);
        const auto native_it = native_final_by_time.find(key);
        ASSERT_NE(native_it, native_final_by_time.end()) << "result=" << result_index;
        const auto& native_result = *native_it->second;

        EXPECT_EQ(native_result.rid, oracle_result.rid) << "result=" << result_index;
        EXPECT_EQ(native_result.area_number, oracle_result.area_number)
            << "result=" << result_index;

        int compared_satellites = 0;
        for (int sat_index = 0; sat_index < madoca_parity::kMadocalibMaxSat;
             ++sat_index) {
            const bool native_valid = native_result.t0[sat_index].time != 0;
            const bool oracle_valid = oracle_result.t0[sat_index].time != 0;
            ASSERT_EQ(native_valid, oracle_valid)
                << "result=" << result_index << " sat_index=" << sat_index;
            if (!native_valid) {
                continue;
            }

            ++compared_satellites;
            EXPECT_EQ(native_result.t0[sat_index].time, oracle_result.t0[sat_index].time)
                << "result=" << result_index << " sat_index=" << sat_index;
            EXPECT_NEAR(native_result.t0[sat_index].sec,
                        oracle_result.t0[sat_index].sec,
                        1e-12)
                << "result=" << result_index << " sat_index=" << sat_index;
            if (std::isfinite(native_result.delay[sat_index]) ||
                std::isfinite(oracle_result.delay[sat_index])) {
                EXPECT_NEAR(native_result.delay[sat_index],
                            oracle_result.delay[sat_index],
                            1e-6)
                    << "result=" << result_index << " sat_index=" << sat_index;
            } else {
                EXPECT_EQ(std::signbit(native_result.delay[sat_index]),
                          std::signbit(oracle_result.delay[sat_index]))
                    << "result=" << result_index << " sat_index=" << sat_index;
            }
            EXPECT_NEAR(native_result.std[sat_index], oracle_result.std[sat_index], 1e-6)
                << "result=" << result_index << " sat_index=" << sat_index;
        }
        EXPECT_GT(compared_satellites, 0) << "result=" << result_index;
    }
}

TEST(MadocaCoreTest, OracleLinkedL6EFileMatchesMadocalibOrbitClockForMizuSample) {
    using namespace libgnss::algorithms::madoca_core;

    if (madocalib_oracle::available() == false) {
        GTEST_SKIP() << "MADOCALIB oracle unavailable; configure with "
                     << "-DMADOCALIB_PARITY_LINK=ON";
    }

    const std::filesystem::path root(madocalib_oracle::rootDirectory());
    const std::filesystem::path l6e_204 =
        root / "sample_data" / "data" / "l6_is-qzss-mdc-004" / "2025" /
        "091" / "2025091A.204.l6";
    const std::filesystem::path l6e_206 =
        root / "sample_data" / "data" / "l6_is-qzss-mdc-004" / "2025" /
        "091" / "2025091A.206.l6";
    if (!std::filesystem::exists(l6e_204) || !std::filesystem::exists(l6e_206)) {
        GTEST_SKIP() << "MADOCALIB sample L6E files not found under: " << root;
    }
    const std::vector<std::string> l6e_files = {l6e_204.string(), l6e_206.string()};

    const auto native_epochs = decodeL6EFiles(l6e_files, 2360);
    const auto native_rows = materializeL6ECorrections(native_epochs);
    const auto oracle_rows = madocalib_oracle::decode_l6e_files(l6e_files, 2360);

    ASSERT_FALSE(native_rows.empty());
    ASSERT_FALSE(oracle_rows.empty());

    std::map<SsrOrbitClockKey, const SSROrbitClockCorrection*> native_by_key;
    for (const auto& row : native_rows) {
        if (!row.clock_valid || row.satellite.system == GNSSSystem::UNKNOWN) {
            continue;
        }
        native_by_key.emplace(ssrOrbitClockKey(row.satellite, row.time), &row);
    }

    std::map<SsrOrbitClockKey, const madocalib_oracle::SsrCorrectionSnapshot*>
        oracle_by_key;
    for (const auto& row : oracle_rows) {
        const SatelliteId satellite = satelliteIdFromMadocalibSat(row.sat);
        if (!row.clock_valid || satellite.system == GNSSSystem::UNKNOWN) {
            continue;
        }
        oracle_by_key[ssrOrbitClockKey(row)] = &row;
    }

    ASSERT_FALSE(native_by_key.empty());
    ASSERT_FALSE(oracle_by_key.empty());

    std::vector<SsrOrbitClockKey> missing_native;
    for (const auto& [key, oracle_row] : oracle_by_key) {
        if (native_by_key.count(key) == 0) {
            missing_native.push_back(key);
        }
    }
    std::vector<SsrOrbitClockKey> missing_oracle;
    for (const auto& [key, native_row] : native_by_key) {
        if (oracle_by_key.count(key) == 0) {
            missing_oracle.push_back(key);
        }
    }

    ASSERT_EQ(native_by_key.size(), oracle_by_key.size())
        << " missing_native=" << missing_native.size()
        << " first_missing_native="
        << (missing_native.empty() ? "" : formatSsrOrbitClockKey(missing_native.front()))
        << " missing_oracle=" << missing_oracle.size()
        << " first_missing_oracle="
        << (missing_oracle.empty() ? "" : formatSsrOrbitClockKey(missing_oracle.front()));
    ASSERT_TRUE(missing_native.empty() && missing_oracle.empty())
        << " missing_native=" << missing_native.size()
        << " first_missing_native="
        << (missing_native.empty() ? "" : formatSsrOrbitClockKey(missing_native.front()))
        << " missing_oracle=" << missing_oracle.size()
        << " first_missing_oracle="
        << (missing_oracle.empty() ? "" : formatSsrOrbitClockKey(missing_oracle.front()));

    for (const auto& [key, oracle_row] : oracle_by_key) {
        const auto native_it = native_by_key.find(key);
        ASSERT_NE(native_it, native_by_key.end()) << "sat=" << std::get<0>(key)
                                                  << " week=" << std::get<1>(key)
                                                  << " tow_ns=" << std::get<2>(key);

        const auto& native = *native_it->second;
        EXPECT_TRUE(native.clock_valid);
        EXPECT_NEAR(native.clock_correction_m, oracle_row->dclk[0], 1e-12)
            << "sat=" << std::get<0>(key) << " tow_ns=" << std::get<2>(key);
        EXPECT_EQ(native.orbit_valid, oracle_row->orbit_valid)
            << "sat=" << std::get<0>(key) << " tow_ns=" << std::get<2>(key);
        if (native.orbit_valid && oracle_row->orbit_valid) {
            EXPECT_NEAR(native.orbit_correction_ecef.x(), oracle_row->deph[0], 1e-12);
            EXPECT_NEAR(native.orbit_correction_ecef.y(), oracle_row->deph[1], 1e-12);
            EXPECT_NEAR(native.orbit_correction_ecef.z(), oracle_row->deph[2], 1e-12);
        }
    }
}



TEST(MadocaCoreTest, OracleLinkedL6EFileMatchesMadocalibUraRowsForMizuSample) {
    using namespace libgnss::algorithms::madoca_core;

    if (madocalib_oracle::available() == false) {
        GTEST_SKIP() << "MADOCALIB oracle unavailable; configure with "
                     << "-DMADOCALIB_PARITY_LINK=ON";
    }

    const std::filesystem::path root(madocalib_oracle::rootDirectory());
    const std::filesystem::path l6e_204 =
        root / "sample_data" / "data" / "l6_is-qzss-mdc-004" / "2025" /
        "091" / "2025091A.204.l6";
    const std::filesystem::path l6e_206 =
        root / "sample_data" / "data" / "l6_is-qzss-mdc-004" / "2025" /
        "091" / "2025091A.206.l6";
    if (std::filesystem::exists(l6e_204) == false ||
        std::filesystem::exists(l6e_206) == false) {
        GTEST_SKIP() << "MADOCALIB sample L6E files not found under: " << root;
    }
    const std::vector<std::string> l6e_files = {l6e_204.string(), l6e_206.string()};

    const auto native_epochs = decodeL6EFiles(l6e_files, 2360);
    const auto oracle_rows = madocalib_oracle::decode_l6e_files(l6e_files, 2360);

    ASSERT_FALSE(native_epochs.empty());
    ASSERT_FALSE(oracle_rows.empty());

    std::map<SsrOrbitClockKey, const qzss_l6::CssrEpoch*> native_by_key;
    for (const auto& epoch : native_epochs) {
        const GNSSTime time(epoch.week, epoch.tow);
        for (const auto& [satellite, clock] : epoch.clocks) {
            if (satellite.system == GNSSSystem::UNKNOWN) {
                continue;
            }
            native_by_key.emplace(ssrOrbitClockKey(satellite, time), &epoch);
        }
    }

    std::map<SsrOrbitClockKey, const madocalib_oracle::SsrCorrectionSnapshot*>
        oracle_by_key;
    for (const auto& row : oracle_rows) {
        const SatelliteId satellite = satelliteIdFromMadocalibSat(row.sat);
        if (row.clock_valid == false || satellite.system == GNSSSystem::UNKNOWN) {
            continue;
        }
        oracle_by_key[ssrOrbitClockKey(row)] = &row;
    }

    ASSERT_EQ(native_by_key.size(), oracle_by_key.size());

    int compared_ura_rows = 0;
    for (const auto& [key, oracle_row] : oracle_by_key) {
        const auto native_it = native_by_key.find(key);
        ASSERT_NE(native_it, native_by_key.end())
            << "key=" << formatSsrOrbitClockKey(key);
        const SatelliteId satellite = satelliteIdFromMadocalibSat(oracle_row->sat);
        const auto& native_epoch = *native_it->second;
        const auto native_ura = native_epoch.ura_indices.find(satellite);
        const bool native_valid = native_ura != native_epoch.ura_indices.end();

        ASSERT_EQ(native_valid, oracle_row->ura_valid)
            << "key=" << formatSsrOrbitClockKey(key);
        if (oracle_row->ura_valid) {
            ++compared_ura_rows;
            EXPECT_EQ(native_ura->second, oracle_row->ura)
                << "key=" << formatSsrOrbitClockKey(key);
        }
    }
    EXPECT_GT(compared_ura_rows, 0);
}

TEST(MadocaCoreTest, OracleLinkedL6EFileMatchesMadocalibBiasRowsForMizuSample) {
    using namespace libgnss::algorithms::madoca_core;

    if (madocalib_oracle::available() == false) {
        GTEST_SKIP() << "MADOCALIB oracle unavailable; configure with "
                     << "-DMADOCALIB_PARITY_LINK=ON";
    }

    const std::filesystem::path root(madocalib_oracle::rootDirectory());
    const std::filesystem::path l6e_204 =
        root / "sample_data" / "data" / "l6_is-qzss-mdc-004" / "2025" /
        "091" / "2025091A.204.l6";
    const std::filesystem::path l6e_206 =
        root / "sample_data" / "data" / "l6_is-qzss-mdc-004" / "2025" /
        "091" / "2025091A.206.l6";
    if (std::filesystem::exists(l6e_204) == false ||
        std::filesystem::exists(l6e_206) == false) {
        GTEST_SKIP() << "MADOCALIB sample L6E files not found under: " << root;
    }
    const std::vector<std::string> l6e_files = {l6e_204.string(), l6e_206.string()};

    const auto native_epochs = decodeL6EFiles(l6e_files, 2360);
    const auto oracle_rows = madocalib_oracle::decode_l6e_files(l6e_files, 2360);

    ASSERT_FALSE(native_epochs.empty());
    ASSERT_FALSE(oracle_rows.empty());

    std::map<SsrOrbitClockKey, const qzss_l6::CssrEpoch*> native_by_key;
    for (const auto& epoch : native_epochs) {
        const GNSSTime time(epoch.week, epoch.tow);
        for (const auto& [satellite, clock] : epoch.clocks) {
            if (satellite.system == GNSSSystem::UNKNOWN) {
                continue;
            }
            native_by_key.emplace(ssrOrbitClockKey(satellite, time), &epoch);
        }
    }

    std::map<SsrOrbitClockKey, const madocalib_oracle::SsrCorrectionSnapshot*>
        oracle_by_key;
    for (const auto& row : oracle_rows) {
        const SatelliteId satellite = satelliteIdFromMadocalibSat(row.sat);
        if (row.clock_valid == false || satellite.system == GNSSSystem::UNKNOWN) {
            continue;
        }
        oracle_by_key[ssrOrbitClockKey(row)] = &row;
    }

    ASSERT_FALSE(native_by_key.empty());
    ASSERT_FALSE(oracle_by_key.empty());

    std::vector<SsrOrbitClockKey> missing_native;
    for (const auto& [key, oracle_row] : oracle_by_key) {
        if (native_by_key.count(key) == 0) {
            missing_native.push_back(key);
        }
    }
    std::vector<SsrOrbitClockKey> missing_oracle;
    for (const auto& [key, native_epoch] : native_by_key) {
        if (oracle_by_key.count(key) == 0) {
            missing_oracle.push_back(key);
        }
    }

    ASSERT_EQ(native_by_key.size(), oracle_by_key.size())
        << " missing_native=" << missing_native.size()
        << " first_missing_native="
        << (missing_native.empty() ? "" : formatSsrOrbitClockKey(missing_native.front()))
        << " missing_oracle=" << missing_oracle.size()
        << " first_missing_oracle="
        << (missing_oracle.empty() ? "" : formatSsrOrbitClockKey(missing_oracle.front()));
    ASSERT_TRUE(missing_native.empty())
        << " first_missing_native=" << formatSsrOrbitClockKey(missing_native.front());
    ASSERT_TRUE(missing_oracle.empty())
        << " first_missing_oracle=" << formatSsrOrbitClockKey(missing_oracle.front());

    int compared_code_rows = 0;
    int compared_phase_rows = 0;
    for (const auto& [key, oracle_row] : oracle_by_key) {
        const auto native_it = native_by_key.find(key);
        ASSERT_NE(native_it, native_by_key.end())
            << "key=" << formatSsrOrbitClockKey(key);
        const SatelliteId satellite = satelliteIdFromMadocalibSat(oracle_row->sat);
        const auto& native_epoch = *native_it->second;

        const BiasRows native_code =
            nativeBiasRowsForSatellite(native_epoch.code_biases, satellite);
        const BiasRows oracle_code = oracleBiasRows(oracle_row->cbias,
                                                    oracle_row->vcbias);
        if (native_code.empty() == false || oracle_code.empty() == false) {
            ++compared_code_rows;
        }
        expectBiasRowsNear(native_code, oracle_code, key, "code");

        const BiasRows native_phase =
            nativeBiasRowsForSatellite(native_epoch.phase_biases, satellite);
        const BiasRows oracle_phase = oracleBiasRows(oracle_row->pbias,
                                                     oracle_row->vpbias);
        if (native_phase.empty() == false || oracle_phase.empty() == false) {
            ++compared_phase_rows;
        }
        expectBiasRowsNear(native_phase, oracle_phase, key, "phase");
    }

    EXPECT_GT(compared_code_rows, 0);
    EXPECT_GT(compared_phase_rows, 0);
}


TEST(MadocaCoreTest, StoreCarriesMionoAtmosphereRowsThroughSsrInterpolation) {
    using namespace libgnss::algorithms::madoca_core;

    const int sat_no = madoca_parity::satno(madoca_parity::kSysQzs, 193);
    ASSERT_GT(sat_no, 0);

    const SatelliteId satellite(GNSSSystem::QZSS, 1);
    EXPECT_EQ(satelliteIdFromMadocalibSat(sat_no), satellite);

    const GNSSTime time(2401, 120.5);
    const double target_stec_tecu = 4.75;
    madoca_parity::MionoCorrResult result;
    result.rid = 41;
    result.area_number = 2;
    result.t0[sat_no - 1] = makeMadocaGpsTime(time.week, time.tow);
    result.delay[sat_no - 1] = target_stec_tecu * l1StecDelayFactorMeters(satellite);
    result.std[sat_no - 1] = 0.02;

    NativeCorrectionStore store;
    store.addMionoCorrections(result, 12);

    ASSERT_FALSE(store.empty());
    EXPECT_EQ(store.summary().total, 1U);
    EXPECT_EQ(store.summary().atmosphere, 1U);
    ASSERT_EQ(store.sourceCounts().at(CorrectionSource::L6D), 1U);

    Vector3d orbit = Vector3d::Zero();
    double clock_m = 0.0;
    std::map<std::string, std::string> atmos_tokens;
    ASSERT_TRUE(store.interpolateCorrection(satellite,
                                            time,
                                            orbit,
                                            clock_m,
                                            nullptr,
                                            nullptr,
                                            nullptr,
                                            &atmos_tokens,
                                            12));

    EXPECT_EQ(atmos_tokens.at("atmos_network_id"), "12");
    const double stec_tecu = ppp_atmosphere::atmosphericStecTecu(
        atmos_tokens,
        satellite,
        Vector3d::Zero());
    EXPECT_NEAR(stec_tecu, target_stec_tecu, 1e-12);
}
