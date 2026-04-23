#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

GNSSTime makeTime(int year, int month, int day, int hour, int minute, double second) {
    std::tm epoch_tm{};
    epoch_tm.tm_year = year - 1900;
    epoch_tm.tm_mon = month - 1;
    epoch_tm.tm_mday = day;
    epoch_tm.tm_hour = hour;
    epoch_tm.tm_min = minute;
    epoch_tm.tm_sec = static_cast<int>(std::floor(second));
    const time_t unix_seconds = timegm(&epoch_tm);
    const auto tp = std::chrono::system_clock::from_time_t(unix_seconds) +
        std::chrono::microseconds(
            static_cast<long long>(std::llround((second - std::floor(second)) * 1e6)));
    return GNSSTime::fromSystemTime(tp);
}

int dayOfYearFromTime(const GNSSTime& time) {
    const auto tp = time.toSystemTime();
    const std::time_t unix_time = std::chrono::system_clock::to_time_t(tp);
    const std::tm utc_tm = *gmtime(&unix_time);
    return utc_tm.tm_yday + 1;
}

double modeledPppTroposphereDelay(const Vector3d& receiver_position,
                                  double elevation,
                                  const GNSSTime& time) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    return models::modeledTroposphereDelayClimatology(
        latitude_rad,
        height_m,
        elevation,
        dayOfYearFromTime(time));
}

std::filesystem::path tempFilePath(const std::string& name) {
    return std::filesystem::temp_directory_path() / name;
}

void writeTextFile(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream output(path);
    ASSERT_TRUE(output.is_open());
    output << contents;
    output.close();
}

void writeBinaryFile(const std::filesystem::path& path, const std::vector<uint8_t>& contents) {
    std::ofstream output(path, std::ios::binary);
    ASSERT_TRUE(output.is_open());
    output.write(reinterpret_cast<const char*>(contents.data()),
                 static_cast<std::streamsize>(contents.size()));
    output.close();
}

uint32_t crc24q(const uint8_t* data, size_t length) {
    static const uint32_t table[256] = {
        0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
        0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
        0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
        0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
        0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
        0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
        0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
        0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
        0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
        0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
        0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
        0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
        0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
        0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
        0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
        0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
        0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
        0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
        0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
        0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
        0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
        0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
        0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
        0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
        0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
        0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
        0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
        0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
        0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
        0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
        0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
        0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538
    };
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t table_index = static_cast<uint8_t>(((crc >> 16) ^ data[i]) & 0xFFU);
        crc = (crc << 8) ^ table[table_index];
    }
    return crc & 0x00FFFFFFU;
}

void setUnsignedBits(std::vector<uint8_t>& data, int pos, int len, uint64_t value) {
    for (int i = 0; i < len; ++i) {
        const int bit_index = pos + len - 1 - i;
        const int byte_index = bit_index / 8;
        const int bit_in_byte = 7 - (bit_index % 8);
        const uint8_t mask = static_cast<uint8_t>(1U << bit_in_byte);
        if ((value >> i) & 0x01U) {
            data[byte_index] |= mask;
        } else {
            data[byte_index] &= static_cast<uint8_t>(~mask);
        }
    }
}

void setSignedBits(std::vector<uint8_t>& data, int pos, int len, int64_t value) {
    const uint64_t masked = static_cast<uint64_t>(value) & ((1ULL << len) - 1ULL);
    setUnsignedBits(data, pos, len, masked);
}

std::vector<uint8_t> buildRtcmFrame(const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    frame.reserve(3 + payload.size() + 3);
    frame.push_back(0xD3);
    frame.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0x03U));
    frame.push_back(static_cast<uint8_t>(payload.size() & 0xFFU));
    frame.insert(frame.end(), payload.begin(), payload.end());
    const uint32_t crc = crc24q(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
    return frame;
}

Ephemeris makeBroadcastGpsEphemeris(uint8_t prn) {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GPS, prn);
    eph.week = static_cast<uint16_t>(GNSSTime::fromSystemTime(std::chrono::system_clock::now()).week);
    eph.toe = GNSSTime(eph.week, 345600.0);
    eph.toc = GNSSTime(eph.week, 345616.0);
    eph.toes = eph.toe.tow;
    eph.sqrt_a = 5153.79548931;
    eph.e = 0.0123456789;
    eph.i0 = 0.9599310886;
    eph.omega0 = 1.2345678901;
    eph.omega = -0.9876543210;
    eph.m0 = 0.4567890123;
    eph.delta_n = 4.56789e-09;
    eph.idot = -2.34567e-10;
    eph.i_dot = eph.idot;
    eph.omega_dot = -8.76543e-09;
    eph.cuc = -1.234567e-06;
    eph.cus = 2.345678e-06;
    eph.crc = 245.25;
    eph.crs = -88.75;
    eph.cic = 8.765432e-08;
    eph.cis = -7.654321e-08;
    eph.af0 = 2.345678e-04;
    eph.af1 = -4.567890e-12;
    eph.af2 = 0.0;
    eph.tgd = -1.2345678e-08;
    eph.ura = 2;
    eph.sv_accuracy = 4.85;
    eph.health = 0;
    eph.sv_health = 0.0;
    eph.iode = 77;
    eph.iodc = 301;
    eph.valid = true;
    return eph;
}

std::vector<uint8_t> buildGps1060CombinedFrame(uint8_t prn) {
    constexpr int total_bits = 68 + 205;
    std::vector<uint8_t> payload((total_bits + 7) / 8U, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1060); bit += 12;
    setUnsignedBits(payload, bit, 20, 345600); bit += 20;
    setUnsignedBits(payload, bit, 4, 2); bit += 4;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setUnsignedBits(payload, bit, 4, 7); bit += 4;
    setUnsignedBits(payload, bit, 16, 21); bit += 16;
    setUnsignedBits(payload, bit, 4, 3); bit += 4;
    setUnsignedBits(payload, bit, 6, 1); bit += 6;
    setUnsignedBits(payload, bit, 6, prn); bit += 6;
    setUnsignedBits(payload, bit, 8, 77); bit += 8;
    setSignedBits(payload, bit, 22, 800); bit += 22;
    setSignedBits(payload, bit, 20, -100); bit += 20;
    setSignedBits(payload, bit, 20, 50); bit += 20;
    setSignedBits(payload, bit, 21, 0); bit += 21;
    setSignedBits(payload, bit, 19, 0); bit += 19;
    setSignedBits(payload, bit, 19, 0); bit += 19;
    setSignedBits(payload, bit, 22, 1200); bit += 22;
    setSignedBits(payload, bit, 21, 0); bit += 21;
    setSignedBits(payload, bit, 27, 0); bit += 27;
    return buildRtcmFrame(payload);
}

std::vector<uint8_t> buildGps1062HighRateClockFrame(uint8_t prn, int32_t high_rate_units) {
    constexpr int total_bits = 67 + 28;
    std::vector<uint8_t> payload((total_bits + 7) / 8U, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1062); bit += 12;
    setUnsignedBits(payload, bit, 20, 345600); bit += 20;
    setUnsignedBits(payload, bit, 4, 2); bit += 4;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 4, 7); bit += 4;
    setUnsignedBits(payload, bit, 16, 21); bit += 16;
    setUnsignedBits(payload, bit, 4, 3); bit += 4;
    setUnsignedBits(payload, bit, 6, 1); bit += 6;
    setUnsignedBits(payload, bit, 6, prn); bit += 6;
    setSignedBits(payload, bit, 22, high_rate_units); bit += 22;
    return buildRtcmFrame(payload);
}

std::vector<uint8_t> buildGps1059CodeBiasFrame(uint8_t prn,
                                               uint8_t signal_id,
                                               int32_t bias_centimeters) {
    constexpr int total_bits = 67 + 6 + 5 + 19;
    std::vector<uint8_t> payload((total_bits + 7) / 8U, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1059); bit += 12;
    setUnsignedBits(payload, bit, 20, 345600); bit += 20;
    setUnsignedBits(payload, bit, 4, 2); bit += 4;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 4, 7); bit += 4;
    setUnsignedBits(payload, bit, 16, 21); bit += 16;
    setUnsignedBits(payload, bit, 4, 3); bit += 4;
    setUnsignedBits(payload, bit, 6, 1); bit += 6;
    setUnsignedBits(payload, bit, 6, prn); bit += 6;
    setUnsignedBits(payload, bit, 5, 1); bit += 5;
    setUnsignedBits(payload, bit, 5, signal_id); bit += 5;
    setSignedBits(payload, bit, 14, bias_centimeters); bit += 14;
    return buildRtcmFrame(payload);
}

std::vector<uint8_t> buildGps1061UraFrame(uint8_t prn, uint8_t ura_index) {
    constexpr int total_bits = 67 + 12;
    std::vector<uint8_t> payload((total_bits + 7) / 8U, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1061); bit += 12;
    setUnsignedBits(payload, bit, 20, 345600); bit += 20;
    setUnsignedBits(payload, bit, 4, 2); bit += 4;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 4, 7); bit += 4;
    setUnsignedBits(payload, bit, 16, 21); bit += 16;
    setUnsignedBits(payload, bit, 4, 3); bit += 4;
    setUnsignedBits(payload, bit, 6, 1); bit += 6;
    setUnsignedBits(payload, bit, 6, prn); bit += 6;
    setUnsignedBits(payload, bit, 6, ura_index); bit += 6;
    return buildRtcmFrame(payload);
}

double mappingFunction(double elevation) {
    const double sin_elevation = std::max(std::sin(elevation), 0.1);
    return 1.001 / std::sqrt(0.002001 + sin_elevation * sin_elevation);
}

struct SyntheticSatellite {
    SatelliteId id;
    Vector3d position;
};

std::vector<SyntheticSatellite> makeSyntheticSatellites(const Vector3d& receiver_position) {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(receiver_position, lat, lon, h);

    struct LookAngle {
        double azimuth_deg;
        double elevation_deg;
    };

    const std::vector<LookAngle> look_angles = {
        {0.0, 55.0},
        {60.0, 48.0},
        {120.0, 62.0},
        {180.0, 43.0},
        {240.0, 68.0},
        {300.0, 37.0},
    };

    std::vector<SyntheticSatellite> satellites;
    const double range_m = 26'500'000.0;
    for (size_t i = 0; i < look_angles.size(); ++i) {
        const double az = look_angles[i].azimuth_deg * M_PI / 180.0;
        const double el = look_angles[i].elevation_deg * M_PI / 180.0;
        const double horizontal = range_m * std::cos(el);
        const Vector3d enu(
            horizontal * std::sin(az),
            horizontal * std::cos(az),
            range_m * std::sin(el));
        satellites.push_back({
            SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(i + 1)),
            receiver_position + enu2ecef(enu, lat, lon),
        });
    }
    return satellites;
}

ObservationData makeSyntheticEpoch(const GNSSTime& time,
                                   const Vector3d& true_receiver_position,
                                   const Vector3d& approximate_receiver_position,
                                   const std::vector<SyntheticSatellite>& satellites) {
    NavigationData nav;
    ObservationData epoch(time);
    epoch.receiver_position = approximate_receiver_position;

    for (const auto& satellite : satellites) {
        const auto geometry = nav.calculateGeometry(true_receiver_position, satellite.position);
        const double pseudorange = geodist(satellite.position, true_receiver_position);
        const double trop_delay = modeledPppTroposphereDelay(
            true_receiver_position,
            geometry.elevation,
            time);

        Observation l1(satellite.id, SignalType::GPS_L1CA);
        l1.valid = true;
        l1.has_pseudorange = true;
        l1.has_carrier_phase = true;
        l1.pseudorange = pseudorange + trop_delay;
        l1.carrier_phase = (pseudorange + trop_delay) / constants::GPS_L1_WAVELENGTH;
        l1.snr = 48.0;
        epoch.addObservation(l1);

        Observation l2(satellite.id, SignalType::GPS_L2C);
        l2.valid = true;
        l2.has_pseudorange = true;
        l2.has_carrier_phase = true;
        l2.pseudorange = pseudorange + trop_delay;
        l2.carrier_phase = (pseudorange + trop_delay) / constants::GPS_L2_WAVELENGTH;
        l2.snr = 45.0;
        epoch.addObservation(l2);
    }

    return epoch;
}

double ionosphereDelayMetersForFrequency(double frequency_hz, double stec_tecu) {
    return 40.3e16 * stec_tecu / (frequency_hz * frequency_hz);
}

ObservationData makeSyntheticEpochWithAtmosphericBiases(
    const GNSSTime& time,
    const Vector3d& true_receiver_position,
    const Vector3d& approximate_receiver_position,
    const std::vector<SyntheticSatellite>& satellites,
    double zenith_trop_delay_m,
    double stec_base_tecu,
    SignalType secondary_signal = SignalType::GPS_L2C) {
    NavigationData nav;
    ObservationData epoch(time);
    epoch.receiver_position = approximate_receiver_position;

    for (const auto& satellite : satellites) {
        const auto geometry = nav.calculateGeometry(true_receiver_position, satellite.position);
        const double range_m = geodist(satellite.position, true_receiver_position);
        const double trop_delay_m =
            modeledPppTroposphereDelay(true_receiver_position, geometry.elevation, time) +
            zenith_trop_delay_m * mappingFunction(geometry.elevation);
        const double stec_tecu = stec_base_tecu + 0.5 * static_cast<double>(satellite.id.prn);
        const double l1_iono_m =
            ionosphereDelayMetersForFrequency(constants::GPS_L1_FREQ, stec_tecu);
        const double secondary_frequency_hz = signalFrequencyHz(secondary_signal);
        const double secondary_wavelength_m = signalWavelengthMeters(secondary_signal);
        const double l2_iono_m =
            ionosphereDelayMetersForFrequency(secondary_frequency_hz, stec_tecu);

        Observation l1(satellite.id, SignalType::GPS_L1CA);
        l1.valid = true;
        l1.has_pseudorange = true;
        l1.has_carrier_phase = true;
        l1.pseudorange = range_m + trop_delay_m + l1_iono_m;
        l1.carrier_phase = (range_m + trop_delay_m - l1_iono_m) / constants::GPS_L1_WAVELENGTH;
        l1.snr = 48.0;
        epoch.addObservation(l1);

        Observation l2(satellite.id, secondary_signal);
        l2.valid = true;
        l2.has_pseudorange = true;
        l2.has_carrier_phase = true;
        l2.pseudorange = range_m + trop_delay_m + l2_iono_m;
        l2.carrier_phase = (range_m + trop_delay_m - l2_iono_m) / secondary_wavelength_m;
        l2.snr = 45.0;
        epoch.addObservation(l2);
    }

    return epoch;
}

std::string buildAtmosphericSsrText(const std::vector<SyntheticSatellite>& satellites,
                                    const Vector3d& true_receiver_position,
                                    const GNSSTime& time,
                                    double zenith_trop_delay_m,
                                    double stec_base_tecu,
                                    double stec_std_l1_m = 0.5,
                                    const std::string& extra_tokens = "") {
    NavigationData nav;
    std::ostringstream text;
    text << "# week,tow,sat,dx,dy,dz,dclock_m[,atmos_<name>=<value>...]\n";
    for (const auto& satellite : satellites) {
        const auto geometry = nav.calculateGeometry(true_receiver_position, satellite.position);
        const double trop_delay_m = zenith_trop_delay_m * mappingFunction(geometry.elevation);
        const double stec_tecu = stec_base_tecu + 0.5 * static_cast<double>(satellite.id.prn);
        text << time.week << "," << time.tow << "," << satellite.id.toString()
             << ",0.0,0.0,0.0,0.0"
             << ",atmos_trop_t00_m=" << std::fixed << std::setprecision(6) << trop_delay_m
             << ",atmos_stec_c00_tecu:" << satellite.id.toString() << "="
             << std::fixed << std::setprecision(6) << stec_tecu
             << ",atmos_stec_std_l1_m:" << satellite.id.toString() << "="
             << std::fixed << std::setprecision(6) << stec_std_l1_m
             << extra_tokens << "\n";
    }
    return text.str();
}

std::string buildSp3Text(const std::vector<SyntheticSatellite>& satellites,
                         const GNSSTime& first_time,
                         const GNSSTime& second_time) {
    auto toCalendar = [](const GNSSTime& time) {
        const auto tp = time.toSystemTime();
        const std::time_t unix_time = std::chrono::system_clock::to_time_t(tp);
        std::tm utc_tm = *gmtime(&unix_time);
        const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
            tp.time_since_epoch()).count() % 1'000'000;
        char buffer[64];
        std::snprintf(
            buffer,
            sizeof(buffer),
            "*  %04d %02d %02d %02d %02d %011.8f\n",
            utc_tm.tm_year + 1900,
            utc_tm.tm_mon + 1,
            utc_tm.tm_mday,
            utc_tm.tm_hour,
            utc_tm.tm_min,
            static_cast<double>(utc_tm.tm_sec) + static_cast<double>(micros) * 1e-6);
        return std::string(buffer);
    };

    std::string text;
    text += toCalendar(first_time);
    for (const auto& satellite : satellites) {
        char line[160];
        std::snprintf(
            line,
            sizeof(line),
            "P%s %14.6f %14.6f %14.6f %14.6f\n",
            satellite.id.toString().c_str(),
            satellite.position.x() / 1000.0,
            satellite.position.y() / 1000.0,
            satellite.position.z() / 1000.0,
            0.0);
        text += line;
    }
    text += toCalendar(second_time);
    for (const auto& satellite : satellites) {
        char line[160];
        std::snprintf(
            line,
            sizeof(line),
            "P%s %14.6f %14.6f %14.6f %14.6f\n",
            satellite.id.toString().c_str(),
            satellite.position.x() / 1000.0,
            satellite.position.y() / 1000.0,
            satellite.position.z() / 1000.0,
            0.0);
        text += line;
    }
    return text;
}

std::string buildClockText(const std::vector<SyntheticSatellite>& satellites,
                           const GNSSTime& first_time,
                           const GNSSTime& second_time) {
    auto toCalendarFields = [](const GNSSTime& time) {
        const auto tp = time.toSystemTime();
        const std::time_t unix_time = std::chrono::system_clock::to_time_t(tp);
        std::tm utc_tm = *gmtime(&unix_time);
        const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
            tp.time_since_epoch()).count() % 1'000'000;
        char buffer[64];
        std::snprintf(
            buffer,
            sizeof(buffer),
            "%04d %02d %02d %02d %02d %011.8f",
            utc_tm.tm_year + 1900,
            utc_tm.tm_mon + 1,
            utc_tm.tm_mday,
            utc_tm.tm_hour,
            utc_tm.tm_min,
            static_cast<double>(utc_tm.tm_sec) + static_cast<double>(micros) * 1e-6);
        return std::string(buffer);
    };

    std::string text = "     3.00           C                   RINEX VERSION / TYPE\n";
    text += "END OF HEADER\n";
    const std::array<GNSSTime, 2> epochs = {first_time, second_time};
    for (const auto& epoch : epochs) {
        for (const auto& satellite : satellites) {
            text += "AS " + satellite.id.toString() + " " + toCalendarFields(epoch) +
                "  2  0.000000000000E+00  1.000000000000E-12\n";
        }
    }
    return text;
}

std::string buildSimpleReceiverAntexText(const std::string& antenna_type,
                                         double l1_north_mm,
                                         double l1_east_mm,
                                         double l1_up_mm,
                                         double l2_north_mm,
                                         double l2_east_mm,
                                         double l2_up_mm,
                                         bool include_pcv = false,
                                         double l1_pcv_start_mm = 0.0,
                                         double l1_pcv_step_mm = 0.0,
                                         double l2_pcv_start_mm = 0.0,
                                         double l2_pcv_step_mm = 0.0) {
    std::ostringstream text;
    auto append_noazi = [&](double start_mm, double step_mm) {
        std::ostringstream values;
        values << std::fixed << std::setprecision(1) << "   NOAZI";
        for (int i = 0; i < 19; ++i) {
            values << std::setw(8) << (start_mm + step_mm * static_cast<double>(i));
        }
        text << values.str() << "\n";
    };
    text << std::left << std::setw(60) << "     1.4            M                                       "
         << "ANTEX VERSION / SYST\n";
    text << std::left << std::setw(60) << ""
         << "START OF ANTENNA\n";
    std::ostringstream type_line;
    type_line << std::left << std::setw(20) << antenna_type << std::setw(20) << "NONE";
    text << std::left << std::setw(60) << type_line.str()
         << "TYPE / SERIAL NO\n";
    text << std::left << std::setw(60) << "     2"
         << "# OF FREQUENCIES\n";
    text << std::left << std::setw(60) << "   G01"
         << "START OF FREQUENCY\n";
    {
        std::ostringstream offsets;
        offsets << std::fixed << std::setprecision(1)
                << std::setw(10) << l1_north_mm
                << std::setw(10) << l1_east_mm
                << std::setw(10) << l1_up_mm;
        text << std::left << std::setw(60) << offsets.str()
             << "NORTH / EAST / UP\n";
    }
    if (include_pcv) {
        append_noazi(l1_pcv_start_mm, l1_pcv_step_mm);
    }
    text << std::left << std::setw(60) << ""
         << "END OF FREQUENCY\n";
    text << std::left << std::setw(60) << "   G02"
         << "START OF FREQUENCY\n";
    {
        std::ostringstream offsets;
        offsets << std::fixed << std::setprecision(1)
                << std::setw(10) << l2_north_mm
                << std::setw(10) << l2_east_mm
                << std::setw(10) << l2_up_mm;
        text << std::left << std::setw(60) << offsets.str()
             << "NORTH / EAST / UP\n";
    }
    if (include_pcv) {
        append_noazi(l2_pcv_start_mm, l2_pcv_step_mm);
    }
    text << std::left << std::setw(60) << ""
         << "END OF FREQUENCY\n";
    text << std::left << std::setw(60) << ""
         << "END OF ANTENNA\n";
    return text.str();
}

std::string buildSimpleSatelliteAntexText(const std::vector<SyntheticSatellite>& satellites,
                                          double l1_north_mm,
                                          double l1_east_mm,
                                          double l1_up_mm,
                                          double l2_north_mm,
                                          double l2_east_mm,
                                          double l2_up_mm) {
    std::ostringstream text;
    text << std::left << std::setw(60) << "     1.4            M                                       "
         << "ANTEX VERSION / SYST\n";
    for (const auto& satellite : satellites) {
        text << std::left << std::setw(60) << ""
             << "START OF ANTENNA\n";
        std::ostringstream type_line;
        type_line << std::left << std::setw(20) << "BLOCK TEST"
                  << std::setw(20) << satellite.id.toString();
        text << std::left << std::setw(60) << type_line.str()
             << "TYPE / SERIAL NO\n";
        text << std::left << std::setw(60) << "  2020     1     1     0     0    0.0000000"
             << "VALID FROM\n";
        text << std::left << std::setw(60) << "     2"
             << "# OF FREQUENCIES\n";
        text << std::left << std::setw(60) << "   G01"
             << "START OF FREQUENCY\n";
        {
            std::ostringstream offsets;
            offsets << std::fixed << std::setprecision(1)
                    << std::setw(10) << l1_north_mm
                    << std::setw(10) << l1_east_mm
                    << std::setw(10) << l1_up_mm;
            text << std::left << std::setw(60) << offsets.str()
                 << "NORTH / EAST / UP\n";
        }
        text << std::left << std::setw(60) << ""
             << "END OF FREQUENCY\n";
        text << std::left << std::setw(60) << "   G02"
             << "START OF FREQUENCY\n";
        {
            std::ostringstream offsets;
            offsets << std::fixed << std::setprecision(1)
                    << std::setw(10) << l2_north_mm
                    << std::setw(10) << l2_east_mm
                    << std::setw(10) << l2_up_mm;
            text << std::left << std::setw(60) << offsets.str()
                 << "NORTH / EAST / UP\n";
        }
        text << std::left << std::setw(60) << ""
             << "END OF FREQUENCY\n";
        text << std::left << std::setw(60) << ""
             << "END OF ANTENNA\n";
    }
    return text.str();
}

std::string buildSimpleBlqText(const std::string& station_name,
                               double up_amplitude_m,
                               double west_amplitude_m,
                               double south_amplitude_m) {
    auto rowText = [](double first_value) {
        std::ostringstream row;
        row << std::fixed << std::setprecision(6) << std::setw(10) << first_value;
        for (int i = 1; i < 11; ++i) {
            row << std::setw(10) << 0.0;
        }
        return row.str();
    };

    std::ostringstream text;
    text << "$$ Synthetic BLQ coefficients\n";
    text << station_name << "\n";
    text << rowText(up_amplitude_m) << "\n";
    text << rowText(west_amplitude_m) << "\n";
    text << rowText(south_amplitude_m) << "\n";
    text << rowText(0.0) << "\n";
    text << rowText(0.0) << "\n";
    text << rowText(0.0) << "\n";
    return text.str();
}

}  // namespace

TEST(PPPTest, NiellHydrostaticMappingTracksHeightAndSeason) {
    constexpr double latitude_rad = 45.0 * M_PI / 180.0;
    constexpr double elevation_rad = 10.0 * M_PI / 180.0;
    const double low_height_mapping =
        models::niellHydrostaticMapping(latitude_rad, 0.0, elevation_rad, 15);
    const double high_height_mapping =
        models::niellHydrostaticMapping(latitude_rad, 2500.0, elevation_rad, 15);
    const double summer_mapping =
        models::niellHydrostaticMapping(latitude_rad, 0.0, elevation_rad, 200);
    const double wet_mapping = models::niellWetMapping(latitude_rad, elevation_rad);
    const double legacy_mapping =
        1.001 / std::sqrt(0.002001 + std::pow(std::sin(elevation_rad), 2));

    EXPECT_GT(low_height_mapping, 1.0);
    EXPECT_GT(wet_mapping, 1.0);
    EXPECT_GT(high_height_mapping, low_height_mapping);
    EXPECT_GT(std::abs(low_height_mapping - summer_mapping), 1e-4);
    EXPECT_LT(low_height_mapping, legacy_mapping);
}

TEST(PPPTest, ZenithTroposphereClimatologyTracksLatitudeSeasonAndHeight) {
    constexpr double tropical_lat_rad = 15.0 * M_PI / 180.0;
    constexpr double mid_lat_rad = 45.0 * M_PI / 180.0;
    constexpr double polar_lat_rad = 75.0 * M_PI / 180.0;

    const auto tropical =
        models::estimateZenithTroposphereClimatology(tropical_lat_rad, 0.0, 30);
    const auto mid_winter =
        models::estimateZenithTroposphereClimatology(mid_lat_rad, 0.0, 20);
    const auto mid_summer =
        models::estimateZenithTroposphereClimatology(mid_lat_rad, 0.0, 200);
    const auto mountain =
        models::estimateZenithTroposphereClimatology(mid_lat_rad, 2500.0, 20);
    const auto polar =
        models::estimateZenithTroposphereClimatology(polar_lat_rad, 0.0, 20);

    EXPECT_GT(tropical.totalDelayMeters(), 2.0);
    EXPECT_GT(tropical.wet_delay_m, polar.wet_delay_m);
    EXPECT_GT(mid_winter.totalDelayMeters(), mountain.totalDelayMeters());
    EXPECT_GT(std::abs(mid_winter.wet_delay_m - mid_summer.wet_delay_m), 0.01);
    EXPECT_GT(mid_winter.hydrostatic_delay_m, 2.0);
    EXPECT_GT(mid_winter.temperature_k, mountain.temperature_k);
}

TEST(PPPTest, ModeledTroposphereDelayClimatologyUsesSeparateHydroAndWetMappings) {
    constexpr double latitude_rad = 35.0 * M_PI / 180.0;
    constexpr double height_m = 420.0;
    constexpr int day_of_year = 84;
    constexpr double low_elevation = 12.0 * M_PI / 180.0;
    constexpr double high_elevation = 68.0 * M_PI / 180.0;

    const auto zenith =
        models::estimateZenithTroposphereClimatology(latitude_rad, height_m, day_of_year);
    const double low_delay = models::modeledTroposphereDelayClimatology(
        latitude_rad, height_m, low_elevation, day_of_year);
    const double high_delay = models::modeledTroposphereDelayClimatology(
        latitude_rad, height_m, high_elevation, day_of_year);

    const double low_hydro_mapping =
        models::niellHydrostaticMapping(latitude_rad, height_m, low_elevation, day_of_year);
    const double low_wet_mapping = models::niellWetMapping(latitude_rad, low_elevation);
    const double blended_low_delay =
        0.5 * (low_hydro_mapping + low_wet_mapping) * zenith.totalDelayMeters();

    EXPECT_GT(low_delay, high_delay);
    EXPECT_GT(low_delay, zenith.totalDelayMeters());
    EXPECT_NEAR(
        low_delay,
        zenith.hydrostatic_delay_m * low_hydro_mapping +
            zenith.wet_delay_m * low_wet_mapping,
        1e-9);
    EXPECT_GT(std::abs(low_delay - blended_low_delay), 1e-3);
}

TEST(PPPTest, SolidEarthTidesChangeSyntheticPrecisePppSolutionWithoutBreakingIt) {
    const auto sp3_path = tempFilePath("libgnss_ppp_tides_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_tides_test.clk");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(8.0, -5.0, 3.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 6, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));

    PPPProcessor::PPPConfig base_config;
    base_config.use_precise_orbits = true;
    base_config.use_precise_clocks = true;
    base_config.orbit_file_path = sp3_path.string();
    base_config.clock_file_path = clk_path.string();
    base_config.convergence_min_epochs = 5;
    base_config.convergence_threshold_horizontal = 0.2;
    base_config.estimate_troposphere = false;
    base_config.enable_ambiguity_resolution = false;
    base_config.kinematic_mode = false;
    base_config.apply_ocean_loading = false;

    PPPProcessor::PPPConfig tides_on_config = base_config;
    tides_on_config.apply_solid_earth_tides = true;
    PPPProcessor::PPPConfig tides_off_config = base_config;
    tides_off_config.apply_solid_earth_tides = false;

    PPPProcessor tides_on_processor(tides_on_config);
    PPPProcessor tides_off_processor(tides_off_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(tides_on_processor.initialize(processor_config));
    ASSERT_TRUE(tides_off_processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution tides_on_solution;
    PositionSolution tides_off_solution;
    for (int i = 0; i < 8; ++i) {
        const ObservationData epoch = makeSyntheticEpoch(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites);
        tides_on_solution = tides_on_processor.processEpoch(epoch, nav_data);
        tides_off_solution = tides_off_processor.processEpoch(epoch, nav_data);
    }

    ASSERT_TRUE(tides_on_solution.isValid());
    ASSERT_TRUE(tides_off_solution.isValid());
    EXPECT_LT((tides_on_solution.position_ecef - true_receiver_position).norm(), 1.0);
    EXPECT_LT((tides_off_solution.position_ecef - true_receiver_position).norm(), 1.0);

    const double position_delta_m =
        (tides_on_solution.position_ecef - tides_off_solution.position_ecef).norm();
    EXPECT_GT(position_delta_m, 1e-4);
    EXPECT_LT(position_delta_m, 0.5);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
}

TEST(PPPTest, ReceiverAntexPcoChangesSyntheticPrecisePppSolutionWithoutBreakingIt) {
    const auto sp3_path = tempFilePath("libgnss_ppp_antex_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_antex_test.clk");
    const auto antex_path = tempFilePath("libgnss_ppp_antex_test.atx");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(antex_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(8.0, -5.0, 3.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 7, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));
    writeTextFile(
        antex_path,
        buildSimpleReceiverAntexText("TEST_ANTENNA", 2.0, -1.0, 80.0, 4.0, -2.0, 110.0));

    PPPProcessor::PPPConfig base_config;
    base_config.use_precise_orbits = true;
    base_config.use_precise_clocks = true;
    base_config.orbit_file_path = sp3_path.string();
    base_config.clock_file_path = clk_path.string();
    base_config.convergence_min_epochs = 5;
    base_config.convergence_threshold_horizontal = 0.2;
    base_config.estimate_troposphere = false;
    base_config.enable_ambiguity_resolution = false;
    base_config.kinematic_mode = false;

    PPPProcessor::PPPConfig plain_config = base_config;
    PPPProcessor::PPPConfig antex_config = base_config;
    antex_config.antex_file_path = antex_path.string();
    antex_config.receiver_antenna_type = "TEST_ANTENNA";
    antex_config.receiver_antenna_delta_enu = Vector3d(0.02, -0.01, 0.25);

    PPPProcessor plain_processor(plain_config);
    PPPProcessor antex_processor(antex_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(plain_processor.initialize(processor_config));
    ASSERT_TRUE(antex_processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution plain_solution;
    PositionSolution antex_solution;
    for (int i = 0; i < 8; ++i) {
        const ObservationData epoch = makeSyntheticEpoch(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites);
        plain_solution = plain_processor.processEpoch(epoch, nav_data);
        antex_solution = antex_processor.processEpoch(epoch, nav_data);
    }

    ASSERT_TRUE(plain_solution.isValid());
    ASSERT_TRUE(antex_solution.isValid());
    EXPECT_LT((plain_solution.position_ecef - true_receiver_position).norm(), 1.0);
    EXPECT_LT((antex_solution.position_ecef - true_receiver_position).norm(), 1.0);
    const double position_delta_m =
        (plain_solution.position_ecef - antex_solution.position_ecef).norm();
    EXPECT_GT(position_delta_m, 1e-4);
    EXPECT_LT(position_delta_m, 1.0);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(antex_path);
}


TEST(PPPTest, ReceiverAntexPcvChangesSyntheticPrecisePppSolutionWithoutBreakingIt) {
    const auto sp3_path = tempFilePath("libgnss_ppp_antex_pcv_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_antex_pcv_test.clk");
    const auto antex_path = tempFilePath("libgnss_ppp_antex_pcv_test.atx");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(antex_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(8.0, -5.0, 3.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 7, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));
    writeTextFile(
        antex_path,
        buildSimpleReceiverAntexText(
            "TEST_ANTENNA", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            true, 0.0, 10.0, 5.0, 14.0));

    PPPProcessor::PPPConfig base_config;
    base_config.use_precise_orbits = true;
    base_config.use_precise_clocks = true;
    base_config.orbit_file_path = sp3_path.string();
    base_config.clock_file_path = clk_path.string();
    base_config.convergence_min_epochs = 5;
    base_config.convergence_threshold_horizontal = 0.2;
    base_config.estimate_troposphere = false;
    base_config.enable_ambiguity_resolution = false;
    base_config.kinematic_mode = false;

    PPPProcessor::PPPConfig plain_config = base_config;
    PPPProcessor::PPPConfig antex_config = base_config;
    antex_config.antex_file_path = antex_path.string();
    antex_config.receiver_antenna_type = "TEST_ANTENNA";

    PPPProcessor plain_processor(plain_config);
    PPPProcessor antex_processor(antex_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(plain_processor.initialize(processor_config));
    ASSERT_TRUE(antex_processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution plain_solution;
    PositionSolution antex_solution;
    for (int i = 0; i < 8; ++i) {
        const ObservationData epoch = makeSyntheticEpoch(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites);
        plain_solution = plain_processor.processEpoch(epoch, nav_data);
        antex_solution = antex_processor.processEpoch(epoch, nav_data);
    }

    ASSERT_TRUE(plain_solution.isValid());
    ASSERT_TRUE(antex_solution.isValid());
    EXPECT_LT((plain_solution.position_ecef - true_receiver_position).norm(), 1.0);
    EXPECT_LT((antex_solution.position_ecef - true_receiver_position).norm(), 1.0);
    const double position_delta_m =
        (plain_solution.position_ecef - antex_solution.position_ecef).norm();
    EXPECT_GT(position_delta_m, 1e-4);
    EXPECT_LT(position_delta_m, 1.0);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(antex_path);
}

TEST(PPPTest, SatelliteAntexPcoChangesSyntheticPrecisePppSolutionWithoutBreakingIt) {
    const auto sp3_path = tempFilePath("libgnss_ppp_sat_antex_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_sat_antex_test.clk");
    const auto antex_path = tempFilePath("libgnss_ppp_sat_antex_test.atx");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(antex_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(8.0, -5.0, 3.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 7, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));
    writeTextFile(
        antex_path,
        buildSimpleSatelliteAntexText(satellites, 250.0, -120.0, 800.0, 180.0, -80.0, 650.0));

    PPPProcessor::PPPConfig base_config;
    base_config.use_precise_orbits = true;
    base_config.use_precise_clocks = true;
    base_config.orbit_file_path = sp3_path.string();
    base_config.clock_file_path = clk_path.string();
    base_config.convergence_min_epochs = 5;
    base_config.convergence_threshold_horizontal = 0.2;
    base_config.estimate_troposphere = false;
    base_config.enable_ambiguity_resolution = false;
    base_config.kinematic_mode = false;

    PPPProcessor::PPPConfig plain_config = base_config;
    PPPProcessor::PPPConfig antex_config = base_config;
    antex_config.antex_file_path = antex_path.string();

    PPPProcessor plain_processor(plain_config);
    PPPProcessor antex_processor(antex_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(plain_processor.initialize(processor_config));
    ASSERT_TRUE(antex_processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution plain_solution;
    PositionSolution antex_solution;
    for (int i = 0; i < 8; ++i) {
        const ObservationData epoch = makeSyntheticEpoch(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites);
        plain_solution = plain_processor.processEpoch(epoch, nav_data);
        antex_solution = antex_processor.processEpoch(epoch, nav_data);
    }

    ASSERT_TRUE(plain_solution.isValid());
    ASSERT_TRUE(antex_solution.isValid());
    EXPECT_LT((plain_solution.position_ecef - true_receiver_position).norm(), 1.0);
    EXPECT_LT((antex_solution.position_ecef - true_receiver_position).norm(), 2.0);
    const double position_delta_m =
        (plain_solution.position_ecef - antex_solution.position_ecef).norm();
    EXPECT_GT(position_delta_m, 1e-3);
    EXPECT_LT(position_delta_m, 2.0);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(antex_path);
}

TEST(PPPTest, OceanLoadingCoefficientsChangeSyntheticPrecisePppSolutionWithoutBreakingIt) {
    const auto sp3_path = tempFilePath("libgnss_ppp_blq_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_blq_test.clk");
    const auto blq_path = tempFilePath("libgnss_ppp_blq_test.blq");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(blq_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(8.0, -5.0, 3.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 8, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));
    writeTextFile(blq_path, buildSimpleBlqText("TESTMARK", 0.008, 0.003, 0.002));

    PPPProcessor::PPPConfig base_config;
    base_config.use_precise_orbits = true;
    base_config.use_precise_clocks = true;
    base_config.orbit_file_path = sp3_path.string();
    base_config.clock_file_path = clk_path.string();
    base_config.convergence_min_epochs = 5;
    base_config.convergence_threshold_horizontal = 0.2;
    base_config.estimate_troposphere = false;
    base_config.enable_ambiguity_resolution = false;
    base_config.kinematic_mode = false;
    base_config.apply_solid_earth_tides = false;
    base_config.apply_ocean_loading = false;

    PPPProcessor::PPPConfig blq_config = base_config;
    blq_config.apply_ocean_loading = true;
    blq_config.ocean_loading_file_path = blq_path.string();
    blq_config.ocean_loading_station_name = "TESTMARK";

    PPPProcessor base_processor(base_config);
    PPPProcessor blq_processor(blq_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(base_processor.initialize(processor_config));
    ASSERT_TRUE(blq_processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution base_solution;
    PositionSolution blq_solution;
    for (int i = 0; i < 8; ++i) {
        const ObservationData epoch = makeSyntheticEpoch(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites);
        base_solution = base_processor.processEpoch(epoch, nav_data);
        blq_solution = blq_processor.processEpoch(epoch, nav_data);
    }

    ASSERT_TRUE(base_solution.isValid());
    ASSERT_TRUE(blq_solution.isValid());
    EXPECT_LT((base_solution.position_ecef - true_receiver_position).norm(), 1.0);
    EXPECT_LT((blq_solution.position_ecef - true_receiver_position).norm(), 1.0);
    const double position_delta_m =
        (base_solution.position_ecef - blq_solution.position_ecef).norm();
    EXPECT_GT(position_delta_m, 1e-5);
    EXPECT_LT(position_delta_m, 0.5);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(blq_path);
}

TEST(PPPTest, PreciseProductsLoadSp3AndClockAndInterpolateMidpoint) {
    const auto sp3_path = tempFilePath("libgnss_ppp_precise_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_precise_test.clk");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);

    const std::string sp3_text =
        "*  2026 03 26 00 00 00.00000000\n"
        "PG01   20200.000000   14000.000000   21700.000000       123.000000\n"
        "*  2026 03 26 00 15 00.00000000\n"
        "PG01   20201.500000   14001.500000   21701.500000       223.000000\n";
    const std::string clk_text =
        "     3.00           C                   RINEX VERSION / TYPE\n"
        "END OF HEADER\n"
        "AS G01 2026 03 26 00 00 00.00000000  2  1.230000000000E-04  1.000000000000E-12\n"
        "AS G01 2026 03 26 00 15 00.00000000  2  2.230000000000E-04  1.000000000000E-12\n";

    writeTextFile(sp3_path, sp3_text);
    writeTextFile(clk_path, clk_text);

    PreciseProducts precise_products;
    ASSERT_TRUE(precise_products.loadSP3File(sp3_path.string()));
    ASSERT_TRUE(precise_products.loadClockFile(clk_path.string()));

    Vector3d position = Vector3d::Zero();
    Vector3d velocity = Vector3d::Zero();
    double clock_bias = 0.0;
    double clock_drift = 0.0;
    const GNSSTime midpoint = makeTime(2026, 3, 26, 0, 7, 30.0);
    ASSERT_TRUE(precise_products.interpolateOrbitClock(
        SatelliteId(GNSSSystem::GPS, 1), midpoint, position, velocity, clock_bias, clock_drift));

    EXPECT_NEAR(position.x(), 20'200'750.0, 1e-3);
    EXPECT_NEAR(position.y(), 14'000'750.0, 1e-3);
    EXPECT_NEAR(position.z(), 21'700'750.0, 1e-3);
    EXPECT_NEAR(clock_bias, 1.73e-4, 1e-12);
    EXPECT_NEAR(velocity.norm(), std::sqrt(3.0) * (1500.0 / 900.0), 1e-6);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
}

TEST(PPPTest, IONEXProductsLoadAndInterpolateTecAndRms) {
    const auto ionex_path = tempFilePath("libgnss_ppp_ionex_test.ionex");
    std::filesystem::remove(ionex_path);

    const std::string ionex_text =
        "     1.0           I                   G                   IONEX VERSION / TYPE\n"
        "  3600                                                      INTERVAL\n"
        "    -1                                                      EXPONENT\n"
        "    0.0   10.0   10.0                                       LAT1 / LAT2 / DLAT\n"
        "    0.0   10.0   10.0                                       LON1 / LON2 / DLON\n"
        "  450.0  450.0    0.0                                       HGT1 / HGT2 / DHGT\n"
        "                                                            END OF HEADER\n"
        "    1                                                      START OF TEC MAP\n"
        " 2026     3    26     0     0     0                        EPOCH OF CURRENT MAP\n"
        "    0.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "  100 200\n"
        "   10.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "  300 400\n"
        "                                                            END OF TEC MAP\n"
        "    1                                                      START OF RMS MAP\n"
        " 2026     3    26     0     0     0                        EPOCH OF CURRENT MAP\n"
        "    0.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "   10 20\n"
        "   10.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "   30 40\n"
        "                                                            END OF RMS MAP\n"
        "    2                                                      START OF TEC MAP\n"
        " 2026     3    26     1     0     0                        EPOCH OF CURRENT MAP\n"
        "    0.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "  200 300\n"
        "   10.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "  400 500\n"
        "                                                            END OF TEC MAP\n"
        "    2                                                      START OF RMS MAP\n"
        " 2026     3    26     1     0     0                        EPOCH OF CURRENT MAP\n"
        "    0.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "   20 30\n"
        "   10.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "   40 50\n"
        "                                                            END OF RMS MAP\n";
    writeTextFile(ionex_path, ionex_text);

    IONEXProducts ionex_products;
    ASSERT_TRUE(ionex_products.loadIONEXFile(ionex_path.string()));
    ASSERT_EQ(ionex_products.tec_maps.size(), 2U);
    ASSERT_EQ(ionex_products.rms_maps.size(), 2U);
    EXPECT_EQ(ionex_products.interval_s, 3600);
    EXPECT_EQ(ionex_products.exponent, -1);

    double tecu = 0.0;
    double rms_tecu = 0.0;
    ASSERT_TRUE(ionex_products.interpolateTecu(
        makeTime(2026, 3, 26, 0, 30, 0.0), 5.0, 5.0, tecu, &rms_tecu));
    EXPECT_NEAR(tecu, 30.0, 1e-9);
    EXPECT_NEAR(rms_tecu, 3.0, 1e-9);

    std::filesystem::remove(ionex_path);
}

TEST(PPPTest, DCBProductsLoadBiasSinexAndLookupEntry) {
    const auto dcb_path = tempFilePath("libgnss_ppp_dcb_test.bsx");
    std::filesystem::remove(dcb_path);

    const std::string dcb_text =
        "%=BIA 1.00 TEST TEST 2024:002:00000 TEST\n"
        "+BIAS/SOLUTION\n"
        "*BIAS SVN PRN STATION OBS1 OBS2 BEGIN END UNIT EST STDDEV\n"
        " DSB G01 C1C C2W 2024:002:00000 2024:003:00000 ns 1.234 0.100\n"
        " OSB E11 C1C C5Q 2024:002:00000 2024:003:00000 ns 0.321 0.050\n"
        "-BIAS/SOLUTION\n";
    writeTextFile(dcb_path, dcb_text);

    DCBProducts dcb_products;
    ASSERT_TRUE(dcb_products.loadFile(dcb_path.string()));
    ASSERT_EQ(dcb_products.entries.size(), 2U);

    double bias = 0.0;
    double sigma = 0.0;
    ASSERT_TRUE(dcb_products.getBias(
        SatelliteId(GNSSSystem::GPS, 1), "DSB", "C1C", "C2W", bias, &sigma));
    EXPECT_NEAR(bias, 1.234, 1e-12);
    EXPECT_NEAR(sigma, 0.100, 1e-12);
    EXPECT_FALSE(dcb_products.getBias(
        SatelliteId(GNSSSystem::GPS, 1), "DSB", "C1C", "C5Q", bias, &sigma));

    std::filesystem::remove(dcb_path);
}

TEST(PPPTest, SSRProductsLoadCsvAndInterpolateMidpoint) {
    const auto ssr_path = tempFilePath("libgnss_ppp_ssr_test.csv");
    std::filesystem::remove(ssr_path);

    const std::string ssr_text =
        "# week,tow,sat,dx,dy,dz,dclock_m\n"
        "2414,345600.0,G01,1.0,-2.0,3.0,4.0\n"
        "2414,345660.0,G01,3.0,-4.0,5.0,8.0\n";
    writeTextFile(ssr_path, ssr_text);

    SSRProducts ssr_products;
    ASSERT_TRUE(ssr_products.loadCSVFile(ssr_path.string()));

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    ASSERT_TRUE(ssr_products.interpolateCorrection(
        SatelliteId(GNSSSystem::GPS, 1),
        GNSSTime(2414, 345630.0),
        orbit_correction,
        clock_correction_m));

    EXPECT_NEAR(orbit_correction.x(), 2.0, 1e-9);
    EXPECT_NEAR(orbit_correction.y(), -3.0, 1e-9);
    EXPECT_NEAR(orbit_correction.z(), 4.0, 1e-9);
    EXPECT_NEAR(clock_correction_m, 6.0, 1e-9);

    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, SSRProductsLoadCsvParsesOptionalUraCodeBiasPhaseBiasAndAtmosTokens) {
    const auto ssr_path = tempFilePath("libgnss_ppp_ssr_optional_tokens_test.csv");
    std::filesystem::remove(ssr_path);

    const std::string ssr_text =
        "# week,tow,sat,dx,dy,dz,dclock_m[,ura_sigma_m=<m>][,cbias:<id>=<m>...][,pbias:<id>=<m>...][,atmos_<name>=<value>...]\n"
        "2414,345600.0,G01,1.0,-2.0,3.0,4.0,ura_sigma_m=0.002750,cbias:2=-0.120000,cbias:8=0.050000,pbias:2=0.015000,atmos_network_id=1,atmos_trop_quality=9\n";
    writeTextFile(ssr_path, ssr_text);

    SSRProducts ssr_products;
    ASSERT_TRUE(ssr_products.loadCSVFile(ssr_path.string()));

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    double ura_sigma_m = 0.0;
    std::map<uint8_t, double> code_bias_m;
    std::map<uint8_t, double> phase_bias_m;
    std::map<std::string, std::string> atmos_tokens;
    ASSERT_TRUE(ssr_products.interpolateCorrection(
        SatelliteId(GNSSSystem::GPS, 1),
        GNSSTime(2414, 345600.0),
        orbit_correction,
        clock_correction_m,
        &ura_sigma_m,
        &code_bias_m,
        &phase_bias_m,
        &atmos_tokens));

    EXPECT_NEAR(ura_sigma_m, 0.00275, 1e-12);
    ASSERT_EQ(code_bias_m.size(), 2U);
    EXPECT_NEAR(code_bias_m.at(2U), -0.12, 1e-12);
    EXPECT_NEAR(code_bias_m.at(8U), 0.05, 1e-12);
    ASSERT_EQ(phase_bias_m.size(), 1U);
    EXPECT_NEAR(phase_bias_m.at(2U), 0.015, 1e-12);
    ASSERT_EQ(atmos_tokens.size(), 2U);
    EXPECT_EQ(atmos_tokens.at("atmos_network_id"), "1");
    EXPECT_EQ(atmos_tokens.at("atmos_trop_quality"), "9");

    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, SSRProductsExactAtmosOnlyRowsDoNotSuppressOrbitClockInterpolation) {
    SSRProducts ssr_products;
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    SSROrbitClockCorrection before;
    before.satellite = satellite;
    before.time = GNSSTime(2414, 345570.0);
    before.orbit_correction_ecef = Vector3d(1.0, -2.0, 3.0);
    before.clock_correction_m = 4.0;
    before.ura_sigma_m = 0.5;
    before.orbit_valid = true;
    before.clock_valid = true;
    before.ura_valid = true;
    ssr_products.addCorrection(before);

    SSROrbitClockCorrection after = before;
    after.time = GNSSTime(2414, 345630.0);
    after.orbit_correction_ecef = Vector3d(3.0, -4.0, 5.0);
    after.clock_correction_m = 8.0;
    after.ura_sigma_m = 0.9;
    ssr_products.addCorrection(after);

    SSROrbitClockCorrection atmos;
    atmos.satellite = satellite;
    atmos.time = GNSSTime(2414, 345600.0);
    atmos.atmos_valid = true;
    atmos.atmos_network_id = 5;
    atmos.atmos_tokens["atmos_network_id"] = "5";
    atmos.atmos_tokens["atmos_stec_avail"] = "1";
    ssr_products.addCorrection(atmos);

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    double ura_sigma_m = 0.0;
    std::map<std::string, std::string> atmos_tokens;
    ASSERT_TRUE(ssr_products.interpolateCorrection(
        satellite,
        GNSSTime(2414, 345600.0),
        orbit_correction,
        clock_correction_m,
        &ura_sigma_m,
        nullptr,
        nullptr,
        &atmos_tokens));

    EXPECT_NEAR(orbit_correction.x(), 2.0, 1e-12);
    EXPECT_NEAR(orbit_correction.y(), -3.0, 1e-12);
    EXPECT_NEAR(orbit_correction.z(), 4.0, 1e-12);
    EXPECT_NEAR(clock_correction_m, 6.0, 1e-12);
    EXPECT_NEAR(ura_sigma_m, 0.7, 1e-12);
    ASSERT_EQ(atmos_tokens.size(), 2U);
    EXPECT_EQ(atmos_tokens.at("atmos_network_id"), "5");
}

TEST(PPPTest, SSRProductsLeadingAtmosOnlyRowsUseNearbyOrbitClockSample) {
    SSRProducts ssr_products;
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    SSROrbitClockCorrection atmos;
    atmos.satellite = satellite;
    atmos.time = GNSSTime(2414, 345606.0);
    atmos.atmos_valid = true;
    atmos.atmos_network_id = 5;
    atmos.atmos_tokens["atmos_network_id"] = "5";
    atmos.atmos_tokens["atmos_stec_avail"] = "1";
    ssr_products.addCorrection(atmos);

    SSROrbitClockCorrection orbit_clock;
    orbit_clock.satellite = satellite;
    orbit_clock.time = GNSSTime(2414, 345635.0);
    orbit_clock.orbit_correction_ecef = Vector3d(1.0, -2.0, 3.0);
    orbit_clock.clock_correction_m = 4.0;
    orbit_clock.ura_sigma_m = 0.5;
    orbit_clock.orbit_valid = true;
    orbit_clock.clock_valid = true;
    orbit_clock.ura_valid = true;
    ssr_products.addCorrection(orbit_clock);

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    double ura_sigma_m = 0.0;
    std::map<std::string, std::string> atmos_tokens;
    ASSERT_TRUE(ssr_products.interpolateCorrection(
        satellite,
        GNSSTime(2414, 345600.0),
        orbit_correction,
        clock_correction_m,
        &ura_sigma_m,
        nullptr,
        nullptr,
        &atmos_tokens));

    EXPECT_NEAR(orbit_correction.x(), 1.0, 1e-12);
    EXPECT_NEAR(orbit_correction.y(), -2.0, 1e-12);
    EXPECT_NEAR(orbit_correction.z(), 3.0, 1e-12);
    EXPECT_NEAR(clock_correction_m, 4.0, 1e-12);
    EXPECT_NEAR(ura_sigma_m, 0.5, 1e-12);
    ASSERT_EQ(atmos_tokens.size(), 2U);
    EXPECT_EQ(atmos_tokens.at("atmos_network_id"), "5");
}

TEST(PPPTest, SSRProductsInterveningAtmosRowsDoNotHideNearbyUra) {
    SSRProducts ssr_products;
    const SatelliteId satellite(GNSSSystem::Galileo, 4);

    SSROrbitClockCorrection atmos;
    atmos.satellite = satellite;
    atmos.time = GNSSTime(2414, 345606.0);
    atmos.atmos_valid = true;
    atmos.atmos_network_id = 5;
    atmos.atmos_tokens["atmos_network_id"] = "5";
    ssr_products.addCorrection(atmos);

    SSROrbitClockCorrection orbit_clock;
    orbit_clock.satellite = satellite;
    orbit_clock.time = GNSSTime(2414, 345635.0);
    orbit_clock.orbit_correction_ecef = Vector3d(1.0, 2.0, 3.0);
    orbit_clock.clock_correction_m = 4.0;
    orbit_clock.orbit_valid = true;
    orbit_clock.clock_valid = true;
    ssr_products.addCorrection(orbit_clock);

    SSROrbitClockCorrection ura_sample = orbit_clock;
    ura_sample.time = GNSSTime(2414, 345640.0);
    ura_sample.ura_sigma_m = 0.026;
    ura_sample.ura_valid = true;
    ssr_products.addCorrection(ura_sample);

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    double ura_sigma_m = 0.0;
    std::map<std::string, std::string> atmos_tokens;
    ASSERT_TRUE(ssr_products.interpolateCorrection(
        satellite,
        GNSSTime(2414, 345630.0),
        orbit_correction,
        clock_correction_m,
        &ura_sigma_m,
        nullptr,
        nullptr,
        &atmos_tokens));

    EXPECT_NEAR(orbit_correction.x(), 1.0, 1e-12);
    EXPECT_NEAR(clock_correction_m, 4.0, 1e-12);
    EXPECT_NEAR(ura_sigma_m, 0.026, 1e-12);
    ASSERT_EQ(atmos_tokens.size(), 1U);
    EXPECT_EQ(atmos_tokens.at("atmos_network_id"), "5");
}

TEST(PPPTest, SSRProductsCausalModeDoesNotUseFutureOrbitClockCorrections) {
    SSRProducts ssr_products;
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    SSROrbitClockCorrection before;
    before.satellite = satellite;
    before.time = GNSSTime(2414, 345570.0);
    before.orbit_correction_ecef = Vector3d(1.0, -2.0, 3.0);
    before.clock_correction_m = 4.0;
    before.orbit_valid = true;
    before.clock_valid = true;
    ssr_products.addCorrection(before);

    SSROrbitClockCorrection after = before;
    after.time = GNSSTime(2414, 345630.0);
    after.orbit_correction_ecef = Vector3d(3.0, -4.0, 5.0);
    after.clock_correction_m = 8.0;
    ssr_products.addCorrection(after);

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    ASSERT_TRUE(ssr_products.interpolateCorrection(satellite,
                                                   GNSSTime(2414, 345600.0),
                                                   orbit_correction,
                                                   clock_correction_m,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   0,
                                                   false,
                                                   true));
    EXPECT_NEAR(orbit_correction.x(), 1.0, 1e-12);
    EXPECT_NEAR(orbit_correction.y(), -2.0, 1e-12);
    EXPECT_NEAR(orbit_correction.z(), 3.0, 1e-12);
    EXPECT_NEAR(clock_correction_m, 4.0, 1e-12);

    SSRProducts future_only;
    future_only.addCorrection(after);
    orbit_correction = Vector3d(9.0, 9.0, 9.0);
    clock_correction_m = 9.0;
    EXPECT_FALSE(future_only.interpolateCorrection(satellite,
                                                   GNSSTime(2414, 345600.0),
                                                   orbit_correction,
                                                   clock_correction_m,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   0,
                                                   false,
                                                   true));
    EXPECT_TRUE(orbit_correction.isZero(0.0));
    EXPECT_DOUBLE_EQ(clock_correction_m, 0.0);
}

TEST(PPPTest, NavigationDataSelectsEphemerisByIode) {
    NavigationData nav_data;

    Ephemeris matching = makeBroadcastGpsEphemeris(1);
    matching.iode = 77;
    matching.toe = GNSSTime(matching.week, 345600.0);
    matching.toc = matching.toe;
    matching.toes = matching.toe.tow;

    Ephemeris closer_wrong_iode = matching;
    closer_wrong_iode.iode = 78;
    closer_wrong_iode.toe = GNSSTime(matching.week, 345630.0);
    closer_wrong_iode.toc = closer_wrong_iode.toe;
    closer_wrong_iode.toes = closer_wrong_iode.toe.tow;

    nav_data.addEphemeris(matching);
    nav_data.addEphemeris(closer_wrong_iode);

    const GNSSTime query_time(matching.week, 345625.0);
    const Ephemeris* default_eph = nav_data.getEphemeris(matching.satellite, query_time);
    ASSERT_NE(default_eph, nullptr);
    EXPECT_EQ(default_eph->iode, 78);

    const Ephemeris* matched_eph = nav_data.getEphemerisByIode(matching.satellite, 77, query_time);
    ASSERT_NE(matched_eph, nullptr);
    EXPECT_EQ(matched_eph->iode, 77);
    EXPECT_EQ(nav_data.getEphemerisByIode(matching.satellite, 99, query_time), nullptr);
}

TEST(PPPTest, NavigationDataRtklibSelectionSkipsCurrentGalileoToe) {
    NavigationData nav_data;

    Ephemeris previous = makeBroadcastGpsEphemeris(6);
    previous.satellite = SatelliteId(GNSSSystem::Galileo, 6);
    previous.iode = 34;
    previous.toe = GNSSTime(previous.week, 172800.0);
    previous.toc = previous.toe;
    previous.toes = previous.toe.tow;

    Ephemeris strict_ssr = previous;
    strict_ssr.iode = 33;
    strict_ssr.toe = GNSSTime(previous.week, 172200.0);
    strict_ssr.toc = strict_ssr.toe;
    strict_ssr.toes = strict_ssr.toe.tow;

    Ephemeris current = previous;
    current.iode = 35;
    current.toe = GNSSTime(previous.week, 173400.0);
    current.toc = current.toe;
    current.toes = current.toe.tow;

    nav_data.addEphemeris(strict_ssr);
    nav_data.addEphemeris(previous);
    nav_data.addEphemeris(current);

    const GNSSTime query_time(previous.week, 173400.0);
    const Ephemeris* default_eph = nav_data.getEphemeris(previous.satellite, query_time);
    ASSERT_NE(default_eph, nullptr);
    EXPECT_EQ(default_eph->iode, 35);

    const Ephemeris* rtklib_eph = nav_data.getRtklibEphemeris(previous.satellite, query_time);
    ASSERT_NE(rtklib_eph, nullptr);
    EXPECT_EQ(rtklib_eph->iode, 34);
}

TEST(PPPTest, NavigationDataMatchesBeiDouSsrIodeByToeModulo) {
    NavigationData nav_data;

    Ephemeris matching = makeBroadcastGpsEphemeris(29);
    matching.satellite = SatelliteId(GNSSSystem::BeiDou, 29);
    matching.iode = 5;
    matching.toe = GNSSTime(matching.week, 345600.0);
    matching.toc = matching.toe;
    matching.toes = 2048.0 + 12.0 * 8.0;

    Ephemeris wrong_toe = matching;
    wrong_toe.iode = 12;
    wrong_toe.toe = GNSSTime(matching.week, 345630.0);
    wrong_toe.toc = wrong_toe.toe;
    wrong_toe.toes = 2048.0 + 13.0 * 8.0;

    nav_data.addEphemeris(matching);
    nav_data.addEphemeris(wrong_toe);

    const Ephemeris* matched_eph = nav_data.getEphemerisByIode(
        matching.satellite, 12, GNSSTime(matching.week, 345625.0));
    ASSERT_NE(matched_eph, nullptr);
    EXPECT_DOUBLE_EQ(matched_eph->toes, matching.toes);
    EXPECT_EQ(nav_data.getEphemerisByIode(
                  matching.satellite, 99, GNSSTime(matching.week, 345625.0)),
              nullptr);
}

TEST(PPPTest, SSRProductsReturnsOrbitIodeAndAvoidsCrossIodeOrbitInterpolation) {
    SSRProducts ssr_products;
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    SSROrbitClockCorrection before;
    before.satellite = satellite;
    before.time = GNSSTime(2414, 345570.0);
    before.orbit_correction_ecef = Vector3d(1.0, -2.0, 3.0);
    before.clock_correction_m = 4.0;
    before.orbit_iode = 77;
    before.orbit_valid = true;
    before.clock_valid = true;
    ssr_products.addCorrection(before);

    SSROrbitClockCorrection after = before;
    after.time = GNSSTime(2414, 345630.0);
    after.orbit_correction_ecef = Vector3d(3.0, -4.0, 5.0);
    after.clock_correction_m = 8.0;
    after.orbit_iode = 78;
    ssr_products.addCorrection(after);

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    int orbit_iode = -1;
    ASSERT_TRUE(ssr_products.interpolateCorrection(satellite,
                                                   GNSSTime(2414, 345600.0),
                                                   orbit_correction,
                                                   clock_correction_m,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   nullptr,
                                                   0,
                                                   true,
                                                   true,
                                                   &orbit_iode));

    EXPECT_NEAR(orbit_correction.x(), 1.0, 1e-12);
    EXPECT_NEAR(orbit_correction.y(), -2.0, 1e-12);
    EXPECT_NEAR(orbit_correction.z(), 3.0, 1e-12);
    EXPECT_NEAR(clock_correction_m, 6.0, 1e-12);
    EXPECT_EQ(orbit_iode, 77);
}

TEST(PPPTest, ProcessorReportsSsrOrbitIodeMismatchDiagnostics) {
    const auto ssr_path = tempFilePath("libgnss_ppp_iode_mismatch_diagnostic.csv");
    std::filesystem::remove(ssr_path);

    Ephemeris eph = makeBroadcastGpsEphemeris(1);
    eph.iode = 77;
    eph.valid = true;
    const GNSSTime epoch_time = eph.toe;

    std::ostringstream ssr_text;
    ssr_text << epoch_time.week << "," << epoch_time.tow
             << ",G01,0.1,0.0,0.0,0.2,orbit_iode=99\n";
    writeTextFile(ssr_path, ssr_text.str());

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(10.0, -5.0, 3.0);
    std::vector<SyntheticSatellite> satellites = makeSyntheticSatellites(true_receiver_position);
    satellites.resize(1);
    ObservationData epoch = makeSyntheticEpoch(
        epoch_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites);

    NavigationData nav_data;
    nav_data.addEphemeris(eph);

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = false;
    ppp_config.use_precise_clocks = false;
    ppp_config.use_ssr_corrections = true;
    ppp_config.require_ssr_orbit_clock = true;
    ppp_config.enforce_ssr_orbit_iode = true;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.prefer_receiver_position_seed = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.convergence_min_epochs = 1;
    PPPProcessor processor(ppp_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 1;
    processor_config.elevation_mask = 0.0;
    ASSERT_TRUE(processor.initialize(processor_config));

    (void)processor.processEpoch(epoch, nav_data);

    const auto& diagnostics = processor.getLastSSRApplicationDiagnostics();
    ASSERT_FALSE(diagnostics.empty());
    bool saw_mismatch = false;
    for (const auto& diagnostic : diagnostics) {
        if (diagnostic.satellite == SatelliteId(GNSSSystem::GPS, 1)) {
            saw_mismatch = true;
            EXPECT_TRUE(diagnostic.ssr_available);
            EXPECT_EQ(diagnostic.ssr_orbit_iode, 99);
            EXPECT_EQ(diagnostic.broadcast_iode, 77);
            EXPECT_FALSE(diagnostic.orbit_clock_applied);
            EXPECT_FALSE(diagnostic.valid_after_corrections);
            EXPECT_EQ(
                diagnostic.orbit_clock_skip_reason,
                "ssr_orbit_iode_no_matching_ephemeris");
        }
    }
    EXPECT_TRUE(saw_mismatch);

    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, ProcessorAdmissionOnlySsrOrbitIodeRejectsMismatch) {
    const auto ssr_path = tempFilePath("libgnss_ppp_iode_admission_only_test.csv");
    std::filesystem::remove(ssr_path);

    Ephemeris eph = makeBroadcastGpsEphemeris(1);
    eph.iode = 77;
    eph.valid = true;
    const GNSSTime epoch_time = eph.toe;

    std::ostringstream ssr_text;
    ssr_text << epoch_time.week << "," << epoch_time.tow
             << ",G01,0.1,0.0,0.0,0.2,orbit_iode=99\n";
    writeTextFile(ssr_path, ssr_text.str());

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(10.0, -5.0, 3.0);
    std::vector<SyntheticSatellite> satellites = makeSyntheticSatellites(true_receiver_position);
    satellites.resize(1);
    ObservationData epoch = makeSyntheticEpoch(
        epoch_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites);

    NavigationData nav_data;
    nav_data.addEphemeris(eph);

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = false;
    ppp_config.use_precise_clocks = false;
    ppp_config.use_ssr_corrections = true;
    ppp_config.require_ssr_orbit_clock = true;
    ppp_config.enforce_ssr_orbit_iode = false;
    ppp_config.enforce_ssr_orbit_iode_admission_only = true;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.prefer_receiver_position_seed = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.convergence_min_epochs = 1;
    PPPProcessor processor(ppp_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 1;
    processor_config.elevation_mask = 0.0;
    ASSERT_TRUE(processor.initialize(processor_config));

    (void)processor.processEpoch(epoch, nav_data);

    const auto& diagnostics = processor.getLastSSRApplicationDiagnostics();
    ASSERT_FALSE(diagnostics.empty());
    bool saw_mismatch = false;
    for (const auto& diagnostic : diagnostics) {
        if (diagnostic.satellite == SatelliteId(GNSSSystem::GPS, 1)) {
            saw_mismatch = true;
            EXPECT_TRUE(diagnostic.ssr_available);
            EXPECT_EQ(diagnostic.ssr_orbit_iode, 99);
            // Admission-only: broadcast IODE field should reflect the
            // RTKLIB-selected ephemeris, not the mismatched match attempt.
            EXPECT_FALSE(diagnostic.orbit_clock_applied);
            EXPECT_FALSE(diagnostic.valid_after_corrections);
            EXPECT_EQ(
                diagnostic.orbit_clock_skip_reason,
                "ssr_orbit_iode_no_matching_ephemeris");
        }
    }
    EXPECT_TRUE(saw_mismatch);

    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, ProcessorAdmissionOnlySsrOrbitIodeWarmupEpochsAdmitsEarly) {
    const auto ssr_path = tempFilePath("libgnss_ppp_iode_warmup_test.csv");
    std::filesystem::remove(ssr_path);

    Ephemeris eph = makeBroadcastGpsEphemeris(1);
    eph.iode = 77;
    eph.valid = true;
    const GNSSTime epoch_time = eph.toe;

    std::ostringstream ssr_text;
    ssr_text << epoch_time.week << "," << epoch_time.tow
             << ",G01,0.1,0.0,0.0,0.2,orbit_iode=99\n";
    writeTextFile(ssr_path, ssr_text.str());

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(10.0, -5.0, 3.0);
    std::vector<SyntheticSatellite> satellites = makeSyntheticSatellites(true_receiver_position);
    satellites.resize(1);
    ObservationData epoch = makeSyntheticEpoch(
        epoch_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites);

    NavigationData nav_data;
    nav_data.addEphemeris(eph);

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = false;
    ppp_config.use_precise_clocks = false;
    ppp_config.use_ssr_corrections = true;
    ppp_config.require_ssr_orbit_clock = true;
    ppp_config.enforce_ssr_orbit_iode = false;
    ppp_config.enforce_ssr_orbit_iode_admission_only = true;
    ppp_config.ssr_orbit_iode_admission_gate_warmup_epochs = 5;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.prefer_receiver_position_seed = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.convergence_min_epochs = 1;
    PPPProcessor processor(ppp_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 1;
    processor_config.elevation_mask = 0.0;
    ASSERT_TRUE(processor.initialize(processor_config));

    (void)processor.processEpoch(epoch, nav_data);

    // With warmup=5 epochs, the first epoch should NOT reject the IODE
    // mismatch. The SSR is still applied (orbit/clock) using the existing
    // RTKLIB-selection broadcast state.
    const auto& diagnostics = processor.getLastSSRApplicationDiagnostics();
    ASSERT_FALSE(diagnostics.empty());
    bool saw_applied = false;
    for (const auto& diagnostic : diagnostics) {
        if (diagnostic.satellite == SatelliteId(GNSSSystem::GPS, 1)) {
            saw_applied = true;
            EXPECT_TRUE(diagnostic.ssr_available);
            EXPECT_EQ(diagnostic.ssr_orbit_iode, 99);
            // During warmup, the admission-only gate is inactive; the SSR
            // orbit/clock should be applied, and skip_reason should not be
            // the IODE mismatch string.
            EXPECT_NE(
                diagnostic.orbit_clock_skip_reason,
                "ssr_orbit_iode_no_matching_ephemeris");
        }
    }
    EXPECT_TRUE(saw_applied);

    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, ProcessorLoadsRtcmSsrCorrectionsFromFile) {
    const auto rtcm_path = tempFilePath("libgnss_ppp_ssr_rtcm_test.rtcm3");
    std::filesystem::remove(rtcm_path);

    NavigationData nav_data;
    nav_data.addEphemeris(makeBroadcastGpsEphemeris(1));
    writeBinaryFile(rtcm_path, buildGps1060CombinedFrame(1));

    PPPProcessor processor;
    ASSERT_TRUE(processor.loadRTCMSSRProducts(rtcm_path.string(), nav_data, 1.0));
    EXPECT_TRUE(processor.hasLoadedSSRProducts());

    std::filesystem::remove(rtcm_path);
}

TEST(PPPTest, ProcessorAppliesRtcmSsrHighRateClockFromFile) {
    const auto rtcm_path = tempFilePath("libgnss_ppp_ssr_rtcm_high_rate_test.rtcm3");
    std::filesystem::remove(rtcm_path);

    NavigationData nav_data;
    const Ephemeris eph = makeBroadcastGpsEphemeris(1);
    nav_data.addEphemeris(eph);

    std::vector<uint8_t> frames = buildGps1060CombinedFrame(1);
    const auto high_rate_frame = buildGps1062HighRateClockFrame(1, 2500);  // 0.25 m
    frames.insert(frames.end(), high_rate_frame.begin(), high_rate_frame.end());
    writeBinaryFile(rtcm_path, frames);

    PPPProcessor processor;
    ASSERT_TRUE(processor.loadRTCMSSRProducts(rtcm_path.string(), nav_data, 1.0));
    EXPECT_TRUE(processor.hasLoadedSSRProducts());

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    double ura_sigma_m = 0.0;
    ASSERT_TRUE(processor.interpolateLoadedSSRCorrection(
        SatelliteId(GNSSSystem::GPS, 1),
        GNSSTime(eph.toe.week, 345600.0),
        orbit_correction,
        clock_correction_m,
        &ura_sigma_m));
    EXPECT_NEAR(clock_correction_m, 0.12 + 0.25, 1e-6);
    EXPECT_DOUBLE_EQ(ura_sigma_m, 0.0);

    std::filesystem::remove(rtcm_path);
}

TEST(PPPTest, ProcessorLoadsRtcmSsrUraFromFile) {
    const auto rtcm_path = tempFilePath("libgnss_ppp_ssr_rtcm_ura_test.rtcm3");
    std::filesystem::remove(rtcm_path);

    NavigationData nav_data;
    const Ephemeris eph = makeBroadcastGpsEphemeris(1);
    nav_data.addEphemeris(eph);

    std::vector<uint8_t> frames = buildGps1060CombinedFrame(1);
    const auto ura_frame = buildGps1061UraFrame(1, 5);
    frames.insert(frames.end(), ura_frame.begin(), ura_frame.end());
    writeBinaryFile(rtcm_path, frames);

    PPPProcessor processor;
    ASSERT_TRUE(processor.loadRTCMSSRProducts(rtcm_path.string(), nav_data, 1.0));

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    double ura_sigma_m = 0.0;
    ASSERT_TRUE(processor.interpolateLoadedSSRCorrection(
        SatelliteId(GNSSSystem::GPS, 1),
        GNSSTime(eph.toe.week, 345600.0),
        orbit_correction,
        clock_correction_m,
        &ura_sigma_m));
    EXPECT_NEAR(clock_correction_m, 0.12, 1e-6);
    EXPECT_NEAR(ura_sigma_m, 0.00125, 1e-9);

    std::filesystem::remove(rtcm_path);
}

TEST(PPPTest, ProcessorLoadsRtcmSsrCodeBiasFromFile) {
    const auto rtcm_path = tempFilePath("libgnss_ppp_ssr_rtcm_code_bias_test.rtcm3");
    std::filesystem::remove(rtcm_path);

    NavigationData nav_data;
    const Ephemeris eph = makeBroadcastGpsEphemeris(1);
    nav_data.addEphemeris(eph);

    std::vector<uint8_t> frames = buildGps1060CombinedFrame(1);
    const auto code_bias_frame = buildGps1059CodeBiasFrame(1, 2, -12);  // -0.12 m on GPS L1 C/A
    frames.insert(frames.end(), code_bias_frame.begin(), code_bias_frame.end());
    writeBinaryFile(rtcm_path, frames);

    PPPProcessor processor;
    ASSERT_TRUE(processor.loadRTCMSSRProducts(rtcm_path.string(), nav_data, 1.0));

    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    double ura_sigma_m = 0.0;
    std::map<uint8_t, double> code_bias_m;
    ASSERT_TRUE(processor.interpolateLoadedSSRCorrection(
        SatelliteId(GNSSSystem::GPS, 1),
        GNSSTime(eph.toe.week, 345600.0),
        orbit_correction,
        clock_correction_m,
        &ura_sigma_m,
        &code_bias_m));
    ASSERT_EQ(code_bias_m.size(), 1U);
    ASSERT_TRUE(code_bias_m.find(2U) != code_bias_m.end());
    EXPECT_NEAR(code_bias_m.at(2U), -0.12, 1e-9);
    EXPECT_DOUBLE_EQ(ura_sigma_m, 0.0);

    std::filesystem::remove(rtcm_path);
}

TEST(PPPTest, ProcessorProducesConvergedFloatSolutionWithSyntheticPreciseProducts) {
    const auto sp3_path = tempFilePath("libgnss_ppp_processor_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_processor_test.clk");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);

    const Vector3d true_receiver_position = geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position = true_receiver_position + Vector3d(8.0, -5.0, 3.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 1, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.convergence_min_epochs = 5;
    ppp_config.convergence_threshold_horizontal = 0.2;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution last_solution;
    for (int i = 0; i < 8; ++i) {
        const ObservationData epoch = makeSyntheticEpoch(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites);
        last_solution = processor.processEpoch(epoch, nav_data);
        ASSERT_TRUE(last_solution.isValid());
        EXPECT_EQ(last_solution.status, SolutionStatus::PPP_FLOAT);
        EXPECT_GE(last_solution.num_satellites, 6);
    }

    EXPECT_LT((last_solution.position_ecef - true_receiver_position).norm(), 1.0);
    EXPECT_LT(last_solution.residual_rms, 0.2);

    const auto stats = processor.getStats();
    EXPECT_EQ(stats.total_epochs, 8U);
    EXPECT_EQ(stats.valid_solutions, 8U);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
}


TEST(PPPTest, ProcessorSkipsBeiDouGeoSatellites) {
    const auto sp3_path = tempFilePath("libgnss_ppp_bds_geo_gate_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_bds_geo_gate_test.clk");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(8.0, -5.0, 3.0);
    const auto gps_satellites = makeSyntheticSatellites(true_receiver_position);
    std::vector<SyntheticSatellite> precise_satellites = gps_satellites;
    const SyntheticSatellite beidou_geo{
        SatelliteId(GNSSSystem::BeiDou, 3),
        gps_satellites.front().position,
    };
    precise_satellites.push_back(beidou_geo);

    const GNSSTime first_time = makeTime(2026, 3, 26, 5, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(precise_satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(precise_satellites, first_time, last_precise_time));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.convergence_min_epochs = 1;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    ObservationData epoch = makeSyntheticEpoch(
        first_time,
        true_receiver_position,
        approximate_receiver_position,
        gps_satellites);

    NavigationData geometry_nav;
    const auto geometry = geometry_nav.calculateGeometry(
        true_receiver_position,
        beidou_geo.position);
    const double range_m = geodist(beidou_geo.position, true_receiver_position);
    const double trop_delay_m = modeledPppTroposphereDelay(
        true_receiver_position,
        geometry.elevation,
        first_time);

    Observation b1(beidou_geo.id, SignalType::BDS_B1I);
    b1.valid = true;
    b1.has_pseudorange = true;
    b1.has_carrier_phase = true;
    b1.pseudorange = range_m + trop_delay_m;
    b1.carrier_phase = (range_m + trop_delay_m) / constants::BDS_B1I_WAVELENGTH;
    b1.snr = 48.0;
    epoch.addObservation(b1);

    Observation b2(beidou_geo.id, SignalType::BDS_B2I);
    b2.valid = true;
    b2.has_pseudorange = true;
    b2.has_carrier_phase = true;
    b2.pseudorange = range_m + trop_delay_m;
    b2.carrier_phase = (range_m + trop_delay_m) / constants::BDS_B2I_WAVELENGTH;
    b2.snr = 45.0;
    epoch.addObservation(b2);

    NavigationData nav_data;
    const PositionSolution solution = processor.processEpoch(epoch, nav_data);
    ASSERT_TRUE(solution.isValid());
    EXPECT_EQ(solution.num_satellites, static_cast<int>(gps_satellites.size()));
    const std::string beidou_geo_id = beidou_geo.id.toString();
    for (const auto& satellite : solution.satellites_used) {
        EXPECT_NE(satellite.toString(), beidou_geo_id);
    }
    for (const auto& diagnostic : processor.getLastPPPResidualDiagnostics()) {
        EXPECT_NE(diagnostic.satellite.toString(), beidou_geo_id);
    }

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
}

TEST(PPPTest, ProcessorAppliesAtmosphericCorrectionsFromSampledSSRWithPreciseProducts) {
    const auto sp3_path = tempFilePath("libgnss_ppp_atmos_processor_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_atmos_processor_test.clk");
    const auto ssr_path = tempFilePath("libgnss_ppp_atmos_processor_test.csv");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(15.0, -10.0, 6.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 3, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));
    writeTextFile(
        ssr_path,
        buildAtmosphericSsrText(satellites, true_receiver_position, first_time, 2.3, 12.0));

    PPPProcessor::PPPConfig base_config;
    base_config.use_precise_orbits = true;
    base_config.use_precise_clocks = true;
    base_config.orbit_file_path = sp3_path.string();
    base_config.clock_file_path = clk_path.string();
    base_config.use_ionosphere_free = false;
    base_config.estimate_troposphere = false;
    base_config.enable_ambiguity_resolution = false;
    base_config.convergence_min_epochs = 1;
    base_config.kinematic_mode = false;

    PPPProcessor corrected_processor(base_config);
    PPPProcessor::PPPConfig atmos_config = base_config;
    atmos_config.use_ssr_corrections = true;
    atmos_config.ssr_file_path = ssr_path.string();
    PPPProcessor atmos_processor(atmos_config);

    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(corrected_processor.initialize(processor_config));
    ASSERT_TRUE(atmos_processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution baseline_solution;
    PositionSolution corrected_solution;
    for (int i = 0; i < 3; ++i) {
        const ObservationData epoch = makeSyntheticEpochWithAtmosphericBiases(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites,
            2.3,
            12.0);
        baseline_solution = corrected_processor.processEpoch(epoch, nav_data);
        corrected_solution = atmos_processor.processEpoch(epoch, nav_data);
    }

    ASSERT_TRUE(baseline_solution.isValid());
    ASSERT_TRUE(corrected_solution.isValid());
    EXPECT_GT(atmos_processor.getLastAppliedAtmosphericTroposphereCorrections(), 0);
    EXPECT_GT(atmos_processor.getLastAppliedAtmosphericIonosphereCorrections(), 0);
    EXPECT_GT(atmos_processor.getLastAppliedAtmosphericTroposphereMeters(), 0.0);
    EXPECT_GT(atmos_processor.getLastAppliedAtmosphericIonosphereMeters(), 0.0);

    const auto& diagnostics = atmos_processor.getLastSSRApplicationDiagnostics();
    ASSERT_FALSE(diagnostics.empty());
    bool saw_valid_atmospheric_diagnostic = false;
    for (const auto& diagnostic : diagnostics) {
        if (diagnostic.valid_after_corrections && diagnostic.ssr_available &&
            diagnostic.atmos_token_count > 0 &&
            std::abs(diagnostic.trop_correction_m) > 0.0 &&
            std::abs(diagnostic.iono_correction_m) > 0.0) {
            EXPECT_GT(diagnostic.elevation_deg, 0.0);
            EXPECT_GT(diagnostic.variance_pr, 0.0);
            EXPECT_GT(diagnostic.variance_cp, 0.0);
            saw_valid_atmospheric_diagnostic = true;
        }
    }
    EXPECT_TRUE(saw_valid_atmospheric_diagnostic);

    const auto& filter_diagnostics =
        atmos_processor.getLastPPPFilterIterationDiagnostics();
    ASSERT_FALSE(filter_diagnostics.empty());
    EXPECT_GT(filter_diagnostics.back().rows, 0);
    EXPECT_GT(filter_diagnostics.back().code_rows, 0);
    EXPECT_GT(filter_diagnostics.back().phase_rows, 0);
    EXPECT_GE(filter_diagnostics.back().pos_delta_m, 0.0);
    EXPECT_GE(filter_diagnostics.back().code_residual_rms_m, 0.0);
    EXPECT_GE(filter_diagnostics.back().phase_residual_rms_m, 0.0);

    const auto& residual_diagnostics =
        atmos_processor.getLastPPPResidualDiagnostics();
    ASSERT_FALSE(residual_diagnostics.empty());
    bool saw_code_residual = false;
    bool saw_phase_residual = false;
    bool saw_positive_elevation = false;
    for (const auto& diagnostic : residual_diagnostics) {
        saw_code_residual = saw_code_residual || !diagnostic.carrier_phase;
        saw_phase_residual = saw_phase_residual || diagnostic.carrier_phase;
        saw_positive_elevation = saw_positive_elevation || diagnostic.elevation_deg > 0.0;
        EXPECT_GT(diagnostic.variance_m2, 0.0);
        EXPECT_TRUE(std::isfinite(diagnostic.residual_m));
    }
    EXPECT_TRUE(saw_code_residual);
    EXPECT_TRUE(saw_phase_residual);
    EXPECT_TRUE(saw_positive_elevation);

    EXPECT_LT(corrected_solution.residual_rms, baseline_solution.residual_rms);
    EXPECT_LT(
        (corrected_solution.position_ecef - true_receiver_position).norm(),
        (baseline_solution.position_ecef - true_receiver_position).norm());
    EXPECT_LT(corrected_solution.residual_rms, 3.0);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);
}


TEST(PPPTest, ProcessorSkipsHighStdStecConstraint) {
    const Vector3d true_receiver_position(-2700000.0, -4300000.0, 3850000.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(4.0, -3.0, 2.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const auto sp3_path = tempFilePath("ppp_high_std_stec.sp3");
    const auto clk_path = tempFilePath("ppp_high_std_stec.clk");
    const auto ssr_path = tempFilePath("ppp_high_std_stec.ssr");
    const GNSSTime first_time = makeTime(2026, 3, 26, 3, 30, 0.0);
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, first_time + 300.0));
    writeTextFile(clk_path, buildClockText(satellites, first_time, first_time + 300.0));
    writeTextFile(
        ssr_path,
        buildAtmosphericSsrText(satellites, true_receiver_position, first_time, 0.0, 15.0, 5.4665));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.use_ssr_corrections = true;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.use_ionosphere_free = false;
    ppp_config.estimate_ionosphere = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.convergence_min_epochs = 1;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    const ObservationData epoch = makeSyntheticEpochWithAtmosphericBiases(
        first_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites,
        0.0,
        15.0);
    ASSERT_TRUE(processor.processEpoch(epoch, nav_data).isValid());

    bool saw_stec_diagnostic = false;
    for (const auto& diagnostic : processor.getLastSSRApplicationDiagnostics()) {
        if (diagnostic.valid_after_corrections && diagnostic.stec_tecu > 0.0) {
            saw_stec_diagnostic = true;
            EXPECT_FALSE(diagnostic.ionosphere_estimation_constraint);
        }
    }
    EXPECT_TRUE(saw_stec_diagnostic);
    EXPECT_EQ(processor.getLastAppliedAtmosphericIonosphereCorrections(), 0);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);
}


TEST(PPPTest, ProcessorAddsSecondaryCodeRowsInEstimatedIonosphereMode) {
    const auto sp3_path = tempFilePath("libgnss_ppp_multifrequency_code_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_multifrequency_code_test.clk");
    const auto ssr_path = tempFilePath("libgnss_ppp_multifrequency_code_test.csv");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(12.0, -8.0, 4.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 3, 45, 0.0);
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, first_time + 300.0));
    writeTextFile(clk_path, buildClockText(satellites, first_time, first_time + 300.0));
    writeTextFile(
        ssr_path,
        buildAtmosphericSsrText(satellites, true_receiver_position, first_time, 0.0, 15.0));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.use_ssr_corrections = true;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.use_ionosphere_free = false;
    ppp_config.estimate_ionosphere = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.convergence_min_epochs = 1;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    const ObservationData epoch = makeSyntheticEpochWithAtmosphericBiases(
        first_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites,
        0.0,
        15.0);
    const PositionSolution first_solution = processor.processEpoch(epoch, nav_data);
    ASSERT_TRUE(first_solution.isValid());
    EXPECT_EQ(first_solution.num_satellites, static_cast<int>(satellites.size()));
    EXPECT_EQ(first_solution.num_frequencies, 2);

    int l1_diagnostics = 0;
    int l2_diagnostics = 0;
    int l1_constraints = 0;
    int l2_constraints = 0;
    double prn1_l1_iono_m = 0.0;
    double prn1_l2_iono_m = 0.0;
    for (const auto& diagnostic : processor.getLastSSRApplicationDiagnostics()) {
        if (!diagnostic.valid_after_corrections) {
            continue;
        }
        if (diagnostic.primary_signal == SignalType::GPS_L1CA) {
            ++l1_diagnostics;
            if (diagnostic.ionosphere_estimation_constraint) {
                ++l1_constraints;
            }
            if (diagnostic.satellite.prn == 1) {
                prn1_l1_iono_m = diagnostic.iono_correction_m;
            }
        } else if (diagnostic.primary_signal == SignalType::GPS_L2C) {
            ++l2_diagnostics;
            if (diagnostic.ionosphere_estimation_constraint) {
                ++l2_constraints;
            }
            if (diagnostic.satellite.prn == 1) {
                prn1_l2_iono_m = diagnostic.iono_correction_m;
            }
        }
    }
    EXPECT_EQ(l1_diagnostics, static_cast<int>(satellites.size()));
    EXPECT_EQ(l2_diagnostics, static_cast<int>(satellites.size()));
    EXPECT_EQ(l1_constraints, static_cast<int>(satellites.size()));
    EXPECT_EQ(l2_constraints, 0);
    ASSERT_GT(prn1_l1_iono_m, 0.0);
    ASSERT_GT(prn1_l2_iono_m, 0.0);
    EXPECT_NEAR(
        prn1_l2_iono_m / prn1_l1_iono_m,
        std::pow(constants::GPS_L1_FREQ / constants::GPS_L2_FREQ, 2.0),
        1e-9);

    const auto first_filter_diagnostics = processor.getLastPPPFilterIterationDiagnostics();
    ASSERT_FALSE(first_filter_diagnostics.empty());
    EXPECT_EQ(
        first_filter_diagnostics.front().code_rows,
        static_cast<int>(satellites.size() * 2U));
    EXPECT_EQ(
        first_filter_diagnostics.front().ionosphere_constraint_rows,
        static_cast<int>(satellites.size()));

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, ProcessorRejectsRowsMissingRequiredSsrCodeBias) {
    const auto sp3_path = tempFilePath("libgnss_ppp_required_ssr_bias_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_required_ssr_bias_test.clk");
    const auto ssr_path = tempFilePath("libgnss_ppp_required_ssr_bias_test.csv");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(12.0, -8.0, 4.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 3, 45, 0.0);
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, first_time + 300.0));
    writeTextFile(clk_path, buildClockText(satellites, first_time, first_time + 300.0));
    writeTextFile(
        ssr_path,
        buildAtmosphericSsrText(
            satellites,
            true_receiver_position,
            first_time,
            0.0,
            15.0,
            0.5,
            ",cbias:2=0.000000,pbias:2=0.000000"));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.use_ssr_corrections = true;
    ppp_config.require_ssr_observation_biases = true;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.use_ionosphere_free = false;
    ppp_config.estimate_ionosphere = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.convergence_min_epochs = 1;
    ppp_config.enable_outlier_detection = false;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    const ObservationData epoch = makeSyntheticEpochWithAtmosphericBiases(
        first_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites,
        0.0,
        15.0);
    ASSERT_TRUE(processor.processEpoch(epoch, nav_data).isValid());

    int l1_valid = 0;
    int l2_rejected = 0;
    for (const auto& diagnostic : processor.getLastSSRApplicationDiagnostics()) {
        if (diagnostic.primary_signal == SignalType::GPS_L1CA) {
            if (diagnostic.valid_after_corrections) {
                ++l1_valid;
            }
        } else if (diagnostic.primary_signal == SignalType::GPS_L2C) {
            if (!diagnostic.valid_after_corrections &&
                diagnostic.orbit_clock_skip_reason == "ssr_code_bias_unavailable") {
                ++l2_rejected;
            }
        }
    }
    EXPECT_EQ(l1_valid, static_cast<int>(satellites.size()));
    EXPECT_EQ(l2_rejected, static_cast<int>(satellites.size()));

    const auto filter_diagnostics = processor.getLastPPPFilterIterationDiagnostics();
    ASSERT_FALSE(filter_diagnostics.empty());
    EXPECT_EQ(filter_diagnostics.front().code_rows, static_cast<int>(satellites.size()));
    EXPECT_EQ(filter_diagnostics.front().ionosphere_constraint_rows, static_cast<int>(satellites.size()));

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, ProcessorAppliesRawSecondaryBiasesToEstimatedIonosphereRows) {
    const auto sp3_path = tempFilePath("libgnss_ppp_secondary_bias_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_secondary_bias_test.clk");
    const auto ssr_path = tempFilePath("libgnss_ppp_secondary_bias_test.csv");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(12.0, -8.0, 4.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 3, 45, 0.0);
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, first_time + 300.0));
    writeTextFile(clk_path, buildClockText(satellites, first_time, first_time + 300.0));
    writeTextFile(
        ssr_path,
        buildAtmosphericSsrText(
            satellites,
            true_receiver_position,
            first_time,
            0.0,
            15.0,
            0.5,
            ",cbias:2=-0.120000,cbias:9=0.340000,pbias:2=0.015000,pbias:9=-0.025000"));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.use_ssr_corrections = true;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.use_ionosphere_free = false;
    ppp_config.estimate_ionosphere = true;
    ppp_config.enable_per_frequency_phase_bias_states = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.convergence_min_epochs = 1;
    ppp_config.enable_outlier_detection = false;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    const ObservationData epoch = makeSyntheticEpochWithAtmosphericBiases(
        first_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites,
        0.0,
        15.0,
        SignalType::GPS_L2P);
    ASSERT_TRUE(processor.processEpoch(epoch, nav_data).isValid());

    bool saw_l1_bias = false;
    bool saw_l2_bias = false;
    for (const auto& diagnostic : processor.getLastSSRApplicationDiagnostics()) {
        if (!diagnostic.valid_after_corrections || diagnostic.satellite.prn != 1) {
            continue;
        }
        if (diagnostic.primary_signal == SignalType::GPS_L1CA) {
            saw_l1_bias = true;
            EXPECT_EQ(diagnostic.secondary_signal, SignalType::GPS_L2P);
            EXPECT_EQ(diagnostic.frequency_index, 0);
            EXPECT_NEAR(diagnostic.ionosphere_coefficient, 1.0, 1e-12);
            EXPECT_TRUE(diagnostic.has_carrier_phase);
            EXPECT_NEAR(diagnostic.code_bias_m, -0.12, 1e-12);
            EXPECT_NEAR(diagnostic.phase_bias_m, 0.015, 1e-12);
            EXPECT_TRUE(diagnostic.ionosphere_estimation_constraint);
        } else if (diagnostic.primary_signal == SignalType::GPS_L2P) {
            saw_l2_bias = true;
            EXPECT_EQ(diagnostic.secondary_signal, SignalType::SIGNAL_TYPE_COUNT);
            EXPECT_EQ(diagnostic.frequency_index, 1);
            EXPECT_GT(diagnostic.ionosphere_coefficient, 1.0);
            EXPECT_TRUE(diagnostic.has_carrier_phase);
            EXPECT_NEAR(diagnostic.code_bias_m, 0.34, 1e-12);
            EXPECT_NEAR(diagnostic.phase_bias_m, -0.025, 1e-12);
            EXPECT_GT(diagnostic.iono_correction_m, 0.0);
            EXPECT_FALSE(diagnostic.ionosphere_estimation_constraint);
        }
    }
    EXPECT_TRUE(saw_l1_bias);
    EXPECT_TRUE(saw_l2_bias);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, ProcessorAddsSecondaryCarrierRowsWhenPerFrequencyStatesEnabled) {
    const auto sp3_path = tempFilePath("libgnss_ppp_multifrequency_phase_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_multifrequency_phase_test.clk");
    const auto ssr_path = tempFilePath("libgnss_ppp_multifrequency_phase_test.csv");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(12.0, -8.0, 4.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 3, 45, 0.0);
    const GNSSTime second_time = first_time + 30.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, first_time + 300.0));
    writeTextFile(clk_path, buildClockText(satellites, first_time, first_time + 300.0));
    writeTextFile(
        ssr_path,
        buildAtmosphericSsrText(satellites, true_receiver_position, first_time, 0.0, 15.0) +
            buildAtmosphericSsrText(satellites, true_receiver_position, second_time, 0.0, 15.0));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.use_ssr_corrections = true;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.use_ionosphere_free = false;
    ppp_config.estimate_ionosphere = true;
    ppp_config.enable_per_frequency_phase_bias_states = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.convergence_min_epochs = 1;
    ppp_config.enable_outlier_detection = false;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    const ObservationData first_epoch = makeSyntheticEpochWithAtmosphericBiases(
        first_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites,
        0.0,
        15.0);
    ASSERT_TRUE(processor.processEpoch(first_epoch, nav_data).isValid());

    const ObservationData second_epoch = makeSyntheticEpochWithAtmosphericBiases(
        second_time,
        true_receiver_position,
        approximate_receiver_position,
        satellites,
        0.0,
        15.0);
    ASSERT_TRUE(processor.processEpoch(second_epoch, nav_data).isValid());

    const auto diagnostics = processor.getLastPPPFilterIterationDiagnostics();
    ASSERT_FALSE(diagnostics.empty());
    EXPECT_EQ(diagnostics.front().code_rows, static_cast<int>(satellites.size() * 2U));
    EXPECT_EQ(diagnostics.front().phase_rows, static_cast<int>(satellites.size() * 2U));
    EXPECT_EQ(
        diagnostics.front().ionosphere_constraint_rows,
        static_cast<int>(satellites.size()));

    int l1_phase_diagnostics = 0;
    int l2_phase_diagnostics = 0;
    for (const auto& diagnostic : processor.getLastSSRApplicationDiagnostics()) {
        if (!diagnostic.valid_after_corrections) {
            continue;
        }
        if (diagnostic.frequency_index == 0 && diagnostic.has_carrier_phase) {
            ++l1_phase_diagnostics;
        } else if (diagnostic.frequency_index == 1 && diagnostic.has_carrier_phase) {
            ++l2_phase_diagnostics;
            EXPECT_GT(diagnostic.ionosphere_coefficient, 1.0);
        }
    }
    EXPECT_EQ(l1_phase_diagnostics, static_cast<int>(satellites.size()));
    EXPECT_EQ(l2_phase_diagnostics, static_cast<int>(satellites.size()));

    int l1_phase_residuals = 0;
    int l2_phase_residuals = 0;
    int ionosphere_constraint_residuals = 0;
    for (const auto& residual : processor.getLastPPPResidualDiagnostics()) {
        if (residual.ionosphere_constraint) {
            ++ionosphere_constraint_residuals;
            EXPECT_EQ(residual.frequency_index, 0);
            continue;
        }
        if (residual.carrier_phase && !residual.phase_candidate && residual.iono_state_m != 0.0) {
            EXPECT_TRUE(residual.phase_accepted);
            EXPECT_TRUE(residual.phase_ready);
            EXPECT_GE(residual.ambiguity_state_index, 0);
            EXPECT_GE(residual.ambiguity_lock_count, residual.required_lock_count);
            if (residual.frequency_index == 0) {
                ++l1_phase_residuals;
                EXPECT_EQ(residual.primary_signal, SignalType::GPS_L1CA);
            } else if (residual.frequency_index == 1) {
                ++l2_phase_residuals;
                EXPECT_EQ(residual.primary_signal, SignalType::GPS_L2C);
                EXPECT_EQ(residual.secondary_signal, SignalType::SIGNAL_TYPE_COUNT);
                EXPECT_GT(residual.ionosphere_coefficient, 1.0);
            }
        }
    }
    EXPECT_GT(l1_phase_residuals, 0);
    EXPECT_GT(l2_phase_residuals, 0);
    EXPECT_GT(ionosphere_constraint_residuals, 0);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);
}

TEST(PPPTest, ProcessorCreatesIonosphereStateForLateSatellite) {
    const auto sp3_path = tempFilePath("libgnss_ppp_late_iono_state_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_late_iono_state_test.clk");
    const auto ssr_path = tempFilePath("libgnss_ppp_late_iono_state_test.csv");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(12.0, -8.0, 4.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);
    ASSERT_GE(satellites.size(), 6U);
    const SatelliteId late_satellite = satellites.back().id;
    const std::vector<SyntheticSatellite> initial_satellites(
        satellites.begin(), satellites.end() - 1);

    const GNSSTime first_time = makeTime(2026, 3, 26, 4, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));
    writeTextFile(
        ssr_path,
        buildAtmosphericSsrText(satellites, true_receiver_position, first_time, 0.0, 15.0));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.use_ssr_corrections = true;
    ppp_config.ssr_file_path = ssr_path.string();
    ppp_config.use_ionosphere_free = false;
    ppp_config.estimate_ionosphere = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.convergence_min_epochs = 1;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    const ObservationData first_epoch = makeSyntheticEpochWithAtmosphericBiases(
        first_time,
        true_receiver_position,
        approximate_receiver_position,
        initial_satellites,
        0.0,
        15.0);
    ASSERT_TRUE(processor.processEpoch(first_epoch, nav_data).isValid());

    const ObservationData second_epoch = makeSyntheticEpochWithAtmosphericBiases(
        first_time + 30.0,
        true_receiver_position,
        approximate_receiver_position,
        satellites,
        0.0,
        15.0);
    ASSERT_TRUE(processor.processEpoch(second_epoch, nav_data).isValid());

    bool late_satellite_constrained = false;
    for (const auto& diagnostic : processor.getLastSSRApplicationDiagnostics()) {
        if (diagnostic.satellite == late_satellite) {
            late_satellite_constrained =
                late_satellite_constrained || diagnostic.ionosphere_estimation_constraint;
            EXPECT_TRUE(diagnostic.valid_after_corrections);
            EXPECT_GT(diagnostic.stec_tecu, 0.0);
        }
    }
    EXPECT_TRUE(late_satellite_constrained);

    bool late_satellite_has_iono_state = false;
    for (const auto& diagnostic : processor.getLastPPPResidualDiagnostics()) {
        if (diagnostic.satellite == late_satellite && !diagnostic.carrier_phase) {
            late_satellite_has_iono_state = std::abs(diagnostic.iono_state_m) > 0.1;
        }
    }
    EXPECT_TRUE(late_satellite_has_iono_state);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
    std::filesystem::remove(ssr_path);
}


TEST(PPPTest, ProcessorUsesModeledTroposphereWhenZenithStateIsDisabled) {
    const auto sp3_path = tempFilePath("libgnss_ppp_modeled_trop_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_modeled_trop_test.clk");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);

    const Vector3d true_receiver_position =
        geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position =
        true_receiver_position + Vector3d(10.0, -7.0, 5.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 4, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.use_ionosphere_free = true;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = false;
    ppp_config.convergence_min_epochs = 1;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution solution;
    for (int i = 0; i < 8; ++i) {
        const ObservationData epoch = makeSyntheticEpochWithAtmosphericBiases(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites,
            2.3,
            0.0);
        solution = processor.processEpoch(epoch, nav_data);
    }

    ASSERT_TRUE(solution.isValid());
    EXPECT_LT(solution.residual_rms, 1.0);
    EXPECT_LT(
        (solution.position_ecef - true_receiver_position).norm(),
        (approximate_receiver_position - true_receiver_position).norm());

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
}

TEST(PPPTest, ProcessorFixesSyntheticAmbiguitiesWithPreciseProducts) {
    const auto sp3_path = tempFilePath("libgnss_ppp_ar_test.sp3");
    const auto clk_path = tempFilePath("libgnss_ppp_ar_test.clk");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);

    const Vector3d true_receiver_position = geodetic2ecef(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0, 45.0);
    const Vector3d approximate_receiver_position = true_receiver_position + Vector3d(8.0, -5.0, 3.0);
    const auto satellites = makeSyntheticSatellites(true_receiver_position);

    const GNSSTime first_time = makeTime(2026, 3, 26, 2, 0, 0.0);
    const GNSSTime last_precise_time = first_time + 600.0;
    writeTextFile(sp3_path, buildSp3Text(satellites, first_time, last_precise_time));
    writeTextFile(clk_path, buildClockText(satellites, first_time, last_precise_time));

    PPPProcessor::PPPConfig ppp_config;
    ppp_config.use_precise_orbits = true;
    ppp_config.use_precise_clocks = true;
    ppp_config.orbit_file_path = sp3_path.string();
    ppp_config.clock_file_path = clk_path.string();
    ppp_config.convergence_min_epochs = 4;
    ppp_config.convergence_threshold_horizontal = 0.2;
    ppp_config.estimate_troposphere = false;
    ppp_config.enable_ambiguity_resolution = true;
    ppp_config.ar_ratio_threshold = 2.0;
    ppp_config.kinematic_mode = false;

    PPPProcessor processor(ppp_config);
    ProcessorConfig processor_config;
    processor_config.mode = PositioningMode::PPP;
    processor_config.min_satellites = 4;
    processor_config.use_precise_orbits = true;
    processor_config.use_precise_clocks = true;
    processor_config.orbit_file_path = sp3_path.string();
    processor_config.clock_file_path = clk_path.string();
    ASSERT_TRUE(processor.initialize(processor_config));

    NavigationData nav_data;
    PositionSolution last_solution;
    bool saw_fixed_solution = false;
    int best_fixed_ambiguities = 0;
    double best_ratio = 0.0;
    for (int i = 0; i < 8; ++i) {
        const ObservationData epoch = makeSyntheticEpoch(
            first_time + 30.0 * static_cast<double>(i),
            true_receiver_position,
            approximate_receiver_position,
            satellites);
        last_solution = processor.processEpoch(epoch, nav_data);
        ASSERT_TRUE(last_solution.isValid());
        if (last_solution.status == SolutionStatus::PPP_FIXED) {
            saw_fixed_solution = true;
            best_fixed_ambiguities =
                std::max(best_fixed_ambiguities, last_solution.num_fixed_ambiguities);
            best_ratio = std::max(best_ratio, last_solution.ratio);
        }
    }

    EXPECT_LT((last_solution.position_ecef - true_receiver_position).norm(), 1.0);
    EXPECT_GE(last_solution.ratio, 0.0);
    EXPECT_GE(last_solution.num_fixed_ambiguities, 0);
    if (saw_fixed_solution) {
        EXPECT_GE(best_fixed_ambiguities, 1);
        EXPECT_GT(best_ratio, 2.0);
    }

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
}
