#include <gtest/gtest.h>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

using namespace libgnss;

namespace {

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

std::vector<uint8_t> buildRtcm1005(double x_m, double y_m, double z_m) {
    std::vector<uint8_t> payload(19, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1005); bit += 12;
    setUnsignedBits(payload, bit, 12, 42); bit += 12;
    setUnsignedBits(payload, bit, 6, 0); bit += 6;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setSignedBits(payload, bit, 38, static_cast<int64_t>(std::llround(x_m * 10000.0))); bit += 38;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setSignedBits(payload, bit, 38, static_cast<int64_t>(std::llround(y_m * 10000.0))); bit += 38;
    setUnsignedBits(payload, bit, 2, 0); bit += 2;
    setSignedBits(payload, bit, 38, static_cast<int64_t>(std::llround(z_m * 10000.0)));

    std::vector<uint8_t> frame;
    frame.push_back(0xD3);
    frame.push_back(0x00);
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());

    const uint32_t crc = crc24q(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
    return frame;
}

std::vector<uint8_t> buildRtcmFrame(const io::RTCMMessage& message) {
    std::vector<uint8_t> frame;
    frame.reserve(3 + message.data.size() + 3);
    frame.push_back(0xD3);
    frame.push_back(static_cast<uint8_t>((message.data.size() >> 8) & 0x03U));
    frame.push_back(static_cast<uint8_t>(message.data.size() & 0xFFU));
    frame.insert(frame.end(), message.data.begin(), message.data.end());

    const uint32_t crc = crc24q(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
    return frame;
}

std::vector<uint8_t> buildGpsSsrCombined1060Frame() {
    constexpr int total_bits = 68 + 205;
    std::vector<uint8_t> payload((total_bits + 7) / 8, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1060); bit += 12;
    setUnsignedBits(payload, bit, 20, 345600); bit += 20;
    setUnsignedBits(payload, bit, 4, 2); bit += 4;   // update interval = 5 s
    setUnsignedBits(payload, bit, 1, 0); bit += 1;   // sync
    setUnsignedBits(payload, bit, 1, 1); bit += 1;   // ref datum
    setUnsignedBits(payload, bit, 4, 7); bit += 4;   // iod
    setUnsignedBits(payload, bit, 16, 21); bit += 16; // provider
    setUnsignedBits(payload, bit, 4, 3); bit += 4;   // solution id
    setUnsignedBits(payload, bit, 6, 1); bit += 6;   // nsat

    setUnsignedBits(payload, bit, 6, 7); bit += 6;   // G07
    setUnsignedBits(payload, bit, 8, 12); bit += 8;  // iode
    setSignedBits(payload, bit, 22, 1234); bit += 22;   // 0.1234 m
    setSignedBits(payload, bit, 20, -200); bit += 20;   // -0.0800 m
    setSignedBits(payload, bit, 20, 300); bit += 20;    // 0.1200 m
    setSignedBits(payload, bit, 21, 123); bit += 21;    // 0.000123 m/s
    setSignedBits(payload, bit, 19, -114); bit += 19;   // -0.000456 m/s
    setSignedBits(payload, bit, 19, 57); bit += 19;     // 0.000228 m/s
    setSignedBits(payload, bit, 22, -2500); bit += 22;  // -0.2500 m
    setSignedBits(payload, bit, 21, 1234); bit += 21;   // 0.001234 m/s
    setSignedBits(payload, bit, 27, -200); bit += 27;   // -0.000004 m/s^2

    io::RTCMMessage message(io::RTCMMessageType::RTCM_1060, payload);
    message.valid = true;
    return buildRtcmFrame(message);
}

std::vector<uint8_t> buildGlonassSsrCombined1066Frame() {
    constexpr int total_bits = 65 + 204;
    std::vector<uint8_t> payload((total_bits + 7) / 8, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1066); bit += 12;
    setUnsignedBits(payload, bit, 17, 43210); bit += 17; // GLONASS TOD
    setUnsignedBits(payload, bit, 4, 5); bit += 4;       // update interval = 30 s
    setUnsignedBits(payload, bit, 1, 0); bit += 1;       // sync
    setUnsignedBits(payload, bit, 1, 0); bit += 1;       // ref datum
    setUnsignedBits(payload, bit, 4, 9); bit += 4;       // iod
    setUnsignedBits(payload, bit, 16, 8); bit += 16;     // provider
    setUnsignedBits(payload, bit, 4, 1); bit += 4;       // solution id
    setUnsignedBits(payload, bit, 6, 1); bit += 6;       // nsat

    setUnsignedBits(payload, bit, 5, 8); bit += 5;       // R08
    setUnsignedBits(payload, bit, 8, 44); bit += 8;      // iode
    setSignedBits(payload, bit, 22, -800); bit += 22;    // -0.0800 m
    setSignedBits(payload, bit, 20, 175); bit += 20;     // 0.0700 m
    setSignedBits(payload, bit, 20, -250); bit += 20;    // -0.1000 m
    setSignedBits(payload, bit, 21, -90); bit += 21;     // -0.000090 m/s
    setSignedBits(payload, bit, 19, 80); bit += 19;      // 0.000320 m/s
    setSignedBits(payload, bit, 19, -40); bit += 19;     // -0.000160 m/s
    setSignedBits(payload, bit, 22, 1800); bit += 22;    // 0.1800 m
    setSignedBits(payload, bit, 21, -220); bit += 21;    // -0.000220 m/s
    setSignedBits(payload, bit, 27, 150); bit += 27;     // 0.000003 m/s^2

    io::RTCMMessage message(io::RTCMMessageType::RTCM_1066, payload);
    message.valid = true;
    return buildRtcmFrame(message);
}

std::vector<uint8_t> buildGpsSsrCodeBias1059Frame() {
    constexpr int total_bits = 67 + 6 + 5 + 19;
    std::vector<uint8_t> payload((total_bits + 7) / 8, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1059); bit += 12;
    setUnsignedBits(payload, bit, 20, 345600); bit += 20;
    setUnsignedBits(payload, bit, 4, 2); bit += 4;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 4, 7); bit += 4;
    setUnsignedBits(payload, bit, 16, 21); bit += 16;
    setUnsignedBits(payload, bit, 4, 3); bit += 4;
    setUnsignedBits(payload, bit, 6, 1); bit += 6;

    setUnsignedBits(payload, bit, 6, 7); bit += 6;
    setUnsignedBits(payload, bit, 5, 1); bit += 5;
    setUnsignedBits(payload, bit, 5, 3); bit += 5;
    setSignedBits(payload, bit, 14, -12); bit += 14; // -0.12 m

    io::RTCMMessage message(io::RTCMMessageType::RTCM_1059, payload);
    message.valid = true;
    return buildRtcmFrame(message);
}

std::vector<uint8_t> buildGpsSsrUra1061Frame() {
    constexpr int total_bits = 67 + 6 + 6;
    std::vector<uint8_t> payload((total_bits + 7) / 8, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1061); bit += 12;
    setUnsignedBits(payload, bit, 20, 345600); bit += 20;
    setUnsignedBits(payload, bit, 4, 2); bit += 4;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 4, 7); bit += 4;
    setUnsignedBits(payload, bit, 16, 21); bit += 16;
    setUnsignedBits(payload, bit, 4, 3); bit += 4;
    setUnsignedBits(payload, bit, 6, 1); bit += 6;

    setUnsignedBits(payload, bit, 6, 7); bit += 6;
    setUnsignedBits(payload, bit, 6, 9); bit += 6;

    io::RTCMMessage message(io::RTCMMessageType::RTCM_1061, payload);
    message.valid = true;
    return buildRtcmFrame(message);
}

std::vector<uint8_t> buildGpsSsrHighRateClock1062Frame() {
    constexpr int total_bits = 67 + 6 + 22;
    std::vector<uint8_t> payload((total_bits + 7) / 8, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1062); bit += 12;
    setUnsignedBits(payload, bit, 20, 345600); bit += 20;
    setUnsignedBits(payload, bit, 4, 2); bit += 4;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 4, 7); bit += 4;
    setUnsignedBits(payload, bit, 16, 21); bit += 16;
    setUnsignedBits(payload, bit, 4, 3); bit += 4;
    setUnsignedBits(payload, bit, 6, 1); bit += 6;

    setUnsignedBits(payload, bit, 6, 7); bit += 6;
    setSignedBits(payload, bit, 22, 250); bit += 22; // 0.0250 m

    io::RTCMMessage message(io::RTCMMessageType::RTCM_1062, payload);
    message.valid = true;
    return buildRtcmFrame(message);
}

double wavelengthForSignal(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GPS_L1P:
            return constants::GPS_L1_WAVELENGTH;
        case SignalType::GPS_L2C:
        case SignalType::GPS_L2P:
            return constants::GPS_L2_WAVELENGTH;
        case SignalType::GAL_E1:
            return constants::GAL_E1_WAVELENGTH;
        case SignalType::GAL_E5A:
            return constants::GAL_E5A_WAVELENGTH;
        case SignalType::GAL_E5B:
            return constants::GAL_E5B_WAVELENGTH;
        case SignalType::GAL_E6:
            return constants::GAL_E6_WAVELENGTH;
        case SignalType::BDS_B1I:
            return constants::BDS_B1I_WAVELENGTH;
        case SignalType::BDS_B2I:
            return constants::BDS_B2I_WAVELENGTH;
        case SignalType::BDS_B3I:
            return constants::BDS_B3I_WAVELENGTH;
        default:
            return 0.0;
    }
}

double glonassWavelengthForSignal(SignalType signal, int frequency_channel) {
    switch (signal) {
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
            return constants::SPEED_OF_LIGHT /
                   (constants::GLO_L1_BASE_FREQ +
                    static_cast<double>(frequency_channel) * constants::GLO_L1_STEP_FREQ);
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
            return constants::SPEED_OF_LIGHT /
                   (constants::GLO_L2_BASE_FREQ +
                    static_cast<double>(frequency_channel) * constants::GLO_L2_STEP_FREQ);
        default:
            return 0.0;
    }
}

Observation makeObservation(GNSSSystem system,
                            uint8_t prn,
                            SignalType signal,
                            double pseudorange,
                            double carrier_offset_m,
                            double snr_dbhz,
                            bool loss_of_lock = false,
                            std::optional<double> doppler_hz = std::nullopt) {
    Observation obs(SatelliteId(system, prn), signal);
    obs.has_pseudorange = true;
    obs.has_carrier_phase = true;
    obs.pseudorange = pseudorange;
    obs.carrier_phase = (pseudorange + carrier_offset_m) / wavelengthForSignal(signal);
    obs.snr = snr_dbhz;
    obs.signal_strength = static_cast<int>(std::lround(snr_dbhz / 6.0));
    obs.loss_of_lock = loss_of_lock;
    obs.lli = loss_of_lock ? 1U : 0U;
    if (doppler_hz.has_value()) {
        obs.has_doppler = true;
        obs.doppler = *doppler_hz;
    }
    obs.valid = true;
    return obs;
}

Observation makeGpsObservation(uint8_t prn,
                               SignalType signal,
                               double pseudorange,
                               double carrier_offset_m,
                               double snr_dbhz,
                               bool loss_of_lock = false,
                               std::optional<double> doppler_hz = std::nullopt) {
    return makeObservation(
        GNSSSystem::GPS,
        prn,
        signal,
        pseudorange,
        carrier_offset_m,
        snr_dbhz,
        loss_of_lock,
        doppler_hz);
}

Observation makeGlonassObservation(uint8_t prn,
                                   SignalType signal,
                                   double pseudorange,
                                   double carrier_offset_m,
                                   double snr_dbhz,
                                   int frequency_channel,
                                   bool loss_of_lock = false,
                                   std::optional<double> doppler_hz = std::nullopt) {
    Observation obs(SatelliteId(GNSSSystem::GLONASS, prn), signal);
    obs.has_pseudorange = true;
    obs.has_carrier_phase = true;
    obs.pseudorange = pseudorange;
    obs.carrier_phase =
        (pseudorange + carrier_offset_m) / glonassWavelengthForSignal(signal, frequency_channel);
    obs.snr = snr_dbhz;
    obs.signal_strength = static_cast<int>(std::lround(snr_dbhz / 6.0));
    obs.loss_of_lock = loss_of_lock;
    obs.lli = loss_of_lock ? 1U : 0U;
    if (doppler_hz.has_value()) {
        obs.has_doppler = true;
        obs.doppler = *doppler_hz;
    }
    obs.valid = true;
    obs.has_glonass_frequency_channel = true;
    obs.glonass_frequency_channel = frequency_channel;
    return obs;
}

std::optional<Observation> findObservation(const ObservationData& obs_data,
                                           GNSSSystem system,
                                           uint8_t prn,
                                           SignalType signal) {
    for (const auto& obs : obs_data.observations) {
        if (obs.satellite.system == system &&
            obs.satellite.prn == prn &&
            obs.signal == signal) {
            return obs;
        }
    }
    return std::nullopt;
}

std::optional<Observation> findObservation(const ObservationData& obs_data,
                                           uint8_t prn,
                                           SignalType signal) {
    return findObservation(obs_data, GNSSSystem::GPS, prn, signal);
}

int currentGpsWeek() {
    return GNSSTime::fromSystemTime(std::chrono::system_clock::now()).week;
}

GNSSTime currentGpsTime() {
    return GNSSTime::fromSystemTime(std::chrono::system_clock::now());
}

int currentLeapSeconds() {
    return 18;
}

Ephemeris makeGpsEphemeris() {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GPS, 12);
    eph.week = static_cast<uint16_t>(currentGpsWeek());
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

Ephemeris makeGlonassEphemeris() {
    const GNSSTime now = currentGpsTime();
    const double leap_seconds = static_cast<double>(currentLeapSeconds());
    const double current_utc_tow = now.tow - leap_seconds;
    const double toe_utc_tow = std::floor(current_utc_tow / 900.0) * 900.0;

    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GLONASS, 7);
    eph.week = static_cast<uint16_t>(now.week);
    // Keep toe/tof close to "now" so RTCM 1020 day alignment stays stable across UTC midnight.
    eph.toe = GNSSTime(now.week, toe_utc_tow + leap_seconds);
    eph.tof = eph.toe + 1800.0;
    eph.toc = eph.toe;
    eph.toes = eph.toe.tow;
    eph.glonass_position = Vector3d(19123456.5, -12345678.0, 21765432.5);
    eph.glonass_velocity = Vector3d(-1325.125, 2450.75, 985.5);
    eph.glonass_acceleration = Vector3d(4.7e-6, -2.8e-6, 3.2e-6);
    eph.glonass_taun = -8.1234e-5;
    eph.glonass_gamn = 4.5e-10;
    eph.glonass_frequency_channel = -4;
    eph.glonass_age = 7;
    eph.health = 0;
    eph.sv_health = 0.0;
    eph.iode = 48;
    eph.valid = true;
    return eph;
}

#ifndef _WIN32
class LocalNtripServer {
public:
    explicit LocalNtripServer(std::vector<uint8_t> payload)
        : payload_(std::move(payload)) {
        server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            return;
        }

        int reuse = 1;
        ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            ::close(server_fd_);
            server_fd_ = -1;
            return;
        }
        if (::listen(server_fd_, 1) != 0) {
            ::close(server_fd_);
            server_fd_ = -1;
            return;
        }

        socklen_t address_size = sizeof(address);
        if (::getsockname(server_fd_, reinterpret_cast<sockaddr*>(&address), &address_size) != 0) {
            ::close(server_fd_);
            server_fd_ = -1;
            return;
        }
        port_ = ntohs(address.sin_port);
        worker_ = std::thread([this]() { serveOneClient(); });
    }

    ~LocalNtripServer() {
        if (server_fd_ >= 0) {
            ::shutdown(server_fd_, SHUT_RDWR);
            ::close(server_fd_);
            server_fd_ = -1;
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    bool isReady() const { return server_fd_ >= 0 && port_ != 0; }
    uint16_t port() const { return port_; }

private:
    void serveOneClient() {
        sockaddr_in client_address{};
        socklen_t client_size = sizeof(client_address);
        const int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client_address), &client_size);
        if (client_fd < 0) {
            return;
        }

        char request[1024];
        (void)::recv(client_fd, request, sizeof(request), 0);

        static constexpr char kResponseHeader[] = "ICY 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n\r\n";
        (void)::send(client_fd, kResponseHeader, sizeof(kResponseHeader) - 1, 0);
        (void)::send(client_fd,
                     reinterpret_cast<const char*>(payload_.data()),
                     static_cast<int>(payload_.size()),
                     0);
        ::shutdown(client_fd, SHUT_RDWR);
        ::close(client_fd);
    }

    std::vector<uint8_t> payload_;
    int server_fd_ = -1;
    uint16_t port_ = 0;
    std::thread worker_;
};

class LocalTcpServer {
public:
    explicit LocalTcpServer(std::vector<uint8_t> payload)
        : payload_(std::move(payload)) {
        server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            return;
        }

        int reuse = 1;
        ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            ::close(server_fd_);
            server_fd_ = -1;
            return;
        }
        if (::listen(server_fd_, 1) != 0) {
            ::close(server_fd_);
            server_fd_ = -1;
            return;
        }

        socklen_t address_size = sizeof(address);
        if (::getsockname(server_fd_, reinterpret_cast<sockaddr*>(&address), &address_size) != 0) {
            ::close(server_fd_);
            server_fd_ = -1;
            return;
        }
        port_ = ntohs(address.sin_port);
        worker_ = std::thread([this]() { serveOneClient(); });
    }

    ~LocalTcpServer() {
        if (server_fd_ >= 0) {
            ::shutdown(server_fd_, SHUT_RDWR);
            ::close(server_fd_);
            server_fd_ = -1;
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    bool isReady() const { return server_fd_ >= 0 && port_ != 0; }
    uint16_t port() const { return port_; }

private:
    void serveOneClient() {
        sockaddr_in client_address{};
        socklen_t client_size = sizeof(client_address);
        const int client_fd =
            ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client_address), &client_size);
        if (client_fd < 0) {
            return;
        }

        (void)::send(client_fd,
                     reinterpret_cast<const char*>(payload_.data()),
                     static_cast<int>(payload_.size()),
                     0);
        ::shutdown(client_fd, SHUT_RDWR);
        ::close(client_fd);
    }

    std::vector<uint8_t> payload_;
    int server_fd_ = -1;
    uint16_t port_ = 0;
    std::thread worker_;
};

struct PseudoTerminal {
    int master_fd = -1;
    std::string slave_path;
};

PseudoTerminal openPseudoTerminal() {
    PseudoTerminal pty;
    pty.master_fd = posix_openpt(O_RDWR | O_NOCTTY);
    EXPECT_GE(pty.master_fd, 0);
    EXPECT_EQ(grantpt(pty.master_fd), 0);
    EXPECT_EQ(unlockpt(pty.master_fd), 0);
    char* name = ptsname(pty.master_fd);
    EXPECT_NE(name, nullptr);
    if (name != nullptr) {
        pty.slave_path = name;
    }
    return pty;
}
#endif

}  // namespace

class RTCMProcessorTest : public ::testing::Test {
protected:
    void TearDown() override {
        processor.clear();
    }

    io::RTCMProcessor processor;
};

TEST_F(RTCMProcessorTest, RejectsMessageWithInvalidCRC) {
    const uint8_t invalid_rtcm_1005[] = {
        0xD3, 0x00, 0x0F, 0x3E, 0xD4, 0x00, 0xFB, 0x9D, 0xC7, 0x9E,
        0x82, 0x0F, 0x06, 0x84, 0xF8, 0xE0, 0xC3, 0x72, 0x4F, 0x0F,
        0x4D
    };

    const auto decoded_messages =
        processor.decode(invalid_rtcm_1005, sizeof(invalid_rtcm_1005));

    EXPECT_TRUE(decoded_messages.empty());
    EXPECT_FALSE(processor.hasReferencePosition());
}

TEST_F(RTCMProcessorTest, DecodesReferenceStationPositionFrom1005) {
    const Vector3d expected(3875000.1234, 332100.5, 5026000.9876);
    const auto frame = buildRtcm1005(expected.x(), expected.y(), expected.z());

    const auto decoded_messages = processor.decode(frame.data(), frame.size());

    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1005);
    ASSERT_TRUE(processor.hasReferencePosition());
    const Vector3d decoded = processor.getReferencePosition();
    EXPECT_NEAR(decoded.x(), expected.x(), 1e-4);
    EXPECT_NEAR(decoded.y(), expected.y(), 1e-4);
    EXPECT_NEAR(decoded.z(), expected.z(), 1e-4);
}

TEST_F(RTCMProcessorTest, ClearResetsReferencePositionState) {
    const auto frame = buildRtcm1005(1000.0, 2000.0, 3000.0);
    processor.decode(frame.data(), frame.size());
    ASSERT_TRUE(processor.hasReferencePosition());

    processor.clear();

    EXPECT_FALSE(processor.hasReferencePosition());
    EXPECT_DOUBLE_EQ(processor.getReferencePosition().x(), 0.0);
    EXPECT_DOUBLE_EQ(processor.getReferencePosition().y(), 0.0);
    EXPECT_DOUBLE_EQ(processor.getReferencePosition().z(), 0.0);
}

TEST_F(RTCMProcessorTest, TracksDecodeStatistics) {
    auto valid = buildRtcm1005(100.0, 200.0, 300.0);
    auto invalid = valid;
    invalid.back() ^= 0x55U;

    std::vector<uint8_t> stream = {0x00, 0x01, 0x02};
    stream.insert(stream.end(), invalid.begin(), invalid.end());
    stream.insert(stream.end(), valid.begin(), valid.end());

    const auto decoded_messages = processor.decode(stream.data(), stream.size());
    const auto stats = processor.getStats();

    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(stats.total_messages, 2U);
    EXPECT_EQ(stats.valid_messages, 1U);
    EXPECT_EQ(stats.crc_errors, 1U);
    EXPECT_EQ(stats.message_counts.at(io::RTCMMessageType::RTCM_1005), 1U);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGps1004PayloadRoundTrip) {
    processor.setReferencePosition(Vector3d(1111.0, 2222.0, 3333.0));

    ObservationData input(GNSSTime(2300, 345678.125));
    input.addObservation(makeGpsObservation(3, SignalType::GPS_L1CA, 21456789.12, 2.50, 46.0));
    input.addObservation(makeGpsObservation(3, SignalType::GPS_L2C, 21456790.54, 1.75, 43.0));
    input.addObservation(makeGpsObservation(11, SignalType::GPS_L1P, 22456780.34, -1.25, 41.5, true));
    input.addObservation(makeGpsObservation(11, SignalType::GPS_L2P, 22456781.10, -0.80, 39.5));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1004);
    ASSERT_FALSE(encoded.data.empty());

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    EXPECT_NEAR(decoded.time.tow, input.time.tow, 1e-3);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.x(), 1111.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.y(), 2222.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.z(), 3333.0);
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn3_l1 = findObservation(decoded, 3, SignalType::GPS_L1CA);
    const auto prn3_l2 = findObservation(decoded, 3, SignalType::GPS_L2C);
    const auto prn11_l1 = findObservation(decoded, 11, SignalType::GPS_L1P);
    const auto prn11_l2 = findObservation(decoded, 11, SignalType::GPS_L2P);
    ASSERT_TRUE(prn3_l1.has_value());
    ASSERT_TRUE(prn3_l2.has_value());
    ASSERT_TRUE(prn11_l1.has_value());
    ASSERT_TRUE(prn11_l2.has_value());

    EXPECT_NEAR(prn3_l1->pseudorange, 21456789.12, 0.02);
    EXPECT_NEAR(prn3_l2->pseudorange, 21456790.54, 0.02);
    EXPECT_NEAR(prn11_l1->pseudorange, 22456780.34, 0.02);
    EXPECT_NEAR(prn11_l2->pseudorange, 22456781.10, 0.02);
    EXPECT_NEAR(prn3_l1->carrier_phase * constants::GPS_L1_WAVELENGTH, 21456791.62, 0.03);
    EXPECT_NEAR(prn3_l2->carrier_phase * constants::GPS_L2_WAVELENGTH, 21456792.29, 0.03);
    EXPECT_NEAR(prn11_l1->carrier_phase * constants::GPS_L1_WAVELENGTH, 22456779.09, 0.03);
    EXPECT_NEAR(prn11_l2->carrier_phase * constants::GPS_L2_WAVELENGTH, 22456780.30, 0.03);
    EXPECT_FALSE(prn3_l1->loss_of_lock);
    EXPECT_TRUE(prn11_l1->loss_of_lock);
}

TEST_F(RTCMProcessorTest, DecodesGps1004FrameAfterStationMessage) {
    ObservationData input(GNSSTime(2301, 12345.250));
    input.addObservation(makeGpsObservation(7, SignalType::GPS_L1CA, 20200000.20, 1.25, 45.0));
    input.addObservation(makeGpsObservation(7, SignalType::GPS_L2C, 20200001.00, 1.60, 42.0));

    const Vector3d expected_station(3875000.1234, 332100.5, 5026000.9876);
    const auto station_frame = buildRtcm1005(expected_station.x(), expected_station.y(), expected_station.z());
    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(encoded.valid);
    const auto obs_frame = buildRtcmFrame(encoded);

    std::vector<uint8_t> stream;
    stream.reserve(station_frame.size() + obs_frame.size());
    stream.insert(stream.end(), station_frame.begin(), station_frame.end());
    stream.insert(stream.end(), obs_frame.begin(), obs_frame.end());

    const auto decoded_messages = processor.decode(stream.data(), stream.size());
    ASSERT_EQ(decoded_messages.size(), 2U);
    EXPECT_EQ(decoded_messages[0].type, io::RTCMMessageType::RTCM_1005);
    EXPECT_EQ(decoded_messages[1].type, io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(processor.hasReferencePosition());

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(decoded_messages[1], decoded));
    EXPECT_NEAR(decoded.time.tow, input.time.tow, 1e-3);
    EXPECT_NEAR(decoded.receiver_position.x(), expected_station.x(), 1e-4);
    EXPECT_NEAR(decoded.receiver_position.y(), expected_station.y(), 1e-4);
    EXPECT_NEAR(decoded.receiver_position.z(), expected_station.z(), 1e-4);
    ASSERT_EQ(decoded.observations.size(), 2U);

    const auto l1 = findObservation(decoded, 7, SignalType::GPS_L1CA);
    const auto l2 = findObservation(decoded, 7, SignalType::GPS_L2C);
    ASSERT_TRUE(l1.has_value());
    ASSERT_TRUE(l2.has_value());
    EXPECT_NEAR(l1->pseudorange, 20200000.20, 0.02);
    EXPECT_NEAR(l2->pseudorange, 20200001.00, 0.02);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGps1003WithoutCn0Fields) {
    ObservationData input(GNSSTime(2302, 54321.750));
    input.addObservation(makeGpsObservation(19, SignalType::GPS_L1CA, 23200000.40, 0.80, 47.0));
    input.addObservation(makeGpsObservation(19, SignalType::GPS_L2C, 23200001.20, 1.15, 44.0));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1003);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1003);
    ASSERT_FALSE(encoded.data.empty());

    const auto frame = buildRtcmFrame(encoded);
    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1003);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 2U);

    const auto l1 = findObservation(decoded, 19, SignalType::GPS_L1CA);
    const auto l2 = findObservation(decoded, 19, SignalType::GPS_L2C);
    ASSERT_TRUE(l1.has_value());
    ASSERT_TRUE(l2.has_value());
    EXPECT_NEAR(l1->pseudorange, 23200000.40, 0.02);
    EXPECT_NEAR(l2->pseudorange, 23200001.20, 0.02);
    EXPECT_DOUBLE_EQ(l1->snr, 0.0);
    EXPECT_DOUBLE_EQ(l2->snr, 0.0);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGps1074PayloadRoundTrip) {
    processor.setReferencePosition(Vector3d(4321.0, 5432.0, 6543.0));

    ObservationData input(GNSSTime(2303, 123456.250));
    input.addObservation(makeGpsObservation(5, SignalType::GPS_L1CA, 21456789.12, 2.50, 46.0));
    input.addObservation(makeGpsObservation(5, SignalType::GPS_L2C, 21456790.54, 1.75, 43.0));
    Observation prn14_l1 = makeGpsObservation(14, SignalType::GPS_L1P, 22456780.34, -1.25, 41.5);
    prn14_l1.lli |= 0x02U;
    input.addObservation(prn14_l1);
    input.addObservation(makeGpsObservation(14, SignalType::GPS_L2P, 22456781.10, -0.80, 39.5, true));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1074);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1074);
    ASSERT_FALSE(encoded.data.empty());

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    EXPECT_NEAR(decoded.time.tow, input.time.tow, 1e-3);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.x(), 4321.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.y(), 5432.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.z(), 6543.0);
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn5_l1 = findObservation(decoded, 5, SignalType::GPS_L1CA);
    const auto prn5_l2 = findObservation(decoded, 5, SignalType::GPS_L2C);
    const auto prn14_l1_decoded = findObservation(decoded, 14, SignalType::GPS_L1P);
    const auto prn14_l2 = findObservation(decoded, 14, SignalType::GPS_L2P);
    ASSERT_TRUE(prn5_l1.has_value());
    ASSERT_TRUE(prn5_l2.has_value());
    ASSERT_TRUE(prn14_l1_decoded.has_value());
    ASSERT_TRUE(prn14_l2.has_value());

    EXPECT_NEAR(prn5_l1->pseudorange, 21456789.12, 0.03);
    EXPECT_NEAR(prn5_l2->pseudorange, 21456790.54, 0.03);
    EXPECT_NEAR(prn14_l1_decoded->pseudorange, 22456780.34, 0.03);
    EXPECT_NEAR(prn14_l2->pseudorange, 22456781.10, 0.03);
    EXPECT_NEAR(prn5_l1->carrier_phase * constants::GPS_L1_WAVELENGTH, 21456791.62, 0.03);
    EXPECT_NEAR(prn5_l2->carrier_phase * constants::GPS_L2_WAVELENGTH, 21456792.29, 0.03);
    EXPECT_NEAR(prn14_l1_decoded->carrier_phase * constants::GPS_L1_WAVELENGTH, 22456779.09, 0.03);
    EXPECT_NEAR(prn14_l2->carrier_phase * constants::GPS_L2_WAVELENGTH, 22456780.30, 0.03);
    EXPECT_FALSE(prn5_l1->loss_of_lock);
    EXPECT_TRUE(prn14_l2->loss_of_lock);
    EXPECT_EQ(prn14_l1_decoded->lli & 0x02U, 0x02U);
}

TEST_F(RTCMProcessorTest, DecodesGps1074FrameAfterStationMessage) {
    ObservationData input(GNSSTime(2304, 65432.750));
    input.addObservation(makeGpsObservation(7, SignalType::GPS_L1CA, 20200000.20, 1.25, 45.0));
    input.addObservation(makeGpsObservation(7, SignalType::GPS_L2C, 20200001.00, 1.60, 42.0));

    const Vector3d expected_station(3875000.1234, 332100.5, 5026000.9876);
    const auto station_frame = buildRtcm1005(expected_station.x(), expected_station.y(), expected_station.z());
    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1074);
    ASSERT_TRUE(encoded.valid);
    const auto obs_frame = buildRtcmFrame(encoded);

    std::vector<uint8_t> stream;
    stream.reserve(station_frame.size() + obs_frame.size());
    stream.insert(stream.end(), station_frame.begin(), station_frame.end());
    stream.insert(stream.end(), obs_frame.begin(), obs_frame.end());

    const auto decoded_messages = processor.decode(stream.data(), stream.size());
    ASSERT_EQ(decoded_messages.size(), 2U);
    EXPECT_EQ(decoded_messages[0].type, io::RTCMMessageType::RTCM_1005);
    EXPECT_EQ(decoded_messages[1].type, io::RTCMMessageType::RTCM_1074);
    ASSERT_TRUE(processor.hasReferencePosition());

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(decoded_messages[1], decoded));
    EXPECT_NEAR(decoded.time.tow, input.time.tow, 1e-3);
    EXPECT_NEAR(decoded.receiver_position.x(), expected_station.x(), 1e-4);
    EXPECT_NEAR(decoded.receiver_position.y(), expected_station.y(), 1e-4);
    EXPECT_NEAR(decoded.receiver_position.z(), expected_station.z(), 1e-4);
    ASSERT_EQ(decoded.observations.size(), 2U);

    const auto l1 = findObservation(decoded, 7, SignalType::GPS_L1CA);
    const auto l2 = findObservation(decoded, 7, SignalType::GPS_L2C);
    ASSERT_TRUE(l1.has_value());
    ASSERT_TRUE(l2.has_value());
    EXPECT_NEAR(l1->pseudorange, 20200000.20, 0.03);
    EXPECT_NEAR(l2->pseudorange, 20200001.00, 0.03);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGps1075PayloadRoundTripWithDoppler) {
    processor.setReferencePosition(Vector3d(4321.0, 5432.0, 6543.0));
    const double prn5_rate_mps = 234.85;
    const double prn14_rate_mps = -206.40;
    const double prn5_l1_doppler = -prn5_rate_mps / constants::GPS_L1_WAVELENGTH;
    const double prn5_l2_doppler = -prn5_rate_mps / constants::GPS_L2_WAVELENGTH;
    const double prn14_l1_doppler = -prn14_rate_mps / constants::GPS_L1_WAVELENGTH;
    const double prn14_l2_doppler = -prn14_rate_mps / constants::GPS_L2_WAVELENGTH;

    ObservationData input(GNSSTime(2304, 223344.500));
    input.addObservation(makeGpsObservation(
        5, SignalType::GPS_L1CA, 21456789.12, 2.50, 46.0, false, prn5_l1_doppler));
    input.addObservation(makeGpsObservation(
        5, SignalType::GPS_L2C, 21456790.54, 1.75, 43.0, false, prn5_l2_doppler));
    input.addObservation(makeGpsObservation(
        14, SignalType::GPS_L1P, 22456780.34, -1.25, 41.5, false, prn14_l1_doppler));
    input.addObservation(makeGpsObservation(
        14, SignalType::GPS_L2P, 22456781.10, -0.80, 39.5, true, prn14_l2_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1075);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1075);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn5_l1 = findObservation(decoded, 5, SignalType::GPS_L1CA);
    const auto prn5_l2 = findObservation(decoded, 5, SignalType::GPS_L2C);
    const auto prn14_l1 = findObservation(decoded, 14, SignalType::GPS_L1P);
    const auto prn14_l2 = findObservation(decoded, 14, SignalType::GPS_L2P);
    ASSERT_TRUE(prn5_l1.has_value());
    ASSERT_TRUE(prn5_l2.has_value());
    ASSERT_TRUE(prn14_l1.has_value());
    ASSERT_TRUE(prn14_l2.has_value());

    EXPECT_TRUE(prn5_l1->has_doppler);
    EXPECT_TRUE(prn14_l2->has_doppler);
    EXPECT_NEAR(prn5_l1->doppler, prn5_l1_doppler, 0.02);
    EXPECT_NEAR(prn5_l2->doppler, prn5_l2_doppler, 0.02);
    EXPECT_NEAR(prn14_l1->doppler, prn14_l1_doppler, 0.02);
    EXPECT_NEAR(prn14_l2->doppler, prn14_l2_doppler, 0.02);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGps1076PayloadRoundTripHighResolution) {
    ObservationData input(GNSSTime(2304, 244466.250));
    input.addObservation(makeGpsObservation(6, SignalType::GPS_L1CA, 21456789.12, 2.50, 46.25));
    input.addObservation(makeGpsObservation(6, SignalType::GPS_L2C, 21456790.54, 1.75, 43.75));
    input.addObservation(makeGpsObservation(15, SignalType::GPS_L1P, 22456780.34, -1.25, 41.50));
    input.addObservation(makeGpsObservation(15, SignalType::GPS_L2P, 22456781.10, -0.80, 39.25, true));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1076);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1076);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn6_l1 = findObservation(decoded, 6, SignalType::GPS_L1CA);
    const auto prn6_l2 = findObservation(decoded, 6, SignalType::GPS_L2C);
    const auto prn15_l1 = findObservation(decoded, 15, SignalType::GPS_L1P);
    const auto prn15_l2 = findObservation(decoded, 15, SignalType::GPS_L2P);
    ASSERT_TRUE(prn6_l1.has_value());
    ASSERT_TRUE(prn6_l2.has_value());
    ASSERT_TRUE(prn15_l1.has_value());
    ASSERT_TRUE(prn15_l2.has_value());

    EXPECT_NEAR(prn6_l1->pseudorange, 21456789.12, 0.01);
    EXPECT_NEAR(prn6_l2->pseudorange, 21456790.54, 0.01);
    EXPECT_NEAR(prn15_l1->carrier_phase * constants::GPS_L1_WAVELENGTH, 22456779.09, 0.01);
    EXPECT_NEAR(prn15_l2->carrier_phase * constants::GPS_L2_WAVELENGTH, 22456780.30, 0.01);
    EXPECT_NEAR(prn6_l1->snr, 46.25, 0.1);
    EXPECT_NEAR(prn15_l2->snr, 39.25, 0.1);
    EXPECT_FALSE(prn6_l1->has_doppler);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGps1077PayloadRoundTripHighResolutionWithDoppler) {
    const double prn6_rate_mps = 241.20;
    const double prn15_rate_mps = -199.85;
    const double prn6_l1_doppler = -prn6_rate_mps / constants::GPS_L1_WAVELENGTH;
    const double prn6_l2_doppler = -prn6_rate_mps / constants::GPS_L2_WAVELENGTH;
    const double prn15_l1_doppler = -prn15_rate_mps / constants::GPS_L1_WAVELENGTH;
    const double prn15_l2_doppler = -prn15_rate_mps / constants::GPS_L2_WAVELENGTH;

    ObservationData input(GNSSTime(2304, 255577.250));
    input.addObservation(makeGpsObservation(6, SignalType::GPS_L1CA, 21456789.12, 2.50, 46.25, false, prn6_l1_doppler));
    input.addObservation(makeGpsObservation(6, SignalType::GPS_L2C, 21456790.54, 1.75, 43.75, false, prn6_l2_doppler));
    input.addObservation(makeGpsObservation(15, SignalType::GPS_L1P, 22456780.34, -1.25, 41.50, false, prn15_l1_doppler));
    input.addObservation(makeGpsObservation(15, SignalType::GPS_L2P, 22456781.10, -0.80, 39.25, true, prn15_l2_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1077);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1077);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn6_l1 = findObservation(decoded, 6, SignalType::GPS_L1CA);
    const auto prn15_l2 = findObservation(decoded, 15, SignalType::GPS_L2P);
    ASSERT_TRUE(prn6_l1.has_value());
    ASSERT_TRUE(prn15_l2.has_value());
    EXPECT_NEAR(prn6_l1->snr, 46.25, 0.1);
    EXPECT_NEAR(prn15_l2->snr, 39.25, 0.1);
    EXPECT_TRUE(prn6_l1->has_doppler);
    EXPECT_TRUE(prn15_l2->has_doppler);
    EXPECT_NEAR(prn6_l1->doppler, prn6_l1_doppler, 0.02);
    EXPECT_NEAR(prn15_l2->doppler, prn15_l2_doppler, 0.02);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGlonass1084PayloadRoundTrip) {
    processor.setReferencePosition(Vector3d(9753.0, 8642.0, 7531.0));
    processor.setGlonassFrequencyChannel(SatelliteId(GNSSSystem::GLONASS, 7), -4);
    processor.setGlonassFrequencyChannel(SatelliteId(GNSSSystem::GLONASS, 8), 1);

    ObservationData input(GNSSTime(2305, 111222.500));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L1CA, 21456780.25, 1.40, 43.0, -4));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L2CA, 21456781.05, 0.95, 40.0, -4));
    Observation glo_l1p = makeGlonassObservation(
        8, SignalType::GLO_L1P, 22456770.80, -0.65, 39.0, 1);
    glo_l1p.lli |= 0x02U;
    input.addObservation(glo_l1p);
    input.addObservation(makeGlonassObservation(
        8, SignalType::GLO_L2P, 22456771.55, -0.35, 37.0, 1, true));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1084);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1084);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    EXPECT_NEAR(decoded.time.tow, input.time.tow, 1e-3);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.x(), 9753.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.y(), 8642.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.z(), 7531.0);
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn7_l1 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA);
    const auto prn7_l2 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L2CA);
    const auto prn8_l1 = findObservation(decoded, GNSSSystem::GLONASS, 8, SignalType::GLO_L1P);
    const auto prn8_l2 = findObservation(decoded, GNSSSystem::GLONASS, 8, SignalType::GLO_L2P);
    ASSERT_TRUE(prn7_l1.has_value());
    ASSERT_TRUE(prn7_l2.has_value());
    ASSERT_TRUE(prn8_l1.has_value());
    ASSERT_TRUE(prn8_l2.has_value());

    EXPECT_TRUE(prn7_l1->has_glonass_frequency_channel);
    EXPECT_EQ(prn7_l1->glonass_frequency_channel, -4);
    EXPECT_TRUE(prn8_l2->has_glonass_frequency_channel);
    EXPECT_EQ(prn8_l2->glonass_frequency_channel, 1);
    EXPECT_NEAR(prn7_l1->pseudorange, 21456780.25, 0.03);
    EXPECT_NEAR(prn7_l2->pseudorange, 21456781.05, 0.03);
    EXPECT_NEAR(prn8_l1->pseudorange, 22456770.80, 0.03);
    EXPECT_NEAR(prn8_l2->pseudorange, 22456771.55, 0.03);
    EXPECT_NEAR(prn7_l1->carrier_phase * glonassWavelengthForSignal(SignalType::GLO_L1CA, -4), 21456781.65, 0.03);
    EXPECT_NEAR(prn7_l2->carrier_phase * glonassWavelengthForSignal(SignalType::GLO_L2CA, -4), 21456782.00, 0.03);
    EXPECT_NEAR(prn8_l1->carrier_phase * glonassWavelengthForSignal(SignalType::GLO_L1P, 1), 22456770.15, 0.03);
    EXPECT_NEAR(prn8_l2->carrier_phase * glonassWavelengthForSignal(SignalType::GLO_L2P, 1), 22456771.20, 0.03);
    EXPECT_TRUE(prn8_l2->loss_of_lock);
    EXPECT_EQ(prn8_l1->lli & 0x02U, 0x02U);
}

TEST_F(RTCMProcessorTest, DecodesGlonass1084FrameAfterStationMessage) {
    processor.setGlonassFrequencyChannel(SatelliteId(GNSSSystem::GLONASS, 7), -4);

    ObservationData input(GNSSTime(2305, 211122.250));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L1CA, 20200000.10, 0.85, 42.0, -4));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L2CA, 20200000.95, 1.20, 39.0, -4));

    const Vector3d expected_station(3875000.1234, 332100.5, 5026000.9876);
    const auto station_frame = buildRtcm1005(expected_station.x(), expected_station.y(), expected_station.z());
    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1084);
    ASSERT_TRUE(encoded.valid);
    const auto obs_frame = buildRtcmFrame(encoded);

    std::vector<uint8_t> stream;
    stream.reserve(station_frame.size() + obs_frame.size());
    stream.insert(stream.end(), station_frame.begin(), station_frame.end());
    stream.insert(stream.end(), obs_frame.begin(), obs_frame.end());

    const auto decoded_messages = processor.decode(stream.data(), stream.size());
    ASSERT_EQ(decoded_messages.size(), 2U);
    EXPECT_EQ(decoded_messages[0].type, io::RTCMMessageType::RTCM_1005);
    EXPECT_EQ(decoded_messages[1].type, io::RTCMMessageType::RTCM_1084);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(decoded_messages[1], decoded));
    EXPECT_NEAR(decoded.time.tow, input.time.tow, 1e-3);
    EXPECT_NEAR(decoded.receiver_position.x(), expected_station.x(), 1e-4);
    EXPECT_NEAR(decoded.receiver_position.y(), expected_station.y(), 1e-4);
    EXPECT_NEAR(decoded.receiver_position.z(), expected_station.z(), 1e-4);
    ASSERT_EQ(decoded.observations.size(), 2U);

    const auto l1 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA);
    const auto l2 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L2CA);
    ASSERT_TRUE(l1.has_value());
    ASSERT_TRUE(l2.has_value());
    EXPECT_NEAR(l1->pseudorange, 20200000.10, 0.03);
    EXPECT_NEAR(l2->pseudorange, 20200000.95, 0.03);
    EXPECT_TRUE(l1->has_glonass_frequency_channel);
    EXPECT_EQ(l1->glonass_frequency_channel, -4);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGlonass1085PayloadRoundTripWithDoppler) {
    processor.setReferencePosition(Vector3d(9753.0, 8642.0, 7531.0));
    const double prn7_rate_mps = 226.35;
    const double prn8_rate_mps = -182.75;
    const double prn7_l1_doppler =
        -prn7_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L1CA, -4);
    const double prn7_l2_doppler =
        -prn7_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L2CA, -4);
    const double prn8_l1_doppler =
        -prn8_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L1P, 1);
    const double prn8_l2_doppler =
        -prn8_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L2P, 1);

    ObservationData input(GNSSTime(2305, 188222.750));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L1CA, 21456780.25, 1.40, 43.0, -4, false, prn7_l1_doppler));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L2CA, 21456781.05, 0.95, 40.0, -4, false, prn7_l2_doppler));
    input.addObservation(makeGlonassObservation(
        8, SignalType::GLO_L1P, 22456770.80, -0.65, 39.0, 1, false, prn8_l1_doppler));
    input.addObservation(makeGlonassObservation(
        8, SignalType::GLO_L2P, 22456771.55, -0.35, 37.0, 1, true, prn8_l2_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1085);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1085);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn7_l1 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA);
    const auto prn7_l2 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L2CA);
    const auto prn8_l1 = findObservation(decoded, GNSSSystem::GLONASS, 8, SignalType::GLO_L1P);
    const auto prn8_l2 = findObservation(decoded, GNSSSystem::GLONASS, 8, SignalType::GLO_L2P);
    ASSERT_TRUE(prn7_l1.has_value());
    ASSERT_TRUE(prn7_l2.has_value());
    ASSERT_TRUE(prn8_l1.has_value());
    ASSERT_TRUE(prn8_l2.has_value());

    EXPECT_TRUE(prn7_l1->has_glonass_frequency_channel);
    EXPECT_EQ(prn7_l1->glonass_frequency_channel, -4);
    EXPECT_TRUE(prn8_l2->has_glonass_frequency_channel);
    EXPECT_EQ(prn8_l2->glonass_frequency_channel, 1);
    EXPECT_TRUE(prn7_l1->has_doppler);
    EXPECT_TRUE(prn8_l2->has_doppler);
    EXPECT_NEAR(prn7_l1->doppler, prn7_l1_doppler, 0.05);
    EXPECT_NEAR(prn7_l2->doppler, prn7_l2_doppler, 0.05);
    EXPECT_NEAR(prn8_l1->doppler, prn8_l1_doppler, 0.05);
    EXPECT_NEAR(prn8_l2->doppler, prn8_l2_doppler, 0.05);
}

TEST_F(RTCMProcessorTest, DecodesGlonass1085FrameWithoutFrequencyChannelCache) {
    const double prn7_rate_mps = 188.40;
    const double prn7_l1_doppler =
        -prn7_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L1CA, -4);
    const double prn7_l2_doppler =
        -prn7_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L2CA, -4);
    ObservationData input(GNSSTime(2305, 211188.250));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L1CA, 20200000.10, 0.85, 42.0, -4, false, prn7_l1_doppler));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L2CA, 20200000.95, 1.20, 39.0, -4, false, prn7_l2_doppler));

    const Vector3d expected_station(3875000.1234, 332100.5, 5026000.9876);
    const auto station_frame = buildRtcm1005(expected_station.x(), expected_station.y(), expected_station.z());
    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1085);
    ASSERT_TRUE(encoded.valid);

    processor.clear();

    const auto obs_frame = buildRtcmFrame(encoded);
    std::vector<uint8_t> stream;
    stream.reserve(station_frame.size() + obs_frame.size());
    stream.insert(stream.end(), station_frame.begin(), station_frame.end());
    stream.insert(stream.end(), obs_frame.begin(), obs_frame.end());

    const auto decoded_messages = processor.decode(stream.data(), stream.size());
    ASSERT_EQ(decoded_messages.size(), 2U);
    EXPECT_EQ(decoded_messages[1].type, io::RTCMMessageType::RTCM_1085);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(decoded_messages[1], decoded));
    const auto l1 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA);
    const auto l2 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L2CA);
    ASSERT_TRUE(l1.has_value());
    ASSERT_TRUE(l2.has_value());
    EXPECT_TRUE(l1->has_glonass_frequency_channel);
    EXPECT_EQ(l1->glonass_frequency_channel, -4);
    EXPECT_TRUE(l1->has_doppler);
    EXPECT_NEAR(l1->doppler, prn7_l1_doppler, 0.05);
    EXPECT_NEAR(l2->doppler, prn7_l2_doppler, 0.05);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGlonass1086PayloadRoundTripHighResolution) {
    processor.setGlonassFrequencyChannel(SatelliteId(GNSSSystem::GLONASS, 7), -4);
    processor.setGlonassFrequencyChannel(SatelliteId(GNSSSystem::GLONASS, 8), 1);

    ObservationData input(GNSSTime(2305, 199333.500));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L1CA, 21456780.25, 1.40, 43.125, -4));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L2CA, 21456781.05, 0.95, 40.875, -4));
    input.addObservation(makeGlonassObservation(
        8, SignalType::GLO_L1P, 22456770.80, -0.65, 39.500, 1));
    input.addObservation(makeGlonassObservation(
        8, SignalType::GLO_L2P, 22456771.55, -0.35, 37.250, 1, true));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1086);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1086);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn7_l1 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA);
    const auto prn8_l2 = findObservation(decoded, GNSSSystem::GLONASS, 8, SignalType::GLO_L2P);
    ASSERT_TRUE(prn7_l1.has_value());
    ASSERT_TRUE(prn8_l2.has_value());

    EXPECT_TRUE(prn7_l1->has_glonass_frequency_channel);
    EXPECT_EQ(prn7_l1->glonass_frequency_channel, -4);
    EXPECT_NEAR(prn7_l1->snr, 43.125, 0.1);
    EXPECT_NEAR(prn8_l2->snr, 37.250, 0.1);
    EXPECT_FALSE(prn7_l1->has_doppler);
}

TEST_F(RTCMProcessorTest, DecodesGlonass1086FrameAfter1020CacheSeed) {
    ObservationData input(GNSSTime(2305, 201234.750));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L1CA, 20200000.10, 0.85, 42.125, -4));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L2CA, 20200000.95, 1.20, 39.875, -4));

    const io::RTCMMessage eph_message = processor.encodeEphemeris(makeGlonassEphemeris());
    ASSERT_TRUE(eph_message.valid);
    const io::RTCMMessage obs_message =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1086);
    ASSERT_TRUE(obs_message.valid);

    processor.clear();

    const auto eph_frame = buildRtcmFrame(eph_message);
    const auto obs_frame = buildRtcmFrame(obs_message);
    std::vector<uint8_t> stream;
    stream.reserve(eph_frame.size() + obs_frame.size());
    stream.insert(stream.end(), eph_frame.begin(), eph_frame.end());
    stream.insert(stream.end(), obs_frame.begin(), obs_frame.end());

    const auto decoded_messages = processor.decode(stream.data(), stream.size());
    ASSERT_EQ(decoded_messages.size(), 2U);
    EXPECT_EQ(decoded_messages[0].type, io::RTCMMessageType::RTCM_1020);
    EXPECT_EQ(decoded_messages[1].type, io::RTCMMessageType::RTCM_1086);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(decoded_messages[1], decoded));
    const auto l1 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA);
    const auto l2 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L2CA);
    ASSERT_TRUE(l1.has_value());
    ASSERT_TRUE(l2.has_value());
    EXPECT_TRUE(l1->has_glonass_frequency_channel);
    EXPECT_EQ(l1->glonass_frequency_channel, -4);
    EXPECT_NEAR(l1->carrier_phase * glonassWavelengthForSignal(SignalType::GLO_L1CA, -4), 20200000.95, 0.01);
    EXPECT_NEAR(l2->carrier_phase * glonassWavelengthForSignal(SignalType::GLO_L2CA, -4), 20200002.15, 0.01);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGlonass1087PayloadRoundTripHighResolutionWithDoppler) {
    const double prn7_rate_mps = 214.60;
    const double prn8_rate_mps = -176.30;
    const double prn7_l1_doppler =
        -prn7_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L1CA, -4);
    const double prn7_l2_doppler =
        -prn7_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L2CA, -4);
    const double prn8_l1_doppler =
        -prn8_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L1P, 1);
    const double prn8_l2_doppler =
        -prn8_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L2P, 1);

    ObservationData input(GNSSTime(2305, 204567.500));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L1CA, 21456780.25, 1.40, 43.125, -4, false, prn7_l1_doppler));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L2CA, 21456781.05, 0.95, 40.875, -4, false, prn7_l2_doppler));
    input.addObservation(makeGlonassObservation(
        8, SignalType::GLO_L1P, 22456770.80, -0.65, 39.500, 1, false, prn8_l1_doppler));
    input.addObservation(makeGlonassObservation(
        8, SignalType::GLO_L2P, 22456771.55, -0.35, 37.250, 1, true, prn8_l2_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1087);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1087);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto prn7_l1 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA);
    const auto prn8_l2 = findObservation(decoded, GNSSSystem::GLONASS, 8, SignalType::GLO_L2P);
    ASSERT_TRUE(prn7_l1.has_value());
    ASSERT_TRUE(prn8_l2.has_value());
    EXPECT_TRUE(prn7_l1->has_glonass_frequency_channel);
    EXPECT_EQ(prn7_l1->glonass_frequency_channel, -4);
    EXPECT_NEAR(prn7_l1->snr, 43.125, 0.1);
    EXPECT_NEAR(prn8_l2->snr, 37.250, 0.1);
    EXPECT_TRUE(prn7_l1->has_doppler);
    EXPECT_TRUE(prn8_l2->has_doppler);
    EXPECT_NEAR(prn7_l1->doppler, prn7_l1_doppler, 0.05);
    EXPECT_NEAR(prn8_l2->doppler, prn8_l2_doppler, 0.05);
}

TEST_F(RTCMProcessorTest, DecodesGlonass1087FrameWithoutFrequencyChannelCache) {
    const double prn7_rate_mps = 188.40;
    const double prn7_l1_doppler =
        -prn7_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L1CA, -4);
    const double prn7_l2_doppler =
        -prn7_rate_mps / glonassWavelengthForSignal(SignalType::GLO_L2CA, -4);
    ObservationData input(GNSSTime(2305, 211588.250));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L1CA, 20200000.10, 0.85, 42.125, -4, false, prn7_l1_doppler));
    input.addObservation(makeGlonassObservation(
        7, SignalType::GLO_L2CA, 20200000.95, 1.20, 39.875, -4, false, prn7_l2_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1087);
    ASSERT_TRUE(encoded.valid);

    processor.clear();

    const auto obs_frame = buildRtcmFrame(encoded);
    const auto decoded_messages = processor.decode(obs_frame.data(), obs_frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages[0].type, io::RTCMMessageType::RTCM_1087);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(decoded_messages[0], decoded));
    const auto l1 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA);
    const auto l2 = findObservation(decoded, GNSSSystem::GLONASS, 7, SignalType::GLO_L2CA);
    ASSERT_TRUE(l1.has_value());
    ASSERT_TRUE(l2.has_value());
    EXPECT_TRUE(l1->has_glonass_frequency_channel);
    EXPECT_EQ(l1->glonass_frequency_channel, -4);
    EXPECT_TRUE(l1->has_doppler);
    EXPECT_NEAR(l1->doppler, prn7_l1_doppler, 0.05);
    EXPECT_NEAR(l2->doppler, prn7_l2_doppler, 0.05);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGalileo1094PayloadRoundTrip) {
    processor.setReferencePosition(Vector3d(7654.0, 8765.0, 9876.0));

    ObservationData input(GNSSTime(2305, 223344.125));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 11, SignalType::GAL_E1, 24456789.25, 1.90, 45.0));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 11, SignalType::GAL_E5A, 24456790.10, 1.15, 42.0));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 19, SignalType::GAL_E6, 25456780.80, -0.45, 40.0));
    Observation gal_e5b = makeObservation(
        GNSSSystem::Galileo, 19, SignalType::GAL_E5B, 25456781.55, -0.20, 38.0, true);
    gal_e5b.lli |= 0x02U;
    input.addObservation(gal_e5b);

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1094);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1094);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    EXPECT_NEAR(decoded.time.tow, input.time.tow, 1e-3);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.x(), 7654.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.y(), 8765.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.z(), 9876.0);
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto e1 = findObservation(decoded, GNSSSystem::Galileo, 11, SignalType::GAL_E1);
    const auto e5a = findObservation(decoded, GNSSSystem::Galileo, 11, SignalType::GAL_E5A);
    const auto e6 = findObservation(decoded, GNSSSystem::Galileo, 19, SignalType::GAL_E6);
    const auto e5b = findObservation(decoded, GNSSSystem::Galileo, 19, SignalType::GAL_E5B);
    ASSERT_TRUE(e1.has_value());
    ASSERT_TRUE(e5a.has_value());
    ASSERT_TRUE(e6.has_value());
    ASSERT_TRUE(e5b.has_value());

    EXPECT_NEAR(e1->pseudorange, 24456789.25, 0.03);
    EXPECT_NEAR(e5a->pseudorange, 24456790.10, 0.03);
    EXPECT_NEAR(e6->pseudorange, 25456780.80, 0.03);
    EXPECT_NEAR(e5b->pseudorange, 25456781.55, 0.03);
    EXPECT_NEAR(e1->carrier_phase * constants::GAL_E1_WAVELENGTH, 24456791.15, 0.03);
    EXPECT_NEAR(e5a->carrier_phase * constants::GAL_E5A_WAVELENGTH, 24456791.25, 0.03);
    EXPECT_NEAR(e6->carrier_phase * constants::GAL_E6_WAVELENGTH, 25456780.35, 0.03);
    EXPECT_NEAR(e5b->carrier_phase * constants::GAL_E5B_WAVELENGTH, 25456781.35, 0.03);
    EXPECT_TRUE(e5b->loss_of_lock);
    EXPECT_EQ(e5b->lli & 0x02U, 0x02U);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGalileo1095PayloadRoundTripWithDoppler) {
    const double prn11_rate_mps = 251.75;
    const double prn19_rate_mps = -190.30;
    const double e1_doppler = -prn11_rate_mps / constants::GAL_E1_WAVELENGTH;
    const double e5a_doppler = -prn11_rate_mps / constants::GAL_E5A_WAVELENGTH;
    const double e6_doppler = -prn19_rate_mps / constants::GAL_E6_WAVELENGTH;
    const double e5b_doppler = -prn19_rate_mps / constants::GAL_E5B_WAVELENGTH;
    ObservationData input(GNSSTime(2305, 255577.625));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 11, SignalType::GAL_E1, 24456789.25, 1.90, 45.0, false, e1_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 11, SignalType::GAL_E5A, 24456790.10, 1.15, 42.0, false, e5a_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 19, SignalType::GAL_E6, 25456780.80, -0.45, 40.0, false, e6_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 19, SignalType::GAL_E5B, 25456781.55, -0.20, 38.0, true, e5b_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1095);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1095);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto e1 = findObservation(decoded, GNSSSystem::Galileo, 11, SignalType::GAL_E1);
    const auto e5a = findObservation(decoded, GNSSSystem::Galileo, 11, SignalType::GAL_E5A);
    const auto e6 = findObservation(decoded, GNSSSystem::Galileo, 19, SignalType::GAL_E6);
    const auto e5b = findObservation(decoded, GNSSSystem::Galileo, 19, SignalType::GAL_E5B);
    ASSERT_TRUE(e1.has_value());
    ASSERT_TRUE(e5a.has_value());
    ASSERT_TRUE(e6.has_value());
    ASSERT_TRUE(e5b.has_value());

    EXPECT_TRUE(e1->has_doppler);
    EXPECT_TRUE(e5b->has_doppler);
    EXPECT_NEAR(e1->doppler, e1_doppler, 0.02);
    EXPECT_NEAR(e5a->doppler, e5a_doppler, 0.02);
    EXPECT_NEAR(e6->doppler, e6_doppler, 0.02);
    EXPECT_NEAR(e5b->doppler, e5b_doppler, 0.02);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGalileo1096PayloadRoundTripHighResolution) {
    ObservationData input(GNSSTime(2305, 277799.125));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 11, SignalType::GAL_E1, 24456789.25, 1.90, 45.125));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 11, SignalType::GAL_E5A, 24456790.10, 1.15, 42.875));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 19, SignalType::GAL_E6, 25456780.80, -0.45, 40.500));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 19, SignalType::GAL_E5B, 25456781.55, -0.20, 38.250, true));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1096);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1096);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto e1 = findObservation(decoded, GNSSSystem::Galileo, 11, SignalType::GAL_E1);
    const auto e5b = findObservation(decoded, GNSSSystem::Galileo, 19, SignalType::GAL_E5B);
    ASSERT_TRUE(e1.has_value());
    ASSERT_TRUE(e5b.has_value());
    EXPECT_NEAR(e1->snr, 45.125, 0.1);
    EXPECT_NEAR(e5b->snr, 38.250, 0.1);
    EXPECT_FALSE(e1->has_doppler);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGalileo1097PayloadRoundTripHighResolutionWithDoppler) {
    const double prn11_rate_mps = 251.75;
    const double prn19_rate_mps = -190.30;
    const double e1_doppler = -prn11_rate_mps / constants::GAL_E1_WAVELENGTH;
    const double e5a_doppler = -prn11_rate_mps / constants::GAL_E5A_WAVELENGTH;
    const double e6_doppler = -prn19_rate_mps / constants::GAL_E6_WAVELENGTH;
    const double e5b_doppler = -prn19_rate_mps / constants::GAL_E5B_WAVELENGTH;

    ObservationData input(GNSSTime(2305, 288899.125));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 11, SignalType::GAL_E1, 24456789.25, 1.90, 45.125, false, e1_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 11, SignalType::GAL_E5A, 24456790.10, 1.15, 42.875, false, e5a_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 19, SignalType::GAL_E6, 25456780.80, -0.45, 40.500, false, e6_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::Galileo, 19, SignalType::GAL_E5B, 25456781.55, -0.20, 38.250, true, e5b_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1097);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1097);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 4U);

    const auto e1 = findObservation(decoded, GNSSSystem::Galileo, 11, SignalType::GAL_E1);
    const auto e5b = findObservation(decoded, GNSSSystem::Galileo, 19, SignalType::GAL_E5B);
    ASSERT_TRUE(e1.has_value());
    ASSERT_TRUE(e5b.has_value());
    EXPECT_NEAR(e1->snr, 45.125, 0.1);
    EXPECT_NEAR(e5b->snr, 38.250, 0.1);
    EXPECT_TRUE(e1->has_doppler);
    EXPECT_TRUE(e5b->has_doppler);
    EXPECT_NEAR(e1->doppler, e1_doppler, 0.02);
    EXPECT_NEAR(e5b->doppler, e5b_doppler, 0.02);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesBeiDou1124PayloadRoundTrip) {
    processor.setReferencePosition(Vector3d(1357.0, 2468.0, 3579.0));

    ObservationData input(GNSSTime(2306, 323456.875));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 8, SignalType::BDS_B1I, 21456780.50, 1.20, 44.0));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 8, SignalType::BDS_B2I, 21456781.20, 0.85, 41.0));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 18, SignalType::BDS_B3I, 22456770.40, -0.55, 39.0));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1124);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1124);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    EXPECT_NEAR(decoded.time.tow, input.time.tow, 1e-3);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.x(), 1357.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.y(), 2468.0);
    EXPECT_DOUBLE_EQ(decoded.receiver_position.z(), 3579.0);
    ASSERT_EQ(decoded.observations.size(), 3U);

    const auto b1i = findObservation(decoded, GNSSSystem::BeiDou, 8, SignalType::BDS_B1I);
    const auto b2i = findObservation(decoded, GNSSSystem::BeiDou, 8, SignalType::BDS_B2I);
    const auto b3i = findObservation(decoded, GNSSSystem::BeiDou, 18, SignalType::BDS_B3I);
    ASSERT_TRUE(b1i.has_value());
    ASSERT_TRUE(b2i.has_value());
    ASSERT_TRUE(b3i.has_value());

    EXPECT_NEAR(b1i->pseudorange, 21456780.50, 0.03);
    EXPECT_NEAR(b2i->pseudorange, 21456781.20, 0.03);
    EXPECT_NEAR(b3i->pseudorange, 22456770.40, 0.03);
    EXPECT_NEAR(b1i->carrier_phase * constants::BDS_B1I_WAVELENGTH, 21456781.70, 0.03);
    EXPECT_NEAR(b2i->carrier_phase * constants::BDS_B2I_WAVELENGTH, 21456782.05, 0.03);
    EXPECT_NEAR(b3i->carrier_phase * constants::BDS_B3I_WAVELENGTH, 22456769.85, 0.03);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesBeiDou1125PayloadRoundTripWithDoppler) {
    const double prn8_rate_mps = 238.60;
    const double prn18_rate_mps = -196.45;
    const double b1i_doppler = -prn8_rate_mps / constants::BDS_B1I_WAVELENGTH;
    const double b2i_doppler = -prn8_rate_mps / constants::BDS_B2I_WAVELENGTH;
    const double b3i_doppler = -prn18_rate_mps / constants::BDS_B3I_WAVELENGTH;
    ObservationData input(GNSSTime(2306, 423456.875));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 8, SignalType::BDS_B1I, 21456780.50, 1.20, 44.0, false, b1i_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 8, SignalType::BDS_B2I, 21456781.20, 0.85, 41.0, false, b2i_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 18, SignalType::BDS_B3I, 22456770.40, -0.55, 39.0, false, b3i_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1125);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1125);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 3U);

    const auto b1i = findObservation(decoded, GNSSSystem::BeiDou, 8, SignalType::BDS_B1I);
    const auto b2i = findObservation(decoded, GNSSSystem::BeiDou, 8, SignalType::BDS_B2I);
    const auto b3i = findObservation(decoded, GNSSSystem::BeiDou, 18, SignalType::BDS_B3I);
    ASSERT_TRUE(b1i.has_value());
    ASSERT_TRUE(b2i.has_value());
    ASSERT_TRUE(b3i.has_value());

    EXPECT_TRUE(b1i->has_doppler);
    EXPECT_TRUE(b3i->has_doppler);
    EXPECT_NEAR(b1i->doppler, b1i_doppler, 0.02);
    EXPECT_NEAR(b2i->doppler, b2i_doppler, 0.02);
    EXPECT_NEAR(b3i->doppler, b3i_doppler, 0.02);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesBeiDou1126PayloadRoundTripHighResolution) {
    ObservationData input(GNSSTime(2306, 455678.500));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 8, SignalType::BDS_B1I, 21456780.50, 1.20, 44.125));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 8, SignalType::BDS_B2I, 21456781.20, 0.85, 41.875));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 18, SignalType::BDS_B3I, 22456770.40, -0.55, 39.500));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1126);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1126);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 3U);

    const auto b1i = findObservation(decoded, GNSSSystem::BeiDou, 8, SignalType::BDS_B1I);
    const auto b3i = findObservation(decoded, GNSSSystem::BeiDou, 18, SignalType::BDS_B3I);
    ASSERT_TRUE(b1i.has_value());
    ASSERT_TRUE(b3i.has_value());
    EXPECT_NEAR(b1i->snr, 44.125, 0.1);
    EXPECT_NEAR(b3i->snr, 39.500, 0.1);
    EXPECT_FALSE(b1i->has_doppler);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesBeiDou1127PayloadRoundTripHighResolutionWithDoppler) {
    const double prn8_rate_mps = 238.60;
    const double prn18_rate_mps = -196.45;
    const double b1i_doppler = -prn8_rate_mps / constants::BDS_B1I_WAVELENGTH;
    const double b2i_doppler = -prn8_rate_mps / constants::BDS_B2I_WAVELENGTH;
    const double b3i_doppler = -prn18_rate_mps / constants::BDS_B3I_WAVELENGTH;

    ObservationData input(GNSSTime(2306, 466789.500));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 8, SignalType::BDS_B1I, 21456780.50, 1.20, 44.125, false, b1i_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 8, SignalType::BDS_B2I, 21456781.20, 0.85, 41.875, false, b2i_doppler));
    input.addObservation(makeObservation(
        GNSSSystem::BeiDou, 18, SignalType::BDS_B3I, 22456770.40, -0.55, 39.500, false, b3i_doppler));

    const io::RTCMMessage encoded =
        processor.encodeObservations(input, io::RTCMMessageType::RTCM_1127);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1127);

    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(encoded, decoded));
    ASSERT_EQ(decoded.observations.size(), 3U);

    const auto b1i = findObservation(decoded, GNSSSystem::BeiDou, 8, SignalType::BDS_B1I);
    const auto b3i = findObservation(decoded, GNSSSystem::BeiDou, 18, SignalType::BDS_B3I);
    ASSERT_TRUE(b1i.has_value());
    ASSERT_TRUE(b3i.has_value());
    EXPECT_NEAR(b1i->snr, 44.125, 0.1);
    EXPECT_NEAR(b3i->snr, 39.500, 0.1);
    EXPECT_TRUE(b1i->has_doppler);
    EXPECT_TRUE(b3i->has_doppler);
    EXPECT_NEAR(b1i->doppler, b1i_doppler, 0.02);
    EXPECT_NEAR(b3i->doppler, b3i_doppler, 0.02);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGps1019EphemerisRoundTrip) {
    const Ephemeris input = makeGpsEphemeris();

    const io::RTCMMessage encoded = processor.encodeEphemeris(input);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1019);
    ASSERT_FALSE(encoded.data.empty());

    NavigationData nav_data;
    ASSERT_TRUE(processor.decodeNavigationData(encoded, nav_data));
    const Ephemeris* decoded = nav_data.getEphemeris(input.satellite, input.toe);
    ASSERT_NE(decoded, nullptr);

    EXPECT_EQ(decoded->satellite.prn, input.satellite.prn);
    EXPECT_EQ(decoded->week, input.week);
    EXPECT_EQ(decoded->iode, input.iode);
    EXPECT_EQ(decoded->iodc, input.iodc);
    EXPECT_EQ(decoded->ura, input.ura);
    EXPECT_EQ(decoded->health, input.health);
    EXPECT_NEAR(decoded->toe.tow, input.toe.tow, 1e-6);
    EXPECT_NEAR(decoded->toc.tow, input.toc.tow, 1e-6);
    EXPECT_NEAR(decoded->sqrt_a, input.sqrt_a, 2.0e-6);
    EXPECT_NEAR(decoded->e, input.e, 2.5e-10);
    EXPECT_NEAR(decoded->m0, input.m0, 2.0e-9);
    EXPECT_NEAR(decoded->delta_n, input.delta_n, 5.0e-13);
    EXPECT_NEAR(decoded->omega0, input.omega0, 2.0e-9);
    EXPECT_NEAR(decoded->omega, input.omega, 2.0e-9);
    EXPECT_NEAR(decoded->omega_dot, input.omega_dot, 5.0e-13);
    EXPECT_NEAR(decoded->i0, input.i0, 2.0e-9);
    EXPECT_NEAR(decoded->idot, input.idot, 5.0e-13);
    EXPECT_NEAR(decoded->cuc, input.cuc, 3.0e-9);
    EXPECT_NEAR(decoded->cus, input.cus, 3.0e-9);
    EXPECT_NEAR(decoded->cic, input.cic, 3.0e-9);
    EXPECT_NEAR(decoded->cis, input.cis, 3.0e-9);
    EXPECT_NEAR(decoded->crc, input.crc, 0.05);
    EXPECT_NEAR(decoded->crs, input.crs, 0.05);
    EXPECT_NEAR(decoded->af0, input.af0, 1.0e-9);
    EXPECT_NEAR(decoded->af1, input.af1, 5.0e-14);
    EXPECT_NEAR(decoded->af2, input.af2, 1.0e-16);
    EXPECT_NEAR(decoded->tgd, input.tgd, 1.0e-9);
}

TEST_F(RTCMProcessorTest, DecodesGps1019FrameIntoNavigationData) {
    const Ephemeris input = makeGpsEphemeris();
    const io::RTCMMessage encoded = processor.encodeEphemeris(input);
    ASSERT_TRUE(encoded.valid);

    const auto frame = buildRtcmFrame(encoded);
    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1019);

    NavigationData nav_data;
    ASSERT_TRUE(processor.decodeNavigationData(decoded_messages.front(), nav_data));
    const Ephemeris* decoded = nav_data.getEphemeris(input.satellite, input.toe);
    ASSERT_NE(decoded, nullptr);

    Vector3d input_pos;
    Vector3d input_vel;
    Vector3d decoded_pos;
    Vector3d decoded_vel;
    double input_clk_bias = 0.0;
    double input_clk_drift = 0.0;
    double decoded_clk_bias = 0.0;
    double decoded_clk_drift = 0.0;
    const GNSSTime eval_time = input.toe + 60.0;
    ASSERT_TRUE(input.calculateSatelliteState(eval_time, input_pos, input_vel, input_clk_bias, input_clk_drift));
    ASSERT_TRUE(decoded->calculateSatelliteState(
        eval_time, decoded_pos, decoded_vel, decoded_clk_bias, decoded_clk_drift));
    EXPECT_NEAR((decoded_pos - input_pos).norm(), 0.0, 0.5);
    EXPECT_NEAR((decoded_vel - input_vel).norm(), 0.0, 0.02);
    EXPECT_NEAR(decoded_clk_bias, input_clk_bias, 1.0e-9);
    EXPECT_NEAR(decoded_clk_drift, input_clk_drift, 1.0e-12);
}

TEST_F(RTCMProcessorTest, EncodesAndDecodesGlonass1020EphemerisRoundTrip) {
    const Ephemeris input = makeGlonassEphemeris();

    const io::RTCMMessage encoded = processor.encodeEphemeris(input);
    ASSERT_TRUE(encoded.valid);
    ASSERT_EQ(encoded.type, io::RTCMMessageType::RTCM_1020);
    ASSERT_FALSE(encoded.data.empty());

    NavigationData nav_data;
    ASSERT_TRUE(processor.decodeNavigationData(encoded, nav_data));
    const Ephemeris* decoded = nav_data.getEphemeris(input.satellite, input.toe);
    ASSERT_NE(decoded, nullptr);

    EXPECT_EQ(decoded->satellite.system, GNSSSystem::GLONASS);
    EXPECT_EQ(decoded->satellite.prn, input.satellite.prn);
    EXPECT_EQ(decoded->glonass_frequency_channel, input.glonass_frequency_channel);
    EXPECT_EQ(decoded->glonass_age, input.glonass_age);
    EXPECT_EQ(decoded->health, input.health);
    EXPECT_NEAR(decoded->toe.tow, input.toe.tow, 1e-6);
    EXPECT_NEAR(decoded->tof.tow, input.tof.tow, 1e-6);
    EXPECT_NEAR(decoded->glonass_position.x(), input.glonass_position.x(), 0.5);
    EXPECT_NEAR(decoded->glonass_position.y(), input.glonass_position.y(), 0.5);
    EXPECT_NEAR(decoded->glonass_position.z(), input.glonass_position.z(), 0.5);
    EXPECT_NEAR(decoded->glonass_velocity.x(), input.glonass_velocity.x(), 0.002);
    EXPECT_NEAR(decoded->glonass_velocity.y(), input.glonass_velocity.y(), 0.002);
    EXPECT_NEAR(decoded->glonass_velocity.z(), input.glonass_velocity.z(), 0.002);
    EXPECT_NEAR(decoded->glonass_acceleration.x(), input.glonass_acceleration.x(), 1.0e-6);
    EXPECT_NEAR(decoded->glonass_acceleration.y(), input.glonass_acceleration.y(), 1.0e-6);
    EXPECT_NEAR(decoded->glonass_acceleration.z(), input.glonass_acceleration.z(), 1.0e-6);
    EXPECT_NEAR(decoded->glonass_taun, input.glonass_taun, 2.0e-9);
    EXPECT_NEAR(decoded->glonass_gamn, input.glonass_gamn, 1.0e-12);
}

TEST_F(RTCMProcessorTest, DecodesGlonass1020FrameIntoNavigationData) {
    const Ephemeris input = makeGlonassEphemeris();
    const io::RTCMMessage encoded = processor.encodeEphemeris(input);
    ASSERT_TRUE(encoded.valid);

    const auto frame = buildRtcmFrame(encoded);
    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1020);

    NavigationData nav_data;
    ASSERT_TRUE(processor.decodeNavigationData(decoded_messages.front(), nav_data));
    const Ephemeris* decoded = nav_data.getEphemeris(input.satellite, input.toe);
    ASSERT_NE(decoded, nullptr);

    Vector3d input_pos;
    Vector3d input_vel;
    Vector3d decoded_pos;
    Vector3d decoded_vel;
    double input_clk_bias = 0.0;
    double input_clk_drift = 0.0;
    double decoded_clk_bias = 0.0;
    double decoded_clk_drift = 0.0;
    const GNSSTime eval_time = input.toe + 60.0;
    ASSERT_TRUE(input.calculateSatelliteState(
        eval_time, input_pos, input_vel, input_clk_bias, input_clk_drift));
    ASSERT_TRUE(decoded->calculateSatelliteState(
        eval_time, decoded_pos, decoded_vel, decoded_clk_bias, decoded_clk_drift));
    EXPECT_NEAR((decoded_pos - input_pos).norm(), 0.0, 2.0);
    EXPECT_NEAR((decoded_vel - input_vel).norm(), 0.0, 0.01);
    EXPECT_NEAR(decoded_clk_bias, input_clk_bias, 2.0e-9);
    EXPECT_NEAR(decoded_clk_drift, input_clk_drift, 1.0e-12);
}

namespace {

struct GalileoEphemerisRaw {
    uint8_t prn = 11;
    int gst_week = 1251;       // GPS week 2275 (2023-08-17)
    int iodnav = 77;
    int sisa = 107;            // 2.0 + 7 * 0.16 m
    int64_t idot = -120;
    int toc_min = 5880;        // 352800 s
    int64_t af2 = 0;
    int64_t af1 = -1234;
    int64_t af0 = -123456789;
    int64_t crs = -1500;
    int64_t delta_n = 9000;
    int64_t m0 = 1000000000;
    int64_t cuc = -2000;
    uint64_t e = 2000000;      // 2.3e-4
    int64_t cus = 3000;
    uint64_t sqrt_a = 2852062985ULL;  // ~5440.6 m^0.5
    int toe_min = 5880;
    int64_t cic = 40;
    int64_t omega0 = -700000000;
    int64_t cis = -30;
    int64_t i0 = 660000000;
    int64_t crc = 4000;
    int64_t omega = 300000000;
    int64_t omega_dot = -1200;
    int64_t bgd_e5a = -5;
    int64_t bgd_e5b = -7;
};

io::RTCMMessage buildGalileoEphemerisMessage(const GalileoEphemerisRaw& raw, bool inav) {
    std::vector<uint8_t> payload(inav ? 63U : 62U, 0);
    int bit = 0;
    const auto u = [&](int len, uint64_t value) { setUnsignedBits(payload, bit, len, value); bit += len; };
    const auto s = [&](int len, int64_t value) { setSignedBits(payload, bit, len, value); bit += len; };
    u(12, inav ? 1046U : 1045U);
    u(6, raw.prn);
    u(12, static_cast<uint64_t>(raw.gst_week));
    u(10, static_cast<uint64_t>(raw.iodnav));
    u(8, static_cast<uint64_t>(raw.sisa));
    s(14, raw.idot);
    u(14, static_cast<uint64_t>(raw.toc_min));
    s(6, raw.af2);
    s(21, raw.af1);
    s(31, raw.af0);
    s(16, raw.crs);
    s(16, raw.delta_n);
    s(32, raw.m0);
    s(16, raw.cuc);
    u(32, raw.e);
    s(16, raw.cus);
    u(32, raw.sqrt_a);
    u(14, static_cast<uint64_t>(raw.toe_min));
    s(16, raw.cic);
    s(32, raw.omega0);
    s(16, raw.cis);
    s(32, raw.i0);
    s(16, raw.crc);
    s(32, raw.omega);
    s(24, raw.omega_dot);
    s(10, raw.bgd_e5a);
    if (inav) {
        s(10, raw.bgd_e5b);
        u(2, 0);  // E5b health
        u(1, 0);
        u(2, 1);  // E1-B health: out of service
        u(1, 0);
    } else {
        u(2, 0);
        u(1, 1);  // E5a data validity: working without guarantee
    }
    return io::RTCMMessage(inav ? io::RTCMMessageType::RTCM_1046 : io::RTCMMessageType::RTCM_1045,
                           payload);
}

}  // namespace

TEST_F(RTCMProcessorTest, DecodesGalileo1046InavEphemeris) {
    const GalileoEphemerisRaw raw;
    const auto frame = buildRtcmFrame(buildGalileoEphemerisMessage(raw, true));
    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    ASSERT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1046);

    NavigationData nav_data;
    ASSERT_TRUE(processor.decodeNavigationData(decoded_messages.front(), nav_data));
    const SatelliteId sat(GNSSSystem::Galileo, raw.prn);
    const auto records = nav_data.getEphemeris(sat);
    ASSERT_EQ(records.size(), 1U);
    const Ephemeris& eph = records.front();
    constexpr double kPi = 3.14159265358979323846;
    EXPECT_EQ(eph.iode, 77U);
    EXPECT_EQ(eph.week, 2275U);
    EXPECT_DOUBLE_EQ(eph.toe.tow, 352800.0);
    EXPECT_EQ(eph.toe.week, 2275);
    EXPECT_DOUBLE_EQ(eph.toc.tow, 352800.0);
    EXPECT_DOUBLE_EQ(eph.toes, 352800.0);
    EXPECT_NEAR(eph.sv_accuracy, 3.12, 1e-12);
    EXPECT_DOUBLE_EQ(eph.af0, -123456789.0 * std::ldexp(1.0, -34));
    EXPECT_DOUBLE_EQ(eph.af1, -1234.0 * std::ldexp(1.0, -46));
    EXPECT_DOUBLE_EQ(eph.sqrt_a, 2852062985.0 * std::ldexp(1.0, -19));
    EXPECT_DOUBLE_EQ(eph.e, 2000000.0 * std::ldexp(1.0, -33));
    EXPECT_DOUBLE_EQ(eph.m0, 1000000000.0 * std::ldexp(1.0, -31) * kPi);
    EXPECT_DOUBLE_EQ(eph.omega_dot, -1200.0 * std::ldexp(1.0, -43) * kPi);
    EXPECT_DOUBLE_EQ(eph.tgd, -5.0 * std::ldexp(1.0, -32));
    EXPECT_DOUBLE_EQ(eph.tgd_secondary, -7.0 * std::ldexp(1.0, -32));
    EXPECT_EQ(eph.data_source_code, 513);
    EXPECT_EQ(eph.health, 2U);  // E1-B OSHS=1 -> bit 1

    Vector3d pos;
    Vector3d vel;
    double clock_bias = 0.0;
    double clock_drift = 0.0;
    ASSERT_TRUE(eph.calculateSatelliteState(eph.toe + 60.0, pos, vel, clock_bias, clock_drift));
    EXPECT_NEAR(pos.norm(), 29.6e6, 0.2e6);
}

TEST_F(RTCMProcessorTest, DecodesGalileo1045FnavEphemeris) {
    const GalileoEphemerisRaw raw;
    const auto frame = buildRtcmFrame(buildGalileoEphemerisMessage(raw, false));
    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    NavigationData nav_data;
    ASSERT_TRUE(processor.decodeNavigationData(decoded_messages.front(), nav_data));
    const auto records = nav_data.getEphemeris(SatelliteId(GNSSSystem::Galileo, raw.prn));
    ASSERT_EQ(records.size(), 1U);
    EXPECT_EQ(records.front().data_source_code, 258);
    EXPECT_EQ(records.front().health, 8U);  // E5a DVS bit
    EXPECT_DOUBLE_EQ(records.front().tgd_secondary, 0.0);
}

TEST_F(RTCMProcessorTest, Galileo1045And1046RoundTripThroughRinexNavigation) {
    // RTCM 1046 (I/NAV) and 1045 (F/NAV) -> RINEX navigation -> reader keeps
    // the data-source word and SISA in metres (index 107 = 3.12 m).
    GalileoEphemerisRaw inav_raw;
    GalileoEphemerisRaw fnav_raw;
    fnav_raw.prn = 12;
    NavigationData decoded;
    for (const bool inav : {true, false}) {
        const auto frame =
            buildRtcmFrame(buildGalileoEphemerisMessage(inav ? inav_raw : fnav_raw, inav));
        const auto messages = processor.decode(frame.data(), frame.size());
        ASSERT_EQ(messages.size(), 1U);
        ASSERT_TRUE(processor.decodeNavigationData(messages.front(), decoded));
    }

    const auto path =
        std::filesystem::temp_directory_path() / "libgnss_test_rtcm_gal_roundtrip.nav";
    io::RINEXWriter writer;
    io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = io::RINEXReader::FileType::NAVIGATION;
    header.satellite_system = "M";
    ASSERT_TRUE(writer.createNavigationFile(path.string(), header));
    for (const uint8_t prn : {inav_raw.prn, fnav_raw.prn}) {
        for (const auto& eph : decoded.getEphemeris(SatelliteId(GNSSSystem::Galileo, prn))) {
            ASSERT_TRUE(writer.writeNavigationMessage(eph));
        }
    }
    writer.close();

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(path.string()));
    NavigationData nav;
    ASSERT_TRUE(reader.readNavigationData(nav));
    reader.close();
    std::filesystem::remove(path);

    for (const auto& [prn, source] :
         {std::pair<uint8_t, int>{inav_raw.prn, 513}, std::pair<uint8_t, int>{fnav_raw.prn, 258}}) {
        const SatelliteId sat(GNSSSystem::Galileo, prn);
        const auto original = decoded.getEphemeris(sat);
        const auto records = nav.getEphemeris(sat);
        ASSERT_EQ(records.size(), 1U);
        ASSERT_EQ(original.size(), 1U);
        EXPECT_EQ(records.front().data_source_code, source);
        EXPECT_NEAR(records.front().sv_accuracy, 3.12, 1e-12);
        EXPECT_EQ(records.front().iode, original.front().iode);
        EXPECT_NEAR(records.front().sqrt_a, original.front().sqrt_a, 1e-9);
        EXPECT_NEAR(records.front().af0, original.front().af0, 1e-15);
    }
}

TEST(RTCMEphemerisMergeTest, MergesStreamEphemeridesOnceAndSelectsInav) {
    const GalileoEphemerisRaw raw;
    std::vector<uint8_t> stream;
    for (const bool inav : {true, false, true}) {  // duplicate I/NAV record
        const auto frame = buildRtcmFrame(buildGalileoEphemerisMessage(raw, inav));
        stream.insert(stream.end(), frame.begin(), frame.end());
    }
    const auto path = std::filesystem::temp_directory_path() / "libgnss_test_rtcm_gal_eph.rtcm3";
    {
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(stream.data()),
                     static_cast<std::streamsize>(stream.size()));
    }

    NavigationData nav;
    EXPECT_EQ(io::mergeRTCMEphemerides(path.string(), nav), 2U);
    EXPECT_EQ(io::mergeRTCMEphemerides(path.string(), nav), 0U);
    const SatelliteId sat(GNSSSystem::Galileo, raw.prn);
    ASSERT_EQ(nav.getEphemeris(sat).size(), 2U);

    const GNSSTime query(2275, 352900.0);
    nav.setGalileoEphemerisSource(NavigationData::GalileoEphemerisSource::INavOnly);
    for (int pass = 0; pass < 2; ++pass) {
        const Ephemeris* selected = pass == 0 ? nav.getEphemeris(sat, query)
                                              : nav.getEphemeris(sat, query, 77);
        ASSERT_NE(selected, nullptr);
        EXPECT_EQ(selected->data_source_code, 513);
    }
    std::filesystem::remove(path);
}

TEST(RTCMSsrSignalIdTest, MapsRtcmSsrWireIdsToSignals) {
    int rank = -1;
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::GPS, 0, &rank), SignalType::GPS_L1CA);
    EXPECT_EQ(rank, 0);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::GPS, 8), SignalType::GPS_L2C);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::GPS, 10, &rank), SignalType::GPS_L2P);
    EXPECT_EQ(rank, 1);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::GPS, 11, &rank), SignalType::GPS_L2P);
    EXPECT_EQ(rank, 0);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::GPS, 15), SignalType::GPS_L5);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::Galileo, 2), SignalType::GAL_E1);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::Galileo, 6), SignalType::GAL_E5A);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::Galileo, 9), SignalType::GAL_E5B);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::Galileo, 16), SignalType::GAL_E6);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::Galileo, 12), SignalType::SIGNAL_TYPE_COUNT);
    EXPECT_EQ(signalTypeFromRtcmSsrSignalId(GNSSSystem::BeiDou, 0), SignalType::SIGNAL_TYPE_COUNT);
}

TEST_F(RTCMProcessorTest, DecodesGps1060CombinedSsrCorrections) {
    const auto frame = buildGpsSsrCombined1060Frame();

    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1060);

    std::vector<io::RTCMSSRCorrection> corrections;
    ASSERT_TRUE(processor.decodeSSRCorrections(decoded_messages.front(), corrections));
    ASSERT_EQ(corrections.size(), 1U);

    const auto& correction = corrections.front();
    EXPECT_EQ(correction.satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(correction.satellite.prn, 7);
    EXPECT_DOUBLE_EQ(correction.update_interval_seconds, 5.0);
    EXPECT_EQ(correction.issue_of_data, 7);
    EXPECT_EQ(correction.provider_id, 21);
    EXPECT_EQ(correction.solution_id, 3);
    EXPECT_TRUE(correction.reference_datum);
    EXPECT_EQ(correction.iode, 12);
    EXPECT_TRUE(correction.has_orbit);
    EXPECT_TRUE(correction.has_clock);
    EXPECT_NEAR(correction.time.tow, 345600.0, 1e-6);
    EXPECT_NEAR(correction.orbit_delta_rac_m.x(), 0.1234, 1e-6);
    EXPECT_NEAR(correction.orbit_delta_rac_m.y(), -0.0800, 1e-6);
    EXPECT_NEAR(correction.orbit_delta_rac_m.z(), 0.1200, 1e-6);
    EXPECT_NEAR(correction.orbit_rate_rac_mps.x(), 0.000123, 1e-9);
    EXPECT_NEAR(correction.orbit_rate_rac_mps.y(), -0.000456, 1e-9);
    EXPECT_NEAR(correction.orbit_rate_rac_mps.z(), 0.000228, 1e-9);
    EXPECT_NEAR(correction.clock_delta_poly.x(), -0.2500, 1e-6);
    EXPECT_NEAR(correction.clock_delta_poly.y(), 0.001234, 1e-9);
    EXPECT_NEAR(correction.clock_delta_poly.z(), -0.000004, 1e-10);
}

TEST_F(RTCMProcessorTest, DecodesGlonass1066CombinedSsrCorrections) {
    const auto frame = buildGlonassSsrCombined1066Frame();

    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1066);

    std::vector<io::RTCMSSRCorrection> corrections;
    ASSERT_TRUE(processor.decodeSSRCorrections(decoded_messages.front(), corrections));
    ASSERT_EQ(corrections.size(), 1U);

    const auto& correction = corrections.front();
    EXPECT_EQ(correction.satellite.system, GNSSSystem::GLONASS);
    EXPECT_EQ(correction.satellite.prn, 8);
    EXPECT_DOUBLE_EQ(correction.update_interval_seconds, 30.0);
    EXPECT_EQ(correction.issue_of_data, 9);
    EXPECT_EQ(correction.provider_id, 8);
    EXPECT_EQ(correction.solution_id, 1);
    EXPECT_FALSE(correction.reference_datum);
    EXPECT_EQ(correction.iode, 44);
    EXPECT_TRUE(correction.has_orbit);
    EXPECT_TRUE(correction.has_clock);
    EXPECT_NEAR(std::fmod(correction.time.tow + 86400.0, 86400.0), 32428.0, 1.0);
    EXPECT_NEAR(correction.orbit_delta_rac_m.x(), -0.0800, 1e-6);
    EXPECT_NEAR(correction.orbit_delta_rac_m.y(), 0.0700, 1e-6);
    EXPECT_NEAR(correction.orbit_delta_rac_m.z(), -0.1000, 1e-6);
    EXPECT_NEAR(correction.orbit_rate_rac_mps.x(), -0.000090, 1e-9);
    EXPECT_NEAR(correction.orbit_rate_rac_mps.y(), 0.000320, 1e-9);
    EXPECT_NEAR(correction.orbit_rate_rac_mps.z(), -0.000160, 1e-9);
    EXPECT_NEAR(correction.clock_delta_poly.x(), 0.1800, 1e-6);
    EXPECT_NEAR(correction.clock_delta_poly.y(), -0.000220, 1e-9);
    EXPECT_NEAR(correction.clock_delta_poly.z(), 0.000003, 1e-10);
}

TEST_F(RTCMProcessorTest, DecodesGps1059CodeBiasCorrections) {
    const auto frame = buildGpsSsrCodeBias1059Frame();

    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1059);

    std::vector<io::RTCMSSRCorrection> corrections;
    ASSERT_TRUE(processor.decodeSSRCorrections(decoded_messages.front(), corrections));
    ASSERT_EQ(corrections.size(), 1U);

    const auto& correction = corrections.front();
    EXPECT_EQ(correction.satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(correction.satellite.prn, 7);
    EXPECT_TRUE(correction.has_code_bias);
    ASSERT_EQ(correction.code_bias_m.size(), 1U);
    EXPECT_NEAR(correction.code_bias_m.at(3), -0.12, 1e-9);
}

TEST_F(RTCMProcessorTest, DecodesGps1061UraCorrections) {
    const auto frame = buildGpsSsrUra1061Frame();

    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1061);

    std::vector<io::RTCMSSRCorrection> corrections;
    ASSERT_TRUE(processor.decodeSSRCorrections(decoded_messages.front(), corrections));
    ASSERT_EQ(corrections.size(), 1U);

    const auto& correction = corrections.front();
    EXPECT_EQ(correction.satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(correction.satellite.prn, 7);
    EXPECT_TRUE(correction.has_ura);
    EXPECT_EQ(correction.ura_index, 9);
    EXPECT_NEAR(correction.ura_sigma_m, 0.00275, 1e-12);
}

TEST_F(RTCMProcessorTest, DecodesGps1062HighRateClockCorrections) {
    const auto frame = buildGpsSsrHighRateClock1062Frame();

    const auto decoded_messages = processor.decode(frame.data(), frame.size());
    ASSERT_EQ(decoded_messages.size(), 1U);
    EXPECT_EQ(decoded_messages.front().type, io::RTCMMessageType::RTCM_1062);

    std::vector<io::RTCMSSRCorrection> corrections;
    ASSERT_TRUE(processor.decodeSSRCorrections(decoded_messages.front(), corrections));
    ASSERT_EQ(corrections.size(), 1U);

    const auto& correction = corrections.front();
    EXPECT_EQ(correction.satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(correction.satellite.prn, 7);
    EXPECT_TRUE(correction.has_high_rate_clock);
    EXPECT_NEAR(correction.high_rate_clock_m, 0.0250, 1e-9);
}

TEST(RTCMReaderTest, ReadsMessagesFromFile) {
    const auto frame = buildRtcm1005(10.0, 20.0, 30.0);
    const std::filesystem::path temp_path =
        std::filesystem::temp_directory_path() / "libgnss_test_rtcm1005.bin";

    std::ofstream output(temp_path, std::ios::binary);
    ASSERT_TRUE(output.is_open());
    output.put(static_cast<char>(0x01));
    output.write(reinterpret_cast<const char*>(frame.data()), static_cast<std::streamsize>(frame.size()));
    output.close();

    io::RTCMReader reader;
    ASSERT_TRUE(reader.open(temp_path.string()));
    io::RTCMMessage message;
    ASSERT_TRUE(reader.readMessage(message));
    EXPECT_EQ(message.type, io::RTCMMessageType::RTCM_1005);
    EXPECT_FALSE(reader.readMessage(message));

    std::filesystem::remove(temp_path);
}

#ifndef _WIN32
TEST(RTCMReaderTest, ReadsMessagesFromNetworkViaNtrip) {
    const auto frame = buildRtcm1005(11.0, 22.0, 33.0);
    LocalNtripServer server(frame);
    ASSERT_TRUE(server.isReady());

    io::RTCMReader reader;
    ASSERT_TRUE(reader.open("ntrip://127.0.0.1:" + std::to_string(server.port()) + "/MOUNT1"));

    io::RTCMMessage message;
    ASSERT_TRUE(reader.readMessage(message));
    EXPECT_EQ(message.type, io::RTCMMessageType::RTCM_1005);
    const auto stats = reader.getStats();
    EXPECT_EQ(stats.valid_messages, 1U);
}

TEST(RTCMReaderTest, ReadsMessagesFromRawTcpSocket) {
    const auto frame = buildRtcm1005(11.5, 23.0, 34.5);
    LocalTcpServer server(frame);
    ASSERT_TRUE(server.isReady());

    io::RTCMReader reader;
    ASSERT_TRUE(reader.open("tcp://127.0.0.1:" + std::to_string(server.port())));

    io::RTCMMessage message;
    ASSERT_TRUE(reader.readMessage(message));
    EXPECT_EQ(message.type, io::RTCMMessageType::RTCM_1005);
    const auto stats = reader.getStats();
    EXPECT_EQ(stats.valid_messages, 1U);
}

TEST(RTCMReaderTest, ReadsMessagesFromSerialDevice) {
    const auto frame = buildRtcm1005(12.0, 24.0, 36.0);
    PseudoTerminal pty = openPseudoTerminal();
    ASSERT_GE(pty.master_fd, 0);
    ASSERT_FALSE(pty.slave_path.empty());

    std::thread writer([master_fd = pty.master_fd, frame]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const ssize_t written =
            ::write(master_fd, frame.data(), static_cast<ssize_t>(frame.size()));
        EXPECT_EQ(written, static_cast<ssize_t>(frame.size()));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ::close(master_fd);
    });

    io::RTCMReader reader;
    const bool opened = reader.open("serial://" + pty.slave_path + "?baud=115200");

    io::RTCMMessage message;
    const bool read_ok = opened && reader.readMessage(message);
    writer.join();

    ASSERT_TRUE(opened);
    ASSERT_TRUE(read_ok);
    EXPECT_EQ(message.type, io::RTCMMessageType::RTCM_1005);
    EXPECT_FALSE(reader.readMessage(message));
    const auto stats = reader.getStats();
    EXPECT_EQ(stats.valid_messages, 1U);
}
#endif

TEST(RTCMUtilsTest, ClassifiesMessageFamilies) {
    EXPECT_TRUE(io::rtcm_utils::isStationMessage(io::RTCMMessageType::RTCM_1005));
    EXPECT_TRUE(io::rtcm_utils::isObservationMessage(io::RTCMMessageType::RTCM_1074));
    EXPECT_TRUE(io::rtcm_utils::isEphemerisMessage(io::RTCMMessageType::RTCM_1019));
    EXPECT_TRUE(io::rtcm_utils::isSSRMessage(io::RTCMMessageType::RTCM_1057));
    EXPECT_TRUE(io::rtcm_utils::isSSRMessage(io::RTCMMessageType::RTCM_1059));
    EXPECT_TRUE(io::rtcm_utils::isSSRMessage(io::RTCMMessageType::RTCM_1061));
    EXPECT_TRUE(io::rtcm_utils::isSSRMessage(io::RTCMMessageType::RTCM_1062));
    EXPECT_FALSE(io::rtcm_utils::isObservationMessage(io::RTCMMessageType::RTCM_1005));
}

TEST(RTCMUtilsTest, ReportsNamesAndSystems) {
    EXPECT_EQ(io::rtcm_utils::getMessageTypeName(io::RTCMMessageType::RTCM_1005),
              "Reference Station ARP");
    EXPECT_EQ(io::rtcm_utils::getMessageTypeName(io::RTCMMessageType::RTCM_1060),
              "GPS SSR Combined Orbit/Clock Correction");
    EXPECT_EQ(io::rtcm_utils::getMessageTypeName(io::RTCMMessageType::RTCM_1059),
              "GPS SSR Code Bias");
    EXPECT_EQ(io::rtcm_utils::getMessageTypeName(io::RTCMMessageType::RTCM_1061),
              "GPS SSR URA");
    EXPECT_EQ(io::rtcm_utils::getMessageTypeName(io::RTCMMessageType::RTCM_1062),
              "GPS SSR High-Rate Clock Correction");
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1077),
              GNSSSystem::GPS);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1059),
              GNSSSystem::GPS);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1066),
              GNSSSystem::GLONASS);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1087),
              GNSSSystem::GLONASS);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1097),
              GNSSSystem::Galileo);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1243),
              GNSSSystem::Galileo);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1127),
              GNSSSystem::BeiDou);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1261),
              GNSSSystem::BeiDou);
}

TEST(RTCMUtilsTest, ConvertsGpsTimeToAndFromRtcmMilliseconds) {
    const GNSSTime gps_time(2200, 345678.901);
    const uint32_t rtcm_time = io::rtcm_utils::gpsTimeToRTCMTime(gps_time);
    const GNSSTime round_trip = io::rtcm_utils::rtcmTimeToGPSTime(rtcm_time, 2200);

    EXPECT_EQ(rtcm_time, 345678901U);
    EXPECT_EQ(round_trip.week, 2200);
    EXPECT_NEAR(round_trip.tow, gps_time.tow, 1e-3);
}

// ---------------------------------------------------------------------------
// MSM signal-ID decoding.  The bitstreams below are built by hand (not through
// RTCMProcessor::encodeObservations, which only writes a few GPS/GAL/BDS/GLO
// signal IDs) so the decoder is exercised against the RTCM 10403.3 signal-ID
// tables independently of the encoder.
// ---------------------------------------------------------------------------
namespace {

constexpr double kMsmUnitM = constants::SPEED_OF_LIGHT * 0.001;  // one light-millisecond

struct MsmTestCell {
    int sat;                // MSM satellite ID (1-64)
    int signal_id;          // MSM signal ID (1-32)
    double pr_offset_m;     // pseudorange minus the satellite rough range
    double cp_offset_m;     // carrier range minus the satellite rough range
    bool with_phase = true;
};

// Build an MSM4..MSM7 payload.  Cells are laid out satellite-major, signal-minor
// in the order the wire format requires, whatever the order of `cells`.
io::RTCMMessage buildMsmMessage(uint16_t type_number,
                                int msm_level,  // 4..7
                                uint32_t epoch_field,
                                std::vector<MsmTestCell> cells,
                                int glonass_fcn = 99) {
    std::vector<int> sats;
    std::vector<int> sigs;
    for (const auto& cell : cells) {
        sats.push_back(cell.sat);
        sigs.push_back(cell.signal_id);
    }
    std::sort(sats.begin(), sats.end());
    sats.erase(std::unique(sats.begin(), sats.end()), sats.end());
    std::sort(sigs.begin(), sigs.end());
    sigs.erase(std::unique(sigs.begin(), sigs.end()), sigs.end());

    const bool ext = msm_level >= 6;                       // MSM6/7 high resolution
    const bool rate = msm_level == 5 || msm_level == 7;    // MSM5/7 carry Doppler fields
    const size_t nsat = sats.size();
    const size_t nsig = sigs.size();
    const size_t nmask = nsat * nsig;

    std::map<std::pair<int, int>, const MsmTestCell*> by_key;
    for (const auto& cell : cells) {
        by_key[{cell.sat, cell.signal_id}] = &cell;
    }
    size_t ncell = by_key.size();

    const size_t pr_bits = ext ? 20 : 15;
    const size_t cp_bits = ext ? 24 : 22;
    const size_t lock_bits = ext ? 10 : 4;
    const size_t cnr_bits = ext ? 10 : 6;
    const size_t total_bits =
        169 + nmask + nsat * (8 + 10 + (rate ? 4 + 14 : 0)) +
        ncell * (pr_bits + cp_bits + lock_bits + 1 + cnr_bits + (rate ? 15 : 0));
    std::vector<uint8_t> payload((total_bits + 7) / 8, 0);

    int bit = 0;
    const auto put = [&](int len, uint64_t value) {
        setUnsignedBits(payload, bit, len, value);
        bit += len;
    };
    const auto put_signed = [&](int len, int64_t value) {
        setSignedBits(payload, bit, len, value);
        bit += len;
    };

    put(12, type_number);
    put(12, 7);  // reference station id
    put(30, epoch_field);
    put(1, 0);   // multiple message bit
    put(3, 0);   // IODS
    put(7, 0);   // reserved
    put(2, 0);   // clock steering
    put(2, 0);   // external clock
    put(1, 0);   // smoothing indicator
    put(3, 0);   // smoothing interval
    for (int sat = 1; sat <= 64; ++sat) {
        put(1, std::binary_search(sats.begin(), sats.end(), sat) ? 1 : 0);
    }
    for (int sig = 1; sig <= 32; ++sig) {
        put(1, std::binary_search(sigs.begin(), sigs.end(), sig) ? 1 : 0);
    }
    for (int sat : sats) {
        for (int sig : sigs) {
            put(1, by_key.count({sat, sig}) ? 1 : 0);
        }
    }

    // Rough range of satellite k: (70 + k) light-milliseconds, exact.
    const auto rough_ms = [](int sat) { return 70 + sat % 20; };
    for (int sat : sats) put(8, static_cast<uint64_t>(rough_ms(sat)));
    if (rate) {
        for (int sat : sats) {
            put(4, glonass_fcn == 99 ? 15U : static_cast<uint64_t>(glonass_fcn + 7));
        }
    }
    for (int sat : sats) { (void)sat; put(10, 0); }
    if (rate) {
        for (size_t i = 0; i < nsat; ++i) put_signed(14, -8192);  // rate unavailable
    }

    const auto for_each_cell = [&](const auto& fn) {
        for (int sat : sats) {
            for (int sig : sigs) {
                const auto it = by_key.find({sat, sig});
                if (it != by_key.end()) fn(*it->second);
            }
        }
    };
    for_each_cell([&](const MsmTestCell& cell) {
        const double scale = ext ? 536870912.0 : 16777216.0;  // 2^29 / 2^24
        put_signed(static_cast<int>(pr_bits),
                   static_cast<int64_t>(std::llround(cell.pr_offset_m / kMsmUnitM * scale)));
    });
    for_each_cell([&](const MsmTestCell& cell) {
        if (!cell.with_phase) {
            put_signed(static_cast<int>(cp_bits), ext ? -8388608 : -2097152);
            return;
        }
        const double scale = ext ? 2147483648.0 : 536870912.0;  // 2^31 / 2^29
        put_signed(static_cast<int>(cp_bits),
                   static_cast<int64_t>(std::llround(cell.cp_offset_m / kMsmUnitM * scale)));
    });
    for_each_cell([&](const MsmTestCell&) { put(static_cast<int>(lock_bits), ext ? 100 : 12); });
    for_each_cell([&](const MsmTestCell&) { put(1, 0); });
    for_each_cell([&](const MsmTestCell&) {
        put(static_cast<int>(cnr_bits), ext ? 45 * 16 : 45);
    });
    if (rate) {
        for_each_cell([&](const MsmTestCell&) { put_signed(15, -16384); });
    }
    EXPECT_EQ(static_cast<size_t>(bit), total_bits);

    io::RTCMMessage message;
    message.type = static_cast<io::RTCMMessageType>(type_number);
    message.length = static_cast<uint16_t>(payload.size());
    message.data = std::move(payload);
    message.valid = true;
    return message;
}

double msmRoughRangeM(int sat) { return static_cast<double>(70 + sat % 20) * kMsmUnitM; }

struct MsmIdExpectation {
    GNSSSystem system;
    uint16_t msm7_type;
    int signal_id;
    SignalType signal;  // SIGNAL_TYPE_COUNT = must be skipped
    const char* code;   // expected RINEX band+attribute when decoded
    double frequency_hz;
};

// Independent restatement of RTCM 10403.3 / RTKLIB msm_sig_* with the RINEX
// reader's code -> SignalType mapping.
const std::vector<MsmIdExpectation>& msmIdExpectations() {
    constexpr auto kNone = SignalType::SIGNAL_TYPE_COUNT;
    static const std::vector<MsmIdExpectation> table = {
        // GPS (1077)
        {GNSSSystem::GPS, 1077, 2, SignalType::GPS_L1CA, "1C", 1575.42e6},
        {GNSSSystem::GPS, 1077, 3, SignalType::GPS_L1P, "1P", 1575.42e6},
        {GNSSSystem::GPS, 1077, 4, SignalType::GPS_L1CA, "1W", 1575.42e6},
        {GNSSSystem::GPS, 1077, 8, SignalType::GPS_L2C, "2C", 1227.60e6},
        {GNSSSystem::GPS, 1077, 9, SignalType::GPS_L2P, "2P", 1227.60e6},
        {GNSSSystem::GPS, 1077, 10, SignalType::GPS_L2C, "2W", 1227.60e6},
        {GNSSSystem::GPS, 1077, 15, SignalType::GPS_L2C, "2S", 1227.60e6},
        {GNSSSystem::GPS, 1077, 16, SignalType::GPS_L2C, "2L", 1227.60e6},
        {GNSSSystem::GPS, 1077, 17, SignalType::GPS_L2C, "2X", 1227.60e6},
        {GNSSSystem::GPS, 1077, 22, SignalType::GPS_L5, "5I", 1176.45e6},
        {GNSSSystem::GPS, 1077, 23, SignalType::GPS_L5, "5Q", 1176.45e6},
        {GNSSSystem::GPS, 1077, 24, SignalType::GPS_L5, "5X", 1176.45e6},
        {GNSSSystem::GPS, 1077, 30, SignalType::GPS_L1CA, "1S", 1575.42e6},
        {GNSSSystem::GPS, 1077, 31, SignalType::GPS_L1CA, "1L", 1575.42e6},
        {GNSSSystem::GPS, 1077, 32, SignalType::GPS_L1CA, "1X", 1575.42e6},
        // GLONASS (1087); frequency is the channel-0 value
        {GNSSSystem::GLONASS, 1087, 2, SignalType::GLO_L1CA, "1C", 0.0},
        {GNSSSystem::GLONASS, 1087, 3, SignalType::GLO_L1P, "1P", 0.0},
        {GNSSSystem::GLONASS, 1087, 8, SignalType::GLO_L2CA, "2C", 0.0},
        {GNSSSystem::GLONASS, 1087, 9, SignalType::GLO_L2P, "2P", 0.0},
        // Galileo (1097)
        {GNSSSystem::Galileo, 1097, 2, SignalType::GAL_E1, "1C", 1575.42e6},
        {GNSSSystem::Galileo, 1097, 3, SignalType::GAL_E1, "1A", 1575.42e6},
        {GNSSSystem::Galileo, 1097, 4, SignalType::GAL_E1, "1B", 1575.42e6},
        {GNSSSystem::Galileo, 1097, 5, SignalType::GAL_E1, "1X", 1575.42e6},
        {GNSSSystem::Galileo, 1097, 6, SignalType::GAL_E1, "1Z", 1575.42e6},
        {GNSSSystem::Galileo, 1097, 8, SignalType::GAL_E6, "6C", 1278.75e6},
        {GNSSSystem::Galileo, 1097, 9, SignalType::GAL_E6, "6A", 1278.75e6},
        {GNSSSystem::Galileo, 1097, 10, SignalType::GAL_E6, "6B", 1278.75e6},
        {GNSSSystem::Galileo, 1097, 11, SignalType::GAL_E6, "6X", 1278.75e6},
        {GNSSSystem::Galileo, 1097, 12, SignalType::GAL_E6, "6Z", 1278.75e6},
        {GNSSSystem::Galileo, 1097, 14, SignalType::GAL_E5B, "7I", 1207.14e6},
        {GNSSSystem::Galileo, 1097, 15, SignalType::GAL_E5B, "7Q", 1207.14e6},
        {GNSSSystem::Galileo, 1097, 16, SignalType::GAL_E5B, "7X", 1207.14e6},
        {GNSSSystem::Galileo, 1097, 18, kNone, "8I", 0.0},  // E5 AltBOC: no SignalType
        {GNSSSystem::Galileo, 1097, 19, kNone, "8Q", 0.0},
        {GNSSSystem::Galileo, 1097, 20, kNone, "8X", 0.0},
        {GNSSSystem::Galileo, 1097, 22, SignalType::GAL_E5A, "5I", 1176.45e6},
        {GNSSSystem::Galileo, 1097, 23, SignalType::GAL_E5A, "5Q", 1176.45e6},
        {GNSSSystem::Galileo, 1097, 24, SignalType::GAL_E5A, "5X", 1176.45e6},
        // QZSS (1117)
        {GNSSSystem::QZSS, 1117, 2, SignalType::QZS_L1CA, "1C", 1575.42e6},
        {GNSSSystem::QZSS, 1117, 9, kNone, "6S", 0.0},  // L6: no SignalType
        {GNSSSystem::QZSS, 1117, 10, kNone, "6L", 0.0},
        {GNSSSystem::QZSS, 1117, 11, kNone, "6X", 0.0},
        {GNSSSystem::QZSS, 1117, 15, SignalType::QZS_L2C, "2S", 1227.60e6},
        {GNSSSystem::QZSS, 1117, 16, SignalType::QZS_L2C, "2L", 1227.60e6},
        {GNSSSystem::QZSS, 1117, 17, SignalType::QZS_L2C, "2X", 1227.60e6},
        {GNSSSystem::QZSS, 1117, 22, SignalType::QZS_L5, "5I", 1176.45e6},
        {GNSSSystem::QZSS, 1117, 23, SignalType::QZS_L5, "5Q", 1176.45e6},
        {GNSSSystem::QZSS, 1117, 24, SignalType::QZS_L5, "5X", 1176.45e6},
        {GNSSSystem::QZSS, 1117, 30, SignalType::QZS_L1CA, "1S", 1575.42e6},
        {GNSSSystem::QZSS, 1117, 31, SignalType::QZS_L1CA, "1L", 1575.42e6},
        {GNSSSystem::QZSS, 1117, 32, SignalType::QZS_L1CA, "1X", 1575.42e6},
        // BeiDou (1127)
        {GNSSSystem::BeiDou, 1127, 2, SignalType::BDS_B1I, "2I", 1561.098e6},
        {GNSSSystem::BeiDou, 1127, 3, SignalType::BDS_B1I, "2Q", 1561.098e6},
        {GNSSSystem::BeiDou, 1127, 4, SignalType::BDS_B1I, "2X", 1561.098e6},
        {GNSSSystem::BeiDou, 1127, 8, SignalType::BDS_B3I, "6I", 1268.52e6},
        {GNSSSystem::BeiDou, 1127, 9, SignalType::BDS_B3I, "6Q", 1268.52e6},
        {GNSSSystem::BeiDou, 1127, 10, SignalType::BDS_B3I, "6X", 1268.52e6},
        {GNSSSystem::BeiDou, 1127, 14, SignalType::BDS_B2I, "7I", 1207.14e6},
        {GNSSSystem::BeiDou, 1127, 15, SignalType::BDS_B2I, "7Q", 1207.14e6},
        {GNSSSystem::BeiDou, 1127, 16, SignalType::BDS_B2I, "7X", 1207.14e6},
        {GNSSSystem::BeiDou, 1127, 22, SignalType::BDS_B2A, "5D", 1176.45e6},
        {GNSSSystem::BeiDou, 1127, 23, SignalType::BDS_B2A, "5P", 1176.45e6},
        {GNSSSystem::BeiDou, 1127, 24, SignalType::BDS_B2A, "5X", 1176.45e6},
        {GNSSSystem::BeiDou, 1127, 25, kNone, "7D", 0.0},  // B2b: no SignalType
        {GNSSSystem::BeiDou, 1127, 30, SignalType::BDS_B1C, "1D", 1575.42e6},
        {GNSSSystem::BeiDou, 1127, 31, SignalType::BDS_B1C, "1P", 1575.42e6},
        {GNSSSystem::BeiDou, 1127, 32, SignalType::BDS_B1C, "1X", 1575.42e6},
        // NavIC (1137): RINEX reader maps band 5 to GPS_L5
        {GNSSSystem::NavIC, 1137, 22, SignalType::GPS_L5, "5A", 1176.45e6},
    };
    return table;
}

// One satellite, one signal: decode through a framed message so the message
// number dispatch is part of the test.  Returns the decoded epoch.
bool decodeSingleCell(io::RTCMProcessor& processor,
                      uint16_t type_number,
                      int msm_level,
                      const MsmTestCell& cell,
                      ObservationData& decoded,
                      int glonass_fcn = 99) {
    const io::RTCMMessage built = buildMsmMessage(
        type_number, msm_level, 100000000U, {cell}, glonass_fcn);
    const std::vector<uint8_t> frame = buildRtcmFrame(built);
    const auto messages = processor.decode(frame.data(), frame.size());
    EXPECT_EQ(messages.size(), 1U);
    if (messages.size() != 1U) return false;
    EXPECT_EQ(static_cast<uint16_t>(messages[0].type), type_number);
    EXPECT_TRUE(io::rtcm_utils::isObservationMessage(messages[0].type));
    return processor.decodeObservationData(messages[0], decoded);
}

}  // namespace

TEST_F(RTCMProcessorTest, MsmSignalIdsMapToRinexConsistentSignalFrequencyAndCode) {
    for (const auto& expected : msmIdExpectations()) {
        SCOPED_TRACE(::testing::Message()
                     << "system=" << static_cast<int>(expected.system)
                     << " signal_id=" << expected.signal_id << " code=" << expected.code);
        const bool glonass = expected.system == GNSSSystem::GLONASS;
        const int fcn = glonass ? -3 : 99;
        const MsmTestCell cell{3, expected.signal_id, 12.5, 7.25};
        ObservationData decoded;
        const bool ok =
            decodeSingleCell(processor, expected.msm7_type, 7, cell, decoded, fcn);
        if (expected.signal == SignalType::SIGNAL_TYPE_COUNT) {
            EXPECT_FALSE(ok);
            EXPECT_TRUE(decoded.observations.empty());
            continue;
        }
        ASSERT_TRUE(ok);
        ASSERT_EQ(decoded.observations.size(), 1U);
        const Observation& obs = decoded.observations.front();
        EXPECT_EQ(obs.satellite.system, expected.system);
        EXPECT_EQ(obs.satellite.prn, 3);
        EXPECT_EQ(obs.signal, expected.signal);

        // The RINEX reader maps the same code to the same SignalType.
        SignalType rinex_signal = SignalType::SIGNAL_TYPE_COUNT;
        const std::string rinex_pseudorange = std::string("C") + expected.code;
        ASSERT_TRUE(signal_policy::trySignalForObservationType(
            expected.system, rinex_pseudorange, rinex_signal));
        // Legacy GPS 1P / 2P (IDs 3, 9) intentionally keep GPS_L1P / GPS_L2P
        // (what the MSM encoder writes); GLONASS P codes are distinct types in
        // both readers.
        const bool legacy_p = expected.system == GNSSSystem::GPS &&
                              (expected.signal_id == 3 || expected.signal_id == 9);
        if (!legacy_p) {
            EXPECT_EQ(rinex_signal, expected.signal);
        }

        EXPECT_EQ(obs.pseudorange_observation_type, rinex_pseudorange);
        EXPECT_EQ(obs.carrier_phase_observation_type, std::string("L") + expected.code);
        const double rough = msmRoughRangeM(3);
        EXPECT_NEAR(obs.pseudorange, rough + 12.5, 1e-3);
        ASSERT_TRUE(obs.has_carrier_phase);

        double frequency_hz = expected.frequency_hz;
        if (glonass) {
            const bool l1 = expected.signal == SignalType::GLO_L1CA ||
                            expected.signal == SignalType::GLO_L1P;
            frequency_hz = l1 ? constants::GLO_L1_BASE_FREQ + fcn * constants::GLO_L1_STEP_FREQ
                              : constants::GLO_L2_BASE_FREQ + fcn * constants::GLO_L2_STEP_FREQ;
        } else {
            EXPECT_DOUBLE_EQ(signalFrequencyHz(obs.signal), expected.frequency_hz);
        }
        EXPECT_NEAR(obs.carrier_phase * (constants::SPEED_OF_LIGHT / frequency_hz),
                    rough + 7.25, 2e-3);

        // The complete tracking-code entry mirrors the RINEX v3 reader.
        const Observation* tracking = decoded.getRinexTrackingObservation(
            obs.satellite, expected.code);
        ASSERT_NE(tracking, nullptr);
        EXPECT_EQ(tracking->signal, expected.signal);
    }
}

TEST_F(RTCMProcessorTest, MsmUndefinedSignalIdsAreStillSkipped) {
    // Every ID that is not in the expectation table above must be dropped, for
    // every constellation the decoder handles, without disturbing the other
    // cells of the same satellite.
    struct SystemCase { GNSSSystem system; uint16_t type; int good_id; SignalType good_signal; };
    const std::vector<SystemCase> systems = {
        {GNSSSystem::GPS, 1077, 2, SignalType::GPS_L1CA},
        {GNSSSystem::GLONASS, 1087, 2, SignalType::GLO_L1CA},
        {GNSSSystem::Galileo, 1097, 2, SignalType::GAL_E1},
        {GNSSSystem::QZSS, 1117, 2, SignalType::QZS_L1CA},
        {GNSSSystem::BeiDou, 1127, 2, SignalType::BDS_B1I},
        {GNSSSystem::NavIC, 1137, 22, SignalType::GPS_L5},
    };
    for (const auto& sys : systems) {
        std::set<int> defined;
        for (const auto& e : msmIdExpectations()) {
            if (e.system == sys.system) defined.insert(e.signal_id);
        }
        for (int id = 1; id <= 32; ++id) {
            if (defined.count(id) != 0) continue;
            SCOPED_TRACE(::testing::Message() << "system=" << static_cast<int>(sys.system)
                                              << " undefined id=" << id);
            // Alone: nothing decodes.
            ObservationData alone;
            EXPECT_FALSE(decodeSingleCell(processor, sys.type, 7, {4, id, 5.0, 2.0}, alone, -3));
            EXPECT_TRUE(alone.observations.empty());

            // Next to a supported signal: only the supported one survives and
            // the cell bookkeeping stays aligned (the good cell keeps its data).
            const io::RTCMMessage mixed = buildMsmMessage(
                sys.type, 7, 100000000U,
                {{4, id, 5.0, 2.0}, {4, sys.good_id, 9.5, 3.5}}, -3);
            ObservationData decoded;
            ASSERT_TRUE(processor.decodeObservationData(mixed, decoded));
            ASSERT_EQ(decoded.observations.size(), 1U);
            EXPECT_EQ(decoded.observations[0].signal, sys.good_signal);
            EXPECT_NEAR(decoded.observations[0].pseudorange, msmRoughRangeM(4) + 9.5, 1e-3);
        }
    }
}

TEST_F(RTCMProcessorTest, MsmSameSignalTypeCellsResolveByTrackingPriority) {
    struct PriorityCase {
        const char* name;
        GNSSSystem system;
        uint16_t type;
        std::vector<int> ids;       // all present on one satellite
        SignalType signal;
        const char* winner;         // expected surviving RINEX code
    };
    const std::vector<PriorityCase> cases = {
        // GPS L2: C > W > L > S > X (RTKLIB demo5 "CPYWMNDLSX"), GPS L1: C > W > S > L > X
        // ("CPYWMNSLX"); the wire ID order is not the tie-break.
        {"gps2 C beats W/L/X", GNSSSystem::GPS, 1077, {8, 10, 16, 17}, SignalType::GPS_L2C, "2C"},
        {"gps2 W beats L/S/X", GNSSSystem::GPS, 1077, {10, 15, 16, 17}, SignalType::GPS_L2C, "2W"},
        {"gps2 L beats S/X", GNSSSystem::GPS, 1077, {15, 16, 17}, SignalType::GPS_L2C, "2L"},
        {"gps2 S beats X", GNSSSystem::GPS, 1077, {15, 17}, SignalType::GPS_L2C, "2S"},
        {"gps1 C beats W/S/L/X", GNSSSystem::GPS, 1077, {2, 4, 30, 31, 32}, SignalType::GPS_L1CA, "1C"},
        {"gps1 W beats S/L/X", GNSSSystem::GPS, 1077, {4, 30, 31, 32}, SignalType::GPS_L1CA, "1W"},
        {"gps1 S beats L/X", GNSSSystem::GPS, 1077, {30, 31, 32}, SignalType::GPS_L1CA, "1S"},
        {"gps5 I beats Q/X", GNSSSystem::GPS, 1077, {22, 23, 24}, SignalType::GPS_L5, "5I"},
        {"gps5 Q beats X", GNSSSystem::GPS, 1077, {23, 24}, SignalType::GPS_L5, "5Q"},
        {"gal1 C beats B/X", GNSSSystem::Galileo, 1097, {2, 4, 5}, SignalType::GAL_E1, "1C"},
        {"gal1 B beats X", GNSSSystem::Galileo, 1097, {4, 5}, SignalType::GAL_E1, "1B"},
        {"gal5a X beats I/Q", GNSSSystem::Galileo, 1097, {22, 23, 24}, SignalType::GAL_E5A, "5X"},
        {"gal5b X beats I/Q", GNSSSystem::Galileo, 1097, {14, 15, 16}, SignalType::GAL_E5B, "7X"},
        {"gal6 A beats C/X", GNSSSystem::Galileo, 1097, {8, 9, 11}, SignalType::GAL_E6, "6A"},
        {"qzs1 C beats L/S/X", GNSSSystem::QZSS, 1117, {2, 30, 31, 32}, SignalType::QZS_L1CA, "1C"},
        {"qzs1 L beats S/X", GNSSSystem::QZSS, 1117, {30, 31, 32}, SignalType::QZS_L1CA, "1L"},
        {"qzs2 L beats S/X", GNSSSystem::QZSS, 1117, {15, 16, 17}, SignalType::QZS_L2C, "2L"},
        {"qzs5 I beats Q/X", GNSSSystem::QZSS, 1117, {22, 23, 24}, SignalType::QZS_L5, "5I"},
        {"bds1i I beats Q/X", GNSSSystem::BeiDou, 1127, {2, 3, 4}, SignalType::BDS_B1I, "2I"},
        {"bds2i I beats Q/X", GNSSSystem::BeiDou, 1127, {14, 15, 16}, SignalType::BDS_B2I, "7I"},
        {"bds3i Q beats X", GNSSSystem::BeiDou, 1127, {9, 10}, SignalType::BDS_B3I, "6Q"},
        {"bds2a D beats P/X", GNSSSystem::BeiDou, 1127, {22, 23, 24}, SignalType::BDS_B2A, "5D"},
        {"bds2a P beats X", GNSSSystem::BeiDou, 1127, {23, 24}, SignalType::BDS_B2A, "5P"},
        // BeiDou B1C follows the shared RINEX/RTKLIB demo5 order "DPXSLZAN"
        // (D > P > X); RTCM used to carry a private "XDP" table that preferred X.
        {"bds1c D beats P/X", GNSSSystem::BeiDou, 1127, {30, 31, 32}, SignalType::BDS_B1C, "1D"},
        {"bds1c P beats X", GNSSSystem::BeiDou, 1127, {31, 32}, SignalType::BDS_B1C, "1P"},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        std::vector<MsmTestCell> cells;
        double offset = 1.0;
        for (int id : c.ids) {
            cells.push_back({6, id, offset, offset + 0.5});
            offset += 1.0;  // distinct data per code so the winner is identifiable
        }
        const io::RTCMMessage message =
            buildMsmMessage(c.type, 7, 100000000U, cells);
        ObservationData decoded;
        ASSERT_TRUE(processor.decodeObservationData(message, decoded));

        // Exactly one observation of the contested SignalType.
        int count = 0;
        const Observation* kept = nullptr;
        for (const auto& obs : decoded.observations) {
            if (obs.signal == c.signal) {
                ++count;
                kept = &obs;
            }
        }
        ASSERT_EQ(count, 1);
        EXPECT_EQ(kept->pseudorange_observation_type, std::string("C") + c.winner);
        EXPECT_EQ(kept->carrier_phase_observation_type, std::string("L") + c.winner);

        // Its data belongs to the winning cell, not to the first/last one.
        double expected_offset = 0.0;
        double idx_offset = 1.0;
        for (int id : c.ids) {
            const std::string code = [&] {
                for (const auto& e : msmIdExpectations()) {
                    if (e.system == c.system && e.signal_id == id) return std::string(e.code);
                }
                return std::string();
            }();
            if (code == c.winner) expected_offset = idx_offset;
            idx_offset += 1.0;
        }
        EXPECT_NEAR(kept->pseudorange, msmRoughRangeM(6) + expected_offset, 1e-3);

        // Every cell stays reachable through the RINEX tracking-code store.
        for (int id : c.ids) {
            for (const auto& e : msmIdExpectations()) {
                if (e.system == c.system && e.signal_id == id) {
                    EXPECT_NE(decoded.getRinexTrackingObservation(kept->satellite, e.code),
                              nullptr)
                        << e.code;
                }
            }
        }
    }
}

namespace {

std::string rinexHeaderLine(std::string content, const std::string& label) {
    if (content.size() < 60) content.append(60 - content.size(), ' ');
    return content + label + "\n";
}

char rinexSystemChar(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return 'G';
        case GNSSSystem::GLONASS: return 'R';
        case GNSSSystem::Galileo: return 'E';
        case GNSSSystem::QZSS: return 'J';
        case GNSSSystem::BeiDou: return 'C';
        case GNSSSystem::NavIC: return 'I';
        default: return '?';
    }
}

// One-epoch, one-satellite RINEX 3.04 file with a pseudorange and a phase for
// each code, declared in the given order.
ObservationData readRinexWithCodes(GNSSSystem system, int prn,
                                   const std::vector<std::string>& codes) {
    std::string text;
    text += rinexHeaderLine("     3.04           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
    text += rinexHeaderLine("unit test", "PGM / RUN BY / DATE");
    text += rinexHeaderLine("TEST", "MARKER NAME");
    text += rinexHeaderLine("  -3957000.0000  3310000.0000  3737000.0000", "APPROX POSITION XYZ");
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%c  %3d", rinexSystemChar(system),
                  static_cast<int>(codes.size() * 2));
    std::string types = buf;
    for (const auto& code : codes) types += " C" + code + " L" + code;
    text += rinexHeaderLine(types, "SYS / # / OBS TYPES");
    text += rinexHeaderLine("  2024     1     1     0     0    0.0000000     GPS",
                            "TIME OF FIRST OBS");
    text += rinexHeaderLine("", "END OF HEADER");
    text += "> 2024 01 01 00 00  0.0000000  0  1\n";
    std::snprintf(buf, sizeof(buf), "%c%02d", rinexSystemChar(system), prn);
    std::string row = buf;
    double k = 1.0;
    for (size_t i = 0; i < codes.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%14.3f  ", 20000000.0 + 1000.0 * k);
        row += buf;
        std::snprintf(buf, sizeof(buf), "%14.3f  ", 100000000.0 + 1000.0 * k);
        row += buf;
        k += 1.0;
    }
    text += row + "\n";

    static int counter = 0;
    const auto path = std::filesystem::temp_directory_path() /
                      ("libgnss_rtcm_prio_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "_" +
                       std::to_string(++counter) + ".obs");
    {
        std::ofstream file(path, std::ios::binary);
        file << text;
    }
    ObservationData epoch;
    io::RINEXReader reader;
    reader.setPreserveAdditionalFrequencyBands(true);
    io::RINEXReader::RINEXHeader header;
    if (reader.open(path.string()) && reader.readHeader(header)) {
        reader.readObservationEpoch(epoch);
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return epoch;
}

}  // namespace

// RTCM MSM and RINEX share one tracking-attribute priority
// (signal_policy::trackingAttributeRank).  For every system/SignalType that
// has several MSM tracking modes, any subset of those codes must resolve to
// the same code through the RTCM decoder and the RINEX reader, whichever order
// the RINEX header declares them in.
TEST_F(RTCMProcessorTest, MsmAndRinexPickTheSameTrackingCodeForEverySystemAndBand) {
    std::map<std::pair<int, int>, std::vector<const MsmIdExpectation*>> groups;
    for (const auto& e : msmIdExpectations()) {
        if (e.signal == SignalType::SIGNAL_TYPE_COUNT) continue;
        groups[{static_cast<int>(e.system), static_cast<int>(e.signal)}].push_back(&e);
    }
    int compared_subsets = 0;
    int contested_groups = 0;
    for (const auto& [key, members] : groups) {
        if (members.size() < 2) continue;
        ++contested_groups;
        const GNSSSystem system = members.front()->system;
        const SignalType signal = members.front()->signal;
        const int n = static_cast<int>(members.size());
        for (unsigned mask = 1; mask < (1U << n); ++mask) {
            if ((mask & (mask - 1U)) == 0U) continue;  // single code: nothing to decide
            std::vector<const MsmIdExpectation*> subset;
            for (int i = 0; i < n; ++i) {
                if (mask & (1U << i)) subset.push_back(members[i]);
            }
            std::string label;
            for (const auto* e : subset) label += std::string(e->code) + " ";
            SCOPED_TRACE(::testing::Message() << "system=" << static_cast<int>(system)
                                              << " signal=" << static_cast<int>(signal)
                                              << " codes=" << label);

            std::vector<MsmTestCell> cells;
            double offset = 1.0;
            for (const auto* e : subset) {
                cells.push_back({6, e->signal_id, offset, offset + 0.5});
                offset += 1.0;
            }
            processor.clear();
            ObservationData rtcm_epoch;
            ASSERT_TRUE(processor.decodeObservationData(
                buildMsmMessage(subset.front()->msm7_type, 7, 100000000U, cells), rtcm_epoch));
            std::string rtcm_pick;
            for (const auto& obs : rtcm_epoch.observations) {
                if (obs.signal == signal) rtcm_pick = obs.pseudorange_observation_type;
            }
            ASSERT_FALSE(rtcm_pick.empty());

            // The shared helper's own verdict: lowest rank.
            int best_rank = 1 << 30;
            std::string expected;
            for (const auto* e : subset) {
                const int rank = signal_policy::trackingAttributeRank(
                    system, e->code[0] - '0', e->code[1]);
                if (rank < best_rank) {
                    best_rank = rank;
                    expected = std::string("C") + e->code;
                }
            }
            EXPECT_EQ(rtcm_pick, expected);

            std::vector<std::string> codes;
            for (const auto* e : subset) codes.push_back(e->code);
            for (int pass = 0; pass < 2; ++pass) {
                if (pass == 1) std::reverse(codes.begin(), codes.end());
                const ObservationData rinex_epoch = readRinexWithCodes(system, 6, codes);
                std::string rinex_pick;
                for (const auto& obs : rinex_epoch.observations) {
                    if (obs.signal == signal) rinex_pick = obs.pseudorange_observation_type;
                }
                EXPECT_EQ(rinex_pick, rtcm_pick) << "header order pass " << pass;
            }
            ++compared_subsets;
        }
    }
    // GPS L1/L2/L5, Galileo E1/E5a/E5b/E6, QZSS L1/L2/L5, BeiDou B1I/B2I/B3I/B2a/B1C.
    EXPECT_GE(contested_groups, 15);
    EXPECT_GT(compared_subsets, 50);
}

TEST_F(RTCMProcessorTest, MsmNeverEmitsTwoObservationsOfOneSignalTypePerSatellite) {
    // A GPS satellite tracked on every GPS civil tracking mode, plus the
    // legacy P-code IDs, on two satellites.
    std::vector<MsmTestCell> cells;
    for (int sat : {5, 9}) {
        for (int id : {2, 3, 4, 8, 9, 10, 15, 16, 17, 22, 23, 24, 30, 31, 32}) {
            cells.push_back({sat, id, 1.0 + id * 0.1, 2.0 + id * 0.1});
        }
    }
    const io::RTCMMessage message = buildMsmMessage(1077, 7, 100000000U, cells);
    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(message, decoded));
    std::set<std::pair<int, int>> seen;
    for (const auto& obs : decoded.observations) {
        EXPECT_TRUE(seen.insert({obs.satellite.prn, static_cast<int>(obs.signal)}).second);
    }
    // L1CA, L1P, L2C, L2P, L5 per satellite.
    EXPECT_EQ(decoded.observations.size(), 10U);
    const auto l2c = findObservation(decoded, 5, SignalType::GPS_L2C);
    ASSERT_TRUE(l2c.has_value());
    EXPECT_EQ(l2c->pseudorange_observation_type, "C2C");
    const auto l2p = findObservation(decoded, 5, SignalType::GPS_L2P);
    ASSERT_TRUE(l2p.has_value());
    EXPECT_EQ(l2p->pseudorange_observation_type, "C2P");
}

TEST_F(RTCMProcessorTest, MsmPriorityBeatsDataCompleteness) {
    // 2C carries only a pseudorange, 2W carries code and phase: the RTKLIB code
    // priority still selects 2C (the library never mixes fields across codes).
    const io::RTCMMessage message = buildMsmMessage(
        1077, 7, 100000000U,
        {MsmTestCell{3, 8, 4.0, 0.0, false}, MsmTestCell{3, 10, 5.0, 3.0, true}});
    ObservationData decoded;
    ASSERT_TRUE(processor.decodeObservationData(message, decoded));
    ASSERT_EQ(decoded.observations.size(), 1U);
    const Observation& obs = decoded.observations.front();
    EXPECT_EQ(obs.signal, SignalType::GPS_L2C);
    EXPECT_EQ(obs.pseudorange_observation_type, "C2C");
    EXPECT_TRUE(obs.carrier_phase_observation_type.empty());
    EXPECT_FALSE(obs.has_carrier_phase);
    ASSERT_NE(decoded.getRinexTrackingObservation(obs.satellite, "2W"), nullptr);
    EXPECT_TRUE(decoded.getRinexTrackingObservation(obs.satellite, "2W")->has_carrier_phase);
}

TEST_F(RTCMProcessorTest, DecodesQzssMsmAcrossResolutionsAndTimeScale) {
    // MSM4 (1114), MSM5 (1115), MSM6 (1116), MSM7 (1117).  QZSS epochs are GPS
    // time; the satellite ID is the RINEX J number (1-10).
    const std::vector<std::pair<uint16_t, int>> messages = {
        {1114, 4}, {1115, 5}, {1116, 6}, {1117, 7}};
    for (const auto& [type, level] : messages) {
        SCOPED_TRACE(type);
        const io::RTCMMessage built = buildMsmMessage(
            type, level, 345600500U,
            {{2, 2, 20.0, 10.0}, {2, 16, 21.0, 11.0}, {2, 23, 22.0, 12.0},
             {3, 2, 30.0, 15.0}});
        const std::vector<uint8_t> frame = buildRtcmFrame(built);
        const auto framed = processor.decode(frame.data(), frame.size());
        ASSERT_EQ(framed.size(), 1U);
        EXPECT_EQ(framed[0].type, static_cast<io::RTCMMessageType>(type));
        EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(framed[0].type), GNSSSystem::QZSS);
        EXPECT_TRUE(io::rtcm_utils::isObservationMessage(framed[0].type));

        ObservationData decoded;
        ASSERT_TRUE(processor.decodeObservationData(framed[0], decoded));
        EXPECT_NEAR(decoded.time.tow, 345600.5, 1e-3);  // no BDT offset
        ASSERT_EQ(decoded.observations.size(), 4U);
        const GNSSSystem qzss = GNSSSystem::QZSS;
        const auto l1 = findObservation(decoded, qzss, 2, SignalType::QZS_L1CA);
        const auto l2 = findObservation(decoded, qzss, 2, SignalType::QZS_L2C);
        const auto l5 = findObservation(decoded, qzss, 2, SignalType::QZS_L5);
        const auto j3 = findObservation(decoded, qzss, 3, SignalType::QZS_L1CA);
        ASSERT_TRUE(l1.has_value());
        ASSERT_TRUE(l2.has_value());
        ASSERT_TRUE(l5.has_value());
        ASSERT_TRUE(j3.has_value());
        const double tol = level >= 6 ? 2e-3 : 0.05;
        EXPECT_NEAR(l1->pseudorange, msmRoughRangeM(2) + 20.0, tol);
        EXPECT_NEAR(l2->pseudorange, msmRoughRangeM(2) + 21.0, tol);
        EXPECT_NEAR(l5->pseudorange, msmRoughRangeM(2) + 22.0, tol);
        EXPECT_NEAR(l1->carrier_phase * constants::GPS_L1_WAVELENGTH, msmRoughRangeM(2) + 10.0, tol);
        EXPECT_NEAR(l2->carrier_phase * constants::GPS_L2_WAVELENGTH, msmRoughRangeM(2) + 11.0, tol);
        EXPECT_NEAR(l5->carrier_phase * constants::GPS_L5_WAVELENGTH, msmRoughRangeM(2) + 12.0, tol);
        EXPECT_EQ(l1->pseudorange_observation_type, "C1C");
        EXPECT_EQ(l2->carrier_phase_observation_type, "L2L");
        EXPECT_EQ(l5->pseudorange_observation_type, "C5Q");
        EXPECT_NEAR(j3->pseudorange, msmRoughRangeM(3) + 30.0, tol);
        EXPECT_NEAR(l1->snr, 45.0, 1e-6);
    }
}

TEST_F(RTCMProcessorTest, DecodesNavicMsmL5AndKeepsOtherSignalsOut) {
    const std::vector<std::pair<uint16_t, int>> messages = {
        {1134, 4}, {1135, 5}, {1136, 6}, {1137, 7}};
    for (const auto& [type, level] : messages) {
        SCOPED_TRACE(type);
        // 5A (ID 22) is decoded; ID 9 (S-band 9A in later amendments) is not defined here.
        const io::RTCMMessage built = buildMsmMessage(
            type, level, 100000250U,
            {{4, 22, 40.0, 20.0}, {4, 9, 41.0, 21.0}, {11, 22, 50.0, 25.0}});
        const std::vector<uint8_t> frame = buildRtcmFrame(built);
        const auto framed = processor.decode(frame.data(), frame.size());
        ASSERT_EQ(framed.size(), 1U);
        EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(framed[0].type), GNSSSystem::NavIC);
        ObservationData decoded;
        ASSERT_TRUE(processor.decodeObservationData(framed[0], decoded));
        ASSERT_EQ(decoded.observations.size(), 2U);
        const auto sat4 = findObservation(decoded, GNSSSystem::NavIC, 4, SignalType::GPS_L5);
        const auto sat11 = findObservation(decoded, GNSSSystem::NavIC, 11, SignalType::GPS_L5);
        ASSERT_TRUE(sat4.has_value());
        ASSERT_TRUE(sat11.has_value());
        const double tol = level >= 6 ? 2e-3 : 0.05;
        EXPECT_NEAR(sat4->pseudorange, msmRoughRangeM(4) + 40.0, tol);
        EXPECT_NEAR(sat11->carrier_phase * constants::GPS_L5_WAVELENGTH,
                    msmRoughRangeM(11) + 25.0, tol);
        EXPECT_EQ(sat4->pseudorange_observation_type, "C5A");
    }
}

TEST(RTCMUtilsTest, ClassifiesQzssAndNavicMsmMessages) {
    for (uint16_t type : {1114, 1115, 1116, 1117, 1134, 1135, 1136, 1137}) {
        EXPECT_TRUE(io::rtcm_utils::isObservationMessage(static_cast<io::RTCMMessageType>(type)))
            << type;
    }
    // MSM1-3 and SBAS MSM remain non-decoded.
    for (uint16_t type : {1111, 1112, 1113, 1131, 1132, 1133, 1101, 1104, 1105, 1106, 1107}) {
        EXPECT_FALSE(io::rtcm_utils::isObservationMessage(static_cast<io::RTCMMessageType>(type)))
            << type;
    }
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1117),
              GNSSSystem::QZSS);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1137),
              GNSSSystem::NavIC);
    EXPECT_EQ(io::rtcm_utils::getMessageTypeName(io::RTCMMessageType::RTCM_1114), "QZSS MSM4");
    EXPECT_EQ(io::rtcm_utils::getMessageTypeName(io::RTCMMessageType::RTCM_1137), "NavIC MSM7");
}
