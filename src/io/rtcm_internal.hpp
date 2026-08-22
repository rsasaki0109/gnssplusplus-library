#pragma once

// Shared file-local helpers for the RTCM implementation TUs; extracted
// from the former monolithic rtcm.cpp anonymous namespace.

#include "../../include/libgnss++/io/rtcm.hpp"
#include "../../include/libgnss++/io/ntrip.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#ifndef _WIN32
#include <netdb.h>
#include <sys/socket.h>
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace libgnss {
namespace io {
namespace rtcm_internal {


constexpr uint8_t kRTCMPreamble = 0xD3;
constexpr double kRTCMStationCoordinateScale = 1e-4;
constexpr uint32_t kMillisecondsPerGPSWeek = 604800000U;
constexpr double kRTCMGpsPrUnitMeters = constants::SPEED_OF_LIGHT * 0.001;
constexpr double kRTCMGpsPseudorangeResolution = 0.02;
constexpr double kRTCMGpsPhaseResolution = 0.0005;
constexpr double kRTCMCn0Resolution = 0.25;
constexpr double kSemiCircleToRadians = 3.14159265358979323846;
constexpr double kPow2Neg5 = 3.12500000000000000000e-02;
constexpr double kPow2Neg10 = 9.76562500000000000000e-04;
constexpr double kPow2Neg11 = 4.88281250000000000000e-04;
constexpr double kPow2Neg19 = 1.90734863281250000000e-06;
constexpr double kPow2Neg20 = 9.53674316406250000000e-07;
constexpr double kPow2Neg24 = 5.96046447753906250000e-08;
constexpr double kPow2Neg29 = 1.86264514923095703125e-09;
constexpr double kPow2Neg30 = 9.31322574615478515625e-10;
constexpr double kPow2Neg31 = 4.65661287307739257812e-10;
constexpr double kPow2Neg33 = 1.16415321826934814453e-10;
constexpr double kPow2Neg40 = 9.09494701772928237915e-13;
constexpr double kPow2Neg43 = 1.13686837721616029739e-13;
constexpr double kPow2Neg55 = 2.77555756156289135106e-17;
constexpr uint16_t kMinimumExpandedGpsWeek = 1560;
constexpr double kSecondsPerDay = 86400.0;
constexpr double kHalfDaySeconds = 43200.0;
constexpr int kDefaultSerialBaud = 115200;
constexpr double kUraMetersTable[16] = {
    2.4,   3.4,   4.85,   6.85,
    9.65,  13.65, 24.0,   48.0,
    96.0,  192.0, 384.0,  768.0,
    1536.0,3072.0,6144.0, 0.0
};
constexpr double kSsrUpdateIntervals[16] = {
    1.0, 2.0, 5.0, 10.0, 15.0, 30.0, 60.0, 120.0,
    240.0, 300.0, 600.0, 900.0, 1800.0, 3600.0, 7200.0, 10800.0
};

// CRC-24Q lookup table from gpsd/crc24q.c
static const uint32_t crc24q_table[256] = {
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

inline uint64_t readUnsignedBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    if (!data || num_bits <= 0 || num_bits > 64 || bit_pos < 0) {
        return 0;
    }
    if (static_cast<size_t>(bit_pos + num_bits) > data_size * 8U) {
        return 0;
    }

    uint64_t value = 0;
    for (int i = 0; i < num_bits; ++i) {
        const int current_bit_pos = bit_pos + i;
        const int byte_index = current_bit_pos / 8;
        const int bit_in_byte = 7 - (current_bit_pos % 8);
        value = (value << 1) | ((data[byte_index] >> bit_in_byte) & 0x01U);
    }
    return value;
}

inline void writeUnsignedBits(std::vector<uint8_t>& data, int bit_pos, int num_bits, uint64_t value) {
    for (int i = 0; i < num_bits; ++i) {
        const int current_bit_pos = bit_pos + num_bits - 1 - i;
        const int byte_index = current_bit_pos / 8;
        const int bit_in_byte = 7 - (current_bit_pos % 8);
        const uint8_t mask = static_cast<uint8_t>(1U << bit_in_byte);
        if ((value >> i) & 0x01U) {
            data[byte_index] |= mask;
        } else {
            data[byte_index] &= static_cast<uint8_t>(~mask);
        }
    }
}

inline void writeSignedBits(std::vector<uint8_t>& data, int bit_pos, int num_bits, int64_t value) {
    const uint64_t masked = static_cast<uint64_t>(value) & ((1ULL << num_bits) - 1ULL);
    writeUnsignedBits(data, bit_pos, num_bits, masked);
}

inline void writeSignMagnitudeBits(std::vector<uint8_t>& data, int bit_pos, int num_bits, int64_t value) {
    if (num_bits <= 1) {
        return;
    }
    writeUnsignedBits(data, bit_pos, 1, value < 0 ? 1U : 0U);
    writeUnsignedBits(
        data,
        bit_pos + 1,
        num_bits - 1,
        static_cast<uint64_t>(std::llabs(value)));
}

inline int64_t signExtend(uint64_t value, int num_bits) {
    if (num_bits <= 0 || num_bits >= 64) {
        return static_cast<int64_t>(value);
    }
    const uint64_t sign_bit = 1ULL << (num_bits - 1);
    if ((value & sign_bit) == 0) {
        return static_cast<int64_t>(value);
    }
    value |= ~((1ULL << num_bits) - 1ULL);
    return static_cast<int64_t>(value);
}

inline int64_t readSignedBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    return signExtend(readUnsignedBits(data, data_size, bit_pos, num_bits), num_bits);
}

inline int64_t readSignMagnitudeBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    if (num_bits <= 1) {
        return 0;
    }
    const int64_t magnitude = static_cast<int64_t>(
        readUnsignedBits(data, data_size, bit_pos + 1, num_bits - 1));
    const bool negative = readUnsignedBits(data, data_size, bit_pos, 1) != 0;
    return negative ? -magnitude : magnitude;
}

inline bool decodeReferenceStationPosition(const RTCMMessage& message, Vector3d& position) {
    const auto type = static_cast<uint16_t>(message.type);
    if (type != 1005 && type != 1006) {
        return false;
    }

    const size_t min_bits = (type == 1005) ? 152U : 168U;
    if (message.data.size() * 8U < min_bits) {
        return false;
    }

    int bit_pos = 12;  // Message type
    bit_pos += 12;     // Reference station ID
    bit_pos += 6;      // ITRF realization year
    bit_pos += 1;      // GPS indicator
    bit_pos += 1;      // GLONASS indicator
    bit_pos += 1;      // Galileo indicator
    bit_pos += 1;      // Reference-station indicator

    const int64_t ecef_x_raw = readSignedBits(message.data.data(), message.data.size(), bit_pos, 38);
    bit_pos += 38;
    bit_pos += 1;  // Single receiver oscillator indicator
    bit_pos += 1;  // Reserved
    const int64_t ecef_y_raw = readSignedBits(message.data.data(), message.data.size(), bit_pos, 38);
    bit_pos += 38;
    bit_pos += 2;  // Quarter cycle indicator
    const int64_t ecef_z_raw = readSignedBits(message.data.data(), message.data.size(), bit_pos, 38);

    position = Vector3d(
        static_cast<double>(ecef_x_raw) * kRTCMStationCoordinateScale,
        static_cast<double>(ecef_y_raw) * kRTCMStationCoordinateScale,
        static_cast<double>(ecef_z_raw) * kRTCMStationCoordinateScale);
    return true;
}

inline int lockIndicatorFromObservation(const Observation& obs) {
    return obs.loss_of_lock ? 0 : 127;
}

inline int cnrUnitsFromObservation(const Observation& obs) {
    const double snr = (obs.snr > 0.0) ? obs.snr : static_cast<double>(obs.signal_strength);
    return static_cast<int>(std::llround(std::clamp(snr / kRTCMCn0Resolution, 0.0, 255.0)));
}

inline bool isSupportedGpsL1Signal(SignalType signal) {
    return signal == SignalType::GPS_L1CA || signal == SignalType::GPS_L1P;
}

inline bool isSupportedGpsL2Signal(SignalType signal) {
    return signal == SignalType::GPS_L2C || signal == SignalType::GPS_L2P;
}

inline bool isSupportedGpsMsmSignal(SignalType signal) {
    return isSupportedGpsL1Signal(signal) || isSupportedGpsL2Signal(signal);
}

inline bool isSupportedGlonassMsmSignal(SignalType signal) {
    return signal == SignalType::GLO_L1CA || signal == SignalType::GLO_L1P ||
           signal == SignalType::GLO_L2CA || signal == SignalType::GLO_L2P;
}

inline bool isFixedFrequencyMsm4MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1074 ||
           message_type == RTCMMessageType::RTCM_1094 ||
           message_type == RTCMMessageType::RTCM_1124;
}

inline bool isFixedFrequencyMsm5MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1075 ||
           message_type == RTCMMessageType::RTCM_1095 ||
           message_type == RTCMMessageType::RTCM_1125;
}

inline bool isFixedFrequencyMsm6MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1076 ||
           message_type == RTCMMessageType::RTCM_1096 ||
           message_type == RTCMMessageType::RTCM_1126;
}

inline bool isFixedFrequencyMsm7MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1077 ||
           message_type == RTCMMessageType::RTCM_1097 ||
           message_type == RTCMMessageType::RTCM_1127;
}

inline bool isFixedFrequencyMsmMessageType(RTCMMessageType message_type) {
    return isFixedFrequencyMsm4MessageType(message_type) ||
           isFixedFrequencyMsm5MessageType(message_type) ||
           isFixedFrequencyMsm6MessageType(message_type) ||
           isFixedFrequencyMsm7MessageType(message_type);
}

inline bool isGlonassMsm4MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1084;
}

inline bool isGlonassMsm5MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1085;
}

inline bool isGlonassMsm6MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1086;
}

inline bool isGlonassMsm7MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1087;
}

inline bool isGlonassMsmMessageType(RTCMMessageType message_type) {
    return isGlonassMsm4MessageType(message_type) ||
           isGlonassMsm5MessageType(message_type) ||
           isGlonassMsm6MessageType(message_type) ||
           isGlonassMsm7MessageType(message_type);
}

inline bool isMsm5MessageType(RTCMMessageType message_type) {
    return isFixedFrequencyMsm5MessageType(message_type) ||
           isGlonassMsm5MessageType(message_type);
}

inline bool isMsm6MessageType(RTCMMessageType message_type) {
    return isFixedFrequencyMsm6MessageType(message_type) ||
           isGlonassMsm6MessageType(message_type);
}

inline bool isMsm7MessageType(RTCMMessageType message_type) {
    return isFixedFrequencyMsm7MessageType(message_type) ||
           isGlonassMsm7MessageType(message_type);
}

inline bool isSsrOrbitMessageType(RTCMMessageType message_type) {
    switch (message_type) {
        case RTCMMessageType::RTCM_1057:
        case RTCMMessageType::RTCM_1063:
        case RTCMMessageType::RTCM_1240:
        case RTCMMessageType::RTCM_1246:
        case RTCMMessageType::RTCM_1258:
            return true;
        default:
            return false;
    }
}

inline bool isSsrClockMessageType(RTCMMessageType message_type) {
    switch (message_type) {
        case RTCMMessageType::RTCM_1058:
        case RTCMMessageType::RTCM_1064:
        case RTCMMessageType::RTCM_1241:
        case RTCMMessageType::RTCM_1247:
        case RTCMMessageType::RTCM_1259:
            return true;
        default:
            return false;
    }
}

inline bool isSsrCodeBiasMessageType(RTCMMessageType message_type) {
    switch (message_type) {
        case RTCMMessageType::RTCM_1059:
        case RTCMMessageType::RTCM_1065:
        case RTCMMessageType::RTCM_1242:
        case RTCMMessageType::RTCM_1248:
        case RTCMMessageType::RTCM_1260:
            return true;
        default:
            return false;
    }
}

inline bool isSsrCombinedMessageType(RTCMMessageType message_type) {
    switch (message_type) {
        case RTCMMessageType::RTCM_1060:
        case RTCMMessageType::RTCM_1066:
        case RTCMMessageType::RTCM_1243:
        case RTCMMessageType::RTCM_1249:
        case RTCMMessageType::RTCM_1261:
            return true;
        default:
            return false;
    }
}

inline bool isSsrUraMessageType(RTCMMessageType message_type) {
    switch (message_type) {
        case RTCMMessageType::RTCM_1061:
        case RTCMMessageType::RTCM_1067:
        case RTCMMessageType::RTCM_1244:
        case RTCMMessageType::RTCM_1250:
        case RTCMMessageType::RTCM_1262:
            return true;
        default:
            return false;
    }
}

inline bool isSsrHighRateClockMessageType(RTCMMessageType message_type) {
    switch (message_type) {
        case RTCMMessageType::RTCM_1062:
        case RTCMMessageType::RTCM_1068:
        case RTCMMessageType::RTCM_1245:
        case RTCMMessageType::RTCM_1251:
        case RTCMMessageType::RTCM_1263:
            return true;
        default:
            return false;
    }
}

inline bool isSupportedSsrMessageType(RTCMMessageType message_type) {
    return isSsrOrbitMessageType(message_type) ||
           isSsrClockMessageType(message_type) ||
           isSsrCodeBiasMessageType(message_type) ||
           isSsrCombinedMessageType(message_type) ||
           isSsrUraMessageType(message_type) ||
           isSsrHighRateClockMessageType(message_type);
}

struct SsrSystemDescriptor {
    GNSSSystem system = GNSSSystem::UNKNOWN;
    int prn_bits = 0;
    int iode_bits = 0;
    int iodcrc_bits = 0;
    int nsat_bits = 6;
    int prn_offset = 0;
};

inline SsrSystemDescriptor ssrDescriptorForMessageType(RTCMMessageType message_type) {
    switch (message_type) {
        case RTCMMessageType::RTCM_1057:
        case RTCMMessageType::RTCM_1058:
        case RTCMMessageType::RTCM_1059:
        case RTCMMessageType::RTCM_1060:
        case RTCMMessageType::RTCM_1061:
        case RTCMMessageType::RTCM_1062:
            return {GNSSSystem::GPS, 6, 8, 0, 6, 0};
        case RTCMMessageType::RTCM_1063:
        case RTCMMessageType::RTCM_1064:
        case RTCMMessageType::RTCM_1065:
        case RTCMMessageType::RTCM_1066:
        case RTCMMessageType::RTCM_1067:
        case RTCMMessageType::RTCM_1068:
            return {GNSSSystem::GLONASS, 5, 8, 0, 6, 0};
        case RTCMMessageType::RTCM_1240:
        case RTCMMessageType::RTCM_1241:
        case RTCMMessageType::RTCM_1242:
        case RTCMMessageType::RTCM_1243:
        case RTCMMessageType::RTCM_1244:
        case RTCMMessageType::RTCM_1245:
            return {GNSSSystem::Galileo, 6, 10, 0, 6, 0};
        case RTCMMessageType::RTCM_1246:
        case RTCMMessageType::RTCM_1247:
        case RTCMMessageType::RTCM_1248:
        case RTCMMessageType::RTCM_1249:
        case RTCMMessageType::RTCM_1250:
        case RTCMMessageType::RTCM_1251:
            return {GNSSSystem::QZSS, 4, 8, 0, 4, 0};
        case RTCMMessageType::RTCM_1258:
        case RTCMMessageType::RTCM_1259:
        case RTCMMessageType::RTCM_1260:
        case RTCMMessageType::RTCM_1261:
        case RTCMMessageType::RTCM_1262:
        case RTCMMessageType::RTCM_1263:
            return {GNSSSystem::BeiDou, 6, 10, 24, 6, 1};
        default:
            return {};
    }
}

struct SsrHeader {
    GNSSTime time;
    double update_interval_seconds = 0.0;
    int sync = 0;
    int issue_of_data = 0;
    int provider_id = 0;
    int solution_id = 0;
    int satellite_count = 0;
    int refd = 0;
    int bit_pos = 0;
};

inline GNSSSystem fixedFrequencyMsmSystem(RTCMMessageType message_type) {
    switch (message_type) {
        case RTCMMessageType::RTCM_1074:
        case RTCMMessageType::RTCM_1075:
        case RTCMMessageType::RTCM_1076:
        case RTCMMessageType::RTCM_1077:
            return GNSSSystem::GPS;
        case RTCMMessageType::RTCM_1094:
        case RTCMMessageType::RTCM_1095:
        case RTCMMessageType::RTCM_1096:
        case RTCMMessageType::RTCM_1097:
            return GNSSSystem::Galileo;
        case RTCMMessageType::RTCM_1124:
        case RTCMMessageType::RTCM_1125:
        case RTCMMessageType::RTCM_1126:
        case RTCMMessageType::RTCM_1127:
            return GNSSSystem::BeiDou;
        default: return GNSSSystem::UNKNOWN;
    }
}

inline bool isSupportedFixedFrequencyMsmSignal(GNSSSystem system, SignalType signal) {
    switch (system) {
        case GNSSSystem::GPS:
            return isSupportedGpsMsmSignal(signal);
        case GNSSSystem::Galileo:
            return signal == SignalType::GAL_E1 ||
                   signal == SignalType::GAL_E5A ||
                   signal == SignalType::GAL_E5B ||
                   signal == SignalType::GAL_E6;
        case GNSSSystem::BeiDou:
            return signal == SignalType::BDS_B1I ||
                   signal == SignalType::BDS_B2I ||
                   signal == SignalType::BDS_B3I;
        default:
            return false;
    }
}

inline uint8_t glonassMsmSignalId(SignalType signal) {
    switch (signal) {
        case SignalType::GLO_L1CA: return 2U; // 1C
        case SignalType::GLO_L1P: return 3U;  // 1P
        case SignalType::GLO_L2CA: return 8U; // 2C
        case SignalType::GLO_L2P: return 9U;  // 2P
        default: return 0U;
    }
}

inline SignalType decodeGpsL1Signal(uint64_t code_indicator) {
    return code_indicator == 0 ? SignalType::GPS_L1CA : SignalType::GPS_L1P;
}

inline SignalType decodeGpsL2Signal(uint64_t code_indicator) {
    return code_indicator == 0 ? SignalType::GPS_L2C : SignalType::GPS_L2P;
}

inline uint8_t gpsMsmSignalId(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return 2U;  // 1C
        case SignalType::GPS_L1P: return 3U;   // 1P
        case SignalType::GPS_L2C: return 8U;   // 2C
        case SignalType::GPS_L2P: return 9U;   // 2P
        default: return 0U;
    }
}

inline uint8_t fixedFrequencyMsmSignalId(GNSSSystem system, SignalType signal) {
    switch (system) {
        case GNSSSystem::GPS:
            return gpsMsmSignalId(signal);
        case GNSSSystem::Galileo:
            switch (signal) {
                case SignalType::GAL_E1: return 2U;   // 1C
                case SignalType::GAL_E5A: return 22U; // 5I
                case SignalType::GAL_E5B: return 14U; // 7I
                case SignalType::GAL_E6: return 8U;   // 6C
                default: return 0U;
            }
        case GNSSSystem::BeiDou:
            switch (signal) {
                case SignalType::BDS_B1I: return 2U;  // 1I
                case SignalType::BDS_B3I: return 8U;  // 6I
                case SignalType::BDS_B2I: return 14U; // 7I
                default: return 0U;
            }
        default:
            return 0U;
    }
}

inline SignalType decodeGpsMsmSignal(uint8_t signal_id) {
    switch (signal_id) {
        case 2: return SignalType::GPS_L1CA;
        case 3: return SignalType::GPS_L1P;
        case 8: return SignalType::GPS_L2C;
        case 9: return SignalType::GPS_L2P;
        default: return SignalType::SIGNAL_TYPE_COUNT;
    }
}

inline SignalType decodeFixedFrequencyMsmSignal(GNSSSystem system, uint8_t signal_id) {
    switch (system) {
        case GNSSSystem::GPS:
            return decodeGpsMsmSignal(signal_id);
        case GNSSSystem::Galileo:
            switch (signal_id) {
                case 2: return SignalType::GAL_E1;
                case 22: return SignalType::GAL_E5A;
                case 14: return SignalType::GAL_E5B;
                case 8: return SignalType::GAL_E6;
                default: return SignalType::SIGNAL_TYPE_COUNT;
            }
        case GNSSSystem::BeiDou:
            switch (signal_id) {
                case 2: return SignalType::BDS_B1I;
                case 8: return SignalType::BDS_B3I;
                case 14: return SignalType::BDS_B2I;
                default: return SignalType::SIGNAL_TYPE_COUNT;
            }
        default:
            return SignalType::SIGNAL_TYPE_COUNT;
    }
}

inline SignalType decodeGlonassMsmSignal(uint8_t signal_id) {
    switch (signal_id) {
        case 2: return SignalType::GLO_L1CA;
        case 3: return SignalType::GLO_L1P;
        case 8: return SignalType::GLO_L2CA;
        case 9: return SignalType::GLO_L2P;
        default: return SignalType::SIGNAL_TYPE_COUNT;
    }
}

inline double gpsSignalWavelength(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GPS_L1P:
            return constants::GPS_L1_WAVELENGTH;
        case SignalType::GPS_L2C:
        case SignalType::GPS_L2P:
            return constants::GPS_L2_WAVELENGTH;
        default:
            return 0.0;
    }
}

inline double glonassSignalWavelength(SignalType signal, int frequency_channel) {
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

inline double signalWavelength(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GPS_L1P:
        case SignalType::QZS_L1CA:
            return constants::GPS_L1_WAVELENGTH;
        case SignalType::GPS_L2C:
        case SignalType::GPS_L2P:
        case SignalType::QZS_L2C:
            return constants::GPS_L2_WAVELENGTH;
        case SignalType::GPS_L5:
        case SignalType::QZS_L5:
            return constants::GPS_L5_WAVELENGTH;
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
            return 0.0;
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
        case SignalType::BDS_B1C:
            return constants::BDS_B1C_WAVELENGTH;
        case SignalType::BDS_B2A:
            return constants::BDS_B2A_WAVELENGTH;
        default:
            return 0.0;
    }
}

inline double observationRangeMeters(const Observation& obs) {
    if (obs.has_pseudorange && std::isfinite(obs.pseudorange)) {
        return obs.pseudorange;
    }
    if (obs.has_carrier_phase) {
        const double wavelength = signalWavelength(obs.signal);
        if (wavelength > 0.0 && std::isfinite(obs.carrier_phase)) {
            return obs.carrier_phase * wavelength;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

inline double observationRangeRateMetersPerSecond(const Observation& obs) {
    if (!obs.has_doppler || !std::isfinite(obs.doppler)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double wavelength = signalWavelength(obs.signal);
    if (wavelength <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return -obs.doppler * wavelength;
}

inline double glonassObservationRangeRateMetersPerSecond(const Observation& obs, int frequency_channel) {
    if (!obs.has_doppler || !std::isfinite(obs.doppler)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double wavelength = glonassSignalWavelength(obs.signal, frequency_channel);
    if (wavelength <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return -obs.doppler * wavelength;
}

inline uint8_t glonassMsmExtendedInfo(int frequency_channel) {
    return (frequency_channel >= -7 && frequency_channel <= 6)
               ? static_cast<uint8_t>(frequency_channel + 7)
               : 15U;
}

inline bool decodeGlonassMsmExtendedInfo(uint8_t info, int& frequency_channel) {
    if (info <= 13U) {
        frequency_channel = static_cast<int>(info) - 7;
        return true;
    }
    return false;
}

inline std::string resolveSerialPath(const std::string& source) {
    constexpr const char* kPrefix = "serial://";
    if (source.rfind(kPrefix, 0) != 0) {
        return source;
    }

    std::string path = source.substr(std::char_traits<char>::length(kPrefix));
    const size_t query_pos = path.find('?');
    if (query_pos != std::string::npos) {
        path.resize(query_pos);
    }
    if (!path.empty() && path.front() != '/') {
        return path;
    }
    while (path.size() > 1 && path[0] == '/' && path[1] == '/') {
        path.erase(path.begin());
    }
    return path;
}

inline int parseSerialBaud(const std::string& source) {
    constexpr const char* kQuery = "?baud=";
    const size_t query_pos = source.find(kQuery);
    if (query_pos == std::string::npos) {
        return kDefaultSerialBaud;
    }
    const std::string baud_text = source.substr(query_pos + std::char_traits<char>::length(kQuery));
    if (baud_text.empty()) {
        return kDefaultSerialBaud;
    }
    return std::stoi(baud_text);
}

struct TcpEndpoint {
    std::string host;
    std::string port;
};

inline bool isTcpSource(const std::string& source) {
    return source.rfind("tcp://", 0) == 0;
}

inline TcpEndpoint parseTcpEndpoint(const std::string& source) {
    constexpr const char* kPrefix = "tcp://";
    if (source.rfind(kPrefix, 0) != 0) {
        throw std::invalid_argument("TCP source must start with tcp://");
    }

    const std::string authority = source.substr(std::char_traits<char>::length(kPrefix));
    const size_t colon_pos = authority.rfind(':');
    if (colon_pos == std::string::npos || colon_pos == 0 || colon_pos + 1 >= authority.size()) {
        throw std::invalid_argument("TCP source must be tcp://host:port");
    }

    TcpEndpoint endpoint;
    endpoint.host = authority.substr(0, colon_pos);
    endpoint.port = authority.substr(colon_pos + 1);
    return endpoint;
}

#ifndef _WIN32
inline speed_t baudRateConstant(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            throw std::invalid_argument("unsupported serial baud rate");
    }
}

inline bool configureSerialPort(int fd, int baud) {
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        return false;
    }
    cfmakeraw(&tio);
    const speed_t speed = baudRateConstant(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &tio) == 0;
}

inline int connectTcpSocket(const std::string& source) {
    const auto endpoint = parseTcpEndpoint(source);

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* result = nullptr;
    if (::getaddrinfo(endpoint.host.c_str(), endpoint.port.c_str(), &hints, &result) != 0) {
        return -1;
    }

    int fd = -1;
    for (addrinfo* address = result; address != nullptr; address = address->ai_next) {
        fd = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (::connect(fd, address->ai_addr, address->ai_addrlen) == 0) {
            break;
        }
        ::close(fd);
        fd = -1;
    }

    ::freeaddrinfo(result);
    return fd;
}
#endif

inline uint32_t fixedFrequencyMsmEpochMs(RTCMMessageType message_type, const GNSSTime& time) {
    if (message_type == RTCMMessageType::RTCM_1124 ||
        message_type == RTCMMessageType::RTCM_1125 ||
        message_type == RTCMMessageType::RTCM_1126 ||
        message_type == RTCMMessageType::RTCM_1127) {
        return rtcm_utils::gpsTimeToRTCMTime(time - 14.0);
    }
    return rtcm_utils::gpsTimeToRTCMTime(time);
}

inline GNSSTime fixedFrequencyMsmEpochToTime(RTCMMessageType message_type, uint32_t epoch_ms) {
    GNSSTime time = rtcm_utils::rtcmTimeToGPSTime(epoch_ms, 0);
    if (message_type == RTCMMessageType::RTCM_1124 ||
        message_type == RTCMMessageType::RTCM_1125 ||
        message_type == RTCMMessageType::RTCM_1126 ||
        message_type == RTCMMessageType::RTCM_1127) {
        time.tow += 14.0;
        while (time.tow < 0.0) {
            time.tow += constants::SECONDS_PER_WEEK;
            --time.week;
        }
        while (time.tow >= constants::SECONDS_PER_WEEK) {
            time.tow -= constants::SECONDS_PER_WEEK;
            ++time.week;
        }
    }
    return time;
}

inline int msmLockIndicatorFromObservation(const Observation& obs) {
    return obs.loss_of_lock ? 0 : 15;
}

inline uint16_t msmExtendedLockIndicatorFromObservation(const Observation& obs) {
    return obs.loss_of_lock ? 0U : 1023U;
}

inline uint8_t msmCn0UnitsFromObservation(const Observation& obs) {
    const double snr = (obs.snr > 0.0) ? obs.snr : static_cast<double>(obs.signal_strength);
    return static_cast<uint8_t>(std::llround(std::clamp(snr, 0.0, 63.0)));
}

inline uint16_t msmExtendedCn0UnitsFromObservation(const Observation& obs) {
    const double snr = (obs.snr > 0.0) ? obs.snr : static_cast<double>(obs.signal_strength);
    return static_cast<uint16_t>(std::llround(std::clamp(snr / 0.0625, 0.0, 1023.0)));
}

inline int leapSecondsForDate(int year, int month, int day) {
    struct LeapEntry { int year; int month; int day; int leap_seconds; };
    static constexpr LeapEntry kLeapTable[] = {
        {1981, 7, 1, 1},  {1982, 7, 1, 2},  {1983, 7, 1, 3},  {1985, 7, 1, 4},
        {1988, 1, 1, 5},  {1990, 1, 1, 6},  {1991, 1, 1, 7},  {1992, 7, 1, 8},
        {1993, 7, 1, 9},  {1994, 7, 1, 10}, {1996, 1, 1, 11}, {1997, 7, 1, 12},
        {1999, 1, 1, 13}, {2006, 1, 1, 14}, {2009, 1, 1, 15}, {2012, 7, 1, 16},
        {2015, 7, 1, 17}, {2017, 1, 1, 18},
    };

    int leap_seconds = 0;
    for (const auto& entry : kLeapTable) {
        if (year > entry.year ||
            (year == entry.year &&
             (month > entry.month || (month == entry.month && day >= entry.day)))) {
            leap_seconds = entry.leap_seconds;
        }
    }
    return leap_seconds;
}

inline std::tm toUtcTm(const std::chrono::system_clock::time_point& tp) {
    const std::time_t raw_time = std::chrono::system_clock::to_time_t(tp);
    std::tm utc_tm{};
#if defined(_WIN32)
    gmtime_s(&utc_tm, &raw_time);
#else
    gmtime_r(&raw_time, &utc_tm);
#endif
    return utc_tm;
}

inline GNSSTime normalizeWeekTow(int week, double tow) {
    while (tow < 0.0) {
        tow += constants::SECONDS_PER_WEEK;
        --week;
    }
    while (tow >= constants::SECONDS_PER_WEEK) {
        tow -= constants::SECONDS_PER_WEEK;
        ++week;
    }
    return GNSSTime(week, tow);
}

inline int leapSecondsForApproxTime(const GNSSTime& time) {
    const std::tm utc_tm = toUtcTm(time.toSystemTime());
    return leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday);
}

inline GNSSTime currentGpstApprox() {
    const auto now = std::chrono::system_clock::now();
    const GNSSTime utc_like = GNSSTime::fromSystemTime(now);
    const std::tm utc_tm = toUtcTm(now);
    return normalizeWeekTow(
        utc_like.week,
        utc_like.tow +
            static_cast<double>(
                leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday)));
}

inline GNSSTime gpstToUtcApprox(const GNSSTime& gps_time) {
    return normalizeWeekTow(
        gps_time.week,
        gps_time.tow - static_cast<double>(leapSecondsForApproxTime(gps_time)));
}

inline GNSSTime utcToGpstApprox(const GNSSTime& utc_time) {
    const std::tm utc_tm = toUtcTm(utc_time.toSystemTime());
    return normalizeWeekTow(
        utc_time.week,
        utc_time.tow +
            static_cast<double>(
                leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday)));
}

inline double secondsOfDay(double tow) {
    double tod = std::fmod(tow, kSecondsPerDay);
    if (tod < 0.0) {
        tod += kSecondsPerDay;
    }
    return tod;
}

inline GNSSTime alignUtcTimeOfDay(double tod_utc, const GNSSTime& reference_utc) {
    const double ref_tod = secondsOfDay(reference_utc.tow);
    if (tod_utc < ref_tod - kHalfDaySeconds) {
        tod_utc += kSecondsPerDay;
    } else if (tod_utc > ref_tod + kHalfDaySeconds) {
        tod_utc -= kSecondsPerDay;
    }
    const double base_tow = reference_utc.tow - ref_tod;
    return normalizeWeekTow(reference_utc.week, base_tow + tod_utc);
}

inline uint32_t gpstToGlonassMsmEpoch(const GNSSTime& gps_time) {
    GNSSTime glot = gpstToUtcApprox(gps_time);
    glot = normalizeWeekTow(glot.week, glot.tow + 10800.0);
    const uint32_t dow = static_cast<uint32_t>(std::floor(glot.tow / kSecondsPerDay)) % 7U;
    const double tod = glot.tow - static_cast<double>(dow) * kSecondsPerDay;
    const uint32_t tod_ms = static_cast<uint32_t>(std::llround(tod * 1000.0)) & ((1U << 27) - 1U);
    return (dow << 27) | tod_ms;
}

inline GNSSTime glonassMsmEpochToGpst(uint32_t epoch) {
    const uint32_t dow = epoch >> 27;
    const double tod = static_cast<double>(epoch & ((1U << 27) - 1U)) * 1e-3;
    const GNSSTime current = currentGpstApprox();
    GNSSTime glot(current.week, static_cast<double>(dow) * kSecondsPerDay + tod);
    const double diff = glot - current;
    if (diff < -302400.0) {
        glot = glot + constants::SECONDS_PER_WEEK;
    } else if (diff > 302400.0) {
        glot = glot - constants::SECONDS_PER_WEEK;
    }
    return utcToGpstApprox(normalizeWeekTow(glot.week, glot.tow - 10800.0));
}

inline GNSSTime alignGpsSsrEpoch(double tow_seconds) {
    GNSSTime aligned(currentGpstApprox().week, tow_seconds);
    const GNSSTime current = currentGpstApprox();
    const double diff = aligned - current;
    if (diff < -302400.0) {
        aligned = aligned + constants::SECONDS_PER_WEEK;
    } else if (diff > 302400.0) {
        aligned = aligned - constants::SECONDS_PER_WEEK;
    }
    return normalizeWeekTow(aligned.week, aligned.tow);
}

inline GNSSTime alignGlonassSsrEpoch(double tod_glonass_seconds) {
    GNSSTime current_glonass = gpstToUtcApprox(currentGpstApprox());
    current_glonass = normalizeWeekTow(current_glonass.week, current_glonass.tow + 10800.0);
    const GNSSTime aligned_glonass = alignUtcTimeOfDay(tod_glonass_seconds, current_glonass);
    return utcToGpstApprox(normalizeWeekTow(aligned_glonass.week, aligned_glonass.tow - 10800.0));
}

inline bool decodeSsrOrbitHeader(const RTCMMessage& message,
                          const SsrSystemDescriptor& descriptor,
                          SsrHeader& header) {
    if (descriptor.system == GNSSSystem::UNKNOWN) {
        return false;
    }

    int bit_pos = 12;
    if (descriptor.system == GNSSSystem::GLONASS) {
        if (message.data.size() * 8U < 65U) {
            return false;
        }
        const double tod_utc_seconds = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 17));
        bit_pos += 17;
        header.time = alignGlonassSsrEpoch(tod_utc_seconds);
    } else {
        if (message.data.size() * 8U < 64U) {
            return false;
        }
        const double tow_seconds = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 20));
        bit_pos += 20;
        header.time = alignGpsSsrEpoch(tow_seconds);
    }

    const uint8_t update_index = static_cast<uint8_t>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
    bit_pos += 4;
    header.update_interval_seconds = kSsrUpdateIntervals[update_index];
    header.sync = static_cast<int>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1));
    bit_pos += 1;
    header.refd = static_cast<int>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1));
    bit_pos += 1;
    header.issue_of_data = static_cast<int>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
    bit_pos += 4;
    header.provider_id = static_cast<int>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 16));
    bit_pos += 16;
    header.solution_id = static_cast<int>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
    bit_pos += 4;
    header.satellite_count = static_cast<int>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, descriptor.nsat_bits));
    bit_pos += descriptor.nsat_bits;
    header.bit_pos = bit_pos;
    return true;
}

inline bool decodeSsrClockHeader(const RTCMMessage& message,
                          const SsrSystemDescriptor& descriptor,
                          SsrHeader& header) {
    if (descriptor.system == GNSSSystem::UNKNOWN) {
        return false;
    }

    int bit_pos = 12;
    if (descriptor.system == GNSSSystem::GLONASS) {
        if (message.data.size() * 8U < 64U) {
            return false;
        }
        const double tod_utc_seconds = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 17));
        bit_pos += 17;
        header.time = alignGlonassSsrEpoch(tod_utc_seconds);
    } else {
        if (message.data.size() * 8U < 63U) {
            return false;
        }
        const double tow_seconds = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 20));
        bit_pos += 20;
        header.time = alignGpsSsrEpoch(tow_seconds);
    }

    const uint8_t update_index = static_cast<uint8_t>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
    bit_pos += 4;
    header.update_interval_seconds = kSsrUpdateIntervals[update_index];
    header.sync = static_cast<int>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1));
    bit_pos += 1;
    header.issue_of_data = static_cast<int>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
    bit_pos += 4;
    header.provider_id = static_cast<int>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 16));
    bit_pos += 16;
    header.solution_id = static_cast<int>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
    bit_pos += 4;
    header.satellite_count = static_cast<int>(
        readUnsignedBits(message.data.data(), message.data.size(), bit_pos, descriptor.nsat_bits));
    bit_pos += descriptor.nsat_bits;
    header.bit_pos = bit_pos;
    return true;
}

inline uint8_t decodeSsrPrn(const SsrSystemDescriptor& descriptor, uint64_t raw_prn) {
    return static_cast<uint8_t>(raw_prn + static_cast<uint64_t>(descriptor.prn_offset));
}

inline int adjustGpsWeek(uint16_t week_mod_1024) {
    int current_week = currentGpstApprox().week;
    if (current_week < kMinimumExpandedGpsWeek) {
        current_week = kMinimumExpandedGpsWeek;
    }
    return static_cast<int>(week_mod_1024) +
           ((current_week - static_cast<int>(week_mod_1024) + 512) / 1024) * 1024;
}

inline uint8_t uraIndexFromMeters(double ura_meters) {
    if (!std::isfinite(ura_meters) || ura_meters <= 0.0) {
        return 15U;
    }
    for (uint8_t i = 0; i < 15; ++i) {
        if (kUraMetersTable[i] >= ura_meters) {
            return i;
        }
    }
    return 15U;
}

inline double uraMetersFromIndex(uint8_t ura_index) {
    return (ura_index < 15U) ? kUraMetersTable[ura_index] : 32767.0;
}

inline double uraMetersFromSsrIndex(uint8_t ura_index) {
    if (ura_index == 0U) {
        return 0.15;
    }
    if (ura_index >= 63U) {
        return 5.4665;
    }
    return (std::pow(3.0, static_cast<double>((ura_index >> 3) & 0x07U)) *
                (1.0 + static_cast<double>(ura_index & 0x07U) / 4.0) -
            1.0) *
           1e-3;
}

}  // namespace rtcm_internal
}  // namespace io
}  // namespace libgnss
