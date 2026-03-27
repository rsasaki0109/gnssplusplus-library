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

namespace {

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

uint64_t readUnsignedBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
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

void writeUnsignedBits(std::vector<uint8_t>& data, int bit_pos, int num_bits, uint64_t value) {
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

void writeSignedBits(std::vector<uint8_t>& data, int bit_pos, int num_bits, int64_t value) {
    const uint64_t masked = static_cast<uint64_t>(value) & ((1ULL << num_bits) - 1ULL);
    writeUnsignedBits(data, bit_pos, num_bits, masked);
}

void writeSignMagnitudeBits(std::vector<uint8_t>& data, int bit_pos, int num_bits, int64_t value) {
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

int64_t signExtend(uint64_t value, int num_bits) {
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

int64_t readSignedBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    return signExtend(readUnsignedBits(data, data_size, bit_pos, num_bits), num_bits);
}

int64_t readSignMagnitudeBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    if (num_bits <= 1) {
        return 0;
    }
    const int64_t magnitude = static_cast<int64_t>(
        readUnsignedBits(data, data_size, bit_pos + 1, num_bits - 1));
    const bool negative = readUnsignedBits(data, data_size, bit_pos, 1) != 0;
    return negative ? -magnitude : magnitude;
}

bool decodeReferenceStationPosition(const RTCMMessage& message, Vector3d& position) {
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

int lockIndicatorFromObservation(const Observation& obs) {
    return obs.loss_of_lock ? 0 : 127;
}

int cnrUnitsFromObservation(const Observation& obs) {
    const double snr = (obs.snr > 0.0) ? obs.snr : static_cast<double>(obs.signal_strength);
    return static_cast<int>(std::llround(std::clamp(snr / kRTCMCn0Resolution, 0.0, 255.0)));
}

bool isSupportedGpsL1Signal(SignalType signal) {
    return signal == SignalType::GPS_L1CA || signal == SignalType::GPS_L1P;
}

bool isSupportedGpsL2Signal(SignalType signal) {
    return signal == SignalType::GPS_L2C || signal == SignalType::GPS_L2P;
}

bool isSupportedGpsMsmSignal(SignalType signal) {
    return isSupportedGpsL1Signal(signal) || isSupportedGpsL2Signal(signal);
}

bool isSupportedGlonassMsmSignal(SignalType signal) {
    return signal == SignalType::GLO_L1CA || signal == SignalType::GLO_L1P ||
           signal == SignalType::GLO_L2CA || signal == SignalType::GLO_L2P;
}

bool isFixedFrequencyMsm4MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1074 ||
           message_type == RTCMMessageType::RTCM_1094 ||
           message_type == RTCMMessageType::RTCM_1124;
}

bool isFixedFrequencyMsm5MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1075 ||
           message_type == RTCMMessageType::RTCM_1095 ||
           message_type == RTCMMessageType::RTCM_1125;
}

bool isFixedFrequencyMsm6MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1076 ||
           message_type == RTCMMessageType::RTCM_1096 ||
           message_type == RTCMMessageType::RTCM_1126;
}

bool isFixedFrequencyMsm7MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1077 ||
           message_type == RTCMMessageType::RTCM_1097 ||
           message_type == RTCMMessageType::RTCM_1127;
}

bool isFixedFrequencyMsmMessageType(RTCMMessageType message_type) {
    return isFixedFrequencyMsm4MessageType(message_type) ||
           isFixedFrequencyMsm5MessageType(message_type) ||
           isFixedFrequencyMsm6MessageType(message_type) ||
           isFixedFrequencyMsm7MessageType(message_type);
}

bool isGlonassMsm4MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1084;
}

bool isGlonassMsm5MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1085;
}

bool isGlonassMsm6MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1086;
}

bool isGlonassMsm7MessageType(RTCMMessageType message_type) {
    return message_type == RTCMMessageType::RTCM_1087;
}

bool isGlonassMsmMessageType(RTCMMessageType message_type) {
    return isGlonassMsm4MessageType(message_type) ||
           isGlonassMsm5MessageType(message_type) ||
           isGlonassMsm6MessageType(message_type) ||
           isGlonassMsm7MessageType(message_type);
}

bool isMsm5MessageType(RTCMMessageType message_type) {
    return isFixedFrequencyMsm5MessageType(message_type) ||
           isGlonassMsm5MessageType(message_type);
}

bool isMsm6MessageType(RTCMMessageType message_type) {
    return isFixedFrequencyMsm6MessageType(message_type) ||
           isGlonassMsm6MessageType(message_type);
}

bool isMsm7MessageType(RTCMMessageType message_type) {
    return isFixedFrequencyMsm7MessageType(message_type) ||
           isGlonassMsm7MessageType(message_type);
}

bool isSsrOrbitMessageType(RTCMMessageType message_type) {
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

bool isSsrClockMessageType(RTCMMessageType message_type) {
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

bool isSsrCodeBiasMessageType(RTCMMessageType message_type) {
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

bool isSsrCombinedMessageType(RTCMMessageType message_type) {
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

bool isSsrUraMessageType(RTCMMessageType message_type) {
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

bool isSsrHighRateClockMessageType(RTCMMessageType message_type) {
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

bool isSupportedSsrMessageType(RTCMMessageType message_type) {
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

SsrSystemDescriptor ssrDescriptorForMessageType(RTCMMessageType message_type) {
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

GNSSSystem fixedFrequencyMsmSystem(RTCMMessageType message_type) {
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

bool isSupportedFixedFrequencyMsmSignal(GNSSSystem system, SignalType signal) {
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

uint8_t glonassMsmSignalId(SignalType signal) {
    switch (signal) {
        case SignalType::GLO_L1CA: return 2U; // 1C
        case SignalType::GLO_L1P: return 3U;  // 1P
        case SignalType::GLO_L2CA: return 8U; // 2C
        case SignalType::GLO_L2P: return 9U;  // 2P
        default: return 0U;
    }
}

SignalType decodeGpsL1Signal(uint64_t code_indicator) {
    return code_indicator == 0 ? SignalType::GPS_L1CA : SignalType::GPS_L1P;
}

SignalType decodeGpsL2Signal(uint64_t code_indicator) {
    return code_indicator == 0 ? SignalType::GPS_L2C : SignalType::GPS_L2P;
}

uint8_t gpsMsmSignalId(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return 2U;  // 1C
        case SignalType::GPS_L1P: return 3U;   // 1P
        case SignalType::GPS_L2C: return 8U;   // 2C
        case SignalType::GPS_L2P: return 9U;   // 2P
        default: return 0U;
    }
}

uint8_t fixedFrequencyMsmSignalId(GNSSSystem system, SignalType signal) {
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

SignalType decodeGpsMsmSignal(uint8_t signal_id) {
    switch (signal_id) {
        case 2: return SignalType::GPS_L1CA;
        case 3: return SignalType::GPS_L1P;
        case 8: return SignalType::GPS_L2C;
        case 9: return SignalType::GPS_L2P;
        default: return SignalType::SIGNAL_TYPE_COUNT;
    }
}

SignalType decodeFixedFrequencyMsmSignal(GNSSSystem system, uint8_t signal_id) {
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

SignalType decodeGlonassMsmSignal(uint8_t signal_id) {
    switch (signal_id) {
        case 2: return SignalType::GLO_L1CA;
        case 3: return SignalType::GLO_L1P;
        case 8: return SignalType::GLO_L2CA;
        case 9: return SignalType::GLO_L2P;
        default: return SignalType::SIGNAL_TYPE_COUNT;
    }
}

double gpsSignalWavelength(SignalType signal) {
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

double glonassSignalWavelength(SignalType signal, int frequency_channel) {
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

double signalWavelength(SignalType signal) {
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

double observationRangeMeters(const Observation& obs) {
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

double observationRangeRateMetersPerSecond(const Observation& obs) {
    if (!obs.has_doppler || !std::isfinite(obs.doppler)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double wavelength = signalWavelength(obs.signal);
    if (wavelength <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return -obs.doppler * wavelength;
}

double glonassObservationRangeRateMetersPerSecond(const Observation& obs, int frequency_channel) {
    if (!obs.has_doppler || !std::isfinite(obs.doppler)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double wavelength = glonassSignalWavelength(obs.signal, frequency_channel);
    if (wavelength <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return -obs.doppler * wavelength;
}

uint8_t glonassMsmExtendedInfo(int frequency_channel) {
    return (frequency_channel >= -7 && frequency_channel <= 6)
               ? static_cast<uint8_t>(frequency_channel + 7)
               : 15U;
}

bool decodeGlonassMsmExtendedInfo(uint8_t info, int& frequency_channel) {
    if (info <= 13U) {
        frequency_channel = static_cast<int>(info) - 7;
        return true;
    }
    return false;
}

std::string resolveSerialPath(const std::string& source) {
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

int parseSerialBaud(const std::string& source) {
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

bool isTcpSource(const std::string& source) {
    return source.rfind("tcp://", 0) == 0;
}

TcpEndpoint parseTcpEndpoint(const std::string& source) {
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
speed_t baudRateConstant(int baud) {
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

bool configureSerialPort(int fd, int baud) {
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

int connectTcpSocket(const std::string& source) {
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

uint32_t fixedFrequencyMsmEpochMs(RTCMMessageType message_type, const GNSSTime& time) {
    if (message_type == RTCMMessageType::RTCM_1124 ||
        message_type == RTCMMessageType::RTCM_1125 ||
        message_type == RTCMMessageType::RTCM_1126 ||
        message_type == RTCMMessageType::RTCM_1127) {
        return rtcm_utils::gpsTimeToRTCMTime(time - 14.0);
    }
    return rtcm_utils::gpsTimeToRTCMTime(time);
}

GNSSTime fixedFrequencyMsmEpochToTime(RTCMMessageType message_type, uint32_t epoch_ms) {
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

int msmLockIndicatorFromObservation(const Observation& obs) {
    return obs.loss_of_lock ? 0 : 15;
}

uint16_t msmExtendedLockIndicatorFromObservation(const Observation& obs) {
    return obs.loss_of_lock ? 0U : 1023U;
}

uint8_t msmCn0UnitsFromObservation(const Observation& obs) {
    const double snr = (obs.snr > 0.0) ? obs.snr : static_cast<double>(obs.signal_strength);
    return static_cast<uint8_t>(std::llround(std::clamp(snr, 0.0, 63.0)));
}

uint16_t msmExtendedCn0UnitsFromObservation(const Observation& obs) {
    const double snr = (obs.snr > 0.0) ? obs.snr : static_cast<double>(obs.signal_strength);
    return static_cast<uint16_t>(std::llround(std::clamp(snr / 0.0625, 0.0, 1023.0)));
}

int leapSecondsForDate(int year, int month, int day) {
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

std::tm toUtcTm(const std::chrono::system_clock::time_point& tp) {
    const std::time_t raw_time = std::chrono::system_clock::to_time_t(tp);
    std::tm utc_tm{};
#if defined(_WIN32)
    gmtime_s(&utc_tm, &raw_time);
#else
    gmtime_r(&raw_time, &utc_tm);
#endif
    return utc_tm;
}

GNSSTime normalizeWeekTow(int week, double tow) {
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

int leapSecondsForApproxTime(const GNSSTime& time) {
    const std::tm utc_tm = toUtcTm(time.toSystemTime());
    return leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday);
}

GNSSTime currentGpstApprox() {
    const auto now = std::chrono::system_clock::now();
    const GNSSTime utc_like = GNSSTime::fromSystemTime(now);
    const std::tm utc_tm = toUtcTm(now);
    return normalizeWeekTow(
        utc_like.week,
        utc_like.tow +
            static_cast<double>(
                leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday)));
}

GNSSTime gpstToUtcApprox(const GNSSTime& gps_time) {
    return normalizeWeekTow(
        gps_time.week,
        gps_time.tow - static_cast<double>(leapSecondsForApproxTime(gps_time)));
}

GNSSTime utcToGpstApprox(const GNSSTime& utc_time) {
    const std::tm utc_tm = toUtcTm(utc_time.toSystemTime());
    return normalizeWeekTow(
        utc_time.week,
        utc_time.tow +
            static_cast<double>(
                leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday)));
}

double secondsOfDay(double tow) {
    double tod = std::fmod(tow, kSecondsPerDay);
    if (tod < 0.0) {
        tod += kSecondsPerDay;
    }
    return tod;
}

GNSSTime alignUtcTimeOfDay(double tod_utc, const GNSSTime& reference_utc) {
    const double ref_tod = secondsOfDay(reference_utc.tow);
    if (tod_utc < ref_tod - kHalfDaySeconds) {
        tod_utc += kSecondsPerDay;
    } else if (tod_utc > ref_tod + kHalfDaySeconds) {
        tod_utc -= kSecondsPerDay;
    }
    const double base_tow = reference_utc.tow - ref_tod;
    return normalizeWeekTow(reference_utc.week, base_tow + tod_utc);
}

uint32_t gpstToGlonassMsmEpoch(const GNSSTime& gps_time) {
    GNSSTime glot = gpstToUtcApprox(gps_time);
    glot = normalizeWeekTow(glot.week, glot.tow + 10800.0);
    const uint32_t dow = static_cast<uint32_t>(std::floor(glot.tow / kSecondsPerDay)) % 7U;
    const double tod = glot.tow - static_cast<double>(dow) * kSecondsPerDay;
    const uint32_t tod_ms = static_cast<uint32_t>(std::llround(tod * 1000.0)) & ((1U << 27) - 1U);
    return (dow << 27) | tod_ms;
}

GNSSTime glonassMsmEpochToGpst(uint32_t epoch) {
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

GNSSTime alignGpsSsrEpoch(double tow_seconds) {
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

GNSSTime alignGlonassSsrEpoch(double tod_glonass_seconds) {
    GNSSTime current_glonass = gpstToUtcApprox(currentGpstApprox());
    current_glonass = normalizeWeekTow(current_glonass.week, current_glonass.tow + 10800.0);
    const GNSSTime aligned_glonass = alignUtcTimeOfDay(tod_glonass_seconds, current_glonass);
    return utcToGpstApprox(normalizeWeekTow(aligned_glonass.week, aligned_glonass.tow - 10800.0));
}

bool decodeSsrOrbitHeader(const RTCMMessage& message,
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

bool decodeSsrClockHeader(const RTCMMessage& message,
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

uint8_t decodeSsrPrn(const SsrSystemDescriptor& descriptor, uint64_t raw_prn) {
    return static_cast<uint8_t>(raw_prn + static_cast<uint64_t>(descriptor.prn_offset));
}

int adjustGpsWeek(uint16_t week_mod_1024) {
    int current_week = currentGpstApprox().week;
    if (current_week < kMinimumExpandedGpsWeek) {
        current_week = kMinimumExpandedGpsWeek;
    }
    return static_cast<int>(week_mod_1024) +
           ((current_week - static_cast<int>(week_mod_1024) + 512) / 1024) * 1024;
}

uint8_t uraIndexFromMeters(double ura_meters) {
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

double uraMetersFromIndex(uint8_t ura_index) {
    return (ura_index < 15U) ? kUraMetersTable[ura_index] : 32767.0;
}

double uraMetersFromSsrIndex(uint8_t ura_index) {
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

}  // namespace

// RTCMProcessor implementation
void RTCMProcessor::clear() {
    reference_position_.setZero();
    has_reference_position_ = false;
    glonass_frequency_channels_.clear();
}

std::vector<RTCMMessage> RTCMProcessor::decode(const uint8_t* buffer, size_t size) {
    std::vector<RTCMMessage> messages;
    size_t i = 0;

    while (i < size) {
        if (buffer[i] != kRTCMPreamble) {
            ++i;
            continue;
        }

        if (i + 5 > size) {
            break;
        }

        RTCMMessageType message_type = RTCMMessageType::RTCM_UNKNOWN;
        uint16_t payload_len = 0;
        if (!parseHeader(buffer + i, message_type, payload_len)) {
            ++i;
            continue;
        }

        const size_t total_len = 3U + payload_len + 3U;
        if (i + total_len > size) {
            break;
        }

        ++stats_.total_messages;

        const uint32_t calculated_crc = calculateCRC24(buffer + i, 3 + payload_len);
        const uint8_t* received_crc_ptr = buffer + i + 3 + payload_len;
        const uint32_t received_crc =
            (static_cast<uint32_t>(received_crc_ptr[0]) << 16) |
            (static_cast<uint32_t>(received_crc_ptr[1]) << 8) |
            static_cast<uint32_t>(received_crc_ptr[2]);

        if (calculated_crc != received_crc) {
            ++stats_.crc_errors;
            ++i;
            continue;
        }

        RTCMMessage message;
        message.type = message_type;
        message.length = payload_len;
        message.data = extractPayload(buffer + i, total_len);
        message.crc = received_crc;
        message.valid = !message.data.empty();

        if (!message.valid || message.data.size() != payload_len) {
            ++stats_.decode_errors;
            i += total_len;
            continue;
        }

        ++stats_.valid_messages;
        ++stats_.message_counts[message.type];

        if (rtcm_utils::isStationMessage(message.type)) {
            Vector3d station_position = Vector3d::Zero();
            if (decodeReferenceStationPosition(message, station_position)) {
                setReferencePosition(station_position);
            }
        } else if (message.type == RTCMMessageType::RTCM_1020) {
            NavigationData nav_data;
            (void)decodeEphemerisMessage(message, nav_data);
        }

        messages.push_back(std::move(message));
        i += total_len;
    }

    return messages;
}


RTCMMessage RTCMProcessor::encodeObservations(const ObservationData& obs_data,
                                             RTCMMessageType message_type) {
    if (isGlonassMsmMessageType(message_type)) {
        const bool msm5 = isMsm5MessageType(message_type);
        const bool msm6 = isMsm6MessageType(message_type);
        const bool msm7 = isMsm7MessageType(message_type);
        struct MsmSatelliteData {
            uint8_t prn = 0;
            int glonass_frequency_channel = 0;
            uint8_t extended_info = 15U;
            uint32_t rough_range_units = 0;
            double rough_range = 0.0;
            int16_t rough_phase_range_rate = 0;
        };
        struct MsmCellData {
            const Observation* obs = nullptr;
            uint8_t prn = 0;
            uint8_t signal_id = 0;
        };

        std::map<uint8_t, std::map<uint8_t, const Observation*>> glonass_cells;
        for (const auto& obs : obs_data.observations) {
            if (obs.satellite.system != GNSSSystem::GLONASS || obs.satellite.prn == 0 ||
                obs.satellite.prn > 64 || !isSupportedGlonassMsmSignal(obs.signal)) {
                continue;
            }
            if (!obs.has_pseudorange && !obs.has_carrier_phase) {
                continue;
            }
            if (obs.has_glonass_frequency_channel) {
                glonass_frequency_channels_[obs.satellite] = obs.glonass_frequency_channel;
            }

            const uint8_t signal_id = glonassMsmSignalId(obs.signal);
            if (signal_id == 0) {
                continue;
            }
            auto& slot = glonass_cells[obs.satellite.prn][signal_id];
            if (slot == nullptr ||
                (!slot->has_pseudorange && obs.has_pseudorange) ||
                (!slot->has_carrier_phase && obs.has_carrier_phase)) {
                slot = &obs;
            }
        }

        if (glonass_cells.empty()) {
            return RTCMMessage();
        }

        std::vector<uint8_t> sat_ids;
        std::vector<uint8_t> signal_ids;
        sat_ids.reserve(glonass_cells.size());
        std::map<uint8_t, MsmSatelliteData> sat_data_by_prn;
        std::map<uint8_t, bool> signal_present;

        for (const auto& [prn, cells] : glonass_cells) {
            double base_range = std::numeric_limits<double>::quiet_NaN();
            double base_rate = std::numeric_limits<double>::quiet_NaN();
            int frequency_channel = 0;
            bool has_frequency_channel = false;

            for (const auto& [signal_id, obs] : cells) {
                signal_present[signal_id] = true;
                if (obs->has_glonass_frequency_channel) {
                    frequency_channel = obs->glonass_frequency_channel;
                    has_frequency_channel = true;
                } else {
                    const SatelliteId sat_id(GNSSSystem::GLONASS, prn);
                    const auto cache_it = glonass_frequency_channels_.find(sat_id);
                    if (cache_it != glonass_frequency_channels_.end()) {
                        frequency_channel = cache_it->second;
                        has_frequency_channel = true;
                    }
                }
            }

            if (!has_frequency_channel) {
                continue;
            }

            for (const auto& [signal_id, obs] : cells) {
                if (obs->has_pseudorange && std::isfinite(obs->pseudorange)) {
                    base_range = obs->pseudorange;
                    break;
                }
                if (has_frequency_channel &&
                    obs->has_carrier_phase && std::isfinite(obs->carrier_phase)) {
                    const double wavelength = glonassSignalWavelength(obs->signal, frequency_channel);
                    if (wavelength > 0.0) {
                        base_range = obs->carrier_phase * wavelength;
                        break;
                    }
                }
            }
            if (!std::isfinite(base_range) || base_range <= 0.0) {
                continue;
            }
            if (has_frequency_channel) {
                for (const auto& [signal_id, obs] : cells) {
                    base_rate = glonassObservationRangeRateMetersPerSecond(*obs, frequency_channel);
                    if (std::isfinite(base_rate)) {
                        break;
                    }
                }
            }

            const int64_t rough_range_units = static_cast<int64_t>(
                std::llround(base_range / (kRTCMGpsPrUnitMeters * kPow2Neg10)));
            const uint32_t rough_range_int_ms = static_cast<uint32_t>(rough_range_units >> 10);
            if (rough_range_units <= 0 || rough_range_int_ms >= 255U) {
                continue;
            }

            MsmSatelliteData sat_data;
            sat_data.prn = prn;
            sat_data.glonass_frequency_channel = frequency_channel;
            sat_data.extended_info =
                has_frequency_channel ? glonassMsmExtendedInfo(frequency_channel) : 15U;
            sat_data.rough_range_units = static_cast<uint32_t>(rough_range_units);
            sat_data.rough_range =
                static_cast<double>(sat_data.rough_range_units) * kPow2Neg10 * kRTCMGpsPrUnitMeters;
            if (std::isfinite(base_rate)) {
                if (std::abs(base_rate) > 8191.0) {
                    sat_data.rough_phase_range_rate = -8192;
                } else {
                    sat_data.rough_phase_range_rate =
                        static_cast<int16_t>(std::llround(base_rate));
                }
            }
            sat_data_by_prn[prn] = sat_data;
            if (has_frequency_channel) {
                glonass_frequency_channels_[SatelliteId(GNSSSystem::GLONASS, prn)] = frequency_channel;
            }
            sat_ids.push_back(prn);
        }

        if (sat_ids.empty()) {
            return RTCMMessage();
        }

        signal_ids.reserve(signal_present.size());
        for (const auto& [signal_id, present] : signal_present) {
            if (present) {
                signal_ids.push_back(signal_id);
            }
        }

        if (signal_ids.empty() || sat_ids.size() > 64 || signal_ids.size() > 32 ||
            sat_ids.size() * signal_ids.size() > 64) {
            return RTCMMessage();
        }

        std::vector<MsmCellData> cells;
        std::vector<bool> cell_mask(sat_ids.size() * signal_ids.size(), false);
        cells.reserve(glonass_cells.size() * 2U);
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const auto sat_it = glonass_cells.find(sat_ids[sat_index]);
            if (sat_it == glonass_cells.end()) {
                continue;
            }
            for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                const auto cell_it = sat_it->second.find(signal_ids[sig_index]);
                if (cell_it == sat_it->second.end()) {
                    continue;
                }
                cell_mask[sat_index * signal_ids.size() + sig_index] = true;
                cells.push_back({cell_it->second, sat_ids[sat_index], signal_ids[sig_index]});
            }
        }

        if (cells.empty()) {
            return RTCMMessage();
        }

        const size_t total_bits =
            169U + cell_mask.size() + sat_ids.size() * ((msm5 || msm7) ? 36U : 18U) +
            cells.size() * (msm7 ? 80U : (msm5 ? 63U : (msm6 ? 65U : 48U)));
        std::vector<uint8_t> payload((total_bits + 7U) / 8U, 0);

        int bit_pos = 0;
        writeUnsignedBits(payload, bit_pos, 12, static_cast<uint16_t>(message_type)); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 12, 0); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 30, gpstToGlonassMsmEpoch(obs_data.time)); bit_pos += 30;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;
        writeUnsignedBits(payload, bit_pos, 7, 0); bit_pos += 7;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;

        for (int prn = 1; prn <= 64; ++prn) {
            writeUnsignedBits(
                payload,
                bit_pos,
                1,
                std::binary_search(sat_ids.begin(), sat_ids.end(), static_cast<uint8_t>(prn)) ? 1U : 0U);
            bit_pos += 1;
        }
        for (int signal_id = 1; signal_id <= 32; ++signal_id) {
            writeUnsignedBits(
                payload,
                bit_pos,
                1,
                std::binary_search(signal_ids.begin(), signal_ids.end(), static_cast<uint8_t>(signal_id)) ? 1U : 0U);
            bit_pos += 1;
        }
        for (const bool present : cell_mask) {
            writeUnsignedBits(payload, bit_pos, 1, present ? 1U : 0U);
            bit_pos += 1;
        }

        for (const uint8_t prn : sat_ids) {
            const auto sat_it = sat_data_by_prn.find(prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            writeUnsignedBits(payload, bit_pos, 8, sat_it->second.rough_range_units >> 10); bit_pos += 8;
        }
        if (msm5 || msm7) {
            for (const uint8_t prn : sat_ids) {
                const auto sat_it = sat_data_by_prn.find(prn);
                if (sat_it == sat_data_by_prn.end()) {
                    return RTCMMessage();
                }
                writeUnsignedBits(payload, bit_pos, 4, sat_it->second.extended_info); bit_pos += 4;
            }
        }
        for (const uint8_t prn : sat_ids) {
            const auto sat_it = sat_data_by_prn.find(prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            writeUnsignedBits(payload, bit_pos, 10, sat_it->second.rough_range_units & 0x3FFU); bit_pos += 10;
        }
        if (msm5 || msm7) {
            for (const uint8_t prn : sat_ids) {
                const auto sat_it = sat_data_by_prn.find(prn);
                if (sat_it == sat_data_by_prn.end()) {
                    return RTCMMessage();
                }
                writeSignedBits(payload, bit_pos, 14, sat_it->second.rough_phase_range_rate);
                bit_pos += 14;
            }
        }

        struct EncodedMsmCellData {
            int32_t fine_pr = -16384;
            int32_t fine_cp = -2097152;
            uint8_t lock = 0;
            uint8_t half = 0;
            uint8_t cnr = 0;
            int16_t fine_rate = -16384;
            int32_t fine_pr_ext = -524288;
            int32_t fine_cp_ext = -8388608;
            uint16_t lock_ext = 0;
            uint16_t cnr_ext = 0;
        };
        std::vector<EncodedMsmCellData> encoded_cells;
        encoded_cells.reserve(cells.size());
        for (const auto& cell : cells) {
            const auto sat_it = sat_data_by_prn.find(cell.prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            const double wavelength =
                glonassSignalWavelength(cell.obs->signal, sat_it->second.glonass_frequency_channel);
            EncodedMsmCellData encoded_cell;

            if (cell.obs->has_pseudorange && std::isfinite(cell.obs->pseudorange)) {
                const double value = cell.obs->pseudorange - sat_it->second.rough_range;
                if (msm6 || msm7) {
                    if (std::abs(value) <= 292.7) {
                        encoded_cell.fine_pr_ext = static_cast<int32_t>(
                            std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg29)));
                    }
                } else if (std::abs(value) <= 292.7) {
                    encoded_cell.fine_pr = static_cast<int32_t>(
                        std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg24)));
                }
            }
            if (cell.obs->has_carrier_phase && wavelength > 0.0 && std::isfinite(cell.obs->carrier_phase)) {
                const double value = cell.obs->carrier_phase * wavelength - sat_it->second.rough_range;
                if (msm6 || msm7) {
                    if (std::abs(value) <= 1171.0) {
                        encoded_cell.fine_cp_ext = static_cast<int32_t>(
                            std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg31)));
                    }
                } else if (std::abs(value) <= 1171.0) {
                    encoded_cell.fine_cp = static_cast<int32_t>(
                        std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg29)));
                }
            }
            encoded_cell.lock = static_cast<uint8_t>(msmLockIndicatorFromObservation(*cell.obs));
            encoded_cell.lock_ext = msmExtendedLockIndicatorFromObservation(*cell.obs);
            encoded_cell.half = (cell.obs->lli & 0x02U) != 0U ? 1U : 0U;
            encoded_cell.cnr = msmCn0UnitsFromObservation(*cell.obs);
            encoded_cell.cnr_ext = msmExtendedCn0UnitsFromObservation(*cell.obs);
            if ((msm5 || msm7) && wavelength > 0.0) {
                const double total_rate = glonassObservationRangeRateMetersPerSecond(
                    *cell.obs, sat_it->second.glonass_frequency_channel);
                if (std::isfinite(total_rate) && sat_it->second.rough_phase_range_rate != -8192) {
                    const double fine_rate =
                        total_rate - static_cast<double>(sat_it->second.rough_phase_range_rate);
                    if (std::abs(fine_rate) <= 1.6383) {
                        encoded_cell.fine_rate =
                            static_cast<int16_t>(std::llround(fine_rate / 1e-4));
                    }
                }
            }
            encoded_cells.push_back(encoded_cell);
        }

        for (const auto& encoded_cell : encoded_cells) {
            writeSignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 20 : 15,
                (msm6 || msm7) ? encoded_cell.fine_pr_ext : encoded_cell.fine_pr);
            bit_pos += (msm6 || msm7) ? 20 : 15;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeSignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 24 : 22,
                (msm6 || msm7) ? encoded_cell.fine_cp_ext : encoded_cell.fine_cp);
            bit_pos += (msm6 || msm7) ? 24 : 22;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 10 : 4,
                (msm6 || msm7) ? encoded_cell.lock_ext : encoded_cell.lock);
            bit_pos += (msm6 || msm7) ? 10 : 4;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(payload, bit_pos, 1, encoded_cell.half); bit_pos += 1;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 10 : 6,
                (msm6 || msm7) ? encoded_cell.cnr_ext : encoded_cell.cnr);
            bit_pos += (msm6 || msm7) ? 10 : 6;
        }
        if (msm5 || msm7) {
            for (const auto& encoded_cell : encoded_cells) {
                writeSignedBits(payload, bit_pos, 15, encoded_cell.fine_rate); bit_pos += 15;
            }
        }

        RTCMMessage message;
        message.type = message_type;
        message.length = static_cast<uint16_t>(payload.size());
        message.data = std::move(payload);
        message.valid = true;
        return message;
    }

    if (isFixedFrequencyMsmMessageType(message_type)) {
        const bool msm5 = isMsm5MessageType(message_type);
        const bool msm6 = isMsm6MessageType(message_type);
        const bool msm7 = isMsm7MessageType(message_type);
        const GNSSSystem system = fixedFrequencyMsmSystem(message_type);
        if (system == GNSSSystem::UNKNOWN) {
            return RTCMMessage();
        }

        struct MsmSatelliteData {
            uint8_t prn = 0;
            uint8_t extended_info = 0U;
            uint32_t rough_range_units = 0;
            double rough_range = 0.0;
            int16_t rough_phase_range_rate = 0;
        };
        struct MsmCellData {
            const Observation* obs = nullptr;
            uint8_t prn = 0;
            uint8_t signal_id = 0;
        };

        std::map<uint8_t, std::map<uint8_t, const Observation*>> system_cells;
        for (const auto& obs : obs_data.observations) {
            if (obs.satellite.system != system || obs.satellite.prn == 0 ||
                obs.satellite.prn > 64 || !isSupportedFixedFrequencyMsmSignal(system, obs.signal)) {
                continue;
            }
            if (!obs.has_pseudorange && !obs.has_carrier_phase) {
                continue;
            }

            const uint8_t signal_id = fixedFrequencyMsmSignalId(system, obs.signal);
            if (signal_id == 0) {
                continue;
            }
            auto& slot = system_cells[obs.satellite.prn][signal_id];
            if (slot == nullptr ||
                (!slot->has_pseudorange && obs.has_pseudorange) ||
                (!slot->has_carrier_phase && obs.has_carrier_phase)) {
                slot = &obs;
            }
        }

        if (system_cells.empty()) {
            return RTCMMessage();
        }

        std::vector<uint8_t> sat_ids;
        std::vector<uint8_t> signal_ids;
        sat_ids.reserve(system_cells.size());
        std::map<uint8_t, MsmSatelliteData> sat_data_by_prn;
        std::map<uint8_t, bool> signal_present;

        for (const auto& [prn, cells] : system_cells) {
            double base_range = std::numeric_limits<double>::quiet_NaN();
            double base_rate = std::numeric_limits<double>::quiet_NaN();
            for (const auto& [signal_id, obs] : cells) {
                signal_present[signal_id] = true;
                if (!std::isfinite(base_range)) {
                    base_range = observationRangeMeters(*obs);
                }
                if (!std::isfinite(base_rate)) {
                    base_rate = observationRangeRateMetersPerSecond(*obs);
                }
            }
            if (!std::isfinite(base_range) || base_range <= 0.0) {
                continue;
            }

            const int64_t rough_range_units = static_cast<int64_t>(
                std::llround(base_range / (kRTCMGpsPrUnitMeters * kPow2Neg10)));
            const uint32_t rough_range_int_ms = static_cast<uint32_t>(rough_range_units >> 10);
            if (rough_range_units <= 0 || rough_range_int_ms >= 255U) {
                continue;
            }

            MsmSatelliteData sat_data;
            sat_data.prn = prn;
            sat_data.rough_range_units = static_cast<uint32_t>(rough_range_units);
            sat_data.rough_range =
                static_cast<double>(sat_data.rough_range_units) * kPow2Neg10 * kRTCMGpsPrUnitMeters;
            if (std::isfinite(base_rate)) {
                if (std::abs(base_rate) > 8191.0) {
                    sat_data.rough_phase_range_rate = -8192;
                } else {
                    sat_data.rough_phase_range_rate =
                        static_cast<int16_t>(std::llround(base_rate));
                }
            }
            sat_data_by_prn[prn] = sat_data;
            sat_ids.push_back(prn);
        }

        if (sat_ids.empty()) {
            return RTCMMessage();
        }

        signal_ids.reserve(signal_present.size());
        for (const auto& [signal_id, present] : signal_present) {
            if (present) {
                signal_ids.push_back(signal_id);
            }
        }

        if (signal_ids.empty() || sat_ids.size() > 64 || signal_ids.size() > 32 ||
            sat_ids.size() * signal_ids.size() > 64) {
            return RTCMMessage();
        }

        std::vector<MsmCellData> cells;
        std::vector<bool> cell_mask(sat_ids.size() * signal_ids.size(), false);
        cells.reserve(system_cells.size() * 2U);
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const auto sat_it = system_cells.find(sat_ids[sat_index]);
            if (sat_it == system_cells.end()) {
                continue;
            }
            for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                const auto cell_it = sat_it->second.find(signal_ids[sig_index]);
                if (cell_it == sat_it->second.end()) {
                    continue;
                }
                cell_mask[sat_index * signal_ids.size() + sig_index] = true;
                cells.push_back({cell_it->second, sat_ids[sat_index], signal_ids[sig_index]});
            }
        }

        if (cells.empty()) {
            return RTCMMessage();
        }

        const size_t total_bits =
            169U + cell_mask.size() + sat_ids.size() * ((msm5 || msm7) ? 36U : 18U) +
            cells.size() * (msm7 ? 80U : (msm5 ? 63U : (msm6 ? 65U : 48U)));
        std::vector<uint8_t> payload((total_bits + 7U) / 8U, 0);

        int bit_pos = 0;
        writeUnsignedBits(payload, bit_pos, 12, static_cast<uint16_t>(message_type)); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 12, 0); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 30, fixedFrequencyMsmEpochMs(message_type, obs_data.time)); bit_pos += 30;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;
        writeUnsignedBits(payload, bit_pos, 7, 0); bit_pos += 7;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;

        for (int prn = 1; prn <= 64; ++prn) {
            writeUnsignedBits(
                payload,
                bit_pos,
                1,
                std::binary_search(sat_ids.begin(), sat_ids.end(), static_cast<uint8_t>(prn)) ? 1U : 0U);
            bit_pos += 1;
        }
        for (int signal_id = 1; signal_id <= 32; ++signal_id) {
            writeUnsignedBits(
                payload,
                bit_pos,
                1,
                std::binary_search(signal_ids.begin(), signal_ids.end(), static_cast<uint8_t>(signal_id)) ? 1U : 0U);
            bit_pos += 1;
        }
        for (const bool present : cell_mask) {
            writeUnsignedBits(payload, bit_pos, 1, present ? 1U : 0U);
            bit_pos += 1;
        }

        for (const uint8_t prn : sat_ids) {
            const auto sat_it = sat_data_by_prn.find(prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            writeUnsignedBits(payload, bit_pos, 8, sat_it->second.rough_range_units >> 10); bit_pos += 8;
        }
        if (msm5 || msm7) {
            for (const uint8_t prn : sat_ids) {
                const auto sat_it = sat_data_by_prn.find(prn);
                if (sat_it == sat_data_by_prn.end()) {
                    return RTCMMessage();
                }
                writeUnsignedBits(payload, bit_pos, 4, sat_it->second.extended_info); bit_pos += 4;
            }
        }
        for (const uint8_t prn : sat_ids) {
            const auto sat_it = sat_data_by_prn.find(prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            writeUnsignedBits(payload, bit_pos, 10, sat_it->second.rough_range_units & 0x3FFU); bit_pos += 10;
        }
        if (msm5 || msm7) {
            for (const uint8_t prn : sat_ids) {
                const auto sat_it = sat_data_by_prn.find(prn);
                if (sat_it == sat_data_by_prn.end()) {
                    return RTCMMessage();
                }
                writeSignedBits(payload, bit_pos, 14, sat_it->second.rough_phase_range_rate);
                bit_pos += 14;
            }
        }

        struct EncodedMsmCellData {
            int32_t fine_pr = -16384;
            int32_t fine_cp = -2097152;
            uint8_t lock = 0;
            uint8_t half = 0;
            uint8_t cnr = 0;
            int16_t fine_rate = -16384;
            int32_t fine_pr_ext = -524288;
            int32_t fine_cp_ext = -8388608;
            uint16_t lock_ext = 0;
            uint16_t cnr_ext = 0;
        };
        std::vector<EncodedMsmCellData> encoded_cells;
        encoded_cells.reserve(cells.size());
        for (const auto& cell : cells) {
            const auto sat_it = sat_data_by_prn.find(cell.prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            const double wavelength = signalWavelength(cell.obs->signal);
            EncodedMsmCellData encoded_cell;

            if (cell.obs->has_pseudorange && std::isfinite(cell.obs->pseudorange)) {
                const double value = cell.obs->pseudorange - sat_it->second.rough_range;
                if (msm6 || msm7) {
                    if (std::abs(value) <= 292.7) {
                        encoded_cell.fine_pr_ext = static_cast<int32_t>(
                            std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg29)));
                    }
                } else if (std::abs(value) <= 292.7) {
                    encoded_cell.fine_pr = static_cast<int32_t>(
                        std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg24)));
                }
            }
            if (cell.obs->has_carrier_phase && wavelength > 0.0 && std::isfinite(cell.obs->carrier_phase)) {
                const double value = cell.obs->carrier_phase * wavelength - sat_it->second.rough_range;
                if (msm6 || msm7) {
                    if (std::abs(value) <= 1171.0) {
                        encoded_cell.fine_cp_ext = static_cast<int32_t>(
                            std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg31)));
                    }
                } else if (std::abs(value) <= 1171.0) {
                    encoded_cell.fine_cp = static_cast<int32_t>(
                        std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg29)));
                }
            }
            encoded_cell.lock = static_cast<uint8_t>(msmLockIndicatorFromObservation(*cell.obs));
            encoded_cell.lock_ext = msmExtendedLockIndicatorFromObservation(*cell.obs);
            encoded_cell.half = (cell.obs->lli & 0x02U) != 0U ? 1U : 0U;
            encoded_cell.cnr = msmCn0UnitsFromObservation(*cell.obs);
            encoded_cell.cnr_ext = msmExtendedCn0UnitsFromObservation(*cell.obs);
            if ((msm5 || msm7) && wavelength > 0.0) {
                const double total_rate = observationRangeRateMetersPerSecond(*cell.obs);
                if (std::isfinite(total_rate) && sat_it->second.rough_phase_range_rate != -8192) {
                    const double fine_rate =
                        total_rate - static_cast<double>(sat_it->second.rough_phase_range_rate);
                    if (std::abs(fine_rate) <= 1.6383) {
                        encoded_cell.fine_rate =
                            static_cast<int16_t>(std::llround(fine_rate / 1e-4));
                    }
                }
            }
            encoded_cells.push_back(encoded_cell);
        }

        for (const auto& encoded_cell : encoded_cells) {
            writeSignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 20 : 15,
                (msm6 || msm7) ? encoded_cell.fine_pr_ext : encoded_cell.fine_pr);
            bit_pos += (msm6 || msm7) ? 20 : 15;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeSignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 24 : 22,
                (msm6 || msm7) ? encoded_cell.fine_cp_ext : encoded_cell.fine_cp);
            bit_pos += (msm6 || msm7) ? 24 : 22;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 10 : 4,
                (msm6 || msm7) ? encoded_cell.lock_ext : encoded_cell.lock);
            bit_pos += (msm6 || msm7) ? 10 : 4;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(payload, bit_pos, 1, encoded_cell.half); bit_pos += 1;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 10 : 6,
                (msm6 || msm7) ? encoded_cell.cnr_ext : encoded_cell.cnr);
            bit_pos += (msm6 || msm7) ? 10 : 6;
        }
        if (msm5 || msm7) {
            for (const auto& encoded_cell : encoded_cells) {
                writeSignedBits(payload, bit_pos, 15, encoded_cell.fine_rate); bit_pos += 15;
            }
        }

        RTCMMessage message;
        message.type = message_type;
        message.length = static_cast<uint16_t>(payload.size());
        message.data = std::move(payload);
        message.valid = true;
        return message;
    }

    const bool extended = message_type == RTCMMessageType::RTCM_1004;
    if (!extended && message_type != RTCMMessageType::RTCM_1003) {
        return RTCMMessage();
    }

    struct GPSPair {
        const Observation* l1 = nullptr;
        const Observation* l2 = nullptr;
    };

    std::map<uint8_t, GPSPair> gps_pairs;
    for (const auto& obs : obs_data.observations) {
        if (obs.satellite.system != GNSSSystem::GPS) {
            continue;
        }
        auto& pair = gps_pairs[obs.satellite.prn];
        if (isSupportedGpsL1Signal(obs.signal)) {
            if (pair.l1 == nullptr && obs.has_pseudorange && obs.has_carrier_phase) {
                pair.l1 = &obs;
            }
        } else if (isSupportedGpsL2Signal(obs.signal)) {
            if (pair.l2 == nullptr && obs.has_pseudorange && obs.has_carrier_phase) {
                pair.l2 = &obs;
            }
        }
    }

    struct EncodedSatellite {
        uint8_t prn = 0;
        uint8_t l1_code = 0;
        uint32_t l1_pr_units = 0;
        int32_t l1_phase_delta_units = 0;
        uint8_t l1_lock = 0;
        uint8_t l1_cnr = 0;
        uint8_t ambiguity = 0;
        uint8_t l2_code = 0;
        int32_t l2_pr_delta_units = 0;
        int32_t l2_phase_delta_units = 0;
        uint8_t l2_lock = 0;
        uint8_t l2_cnr = 0;
    };

    std::vector<EncodedSatellite> encoded_sats;
    encoded_sats.reserve(gps_pairs.size());
    for (const auto& [prn, pair] : gps_pairs) {
        if (pair.l1 == nullptr || pair.l2 == nullptr || prn == 0 || prn > 32) {
            continue;
        }

        const double p1 = pair.l1->pseudorange;
        const double p2 = pair.l2->pseudorange;
        const double l1_range = pair.l1->carrier_phase * constants::GPS_L1_WAVELENGTH;
        const double l2_range = pair.l2->carrier_phase * constants::GPS_L2_WAVELENGTH;
        if (!std::isfinite(p1) || !std::isfinite(p2) ||
            !std::isfinite(l1_range) || !std::isfinite(l2_range) ||
            p1 < 0.0 || p2 < 0.0) {
            continue;
        }

        const double ambiguity_d = std::floor(p1 / kRTCMGpsPrUnitMeters);
        const double l1_pr_mod = p1 - ambiguity_d * kRTCMGpsPrUnitMeters;
        const int64_t ambiguity = static_cast<int64_t>(ambiguity_d);
        const int64_t l1_pr_units = static_cast<int64_t>(std::llround(l1_pr_mod / kRTCMGpsPseudorangeResolution));
        const int64_t l1_phase_delta_units =
            static_cast<int64_t>(std::llround((l1_range - p1) / kRTCMGpsPhaseResolution));
        const int64_t l2_pr_delta_units =
            static_cast<int64_t>(std::llround((p2 - p1) / kRTCMGpsPseudorangeResolution));
        const int64_t l2_phase_delta_units =
            static_cast<int64_t>(std::llround((l2_range - p1) / kRTCMGpsPhaseResolution));

        if (ambiguity < 0 || ambiguity > 255 ||
            l1_pr_units < 0 || l1_pr_units > 0xFFFFFF ||
            l1_phase_delta_units < -(1 << 19) || l1_phase_delta_units > ((1 << 19) - 1) ||
            l2_pr_delta_units < -(1 << 13) || l2_pr_delta_units > ((1 << 13) - 1) ||
            l2_phase_delta_units < -(1 << 19) || l2_phase_delta_units > ((1 << 19) - 1)) {
            continue;
        }

        EncodedSatellite encoded;
        encoded.prn = prn;
        encoded.l1_code = pair.l1->signal == SignalType::GPS_L1CA ? 0U : 1U;
        encoded.l1_pr_units = static_cast<uint32_t>(l1_pr_units);
        encoded.l1_phase_delta_units = static_cast<int32_t>(l1_phase_delta_units);
        encoded.l1_lock = static_cast<uint8_t>(lockIndicatorFromObservation(*pair.l1));
        encoded.l1_cnr = static_cast<uint8_t>(cnrUnitsFromObservation(*pair.l1));
        encoded.ambiguity = static_cast<uint8_t>(ambiguity);
        encoded.l2_code = pair.l2->signal == SignalType::GPS_L2C ? 0U : 1U;
        encoded.l2_pr_delta_units = static_cast<int32_t>(l2_pr_delta_units);
        encoded.l2_phase_delta_units = static_cast<int32_t>(l2_phase_delta_units);
        encoded.l2_lock = static_cast<uint8_t>(lockIndicatorFromObservation(*pair.l2));
        encoded.l2_cnr = static_cast<uint8_t>(cnrUnitsFromObservation(*pair.l2));
        encoded_sats.push_back(encoded);
    }

    if (encoded_sats.empty()) {
        return RTCMMessage();
    }

    const int num_sats = static_cast<int>(encoded_sats.size());
    const int bits_per_sat = extended ? 125 : 109;
    const int total_bits = 64 + num_sats * bits_per_sat;
    std::vector<uint8_t> payload(static_cast<size_t>((total_bits + 7) / 8), 0);

    int bit_pos = 0;
    writeUnsignedBits(payload, bit_pos, 12, static_cast<uint16_t>(message_type)); bit_pos += 12;
    writeUnsignedBits(payload, bit_pos, 12, 0); bit_pos += 12;
    const uint32_t epoch_ms = rtcm_utils::gpsTimeToRTCMTime(obs_data.time);
    writeUnsignedBits(payload, bit_pos, 30, epoch_ms); bit_pos += 30;
    writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
    writeUnsignedBits(payload, bit_pos, 5, static_cast<uint64_t>(num_sats)); bit_pos += 5;
    writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
    writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;

    for (const auto& sat : encoded_sats) {
        writeUnsignedBits(payload, bit_pos, 6, sat.prn); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 1, sat.l1_code); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 24, sat.l1_pr_units); bit_pos += 24;
        writeSignedBits(payload, bit_pos, 20, sat.l1_phase_delta_units); bit_pos += 20;
        writeUnsignedBits(payload, bit_pos, 7, sat.l1_lock); bit_pos += 7;
        writeUnsignedBits(payload, bit_pos, 8, sat.ambiguity); bit_pos += 8;
        if (extended) {
            writeUnsignedBits(payload, bit_pos, 8, sat.l1_cnr); bit_pos += 8;
        }
        writeUnsignedBits(payload, bit_pos, 2, sat.l2_code); bit_pos += 2;
        writeSignedBits(payload, bit_pos, 14, sat.l2_pr_delta_units); bit_pos += 14;
        writeSignedBits(payload, bit_pos, 20, sat.l2_phase_delta_units); bit_pos += 20;
        writeUnsignedBits(payload, bit_pos, 7, sat.l2_lock); bit_pos += 7;
        if (extended) {
            writeUnsignedBits(payload, bit_pos, 8, sat.l2_cnr); bit_pos += 8;
        }
    }

    RTCMMessage message;
    message.type = message_type;
    message.length = static_cast<uint16_t>(payload.size());
    message.data = std::move(payload);
    message.valid = true;
    return message;
}

RTCMMessage RTCMProcessor::encodeEphemeris(const Ephemeris& ephemeris) {
    if (!ephemeris.valid) {
        return RTCMMessage();
    }

    if (ephemeris.satellite.system == GNSSSystem::GPS &&
        ephemeris.satellite.prn >= 1 && ephemeris.satellite.prn <= 32 &&
        ephemeris.sqrt_a > 0.0) {
        const double toe_seconds = ephemeris.toes != 0.0 ? ephemeris.toes : ephemeris.toe.tow;
        const double toc_seconds = ephemeris.toc.tow;
        const int64_t idot = static_cast<int64_t>(
            std::llround(ephemeris.idot / (kPow2Neg43 * kSemiCircleToRadians)));
        const int64_t af2 = static_cast<int64_t>(std::llround(ephemeris.af2 / kPow2Neg55));
        const int64_t af1 = static_cast<int64_t>(std::llround(ephemeris.af1 / kPow2Neg43));
        const int64_t af0 = static_cast<int64_t>(std::llround(ephemeris.af0 / kPow2Neg31));
        const int64_t crs = static_cast<int64_t>(std::llround(ephemeris.crs / kPow2Neg5));
        const int64_t delta_n = static_cast<int64_t>(
            std::llround(ephemeris.delta_n / (kPow2Neg43 * kSemiCircleToRadians)));
        const int64_t m0 = static_cast<int64_t>(
            std::llround(ephemeris.m0 / (kPow2Neg31 * kSemiCircleToRadians)));
        const int64_t cuc = static_cast<int64_t>(std::llround(ephemeris.cuc / kPow2Neg29));
        const uint64_t eccentricity = static_cast<uint64_t>(std::llround(ephemeris.e / kPow2Neg33));
        const int64_t cus = static_cast<int64_t>(std::llround(ephemeris.cus / kPow2Neg29));
        const uint64_t sqrt_a = static_cast<uint64_t>(std::llround(ephemeris.sqrt_a / kPow2Neg19));
        const int64_t cic = static_cast<int64_t>(std::llround(ephemeris.cic / kPow2Neg29));
        const int64_t omega0 = static_cast<int64_t>(
            std::llround(ephemeris.omega0 / (kPow2Neg31 * kSemiCircleToRadians)));
        const int64_t cis = static_cast<int64_t>(std::llround(ephemeris.cis / kPow2Neg29));
        const int64_t i0 = static_cast<int64_t>(
            std::llround(ephemeris.i0 / (kPow2Neg31 * kSemiCircleToRadians)));
        const int64_t crc = static_cast<int64_t>(std::llround(ephemeris.crc / kPow2Neg5));
        const int64_t omega = static_cast<int64_t>(
            std::llround(ephemeris.omega / (kPow2Neg31 * kSemiCircleToRadians)));
        const int64_t omega_dot = static_cast<int64_t>(
            std::llround(ephemeris.omega_dot / (kPow2Neg43 * kSemiCircleToRadians)));
        const int64_t tgd = static_cast<int64_t>(std::llround(ephemeris.tgd / kPow2Neg31));
        const uint64_t toe = static_cast<uint64_t>(std::llround(toe_seconds / 16.0));
        const uint64_t toc = static_cast<uint64_t>(std::llround(toc_seconds / 16.0));

        if (idot < -(1 << 13) || idot > ((1 << 13) - 1) ||
            af2 < -(1 << 7) || af2 > ((1 << 7) - 1) ||
            af1 < -(1 << 15) || af1 > ((1 << 15) - 1) ||
            af0 < -(1 << 21) || af0 > ((1 << 21) - 1) ||
            crs < -(1 << 15) || crs > ((1 << 15) - 1) ||
            delta_n < -(1 << 15) || delta_n > ((1 << 15) - 1) ||
            m0 < -(1LL << 31) || m0 > ((1LL << 31) - 1) ||
            cuc < -(1 << 15) || cuc > ((1 << 15) - 1) ||
            eccentricity > 0xFFFFFFFFULL ||
            cus < -(1 << 15) || cus > ((1 << 15) - 1) ||
            sqrt_a > 0xFFFFFFFFULL ||
            toe > 0xFFFFULL ||
            cic < -(1 << 15) || cic > ((1 << 15) - 1) ||
            omega0 < -(1LL << 31) || omega0 > ((1LL << 31) - 1) ||
            cis < -(1 << 15) || cis > ((1 << 15) - 1) ||
            i0 < -(1LL << 31) || i0 > ((1LL << 31) - 1) ||
            crc < -(1 << 15) || crc > ((1 << 15) - 1) ||
            omega < -(1LL << 31) || omega > ((1LL << 31) - 1) ||
            omega_dot < -(1 << 23) || omega_dot > ((1 << 23) - 1) ||
            tgd < -(1 << 7) || tgd > ((1 << 7) - 1) ||
            toc > 0xFFFFULL) {
            return RTCMMessage();
        }

        std::vector<uint8_t> payload(61, 0);
        int bit_pos = 0;
        writeUnsignedBits(payload, bit_pos, 12, 1019); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 6, ephemeris.satellite.prn); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 10, ephemeris.week % 1024U); bit_pos += 10;
        const uint8_t ura_index =
            (std::isfinite(ephemeris.sv_accuracy) && ephemeris.sv_accuracy > 0.0)
                ? uraIndexFromMeters(ephemeris.sv_accuracy)
                : ephemeris.ura;
        writeUnsignedBits(payload, bit_pos, 4, ura_index); bit_pos += 4;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeSignedBits(payload, bit_pos, 14, idot); bit_pos += 14;
        writeUnsignedBits(payload, bit_pos, 8, ephemeris.iode & 0xFFU); bit_pos += 8;
        writeUnsignedBits(payload, bit_pos, 16, toc); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 8, af2); bit_pos += 8;
        writeSignedBits(payload, bit_pos, 16, af1); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 22, af0); bit_pos += 22;
        writeUnsignedBits(payload, bit_pos, 10, ephemeris.iodc & 0x03FFU); bit_pos += 10;
        writeSignedBits(payload, bit_pos, 16, crs); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 16, delta_n); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 32, m0); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 16, cuc); bit_pos += 16;
        writeUnsignedBits(payload, bit_pos, 32, eccentricity); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 16, cus); bit_pos += 16;
        writeUnsignedBits(payload, bit_pos, 32, sqrt_a); bit_pos += 32;
        writeUnsignedBits(payload, bit_pos, 16, toe); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 16, cic); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 32, omega0); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 16, cis); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 32, i0); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 16, crc); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 32, omega); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 24, omega_dot); bit_pos += 24;
        writeSignedBits(payload, bit_pos, 8, tgd); bit_pos += 8;
        writeUnsignedBits(payload, bit_pos, 6, ephemeris.health & 0x3FU); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;

        RTCMMessage message;
        message.type = RTCMMessageType::RTCM_1019;
        message.length = static_cast<uint16_t>(payload.size());
        message.data = std::move(payload);
        message.valid = true;
        return message;
    }

    if (ephemeris.satellite.system == GNSSSystem::GLONASS &&
        ephemeris.satellite.prn >= 1 && ephemeris.satellite.prn <= 32) {
        const GNSSTime toe_gpst = ephemeris.toe;
        const GNSSTime tof_gpst =
            (ephemeris.tof.week != 0 || std::abs(ephemeris.tof.tow) > 0.0) ? ephemeris.tof : ephemeris.toe;
        const GNSSTime toe_utc = gpstToUtcApprox(toe_gpst);
        const GNSSTime tof_utc = gpstToUtcApprox(tof_gpst);
        const double toe_local = secondsOfDay(toe_utc.tow + 10800.0);
        const double tof_local = secondsOfDay(tof_utc.tow + 10800.0);
        const int half_minutes = static_cast<int>(std::llround(tof_local / 30.0));
        const int tk_h = (half_minutes / 120) % 24;
        const int tk_m = (half_minutes % 120) / 2;
        const int tk_s = half_minutes % 2;
        const int tb = static_cast<int>(std::llround(toe_local / 900.0)) % 96;
        const int fcn = ephemeris.glonass_frequency_channel + 7;
        const int64_t pos_x = static_cast<int64_t>(std::llround(ephemeris.glonass_position.x() / (kPow2Neg11 * 1e3)));
        const int64_t pos_y = static_cast<int64_t>(std::llround(ephemeris.glonass_position.y() / (kPow2Neg11 * 1e3)));
        const int64_t pos_z = static_cast<int64_t>(std::llround(ephemeris.glonass_position.z() / (kPow2Neg11 * 1e3)));
        const int64_t vel_x = static_cast<int64_t>(std::llround(ephemeris.glonass_velocity.x() / (kPow2Neg20 * 1e3)));
        const int64_t vel_y = static_cast<int64_t>(std::llround(ephemeris.glonass_velocity.y() / (kPow2Neg20 * 1e3)));
        const int64_t vel_z = static_cast<int64_t>(std::llround(ephemeris.glonass_velocity.z() / (kPow2Neg20 * 1e3)));
        const int64_t acc_x = static_cast<int64_t>(std::llround(ephemeris.glonass_acceleration.x() / (kPow2Neg30 * 1e3)));
        const int64_t acc_y = static_cast<int64_t>(std::llround(ephemeris.glonass_acceleration.y() / (kPow2Neg30 * 1e3)));
        const int64_t acc_z = static_cast<int64_t>(std::llround(ephemeris.glonass_acceleration.z() / (kPow2Neg30 * 1e3)));
        const int64_t gamn = static_cast<int64_t>(std::llround(ephemeris.glonass_gamn / kPow2Neg40));
        const int64_t taun = static_cast<int64_t>(std::llround(ephemeris.glonass_taun / kPow2Neg30));

        if (fcn < 0 || fcn > 31 ||
            pos_x < -((1 << 26) - 1) || pos_x > ((1 << 26) - 1) ||
            pos_y < -((1 << 26) - 1) || pos_y > ((1 << 26) - 1) ||
            pos_z < -((1 << 26) - 1) || pos_z > ((1 << 26) - 1) ||
            vel_x < -((1 << 23) - 1) || vel_x > ((1 << 23) - 1) ||
            vel_y < -((1 << 23) - 1) || vel_y > ((1 << 23) - 1) ||
            vel_z < -((1 << 23) - 1) || vel_z > ((1 << 23) - 1) ||
            acc_x < -((1 << 4) - 1) || acc_x > ((1 << 4) - 1) ||
            acc_y < -((1 << 4) - 1) || acc_y > ((1 << 4) - 1) ||
            acc_z < -((1 << 4) - 1) || acc_z > ((1 << 4) - 1) ||
            gamn < -((1 << 10) - 1) || gamn > ((1 << 10) - 1) ||
            taun < -((1 << 21) - 1) || taun > ((1 << 21) - 1)) {
            return RTCMMessage();
        }

        std::vector<uint8_t> payload(45, 0);
        int bit_pos = 0;
        writeUnsignedBits(payload, bit_pos, 12, 1020); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 6, ephemeris.satellite.prn); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 5, static_cast<uint64_t>(fcn)); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 4, 0); bit_pos += 4;
        writeUnsignedBits(payload, bit_pos, 5, static_cast<uint64_t>(tk_h)); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 6, static_cast<uint64_t>(tk_m)); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 1, static_cast<uint64_t>(tk_s)); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 1, ephemeris.health != 0 ? 1U : 0U); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 7, static_cast<uint64_t>(tb)); bit_pos += 7;
        writeSignMagnitudeBits(payload, bit_pos, 24, vel_x); bit_pos += 24;
        writeSignMagnitudeBits(payload, bit_pos, 27, pos_x); bit_pos += 27;
        writeSignMagnitudeBits(payload, bit_pos, 5, acc_x); bit_pos += 5;
        writeSignMagnitudeBits(payload, bit_pos, 24, vel_y); bit_pos += 24;
        writeSignMagnitudeBits(payload, bit_pos, 27, pos_y); bit_pos += 27;
        writeSignMagnitudeBits(payload, bit_pos, 5, acc_y); bit_pos += 5;
        writeSignMagnitudeBits(payload, bit_pos, 24, vel_z); bit_pos += 24;
        writeSignMagnitudeBits(payload, bit_pos, 27, pos_z); bit_pos += 27;
        writeSignMagnitudeBits(payload, bit_pos, 5, acc_z); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeSignMagnitudeBits(payload, bit_pos, 11, gamn); bit_pos += 11;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;
        writeSignMagnitudeBits(payload, bit_pos, 22, taun); bit_pos += 22;
        writeUnsignedBits(payload, bit_pos, 5, 0); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 5, ephemeris.glonass_age & 0x1FU); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 4, 0); bit_pos += 4;
        writeUnsignedBits(payload, bit_pos, 11, 0); bit_pos += 11;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 11, 0); bit_pos += 11;
        writeUnsignedBits(payload, bit_pos, 32, 0); bit_pos += 32;
        writeUnsignedBits(payload, bit_pos, 5, 0); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 22, 0); bit_pos += 22;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 7, 0); bit_pos += 7;

        RTCMMessage message;
        message.type = RTCMMessageType::RTCM_1020;
        message.length = static_cast<uint16_t>(payload.size());
        message.data = std::move(payload);
        message.valid = true;
        return message;
    }

    return RTCMMessage();
}

bool RTCMProcessor::decodeObservationMessage(const RTCMMessage& message, ObservationData& obs_data) {
    if (isGlonassMsmMessageType(message.type)) {
        const bool msm5 = isMsm5MessageType(message.type);
        const bool msm6 = isMsm6MessageType(message.type);
        const bool msm7 = isMsm7MessageType(message.type);
        if (message.data.size() * 8U < 187U) {
            return false;
        }

        int bit_pos = 0;
        const uint64_t message_number = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 12);
        bit_pos += 12;
        if (message_number != static_cast<uint16_t>(message.type)) {
            return false;
        }

        bit_pos += 12;  // reference station id
        const uint32_t epoch_field =
            static_cast<uint32_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 30));
        bit_pos += 30;
        bit_pos += 1;  // multiple message bit
        bit_pos += 3;  // issue of data station
        bit_pos += 7;  // session transmit time
        bit_pos += 2;  // clock steering
        bit_pos += 2;  // external clock
        bit_pos += 1;  // smoothing indicator
        bit_pos += 3;  // smoothing interval

        std::vector<uint8_t> sat_ids;
        std::vector<uint8_t> signal_ids;
        sat_ids.reserve(64);
        signal_ids.reserve(32);
        for (int sat = 1; sat <= 64; ++sat) {
            if (readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0) {
                sat_ids.push_back(static_cast<uint8_t>(sat));
            }
            bit_pos += 1;
        }
        for (int signal_id = 1; signal_id <= 32; ++signal_id) {
            if (readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0) {
                signal_ids.push_back(static_cast<uint8_t>(signal_id));
            }
            bit_pos += 1;
        }

        if (sat_ids.empty() || signal_ids.empty() || sat_ids.size() * signal_ids.size() > 64) {
            return false;
        }

        std::vector<bool> cell_mask(sat_ids.size() * signal_ids.size(), false);
        size_t ncell = 0;
        for (size_t i = 0; i < cell_mask.size(); ++i) {
            cell_mask[i] = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0;
            bit_pos += 1;
            if (cell_mask[i]) {
                ++ncell;
            }
        }

        const size_t required_bits =
            169U + cell_mask.size() + sat_ids.size() * ((msm5 || msm7) ? 36U : 18U) +
            ncell * (msm7 ? 80U : (msm5 ? 63U : (msm6 ? 65U : 48U)));
        if (message.data.size() * 8U < required_bits) {
            return false;
        }

        std::vector<double> rough_ranges(sat_ids.size(), 0.0);
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const uint32_t int_ms = static_cast<uint32_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
            bit_pos += 8;
            rough_ranges[sat_index] = int_ms == 255U ? 0.0 : static_cast<double>(int_ms) * kRTCMGpsPrUnitMeters;
        }
        std::vector<uint8_t> satellite_info(sat_ids.size(), 15U);
        if (msm5 || msm7) {
            for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
                satellite_info[sat_index] = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
                bit_pos += 4;
            }
        }
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const uint32_t mod_ms = static_cast<uint32_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
            bit_pos += 10;
            if (rough_ranges[sat_index] != 0.0) {
                rough_ranges[sat_index] += static_cast<double>(mod_ms) * kPow2Neg10 * kRTCMGpsPrUnitMeters;
            }
        }
        std::vector<double> rough_phase_range_rates(sat_ids.size(), 0.0);
        std::vector<bool> has_rough_phase_range_rate(sat_ids.size(), false);
        if (msm5 || msm7) {
            for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
                const int32_t rough_rate = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 14));
                bit_pos += 14;
                if (rough_rate != -8192) {
                    rough_phase_range_rates[sat_index] = static_cast<double>(rough_rate);
                    has_rough_phase_range_rate[sat_index] = true;
                }
            }
        }

        struct DecodedCellData {
            int32_t fine_pr = -16384;
            int32_t fine_cp = -2097152;
            uint8_t lock = 0;
            bool half = false;
            uint8_t cnr = 0;
            int16_t fine_rate = -16384;
            int32_t fine_pr_ext = -524288;
            int32_t fine_cp_ext = -8388608;
            uint16_t lock_ext = 0;
            uint16_t cnr_ext = 0;
        };
        std::vector<DecodedCellData> cells(ncell);
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.fine_pr_ext = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 20));
                bit_pos += 20;
            } else {
                cell.fine_pr = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 15));
                bit_pos += 15;
            }
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.fine_cp_ext = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 24));
                bit_pos += 24;
            } else {
                cell.fine_cp = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 22));
                bit_pos += 22;
            }
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.lock_ext = static_cast<uint16_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
                bit_pos += 10;
                cell.lock = static_cast<uint8_t>(cell.lock_ext == 0 ? 0U : 15U);
            } else {
                cell.lock = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
                bit_pos += 4;
            }
        }
        for (auto& cell : cells) {
            cell.half = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0;
            bit_pos += 1;
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.cnr_ext = static_cast<uint16_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
                bit_pos += 10;
            } else {
                cell.cnr = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
                bit_pos += 6;
            }
        }
        if (msm5 || msm7) {
            for (auto& cell : cells) {
                cell.fine_rate = static_cast<int16_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 15));
                bit_pos += 15;
            }
        }

        obs_data.clear();
        obs_data.time = glonassMsmEpochToGpst(epoch_field);
        if (has_reference_position_) {
            obs_data.receiver_position = reference_position_;
        }

        size_t cell_index = 0;
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const double rough_range = rough_ranges[sat_index];
            const SatelliteId sat_id(GNSSSystem::GLONASS, sat_ids[sat_index]);
            int frequency_channel = 0;
            bool has_frequency_channel =
                (msm5 || msm7) &&
                decodeGlonassMsmExtendedInfo(satellite_info[sat_index], frequency_channel);
            if (!has_frequency_channel) {
                const auto fcn_it = glonass_frequency_channels_.find(sat_id);
                has_frequency_channel = fcn_it != glonass_frequency_channels_.end();
                if (has_frequency_channel) {
                    frequency_channel = fcn_it->second;
                }
            } else {
                glonass_frequency_channels_[sat_id] = frequency_channel;
            }

            if (rough_range == 0.0) {
                for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                    if (cell_mask[sat_index * signal_ids.size() + sig_index]) {
                        ++cell_index;
                    }
                }
                continue;
            }
            for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                if (!cell_mask[sat_index * signal_ids.size() + sig_index]) {
                    continue;
                }
                if (cell_index >= cells.size()) {
                    return false;
                }

                const SignalType signal = decodeGlonassMsmSignal(signal_ids[sig_index]);
                const double wavelength =
                    has_frequency_channel ? glonassSignalWavelength(signal, frequency_channel) : 0.0;
                const auto& cell = cells[cell_index++];
                if (signal == SignalType::SIGNAL_TYPE_COUNT) {
                    continue;
                }

                Observation obs(sat_id, signal);
                if (has_frequency_channel) {
                    obs.has_glonass_frequency_channel = true;
                    obs.glonass_frequency_channel = frequency_channel;
                }
                if (msm6 || msm7) {
                    if (cell.fine_pr_ext != -524288) {
                        obs.has_pseudorange = true;
                        obs.pseudorange =
                            rough_range + static_cast<double>(cell.fine_pr_ext) * kPow2Neg29 * kRTCMGpsPrUnitMeters;
                    }
                } else if (cell.fine_pr != -16384) {
                    obs.has_pseudorange = true;
                    obs.pseudorange =
                        rough_range + static_cast<double>(cell.fine_pr) * kPow2Neg24 * kRTCMGpsPrUnitMeters;
                }
                if (msm6 || msm7) {
                    if (cell.fine_cp_ext != -8388608 && wavelength > 0.0) {
                        obs.has_carrier_phase = true;
                        obs.carrier_phase =
                            (rough_range + static_cast<double>(cell.fine_cp_ext) * kPow2Neg31 * kRTCMGpsPrUnitMeters) /
                            wavelength;
                    }
                } else if (cell.fine_cp != -2097152 && wavelength > 0.0) {
                    obs.has_carrier_phase = true;
                    obs.carrier_phase =
                        (rough_range + static_cast<double>(cell.fine_cp) * kPow2Neg29 * kRTCMGpsPrUnitMeters) /
                        wavelength;
                }
                if ((msm5 || msm7) && cell.fine_rate != -16384 && wavelength > 0.0) {
                    const double rate =
                        (has_rough_phase_range_rate[sat_index] ? rough_phase_range_rates[sat_index] : 0.0) +
                        static_cast<double>(cell.fine_rate) * 1e-4;
                    obs.has_doppler = true;
                    obs.doppler = -rate / wavelength;
                }
                obs.loss_of_lock = cell.lock == 0;
                obs.lli = obs.loss_of_lock ? 1U : 0U;
                if (cell.half) {
                    obs.lli |= 0x02U;
                }
                obs.snr = (msm6 || msm7) ? static_cast<double>(cell.cnr_ext) * 0.0625
                                         : static_cast<double>(cell.cnr);
                obs.signal_strength = static_cast<int>(std::lround(obs.snr / 6.0));
                obs.valid = obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler;
                if (obs.valid) {
                    obs_data.addObservation(obs);
                }
            }
        }

        return !obs_data.isEmpty();
    }

    if (isFixedFrequencyMsmMessageType(message.type)) {
        const bool msm5 = isMsm5MessageType(message.type);
        const bool msm6 = isMsm6MessageType(message.type);
        const bool msm7 = isMsm7MessageType(message.type);
        const GNSSSystem system = fixedFrequencyMsmSystem(message.type);
        if (system == GNSSSystem::UNKNOWN) {
            return false;
        }

        if (message.data.size() * 8U < 187U) {
            return false;
        }

        int bit_pos = 0;
        const uint64_t message_number = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 12);
        bit_pos += 12;
        if (message_number != static_cast<uint16_t>(message.type)) {
            return false;
        }

        bit_pos += 12;  // reference station id
        const uint32_t epoch_ms =
            static_cast<uint32_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 30));
        bit_pos += 30;
        bit_pos += 1;  // multiple message bit
        bit_pos += 3;  // issue of data station
        bit_pos += 7;  // session transmit time
        bit_pos += 2;  // clock steering
        bit_pos += 2;  // external clock
        bit_pos += 1;  // smoothing indicator
        bit_pos += 3;  // smoothing interval

        std::vector<uint8_t> sat_ids;
        std::vector<uint8_t> signal_ids;
        sat_ids.reserve(64);
        signal_ids.reserve(32);
        for (int sat = 1; sat <= 64; ++sat) {
            if (readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0) {
                sat_ids.push_back(static_cast<uint8_t>(sat));
            }
            bit_pos += 1;
        }
        for (int signal_id = 1; signal_id <= 32; ++signal_id) {
            if (readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0) {
                signal_ids.push_back(static_cast<uint8_t>(signal_id));
            }
            bit_pos += 1;
        }

        if (sat_ids.empty() || signal_ids.empty() || sat_ids.size() * signal_ids.size() > 64) {
            return false;
        }

        std::vector<bool> cell_mask(sat_ids.size() * signal_ids.size(), false);
        size_t ncell = 0;
        for (size_t i = 0; i < cell_mask.size(); ++i) {
            cell_mask[i] = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0;
            bit_pos += 1;
            if (cell_mask[i]) {
                ++ncell;
            }
        }

        const size_t required_bits =
            169U + cell_mask.size() + sat_ids.size() * ((msm5 || msm7) ? 36U : 18U) +
            ncell * (msm7 ? 80U : (msm5 ? 63U : (msm6 ? 65U : 48U)));
        if (message.data.size() * 8U < required_bits) {
            return false;
        }

        std::vector<double> rough_ranges(sat_ids.size(), 0.0);
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const uint32_t int_ms = static_cast<uint32_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
            bit_pos += 8;
            rough_ranges[sat_index] = int_ms == 255U ? 0.0 : static_cast<double>(int_ms) * kRTCMGpsPrUnitMeters;
        }
        if (msm5 || msm7) {
            for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
                (void)readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4);
                bit_pos += 4;
            }
        }
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const uint32_t mod_ms = static_cast<uint32_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
            bit_pos += 10;
            if (rough_ranges[sat_index] != 0.0) {
                rough_ranges[sat_index] += static_cast<double>(mod_ms) * kPow2Neg10 * kRTCMGpsPrUnitMeters;
            }
        }
        std::vector<double> rough_phase_range_rates(sat_ids.size(), 0.0);
        std::vector<bool> has_rough_phase_range_rate(sat_ids.size(), false);
        if (msm5 || msm7) {
            for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
                const int32_t rough_rate = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 14));
                bit_pos += 14;
                if (rough_rate != -8192) {
                    rough_phase_range_rates[sat_index] = static_cast<double>(rough_rate);
                    has_rough_phase_range_rate[sat_index] = true;
                }
            }
        }

        struct DecodedCellData {
            int32_t fine_pr = -16384;
            int32_t fine_cp = -2097152;
            uint8_t lock = 0;
            bool half = false;
            uint8_t cnr = 0;
            int16_t fine_rate = -16384;
            int32_t fine_pr_ext = -524288;
            int32_t fine_cp_ext = -8388608;
            uint16_t lock_ext = 0;
            uint16_t cnr_ext = 0;
        };
        std::vector<DecodedCellData> cells(ncell);
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.fine_pr_ext = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 20));
                bit_pos += 20;
            } else {
                cell.fine_pr = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 15));
                bit_pos += 15;
            }
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.fine_cp_ext = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 24));
                bit_pos += 24;
            } else {
                cell.fine_cp = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 22));
                bit_pos += 22;
            }
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.lock_ext = static_cast<uint16_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
                bit_pos += 10;
                cell.lock = static_cast<uint8_t>(cell.lock_ext == 0 ? 0U : 15U);
            } else {
                cell.lock = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
                bit_pos += 4;
            }
        }
        for (auto& cell : cells) {
            cell.half = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0;
            bit_pos += 1;
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.cnr_ext = static_cast<uint16_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
                bit_pos += 10;
            } else {
                cell.cnr = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
                bit_pos += 6;
            }
        }
        if (msm5 || msm7) {
            for (auto& cell : cells) {
                cell.fine_rate = static_cast<int16_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 15));
                bit_pos += 15;
            }
        }

        obs_data.clear();
        obs_data.time = fixedFrequencyMsmEpochToTime(message.type, epoch_ms);
        if (has_reference_position_) {
            obs_data.receiver_position = reference_position_;
        }

        size_t cell_index = 0;
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const double rough_range = rough_ranges[sat_index];
            if (rough_range == 0.0) {
                for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                    if (cell_mask[sat_index * signal_ids.size() + sig_index]) {
                        ++cell_index;
                    }
                }
                continue;
            }
            for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                if (!cell_mask[sat_index * signal_ids.size() + sig_index]) {
                    continue;
                }
                if (cell_index >= cells.size()) {
                    return false;
                }

                const SignalType signal = decodeFixedFrequencyMsmSignal(system, signal_ids[sig_index]);
                const double wavelength = signalWavelength(signal);
                const auto& cell = cells[cell_index++];
                if (signal == SignalType::SIGNAL_TYPE_COUNT) {
                    continue;
                }

                Observation obs(SatelliteId(system, sat_ids[sat_index]), signal);
                if (msm6 || msm7) {
                    if (cell.fine_pr_ext != -524288) {
                        obs.has_pseudorange = true;
                        obs.pseudorange =
                            rough_range + static_cast<double>(cell.fine_pr_ext) * kPow2Neg29 * kRTCMGpsPrUnitMeters;
                    }
                } else if (cell.fine_pr != -16384) {
                    obs.has_pseudorange = true;
                    obs.pseudorange =
                        rough_range + static_cast<double>(cell.fine_pr) * kPow2Neg24 * kRTCMGpsPrUnitMeters;
                }
                if (msm6 || msm7) {
                    if (cell.fine_cp_ext != -8388608 && wavelength > 0.0) {
                        obs.has_carrier_phase = true;
                        obs.carrier_phase =
                            (rough_range + static_cast<double>(cell.fine_cp_ext) * kPow2Neg31 * kRTCMGpsPrUnitMeters) /
                            wavelength;
                    }
                } else if (cell.fine_cp != -2097152 && wavelength > 0.0) {
                    obs.has_carrier_phase = true;
                    obs.carrier_phase =
                        (rough_range + static_cast<double>(cell.fine_cp) * kPow2Neg29 * kRTCMGpsPrUnitMeters) /
                        wavelength;
                }
                if ((msm5 || msm7) && cell.fine_rate != -16384 && wavelength > 0.0) {
                    const double rate =
                        (has_rough_phase_range_rate[sat_index] ? rough_phase_range_rates[sat_index] : 0.0) +
                        static_cast<double>(cell.fine_rate) * 1e-4;
                    obs.has_doppler = true;
                    obs.doppler = -rate / wavelength;
                }
                obs.loss_of_lock = cell.lock == 0;
                obs.lli = obs.loss_of_lock ? 1U : 0U;
                if (cell.half) {
                    obs.lli |= 0x02U;
                }
                obs.snr = (msm6 || msm7) ? static_cast<double>(cell.cnr_ext) * 0.0625
                                         : static_cast<double>(cell.cnr);
                obs.signal_strength = static_cast<int>(std::lround(obs.snr / 6.0));
                obs.valid = obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler;
                if (obs.valid) {
                    obs_data.addObservation(obs);
                }
            }
        }

        return !obs_data.isEmpty();
    }

    const bool extended = message.type == RTCMMessageType::RTCM_1004;
    if (!extended && message.type != RTCMMessageType::RTCM_1003) {
        return false;
    }
    if (message.data.size() * 8U < 64U) {
        return false;
    }

    int bit_pos = 0;
    const uint64_t message_number = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 12);
    bit_pos += 12;
    if (message_number != static_cast<uint16_t>(message.type)) {
        return false;
    }

    bit_pos += 12;  // reference station id
    const uint32_t epoch_ms =
        static_cast<uint32_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 30));
    bit_pos += 30;
    bit_pos += 1;  // synchronous GNSS flag
    const uint64_t num_sats = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5);
    bit_pos += 5;
    bit_pos += 1;  // smoothing indicator
    bit_pos += 3;  // smoothing interval

    const size_t bits_per_sat = extended ? 125U : 109U;
    const size_t required_bits = 64U + static_cast<size_t>(num_sats) * bits_per_sat;
    if (message.data.size() * 8U < required_bits) {
        return false;
    }

    obs_data.clear();
    obs_data.time = rtcm_utils::rtcmTimeToGPSTime(epoch_ms, 0);
    if (has_reference_position_) {
        obs_data.receiver_position = reference_position_;
    }

    for (uint64_t sat_index = 0; sat_index < num_sats; ++sat_index) {
        const uint8_t prn =
            static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        bit_pos += 6;
        const uint64_t l1_code = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1);
        bit_pos += 1;
        const uint64_t l1_pr_units = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 24);
        bit_pos += 24;
        const int64_t l1_phase_delta_units = readSignedBits(message.data.data(), message.data.size(), bit_pos, 20);
        bit_pos += 20;
        const uint8_t l1_lock =
            static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 7));
        bit_pos += 7;
        const uint64_t ambiguity = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8);
        bit_pos += 8;
        uint8_t l1_cnr = 0;
        if (extended) {
            l1_cnr =
                static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
            bit_pos += 8;
        }
        const uint64_t l2_code = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 2);
        bit_pos += 2;
        const int64_t l2_pr_delta_units = readSignedBits(message.data.data(), message.data.size(), bit_pos, 14);
        bit_pos += 14;
        const int64_t l2_phase_delta_units = readSignedBits(message.data.data(), message.data.size(), bit_pos, 20);
        bit_pos += 20;
        const uint8_t l2_lock =
            static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 7));
        bit_pos += 7;
        uint8_t l2_cnr = 0;
        if (extended) {
            l2_cnr =
                static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
            bit_pos += 8;
        }

        if (prn == 0 || prn > 32) {
            continue;
        }

        const double p1 = static_cast<double>(ambiguity) * kRTCMGpsPrUnitMeters +
                          static_cast<double>(l1_pr_units) * kRTCMGpsPseudorangeResolution;
        const double p2 = p1 + static_cast<double>(l2_pr_delta_units) * kRTCMGpsPseudorangeResolution;
        const double l1_range = p1 + static_cast<double>(l1_phase_delta_units) * kRTCMGpsPhaseResolution;
        const double l2_range = p1 + static_cast<double>(l2_phase_delta_units) * kRTCMGpsPhaseResolution;

        Observation l1_obs(SatelliteId(GNSSSystem::GPS, prn), decodeGpsL1Signal(l1_code));
        l1_obs.has_pseudorange = true;
        l1_obs.has_carrier_phase = true;
        l1_obs.pseudorange = p1;
        l1_obs.carrier_phase = l1_range / constants::GPS_L1_WAVELENGTH;
        l1_obs.snr = static_cast<double>(l1_cnr) * kRTCMCn0Resolution;
        l1_obs.signal_strength = static_cast<int>(l1_cnr);
        l1_obs.loss_of_lock = l1_lock == 0;
        l1_obs.lli = l1_obs.loss_of_lock ? 1U : 0U;
        l1_obs.valid = true;
        obs_data.addObservation(l1_obs);

        Observation l2_obs(SatelliteId(GNSSSystem::GPS, prn), decodeGpsL2Signal(l2_code));
        l2_obs.has_pseudorange = true;
        l2_obs.has_carrier_phase = true;
        l2_obs.pseudorange = p2;
        l2_obs.carrier_phase = l2_range / constants::GPS_L2_WAVELENGTH;
        l2_obs.snr = static_cast<double>(l2_cnr) * kRTCMCn0Resolution;
        l2_obs.signal_strength = static_cast<int>(l2_cnr);
        l2_obs.loss_of_lock = l2_lock == 0;
        l2_obs.lli = l2_obs.loss_of_lock ? 1U : 0U;
        l2_obs.valid = true;
        obs_data.addObservation(l2_obs);
    }

    return !obs_data.isEmpty();
}

bool RTCMProcessor::decodeEphemerisMessage(const RTCMMessage& message, NavigationData& nav_data) {
    int bit_pos = 0;
    const uint64_t message_number = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 12);
    bit_pos += 12;

    if (message.type == RTCMMessageType::RTCM_1019) {
        if (message.data.size() * 8U < 488U || message_number != 1019U) {
            return false;
        }

        const uint8_t prn = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        bit_pos += 6;
        if (prn == 0 || prn > 32) {
            return false;
        }

        Ephemeris eph;
        eph.satellite = SatelliteId(GNSSSystem::GPS, prn);
        eph.week = static_cast<uint16_t>(adjustGpsWeek(static_cast<uint16_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10))));
        bit_pos += 10;
        eph.ura = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
        eph.sv_accuracy = uraMetersFromIndex(eph.ura);
        bit_pos += 4;
        bit_pos += 2;  // code on L2
        eph.idot = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 14)) *
                   kPow2Neg43 * kSemiCircleToRadians;
        eph.i_dot = eph.idot;
        bit_pos += 14;
        eph.iode = static_cast<uint16_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
        bit_pos += 8;
        const double toc = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * 16.0;
        bit_pos += 16;
        eph.af2 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 8)) * kPow2Neg55;
        bit_pos += 8;
        eph.af1 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg43;
        bit_pos += 16;
        eph.af0 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 22)) * kPow2Neg31;
        bit_pos += 22;
        eph.iodc = static_cast<uint16_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
        bit_pos += 10;
        eph.crs = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg5;
        bit_pos += 16;
        eph.delta_n = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) *
                      kPow2Neg43 * kSemiCircleToRadians;
        bit_pos += 16;
        eph.m0 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 32)) *
                 kPow2Neg31 * kSemiCircleToRadians;
        bit_pos += 32;
        eph.cuc = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg29;
        bit_pos += 16;
        eph.e = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 32)) * kPow2Neg33;
        bit_pos += 32;
        eph.cus = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg29;
        bit_pos += 16;
        eph.sqrt_a = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 32)) * kPow2Neg19;
        bit_pos += 32;
        eph.toes = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * 16.0;
        bit_pos += 16;
        eph.cic = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg29;
        bit_pos += 16;
        eph.omega0 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 32)) *
                     kPow2Neg31 * kSemiCircleToRadians;
        bit_pos += 32;
        eph.cis = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg29;
        bit_pos += 16;
        eph.i0 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 32)) *
                 kPow2Neg31 * kSemiCircleToRadians;
        bit_pos += 32;
        eph.crc = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg5;
        bit_pos += 16;
        eph.omega = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 32)) *
                    kPow2Neg31 * kSemiCircleToRadians;
        bit_pos += 32;
        eph.omega_dot = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 24)) *
                        kPow2Neg43 * kSemiCircleToRadians;
        bit_pos += 24;
        eph.tgd = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 8)) * kPow2Neg31;
        bit_pos += 8;
        eph.health = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        eph.sv_health = static_cast<double>(eph.health);
        bit_pos += 6;
        bit_pos += 1;
        bit_pos += 1;

        eph.toe = GNSSTime(eph.week, eph.toes);
        eph.toc = GNSSTime(eph.week, toc);
        eph.valid = true;
        nav_data.addEphemeris(eph);
        return true;
    }

    if (message.type == RTCMMessageType::RTCM_1020) {
        if (message.data.size() * 8U < 360U || message_number != 1020U) {
            return false;
        }

        const uint8_t prn = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        bit_pos += 6;
        if (prn == 0 || prn > 32) {
            return false;
        }

        Ephemeris eph;
        eph.satellite = SatelliteId(GNSSSystem::GLONASS, prn);
        eph.glonass_frequency_channel = static_cast<int>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5)) - 7;
        bit_pos += 5;
        bit_pos += 4;
        const double tk_h = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5));
        bit_pos += 5;
        const double tk_m = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        bit_pos += 6;
        const double tk_s = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1)) * 30.0;
        bit_pos += 1;
        const uint8_t bn = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1));
        bit_pos += 1;
        bit_pos += 1;
        const uint8_t tb = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 7));
        bit_pos += 7;
        eph.glonass_velocity.x() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 24)) * kPow2Neg20 * 1e3;
        bit_pos += 24;
        eph.glonass_position.x() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 27)) * kPow2Neg11 * 1e3;
        bit_pos += 27;
        eph.glonass_acceleration.x() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 5)) * kPow2Neg30 * 1e3;
        bit_pos += 5;
        eph.glonass_velocity.y() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 24)) * kPow2Neg20 * 1e3;
        bit_pos += 24;
        eph.glonass_position.y() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 27)) * kPow2Neg11 * 1e3;
        bit_pos += 27;
        eph.glonass_acceleration.y() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 5)) * kPow2Neg30 * 1e3;
        bit_pos += 5;
        eph.glonass_velocity.z() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 24)) * kPow2Neg20 * 1e3;
        bit_pos += 24;
        eph.glonass_position.z() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 27)) * kPow2Neg11 * 1e3;
        bit_pos += 27;
        eph.glonass_acceleration.z() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 5)) * kPow2Neg30 * 1e3;
        bit_pos += 5;
        bit_pos += 1;
        eph.glonass_gamn = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 11)) * kPow2Neg40;
        bit_pos += 11;
        bit_pos += 3;
        eph.glonass_taun = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 22)) * kPow2Neg30;
        bit_pos += 22;
        bit_pos += 5;
        eph.glonass_age = static_cast<int>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5));
        bit_pos += 5;
        bit_pos += 1;
        bit_pos += 4;
        bit_pos += 11;
        bit_pos += 2;
        bit_pos += 1;
        bit_pos += 11;
        bit_pos += 32;
        bit_pos += 5;
        bit_pos += 22;
        bit_pos += 1;
        bit_pos += 7;

        eph.health = bn;
        eph.sv_health = static_cast<double>(bn);
        eph.iode = tb & 0x7FU;
        const GNSSTime current_utc = gpstToUtcApprox(currentGpstApprox());
        const GNSSTime tof_utc = alignUtcTimeOfDay(tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0, current_utc);
        const GNSSTime toe_utc = alignUtcTimeOfDay(static_cast<double>(tb) * 900.0 - 10800.0, current_utc);
        eph.tof = utcToGpstApprox(tof_utc);
        eph.toe = utcToGpstApprox(toe_utc);
        eph.week = static_cast<uint16_t>(eph.toe.week);
        eph.valid = true;
        glonass_frequency_channels_[eph.satellite] = eph.glonass_frequency_channel;
        nav_data.addEphemeris(eph);
        return true;
    }

    return false;
}

bool RTCMProcessor::decodeObservationData(const RTCMMessage& message, ObservationData& obs_data) {
    return decodeObservationMessage(message, obs_data);
}

bool RTCMProcessor::decodeNavigationData(const RTCMMessage& message, NavigationData& nav_data) {
    return decodeEphemerisMessage(message, nav_data);
}

bool RTCMProcessor::decodeSSRCorrections(const RTCMMessage& message,
                                         std::vector<RTCMSSRCorrection>& corrections) {
    corrections.clear();
    if (!message.valid || message.data.empty() || !isSupportedSsrMessageType(message.type)) {
        return false;
    }

    const uint64_t message_number =
        readUnsignedBits(message.data.data(), message.data.size(), 0, 12);
    if (message_number != static_cast<uint16_t>(message.type)) {
        return false;
    }

    const auto descriptor = ssrDescriptorForMessageType(message.type);
    if (descriptor.system == GNSSSystem::UNKNOWN) {
        return false;
    }

    SsrHeader header;
    const bool orbit_message =
        isSsrOrbitMessageType(message.type) || isSsrCombinedMessageType(message.type);
    const bool clock_message =
        isSsrClockMessageType(message.type) || isSsrCombinedMessageType(message.type);
    const bool code_bias_message = isSsrCodeBiasMessageType(message.type);
    const bool ura_message = isSsrUraMessageType(message.type);
    const bool high_rate_clock_message = isSsrHighRateClockMessageType(message.type);
    const bool header_ok = orbit_message
        ? decodeSsrOrbitHeader(message, descriptor, header)
        : decodeSsrClockHeader(message, descriptor, header);
    if (!header_ok) {
        return false;
    }

    int bit_pos = header.bit_pos;
    const size_t data_bits = message.data.size() * 8U;
    corrections.reserve(static_cast<size_t>(header.satellite_count));

    for (int index = 0; index < header.satellite_count; ++index) {
        RTCMSSRCorrection correction;
        correction.time = header.time;
        correction.update_interval_seconds = header.update_interval_seconds;
        correction.issue_of_data = static_cast<uint8_t>(header.issue_of_data);
        correction.provider_id = header.provider_id;
        correction.solution_id = header.solution_id;
        correction.reference_datum = header.refd != 0;

        if (orbit_message) {
            const int required_bits =
                descriptor.prn_bits + descriptor.iode_bits + descriptor.iodcrc_bits + 121;
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }

            const uint64_t raw_prn = readUnsignedBits(
                message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
            bit_pos += descriptor.prn_bits;
            const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
            if (prn == 0) {
                return false;
            }
            correction.satellite = SatelliteId(descriptor.system, prn);
            correction.iode = static_cast<int>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, descriptor.iode_bits));
            bit_pos += descriptor.iode_bits;
            if (descriptor.iodcrc_bits > 0) {
                correction.iodcrc = static_cast<int>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, descriptor.iodcrc_bits));
                bit_pos += descriptor.iodcrc_bits;
            }
            correction.orbit_delta_rac_m.x() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 22)) * 1e-4;
            bit_pos += 22;
            correction.orbit_delta_rac_m.y() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 20)) * 4e-4;
            bit_pos += 20;
            correction.orbit_delta_rac_m.z() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 20)) * 4e-4;
            bit_pos += 20;
            correction.orbit_rate_rac_mps.x() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 21)) * 1e-6;
            bit_pos += 21;
            correction.orbit_rate_rac_mps.y() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 19)) * 4e-6;
            bit_pos += 19;
            correction.orbit_rate_rac_mps.z() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 19)) * 4e-6;
            bit_pos += 19;
            correction.has_orbit = true;
        }

        if (clock_message) {
            const int required_bits = orbit_message ? 70 : (descriptor.prn_bits + 70);
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }

            if (!orbit_message) {
                const uint64_t raw_prn = readUnsignedBits(
                    message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
                bit_pos += descriptor.prn_bits;
                const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
                if (prn == 0) {
                    return false;
                }
                correction.satellite = SatelliteId(descriptor.system, prn);
            }
            correction.clock_delta_poly.x() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 22)) * 1e-4;
            bit_pos += 22;
            correction.clock_delta_poly.y() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 21)) * 1e-6;
            bit_pos += 21;
            correction.clock_delta_poly.z() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 27)) * 2e-8;
            bit_pos += 27;
            correction.has_clock = true;
        }

        if (code_bias_message) {
            const int required_bits = descriptor.prn_bits + 5;
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }
            const uint64_t raw_prn = readUnsignedBits(
                message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
            bit_pos += descriptor.prn_bits;
            const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
            if (prn == 0) {
                return false;
            }
            correction.satellite = SatelliteId(descriptor.system, prn);

            const uint8_t num_biases = static_cast<uint8_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5));
            bit_pos += 5;
            for (uint8_t bias_index = 0; bias_index < num_biases; ++bias_index) {
                if (static_cast<size_t>(bit_pos + 19) > data_bits) {
                    return false;
                }
                const uint8_t signal_id = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5));
                bit_pos += 5;
                const double bias_m = static_cast<double>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 14)) * 0.01;
                bit_pos += 14;
                correction.code_bias_m[signal_id] = bias_m;
            }
            correction.has_code_bias = !correction.code_bias_m.empty();
        }

        if (ura_message) {
            const int required_bits = descriptor.prn_bits + 6;
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }
            const uint64_t raw_prn = readUnsignedBits(
                message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
            bit_pos += descriptor.prn_bits;
            const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
            if (prn == 0) {
                return false;
            }
            correction.satellite = SatelliteId(descriptor.system, prn);
            correction.ura_index = static_cast<uint8_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
            bit_pos += 6;
            correction.ura_sigma_m = uraMetersFromSsrIndex(correction.ura_index);
            correction.has_ura = true;
        }

        if (high_rate_clock_message) {
            const int required_bits = descriptor.prn_bits + 22;
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }
            const uint64_t raw_prn = readUnsignedBits(
                message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
            bit_pos += descriptor.prn_bits;
            const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
            if (prn == 0) {
                return false;
            }
            correction.satellite = SatelliteId(descriptor.system, prn);
            correction.high_rate_clock_m = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 22)) * 1e-4;
            bit_pos += 22;
            correction.has_high_rate_clock = true;
        }

        corrections.push_back(correction);
    }

    return !corrections.empty();
}


uint32_t RTCMProcessor::calculateCRC24(const uint8_t* data, size_t length) {
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t table_index = static_cast<uint8_t>(((crc >> 16) ^ data[i]) & 0xFFU);
        crc = (crc << 8) ^ crc24q_table[table_index];
    }
    return crc & 0x00FFFFFF;
}


int64_t RTCMProcessor::getBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    return static_cast<int64_t>(readUnsignedBits(data, data_size, bit_pos, num_bits));
}

void RTCMProcessor::setBits(uint8_t* data, int pos, int len, uint32_t value) {
    if (!data || len <= 0 || len > 32 || pos < 0) {
        return;
    }
    for (int i = 0; i < len; ++i) {
        const int bit_pos = pos + len - 1 - i;
        const int byte_index = bit_pos / 8;
        const int bit_in_byte = 7 - (bit_pos % 8);
        const uint8_t mask = static_cast<uint8_t>(1U << bit_in_byte);
        if ((value >> i) & 0x01U) {
            data[byte_index] |= mask;
        } else {
            data[byte_index] &= static_cast<uint8_t>(~mask);
        }
    }
}

bool RTCMProcessor::parseHeader(const uint8_t* data, RTCMMessageType& type, uint16_t& length) {
    if (!data || data[0] != kRTCMPreamble) {
        return false;
    }

    length = static_cast<uint16_t>(((data[1] & 0x03U) << 8) | data[2]);
    if (length < 2) {
        type = RTCMMessageType::RTCM_UNKNOWN;
        return false;
    }

    type = static_cast<RTCMMessageType>(getBits(data + 3, length, 0, 12));
    return true;
}

std::vector<uint8_t> RTCMProcessor::extractPayload(const uint8_t* data, size_t total_length) {
    if (!data || total_length < 6) {
        return {};
    }
    const size_t payload_length = total_length - 6;
    return std::vector<uint8_t>(data + 3, data + 3 + payload_length);
}

// RTCMReader implementation
RTCMReader::~RTCMReader() {
    close();
}

bool RTCMReader::open(const std::string& source) {
    close();
    if (source.rfind("ntrip://", 0) == 0 || source.rfind("http://", 0) == 0 ||
        source.rfind("https://", 0) == 0) {
        return readFromNetwork(source);
    }
    if (isTcpSource(source)) {
        return readFromTcp(source);
    }
    if (source.rfind("serial://", 0) == 0) {
        return readFromSerial(source);
    }
    std::error_code ec;
    const auto status = std::filesystem::status(source, ec);
    if (!ec && std::filesystem::is_character_file(status)) {
        return readFromSerial(source);
    }
    return readFromFile(source);
}

void RTCMReader::close() {
    if (ntrip_client_) {
        ntrip_client_->disconnect();
        delete ntrip_client_;
        ntrip_client_ = nullptr;
    }
#ifndef _WIN32
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
    if (tcp_fd_ >= 0) {
        ::close(tcp_fd_);
        tcp_fd_ = -1;
    }
#endif
    buffer_.clear();
    buffer_pos_ = 0;
    is_open_ = false;
    processor_.clear();
    processor_.resetStats();
}

bool RTCMReader::readMessage(RTCMMessage& message) {
    if (!is_open_) {
        return false;
    }

    if (ntrip_client_) {
        return ntrip_client_->readMessage(message);
    }

    auto tryDecodeBufferedMessage = [&]() -> bool {
        while (buffer_pos_ < buffer_.size()) {
            if (buffer_[buffer_pos_] != kRTCMPreamble) {
                ++buffer_pos_;
                continue;
            }

            if (buffer_pos_ + 5 > buffer_.size()) {
                return false;
            }

            const uint16_t payload_length =
                static_cast<uint16_t>(((buffer_[buffer_pos_ + 1] & 0x03U) << 8) | buffer_[buffer_pos_ + 2]);
            const size_t total_length = 3U + payload_length + 3U;
            if (buffer_pos_ + total_length > buffer_.size()) {
                return false;
            }

            auto decoded = processor_.decode(buffer_.data() + buffer_pos_, total_length);
            if (!decoded.empty()) {
                message = std::move(decoded.front());
                buffer_pos_ += total_length;
                return true;
            }

            ++buffer_pos_;
        }
        return false;
    };

    const int stream_fd =
#ifndef _WIN32
        (serial_fd_ >= 0 ? serial_fd_ : tcp_fd_);
#else
        -1;
#endif
    if (stream_fd < 0) {
        return tryDecodeBufferedMessage();
    }

#ifndef _WIN32
    std::vector<uint8_t> chunk(4096);
    while (true) {
        if (tryDecodeBufferedMessage()) {
            return true;
        }

        if (buffer_pos_ > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(buffer_pos_));
            buffer_pos_ = 0;
        }

        const ssize_t count = ::read(stream_fd, chunk.data(), chunk.size());
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (count == 0) {
            return tryDecodeBufferedMessage();
        }
        buffer_.insert(buffer_.end(), chunk.begin(), chunk.begin() + count);
    }
#else
    return false;
#endif
}

bool RTCMReader::readFromFile(const std::string& filename) {
    std::ifstream input(filename, std::ios::binary);
    if (!input) {
        return false;
    }

    buffer_.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    buffer_pos_ = 0;
    is_open_ = true;
    return true;
}

bool RTCMReader::readFromNetwork(const std::string& url) {
    ntrip_client_ = new NTRIPClient();
    if (!ntrip_client_->connect(url)) {
        delete ntrip_client_;
        ntrip_client_ = nullptr;
        return false;
    }

    is_open_ = true;
    return true;
}

bool RTCMReader::readFromSerial(const std::string& source) {
#ifndef _WIN32
    const std::string path = resolveSerialPath(source);
    const int baud = parseSerialBaud(source);
    const int fd = ::open(path.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        return false;
    }
    try {
        if (!configureSerialPort(fd, baud)) {
            ::close(fd);
            return false;
        }
    } catch (const std::exception&) {
        ::close(fd);
        return false;
    }

    serial_fd_ = fd;
    is_open_ = true;
    return true;
#else
    (void)source;
    return false;
#endif
}

bool RTCMReader::readFromTcp(const std::string& source) {
#ifndef _WIN32
    try {
        const int fd = connectTcpSocket(source);
        if (fd < 0) {
            return false;
        }
        tcp_fd_ = fd;
        is_open_ = true;
        return true;
    } catch (const std::exception&) {
        return false;
    }
#else
    (void)source;
    return false;
#endif
}

RTCMProcessor::RTCMStats RTCMReader::getStats() const {
    if (ntrip_client_) {
        return ntrip_client_->getStats();
    }
    return processor_.getStats();
}

// rtcm_utils implementation
namespace rtcm_utils {

uint32_t gpsTimeToRTCMTime(const GNSSTime& gps_time) {
    if (!std::isfinite(gps_time.tow)) {
        return 0;
    }

    const double tow_ms = std::round(gps_time.tow * 1000.0);
    const double wrapped = std::fmod(std::fmod(tow_ms, static_cast<double>(kMillisecondsPerGPSWeek)) +
                                         static_cast<double>(kMillisecondsPerGPSWeek),
                                     static_cast<double>(kMillisecondsPerGPSWeek));
    return static_cast<uint32_t>(wrapped);
}

GNSSTime rtcmTimeToGPSTime(uint32_t rtcm_time, uint16_t week) {
    return GNSSTime(static_cast<int>(week),
                    static_cast<double>(rtcm_time % kMillisecondsPerGPSWeek) * 1e-3);
}

std::string getMessageTypeName(RTCMMessageType type) {
    switch (type) {
        case RTCMMessageType::RTCM_1001: return "GPS L1 RTK Observables";
        case RTCMMessageType::RTCM_1002: return "GPS Extended L1 RTK Observables";
        case RTCMMessageType::RTCM_1003: return "GPS L1/L2 RTK Observables";
        case RTCMMessageType::RTCM_1004: return "GPS Extended L1/L2 RTK Observables";
        case RTCMMessageType::RTCM_1005: return "Reference Station ARP";
        case RTCMMessageType::RTCM_1006: return "Reference Station ARP With Height";
        case RTCMMessageType::RTCM_1007: return "Antenna Descriptor";
        case RTCMMessageType::RTCM_1008: return "Antenna Descriptor With Serial";
        case RTCMMessageType::RTCM_1009: return "GLONASS L1 RTK Observables";
        case RTCMMessageType::RTCM_1010: return "GLONASS Extended L1 RTK Observables";
        case RTCMMessageType::RTCM_1011: return "GLONASS L1/L2 RTK Observables";
        case RTCMMessageType::RTCM_1012: return "GLONASS Extended L1/L2 RTK Observables";
        case RTCMMessageType::RTCM_1019: return "GPS Ephemeris";
        case RTCMMessageType::RTCM_1020: return "GLONASS Ephemeris";
        case RTCMMessageType::RTCM_1033: return "Receiver And Antenna Descriptor";
        case RTCMMessageType::RTCM_1057: return "GPS SSR Orbit Correction";
        case RTCMMessageType::RTCM_1058: return "GPS SSR Clock Correction";
        case RTCMMessageType::RTCM_1059: return "GPS SSR Code Bias";
        case RTCMMessageType::RTCM_1060: return "GPS SSR Combined Orbit/Clock Correction";
        case RTCMMessageType::RTCM_1061: return "GPS SSR URA";
        case RTCMMessageType::RTCM_1062: return "GPS SSR High-Rate Clock Correction";
        case RTCMMessageType::RTCM_1063: return "GLONASS SSR Orbit Correction";
        case RTCMMessageType::RTCM_1064: return "GLONASS SSR Clock Correction";
        case RTCMMessageType::RTCM_1065: return "GLONASS SSR Code Bias";
        case RTCMMessageType::RTCM_1066: return "GLONASS SSR Combined Orbit/Clock Correction";
        case RTCMMessageType::RTCM_1067: return "GLONASS SSR URA";
        case RTCMMessageType::RTCM_1068: return "GLONASS SSR High-Rate Clock Correction";
        case RTCMMessageType::RTCM_1074: return "GPS MSM4";
        case RTCMMessageType::RTCM_1075: return "GPS MSM5";
        case RTCMMessageType::RTCM_1076: return "GPS MSM6";
        case RTCMMessageType::RTCM_1077: return "GPS MSM7";
        case RTCMMessageType::RTCM_1084: return "GLONASS MSM4";
        case RTCMMessageType::RTCM_1085: return "GLONASS MSM5";
        case RTCMMessageType::RTCM_1086: return "GLONASS MSM6";
        case RTCMMessageType::RTCM_1087: return "GLONASS MSM7";
        case RTCMMessageType::RTCM_1094: return "Galileo MSM4";
        case RTCMMessageType::RTCM_1095: return "Galileo MSM5";
        case RTCMMessageType::RTCM_1096: return "Galileo MSM6";
        case RTCMMessageType::RTCM_1097: return "Galileo MSM7";
        case RTCMMessageType::RTCM_1124: return "BeiDou MSM4";
        case RTCMMessageType::RTCM_1125: return "BeiDou MSM5";
        case RTCMMessageType::RTCM_1126: return "BeiDou MSM6";
        case RTCMMessageType::RTCM_1127: return "BeiDou MSM7";
        case RTCMMessageType::RTCM_1240: return "Galileo SSR Orbit Correction";
        case RTCMMessageType::RTCM_1241: return "Galileo SSR Clock Correction";
        case RTCMMessageType::RTCM_1242: return "Galileo SSR Code Bias";
        case RTCMMessageType::RTCM_1243: return "Galileo SSR Combined Orbit/Clock Correction";
        case RTCMMessageType::RTCM_1244: return "Galileo SSR URA";
        case RTCMMessageType::RTCM_1245: return "Galileo SSR High-Rate Clock Correction";
        case RTCMMessageType::RTCM_1246: return "QZSS SSR Orbit Correction";
        case RTCMMessageType::RTCM_1247: return "QZSS SSR Clock Correction";
        case RTCMMessageType::RTCM_1248: return "QZSS SSR Code Bias";
        case RTCMMessageType::RTCM_1249: return "QZSS SSR Combined Orbit/Clock Correction";
        case RTCMMessageType::RTCM_1250: return "QZSS SSR URA";
        case RTCMMessageType::RTCM_1251: return "QZSS SSR High-Rate Clock Correction";
        case RTCMMessageType::RTCM_1258: return "BeiDou SSR Orbit Correction";
        case RTCMMessageType::RTCM_1259: return "BeiDou SSR Clock Correction";
        case RTCMMessageType::RTCM_1260: return "BeiDou SSR Code Bias";
        case RTCMMessageType::RTCM_1261: return "BeiDou SSR Combined Orbit/Clock Correction";
        case RTCMMessageType::RTCM_1262: return "BeiDou SSR URA";
        case RTCMMessageType::RTCM_1263: return "BeiDou SSR High-Rate Clock Correction";
        default:
            return "RTCM " + std::to_string(static_cast<uint16_t>(type));
    }
}

bool isObservationMessage(RTCMMessageType type) {
    auto type_val = static_cast<uint16_t>(type);
    return (type_val >= 1001 && type_val <= 1004) ||
           (type_val >= 1009 && type_val <= 1012) ||
           (type_val >= 1074 && type_val <= 1077) ||
           (type_val >= 1084 && type_val <= 1087) ||
           (type_val >= 1094 && type_val <= 1097) ||
           (type_val >= 1124 && type_val <= 1127);
}

bool isEphemerisMessage(RTCMMessageType type) {
    auto type_val = static_cast<uint16_t>(type);
    return type_val == 1019 || type_val == 1020;
}

bool isSSRMessage(RTCMMessageType type) {
    const auto type_val = static_cast<uint16_t>(type);
    return (type_val >= 1057 && type_val <= 1068) ||
           (type_val >= 1240 && type_val <= 1263);
}

GNSSSystem getSystemFromMessageType(RTCMMessageType type) {
    const auto type_val = static_cast<uint16_t>(type);
    if ((type_val >= 1001 && type_val <= 1004) || type_val == 1019 ||
        (type_val >= 1057 && type_val <= 1062) ||
        (type_val >= 1074 && type_val <= 1077)) {
        return GNSSSystem::GPS;
    }
    if ((type_val >= 1009 && type_val <= 1012) || type_val == 1020 ||
        (type_val >= 1063 && type_val <= 1068) ||
        (type_val >= 1084 && type_val <= 1087)) {
        return GNSSSystem::GLONASS;
    }
    if ((type_val >= 1094 && type_val <= 1097) ||
        (type_val >= 1240 && type_val <= 1245)) {
        return GNSSSystem::Galileo;
    }
    if (type_val >= 1246 && type_val <= 1251) {
        return GNSSSystem::QZSS;
    }
    if ((type_val >= 1124 && type_val <= 1127) ||
        (type_val >= 1258 && type_val <= 1263)) {
        return GNSSSystem::BeiDou;
    }
    if (type_val >= 1252 && type_val <= 1257) {
        return GNSSSystem::SBAS;
    }
    return GNSSSystem::UNKNOWN;
}

bool isStationMessage(RTCMMessageType type) {
    auto type_val = static_cast<uint16_t>(type);
    return (type_val >= 1005 && type_val <= 1008) || type_val == 1033;
}

} // namespace rtcm_utils

} // namespace io
} // namespace libgnss
