#include "../../include/libgnss++/io/rtcm.hpp"

namespace libgnss {
namespace io {

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

// RTCMProcessor implementation
void RTCMProcessor::clear() {
    reference_position_.setZero();
}

std::vector<RTCMMessage> RTCMProcessor::decode(const uint8_t* buffer, size_t size) {
    std::vector<RTCMMessage> messages;
    size_t i = 0;
    while (i < size) {
        // Search for preamble
        if (buffer[i] != 0xD3) {
            i++;
            continue;
        }

        if (i + 3 > size) { // Not enough data for header
            break;
        }

        uint16_t payload_len = ((buffer[i + 1] & 0x03) << 8) | buffer[i + 2];
        size_t total_len = 3 + payload_len + 3; // Header + Payload + CRC

        if (i + total_len > size) {
            break; // Incomplete message in buffer
        }

        // First, verify CRC to ensure message integrity before parsing
        uint32_t calculated_crc = calculateCRC24(buffer + i, 3 + payload_len);
        const uint8_t* received_crc_ptr = buffer + i + 3 + payload_len;
        uint32_t received_crc = (received_crc_ptr[0] << 16) | (received_crc_ptr[1] << 8) | received_crc_ptr[2];

        if (calculated_crc == received_crc) {
            // CRC is valid, now create and populate the message
            const uint8_t* payload_ptr = buffer + i + 3;
            RTCMMessage message;
            message.data.assign(payload_ptr, payload_ptr + payload_len);
            message.length = payload_len;
            message.type = static_cast<RTCMMessageType>(getBits(payload_ptr, payload_len, 0, 12));
            message.valid = true;
            messages.push_back(message);

            // Directly decode station message if applicable
            if (message.type == RTCMMessageType::RTCM_1005) {
                int bit_pos = 12; // Skip 12-bit message type
                int64_t ecef_x_raw = getBits(payload_ptr, payload_len, bit_pos, 38); bit_pos += 38;
                int64_t ecef_y_raw = getBits(payload_ptr, payload_len, bit_pos, 38); bit_pos += 38;
                int64_t ecef_z_raw = getBits(payload_ptr, payload_len, bit_pos, 38); bit_pos += 38;
                double ecef_x = ecef_x_raw * 0.0001;
                double ecef_y = ecef_y_raw * 0.0001;
                double ecef_z = ecef_z_raw * 0.0001;
                setReferencePosition({ecef_x, ecef_y, ecef_z});
            }
            i += total_len; // Move to the start of the next message
        } else {
            // CRC failed, move to the next byte to re-sync
            i++;
        }
    }
    return messages;
}


RTCMMessage RTCMProcessor::encodeObservations(const ObservationData& obs_data,
                                             RTCMMessageType message_type) {
    // Stub implementation
    return RTCMMessage();
}

RTCMMessage RTCMProcessor::encodeEphemeris(const Ephemeris& ephemeris) {
    // Stub implementation
    return RTCMMessage();
}

bool RTCMProcessor::decodeObservationMessage(const RTCMMessage& message, ObservationData& obs_data) {
    // Stub implementation
    return false;
}

bool RTCMProcessor::decodeEphemerisMessage(const RTCMMessage& message, NavigationData& nav_data) {
    // Stub implementation
    return false;
}


uint32_t RTCMProcessor::calculateCRC24(const uint8_t* data, size_t length) {
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = (crc << 8) ^ crc24q_table[(crc >> 16) ^ data[i]];
    }
    return crc & 0x00FFFFFF;
}


int64_t RTCMProcessor::getBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    // Ensure the number of bits to extract is within a valid range
    if (num_bits <= 0 || num_bits > 64) {
        return 0; // Invalid number of bits
    }

    // Check if the entire bit range is within the buffer
    if ((bit_pos + num_bits) > (data_size * 8)) {
        return 0; // Attempting to read beyond the buffer
    }

    uint64_t value = 0;
    for (int i = 0; i < num_bits; ++i) {
        int current_bit_pos = bit_pos + i;
        int byte_index = current_bit_pos / 8;

        // This check should be redundant due to the initial check, but as a safeguard:
        if (byte_index >= data_size) {
            return 0; // Should not happen
        }

        int bit_in_byte = 7 - (current_bit_pos % 8);
        uint64_t bit = (data[byte_index] >> bit_in_byte) & 1;
        value = (value << 1) | bit;
    }

    // Sign extension for signed values
    if (num_bits > 0 && num_bits < 64) {
        uint64_t sign_bit = 1ULL << (num_bits - 1);
        if (value & sign_bit) {
            value |= ~((1ULL << num_bits) - 1);
        }
    }

    return (int64_t)value;
}

void RTCMProcessor::setBits(uint8_t* data, int pos, int len, uint32_t value) {
    // Stub implementation
}

bool RTCMProcessor::parseHeader(const uint8_t* data, RTCMMessageType& type, uint16_t& length) {
    // Stub implementation
    return false;
}

std::vector<uint8_t> RTCMProcessor::extractPayload(const uint8_t* data, size_t total_length) {
    // Stub implementation
    return {};
}

// RTCMReader implementation
bool RTCMReader::open(const std::string& source) {
    // Stub implementation
    return false;
}

bool RTCMReader::readFromNetwork(const std::string& url) {
    // Stub implementation
    return false;
}

// rtcm_utils implementation
namespace rtcm_utils {

uint32_t gpsTimeToRTCMTime(const GNSSTime& gps_time) {
    // Stub implementation
    return 0;
}

GNSSTime rtcmTimeToGPSTime(uint32_t rtcm_time, uint16_t week) {
    // Stub implementation
    return GNSSTime();
}

std::string getMessageTypeName(RTCMMessageType type) {
    // Stub implementation
    return "Unknown";
}

bool isObservationMessage(RTCMMessageType type) {
    auto type_val = static_cast<uint16_t>(type);
    return (type_val >= 1001 && type_val <= 1012) || (type_val >= 1074 && type_val <= 1127);
}

bool isEphemerisMessage(RTCMMessageType type) {
    auto type_val = static_cast<uint16_t>(type);
    return type_val == 1019 || type_val == 1020;
}

GNSSSystem getSystemFromMessageType(RTCMMessageType type) {
    // Stub implementation
        return GNSSSystem::UNKNOWN;
}

bool isStationMessage(RTCMMessageType type) {
    auto type_val = static_cast<uint16_t>(type);
    return (type_val >= 1005 && type_val <= 1008) || type_val == 1033;
}

} // namespace rtcm_utils

} // namespace io
} // namespace libgnss
