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

#include "rtcm_internal.hpp"


namespace libgnss {
namespace io {
using namespace rtcm_internal;

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



bool RTCMProcessor::decodeObservationData(const RTCMMessage& message, ObservationData& obs_data) {
    return decodeObservationMessage(message, obs_data);
}

bool RTCMProcessor::decodeNavigationData(const RTCMMessage& message, NavigationData& nav_data) {
    return decodeEphemerisMessage(message, nav_data);
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

} // namespace io
} // namespace libgnss
