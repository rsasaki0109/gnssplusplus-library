#include <libgnss++/io/ubx.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace libgnss {
namespace io {

namespace {

constexpr uint8_t kUBXSync1 = 0xB5;
constexpr uint8_t kUBXSync2 = 0x62;

template <typename T>
T readLittleEndian(const uint8_t* data) {
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

SignalType defaultSignalType(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return SignalType::GPS_L1CA;
        case GNSSSystem::GLONASS: return SignalType::GLO_L1CA;
        case GNSSSystem::Galileo: return SignalType::GAL_E1;
        case GNSSSystem::BeiDou: return SignalType::BDS_B1I;
        case GNSSSystem::QZSS: return SignalType::QZS_L1CA;
        case GNSSSystem::NavIC: return SignalType::GPS_L5;
        default: return SignalType::GPS_L1CA;
    }
}

bool validateUbxChecksum(const uint8_t* data, size_t payload_length, uint8_t ck_a, uint8_t ck_b) {
    uint8_t computed_a = 0;
    uint8_t computed_b = 0;
    for (size_t i = 0; i < payload_length; ++i) {
        computed_a = static_cast<uint8_t>(computed_a + data[i]);
        computed_b = static_cast<uint8_t>(computed_b + computed_a);
    }
    return computed_a == ck_a && computed_b == ck_b;
}

uint32_t readBitsMsbFirst(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    if (data == nullptr || data_size == 0 || num_bits <= 0) {
        return 0;
    }

    uint32_t value = 0;
    for (int index = 0; index < num_bits; ++index) {
        const int absolute_bit = bit_pos + index;
        const size_t byte_index = static_cast<size_t>(absolute_bit / 8);
        if (byte_index >= data_size) {
            value <<= (num_bits - index);
            break;
        }
        const int bit_index = 7 - (absolute_bit % 8);
        value = static_cast<uint32_t>((value << 1) |
                                      ((data[byte_index] >> bit_index) & 0x01U));
    }
    return value;
}

std::vector<uint8_t> rawWordsToBigEndianBytes(const std::vector<uint32_t>& words,
                                              size_t max_words) {
    const size_t count = std::min(words.size(), max_words);
    std::vector<uint8_t> bytes;
    bytes.reserve(count * 4U);
    for (size_t index = 0; index < count; ++index) {
        const uint32_t word = words[index];
        bytes.push_back(static_cast<uint8_t>((word >> 24) & 0xFFU));
        bytes.push_back(static_cast<uint8_t>((word >> 16) & 0xFFU));
        bytes.push_back(static_cast<uint8_t>((word >> 8) & 0xFFU));
        bytes.push_back(static_cast<uint8_t>(word & 0xFFU));
    }
    return bytes;
}

}  // namespace

void UBXDecoder::clear() {
    stats_ = UBXStats{};
    last_nav_pvt_ = UBXNavPVT{};
    has_last_nav_pvt_ = false;
    last_gps_week_ = 0;
    has_last_gps_week_ = false;
}

std::vector<UBXMessage> UBXDecoder::decode(const uint8_t* buffer, size_t size) {
    std::vector<UBXMessage> messages;
    size_t offset = 0;

    while (offset < size) {
        if (buffer[offset] != kUBXSync1 || offset + 1 >= size || buffer[offset + 1] != kUBXSync2) {
            ++offset;
            continue;
        }

        if (offset + 8 > size) {
            break;
        }

        const uint8_t message_class = buffer[offset + 2];
        const uint8_t message_id = buffer[offset + 3];
        const uint16_t payload_length = readLittleEndian<uint16_t>(buffer + offset + 4);
        const size_t total_length = 6U + payload_length + 2U;
        if (offset + total_length > size) {
            break;
        }

        ++stats_.total_messages;

        const uint8_t ck_a = buffer[offset + 6 + payload_length];
        const uint8_t ck_b = buffer[offset + 7 + payload_length];
        if (!validateChecksum(buffer + offset + 2, 4U + payload_length, ck_a, ck_b)) {
            ++stats_.checksum_errors;
            ++offset;
            continue;
        }

        UBXMessage message;
        message.message_class = message_class;
        message.message_id = message_id;
        message.length = payload_length;
        message.payload.assign(buffer + offset + 6, buffer + offset + 6 + payload_length);
        message.valid = true;
        messages.push_back(message);

        ++stats_.valid_messages;
        ++stats_.message_counts[messageKey(message_class, message_id)];
        offset += total_length;
    }

    return messages;
}

bool UBXDecoder::decodeNavPVT(const UBXMessage& message, UBXNavPVT& nav_pvt) {
    if (message.message_class != static_cast<uint8_t>(UBXMessageClass::NAV) ||
        message.message_id != 0x07 || message.payload.size() < 92) {
        return false;
    }

    nav_pvt = UBXNavPVT{};
    const auto& payload = message.payload;
    const double tow_seconds =
        static_cast<double>(readLittleEndian<uint32_t>(payload.data())) * 1e-3 +
        static_cast<double>(readLittleEndian<int32_t>(payload.data() + 16)) * 1e-9;

    nav_pvt.time = GNSSTime(has_last_gps_week_ ? static_cast<int>(last_gps_week_) : 0, tow_seconds);
    nav_pvt.valid_time = has_last_gps_week_;
    nav_pvt.fix_type = payload[20];
    nav_pvt.gnss_fix_ok = (payload[21] & 0x01U) != 0;
    nav_pvt.differential_solution = (payload[21] & 0x02U) != 0;
    nav_pvt.carrier_solution = static_cast<uint8_t>((payload[21] >> 6) & 0x03U);
    nav_pvt.num_sv = payload[23];

    const int32_t lon_raw = readLittleEndian<int32_t>(payload.data() + 24);
    const int32_t lat_raw = readLittleEndian<int32_t>(payload.data() + 28);
    const int32_t height_raw = readLittleEndian<int32_t>(payload.data() + 32);
    nav_pvt.position_geodetic = GeodeticCoord(
        static_cast<double>(lat_raw) * 1e-7 * M_PI / 180.0,
        static_cast<double>(lon_raw) * 1e-7 * M_PI / 180.0,
        static_cast<double>(height_raw) * 1e-3);
    nav_pvt.position_ecef = geodetic2ecef(nav_pvt.position_geodetic.latitude,
                                          nav_pvt.position_geodetic.longitude,
                                          nav_pvt.position_geodetic.height);
    nav_pvt.horizontal_accuracy_m =
        static_cast<double>(readLittleEndian<uint32_t>(payload.data() + 40)) * 1e-3;
    nav_pvt.vertical_accuracy_m =
        static_cast<double>(readLittleEndian<uint32_t>(payload.data() + 44)) * 1e-3;

    const uint16_t flags3 = readLittleEndian<uint16_t>(payload.data() + 78);
    nav_pvt.valid_position =
        nav_pvt.fix_type >= 2 && nav_pvt.gnss_fix_ok && ((flags3 & 0x0001U) == 0);

    last_nav_pvt_ = nav_pvt;
    has_last_nav_pvt_ = true;
    return true;
}

bool UBXDecoder::decodeRawx(const UBXMessage& message, ObservationData& obs_data) {
    if (message.message_class != static_cast<uint8_t>(UBXMessageClass::RXM) ||
        message.message_id != 0x15 || message.payload.size() < 16) {
        return false;
    }

    const auto& payload = message.payload;
    const uint8_t num_measurements = payload[11];
    const size_t expected_length = 16U + static_cast<size_t>(num_measurements) * 32U;
    if (payload.size() < expected_length) {
        return false;
    }

    const double receiver_tow = readLittleEndian<double>(payload.data());
    const uint16_t gps_week = readLittleEndian<uint16_t>(payload.data() + 8);
    last_gps_week_ = gps_week;
    has_last_gps_week_ = true;

    obs_data.clear();
    obs_data.time = GNSSTime(static_cast<int>(gps_week), receiver_tow);
    if (has_last_nav_pvt_ && last_nav_pvt_.valid_position) {
        obs_data.receiver_position = last_nav_pvt_.position_ecef;
    }

    for (uint8_t index = 0; index < num_measurements; ++index) {
        const size_t base = 16U + static_cast<size_t>(index) * 32U;
        const uint8_t gnss_id = payload[base + 20];
        const uint8_t sv_id = payload[base + 21];
        const uint8_t sig_id = payload[base + 22];
        const uint8_t cno = payload[base + 26];
        const uint8_t trk_stat = payload[base + 30];
        const uint16_t locktime = readLittleEndian<uint16_t>(payload.data() + base + 24);

        const GNSSSystem system = ubx_utils::getSystemFromGnssId(gnss_id);
        if (system == GNSSSystem::UNKNOWN || sv_id == 0) {
            continue;
        }

        SignalType signal_type = defaultSignalType(system);
        ubx_utils::getSignalType(gnss_id, sig_id, signal_type);

        Observation obs(SatelliteId(system, sv_id), signal_type);
        obs.code = sig_id;
        obs.snr = static_cast<double>(cno);
        obs.signal_strength = static_cast<int>(cno);

        const double pseudorange = readLittleEndian<double>(payload.data() + base);
        const double carrier_phase = readLittleEndian<double>(payload.data() + base + 8);
        const double doppler = static_cast<double>(readLittleEndian<float>(payload.data() + base + 16));

        obs.has_pseudorange = (trk_stat & 0x01U) != 0 && std::isfinite(pseudorange);
        obs.has_carrier_phase = (trk_stat & 0x02U) != 0 && std::isfinite(carrier_phase);
        obs.has_doppler = std::isfinite(doppler);
        obs.pseudorange = pseudorange;
        obs.carrier_phase = carrier_phase;
        obs.doppler = doppler;
        obs.loss_of_lock = locktime == 0;
        obs.lli = obs.loss_of_lock ? 1 : 0;
        obs.valid = obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler;

        if (obs.valid) {
            obs_data.addObservation(obs);
        }
    }

    return !obs_data.isEmpty();
}

bool UBXDecoder::decodeSfrbx(const UBXMessage& message, UBXSfrbx& sfrbx) {
    if (message.message_class != static_cast<uint8_t>(UBXMessageClass::RXM) ||
        message.message_id != 0x13 || message.payload.size() < 8) {
        return false;
    }

    const auto& payload = message.payload;
    const uint8_t num_words = payload[1];
    const size_t expected_length = 8U + static_cast<size_t>(num_words) * 4U;
    if (payload.size() < expected_length) {
        return false;
    }

    sfrbx = UBXSfrbx{};
    sfrbx.version = payload[0];
    sfrbx.channel = payload[2];
    sfrbx.system = ubx_utils::getSystemFromGnssId(payload[4]);
    sfrbx.sv_id = payload[5];
    sfrbx.frequency_id = payload[7];
    sfrbx.words.reserve(num_words);

    for (uint8_t index = 0; index < num_words; ++index) {
        const size_t offset = 8U + static_cast<size_t>(index) * 4U;
        sfrbx.words.push_back(readLittleEndian<uint32_t>(payload.data() + offset));
    }

    return sfrbx.system != GNSSSystem::UNKNOWN && sfrbx.sv_id != 0 && !sfrbx.words.empty();
}

uint16_t UBXDecoder::messageKey(uint8_t message_class, uint8_t message_id) {
    return static_cast<uint16_t>((static_cast<uint16_t>(message_class) << 8) | message_id);
}

bool UBXDecoder::validateChecksum(const uint8_t* data, size_t payload_length, uint8_t ck_a, uint8_t ck_b) {
    return validateUbxChecksum(data, payload_length, ck_a, ck_b);
}

namespace ubx_utils {

std::string getMessageName(uint8_t message_class, uint8_t message_id) {
    if (message_class == static_cast<uint8_t>(UBXMessageClass::NAV) && message_id == 0x07) {
        return "UBX-NAV-PVT";
    }
    if (message_class == static_cast<uint8_t>(UBXMessageClass::RXM) && message_id == 0x15) {
        return "UBX-RXM-RAWX";
    }
    if (message_class == static_cast<uint8_t>(UBXMessageClass::RXM) && message_id == 0x13) {
        return "UBX-RXM-SFRBX";
    }
    return "UBX";
}

GNSSSystem getSystemFromGnssId(uint8_t gnss_id) {
    switch (gnss_id) {
        case 0: return GNSSSystem::GPS;
        case 1: return GNSSSystem::SBAS;
        case 2: return GNSSSystem::Galileo;
        case 3: return GNSSSystem::BeiDou;
        case 5: return GNSSSystem::QZSS;
        case 6: return GNSSSystem::GLONASS;
        case 7: return GNSSSystem::NavIC;
        default: return GNSSSystem::UNKNOWN;
    }
}

bool getSignalType(uint8_t gnss_id, uint8_t sig_id, SignalType& signal_type) {
    switch (gnss_id) {
        case 0:
            switch (sig_id) {
                case 0: signal_type = SignalType::GPS_L1CA; return true;
                case 3:
                case 4: signal_type = SignalType::GPS_L2C; return true;
                case 6:
                case 7: signal_type = SignalType::GPS_L5; return true;
                default: signal_type = SignalType::GPS_L1CA; return false;
            }
        case 2:
            switch (sig_id) {
                case 0:
                case 1: signal_type = SignalType::GAL_E1; return true;
                case 3:
                case 4: signal_type = SignalType::GAL_E5A; return true;
                case 5:
                case 6: signal_type = SignalType::GAL_E5B; return true;
                default: signal_type = SignalType::GAL_E1; return false;
            }
        case 3:
            switch (sig_id) {
                case 0:
                case 1: signal_type = SignalType::BDS_B1I; return true;
                case 2:
                case 3: signal_type = SignalType::BDS_B2I; return true;
                case 5:
                case 6: signal_type = SignalType::BDS_B1C; return true;
                case 7:
                case 8: signal_type = SignalType::BDS_B2A; return true;
                default: signal_type = SignalType::BDS_B1I; return false;
            }
        case 5:
            switch (sig_id) {
                case 0: signal_type = SignalType::QZS_L1CA; return true;
                case 4:
                case 5: signal_type = SignalType::QZS_L2C; return true;
                case 8:
                case 9: signal_type = SignalType::QZS_L5; return true;
                default: signal_type = SignalType::QZS_L1CA; return false;
            }
        case 6:
            switch (sig_id) {
                case 0: signal_type = SignalType::GLO_L1CA; return true;
                case 2: signal_type = SignalType::GLO_L2CA; return true;
                default: signal_type = SignalType::GLO_L1CA; return false;
            }
        case 7:
            signal_type = SignalType::GPS_L5;
            return sig_id == 0;
        default:
            return false;
    }
}

bool decodeSfrbxFrameInfo(const UBXSfrbx& sfrbx, UBXSfrbxFrameInfo& frame_info) {
    frame_info = UBXSfrbxFrameInfo{};

    switch (sfrbx.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS: {
            if (sfrbx.words.size() < 2) {
                return false;
            }
            const uint32_t word2 = sfrbx.words[1] >> 6;
            const int subframe_id = static_cast<int>((word2 >> 2) & 0x07U);
            if (subframe_id < 1 || subframe_id > 5) {
                return false;
            }
            frame_info.kind = sfrbx.system == GNSSSystem::GPS
                                  ? UBXSfrbxFrameInfo::Kind::GPS_LNAV
                                  : UBXSfrbxFrameInfo::Kind::QZSS_LNAV;
            frame_info.frame_id = subframe_id;
            frame_info.valid = true;
            return true;
        }

        case GNSSSystem::BeiDou: {
            if (sfrbx.words.empty()) {
                return false;
            }
            const uint32_t word1 = sfrbx.words[0] & 0x3FFFFFFFU;
            const int subframe_id = static_cast<int>((word1 >> 12) & 0x07U);
            if (subframe_id < 1 || subframe_id > 5) {
                return false;
            }
            frame_info.frame_id = subframe_id;
            if (sfrbx.sv_id > 5) {
                frame_info.kind = UBXSfrbxFrameInfo::Kind::BDS_D1;
                frame_info.valid = true;
                return true;
            }
            frame_info.kind = UBXSfrbxFrameInfo::Kind::BDS_D2;
            if (subframe_id != 1 || sfrbx.words.size() < 2) {
                return false;
            }
            const uint32_t word2 = sfrbx.words[1] & 0x3FFFFFFFU;
            const int page_id = static_cast<int>((word2 >> 14) & 0x0FU);
            if (page_id < 1 || page_id > 10) {
                return false;
            }
            frame_info.page_id = page_id;
            frame_info.has_page_id = true;
            frame_info.valid = true;
            return true;
        }

        case GNSSSystem::Galileo: {
            if (sfrbx.words.size() < 8) {
                return false;
            }
            const auto bytes = rawWordsToBigEndianBytes(sfrbx.words, 8);
            const int part1 = static_cast<int>(readBitsMsbFirst(bytes.data(), 16U, 0, 1));
            const int page1 = static_cast<int>(readBitsMsbFirst(bytes.data(), 16U, 1, 1));
            const int part2 = static_cast<int>(readBitsMsbFirst(bytes.data() + 16, 16U, 0, 1));
            const int page2 = static_cast<int>(readBitsMsbFirst(bytes.data() + 16, 16U, 1, 1));
            const int word_type = static_cast<int>(readBitsMsbFirst(bytes.data(), bytes.size(), 2, 6));
            if (page1 == 1 || page2 == 1 || part1 != 0 || part2 != 1 || word_type <= 0) {
                return false;
            }
            frame_info.kind = UBXSfrbxFrameInfo::Kind::GAL_INAV;
            frame_info.frame_id = word_type;
            frame_info.valid = true;
            return true;
        }

        case GNSSSystem::GLONASS: {
            if (sfrbx.words.size() < 4) {
                return false;
            }
            const auto bytes = rawWordsToBigEndianBytes(sfrbx.words, 4);
            const int string_number =
                static_cast<int>(readBitsMsbFirst(bytes.data(), bytes.size(), 1, 4));
            if (string_number < 1 || string_number > 15) {
                return false;
            }
            frame_info.kind = UBXSfrbxFrameInfo::Kind::GLO_NAV;
            frame_info.frame_id = string_number;
            frame_info.valid = true;
            return true;
        }

        case GNSSSystem::SBAS: {
            frame_info.kind = UBXSfrbxFrameInfo::Kind::SBAS;
            frame_info.valid = !sfrbx.words.empty();
            return frame_info.valid;
        }

        default:
            return false;
    }
}

const char* getSfrbxFrameKindName(UBXSfrbxFrameInfo::Kind kind) {
    switch (kind) {
        case UBXSfrbxFrameInfo::Kind::GPS_LNAV: return "GPS_LNAV";
        case UBXSfrbxFrameInfo::Kind::QZSS_LNAV: return "QZSS_LNAV";
        case UBXSfrbxFrameInfo::Kind::GAL_INAV: return "GAL_INAV";
        case UBXSfrbxFrameInfo::Kind::BDS_D1: return "BDS_D1";
        case UBXSfrbxFrameInfo::Kind::BDS_D2: return "BDS_D2";
        case UBXSfrbxFrameInfo::Kind::GLO_NAV: return "GLO_NAV";
        case UBXSfrbxFrameInfo::Kind::SBAS: return "SBAS";
        default: return "UNKNOWN";
    }
}

}  // namespace ubx_utils

void UBXStreamDecoder::clear() {
    decoder_.clear();
    buffer_.clear();
}

bool UBXStreamDecoder::pushBytes(const uint8_t* data,
                                 size_t size,
                                 std::vector<UBXStreamDecoder::Event>& events) {
    if (data != nullptr && size != 0) {
        buffer_.insert(buffer_.end(), data, data + size);
    }
    events.clear();

    while (!buffer_.empty()) {
        size_t sync_index = 0;
        while (sync_index + 1 < buffer_.size() &&
               (buffer_[sync_index] != kUBXSync1 || buffer_[sync_index + 1] != kUBXSync2)) {
            ++sync_index;
        }

        if (sync_index + 1 >= buffer_.size()) {
            const bool keep_sync_prefix = buffer_.back() == kUBXSync1;
            buffer_.assign(keep_sync_prefix ? 1U : 0U, kUBXSync1);
            return !events.empty();
        }
        if (sync_index > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(sync_index));
        }
        if (buffer_.size() < 8U) {
            break;
        }

        const uint16_t payload_length = readLittleEndian<uint16_t>(buffer_.data() + 4);
        const size_t total_length = 6U + static_cast<size_t>(payload_length) + 2U;
        if (buffer_.size() < total_length) {
            break;
        }

        const uint8_t ck_a = buffer_[6U + payload_length];
        const uint8_t ck_b = buffer_[7U + payload_length];
        if (!validateUbxChecksum(buffer_.data() + 2, 4U + payload_length, ck_a, ck_b)) {
            const auto ignored = decoder_.decode(buffer_.data(), total_length);
            (void)ignored;
            buffer_.erase(buffer_.begin());
            continue;
        }

        const auto decoded = decoder_.decode(buffer_.data(), total_length);
        if (!decoded.empty()) {
            const auto& message = decoded.front();
            Event event;
            event.has_message = true;
            event.message = message;
            event.has_nav_pvt = decoder_.decodeNavPVT(message, event.nav_pvt);
            event.has_observation = decoder_.decodeRawx(message, event.observation);
            event.has_sfrbx = decoder_.decodeSfrbx(message, event.sfrbx);
            events.push_back(std::move(event));
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total_length));
    }
    return !events.empty();
}

}  // namespace io
}  // namespace libgnss
