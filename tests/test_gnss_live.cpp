#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/rtcm_stream.hpp>

#include "../apps/native/navigation_merge.hpp"

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <atomic>

using namespace libgnss;

namespace {

bool sourcePathExists(const std::filesystem::path& root, const std::string& relative_path) {
    return std::filesystem::exists(root / relative_path);
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
        const uint8_t index = static_cast<uint8_t>(((crc >> 16) ^ data[i]) & 0xFFU);
        crc = (crc << 8) ^ table[index];
    }
    return crc & 0x00FFFFFFU;
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

template <typename T>
void appendLittleEndian(std::vector<uint8_t>& buffer, T value) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
}

std::vector<uint8_t> buildUbxMessage(uint8_t message_class,
                                     uint8_t message_id,
                                     const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> message = {0xB5, 0x62, message_class, message_id};
    appendLittleEndian<uint16_t>(message, static_cast<uint16_t>(payload.size()));
    message.insert(message.end(), payload.begin(), payload.end());

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < message.size(); ++i) {
        ck_a = static_cast<uint8_t>(ck_a + message[i]);
        ck_b = static_cast<uint8_t>(ck_b + ck_a);
    }
    message.push_back(ck_a);
    message.push_back(ck_b);
    return message;
}

bool ubxSignalIds(const Observation& obs, uint8_t& gnss_id, uint8_t& sig_id) {
    switch (obs.signal) {
        case SignalType::GPS_L1CA:
            gnss_id = 0;
            sig_id = 0;
            return true;
        case SignalType::GPS_L2C:
            gnss_id = 0;
            sig_id = 3;
            return true;
        case SignalType::GPS_L5:
            gnss_id = 0;
            sig_id = 6;
            return true;
        case SignalType::GAL_E1:
            gnss_id = 2;
            sig_id = 0;
            return true;
        case SignalType::GAL_E5A:
            gnss_id = 2;
            sig_id = 3;
            return true;
        case SignalType::GAL_E5B:
            gnss_id = 2;
            sig_id = 5;
            return true;
        case SignalType::BDS_B1I:
            gnss_id = 3;
            sig_id = 0;
            return true;
        case SignalType::BDS_B2I:
            gnss_id = 3;
            sig_id = 2;
            return true;
        case SignalType::BDS_B1C:
            gnss_id = 3;
            sig_id = 5;
            return true;
        case SignalType::BDS_B2A:
            gnss_id = 3;
            sig_id = 7;
            return true;
        case SignalType::QZS_L1CA:
            gnss_id = 5;
            sig_id = 0;
            return true;
        case SignalType::QZS_L2C:
            gnss_id = 5;
            sig_id = 4;
            return true;
        case SignalType::QZS_L5:
            gnss_id = 5;
            sig_id = 8;
            return true;
        case SignalType::GLO_L1CA:
            gnss_id = 6;
            sig_id = 0;
            return true;
        case SignalType::GLO_L2CA:
            gnss_id = 6;
            sig_id = 2;
            return true;
        default:
            return false;
    }
}

std::vector<uint8_t> buildUbxRawxMessage(const ObservationData& epoch) {
    std::vector<const Observation*> usable;
    for (const auto& obs : epoch.observations) {
        uint8_t gnss_id = 0;
        uint8_t sig_id = 0;
        if (!ubxSignalIds(obs, gnss_id, sig_id)) {
            continue;
        }
        if (!obs.has_pseudorange && !obs.has_carrier_phase) {
            continue;
        }
        usable.push_back(&obs);
    }

    std::vector<uint8_t> payload;
    appendLittleEndian<double>(payload, epoch.time.tow);
    appendLittleEndian<uint16_t>(payload, static_cast<uint16_t>(epoch.time.week));
    payload.push_back(18);
    payload.push_back(static_cast<uint8_t>(usable.size()));
    payload.push_back(0x01);
    payload.push_back(0x01);
    payload.push_back(0x00);
    payload.push_back(0x00);

    for (const Observation* obs_ptr : usable) {
        const auto& obs = *obs_ptr;
        uint8_t gnss_id = 0;
        uint8_t sig_id = 0;
        const bool mapped = ubxSignalIds(obs, gnss_id, sig_id);
        EXPECT_TRUE(mapped);
        appendLittleEndian<double>(payload, obs.has_pseudorange ? obs.pseudorange : 0.0);
        appendLittleEndian<double>(payload, obs.has_carrier_phase ? obs.carrier_phase : 0.0);
        appendLittleEndian<float>(payload, static_cast<float>(obs.has_doppler ? obs.doppler : 0.0));
        payload.push_back(gnss_id);
        payload.push_back(static_cast<uint8_t>(obs.satellite.prn));
        payload.push_back(sig_id);
        payload.push_back(0);
        appendLittleEndian<uint16_t>(payload, 500);
        payload.push_back(static_cast<uint8_t>(obs.signal_strength > 0 ? obs.signal_strength : 45));
        payload.push_back(0);
        payload.push_back(0);
        payload.push_back(0);
        uint8_t trk_stat = 0;
        if (obs.has_pseudorange) trk_stat |= 0x01U;
        if (obs.has_carrier_phase) trk_stat |= 0x02U;
        payload.push_back(trk_stat);
        payload.push_back(0);
    }

    return buildUbxMessage(0x02, 0x15, payload);
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
    return buildRtcmFrame(io::RTCMMessage(io::RTCMMessageType::RTCM_1005, payload));
}

ObservationData filterGps1004Epoch(const ObservationData& input) {
    ObservationData filtered(input.time);
    filtered.receiver_position = input.receiver_position;
    filtered.receiver_clock_bias = input.receiver_clock_bias;

    std::map<SatelliteId, std::pair<const Observation*, const Observation*>> gps_pairs;
    for (const auto& obs : input.observations) {
        if (obs.satellite.system != GNSSSystem::GPS) {
            continue;
        }
        auto& slot = gps_pairs[obs.satellite];
        if (obs.signal == SignalType::GPS_L1CA) {
            slot.first = &obs;
        } else if (obs.signal == SignalType::GPS_L2C) {
            slot.second = &obs;
        }
    }

    for (const auto& [satellite, pair] : gps_pairs) {
        (void)satellite;
        if (pair.first != nullptr && pair.second != nullptr) {
            filtered.addObservation(*pair.first);
            filtered.addObservation(*pair.second);
        }
    }
    return filtered;
}

std::vector<ObservationData> loadBaseEpochs(const std::filesystem::path& path,
                                            io::RINEXReader::RINEXHeader& header,
                                            size_t count) {
    io::RINEXReader reader;
    EXPECT_TRUE(reader.open(path.string()));
    EXPECT_TRUE(reader.readHeader(header));

    std::vector<ObservationData> epochs;
    ObservationData epoch;
    while (epochs.size() < count && reader.readObservationEpoch(epoch)) {
        epoch.receiver_position = header.approximate_position;
        epochs.push_back(filterGps1004Epoch(epoch));
    }
    reader.close();
    return epochs;
}

ObservationData findEpochAtTime(const std::filesystem::path& path,
                                const GNSSTime& target_time) {
    io::RINEXReader reader;
    io::RINEXReader::RINEXHeader header;
    EXPECT_TRUE(reader.open(path.string()));
    EXPECT_TRUE(reader.readHeader(header));

    ObservationData epoch;
    while (reader.readObservationEpoch(epoch)) {
        if (std::abs(epoch.time - target_time) <= 1e-6) {
            reader.close();
            return filterGps1004Epoch(epoch);
        }
    }
    reader.close();
    return ObservationData();
}

NavigationData loadNavigation(const std::filesystem::path& path) {
    io::RINEXReader reader;
    EXPECT_TRUE(reader.open(path.string()));
    NavigationData nav;
    EXPECT_TRUE(reader.readNavigationData(nav));
    reader.close();
    return nav;
}

void writeBinaryFile(const std::filesystem::path& path,
                     const std::vector<std::vector<uint8_t>>& frames) {
    std::ofstream output(path, std::ios::binary);
    ASSERT_TRUE(output.is_open());
    for (const auto& frame : frames) {
        output.write(reinterpret_cast<const char*>(frame.data()),
                     static_cast<std::streamsize>(frame.size()));
    }
}

std::string readTextFile(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::stringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string extractSummaryToken(const std::string& log, const std::string& key) {
    const size_t summary_pos = log.rfind("summary:");
    if (summary_pos == std::string::npos) {
        return "";
    }
    const std::string prefix = key + "=";
    const size_t key_pos = log.find(prefix, summary_pos);
    if (key_pos == std::string::npos) {
        return "";
    }
    const size_t value_begin = key_pos + prefix.size();
    const size_t value_end = log.find_first_of(" \r\n", value_begin);
    return log.substr(value_begin, value_end == std::string::npos ? std::string::npos
                                                                  : value_end - value_begin);
}

double extractSummaryDouble(const std::string& log, const std::string& key) {
    const std::string token = extractSummaryToken(log, key);
    EXPECT_FALSE(token.empty()) << "missing summary token: " << key << "\n" << log;
    if (token.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::stod(token);
}

std::vector<uint8_t> corruptFrame(std::vector<uint8_t> frame) {
    if (!frame.empty()) {
        frame.back() ^= 0xFFU;
    }
    return frame;
}

#ifndef _WIN32
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

std::filesystem::path makeUniqueTempDir(const std::string& prefix) {
    namespace fs = std::filesystem;
    const auto unique_suffix =
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const fs::path temp_dir = fs::temp_directory_path() / (prefix + "_" + unique_suffix);
    fs::create_directories(temp_dir);
    return temp_dir;
}

}  // namespace

TEST(GNSSLiveTest, InterpolatesBaseEpochAndUsesInlineBaseMetadata) {
    namespace fs = std::filesystem;

    const fs::path source_dir = GNSSPP_SOURCE_DIR;
    if (!sourcePathExists(source_dir, "data/driving/base.obs") ||
        !sourcePathExists(source_dir, "data/driving/rover.obs") ||
        !sourcePathExists(source_dir, "data/driving/navigation.nav")) {
        GTEST_SKIP() << "repo driving test data is not available";
    }
    const fs::path binary_dir = GNSSPP_BINARY_DIR;
    const fs::path live_binary = binary_dir / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    io::RINEXReader::RINEXHeader base_header;
    const auto base_epochs =
        loadBaseEpochs(source_dir / "data" / "driving" / "base.obs", base_header, 2);
    ASSERT_EQ(base_epochs.size(), 2U);
    for (const auto& epoch : base_epochs) {
        ASSERT_GE(epoch.getNumSatellites(), 4U);
    }

    const GNSSTime rover_target_time(base_epochs[0].time.week, base_epochs[0].time.tow + 0.5);
    const ObservationData rover_epoch =
        findEpochAtTime(source_dir / "data" / "driving" / "rover.obs", rover_target_time);
    ASSERT_FALSE(rover_epoch.isEmpty());
    ASSERT_GE(rover_epoch.getNumSatellites(), 4U);

    const NavigationData nav =
        loadNavigation(source_dir / "data" / "driving" / "navigation.nav");

    io::RTCMProcessor encoder;
    std::set<SatelliteId> satellites;
    for (const auto& obs : base_epochs[0].observations) satellites.insert(obs.satellite);
    for (const auto& obs : base_epochs[1].observations) satellites.insert(obs.satellite);
    for (const auto& obs : rover_epoch.observations) satellites.insert(obs.satellite);

    std::vector<std::vector<uint8_t>> base_frames;
    base_frames.push_back(buildRtcm1005(base_header.approximate_position.x(),
                                        base_header.approximate_position.y(),
                                        base_header.approximate_position.z()));

    size_t nav_frames = 0;
    for (const auto& satellite : satellites) {
        const Ephemeris* eph = nav.getEphemeris(satellite, rover_epoch.time);
        if (eph == nullptr || satellite.system != GNSSSystem::GPS) {
            continue;
        }
        const auto message = encoder.encodeEphemeris(*eph);
        ASSERT_TRUE(message.valid);
        base_frames.push_back(buildRtcmFrame(message));
        ++nav_frames;
    }
    ASSERT_GE(nav_frames, 4U);

    const auto base_before_message =
        encoder.encodeObservations(base_epochs[0], io::RTCMMessageType::RTCM_1004);
    const auto base_after_message =
        encoder.encodeObservations(base_epochs[1], io::RTCMMessageType::RTCM_1004);
    const auto rover_message =
        encoder.encodeObservations(rover_epoch, io::RTCMMessageType::RTCM_1004);

    ASSERT_TRUE(base_before_message.valid);
    ASSERT_TRUE(base_after_message.valid);
    ASSERT_TRUE(rover_message.valid);

    base_frames.push_back(buildRtcmFrame(base_before_message));
    base_frames.push_back(buildRtcmFrame(base_after_message));

    std::vector<std::vector<uint8_t>> rover_frames;
    rover_frames.push_back(buildRtcmFrame(rover_message));

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_interp_test");
    const fs::path rover_path = temp_dir / "rover.rtcm3";
    const fs::path base_path = temp_dir / "base.rtcm3";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";
    const fs::path no_interp_out_path = temp_dir / "live_no_interp.pos";
    const fs::path no_interp_log_path = temp_dir / "live_no_interp.log";

    writeBinaryFile(base_path, base_frames);
    writeBinaryFile(rover_path, rover_frames);

    const std::string run_command =
        "\"" + live_binary.string() + "\" --rover-rtcm \"" + rover_path.string() +
        "\" --base-rtcm \"" + base_path.string() +
        "\" --out \"" + out_path.string() +
        "\" --max-epochs 1 --quiet > \"" + log_path.string() + "\" 2>&1";
    const int run_rc = std::system(run_command.c_str());
    ASSERT_EQ(run_rc, 0) << readTextFile(log_path);

    const std::string log = readTextFile(log_path);
    EXPECT_EQ(extractSummaryToken(log, "termination"), "max_epochs_reached") << log;
    EXPECT_NE(log.find("interpolated_base_epochs=1"), std::string::npos) << log;
    EXPECT_NE(log.find("held_base_epochs=0"), std::string::npos) << log;
    EXPECT_NE(log.find("written_solutions=1"), std::string::npos) << log;
    EXPECT_GT(extractSummaryDouble(log, "solver_wall_time_s"), 0.0) << log;
    EXPECT_GE(extractSummaryDouble(log, "effective_epoch_rate_hz"), 0.0) << log;
    EXPECT_EQ(extractSummaryToken(log, "rover_decoder_errors"), "0") << log;
    EXPECT_EQ(extractSummaryToken(log, "base_decoder_errors"), "0") << log;
    ASSERT_TRUE(fs::exists(out_path));
    const std::string pos_text = readTextFile(out_path);
    EXPECT_NE(pos_text.find("LibGNSS++ Position Solution"), std::string::npos);

    const std::string no_interp_command =
        "\"" + live_binary.string() + "\" --rover-rtcm \"" + rover_path.string() +
        "\" --base-rtcm \"" + base_path.string() +
        "\" --out \"" + no_interp_out_path.string() +
        "\" --max-epochs 1 --no-base-interp --base-hold-seconds 0 --quiet > \"" +
        no_interp_log_path.string() + "\" 2>&1";
    const int no_interp_rc = std::system(no_interp_command.c_str());
    EXPECT_NE(no_interp_rc, 0);

    const std::string no_interp_log = readTextFile(no_interp_log_path);
    EXPECT_EQ(extractSummaryToken(no_interp_log, "termination"), "no_aligned_epochs")
        << no_interp_log;
    EXPECT_NE(no_interp_log.find("interpolated_base_epochs=0"), std::string::npos) << no_interp_log;
    EXPECT_NE(no_interp_log.find("held_base_epochs=0"), std::string::npos) << no_interp_log;
    EXPECT_NE(no_interp_log.find("written_solutions=0"), std::string::npos) << no_interp_log;

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveTest, ReportsRealtimeMetricsForMultiEpochReplay) {
    namespace fs = std::filesystem;

    const fs::path source_dir = GNSSPP_SOURCE_DIR;
    if (!sourcePathExists(source_dir, "data/driving/base.obs") ||
        !sourcePathExists(source_dir, "data/driving/rover.obs") ||
        !sourcePathExists(source_dir, "data/driving/navigation.nav")) {
        GTEST_SKIP() << "repo driving test data is not available";
    }
    const fs::path binary_dir = GNSSPP_BINARY_DIR;
    const fs::path live_binary = binary_dir / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    io::RINEXReader::RINEXHeader base_header;
    const auto base_epochs =
        loadBaseEpochs(source_dir / "data" / "driving" / "base.obs", base_header, 3);
    ASSERT_EQ(base_epochs.size(), 3U);
    for (const auto& epoch : base_epochs) {
        ASSERT_GE(epoch.getNumSatellites(), 4U);
    }

    std::vector<ObservationData> rover_epochs;
    for (const auto& base_epoch : base_epochs) {
        const ObservationData rover_epoch =
            findEpochAtTime(source_dir / "data" / "driving" / "rover.obs", base_epoch.time);
        ASSERT_FALSE(rover_epoch.isEmpty());
        ASSERT_GE(rover_epoch.getNumSatellites(), 4U);
        rover_epochs.push_back(rover_epoch);
    }

    const NavigationData nav =
        loadNavigation(source_dir / "data" / "driving" / "navigation.nav");

    io::RTCMProcessor encoder;
    std::set<SatelliteId> satellites;
    for (const auto& epoch : base_epochs) {
        for (const auto& obs : epoch.observations) satellites.insert(obs.satellite);
    }
    for (const auto& epoch : rover_epochs) {
        for (const auto& obs : epoch.observations) satellites.insert(obs.satellite);
    }

    std::vector<std::vector<uint8_t>> base_frames;
    base_frames.push_back(buildRtcm1005(base_header.approximate_position.x(),
                                        base_header.approximate_position.y(),
                                        base_header.approximate_position.z()));

    size_t nav_frames = 0;
    for (const auto& satellite : satellites) {
        const Ephemeris* eph = nav.getEphemeris(satellite, rover_epochs.front().time);
        if (eph == nullptr || satellite.system != GNSSSystem::GPS) {
            continue;
        }
        const auto message = encoder.encodeEphemeris(*eph);
        ASSERT_TRUE(message.valid);
        base_frames.push_back(buildRtcmFrame(message));
        ++nav_frames;
    }
    ASSERT_GE(nav_frames, 4U);

    for (const auto& epoch : base_epochs) {
        const auto message = encoder.encodeObservations(epoch, io::RTCMMessageType::RTCM_1004);
        ASSERT_TRUE(message.valid);
        base_frames.push_back(buildRtcmFrame(message));
    }

    std::vector<std::vector<uint8_t>> rover_frames;
    for (const auto& epoch : rover_epochs) {
        const auto message = encoder.encodeObservations(epoch, io::RTCMMessageType::RTCM_1004);
        ASSERT_TRUE(message.valid);
        rover_frames.push_back(buildRtcmFrame(message));
    }

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_realtime_test");
    const fs::path rover_path = temp_dir / "rover.rtcm3";
    const fs::path base_path = temp_dir / "base.rtcm3";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";

    writeBinaryFile(base_path, base_frames);
    writeBinaryFile(rover_path, rover_frames);

    const std::string run_command =
        "\"" + live_binary.string() + "\" --rover-rtcm \"" + rover_path.string() +
        "\" --base-rtcm \"" + base_path.string() +
        "\" --out \"" + out_path.string() +
        "\" --max-epochs 3 --quiet > \"" + log_path.string() + "\" 2>&1";
    const int run_rc = std::system(run_command.c_str());
    ASSERT_EQ(run_rc, 0) << readTextFile(log_path);

    const std::string log = readTextFile(log_path);
    EXPECT_EQ(extractSummaryToken(log, "termination"), "max_epochs_reached") << log;
    EXPECT_EQ(extractSummaryToken(log, "written_solutions"), "3") << log;
    EXPECT_EQ(extractSummaryToken(log, "rover_decoder_errors"), "0") << log;
    EXPECT_EQ(extractSummaryToken(log, "base_decoder_errors"), "0") << log;
    EXPECT_GT(extractSummaryDouble(log, "solution_span_s"), 0.0) << log;
    EXPECT_GT(extractSummaryDouble(log, "effective_epoch_rate_hz"), 1.0) << log;
    EXPECT_GT(extractSummaryDouble(log, "realtime_factor"), 1.0) << log;
    ASSERT_TRUE(fs::exists(out_path));

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveTest, HoldsRecentBaseEpochWhenFutureBaseHasNotArrivedYet) {
    namespace fs = std::filesystem;

    const fs::path source_dir = GNSSPP_SOURCE_DIR;
    if (!sourcePathExists(source_dir, "data/driving/base.obs") ||
        !sourcePathExists(source_dir, "data/driving/rover.obs") ||
        !sourcePathExists(source_dir, "data/driving/navigation.nav")) {
        GTEST_SKIP() << "repo driving test data is not available";
    }
    const fs::path binary_dir = GNSSPP_BINARY_DIR;
    const fs::path live_binary = binary_dir / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    io::RINEXReader::RINEXHeader base_header;
    const auto base_epochs =
        loadBaseEpochs(source_dir / "data" / "driving" / "base.obs", base_header, 1);
    ASSERT_EQ(base_epochs.size(), 1U);
    ASSERT_GE(base_epochs[0].getNumSatellites(), 4U);

    const GNSSTime rover_target_time(base_epochs[0].time.week, base_epochs[0].time.tow + 0.4);
    const ObservationData rover_epoch =
        findEpochAtTime(source_dir / "data" / "driving" / "rover.obs", rover_target_time);
    ASSERT_FALSE(rover_epoch.isEmpty());
    ASSERT_GE(rover_epoch.getNumSatellites(), 4U);

    const NavigationData nav =
        loadNavigation(source_dir / "data" / "driving" / "navigation.nav");

    io::RTCMProcessor encoder;
    std::set<SatelliteId> satellites;
    for (const auto& obs : base_epochs[0].observations) satellites.insert(obs.satellite);
    for (const auto& obs : rover_epoch.observations) satellites.insert(obs.satellite);

    std::vector<std::vector<uint8_t>> base_frames;
    base_frames.push_back(buildRtcm1005(base_header.approximate_position.x(),
                                        base_header.approximate_position.y(),
                                        base_header.approximate_position.z()));

    size_t nav_frames = 0;
    for (const auto& satellite : satellites) {
        const Ephemeris* eph = nav.getEphemeris(satellite, rover_epoch.time);
        if (eph == nullptr || satellite.system != GNSSSystem::GPS) {
            continue;
        }
        const auto message = encoder.encodeEphemeris(*eph);
        ASSERT_TRUE(message.valid);
        base_frames.push_back(buildRtcmFrame(message));
        ++nav_frames;
    }
    ASSERT_GE(nav_frames, 4U);

    const auto base_message =
        encoder.encodeObservations(base_epochs[0], io::RTCMMessageType::RTCM_1004);
    const auto rover_message =
        encoder.encodeObservations(rover_epoch, io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(base_message.valid);
    ASSERT_TRUE(rover_message.valid);

    base_frames.push_back(buildRtcmFrame(base_message));

    std::vector<std::vector<uint8_t>> rover_frames;
    rover_frames.push_back(buildRtcmFrame(rover_message));

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_hold_test");
    const fs::path rover_path = temp_dir / "rover.rtcm3";
    const fs::path base_path = temp_dir / "base.rtcm3";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";
    const fs::path disabled_out_path = temp_dir / "live_disabled.pos";
    const fs::path disabled_log_path = temp_dir / "live_disabled.log";

    writeBinaryFile(base_path, base_frames);
    writeBinaryFile(rover_path, rover_frames);

    const std::string hold_command =
        "\"" + live_binary.string() + "\" --rover-rtcm \"" + rover_path.string() +
        "\" --base-rtcm \"" + base_path.string() +
        "\" --out \"" + out_path.string() +
        "\" --max-epochs 1 --base-hold-seconds 0.5 --quiet > \"" +
        log_path.string() + "\" 2>&1";
    const int hold_rc = std::system(hold_command.c_str());
    ASSERT_EQ(hold_rc, 0) << readTextFile(log_path);

    const std::string hold_log = readTextFile(log_path);
    EXPECT_EQ(extractSummaryToken(hold_log, "termination"), "max_epochs_reached") << hold_log;
    EXPECT_NE(hold_log.find("held_base_epochs=1"), std::string::npos) << hold_log;
    EXPECT_NE(hold_log.find("written_solutions=1"), std::string::npos) << hold_log;
    EXPECT_GT(extractSummaryDouble(hold_log, "solver_wall_time_s"), 0.0) << hold_log;
    ASSERT_TRUE(fs::exists(out_path));

    const std::string disabled_command =
        "\"" + live_binary.string() + "\" --rover-rtcm \"" + rover_path.string() +
        "\" --base-rtcm \"" + base_path.string() +
        "\" --out \"" + disabled_out_path.string() +
        "\" --max-epochs 1 --no-base-interp --base-hold-seconds 0 --quiet > \"" +
        disabled_log_path.string() + "\" 2>&1";
    const int disabled_rc = std::system(disabled_command.c_str());
    EXPECT_NE(disabled_rc, 0);

    const std::string disabled_log = readTextFile(disabled_log_path);
    EXPECT_EQ(extractSummaryToken(disabled_log, "termination"), "no_aligned_epochs")
        << disabled_log;
    EXPECT_NE(disabled_log.find("held_base_epochs=0"), std::string::npos) << disabled_log;
    EXPECT_NE(disabled_log.find("written_solutions=0"), std::string::npos) << disabled_log;

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveTest, ReportsDecoderErrorsWhenBaseRtcmContainsOnlyInvalidFrames) {
    namespace fs = std::filesystem;

    const fs::path source_dir = GNSSPP_SOURCE_DIR;
    if (!sourcePathExists(source_dir, "data/driving/rover.obs")) {
        GTEST_SKIP() << "repo driving test data is not available";
    }
    const fs::path binary_dir = GNSSPP_BINARY_DIR;
    const fs::path live_binary = binary_dir / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    io::RINEXReader::RINEXHeader rover_header;
    const auto rover_epochs =
        loadBaseEpochs(source_dir / "data" / "driving" / "rover.obs", rover_header, 1);
    ASSERT_EQ(rover_epochs.size(), 1U);
    const ObservationData rover_epoch = rover_epochs[0];

    io::RTCMProcessor encoder;
    const auto rover_message =
        encoder.encodeObservations(rover_epoch, io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(rover_message.valid);

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_invalid_base_rtcm");
    const fs::path rover_path = temp_dir / "rover.rtcm3";
    const fs::path base_path = temp_dir / "base_invalid.rtcm3";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";

    writeBinaryFile(rover_path, {buildRtcmFrame(rover_message)});
    writeBinaryFile(base_path, {corruptFrame(buildRtcm1005(0.0, 0.0, 0.0))});

    const std::string command =
        "\"" + live_binary.string() + "\" --rover-rtcm \"" + rover_path.string() +
        "\" --base-rtcm \"" + base_path.string() +
        "\" --out \"" + out_path.string() +
        "\" --max-epochs 1 --quiet > \"" + log_path.string() + "\" 2>&1";
    const int rc = std::system(command.c_str());
    EXPECT_NE(rc, 0);

    const std::string log = readTextFile(log_path);
    EXPECT_EQ(extractSummaryToken(log, "termination"), "no_initial_base_observation") << log;
    EXPECT_EQ(extractSummaryToken(log, "base_valid_messages"), "0") << log;
    EXPECT_EQ(extractSummaryToken(log, "rover_total_messages"), "1") << log;
    EXPECT_EQ(extractSummaryToken(log, "written_solutions"), "0") << log;
    EXPECT_GT(extractSummaryDouble(log, "solver_wall_time_s"), 0.0) << log;

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveTest, ReportsDecoderErrorsWhenRoverUbxContainsOnlyInvalidMessages) {
    namespace fs = std::filesystem;

    const fs::path source_dir = GNSSPP_SOURCE_DIR;
    if (!sourcePathExists(source_dir, "data/driving/base.obs") ||
        !sourcePathExists(source_dir, "data/driving/rover.obs")) {
        GTEST_SKIP() << "repo driving test data is not available";
    }
    const fs::path binary_dir = GNSSPP_BINARY_DIR;
    const fs::path live_binary = binary_dir / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    io::RINEXReader::RINEXHeader base_header;
    const auto base_epochs =
        loadBaseEpochs(source_dir / "data" / "driving" / "base.obs", base_header, 1);
    ASSERT_EQ(base_epochs.size(), 1U);
    const ObservationData rover_epoch =
        findEpochAtTime(source_dir / "data" / "driving" / "rover.obs", base_epochs[0].time);
    ASSERT_FALSE(rover_epoch.isEmpty());

    io::RTCMProcessor encoder;
    const auto base_message =
        encoder.encodeObservations(base_epochs[0], io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(base_message.valid);

    std::vector<uint8_t> invalid_rover_message = buildUbxRawxMessage(rover_epoch);
    ASSERT_FALSE(invalid_rover_message.empty());
    invalid_rover_message.back() ^= 0xFFU;

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_invalid_rover_ubx");
    const fs::path rover_path = temp_dir / "rover_invalid.ubx";
    const fs::path base_path = temp_dir / "base.rtcm3";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";

    {
        std::ofstream rover_output(rover_path, std::ios::binary);
        ASSERT_TRUE(rover_output.is_open());
        rover_output.write(reinterpret_cast<const char*>(invalid_rover_message.data()),
                           static_cast<std::streamsize>(invalid_rover_message.size()));
    }
    writeBinaryFile(base_path,
                    {buildRtcm1005(base_header.approximate_position.x(),
                                   base_header.approximate_position.y(),
                                   base_header.approximate_position.z()),
                     buildRtcmFrame(base_message)});

    const std::string command =
        "\"" + live_binary.string() + "\" --rover-ubx \"" + rover_path.string() +
        "\" --base-rtcm \"" + base_path.string() +
        "\" --out \"" + out_path.string() +
        "\" --max-epochs 1 --quiet > \"" + log_path.string() + "\" 2>&1";
    const int rc = std::system(command.c_str());
    EXPECT_NE(rc, 0);

    const std::string log = readTextFile(log_path);
    EXPECT_EQ(extractSummaryToken(log, "termination"), "no_initial_rover_observation") << log;
    EXPECT_EQ(extractSummaryToken(log, "rover_total_messages"), "1") << log;
    EXPECT_EQ(extractSummaryToken(log, "rover_valid_messages"), "0") << log;
    EXPECT_EQ(extractSummaryToken(log, "rover_decoder_errors"), "1") << log;
    EXPECT_EQ(extractSummaryToken(log, "written_solutions"), "0") << log;
    EXPECT_GT(extractSummaryDouble(log, "solver_wall_time_s"), 0.0) << log;

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveTest, SolvesAgainstBaseRtcmWithRoverUbxFile) {
    namespace fs = std::filesystem;

    const fs::path source_dir = GNSSPP_SOURCE_DIR;
    if (!sourcePathExists(source_dir, "data/driving/base.obs") ||
        !sourcePathExists(source_dir, "data/driving/rover.obs") ||
        !sourcePathExists(source_dir, "data/driving/navigation.nav")) {
        GTEST_SKIP() << "repo driving test data is not available";
    }
    const fs::path binary_dir = GNSSPP_BINARY_DIR;
    const fs::path live_binary = binary_dir / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    io::RINEXReader::RINEXHeader base_header;
    const auto base_epochs =
        loadBaseEpochs(source_dir / "data" / "driving" / "base.obs", base_header, 1);
    ASSERT_EQ(base_epochs.size(), 1U);
    ASSERT_GE(base_epochs[0].getNumSatellites(), 4U);

    const ObservationData rover_epoch =
        findEpochAtTime(source_dir / "data" / "driving" / "rover.obs", base_epochs[0].time);
    ASSERT_FALSE(rover_epoch.isEmpty());
    ASSERT_GE(rover_epoch.getNumSatellites(), 4U);

    const NavigationData nav =
        loadNavigation(source_dir / "data" / "driving" / "navigation.nav");

    io::RTCMProcessor encoder;
    std::set<SatelliteId> satellites;
    for (const auto& obs : base_epochs[0].observations) satellites.insert(obs.satellite);
    for (const auto& obs : rover_epoch.observations) satellites.insert(obs.satellite);

    std::vector<std::vector<uint8_t>> base_frames;
    base_frames.push_back(buildRtcm1005(base_header.approximate_position.x(),
                                        base_header.approximate_position.y(),
                                        base_header.approximate_position.z()));

    size_t nav_frames = 0;
    for (const auto& satellite : satellites) {
        const Ephemeris* eph = nav.getEphemeris(satellite, rover_epoch.time);
        if (eph == nullptr || satellite.system != GNSSSystem::GPS) {
            continue;
        }
        const auto message = encoder.encodeEphemeris(*eph);
        ASSERT_TRUE(message.valid);
        base_frames.push_back(buildRtcmFrame(message));
        ++nav_frames;
    }
    ASSERT_GE(nav_frames, 4U);

    const auto base_message =
        encoder.encodeObservations(base_epochs[0], io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(base_message.valid);
    base_frames.push_back(buildRtcmFrame(base_message));

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_ubx_test");
    const fs::path rover_path = temp_dir / "rover.ubx";
    const fs::path base_path = temp_dir / "base.rtcm3";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";

    writeBinaryFile(base_path, base_frames);
    writeBinaryFile(rover_path, {buildUbxRawxMessage(rover_epoch)});

    const std::string run_command =
        "\"" + live_binary.string() + "\" --rover-ubx \"" + rover_path.string() +
        "\" --base-rtcm \"" + base_path.string() +
        "\" --nav-rinex \"" + (source_dir / "data" / "driving" / "navigation.nav").string() +
        "\" --out \"" + out_path.string() +
        "\" --max-epochs 1 --quiet > \"" + log_path.string() + "\" 2>&1";
    const int run_rc = std::system(run_command.c_str());
    ASSERT_EQ(run_rc, 0) << readTextFile(log_path);

    const std::string log = readTextFile(log_path);
    EXPECT_NE(log.find("rover_source=ubx"), std::string::npos) << log;
    EXPECT_NE(log.find("written_solutions=1"), std::string::npos) << log;
    ASSERT_TRUE(fs::exists(out_path));

    fs::remove_all(temp_dir);
}

#ifndef _WIN32
TEST(GNSSLiveTest, SolvesAgainstBaseRtcmWithRoverUbxSerialDevice) {
    namespace fs = std::filesystem;

    const fs::path source_dir = GNSSPP_SOURCE_DIR;
    if (!sourcePathExists(source_dir, "data/driving/base.obs") ||
        !sourcePathExists(source_dir, "data/driving/rover.obs") ||
        !sourcePathExists(source_dir, "data/driving/navigation.nav")) {
        GTEST_SKIP() << "repo driving test data is not available";
    }
    const fs::path binary_dir = GNSSPP_BINARY_DIR;
    const fs::path live_binary = binary_dir / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    io::RINEXReader::RINEXHeader base_header;
    const auto base_epochs =
        loadBaseEpochs(source_dir / "data" / "driving" / "base.obs", base_header, 1);
    ASSERT_EQ(base_epochs.size(), 1U);

    const ObservationData rover_epoch =
        findEpochAtTime(source_dir / "data" / "driving" / "rover.obs", base_epochs[0].time);
    ASSERT_FALSE(rover_epoch.isEmpty());

    const NavigationData nav =
        loadNavigation(source_dir / "data" / "driving" / "navigation.nav");

    io::RTCMProcessor encoder;
    std::set<SatelliteId> satellites;
    for (const auto& obs : base_epochs[0].observations) satellites.insert(obs.satellite);
    for (const auto& obs : rover_epoch.observations) satellites.insert(obs.satellite);

    std::vector<std::vector<uint8_t>> base_frames;
    base_frames.push_back(buildRtcm1005(base_header.approximate_position.x(),
                                        base_header.approximate_position.y(),
                                        base_header.approximate_position.z()));
    for (const auto& satellite : satellites) {
        const Ephemeris* eph = nav.getEphemeris(satellite, rover_epoch.time);
        if (eph == nullptr || satellite.system != GNSSSystem::GPS) {
            continue;
        }
        const auto message = encoder.encodeEphemeris(*eph);
        ASSERT_TRUE(message.valid);
        base_frames.push_back(buildRtcmFrame(message));
    }
    const auto base_message =
        encoder.encodeObservations(base_epochs[0], io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(base_message.valid);
    base_frames.push_back(buildRtcmFrame(base_message));

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_serial_test");
    const fs::path base_path = temp_dir / "base.rtcm3";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";
    writeBinaryFile(base_path, base_frames);

    PseudoTerminal pty = openPseudoTerminal();
    ASSERT_GE(pty.master_fd, 0);
    ASSERT_FALSE(pty.slave_path.empty());

    const std::vector<uint8_t> rover_ubx = buildUbxRawxMessage(rover_epoch);
    std::thread writer([master_fd = pty.master_fd, rover_ubx]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const ssize_t written = ::write(master_fd, rover_ubx.data(), static_cast<ssize_t>(rover_ubx.size()));
        EXPECT_EQ(written, static_cast<ssize_t>(rover_ubx.size()));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ::close(master_fd);
    });

    const std::string run_command =
        "\"" + live_binary.string() + "\" --rover-ubx \"" + pty.slave_path +
        "\" --rover-ubx-baud 115200 --base-rtcm \"" + base_path.string() +
        "\" --nav-rinex \"" + (source_dir / "data" / "driving" / "navigation.nav").string() +
        "\" --out \"" + out_path.string() +
        "\" --max-epochs 1 --quiet > \"" + log_path.string() + "\" 2>&1";
    const int run_rc = std::system(run_command.c_str());
    writer.join();
    ASSERT_EQ(run_rc, 0) << readTextFile(log_path);

    const std::string log = readTextFile(log_path);
    EXPECT_NE(log.find("rover_source=ubx"), std::string::npos) << log;
    EXPECT_NE(log.find("written_solutions=1"), std::string::npos) << log;
    ASSERT_TRUE(fs::exists(out_path));

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveTest, SolvesAgainstBaseRtcmSerialDeviceWithRoverUbxFile) {
    namespace fs = std::filesystem;

    const fs::path source_dir = GNSSPP_SOURCE_DIR;
    if (!sourcePathExists(source_dir, "data/driving/base.obs") ||
        !sourcePathExists(source_dir, "data/driving/rover.obs") ||
        !sourcePathExists(source_dir, "data/driving/navigation.nav")) {
        GTEST_SKIP() << "repo driving test data is not available";
    }
    const fs::path binary_dir = GNSSPP_BINARY_DIR;
    const fs::path live_binary = binary_dir / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    io::RINEXReader::RINEXHeader base_header;
    const auto base_epochs =
        loadBaseEpochs(source_dir / "data" / "driving" / "base.obs", base_header, 1);
    ASSERT_EQ(base_epochs.size(), 1U);
    ASSERT_GE(base_epochs[0].getNumSatellites(), 4U);

    const ObservationData rover_epoch =
        findEpochAtTime(source_dir / "data" / "driving" / "rover.obs", base_epochs[0].time);
    ASSERT_FALSE(rover_epoch.isEmpty());
    ASSERT_GE(rover_epoch.getNumSatellites(), 4U);

    const NavigationData nav =
        loadNavigation(source_dir / "data" / "driving" / "navigation.nav");

    io::RTCMProcessor encoder;
    std::set<SatelliteId> satellites;
    for (const auto& obs : base_epochs[0].observations) satellites.insert(obs.satellite);
    for (const auto& obs : rover_epoch.observations) satellites.insert(obs.satellite);

    std::vector<std::vector<uint8_t>> base_frames;
    base_frames.push_back(buildRtcm1005(base_header.approximate_position.x(),
                                        base_header.approximate_position.y(),
                                        base_header.approximate_position.z()));
    for (const auto& satellite : satellites) {
        const Ephemeris* eph = nav.getEphemeris(satellite, rover_epoch.time);
        if (eph == nullptr || satellite.system != GNSSSystem::GPS) {
            continue;
        }
        const auto message = encoder.encodeEphemeris(*eph);
        ASSERT_TRUE(message.valid);
        base_frames.push_back(buildRtcmFrame(message));
    }
    const auto base_message =
        encoder.encodeObservations(base_epochs[0], io::RTCMMessageType::RTCM_1004);
    ASSERT_TRUE(base_message.valid);
    base_frames.push_back(buildRtcmFrame(base_message));

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_base_serial_test");
    const fs::path rover_path = temp_dir / "rover.ubx";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";
    writeBinaryFile(rover_path, {buildUbxRawxMessage(rover_epoch)});

    PseudoTerminal pty = openPseudoTerminal();
    ASSERT_GE(pty.master_fd, 0);
    ASSERT_FALSE(pty.slave_path.empty());

    std::thread writer([master_fd = pty.master_fd, base_frames]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for (const auto& frame : base_frames) {
            const ssize_t written =
                ::write(master_fd, frame.data(), static_cast<ssize_t>(frame.size()));
            EXPECT_EQ(written, static_cast<ssize_t>(frame.size()));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ::close(master_fd);
    });

    const std::string run_command =
        "\"" + live_binary.string() + "\" --rover-ubx \"" + rover_path.string() +
        "\" --base-rtcm \"serial://" + pty.slave_path + "?baud=115200\"" +
        " --nav-rinex \"" + (source_dir / "data" / "driving" / "navigation.nav").string() +
        "\" --out \"" + out_path.string() +
        "\" --max-epochs 1 --quiet > \"" + log_path.string() + "\" 2>&1";
    const int run_rc = std::system(run_command.c_str());
    writer.join();
    ASSERT_EQ(run_rc, 0) << readTextFile(log_path);

    const std::string log = readTextFile(log_path);
    EXPECT_NE(log.find("rover_source=ubx"), std::string::npos) << log;
    EXPECT_NE(log.find("written_solutions=1"), std::string::npos) << log;
    ASSERT_TRUE(fs::exists(out_path));

    fs::remove_all(temp_dir);
}
#endif

// ---------------------------------------------------------------------------------------------
// Recorded-data regression tests (tests/fixtures/realtime_stress: 40 s of PPC Tokyo run1, GPS-only
// RTCM3 MSM7). They need no external data and cover stream-fault behaviour found by
// `gnss realtime-stress`.
// ---------------------------------------------------------------------------------------------
namespace {

struct FixtureEpoch {
    size_t offset = 0;
    size_t length = 0;
};

std::filesystem::path realtimeFixtureDir() {
    return std::filesystem::path(GNSSPP_SOURCE_DIR) / "tests" / "fixtures" / "realtime_stress";
}

std::vector<uint8_t> readBinaryFile(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

std::vector<FixtureEpoch> readFixtureIndex(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::vector<FixtureEpoch> epochs;
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream in(line);
        double tow = 0.0;
        int week = 0;
        FixtureEpoch epoch;
        size_t obs = 0;
        size_t aux = 0;
        if (in >> tow >> week >> epoch.offset >> epoch.length >> obs >> aux) {
            epochs.push_back(epoch);
        }
    }
    return epochs;
}

std::vector<uint8_t> epochBytes(const std::vector<uint8_t>& data,
                                const std::vector<FixtureEpoch>& epochs,
                                size_t first,
                                size_t last) {
    std::vector<uint8_t> out;
    for (size_t i = first; i < last && i < epochs.size(); ++i) {
        out.insert(out.end(), data.begin() + static_cast<std::ptrdiff_t>(epochs[i].offset),
                   data.begin() + static_cast<std::ptrdiff_t>(epochs[i].offset + epochs[i].length));
    }
    return out;
}

int runLiveCapture(const std::string& arguments, const std::filesystem::path& log_path) {
    const std::filesystem::path live_binary =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "apps" / "gnss_live";
    const std::string command = "\"" + live_binary.string() + "\" " + arguments + " --quiet > \"" +
                                log_path.string() + "\" 2>&1";
    return std::system(command.c_str());
}

}  // namespace

TEST(GNSSLiveRecordedStreamTest, ReportsRtcmCrcFailuresInDecoderErrorSummary) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "base.rtcm")) << fixture;
    const fs::path live_binary = fs::path(GNSSPP_BINARY_DIR) / "apps" / "gnss_live";
    ASSERT_TRUE(fs::exists(live_binary));

    // Flip one payload bit of a mid-stream base observation message.
    auto base = readBinaryFile(fixture / "base.rtcm");
    const auto base_epochs = readFixtureIndex(fixture / "base.idx");
    ASSERT_GT(base_epochs.size(), 20U);
    const size_t victim = base_epochs[20].offset + base_epochs[20].length - 40;
    base[victim] ^= 0x10U;

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_crc_failures");
    const fs::path base_path = temp_dir / "base_corrupt.rtcm";
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";
    {
        std::ofstream output(base_path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(base.data()), static_cast<std::streamsize>(base.size()));
    }

    const int rc = runLiveCapture("--rover-rtcm \"" + (fixture / "rover.rtcm").string() + "\" --base-rtcm \"" +
                                      base_path.string() + "\" --out \"" + out_path.string() + "\"",
                                  log_path);
    const std::string log = readTextFile(log_path);
    EXPECT_EQ(rc, 0) << log;  // survives the corrupted frame
    EXPECT_GT(extractSummaryDouble(log, "written_solutions"), 100.0) << log;
    EXPECT_GE(extractSummaryDouble(log, "base_decoder_errors"), 1.0) << log;
    EXPECT_EQ(extractSummaryToken(log, "rover_decoder_errors"), "0") << log;

    fs::remove_all(temp_dir);
}


TEST(GNSSLiveRecordedStreamTest, KinematicPostFilterIsOptInAndSmokeRuns) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "base.rtcm")) << fixture;
    const fs::path temp_dir = makeUniqueTempDir("gnss_live_output_guards");
    const std::string inputs = "--rover-rtcm \"" + (fixture / "rover.rtcm").string() + "\" --base-rtcm \"" +
                               (fixture / "base.rtcm").string() + "\" --preset low-cost";

    const fs::path base_out = temp_dir / "baseline.pos";
    const fs::path base_log = temp_dir / "baseline.log";
    ASSERT_EQ(runLiveCapture(inputs + " --out \"" + base_out.string() + "\"", base_log), 0) << readTextFile(base_log);
    const std::string baseline = readTextFile(base_log);
    // Opt-in: without the flag the summary carries no guard counter.
    EXPECT_EQ(baseline.find("post_filter_dropped_epochs"), std::string::npos) << baseline;

    const fs::path guard_out = temp_dir / "guarded.pos";
    const fs::path guard_log = temp_dir / "guarded.log";
    ASSERT_EQ(runLiveCapture(inputs + " --kinematic-post-filter --out \"" + guard_out.string() + "\"",
                             guard_log),
              0)
        << readTextFile(guard_log);
    const std::string guarded = readTextFile(guard_log);
    EXPECT_NE(guarded.find("post_filter_dropped_epochs="), std::string::npos) << guarded;
    const double baseline_written = extractSummaryDouble(baseline, "written_solutions");
    const double guarded_written = extractSummaryDouble(guarded, "written_solutions");
    const double dropped = extractSummaryDouble(guarded, "post_filter_dropped_epochs");
    EXPECT_GT(guarded_written, 0.0) << guarded;
    // Every dropped epoch is a missing row; nothing is added.
    EXPECT_LE(guarded_written, baseline_written) << guarded;
    EXPECT_GE(dropped, 0.0) << guarded;
    EXPECT_LE(extractSummaryDouble(guarded, "fixed_solutions"), extractSummaryDouble(baseline, "fixed_solutions"))
        << guarded;

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveRecordedStreamTest, PosFixDiagnosticsAreOptInAndLeaveOtherColumnsUnchanged) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "base.rtcm")) << fixture;
    const fs::path temp_dir = makeUniqueTempDir("gnss_live_fix_diag");
    const std::string inputs = "--rover-rtcm \"" + (fixture / "rover.rtcm").string() + "\" --base-rtcm \"" +
                               (fixture / "base.rtcm").string() + "\" --preset low-cost";

    const fs::path off_out = temp_dir / "off.pos";
    const fs::path on_out = temp_dir / "on.pos";
    const fs::path log = temp_dir / "run.log";
    ASSERT_EQ(runLiveCapture(inputs + " --out \"" + off_out.string() + "\"", log), 0) << readTextFile(log);
    ASSERT_EQ(runLiveCapture(inputs + " --pos-fix-diagnostics --out \"" + on_out.string() + "\"", log), 0)
        << readTextFile(log);

    const auto split_lines = [](const std::string& text) {
        std::vector<std::string> lines;
        std::istringstream stream(text);
        std::string line;
        while (std::getline(stream, line)) lines.push_back(line);
        return lines;
    };
    const auto off_lines = split_lines(readTextFile(off_out));
    const auto on_lines = split_lines(readTextFile(on_out));
    ASSERT_EQ(off_lines.size(), on_lines.size());
    ASSERT_GT(off_lines.size(), 3u);
    EXPECT_EQ(off_lines[1].find("FloatFixH"), std::string::npos);
    EXPECT_EQ(on_lines[1], off_lines[1] + " FloatFixH(m) FloatFixU(m)");

    size_t fixed_rows = 0;
    for (size_t i = 2; i < off_lines.size(); ++i) {
        // Every column except the two appended ones is byte-identical.
        ASSERT_EQ(on_lines[i].compare(0, off_lines[i].size(), off_lines[i]), 0) << i;
        std::istringstream tail(on_lines[i].substr(off_lines[i].size()));
        std::string h_token;
        std::string u_token;
        ASSERT_TRUE(static_cast<bool>(tail >> h_token >> u_token)) << on_lines[i];
        std::istringstream status_stream(off_lines[i]);
        std::string token;
        int status = -1;
        for (int column = 0; column < 9 && (status_stream >> token); ++column) {
            if (column == 8) status = std::stoi(token);
        }
        if (status == static_cast<int>(libgnss::SolutionStatus::FIXED)) {
            ++fixed_rows;
            EXPECT_TRUE(std::isfinite(std::stod(h_token))) << on_lines[i];
            EXPECT_TRUE(std::isfinite(std::stod(u_token))) << on_lines[i];
            EXPECT_GE(std::stod(h_token), 0.0);
            EXPECT_GE(std::stod(u_token), 0.0);
        } else {
            EXPECT_EQ(h_token, "nan") << on_lines[i];
            EXPECT_EQ(u_token, "nan") << on_lines[i];
        }
    }
    EXPECT_GT(fixed_rows, 0u) << "fixture should contain FIXED epochs";

    // Requires --format pos.
    EXPECT_NE(runLiveCapture(inputs + " --format llh --pos-fix-diagnostics --out \"" +
                                 (temp_dir / "llh.pos").string() + "\"",
                             log),
              0);

    fs::remove_all(temp_dir);
}

// gnss_live reads a UBX rover in 4096-byte chunks. A RAWX-only stream (or a high rate) completes
// several UBX-RXM-RAWX epochs per chunk; the reader used to return only the first and silently
// drop the rest. The same epochs, each alone in its own 4096-byte block, are the reference.
TEST(GNSSLiveRecordedStreamTest, UbxRoverEpochsPackedIntoOneReadAreAllProcessed) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "rover.rtcm")) << fixture;
    ASSERT_TRUE(fs::exists(fixture / "base.rtcm")) << fixture;

    int week = 0;
    {
        std::ifstream index(fixture / "rover.idx");
        double tow = 0.0;
        ASSERT_TRUE(static_cast<bool>(index >> tow >> week));
    }
    ASSERT_GT(week, 0);

    // The recorded 5 Hz rover epochs, re-encoded as UBX-RXM-RAWX (MSM epochs are TOW-only).
    io::RTCMReader reader;
    ASSERT_TRUE(reader.open((fixture / "rover.rtcm").string()));
    io::RTCMStreamDecoder decoder;
    io::RTCMMessage message;
    std::vector<std::vector<uint8_t>> rawx;
    while (reader.readMessage(message)) {
        const auto frame = buildRtcmFrame(message);
        std::vector<io::RTCMStreamDecoder::Event> events;
        decoder.pushFrame(frame.data(), frame.size(), events);
        for (auto& event : events) {
            if (!event.has_observation) continue;
            ObservationData epoch = event.observation;
            if (epoch.time.week == 0) epoch.time.week = week;
            rawx.push_back(buildUbxRawxMessage(epoch));
        }
    }
    const size_t epoch_count = rawx.size();
    ASSERT_GT(epoch_count, 150U);

    constexpr size_t kReadSize = 4096;  // gnss_live's rover read size
    std::vector<uint8_t> packed;
    std::vector<uint8_t> spaced;
    std::map<size_t, size_t> completed_per_read;
    for (const auto& frame : rawx) {
        ASSERT_LT(frame.size(), kReadSize);
        packed.insert(packed.end(), frame.begin(), frame.end());
        ++completed_per_read[(packed.size() - 1) / kReadSize];
        spaced.insert(spaced.end(), frame.begin(), frame.end());
        spaced.resize(spaced.size() + (kReadSize - frame.size()), 0x00);  // one epoch per read
    }
    size_t max_completed = 0;
    for (const auto& [read_index, completed] : completed_per_read) {
        (void)read_index;
        max_completed = std::max(max_completed, completed);
    }
    ASSERT_GE(max_completed, 3U) << "the packed stream must complete several epochs in one read";

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_ubx_packed_rawx");
    const fs::path packed_path = temp_dir / "packed.ubx";
    const fs::path spaced_path = temp_dir / "spaced.ubx";
    writeBinaryFile(packed_path, {packed});
    writeBinaryFile(spaced_path, {spaced});

    const std::string base_args = "--base-rtcm \"" + (fixture / "base.rtcm").string() + "\" --preset low-cost";
    const fs::path packed_out = temp_dir / "packed.pos";
    const fs::path spaced_out = temp_dir / "spaced.pos";
    const fs::path packed_log = temp_dir / "packed.log";
    const fs::path spaced_log = temp_dir / "spaced.log";
    ASSERT_EQ(runLiveCapture("--rover-ubx \"" + spaced_path.string() + "\" " + base_args + " --out \"" +
                                 spaced_out.string() + "\"",
                             spaced_log),
              0)
        << readTextFile(spaced_log);
    ASSERT_EQ(runLiveCapture("--rover-ubx \"" + packed_path.string() + "\" " + base_args + " --out \"" +
                                 packed_out.string() + "\"",
                             packed_log),
              0)
        << readTextFile(packed_log);

    const std::string spaced_summary = readTextFile(spaced_log);
    const std::string packed_summary = readTextFile(packed_log);
    const std::string expected = std::to_string(epoch_count);
    // Reference layout: every epoch reaches the solver.
    EXPECT_EQ(extractSummaryToken(spaced_summary, "aligned_epochs"), expected) << spaced_summary;
    EXPECT_EQ(extractSummaryToken(spaced_summary, "rover_valid_messages"), expected) << spaced_summary;
    // Packed layout: the same epochs, the same result.
    EXPECT_EQ(extractSummaryToken(packed_summary, "rover_valid_messages"), expected) << packed_summary;
    EXPECT_EQ(extractSummaryToken(packed_summary, "aligned_epochs"), expected) << packed_summary;
    EXPECT_EQ(extractSummaryToken(packed_summary, "skipped_rover_epochs"), "0") << packed_summary;
    EXPECT_EQ(extractSummaryToken(packed_summary, "termination"), "rover_source_exhausted") << packed_summary;
    EXPECT_GT(extractSummaryDouble(packed_summary, "written_solutions"), 100.0) << packed_summary;
    EXPECT_EQ(extractSummaryToken(packed_summary, "written_solutions"),
              extractSummaryToken(spaced_summary, "written_solutions"));
    EXPECT_EQ(extractSummaryToken(packed_summary, "fixed_solutions"),
              extractSummaryToken(spaced_summary, "fixed_solutions"));
    EXPECT_EQ(readTextFile(packed_out), readTextFile(spaced_out));

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveRecordedStreamTest, RebroadcastEphemeridesDoNotGrowTheNavigationDatabase) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "base.rtcm")) << fixture;

    // Collect every 1019 the recorded base stream carries (each repeated every 20 s).
    io::RTCMReader reader;
    ASSERT_TRUE(reader.open((fixture / "base.rtcm").string()));
    io::RTCMProcessor processor;
    std::vector<NavigationData> per_message;
    io::RTCMMessage message;
    while (reader.readMessage(message)) {
        if (message.type != io::RTCMMessageType::RTCM_1019) continue;
        NavigationData nav;
        if (processor.decodeNavigationData(message, nav)) per_message.push_back(nav);
    }
    ASSERT_GT(per_message.size(), 30U);

    auto count = [](const NavigationData& nav) {
        size_t total = 0;
        for (const auto& [sat, list] : nav.ephemeris_data) { (void)sat; total += list.size(); }
        return total;
    };

    NavigationData grown, bounded;
    for (int pass = 0; pass < 5; ++pass) {
        for (const auto& nav : per_message) {
            libgnss_apps::mergeNavigationData(grown, nav);
            libgnss_apps::mergeNavigationDataIfNew(bounded, nav);
        }
    }
    // The plain merge keeps every rebroadcast; the live-stream merge keeps one copy per record.
    EXPECT_EQ(count(grown), per_message.size() * 5);
    EXPECT_LT(count(bounded), per_message.size());
    EXPECT_GE(count(bounded), 10U);

    // Same guarantee inside the RTCM stream decoder used by gnss_live.
    io::RTCMStreamDecoder decoder;
    reader.close();
    ASSERT_TRUE(reader.open((fixture / "base.rtcm").string()));
    std::vector<uint8_t> frame;
    size_t after_first_pass = 0;
    for (int pass = 0; pass < 3; ++pass) {
        ASSERT_TRUE(reader.open((fixture / "base.rtcm").string()));
        while (reader.readMessage(message)) {
            frame = buildRtcmFrame(message);
            std::vector<io::RTCMStreamDecoder::Event> events;
            decoder.pushFrame(frame.data(), frame.size(), events);
        }
        if (pass == 0) after_first_pass = count(decoder.getNavigationData());
    }
    EXPECT_EQ(count(decoder.getNavigationData()), after_first_pass);
}


namespace {

#ifndef _WIN32
// One scripted peer: each accepted connection replays the next script (bytes, pauses), then is
// closed. After the last script the listener is closed too, so further connects are refused.
class ScriptedRtcmServer {
public:
    struct Step {
        std::vector<uint8_t> bytes;
        int pause_ms_after = 0;
    };

    ScriptedRtcmServer(std::vector<std::vector<Step>> connections, bool ntrip)
        : connections_(std::move(connections)), ntrip_(ntrip) {
        // CLOEXEC: gnss_live is spawned with std::system and must not inherit (and thereby keep alive) the listener.
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        int one = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        EXPECT_EQ(::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
        EXPECT_EQ(::listen(listen_fd_, 4), 0);
        socklen_t len = sizeof(addr);
        ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
        port_ = ntohs(addr.sin_port);
        thread_ = std::thread([this]() { run(); });
    }

    ~ScriptedRtcmServer() {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        if (listen_fd_ >= 0) ::close(listen_fd_);
    }

    int port() const { return port_; }
    int acceptedConnections() const { return accepted_; }

private:
    void run() {
        for (const auto& script : connections_) {
            pollfd pfd{listen_fd_, POLLIN, 0};
            bool ready = false;
            for (int waited = 0; waited < 15000 && !stop_; waited += 50) {
                if (::poll(&pfd, 1, 50) > 0) { ready = true; break; }
            }
            if (!ready) break;
            const int fd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC);
            if (fd < 0) break;
            ++accepted_;
            if (ntrip_) {
                std::string request;
                char buffer[512];
                while (request.find("\r\n\r\n") == std::string::npos) {
                    const ssize_t n = ::recv(fd, buffer, sizeof(buffer), 0);
                    if (n <= 0) break;
                    request.append(buffer, static_cast<size_t>(n));
                }
                const std::string reply = "ICY 200 OK\r\n\r\n";
                ::send(fd, reply.data(), reply.size(), MSG_NOSIGNAL);
            }
            for (const auto& step : script) {
                size_t sent = 0;
                while (sent < step.bytes.size()) {
                    const ssize_t n = ::send(fd, step.bytes.data() + sent, step.bytes.size() - sent, MSG_NOSIGNAL);
                    if (n <= 0) break;
                    sent += static_cast<size_t>(n);
                }
                if (step.pause_ms_after > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(step.pause_ms_after));
                }
            }
            ::shutdown(fd, SHUT_RDWR);
            ::close(fd);
        }
        ::close(listen_fd_);
        listen_fd_ = -1;
    }

    std::vector<std::vector<Step>> connections_;
    bool ntrip_ = false;
    int listen_fd_ = -1;
    int port_ = 0;
    std::atomic<bool> stop_{false};
    std::atomic<int> accepted_{0};
    std::thread thread_;
};
#endif

}  // namespace

#ifndef _WIN32
TEST(GNSSLiveRecordedStreamTest, ReconnectResumesTcpRoverAfterDroppedConnection) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "rover.rtcm")) << fixture;
    const auto rover = readBinaryFile(fixture / "rover.rtcm");
    const auto rover_epochs = readFixtureIndex(fixture / "rover.idx");
    ASSERT_GT(rover_epochs.size(), 150U);

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_tcp_reconnect");
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";
    const std::string base_arg = "--base-rtcm \"" + (fixture / "base.rtcm").string() + "\"";

    // The rover link drops after 60 epochs and comes back with the rest of the recording.
    auto script = [&]() {
        std::vector<std::vector<ScriptedRtcmServer::Step>> connections(2);
        connections[0].push_back({epochBytes(rover, rover_epochs, 0, 60), 0});
        connections[1].push_back({epochBytes(rover, rover_epochs, 70, rover_epochs.size()), 0});
        return connections;
    };

    double without_reconnect = 0.0;
    {
        ScriptedRtcmServer server(script(), false);
        const int rc = runLiveCapture("--rover-rtcm tcp://127.0.0.1:" + std::to_string(server.port()) + " " + base_arg +
                                          " --out \"" + out_path.string() + "\"",
                                      log_path);
        const std::string log = readTextFile(log_path);
        EXPECT_EQ(rc, 0) << log;
        // Default behaviour is unchanged: the first dropped connection ends the run.
        EXPECT_EQ(extractSummaryToken(log, "termination"), "rover_source_exhausted") << log;
        EXPECT_EQ(server.acceptedConnections(), 1);
        without_reconnect = extractSummaryDouble(log, "written_solutions");
        EXPECT_LE(without_reconnect, 60.0) << log;
    }
    {
        ScriptedRtcmServer server(script(), false);
        const int rc = runLiveCapture("--rover-rtcm tcp://127.0.0.1:" + std::to_string(server.port()) + " " + base_arg +
                                          " --reconnect --reconnect-delay-ms 50 --reconnect-attempts 3 --out \"" +
                                          out_path.string() + "\"",
                                      log_path);
        const std::string log = readTextFile(log_path);
        EXPECT_EQ(rc, 0) << log;
        EXPECT_EQ(server.acceptedConnections(), 2);
        EXPECT_GE(extractSummaryDouble(log, "rover_reconnects"), 1.0) << log;
        EXPECT_GT(extractSummaryDouble(log, "written_solutions"), without_reconnect + 80.0) << log;
        // After the second connection closes the retries are refused and the run ends normally.
        EXPECT_EQ(extractSummaryToken(log, "termination"), "rover_source_exhausted") << log;
    }

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveRecordedStreamTest, NtripFrameSplitAcrossReadsIsNotTheEndOfTheStream) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "rover.rtcm")) << fixture;
    const auto rover = readBinaryFile(fixture / "rover.rtcm");
    const auto rover_epochs = readFixtureIndex(fixture / "rover.idx");
    ASSERT_GT(rover_epochs.size(), 150U);

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_ntrip_split");
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";

    // The 60th epoch's frame reaches the client in two TCP segments 100 ms apart, so one read()
    // returns only half a frame; the caster is otherwise steady.
    const size_t cut = rover_epochs[60].offset + rover_epochs[60].length / 2;
    std::vector<std::vector<ScriptedRtcmServer::Step>> connections(1);
    connections[0].push_back({std::vector<uint8_t>(rover.begin(), rover.begin() + static_cast<std::ptrdiff_t>(cut)), 100});
    connections[0].push_back({std::vector<uint8_t>(rover.begin() + static_cast<std::ptrdiff_t>(cut), rover.end()), 0});
    ScriptedRtcmServer server(std::move(connections), true);

    const int rc = runLiveCapture("--rover-rtcm ntrip://127.0.0.1:" + std::to_string(server.port()) + "/ROVER --base-rtcm \"" +
                                      (fixture / "base.rtcm").string() + "\" --out \"" + out_path.string() + "\"",
                                  log_path);
    const std::string log = readTextFile(log_path);
    EXPECT_EQ(rc, 0) << log;
    EXPECT_GT(extractSummaryDouble(log, "written_solutions"), 180.0) << log;
    EXPECT_EQ(extractSummaryToken(log, "rover_decoder_errors"), "0") << log;

    fs::remove_all(temp_dir);
}

TEST(GNSSLiveRecordedStreamTest, ReconnectKeepsWaitingThroughNtripReadTimeout) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "rover.rtcm")) << fixture;
    const auto rover = readBinaryFile(fixture / "rover.rtcm");
    const auto rover_epochs = readFixtureIndex(fixture / "rover.idx");
    ASSERT_GT(rover_epochs.size(), 150U);

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_ntrip_stall");
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";
    const std::string base_arg = "--base-rtcm \"" + (fixture / "base.rtcm").string() + "\"";

    // The caster goes quiet for 1.6 s (longer than the 1 s NTRIP read timeout), then carries on.
    auto script = [&]() {
        std::vector<std::vector<ScriptedRtcmServer::Step>> connections(1);
        connections[0].push_back({epochBytes(rover, rover_epochs, 0, 60), 1600});
        connections[0].push_back({epochBytes(rover, rover_epochs, 60, rover_epochs.size()), 0});
        return connections;
    };

    double without_reconnect = 0.0;
    {
        ScriptedRtcmServer server(script(), true);
        const int rc = runLiveCapture("--rover-rtcm ntrip://127.0.0.1:" + std::to_string(server.port()) + "/ROVER " +
                                          base_arg + " --out \"" + out_path.string() + "\"",
                                      log_path);
        const std::string log = readTextFile(log_path);
        EXPECT_EQ(rc, 0) << log;
        // Unchanged default: one empty NTRIP read is treated as the end of the stream.
        EXPECT_EQ(extractSummaryToken(log, "termination"), "rover_source_exhausted") << log;
        without_reconnect = extractSummaryDouble(log, "written_solutions");
        EXPECT_LE(without_reconnect, 60.0) << log;
    }
    {
        ScriptedRtcmServer server(script(), true);
        const int rc = runLiveCapture("--rover-rtcm ntrip://127.0.0.1:" + std::to_string(server.port()) + "/ROVER " +
                                          base_arg + " --reconnect --reconnect-delay-ms 50 --reconnect-attempts 4 " +
                                          "--out \"" + out_path.string() + "\"",
                                      log_path);
        const std::string log = readTextFile(log_path);
        EXPECT_EQ(rc, 0) << log;
        EXPECT_GT(extractSummaryDouble(log, "written_solutions"), without_reconnect + 100.0) << log;
    }

    fs::remove_all(temp_dir);
}
#endif

// ---------------------------------------------------------------------------------------------
// --base-outage-fallback spp: rover epochs without a usable base get an SPP row (status 1), never
// FLOAT/FIXED, and the RTK rows are unchanged.
// ---------------------------------------------------------------------------------------------
namespace {

struct PosRow {
    std::string line;
    double tow = 0.0;
    int status = 0;
};

std::vector<PosRow> readPosRows(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::vector<PosRow> rows;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '%') continue;
        std::istringstream in(line);
        double week = 0.0, tow = 0.0, x = 0.0, y = 0.0, z = 0.0, lat = 0.0, lon = 0.0, height = 0.0;
        int status = 0;
        if (in >> week >> tow >> x >> y >> z >> lat >> lon >> height >> status) {
            rows.push_back({line, tow, status});
        }
    }
    return rows;
}

constexpr double kFixtureT0 = 187470.0;
constexpr int kSppStatus = static_cast<int>(SolutionStatus::SPP);

}  // namespace

TEST(GNSSLiveRecordedStreamTest, BaseOutageFallbackWritesSppRowsOnlyWithTheFlagInFileMode) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "base.rtcm")) << fixture;

    // Base epochs 11..20 (tow 187481..187490) are missing: an 11 s hole between base epochs.
    const auto base = readBinaryFile(fixture / "base.rtcm");
    const auto base_epochs = readFixtureIndex(fixture / "base.idx");
    ASSERT_GT(base_epochs.size(), 30U);
    auto gapped = epochBytes(base, base_epochs, 0, 11);
    const auto tail = epochBytes(base, base_epochs, 21, base_epochs.size());
    gapped.insert(gapped.end(), tail.begin(), tail.end());

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_outage_fallback_file");
    const fs::path base_path = temp_dir / "base_gap.rtcm";
    {
        std::ofstream output(base_path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(gapped.data()), static_cast<std::streamsize>(gapped.size()));
    }
    const std::string inputs = "--rover-rtcm \"" + (fixture / "rover.rtcm").string() + "\" --base-rtcm \"" +
                               base_path.string() + "\"";

    const fs::path plain_pos = temp_dir / "plain.pos";
    const fs::path plain_log = temp_dir / "plain.log";
    ASSERT_EQ(runLiveCapture(inputs + " --out \"" + plain_pos.string() + "\"", plain_log), 0) << readTextFile(plain_log);
    const std::string plain = readTextFile(plain_log);
    // Opt-in: no counter without the flag.
    EXPECT_EQ(plain.find("fallback_spp_epochs"), std::string::npos) << plain;

    const fs::path fb_pos = temp_dir / "fallback.pos";
    const fs::path fb_log = temp_dir / "fallback.log";
    ASSERT_EQ(runLiveCapture(inputs + " --base-outage-fallback spp --out \"" + fb_pos.string() + "\"", fb_log), 0)
        << readTextFile(fb_log);
    const std::string fb = readTextFile(fb_log);
    EXPECT_GT(extractSummaryDouble(fb, "fallback_spp_epochs"), 30.0) << fb;

    const auto plain_rows = readPosRows(plain_pos);
    const auto fb_rows = readPosRows(fb_pos);
    ASSERT_FALSE(plain_rows.empty());

    // Strictly inside the hole (past the 0.5 s base hold, before the next base epoch).
    const double hole_begin = kFixtureT0 + 11.0;
    const double hole_end = kFixtureT0 + 20.0;
    size_t plain_in_hole = 0, fb_in_hole = 0;
    for (const auto& row : plain_rows) {
        if (row.tow >= hole_begin && row.tow <= hole_end) ++plain_in_hole;
    }
    for (const auto& row : fb_rows) {
        if (row.tow >= hole_begin && row.tow <= hole_end) {
            ++fb_in_hole;
            EXPECT_EQ(row.status, kSppStatus) << row.line;  // never FLOAT/FIXED in the hole
        }
    }
    EXPECT_EQ(plain_in_hole, 0U);
    EXPECT_GE(fb_in_hole, 40U);  // 9 s at 5 Hz is 46 epochs

    // Every non-SPP row of the fallback run is a row of the plain run, in the same order: the RTK
    // filter state is not touched by the fallback epochs.
    std::vector<std::string> fb_rtk, plain_all;
    for (const auto& row : fb_rows) {
        if (row.status != kSppStatus) fb_rtk.push_back(row.line);
    }
    for (const auto& row : plain_rows) plain_all.push_back(row.line);
    EXPECT_EQ(fb_rtk, plain_all);

    fs::remove_all(temp_dir);
}

#ifndef _WIN32
TEST(GNSSLiveRecordedStreamTest, BaseOutageFallbackKeepsTheRoverFlowingWhileATcpBaseIsSilent) {
    namespace fs = std::filesystem;
    const fs::path fixture = realtimeFixtureDir();
    ASSERT_TRUE(fs::exists(fixture / "rover.rtcm")) << fixture;
    const auto rover = readBinaryFile(fixture / "rover.rtcm");
    const auto rover_epochs = readFixtureIndex(fixture / "rover.idx");
    const auto base = readBinaryFile(fixture / "base.rtcm");
    const auto base_epochs = readFixtureIndex(fixture / "base.idx");
    ASSERT_GT(rover_epochs.size(), 150U);

    const fs::path temp_dir = makeUniqueTempDir("gnss_live_outage_fallback_tcp");
    const fs::path out_path = temp_dir / "live.pos";
    const fs::path log_path = temp_dir / "live.log";

    // The base sends 11 epochs (tow 187470..187480), then the link stays up and silent for 4 s and
    // closes. The rover sends its whole recording at once.
    auto rover_script = [&]() {
        std::vector<std::vector<ScriptedRtcmServer::Step>> connections(1);
        connections[0].push_back({epochBytes(rover, rover_epochs, 0, rover_epochs.size()), 0});
        return connections;
    };
    auto base_script = [&]() {
        std::vector<std::vector<ScriptedRtcmServer::Step>> connections(1);
        connections[0].push_back({epochBytes(base, base_epochs, 0, 11), 4000});
        return connections;
    };

    {
        // Default: the base read blocks, the rover is not read, nothing is written during the outage.
        ScriptedRtcmServer rover_server(rover_script(), false);
        ScriptedRtcmServer base_server(base_script(), false);
        const int rc = runLiveCapture("--rover-rtcm tcp://127.0.0.1:" + std::to_string(rover_server.port()) +
                                          " --base-rtcm tcp://127.0.0.1:" + std::to_string(base_server.port()) +
                                          " --out \"" + out_path.string() + "\"",
                                      log_path);
        const std::string log = readTextFile(log_path);
        EXPECT_EQ(rc, 0) << log;
        EXPECT_EQ(log.find("fallback_spp_epochs"), std::string::npos) << log;
        EXPECT_EQ(extractSummaryToken(log, "termination"), "base_source_exhausted") << log;
        EXPECT_LE(extractSummaryDouble(log, "written_solutions"), 60.0) << log;
    }
    {
        ScriptedRtcmServer rover_server(rover_script(), false);
        ScriptedRtcmServer base_server(base_script(), false);
        const auto begin = std::chrono::steady_clock::now();
        const int rc = runLiveCapture("--rover-rtcm tcp://127.0.0.1:" + std::to_string(rover_server.port()) +
                                          " --base-rtcm tcp://127.0.0.1:" + std::to_string(base_server.port()) +
                                          " --base-outage-fallback spp --out \"" + out_path.string() + "\"",
                                      log_path);
        const double wall_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        const std::string log = readTextFile(log_path);
        EXPECT_EQ(rc, 0) << log;
        // The rover ends first; the blocked base thread is aborted, not waited for.
        EXPECT_EQ(extractSummaryToken(log, "termination"), "rover_source_exhausted") << log;
        EXPECT_GE(extractSummaryDouble(log, "fallback_spp_epochs"), 100.0) << log;
        EXPECT_LT(wall_s, 3.9) << "fallback run waited for the silent base\n" << log;
        for (const auto& row : readPosRows(out_path)) {
            if (row.tow > kFixtureT0 + 11.0) {
                EXPECT_EQ(row.status, kSppStatus) << row.line;  // never FLOAT/FIXED without a base
            }
        }
    }

    fs::remove_all(temp_dir);
}
#endif
