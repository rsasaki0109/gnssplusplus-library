#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>

#ifndef _WIN32
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#endif

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
    EXPECT_NE(log.find("interpolated_base_epochs=1"), std::string::npos) << log;
    EXPECT_NE(log.find("held_base_epochs=0"), std::string::npos) << log;
    EXPECT_NE(log.find("written_solutions=1"), std::string::npos) << log;
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
    EXPECT_NE(no_interp_log.find("interpolated_base_epochs=0"), std::string::npos) << no_interp_log;
    EXPECT_NE(no_interp_log.find("held_base_epochs=0"), std::string::npos) << no_interp_log;
    EXPECT_NE(no_interp_log.find("written_solutions=0"), std::string::npos) << no_interp_log;

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
    EXPECT_NE(hold_log.find("held_base_epochs=1"), std::string::npos) << hold_log;
    EXPECT_NE(hold_log.find("written_solutions=1"), std::string::npos) << hold_log;
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
    EXPECT_NE(disabled_log.find("held_base_epochs=0"), std::string::npos) << disabled_log;
    EXPECT_NE(disabled_log.find("written_solutions=0"), std::string::npos) << disabled_log;

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
