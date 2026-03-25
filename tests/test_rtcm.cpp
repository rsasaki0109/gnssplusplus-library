#include <gtest/gtest.h>
#include <libgnss++/io/rtcm.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <thread>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
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
    const double current_utc_tow = now.tow - static_cast<double>(currentLeapSeconds());
    const double current_utc_day_start = std::floor(current_utc_tow / 86400.0) * 86400.0;
    const double current_utc_day_start_gpst =
        current_utc_day_start + static_cast<double>(currentLeapSeconds());

    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GLONASS, 7);
    eph.week = static_cast<uint16_t>(now.week);
    eph.toe = GNSSTime(now.week, current_utc_day_start_gpst + 9.0 * 3600.0);
    eph.tof = GNSSTime(now.week, current_utc_day_start_gpst + 9.0 * 3600.0 + 30.0 * 60.0);
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
#endif

TEST(RTCMUtilsTest, ClassifiesMessageFamilies) {
    EXPECT_TRUE(io::rtcm_utils::isStationMessage(io::RTCMMessageType::RTCM_1005));
    EXPECT_TRUE(io::rtcm_utils::isObservationMessage(io::RTCMMessageType::RTCM_1074));
    EXPECT_TRUE(io::rtcm_utils::isEphemerisMessage(io::RTCMMessageType::RTCM_1019));
    EXPECT_FALSE(io::rtcm_utils::isObservationMessage(io::RTCMMessageType::RTCM_1005));
}

TEST(RTCMUtilsTest, ReportsNamesAndSystems) {
    EXPECT_EQ(io::rtcm_utils::getMessageTypeName(io::RTCMMessageType::RTCM_1005),
              "Reference Station ARP");
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1077),
              GNSSSystem::GPS);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1087),
              GNSSSystem::GLONASS);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1097),
              GNSSSystem::Galileo);
    EXPECT_EQ(io::rtcm_utils::getSystemFromMessageType(io::RTCMMessageType::RTCM_1127),
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
