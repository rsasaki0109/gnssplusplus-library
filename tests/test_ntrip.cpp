#include <gtest/gtest.h>
#include <libgnss++/io/ntrip.hpp>

#include <cmath>
#include <cstdint>
#include <string>
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

#ifndef _WIN32
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
    setUnsignedBits(payload, bit, 12, 7); bit += 12;
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

    std::vector<uint8_t> frame = {0xD3, 0x00, static_cast<uint8_t>(payload.size())};
    frame.insert(frame.end(), payload.begin(), payload.end());
    const uint32_t crc = crc24q(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
    return frame;
}

class LocalNtripServer {
public:
    LocalNtripServer(std::string response_header, std::vector<uint8_t> body)
        : response_header_(std::move(response_header)),
          body_(std::move(body)) {
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

        (void)::send(client_fd, response_header_.data(), static_cast<int>(response_header_.size()), 0);
        if (!body_.empty()) {
            (void)::send(client_fd, reinterpret_cast<const char*>(body_.data()), static_cast<int>(body_.size()), 0);
        }
        ::shutdown(client_fd, SHUT_RDWR);
        ::close(client_fd);
    }

    std::string response_header_;
    std::vector<uint8_t> body_;
    int server_fd_ = -1;
    uint16_t port_ = 0;
    std::thread worker_;
};
#endif

}  // namespace

TEST(NTRIPClientTest, ParsesCasterUrlWithCredentials) {
    io::NTRIPStreamInfo info;
    ASSERT_TRUE(io::NTRIPClient::parseURL("ntrip://demo:secret@caster.example.com:2201/MOUNT1", info));

    EXPECT_EQ(info.scheme, "ntrip");
    EXPECT_EQ(info.host, "caster.example.com");
    EXPECT_EQ(info.port, 2201);
    EXPECT_EQ(info.mountpoint, "MOUNT1");
    EXPECT_EQ(info.username, "demo");
    EXPECT_EQ(info.password, "secret");
}

TEST(NTRIPClientTest, ParsesCasterUrlWithoutCredentials) {
    io::NTRIPStreamInfo info;
    ASSERT_TRUE(io::NTRIPClient::parseURL("caster.example.com/MOUNT2", info));

    EXPECT_EQ(info.host, "caster.example.com");
    EXPECT_EQ(info.port, 2101);
    EXPECT_EQ(info.mountpoint, "MOUNT2");
    EXPECT_TRUE(info.username.empty());
    EXPECT_TRUE(info.password.empty());
}

TEST(NTRIPClientTest, RejectsIncompleteCasterUrl) {
    io::NTRIPStreamInfo info;
    EXPECT_FALSE(io::NTRIPClient::parseURL("ntrip://caster.example.com", info));
    EXPECT_FALSE(io::NTRIPClient::parseURL("ntrip://:badport/MOUNT", info));
}

#ifndef _WIN32
TEST(NTRIPClientTest, ConnectsAndReadsRtcmMessage) {
    static constexpr char kOkHeader[] = "ICY 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n\r\n";
    LocalNtripServer server(kOkHeader, buildRtcm1005(1.0, 2.0, 3.0));
    ASSERT_TRUE(server.isReady());

    io::NTRIPClient client;
    ASSERT_TRUE(client.connect("ntrip://127.0.0.1:" + std::to_string(server.port()) + "/MOUNT1"));

    io::RTCMMessage message;
    ASSERT_TRUE(client.readMessage(message));
    EXPECT_EQ(message.type, io::RTCMMessageType::RTCM_1005);
    EXPECT_EQ(client.getStats().valid_messages, 1U);
}

TEST(NTRIPClientTest, ReadsMultipleRtcmMessagesFromSingleResponse) {
    static constexpr char kOkHeader[] = "ICY 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n\r\n";
    auto first = buildRtcm1005(10.0, 20.0, 30.0);
    auto second = buildRtcm1005(40.0, 50.0, 60.0);
    std::vector<uint8_t> body = first;
    body.insert(body.end(), second.begin(), second.end());
    LocalNtripServer server(kOkHeader, std::move(body));
    ASSERT_TRUE(server.isReady());

    io::NTRIPClient client;
    ASSERT_TRUE(client.connect("ntrip://127.0.0.1:" + std::to_string(server.port()) + "/MOUNT1"));

    std::vector<io::RTCMMessage> messages;
    ASSERT_TRUE(client.readMessages(messages));
    ASSERT_EQ(messages.size(), 2U);
    EXPECT_EQ(messages[0].type, io::RTCMMessageType::RTCM_1005);
    EXPECT_EQ(messages[1].type, io::RTCMMessageType::RTCM_1005);
    EXPECT_EQ(client.getStats().valid_messages, 2U);
}

TEST(NTRIPClientTest, ReportsRejectedCasterResponse) {
    static constexpr char kRejectHeader[] = "HTTP/1.0 401 Unauthorized\r\n\r\n";
    LocalNtripServer server(kRejectHeader, {});
    ASSERT_TRUE(server.isReady());

    io::NTRIPClient client;
    EXPECT_FALSE(client.connect("ntrip://127.0.0.1:" + std::to_string(server.port()) + "/MOUNT1"));
    EXPECT_NE(client.getLastError().find("rejected"), std::string::npos);
}
#endif
