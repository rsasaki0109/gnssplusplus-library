#include <libgnss++/io/rtcm.hpp>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {

constexpr uint8_t kRTCMPreamble = 0xD3;
constexpr int kDefaultSerialBaud = 115200;

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

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --input <path|ntrip://...|serial://...|tcp://host:port> [options]\n"
        << "Options:\n"
        << "  --output <file|serial://...|tcp://host:port> Relay decoded RTCM frames to a binary file, serial sink, or TCP sink\n"
        << "  --limit <count>           Stop after this many messages (0 = until EOF)\n"
        << "  --decode-observations     Print decoded observation epoch summaries\n"
        << "  --decode-navigation       Print decoded navigation message summaries\n"
        << "  --quiet                   Suppress per-message type lines\n"
        << "  --help                    Show this help text\n";
}

std::string resolveSerialPath(const std::string& path) {
    constexpr const char* kPrefix = "serial://";
    if (path.rfind(kPrefix, 0) != 0) {
        return path;
    }

    std::string value = path.substr(std::char_traits<char>::length(kPrefix));
    const size_t query_pos = value.find('?');
    if (query_pos != std::string::npos) {
        value.resize(query_pos);
    }
    if (!value.empty() && value.front() != '/') {
        return value;
    }
    while (value.size() > 1 && value[0] == '/' && value[1] == '/') {
        value.erase(value.begin());
    }
    return value;
}

int parseSerialBaud(const std::string& path) {
    constexpr const char* kQuery = "?baud=";
    const size_t query_pos = path.find(kQuery);
    if (query_pos == std::string::npos) {
        return kDefaultSerialBaud;
    }
    const std::string baud_text =
        path.substr(query_pos + std::char_traits<char>::length(kQuery));
    return baud_text.empty() ? kDefaultSerialBaud : std::stoi(baud_text);
}

bool isTcpPath(const std::string& path) {
    return path.rfind("tcp://", 0) == 0;
}

struct TcpEndpoint {
    std::string host;
    std::string port;
};

TcpEndpoint parseTcpEndpoint(const std::string& path) {
    constexpr const char* kPrefix = "tcp://";
    if (path.rfind(kPrefix, 0) != 0) {
        throw std::invalid_argument("TCP sink must start with tcp://");
    }

    const std::string target = path.substr(std::char_traits<char>::length(kPrefix));
    const size_t colon_pos = target.rfind(':');
    if (colon_pos == std::string::npos || colon_pos == 0 || colon_pos + 1 >= target.size()) {
        throw std::invalid_argument("TCP sink must be tcp://host:port");
    }

    TcpEndpoint endpoint;
    endpoint.host = target.substr(0, colon_pos);
    endpoint.port = target.substr(colon_pos + 1);
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

int connectTcpSocket(const std::string& path) {
    const auto endpoint = parseTcpEndpoint(path);

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* result = nullptr;
    if (getaddrinfo(endpoint.host.c_str(), endpoint.port.c_str(), &hints, &result) != 0) {
        return -1;
    }

    int fd = -1;
    for (addrinfo* cursor = result; cursor != nullptr; cursor = cursor->ai_next) {
        fd = ::socket(cursor->ai_family, cursor->ai_socktype, cursor->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (::connect(fd, cursor->ai_addr, cursor->ai_addrlen) == 0) {
            break;
        }
        ::close(fd);
        fd = -1;
    }

    freeaddrinfo(result);
    return fd;
}
#endif

struct RelaySink {
    std::ofstream file;
#ifndef _WIN32
    int serial_fd = -1;
    int tcp_fd = -1;
#endif

    bool open(const std::string& output_path) {
        close();

        if (isTcpPath(output_path)) {
#ifndef _WIN32
            tcp_fd = connectTcpSocket(output_path);
            return tcp_fd >= 0;
#else
            (void)output_path;
            return false;
#endif
        }

        std::error_code ec;
        const std::string serial_path = resolveSerialPath(output_path);
        const auto status = std::filesystem::status(serial_path, ec);
        const bool wants_serial =
            output_path.rfind("serial://", 0) == 0 ||
            (!ec && std::filesystem::is_character_file(status));

        if (!wants_serial) {
            file.open(output_path, std::ios::binary);
            return file.is_open();
        }

#ifndef _WIN32
        const int fd = ::open(serial_path.c_str(), O_WRONLY | O_NOCTTY);
        if (fd < 0) {
            return false;
        }
        if (!configureSerialPort(fd, parseSerialBaud(output_path))) {
            ::close(fd);
            return false;
        }
        serial_fd = fd;
        return true;
#else
        (void)output_path;
        return false;
#endif
    }

    void close() {
        if (file.is_open()) {
            file.close();
        }
#ifndef _WIN32
        if (serial_fd >= 0) {
            ::close(serial_fd);
            serial_fd = -1;
        }
        if (tcp_fd >= 0) {
            ::close(tcp_fd);
            tcp_fd = -1;
        }
#endif
    }

    bool isOpen() const {
        if (file.is_open()) {
            return true;
        }
#ifndef _WIN32
        if (serial_fd >= 0) {
            return true;
        }
        if (tcp_fd >= 0) {
            return true;
        }
#endif
        return false;
    }

    bool write(const libgnss::io::RTCMMessage& message) {
        const uint16_t payload_length = static_cast<uint16_t>(message.data.size());
        std::string frame;
        frame.resize(3 + payload_length + 3);
        frame[0] = static_cast<char>(kRTCMPreamble);
        frame[1] = static_cast<char>((payload_length >> 8) & 0x03U);
        frame[2] = static_cast<char>(payload_length & 0xFFU);
        std::copy(message.data.begin(), message.data.end(), frame.begin() + 3);
        const uint32_t crc =
            crc24q(reinterpret_cast<const uint8_t*>(frame.data()), 3 + payload_length);
        frame[3 + payload_length] = static_cast<char>((crc >> 16) & 0xFFU);
        frame[4 + payload_length] = static_cast<char>((crc >> 8) & 0xFFU);
        frame[5 + payload_length] = static_cast<char>(crc & 0xFFU);

        if (file.is_open()) {
            file.write(frame.data(), static_cast<std::streamsize>(frame.size()));
            return static_cast<bool>(file);
        }
#ifndef _WIN32
        if (serial_fd >= 0) {
            const char* data = frame.data();
            size_t remaining = frame.size();
            while (remaining > 0) {
                const ssize_t count = ::write(serial_fd, data, remaining);
                if (count < 0) {
                    return false;
                }
                remaining -= static_cast<size_t>(count);
                data += count;
            }
            return true;
        }
        if (tcp_fd >= 0) {
            const char* data = frame.data();
            size_t remaining = frame.size();
            while (remaining > 0) {
                const ssize_t count = ::send(tcp_fd, data, remaining, 0);
                if (count < 0) {
                    if (errno == EINTR) {
                        continue;
                    }
                    return false;
                }
                remaining -= static_cast<size_t>(count);
                data += count;
            }
            return true;
        }
#endif
        return false;
    }
};

}  // namespace

int main(int argc, char** argv) {
    std::string input_path;
    std::string output_path;
    size_t limit = 0;
    bool decode_observations = false;
    bool decode_navigation = false;
    bool quiet = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--limit" && i + 1 < argc) {
            limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--decode-observations") {
            decode_observations = true;
        } else if (arg == "--decode-navigation") {
            decode_navigation = true;
        } else if (arg == "--quiet") {
            quiet = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Error: unknown or incomplete argument: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (input_path.empty()) {
        std::cerr << "Error: --input is required\n";
        printUsage(argv[0]);
        return 1;
    }

    libgnss::io::RTCMReader reader;
    if (!reader.open(input_path)) {
        std::cerr << "Error: failed to open RTCM source: " << input_path << "\n";
        return 1;
    }

    RelaySink output_sink;
    if (!output_path.empty()) {
        if (!output_sink.open(output_path)) {
            std::cerr << "Error: failed to open output sink: " << output_path << "\n";
            return 1;
        }
    }

    libgnss::io::RTCMProcessor processor;
    size_t message_count = 0;
    libgnss::io::RTCMMessage message;
    while ((limit == 0 || message_count < limit) && reader.readMessage(message)) {
        ++message_count;
        if (output_sink.isOpen() && !output_sink.write(message)) {
            std::cerr << "Error: failed to relay RTCM frame to output sink\n";
            output_sink.close();
            reader.close();
            return 1;
        }

        if (!quiet) {
            std::cout << std::setw(5) << message_count << " "
                      << libgnss::io::rtcm_utils::getMessageTypeName(message.type)
                      << " (" << static_cast<uint16_t>(message.type) << ")\n";
        }

        if (decode_observations && libgnss::io::rtcm_utils::isObservationMessage(message.type)) {
            libgnss::ObservationData obs_data;
            if (processor.decodeObservationData(message, obs_data)) {
                std::cout << "  obs: week=" << obs_data.time.week
                          << " tow=" << std::fixed << std::setprecision(3) << obs_data.time.tow
                          << " sats=" << obs_data.getNumSatellites()
                          << " obs=" << obs_data.observations.size() << "\n";
            }
        }
        if (decode_navigation && libgnss::io::rtcm_utils::isEphemerisMessage(message.type)) {
            libgnss::NavigationData nav_data;
            if (processor.decodeNavigationData(message, nav_data) && !nav_data.ephemeris_data.empty()) {
                const auto& eph = nav_data.ephemeris_data.begin()->second.back();
                std::cout << "  nav: sat=" << eph.satellite.toString()
                          << " week=" << eph.week << " toe=" << eph.toe.tow << "\n";
            }
        }
    }

    output_sink.close();
    reader.close();

    const auto stats = reader.getStats();
    std::cout << "summary: messages=" << message_count
              << " valid=" << stats.valid_messages
              << " crc_errors=" << stats.crc_errors
              << " decode_errors=" << stats.decode_errors << "\n";
    return 0;
}
