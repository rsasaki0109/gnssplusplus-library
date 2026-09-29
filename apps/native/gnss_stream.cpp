#include <libgnss++/io/rtcm.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "rtcm_frame_common.hpp"
#include "serial_port_common.hpp"

namespace {

using libgnss_apps::crc24q;
using libgnss_apps::parseSerialBaud;
using libgnss_apps::resolveSerialPath;
#ifndef _WIN32
using libgnss_apps::configureSerialPort;
#endif

constexpr uint8_t kRTCMPreamble = 0xD3;

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --input <path|ntrip://...|serial://...|tcp://host:port> [options]\n"
        << "Options:\n"
        << "  --output <file|serial://...|tcp://host:port> Relay decoded RTCM frames to a binary file, serial sink, or TCP sink\n"
        << "  --limit <count>           Stop after this many messages (0 = until EOF)\n"
        << "  --reconnect               Auto-reconnect NTRIP streams after disconnect\n"
        << "  --reconnect-delay-ms <ms> Delay before NTRIP reconnect (default: 2000)\n"
        << "  --stats-json <path>       Write stream stats JSON for gnss web dashboard\n"
        << "  --decode-observations     Print decoded observation epoch summaries\n"
        << "  --decode-navigation       Print decoded navigation message summaries\n"
        << "  --quiet                   Suppress per-message type lines\n"
        << "  --help                    Show this help text\n";
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

std::string jsonEscape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (char ch : value) {
        switch (ch) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            default:
                escaped += ch;
                break;
        }
    }
    return escaped;
}

void writeStreamStatsJson(const std::string& path,
                          const libgnss::io::RTCMReader& reader,
                          size_t message_count,
                          const libgnss::io::RTCMMessage* last_message) {
    const auto stats = reader.getStats();
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{\n";
    out << "  \"source\": \"" << jsonEscape(reader.source()) << "\",\n";
    out << "  \"connected\": " << (reader.isConnected() ? "true" : "false") << ",\n";
    out << "  \"reconnect_count\": " << reader.reconnectCount() << ",\n";
    out << "  \"messages_total\": " << message_count << ",\n";
    out << "  \"valid_messages\": " << stats.valid_messages << ",\n";
    out << "  \"crc_errors\": " << stats.crc_errors << ",\n";
    out << "  \"decode_errors\": " << stats.decode_errors << ",\n";
    if (last_message != nullptr) {
        out << "  \"last_message_type\": "
            << static_cast<uint16_t>(last_message->type) << ",\n";
        out << "  \"last_message_name\": \""
            << jsonEscape(libgnss::io::rtcm_utils::getMessageTypeName(last_message->type))
            << "\",\n";
    } else {
        out << "  \"last_message_type\": null,\n";
        out << "  \"last_message_name\": null,\n";
    }
    out << "  \"message_counts\": {\n";
    bool first = true;
    for (const auto& entry : stats.message_counts) {
        if (!first) {
            out << ",\n";
        }
        first = false;
        out << "    \"" << static_cast<uint16_t>(entry.first) << "\": " << entry.second;
    }
    out << "\n  },\n";
    const auto now = std::chrono::system_clock::now();
    const auto epoch_seconds =
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    out << "  \"updated_at_epoch_s\": " << epoch_seconds << "\n";
    out << "}\n";

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream file(path, std::ios::trunc);
    if (file) {
        file << out.str();
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string input_path;
    std::string output_path;
    std::string stats_json_path;
    size_t limit = 0;
    bool decode_observations = false;
    bool decode_navigation = false;
    bool quiet = false;
    bool reconnect = false;
    int reconnect_delay_ms = 2000;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--limit" && i + 1 < argc) {
            limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--reconnect") {
            reconnect = true;
        } else if (arg == "--reconnect-delay-ms" && i + 1 < argc) {
            reconnect_delay_ms = std::stoi(argv[++i]);
        } else if (arg == "--stats-json" && i + 1 < argc) {
            stats_json_path = argv[++i];
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
    if (reconnect) {
        reader.setAutoReconnect(true, reconnect_delay_ms);
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
    libgnss::io::RTCMMessage last_message;
    bool has_last_message = false;
    while (limit == 0 || message_count < limit) {
        if (!reader.readMessage(message)) {
            if (reconnect && reader.isNtripSource() && (limit == 0 || message_count < limit)) {
                if (!quiet) {
                    std::cerr << "[gnss_stream] waiting for NTRIP data"
                              << " reconnect_count=" << reader.reconnectCount();
                    const std::string error = reader.lastError();
                    if (!error.empty()) {
                        std::cerr << " last_error=" << error;
                    }
                    std::cerr << "\n";
                }
                if (!stats_json_path.empty()) {
                    writeStreamStatsJson(stats_json_path, reader, message_count, nullptr);
                }
                continue;
            }
            break;
        }
        ++message_count;
        last_message = message;
        has_last_message = true;
        if (!stats_json_path.empty()) {
            writeStreamStatsJson(
                stats_json_path,
                reader,
                message_count,
                has_last_message ? &last_message : nullptr);
        }
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
    if (!stats_json_path.empty()) {
        writeStreamStatsJson(
            stats_json_path,
            reader,
            message_count,
            has_last_message ? &last_message : nullptr);
    }
    reader.close();

    const auto stats = reader.getStats();
    std::cout << "summary: messages=" << message_count
              << " valid=" << stats.valid_messages
              << " crc_errors=" << stats.crc_errors
              << " decode_errors=" << stats.decode_errors
              << " reconnect_count=" << reader.reconnectCount() << "\n";
    return 0;
}
