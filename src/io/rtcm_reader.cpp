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

bool RTCMReader::open(const std::string& source) {
    close();
    source_ = source;
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

void RTCMReader::setAutoReconnect(bool enabled, int delay_ms) {
    if (ntrip_client_) {
        ntrip_client_->setAutoReconnect(enabled, delay_ms);
    }
}

int RTCMReader::reconnectCount() const {
    return ntrip_client_ ? ntrip_client_->reconnectCount() : 0;
}

bool RTCMReader::isConnected() const {
    return ntrip_client_ ? ntrip_client_->isConnected() : is_open_;
}

std::string RTCMReader::lastError() const {
    return ntrip_client_ ? ntrip_client_->getLastError() : std::string();
}


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
