#pragma once

// serial:// source/sink helpers shared by the native stream tools
// (gnss_convert, gnss_ubx_info, gnss_stream, gnss_live).

#include <cstddef>
#include <stdexcept>
#include <string>

#ifndef _WIN32
#include <termios.h>
#endif

namespace libgnss_apps {

inline constexpr int kDefaultSerialBaud = 115200;

// Strips the serial:// prefix and any ?query suffix; repeated leading
// slashes collapse to one.
inline std::string resolveSerialPath(const std::string& source) {
    constexpr const char* kPrefix = "serial://";
    if (source.rfind(kPrefix, 0) != 0) {
        return source;
    }

    std::string path = source.substr(std::char_traits<char>::length(kPrefix));
    const size_t query_pos = path.find('?');
    if (query_pos != std::string::npos) {
        path.resize(query_pos);
    }
    if (!path.empty() && path.front() != '/') {
        return path;
    }
    while (path.size() > 1 && path[0] == '/' && path[1] == '/') {
        path.erase(path.begin());
    }
    return path;
}

inline int parseSerialBaud(const std::string& source) {
    constexpr const char* kQuery = "?baud=";
    const size_t query_pos = source.find(kQuery);
    if (query_pos == std::string::npos) {
        return kDefaultSerialBaud;
    }
    const std::string baud_text =
        source.substr(query_pos + std::char_traits<char>::length(kQuery));
    if (baud_text.empty()) {
        return kDefaultSerialBaud;
    }
    return std::stoi(baud_text);
}

#ifndef _WIN32
inline speed_t baudRateConstant(int baud) {
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

// Raw 8N1 without flow control; returns false when the port attributes
// cannot be read or applied.
inline bool configureSerialPort(int fd, int baud) {
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
#endif

}  // namespace libgnss_apps
