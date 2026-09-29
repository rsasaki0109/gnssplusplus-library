#pragma once

// File-or-serial UBX input used by gnss_convert and gnss_ubx_info.

#include <libgnss++/io/ubx.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#endif

#include "serial_port_common.hpp"

namespace libgnss_apps {

struct UbxInputSource {
    bool serial = false;
    bool eof = false;
    std::ifstream file;
#ifndef _WIN32
    int serial_fd = -1;
#endif
};

inline void closeUbxInputSource(UbxInputSource& source) {
    if (source.file.is_open()) {
        source.file.close();
    }
#ifndef _WIN32
    if (source.serial_fd >= 0) {
        ::close(source.serial_fd);
        source.serial_fd = -1;
    }
#endif
    source.serial = false;
    source.eof = false;
}

inline bool openUbxInputSource(const std::string& input_path, UbxInputSource& source) {
    closeUbxInputSource(source);

    const std::string resolved_path = resolveSerialPath(input_path);
    std::error_code ec;
    const auto status = std::filesystem::status(resolved_path, ec);
    if (!ec && std::filesystem::is_regular_file(status)) {
        source.file.open(resolved_path, std::ios::binary);
        return source.file.is_open();
    }

#ifndef _WIN32
    const int baud = parseSerialBaud(input_path);
    const int fd = ::open(resolved_path.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        return false;
    }
    if (!configureSerialPort(fd, baud)) {
        ::close(fd);
        return false;
    }
    source.serial = true;
    source.serial_fd = fd;
    return true;
#else
    (void)input_path;
    return false;
#endif
}

inline bool readNextUbxEvents(libgnss::io::UBXStreamDecoder& decoder,
                              UbxInputSource& source,
                              std::vector<libgnss::io::UBXStreamDecoder::Event>& events) {
    events.clear();
    std::array<uint8_t, 4096> chunk{};

    while (true) {
        size_t bytes_read = 0;
        if (source.serial) {
#ifndef _WIN32
            const ssize_t count = ::read(source.serial_fd, chunk.data(), chunk.size());
            if (count < 0) {
                if (errno == EINTR) {
                    continue;
                }
                source.eof = true;
                return false;
            }
            if (count == 0) {
                source.eof = true;
                return false;
            }
            bytes_read = static_cast<size_t>(count);
#else
            source.eof = true;
            return false;
#endif
        } else {
            source.file.read(reinterpret_cast<char*>(chunk.data()),
                             static_cast<std::streamsize>(chunk.size()));
            bytes_read = static_cast<size_t>(source.file.gcount());
            if (bytes_read == 0) {
                source.eof = true;
                return false;
            }
        }

        decoder.pushBytes(chunk.data(), bytes_read, events);
        if (!events.empty()) {
            return true;
        }
    }
}

}  // namespace libgnss_apps
