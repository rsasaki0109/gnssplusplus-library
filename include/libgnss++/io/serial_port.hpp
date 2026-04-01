#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace libgnss {
namespace io {

/**
 * @brief POSIX termios-based serial port for GNSS receiver communication.
 */
class SerialPort {
public:
    struct Config {
        std::string device = "/dev/ttyUSB0";
        uint32_t baud_rate = 115200;
        uint8_t data_bits = 8;
        uint8_t stop_bits = 1;
        bool parity = false;
        int read_timeout_ms = 100;
    };

    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const Config& config);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    /// Non-blocking read. Returns bytes actually read, or -1 on error.
    ssize_t read(uint8_t* buffer, size_t max_bytes);

    /// Write data to the port. Returns bytes written, or -1 on error.
    ssize_t write(const uint8_t* data, size_t length);

    /// Block until data is available or timeout expires. Returns true if data ready.
    bool waitForData(int timeout_ms = 1000);

    /// Get the underlying file descriptor (for external select/poll).
    int getFd() const { return fd_; }

    const Config& getConfig() const { return config_; }

private:
    int fd_ = -1;
    Config config_;

    bool configureTTY();
    static unsigned int baudToSpeed(uint32_t baud);
};

}  // namespace io
}  // namespace libgnss
