#include "libgnss++/io/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace libgnss {
namespace io {

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const Config& config) {
    close();
    config_ = config;

    fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    if (!configureTTY()) {
        close();
        return false;
    }
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

ssize_t SerialPort::read(uint8_t* buffer, size_t max_bytes) {
    if (fd_ < 0) return -1;
    ssize_t n = ::read(fd_, buffer, max_bytes);
    if (n < 0 && errno == EAGAIN) return 0;  // no data available (non-blocking)
    return n;
}

ssize_t SerialPort::write(const uint8_t* data, size_t length) {
    if (fd_ < 0) return -1;
    return ::write(fd_, data, length);
}

bool SerialPort::waitForData(int timeout_ms) {
    if (fd_ < 0) return false;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    return ret > 0;
}

bool SerialPort::configureTTY() {
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) return false;

    // Raw mode
    cfmakeraw(&tty);

    // Baud rate
    speed_t speed = baudToSpeed(config_.baud_rate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // Data bits
    tty.c_cflag &= ~CSIZE;
    switch (config_.data_bits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        default: tty.c_cflag |= CS8; break;
    }

    // Stop bits
    if (config_.stop_bits == 2) {
        tty.c_cflag |= CSTOPB;
    } else {
        tty.c_cflag &= ~CSTOPB;
    }

    // Parity
    if (config_.parity) {
        tty.c_cflag |= PARENB;
    } else {
        tty.c_cflag &= ~PARENB;
    }

    // No flow control
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL | CREAD;

    // VMIN=0, VTIME=0: non-blocking reads (fd opened with O_NONBLOCK)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    return tcsetattr(fd_, TCSANOW, &tty) == 0;
}

unsigned int SerialPort::baudToSpeed(uint32_t baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

}  // namespace io
}  // namespace libgnss
