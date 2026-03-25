#include <libgnss++/io/ntrip.hpp>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iterator>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace libgnss {
namespace io {

namespace {

constexpr uint8_t kRTCMPreamble = 0xD3;
constexpr size_t kMaxHeaderSize = 8192;

std::string trimSlashes(std::string value) {
    while (!value.empty() && value.front() == '/') {
        value.erase(value.begin());
    }
    while (!value.empty() && value.back() == '/') {
        value.pop_back();
    }
    return value;
}

}  // namespace

NTRIPClient::~NTRIPClient() {
    disconnect();
}

bool NTRIPClient::parseURL(const std::string& url, NTRIPStreamInfo& info) {
    info = NTRIPStreamInfo{};
    if (url.empty()) {
        return false;
    }

    std::string working = url;
    const auto scheme_pos = working.find("://");
    if (scheme_pos != std::string::npos) {
        info.scheme = working.substr(0, scheme_pos);
        working = working.substr(scheme_pos + 3);
    }

    const auto slash_pos = working.find('/');
    if (slash_pos == std::string::npos || slash_pos == working.size() - 1) {
        return false;
    }

    std::string authority = working.substr(0, slash_pos);
    info.mountpoint = trimSlashes(working.substr(slash_pos));
    if (authority.empty() || info.mountpoint.empty()) {
        return false;
    }

    const auto at_pos = authority.rfind('@');
    std::string host_port = authority;
    if (at_pos != std::string::npos) {
        const std::string credentials = authority.substr(0, at_pos);
        host_port = authority.substr(at_pos + 1);
        const auto colon_pos = credentials.find(':');
        if (colon_pos == std::string::npos) {
            info.username = credentials;
        } else {
            info.username = credentials.substr(0, colon_pos);
            info.password = credentials.substr(colon_pos + 1);
        }
    }

    const auto port_pos = host_port.rfind(':');
    if (port_pos != std::string::npos && host_port.find(':') == port_pos) {
        info.host = host_port.substr(0, port_pos);
        const std::string port_text = host_port.substr(port_pos + 1);
        if (port_text.empty()) {
            return false;
        }
        int parsed_port = 0;
        try {
            parsed_port = std::stoi(port_text);
        } catch (...) {
            return false;
        }
        if (parsed_port <= 0 || parsed_port > 65535) {
            return false;
        }
        info.port = static_cast<uint16_t>(parsed_port);
    } else {
        info.host = host_port;
    }

    return !info.host.empty();
}

bool NTRIPClient::connect(const std::string& url) {
    NTRIPStreamInfo info;
    if (!parseURL(url, info)) {
        last_error_ = "invalid NTRIP URL";
        return false;
    }
    return connect(info.host, info.port, info.mountpoint, info.username, info.password);
}

bool NTRIPClient::connect(const std::string& host,
                          uint16_t port,
                          const std::string& mountpoint,
                          const std::string& username,
                          const std::string& password) {
    disconnect();

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* result = nullptr;
    const std::string port_text = std::to_string(port);
    if (::getaddrinfo(host.c_str(), port_text.c_str(), &hints, &result) != 0) {
        last_error_ = "failed to resolve NTRIP host";
        return false;
    }

    for (addrinfo* addr = result; addr != nullptr; addr = addr->ai_next) {
        socket_fd_ = ::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
        if (socket_fd_ < 0) {
            continue;
        }
        if (::connect(socket_fd_, addr->ai_addr, addr->ai_addrlen) == 0) {
            break;
        }
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    ::freeaddrinfo(result);

    if (socket_fd_ < 0) {
        last_error_ = "failed to connect to NTRIP caster";
        return false;
    }

    NTRIPStreamInfo info;
    info.host = host;
    info.port = port;
    info.mountpoint = trimSlashes(mountpoint);
    info.username = username;
    info.password = password;
    if (!sendRequest(info) || !readResponseHeader()) {
        disconnect();
        return false;
    }

    last_error_.clear();
    return true;
}

void NTRIPClient::disconnect() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    buffer_.clear();
    pending_messages_.clear();
}

bool NTRIPClient::readMessage(RTCMMessage& message, int timeout_ms) {
    if (!pending_messages_.empty()) {
        message = std::move(pending_messages_.front());
        pending_messages_.pop_front();
        return true;
    }

    std::vector<RTCMMessage> messages;
    if (!readMessages(messages, timeout_ms)) {
        return false;
    }

    pending_messages_.insert(pending_messages_.end(),
                             std::make_move_iterator(messages.begin()),
                             std::make_move_iterator(messages.end()));
    if (pending_messages_.empty()) {
        return false;
    }

    message = std::move(pending_messages_.front());
    pending_messages_.pop_front();
    return true;
}

bool NTRIPClient::readMessages(std::vector<RTCMMessage>& messages, int timeout_ms) {
    messages.clear();

    if (!pending_messages_.empty()) {
        messages.insert(messages.end(),
                        std::make_move_iterator(pending_messages_.begin()),
                        std::make_move_iterator(pending_messages_.end()));
        pending_messages_.clear();
        return true;
    }

    consumeBufferedMessages(messages);
    if (!messages.empty()) {
        return true;
    }

    if (!isConnected() || !receiveIntoBuffer(timeout_ms)) {
        return false;
    }

    consumeBufferedMessages(messages);
    return !messages.empty();
}

std::string NTRIPClient::base64Encode(const std::string& input) {
    static const char alphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string output;
    output.reserve(((input.size() + 2) / 3) * 4);

    uint32_t value = 0;
    int bits = -6;
    for (unsigned char byte : input) {
        value = (value << 8) | byte;
        bits += 8;
        while (bits >= 0) {
            output.push_back(alphabet[(value >> bits) & 0x3F]);
            bits -= 6;
        }
    }
    if (bits > -6) {
        output.push_back(alphabet[((value << 8) >> (bits + 8)) & 0x3F]);
    }
    while (output.size() % 4 != 0) {
        output.push_back('=');
    }
    return output;
}

bool NTRIPClient::sendRequest(const NTRIPStreamInfo& info) {
    std::string request = "GET /" + info.mountpoint + " HTTP/1.0\r\n";
    request += "Host: " + info.host + "\r\n";
    request += "User-Agent: NTRIP libgnss++/0.1\r\n";
    request += "Accept: */*\r\n";
    request += "Ntrip-Version: Ntrip/2.0\r\n";
    if (!info.username.empty()) {
        request += "Authorization: Basic " +
                   base64Encode(info.username + ":" + info.password) + "\r\n";
    }
    request += "\r\n";

    const ssize_t sent = ::send(socket_fd_, request.data(), request.size(), 0);
    if (sent < 0 || static_cast<size_t>(sent) != request.size()) {
        last_error_ = "failed to send NTRIP request";
        return false;
    }
    return true;
}

bool NTRIPClient::readResponseHeader() {
    while (buffer_.size() < kMaxHeaderSize) {
        const auto header_end =
            std::search(buffer_.begin(), buffer_.end(), "\r\n\r\n", "\r\n\r\n" + 4);
        if (header_end != buffer_.end()) {
            const size_t header_size = static_cast<size_t>(header_end - buffer_.begin()) + 4;
            const std::string header(buffer_.begin(), buffer_.begin() + header_size);
            const auto line_end = header.find("\r\n");
            const std::string status_line = header.substr(0, line_end);
            const bool success = status_line.rfind("ICY 200", 0) == 0 ||
                                 status_line.rfind("HTTP/1.0 200", 0) == 0 ||
                                 status_line.rfind("HTTP/1.1 200", 0) == 0;
            if (!success) {
                last_error_ = "NTRIP caster rejected request: " + status_line;
                return false;
            }
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(header_size));
            return true;
        }

        uint8_t chunk[1024];
        const ssize_t received = ::recv(socket_fd_, chunk, sizeof(chunk), 0);
        if (received <= 0) {
            last_error_ = "failed to read NTRIP response header";
            return false;
        }
        buffer_.insert(buffer_.end(), chunk, chunk + received);
    }

    last_error_ = "NTRIP response header too large";
    return false;
}

bool NTRIPClient::receiveIntoBuffer(int timeout_ms) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int ready = ::select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready < 0) {
        last_error_ = std::string("select failed: ") + std::strerror(errno);
        return false;
    }
    if (ready == 0) {
        return false;
    }

    uint8_t chunk[4096];
    const ssize_t received = ::recv(socket_fd_, chunk, sizeof(chunk), 0);
    if (received <= 0) {
        last_error_ = "NTRIP stream closed";
        disconnect();
        return false;
    }

    buffer_.insert(buffer_.end(), chunk, chunk + received);
    return true;
}

size_t NTRIPClient::consumeBufferedMessages(std::vector<RTCMMessage>& messages) {
    size_t consumed = 0;
    size_t offset = 0;

    while (offset < buffer_.size()) {
        if (buffer_[offset] != kRTCMPreamble) {
            ++offset;
            consumed = offset;
            continue;
        }

        if (offset + 3 > buffer_.size()) {
            break;
        }

        const uint16_t payload_length =
            static_cast<uint16_t>(((buffer_[offset + 1] & 0x03U) << 8) | buffer_[offset + 2]);
        const size_t total_length = 3U + payload_length + 3U;
        if (offset + total_length > buffer_.size()) {
            break;
        }

        auto decoded = processor_.decode(buffer_.data() + offset, total_length);
        if (!decoded.empty()) {
            messages.insert(messages.end(),
                            std::make_move_iterator(decoded.begin()),
                            std::make_move_iterator(decoded.end()));
            offset += total_length;
            consumed = offset;
            continue;
        }

        ++offset;
        consumed = offset;
    }

    if (consumed > 0) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(consumed));
    }
    return consumed;
}

}  // namespace io
}  // namespace libgnss
