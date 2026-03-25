#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "rtcm.hpp"

namespace libgnss {
namespace io {

struct NTRIPStreamInfo {
    std::string scheme = "ntrip";
    std::string host;
    uint16_t port = 2101;
    std::string mountpoint;
    std::string username;
    std::string password;
};

class NTRIPClient {
public:
    NTRIPClient() = default;
    ~NTRIPClient();

    bool connect(const std::string& url);
    bool connect(const std::string& host,
                 uint16_t port,
                 const std::string& mountpoint,
                 const std::string& username = "",
                 const std::string& password = "");
    void disconnect();

    bool isConnected() const { return socket_fd_ >= 0; }
    bool readMessage(RTCMMessage& message, int timeout_ms = 1000);
    bool readMessages(std::vector<RTCMMessage>& messages, int timeout_ms = 1000);

    RTCMProcessor::RTCMStats getStats() const { return processor_.getStats(); }
    std::string getLastError() const { return last_error_; }

    static bool parseURL(const std::string& url, NTRIPStreamInfo& info);

private:
    int socket_fd_ = -1;
    RTCMProcessor processor_;
    std::vector<uint8_t> buffer_;
    std::deque<RTCMMessage> pending_messages_;
    std::string last_error_;

    static std::string base64Encode(const std::string& input);

    bool sendRequest(const NTRIPStreamInfo& info);
    bool readResponseHeader();
    bool receiveIntoBuffer(int timeout_ms);
    size_t consumeBufferedMessages(std::vector<RTCMMessage>& messages);
};

}  // namespace io
}  // namespace libgnss
