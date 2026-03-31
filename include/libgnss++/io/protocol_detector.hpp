#pragma once

#include <cstdint>
#include <cstddef>

namespace libgnss {
namespace io {

enum class ProtocolType { UBX, SBF, NMEA, RTCM3, UNKNOWN };

/**
 * @brief Auto-detect GNSS receiver binary protocol from raw byte stream.
 *
 * Feed bytes incrementally. Detection is confirmed after 3 valid sync
 * patterns of the same protocol type.
 */
class ProtocolDetector {
public:
    /// Feed bytes. Returns detected protocol (UNKNOWN until confirmed).
    ProtocolType detect(const uint8_t* buffer, size_t size);

    bool isDetected() const { return detected_; }
    ProtocolType getProtocol() const { return protocol_; }
    void reset();

private:
    bool detected_ = false;
    ProtocolType protocol_ = ProtocolType::UNKNOWN;
    int ubx_count_ = 0;
    int sbf_count_ = 0;
    int nmea_count_ = 0;
    int rtcm_count_ = 0;

    static constexpr int kDetectionThreshold = 3;
};

}  // namespace io
}  // namespace libgnss
