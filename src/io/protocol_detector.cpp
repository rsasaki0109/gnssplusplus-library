#include "libgnss++/io/protocol_detector.hpp"

namespace libgnss {
namespace io {

ProtocolType ProtocolDetector::detect(const uint8_t* buffer, size_t size) {
    if (detected_) return protocol_;

    for (size_t i = 0; i + 1 < size; ++i) {
        uint8_t b0 = buffer[i];
        uint8_t b1 = buffer[i + 1];

        // UBX: 0xB5 0x62
        if (b0 == 0xB5 && b1 == 0x62) {
            ++ubx_count_;
        }
        // SBF: '$' '@'
        else if (b0 == 0x24 && b1 == 0x40) {
            ++sbf_count_;
        }
        // NMEA: '$G' or '$P'
        else if (b0 == 0x24 && (b1 == 'G' || b1 == 'P')) {
            ++nmea_count_;
        }
        // RTCM3: 0xD3 followed by 6 zero bits in high byte of length
        else if (b0 == 0xD3 && (b1 & 0xFC) == 0x00) {
            ++rtcm_count_;
        }
    }

    // Check threshold
    if (ubx_count_ >= kDetectionThreshold) {
        detected_ = true;
        protocol_ = ProtocolType::UBX;
    } else if (sbf_count_ >= kDetectionThreshold) {
        detected_ = true;
        protocol_ = ProtocolType::SBF;
    } else if (nmea_count_ >= kDetectionThreshold) {
        detected_ = true;
        protocol_ = ProtocolType::NMEA;
    } else if (rtcm_count_ >= kDetectionThreshold) {
        detected_ = true;
        protocol_ = ProtocolType::RTCM3;
    }

    return protocol_;
}

void ProtocolDetector::reset() {
    detected_ = false;
    protocol_ = ProtocolType::UNKNOWN;
    ubx_count_ = 0;
    sbf_count_ = 0;
    nmea_count_ = 0;
    rtcm_count_ = 0;
}

}  // namespace io
}  // namespace libgnss
