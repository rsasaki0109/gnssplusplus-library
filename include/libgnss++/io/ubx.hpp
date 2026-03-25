#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "../core/observation.hpp"
#include "../core/coordinates.hpp"

namespace libgnss {
namespace io {

enum class UBXMessageClass : uint8_t {
    NAV = 0x01,
    RXM = 0x02
};

struct UBXMessage {
    uint8_t message_class = 0;
    uint8_t message_id = 0;
    uint16_t length = 0;
    std::vector<uint8_t> payload;
    bool valid = false;
};

struct UBXNavPVT {
    GNSSTime time;
    GeodeticCoord position_geodetic;
    Vector3d position_ecef = Vector3d::Zero();
    bool valid_time = false;
    bool valid_position = false;
    bool gnss_fix_ok = false;
    bool differential_solution = false;
    uint8_t fix_type = 0;
    uint8_t carrier_solution = 0;
    uint8_t num_sv = 0;
    double horizontal_accuracy_m = 0.0;
    double vertical_accuracy_m = 0.0;
};

class UBXDecoder {
public:
    struct UBXStats {
        size_t total_messages = 0;
        size_t valid_messages = 0;
        size_t checksum_errors = 0;
        std::map<uint16_t, size_t> message_counts;
    };

    UBXDecoder() = default;
    ~UBXDecoder() = default;

    void clear();
    std::vector<UBXMessage> decode(const uint8_t* buffer, size_t size);

    bool decodeNavPVT(const UBXMessage& message, UBXNavPVT& nav_pvt);
    bool decodeRawx(const UBXMessage& message, ObservationData& obs_data);

    UBXStats getStats() const { return stats_; }
    bool hasLastNavPVT() const { return has_last_nav_pvt_; }
    UBXNavPVT getLastNavPVT() const { return last_nav_pvt_; }

private:
    UBXStats stats_;
    UBXNavPVT last_nav_pvt_;
    bool has_last_nav_pvt_ = false;
    uint16_t last_gps_week_ = 0;
    bool has_last_gps_week_ = false;

    static uint16_t messageKey(uint8_t message_class, uint8_t message_id);
    static bool validateChecksum(const uint8_t* data, size_t payload_length, uint8_t ck_a, uint8_t ck_b);
};

namespace ubx_utils {

std::string getMessageName(uint8_t message_class, uint8_t message_id);
GNSSSystem getSystemFromGnssId(uint8_t gnss_id);
bool getSignalType(uint8_t gnss_id, uint8_t sig_id, SignalType& signal_type);

}  // namespace ubx_utils

}  // namespace io
}  // namespace libgnss
