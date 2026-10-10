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

/**
 * @brief UBX-RXM-SFRBX broadcast navigation data subframe.
 *
 * Payload layout (u-blox M8/F9/X20 interface descriptions): gnssId U1,
 * svId U1, sigId U1 (reserved, 0, before protocol 27), freqId U1 (GLONASS
 * frequency slot + 7), numWords U1, chn U1, version U1 (0x02), reserved U1,
 * then numWords little-endian U4 data words.
 */
struct UBXSfrbx {
    GNSSSystem system = GNSSSystem::UNKNOWN;
    uint8_t sv_id = 0;
    uint8_t signal_id = 0;
    uint8_t frequency_id = 0;
    uint8_t channel = 0;
    uint8_t version = 0;
    std::vector<uint32_t> words;
};

struct UBXSfrbxFrameInfo {
    enum class Kind {
        UNKNOWN,
        GPS_LNAV,
        QZSS_LNAV,
        GAL_INAV,
        BDS_D1,
        BDS_D2,
        GLO_NAV,
        SBAS
    };

    Kind kind = Kind::UNKNOWN;
    int frame_id = 0;
    int page_id = 0;
    bool has_page_id = false;
    bool valid = false;
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
    bool decodeSfrbx(const UBXMessage& message, UBXSfrbx& sfrbx);

    UBXStats getStats() const { return stats_; }
    bool hasLastNavPVT() const { return has_last_nav_pvt_; }
    UBXNavPVT getLastNavPVT() const { return last_nav_pvt_; }

private:
    UBXStats stats_;
    UBXNavPVT last_nav_pvt_;
    bool has_last_nav_pvt_ = false;
    uint16_t last_gps_week_ = 0;
    bool has_last_gps_week_ = false;

    /// Per (gnssId, svId, sigId) carrier-phase tracking state carried across
    /// UBX-RXM-RAWX epochs for cycle-slip detection (RTKLIB demo5
    /// decode_rxmrawx lockt/halfc/lockflag).
    struct RawxTrackState {
        double lock_time_s = 0.0;   ///< locktime of the previous epoch [s]
        bool half_subtracted = false; ///< previous trkStat halfSub bit
        bool slip_pending = false;  ///< slip seen, not yet reported on a valid phase
    };
    std::map<uint32_t, RawxTrackState> rawx_track_state_;
    /// Set once a sigId > 1 is seen (u-blox Gen9 / F9 family); selects the
    /// looser cpStdev validity threshold.
    bool rawx_gen9_receiver_ = false;

    static uint16_t messageKey(uint8_t message_class, uint8_t message_id);
    static bool validateChecksum(const uint8_t* data, size_t payload_length, uint8_t ck_a, uint8_t ck_b);
};

namespace ubx_utils {

std::string getMessageName(uint8_t message_class, uint8_t message_id);
GNSSSystem getSystemFromGnssId(uint8_t gnss_id);
bool getSignalType(uint8_t gnss_id, uint8_t sig_id, SignalType& signal_type);
/**
 * @brief True when the subframe carries a legacy navigation message handled
 * by the frame decoders: GPS/QZSS L1 C/A LNAV (sigId 0), Galileo E1-B /
 * E5b-I I/NAV (sigId 1 / 5), BeiDou B1I / B2I / B3I D1 or D2 (sigId 0-4,
 * 10), GLONASS L1OF / L2OF strings (sigId 0 / 2) and SBAS L1 (sigId 0).
 * CNAV, F/NAV, E6, B-CNAV and other signals return false.  Receivers
 * older than protocol 27 report sigId 0 and only track these signals.
 */
bool isSfrbxLegacyNavigation(const UBXSfrbx& sfrbx);
/**
 * @brief True when a BeiDou legacy subframe is D2 (GEO) rather than D1:
 * sigId 1 / 3 / 10 (B1I / B2I / B3I D2), or sigId 0 from a GEO PRN.
 */
bool isSfrbxBeiDouD2(const UBXSfrbx& sfrbx);
bool decodeSfrbxFrameInfo(const UBXSfrbx& sfrbx, UBXSfrbxFrameInfo& frame_info);
const char* getSfrbxFrameKindName(UBXSfrbxFrameInfo::Kind kind);

}  // namespace ubx_utils

class UBXStreamDecoder {
public:
    struct Event {
        bool has_message = false;
        UBXMessage message;
        bool has_nav_pvt = false;
        UBXNavPVT nav_pvt;
        bool has_observation = false;
        ObservationData observation;
        bool has_sfrbx = false;
        UBXSfrbx sfrbx;
    };

    UBXStreamDecoder() = default;
    ~UBXStreamDecoder() = default;

    void clear();
    bool pushBytes(const uint8_t* data, size_t size, std::vector<Event>& events);

    const UBXDecoder& getDecoder() const { return decoder_; }
    UBXDecoder::UBXStats getStats() const { return decoder_.getStats(); }

private:
    UBXDecoder decoder_;
    std::vector<uint8_t> buffer_;
};

}  // namespace io
}  // namespace libgnss
