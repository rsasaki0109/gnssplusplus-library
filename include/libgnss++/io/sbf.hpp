#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"

namespace libgnss {
namespace io {

// ============================================================================
// SBF Protocol Constants
// ============================================================================

static constexpr uint8_t SBF_SYNC_1 = 0x24;  // '$'
static constexpr uint8_t SBF_SYNC_2 = 0x40;  // '@'
static constexpr uint8_t SBF_HEADER_LEN = 8;  // sync(2) + crc(2) + id(2) + length(2)

// ============================================================================
// SBF Block IDs
// ============================================================================

enum class SBFBlockId : uint16_t {
    MeasEpoch      = 4027,
    MeasExtra      = 4000,
    PVTCartesian   = 4006,
    PVTGeodetic    = 4007,
    PosCovCartesian = 5905,
    PosCovGeodetic  = 5906,
    DOP            = 4001,
    VelCovCartesian = 5907,
    VelCovGeodetic  = 5908,
    ChannelStatus  = 4013,
    ReceiverTime   = 5914,
    QualityInd     = 4082,
    EndOfPVT       = 5921,
};

// ============================================================================
// SBF Message Wrapper
// ============================================================================

struct SBFMessage {
    SBFBlockId id;
    uint16_t revision = 0;
    uint16_t length = 0;         // total block length including header
    uint32_t tow_ms = 0;        // time of week in ms
    uint16_t wnc = 0;           // GPS week number
    std::vector<uint8_t> payload; // bytes after the 14-byte header (tow + wnc = 6 extra)
    bool valid = false;
};

// ============================================================================
// Decoded SBF Structures
// ============================================================================

#pragma pack(push, 1)

/// Common SBF block header (14 bytes: sync(2)+crc(2)+id(2)+length(2)+tow(4)+wnc(2))
struct SBFBlockHeader {
    uint16_t sync;
    uint16_t crc;
    uint16_t id_rev;   // block number (13 bits) + revision (3 bits)
    uint16_t length;
    uint32_t tow;      // ms
    uint16_t wnc;
};

/// PVTGeodetic sub-block (after 14-byte header)
struct SBFPvtGeodetic {
    uint8_t  mode;
    uint8_t  error;
    double   latitude;     // rad
    double   longitude;    // rad
    double   height;       // m (ellipsoidal)
    float    undulation;   // m (geoid - ellipsoid)
    float    vn;           // m/s
    float    ve;           // m/s
    float    vu;           // m/s
    float    cog;          // deg
    double   rx_clk_bias;  // ms
    float    rx_clk_drift; // ppm
    uint8_t  time_system;
    uint8_t  datum;
    uint8_t  nr_sv;
    uint8_t  wa_corr_info;
    uint16_t reference_id;
    uint16_t mean_corr_age; // 0.01 s
    uint32_t signal_info;
    uint8_t  alert_flag;
    uint8_t  nr_bases;
    uint16_t ppp_info;
    uint16_t latency;      // 0.0001 s
    uint16_t h_accuracy;   // 0.01 m
    uint16_t v_accuracy;   // 0.01 m
    uint8_t  misc;
};

/// PVTCartesian sub-block
struct SBFPvtCartesian {
    uint8_t  mode;
    uint8_t  error;
    double   x;            // m ECEF
    double   y;
    double   z;
    float    undulation;
    float    vx;           // m/s
    float    vy;
    float    vz;
    float    cog;
    double   rx_clk_bias;
    float    rx_clk_drift;
    uint8_t  time_system;
    uint8_t  datum;
    uint8_t  nr_sv;
    uint8_t  wa_corr_info;
    uint16_t reference_id;
    uint16_t mean_corr_age;
    uint32_t signal_info;
    uint8_t  alert_flag;
    uint8_t  nr_bases;
    uint16_t ppp_info;
    uint16_t latency;
    uint16_t h_accuracy;
    uint16_t v_accuracy;
    uint8_t  misc;
};

/// DOP block
struct SBFDop {
    uint8_t  nr_sv;
    uint8_t  reserved;
    uint16_t pdop;    // 0.01 scale
    uint16_t tdop;
    uint16_t hdop;
    uint16_t vdop;
    float    hpl;     // m horizontal protection level
    float    vpl;     // m vertical protection level
};

#pragma pack(pop)

// ============================================================================
// SBF "Do Not Use" sentinel values
// ============================================================================

namespace sbf_sentinel {
    constexpr double   DNO_DOUBLE = -2.0e10;
    constexpr float    DNO_FLOAT  = -2.0e10f;
    constexpr uint16_t DNO_UINT16 = 65535;
    constexpr uint32_t DNO_UINT32 = 4294967295u;
    constexpr uint8_t  DNO_UINT8  = 255;

    inline bool isDoNotUse(double v) { return v < -1.0e10; }
    inline bool isDoNotUse(float v)  { return v < -1.0e10f; }
    inline bool isDoNotUse(uint16_t v) { return v == DNO_UINT16; }
    inline bool isDoNotUse(uint32_t v) { return v == DNO_UINT32; }
    inline bool isDoNotUse(uint8_t v)  { return v == DNO_UINT8; }
}

// ============================================================================
// SBF Processor
// ============================================================================

class SBFProcessor {
public:
    SBFProcessor() = default;
    ~SBFProcessor() = default;

    std::vector<SBFMessage> decode(const uint8_t* buffer, size_t size);

    bool decodeMessage(const SBFMessage& message,
                       ObservationData* obs_data = nullptr,
                       NavigationData* nav_data = nullptr,
                       PositionSolution* pos_data = nullptr);

    bool decodePvtGeodetic(const SBFMessage& msg, SBFPvtGeodetic& pvt);
    bool decodePvtCartesian(const SBFMessage& msg, SBFPvtCartesian& pvt);
    bool decodeDop(const SBFMessage& msg, SBFDop& dop);

    static GNSSSystem sbfSignalTypeToSystem(uint8_t sig_type);
    static SignalType sbfSignalTypeToSignal(uint8_t sig_type);

    // CRC-16-CCITT
    static uint16_t computeCRC16(const uint8_t* buf, size_t length);

    struct SBFStats {
        size_t total_bytes = 0;
        size_t total_messages = 0;
        size_t valid_messages = 0;
        size_t crc_errors = 0;
        size_t sync_errors = 0;
        std::map<SBFBlockId, size_t> message_counts;
    };

    SBFStats getStats() const { return stats_; }
    void resetStats() { stats_ = SBFStats{}; }

private:
    SBFStats stats_;
    std::vector<uint8_t> residual_;

    static bool verifyCRC(const uint8_t* block, size_t length);

    bool pvtGeodeticToPosition(const SBFPvtGeodetic& pvt, uint32_t tow_ms,
                               uint16_t wnc, PositionSolution& pos);

    // CRC lookup table (CRC-CCITT)
    static const std::array<uint16_t, 256> crc_table_;
};

// ============================================================================
// Utilities
// ============================================================================

namespace sbf_utils {

std::string getBlockIdName(SBFBlockId id);
bool isMeasurementBlock(SBFBlockId id);
bool isPositionBlock(SBFBlockId id);

}  // namespace sbf_utils

}  // namespace io
}  // namespace libgnss
