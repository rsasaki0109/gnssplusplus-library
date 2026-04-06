#pragma once

#include <libgnss++/core/types.hpp>

namespace libgnss { class SSRProducts; }

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace libgnss::qzss_l6 {

constexpr uint32_t kL6Preamble = 0x1ACFFC1D;
constexpr int kFrameBytes = 250;
constexpr int kDataPartBits = 1695;
constexpr int kSubframeFrames = 5;
constexpr int kSubframeBits = kDataPartBits * kSubframeFrames;  // 8475

/// Bit reader for packed binary data.
class BitReader {
public:
    BitReader(const uint8_t* data, int total_bits)
        : data_(data), total_bits_(total_bits) {}

    /// Read n bits as unsigned integer (big-endian, MSB first).
    uint64_t readU(int n) {
        uint64_t val = 0;
        for (int i = 0; i < n; ++i) {
            val = (val << 1) | readBit();
        }
        return val;
    }

    /// Read n bits as signed integer (two's complement).
    int64_t readS(int n) {
        uint64_t val = readU(n);
        if (n > 0 && (val & (1ULL << (n - 1)))) {
            val |= ~((1ULL << n) - 1);  // sign extend
        }
        return static_cast<int64_t>(val);
    }

    int position() const { return bit_pos_; }
    int remaining() const { return total_bits_ - bit_pos_; }
    void skip(int n) { bit_pos_ += n; }

private:
    int readBit() {
        if (bit_pos_ >= total_bits_) return 0;
        const int byte_idx = bit_pos_ / 8;
        const int bit_idx = 7 - (bit_pos_ % 8);
        ++bit_pos_;
        return (data_[byte_idx] >> bit_idx) & 1;
    }

    const uint8_t* data_;
    int total_bits_;
    int bit_pos_ = 0;
};

/// CSSR satellite entry from subtype 1 mask.
struct CssrSatellite {
    GNSSSystem system = GNSSSystem::UNKNOWN;
    uint8_t prn = 0;
    std::vector<int> signal_slots;  // signal IDs with active bias
};

/// CSSR mask state (from subtype 1).
struct CssrMaskState {
    int iod = -1;
    int tow0 = -1;  // hourly epoch TOW
    std::vector<CssrSatellite> satellites;
};

/// Per-satellite orbit correction.
struct CssrOrbitCorrection {
    double dx = 0.0, dy = 0.0, dz = 0.0;  // RAC (radial, along, cross) in meters
};

/// Per-satellite clock correction.
struct CssrClockCorrection {
    double dclock_m = 0.0;
};

/// Decoded CSSR corrections for one epoch.
struct CssrEpoch {
    int week = 0;
    double tow = 0.0;
    int iod = -1;
    CssrMaskState mask;
    std::map<SatelliteId, CssrOrbitCorrection> orbits;
    std::map<SatelliteId, CssrClockCorrection> clocks;
    std::map<SatelliteId, std::map<int, double>> code_biases;   // signal_id → meters
    std::map<SatelliteId, std::map<int, double>> phase_biases;  // signal_id → meters
    /// Network-specific phase biases from ST6, keyed by network_id
    std::map<int, std::map<SatelliteId, std::map<int, double>>> network_phase_biases;
    /// Atmosphere tokens (STEC + gridded trop), keyed by network_id
    std::map<int, std::map<std::string, std::string>> atmos_by_network;
    int last_gridded_network_id = 0;  ///< Last ST9 network processed
    /// Merged atmos tokens (union of all networks, Python pending_atmos equivalent)
    std::map<std::string, std::string> merged_atmos;
    bool has_orbit = false;
    bool has_clock = false;
    bool has_code_bias = false;
    bool has_phase_bias = false;
    bool has_atmos = false;
};

/// L6 frame decoder and CSSR message parser.
class L6Decoder {
public:
    /// Feed raw L6 data (frame-aligned, 250 bytes per frame).
    /// Returns decoded CSSR epochs (may be empty if subframe not yet complete).
    std::vector<CssrEpoch> feedFrame(const uint8_t* frame_data, int gps_week);

    /// Feed an entire L6 file.
    std::vector<CssrEpoch> decodeFile(const std::string& path, int gps_week);

    /// Get current mask state.
    const CssrMaskState& maskState() const { return mask_; }

private:
    struct PendingSubframe {
        std::vector<std::vector<uint8_t>> frame_data_parts;
    };

    void processSubframe(const uint8_t* data, int bits, int gps_week);
    void decodeCssrMessage(BitReader& reader, int gps_week);
    void decodeSubtype1(BitReader& reader);
    void decodeSubtype2(BitReader& reader);
    void decodeSubtype3(BitReader& reader);
    void decodeSubtype4(BitReader& reader);
    void decodeSubtype5(BitReader& reader);
    void decodeSubtype6(BitReader& reader);
    void decodeSubtype8(BitReader& reader);
    void decodeSubtype9(BitReader& reader);
    void decodeSubtype11(BitReader& reader);

    SatelliteId maskIndexToSatId(int index) const;
    double computeTow(int epoch_time) const;

    CssrMaskState mask_;
    CssrEpoch current_epoch_;
    /// Persistent atmos tokens accumulated across subframes
    std::map<std::string, std::string> merged_atmos_;
    /// Base phase biases from ST5 (Python pending_base_phase_bias)
    std::map<SatelliteId, std::map<int, double>> base_phase_biases_;
    /// Network phase biases from ST6 (Python pending_phase_bias)
    std::map<SatelliteId, std::map<int, double>> network_phase_biases_;
    /// Base code biases from ST4
    std::map<SatelliteId, std::map<int, double>> base_code_biases_;
    /// Network code biases from ST6
    std::map<SatelliteId, std::map<int, double>> network_code_biases_;
    std::vector<CssrEpoch> completed_epochs_;
    std::map<std::tuple<int, int, int>, PendingSubframe> pending_subframes_;
    int frame_index_ = 0;
    int gps_week_ = 0;
};

/// Convert decoded CSSR epochs into SSRProducts for use with PPP pipeline.
/// This bypasses the CSV intermediate format entirely.
/// preferred_network_id: if >0, use this network for atmos grid reference.
/// If 0, auto-select from the last gridded network in each epoch.
void populateSSRProducts(
    const std::vector<CssrEpoch>& epochs,
    class libgnss::SSRProducts& products,
    int preferred_network_id = 0);

}  // namespace libgnss::qzss_l6
