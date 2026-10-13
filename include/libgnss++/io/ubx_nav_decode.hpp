#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "../core/navigation.hpp"
#include "../core/types.hpp"
#include "ubx.hpp"

namespace libgnss {
namespace io {

/**
 * @brief Counters kept by UBXNavDecoder, per satellite system.
 *
 * `dropped` counts *decode attempts that failed* (a complete frame set was
 * present and was rejected), keyed by a short reason string: "subframe_id",
 * "iode_mismatch", "toc_toe_mismatch", "sow_sequence", "page_sequence",
 * "iod_nav_mismatch", "word_type", "crc", "hamming", "string_number",
 * "satellite_mismatch", "no_time_reference".
 */
struct UBXNavDecodeStats {
    struct SystemStats {
        size_t subframes = 0;          ///< SFRBX messages handed to this system's decoder
        size_t skipped_signal = 0;     ///< messages on out-of-scope signals (CNAV, F/NAV, E5b, B-CNAV, ...)
        size_t ephemerides = 0;        ///< new ephemerides emitted
        size_t duplicates = 0;         ///< complete records equal to the last emitted one
        std::map<std::string, size_t> dropped;
    };
    std::map<GNSSSystem, SystemStats> systems;
    size_t klobuchar_sets = 0;         ///< GPS subframe 4 page 18 sets decoded
};

/**
 * @brief Broadcast ephemerides and the GPS Klobuchar set from UBX-RXM-SFRBX.
 *
 * Accumulates the navigation words of each (system, satellite, message
 * family) and emits an `Ephemeris` whenever a consistent frame set completes,
 * in the representation the RINEX navigation reader produces for the same
 * record.  A message family is one navigation message layout: GPS L1 C/A LNAV,
 * QZSS L1 C/A LNAV, Galileo E1-B I/NAV, BeiDou D1, BeiDou D2, GLONASS strings.
 * Signals of one family (BeiDou B1I/B2I/B3I D1, GLONASS L1OF/L2OF) carry
 * bit-identical content and share one buffer, as RTKLIB does: a receiver
 * forwards only a subset of the subframes of each signal, so the signals
 * complete each other's sets.
 * Layouts, consistency checks and the completion triggers follow RTKLIB 2.4.3
 * b34 (rcv/ublox.c decode_rxmsfrbx, rcvraw.c decode_frame, decode_gal_inav,
 * decode_bds_d1/d2, decode_glostr):
 *
 *  - GPS / QZSS L1 C/A LNAV (sigId 0): subframes 1-3, emitted on subframe 3
 *    when IODE (subframes 2, 3) equals the low byte of the IODC (subframe 1);
 *    GPS subframe 4 page 18 gives the Klobuchar alpha/beta.
 *  - Galileo E1-B I/NAV (sigId 0/1): CRC-24Q checked word types 1-5, emitted
 *    on word type 5 when the four IODnav agree and word type 4 names the
 *    satellite.  Data-source word: I/NAV E1-B, clock referenced to E5b/E1.
 *  - BeiDou B1I/B2I/B3I D1 (MEO/IGSO) subframes 1-3, emitted on subframe 3,
 *    and D2 (GEO) subframe 1 pages 1-10, emitted on page 10; the subframe
 *    time stamps must be consecutive and toc must equal toe.
 *  - GLONASS L1OF/L2OF strings 1-4 (Hamming checked), emitted on string 4.
 *
 * Skipped by design: GPS/QZSS CNAV (L2C, L5), QZSS L1S, Galileo F/NAV (E5a)
 * and E5b-I I/NAV, BeiDou B-CNAV (B1C, B2a), NavIC, SBAS.
 *
 * There is no parity check for GPS/BeiDou words, as in RTKLIB: u-blox
 * receivers only forward subframes that passed their own parity check.
 *
 * Time reference.  GPS/QZSS carry a 10-bit week and GLONASS a time of day, so
 * those need the receiver time: call setReferenceTime() with every
 * UBX-RXM-RAWX epoch time (GPST).  Without a reference they are dropped with
 * reason "no_time_reference"; Galileo (12-bit GST week) and BeiDou (13-bit
 * BDT week) decode without one.
 */
class UBXNavDecoder {
public:
    UBXNavDecoder() = default;

    /// Forget all frame buffers, the last emitted records and the statistics.
    void clear();

    /// GPST of the receiver, used to resolve the GPS week rollover and the GLONASS day.
    void setReferenceTime(const GNSSTime& gpst);
    bool hasReferenceTime() const { return has_reference_; }
    const GNSSTime& referenceTime() const { return reference_; }

    /**
     * @brief Feed one UBX-RXM-SFRBX message.
     * @return number of ephemerides appended to `out` (0 or 1).
     */
    size_t pushSfrbx(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out);

    /// True once a GPS Klobuchar set has been decoded.
    bool hasKlobuchar() const { return klobuchar_.valid; }
    /// The most recently decoded GPS Klobuchar set (alpha/beta, `valid` true), else invalid.
    const IonosphereModel& klobuchar() const { return klobuchar_; }

    const UBXNavDecodeStats& stats() const { return stats_; }

private:
    struct GpsState {
        std::array<uint8_t, 150> subframes{};  ///< 5 x 30 bytes, 24 data bits per word, parity removed
        std::array<bool, 5> present{};
    };
    struct GalileoState {
        std::array<uint8_t, 7 * 16> words{};   ///< word types 0-6, 128 bits each
        std::array<bool, 7> present{};
    };
    struct BeiDouState {
        std::array<uint8_t, 11 * 38> frames{}; ///< D1: subframes 1-5; D2: pages 1-10 (+ page 102)
        std::array<bool, 11> present{};
    };
    struct GlonassState {
        std::array<uint8_t, 40> strings{};     ///< strings 1-4, 80 bits each
        std::array<bool, 4> present{};
        std::array<uint8_t, 2> frame_id{};
        bool has_frame_id = false;
    };
    struct Key {
        GNSSSystem system;
        uint8_t sv;
        uint8_t family;  ///< BeiDou: 0 = D1, 1 = D2; otherwise 0 (Galileo 1)
        bool operator<(const Key& o) const {
            if (system != o.system) return system < o.system;
            if (sv != o.sv) return sv < o.sv;
            return family < o.family;
        }
    };

    size_t pushGps(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out);
    size_t pushGalileo(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out);
    size_t pushBeiDou(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out);
    size_t pushGlonass(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out);

    bool emit(const Ephemeris& eph, std::vector<Ephemeris>& out);
    UBXNavDecodeStats::SystemStats& systemStats(GNSSSystem system) { return stats_.systems[system]; }

    GNSSTime reference_;
    bool has_reference_ = false;
    UBXNavDecodeStats stats_;
    IonosphereModel klobuchar_;

    std::map<Key, GpsState> gps_;
    std::map<Key, GalileoState> galileo_;
    std::map<Key, BeiDouState> beidou_;
    std::map<Key, GlonassState> glonass_;
    /// Last emitted record per satellite (RTKLIB compares against the stored ephemeris).
    std::map<SatelliteId, Ephemeris> last_emitted_;
};

namespace ubx_utils {

/// CRC-24Q (Galileo I/NAV / RTCM 3 polynomial 0x1864CFB, zero initial value) over `size` bytes.
uint32_t crc24q(const uint8_t* data, size_t size);

/// Hamming code check of a GLONASS navigation string (ICD 5.1 section 4.7); 11 bytes, string bit 85 first.
bool testGlonassString(const uint8_t* data);

}  // namespace ubx_utils

}  // namespace io
}  // namespace libgnss
