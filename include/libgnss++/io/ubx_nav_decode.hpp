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
 * @brief Which SFRBX messages a decoded record was assembled from.
 *
 * `seq` numbers are the decoder's own count of the SFRBX messages passed to pushSfrbx() (1 for the
 * first message of any system or signal; out-of-scope messages count too).  A record is built from
 * the stored frames of its satellite, so its oldest message can be far older than the one that
 * completed it: `first_seq` is the oldest message of any frame the record actually uses (GPS/QZSS
 * subframes 1-3, Galileo word types 1-5, BeiDou D1 subframes 1-3 or D2 pages 1, 3-10, GLONASS
 * strings 1-4) and `last_seq` the completing message.  The `*_gpst_s` times are the receiver GPST
 * (week * 604800 + tow) of the newest RAWX epoch seen when those messages arrived; NaN while the
 * decoder has no time reference.
 */
struct UBXNavReception {
    uint64_t first_seq = 0;
    uint64_t last_seq = 0;
    double first_gpst_s = 0.0;
    double last_gpst_s = 0.0;
};

/// One complete frame set decoded by UBXNavDecoder, before its duplicate suppression.
struct UBXNavDecoded {
    Ephemeris eph;
    UBXNavReception reception;
    /// True when the decoder also appended it to `out` of pushSfrbx() (it differs from the last
    /// record emitted for the satellite in the decoder's issue fields); false for a repeat of it.
    bool emitted = false;
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
     * @param decoded optional; receives every complete frame set decoded from this message (0 or 1)
     *        with the messages it was built from, including a repeat of the last emitted record
     *        that `out` leaves out.  Confirm-twice (UBXNavConfirmTwice) needs those repeats.
     * @return number of ephemerides appended to `out` (0 or 1).
     */
    size_t pushSfrbx(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out,
                     std::vector<UBXNavDecoded>* decoded = nullptr);

    /// True once a GPS Klobuchar set has been decoded.
    bool hasKlobuchar() const { return klobuchar_.valid; }
    /// The most recently decoded GPS Klobuchar set (alpha/beta, `valid` true), else invalid.
    const IonosphereModel& klobuchar() const { return klobuchar_; }

    const UBXNavDecodeStats& stats() const { return stats_; }

private:
    /// Reception of the message that last wrote a stored frame.
    struct Stamp {
        uint64_t seq = 0;
        double gpst_s = 0.0;
    };
    struct GpsState {
        std::array<uint8_t, 150> subframes{};  ///< 5 x 30 bytes, 24 data bits per word, parity removed
        std::array<bool, 5> present{};
        std::array<Stamp, 5> stamp{};
    };
    struct GalileoState {
        std::array<uint8_t, 7 * 16> words{};   ///< word types 0-6, 128 bits each
        std::array<bool, 7> present{};
        std::array<Stamp, 7> stamp{};
    };
    struct BeiDouState {
        std::array<uint8_t, 11 * 38> frames{}; ///< D1: subframes 1-5; D2: pages 1-10 (+ page 102)
        std::array<bool, 11> present{};
        std::array<Stamp, 11> stamp{};
    };
    struct GlonassState {
        std::array<uint8_t, 40> strings{};     ///< strings 1-4, 80 bits each
        std::array<bool, 4> present{};
        std::array<Stamp, 4> stamp{};
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

    Stamp nowStamp() const;
    /// Reception of a record built from the frames stamped `used` (the newest is the current message).
    static UBXNavReception receptionOf(const std::vector<Stamp>& used);
    bool emit(const Ephemeris& eph, const UBXNavReception& reception, std::vector<Ephemeris>& out);
    UBXNavDecodeStats::SystemStats& systemStats(GNSSSystem system) { return stats_.systems[system]; }

    GNSSTime reference_;
    bool has_reference_ = false;
    uint64_t message_seq_ = 0;                ///< SFRBX messages pushed so far
    std::vector<UBXNavDecoded>* decoded_ = nullptr;  ///< pushSfrbx() output of the call in progress
    UBXNavDecodeStats stats_;
    IonosphereModel klobuchar_;

    std::map<Key, GpsState> gps_;
    std::map<Key, GalileoState> galileo_;
    std::map<Key, BeiDouState> beidou_;
    std::map<Key, GlonassState> glonass_;
    /// Last emitted record per satellite (RTKLIB compares against the stored ephemeris).
    std::map<SatelliteId, Ephemeris> last_emitted_;
};

/**
 * @brief True when the health flags of a decoded record say the satellite is usable.
 *
 * `gnss live --ubx-nav` merges only records for which this holds.  The decoder itself, and
 * `--nav-rinex-out`, keep every record with its health as received.
 *  - GPS/QZSS: LNAV svh == 0.
 *  - Galileo: E1-B HS == 0 and E1-B DVS == 0 (the E5b flags are ignored).
 *  - BeiDou: SatH1 == 0.
 *  - GLONASS: Bn MSB == 0 and string-3 ln == 0 (`Ephemeris::glonass_ln`; 0 when not present).
 */
bool ubxNavEphemerisUsable(const Ephemeris& eph);

/**
 * @brief True when two decoded records carry the same broadcast content, bit for bit.
 *
 * Compared: satellite; toe, toc, toes and week; every orbit, clock and group-delay term (doubles by
 * bit pattern, so +0 and -0 differ); GLONASS state vector, acceleration, tau_n, gamma_n, frequency
 * channel (and whether it is present), age and ln; health, sv_health, URA / SISA, IODC, IODE, the
 * RINEX data-source word and the Galileo I/NAV flag (navigation_message_type), valid.
 *
 * Not compared, because the decoder derives them from the moment of reception:
 *  - `tof` (time of frame): the HOW / word-type-5 / BeiDou SOW / GLONASS tk of the transmission,
 *    resolved against the receiver time.  It differs between two transmissions of one record.
 * Not populated by this decoder, so not compared: `glonass_cdma_data`,
 * `glonass_canonical_geph_data_valid`, `glonass_canonical_geph_reject_reason`.
 * The week of toe, toc and `week` is resolved against the receiver time, but it follows the
 * broadcast toe, so two copies of one record agree on it; a copy pair that straddles a week
 * resolution edge would simply not match (the safe direction: one more transmission is needed).
 */
bool ubxNavSameRecord(const Ephemeris& a, const Ephemeris& b);

/**
 * @brief Confirm-twice (D2): accept a decoded ephemeris only after two transmissions agree.
 *
 * A frame set with an undetected error (one GLONASS string that passed the Hamming check put a
 * satellite up to 9,470 km off in a development log) can decode into a record that passes every
 * consistency check, and the receiver forwards it once.  The satellite never repeats such a record,
 * so requiring the same record from two *different transmissions* removes it.  The price is one
 * transmission interval of the frame set (for GPS/QZSS LNAV, Galileo I/NAV and GLONASS the 30 s frame
 * repeat; BeiDou D1/D2 likewise one repeat of the frame set) on the first appearance of every new
 * ephemeris.
 *
 * Per satellite:
 *  - a copy equal to the last accepted record is a duplicate and changes nothing;
 *  - otherwise, with no candidate, it becomes the candidate;
 *  - a copy equal to the candidate and built entirely from SFRBX messages received after the last
 *    message of the candidate (reception.first_seq > candidate.last_seq) confirms it: the record is
 *    accepted and the candidate cleared;
 *  - a copy equal to the candidate but built from some of the same messages (or older ones) is the
 *    same transmission seen again, not a confirmation; the candidate is kept as it is;
 *  - a copy that differs from the candidate replaces it (a conflict) and nothing is accepted.
 *
 * Feed it every copy from UBXNavDecoder::pushSfrbx(..., &decoded), not the deduplicated `out`:
 * the repeat of a record is exactly what confirms it.
 */
class UBXNavConfirmTwice {
public:
    struct Counters {
        size_t copies = 0;            ///< complete frame sets seen
        size_t candidates = 0;        ///< copies that became the candidate
        size_t accepted = 0;          ///< candidates confirmed by a second transmission
        size_t duplicates = 0;        ///< copies equal to the last accepted record
        size_t conflicts = 0;         ///< candidates replaced by a differing copy
        size_t same_transmission = 0; ///< equal copies that shared a message with the candidate
    };
    struct Outcome {
        bool accepted = false;
        UBXNavReception first_copy;   ///< reception of the confirmed candidate (valid when accepted)
    };

    /// Process one copy.  `copy.eph` is the accepted record when `accepted` is set.
    Outcome push(const UBXNavDecoded& copy);

    /// Candidates that were not confirmed: replaced ones plus the ones still pending.
    size_t unconfirmed() const;
    size_t conflicts() const;
    size_t accepted() const;
    /// Per-system counters (keyed by the record's satellite system).
    const std::map<GNSSSystem, Counters>& counters() const { return counters_; }
    void clear();

private:
    struct SatState {
        std::optional<Ephemeris> accepted;
        std::optional<Ephemeris> candidate;
        UBXNavReception candidate_reception;
    };
    std::map<SatelliteId, SatState> sats_;
    std::map<GNSSSystem, Counters> counters_;
};

namespace ubx_utils {

/// CRC-24Q (Galileo I/NAV / RTCM 3 polynomial 0x1864CFB, zero initial value) over `size` bytes.
uint32_t crc24q(const uint8_t* data, size_t size);

/// Hamming code check of a GLONASS navigation string (ICD 5.1 section 4.7); 11 bytes, string bit 85 first.
bool testGlonassString(const uint8_t* data);

}  // namespace ubx_utils

}  // namespace io
}  // namespace libgnss
