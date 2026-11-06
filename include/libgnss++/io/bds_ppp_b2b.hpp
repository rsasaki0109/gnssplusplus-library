#pragma once

/**
 * @file bds_ppp_b2b.hpp
 * @brief BeiDou PPP-B2b correction decoder.
 *
 * Decodes the PPP service messages the BDS-3 GEO satellites (C59-C63)
 * broadcast on B2b-I (BDS-SIS-ICD-PPP-B2b, Version 1.0, July 2020):
 *
 * - message records from the cssrlib `wn tow prn type len hex` text format
 *   (one 486-bit message per line: message type 6, data 456, CRC-24Q 24),
 *   with the CRC-24Q check over the first 462 bits;
 * - message types 1 (satellite mask), 2 (orbit correction and URA),
 *   3 (differential code bias) and 4 (clock correction); type 63 is the null
 *   message, types 5-7 (URA, combined messages) are counted and skipped;
 * - the IODP / IODSSR bookkeeping of ICD section 6 and the reference time
 *   (BDT seconds of day -> GPST).
 *
 * Sign conventions (ICD 7.2 / 7.3): the orbit correction is in the radial /
 * along-track / cross-track frame e_r = r / |r|, e_c = (r x v) / |r x v|,
 * e_a = e_c x e_r, and is SUBTRACTED from the broadcast position; the clock
 * correction C0 is SUBTRACTED from the broadcast clock (t = t_brdc - C0 / c);
 * the code bias is subtracted from the pseudorange of its signal. The
 * broadcast ephemeris is B-CNAV1 for BDS and LNAV for GPS, selected by
 * IODN (BDS: IODC, whose low 8 bits are the IODE; GPS: IODC, low 8 bits IODE).
 */

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "../core/types.hpp"

namespace libgnss {
namespace io {

constexpr int kB2bMessageBits = 486;   ///< message type (6) + data (456) + CRC-24Q (24)
constexpr int kB2bMessageBytes = 61;   ///< 486 bits padded to whole bytes

/// One PPP-B2b message as logged by a receiver.
struct BdsB2bMessage {
    GNSSTime time;                                 ///< reception time (GPST)
    int prn = 0;                                   ///< BDS GEO PRN that broadcast it
    std::array<uint8_t, kB2bMessageBytes> bits{};  ///< MSB-first, bit 0 = first message-type bit
};

struct BdsB2bReadStats {
    size_t records = 0;   ///< non-comment lines
    size_t messages = 0;  ///< messages read
    size_t skipped = 0;   ///< malformed lines
};

/**
 * @brief Read PPP-B2b messages from a cssrlib `wn tow prn type len hex` log
 * (GPS week and time of week, BDS PRN, receiver type and length fields
 * ignored, at least 122 hex digits). With @p prn > 0 only that satellite's
 * messages are kept.
 */
bool readBdsB2bMessages(const std::string& path,
                        std::vector<BdsB2bMessage>& messages,
                        int prn = 0,
                        BdsB2bReadStats* stats = nullptr,
                        std::string* error = nullptr);

/// CRC-24Q over the message type and data bits, compared with the message CRC.
bool checkBdsB2bCrc(const BdsB2bMessage& message);

/// Message type (first six bits).
int bdsB2bMessageType(const BdsB2bMessage& message);

/// Satellite of a PPP-B2b satellite slot (ICD Table 6-2): 1-63 BDS,
/// 64-100 GPS, 101-137 Galileo, 138-174 GLONASS; UNKNOWN system otherwise.
SatelliteId bdsB2bSlotSatellite(int slot);

/// URA (m) of a URA class / value pair (ICD 6.2.3), nullopt when unknown (0/0).
std::optional<double> bdsB2bUraMeters(int ura_class, int ura_value);

/// Header shared by message types 1-4.
struct BdsB2bHeader {
    int tod = 0;      ///< epoch time, BDT seconds of day
    int iodssr = 0;   ///< issue of data, SSR
};

struct BdsB2bMask {
    BdsB2bHeader header;
    int iodp = 0;                          ///< issue of data, PRN mask
    std::vector<SatelliteId> satellites;   ///< mask order: BDS, GPS, Galileo, GLONASS
};

struct BdsB2bOrbit {
    SatelliteId satellite;
    int slot = 0;
    int iodn = 0;                 ///< 10-bit issue of data, navigation
    int iod_corr = 0;             ///< 3-bit issue of data, correction
    double radial_m = 0.0;
    double along_m = 0.0;
    double cross_m = 0.0;
    bool available = false;       ///< false when a component is the "invalid" value
    int ura_class = 0;
    int ura_value = 0;
};

struct BdsB2bClock {
    SatelliteId satellite;
    int iod_corr = 0;
    double c0_m = 0.0;            ///< subtracted from the broadcast clock
    bool available = false;
};

struct BdsB2bCodeBias {
    SatelliteId satellite;
    std::map<int, double> bias_m;   ///< signal index (ICD Table 6-6) -> bias (m)
};

struct BdsB2bDecodedMessage {
    GNSSTime reception_time;
    GNSSTime reference_time;      ///< tod in the BDT day of reception, as GPST
    int prn = 0;
    int type = 0;
    BdsB2bHeader header;
    int iodp = -1;                ///< types 1 and 4
    int subtype = -1;             ///< type 4: satellite subset (23 satellites each)
    std::vector<SatelliteId> mask;           ///< type 1
    std::vector<BdsB2bOrbit> orbits;         ///< type 2 (entries of masked satellites)
    std::vector<BdsB2bCodeBias> code_biases; ///< type 3
    std::vector<BdsB2bClock> clocks;         ///< type 4
};

/// One satellite's held correction when its orbit and a clock of the same
/// IOD Corr are known (ICD 6.2.2: orbit and clock are used in pairs).
struct BdsB2bUpdate {
    SatelliteId satellite;
    GNSSTime time;                ///< reference time of the newer of the orbit / clock
    GNSSTime orbit_time;
    GNSSTime clock_time;
    int iodssr = 0;
    int iodp = 0;
    int iodn = 0;                 ///< 10-bit IODN of the orbit correction
    int iode = 0;                 ///< broadcast IODE to match: IODN & 0xff (BDS, GPS)
    int iod_corr = 0;
    double radial_m = 0.0;        ///< B2b sign: subtracted from the broadcast position
    double along_m = 0.0;
    double cross_m = 0.0;
    double c0_m = 0.0;            ///< B2b sign: t = t_brdc - c0_m / c
    std::optional<double> ura_m;
    std::map<int, double> code_bias_m;   ///< signal index -> bias, empty until a type 3
};

/// Pseudorange signal of a PPP-B2b code-bias signal index (ICD Table 6-6),
/// as the RINEX observation code ("2I", "1P", "1C", ...); "" when reserved.
const char* bdsB2bSignalCode(GNSSSystem system, int signal_index);

struct BdsB2bDecoderStats {
    size_t messages = 0;
    size_t other_satellite_messages = 0;  ///< from another GEO than the one followed
    size_t crc_failures = 0;
    size_t null_messages = 0;           ///< type 63
    size_t unsupported_messages = 0;    ///< types 5-62
    size_t masks = 0;
    size_t mask_changes = 0;            ///< IODP changes (correction state reset)
    size_t orbit_messages = 0;
    size_t clock_messages = 0;
    size_t code_bias_messages = 0;
    size_t iodssr_mismatches = 0;       ///< types 2-4 skipped: IODSSR differs from the mask's
    size_t iodp_mismatches = 0;         ///< type 4 skipped: IODP differs from the mask's
    size_t without_mask = 0;            ///< types 2-4 before any mask
    size_t updates = 0;
};

/**
 * @brief Stateful PPP-B2b decoder: messages in, decoded messages and held
 * per-satellite updates out.
 *
 * Each GEO satellite broadcasts its own message stream with its own IODP /
 * IODSSR sequence, so one decoder follows one satellite: Options::prn, or
 * the first satellite it is given; messages of the others are counted and
 * ignored.
 *
 * Follows the ICD bookkeeping (and cssrlib cssr_bds): a mask with a new IODP
 * resets every correction; types 2-4 with another IODSSR than the current
 * mask are ignored, as is a type 4 with another IODP; orbit and code-bias
 * entries of satellites outside the mask are ignored. A clock whose IOD
 * Corr changes keeps the previous value, so an orbit still on the previous
 * IOD Corr pairs with it.
 */
class BdsB2bDecoder {
public:
    struct Options {
        int prn = 0;                 ///< GEO PRN to follow (0: the first one seen)
        bool check_crc = true;
        bool keep_messages = true;   ///< keep decoded messages (for dumps and tests)
    };

    BdsB2bDecoder() : BdsB2bDecoder(Options{}) {}
    explicit BdsB2bDecoder(const Options& options) : options_(options), prn_(options.prn) {}

    /// Decode one message; returns its type, or -1 when it fails the CRC or
    /// comes from another satellite than the one followed.
    int addMessage(const BdsB2bMessage& message);

    const std::vector<BdsB2bDecodedMessage>& messages() const { return messages_; }
    const std::vector<BdsB2bUpdate>& updates() const { return updates_; }
    const BdsB2bDecoderStats& stats() const { return stats_; }
    int iodp() const { return iodp_; }
    int prn() const { return prn_; }
    int iodssr() const { return iodssr_; }

private:
    struct ClockState {
        GNSSTime time;
        int iod_corr = -1;
        double c0_m = 0.0;
        bool available = false;
        bool has_previous = false;
        int previous_iod_corr = -1;
        double previous_c0_m = 0.0;
        bool previous_available = false;
        GNSSTime previous_time;
    };
    struct OrbitState {
        GNSSTime time;
        BdsB2bOrbit orbit;
    };
    struct BiasState {
        GNSSTime time;
        std::map<int, double> bias_m;
    };

    void reset();
    void emit(const SatelliteId& satellite, const GNSSTime& time);
    bool masked(const SatelliteId& satellite) const;

    Options options_;
    int prn_ = 0;
    int iodp_ = -1;
    int iodssr_ = -1;
    std::vector<SatelliteId> mask_;
    std::map<SatelliteId, OrbitState> orbits_;
    std::map<SatelliteId, ClockState> clocks_;
    std::map<SatelliteId, BiasState> code_biases_;
    std::vector<BdsB2bDecodedMessage> messages_;
    std::vector<BdsB2bUpdate> updates_;
    BdsB2bDecoderStats stats_;
};

}  // namespace io
}  // namespace libgnss
