#pragma once

/**
 * @file galileo_has.hpp
 * @brief Galileo High Accuracy Service (HAS) signal-in-space decoder.
 *
 * Decodes the HAS corrections broadcast on the Galileo E6-B C/NAV signal
 * (Galileo HAS SIS ICD, Issue 1.0, May 2022):
 *
 * - C/NAV page extraction from u-blox UBX-RXM-SFRBX, Septentrio SBF
 *   GALRawCNAV (block 4024) and the cssrlib `wn tow prn type len hex` text
 *   format, with the CRC-24Q page check (ICD section 2.3.3);
 * - HAS page header parsing and the High Parity Vertical Reed-Solomon
 *   (RS(255,32) over GF(256), primitive polynomial 0x11D) erasure decoder
 *   (ICD section 6); the generator matrix of ICD Annex B is derived from the
 *   generator polynomial instead of being stored;
 * - Message Type 1 decoding (mask, orbit, clock full-set, clock subset, code
 *   bias and phase bias blocks, ICD section 5) with the Mask ID / IOD Set ID
 *   caches of ICD section 7.6;
 * - conversion into IODE-tagged, held correction updates with the ICD
 *   validity intervals, TOH -> GST time of applicability (ICD Eq. 28/29) and
 *   the "don't use" (HASS = 3) flush.
 *
 * Conventions (ICD section 7): HAS orbit corrections are ADDED to the
 * broadcast position (x = x_brdc + R_ntw->ecef * dR), clock corrections are
 * added to the broadcast clock (dt = dt_brdc + dC / c) and code / phase biases
 * are added to the observations. hasUpdateToRtcmSsrCorrection() converts an
 * update to the RTCM SSR storage used by the PPP SSR container (orbit negated,
 * code biases keyed by RTCM SSR signal IDs).
 */

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "../core/observation.hpp"
#include "../core/types.hpp"
#include "rtcm.hpp"

namespace libgnss {
namespace io {

constexpr int kGalileoCnavPageBits = 492;       ///< reserved(14) + HAS page(448) + CRC(24) + tail(6)
constexpr int kGalileoCnavPageBytes = 62;       ///< 492 bits padded to whole bytes
constexpr int kHasEncodedPageBytes = 53;        ///< 424-bit HAS encoded page
constexpr int kHasRsCodeLength = 255;           ///< RS(255,32)
constexpr int kHasRsInfoLength = 32;
constexpr uint32_t kHasDummyPageHeader = 0xAF3BC3U;

/// One C/NAV page (the 492 bits after the synchronisation pattern).
struct GalileoCnavPage {
    GNSSTime time;                                       ///< reception time (GPST)
    int prn = 0;                                         ///< Galileo SVID
    std::array<uint8_t, kGalileoCnavPageBytes> bits{};   ///< MSB-first, bit 0 = first reserved bit
    bool receiver_crc_failed = false;                    ///< receiver flagged the page as failing its CRC
};

enum class HasPageInputFormat { Ubx, CssrlibText, Sbf };

bool parseHasPageInputFormat(const std::string& text, HasPageInputFormat& format);
const char* hasPageInputFormatName(HasPageInputFormat format);

struct HasPageReadStats {
    size_t records = 0;          ///< input records inspected (UBX / SBF messages, text lines)
    size_t pages = 0;            ///< C/NAV pages extracted
    size_t skipped = 0;          ///< malformed records
};

/**
 * @brief Read Galileo E6-B C/NAV pages from a receiver log.
 *
 * UBX: RXM-SFRBX with gnssId 2 and sigId 8 (E6-B), 16 data words; the page
 * time is the latest RXM-RAWX epoch (GPST). SBF: GALRawCNAV (block 4024), time
 * from the block TOW / WNc. cssrlib text: `wn tow prn type len hex` lines.
 */
bool readGalileoCnavPages(const std::string& path,
                          HasPageInputFormat format,
                          std::vector<GalileoCnavPage>& pages,
                          HasPageReadStats* stats = nullptr,
                          std::string* error = nullptr);

/// Format implied by the file extension (.ubx, .sbf; anything else: cssrlib text).
HasPageInputFormat guessHasPageInputFormat(const std::string& path);

/// CRC-24Q over the 462 reserved + HAS page bits, compared with the page CRC.
bool checkGalileoCnavCrc(const GalileoCnavPage& page);

struct HasPageHeader {
    int hass = 0;        ///< 0 test, 1 operational, 2 reserved, 3 don't use
    int reserved = 0;
    int mt = 0;          ///< message type (1 = MT1)
    int mid = 0;         ///< message ID 0-31
    int ms = 0;          ///< message size in pages (1-32, stored value + 1)
    int pid = 0;         ///< encoded page ID 1-255
    bool dummy = false;  ///< HAS dummy page (header 0xAF3BC3)
};

HasPageHeader decodeHasPageHeader(const GalileoCnavPage& page);
std::array<uint8_t, kHasEncodedPageBytes> extractHasEncodedPage(const GalileoCnavPage& page);

/// Entry (row, col) of the HAS RS(255,32) systematic generator matrix G
/// (ICD Eq. 13-15 / Annex B); row 0..254 is PID - 1, col 0..31.
uint8_t hasRsGeneratorMatrixEntry(int row, int col);

/**
 * @brief HPVRS erasure decoding (ICD section 6.4).
 *
 * @param pids           distinct encoded page IDs (1..255), at least @p ms
 * @param encoded_pages  the 53-octet encoded pages, same order as @p pids
 * @param ms             message size k in pages (1..32)
 * @param message        decoded k * 53 octets
 * @return false when fewer than k pages are given or the sub-matrix is singular
 */
bool decodeHasHpvrs(const std::vector<int>& pids,
                    const std::vector<std::array<uint8_t, kHasEncodedPageBytes>>& encoded_pages,
                    int ms,
                    std::vector<uint8_t>& message);

/// Re-encode the page with ID @p pid of a decoded k-page message.
std::array<uint8_t, kHasEncodedPageBytes> encodeHasPage(const std::vector<uint8_t>& message,
                                                        int ms,
                                                        int pid);

/// ICD Table 23 validity interval in seconds, or -1 for the reserved index.
double hasValidityIntervalSeconds(int index);

/// ICD Eq. 28/29: message reference time from the reception time and TOH.
GNSSTime hasMessageReferenceTime(const GNSSTime& reception_time, int toh);

/// Map a HAS GNSS ID (ICD Table 18) to a GNSSSystem (UNKNOWN when reserved).
GNSSSystem hasGnssIdToSystem(int gnss_id);

/// ICD Table 20 signal name, or "" for reserved indices.
const char* hasSignalName(GNSSSystem system, int signal_index);

/// RTCM 10403.3 SSR bias signal ID for a HAS signal index (255 when none).
uint8_t hasSignalToRtcmSsrSignalId(GNSSSystem system, int signal_index);

/// Carrier wavelength of a HAS signal (m), 0 when unknown.
double hasSignalWavelength(GNSSSystem system, int signal_index);

struct HasMaskSystem {
    int gnss_id = 0;
    GNSSSystem system = GNSSSystem::UNKNOWN;
    std::vector<int> prns;                  ///< corrected satellites (SatM order)
    std::vector<int> signals;               ///< signal indices (SigM order)
    bool cell_mask_available = false;
    std::vector<std::vector<bool>> cell_mask;   ///< [satellite][signal]
    int nav_message = 0;                    ///< NM (0 = Galileo I/NAV, GPS LNAV)
};

struct HasMask {
    int mask_id = -1;
    std::vector<HasMaskSystem> systems;
    size_t satelliteCount() const;
};

struct HasOrbitEntry {
    SatelliteId satellite;
    int iodref = 0;
    double radial_m = 0.0;
    double in_track_m = 0.0;
    double cross_track_m = 0.0;
    bool available = false;                 ///< false when any component is "data not available"
};

enum class HasClockStatus { Ok, NotAvailable, DoNotUse };

struct HasClockEntry {
    SatelliteId satellite;
    int dcc_raw = 0;
    int multiplier = 1;
    double delta_clock_m = 0.0;             ///< DCC * DCM (added to the broadcast clock)
    HasClockStatus status = HasClockStatus::NotAvailable;
    bool subset = false;
};

struct HasBiasEntry {
    SatelliteId satellite;
    int signal_index = 0;
    double value = 0.0;                     ///< code bias (m) or phase bias (cycles)
    int discontinuity = 0;                  ///< phase discontinuity indicator (phase only)
    bool available = false;
};

struct HasMt1Message {
    int toh = 0;
    int flags = 0;                          ///< mask|orbit|clock full|clock subset|code bias|phase bias
    int mask_id = 0;
    int iod_set_id = 0;
    bool has_mask = false;
    bool has_orbit = false;
    bool has_clock_full = false;
    bool has_clock_subset = false;
    bool has_code_bias = false;
    bool has_phase_bias = false;
    HasMask mask;                           ///< mask in effect (decoded or cached)
    int orbit_vi = -1;
    int clock_full_vi = -1;
    int clock_subset_vi = -1;
    int code_bias_vi = -1;
    int phase_bias_vi = -1;
    std::vector<HasOrbitEntry> orbits;
    std::vector<HasClockEntry> clocks;      ///< full set and subset entries
    std::vector<HasBiasEntry> code_biases;
    std::vector<HasBiasEntry> phase_biases;
    size_t bits_used = 0;
};

enum class HasMt1Status { Ok, MissingMask, Truncated, InvalidHeader, InvalidMask };
const char* hasMt1StatusName(HasMt1Status status);

/// Decode one MT1 message body. @p known_masks supplies the mask when the
/// message carries no mask block.
HasMt1Status decodeHasMt1(const std::vector<uint8_t>& message,
                          const std::map<int, HasMask>& known_masks,
                          HasMt1Message& decoded);

/// One held HAS correction for a satellite, in HAS (ICD) sign conventions.
struct HasSsrUpdate {
    SatelliteId satellite;
    GNSSTime time;                          ///< applicability start (GPST)
    GNSSTime valid_until;                   ///< end of the orbit / clock validity
    int iode = -1;                          ///< IODref (Galileo IODnav, GPS IODE)
    int mask_id = -1;
    int iod_set_id = -1;
    GNSSTime orbit_time;
    GNSSTime clock_time;
    double radial_m = 0.0;                  ///< ICD sign: added to the broadcast position
    double in_track_m = 0.0;
    double cross_track_m = 0.0;
    double clock_m = 0.0;                   ///< ICD sign: dt = dt_brdc + clock_m / c
    std::map<int, double> code_bias_m;      ///< HAS signal index -> bias added to the pseudorange
    std::map<int, double> phase_bias_cycles;
    std::map<int, int> phase_discontinuity;
};

/// Convert to RTCM SSR storage: orbit negated (RTCM subtracts), clock kept,
/// code biases keyed by RTCM SSR signal IDs (RTCM sign: added to the observation).
RTCMSSRCorrection hasUpdateToRtcmSsrCorrection(const HasSsrUpdate& update);

struct HasDecodedMessage {
    GNSSTime reception_time;
    GNSSTime reference_time;
    int hass = 0;
    int mid = 0;
    int ms = 0;
    HasMt1Status status = HasMt1Status::Ok;
    HasMt1Message mt1;
};

struct HasDecoderStats {
    size_t pages = 0;
    size_t crc_failures = 0;
    size_t dummy_pages = 0;
    size_t reserved_status_pages = 0;       ///< HASS = 2
    size_t dont_use_pages = 0;              ///< HASS = 3
    size_t test_mode_pages = 0;             ///< HASS = 0 (used)
    size_t unsupported_type_pages = 0;      ///< MT != 1, PID 0 or an all-zero PID (MS < PID <= 32)
    size_t redundant_pages = 0;             ///< pages of already decoded messages
    size_t collection_resets = 0;
    size_t messages_decoded = 0;
    size_t rs_failures = 0;
    size_t mt1_missing_mask = 0;
    size_t mt1_errors = 0;
    size_t unpaired_clocks = 0;             ///< clocks without an orbit of their IOD set
    size_t updates = 0;
    size_t flushes = 0;
};

/**
 * @brief Stateful HAS SIS decoder: pages in, held correction updates out.
 */
class GalileoHasDecoder {
public:
    struct Options {
        bool check_crc = true;
        bool accept_test_mode = true;       ///< use HASS = 0 (test) pages
        double collection_timeout_s = 120.0;  ///< drop an incomplete page collection after this
        bool keep_messages = true;          ///< keep decoded messages for dumps
    };

    GalileoHasDecoder() : GalileoHasDecoder(Options{}) {}
    explicit GalileoHasDecoder(const Options& options);

    void addPage(const GalileoCnavPage& page);
    /// Decode a complete MT1 message (bypassing the page layer).
    HasMt1Status addMessage(const std::vector<uint8_t>& message,
                            const GNSSTime& reception_time,
                            int hass = 1,
                            int mid = -1,
                            int ms = 0);

    const std::vector<HasSsrUpdate>& updates() const { return updates_; }
    const std::vector<HasDecodedMessage>& messages() const { return messages_; }
    const HasDecoderStats& stats() const { return stats_; }

private:
    struct Collection {
        bool active = false;
        bool decoded = false;
        int ms = 0;
        GNSSTime first_time;
        GNSSTime last_time;
        std::map<int, std::array<uint8_t, kHasEncodedPageBytes>> pages;
        std::vector<uint8_t> message;
    };
    struct OrbitSet {
        GNSSTime time;
        double validity_s = 0.0;
        std::map<SatelliteId, HasOrbitEntry> entries;
    };
    struct ClockState {
        GNSSTime time;
        double validity_s = 0.0;
        HasClockEntry entry;
        int mask_id = -1;
        int iod_set_id = -1;
    };
    struct BiasState {
        GNSSTime time;
        double validity_s = 0.0;
        std::map<int, HasBiasEntry> entries;
    };

    void flush(const GNSSTime& time);
    void cut(const SatelliteId& satellite, const GNSSTime& time);
    void tryEmit(const SatelliteId& satellite);

    Options options_;
    std::array<Collection, 32> collections_{};
    std::map<int, HasMask> masks_;
    std::map<std::pair<int, int>, OrbitSet> orbit_sets_;
    std::map<SatelliteId, ClockState> clocks_;
    std::map<SatelliteId, BiasState> code_biases_;
    std::map<SatelliteId, BiasState> phase_biases_;
    std::map<SatelliteId, size_t> last_update_index_;
    std::vector<HasSsrUpdate> updates_;
    std::vector<HasDecodedMessage> messages_;
    HasDecoderStats stats_;
};

/// Read a page log and feed every page, in file order, to @p decoder.
bool decodeGalileoHasPages(const std::string& path,
                           HasPageInputFormat format,
                           GalileoHasDecoder& decoder,
                           HasPageReadStats* stats = nullptr,
                           std::string* error = nullptr);

}  // namespace io
}  // namespace libgnss
