#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <libgnss++/io/madoca_l6.hpp>  // MadocaGtime

namespace libgnss::io {

// Per-satellite STEC polynomial entry decoded from a QZSS L6D MADOCA-PPP wide
// area ionospheric correction stream (IS-QZSS-MDC-002). Mirrors miono_sat_t.
struct MadocaIonoSat {
    MadocaGtime t0;        // correction time
    int sqi = 0;           // SSR STEC quality indicator
    double coef[6] = {};   // STEC poly. coef. {C00,C01,C10,C11,C02,C20}
};

// One STEC correction area (a region holds several). Mirrors miono_area_t.
struct MadocaIonoArea {
    static constexpr int kMaxSat = 221;  // RTKLIB MAXSAT (linked build)

    int avalid = 0;        // 0:invalid, 1:valid
    int sid = 0;           // shape ID (0:rectangle, 1:circle)
    int type = 0;          // STEC correction type (poly. order)
    double ref[2] = {};    // reference point {lat, lon} (deg)
    double span[2] = {};   // rect span {lat, lon} or circle {range, n/a}
    MadocaIonoSat sat[kMaxSat];
};

// One STEC coverage region. Mirrors miono_region_t.
struct MadocaIonoRegion {
    static constexpr int kMaxArea = 32;  // MIONO_MAX_ANUM

    int rvalid = 0;        // 0:invalid, 1:valid
    int ralert = 0;        // region alert flag
    int narea = 0;         // number of areas
    MadocaIonoArea area[kMaxArea];
};

// Decoder for the QZSS L6D MADOCA-PPP wide area ionospheric correction message
// stream. Faithfully ports the MADOCALIB mdciono.c byte synchronizer, the
// up-to-40-frame sub-frame assembler, and the MT1 (STEC coverage) / MT2 (STEC
// correction) decoders. After each fully assembled message the decoder exposes
// the freshly decoded region; callers persist it keyed by regionId() and then
// call clearRegionAfterUse(), mirroring postpos.c update_qzssl6d().
class MadocaL6dDecoder {
public:
    static constexpr int kMaxPrn = 3;        // MIONO_MAX_PRN (200,201,197-test)
    static constexpr int kMaxFrame = 40;     // MIONO_MAX_FRAME
    static constexpr int kL6ByteLen = 218;   // L6 message byte length w/o R-S
    static constexpr int kBuffLen = kL6ByteLen * kMaxFrame;  // assembled buffer

    MadocaL6dDecoder();

    // Seed the GPS week-number determination from a calendar epoch
    // (ep = {year,mon,day,hour,min,sec}), mirroring MADOCALIB init_miono().
    // Resets all decode and region state. Must precede inputByte().
    void setReferenceEpoch(const double ep[6]);

    // Feed one L6D byte. Return value matches MADOCALIB input_qzssl6d():
    //   -1: error message, 0: no message, 10: iono. corr. messages input.
    int inputByte(std::uint8_t data);

    // The region decoded by the most recent message that returned 10.
    const MadocaIonoRegion& region() const { return re_; }
    int regionId() const { return rid_; }

    // Epoch on the primary (PRN-200) channel, mirroring MADOCALIB _miono[0].gt:
    // the staleness gate consumed by MadocaIonoStore::getCorr / miono_get_corr.
    const MadocaGtime& decodeTime() const { return channels_[0].gt; }

    // Epoch of the most recently completed message, mirroring
    // mdcl6d_t::time. Unlike decodeTime(), this advances for PRN 201/197 and is
    // therefore the causal key for file snapshots from a secondary channel.
    const MadocaGtime& messageTime() const { return last_message_time_; }

    // Mirror postpos.c update_qzssl6d() reset of the scratch region between
    // messages: clear rvalid and every area's avalid plus per-sat t0.time.
    void clearRegionAfterUse();

    // Clear all decoder and region state.
    void reset();

private:
    // Per-channel (per L6D PRN) frame-assembly state. Mirrors miono_t.
    struct ChannelState {
        MadocaGtime gt;                 // current epoch (week-rollover adjusted)
        int mgfid = -1;                 // current Msg. Gen. Facility ID
        std::uint8_t buff[kBuffLen] = {};  // assembled L6D message
        int ibuff = 0;                  // end of buff
        int maxframe = 0;               // framelen[first subtype]
        int frame = 0;                  // frame counter
        int tow0 = -1;                  // hourly epoch base (<0 until coverage)
        int ep0 = 0;                    // hourly epoch regress check
        int iod = 0;                    // IOD SSR
    };

    int decodeL6dMessage();
    int decodeCoverage(ChannelState& ch, int i, int* imax);
    int decodeCorrection(ChannelState& ch, int i, int apply);

    std::uint8_t msg_buff_[256] = {};   // raw L6 message (mdcl6d.buff)
    int nbyte_ = 0;
    MadocaGtime seed_;                  // week-determination reference epoch
    ChannelState channels_[kMaxPrn];
    MadocaGtime last_message_time_;
    MadocaIonoRegion re_;               // decoded scratch region (mdcl6d.re)
    int rid_ = 0;                       // decoded region ID (mdcl6d.rid)
};

// Per-satellite materialized L1 ionospheric correction for a receiver position,
// mirroring RTKLIB pppiono_corr_t plus the selected region/area. Produced by
// MadocaIonoStore::getCorr.
struct MadocaIonoCorr {
    static constexpr int kMaxSat = 221;  // RTKLIB MAXSAT (linked build)

    MadocaGtime t0[kMaxSat];   // per-satellite correction time (time==0: absent)
    double dly[kMaxSat] = {};  // L1 slant ionospheric delay (m)
    double std[kMaxSat] = {};  // L1 slant delay standard deviation (m)
    int rid = -1;              // selected region ID
    int anum = 0;              // selected area number
};

// Persistent MADOCA-PPP L6D ionospheric region store plus the area-selection and
// STEC delay/std applier. Mirrors the RTKLIB pppiono_t region table: update()
// persists a freshly decoded region (mirroring postpos.c update_qzssl6d), and
// getCorr() mirrors miono_get_corr() — selecting the nearest valid area for a
// receiver ECEF position and materializing per-satellite L1 slant delay/std/time.
// The 256-slot region array is held sparsely (only used region IDs allocate).
class MadocaIonoStore {
public:
    static constexpr int kMaxRid = 256;  // MIONO_MAX_RID

    // Persist a decoded region into slot rid (0..kMaxRid-1). Out-of-range rid is
    // ignored. Mirrors update_qzssl6d: navs.pppiono.re[rid] = mdcl6d.re.
    void update(int rid, const MadocaIonoRegion& region);

    // Mirror miono_get_corr(): given receiver ECEF rr and the decoder epoch
    // (decode_time mirrors _miono[0].gt, the staleness gate), select the nearest
    // valid area and materialize out. Returns 1 when a correction was produced,
    // 0 otherwise. The L1 carrier frequency follows RTKLIB sat2freq(sat,CODE_L1C):
    // 1575.42 MHz for GPS/Galileo/QZSS/BeiDou. GLONASS is FCN-dependent; pass
    // gloFreqHz[sat-1] (Hz) to supply it, otherwise GLONASS delays are left at 0.
    int getCorr(const double rr[3], const MadocaGtime& decode_time,
                MadocaIonoCorr& out, const double* gloFreqHz = nullptr) const;

    // Clear the persistent region store.
    void reset();

private:
    const MadocaIonoArea* selectArea(const double rr[3], const double ll[2],
                                     int* rid, int* anum) const;

    std::map<int, MadocaIonoRegion> re_;  // sparse pppiono.re[MIONO_MAX_RID]
};

// Receiver-specific correction view emitted after one complete L6D region
// update. updated_region_id identifies the message that triggered the snapshot;
// correction.rid/anum identify the nearest area selected from the persistent
// store (which may belong to an earlier region update).
struct MadocaIonoSnapshot {
    MadocaGtime decode_time;
    int updated_region_id = -1;
    MadocaIonoCorr correction;
};

// Time-indexed receiver-specific L6D products. This container owns ordering,
// duplicate replacement, causal lookup, and freshness policy so PPP consumers
// do not need to know how raw files were replayed. Returned lookup pointers stay
// valid until the next mutating call.
class MadocaIonoProducts {
public:
    bool addSnapshot(const MadocaIonoSnapshot& snapshot);
    std::size_t addSnapshots(const std::vector<MadocaIonoSnapshot>& snapshots);

    // Latest snapshot whose decode time is not later than query_time. A
    // negative max_age_s disables the freshness gate; otherwise snapshots
    // older than max_age_s are rejected.
    const MadocaIonoSnapshot* latestAtOrBefore(
        const MadocaGtime& query_time, double max_age_s = -1.0) const;

    bool empty() const { return snapshots_.empty(); }
    std::size_t size() const { return snapshots_.size(); }
    const std::vector<MadocaIonoSnapshot>& snapshots() const { return snapshots_; }
    void clear() { snapshots_.clear(); }

private:
    std::vector<MadocaIonoSnapshot> snapshots_;
};

// Materialize the current persistent L6D store for one receiver. This is the
// reusable boundary shared by file replay and future streaming PPP ingestion.
// Returns true when a receiver area was selected and a snapshot was appended.
bool appendMadocaIonoSnapshot(const MadocaIonoStore& store,
                              const double receiver_ecef[3],
                              const MadocaGtime& decode_time,
                              int updated_region_id,
                              std::vector<MadocaIonoSnapshot>& snapshots,
                              const double* gloFreqHz = nullptr);

// Decode one raw QZSS L6D file and materialize a snapshot after every complete
// region update. The reference epoch seeds GPS-week reconstruction. Returns
// false for invalid arguments, unreadable input, decoder errors, or a file with
// no complete L6D messages; error_message is optional.
bool decodeMadocaL6dFileToSnapshots(
    const std::string& path,
    const double reference_epoch[6],
    const double receiver_ecef[3],
    std::vector<MadocaIonoSnapshot>& snapshots,
    std::string* error_message = nullptr,
    const double* gloFreqHz = nullptr);

}  // namespace libgnss::io
