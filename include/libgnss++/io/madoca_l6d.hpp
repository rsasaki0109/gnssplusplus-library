#pragma once

#include <cstdint>

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
    MadocaIonoRegion re_;               // decoded scratch region (mdcl6d.re)
    int rid_ = 0;                       // decoded region ID (mdcl6d.rid)
};

}  // namespace libgnss::io
