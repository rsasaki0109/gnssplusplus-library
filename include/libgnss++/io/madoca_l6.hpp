#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace libgnss {
class SSRProducts;
enum class GNSSSystem : std::uint8_t;
}  // namespace libgnss

namespace libgnss::io {

// GPST instant in RTKLIB gtime_t form: integer seconds since 1970 plus a
// fractional part. Mirrors the layout the MADOCALIB decoder writes into ssr_t.
struct MadocaGtime {
    std::int64_t time = 0;  // integer seconds (time_t)
    double sec = 0.0;       // fraction of second
};

// Per-satellite MADOCA-PPP SSR correction values decoded from a QZSS L6E
// Compact SSR stream (IS-QZSS-MDC-004). This mirrors the value fields of the
// RTKLIB ssr_t struct that the MADOCALIB decoder fills. Epoch-time (t0) parity
// is intentionally deferred to a later increment; this struct carries the
// substantive correction quantities (orbit/clock/bias/URA) plus the metadata
// needed to interpret them.
struct MadocaSsrCorrection {
    static constexpr int kMaxCode = 68;  // RTKLIB MAXCODE

    MadocaGtime t0[6];     // epoch time {eph,clk,hrclk,ura,bias,pbias}
    int iode = 0;          // issue of data ephemeris
    int iod[6] = {};       // iod ssr {eph,clk,hrclk,ura,bias,pbias}
    double udi[6] = {};    // SSR update interval (s)
    int ura = 0;           // URA indicator
    double deph[3] = {};   // delta orbit {radial,along,cross} (m)
    double ddeph[3] = {};  // dot delta orbit (m/s)
    double dclk[3] = {};   // delta clock {c0,c1,c2} (m,m/s,m/s^2)
    float cbias[kMaxCode] = {};   // code biases (m), indexed by CODE-1
    int vcbias[kMaxCode] = {};    // code-bias valid flag
    double pbias[kMaxCode] = {};  // phase biases (m), indexed by CODE-1
    int vpbias[kMaxCode] = {};    // phase-bias valid flag
    int discnt[kMaxCode] = {};    // phase-bias discontinuity counter
    double yaw_ang = 0.0;
    double yaw_rate = 0.0;
    int update = 0;        // update flag (0:no update,1:update)
};

// Decoder for the QZSS L6E MADOCA-PPP Compact SSR message stream. Faithfully
// ports the MADOCALIB mdccssr.c byte synchronizer, sub-frame assembler, and
// subtype 1/2/3/4/5/7 decoders. Corrections accumulate into a per-satellite
// table that callers read after each successful decode.
class MadocaL6eDecoder {
public:
    static constexpr int kMaxSat = 221;     // RTKLIB MAXSAT (linked build)
    static constexpr int kMaxPrn = 7;       // defined MADOCA-PPP L6E channels
    static constexpr int kMaxSys = 16;      // GNSS ID space
    static constexpr int kMaxSigMask = 16;  // signal mask width

    MadocaL6eDecoder();

    // Seed the GPS week-number determination from a calendar epoch
    // (ep = {year,mon,day,hour,min,sec}), mirroring MADOCALIB init_mcssr().
    // Pick an epoch within half a week of the stream's true time. Resets
    // decode state. Must precede inputByte().
    void setReferenceEpoch(const double ep[6]);

    // Feed one L6E byte. Return value matches MADOCALIB input_qzssl6e():
    //   -1: error message, 0: no message, 10: SSR messages input this byte.
    int inputByte(std::uint8_t data);

    // Read the accumulated correction for satellite number sat (1..kMaxSat).
    // Out-of-range satellites return a static empty correction.
    const MadocaSsrCorrection& correction(int sat) const;

    // Clear all decoder and correction state.
    void reset();

private:
    // Per-channel (per L6E PRN) Compact SSR decode state. Mirrors mcssr_t.
    struct ChannelState {
        MadocaGtime gt;                // current epoch (week-rollover adjusted)
        std::uint8_t buff[1060] = {};  // assembled 5-frame Compact SSR message
        int ibuff = 0;
        int satlist[kMaxSat] = {};     // 0:invalid, else satno
        int gidlist[kMaxSat] = {};     // GNSS ID per listed satellite
        int cellmask[kMaxSat] = {};
        int ncell[kMaxSat] = {};
        int siglist[kMaxSys][kMaxSigMask] = {};  // 0:terminate, else CODE_xxx
        int nsat[kMaxSys] = {};
        int nsig[kMaxSys] = {};
        int ns = 0;
        int maxframe = 0;
        int frame = 0;
        int iod = 0;
        int tow0 = -1;  // <0 until a mask (ST1) has been processed
        int ep0 = 0;
        int prn = -1;
        int prn_chk_cnt = 0;
        int mgfid = -1;
    };

    int decodeL6eMessage();
    int decodeHead(const ChannelState& ch, int* sync, int* iod, int* ep,
                   double* udint, int i, int st) const;
    int decodeMask(ChannelState& ch, int i);
    int decodeOrbit(ChannelState& ch, int i, int apply);
    int decodeClock(ChannelState& ch, int i, int apply);
    int decodeCodeBias(ChannelState& ch, int i, int apply);
    int decodePhaseBias(ChannelState& ch, int i, int apply);
    int decodeUra(ChannelState& ch, int i, int apply);

    std::uint8_t msg_buff_[218] = {};  // raw L6 message (L6BYTELEN)
    int nbyte_ = 0;
    MadocaGtime seed_;                 // week-determination reference epoch
    ChannelState channels_[kMaxPrn];
    MadocaSsrCorrection ssr_[kMaxSat];
};

// Map a MADOCA Compact SSR bias code (an RTKLIB CODE_* enum value, i.e. the
// 1-based index used by MadocaSsrCorrection::cbias/pbias) to the RTCM SSR signal
// id that keys SSRProducts code/phase biases (see core/signals.hpp
// rtcmSsrSignalId). Returns 0 when the code has no native RTCM SSR id for the
// system. Only the per-band representative codes emitted by mcssr_sel_biascode
// are mapped, so distinct codes never collide on one id.
std::uint8_t madocaBiasCodeToRtcmSsrId(libgnss::GNSSSystem system, int code);

// Convert the decoder's current per-satellite Compact SSR snapshot into native
// SSR products: RAC orbit deltas (the products' RAC flag is set), the clock c0
// term (m), and code/phase biases re-keyed from RTKLIB CODE_* to RTCM SSR ids,
// each time-stamped from the clock t0 (GPS week/tow). Corrections are appended
// (existing products are kept). Returns the number of satellites added.
int madocaL6eSnapshotToProducts(const MadocaL6eDecoder& decoder,
                                libgnss::SSRProducts& products);

// Decode one or more MADOCA L6E files into a time series of SSR products,
// driving the decoder byte-for-byte and snapshotting each satellite whenever
// its orbit or clock correction epoch advances. gps_week seeds the decoder's
// week-rollover (a mid-week reference epoch is used; <=0 falls back to the
// sample epoch). Orbit deltas are RAC. Returns the number of samples appended.
int decodeMadocaL6eFilesToProducts(const std::vector<std::string>& files,
                                   int gps_week,
                                   libgnss::SSRProducts& products);

}  // namespace libgnss::io
