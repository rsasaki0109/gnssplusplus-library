#include <libgnss++/io/madoca_l6.hpp>

#include <libgnss++/algorithms/madoca_parity.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/types.hpp>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace libgnss::io {
namespace {

namespace mp = libgnss::algorithms::madoca_parity;

// L6 framing (mdccssr.c) -----------------------------------------------------
constexpr std::uint32_t kPreamble = 0x1ACFFC1Du;
constexpr int kL6ByteLen = 218;     // L6 message byte length w/o R-S code
constexpr int kL6HeadBitLen = 49;   // L6 message header bit length
constexpr int kL6DataBitLen = 1695; // L6 message data part bit length
constexpr int kRtcmMnCssr = 4073;   // Compact SSR message number

// Compact SSR subtypes -------------------------------------------------------
constexpr int kStMask = 1;
constexpr int kStOc = 2;
constexpr int kStCc = 3;
constexpr int kStCb = 4;
constexpr int kStPb = 5;
constexpr int kStUra = 7;

// GNSS IDs (Table 4.2.2-7) ---------------------------------------------------
constexpr int kSysGpsId = 0;
constexpr int kSysGloId = 1;
constexpr int kSysGalId = 2;
constexpr int kSysBdsId = 3;
constexpr int kSysQzsId = 4;
constexpr int kSysBd3Id = 7;

constexpr int kMaxSatMask = 40;   // Satellite Mask (Table 4.2.2-8)
constexpr int kMaxSigMask = 16;   // Signal Mask (Table 4.2.2-9)

constexpr int kInvalid15Bit = -16384;
constexpr int kInvalid13Bit = -4096;
constexpr int kInvalid11Bit = -1024;

constexpr double kSsrInvalidCbias = -81.92;    // RTKLIB SSR_INVALID_CBIAS
constexpr double kSsrInvalidPbias = -52.4288;  // RTKLIB SSR_INVALID_PBIAS

// Satellite mask defaults per GNSS (Table 4.2.2-8), 40-bit (U<<8 | L).
constexpr std::uint64_t satmask(std::uint32_t upper, std::uint32_t lower) {
    return (static_cast<std::uint64_t>(upper) << 8) | lower;
}

// SSR update intervals (Table 4.2.2-6).
constexpr double kSsrUdInt[16] = {
    1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800,
};

// MADOCA-PPP Sub Type Transmission Pattern (Figure 4.2.2-1): frame length
// (-1: invalid) keyed by the first subtype in a sub-frame group.
constexpr int kFrameLen[16] = {
    -1, 5, -1, 3, 5, 5, -1, 3, -1, -1, -1, -1, -1, -1, -1, -1,
};

// Compact SSR signal masks (Table 4.2.2-9) w/o SBAS. Values are RTKLIB CODE_*.
constexpr std::uint8_t kCssrSigGps[16] = {
    1, 2, 3, 7, 8, 12, 16, 17, 18, 19, 20, 24, 25, 26, 0, 0,
    // L1C,L1P,L1W,L1S,L1L,L1X,L2S,L2L,L2X,L2P,L2W,L5I,L5Q,L5X
};
constexpr std::uint8_t kCssrSigGlo[16] = {
    1, 2, 14, 19, 66, 67, 68, 30, 31, 33, 44, 45, 46, 0, 0, 0,
    // L1C,L1P,L2C,L2P,L4A,L4B,L4X,L6A,L6B,L6X,L3I,L3Q,L3X
};
constexpr std::uint8_t kCssrSigGal[16] = {
    11, 1, 12, 24, 25, 26, 27, 28, 29, 37, 38, 39, 31, 32, 0, 0,
    // L1B,L1C,L1X,L5I,L5Q,L5X,L7I,L7Q,L7X,L8I,L8Q,L8X,L6B,L6C
};
constexpr std::uint8_t kCssrSigBd2[16] = {
    40, 41, 18, 42, 43, 33, 27, 28, 29, 0, 0, 0, 0, 0, 0, 0,
    // L2I,L2Q,L2X,L6I,L6Q,L6X,L7I,L7Q,L7X
};
constexpr std::uint8_t kCssrSigBd3[16] = {
    40, 41, 0, 42, 43, 0, 61, 62, 63, 56, 2, 12, 57, 58, 26, 0,
    // L2I,L2Q,-,L6I,L6Q,-,L7D,L7P,L7Z,L1D,L1P,L1X,L5D,L5P,L5X
};
constexpr std::uint8_t kCssrSigQzs[16] = {
    1, 7, 8, 12, 16, 17, 18, 24, 25, 26, 35, 36, 60, 9, 0, 0,
    // L1C,L1S,L1L,L1X,L2S,L2L,L2X,L5I,L5Q,L5X,L6S,L6L,L6E,L1E
};
constexpr std::uint8_t kCssrSigRsv[16] = {0};

int flgcnt16(std::uint16_t flg) {
    int cnt = 0;
    for (int i = 0; i < 16; ++i) {
        if ((flg >> i) & 0x1) ++cnt;
    }
    return cnt;
}

int gnssid2sys(int gnssid) {
    switch (gnssid) {
        case kSysGpsId: return mp::kSysGps;
        case kSysGloId: return mp::kSysGlo;
        case kSysGalId: return mp::kSysGal;
        case kSysBdsId: return mp::kSysCmp;
        case kSysQzsId: return mp::kSysQzs;
        case kSysBd3Id: return mp::kSysCmp;
        default: return mp::kSysNone;
    }
}

int svmask2list(std::uint64_t mask, int gnssid, int* satlist, int* gidlist) {
    int ns = 0;
    std::uint64_t svbit = static_cast<std::uint64_t>(1) << (kMaxSatMask - 1);
    for (int i = 0; i < kMaxSatMask; ++i) {
        if (mask & svbit) {
            int offp;
            switch (gnssid) {
                case kSysQzsId: offp = 193; break;
                case kSysBd3Id: offp = 19; break;
                default: offp = 1;
            }
            satlist[ns] = mp::satno(gnssid2sys(gnssid), offp + i);
            gidlist[ns] = gnssid;
            ++ns;
        }
        svbit >>= 1;
    }
    return ns;
}

// RTKLIB time helpers (rtkcmn.c), ported for the L6E epoch reconstruction.
constexpr double kGpst0[6] = {1980.0, 1.0, 6.0, 0.0, 0.0, 0.0};

MadocaGtime epoch2time(const double* ep) {
    static const int doy[] = {1, 32, 60, 91, 121, 152, 182,
                              213, 244, 274, 305, 335};
    MadocaGtime time;
    const int year = static_cast<int>(ep[0]);
    const int mon = static_cast<int>(ep[1]);
    const int day = static_cast<int>(ep[2]);
    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) {
        return time;
    }
    const int days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] +
                     day - 2 + ((year % 4 == 0 && mon >= 3) ? 1 : 0);
    const int sec = static_cast<int>(std::floor(ep[5]));
    time.time = static_cast<std::int64_t>(days) * 86400 +
                static_cast<int>(ep[3]) * 3600 + static_cast<int>(ep[4]) * 60 + sec;
    time.sec = ep[5] - sec;
    return time;
}

double time2gpst(MadocaGtime t, int* week) {
    const MadocaGtime t0 = epoch2time(kGpst0);
    const std::int64_t sec = t.time - t0.time;
    const int w = static_cast<int>(sec / (86400 * 7));
    if (week != nullptr) {
        *week = w;
    }
    return static_cast<double>(sec - static_cast<std::int64_t>(w) * 86400 * 7) + t.sec;
}

MadocaGtime gpst2time(int week, double sec) {
    MadocaGtime t = epoch2time(kGpst0);
    if (sec < -1e9 || 1e9 < sec) {
        sec = 0.0;
    }
    t.time += static_cast<std::int64_t>(86400) * 7 * week + static_cast<int>(sec);
    t.sec = sec - static_cast<int>(sec);
    return t;
}

// RTKLIB rtkcmn.c time2epoch, ported for the week->reference-epoch seed.
void time2epoch(MadocaGtime t, double* ep) {
    static const int mday[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                               31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                               31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                               31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    const int days = static_cast<int>(t.time / 86400);
    const int sec = static_cast<int>(t.time - static_cast<std::int64_t>(days) * 86400);
    int day = days % 1461;
    int mon = 0;
    for (; mon < 48; ++mon) {
        if (day >= mday[mon]) {
            day -= mday[mon];
        } else {
            break;
        }
    }
    ep[0] = 1970 + days / 1461 * 4 + mon / 12;
    ep[1] = mon % 12 + 1;
    ep[2] = day + 1;
    ep[3] = sec / 3600;
    ep[4] = sec % 3600 / 60;
    ep[5] = sec % 60 + t.sec;
}

// Reference calendar epoch (mid-week) seeding the decoder's week rollover.
void referenceEpochForWeek(int week, double ep[6]) {
    if (week <= 0) {
        ep[0] = 2025.0; ep[1] = 4.0; ep[2] = 1.0;
        ep[3] = ep[4] = ep[5] = 0.0;
        return;
    }
    time2epoch(gpst2time(week, 302400.0), ep);  // Wednesday of the GPS week
}

void adjweek(MadocaGtime* gt, double tow) {
    // gt is seeded with a nonzero reference epoch, so the RTKLIB
    // "if (gt->time == 0) get cpu time" branch never applies here.
    int week;
    const double tow_p = time2gpst(*gt, &week);
    if (tow < tow_p - 302400.0) {
        tow += 604800.0;
    } else if (tow > tow_p + 302400.0) {
        tow -= 604800.0;
    }
    *gt = gpst2time(week, tow);
}

int sigmask2list(std::uint16_t mask, int gnssid, int* siglist) {
    const std::uint8_t* sigs;
    switch (gnssid) {
        case kSysGpsId: sigs = kCssrSigGps; break;
        case kSysGloId: sigs = kCssrSigGlo; break;
        case kSysGalId: sigs = kCssrSigGal; break;
        case kSysBdsId: sigs = kCssrSigBd2; break;
        case kSysQzsId: sigs = kCssrSigQzs; break;
        case kSysBd3Id: sigs = kCssrSigBd3; break;
        default: sigs = kCssrSigRsv;
    }
    int nsig = 0;
    std::uint16_t sigbit = static_cast<std::uint16_t>(1) << (kMaxSigMask - 1);
    for (int i = 0; i < kMaxSigMask; ++i) {
        if (mask & sigbit) {
            siglist[nsig] = sigs[i];
            ++nsig;
        }
        sigbit >>= 1;
    }
    return nsig;
}

}  // namespace

MadocaL6eDecoder::MadocaL6eDecoder() {
    const double ep[6] = {2025.0, 4.0, 1.0, 0.0, 0.0, 0.0};
    seed_ = epoch2time(ep);
    reset();
}

void MadocaL6eDecoder::setReferenceEpoch(const double ep[6]) {
    seed_ = epoch2time(ep);
    reset();
}

void MadocaL6eDecoder::reset() {
    nbyte_ = 0;
    for (auto& b : msg_buff_) b = 0;
    for (int p = 0; p < kMaxPrn; ++p) {
        channels_[p] = ChannelState{};
        channels_[p].gt = seed_;  // seed week-number determination
    }
    for (int s = 0; s < kMaxSat; ++s) {
        ssr_[s] = MadocaSsrCorrection{};
    }
}

const MadocaSsrCorrection& MadocaL6eDecoder::correction(int sat) const {
    static const MadocaSsrCorrection kEmpty{};
    if (sat < 1 || sat > kMaxSat) {
        return kEmpty;
    }
    return ssr_[sat - 1];
}

int MadocaL6eDecoder::decodeHead(const ChannelState& ch, int* sync, int* iod,
                                 int* ep, double* udint, int i, int st) const {
    const int n = (st == kStMask) ? 20 : 12;
    *ep = static_cast<int>(mp::getbitu(ch.buff, i, n));
    i += n;  // GPS or GNSS Hourly Epoch Time 1s
    const int udi = static_cast<int>(mp::getbitu(ch.buff, i, 4));
    i += 4;  // SSR Update Interval
    *sync = static_cast<int>(mp::getbitu(ch.buff, i, 1));
    i += 1;  // Multiple Message Indicator
    *iod = static_cast<int>(mp::getbitu(ch.buff, i, 4));
    i += 4;  // IOD SSR
    *udint = kSsrUdInt[udi];
    return i;
}

int MadocaL6eDecoder::decodeMask(ChannelState& ch, int i) {
    int sync, iod, ep;
    double udint;
    i = decodeHead(ch, &sync, &iod, &ep, &udint, i, kStMask);

    ch.tow0 = ep / 3600 * 3600;
    ch.ep0 = ep % 3600;
    adjweek(&ch.gt, ep);
    ch.iod = iod;
    ch.ns = 0;
    for (int j = 0; j < kMaxSys; ++j) {
        ch.nsig[j] = 0;
        ch.nsat[j] = 0;
    }
    for (int j = 0; j < kMaxSat; ++j) {
        ch.cellmask[j] = 0;
    }

    const int ngnss = static_cast<int>(mp::getbitu(ch.buff, i, 4));
    i += 4;
    for (int j = 0; j < ngnss; ++j) {
        const int gnssid = static_cast<int>(mp::getbitu(ch.buff, i, 4));
        i += 4;
        std::uint64_t gnssmask;
        switch (gnssid) {
            case kSysGpsId: gnssmask = satmask(0xFFFFFFFFu, 0xFF); break;
            case kSysGloId: gnssmask = satmask(0xFFFFFF00u, 0x00); break;
            case kSysGalId: gnssmask = satmask(0xFFFFFFFFu, 0xF8); break;
            case kSysBdsId: gnssmask = satmask(0xFFFFFFFFu, 0xFF); break;
            case kSysQzsId: gnssmask = satmask(0xFFC00000u, 0x00); break;
            case kSysBd3Id: gnssmask = satmask(0xFFFFFFFFu, 0xFF); break;
            default: gnssmask = satmask(0xFFFFFFFFu, 0xFF);
        }
        std::uint64_t svmask = static_cast<std::uint64_t>(mp::getbitu(ch.buff, i, 8)) << 32;
        i += 8;
        svmask |= mp::getbitu(ch.buff, i, 32);
        i += 32;  // Sat mask
        svmask &= gnssmask;
        const std::uint16_t sigmask =
            static_cast<std::uint16_t>(mp::getbitu(ch.buff, i, 16));
        i += 16;  // Signal mask
        const int cmaflg = static_cast<int>(mp::getbitu(ch.buff, i, 1));
        i += 1;  // Cell-mask Availability Flag
        const int nsat =
            svmask2list(svmask, gnssid, &ch.satlist[ch.ns], &ch.gidlist[ch.ns]);
        const int nsig = sigmask2list(sigmask, gnssid, ch.siglist[gnssid]);

        if (cmaflg) {
            for (int k = 0; k < nsat; ++k) {
                const int cellmask = static_cast<int>(mp::getbitu(ch.buff, i, nsig));
                i += nsig;  // Cell-mask
                ch.cellmask[ch.ns + k] = cellmask;
                ch.ncell[ch.ns + k] = flgcnt16(static_cast<std::uint16_t>(cellmask));
            }
        } else {
            for (int k = 0; k < nsat; ++k) {
                ch.cellmask[ch.ns + k] = (1 << nsig) - 1;
                ch.ncell[ch.ns + k] = nsig;
            }
        }
        ch.nsat[gnssid] = nsat;
        ch.nsig[gnssid] = nsig;
        ch.ns += nsat;
    }
    return i;
}

int MadocaL6eDecoder::decodeOrbit(ChannelState& ch, int i, int apply) {
    int sync, iod, ep;
    double udint;
    i = decodeHead(ch, &sync, &iod, &ep, &udint, i, kStOc);
    if (ch.tow0 < 0) {
        return -1;  // unprocessed mask
    }
    if (ch.ep0 > ep) ep += 3600;
    adjweek(&ch.gt, ch.tow0 + ep);
    if (ch.iod != iod) {
        apply = 0;
    }
    for (int j = 0; j < ch.ns; ++j) {
        const int n = (ch.gidlist[j] == kSysGalId) ? 10 : 8;
        const int iode = static_cast<int>(mp::getbitu(ch.buff, i, n));
        i += n;  // IODE
        int deph[3];
        deph[0] = mp::getbits(ch.buff, i, 15); i += 15;  // Delta Radial
        deph[1] = mp::getbits(ch.buff, i, 13); i += 13;  // Delta Along-Track
        deph[2] = mp::getbits(ch.buff, i, 13); i += 13;  // Delta Cross-Track

        const int sat = ch.satlist[j];
        if (sat == 0) continue;  // unsupported sat
        if (deph[0] == kInvalid15Bit || deph[1] == kInvalid13Bit ||
            deph[2] == kInvalid13Bit) {
            continue;  // invalid value
        }
        if (apply) {
            MadocaSsrCorrection& s = ssr_[sat - 1];
            s.t0[0] = ch.gt;
            s.udi[0] = udint;
            s.iod[0] = iod;
            s.iode = iode;
            s.deph[0] = deph[0] * 0.0016;
            s.deph[1] = deph[1] * 0.0064;
            s.deph[2] = deph[2] * 0.0064;
            for (int k = 0; k < 3; ++k) s.ddeph[k] = 0.0;
            s.update = 1;
        }
    }
    return i;
}

int MadocaL6eDecoder::decodeClock(ChannelState& ch, int i, int apply) {
    int sync, iod, ep;
    double udint;
    i = decodeHead(ch, &sync, &iod, &ep, &udint, i, kStCc);
    if (ch.tow0 < 0) {
        return -1;  // unprocessed mask
    }
    if (ch.ep0 > ep) ep += 3600;
    adjweek(&ch.gt, ch.tow0 + ep);
    if (ch.iod != iod) {
        apply = 0;
    }
    for (int j = 0; j < ch.ns; ++j) {
        const int dclk = mp::getbits(ch.buff, i, 15);
        i += 15;  // Delta Clock C0
        const int sat = ch.satlist[j];
        if (sat == 0) continue;  // unsupported sat
        if (dclk == kInvalid15Bit) continue;  // invalid value
        if (apply) {
            MadocaSsrCorrection& s = ssr_[sat - 1];
            s.t0[1] = ch.gt;
            s.udi[1] = udint;
            s.iod[1] = iod;
            s.dclk[0] = dclk * 0.0016;
            s.dclk[1] = 0.0;
            s.dclk[2] = 0.0;
            s.update = 1;
        }
    }
    return i;
}

int MadocaL6eDecoder::decodeCodeBias(ChannelState& ch, int i, int apply) {
    int sync, iod, ep;
    double udint;
    i = decodeHead(ch, &sync, &iod, &ep, &udint, i, kStCb);
    if (ch.tow0 < 0) {
        return -1;  // unprocessed mask
    }
    if (ch.ep0 > ep) ep += 3600;
    adjweek(&ch.gt, ch.tow0 + ep);
    if (ch.iod != iod) {
        apply = 0;
    }
    for (int j = 0; j < ch.ns; ++j) {
        double cbias[MadocaSsrCorrection::kMaxCode] = {};
        int vcbias[MadocaSsrCorrection::kMaxCode] = {};
        const int sat = ch.satlist[j];
        const int nsig = ch.nsig[ch.gidlist[j]];
        int mask = 1 << (nsig - 1);
        for (int k = 0; k < nsig; ++k) {
            if (ch.cellmask[j] & mask) {
                const int cb = mp::getbits(ch.buff, i, 11);
                i += 11;  // Code Bias
                if (sat != 0) {
                    const int idx = ch.siglist[ch.gidlist[j]][k] - 1;
                    cbias[idx] = cb * 0.02;
                    vcbias[idx] = 1;
                    if (cb == kInvalid11Bit) {
                        cbias[idx] = kSsrInvalidCbias;
                    }
                }
            }
            mask >>= 1;
        }
        if (apply && sat != 0) {
            MadocaSsrCorrection& s = ssr_[sat - 1];
            s.t0[4] = ch.gt;
            s.udi[4] = udint;
            s.iod[4] = iod;
            for (int k = 0; k < MadocaSsrCorrection::kMaxCode; ++k) {
                s.cbias[k] = static_cast<float>(cbias[k]);
                s.vcbias[k] = vcbias[k];
            }
            s.update = 1;
        }
    }
    return i;
}

int MadocaL6eDecoder::decodePhaseBias(ChannelState& ch, int i, int apply) {
    int sync, iod, ep;
    double udint;
    i = decodeHead(ch, &sync, &iod, &ep, &udint, i, kStPb);
    if (ch.tow0 < 0) {
        return -1;  // unprocessed mask
    }
    if (ch.ep0 > ep) ep += 3600;
    adjweek(&ch.gt, ch.tow0 + ep);
    if (ch.iod != iod) {
        apply = 0;
    }
    for (int j = 0; j < ch.ns; ++j) {
        double pbias[MadocaSsrCorrection::kMaxCode] = {};
        int discnt[MadocaSsrCorrection::kMaxCode] = {};
        int vpbias[MadocaSsrCorrection::kMaxCode] = {};
        const int sat = ch.satlist[j];
        const int nsig = ch.nsig[ch.gidlist[j]];
        int mask = 1 << (nsig - 1);
        for (int k = 0; k < nsig; ++k) {
            if (ch.cellmask[j] & mask) {
                const int pb = mp::getbits(ch.buff, i, 15);
                i += 15;  // Phase Bias
                const int di = static_cast<int>(mp::getbitu(ch.buff, i, 2));
                i += 2;  // Discontinuity Indicator
                if (sat != 0) {
                    const int idx = ch.siglist[ch.gidlist[j]][k] - 1;
                    pbias[idx] = pb * 0.001;
                    vpbias[idx] = 1;
                    discnt[idx] = di;
                    if (pb == kInvalid15Bit) {
                        pbias[idx] = kSsrInvalidPbias;
                    }
                }
            }
            mask >>= 1;
        }
        if (apply && sat != 0) {
            MadocaSsrCorrection& s = ssr_[sat - 1];
            s.t0[5] = ch.gt;
            s.udi[5] = udint;
            s.iod[5] = iod;
            s.yaw_ang = 0.0;
            s.yaw_rate = 0.0;
            for (int k = 0; k < MadocaSsrCorrection::kMaxCode; ++k) {
                s.pbias[k] = static_cast<float>(pbias[k]);
                s.vpbias[k] = vpbias[k];
                s.discnt[k] = discnt[k];
            }
            s.update = 1;
        }
    }
    return i;
}

int MadocaL6eDecoder::decodeUra(ChannelState& ch, int i, int apply) {
    int sync, iod, ep;
    double udint;
    i = decodeHead(ch, &sync, &iod, &ep, &udint, i, kStUra);
    if (ch.tow0 < 0) {
        return -1;  // unprocessed mask
    }
    if (ch.ep0 > ep) ep += 3600;
    adjweek(&ch.gt, ch.tow0 + ep);
    if (ch.iod != iod) {
        apply = 0;
    }
    for (int j = 0; j < ch.ns; ++j) {
        const int ura = static_cast<int>(mp::getbitu(ch.buff, i, 6));
        i += 6;
        const int sat = ch.satlist[j];
        if (sat == 0) continue;  // unsupported sat
        if (apply) {
            MadocaSsrCorrection& s = ssr_[sat - 1];
            s.t0[3] = ch.gt;
            s.udi[3] = udint;
            s.iod[3] = iod;
            s.ura = ura;
            s.update = 1;
        }
    }
    return i;
}

int MadocaL6eDecoder::decodeL6eMessage() {
    int i = 0;
    const std::uint32_t preamb = mp::getbitu(msg_buff_, i, 32); i += 32;
    const unsigned prn = mp::getbitu(msg_buff_, i, 8); i += 8;
    // The 8-bit "L6 Msg. Type ID" decomposes into vid(3)+mgfid(2)+csid(1)+
    // anme(1)+si(1); MADOCALIB reads it without advancing i, then re-reads the
    // sub-fields from the same position.
    const unsigned vid = mp::getbitu(msg_buff_, i, 3); i += 3;   // Vendor ID
    const unsigned mgfid = mp::getbitu(msg_buff_, i, 2); i += 2; // Msg.Gen.Facility ID
    i += 1;  // Correction Service ID
    i += 1;  // Applicable Nav. Msg. Ext.
    const unsigned si = mp::getbitu(msg_buff_, i, 1); i += 1;     // Subframe Indicator
    const unsigned alert = mp::getbitu(msg_buff_, i, 1); i += 1;  // Alert Flag

    int j;
    switch (prn) {  // ref [1] Table 3-1
        case 204: j = 0; break;
        case 205: j = 1; break;
        case 206: j = 2; break;
        case 207: j = 3; break;
        case 209: j = 4; break;
        case 210: j = 5; break;
        case 211: j = 6; break;
        default: return -1;
    }

    if (preamb != kPreamble) return -1;
    if (alert) return -1;       // alert-ignore option not modeled
    if (vid != 2) return 0;     // unsupported vendor ID

    ChannelState& ch = channels_[j];
    if (ch.prn == -1) {
        ch.prn = static_cast<int>(prn);
    } else if (ch.prn != static_cast<int>(prn)) {
        ch.prn_chk_cnt++;
        if (ch.prn_chk_cnt < kMaxPrn) {
            return -1;
        }
        ch.prn = static_cast<int>(prn);
    }
    ch.prn_chk_cnt = 0;

    if (si == 1) ch.maxframe = 0;  // First Data

    // Frame recognition.
    if (ch.maxframe <= 0) {
        const int mn = static_cast<int>(mp::getbitu(msg_buff_, i, 12));
        const int st = static_cast<int>(mp::getbitu(msg_buff_, i + 12, 4));
        if (mn != kRtcmMnCssr) return -1;
        if ((ch.maxframe = kFrameLen[st]) < 1) return -1;
        ch.frame = 0;
        ch.ibuff = 0;
    }

    // Save data part (1695 bits) into the assembled buffer.
    ch.frame++;
    int ibuff;
    int n;
    std::uint8_t trailing_mask;
    switch (ch.frame) {
        case 1: ibuff = 0;   n = 8; trailing_mask = 0xFE; break;
        case 2: ibuff = 211; n = 1; trailing_mask = 0xFC; break;
        case 3: ibuff = 423; n = 2; trailing_mask = 0xF8; break;
        case 4: ibuff = 635; n = 3; trailing_mask = 0xF0; break;
        case 5: ibuff = 847; n = 4; trailing_mask = 0xE0; break;
        default: return -1;
    }
    ch.buff[ibuff++] |= static_cast<std::uint8_t>(mp::getbitu(msg_buff_, i, n));
    i += n;
    while (i < kL6HeadBitLen + kL6DataBitLen) {
        ch.buff[ibuff++] = static_cast<std::uint8_t>(mp::getbitu(msg_buff_, i, 8));
        i += 8;
    }
    ch.buff[ibuff - 1] &= trailing_mask;

    if (ch.frame != ch.maxframe) return 0;  // no message yet

    int ret = 10;  // input ssr messages
    i = 0;
    while (i < kL6DataBitLen * ch.maxframe) {
        const int mn = static_cast<int>(mp::getbitu(ch.buff, i, 12)); i += 12;
        if (mn != kRtcmMnCssr) continue;
        const int st = static_cast<int>(mp::getbitu(ch.buff, i, 4)); i += 4;
        if (st != kStMask && st != kStOc && st != kStCc && st != kStCb &&
            st != kStPb && st != kStUra) {
            continue;
        }
        if (st == kStMask) {
            if (ch.mgfid != static_cast<int>(mgfid)) {
                ch.mgfid = static_cast<int>(mgfid);
            }
            i = decodeMask(ch, i);
        } else {
            int apply = 1;
            // ref[1] 4.2.1(3): facility IDs 0 and 2 are equivalent, 1 and 3.
            if ((ch.mgfid & 0x1) != (static_cast<int>(mgfid) & 0x1)) {
                apply = 0;
            }
            static const bool kStDump = (std::getenv("GNSS_PPP_MADOCA_CBIAS_DUMP") != nullptr);
            if (kStDump) {
                std::fprintf(stderr, "[MADOCA-ST] st=%d apply=%d mgfid=%d ch.mgfid=%d\n",
                             st, apply, static_cast<int>(mgfid), ch.mgfid);
            }
            switch (st) {
                case kStOc: i = decodeOrbit(ch, i, apply); break;
                case kStCc: i = decodeClock(ch, i, apply); break;
                case kStCb: i = decodeCodeBias(ch, i, apply); break;
                case kStPb: i = decodePhaseBias(ch, i, apply); break;
                case kStUra: i = decodeUra(ch, i, apply); break;
                default: ret = -1; break;
            }
        }
        if (i < 0) {
            ret = -1;
            break;
        }
    }
    ch.maxframe = 0;
    return ret;
}

int MadocaL6eDecoder::inputByte(std::uint8_t data) {
    // Synchronize on the L6 preamble (0x1ACFFC1D).
    if (nbyte_ == 0 && data != 0x1A) {
        return 0;
    } else if ((nbyte_ == 1 && data != 0xCF) ||
               (nbyte_ == 2 && data != 0xFC) ||
               (nbyte_ == 3 && data != 0x1D)) {
        nbyte_ = 0;
        if (data != 0x1A) {
            return 0;
        }
    }
    msg_buff_[nbyte_++] = data;

    if (nbyte_ < kL6ByteLen) return 0;

    nbyte_ = 0;
    return decodeL6eMessage();
}

namespace {

namespace mpc = libgnss::algorithms::madoca_parity;

// MADOCA-PPP GNSS system constant (madoca_parity kSys*) to native enum.
libgnss::GNSSSystem mpSysToGnss(int mpsys) {
    switch (mpsys) {
        case mpc::kSysGps: return libgnss::GNSSSystem::GPS;
        case mpc::kSysGlo: return libgnss::GNSSSystem::GLONASS;
        case mpc::kSysGal: return libgnss::GNSSSystem::Galileo;
        case mpc::kSysQzs: return libgnss::GNSSSystem::QZSS;
        case mpc::kSysCmp: return libgnss::GNSSSystem::BeiDou;
        default: return libgnss::GNSSSystem::UNKNOWN;
    }
}

}  // namespace

// Build one native SSR correction from a satellite's decoded Compact SSR state.
// Returns false when the satellite carries no orbit/clock or maps to no system.
// The sample is time-stamped at the most recent of the orbit/clock t0 so a live
// stream of orbit- and clock-paced updates yields a monotonic time series.
bool buildMadocaSsrCorrection(int sat, const MadocaSsrCorrection& c,
                              libgnss::SSROrbitClockCorrection& out) {
    const bool has_orbit = c.t0[0].time != 0;  // t0[eph]
    const bool has_clock = c.t0[1].time != 0;  // t0[clk]
    if (!has_orbit && !has_clock) {
        return false;
    }
    int prn = 0;
    const libgnss::GNSSSystem gsys = mpSysToGnss(mpc::satsys(sat, &prn));
    if (gsys == libgnss::GNSSSystem::UNKNOWN || prn <= 0) {
        return false;
    }

    // The parity sat-index layout numbers QZSS PRNs 193-202 (RTKLIB satno
    // convention) while the RINEX reader keys QZSS observations 1-based
    // (J02 -> SatelliteId(QZSS, 2)). Without this normalization every QZSS
    // SSR correction is filed under PRN 193+, the per-epoch lookup keyed by
    // the 1-based observation PRN never matches, and QZSS silently runs on
    // uncorrected broadcast orbit/clock/bias (a ~1 m vertical bias at
    // stations that track QZSS).
    //
    // DEFAULT OFF (opt-in via GNSS_PPP_QZSS_SSR_PRN_FIX=1): applying the
    // (correct) QZSS SSR corrections shifts the MADOCA parity solution because
    // the estimator was compensating for the missing QZSS correction; on the
    // current parity datasets it improves ALIC but regresses MIZU, so it is not
    // yet safe to enable by default. Kept as opt-in until the estimator-layer
    // bias it exposes is resolved.
    static const bool kQzssPrnFix =
        (std::getenv("GNSS_PPP_QZSS_SSR_PRN_FIX") != nullptr);
    constexpr int kQzssPrnOffset = 192;  // 193 (RTKLIB) -> 1 (RINEX/native)
    if (kQzssPrnFix && gsys == libgnss::GNSSSystem::QZSS && prn > kQzssPrnOffset) {
        prn -= kQzssPrnOffset;
    }

    out = libgnss::SSROrbitClockCorrection{};
    out.satellite = libgnss::SatelliteId(gsys, static_cast<std::uint8_t>(prn));

    const bool clock_is_later =
        has_clock && (!has_orbit || c.t0[1].time >= c.t0[0].time);
    const MadocaGtime& t0 = clock_is_later ? c.t0[1] : c.t0[0];
    int week = 0;
    const double tow = time2gpst(t0, &week);
    out.time = libgnss::GNSSTime(week, tow);

    if (has_orbit) {
        out.orbit_correction_ecef =
            libgnss::Vector3d(c.deph[0], c.deph[1], c.deph[2]);  // RAC
        out.orbit_valid = true;
        out.iode = c.iode;  // broadcast eph IODE the orbit delta references
    }
    if (has_clock) {
        out.clock_correction_m = c.dclk[0];  // c0 polynomial term
        out.clock_valid = true;
    }

    static const bool kCbiasDump = (std::getenv("GNSS_PPP_MADOCA_CBIAS_DUMP") != nullptr);
    for (int k = 0; k < MadocaSsrCorrection::kMaxCode; ++k) {
        const int code = k + 1;  // cbias/pbias are held by CODE_*-1
        if (c.vcbias[k]) {
            const std::uint8_t id = madocaBiasCodeToRtcmSsrId(gsys, code);
            if (kCbiasDump) {
                std::fprintf(stderr,
                             "[MADOCA-CBIAS] sys=%d prn=%d code=%d id=%u cbias=%.4f\n",
                             static_cast<int>(gsys), prn, code,
                             static_cast<unsigned>(id), c.cbias[k]);
            }
            if (id > 0) {
                out.code_bias_m[id] = c.cbias[k];
            }
        }
        if (c.vpbias[k]) {
            const std::uint8_t id = madocaBiasCodeToRtcmSsrId(gsys, code);
            if (kCbiasDump) {
                std::fprintf(stderr,
                             "[MADOCA-PBIAS] sys=%d prn=%d code=%d id=%u pbias=%.4f\n",
                             static_cast<int>(gsys), prn, code,
                             static_cast<unsigned>(id), c.pbias[k]);
            }
            if (id > 0) {
                out.phase_bias_m[id] = c.pbias[k];
                out.phase_bias_discnt[id] = c.discnt[k];
            }
        }
    }
    out.code_bias_valid = !out.code_bias_m.empty();
    out.phase_bias_valid = !out.phase_bias_m.empty();
    return true;
}

std::uint8_t madocaBiasCodeToRtcmSsrId(libgnss::GNSSSystem system, int code) {
    using libgnss::GNSSSystem;
    switch (system) {
        case GNSSSystem::GPS:
            switch (code) {
                case mpc::kCodeL1C: return 2;                      // L1 C/A
                case mpc::kCodeL1P: case mpc::kCodeL1W: return 3;  // L1 P(Y)
                case mpc::kCodeL2S: case mpc::kCodeL2L:
                case mpc::kCodeL2X: return 8;                      // L2C
                case mpc::kCodeL2P: case mpc::kCodeL2W: return 9;  // L2 P(Y)
                case mpc::kCodeL5I: case mpc::kCodeL5Q:
                case mpc::kCodeL5X: return 22;                     // L5
            }
            break;
        case GNSSSystem::GLONASS:
            switch (code) {
                case mpc::kCodeL1C: return 2;   // G1 C/A
                case mpc::kCodeL1P: return 3;   // G1 P
                case mpc::kCodeL2C: return 8;   // G2 C/A
                case mpc::kCodeL2P: return 9;   // G2 P
            }
            break;
        case GNSSSystem::Galileo:
            switch (code) {
                case mpc::kCodeL1C: case mpc::kCodeL1B:
                case mpc::kCodeL1X: return 2;   // E1
                case mpc::kCodeL6B: case mpc::kCodeL6C:
                case mpc::kCodeL6X: return 8;   // E6
                case mpc::kCodeL7I: case mpc::kCodeL7Q:
                case mpc::kCodeL7X: return 14;  // E5b
                case mpc::kCodeL5I: case mpc::kCodeL5Q:
                case mpc::kCodeL5X: return 22;  // E5a
            }
            break;
        case GNSSSystem::BeiDou:
            switch (code) {
                case mpc::kCodeL2I: case mpc::kCodeL2Q:
                case mpc::kCodeL2X: return 2;   // B1I
                case mpc::kCodeL6I: case mpc::kCodeL6Q:
                case mpc::kCodeL6X: return 8;   // B3I
                case mpc::kCodeL7I: case mpc::kCodeL7Q:
                case mpc::kCodeL7X: return 14;  // B2I
            }
            break;
        case GNSSSystem::QZSS:
            switch (code) {
                case mpc::kCodeL1C: return 2;   // L1 C/A
                case mpc::kCodeL2S: case mpc::kCodeL2L:
                case mpc::kCodeL2X: return 8;   // L2C
                case mpc::kCodeL5I: case mpc::kCodeL5Q:
                case mpc::kCodeL5X: return 22;  // L5
            }
            break;
        default:
            break;
    }
    return 0;
}

int madocaL6eSnapshotToProducts(const MadocaL6eDecoder& decoder,
                                libgnss::SSRProducts& products) {
    products.setOrbitCorrectionsAreRac(true);
    int added = 0;
    for (int sat = 1; sat <= MadocaL6eDecoder::kMaxSat; ++sat) {
        libgnss::SSROrbitClockCorrection corr;
        if (buildMadocaSsrCorrection(sat, decoder.correction(sat), corr)) {
            products.addCorrection(corr);
            ++added;
        }
    }
    return added;
}

int decodeMadocaL6eFilesToProducts(const std::vector<std::string>& files,
                                   int gps_week,
                                   libgnss::SSRProducts& products) {
    products.setOrbitCorrectionsAreRac(true);
    double ref_ep[6];
    referenceEpochForWeek(gps_week, ref_ep);

    MadocaL6eDecoder decoder;
    decoder.setReferenceEpoch(ref_ep);

    constexpr int kN = MadocaL6eDecoder::kMaxSat;
    // Per-satellite last-emitted orbit/clock t0 (snapshot only on a change).
    std::vector<std::int64_t> last_orbit(kN + 1, -1);
    std::vector<std::int64_t> last_clock(kN + 1, -1);

    int added = 0;
    for (const std::string& file : files) {
        std::ifstream in(file, std::ios::binary);
        if (!in) {
            continue;
        }
        char raw = 0;
        while (in.get(raw)) {
            if (decoder.inputByte(static_cast<std::uint8_t>(raw)) != 10) {
                continue;
            }
            for (int sat = 1; sat <= kN; ++sat) {
                const MadocaSsrCorrection& c = decoder.correction(sat);
                const std::int64_t ot = c.t0[0].time;
                const std::int64_t ct = c.t0[1].time;
                if (ot == 0 && ct == 0) {
                    continue;
                }
                if (ot == last_orbit[sat] && ct == last_clock[sat]) {
                    continue;  // no orbit/clock change since the last sample
                }
                last_orbit[sat] = ot;
                last_clock[sat] = ct;
                libgnss::SSROrbitClockCorrection corr;
                if (buildMadocaSsrCorrection(sat, c, corr)) {
                    products.addCorrection(corr);
                    ++added;
                }
            }
        }
    }
    return added;
}

}  // namespace libgnss::io
