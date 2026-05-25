#include <libgnss++/io/madoca_l6d.hpp>

#include <libgnss++/algorithms/madoca_parity.hpp>

#include <cmath>

namespace libgnss::io {
namespace {

namespace mp = libgnss::algorithms::madoca_parity;

// L6 framing (mdciono.c) -----------------------------------------------------
constexpr std::uint32_t kPreamble = 0x1ACFFC1Du;
constexpr int kL6ByteLen = 218;     // L6 message byte length w/o R-S code
constexpr int kL6HeadBitLen = 49;   // L6 message header bit length
constexpr int kL6DataBitLen = 1695; // L6 message data part bit length

// MADOCA-PPP iono. message types (Table 6.3.2-3) -----------------------------
constexpr int kMtCoverage = 1;   // MIONO_MT_COV
constexpr int kMtCorrection = 2;  // MIONO_MT_COR

// GNSS IDs (Table 6.3.2-8) ---------------------------------------------------
constexpr int kSysGpsId = 0;
constexpr int kSysGloId = 1;
constexpr int kSysGalId = 2;
constexpr int kSysBdsId = 3;
constexpr int kSysQzsId = 4;
constexpr int kMaxIonoSys = 5;   // MIONO_MAX_SYS

// Invalid values (Table 6.3.2-9) ---------------------------------------------
constexpr int kInvalid14Bit = -8192;
constexpr int kInvalid12Bit = -2048;
constexpr int kInvalid10Bit = -512;
constexpr int kInvalid8Bit = -128;

int gnssid2sys(int gnssid) {
    switch (gnssid) {
        case kSysGpsId: return mp::kSysGps;
        case kSysGloId: return mp::kSysGlo;
        case kSysGalId: return mp::kSysGal;
        case kSysBdsId: return mp::kSysCmp;
        case kSysQzsId: return mp::kSysQzs;
        default: return -1;
    }
}

// RTKLIB time helpers (rtkcmn.c), ported for the L6D epoch reconstruction.
// Identical to the proven L6E port; duplicated to keep the modules independent.
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

// RTKLIB geometry (rtkcmn.c), ported bit-for-bit for the L6D area selection and
// STEC delay so the latitude/longitude differences match MADOCALIB exactly.
constexpr double kReWgs84 = 6378137.0;            // WGS84 earth semimajor axis (m)
constexpr double kFeWgs84 = 1.0 / 298.257223563;  // WGS84 flattening
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kD2R = kPi / 180.0;
constexpr double kR2D = 180.0 / kPi;

double norm3(const double* a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

void ecef2pos(const double* r, double* pos) {
    const double e2 = kFeWgs84 * (2.0 - kFeWgs84);
    const double r2 = r[0] * r[0] + r[1] * r[1];  // dot(r,r,2)
    double z, zk, v = kReWgs84, sinp = 0.0;
    for (z = r[2], zk = 0.0; std::fabs(z - zk) >= 1E-4;) {
        zk = z;
        sinp = z / std::sqrt(r2 + z * z);
        v = kReWgs84 / std::sqrt(1.0 - e2 * sinp * sinp);
        z = r[2] + v * e2 * sinp;
    }
    pos[0] = r2 > 1E-12 ? std::atan(z / std::sqrt(r2))
                        : (r[2] > 0.0 ? kPi / 2.0 : -kPi / 2.0);
    pos[1] = r2 > 1E-12 ? std::atan2(r[1], r[0]) : 0.0;
    pos[2] = std::sqrt(r2 + z * z) - v;
}

void pos2ecef(const double* pos, double* r) {
    const double sinp = std::sin(pos[0]), cosp = std::cos(pos[0]);
    const double sinl = std::sin(pos[1]), cosl = std::cos(pos[1]);
    const double e2 = kFeWgs84 * (2.0 - kFeWgs84);
    const double v = kReWgs84 / std::sqrt(1.0 - e2 * sinp * sinp);
    r[0] = (v + pos[2]) * cosp * cosl;
    r[1] = (v + pos[2]) * cosp * sinl;
    r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}

// STEC URA -> standard deviation (m). Mirrors mdciono.c miono_std_stecura.
double stecUraStd(int ura) {
    if (ura <= 0) return 5.4665;   // STEC URA undefined/unknown -> worst case
    if (ura >= 63) return 5.4665;
    return (std::pow(3.0, (ura >> 3) & 7) * (1.0 + (ura & 7) / 4.0) - 1.0) * 1E-3;
}

// L1C carrier frequency (Hz), mirroring RTKLIB sat2freq(sat,CODE_L1C,nav).
// GPS/Galileo/QZSS/BeiDou map CODE_L1C to 1575.42 MHz; GLONASS is FCN-dependent.
double l1cFreqHz(int sat, const double* gloFreqHz) {
    int prn = 0;
    if (mp::satsys(sat, &prn) == mp::kSysGlo) {
        return (gloFreqHz != nullptr) ? gloFreqHz[sat - 1] : 0.0;
    }
    return 1575.42E6;
}

// L1 slant ionospheric delay (m) for satellite sat in area a at geodetic ll
// (deg). Mirrors mdciono.c miono_delay, including its use of coef[2] (C10) for
// the squared-longitude term in the type>=3 model (faithful to MADOCALIB).
double ionoDelay(int sat, const MadocaIonoArea& a, const double* ll,
                 const double* gloFreqHz) {
    const double* c = a.sat[sat - 1].coef;
    const double freq = l1cFreqHz(sat, gloFreqHz);
    const double dlat = ll[0] - a.ref[0];
    const double dlon = ll[1] - a.ref[1];
    double stec = c[0];  // STEC poly. {C00,C01,C10,C11,C02,C20}
    if (a.type >= 1) stec += c[1] * dlat + c[2] * dlon;
    if (a.type >= 2) stec += c[3] * dlat * dlon;
    if (a.type >= 3) stec += c[4] * dlat * dlat + c[2] * dlon * dlon;
    if (freq == 0.0) return 0.0;  // GLONASS without FCN: avoid div-by-zero
    return (40.31E16 / freq / freq) * stec;
}

}  // namespace

MadocaL6dDecoder::MadocaL6dDecoder() {
    const double ep[6] = {2025.0, 4.0, 1.0, 0.0, 0.0, 0.0};
    seed_ = epoch2time(ep);
    reset();
}

void MadocaL6dDecoder::setReferenceEpoch(const double ep[6]) {
    seed_ = epoch2time(ep);
    reset();
}

void MadocaL6dDecoder::reset() {
    nbyte_ = 0;
    for (auto& b : msg_buff_) b = 0;
    for (int p = 0; p < kMaxPrn; ++p) {
        channels_[p] = ChannelState{};
        channels_[p].gt = seed_;  // seed week-number determination
    }
    re_ = MadocaIonoRegion{};
    rid_ = 0;
}

void MadocaL6dDecoder::clearRegionAfterUse() {
    // Mirror postpos.c update_qzssl6d(): after persisting the region, clear
    // rvalid and every area's avalid plus the per-sat correction times.
    re_.rvalid = 0;
    for (int a = 0; a < MadocaIonoRegion::kMaxArea; ++a) {
        re_.area[a].avalid = 0;
        for (int s = 0; s < MadocaIonoArea::kMaxSat; ++s) {
            re_.area[a].sat[s].t0.time = 0;
        }
    }
}

int MadocaL6dDecoder::decodeCoverage(ChannelState& ch, int i, int* imax) {
    const int ep = static_cast<int>(mp::getbitu(ch.buff, i, 20)); i += 20;  // GNSS Epoch Time 1s
    i += 4;   // SSR Update Interval
    i += 1;   // Multiple Message Indicator
    const int iod = static_cast<int>(mp::getbitu(ch.buff, i, 4)); i += 4;   // IOD SSR
    rid_ = static_cast<int>(mp::getbitu(ch.buff, i, 8)); i += 8;            // Region ID
    const int ralert = static_cast<int>(mp::getbitu(ch.buff, i, 1)); i += 1;  // Region Alert flag
    const int corlen = static_cast<int>(mp::getbitu(ch.buff, i, 16)); i += 16; // Length of Correction Messages
    const int narea = static_cast<int>(mp::getbitu(ch.buff, i, 5)); i += 5;    // No. of Area

    ch.tow0 = ep / 3600 * 3600;
    ch.ep0 = ep % 3600;
    adjweek(&ch.gt, ep);
    ch.iod = iod;
    re_.ralert = ralert;
    re_.narea = narea;
    re_.rvalid = 1;

    for (int j = 0; j < narea; ++j) {
        const int anum = static_cast<int>(mp::getbitu(ch.buff, i, 5)); i += 5;  // Area Number
        MadocaIonoArea& area = re_.area[anum];
        area.sid = static_cast<int>(mp::getbitu(ch.buff, i, 1)); i += 1;        // Shape ID
        if (area.sid == 0) {
            const int ref0 = mp::getbits(ch.buff, i, 11); i += 11;  // Reference latitude
            const int ref1 = static_cast<int>(mp::getbitu(ch.buff, i, 12)); i += 12;  // Reference longitude
            const int span0 = static_cast<int>(mp::getbitu(ch.buff, i, 8)); i += 8;   // Latitude span
            const int span1 = static_cast<int>(mp::getbitu(ch.buff, i, 8)); i += 8;   // Longitude span
            area.ref[0] = ref0 * 0.1;
            area.ref[1] = ref1 * 0.1;
            area.span[0] = span0 * 0.1;
            area.span[1] = span1 * 0.1;
        } else {
            const int ref0 = mp::getbits(ch.buff, i, 15); i += 15;  // Reference latitude
            const int ref1 = static_cast<int>(mp::getbitu(ch.buff, i, 16)); i += 16;  // Reference longitude
            const int span0 = static_cast<int>(mp::getbitu(ch.buff, i, 8)); i += 8;   // Effective range
            area.ref[0] = ref0 * 0.01;
            area.ref[1] = ref1 * 0.01;
            area.span[0] = span0 * 10.0;
            area.span[1] = 0.0;
        }
    }
    *imax = i + corlen;
    return i;
}

int MadocaL6dDecoder::decodeCorrection(ChannelState& ch, int i, int apply) {
    const int ep = static_cast<int>(mp::getbitu(ch.buff, i, 12)); i += 12;  // GNSS Hourly Epoch Time 1s
    i += 4;  // SSR Update Interval
    i += 1;  // Multiple Message Indicator
    const int iod = static_cast<int>(mp::getbitu(ch.buff, i, 4)); i += 4;   // IOD SSR
    i += 8;  // STEC Region ID
    const int anum = static_cast<int>(mp::getbitu(ch.buff, i, 5)); i += 5;  // STEC Area Number
    const int type = static_cast<int>(mp::getbitu(ch.buff, i, 2)); i += 2;  // STEC Correction Type

    int ep_adj = ep;
    if (ch.tow0 < 0) {
        apply = 0;  // unprocessed coverage
    } else {
        if (ch.ep0 > ep_adj) ep_adj += 3600;
        adjweek(&ch.gt, ch.tow0 + ep_adj);
        if (ch.iod != iod) {
            apply = 0;  // iod mismatch
        }
    }

    int ns[kMaxIonoSys];
    for (int j = 0; j < kMaxIonoSys; ++j) {
        ns[j] = static_cast<int>(mp::getbitu(ch.buff, i, 5)); i += 5;  // No. of Satellites
    }

    for (int j = 0; j < kMaxIonoSys; ++j) {
        for (int k = 0; k < ns[j]; ++k) {
            int coef[6] = {};
            const int id = static_cast<int>(mp::getbitu(ch.buff, i, 6)); i += 6;   // GNSS Satellite ID
            const int sqi = static_cast<int>(mp::getbitu(ch.buff, i, 6)); i += 6;  // SSR STEC Quality Indicator
            coef[0] = mp::getbits(ch.buff, i, 14); i += 14;  // C00
            if (type > 0) {
                coef[1] = mp::getbits(ch.buff, i, 12); i += 12;  // C01
                coef[2] = mp::getbits(ch.buff, i, 12); i += 12;  // C10
            }
            if (type > 1) {
                coef[3] = mp::getbits(ch.buff, i, 10); i += 10;  // C11
            }
            if (type > 2) {
                coef[4] = mp::getbits(ch.buff, i, 8); i += 8;  // C02
                coef[5] = mp::getbits(ch.buff, i, 8); i += 8;  // C20
            }

            const int offp = (j == kSysQzsId) ? 192 : 0;
            const int sat = mp::satno(gnssid2sys(j), offp + id);
            if (sat == 0) continue;  // unsupported sat
            if (coef[0] == kInvalid14Bit || coef[1] == kInvalid12Bit ||
                coef[2] == kInvalid12Bit || coef[3] == kInvalid10Bit ||
                coef[4] == kInvalid8Bit || coef[5] == kInvalid8Bit) {
                continue;  // invalid value
            }

            if (apply) {
                MadocaIonoArea& area = re_.area[anum];
                area.type = type;
                MadocaIonoSat& s = area.sat[sat - 1];
                s.t0 = ch.gt;
                s.sqi = sqi;
                s.coef[0] = coef[0] * 0.05;
                s.coef[1] = coef[1] * 0.02;
                s.coef[2] = coef[2] * 0.02;
                s.coef[3] = coef[3] * 0.02;
                s.coef[4] = coef[4] * 0.005;
                s.coef[5] = coef[5] * 0.005;
            }
        }
    }
    if (apply) re_.area[anum].avalid = 1;
    return i;
}

int MadocaL6dDecoder::decodeL6dMessage() {
    int i = 0;
    const std::uint32_t preamb = mp::getbitu(msg_buff_, i, 32); i += 32;
    const unsigned prn = mp::getbitu(msg_buff_, i, 8); i += 8;
    // The 8-bit "L6 Msg. Type ID" decomposes into vid(3)+mgfid(2)+csid(1)+
    // anme(1)+si(1); MADOCALIB reads it without advancing i, then re-reads the
    // sub-fields from the same position.
    const unsigned vid = mp::getbitu(msg_buff_, i, 3); i += 3;    // Vendor ID
    const unsigned mgfid = mp::getbitu(msg_buff_, i, 2); i += 2;  // Msg.Gen.Facility ID
    i += 1;  // Correction Service ID
    i += 1;  // Applicable Nav. Msg. Ext.
    const unsigned si = mp::getbitu(msg_buff_, i, 1); i += 1;     // Subframe Indicator
    const unsigned alert = mp::getbitu(msg_buff_, i, 1); i += 1;  // Alert Flag

    int j;
    switch (prn) {  // ref [1] Table 3-1
        case 200: j = 0; break;
        case 201: j = 1; break;
        case 197: j = 2; break;  // for test
        default: return -1;
    }

    if (preamb != kPreamble) return -1;
    if (alert) return -1;     // alert-ignore option not modeled
    if (vid != 2) return 0;   // unsupported vendor ID

    ChannelState& ch = channels_[j];

    if (si == 1) ch.maxframe = 0;  // Subframe Indicator: First Data

    // Frame recognition.
    if (ch.maxframe <= 0) {
        const int mn = static_cast<int>(mp::getbitu(msg_buff_, i, 12));  // not i++
        if (mn == kMtCoverage) {
            const int corlen = static_cast<int>(mp::getbitu(msg_buff_, i + 54, 16));
            const int narea = static_cast<int>(mp::getbitu(msg_buff_, i + 70, 5));
            ch.maxframe = (75 + narea * 45 + corlen) / kL6DataBitLen + 1;
            ch.frame = 0;
            ch.ibuff = 0;
        } else {
            return 0;
        }
    }

    // Save data part (1695 bits) into the assembled buffer.
    ch.frame++;
    int ibuff;
    int n;
    std::uint8_t trailing_mask;
    switch (ch.frame % 8) {
        case 1: ibuff = 0    + ch.frame / 8; n = 8; trailing_mask = 0xFE; break;
        case 2: ibuff = 211  + ch.frame / 8; n = 1; trailing_mask = 0xFC; break;
        case 3: ibuff = 423  + ch.frame / 8; n = 2; trailing_mask = 0xF8; break;
        case 4: ibuff = 635  + ch.frame / 8; n = 3; trailing_mask = 0xF0; break;
        case 5: ibuff = 847  + ch.frame / 8; n = 4; trailing_mask = 0xE0; break;
        case 6: ibuff = 1059 + ch.frame / 8; n = 5; trailing_mask = 0xC0; break;
        case 7: ibuff = 1271 + ch.frame / 8; n = 6; trailing_mask = 0x80; break;
        default:ibuff = 1483 + ch.frame / 8; n = 7; trailing_mask = 0xFF; break;
    }
    ch.buff[ibuff++] |= static_cast<std::uint8_t>(mp::getbitu(msg_buff_, i, n));
    i += n;
    while (i < kL6HeadBitLen + kL6DataBitLen) {
        ch.buff[ibuff++] = static_cast<std::uint8_t>(mp::getbitu(msg_buff_, i, 8));
        i += 8;
    }
    ch.buff[ibuff - 1] &= trailing_mask;

    if (ch.frame != ch.maxframe) return 0;  // no message yet

    int ret = 10;  // input MADOCA-PPP iono. messages
    i = 0;
    int imax = 0;
    while (i < kL6DataBitLen * ch.maxframe) {
        const int mn = static_cast<int>(mp::getbitu(ch.buff, i, 12)); i += 12;  // Message Number
        i += 4;  // Message Sub Type ID (skip)
        if (mn == kMtCoverage) {
            if (ch.mgfid != static_cast<int>(mgfid)) {
                ch.mgfid = static_cast<int>(mgfid);
            }
            i = decodeCoverage(ch, i, &imax);
        } else if (mn == kMtCorrection) {
            int apply = 1;
            // ref[1] 4.2.1(3): facility IDs 0 and 2 are equivalent, 1 and 3.
            if ((ch.mgfid & 0x1) != (static_cast<int>(mgfid) & 0x1)) {
                apply = 0;
            }
            i = decodeCorrection(ch, i, apply);
        } else {
            ret = 0;
            break;
        }
        if (i >= imax) break;
    }
    ch.maxframe = 0;
    return ret;
}

int MadocaL6dDecoder::inputByte(std::uint8_t data) {
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
    return decodeL6dMessage();
}

void MadocaIonoStore::update(int rid, const MadocaIonoRegion& region) {
    if (rid < 0 || rid >= kMaxRid) {
        return;
    }
    re_[rid] = region;  // mirror navs.pppiono.re[rid] = mdcl6d.re
}

void MadocaIonoStore::reset() {
    re_.clear();
}

const MadocaIonoArea* MadocaIonoStore::selectArea(const double rr[3],
                                                  const double ll[2], int* rid,
                                                  int* anum) const {
    // Mirror mdciono.c miono_sel_area: choose the valid, non-alerting area whose
    // reference point is nearest (in ECEF km) and that contains the receiver.
    const MadocaIonoArea* best = nullptr;
    int min_i = -1, min_j = 0;
    double min_dist = 1E5;
    for (const auto& [region_id, re] : re_) {  // ascending rid, like for i=0..255
        if (!re.rvalid) continue;
        if (re.ralert) continue;
        for (int j = 0; j < MadocaIonoRegion::kMaxArea; ++j) {
            const MadocaIonoArea& a = re.area[j];
            if (!a.avalid) continue;
            const double ref_llh[3] = {a.ref[0] * kD2R, a.ref[1] * kD2R, 0.0};
            double ref_ecef[3];
            pos2ecef(ref_llh, ref_ecef);
            const double diff[3] = {rr[0] - ref_ecef[0], rr[1] - ref_ecef[1],
                                    rr[2] - ref_ecef[2]};
            const double dist = norm3(diff) / 1E3;  // m -> km
            if (dist > min_dist) continue;
            if (a.sid == 0) {  // rectangle
                if (ll[0] < a.ref[0] - a.span[0] || ll[0] > a.ref[0] + a.span[0] ||
                    ll[1] < a.ref[1] - a.span[1] || ll[1] > a.ref[1] + a.span[1]) {
                    continue;
                }
            } else {  // circle
                if (dist > a.span[0]) continue;
            }
            min_i = region_id;
            min_j = j;
            min_dist = dist;
            best = &a;
        }
    }
    if (min_i == -1) {
        return nullptr;
    }
    *rid = min_i;
    *anum = min_j;
    return best;
}

int MadocaIonoStore::getCorr(const double rr[3], const MadocaGtime& decode_time,
                             MadocaIonoCorr& out, const double* gloFreqHz) const {
    // Mirror mdciono.c miono_get_corr.
    if (decode_time.time == 0) {
        return 0;  // _miono[0].gt.time gate: nothing decoded yet
    }
    if (rr[0] == 0.0 && rr[1] == 0.0 && rr[2] == 0.0) {
        return 0;
    }

    double pos[3];
    ecef2pos(rr, pos);
    const double latlon[2] = {pos[0] * kR2D, pos[1] * kR2D};

    int rid = -1, anum = 0;
    const MadocaIonoArea* area = selectArea(rr, latlon, &rid, &anum);
    if (area == nullptr) {
        return 0;
    }
    out.rid = rid;
    out.anum = anum;

    for (int i = 0; i < MadocaIonoCorr::kMaxSat; ++i) {
        if (area->sat[i].t0.time == 0) {
            out.t0[i].time = 0;  // leave dly/std at default for absent satellites
            continue;
        }
        out.t0[i] = area->sat[i].t0;
        out.dly[i] = ionoDelay(i + 1, *area, latlon, gloFreqHz);
        out.std[i] = stecUraStd(area->sat[i].sqi);
    }
    return 1;
}

}  // namespace libgnss::io
