#include "../../include/libgnss++/io/ubx_nav_decode.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstring>
#include <cstddef>
#include <utility>

#include "rtcm_internal.hpp"

// Broadcast navigation decoding from UBX-RXM-SFRBX.  Bit layouts, scale factors
// and the consistency checks are a port of RTKLIB 2.4.3 b34 (src/rcv/ublox.c
// decode_nav/decode_enav/decode_cnav/decode_gnav and src/rcvraw.c
// decode_frame_eph/decode_frame_ion/decode_gal_inav_eph/decode_bds_d1_eph/
// decode_bds_d2_eph/decode_glostr_eph/test_glostr); the references are
// IS-GPS-200, IS-QZSS-PN, Galileo OS SIS ICD, BDS-SIS-ICD-B1I and GLONASS ICD 5.1.

namespace libgnss {
namespace io {

namespace {

constexpr double kPi = 3.14159265358979323846;  // semi-circle -> radian
constexpr double kP2_5 = 3.12500000000000000000e-02;
constexpr double kP2_6 = 1.56250000000000000000e-02;
constexpr double kP2_11 = 4.88281250000000000000e-04;
constexpr double kP2_19 = 1.90734863281250000000e-06;
constexpr double kP2_20 = 9.53674316406250000000e-07;
constexpr double kP2_24 = 5.96046447753906250000e-08;
constexpr double kP2_27 = 7.45058059692382812500e-09;
constexpr double kP2_29 = 1.86264514923095703125e-09;
constexpr double kP2_30 = 9.31322574615478515625e-10;
constexpr double kP2_31 = 4.65661287307739257812e-10;
constexpr double kP2_32 = 2.32830643653869628906e-10;
constexpr double kP2_33 = 1.16415321826934814453e-10;
constexpr double kP2_34 = 5.82076609134674072266e-11;
constexpr double kP2_40 = 9.09494701772928237915e-13;
constexpr double kP2_43 = 1.13686837721616029739e-13;
constexpr double kP2_46 = 1.42108547152020037174e-14;
constexpr double kP2_50 = 8.88178419700125232339e-16;
constexpr double kP2_55 = 2.77555756156289135106e-17;
constexpr double kP2_59 = 1.73472347597680709441e-18;
constexpr double kP2_66 = 1.35525271560688054251e-20;
constexpr double kP2P11 = 2048.0;
constexpr double kP2P14 = 16384.0;
constexpr double kP2P16 = 65536.0;
constexpr double kHalfWeek = 302400.0;
constexpr int kBdtWeekOffset = 1356;  // BDT week 0 = GPS week 1356
constexpr double kBdtToGpst = 14.0;

// --- bit helpers (MSB first, as RTKLIB getbitu/getbits/setbitu) ---------------

uint32_t getbitu(const uint8_t* buff, int pos, int len) {
    uint32_t bits = 0;
    for (int i = pos; i < pos + len; ++i) {
        bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1U);
    }
    return bits;
}

int32_t getbits(const uint8_t* buff, int pos, int len) {
    const uint32_t bits = getbitu(buff, pos, len);
    if (len <= 0 || 32 <= len || !(bits & (1U << (len - 1)))) {
        return static_cast<int32_t>(bits);
    }
    return static_cast<int32_t>(bits | (~0U << len));
}

void setbitu(uint8_t* buff, int pos, int len, uint32_t data) {
    if (len <= 0 || 32 < len) {
        return;
    }
    uint32_t mask = 1U << (len - 1);
    for (int i = pos; i < pos + len; ++i, mask >>= 1) {
        if (data & mask) {
            buff[i / 8] |= static_cast<uint8_t>(1U << (7 - i % 8));
        } else {
            buff[i / 8] &= static_cast<uint8_t>(~(1U << (7 - i % 8)));
        }
    }
}

uint32_t getbitu2(const uint8_t* buff, int p1, int l1, int p2, int l2) {
    return (getbitu(buff, p1, l1) << l2) + getbitu(buff, p2, l2);
}

int32_t getbits2(const uint8_t* buff, int p1, int l1, int p2, int l2) {
    if (getbitu(buff, p1, 1)) {
        return static_cast<int32_t>((static_cast<uint32_t>(getbits(buff, p1, l1)) << l2) +
                                    getbitu(buff, p2, l2));
    }
    return static_cast<int32_t>(getbitu2(buff, p1, l1, p2, l2));
}

uint32_t getbitu3(const uint8_t* buff, int p1, int l1, int p2, int l2, int p3, int l3) {
    return (getbitu(buff, p1, l1) << (l2 + l3)) + (getbitu(buff, p2, l2) << l3) +
           getbitu(buff, p3, l3);
}

int32_t getbits3(const uint8_t* buff, int p1, int l1, int p2, int l2, int p3, int l3) {
    if (getbitu(buff, p1, 1)) {
        return static_cast<int32_t>((static_cast<uint32_t>(getbits(buff, p1, l1)) << (l2 + l3)) +
                                    (getbitu(buff, p2, l2) << l3) + getbitu(buff, p3, l3));
    }
    return static_cast<int32_t>(getbitu3(buff, p1, l1, p2, l2, p3, l3));
}

uint32_t mergeTwoU(uint32_t a, uint32_t b, int n) {
    return (a << n) + b;
}

int32_t mergeTwoS(int32_t a, uint32_t b, int n) {
    return static_cast<int32_t>((static_cast<uint32_t>(a) << n) + b);
}

/// Sign-magnitude field (GLONASS).
double getbitg(const uint8_t* buff, int pos, int len) {
    const double value = static_cast<double>(getbitu(buff, pos + 1, len - 1));
    return getbitu(buff, pos, 1) ? -value : value;
}

/// The toc/toe/tof week nearest to `reference` (week roll-over safe).
GNSSTime nearestTo(const GNSSTime& reference, int week, double tow) {
    GNSSTime t(week, tow);
    const double diff = t - reference;
    if (diff > kHalfWeek) {
        t = GNSSTime(week - 1, tow);
    } else if (diff < -kHalfWeek) {
        t = GNSSTime(week + 1, tow);
    }
    return t;
}

double beidouUraMeters(uint8_t index) {
    // BDS-SIS-ICD-B1I-3.0 table 5-8: N < 6 -> 2^(N/2+1) m, 6 <= N < 15 -> 2^(N-2) m.
    static constexpr double kTable[16] = {2.0,  2.8,   4.0,   5.7,   8.0,    11.3,
                                          16.0, 32.0,  64.0,  128.0, 256.0,  512.0,
                                          1024.0, 2048.0, 4096.0, 8192.0};
    return kTable[std::min<uint8_t>(index, 15)];
}

}  // namespace

namespace ubx_utils {

uint32_t crc24q(const uint8_t* data, size_t size) {
    uint32_t crc = 0;
    for (size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (int bit = 0; bit < 8; ++bit) {
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x1864CFBU;
            }
        }
    }
    return crc & 0xFFFFFFU;
}

bool testGlonassString(const uint8_t* buff) {
    static const uint8_t kXor8[256] = {
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};
    static const uint8_t kMask[8][12] = {
        {0x55,0x55,0x5A,0xAA,0xAA,0xAA,0xB5,0x55,0x6A,0xD8,0x08},
        {0x66,0x66,0x6C,0xCC,0xCC,0xCC,0xD9,0x99,0xB3,0x68,0x10},
        {0x87,0x87,0x8F,0x0F,0x0F,0x0F,0x1E,0x1E,0x3C,0x70,0x20},
        {0x07,0xF8,0x0F,0xF0,0x0F,0xF0,0x1F,0xE0,0x3F,0x80,0x40},
        {0xF8,0x00,0x0F,0xFF,0xF0,0x00,0x1F,0xFF,0xC0,0x00,0x80},
        {0x00,0x00,0x0F,0xFF,0xFF,0xFF,0xE0,0x00,0x00,0x01,0x00},
        {0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00},
        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF8}};
    uint8_t cs = 0;
    int n = 0;
    for (int i = 0; i < 8; ++i) {
        cs = 0;
        for (int j = 0; j < 11; ++j) {
            cs ^= kXor8[buff[j] & kMask[i][j]];
        }
        if (cs) {
            ++n;
        }
    }
    return n == 0 || (n == 2 && cs);
}

}  // namespace ubx_utils

// ------------------------------------------------------------------------------

void UBXNavDecoder::clear() {
    reference_ = GNSSTime();
    has_reference_ = false;
    stats_ = UBXNavDecodeStats();
    klobuchar_ = IonosphereModel();
    gps_.clear();
    galileo_.clear();
    beidou_.clear();
    glonass_.clear();
    last_emitted_.clear();
    message_seq_ = 0;
    decoded_ = nullptr;
}

void UBXNavDecoder::setReferenceTime(const GNSSTime& gpst) {
    reference_ = gpst;
    has_reference_ = true;
}

namespace {

// Ephemeris::Ephemeris() leaves i_dot, sv_health and sv_accuracy uninitialised.  Not every system fills them
// (GLONASS has no i_dot or sv_accuracy), and confirm-twice compares records bit for bit, so start from zeros.
Ephemeris freshEphemeris() {
    Ephemeris eph;
    eph.i_dot = 0.0;
    eph.sv_health = 0.0;
    eph.sv_accuracy = 0.0;
    return eph;
}

}  // namespace

UBXNavDecoder::Stamp UBXNavDecoder::nowStamp() const {
    Stamp stamp;
    stamp.seq = message_seq_;
    stamp.gpst_s = has_reference_ ? reference_.week * 604800.0 + reference_.tow
                                  : std::numeric_limits<double>::quiet_NaN();
    return stamp;
}

UBXNavReception UBXNavDecoder::receptionOf(const std::vector<Stamp>& used) {
    UBXNavReception rx;
    rx.first_seq = std::numeric_limits<uint64_t>::max();
    rx.first_gpst_s = std::numeric_limits<double>::quiet_NaN();
    rx.last_gpst_s = std::numeric_limits<double>::quiet_NaN();
    for (const Stamp& stamp : used) {
        if (stamp.seq < rx.first_seq) {
            rx.first_seq = stamp.seq;
            rx.first_gpst_s = stamp.gpst_s;
        }
        if (stamp.seq >= rx.last_seq) {
            rx.last_seq = stamp.seq;
            rx.last_gpst_s = stamp.gpst_s;
        }
    }
    return rx;
}

bool UBXNavDecoder::emit(const Ephemeris& eph, const UBXNavReception& reception,
                         std::vector<Ephemeris>& out) {
    auto& st = systemStats(eph.satellite.system);
    const auto last = last_emitted_.find(eph.satellite);
    const bool duplicate = last != last_emitted_.end() && ubxNavSameIssue(last->second, eph);
    if (decoded_ != nullptr) {
        decoded_->push_back({eph, reception, !duplicate});
    }
    if (duplicate) {
        ++st.duplicates;
        return false;
    }
    last_emitted_[eph.satellite] = eph;
    ++st.ephemerides;
    out.push_back(eph);
    return true;
}

size_t UBXNavDecoder::pushSfrbx(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out,
                                std::vector<UBXNavDecoded>* decoded) {
    ++message_seq_;
    decoded_ = decoded;
    size_t emitted = 0;
    switch (sfrbx.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            emitted = pushGps(sfrbx, out);
            break;
        case GNSSSystem::Galileo:
            emitted = pushGalileo(sfrbx, out);
            break;
        case GNSSSystem::BeiDou:
            emitted = pushBeiDou(sfrbx, out);
            break;
        case GNSSSystem::GLONASS:
            emitted = pushGlonass(sfrbx, out);
            break;
        default:
            break;
    }
    decoded_ = nullptr;
    return emitted;
}

// --- GPS / QZSS L1 C/A LNAV -----------------------------------------------------

size_t UBXNavDecoder::pushGps(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out) {
    auto& st = systemStats(sfrbx.system);
    const bool qzss = sfrbx.system == GNSSSystem::QZSS;
    // L1 C/A only; CNAV (L2C/L5) and QZSS L1S have other layouts.
    if (sfrbx.signal_id != 0 || sfrbx.words.size() < 10 || sfrbx.sv_id == 0 ||
        sfrbx.sv_id > (qzss ? 10 : 32)) {
        ++st.skipped_signal;
        return 0;
    }
    ++st.subframes;

    // 24 data bits per word, parity removed (UBX words hold the 30-bit word in bits 29..0).
    uint8_t buff[30] = {0};
    for (int i = 0; i < 10; ++i) {
        setbitu(buff, 24 * i, 24, sfrbx.words[static_cast<size_t>(i)] >> 6);
    }
    const int id = static_cast<int>(getbitu(buff, 43, 3));
    if (id < 1 || id > 5) {
        ++st.dropped["subframe_id"];
        return 0;
    }

    GpsState& state = gps_[{sfrbx.system, sfrbx.sv_id, 0}];
    std::copy(buff, buff + 30, state.subframes.begin() + (id - 1) * 30);
    state.present[static_cast<size_t>(id - 1)] = true;
    state.stamp[static_cast<size_t>(id - 1)] = nowStamp();

    if (id == 4) {
        // Subframe 4 page 18 (SV id 56): ionospheric model (GPS only).
        if (!qzss && getbitu(buff, 50, 6) == 56) {
            int i = 56;
            IonosphereModel model;
            model.alpha[0] = getbits(buff, i, 8) * kP2_30; i += 8;
            model.alpha[1] = getbits(buff, i, 8) * kP2_27; i += 8;
            model.alpha[2] = getbits(buff, i, 8) * kP2_24; i += 8;
            model.alpha[3] = getbits(buff, i, 8) * kP2_24; i += 8;
            model.beta[0] = getbits(buff, i, 8) * kP2P11; i += 8;
            model.beta[1] = getbits(buff, i, 8) * kP2P14; i += 8;
            model.beta[2] = getbits(buff, i, 8) * kP2P16; i += 8;
            model.beta[3] = getbits(buff, i, 8) * kP2P16;
            model.valid = true;
            klobuchar_ = model;
            ++stats_.klobuchar_sets;
        }
        state.present[3] = false;
        return 0;
    }
    if (id == 5) {
        state.present[4] = false;
        return 0;
    }
    if (id != 3) {
        return 0;
    }

    // Ephemeris: subframes 1-3.
    if (!state.present[0] || !state.present[1]) {
        ++st.dropped["subframe_id"];
        return 0;
    }
    if (!has_reference_) {
        ++st.dropped["no_time_reference"];
        return 0;
    }
    const uint8_t* b = state.subframes.data();
    Ephemeris eph = freshEphemeris();
    int i = 240 * 0 + 24;  // subframe 1
    const double tow1 = getbitu(b, i, 17) * 6.0; i += 17 + 2;
    const int id1 = static_cast<int>(getbitu(b, i, 3)); i += 3 + 2;
    const int week10 = static_cast<int>(getbitu(b, i, 10)); i += 10;
    const int l2_code = static_cast<int>(getbitu(b, i, 2)); i += 2;  // code on L2
    const int sva = static_cast<int>(getbitu(b, i, 4)); i += 4;
    const int svh = static_cast<int>(getbitu(b, i, 6)); i += 6;
    const int iodc0 = static_cast<int>(getbitu(b, i, 2)); i += 2;
    i += 1 + 87;  // L2 P data flag, reserved
    const int tgd = getbits(b, i, 8); i += 8;
    const int iodc1 = static_cast<int>(getbitu(b, i, 8)); i += 8;
    const double toc = getbitu(b, i, 16) * 16.0; i += 16;
    eph.af2 = getbits(b, i, 8) * kP2_55; i += 8;
    eph.af1 = getbits(b, i, 16) * kP2_43; i += 16;
    eph.af0 = getbits(b, i, 22) * kP2_31;

    i = 240 * 1 + 24;  // subframe 2
    i += 17 + 2;  // HOW time of week
    const int id2 = static_cast<int>(getbitu(b, i, 3)); i += 3 + 2;
    const int iode = static_cast<int>(getbitu(b, i, 8)); i += 8;
    eph.crs = getbits(b, i, 16) * kP2_5; i += 16;
    eph.delta_n = getbits(b, i, 16) * kP2_43 * kPi; i += 16;
    eph.m0 = getbits(b, i, 32) * kP2_31 * kPi; i += 32;
    eph.cuc = getbits(b, i, 16) * kP2_29; i += 16;
    eph.e = getbitu(b, i, 32) * kP2_33; i += 32;
    eph.cus = getbits(b, i, 16) * kP2_29; i += 16;
    eph.sqrt_a = getbitu(b, i, 32) * kP2_19; i += 32;
    eph.toes = getbitu(b, i, 16) * 16.0;

    i = 240 * 2 + 24;  // subframe 3
    i += 17 + 2;  // HOW time of week
    const int id3 = static_cast<int>(getbitu(b, i, 3)); i += 3 + 2;
    eph.cic = getbits(b, i, 16) * kP2_29; i += 16;
    eph.omega0 = getbits(b, i, 32) * kP2_31 * kPi; i += 32;
    eph.cis = getbits(b, i, 16) * kP2_29; i += 16;
    eph.i0 = getbits(b, i, 32) * kP2_31 * kPi; i += 32;
    eph.crc = getbits(b, i, 16) * kP2_5; i += 16;
    eph.omega = getbits(b, i, 32) * kP2_31 * kPi; i += 32;
    eph.omega_dot = getbits(b, i, 24) * kP2_43 * kPi; i += 24;
    const int iode3 = static_cast<int>(getbitu(b, i, 8)); i += 8;
    eph.idot = getbits(b, i, 14) * kP2_43 * kPi;

    if (id1 != 1 || id2 != 2 || id3 != 3) {
        ++st.dropped["subframe_id"];
        return 0;
    }
    const int iodc = (iodc0 << 8) + iodc1;
    if (iode != iode3 || iode != (iodc & 0xFF)) {
        ++st.dropped["iode_mismatch"];
        return 0;
    }

    eph.satellite = SatelliteId(sfrbx.system, sfrbx.sv_id);
    eph.i_dot = eph.idot;
    eph.iode = static_cast<uint16_t>(iode);
    eph.iodc = static_cast<uint16_t>(iodc);
    eph.tgd = (tgd == -128) ? 0.0 : tgd * kP2_31;
    eph.data_source_code = l2_code;  // the RINEX reader keeps the "codes on L2 channel" word here
    eph.ura = static_cast<uint8_t>(sva);
    eph.sv_accuracy = rtcm_internal::uraMetersFromIndex(eph.ura);
    eph.health = static_cast<uint8_t>(svh);
    eph.sv_health = static_cast<double>(svh);

    int week = rtcm_internal::adjustGpsWeek(static_cast<uint16_t>(week10), reference_.week);
    eph.tof = GNSSTime(week, tow1);
    if (eph.toes < tow1 - kHalfWeek) {
        ++week;
    } else if (eph.toes > tow1 + kHalfWeek) {
        --week;
    }
    eph.week = static_cast<uint16_t>(week);
    eph.toe = GNSSTime(week, eph.toes);
    eph.toc = nearestTo(eph.toe, week, toc);
    eph.valid = true;
    const UBXNavReception reception = receptionOf({state.stamp[0], state.stamp[1], state.stamp[2]});
    return emit(eph, reception, out) ? 1 : 0;
}

// --- Galileo E1-B I/NAV ---------------------------------------------------------

size_t UBXNavDecoder::pushGalileo(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out) {
    auto& st = systemStats(GNSSSystem::Galileo);
    // E1-B only: sigId 1 (0 on receivers before protocol 27).  E5b-I (5), E5a F/NAV (3/4) are out of scope.
    if ((sfrbx.signal_id != 0 && sfrbx.signal_id != 1) || sfrbx.words.size() < 8 ||
        sfrbx.sv_id == 0 || sfrbx.sv_id > 36) {
        ++st.skipped_signal;
        return 0;
    }
    ++st.subframes;

    uint8_t buff[32] = {0};
    for (int i = 0; i < 8; ++i) {
        setbitu(buff, 32 * i, 32, sfrbx.words[static_cast<size_t>(i)]);
    }
    const uint32_t part1 = getbitu(buff, 0, 1);
    const uint32_t page1 = getbitu(buff, 1, 1);
    const uint32_t part2 = getbitu(buff, 128, 1);
    const uint32_t page2 = getbitu(buff, 129, 1);
    if (part1 != 0 || part2 != 1) {
        ++st.dropped["even_odd"];
        return 0;
    }
    if (page1 == 1 || page2 == 1) {
        return 0;  // alert page
    }

    // CRC-24Q over 4 pad bits + 114 even-page bits + 82 odd-page bits.
    uint8_t crc_buff[26] = {0};
    for (int i = 0, j = 4; i < 15; ++i, j += 8) {
        setbitu(crc_buff, j, 8, getbitu(buff, i * 8, 8));
    }
    for (int i = 0, j = 118; i < 11; ++i, j += 8) {
        setbitu(crc_buff, j, 8, getbitu(buff, i * 8 + 128, 8));
    }
    if (ubx_utils::crc24q(crc_buff, 25) != getbitu(buff, 128 + 82, 24)) {
        ++st.dropped["crc"];
        return 0;
    }
    const int type = static_cast<int>(getbitu(buff, 2, 6));
    if (type > 6) {
        return 0;
    }

    GalileoState& state = galileo_[{sfrbx.system, sfrbx.sv_id, 1}];
    // 128-bit word: 112 data bits of the even page + 16 data bits of the odd page.
    uint8_t* slot = state.words.data() + type * 16;
    for (int i = 0, j = 2; i < 14; ++i, j += 8) {
        slot[i] = static_cast<uint8_t>(getbitu(buff, j, 8));
    }
    for (int i = 14, j = 130; i < 16; ++i, j += 8) {
        slot[i] = static_cast<uint8_t>(getbitu(buff, j, 8));
    }
    state.present[static_cast<size_t>(type)] = true;
    state.stamp[static_cast<size_t>(type)] = nowStamp();
    if (type != 5) {
        return 0;
    }
    for (int t = 1; t <= 5; ++t) {
        if (!state.present[static_cast<size_t>(t)]) {
            ++st.dropped["incomplete_set"];
            return 0;
        }
    }

    const uint8_t* b = state.words.data();
    Ephemeris eph = freshEphemeris();
    int type_w[5];
    int iod_nav[4];
    int i = 128;  // word type 1
    type_w[0] = static_cast<int>(getbitu(b, i, 6)); i += 6;
    iod_nav[0] = static_cast<int>(getbitu(b, i, 10)); i += 10;
    eph.toes = getbitu(b, i, 14) * 60.0; i += 14;
    eph.m0 = getbits(b, i, 32) * kP2_31 * kPi; i += 32;
    eph.e = getbitu(b, i, 32) * kP2_33; i += 32;
    eph.sqrt_a = getbitu(b, i, 32) * kP2_19;

    i = 128 * 2;  // word type 2
    type_w[1] = static_cast<int>(getbitu(b, i, 6)); i += 6;
    iod_nav[1] = static_cast<int>(getbitu(b, i, 10)); i += 10;
    eph.omega0 = getbits(b, i, 32) * kP2_31 * kPi; i += 32;
    eph.i0 = getbits(b, i, 32) * kP2_31 * kPi; i += 32;
    eph.omega = getbits(b, i, 32) * kP2_31 * kPi; i += 32;
    eph.idot = getbits(b, i, 14) * kP2_43 * kPi;

    i = 128 * 3;  // word type 3
    type_w[2] = static_cast<int>(getbitu(b, i, 6)); i += 6;
    iod_nav[2] = static_cast<int>(getbitu(b, i, 10)); i += 10;
    eph.omega_dot = getbits(b, i, 24) * kP2_43 * kPi; i += 24;
    eph.delta_n = getbits(b, i, 16) * kP2_43 * kPi; i += 16;
    eph.cuc = getbits(b, i, 16) * kP2_29; i += 16;
    eph.cus = getbits(b, i, 16) * kP2_29; i += 16;
    eph.crc = getbits(b, i, 16) * kP2_5; i += 16;
    eph.crs = getbits(b, i, 16) * kP2_5; i += 16;
    const int sisa = static_cast<int>(getbitu(b, i, 8));

    i = 128 * 4;  // word type 4
    type_w[3] = static_cast<int>(getbitu(b, i, 6)); i += 6;
    iod_nav[3] = static_cast<int>(getbitu(b, i, 10)); i += 10;
    const int svid = static_cast<int>(getbitu(b, i, 6)); i += 6;
    eph.cic = getbits(b, i, 16) * kP2_29; i += 16;
    eph.cis = getbits(b, i, 16) * kP2_29; i += 16;
    const double toc = getbitu(b, i, 14) * 60.0; i += 14;
    eph.af0 = getbits(b, i, 31) * kP2_34; i += 31;
    eph.af1 = getbits(b, i, 21) * kP2_46; i += 21;
    eph.af2 = getbits(b, i, 6) * kP2_59;

    i = 128 * 5;  // word type 5
    type_w[4] = static_cast<int>(getbitu(b, i, 6)); i += 6 + 11 + 11 + 14 + 5;
    eph.tgd = getbits(b, i, 10) * kP2_32; i += 10;            // BGD E5a/E1
    eph.tgd_secondary = getbits(b, i, 10) * kP2_32; i += 10;  // BGD E5b/E1
    const int e5b_hs = static_cast<int>(getbitu(b, i, 2)); i += 2;
    const int e1b_hs = static_cast<int>(getbitu(b, i, 2)); i += 2;
    const int e5b_dvs = static_cast<int>(getbitu(b, i, 1)); i += 1;
    const int e1b_dvs = static_cast<int>(getbitu(b, i, 1)); i += 1;
    int week = static_cast<int>(getbitu(b, i, 12)); i += 12;  // GST week
    const double tow = getbitu(b, i, 20);

    if (type_w[0] != 1 || type_w[1] != 2 || type_w[2] != 3 || type_w[3] != 4 || type_w[4] != 5) {
        ++st.dropped["word_type"];
        return 0;
    }
    if (iod_nav[0] != iod_nav[1] || iod_nav[0] != iod_nav[2] || iod_nav[0] != iod_nav[3]) {
        ++st.dropped["iod_nav_mismatch"];
        return 0;
    }
    if (svid != sfrbx.sv_id) {
        ++st.dropped["satellite_mismatch"];
        return 0;
    }

    eph.satellite = SatelliteId(GNSSSystem::Galileo, sfrbx.sv_id);
    eph.i_dot = eph.idot;
    eph.iode = static_cast<uint16_t>(iod_nav[0]);
    eph.iodc = eph.iode;
    eph.ura = static_cast<uint8_t>(sisa);
    eph.sv_accuracy = galileoSisaMeters(eph.ura);
    const int svh = (e5b_hs << 7) | (e5b_dvs << 6) | (e1b_hs << 1) | e1b_dvs;
    eph.health = static_cast<uint16_t>(svh);  // full width: E5b HS occupies bits 7-8 (as #621 / the RINEX reader)
    eph.sv_health = static_cast<double>(svh);
    eph.data_source_code = galileo_data_source::kInavE1B | galileo_data_source::kClockE5bE1;
    eph.navigation_message_type = NavigationMessageType::INAV;

    // Galileo week in the library/RINEX convention is the GST week + 1024.
    eph.tof = GNSSTime(week + 1024, tow);
    const double tt = eph.toes - tow;
    if (tt > kHalfWeek) {
        --week;
    } else if (tt < -kHalfWeek) {
        ++week;
    }
    eph.week = static_cast<uint16_t>(week + 1024);
    eph.toe = GNSSTime(week + 1024, eph.toes);
    eph.toc = nearestTo(eph.toe, week + 1024, toc);
    eph.valid = true;
    // Word types 1-5 (type 0 and 6 are not used by the record).
    const UBXNavReception reception = receptionOf(
        {state.stamp[1], state.stamp[2], state.stamp[3], state.stamp[4], state.stamp[5]});
    return emit(eph, reception, out) ? 1 : 0;
}

// --- BeiDou D1 / D2 -------------------------------------------------------------

size_t UBXNavDecoder::pushBeiDou(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out) {
    auto& st = systemStats(GNSSSystem::BeiDou);
    if (!ubx_utils::isSfrbxLegacyNavigation(sfrbx) || sfrbx.words.size() < 10 ||
        sfrbx.sv_id == 0 || sfrbx.sv_id > 63) {
        ++st.skipped_signal;
        return 0;
    }
    ++st.subframes;

    uint8_t buff[38] = {0};
    for (int i = 0; i < 10; ++i) {
        setbitu(buff, 30 * i, 30, sfrbx.words[static_cast<size_t>(i)]);
    }
    const int id = static_cast<int>(getbitu(buff, 15, 3));
    if (id < 1 || id > 5) {
        ++st.dropped["subframe_id"];
        return 0;
    }
    const bool geo = ubx_utils::isSfrbxBeiDouD2(sfrbx);
    BeiDouState& state = beidou_[{sfrbx.system, sfrbx.sv_id, static_cast<uint8_t>(geo ? 1 : 0)}];
    Ephemeris eph = freshEphemeris();
    double toc_bds = 0.0;
    double sow1 = 0.0;
    int bdt_week = 0;
    int sva = 0;
    int svh = 0;

    if (!geo) {
        std::copy(buff, buff + 38, state.frames.begin() + (id - 1) * 38);
        state.present[static_cast<size_t>(id - 1)] = true;
        state.stamp[static_cast<size_t>(id - 1)] = nowStamp();
        if (id != 3) {
            return 0;
        }
        if (!state.present[0] || !state.present[1]) {
            ++st.dropped["subframe_id"];
            return 0;
        }
        const uint8_t* b = state.frames.data();
        int i = 8 * 38 * 0;  // subframe 1
        const int frn1 = static_cast<int>(getbitu(b, i + 15, 3));
        const uint32_t s1 = getbitu2(b, i + 18, 8, i + 30, 12);
        svh = static_cast<int>(getbitu(b, i + 42, 1));
        eph.iodc = static_cast<uint16_t>(getbitu(b, i + 43, 5));  // AODC
        sva = static_cast<int>(getbitu(b, i + 48, 4));
        bdt_week = static_cast<int>(getbitu(b, i + 60, 13));
        toc_bds = getbitu2(b, i + 73, 9, i + 90, 8) * 8.0;
        eph.tgd = getbits(b, i + 98, 10) * 0.1 * 1E-9;
        eph.tgd_secondary = getbits2(b, i + 108, 4, i + 120, 6) * 0.1 * 1E-9;
        eph.af2 = getbits(b, i + 214, 11) * kP2_66;
        eph.af0 = getbits2(b, i + 225, 7, i + 240, 17) * kP2_33;
        eph.af1 = getbits2(b, i + 257, 5, i + 270, 17) * kP2_50;
        eph.iode = static_cast<uint16_t>(getbitu(b, i + 287, 5));  // AODE

        i = 8 * 38 * 1;  // subframe 2
        const int frn2 = static_cast<int>(getbitu(b, i + 15, 3));
        const uint32_t s2 = getbitu2(b, i + 18, 8, i + 30, 12);
        eph.delta_n = getbits2(b, i + 42, 10, i + 60, 6) * kP2_43 * kPi;
        eph.cuc = getbits2(b, i + 66, 16, i + 90, 2) * kP2_31;
        eph.m0 = getbits2(b, i + 92, 20, i + 120, 12) * kP2_31 * kPi;
        eph.e = getbitu2(b, i + 132, 10, i + 150, 22) * kP2_33;
        eph.cus = getbits(b, i + 180, 18) * kP2_31;
        eph.crc = getbits2(b, i + 198, 4, i + 210, 14) * kP2_6;
        eph.crs = getbits2(b, i + 224, 8, i + 240, 10) * kP2_6;
        eph.sqrt_a = getbitu2(b, i + 250, 12, i + 270, 20) * kP2_19;
        const uint32_t toe1 = getbitu(b, i + 290, 2);  // TOE 2-MSB

        i = 8 * 38 * 2;  // subframe 3
        const int frn3 = static_cast<int>(getbitu(b, i + 15, 3));
        const uint32_t s3 = getbitu2(b, i + 18, 8, i + 30, 12);
        const uint32_t toe2 = getbitu2(b, i + 42, 10, i + 60, 5);  // TOE 5-LSB
        eph.i0 = getbits2(b, i + 65, 17, i + 90, 15) * kP2_31 * kPi;
        eph.cic = getbits2(b, i + 105, 7, i + 120, 11) * kP2_31;
        eph.omega_dot = getbits2(b, i + 131, 11, i + 150, 13) * kP2_43 * kPi;
        eph.cis = getbits2(b, i + 163, 9, i + 180, 9) * kP2_31;
        eph.idot = getbits2(b, i + 189, 13, i + 210, 1) * kP2_43 * kPi;
        eph.omega0 = getbits2(b, i + 211, 21, i + 240, 11) * kP2_31 * kPi;
        eph.omega = getbits2(b, i + 251, 11, i + 270, 21) * kP2_31 * kPi;
        eph.toes = mergeTwoU(toe1, toe2, 15) * 8.0;
        sow1 = s1;

        if (frn1 != 1 || frn2 != 2 || frn3 != 3) {
            ++st.dropped["subframe_id"];
            return 0;
        }
        if (s2 != s1 + 6 || s3 != s2 + 6) {
            ++st.dropped["sow_sequence"];
            return 0;
        }
        if (toc_bds != eph.toes) {
            ++st.dropped["toc_toe_mismatch"];
            return 0;
        }
    } else {
        // D2 (GEO): subframe 1 pages 1-10.  Page 102 of subframe 5 (UTC) is not stored.
        const int pgn = static_cast<int>(getbitu(buff, 42, 4));
        if (id != 1 || pgn < 1 || pgn > 10) {
            return 0;
        }
        std::copy(buff, buff + 38, state.frames.begin() + (pgn - 1) * 38);
        state.present[static_cast<size_t>(pgn - 1)] = true;
        state.stamp[static_cast<size_t>(pgn - 1)] = nowStamp();
        if (pgn != 10) {
            return 0;
        }
        for (int p : {0, 2, 3, 4, 5, 6, 7, 8, 9}) {
            if (!state.present[static_cast<size_t>(p)]) {
                ++st.dropped["incomplete_set"];
                return 0;
            }
        }
        const uint8_t* b = state.frames.data();
        int i = 8 * 38 * 0;  // page 1
        const int pgn1 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s1 = getbitu2(b, i + 18, 8, i + 30, 12);
        svh = static_cast<int>(getbitu(b, i + 46, 1));
        eph.iodc = static_cast<uint16_t>(getbitu(b, i + 47, 5));  // AODC
        sva = static_cast<int>(getbitu(b, i + 60, 4));
        bdt_week = static_cast<int>(getbitu(b, i + 64, 13));
        toc_bds = getbitu2(b, i + 77, 5, i + 90, 12) * 8.0;
        eph.tgd = getbits(b, i + 102, 10) * 0.1 * 1E-9;
        eph.tgd_secondary = getbits(b, i + 120, 10) * 0.1 * 1E-9;

        i = 8 * 38 * 2;  // page 3
        const int pgn3 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s3 = getbitu2(b, i + 18, 8, i + 30, 12);
        eph.af0 = getbits2(b, i + 100, 12, i + 120, 12) * kP2_33;
        const int f1p3 = getbits(b, i + 132, 4);

        i = 8 * 38 * 3;  // page 4
        const int pgn4 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s4 = getbitu2(b, i + 18, 8, i + 30, 12);
        const uint32_t f1p4 = getbitu2(b, i + 46, 6, i + 60, 12);
        eph.af2 = getbits2(b, i + 72, 10, i + 90, 1) * kP2_66;
        eph.iode = static_cast<uint16_t>(getbitu(b, i + 91, 5));  // AODE
        eph.delta_n = getbits(b, i + 96, 16) * kP2_43 * kPi;
        const int cucp4 = getbits(b, i + 120, 14);

        i = 8 * 38 * 4;  // page 5
        const int pgn5 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s5 = getbitu2(b, i + 18, 8, i + 30, 12);
        const uint32_t cucp5 = getbitu(b, i + 46, 4);
        eph.m0 = getbits3(b, i + 50, 2, i + 60, 22, i + 90, 8) * kP2_31 * kPi;
        eph.cus = getbits2(b, i + 98, 14, i + 120, 4) * kP2_31;
        const int ep5 = getbits(b, i + 124, 10);

        i = 8 * 38 * 5;  // page 6
        const int pgn6 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s6 = getbitu2(b, i + 18, 8, i + 30, 12);
        const uint32_t ep6 = getbitu2(b, i + 46, 6, i + 60, 16);
        eph.sqrt_a = getbitu3(b, i + 76, 6, i + 90, 22, i + 120, 4) * kP2_19;
        const int cicp6 = getbits(b, i + 124, 10);

        i = 8 * 38 * 6;  // page 7
        const int pgn7 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s7 = getbitu2(b, i + 18, 8, i + 30, 12);
        const uint32_t cicp7 = getbitu2(b, i + 46, 6, i + 60, 2);
        eph.cis = getbits(b, i + 62, 18) * kP2_31;
        eph.toes = getbitu2(b, i + 80, 2, i + 90, 15) * 8.0;
        const int i0p7 = getbits2(b, i + 105, 7, i + 120, 14);

        i = 8 * 38 * 7;  // page 8
        const int pgn8 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s8 = getbitu2(b, i + 18, 8, i + 30, 12);
        const uint32_t i0p8 = getbitu2(b, i + 46, 6, i + 60, 5);
        eph.crc = getbits2(b, i + 65, 17, i + 90, 1) * kP2_6;
        eph.crs = getbits(b, i + 91, 18) * kP2_6;
        const int omgdp8 = getbits2(b, i + 109, 3, i + 120, 16);

        i = 8 * 38 * 8;  // page 9
        const int pgn9 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s9 = getbitu2(b, i + 18, 8, i + 30, 12);
        const uint32_t omgdp9 = getbitu(b, i + 46, 5);
        eph.omega0 = getbits3(b, i + 51, 1, i + 60, 22, i + 90, 9) * kP2_31 * kPi;
        const int omgp9 = getbits2(b, i + 99, 13, i + 120, 14);

        i = 8 * 38 * 9;  // page 10
        const int pgn10 = static_cast<int>(getbitu(b, i + 42, 4));
        const uint32_t s10 = getbitu2(b, i + 18, 8, i + 30, 12);
        const uint32_t omgp10 = getbitu(b, i + 46, 5);
        eph.idot = getbits2(b, i + 51, 1, i + 60, 13) * kP2_43 * kPi;

        if (pgn1 != 1 || pgn3 != 3 || pgn4 != 4 || pgn5 != 5 || pgn6 != 6 || pgn7 != 7 ||
            pgn8 != 8 || pgn9 != 9 || pgn10 != 10) {
            ++st.dropped["page_sequence"];
            return 0;
        }
        if (s3 != s1 + 6 || s4 != s3 + 3 || s5 != s4 + 3 || s6 != s5 + 3 || s7 != s6 + 3 ||
            s8 != s7 + 3 || s9 != s8 + 3 || s10 != s9 + 3) {
            ++st.dropped["sow_sequence"];
            return 0;
        }
        if (toc_bds != eph.toes) {
            ++st.dropped["toc_toe_mismatch"];
            return 0;
        }
        eph.af1 = mergeTwoS(f1p3, f1p4, 18) * kP2_50;
        eph.cuc = mergeTwoS(cucp4, cucp5, 4) * kP2_31;
        eph.e = mergeTwoS(ep5, ep6, 22) * kP2_33;
        eph.cic = mergeTwoS(cicp6, cicp7, 8) * kP2_31;
        eph.i0 = mergeTwoS(i0p7, i0p8, 11) * kP2_31 * kPi;
        eph.omega_dot = mergeTwoS(omgdp8, omgdp9, 5) * kP2_43 * kPi;
        eph.omega = mergeTwoS(omgp9, omgp10, 5) * kP2_31 * kPi;
        sow1 = s1;
    }

    eph.satellite = SatelliteId(GNSSSystem::BeiDou, sfrbx.sv_id);
    eph.i_dot = eph.idot;
    eph.ura = static_cast<uint8_t>(sva);
    eph.sv_accuracy = beidouUraMeters(eph.ura);
    eph.health = static_cast<uint8_t>(svh);
    eph.sv_health = static_cast<double>(svh);

    // Week consistent with toe (RTKLIB's sign convention is inverted here; the
    // nearest-week rule below is the ICD one and only differs at the week turn-over).
    const double toes = eph.toes;
    if (toes > sow1 + kHalfWeek) {
        --bdt_week;
    } else if (toes < sow1 - kHalfWeek) {
        ++bdt_week;
    }
    eph.week = static_cast<uint16_t>(bdt_week);
    eph.toe = GNSSTime(bdt_week + kBdtWeekOffset, toes) + kBdtToGpst;
    eph.toc = nearestTo(eph.toe, bdt_week + kBdtWeekOffset, toc_bds) + kBdtToGpst;
    eph.tof = GNSSTime(bdt_week + kBdtWeekOffset, sow1) + kBdtToGpst;
    eph.valid = true;
    // D1: subframes 1-3.  D2: pages 1 and 3-10 (page 2 and the other subframes are not used).
    std::vector<Stamp> used;
    if (geo) {
        for (size_t page : {0, 2, 3, 4, 5, 6, 7, 8, 9}) {
            used.push_back(state.stamp[page]);
        }
    } else {
        used = {state.stamp[0], state.stamp[1], state.stamp[2]};
    }
    const UBXNavReception reception = receptionOf(used);
    return emit(eph, reception, out) ? 1 : 0;
}

// --- GLONASS L1OF / L2OF --------------------------------------------------------

size_t UBXNavDecoder::pushGlonass(const UBXSfrbx& sfrbx, std::vector<Ephemeris>& out) {
    auto& st = systemStats(GNSSSystem::GLONASS);
    if ((sfrbx.signal_id != 0 && sfrbx.signal_id != 2) || sfrbx.words.size() < 4 ||
        sfrbx.sv_id == 0 || sfrbx.sv_id > 27) {
        ++st.skipped_signal;  // includes the slot 255 "unknown satellite" reports
        return 0;
    }
    ++st.subframes;

    // 4 words, each most significant byte first.
    uint8_t buff[16];
    for (int i = 0; i < 4; ++i) {
        const uint32_t w = sfrbx.words[static_cast<size_t>(i)];
        buff[4 * i + 0] = static_cast<uint8_t>(w >> 24);
        buff[4 * i + 1] = static_cast<uint8_t>(w >> 16);
        buff[4 * i + 2] = static_cast<uint8_t>(w >> 8);
        buff[4 * i + 3] = static_cast<uint8_t>(w);
    }
    if (!ubx_utils::testGlonassString(buff)) {
        ++st.dropped["hamming"];
        return 0;
    }
    const int m = static_cast<int>(getbitu(buff, 1, 4));
    if (m < 1 || m > 15) {
        ++st.dropped["string_number"];
        return 0;
    }
    if (m > 4) {
        return 0;  // strings 5-15 (almanac, UTC) are not used
    }

    GlonassState& state = glonass_[{sfrbx.system, sfrbx.sv_id, 0}];
    // Flush the frame buffer when the frame id (bytes 12-13) changes.
    if (!state.has_frame_id || state.frame_id[0] != buff[12] || state.frame_id[1] != buff[13]) {
        state.strings.fill(0);
        state.present.fill(false);
        state.frame_id = {buff[12], buff[13]};
        state.has_frame_id = true;
    }
    std::copy(buff, buff + 10, state.strings.begin() + (m - 1) * 10);
    state.present[static_cast<size_t>(m - 1)] = true;
    state.stamp[static_cast<size_t>(m - 1)] = nowStamp();
    if (m != 4) {
        return 0;
    }
    for (int s = 0; s < 3; ++s) {
        if (!state.present[static_cast<size_t>(s)]) {
            ++st.dropped["incomplete_set"];
            return 0;
        }
    }
    if (!has_reference_) {
        ++st.dropped["no_time_reference"];
        return 0;
    }

    const uint8_t* b = state.strings.data();
    Ephemeris eph = freshEphemeris();
    int i = 1;
    // string 1
    const int frn1 = static_cast<int>(getbitu(b, i, 4)); i += 4 + 2;
    i += 2;  // P1
    const int tk_h = static_cast<int>(getbitu(b, i, 5)); i += 5;
    const int tk_m = static_cast<int>(getbitu(b, i, 6)); i += 6;
    const int tk_s = static_cast<int>(getbitu(b, i, 1)) * 30; i += 1;
    eph.glonass_velocity.x() = getbitg(b, i, 24) * kP2_20 * 1E3; i += 24;
    eph.glonass_acceleration.x() = getbitg(b, i, 5) * kP2_30 * 1E3; i += 5;
    eph.glonass_position.x() = getbitg(b, i, 27) * kP2_11 * 1E3; i += 27 + 4;
    // string 2
    const int frn2 = static_cast<int>(getbitu(b, i, 4)); i += 4;
    const int bn = static_cast<int>(getbitu(b, i, 1)); i += 1 + 2;  // MSB of Bn
    i += 1;  // P2
    const int tb = static_cast<int>(getbitu(b, i, 7)); i += 7 + 5;
    eph.glonass_velocity.y() = getbitg(b, i, 24) * kP2_20 * 1E3; i += 24;
    eph.glonass_acceleration.y() = getbitg(b, i, 5) * kP2_30 * 1E3; i += 5;
    eph.glonass_position.y() = getbitg(b, i, 27) * kP2_11 * 1E3; i += 27 + 4;
    // string 3
    const int frn3 = static_cast<int>(getbitu(b, i, 4)); i += 4;
    i += 1;  // P3
    eph.glonass_gamn = getbitg(b, i, 11) * kP2_40; i += 11 + 1;
    i += 2;  // P
    const int ln = static_cast<int>(getbitu(b, i, 1)); i += 1;  // ln: string-3 health flag
    eph.glonass_velocity.z() = getbitg(b, i, 24) * kP2_20 * 1E3; i += 24;
    eph.glonass_acceleration.z() = getbitg(b, i, 5) * kP2_30 * 1E3; i += 5;
    eph.glonass_position.z() = getbitg(b, i, 27) * kP2_11 * 1E3; i += 27 + 4;
    // string 4
    const int frn4 = static_cast<int>(getbitu(b, i, 4)); i += 4;
    eph.glonass_taun = getbitg(b, i, 22) * kP2_30; i += 22;
    i += 5;  // delta tau_n
    eph.glonass_age = static_cast<int>(getbitu(b, i, 5)); i += 5 + 14;
    i += 1;  // P4
    i += 4 + 3;  // FT, reserved
    i += 11;  // NT
    const int slot = static_cast<int>(getbitu(b, i, 5));

    if (frn1 != 1 || frn2 != 2 || frn3 != 3 || frn4 != 4) {
        ++st.dropped["string_number"];
        return 0;
    }
    if (slot != sfrbx.sv_id) {
        ++st.dropped["satellite_mismatch"];
        return 0;
    }

    eph.satellite = SatelliteId(GNSSSystem::GLONASS, sfrbx.sv_id);
    eph.glonass_frequency_channel = static_cast<int>(sfrbx.frequency_id) - 7;
    eph.glonass_frequency_channel_present = true;
    eph.health = static_cast<uint8_t>(bn);
    eph.sv_health = static_cast<double>(bn);
    eph.glonass_ln = ln;
    eph.iode = static_cast<uint16_t>(tb & 0x7F);

    const GNSSTime ref_utc = rtcm_internal::gpstToUtcApprox(reference_);
    const GNSSTime tof_utc = rtcm_internal::alignUtcTimeOfDay(
        tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0, ref_utc);
    const GNSSTime toe_utc = rtcm_internal::alignUtcTimeOfDay(tb * 900.0 - 10800.0, ref_utc);
    eph.tof = rtcm_internal::utcToGpstApprox(tof_utc);
    eph.toe = rtcm_internal::utcToGpstApprox(toe_utc);
    eph.toc = eph.toe;
    eph.week = static_cast<uint16_t>(eph.toe.week);
    eph.valid = true;
    const UBXNavReception reception =
        receptionOf({state.stamp[0], state.stamp[1], state.stamp[2], state.stamp[3]});
    return emit(eph, reception, out) ? 1 : 0;
}

bool ubxNavSameIssue(const Ephemeris& a, const Ephemeris& b) {
    if (!(a.satellite == b.satellite)) {
        return false;
    }
    switch (a.satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return a.iode == b.iode && a.iodc == b.iodc && a.toe == b.toe && a.toc == b.toc;
        case GNSSSystem::Galileo:
            return a.iode == b.iode && a.toe == b.toe && a.toc == b.toc;
        case GNSSSystem::BeiDou:
            return a.toe == b.toe;
        case GNSSSystem::GLONASS:
            return a.iode == b.iode;
        default:
            return false;
    }
}

bool ubxNavEphemerisUsable(const Ephemeris& eph) {
    switch (eph.satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return eph.health == 0;                // LNAV svh (6 bits)
        case GNSSSystem::Galileo:
            return (eph.health & 0x07) == 0;       // E1-B DVS (bit 0) and E1-B HS (bits 1-2)
        case GNSSSystem::BeiDou:
            return eph.health == 0;                // SatH1
        case GNSSSystem::GLONASS:
            return eph.health == 0 && eph.glonass_ln == 0;  // Bn MSB, ln
        default:
            return true;
    }
}

namespace {

bool sameBits(double a, double b) {
    return std::memcmp(&a, &b, sizeof(double)) == 0;
}

bool sameTime(const GNSSTime& a, const GNSSTime& b) {
    return a.week == b.week && sameBits(a.tow, b.tow);
}

bool sameVector(const Vector3d& a, const Vector3d& b) {
    return sameBits(a.x(), b.x()) && sameBits(a.y(), b.y()) && sameBits(a.z(), b.z());
}

}  // namespace

bool ubxNavSameRecord(const Ephemeris& a, const Ephemeris& b) {
    // Every Ephemeris member is either compared here or listed as excluded in ubx_nav_decode.hpp:
    // `tof` (reception-derived: transmission time resolved against the receiver clock) and the
    // fields this decoder never fills (glonass_cdma_data, glonass_canonical_geph_*).
    return a.satellite == b.satellite &&
           sameTime(a.toe, b.toe) && sameTime(a.toc, b.toc) && sameBits(a.toes, b.toes) &&
           sameBits(a.sqrt_a, b.sqrt_a) && sameBits(a.e, b.e) && sameBits(a.i0, b.i0) &&
           sameBits(a.omega0, b.omega0) && sameBits(a.omega, b.omega) && sameBits(a.m0, b.m0) &&
           sameBits(a.delta_n, b.delta_n) && sameBits(a.idot, b.idot) && sameBits(a.i_dot, b.i_dot) &&
           sameBits(a.omega_dot, b.omega_dot) &&
           sameBits(a.cuc, b.cuc) && sameBits(a.cus, b.cus) && sameBits(a.crc, b.crc) &&
           sameBits(a.crs, b.crs) && sameBits(a.cic, b.cic) && sameBits(a.cis, b.cis) &&
           sameBits(a.af0, b.af0) && sameBits(a.af1, b.af1) && sameBits(a.af2, b.af2) &&
           sameBits(a.tgd, b.tgd) && sameBits(a.tgd_secondary, b.tgd_secondary) &&
           sameBits(a.glonass_taun, b.glonass_taun) && sameBits(a.glonass_gamn, b.glonass_gamn) &&
           sameVector(a.glonass_position, b.glonass_position) &&
           sameVector(a.glonass_velocity, b.glonass_velocity) &&
           sameVector(a.glonass_acceleration, b.glonass_acceleration) &&
           a.glonass_frequency_channel == b.glonass_frequency_channel &&
           a.glonass_frequency_channel_present == b.glonass_frequency_channel_present &&
           a.glonass_age == b.glonass_age && a.glonass_ln == b.glonass_ln &&
           a.week == b.week && a.health == b.health && sameBits(a.sv_health, b.sv_health) &&
           sameBits(a.sv_accuracy, b.sv_accuracy) && a.ura == b.ura && a.iodc == b.iodc &&
           a.iode == b.iode && a.data_source_code == b.data_source_code &&
           a.navigation_message_type == b.navigation_message_type && a.valid == b.valid;
}

UBXNavConfirmTwice::Outcome UBXNavConfirmTwice::push(const UBXNavDecoded& copy) {
    Outcome outcome;
    SatState& sat = sats_[copy.eph.satellite];
    Counters& counters = counters_[copy.eph.satellite.system];
    ++counters.copies;

    if (sat.accepted && ubxNavSameRecord(copy.eph, *sat.accepted)) {
        ++counters.duplicates;
        return outcome;
    }
    if (!sat.candidate) {
        sat.candidate = copy.eph;
        sat.candidate_reception = copy.reception;
        ++counters.candidates;
        return outcome;
    }
    if (!ubxNavSameRecord(copy.eph, *sat.candidate)) {
        ++counters.conflicts;
        ++counters.candidates;
        sat.candidate = copy.eph;
        sat.candidate_reception = copy.reception;
        return outcome;
    }
    if (copy.reception.first_seq <= sat.candidate_reception.last_seq) {
        ++counters.same_transmission;  // shares a message with the candidate: not a second transmission
        return outcome;
    }
    ++counters.accepted;
    outcome.accepted = true;
    outcome.first_copy = sat.candidate_reception;
    sat.accepted = copy.eph;
    sat.candidate.reset();
    return outcome;
}

size_t UBXNavConfirmTwice::unconfirmed() const {
    size_t n = 0;
    for (const auto& [system, c] : counters_) {
        n += c.candidates - c.accepted;
    }
    return n;
}

size_t UBXNavConfirmTwice::conflicts() const {
    size_t n = 0;
    for (const auto& [system, c] : counters_) {
        n += c.conflicts;
    }
    return n;
}

size_t UBXNavConfirmTwice::accepted() const {
    size_t n = 0;
    for (const auto& [system, c] : counters_) {
        n += c.accepted;
    }
    return n;
}

void UBXNavConfirmTwice::clear() {
    sats_.clear();
    counters_.clear();
}

// ---------------------------------------------------------------------------
// D3: accept, then confirm
// ---------------------------------------------------------------------------

bool UBXNavAcceptConfirm::release(const Ephemeris& eph, NavigationData& nav, Counters& counters,
                                  Outcome& outcome) {
    // The receiver forwards unhealthy and default sets (e.g. GPS svh=63) that broadcast navigation
    // files do not carry; they must not reach the database.
    if (!ubxNavEphemerisUsable(eph)) {
        ++counters.unhealthy_skipped;
        outcome.unhealthy_skipped = true;
        return false;
    }
    if (!nav.addEphemerisIfNew(eph)) {
        ++counters.already_present;
        return false;
    }
    ++counters.merged;
    outcome.merged = true;
    return true;
}

bool UBXNavAcceptConfirm::covered(const SatState& sat, const UBXNavReception& reception) {
    // The receiver GPST of the copy's completing message; unknown (NaN) until the first RAWX epoch.
    // Without a time nothing can be said to be valid, so the satellite counts as uncovered (rule 1).
    const double gpst_s = reception.last_gpst_s;
    if (!std::isfinite(gpst_s)) {
        return false;
    }
    const double week = std::floor(gpst_s / 604800.0);
    const GNSSTime time(static_cast<int>(week), gpst_s - week * 604800.0);
    for (const Entry& entry : sat.entries) {
        if (entry.merged && entry.eph.isValid(time)) {
            return true;
        }
    }
    return false;
}

void UBXNavAcceptConfirm::retractUnconfirmed(SatState& sat, const Ephemeris& confirmed,
                                             NavigationData& nav, Counters& counters,
                                             Outcome& outcome) {
    // Rule B.  Held copies were never merged and stay held; confirmed records are not touched.
    for (size_t i = 0; i < sat.entries.size();) {
        const Entry& entry = sat.entries[i];
        if (!entry.merged || entry.confirmed || ubxNavSameIssue(entry.eph, confirmed)) {
            ++i;
            continue;
        }
        if (nav.removeEphemeris(entry.eph)) {
            ++counters.retracted;
            ++counters.retracted_by_confirmation;
            outcome.retracted.push_back({entry.eph, entry.reception});
        }
        sat.entries.erase(sat.entries.begin() + static_cast<std::ptrdiff_t>(i));
    }
}

void UBXNavAcceptConfirm::trimEntries(SatState& sat, Counters& counters) {
    while (sat.entries.size() > kMaxEntries) {
        // The entry just added (the last one) is never the one dropped.  Among the older ones a held copy
        // (rule A) is only a candidate, so the oldest of those goes before the records that cover the
        // satellite: a flood of corrupted new issues cannot push its valid record out of the list.
        const auto newest = sat.entries.end() - 1;
        auto drop = std::find_if(sat.entries.begin(), newest, [](const Entry& entry) { return entry.held; });
        if (drop == newest) {
            drop = sat.entries.begin();
        }
        if ((drop->merged || drop->held) && !drop->confirmed) {
            ++counters.dropped_unconfirmed;
        }
        sat.entries.erase(drop);
    }
}

UBXNavAcceptConfirm::Outcome UBXNavAcceptConfirm::push(const UBXNavDecoded& copy, NavigationData& nav) {
    SatState& sat = sats_[copy.eph.satellite];
    Counters& counters = counters_[copy.eph.satellite.system];
    ++counters.copies;
    if (sat.fallback) {
        return pushFallback(copy, nav, sat, counters);
    }

    Outcome outcome;
    // The newest entry of the same issue, if D3 has seen this issue for the satellite.
    size_t found = sat.entries.size();
    for (size_t i = sat.entries.size(); i-- > 0;) {
        if (ubxNavSameIssue(sat.entries[i].eph, copy.eph)) {
            found = i;
            break;
        }
    }

    if (found == sat.entries.size()) {
        Entry entry;
        entry.eph = copy.eph;
        entry.reception = copy.reception;
        if (covered(sat, copy.reception)) {
            // Rule A: a new issue while the satellite holds a valid record of ours is merged only once
            // confirmed.  A copy corrupted in an issue field looks like this, and there is no second
            // copy of it.
            entry.held = true;
            ++counters.held_new_issue;
            outcome.held = true;
        } else {
            // Rule 1 (first record, or nothing valid held): merge at once; earlier records stay.
            entry.merged = release(copy.eph, nav, counters, outcome);
        }
        sat.entries.push_back(std::move(entry));
        trimEntries(sat, counters);
        return outcome;
    }

    Entry& entry = sat.entries[found];
    if (entry.held) {
        // A copy held by rule A: nothing of it is in the database.
        if (ubxNavSameRecord(copy.eph, entry.eph)) {
            if (copy.reception.first_seq > entry.reception.last_seq) {
                // Two equal copies from different transmissions: confirmed, merged now (D1 first).
                const UBXNavReception first_copy = entry.reception;
                entry.held = false;
                entry.confirmed = true;
                entry.merged = release(copy.eph, nav, counters, outcome);
                ++counters.confirmed;
                outcome.confirmed = true;
                outcome.first_copy = first_copy;
                retractUnconfirmed(sat, copy.eph, nav, counters, outcome);   // rule B (invalidates `entry`)
            } else {
                ++counters.same_transmission;
            }
            return outcome;
        }
        // Conflict between two copies of a held issue: nothing to remove.  The satellite falls back to
        // confirm-twice with both copies as candidates (either may be the corrupted one).
        ++counters.conflicts;
        outcome.conflict = true;
        sat.pending.clear();
        sat.pending.push_back({entry.eph, entry.reception, false});
        sat.pending.push_back({copy.eph, copy.reception, false});
        sat.fallback = true;
        sat.entries.erase(sat.entries.begin() + static_cast<std::ptrdiff_t>(found));
        return outcome;
    }
    if (entry.confirmed) {
        if (ubxNavSameRecord(copy.eph, entry.eph)) {
            ++counters.duplicates;
            return outcome;
        }
        if (entry.merged) {
            // Rule 4: two agreeing transmissions outweigh one differing copy.
            ++counters.diverged_after_confirm;
            return outcome;
        }
        // The confirmed record never reached the database (the health rule refused it, or the database
        // already held the issue).  A differing copy is a record the database does not hold: rule 1 again.
        // (A BeiDou satellite that clears its health flag within one toe is the case seen in development logs.)
        Entry fresh;
        fresh.eph = copy.eph;
        fresh.reception = copy.reception;
        fresh.merged = release(copy.eph, nav, counters, outcome);
        sat.entries.erase(sat.entries.begin() + static_cast<std::ptrdiff_t>(found));
        sat.entries.push_back(std::move(fresh));
        return outcome;
    }
    if (ubxNavSameRecord(copy.eph, entry.eph)) {
        if (copy.reception.first_seq > entry.reception.last_seq) {
            entry.confirmed = true;
            ++counters.confirmed;
            outcome.confirmed = true;
            outcome.first_copy = entry.reception;
            retractUnconfirmed(sat, copy.eph, nav, counters, outcome);   // rule B (invalidates `entry`)
        } else {
            ++counters.same_transmission;  // shares a message with the first copy: not a second transmission
        }
        return outcome;
    }

    // Rule 2, conflict: take the merged record back and fall back to D2 for this satellite.
    ++counters.conflicts;
    outcome.conflict = true;
    if (entry.merged && nav.removeEphemeris(entry.eph)) {
        ++counters.retracted;
        outcome.retracted.push_back({entry.eph, entry.reception});
    }
    // Either of the two differing copies may be the corrupted one, so the first stays a candidate
    // (a seed: already counted as retracted) next to the new one.
    sat.pending.clear();
    sat.pending.push_back({entry.eph, entry.reception, true});
    sat.pending.push_back({copy.eph, copy.reception, false});
    sat.fallback = true;
    sat.entries.erase(sat.entries.begin() + static_cast<std::ptrdiff_t>(found));
    return outcome;
}

UBXNavAcceptConfirm::Outcome UBXNavAcceptConfirm::pushFallback(const UBXNavDecoded& copy,
                                                               NavigationData& nav, SatState& sat,
                                                               Counters& counters) {
    Outcome outcome;
    // A record confirmed earlier for this issue stands: a repeat of it is a duplicate, and a differing copy is ignored
    // when that record is in the database (rule 4).  A differing copy of a confirmed record that never reached the
    // database is a candidate like any other.
    for (const Entry& entry : sat.entries) {
        if (entry.confirmed && ubxNavSameIssue(entry.eph, copy.eph)) {
            if (ubxNavSameRecord(copy.eph, entry.eph)) {
                ++counters.duplicates;
                return outcome;
            }
            if (entry.merged) {
                ++counters.diverged_after_confirm;
                return outcome;
            }
        }
    }
    for (size_t i = 0; i < sat.pending.size(); ++i) {
        const Pending& candidate = sat.pending[i];
        if (!ubxNavSameRecord(copy.eph, candidate.eph)) {
            continue;
        }
        if (copy.reception.first_seq <= candidate.reception.last_seq) {
            ++counters.same_transmission;
            return outcome;
        }
        // Two equal copies from different transmissions: confirmed.  The other candidates are
        // forgotten; the ones that were copies (not the retracted seed) were never confirmed.
        ++counters.confirmed;
        outcome.confirmed = true;
        outcome.first_copy = candidate.reception;
        for (size_t j = 0; j < sat.pending.size(); ++j) {
            if (j != i && !sat.pending[j].seed) {
                ++counters.dropped_unconfirmed;
            }
        }
        Entry entry;
        entry.eph = copy.eph;
        entry.reception = candidate.reception;
        entry.confirmed = true;
        // An unconfirmed entry of this issue left from before the conflict is superseded by it.
        bool stale_merged = false;
        for (size_t k = sat.entries.size(); k-- > 0;) {
            if (ubxNavSameIssue(sat.entries[k].eph, copy.eph)) {
                stale_merged = stale_merged || sat.entries[k].merged;
                sat.entries.erase(sat.entries.begin() + static_cast<std::ptrdiff_t>(k));
            }
        }
        entry.merged = stale_merged || release(copy.eph, nav, counters, outcome);
        sat.entries.push_back(std::move(entry));
        retractUnconfirmed(sat, copy.eph, nav, counters, outcome);   // rule B
        trimEntries(sat, counters);
        sat.pending.clear();
        sat.fallback = false;
        return outcome;
    }
    // Equal to no pending candidate: another differing copy (D2: the candidate is replaced; here the
    // earlier ones stay, up to kMaxPending, because the one that matches a later copy may be any of them).
    ++counters.conflicts;
    outcome.conflict = true;
    sat.pending.push_back({copy.eph, copy.reception, false});
    if (sat.pending.size() > kMaxPending) {
        if (!sat.pending.front().seed) {
            ++counters.dropped_unconfirmed;
        }
        sat.pending.erase(sat.pending.begin());
    }
    return outcome;
}

std::map<GNSSSystem, size_t> UBXNavAcceptConfirm::unconfirmedBySystem() const {
    std::map<GNSSSystem, size_t> result;
    for (const auto& [system, c] : counters_) {
        result[system] += c.dropped_unconfirmed;
    }
    for (const auto& [sat_id, sat] : sats_) {
        for (const Entry& entry : sat.entries) {
            if ((entry.merged || entry.held) && !entry.confirmed) {
                ++result[sat_id.system];
            }
        }
        for (const Pending& candidate : sat.pending) {
            if (!candidate.seed) {
                ++result[sat_id.system];
            }
        }
    }
    return result;
}

size_t UBXNavAcceptConfirm::unconfirmed() const {
    size_t n = 0;
    for (const auto& [system, count] : unconfirmedBySystem()) {
        n += count;
    }
    return n;
}

namespace {
template <typename Member>
size_t sumCounter(const std::map<GNSSSystem, UBXNavAcceptConfirm::Counters>& counters, Member member) {
    size_t n = 0;
    for (const auto& [system, c] : counters) {
        n += c.*member;
    }
    return n;
}
}  // namespace

size_t UBXNavAcceptConfirm::merged() const { return sumCounter(counters_, &Counters::merged); }
size_t UBXNavAcceptConfirm::confirmed() const { return sumCounter(counters_, &Counters::confirmed); }
size_t UBXNavAcceptConfirm::conflicts() const { return sumCounter(counters_, &Counters::conflicts); }
size_t UBXNavAcceptConfirm::retracted() const { return sumCounter(counters_, &Counters::retracted); }
size_t UBXNavAcceptConfirm::heldNewIssue() const { return sumCounter(counters_, &Counters::held_new_issue); }
size_t UBXNavAcceptConfirm::unhealthySkipped() const {
    return sumCounter(counters_, &Counters::unhealthy_skipped);
}

std::map<GNSSSystem, size_t> UBXNavAcceptConfirm::heldBySystem() const {
    std::map<GNSSSystem, size_t> result;
    for (const auto& [sat_id, sat] : sats_) {
        for (const Entry& entry : sat.entries) {
            if (entry.held) {
                ++result[sat_id.system];
            }
        }
    }
    return result;
}

bool UBXNavAcceptConfirm::inFallback(const SatelliteId& sat) const {
    const auto it = sats_.find(sat);
    return it != sats_.end() && it->second.fallback;
}

void UBXNavAcceptConfirm::clear() {
    sats_.clear();
    counters_.clear();
}

}  // namespace io
}  // namespace libgnss
