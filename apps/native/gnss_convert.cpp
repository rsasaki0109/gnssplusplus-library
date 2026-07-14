#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/ubx.hpp>

#include <algorithm>
#include <chrono>
#include <array>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <system_error>
#include <string>
#include <vector>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {

enum class InputFormat {
    RTCM,
    UBX
};

struct ConvertConfig {
    std::string input_path;
    std::string obs_out_path;
    std::string nav_out_path;
    std::string sfrbx_out_path;
    InputFormat format = InputFormat::RTCM;
    size_t limit = 0;
    bool quiet = false;
};

constexpr int kDefaultSerialBaud = 115200;
constexpr double kSemiCircleToRadians = 3.14159265358979323846;
constexpr double kPow2Neg11 = 4.88281250000000000000e-04;
constexpr double kPow2Neg5 = 3.12500000000000000000e-02;
constexpr double kPow2Neg6 = 1.56250000000000000000e-02;
constexpr double kPow2Neg19 = 1.90734863281250000000e-06;
constexpr double kPow2Neg20 = 9.53674316406250000000e-07;
constexpr double kPow2Neg29 = 1.86264514923095703125e-09;
constexpr double kPow2Neg30 = 9.31322574615478515625e-10;
constexpr double kPow2Neg31 = 4.65661287307739257812e-10;
constexpr double kPow2Neg33 = 1.16415321826934814453e-10;
constexpr double kPow2Neg40 = 9.09494701772928237915e-13;
constexpr double kPow2Neg43 = 1.13686837721616029739e-13;
constexpr double kPow2Neg50 = 8.88178419700125232339e-16;
constexpr double kPow2Neg55 = 2.77555756156289135106e-17;
constexpr double kPow2Neg66 = 1.35525271560688054251e-20;
constexpr double kHalfWeekSeconds = 302400.0;
constexpr double kSecondsPerDay = 86400.0;
constexpr double kHalfDaySeconds = 43200.0;
constexpr int kBdtWeekOffset = 1356;
constexpr std::array<double, 16> kUraMetersTable = {
    2.4,   3.4,   4.85,   6.85,
    9.65,  13.65, 24.0,   48.0,
    96.0,  192.0, 384.0,  768.0,
    1536.0,3072.0,6144.0, 32767.0
};

struct GpsLnavFrameSet {
    std::array<std::array<uint8_t, 30>, 3> subframes{};
    std::array<bool, 3> present = {false, false, false};
};

struct GlonassFrameSet {
    std::array<std::array<uint8_t, 10>, 4> strings{};
    std::array<bool, 4> present = {false, false, false, false};
    std::array<uint8_t, 2> frame_id = {0U, 0U};
    bool has_frame_id = false;
};

struct BdsD1FrameSet {
    std::array<std::array<uint8_t, 38>, 3> subframes{};
    std::array<bool, 3> present = {false, false, false};
};

struct BdsD2FrameSet {
    std::array<std::array<uint8_t, 38>, 10> pages{};
    std::array<bool, 10> present = {false, false, false, false, false,
                                    false, false, false, false, false};
};

struct GalileoWordSet {
    std::array<std::array<uint8_t, 16>, 6> words{};
    std::array<bool, 6> present = {false, false, false, false, false, false};
};

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0
        << " --input <file|ntrip://...|serial://...|tcp://host:port|/dev/tty...> --format <rtcm|ubx> [options]\n"
        << "Options:\n"
        << "  --obs-out <file>          Export decoded observations to a simple RINEX file\n"
        << "  --nav-out <file>          Export decoded broadcast nav to a RINEX nav file\n"
        << "  --sfrbx-out <file>        Export UBX RXM-SFRBX subframes to CSV\n"
        << "  --limit <count>           Stop after this many decoded messages (0 = all)\n"
        << "  --quiet                   Suppress per-message type lines\n"
        << "  --help                    Show this help text\n";
}

ConvertConfig parseArguments(int argc, char** argv) {
    ConvertConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            config.input_path = argv[++i];
        } else if (arg == "--obs-out" && i + 1 < argc) {
            config.obs_out_path = argv[++i];
        } else if (arg == "--nav-out" && i + 1 < argc) {
            config.nav_out_path = argv[++i];
        } else if (arg == "--sfrbx-out" && i + 1 < argc) {
            config.sfrbx_out_path = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "rtcm") {
                config.format = InputFormat::RTCM;
            } else if (value == "ubx") {
                config.format = InputFormat::UBX;
            } else {
                throw std::invalid_argument("unsupported --format value: " + value);
            }
        } else if (arg == "--limit" && i + 1 < argc) {
            config.limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--quiet") {
            config.quiet = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown or incomplete argument: " + arg);
        }
    }

    if (config.input_path.empty()) {
        throw std::invalid_argument("--input is required");
    }
    if (config.obs_out_path.empty() && config.nav_out_path.empty() &&
        config.sfrbx_out_path.empty()) {
        throw std::invalid_argument("provide at least one of --obs-out, --nav-out, or --sfrbx-out");
    }
    return config;
}

libgnss::io::RINEXReader::RINEXHeader makeObservationHeader() {
    libgnss::io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = libgnss::io::RINEXReader::FileType::OBSERVATION;
    header.satellite_system = "M";
    header.program = "libgnss++";
    header.run_by = "gnss convert";
    header.observation_types = {"C1C", "L1C", "D1C", "S1C"};
    return header;
}

libgnss::io::RINEXReader::RINEXHeader makeNavigationHeader() {
    libgnss::io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = libgnss::io::RINEXReader::FileType::NAVIGATION;
    header.satellite_system = "M";
    header.program = "libgnss++";
    header.run_by = "gnss convert";
    return header;
}

const char* systemName(libgnss::GNSSSystem system) {
    switch (system) {
        case libgnss::GNSSSystem::GPS: return "GPS";
        case libgnss::GNSSSystem::GLONASS: return "GLONASS";
        case libgnss::GNSSSystem::Galileo: return "Galileo";
        case libgnss::GNSSSystem::BeiDou: return "BeiDou";
        case libgnss::GNSSSystem::QZSS: return "QZSS";
        case libgnss::GNSSSystem::NavIC: return "NavIC";
        case libgnss::GNSSSystem::SBAS: return "SBAS";
        default: return "UNKNOWN";
    }
}

bool writeSfrbxCsvHeader(std::ofstream& output) {
    output << "system,sv_id,frequency_id,channel,version,frame_kind,frame_id,page_id,word_count,words_hex\n";
    return static_cast<bool>(output);
}

bool writeSfrbxCsvRow(std::ofstream& output, const libgnss::io::UBXSfrbx& sfrbx) {
    libgnss::io::UBXSfrbxFrameInfo frame_info;
    const bool has_frame_info = libgnss::io::ubx_utils::decodeSfrbxFrameInfo(sfrbx, frame_info);
    output << systemName(sfrbx.system) << ","
           << static_cast<int>(sfrbx.sv_id) << ","
           << static_cast<int>(sfrbx.frequency_id) << ","
           << static_cast<int>(sfrbx.channel) << ","
           << static_cast<int>(sfrbx.version) << ","
           << libgnss::io::ubx_utils::getSfrbxFrameKindName(frame_info.kind) << ","
           << (has_frame_info ? std::to_string(frame_info.frame_id) : std::string()) << ",";
    if (has_frame_info && frame_info.has_page_id) {
        output << frame_info.page_id;
    }
    output << ","
           << sfrbx.words.size() << ",";
    for (size_t index = 0; index < sfrbx.words.size(); ++index) {
        if (index != 0) {
            output << ";";
        }
        output << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
               << sfrbx.words[index]
               << std::dec << std::setfill(' ');
    }
    output << "\n";
    return static_cast<bool>(output);
}

std::vector<uint8_t> rawWordsToBigEndianBytes(const std::vector<uint32_t>& words,
                                              size_t max_words) {
    const size_t count = std::min(words.size(), max_words);
    std::vector<uint8_t> bytes;
    bytes.reserve(count * 4U);
    for (size_t index = 0; index < count; ++index) {
        const uint32_t word = words[index];
        bytes.push_back(static_cast<uint8_t>((word >> 24) & 0xFFU));
        bytes.push_back(static_cast<uint8_t>((word >> 16) & 0xFFU));
        bytes.push_back(static_cast<uint8_t>((word >> 8) & 0xFFU));
        bytes.push_back(static_cast<uint8_t>(word & 0xFFU));
    }
    return bytes;
}

uint32_t readBitsMsbFirst(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    if (data == nullptr || data_size == 0 || num_bits <= 0) {
        return 0;
    }

    uint32_t value = 0;
    for (int index = 0; index < num_bits; ++index) {
        const int absolute_bit = bit_pos + index;
        const size_t byte_index = static_cast<size_t>(absolute_bit / 8);
        if (byte_index >= data_size) {
            value <<= (num_bits - index);
            break;
        }
        const int bit_index = 7 - (absolute_bit % 8);
        value = static_cast<uint32_t>((value << 1) |
                                      ((data[byte_index] >> bit_index) & 0x01U));
    }
    return value;
}

int32_t readSignedBitsMsbFirst(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    const uint32_t value = readBitsMsbFirst(data, data_size, bit_pos, num_bits);
    if (num_bits <= 0) {
        return 0;
    }
    const uint64_t sign_mask = 1ULL << (num_bits - 1);
    if ((value & sign_mask) == 0U) {
        return static_cast<int32_t>(value);
    }
    const uint64_t extend_mask =
        num_bits >= 32 ? 0xFFFFFFFF00000000ULL : ~((1ULL << num_bits) - 1ULL);
    return static_cast<int32_t>(static_cast<uint64_t>(value) | extend_mask);
}

int64_t readSignMagnitudeBitsMsbFirst(const uint8_t* data,
                                      size_t data_size,
                                      int bit_pos,
                                      int num_bits) {
    if (data == nullptr || num_bits <= 1) {
        return 0;
    }

    const bool negative = readBitsMsbFirst(data, data_size, bit_pos, 1) != 0U;
    const uint32_t magnitude =
        readBitsMsbFirst(data, data_size, bit_pos + 1, num_bits - 1);
    return negative ? -static_cast<int64_t>(magnitude)
                    : static_cast<int64_t>(magnitude);
}

void writeBitsMsbFirst(uint8_t* data, size_t data_size, int bit_pos, int num_bits, uint32_t value) {
    for (int index = 0; index < num_bits; ++index) {
        const int absolute_bit = bit_pos + index;
        const size_t byte_index = static_cast<size_t>(absolute_bit / 8);
        if (byte_index >= data_size) {
            break;
        }
        const int bit_index = 7 - (absolute_bit % 8);
        const uint8_t mask = static_cast<uint8_t>(1U << bit_index);
        const bool set = ((value >> (num_bits - index - 1)) & 0x01U) != 0U;
        if (set) {
            data[byte_index] |= mask;
        } else {
            data[byte_index] &= static_cast<uint8_t>(~mask);
        }
    }
}

template <size_t N>
void writeBitsMsbFirst(std::array<uint8_t, N>& data, int bit_pos, int num_bits, uint32_t value) {
    writeBitsMsbFirst(data.data(), data.size(), bit_pos, num_bits, value);
}

int adjustGpsWeek(uint16_t week_mod_1024) {
    const int current_week =
        libgnss::GNSSTime::fromSystemTime(std::chrono::system_clock::now()).week;
    return static_cast<int>(week_mod_1024) +
           ((current_week - static_cast<int>(week_mod_1024) + 512) / 1024) * 1024;
}

double uraMetersFromIndex(uint8_t ura_index) {
    return kUraMetersTable[std::min<size_t>(ura_index, kUraMetersTable.size() - 1)];
}

libgnss::GNSSTime normalizeGpsWeekTow(int week, double tow) {
    while (tow < 0.0) {
        tow += libgnss::constants::SECONDS_PER_WEEK;
        --week;
    }
    while (tow >= libgnss::constants::SECONDS_PER_WEEK) {
        tow -= libgnss::constants::SECONDS_PER_WEEK;
        ++week;
    }
    return libgnss::GNSSTime(week, tow);
}

libgnss::GNSSTime bdtWeekTowToGpst(int week, double tow) {
    return libgnss::GNSSTime(week + kBdtWeekOffset, tow) + 14.0;
}

libgnss::GNSSTime gpstToBdt(const libgnss::GNSSTime& time) {
    return time - 14.0;
}

int leapSecondsForDate(int year, int month, int day) {
    struct LeapEntry { int year; int month; int day; int leap_seconds; };
    static constexpr LeapEntry kLeapTable[] = {
        {1981, 7, 1, 1},  {1982, 7, 1, 2},  {1983, 7, 1, 3},  {1985, 7, 1, 4},
        {1988, 1, 1, 5},  {1990, 1, 1, 6},  {1991, 1, 1, 7},  {1992, 7, 1, 8},
        {1993, 7, 1, 9},  {1994, 7, 1, 10}, {1996, 1, 1, 11}, {1997, 7, 1, 12},
        {1999, 1, 1, 13}, {2006, 1, 1, 14}, {2009, 1, 1, 15}, {2012, 7, 1, 16},
        {2015, 7, 1, 17}, {2017, 1, 1, 18},
    };

    int leap_seconds = 0;
    for (const auto& entry : kLeapTable) {
        if (year > entry.year ||
            (year == entry.year &&
             (month > entry.month || (month == entry.month && day >= entry.day)))) {
            leap_seconds = entry.leap_seconds;
        }
    }
    return leap_seconds;
}

std::tm toUtcTm(const std::chrono::system_clock::time_point& tp) {
    const std::time_t tt = std::chrono::system_clock::to_time_t(tp);
    std::tm utc_tm{};
#ifdef _WIN32
    gmtime_s(&utc_tm, &tt);
#else
    gmtime_r(&tt, &utc_tm);
#endif
    return utc_tm;
}

int leapSecondsForApproxTime(const libgnss::GNSSTime& time) {
    const std::tm utc_tm = toUtcTm(time.toSystemTime());
    return leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday);
}

libgnss::GNSSTime currentGpstApprox() {
    const auto now = std::chrono::system_clock::now();
    const std::tm utc_tm = toUtcTm(now);
    const libgnss::GNSSTime utc_like = libgnss::GNSSTime::fromSystemTime(now);
    return normalizeGpsWeekTow(
        utc_like.week,
        utc_like.tow +
            static_cast<double>(
                leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday)));
}

libgnss::GNSSTime gpstToUtcApprox(const libgnss::GNSSTime& gps_time) {
    return normalizeGpsWeekTow(
        gps_time.week,
        gps_time.tow - static_cast<double>(leapSecondsForApproxTime(gps_time)));
}

libgnss::GNSSTime utcToGpstApprox(const libgnss::GNSSTime& utc_time) {
    const std::tm utc_tm = toUtcTm(utc_time.toSystemTime());
    return normalizeGpsWeekTow(
        utc_time.week,
        utc_time.tow +
            static_cast<double>(
                leapSecondsForDate(utc_tm.tm_year + 1900, utc_tm.tm_mon + 1, utc_tm.tm_mday)));
}

double secondsOfDay(double tow) {
    double tod = std::fmod(tow, kSecondsPerDay);
    if (tod < 0.0) {
        tod += kSecondsPerDay;
    }
    return tod;
}

libgnss::GNSSTime alignUtcTimeOfDay(double tod_utc, const libgnss::GNSSTime& reference_utc) {
    const double ref_tod = secondsOfDay(reference_utc.tow);
    if (tod_utc < ref_tod - kHalfDaySeconds) {
        tod_utc += kSecondsPerDay;
    } else if (tod_utc > ref_tod + kHalfDaySeconds) {
        tod_utc -= kSecondsPerDay;
    }
    const double base_tow = reference_utc.tow - ref_tod;
    return normalizeGpsWeekTow(reference_utc.week, base_tow + tod_utc);
}

bool buildGpsLnavSubframe(const libgnss::io::UBXSfrbx& sfrbx,
                          std::array<uint8_t, 30>& subframe,
                          int& subframe_id) {
    if (sfrbx.words.size() < 10) {
        return false;
    }

    subframe.fill(0U);
    for (size_t index = 0; index < 10; ++index) {
        const uint32_t word_without_parity = static_cast<uint32_t>(sfrbx.words[index] >> 6);
        writeBitsMsbFirst(subframe, static_cast<int>(index * 24), 24, word_without_parity);
    }

    subframe_id = static_cast<int>(readBitsMsbFirst(subframe.data(), subframe.size(), 43, 3));
    return subframe_id >= 1 && subframe_id <= 5;
}

bool decodeGpsLnavEphemeris(const libgnss::SatelliteId& satellite,
                            const std::array<std::array<uint8_t, 30>, 3>& subframes,
                            libgnss::Ephemeris& eph) {
    eph = libgnss::Ephemeris{};
    eph.satellite = satellite;

    const auto& sf1 = subframes[0];
    const auto& sf2 = subframes[1];
    const auto& sf3 = subframes[2];

    {
        int bit = 48;
        const double tow = static_cast<double>(readBitsMsbFirst(sf1.data(), sf1.size(), 24, 17)) * 6.0;
        const uint16_t week_mod = static_cast<uint16_t>(readBitsMsbFirst(sf1.data(), sf1.size(), bit, 10));
        bit += 10;
        bit += 2;  // code on L2
        eph.ura = static_cast<uint8_t>(readBitsMsbFirst(sf1.data(), sf1.size(), bit, 4));
        eph.sv_accuracy = uraMetersFromIndex(eph.ura);
        bit += 4;
        eph.health = static_cast<uint8_t>(readBitsMsbFirst(sf1.data(), sf1.size(), bit, 6));
        eph.sv_health = static_cast<double>(eph.health);
        bit += 6;
        const uint16_t iodc_msb = static_cast<uint16_t>(readBitsMsbFirst(sf1.data(), sf1.size(), bit, 2));
        bit += 2;
        bit += 1 + 87;  // L2P flag + reserved
        const int32_t tgd = readSignedBitsMsbFirst(sf1.data(), sf1.size(), bit, 8);
        bit += 8;
        const uint16_t iodc_lsb = static_cast<uint16_t>(readBitsMsbFirst(sf1.data(), sf1.size(), bit, 8));
        bit += 8;
        const double toc = static_cast<double>(readBitsMsbFirst(sf1.data(), sf1.size(), bit, 16)) * 16.0;
        bit += 16;
        eph.af2 = static_cast<double>(readSignedBitsMsbFirst(sf1.data(), sf1.size(), bit, 8)) * kPow2Neg55;
        bit += 8;
        eph.af1 = static_cast<double>(readSignedBitsMsbFirst(sf1.data(), sf1.size(), bit, 16)) * kPow2Neg43;
        bit += 16;
        eph.af0 = static_cast<double>(readSignedBitsMsbFirst(sf1.data(), sf1.size(), bit, 22)) * kPow2Neg31;

        eph.tgd = (tgd == -128) ? 0.0 : static_cast<double>(tgd) * kPow2Neg31;
        eph.iodc = static_cast<uint16_t>((iodc_msb << 8) | iodc_lsb);
        eph.week = static_cast<uint16_t>(adjustGpsWeek(week_mod));
        eph.tof = normalizeGpsWeekTow(static_cast<int>(eph.week), tow);
        eph.toc = normalizeGpsWeekTow(static_cast<int>(eph.week), toc);
    }

    {
        int bit = 48;
        eph.iode = static_cast<uint16_t>(readBitsMsbFirst(sf2.data(), sf2.size(), bit, 8));
        bit += 8;
        eph.crs = static_cast<double>(readSignedBitsMsbFirst(sf2.data(), sf2.size(), bit, 16)) * kPow2Neg5;
        bit += 16;
        eph.delta_n = static_cast<double>(readSignedBitsMsbFirst(sf2.data(), sf2.size(), bit, 16)) *
                      kPow2Neg43 * kSemiCircleToRadians;
        bit += 16;
        eph.m0 = static_cast<double>(readSignedBitsMsbFirst(sf2.data(), sf2.size(), bit, 32)) *
                 kPow2Neg31 * kSemiCircleToRadians;
        bit += 32;
        eph.cuc = static_cast<double>(readSignedBitsMsbFirst(sf2.data(), sf2.size(), bit, 16)) * kPow2Neg29;
        bit += 16;
        eph.e = static_cast<double>(readBitsMsbFirst(sf2.data(), sf2.size(), bit, 32)) * kPow2Neg33;
        bit += 32;
        eph.cus = static_cast<double>(readSignedBitsMsbFirst(sf2.data(), sf2.size(), bit, 16)) * kPow2Neg29;
        bit += 16;
        eph.sqrt_a = static_cast<double>(readBitsMsbFirst(sf2.data(), sf2.size(), bit, 32)) * kPow2Neg19;
        bit += 32;
        eph.toes = static_cast<double>(readBitsMsbFirst(sf2.data(), sf2.size(), bit, 16)) * 16.0;
    }

    {
        int bit = 48;
        eph.cic = static_cast<double>(readSignedBitsMsbFirst(sf3.data(), sf3.size(), bit, 16)) * kPow2Neg29;
        bit += 16;
        eph.omega0 = static_cast<double>(readSignedBitsMsbFirst(sf3.data(), sf3.size(), bit, 32)) *
                     kPow2Neg31 * kSemiCircleToRadians;
        bit += 32;
        eph.cis = static_cast<double>(readSignedBitsMsbFirst(sf3.data(), sf3.size(), bit, 16)) * kPow2Neg29;
        bit += 16;
        eph.i0 = static_cast<double>(readSignedBitsMsbFirst(sf3.data(), sf3.size(), bit, 32)) *
                 kPow2Neg31 * kSemiCircleToRadians;
        bit += 32;
        eph.crc = static_cast<double>(readSignedBitsMsbFirst(sf3.data(), sf3.size(), bit, 16)) * kPow2Neg5;
        bit += 16;
        eph.omega = static_cast<double>(readSignedBitsMsbFirst(sf3.data(), sf3.size(), bit, 32)) *
                    kPow2Neg31 * kSemiCircleToRadians;
        bit += 32;
        eph.omega_dot = static_cast<double>(readSignedBitsMsbFirst(sf3.data(), sf3.size(), bit, 24)) *
                        kPow2Neg43 * kSemiCircleToRadians;
        bit += 24;
        const uint16_t iode = static_cast<uint16_t>(readBitsMsbFirst(sf3.data(), sf3.size(), bit, 8));
        bit += 8;
        eph.idot = static_cast<double>(readSignedBitsMsbFirst(sf3.data(), sf3.size(), bit, 14)) *
                   kPow2Neg43 * kSemiCircleToRadians;
        eph.i_dot = eph.idot;

        if (iode != eph.iode || iode != static_cast<uint16_t>(eph.iodc & 0xFFU)) {
            return false;
        }
    }

    int week = static_cast<int>(eph.week);
    double tow = eph.tof.tow;
    const double toc = eph.toc.tow;
    if (eph.toes < tow - kHalfWeekSeconds) {
        ++week;
        tow -= libgnss::constants::SECONDS_PER_WEEK;
    } else if (eph.toes > tow + kHalfWeekSeconds) {
        --week;
        tow += libgnss::constants::SECONDS_PER_WEEK;
    }

    eph.toe = normalizeGpsWeekTow(week, eph.toes);
    eph.toc = normalizeGpsWeekTow(week, toc);
    eph.tof = normalizeGpsWeekTow(week, tow);
    eph.valid = true;
    return true;
}

bool buildGlonassNavigationString(const libgnss::io::UBXSfrbx& sfrbx,
                                  std::array<uint8_t, 10>& string_data,
                                  std::array<uint8_t, 2>& frame_id,
                                  int& string_number) {
    if (sfrbx.words.size() < 4) {
        return false;
    }

    std::array<uint8_t, 16> bytes{};
    for (size_t word_index = 0; word_index < 4; ++word_index) {
        const uint32_t word = sfrbx.words[word_index];
        bytes[word_index * 4 + 0] = static_cast<uint8_t>((word >> 24) & 0xFFU);
        bytes[word_index * 4 + 1] = static_cast<uint8_t>((word >> 16) & 0xFFU);
        bytes[word_index * 4 + 2] = static_cast<uint8_t>((word >> 8) & 0xFFU);
        bytes[word_index * 4 + 3] = static_cast<uint8_t>(word & 0xFFU);
    }

    string_number = static_cast<int>(readBitsMsbFirst(bytes.data(), bytes.size(), 1, 4));
    if (string_number < 1 || string_number > 4) {
        return false;
    }

    std::copy(bytes.begin(), bytes.begin() + string_data.size(), string_data.begin());
    frame_id = {bytes[12], bytes[13]};
    return true;
}

bool decodeGlonassEphemeris(const libgnss::SatelliteId& satellite,
                            const std::array<std::array<uint8_t, 10>, 4>& strings,
                            const libgnss::GNSSTime& reference_gps_time,
                            int glonass_frequency_channel,
                            libgnss::Ephemeris& eph) {
    std::array<uint8_t, 40> frame_bytes{};
    for (size_t index = 0; index < strings.size(); ++index) {
        std::copy(strings[index].begin(), strings[index].end(),
                  frame_bytes.begin() + index * strings[index].size());
    }

    eph = libgnss::Ephemeris{};
    eph.satellite = satellite;

    int bit = 1;
    const int frn1 = static_cast<int>(readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 4));
    bit += 4 + 2;
    bit += 2;  // P1
    const double tk_h = static_cast<double>(
        readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 5));
    bit += 5;
    const double tk_m = static_cast<double>(
        readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 6));
    bit += 6;
    const double tk_s = static_cast<double>(
        readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 1)) * 30.0;
    bit += 1;
    eph.glonass_velocity.x() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 24)) *
        kPow2Neg20 * 1e3;
    bit += 24;
    eph.glonass_acceleration.x() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 5)) *
        kPow2Neg30 * 1e3;
    bit += 5;
    eph.glonass_position.x() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 27)) *
        kPow2Neg11 * 1e3;
    bit += 27 + 4;

    const int frn2 = static_cast<int>(readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 4));
    bit += 4;
    eph.health = static_cast<uint8_t>(
        readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 3));
    eph.sv_health = static_cast<double>(eph.health);
    bit += 3;
    bit += 1;  // P2
    const uint8_t tb = static_cast<uint8_t>(
        readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 7));
    bit += 7 + 5;
    eph.glonass_velocity.y() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 24)) *
        kPow2Neg20 * 1e3;
    bit += 24;
    eph.glonass_acceleration.y() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 5)) *
        kPow2Neg30 * 1e3;
    bit += 5;
    eph.glonass_position.y() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 27)) *
        kPow2Neg11 * 1e3;
    bit += 27 + 4;

    const int frn3 = static_cast<int>(readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 4));
    bit += 4;
    bit += 1;  // P3
    eph.glonass_gamn = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 11)) *
        kPow2Neg40;
    bit += 11 + 1;
    bit += 2;  // P
    bit += 1;  // ln
    eph.glonass_velocity.z() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 24)) *
        kPow2Neg20 * 1e3;
    bit += 24;
    eph.glonass_acceleration.z() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 5)) *
        kPow2Neg30 * 1e3;
    bit += 5;
    eph.glonass_position.z() = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 27)) *
        kPow2Neg11 * 1e3;
    bit += 27 + 4;

    const int frn4 = static_cast<int>(readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 4));
    bit += 4;
    eph.glonass_taun = static_cast<double>(
        readSignMagnitudeBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 22)) *
        kPow2Neg30;
    bit += 22;
    bit += 5;  // dtaun
    eph.glonass_age = static_cast<int>(
        readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 5));
    bit += 5 + 14;
    bit += 1;  // P4
    const uint8_t sva = static_cast<uint8_t>(
        readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 4));
    bit += 4 + 3;
    bit += 11;  // NT
    const uint8_t slot = static_cast<uint8_t>(
        readBitsMsbFirst(frame_bytes.data(), frame_bytes.size(), bit, 5));

    if (frn1 != 1 || frn2 != 2 || frn3 != 3 || frn4 != 4 || slot == 0 || slot != satellite.prn) {
        return false;
    }

    eph.ura = sva;
    eph.sv_accuracy = static_cast<double>(sva);
    eph.iode = tb;
    eph.iodc = tb;
    eph.glonass_frequency_channel = glonass_frequency_channel;

    const libgnss::GNSSTime reference_utc = gpstToUtcApprox(reference_gps_time);
    const libgnss::GNSSTime tof_utc =
        alignUtcTimeOfDay(tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0, reference_utc);
    const libgnss::GNSSTime toe_utc =
        alignUtcTimeOfDay(static_cast<double>(tb) * 900.0 - 10800.0, reference_utc);
    eph.tof = utcToGpstApprox(tof_utc);
    eph.toe = utcToGpstApprox(toe_utc);
    eph.toc = eph.toe;
    eph.week = static_cast<uint16_t>(eph.toe.week);
    eph.toes = secondsOfDay(toe_utc.tow + 10800.0);
    eph.valid = true;
    return true;
}

uint32_t mergeUnsignedFields(uint32_t high, uint32_t low, int low_bits) {
    return (high << low_bits) | low;
}

int32_t mergeSignedFields(int32_t high, uint32_t low, int low_bits) {
    return static_cast<int32_t>((static_cast<int64_t>(high) << low_bits) |
                                static_cast<int64_t>(low));
}

uint32_t mergeUnsignedFields3(uint32_t upper, uint32_t middle, int middle_bits, uint32_t lower, int lower_bits) {
    return (upper << (middle_bits + lower_bits)) | (middle << lower_bits) | lower;
}

int32_t mergeSignedFields3(int32_t upper, uint32_t middle, int middle_bits, uint32_t lower, int lower_bits) {
    return mergeSignedFields(mergeSignedFields(upper, middle, middle_bits), lower, lower_bits);
}

bool buildBeiDouD1Subframe(const libgnss::io::UBXSfrbx& sfrbx,
                           std::array<uint8_t, 38>& subframe,
                           int& subframe_id) {
    if (sfrbx.words.size() < 10 || sfrbx.sv_id <= 5) {
        return false;
    }

    subframe.fill(0U);
    for (size_t word_index = 0; word_index < 10; ++word_index) {
        writeBitsMsbFirst(subframe, static_cast<int>(word_index * 30), 30,
                          sfrbx.words[word_index] & 0x3FFFFFFFU);
    }

    subframe_id = static_cast<int>(readBitsMsbFirst(subframe.data(), subframe.size(), 15, 3));
    return subframe_id >= 1 && subframe_id <= 3;
}

bool decodeBeiDouD1Ephemeris(const libgnss::SatelliteId& satellite,
                             const std::array<std::array<uint8_t, 38>, 3>& subframes,
                             libgnss::Ephemeris& eph) {
    const auto& sf1 = subframes[0];
    const auto& sf2 = subframes[1];
    const auto& sf3 = subframes[2];

    const int frn1 = static_cast<int>(readBitsMsbFirst(sf1.data(), sf1.size(), 15, 3));
    const int frn2 = static_cast<int>(readBitsMsbFirst(sf2.data(), sf2.size(), 15, 3));
    const int frn3 = static_cast<int>(readBitsMsbFirst(sf3.data(), sf3.size(), 15, 3));
    if (frn1 != 1 || frn2 != 2 || frn3 != 3) {
        return false;
    }

    const uint32_t sow1 =
        mergeUnsignedFields(readBitsMsbFirst(sf1.data(), sf1.size(), 18, 8),
                            readBitsMsbFirst(sf1.data(), sf1.size(), 30, 12), 12);
    const uint32_t sow2 =
        mergeUnsignedFields(readBitsMsbFirst(sf2.data(), sf2.size(), 18, 8),
                            readBitsMsbFirst(sf2.data(), sf2.size(), 30, 12), 12);
    const uint32_t sow3 =
        mergeUnsignedFields(readBitsMsbFirst(sf3.data(), sf3.size(), 18, 8),
                            readBitsMsbFirst(sf3.data(), sf3.size(), 30, 12), 12);
    if (sow2 != sow1 + 6U || sow3 != sow2 + 6U) {
        return false;
    }

    eph = libgnss::Ephemeris{};
    eph.satellite = satellite;
    eph.health = static_cast<uint8_t>(readBitsMsbFirst(sf1.data(), sf1.size(), 42, 1));
    eph.sv_health = static_cast<double>(eph.health);
    eph.iodc = static_cast<uint16_t>(readBitsMsbFirst(sf1.data(), sf1.size(), 43, 5));
    eph.ura = static_cast<uint8_t>(readBitsMsbFirst(sf1.data(), sf1.size(), 48, 4));
    eph.sv_accuracy = static_cast<double>(eph.ura);
    const int week_bdt = static_cast<int>(readBitsMsbFirst(sf1.data(), sf1.size(), 60, 13));
    const double toc_bdt = static_cast<double>(
        mergeUnsignedFields(readBitsMsbFirst(sf1.data(), sf1.size(), 73, 9),
                            readBitsMsbFirst(sf1.data(), sf1.size(), 90, 8), 8)) * 8.0;
    eph.tgd = static_cast<double>(
        readSignedBitsMsbFirst(sf1.data(), sf1.size(), 98, 10)) * 0.1e-9;
    eph.tgd_secondary = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf1.data(), sf1.size(), 108, 4),
                          readBitsMsbFirst(sf1.data(), sf1.size(), 120, 6), 6)) * 0.1e-9;
    eph.af2 = static_cast<double>(
        readSignedBitsMsbFirst(sf1.data(), sf1.size(), 214, 11)) * kPow2Neg66;
    eph.af0 = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf1.data(), sf1.size(), 225, 7),
                          readBitsMsbFirst(sf1.data(), sf1.size(), 240, 17), 17)) * kPow2Neg33;
    eph.af1 = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf1.data(), sf1.size(), 257, 5),
                          readBitsMsbFirst(sf1.data(), sf1.size(), 270, 17), 17)) * kPow2Neg50;
    eph.iode = static_cast<uint16_t>(readBitsMsbFirst(sf1.data(), sf1.size(), 287, 5));

    eph.delta_n = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf2.data(), sf2.size(), 42, 10),
                          readBitsMsbFirst(sf2.data(), sf2.size(), 60, 6), 6)) *
                  kPow2Neg43 * kSemiCircleToRadians;
    eph.cuc = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf2.data(), sf2.size(), 66, 16),
                          readBitsMsbFirst(sf2.data(), sf2.size(), 90, 2), 2)) * kPow2Neg31;
    eph.m0 = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf2.data(), sf2.size(), 92, 20),
                          readBitsMsbFirst(sf2.data(), sf2.size(), 120, 12), 12)) *
             kPow2Neg31 * kSemiCircleToRadians;
    eph.e = static_cast<double>(
        mergeUnsignedFields(readBitsMsbFirst(sf2.data(), sf2.size(), 132, 10),
                            readBitsMsbFirst(sf2.data(), sf2.size(), 150, 22), 22)) * kPow2Neg33;
    eph.cus = static_cast<double>(
        readSignedBitsMsbFirst(sf2.data(), sf2.size(), 180, 18)) * kPow2Neg31;
    eph.crc = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf2.data(), sf2.size(), 198, 4),
                          readBitsMsbFirst(sf2.data(), sf2.size(), 210, 14), 14)) * kPow2Neg6;
    eph.crs = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf2.data(), sf2.size(), 224, 8),
                          readBitsMsbFirst(sf2.data(), sf2.size(), 240, 10), 10)) * kPow2Neg6;
    eph.sqrt_a = static_cast<double>(
        mergeUnsignedFields(readBitsMsbFirst(sf2.data(), sf2.size(), 250, 12),
                            readBitsMsbFirst(sf2.data(), sf2.size(), 270, 20), 20)) * kPow2Neg19;
    const uint32_t toe1 = readBitsMsbFirst(sf2.data(), sf2.size(), 290, 2);

    const uint32_t toe2 =
        mergeUnsignedFields(readBitsMsbFirst(sf3.data(), sf3.size(), 42, 10),
                            readBitsMsbFirst(sf3.data(), sf3.size(), 60, 5), 5);
    eph.i0 = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf3.data(), sf3.size(), 65, 17),
                          readBitsMsbFirst(sf3.data(), sf3.size(), 90, 15), 15)) *
             kPow2Neg31 * kSemiCircleToRadians;
    eph.cic = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf3.data(), sf3.size(), 105, 7),
                          readBitsMsbFirst(sf3.data(), sf3.size(), 120, 11), 11)) * kPow2Neg31;
    eph.omega_dot = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf3.data(), sf3.size(), 131, 11),
                          readBitsMsbFirst(sf3.data(), sf3.size(), 150, 13), 13)) *
                    kPow2Neg43 * kSemiCircleToRadians;
    eph.cis = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf3.data(), sf3.size(), 163, 9),
                          readBitsMsbFirst(sf3.data(), sf3.size(), 180, 9), 9)) * kPow2Neg31;
    eph.idot = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf3.data(), sf3.size(), 189, 13),
                          readBitsMsbFirst(sf3.data(), sf3.size(), 210, 1), 1)) *
               kPow2Neg43 * kSemiCircleToRadians;
    eph.i_dot = eph.idot;
    eph.omega0 = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf3.data(), sf3.size(), 211, 21),
                          readBitsMsbFirst(sf3.data(), sf3.size(), 240, 11), 11)) *
                 kPow2Neg31 * kSemiCircleToRadians;
    eph.omega = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(sf3.data(), sf3.size(), 251, 11),
                          readBitsMsbFirst(sf3.data(), sf3.size(), 270, 21), 21)) *
                kPow2Neg31 * kSemiCircleToRadians;
    eph.toes = static_cast<double>(mergeUnsignedFields(toe1, toe2, 15)) * 8.0;

    if (std::abs(toc_bdt - eph.toes) > 1e-6) {
        return false;
    }

    eph.tof = bdtWeekTowToGpst(week_bdt, static_cast<double>(sow1));
    int adjusted_week_bdt = week_bdt;
    if (eph.toes > static_cast<double>(sow1) + kHalfWeekSeconds) {
        ++adjusted_week_bdt;
    } else if (eph.toes < static_cast<double>(sow1) - kHalfWeekSeconds) {
        --adjusted_week_bdt;
    }
    eph.toe = bdtWeekTowToGpst(adjusted_week_bdt, eph.toes);
    eph.toc = bdtWeekTowToGpst(adjusted_week_bdt, toc_bdt);
    eph.week = static_cast<uint16_t>(adjusted_week_bdt);
    eph.valid = true;
    return true;
}

bool buildBeiDouD2Page(const libgnss::io::UBXSfrbx& sfrbx,
                       std::array<uint8_t, 38>& page,
                       int& page_id) {
    if (sfrbx.words.size() < 10 || sfrbx.sv_id > 5) {
        return false;
    }

    page.fill(0U);
    for (size_t word_index = 0; word_index < 10; ++word_index) {
        writeBitsMsbFirst(page, static_cast<int>(word_index * 30), 30,
                          sfrbx.words[word_index] & 0x3FFFFFFFU);
    }

    const int subframe_id = static_cast<int>(readBitsMsbFirst(page.data(), page.size(), 15, 3));
    if (subframe_id != 1) {
        return false;
    }

    page_id = static_cast<int>(readBitsMsbFirst(page.data(), page.size(), 42, 4));
    return page_id >= 1 && page_id <= 10;
}

bool decodeBeiDouD2Ephemeris(const libgnss::SatelliteId& satellite,
                             const std::array<std::array<uint8_t, 38>, 10>& pages,
                             libgnss::Ephemeris& eph) {
    const auto& p1 = pages[0];
    const auto& p3 = pages[2];
    const auto& p4 = pages[3];
    const auto& p5 = pages[4];
    const auto& p6 = pages[5];
    const auto& p7 = pages[6];
    const auto& p8 = pages[7];
    const auto& p9 = pages[8];
    const auto& p10 = pages[9];

    const int pgn1 = static_cast<int>(readBitsMsbFirst(p1.data(), p1.size(), 42, 4));
    const int pgn3 = static_cast<int>(readBitsMsbFirst(p3.data(), p3.size(), 42, 4));
    const int pgn4 = static_cast<int>(readBitsMsbFirst(p4.data(), p4.size(), 42, 4));
    const int pgn5 = static_cast<int>(readBitsMsbFirst(p5.data(), p5.size(), 42, 4));
    const int pgn6 = static_cast<int>(readBitsMsbFirst(p6.data(), p6.size(), 42, 4));
    const int pgn7 = static_cast<int>(readBitsMsbFirst(p7.data(), p7.size(), 42, 4));
    const int pgn8 = static_cast<int>(readBitsMsbFirst(p8.data(), p8.size(), 42, 4));
    const int pgn9 = static_cast<int>(readBitsMsbFirst(p9.data(), p9.size(), 42, 4));
    const int pgn10 = static_cast<int>(readBitsMsbFirst(p10.data(), p10.size(), 42, 4));
    if (pgn1 != 1 || pgn3 != 3 || pgn4 != 4 || pgn5 != 5 || pgn6 != 6 ||
        pgn7 != 7 || pgn8 != 8 || pgn9 != 9 || pgn10 != 10) {
        return false;
    }

    const uint32_t sow1 =
        mergeUnsignedFields(readBitsMsbFirst(p1.data(), p1.size(), 18, 8),
                            readBitsMsbFirst(p1.data(), p1.size(), 30, 12), 12);
    const uint32_t sow3 =
        mergeUnsignedFields(readBitsMsbFirst(p3.data(), p3.size(), 18, 8),
                            readBitsMsbFirst(p3.data(), p3.size(), 30, 12), 12);
    const uint32_t sow4 =
        mergeUnsignedFields(readBitsMsbFirst(p4.data(), p4.size(), 18, 8),
                            readBitsMsbFirst(p4.data(), p4.size(), 30, 12), 12);
    const uint32_t sow5 =
        mergeUnsignedFields(readBitsMsbFirst(p5.data(), p5.size(), 18, 8),
                            readBitsMsbFirst(p5.data(), p5.size(), 30, 12), 12);
    const uint32_t sow6 =
        mergeUnsignedFields(readBitsMsbFirst(p6.data(), p6.size(), 18, 8),
                            readBitsMsbFirst(p6.data(), p6.size(), 30, 12), 12);
    const uint32_t sow7 =
        mergeUnsignedFields(readBitsMsbFirst(p7.data(), p7.size(), 18, 8),
                            readBitsMsbFirst(p7.data(), p7.size(), 30, 12), 12);
    const uint32_t sow8 =
        mergeUnsignedFields(readBitsMsbFirst(p8.data(), p8.size(), 18, 8),
                            readBitsMsbFirst(p8.data(), p8.size(), 30, 12), 12);
    const uint32_t sow9 =
        mergeUnsignedFields(readBitsMsbFirst(p9.data(), p9.size(), 18, 8),
                            readBitsMsbFirst(p9.data(), p9.size(), 30, 12), 12);
    const uint32_t sow10 =
        mergeUnsignedFields(readBitsMsbFirst(p10.data(), p10.size(), 18, 8),
                            readBitsMsbFirst(p10.data(), p10.size(), 30, 12), 12);
    if (sow3 != sow1 + 6U || sow4 != sow3 + 3U || sow5 != sow4 + 3U ||
        sow6 != sow5 + 3U || sow7 != sow6 + 3U || sow8 != sow7 + 3U ||
        sow9 != sow8 + 3U || sow10 != sow9 + 3U) {
        return false;
    }

    eph = libgnss::Ephemeris{};
    eph.satellite = satellite;
    eph.health = static_cast<uint8_t>(readBitsMsbFirst(p1.data(), p1.size(), 46, 1));
    eph.sv_health = static_cast<double>(eph.health);
    eph.iodc = static_cast<uint16_t>(readBitsMsbFirst(p1.data(), p1.size(), 47, 5));
    eph.ura = static_cast<uint8_t>(readBitsMsbFirst(p1.data(), p1.size(), 60, 4));
    eph.sv_accuracy = static_cast<double>(eph.ura);
    const int week_bdt = static_cast<int>(readBitsMsbFirst(p1.data(), p1.size(), 64, 13));
    const double toc_bdt = static_cast<double>(
        mergeUnsignedFields(readBitsMsbFirst(p1.data(), p1.size(), 77, 5),
                            readBitsMsbFirst(p1.data(), p1.size(), 90, 12), 12)) * 8.0;
    eph.tgd = static_cast<double>(
        readSignedBitsMsbFirst(p1.data(), p1.size(), 102, 10)) * 0.1e-9;
    eph.tgd_secondary = static_cast<double>(
        readSignedBitsMsbFirst(p1.data(), p1.size(), 120, 10)) * 0.1e-9;

    eph.af0 = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(p3.data(), p3.size(), 100, 12),
                          readBitsMsbFirst(p3.data(), p3.size(), 120, 12), 12)) * kPow2Neg33;
    const int32_t f1p3 = readSignedBitsMsbFirst(p3.data(), p3.size(), 132, 4);

    const uint32_t f1p4 = mergeUnsignedFields(readBitsMsbFirst(p4.data(), p4.size(), 46, 6),
                                              readBitsMsbFirst(p4.data(), p4.size(), 60, 12), 12);
    eph.af2 = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(p4.data(), p4.size(), 72, 10),
                          readBitsMsbFirst(p4.data(), p4.size(), 90, 1), 1)) * kPow2Neg66;
    eph.iode = static_cast<uint16_t>(readBitsMsbFirst(p4.data(), p4.size(), 91, 5));
    eph.delta_n = static_cast<double>(
        readSignedBitsMsbFirst(p4.data(), p4.size(), 96, 16)) * kPow2Neg43 *
                  kSemiCircleToRadians;
    const int32_t cucp4 = readSignedBitsMsbFirst(p4.data(), p4.size(), 120, 14);

    const uint32_t cucp5 = readBitsMsbFirst(p5.data(), p5.size(), 46, 4);
    eph.m0 = static_cast<double>(
        mergeSignedFields3(readSignedBitsMsbFirst(p5.data(), p5.size(), 50, 2),
                           readBitsMsbFirst(p5.data(), p5.size(), 60, 22), 22,
                           readBitsMsbFirst(p5.data(), p5.size(), 90, 8), 8)) *
             kPow2Neg31 * kSemiCircleToRadians;
    eph.cus = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(p5.data(), p5.size(), 98, 14),
                          readBitsMsbFirst(p5.data(), p5.size(), 120, 4), 4)) * kPow2Neg31;
    const int32_t ep5 = readSignedBitsMsbFirst(p5.data(), p5.size(), 124, 10);

    const uint32_t ep6 = mergeUnsignedFields(readBitsMsbFirst(p6.data(), p6.size(), 46, 6),
                                             readBitsMsbFirst(p6.data(), p6.size(), 60, 16), 16);
    eph.sqrt_a = static_cast<double>(
        mergeUnsignedFields3(readBitsMsbFirst(p6.data(), p6.size(), 76, 6),
                             readBitsMsbFirst(p6.data(), p6.size(), 90, 22), 22,
                             readBitsMsbFirst(p6.data(), p6.size(), 120, 4), 4)) * kPow2Neg19;
    const int32_t cicp6 = readSignedBitsMsbFirst(p6.data(), p6.size(), 124, 10);

    const uint32_t cicp7 = mergeUnsignedFields(readBitsMsbFirst(p7.data(), p7.size(), 46, 6),
                                               readBitsMsbFirst(p7.data(), p7.size(), 60, 2), 2);
    eph.cis = static_cast<double>(
        readSignedBitsMsbFirst(p7.data(), p7.size(), 62, 18)) * kPow2Neg31;
    eph.toes = static_cast<double>(
        mergeUnsignedFields(readBitsMsbFirst(p7.data(), p7.size(), 80, 2),
                            readBitsMsbFirst(p7.data(), p7.size(), 90, 15), 15)) * 8.0;
    const int32_t i0p7 = mergeSignedFields(readSignedBitsMsbFirst(p7.data(), p7.size(), 105, 7),
                                           readBitsMsbFirst(p7.data(), p7.size(), 120, 14), 14);

    const uint32_t i0p8 = mergeUnsignedFields(readBitsMsbFirst(p8.data(), p8.size(), 46, 6),
                                              readBitsMsbFirst(p8.data(), p8.size(), 60, 5), 5);
    eph.crc = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(p8.data(), p8.size(), 65, 17),
                          readBitsMsbFirst(p8.data(), p8.size(), 90, 1), 1)) * kPow2Neg6;
    eph.crs = static_cast<double>(
        readSignedBitsMsbFirst(p8.data(), p8.size(), 91, 18)) * kPow2Neg6;
    const int32_t omgd_p8 = mergeSignedFields(readSignedBitsMsbFirst(p8.data(), p8.size(), 109, 3),
                                              readBitsMsbFirst(p8.data(), p8.size(), 120, 16), 16);

    const uint32_t omgd_p9 = readBitsMsbFirst(p9.data(), p9.size(), 46, 5);
    eph.omega0 = static_cast<double>(
        mergeSignedFields3(readSignedBitsMsbFirst(p9.data(), p9.size(), 51, 1),
                           readBitsMsbFirst(p9.data(), p9.size(), 60, 22), 22,
                           readBitsMsbFirst(p9.data(), p9.size(), 90, 9), 9)) *
                 kPow2Neg31 * kSemiCircleToRadians;
    const int32_t omg_p9 = mergeSignedFields(readSignedBitsMsbFirst(p9.data(), p9.size(), 99, 13),
                                             readBitsMsbFirst(p9.data(), p9.size(), 120, 14), 14);

    const uint32_t omg_p10 = readBitsMsbFirst(p10.data(), p10.size(), 46, 5);
    eph.idot = static_cast<double>(
        mergeSignedFields(readSignedBitsMsbFirst(p10.data(), p10.size(), 51, 1),
                          readBitsMsbFirst(p10.data(), p10.size(), 60, 13), 13)) *
               kPow2Neg43 * kSemiCircleToRadians;
    eph.i_dot = eph.idot;

    if (std::abs(toc_bdt - eph.toes) > 1e-6) {
        return false;
    }

    eph.af1 = static_cast<double>(mergeSignedFields(f1p3, f1p4, 18)) * kPow2Neg50;
    eph.cuc = static_cast<double>(mergeSignedFields(cucp4, cucp5, 4)) * kPow2Neg31;
    eph.e = static_cast<double>(mergeSignedFields(ep5, ep6, 22)) * kPow2Neg33;
    eph.cic = static_cast<double>(mergeSignedFields(cicp6, cicp7, 8)) * kPow2Neg31;
    eph.i0 = static_cast<double>(mergeSignedFields(i0p7, i0p8, 11)) * kPow2Neg31 *
             kSemiCircleToRadians;
    eph.omega_dot = static_cast<double>(mergeSignedFields(omgd_p8, omgd_p9, 5)) * kPow2Neg43 *
                    kSemiCircleToRadians;
    eph.omega = static_cast<double>(mergeSignedFields(omg_p9, omg_p10, 5)) * kPow2Neg31 *
                kSemiCircleToRadians;

    eph.tof = bdtWeekTowToGpst(week_bdt, static_cast<double>(sow1));
    int adjusted_week_bdt = week_bdt;
    if (eph.toes > static_cast<double>(sow1) + kHalfWeekSeconds) {
        ++adjusted_week_bdt;
    } else if (eph.toes < static_cast<double>(sow1) - kHalfWeekSeconds) {
        --adjusted_week_bdt;
    }
    eph.toe = bdtWeekTowToGpst(adjusted_week_bdt, eph.toes);
    eph.toc = bdtWeekTowToGpst(adjusted_week_bdt, toc_bdt);
    eph.week = static_cast<uint16_t>(adjusted_week_bdt);
    eph.valid = true;
    return true;
}

bool buildGalileoInavWord(const libgnss::io::UBXSfrbx& sfrbx,
                          std::array<uint8_t, 16>& word,
                          int& word_type) {
    if (sfrbx.words.size() < 8) {
        return false;
    }

    const auto bytes = rawWordsToBigEndianBytes(sfrbx.words, 8);
    const int part1 = static_cast<int>(readBitsMsbFirst(bytes.data(), 16U, 0, 1));
    const int page1 = static_cast<int>(readBitsMsbFirst(bytes.data(), 16U, 1, 1));
    const int part2 = static_cast<int>(readBitsMsbFirst(bytes.data() + 16, 16U, 0, 1));
    const int page2 = static_cast<int>(readBitsMsbFirst(bytes.data() + 16, 16U, 1, 1));
    if (page1 == 1 || page2 == 1 || part1 != 0 || part2 != 1) {
        return false;
    }

    word_type = static_cast<int>(readBitsMsbFirst(bytes.data(), bytes.size(), 2, 6));
    if (word_type < 0 || word_type > 5) {
        return false;
    }

    word.fill(0U);
    for (int i = 0; i < 14; ++i) {
        word[static_cast<size_t>(i)] =
            static_cast<uint8_t>(readBitsMsbFirst(bytes.data(), 16U, 2 + i * 8, 8));
    }
    for (int i = 0; i < 2; ++i) {
        word[static_cast<size_t>(14 + i)] =
            static_cast<uint8_t>(readBitsMsbFirst(bytes.data() + 16, 16U, 2 + i * 8, 8));
    }
    return true;
}

bool decodeGalileoInavEphemeris(const libgnss::SatelliteId& satellite,
                                const std::array<std::array<uint8_t, 16>, 6>& words,
                                libgnss::Ephemeris& eph) {
    std::array<uint8_t, 96> buffer{};
    for (size_t index = 0; index < words.size(); ++index) {
        std::copy(words[index].begin(), words[index].end(), buffer.begin() + index * 16U);
    }

    int bit = 0;
    const int type0 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 6));
    bit += 6;
    const int time_f = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 2));
    bit += 2 + 88;
    const int gst_week = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 12));
    bit += 12;
    const double tow = static_cast<double>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 20));

    bit = 128;
    const int type1 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 6));
    bit += 6;
    const int iod_nav0 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 10));
    bit += 10;
    const double toes = static_cast<double>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 14)) * 60.0;
    bit += 14;
    const double m0 = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 32)) *
                      kPow2Neg31 * kSemiCircleToRadians;
    bit += 32;
    const double e = static_cast<double>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 32)) * kPow2Neg33;
    bit += 32;
    const double sqrt_a = static_cast<double>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 32)) * kPow2Neg19;

    bit = 256;
    const int type2 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 6));
    bit += 6;
    const int iod_nav1 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 10));
    bit += 10;
    const double omega0 = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 32)) *
                          kPow2Neg31 * kSemiCircleToRadians;
    bit += 32;
    const double i0 = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 32)) *
                      kPow2Neg31 * kSemiCircleToRadians;
    bit += 32;
    const double omega = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 32)) *
                         kPow2Neg31 * kSemiCircleToRadians;
    bit += 32;
    const double idot = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 14)) *
                        kPow2Neg43 * kSemiCircleToRadians;

    bit = 384;
    const int type3 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 6));
    bit += 6;
    const int iod_nav2 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 10));
    bit += 10;
    const double omega_dot = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 24)) *
                             kPow2Neg43 * kSemiCircleToRadians;
    bit += 24;
    const double delta_n = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 16)) *
                           kPow2Neg43 * kSemiCircleToRadians;
    bit += 16;
    const double cuc = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 16)) *
                       kPow2Neg29;
    bit += 16;
    const double cus = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 16)) *
                       kPow2Neg29;
    bit += 16;
    const double crc = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 16)) *
                       kPow2Neg5;
    bit += 16;
    const double crs = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 16)) *
                       kPow2Neg5;
    bit += 16;
    const double sva = static_cast<double>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 8));

    bit = 512;
    const int type4 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 6));
    bit += 6;
    const int iod_nav3 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 10));
    bit += 10;
    const int svid = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 6));
    bit += 6;
    const double cic = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 16)) *
                       kPow2Neg29;
    bit += 16;
    const double cis = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 16)) *
                       kPow2Neg29;
    bit += 16;
    const double toc = static_cast<double>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 14)) * 60.0;
    bit += 14;
    const double af0 = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 31)) *
                       std::ldexp(1.0, -34);
    bit += 31;
    const double af1 = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 21)) *
                       std::ldexp(1.0, -46);
    bit += 21;
    const double af2 = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 6)) *
                       std::ldexp(1.0, -59);

    bit = 640;
    const int type5 = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 6));
    bit += 6 + 41;
    const double bgd_e5a_e1 = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 10)) *
                              std::ldexp(1.0, -32);
    bit += 10;
    const double bgd_e5b_e1 = static_cast<double>(readSignedBitsMsbFirst(buffer.data(), buffer.size(), bit, 10)) *
                              std::ldexp(1.0, -32);
    bit += 10;
    const int e5b_hs = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 2));
    bit += 2;
    const int e1b_hs = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 2));
    bit += 2;
    const int e5b_dvs = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 1));
    bit += 1;
    const int e1b_dvs = static_cast<int>(readBitsMsbFirst(buffer.data(), buffer.size(), bit, 1));

    if (type0 != 0 || type1 != 1 || type2 != 2 || type3 != 3 || type4 != 4 || type5 != 5) {
        return false;
    }
    if (time_f != 2 || iod_nav0 != iod_nav1 || iod_nav0 != iod_nav2 || iod_nav0 != iod_nav3) {
        return false;
    }
    if (svid != satellite.prn) {
        return false;
    }

    eph = libgnss::Ephemeris{};
    eph.satellite = satellite;
    eph.toes = toes;
    eph.m0 = m0;
    eph.e = e;
    eph.sqrt_a = sqrt_a;
    eph.omega0 = omega0;
    eph.i0 = i0;
    eph.omega = omega;
    eph.idot = idot;
    eph.i_dot = idot;
    eph.omega_dot = omega_dot;
    eph.delta_n = delta_n;
    eph.cuc = cuc;
    eph.cus = cus;
    eph.crc = crc;
    eph.crs = crs;
    eph.cic = cic;
    eph.cis = cis;
    eph.sv_accuracy = sva;
    eph.af0 = af0;
    eph.af1 = af1;
    eph.af2 = af2;
    eph.tgd = bgd_e5a_e1;
    eph.tgd_secondary = bgd_e5b_e1;
    eph.health = static_cast<uint8_t>((e5b_hs << 7) | (e5b_dvs << 6) | (e1b_hs << 1) | e1b_dvs);
    eph.sv_health = static_cast<double>(eph.health);
    eph.iode = static_cast<uint16_t>(iod_nav0);
    eph.iodc = static_cast<uint16_t>(iod_nav0);

    int absolute_week = gst_week + 1024;
    const double tt = toes - tow;
    if (tt > kHalfWeekSeconds) {
        --absolute_week;
    } else if (tt < -kHalfWeekSeconds) {
        ++absolute_week;
    }
    eph.tof = libgnss::GNSSTime(gst_week + 1024, tow);
    eph.toe = libgnss::GNSSTime(absolute_week, toes);
    eph.toc = libgnss::GNSSTime(absolute_week, toc);
    eph.week = static_cast<uint16_t>(absolute_week);
    eph.valid = true;
    return true;
}

struct UbxInputSource {
    bool serial = false;
    bool eof = false;
    std::ifstream file;
#ifndef _WIN32
    int serial_fd = -1;
#endif
};

std::string resolveSerialPath(const std::string& source) {
    constexpr const char* kPrefix = "serial://";
    if (source.rfind(kPrefix, 0) != 0) {
        return source;
    }

    std::string path = source.substr(std::char_traits<char>::length(kPrefix));
    const size_t query_pos = path.find('?');
    if (query_pos != std::string::npos) {
        path.resize(query_pos);
    }
    if (!path.empty() && path.front() != '/') {
        return path;
    }
    while (path.size() > 1 && path[0] == '/' && path[1] == '/') {
        path.erase(path.begin());
    }
    return path;
}

int parseSerialBaud(const std::string& source) {
    constexpr const char* kQuery = "?baud=";
    const size_t query_pos = source.find(kQuery);
    if (query_pos == std::string::npos) {
        return kDefaultSerialBaud;
    }
    const std::string baud_text =
        source.substr(query_pos + std::char_traits<char>::length(kQuery));
    if (baud_text.empty()) {
        return kDefaultSerialBaud;
    }
    return std::stoi(baud_text);
}

#ifndef _WIN32
speed_t baudRateConstant(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            throw std::invalid_argument("unsupported serial baud rate");
    }
}

bool configureSerialPort(int fd, int baud) {
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        return false;
    }
    cfmakeraw(&tio);
    const speed_t speed = baudRateConstant(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &tio) == 0;
}
#endif

void closeUbxInputSource(UbxInputSource& source) {
    if (source.file.is_open()) {
        source.file.close();
    }
#ifndef _WIN32
    if (source.serial_fd >= 0) {
        ::close(source.serial_fd);
        source.serial_fd = -1;
    }
#endif
    source.serial = false;
    source.eof = false;
}

bool openUbxInputSource(const std::string& input_path, UbxInputSource& source) {
    closeUbxInputSource(source);

    const std::string resolved_path = resolveSerialPath(input_path);
    std::error_code ec;
    const auto status = std::filesystem::status(resolved_path, ec);
    if (!ec && std::filesystem::is_regular_file(status)) {
        source.file.open(resolved_path, std::ios::binary);
        return source.file.is_open();
    }

#ifndef _WIN32
    const int baud = parseSerialBaud(input_path);
    const int fd = ::open(resolved_path.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        return false;
    }
    if (!configureSerialPort(fd, baud)) {
        ::close(fd);
        return false;
    }
    source.serial = true;
    source.serial_fd = fd;
    return true;
#else
    (void)input_path;
    return false;
#endif
}

bool readNextUbxEvents(libgnss::io::UBXStreamDecoder& decoder,
                       UbxInputSource& source,
                       std::vector<libgnss::io::UBXStreamDecoder::Event>& events) {
    events.clear();
    std::array<uint8_t, 4096> chunk{};

    while (true) {
        size_t bytes_read = 0;
        if (source.serial) {
#ifndef _WIN32
            const ssize_t count = ::read(source.serial_fd, chunk.data(), chunk.size());
            if (count < 0) {
                if (errno == EINTR) {
                    continue;
                }
                source.eof = true;
                return false;
            }
            if (count == 0) {
                source.eof = true;
                return false;
            }
            bytes_read = static_cast<size_t>(count);
#else
            source.eof = true;
            return false;
#endif
        } else {
            source.file.read(reinterpret_cast<char*>(chunk.data()),
                             static_cast<std::streamsize>(chunk.size()));
            bytes_read = static_cast<size_t>(source.file.gcount());
            if (bytes_read == 0) {
                source.eof = true;
                return false;
            }
        }

        decoder.pushBytes(chunk.data(), bytes_read, events);
        if (!events.empty()) {
            return true;
        }
    }
}

int runRTCMConversion(const ConvertConfig& config) {
    libgnss::io::RTCMReader reader;
    if (!reader.open(config.input_path)) {
        std::cerr << "Error: failed to open RTCM source: " << config.input_path << "\n";
        return 1;
    }

    libgnss::io::RTCMProcessor processor;
    libgnss::io::RINEXWriter obs_writer;
    libgnss::io::RINEXWriter nav_writer;
    bool obs_writer_open = false;
    bool nav_writer_open = false;
    size_t processed_messages = 0;
    size_t exported_obs_epochs = 0;
    size_t exported_nav_messages = 0;
    size_t skipped_nav_messages = 0;

    libgnss::io::RTCMMessage message;
    while ((config.limit == 0 || processed_messages < config.limit) && reader.readMessage(message)) {
        ++processed_messages;
        if (!config.quiet) {
            std::cout << processed_messages << " "
                      << libgnss::io::rtcm_utils::getMessageTypeName(message.type)
                      << " (" << static_cast<uint16_t>(message.type) << ")\n";
        }

        if (!config.obs_out_path.empty() &&
            libgnss::io::rtcm_utils::isObservationMessage(message.type)) {
            libgnss::ObservationData obs_data;
            if (processor.decodeObservationData(message, obs_data)) {
                if (!obs_writer_open) {
                    if (!obs_writer.createObservationFile(config.obs_out_path, makeObservationHeader())) {
                        std::cerr << "Error: failed to create observation RINEX: "
                                  << config.obs_out_path << "\n";
                        return 1;
                    }
                    obs_writer_open = true;
                }
                if (!obs_writer.writeObservationEpoch(obs_data)) {
                    std::cerr << "Error: failed to write observation epoch to "
                              << config.obs_out_path << "\n";
                    return 1;
                }
                ++exported_obs_epochs;
            }
        }

        if (!config.nav_out_path.empty() &&
            libgnss::io::rtcm_utils::isEphemerisMessage(message.type)) {
            libgnss::NavigationData nav_data;
            if (processor.decodeNavigationData(message, nav_data)) {
                if (!nav_writer_open) {
                    if (!nav_writer.createNavigationFile(config.nav_out_path, makeNavigationHeader())) {
                        std::cerr << "Error: failed to create navigation RINEX: "
                                  << config.nav_out_path << "\n";
                        return 1;
                    }
                    nav_writer_open = true;
                }
                for (const auto& [satellite, ephemerides] : nav_data.ephemeris_data) {
                    (void)satellite;
                    for (const auto& eph : ephemerides) {
                        if (nav_writer.writeNavigationMessage(eph)) {
                            ++exported_nav_messages;
                        } else {
                            ++skipped_nav_messages;
                        }
                    }
                }
            }
        }
    }

    if (obs_writer_open) {
        obs_writer.close();
    }
    if (nav_writer_open) {
        nav_writer.close();
    }
    reader.close();

    std::cout << "summary: processed_messages=" << processed_messages
              << " exported_obs_epochs=" << exported_obs_epochs
              << " exported_nav_messages=" << exported_nav_messages
              << " skipped_nav_messages=" << skipped_nav_messages << "\n";
    return 0;
}

int runUBXConversion(const ConvertConfig& config) {
    UbxInputSource input_source;
    if (!openUbxInputSource(config.input_path, input_source)) {
        std::cerr << "Error: failed to open UBX source: " << config.input_path << "\n";
        return 1;
    }
    libgnss::io::UBXStreamDecoder decoder;
    libgnss::io::RINEXWriter obs_writer;
    libgnss::io::RINEXWriter nav_writer;
    std::ofstream sfrbx_writer;
    bool obs_writer_open = false;
    bool nav_writer_open = false;
    bool sfrbx_writer_open = false;
    size_t processed_messages = 0;
    size_t exported_obs_epochs = 0;
    size_t exported_nav_messages = 0;
    size_t skipped_nav_messages = 0;
    size_t exported_sfrbx_messages = 0;
    std::vector<libgnss::io::UBXStreamDecoder::Event> events;
    std::map<libgnss::SatelliteId, GpsLnavFrameSet> gps_lnav_frames;
    std::map<libgnss::SatelliteId, GlonassFrameSet> glonass_frames;
    std::map<libgnss::SatelliteId, GalileoWordSet> galileo_words;
    std::map<libgnss::SatelliteId, BdsD1FrameSet> bds_d1_frames;
    std::map<libgnss::SatelliteId, BdsD2FrameSet> bds_d2_frames;
    std::map<libgnss::SatelliteId, libgnss::Ephemeris> last_exported_ephemeris;
    libgnss::GNSSTime nav_time_hint = currentGpstApprox();
    bool has_nav_time_hint = false;

    while ((config.limit == 0 || processed_messages < config.limit) &&
           readNextUbxEvents(decoder, input_source, events)) {
        for (const auto& event : events) {
            if (!event.has_message) {
                continue;
            }
            if (config.limit != 0 && processed_messages >= config.limit) {
                break;
            }
            ++processed_messages;
            if (!config.quiet) {
                std::cout << processed_messages << " "
                          << libgnss::io::ubx_utils::getMessageName(
                                 event.message.message_class, event.message.message_id)
                          << "\n";
            }

            if (!config.sfrbx_out_path.empty() && event.has_sfrbx) {
                if (!sfrbx_writer_open) {
                    sfrbx_writer.open(config.sfrbx_out_path, std::ios::out | std::ios::trunc);
                    if (!sfrbx_writer.is_open() || !writeSfrbxCsvHeader(sfrbx_writer)) {
                        std::cerr << "Error: failed to create SFRBX CSV: "
                                  << config.sfrbx_out_path << "\n";
                        closeUbxInputSource(input_source);
                        return 1;
                    }
                    sfrbx_writer_open = true;
                }
                if (!writeSfrbxCsvRow(sfrbx_writer, event.sfrbx)) {
                    std::cerr << "Error: failed to write SFRBX CSV row to "
                              << config.sfrbx_out_path << "\n";
                    closeUbxInputSource(input_source);
                    return 1;
                }
                ++exported_sfrbx_messages;
            }

            if (event.has_nav_pvt && event.nav_pvt.valid_time) {
                nav_time_hint = event.nav_pvt.time;
                has_nav_time_hint = true;
            }
            if (event.has_observation) {
                nav_time_hint = event.observation.time;
                has_nav_time_hint = true;
            }

            if (!config.nav_out_path.empty() && event.has_sfrbx &&
                (event.sfrbx.system == libgnss::GNSSSystem::GPS ||
                 event.sfrbx.system == libgnss::GNSSSystem::QZSS)) {
                const libgnss::SatelliteId satellite(event.sfrbx.system, event.sfrbx.sv_id);
                std::array<uint8_t, 30> subframe{};
                int subframe_id = 0;
                if (buildGpsLnavSubframe(event.sfrbx, subframe, subframe_id) &&
                    subframe_id >= 1 && subframe_id <= 3) {
                    auto& frame_set = gps_lnav_frames[satellite];
                    frame_set.subframes[static_cast<size_t>(subframe_id - 1)] = subframe;
                    frame_set.present[static_cast<size_t>(subframe_id - 1)] = true;

                    if (frame_set.present[0] && frame_set.present[1] && frame_set.present[2]) {
                        libgnss::Ephemeris ephemeris;
                        if (decodeGpsLnavEphemeris(satellite, frame_set.subframes, ephemeris)) {
                            const auto last_it = last_exported_ephemeris.find(ephemeris.satellite);
                            const bool duplicate =
                                last_it != last_exported_ephemeris.end() &&
                                last_it->second.iode == ephemeris.iode &&
                                last_it->second.iodc == ephemeris.iodc &&
                                std::abs(last_it->second.toe - ephemeris.toe) < 1e-6 &&
                                std::abs(last_it->second.toc - ephemeris.toc) < 1e-6;

                            if (!duplicate) {
                                if (!nav_writer_open) {
                                    if (!nav_writer.createNavigationFile(
                                            config.nav_out_path, makeNavigationHeader())) {
                                        std::cerr << "Error: failed to create navigation RINEX: "
                                                  << config.nav_out_path << "\n";
                                        closeUbxInputSource(input_source);
                                        return 1;
                                    }
                                    nav_writer_open = true;
                                }
                                if (!nav_writer.writeNavigationMessage(ephemeris)) {
                                    std::cerr << "Error: failed to write navigation message to "
                                              << config.nav_out_path << "\n";
                                    closeUbxInputSource(input_source);
                                    return 1;
                                }
                                last_exported_ephemeris[ephemeris.satellite] = ephemeris;
                                ++exported_nav_messages;
                            } else {
                                ++skipped_nav_messages;
                            }
                        } else {
                            ++skipped_nav_messages;
                        }
                    }
                } else {
                    ++skipped_nav_messages;
                }
            }

            if (!config.nav_out_path.empty() && event.has_sfrbx &&
                event.sfrbx.system == libgnss::GNSSSystem::GLONASS) {
                const libgnss::SatelliteId satellite(event.sfrbx.system, event.sfrbx.sv_id);
                std::array<uint8_t, 10> string_data{};
                std::array<uint8_t, 2> frame_id = {0U, 0U};
                int string_number = 0;
                if (buildGlonassNavigationString(event.sfrbx, string_data, frame_id, string_number)) {
                    auto& frame_set = glonass_frames[satellite];
                    if (frame_set.has_frame_id && frame_set.frame_id != frame_id) {
                        frame_set = GlonassFrameSet{};
                    }
                    frame_set.frame_id = frame_id;
                    frame_set.has_frame_id = true;
                    frame_set.strings[static_cast<size_t>(string_number - 1)] = string_data;
                    frame_set.present[static_cast<size_t>(string_number - 1)] = true;

                    if (std::all_of(frame_set.present.begin(), frame_set.present.end(),
                                    [](bool present) { return present; })) {
                        libgnss::Ephemeris ephemeris;
                        const libgnss::GNSSTime reference_time =
                            has_nav_time_hint ? nav_time_hint : currentGpstApprox();
                        if (decodeGlonassEphemeris(
                                satellite,
                                frame_set.strings,
                                reference_time,
                                static_cast<int>(event.sfrbx.frequency_id) - 7,
                                ephemeris)) {
                            const auto last_it = last_exported_ephemeris.find(ephemeris.satellite);
                            const bool duplicate =
                                last_it != last_exported_ephemeris.end() &&
                                last_it->second.iode == ephemeris.iode &&
                                last_it->second.glonass_frequency_channel ==
                                    ephemeris.glonass_frequency_channel &&
                                std::abs(last_it->second.toe - ephemeris.toe) < 1e-6 &&
                                std::abs(last_it->second.tof - ephemeris.tof) < 1e-6;

                            if (!duplicate) {
                                if (!nav_writer_open) {
                                    if (!nav_writer.createNavigationFile(
                                            config.nav_out_path, makeNavigationHeader())) {
                                        std::cerr << "Error: failed to create navigation RINEX: "
                                                  << config.nav_out_path << "\n";
                                        closeUbxInputSource(input_source);
                                        return 1;
                                    }
                                    nav_writer_open = true;
                                }
                                if (!nav_writer.writeNavigationMessage(ephemeris)) {
                                    std::cerr << "Error: failed to write navigation message to "
                                              << config.nav_out_path << "\n";
                                    closeUbxInputSource(input_source);
                                    return 1;
                                }
                                last_exported_ephemeris[ephemeris.satellite] = ephemeris;
                                ++exported_nav_messages;
                            } else {
                                ++skipped_nav_messages;
                            }
                        } else {
                            ++skipped_nav_messages;
                        }
                        frame_set = GlonassFrameSet{};
                    }
                } else {
                    ++skipped_nav_messages;
                }
            }

            if (!config.nav_out_path.empty() && event.has_sfrbx &&
                event.sfrbx.system == libgnss::GNSSSystem::Galileo) {
                const libgnss::SatelliteId satellite(event.sfrbx.system, event.sfrbx.sv_id);
                std::array<uint8_t, 16> word{};
                int word_type = -1;
                if (buildGalileoInavWord(event.sfrbx, word, word_type)) {
                    auto& word_set = galileo_words[satellite];
                    word_set.words[static_cast<size_t>(word_type)] = word;
                    word_set.present[static_cast<size_t>(word_type)] = true;

                    if (std::all_of(word_set.present.begin(), word_set.present.end(),
                                    [](bool present) { return present; })) {
                        libgnss::Ephemeris ephemeris;
                        if (decodeGalileoInavEphemeris(satellite, word_set.words, ephemeris)) {
                            const auto last_it = last_exported_ephemeris.find(ephemeris.satellite);
                            const bool duplicate =
                                last_it != last_exported_ephemeris.end() &&
                                last_it->second.iode == ephemeris.iode &&
                                std::abs(last_it->second.toe - ephemeris.toe) < 1e-6 &&
                                std::abs(last_it->second.toc - ephemeris.toc) < 1e-6;

                            if (!duplicate) {
                                if (!nav_writer_open) {
                                    if (!nav_writer.createNavigationFile(
                                            config.nav_out_path, makeNavigationHeader())) {
                                        std::cerr << "Error: failed to create navigation RINEX: "
                                                  << config.nav_out_path << "\n";
                                        closeUbxInputSource(input_source);
                                        return 1;
                                    }
                                    nav_writer_open = true;
                                }
                                if (!nav_writer.writeNavigationMessage(ephemeris)) {
                                    std::cerr << "Error: failed to write navigation message to "
                                              << config.nav_out_path << "\n";
                                    closeUbxInputSource(input_source);
                                    return 1;
                                }
                                last_exported_ephemeris[ephemeris.satellite] = ephemeris;
                                ++exported_nav_messages;
                            } else {
                                ++skipped_nav_messages;
                            }
                        } else {
                            ++skipped_nav_messages;
                        }
                    }
                } else {
                    ++skipped_nav_messages;
                }
            }

            if (!config.nav_out_path.empty() && event.has_sfrbx &&
                event.sfrbx.system == libgnss::GNSSSystem::BeiDou &&
                event.sfrbx.sv_id > 5) {
                const libgnss::SatelliteId satellite(event.sfrbx.system, event.sfrbx.sv_id);
                std::array<uint8_t, 38> subframe{};
                int subframe_id = 0;
                if (buildBeiDouD1Subframe(event.sfrbx, subframe, subframe_id) &&
                    subframe_id >= 1 && subframe_id <= 3) {
                    auto& frame_set = bds_d1_frames[satellite];
                    frame_set.subframes[static_cast<size_t>(subframe_id - 1)] = subframe;
                    frame_set.present[static_cast<size_t>(subframe_id - 1)] = true;

                    if (frame_set.present[0] && frame_set.present[1] && frame_set.present[2]) {
                        libgnss::Ephemeris ephemeris;
                        if (decodeBeiDouD1Ephemeris(satellite, frame_set.subframes, ephemeris)) {
                            const auto last_it = last_exported_ephemeris.find(ephemeris.satellite);
                            const bool duplicate =
                                last_it != last_exported_ephemeris.end() &&
                                last_it->second.iode == ephemeris.iode &&
                                last_it->second.iodc == ephemeris.iodc &&
                                std::abs(last_it->second.toe - ephemeris.toe) < 1e-6 &&
                                std::abs(last_it->second.toc - ephemeris.toc) < 1e-6;

                            if (!duplicate) {
                                if (!nav_writer_open) {
                                    if (!nav_writer.createNavigationFile(
                                            config.nav_out_path, makeNavigationHeader())) {
                                        std::cerr << "Error: failed to create navigation RINEX: "
                                                  << config.nav_out_path << "\n";
                                        closeUbxInputSource(input_source);
                                        return 1;
                                    }
                                    nav_writer_open = true;
                                }
                                if (!nav_writer.writeNavigationMessage(ephemeris)) {
                                    std::cerr << "Error: failed to write navigation message to "
                                              << config.nav_out_path << "\n";
                                    closeUbxInputSource(input_source);
                                    return 1;
                                }
                                last_exported_ephemeris[ephemeris.satellite] = ephemeris;
                                ++exported_nav_messages;
                            } else {
                                ++skipped_nav_messages;
                            }
                        } else {
                            ++skipped_nav_messages;
                        }
                    }
                } else {
                    ++skipped_nav_messages;
                }
            }

            if (!config.nav_out_path.empty() && event.has_sfrbx &&
                event.sfrbx.system == libgnss::GNSSSystem::BeiDou &&
                event.sfrbx.sv_id <= 5) {
                const libgnss::SatelliteId satellite(event.sfrbx.system, event.sfrbx.sv_id);
                std::array<uint8_t, 38> page{};
                int page_id = 0;
                if (buildBeiDouD2Page(event.sfrbx, page, page_id)) {
                    auto& frame_set = bds_d2_frames[satellite];
                    frame_set.pages[static_cast<size_t>(page_id - 1)] = page;
                    frame_set.present[static_cast<size_t>(page_id - 1)] = true;

                    const bool ready =
                        frame_set.present[0] && frame_set.present[2] && frame_set.present[3] &&
                        frame_set.present[4] && frame_set.present[5] && frame_set.present[6] &&
                        frame_set.present[7] && frame_set.present[8] && frame_set.present[9];
                    if (ready) {
                        libgnss::Ephemeris ephemeris;
                        if (decodeBeiDouD2Ephemeris(satellite, frame_set.pages, ephemeris)) {
                            const auto last_it = last_exported_ephemeris.find(ephemeris.satellite);
                            const bool duplicate =
                                last_it != last_exported_ephemeris.end() &&
                                last_it->second.iode == ephemeris.iode &&
                                last_it->second.iodc == ephemeris.iodc &&
                                std::abs(last_it->second.toe - ephemeris.toe) < 1e-6 &&
                                std::abs(last_it->second.toc - ephemeris.toc) < 1e-6;

                            if (!duplicate) {
                                if (!nav_writer_open) {
                                    if (!nav_writer.createNavigationFile(
                                            config.nav_out_path, makeNavigationHeader())) {
                                        std::cerr << "Error: failed to create navigation RINEX: "
                                                  << config.nav_out_path << "\n";
                                        closeUbxInputSource(input_source);
                                        return 1;
                                    }
                                    nav_writer_open = true;
                                }
                                if (!nav_writer.writeNavigationMessage(ephemeris)) {
                                    std::cerr << "Error: failed to write navigation message to "
                                              << config.nav_out_path << "\n";
                                    closeUbxInputSource(input_source);
                                    return 1;
                                }
                                last_exported_ephemeris[ephemeris.satellite] = ephemeris;
                                ++exported_nav_messages;
                            } else {
                                ++skipped_nav_messages;
                            }
                        } else {
                            ++skipped_nav_messages;
                        }
                    }
                } else {
                    ++skipped_nav_messages;
                }
            }

            if (config.obs_out_path.empty() || !event.has_observation) {
                continue;
            }

            if (!obs_writer_open) {
                auto header = makeObservationHeader();
                const auto& ubx_decoder = decoder.getDecoder();
                if (ubx_decoder.hasLastNavPVT()) {
                    const auto nav = ubx_decoder.getLastNavPVT();
                    if (nav.valid_position) {
                        header.approximate_position = nav.position_ecef;
                    }
                }
                if (!obs_writer.createObservationFile(config.obs_out_path, header)) {
                    std::cerr << "Error: failed to create observation RINEX: "
                              << config.obs_out_path << "\n";
                    closeUbxInputSource(input_source);
                    return 1;
                }
                obs_writer_open = true;
            }
            if (!obs_writer.writeObservationEpoch(event.observation)) {
                std::cerr << "Error: failed to write observation epoch to "
                          << config.obs_out_path << "\n";
                closeUbxInputSource(input_source);
                return 1;
            }
            ++exported_obs_epochs;
        }
    }

    if (obs_writer_open) {
        obs_writer.close();
    }
    if (nav_writer_open) {
        nav_writer.close();
    }
    if (sfrbx_writer_open) {
        sfrbx_writer.close();
    }
    closeUbxInputSource(input_source);

    const auto stats = decoder.getStats();
    std::cout << "summary: processed_messages=" << processed_messages
              << " valid_messages=" << stats.valid_messages
              << " checksum_errors=" << stats.checksum_errors
              << " exported_obs_epochs=" << exported_obs_epochs
              << " exported_nav_messages=" << exported_nav_messages
              << " skipped_nav_messages=" << skipped_nav_messages
              << " exported_sfrbx_messages=" << exported_sfrbx_messages << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const ConvertConfig config = parseArguments(argc, argv);
        if (config.format == InputFormat::RTCM) {
            return runRTCMConversion(config);
        }
        return runUBXConversion(config);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
