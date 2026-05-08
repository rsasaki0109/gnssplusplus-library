#include "libgnss++/iers/eop_table.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace libgnss::iers {

namespace {

bool tryParseC04Row(const std::string& line, EopRecord& out) {
    if (line.empty() || line[0] == '#') {
        return false;
    }
    std::istringstream iss(line);
    int yr = 0, mm = 0, dd = 0, hh = 0;
    double mjd = 0.0, xp = 0.0, yp = 0.0, dut1 = 0.0;
    double dx = 0.0, dy = 0.0, xrt = 0.0, yrt = 0.0, lod = 0.0;
    if (!(iss >> yr >> mm >> dd >> hh >> mjd >> xp >> yp >> dut1
              >> dx >> dy >> xrt >> yrt >> lod)) {
        return false;
    }
    out.mjd_utc = mjd;
    out.xp_arcsec = xp;
    out.yp_arcsec = yp;
    out.ut1_minus_utc_seconds = dut1;
    out.lod_seconds = lod;
    out.dx_arcsec = dx;
    out.dy_arcsec = dy;
    return true;
}

// Fixed-width column offsets (0-indexed) for IERS Bulletin A
// `finals2000A.daily`. Lines are 187 chars; the value fields we
// consume are stable across the IERS-published release history.
constexpr std::size_t kBulletinAMinLineSize    = 78;
constexpr std::size_t kBulletinAPoleFlagCol    = 16;
constexpr std::size_t kBulletinAUt1FlagCol     = 57;
constexpr std::size_t kBulletinAMjdCol         = 7;
constexpr std::size_t kBulletinAMjdLen         = 8;
constexpr std::size_t kBulletinAXpCol          = 18;
constexpr std::size_t kBulletinAYpCol          = 37;
constexpr std::size_t kBulletinAUt1Col         = 58;
constexpr std::size_t kBulletinAFloatLen       = 10;

bool isBulletinAFlag(char c) {
    return c == 'I' || c == 'P' || c == ' ' || c == 'b';
}

bool tryParseBulletinARow(const std::string& line, EopRecord& out) {
    if (line.size() < kBulletinAMinLineSize) {
        return false;
    }
    const char pole_flag = line[kBulletinAPoleFlagCol];
    const char ut1_flag = line[kBulletinAUt1FlagCol];
    // Bulletin A uses `I` for IERS observed and `P` for predicted;
    // a blank/`b` flag indicates a missing value, in which case we
    // skip the row.
    if (pole_flag != 'I' && pole_flag != 'P') return false;
    if (ut1_flag != 'I' && ut1_flag != 'P') return false;

    out.mjd_utc = std::atof(line.substr(kBulletinAMjdCol, kBulletinAMjdLen).c_str());
    if (out.mjd_utc < 30000.0 || out.mjd_utc > 100000.0) {
        return false;
    }
    out.xp_arcsec  = std::atof(line.substr(kBulletinAXpCol,  kBulletinAFloatLen).c_str());
    out.yp_arcsec  = std::atof(line.substr(kBulletinAYpCol,  kBulletinAFloatLen).c_str());
    out.ut1_minus_utc_seconds =
        std::atof(line.substr(kBulletinAUt1Col, kBulletinAFloatLen).c_str());
    out.lod_seconds = 0.0;
    out.dx_arcsec   = 0.0;
    out.dy_arcsec   = 0.0;
    return true;
}

bool looksLikeBulletinA(const std::string& line) {
    // Bulletin A lines are 187 chars and have an `I`/`P`/blank flag
    // at column 17 (0-indexed 16). C04 lines are >= 100 chars but
    // have a digit, sign, or decimal point in that column.
    if (line.size() < kBulletinAMinLineSize) return false;
    return isBulletinAFlag(line[kBulletinAPoleFlagCol])
        && isBulletinAFlag(line[kBulletinAUt1FlagCol]);
}

}  // namespace

EopTable EopTable::fromC04File(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("EopTable::fromC04File: cannot open " + path);
    }
    std::vector<EopRecord> records;
    std::string line;
    while (std::getline(in, line)) {
        EopRecord rec{};
        if (tryParseC04Row(line, rec)) {
            records.push_back(rec);
        }
    }
    if (records.empty()) {
        throw std::runtime_error(
            "EopTable::fromC04File: no data rows parsed from " + path);
    }
    return EopTable(std::move(records));
}

EopTable EopTable::fromBulletinAFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error(
            "EopTable::fromBulletinAFile: cannot open " + path);
    }
    std::vector<EopRecord> records;
    std::string line;
    while (std::getline(in, line)) {
        EopRecord rec{};
        if (tryParseBulletinARow(line, rec)) {
            records.push_back(rec);
        }
    }
    if (records.empty()) {
        throw std::runtime_error(
            "EopTable::fromBulletinAFile: no data rows parsed from " + path);
    }
    return EopTable(std::move(records));
}

EopTable EopTable::fromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("EopTable::fromFile: cannot open " + path);
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (looksLikeBulletinA(line)) {
            return fromBulletinAFile(path);
        }
        // First non-comment line did not match Bulletin A — assume C04.
        return fromC04File(path);
    }
    throw std::runtime_error(
        "EopTable::fromFile: file is empty or contains only comments: " + path);
}

EopTable::EopTable(std::vector<EopRecord> records) : records_(std::move(records)) {
    if (records_.empty()) {
        throw std::invalid_argument("EopTable: must contain at least one record");
    }
    std::sort(records_.begin(), records_.end(),
              [](const EopRecord& a, const EopRecord& b) {
                  return a.mjd_utc < b.mjd_utc;
              });
}

EarthOrientationParams EopTable::interpolateAt(double mjd_utc) const {
    if (mjd_utc < records_.front().mjd_utc || mjd_utc > records_.back().mjd_utc) {
        std::ostringstream msg;
        msg << "EopTable::interpolateAt: mjd_utc " << mjd_utc
            << " outside coverage [" << records_.front().mjd_utc
            << ", " << records_.back().mjd_utc << "]";
        throw std::out_of_range(msg.str());
    }

    auto upper = std::lower_bound(
        records_.begin(), records_.end(), mjd_utc,
        [](const EopRecord& r, double v) { return r.mjd_utc < v; });

    if (upper == records_.begin() || upper->mjd_utc == mjd_utc) {
        const auto& r = *upper;
        return EarthOrientationParams{
            r.ut1_minus_utc_seconds, r.xp_arcsec, r.yp_arcsec};
    }
    const auto lower = upper - 1;
    const double t1 = lower->mjd_utc;
    const double t2 = upper->mjd_utc;
    const double w = (mjd_utc - t1) / (t2 - t1);

    EarthOrientationParams eop;
    eop.xp_arcsec = lower->xp_arcsec + w * (upper->xp_arcsec - lower->xp_arcsec);
    eop.yp_arcsec = lower->yp_arcsec + w * (upper->yp_arcsec - lower->yp_arcsec);

    // UT1-UTC has a 1-second discontinuity at a leap-second day. Detect
    // and snap to the nearest sample instead of interpolating across.
    const double ddut1 =
        upper->ut1_minus_utc_seconds - lower->ut1_minus_utc_seconds;
    if (std::abs(ddut1) > 0.5) {
        eop.ut1_minus_utc_seconds = (w < 0.5)
            ? lower->ut1_minus_utc_seconds
            : upper->ut1_minus_utc_seconds;
    } else {
        eop.ut1_minus_utc_seconds = lower->ut1_minus_utc_seconds + w * ddut1;
    }
    return eop;
}

double EopTable::firstMjd() const { return records_.front().mjd_utc; }
double EopTable::lastMjd() const { return records_.back().mjd_utc; }

}  // namespace libgnss::iers
