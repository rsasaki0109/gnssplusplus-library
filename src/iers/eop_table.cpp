#include "libgnss++/iers/eop_table.hpp"

#include <algorithm>
#include <cmath>
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
