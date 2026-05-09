#pragma once

// libgnss++/iers/eop_table.hpp
//
// In-memory IERS Earth Orientation Parameter (EOP) series. Two
// auth-free upstream formats are supported:
//
//   1. IERS 20 C04 daily series, IAU 2000 / ITRF2020-consistent.
//      Source: https://hpiers.obspm.fr/iers/eop/eopc04/eopc04.1962-now
//      Final values; published with ~1 week lag relative to the epoch.
//
//   2. IERS Bulletin A (USNO `finals2000A.daily`).
//      Source: https://maia.usno.navy.mil/ser7/finals2000A.daily
//      Combined observed (`I` flag) + predicted (`P` flag) rows;
//      predictions extend ~12 months past the last observed epoch,
//      filling the C04 publication-lag gap.
//
// `EopTable::fromFile()` auto-detects which format is on disk and
// dispatches to the appropriate parser. Callers that want to be
// explicit can use the format-specific factories.

#include <cstddef>
#include <string>
#include <vector>

#include "libgnss++/iers/earth_rotation.hpp"

namespace libgnss::iers {

/// @brief A single daily row from an IERS 20 C04 series.
///
/// xp/yp/dx/dy are arcseconds, ut1_minus_utc and lod are seconds.
/// Rates and formal errors from the source file are dropped at parse
/// time — libgnss::iers consumers only need the value columns.
struct EopRecord {
    double mjd_utc{0.0};
    double xp_arcsec{0.0};
    double yp_arcsec{0.0};
    double ut1_minus_utc_seconds{0.0};
    double lod_seconds{0.0};
    double dx_arcsec{0.0};
    double dy_arcsec{0.0};
};

/// @brief Daily-sampled EOP series with linear interpolation.
///
/// Linear interpolation between adjacent daily samples is well below
/// the cm-level PPP requirement during periods covered by the final
/// EOP series. Around a UT1-UTC leap-second insertion the ut1_minus_utc
/// column has a 1-second step at 0h UTC; interpolateAt() detects this
/// and snaps to the nearer sample for ut1_minus_utc only (xp/yp are
/// continuous and are still interpolated).
///
/// For epochs outside [firstMjd(), lastMjd()], interpolateAt() throws
/// std::out_of_range — callers are expected to keep the EOP file
/// current. The C04 series is updated daily.
class EopTable {
public:
    /// @brief Parse an IERS C04 fixed-width file. Comment lines (`#`) are skipped.
    static EopTable fromC04File(const std::string& path);

    /// @brief Parse an IERS Bulletin A `finals2000A.daily` file.
    ///
    /// Per-row format (187-char fixed-width): year, month, day, MJD,
    /// `I`/`P` flag, polar motion (xp, yp ± errors), `I`/`P` flag,
    /// UT1-UTC ± error, LOD ± error, nutation flags + dX/dY...
    /// libgnss::iers consumers only need MJD + xp + yp + UT1-UTC, so
    /// LOD / dX / dY are dropped at parse time.
    ///
    /// Both observed (`I`) and predicted (`P`) rows are kept — the
    /// prediction extension is the value-add over IERS 20 C04 here.
    static EopTable fromBulletinAFile(const std::string& path);

    /// @brief Auto-detect format and dispatch.
    ///
    /// Detection rule: if the first non-`#` non-empty data row has
    /// an `I`/`P`/`b` flag character at column 17 (1-indexed), the
    /// file is treated as Bulletin A; otherwise it is parsed as C04.
    /// The two formats are unambiguous in practice — C04 has all
    /// numeric columns where Bulletin A has its flag character.
    static EopTable fromFile(const std::string& path);

    /// @brief Construct directly from a list of records (used by tests).
    /// Records do not need to be pre-sorted.
    explicit EopTable(std::vector<EopRecord> records);

    /// @brief Interpolate the series at the given UTC MJD.
    EarthOrientationParams interpolateAt(double mjd_utc) const;

    double firstMjd() const;
    double lastMjd() const;
    std::size_t size() const { return records_.size(); }

    const std::vector<EopRecord>& records() const { return records_; }

private:
    std::vector<EopRecord> records_;  // sorted ascending by mjd_utc
};

}  // namespace libgnss::iers
