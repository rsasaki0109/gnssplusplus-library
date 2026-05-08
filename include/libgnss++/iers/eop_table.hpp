#pragma once

// libgnss++/iers/eop_table.hpp
//
// In-memory IERS Earth Orientation Parameter (EOP) series, loaded from
// an IERS 20 C04 file (daily samples at 0h UTC, ITRF2020-consistent).
//
// The canonical auth-free source is the Paris Observatory:
//   https://hpiers.obspm.fr/iers/eop/eopc04/eopc04.1962-now
//
// This is the Phase D-0 scaffolding piece of the IERS Conventions 2010
// integration: PPP itself does not yet consume the table — that wiring
// arrives in Phase D-1 (pole tide) and Phase D-2 (sub-daily EOP).

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
