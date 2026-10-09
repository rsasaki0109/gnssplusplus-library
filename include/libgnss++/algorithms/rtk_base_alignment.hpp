#pragma once

/**
 * @file rtk_base_alignment.hpp
 * @brief Causal, geometry-corrected zero-order hold of a base observation epoch.
 *
 * Library counterpart of the "before" half of the batch
 * `interpolateBaseEpoch()` in apps/native/rtk_base_epoch_align.hpp. The
 * modeled base range (`calculateModeledBaseRange`) and the signal wavelength
 * rule mirror that header formula for formula; the batch header is left
 * untouched so the batch path's numerical behaviour cannot change. The hold
 * carries only a past base epoch forward, so it needs no future base epoch.
 *
 *   modeled(t)  = |sat(t - tau) - base| + Saastamoinen(base, el)
 *   P_target    = modeled(t_target) + (P_base - modeled(t_base))
 *   L_target*l  = modeled(t_target) + (L_base*l - modeled(t_base))
 *                 (only without LLI bit 0 and without loss of lock)
 *
 * See docs/online_rtk_base_extrapolation_v1.md.
 */

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

namespace libgnss {
namespace rtk_base_alignment {

/** Elevation floor [rad] below which a satellite's modeled range is rejected. */
constexpr double kMinModeledElevationRad = 0.05;

/** Carrier wavelength [m] of a signal (GLONASS channel from `nav`); 0 if unknown. */
double signalWavelength(const SatelliteId& satellite, SignalType signal,
                        const GNSSTime& time, const NavigationData& nav);

/**
 * Modeled geometric range plus Saastamoinen troposphere from the base to a
 * satellite for an epoch `time` of base reception. `approx_pseudorange` only
 * sets the signal travel time (0.075 s if not > 1 m). False when the
 * ephemeris is missing or the elevation is <= kMinModeledElevationRad.
 */
bool calculateModeledBaseRange(const SatelliteId& satellite, const GNSSTime& time,
                               double approx_pseudorange, const Vector3d& base_position,
                               const NavigationData& nav, double& modeled_range);

/**
 * Hold `base` (a past epoch) to `target_time` >= base.time. Returns false (and
 * leaves `held` unspecified) when the age target_time - base.time is outside
 * [0, max_age_s] (1e-6 s tolerance on the upper bound) or when no signal
 * survives. Satellites whose modeled range fails are omitted. Doppler, SNR,
 * LLI, code and the remaining observation fields are copied from `base`.
 */
bool holdBaseEpoch(const ObservationData& base, const GNSSTime& target_time,
                   const Vector3d& base_position, const NavigationData& nav,
                   double max_age_s, ObservationData& held);

} // namespace rtk_base_alignment
} // namespace libgnss
