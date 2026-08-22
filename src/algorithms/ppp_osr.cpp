#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/ppp_bias_identity.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>


#include "ppp_osr_internal.hpp"

namespace libgnss {

const char* clasPhaseContinuityPolicyName(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR:
            return "full-repair";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::SIS_CONTINUITY_ONLY:
            return "sis-continuity-only";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::REPAIR_ONLY:
            return "repair-only";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::RAW_PHASE_BIAS:
            return "raw-phase-bias";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::NO_PHASE_BIAS:
            return "no-phase-bias";
    }
    return "full-repair";
}

const char* clasPhaseBiasValuePolicyName(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::FULL:
            return "full";
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::PHASE_BIAS_ONLY:
            return "phase-bias-only";
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::COMPENSATION_ONLY:
            return "compensation-only";
    }
    return "full";
}

const char* clasPhaseBiasReferenceTimePolicyName(
    ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::PHASE_BIAS_REFERENCE:
            return "phase-bias-reference";
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::CLOCK_REFERENCE:
            return "clock-reference";
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::OBSERVATION_EPOCH:
            return "observation-epoch";
    }
    return "phase-bias-reference";
}

const char* clasSsrTimingPolicyName(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::LAG_TOLERANT:
            return "lag-tolerant";
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_PHASE_BIAS:
            return "clock-bound-phase-bias";
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS:
            return "clock-bound-atmos-and-phase-bias";
    }
    return "lag-tolerant";
}

const char* clasExpandedValueConstructionPolicyName(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::FULL_COMPOSED:
            return "full-composed";
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::RESIDUAL_ONLY:
            return "residual-only";
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::POLYNOMIAL_ONLY:
            return "polynomial-only";
    }
    return "full-composed";
}

bool usesClasPhaseBiasTerms(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::NO_PHASE_BIAS;
}

bool usesClasRawPhaseBiasValues(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::COMPENSATION_ONLY;
}

bool usesClasPhaseCompensationValues(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::PHASE_BIAS_ONLY;
}

GNSSTime selectClasPhaseBiasReferenceTime(
    ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy policy,
    const GNSSTime& phase_bias_reference_time,
    const GNSSTime& clock_reference_time,
    const GNSSTime& observation_time) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::PHASE_BIAS_REFERENCE:
            return phase_bias_reference_time;
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::CLOCK_REFERENCE:
            return gnsstimeIsSet(clock_reference_time) ? clock_reference_time
                                                       : phase_bias_reference_time;
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::OBSERVATION_EPOCH:
            return observation_time;
    }
    return phase_bias_reference_time;
}

bool usesClasSisContinuity(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR ||
           policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::SIS_CONTINUITY_ONLY;
}

void updateSisContinuity(
    CLASSisContinuityInfo& info,
    const OSRCorrection& osr,
    bool clock_time_valid) {
    const double current_sis_m = -osr.clock_correction_m + osr.orbit_projection_m;
    if (!clock_time_valid) {
        // Preserve the CLASLIB-style boundary-held delta (gated feature)
        // across transient clock-reference-time gaps: CLASLIB's satcorr[]
        // state (currtow/currsis) is keyed to message reception and outlives
        // a few epochs of missing per-epoch SSR data, so a brief gap inside
        // the 15s hold window (observed on real CLAS data) must not drop the
        // already-captured boundary delta. The rest of the continuity state
        // (current/previous SIS samples, last_delta_m) legitimately resets,
        // matching pre-existing (gate-OFF) behavior.
        const GNSSTime preserved_boundary_time = info.boundary_time;
        const double preserved_boundary_delta_m = info.boundary_delta_m;
        const bool preserved_has_boundary_delta = info.has_boundary_delta;
        const GNSSTime preserved_boundary_prev_time = info.boundary_prev_time;
        const double preserved_boundary_prev_sis_m = info.boundary_prev_sis_m;
        const bool preserved_has_boundary_prev_sis = info.has_boundary_prev_sis;
        const int preserved_boundary_prev_iode = info.boundary_prev_iode;
        const bool preserved_has_boundary_prev_iode = info.has_boundary_prev_iode;
        info = CLASSisContinuityInfo{};
        info.boundary_time = preserved_boundary_time;
        info.boundary_delta_m = preserved_boundary_delta_m;
        info.has_boundary_delta = preserved_has_boundary_delta;
        info.boundary_prev_time = preserved_boundary_prev_time;
        info.boundary_prev_sis_m = preserved_boundary_prev_sis_m;
        info.has_boundary_prev_sis = preserved_has_boundary_prev_sis;
        info.boundary_prev_iode = preserved_boundary_prev_iode;
        info.has_boundary_prev_iode = preserved_has_boundary_prev_iode;
    } else if (!info.has_current) {
        info.current_time = osr.clock_reference_time;
        info.current_sis_m = current_sis_m;
        info.has_current = true;
    } else if (info.current_time != osr.clock_reference_time) {
        const double dt_clock = osr.clock_reference_time - info.current_time;
        info.previous_time = info.current_time;
        info.previous_sis_m = info.current_sis_m;
        info.current_time = osr.clock_reference_time;
        info.current_sis_m = current_sis_m;
        info.has_previous = true;
        if (std::abs(dt_clock - kSsrClockIntervalSeconds) < 0.5) {
            info.last_delta_m = info.current_sis_m - info.previous_sis_m;
            info.has_last_delta = true;
        } else {
            info.last_delta_m = 0.0;
            info.has_last_delta = false;
        }
    }
}

}  // namespace libgnss
