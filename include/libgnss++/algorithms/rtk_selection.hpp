#pragma once

#include "../core/observation.hpp"

#include <vector>

namespace libgnss {
namespace rtk_selection {

struct SatelliteSelectionData {
    SatelliteId satellite;
    bool has_l1 = false;
    bool has_l2 = false;
    bool has_l5 = false;  // Phase 18 Step 4
    double l1_wavelength = 0.0;
    double l2_wavelength = 0.0;
    double l5_wavelength = 0.0;  // Phase 18 Step 4
    double elevation = 0.0;
    bool n1_active = false;
    bool n2_active = false;
    bool n5_active = false;  // Phase 18 Step 4
    int lock_count_l1 = 0;
    int lock_count_l2 = 0;
    int lock_count_l5 = 0;  // Phase 18 Step 4
};

struct SelectionPair {
    SatelliteId ref_sat;
    SatelliteId sat;
    int freq = 0;
};

bool selectSystemReferenceSatellite(const std::vector<SatelliteSelectionData>& satellites,
                                    GNSSSystem system,
                                    int min_lock_count,
                                    SatelliteId& ref_sat);

// forced_ref_sat: optional override for the DD reference satellite
// (RTKProcessor::RTKConfig::cmc_aware_reference_selection: the caller has
// already picked a hysteresis-gated reference for this (system) group and
// wants every DD pair formed against it, instead of the plain highest-
// elevation pick selectSystemReferenceSatellite() would make). When null
// (the default via the 4-argument overload below) or when the referenced
// satellite is not present/eligible (system mismatch, no active L1
// ambiguity) in `satellites`, falls back to selectSystemReferenceSatellite()
// exactly as before -- existing callers that never pass this argument are
// unaffected, bit for bit.
std::vector<SelectionPair> buildDoubleDifferencePairsForSystem(
    const std::vector<SatelliteSelectionData>& satellites,
    GNSSSystem system,
    int min_lock_count,
    bool require_matched_carrier_wavelength,
    const SatelliteId* forced_ref_sat);

inline std::vector<SelectionPair> buildDoubleDifferencePairsForSystem(
    const std::vector<SatelliteSelectionData>& satellites,
    GNSSSystem system,
    int min_lock_count,
    bool require_matched_carrier_wavelength) {
    return buildDoubleDifferencePairsForSystem(
        satellites, system, min_lock_count, require_matched_carrier_wavelength, nullptr);
}

}  // namespace rtk_selection
}  // namespace libgnss
