#pragma once

#include "../core/observation.hpp"

#include <vector>

namespace libgnss {
namespace rtk_selection {

struct SatelliteSelectionData {
    SatelliteId satellite;
    bool has_l1 = false;
    bool has_l2 = false;
    double l1_wavelength = 0.0;
    double l2_wavelength = 0.0;
    double elevation = 0.0;
    bool n1_active = false;
    bool n2_active = false;
    int lock_count_l1 = 0;
    int lock_count_l2 = 0;
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

std::vector<SelectionPair> buildDoubleDifferencePairsForSystem(
    const std::vector<SatelliteSelectionData>& satellites,
    GNSSSystem system,
    int min_lock_count,
    bool require_matched_carrier_wavelength);

}  // namespace rtk_selection
}  // namespace libgnss
