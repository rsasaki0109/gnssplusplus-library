#include <libgnss++/algorithms/rtk_selection.hpp>

#include <cmath>

namespace libgnss {
namespace rtk_selection {

bool selectSystemReferenceSatellite(const std::vector<SatelliteSelectionData>& satellites,
                                    GNSSSystem system,
                                    int min_lock_count,
                                    SatelliteId& ref_sat) {
    SatelliteId best_dual;
    SatelliteId best_l1;
    double best_dual_el = -1.0;
    double best_l1_el = -1.0;

    for (const auto& satellite : satellites) {
        if (satellite.satellite.system != system || !satellite.has_l1 || !satellite.n1_active) {
            continue;
        }
        if (min_lock_count > 0 && satellite.lock_count_l1 < min_lock_count) {
            continue;
        }

        if (satellite.elevation > best_l1_el) {
            best_l1_el = satellite.elevation;
            best_l1 = satellite.satellite;
        }

        if (!satellite.has_l2 || !satellite.n2_active) {
            continue;
        }
        if (min_lock_count > 0 && satellite.lock_count_l2 < min_lock_count) {
            continue;
        }
        if (satellite.elevation > best_dual_el) {
            best_dual_el = satellite.elevation;
            best_dual = satellite.satellite;
        }
    }

    if (best_dual_el >= 0.0) {
        ref_sat = best_dual;
        return true;
    }
    if (best_l1_el >= 0.0) {
        ref_sat = best_l1;
        return true;
    }
    return false;
}

std::vector<SelectionPair> buildDoubleDifferencePairsForSystem(
    const std::vector<SatelliteSelectionData>& satellites,
    GNSSSystem system,
    int min_lock_count,
    bool require_matched_carrier_wavelength) {
    std::vector<SelectionPair> pairs;

    SatelliteId ref_sat;
    if (!selectSystemReferenceSatellite(satellites, system, min_lock_count, ref_sat)) {
        return pairs;
    }

    const SatelliteSelectionData* ref_data = nullptr;
    for (const auto& satellite : satellites) {
        if (satellite.satellite == ref_sat) {
            ref_data = &satellite;
            break;
        }
    }
    if (ref_data == nullptr) {
        return pairs;
    }

    if (ref_data->has_l1 && ref_data->n1_active) {
        for (const auto& satellite : satellites) {
            if (satellite.satellite.system != system || satellite.satellite == ref_sat) {
                continue;
            }
            if (!satellite.has_l1 || !satellite.n1_active) {
                continue;
            }
            if (min_lock_count > 0 && satellite.lock_count_l1 < min_lock_count) {
                continue;
            }
            if (require_matched_carrier_wavelength &&
                std::abs(satellite.l1_wavelength - ref_data->l1_wavelength) > 1e-6) {
                continue;
            }
            pairs.push_back({ref_sat, satellite.satellite, 0});
        }
    }

    if (ref_data->has_l2 && ref_data->n2_active) {
        for (const auto& satellite : satellites) {
            if (satellite.satellite.system != system || satellite.satellite == ref_sat) {
                continue;
            }
            if (!satellite.has_l2 || !satellite.n2_active) {
                continue;
            }
            if (min_lock_count > 0 && satellite.lock_count_l2 < min_lock_count) {
                continue;
            }
            if (require_matched_carrier_wavelength &&
                std::abs(satellite.l2_wavelength - ref_data->l2_wavelength) > 1e-6) {
                continue;
            }
            pairs.push_back({ref_sat, satellite.satellite, 1});
        }
    }

    // Phase 18 Step 4: emit L5 (freq=2) pairs when ref sat carries an active L5 ambiguity.
    // Backward-compat: when L5 collection is disabled (Step 3 default), has_l5/n5_active stay
    // false on every snapshot entry, so this block is a no-op.
    if (ref_data->has_l5 && ref_data->n5_active) {
        for (const auto& satellite : satellites) {
            if (satellite.satellite.system != system || satellite.satellite == ref_sat) {
                continue;
            }
            if (!satellite.has_l5 || !satellite.n5_active) {
                continue;
            }
            if (min_lock_count > 0 && satellite.lock_count_l5 < min_lock_count) {
                continue;
            }
            if (require_matched_carrier_wavelength &&
                std::abs(satellite.l5_wavelength - ref_data->l5_wavelength) > 1e-6) {
                continue;
            }
            pairs.push_back({ref_sat, satellite.satellite, 2});
        }
    }

    return pairs;
}

}  // namespace rtk_selection
}  // namespace libgnss
