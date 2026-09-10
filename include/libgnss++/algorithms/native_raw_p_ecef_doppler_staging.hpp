#pragma once

// Exact same-run transfer contract for the optional Phase171 ECEF-D stage.
// The temporary builder may select a different D row subset, but this helper
// transfers only D rows into the authoritative problem.  P/TDCP rows, epoch
// seeds, and their ordering remain owned by the original problem.

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/source_clock_c0d_initializer.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace libgnss::raw_p_ecef_doppler {

using EpochSeed = FGOProcessor::EpochSeed;
using DopplerFactor = FGOProcessor::UndifferencedDopplerFactor;
using RawEpochIdentity = std::pair<std::size_t, std::int64_t>;

inline bool validRawEpochIdentity(const EpochSeed& epoch) {
    return epoch.raw_source_index != std::numeric_limits<std::size_t>::max() &&
           epoch.raw_utc_time_millis >= 0 &&
           source_clock_c0d::validEpochTime(epoch.time);
}

inline bool validCorrectedEcefDopplerRow(const DopplerFactor& factor) {
    const double los_norm = factor.los.norm();
    return factor.los.allFinite() && std::isfinite(los_norm) &&
           los_norm > 0.0 && std::abs(los_norm - 1.0) <= 1.0e-6 &&
           std::isfinite(factor.residual_mps) &&
           std::isfinite(factor.sigma_mps) && factor.sigma_mps > 0.0 &&
           factor.source_satellite_state_available &&
           factor.source_satellite_position_ecef.allFinite() &&
           factor.source_satellite_velocity_ecef.allFinite() &&
           factor.satellite_position_ecef.allFinite() &&
           factor.satellite_velocity_ecef.allFinite() &&
           std::isfinite(factor.measured_range_rate_mps) &&
           std::isfinite(factor.satellite_clock_drift_mps) &&
           factor.includes_receiver_clock_drift &&
           factor.uses_rotated_satellite_state;
}

// Remap a temporary raw-D builder result to the original retained epoch
// identities.  This function deliberately has no access to P/TDCP vectors or
// files, so a successful transfer cannot replace those authoritative rows.
inline bool remapDopplerFactors(
    const std::vector<EpochSeed>& original_epochs,
    const std::vector<EpochSeed>& staged_epochs,
    const std::vector<DopplerFactor>& staged_factors,
    std::vector<DopplerFactor>& remapped_factors,
    std::string& failure) {
    remapped_factors.clear();
    failure.clear();

    std::map<RawEpochIdentity, std::size_t> original_indices;
    for (std::size_t index = 0; index < original_epochs.size(); ++index) {
        const auto& epoch = original_epochs[index];
        if (!validRawEpochIdentity(epoch)) {
            failure = "invalid original raw epoch identity";
            return false;
        }
        const RawEpochIdentity identity = {
            epoch.raw_source_index, epoch.raw_utc_time_millis};
        if (!original_indices.emplace(identity, index).second) {
            failure = "duplicate original raw epoch identity";
            return false;
        }
    }

    std::vector<std::size_t> staged_to_original(
        staged_epochs.size(), std::numeric_limits<std::size_t>::max());
    std::set<std::size_t> mapped_original_epochs;
    for (std::size_t staged_index = 0; staged_index < staged_epochs.size();
         ++staged_index) {
        const auto& staged = staged_epochs[staged_index];
        if (!validRawEpochIdentity(staged)) {
            failure = "invalid staged raw epoch identity";
            return false;
        }
        const RawEpochIdentity identity = {
            staged.raw_source_index, staged.raw_utc_time_millis};
        const auto original = original_indices.find(identity);
        if (original == original_indices.end() ||
            !source_clock_c0d::strictEpochTimeEqual(
                staged.time, original_epochs[original->second].time)) {
            failure = "staged raw epoch identity/time mismatch";
            return false;
        }
        if (!mapped_original_epochs.insert(original->second).second) {
            failure = "duplicate staged raw epoch identity";
            return false;
        }
        staged_to_original[staged_index] = original->second;
    }

    std::set<std::tuple<std::size_t, int, int, int>> copied_keys;
    std::vector<DopplerFactor> candidate;
    candidate.reserve(staged_factors.size());
    for (const auto& input : staged_factors) {
        if (input.epoch_index >= staged_to_original.size() ||
            !validCorrectedEcefDopplerRow(input)) {
            failure = "invalid staged corrected ECEF Doppler row";
            return false;
        }
        const std::size_t original_epoch = staged_to_original[input.epoch_index];
        const auto key = std::make_tuple(
            original_epoch, static_cast<int>(input.satellite.system),
            static_cast<int>(input.satellite.prn),
            static_cast<int>(input.signal));
        if (!copied_keys.insert(key).second) {
            failure = "duplicate staged corrected ECEF Doppler identity";
            return false;
        }
        DopplerFactor output = input;
        output.epoch_index = original_epoch;
        if (output.previous_epoch_index !=
            std::numeric_limits<std::size_t>::max()) {
            if (output.previous_epoch_index >= staged_to_original.size() ||
                staged_to_original[output.previous_epoch_index] ==
                    std::numeric_limits<std::size_t>::max()) {
                failure = "staged previous-D epoch identity mismatch";
                return false;
            }
            output.previous_epoch_index =
                staged_to_original[output.previous_epoch_index];
        }
        candidate.push_back(std::move(output));
    }
    remapped_factors = std::move(candidate);
    return true;
}

}  // namespace libgnss::raw_p_ecef_doppler
