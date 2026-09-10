#pragma once

// Pair already-admitted factors only. Does not alter measurements, masks or
// factors, and cannot reintroduce rejected raw observations. Research utility.
#include <libgnss++/core/types.hpp>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace libgnss::tdcp_frequency {
struct FactorPair {
    std::size_t l1_index;
    std::size_t l5_index;
};

// Input factor container needs native TimeDifferencedCarrierFactor fields.
// Clock jump at either endpoint precludes pairing; unmatched factors remain
// caller-owned and unchanged. Ordering is deterministic by endpoint/system/PRN.
template<class Factors>
std::vector<FactorPair> pairAdmittedFactors(const Factors& factors,
                                           const std::vector<bool>& clock_jumps) {
    constexpr auto missing = std::numeric_limits<std::size_t>::max();
    using Key = std::tuple<std::size_t, std::size_t, GNSSSystem, unsigned>;
    std::map<Key, std::array<std::size_t, 2>> slots;
    for (std::size_t i=0; i<factors.size(); ++i) {
        const auto& f=factors[i];
        int band=-1;
        if (f.satellite.system==GNSSSystem::GPS) {
            if (f.signal==SignalType::GPS_L1CA) band=0;
            if (f.signal==SignalType::GPS_L5) band=1;
        } else if (f.satellite.system==GNSSSystem::Galileo) {
            if (f.signal==SignalType::GAL_E1) band=0;
            if (f.signal==SignalType::GAL_E5A) band=1;
        }
        if (band<0) continue;
        if (f.previous_epoch_index>=f.current_epoch_index ||
            f.current_epoch_index>=clock_jumps.size() || f.satellite.prn==0 ||
            !std::isfinite(f.dt_s) || f.dt_s<=0.0 ||
            !std::isfinite(f.sigma_m) || f.sigma_m<=0.0 ||
            !std::isfinite(f.delta_carrier_m) ||
            !f.previous_satellite_position_ecef.allFinite() ||
            !f.current_satellite_position_ecef.allFinite()) {
            throw std::invalid_argument("Invalid admitted TDCP pair candidate");
        }
        const Key key{f.previous_epoch_index,f.current_epoch_index,
                      f.satellite.system,f.satellite.prn};
        auto [it, inserted]=slots.emplace(key,std::array<std::size_t,2>{missing,missing});
        (void)inserted;
        if (it->second[band]!=missing) throw std::invalid_argument("Duplicate admitted TDCP band");
        it->second[band]=i;
    }
    std::vector<FactorPair> result;
    for (const auto& [key, indices]: slots) {
        if (indices[0]==missing || indices[1]==missing) continue;
        const auto& first=factors[indices[0]];
        const auto& second=factors[indices[1]];
        if (first.dt_s!=second.dt_s) throw std::invalid_argument("TDCP pair endpoint duration mismatch");
        if (first.current_epoch_index!=first.previous_epoch_index+1 ||
            first.dt_s>1.5 || clock_jumps[first.previous_epoch_index] ||
            clock_jumps[first.current_epoch_index]) continue;
        result.push_back({indices[0],indices[1]});
    }
    return result;
}
template<class Factors, class Epochs>
std::vector<FactorPair> pairAdmittedFactorsAtEpochs(const Factors& factors,
                                                  const std::vector<bool>& clock_jumps,
                                                  const Epochs& epochs) {
    if (clock_jumps.size()!=epochs.size()) {
        throw std::invalid_argument("TDCP pairing clock/epoch domain mismatch");
    }
    auto pairs=pairAdmittedFactors(factors,clock_jumps);
    for (const auto& pair:pairs) {
        const auto& f=factors[pair.l1_index];
        const double actual_dt=epochs[f.current_epoch_index].time-
                               epochs[f.previous_epoch_index].time;
        // Numerical consistency tolerance (1 ns), not a tunable timing offset.
        if (!std::isfinite(actual_dt) || actual_dt<=0.0 || actual_dt>1.5 ||
            std::abs(actual_dt-f.dt_s)>1e-9) {
            throw std::invalid_argument("TDCP pair duration disagrees with native epoch times");
        }
    }
    return pairs;
}
}  // namespace libgnss::tdcp_frequency
