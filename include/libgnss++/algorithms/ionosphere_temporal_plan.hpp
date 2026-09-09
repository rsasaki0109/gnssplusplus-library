#pragma once
#include <libgnss++/core/types.hpp>
#include <libgnss++/algorithms/residual_ionosphere_contract.hpp>
#include <stdexcept>
#include <vector>

namespace libgnss::residual_ionosphere {
struct TemporalEdge { std::size_t previous, current; double sigma_m; };
struct TemporalPlan {
    std::vector<std::size_t> anchors;
    std::vector<TemporalEdge> edges;
};
// Pure graph topology: each epoch has one shared state. Caller supplies reset
// provenance and physical noise parameters; no data-dependent tuning here.
// One anchor per segment, no independent anchor at every epoch. A reset flag
// starts a new segment but does not assert the ionosphere physically jumped.
inline TemporalPlan temporalPlan(const std::vector<GNSSTime>& times,
                                const std::vector<bool>& reset_before,
                                double max_gap_s, double density_m_sqrt_s) {
    if (times.size()!=reset_before.size() || !std::isfinite(max_gap_s) || max_gap_s<=0 ||
        !std::isfinite(density_m_sqrt_s) || density_m_sqrt_s<=0)
        throw std::invalid_argument("Invalid ionosphere temporal configuration");
    TemporalPlan plan;
    for (std::size_t i=0;i<times.size();++i) {
        if (!std::isfinite(times[i].tow) || times[i].tow<0 || times[i].tow>=604800)
            throw std::invalid_argument("Invalid ionosphere epoch time");
        if (!i) { plan.anchors.push_back(i); continue; }
        const double dt=times[i]-times[i-1];
        if (!std::isfinite(dt) || dt<=0)
            throw std::invalid_argument("Unordered ionosphere epochs");
        if (reset_before[i] || dt>max_gap_s) plan.anchors.push_back(i);
        else {
            const double sigma=randomWalkSigma(dt,density_m_sqrt_s);
            if (!std::isfinite(sigma) || sigma<=0)
                throw std::invalid_argument("Invalid ionosphere temporal sigma");
            plan.edges.push_back({i-1,i,sigma});
        }
    }
    return plan;
}
}
