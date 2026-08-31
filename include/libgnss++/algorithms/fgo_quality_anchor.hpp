#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <vector>

namespace libgnss::fgo_quality_anchor {

/**
 * @brief Truth-free quality tuple used to choose an SPP replay anchor.
 *
 * The ordering is deliberately lexicographic and contains no learned or
 * route-specific threshold.  Lower GDOP/residual is only consulted after the
 * number of finite SPP satellites has been maximized.
 */
struct Candidate {
    std::size_t index = 0;
    int satellites = 0;
    double gdop = std::numeric_limits<double>::infinity();
    double normalized_residual = std::numeric_limits<double>::infinity();
};

inline bool eligible(bool solution_valid,
                     bool finite_position,
                     double position_norm,
                     int satellites,
                     double gdop,
                     double normalized_residual) {
    return solution_valid && finite_position && std::isfinite(position_norm) &&
           position_norm > 1e6 && satellites >= 4 && std::isfinite(gdop) &&
           std::isfinite(normalized_residual) && normalized_residual >= 0.0;
}

inline bool better(const Candidate& lhs, const Candidate& rhs) {
    if (lhs.satellites != rhs.satellites) {
        return lhs.satellites > rhs.satellites;
    }
    if (lhs.gdop != rhs.gdop) {
        return lhs.gdop < rhs.gdop;
    }
    if (lhs.normalized_residual != rhs.normalized_residual) {
        return lhs.normalized_residual < rhs.normalized_residual;
    }
    return lhs.index < rhs.index;
}

inline const Candidate* choose(const std::vector<Candidate>& candidates) {
    if (candidates.empty()) {
        return nullptr;
    }
    return &*std::min_element(candidates.begin(), candidates.end(), better);
}

/**
 * @brief Return the deterministic outward order for one stateful SPP pass.
 *
 * The backward order is intentionally decreasing in input index.  It is used
 * only by a reset SPP processor to obtain seeds; the eventual FGO problem is
 * rebuilt in chronological order, so no negative-dt graph factor is made.
 */
inline std::vector<std::size_t> outwardOrder(std::size_t count,
                                             std::size_t anchor,
                                             bool forward) {
    std::vector<std::size_t> order;
    if (anchor >= count) {
        return order;
    }
    order.reserve(count);
    order.push_back(anchor);
    if (forward) {
        for (std::size_t index = anchor + 1U; index < count; ++index) {
            order.push_back(index);
        }
    } else {
        for (std::size_t index = anchor; index > 0U;) {
            --index;
            order.push_back(index);
        }
    }
    return order;
}

inline std::vector<std::size_t> graphOrder(std::size_t count) {
    std::vector<std::size_t> order(count);
    for (std::size_t index = 0; index < count; ++index) {
        order[index] = index;
    }
    return order;
}

}  // namespace libgnss::fgo_quality_anchor
