#pragma once

#include <Eigen/Dense>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>

#include <cstddef>
#include <map>
#include <utility>
#include <vector>

namespace libgnss::forward_backward {

/**
 * @brief Combination of a forward and a backward filter pass (post-processing).
 *
 * The epochs of both passes are matched by time. At each epoch, as RTKLIB's
 * combined solution does:
 * - a solution of better status wins (FIX > FLOAT > DGPS > SPP > PROPAGATED);
 * - two solutions of the same status are averaged with their covariances
 *   (x = Qb (Qf + Qb)^-1 xf + Qf (Qf + Qb)^-1 xb, Q = Qf (Qf + Qb)^-1 Qb);
 * - two FIX solutions further apart than max_fix_difference_m disagree on an
 *   ambiguity: the average is kept as FLOAT;
 * - a FIX that the other pass has no FLOAT or FIX solution for is reported
 *   as FLOAT (require_rtk_confirmation). On the six PPC runs such FIXes were
 *   wrong (> 0.5 m) one time in five, against 1 in 75 for a FIX the other
 *   pass solved FLOAT and 1 in 1500 for two agreeing FIXes.
 */
struct CombineConfig {
    /// Two FIX solutions agree when their 3D distance is at most this.
    double max_fix_difference_m = 0.05;
    /// Epochs of the two passes match within this time difference.
    double time_tolerance_s = 1e-3;
    /// Report a FIX as FLOAT when the other pass has no RTK (FLOAT / FIX)
    /// solution at that epoch.
    bool require_rtk_confirmation = true;
};

struct CombineStats {
    std::size_t forward_only = 0;
    std::size_t backward_only = 0;
    std::size_t both = 0;
    std::size_t averaged = 0;
    /// Better status from the backward pass, e.g. FIX where the forward pass
    /// had FLOAT (status upgrades by pass).
    std::size_t from_forward = 0;
    std::size_t from_backward = 0;
    std::size_t fix_upgrades = 0;           ///< FIX where the forward pass was not
    std::size_t fix_inconsistent = 0;       ///< both FIX but apart: kept FLOAT
    std::size_t fix_unconfirmed = 0;        ///< FIX without an RTK solution in the other pass: FLOAT
};

/**
 * @brief One pass: its solutions, each with the covariance to weight it by
 * (weights[i] for solutions[i]; the solution's own covariance when empty).
 */
struct Pass {
    std::vector<PositionSolution> solutions;
    std::vector<Eigen::Matrix3d> weights;
};

/**
 * @brief Loss-of-lock flags for a backward pass.
 *
 * An LLI flag (bit 0) marks a slip since the previous epoch. Going backward
 * that slip lies before the next epoch processed, so each epoch takes the
 * flag of the epoch after it in time, processed just before it (as RTKLIB
 * does). Call apply() on the epochs in the backward (decreasing time) order.
 */
class BackwardLossOfLock {
public:
    void apply(ObservationData& epoch);
    void reset() { later_slips_.clear(); }

private:
    std::map<std::pair<SatelliteId, SignalType>, bool> later_slips_;
};

/// Status rank used to pick a solution: higher is better, -1 for invalid.
int statusRank(const PositionSolution& solution);

/**
 * @brief Combine the two passes into one solution per epoch, in time order.
 * Invalid solutions are ignored; an epoch valid in one pass only takes it.
 */
std::vector<PositionSolution> combine(const Pass& forward,
                                      const Pass& backward,
                                      const CombineConfig& config = {},
                                      CombineStats* stats = nullptr);

}  // namespace libgnss::forward_backward
