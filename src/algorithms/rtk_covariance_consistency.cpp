#include <libgnss++/algorithms/rtk_covariance_consistency.hpp>

#include <cmath>

namespace libgnss::rtk_covariance_consistency {
namespace {
constexpr double kTargetNis = 3.0;       // E[chi-square] for 3 degrees of freedom
constexpr double kMaxScale = 1.0e12;     // bound for a numerically absurd gap

// d' (s*F + S)^-1 d, or a negative value when the matrix is not positive
// definite.
double statistic(const Eigen::Vector3d& d, const Eigen::Matrix3d& f,
                 const Eigen::Matrix3d& spp, double s) {
    const Eigen::Matrix3d total = s * f + spp;
    const Eigen::LDLT<Eigen::Matrix3d> ldlt(total);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) return -1.0;
    const double value = d.dot(ldlt.solve(d));
    return std::isfinite(value) ? value : -1.0;
}
}  // namespace

double sppConsistencyScale(const Eigen::Vector3d& difference,
                           const Eigen::Matrix3d& float_covariance,
                           const Eigen::Matrix3d& spp_covariance) {
    if (!difference.allFinite() || !float_covariance.allFinite() ||
        !spp_covariance.allFinite()) {
        return 1.0;
    }
    const Eigen::Matrix3d f = 0.5 * (float_covariance + float_covariance.transpose());
    const Eigen::Matrix3d p = 0.5 * (spp_covariance + spp_covariance.transpose());
    const double at_one = statistic(difference, f, p, 1.0);
    if (at_one < 0.0 || at_one <= kTargetNis) return 1.0;
    // The statistic decreases monotonically in s; bisect on log(s).
    double lo = 0.0;                       // log(1): statistic > target
    double hi = std::log(kMaxScale);
    if (statistic(difference, f, p, kMaxScale) > kTargetNis) return kMaxScale;
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double value = statistic(difference, f, p, std::exp(mid));
        if (value < 0.0 || value <= kTargetNis) hi = mid; else lo = mid;
    }
    return std::exp(hi);
}

}  // namespace libgnss::rtk_covariance_consistency
