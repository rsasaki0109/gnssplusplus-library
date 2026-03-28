#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>

#include <Eigen/Cholesky>

namespace libgnss {
namespace rtk_ar_evaluation {

bool shouldSearchPreferredSubsets(bool fixed, double ratio, double threshold) {
    return !fixed || ratio < threshold + 0.5;
}

bool shouldSearchDropSubsets(bool fixed, double ratio, double threshold, double max_variance) {
    return !fixed || ratio < threshold + 0.8 || max_variance > 0.20;
}

SubsetMatrices extractSubset(const Eigen::VectorXd& full_dd_float,
                             const Eigen::MatrixXd& full_Qb,
                             const Eigen::MatrixXd& full_Qab,
                             const std::vector<int>& subset) {
    const int ns = static_cast<int>(subset.size());
    SubsetMatrices matrices;
    matrices.dd_float = Eigen::VectorXd::Zero(ns);
    matrices.Qb = Eigen::MatrixXd::Zero(ns, ns);
    matrices.Qab = Eigen::MatrixXd::Zero(full_Qab.rows(), ns);

    for (int i = 0; i < ns; ++i) {
        matrices.dd_float(i) = full_dd_float(subset[i]);
        for (int j = 0; j < ns; ++j) {
            matrices.Qb(i, j) = full_Qb(subset[i], subset[j]);
        }
        for (int row = 0; row < full_Qab.rows(); ++row) {
            matrices.Qab(row, i) = full_Qab(row, subset[i]);
        }
    }
    matrices.Qb = (matrices.Qb + matrices.Qb.transpose()) * 0.5;
    return matrices;
}

bool preferCandidate(double current_best_ratio, bool current_best_fixed, double candidate_ratio) {
    return !current_best_fixed || candidate_ratio > current_best_ratio + 1e-6;
}

void adoptCandidate(CandidateState& best_candidate,
                    const std::vector<int>& subset,
                    const SubsetMatrices& subset_matrices,
                    const Eigen::VectorXd& dd_fixed,
                    double ratio) {
    best_candidate.fixed = true;
    best_candidate.ratio = ratio;
    best_candidate.subset = subset;
    best_candidate.dd_float = subset_matrices.dd_float;
    best_candidate.Qb = subset_matrices.Qb;
    best_candidate.Qab = subset_matrices.Qab;
    best_candidate.dd_fixed = dd_fixed;
}

bool solveFixedHeadState(const Eigen::VectorXd& head_state,
                         const Eigen::MatrixXd& Qab,
                         const Eigen::MatrixXd& Qb,
                         const Eigen::VectorXd& dd_float,
                         const Eigen::VectorXd& dd_fixed,
                         Eigen::VectorXd& fixed_head_state) {
    const Eigen::VectorXd delta = dd_float - dd_fixed;
    Eigen::LDLT<Eigen::MatrixXd> solver(Qb);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    fixed_head_state = head_state - Qab * solver.solve(delta);
    return fixed_head_state.allFinite();
}

}  // namespace rtk_ar_evaluation
}  // namespace libgnss
