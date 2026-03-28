#pragma once

#include <Eigen/Dense>

#include <vector>

namespace libgnss {
namespace rtk_ar_evaluation {

struct CandidateState {
    bool fixed = false;
    double ratio = 0.0;
    std::vector<int> subset;
    Eigen::VectorXd dd_float;
    Eigen::MatrixXd Qb;
    Eigen::MatrixXd Qab;
    Eigen::VectorXd dd_fixed;
};

struct SubsetMatrices {
    Eigen::VectorXd dd_float;
    Eigen::MatrixXd Qb;
    Eigen::MatrixXd Qab;
};

bool shouldSearchPreferredSubsets(bool fixed, double ratio, double threshold);
bool shouldSearchDropSubsets(bool fixed, double ratio, double threshold, double max_variance);

SubsetMatrices extractSubset(const Eigen::VectorXd& full_dd_float,
                             const Eigen::MatrixXd& full_Qb,
                             const Eigen::MatrixXd& full_Qab,
                             const std::vector<int>& subset);

bool preferCandidate(double current_best_ratio, bool current_best_fixed, double candidate_ratio);

void adoptCandidate(CandidateState& best_candidate,
                    const std::vector<int>& subset,
                    const SubsetMatrices& subset_matrices,
                    const Eigen::VectorXd& dd_fixed,
                    double ratio);

bool solveFixedHeadState(const Eigen::VectorXd& head_state,
                         const Eigen::MatrixXd& Qab,
                         const Eigen::MatrixXd& Qb,
                         const Eigen::VectorXd& dd_float,
                         const Eigen::VectorXd& dd_fixed,
                         Eigen::VectorXd& fixed_head_state);

}  // namespace rtk_ar_evaluation
}  // namespace libgnss
