#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace libgnss::disjoint_satellite_fix_evidence {

// Two independently propagated RTK processors receive disjoint GNSS-system
// partitions.  Neither partition may share a satellite (including a DD
// reference satellite) with the other.  This evaluator only checks the
// declaration-time solution-separation contract; the caller owns and
// enforces the disjoint input construction.
struct Config {
    double maximum_partition_separation_m = 0.25;
    double maximum_primary_separation_m = 0.25;
    double covariance_scale = 16.0;
    // chi-square(3 dof, 0.999) / 3.
    double maximum_nis_per_dimension = 5.422;
    double maximum_statistical_separation_m = 2.0;
};

struct Evidence {
    bool available = false;
    bool inputs_verified_disjoint = false;
    bool primary_ffrt_passed = false;
    bool partition_a_ffrt_passed = false;
    bool partition_b_ffrt_passed = false;
    Eigen::Vector3d partition_a_candidate_ecef =
        Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d partition_b_candidate_ecef =
        Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d primary_candidate_ecef =
        Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix3d partition_a_covariance_ecef =
        Eigen::Matrix3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix3d partition_b_covariance_ecef =
        Eigen::Matrix3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix3d primary_covariance_ecef =
        Eigen::Matrix3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
};

struct Decision {
    bool passed = false;
    double partition_separation_m =
        std::numeric_limits<double>::quiet_NaN();
    double partition_a_primary_separation_m =
        std::numeric_limits<double>::quiet_NaN();
    double partition_b_primary_separation_m =
        std::numeric_limits<double>::quiet_NaN();
    bool hard_separation_passed = false;
    bool statistical_separation_passed = false;
    double partition_nis_per_dimension =
        std::numeric_limits<double>::quiet_NaN();
    double partition_a_primary_nis_per_dimension =
        std::numeric_limits<double>::quiet_NaN();
    double partition_b_primary_nis_per_dimension =
        std::numeric_limits<double>::quiet_NaN();
};

enum class SelectedPair {
    NONE = 0,
    PRIMARY_A = 1,
    PRIMARY_B = 2,
    A_B = 3,
};

struct ThreeCandidateConsensus {
    bool valid = false;
    SelectedPair selected_pair = SelectedPair::NONE;
    Eigen::Vector3d position_ecef =
        Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    double selected_pair_separation_m =
        std::numeric_limits<double>::quiet_NaN();
};

// Under the solution-separation single-fault model, select the mutually
// closest two of primary, partition A, and partition B. This avoids moving
// halfway toward a lone estimator whose covariance is merely diffuse enough
// to pass the statistical separation test.
inline ThreeCandidateConsensus closestPairConsensus(
    const Eigen::Vector3d& primary,
    const Eigen::Vector3d& partition_a,
    const Eigen::Vector3d& partition_b) {
    ThreeCandidateConsensus result;
    if (!primary.allFinite() ||
        !partition_a.allFinite() ||
        !partition_b.allFinite()) {
        return result;
    }
    const double primary_a = (primary - partition_a).norm();
    const double primary_b = (primary - partition_b).norm();
    const double partition_ab = (partition_a - partition_b).norm();
    if (primary_a <= primary_b && primary_a <= partition_ab) {
        result.position_ecef = 0.5 * (primary + partition_a);
        result.selected_pair_separation_m = primary_a;
        result.selected_pair = SelectedPair::PRIMARY_A;
    } else if (primary_b <= partition_ab) {
        result.position_ecef = 0.5 * (primary + partition_b);
        result.selected_pair_separation_m = primary_b;
        result.selected_pair = SelectedPair::PRIMARY_B;
    } else {
        result.position_ecef = 0.5 * (partition_a + partition_b);
        result.selected_pair_separation_m = partition_ab;
        result.selected_pair = SelectedPair::A_B;
    }
    result.valid =
        result.position_ecef.allFinite() &&
        std::isfinite(result.selected_pair_separation_m);
    return result;
}

inline double separationNisPerDimension(
    const Eigen::Vector3d& separation,
    const Eigen::Matrix3d& covariance_a,
    const Eigen::Matrix3d& covariance_b,
    double covariance_scale) {
    if (!separation.allFinite() ||
        !covariance_a.allFinite() ||
        !covariance_b.allFinite() ||
        !std::isfinite(covariance_scale) ||
        covariance_scale <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    Eigen::Matrix3d covariance =
        covariance_scale * 0.5 *
            (covariance_a + covariance_a.transpose() +
             covariance_b + covariance_b.transpose()) +
        Eigen::Matrix3d::Identity() * 0.01;
    Eigen::LDLT<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.isPositive()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto solved = solver.solve(separation);
    return separation.dot(solved) / 3.0;
}

inline Decision evaluate(
    const Config& config,
    const Evidence& evidence) {
    Decision decision;
    if (!evidence.available ||
        !evidence.inputs_verified_disjoint ||
        !evidence.primary_ffrt_passed ||
        !evidence.partition_a_ffrt_passed ||
        !evidence.partition_b_ffrt_passed ||
        !evidence.partition_a_candidate_ecef.allFinite() ||
        !evidence.partition_b_candidate_ecef.allFinite() ||
        !evidence.primary_candidate_ecef.allFinite() ||
        !std::isfinite(config.maximum_partition_separation_m) ||
        config.maximum_partition_separation_m <= 0.0 ||
        !std::isfinite(config.maximum_primary_separation_m) ||
        config.maximum_primary_separation_m <= 0.0 ||
        !std::isfinite(config.covariance_scale) ||
        config.covariance_scale <= 0.0 ||
        !std::isfinite(config.maximum_nis_per_dimension) ||
        config.maximum_nis_per_dimension <= 0.0 ||
        !std::isfinite(
            config.maximum_statistical_separation_m) ||
        config.maximum_statistical_separation_m <= 0.0) {
        return decision;
    }

    decision.partition_separation_m =
        (evidence.partition_a_candidate_ecef -
         evidence.partition_b_candidate_ecef)
            .norm();
    decision.partition_a_primary_separation_m =
        (evidence.partition_a_candidate_ecef -
         evidence.primary_candidate_ecef)
            .norm();
    decision.partition_b_primary_separation_m =
        (evidence.partition_b_candidate_ecef -
         evidence.primary_candidate_ecef)
            .norm();
    decision.hard_separation_passed =
        std::isfinite(decision.partition_separation_m) &&
        decision.partition_separation_m <=
            config.maximum_partition_separation_m &&
        std::isfinite(
            decision.partition_a_primary_separation_m) &&
        decision.partition_a_primary_separation_m <=
            config.maximum_primary_separation_m &&
        std::isfinite(
            decision.partition_b_primary_separation_m) &&
        decision.partition_b_primary_separation_m <=
            config.maximum_primary_separation_m;
    const Eigen::Vector3d partition_separation =
        evidence.partition_a_candidate_ecef -
        evidence.partition_b_candidate_ecef;
    const Eigen::Vector3d partition_a_primary_separation =
        evidence.partition_a_candidate_ecef -
        evidence.primary_candidate_ecef;
    const Eigen::Vector3d partition_b_primary_separation =
        evidence.partition_b_candidate_ecef -
        evidence.primary_candidate_ecef;
    decision.partition_nis_per_dimension =
        separationNisPerDimension(
            partition_separation,
            evidence.partition_a_covariance_ecef,
            evidence.partition_b_covariance_ecef,
            config.covariance_scale);
    decision.partition_a_primary_nis_per_dimension =
        separationNisPerDimension(
            partition_a_primary_separation,
            evidence.partition_a_covariance_ecef,
            evidence.primary_covariance_ecef,
            config.covariance_scale);
    decision.partition_b_primary_nis_per_dimension =
        separationNisPerDimension(
            partition_b_primary_separation,
            evidence.partition_b_covariance_ecef,
            evidence.primary_covariance_ecef,
            config.covariance_scale);
    decision.statistical_separation_passed =
        decision.partition_separation_m <=
            config.maximum_statistical_separation_m &&
        decision.partition_a_primary_separation_m <=
            config.maximum_statistical_separation_m &&
        decision.partition_b_primary_separation_m <=
            config.maximum_statistical_separation_m &&
        std::isfinite(decision.partition_nis_per_dimension) &&
        decision.partition_nis_per_dimension <=
            config.maximum_nis_per_dimension &&
        std::isfinite(
            decision.partition_a_primary_nis_per_dimension) &&
        decision.partition_a_primary_nis_per_dimension <=
            config.maximum_nis_per_dimension &&
        std::isfinite(
            decision.partition_b_primary_nis_per_dimension) &&
        decision.partition_b_primary_nis_per_dimension <=
            config.maximum_nis_per_dimension;
    decision.passed =
        decision.hard_separation_passed ||
        decision.statistical_separation_passed;
    return decision;
}

}  // namespace libgnss::disjoint_satellite_fix_evidence
