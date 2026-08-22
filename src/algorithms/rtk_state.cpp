#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>
#include <libgnss++/algorithms/rtk_ar_selection.hpp>
#include <libgnss++/algorithms/disjoint_satellite_fix_evidence.hpp>
#include <libgnss++/algorithms/fix_failure_budget.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/rtk_cp_pr_gate.hpp>
#include <libgnss++/algorithms/rtk_ddpr_anchor.hpp>
#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_selection.hpp>
#include <libgnss++/algorithms/rtk_ins_time_update.hpp>
#include <libgnss++/algorithms/rtk_tdcp_diagnostics.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>
#include <libgnss++/algorithms/spp_velocity.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "rtk_internal.hpp"

namespace libgnss {

using namespace rtk_internal;

double RTKProcessor::varerr(double elevation, bool is_phase, double snr_dbhz) const {
    double sin_el = std::sin(elevation);
    if (sin_el < 0.1) sin_el = 0.1;
    double a = is_phase ? rtk_config_.carrier_phase_sigma : rtk_config_.pseudorange_sigma;
    double b = is_phase ? rtk_config_.carrier_phase_sigma : rtk_config_.pseudorange_sigma;
    double variance = 2.0 * (a * a + b * b / (sin_el * sin_el));
    const double snr_min_baseline = rtk_config_.snr_min_baseline_m;
    const bool baseline_passes_snr_floor =
        !std::isfinite(snr_min_baseline) ||
        snr_min_baseline <= 0.0 ||
        (filter_state_.state.size() >= BASE_STATES &&
         std::isfinite(filter_state_.state.head<3>().norm()) &&
         filter_state_.state.head<3>().norm() >= snr_min_baseline);
    if (rtk_config_.enable_snr_weighting &&
        baseline_passes_snr_floor &&
        std::isfinite(snr_dbhz) &&
        snr_dbhz > 0.0 &&
        rtk_config_.snr_reference_dbhz > 0.0 &&
        rtk_config_.snr_max_variance_scale >= 1.0) {
        const double snr_deficit_db = std::max(0.0, rtk_config_.snr_reference_dbhz - snr_dbhz);
        const double variance_scale =
            std::clamp(std::pow(10.0, snr_deficit_db / 10.0),
                       1.0,
                       rtk_config_.snr_max_variance_scale);
        variance *= variance_scale;
    }
    return variance;
}

double RTKProcessor::elevationWeight(double elevation) const {
    double sin_el = std::sin(elevation);
    if (sin_el < 0.1) sin_el = 0.1;
    return 1.0 / (sin_el * sin_el);
}

// ============================================================
// State vector management (SD ambiguities)
// ============================================================
void RTKProcessor::expandState(int new_size) {
    int old_size = filter_state_.state.size();
    if (new_size <= old_size) return;
    VectorXd new_state = VectorXd::Zero(new_size);
    MatrixXd new_cov = MatrixXd::Zero(new_size, new_size);
    if (old_size > 0) {
        new_state.head(old_size) = filter_state_.state;
        new_cov.topLeftCorner(old_size, old_size) = filter_state_.covariance;
    }
    filter_state_.state = new_state;
    filter_state_.covariance = new_cov;
}

int RTKProcessor::getOrCreateN1Index(const SatelliteId& sat, double initial_value) {
    int idx = IB(sat, 0);
    if (filter_state_.state(idx) != 0.0) {
        filter_state_.n1_indices[sat] = idx;
        return idx;
    }
    filter_state_.n1_indices[sat] = idx;
    filter_state_.state(idx) = initial_value;
    for (int j = 0; j < filter_state_.state.size(); ++j) {
        filter_state_.covariance(idx, j) = 0.0;
        filter_state_.covariance(j, idx) = 0.0;
    }
    filter_state_.covariance(idx, idx) = 900.0;
    return idx;
}

int RTKProcessor::getOrCreateN2Index(const SatelliteId& sat, double initial_value) {
    int idx = IB(sat, 1);
    if (filter_state_.state(idx) != 0.0) {
        filter_state_.n2_indices[sat] = idx;
        return idx;
    }
    filter_state_.n2_indices[sat] = idx;
    filter_state_.state(idx) = initial_value;
    for (int j = 0; j < filter_state_.state.size(); ++j) {
        filter_state_.covariance(idx, j) = 0.0;
        filter_state_.covariance(j, idx) = 0.0;
    }
    filter_state_.covariance(idx, idx) = 900.0;
    return idx;
}

// Phase 18 Step 4: parallel to N1/N2 index creators.
// IB(sat, 2) maps into the L5 slot of the state vector (FREQ_SLOTS=3 reserved by Step 2).
int RTKProcessor::getOrCreateN5Index(const SatelliteId& sat, double initial_value) {
    int idx = IB(sat, 2);
    if (filter_state_.state(idx) != 0.0) {
        filter_state_.n5_indices[sat] = idx;
        return idx;
    }
    filter_state_.n5_indices[sat] = idx;
    filter_state_.state(idx) = initial_value;
    for (int j = 0; j < filter_state_.state.size(); ++j) {
        filter_state_.covariance(idx, j) = 0.0;
        filter_state_.covariance(j, idx) = 0.0;
    }
    filter_state_.covariance(idx, idx) = 900.0;
    return idx;
}

int RTKProcessor::getOrCreateIonoIndex(const SatelliteId& sat, double initial_value) {
    int idx = II(sat);
    if (filter_state_.covariance(idx, idx) > 0.0) {
        filter_state_.iono_indices[sat] = idx;
        return idx;
    }
    filter_state_.iono_indices[sat] = idx;
    // Keep the state active in the sparse Kalman path even if the initial iono estimate is near zero.
    filter_state_.state(idx) = std::abs(initial_value) > 1e-6 ? initial_value : 1e-3;
    for (int j = 0; j < filter_state_.state.size(); ++j) {
        filter_state_.covariance(idx, j) = 0.0;
        filter_state_.covariance(j, idx) = 0.0;
    }
    filter_state_.covariance(idx, idx) = 100.0;
    return idx;
}

void RTKProcessor::removeSatelliteFromState(const SatelliteId& sat) {
    auto it0 = filter_state_.iono_indices.find(sat);
    if (it0 != filter_state_.iono_indices.end()) {
        int idx = it0->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < filter_state_.state.size(); ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.iono_indices.erase(it0);
    }
    auto it1 = filter_state_.n1_indices.find(sat);
    if (it1 != filter_state_.n1_indices.end()) {
        int idx = it1->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < filter_state_.state.size(); ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.n1_indices.erase(it1);
    }
    auto it2 = filter_state_.n2_indices.find(sat);
    if (it2 != filter_state_.n2_indices.end()) {
        int idx = it2->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < filter_state_.state.size(); ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.n2_indices.erase(it2);
    }
    // Phase 18 Step 2: erase L5 ambiguity slot if present (no-op until Step 3+ populates n5_indices).
    auto it5 = filter_state_.n5_indices.find(sat);
    if (it5 != filter_state_.n5_indices.end()) {
        int idx = it5->second;
        filter_state_.state(idx) = 0.0;
        for (int j = 0; j < filter_state_.state.size(); ++j) {
            filter_state_.covariance(idx, j) = 0.0;
            filter_state_.covariance(j, idx) = 0.0;
        }
        filter_state_.n5_indices.erase(it5);
    }
}

// ============================================================
// Satellite data collection
// ============================================================

}  // namespace libgnss
