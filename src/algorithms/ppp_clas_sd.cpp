// Clock-free single-difference PPP filter (CLASLIB ppprtk.c approach).
//
// Starts from a converged KF position (~0.14m accuracy) and uses full SD
// (code + phase) with raw PRC/CPC (trop + iono included) to refine position.
// NO clock state, NO trop state.  SD cancels receiver clock bias.
//
// The key advantage over the main KF: no position-clock coupling.
// Code SD residuals are ~1m (vs 3m in UD code) because clock noise
// doesn't leak into position through KF covariance.

#include <libgnss++/algorithms/ppp_clas_sd.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <cmath>
#include <iostream>
#include <map>
#include <vector>

namespace libgnss::ppp_clas_sd {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace {

constexpr double kMinElevationRad = 0.26;
constexpr double kCodeVarianceScale = 8.0;
constexpr double kPhaseVariance = 0.01;
constexpr double kAmbiguityInitialVariance = 1e6;
constexpr double kAmbiguityProcessNoise = 1e-8;
constexpr double kOutlierSigmaScale = 30.0;

struct SatObs {
    const OSRCorrection* osr = nullptr;
    double y_code[2] = {};
    double y_phase[2] = {};
    bool valid_code[2] = {};
    bool valid_phase[2] = {};
    Vector3d los = Vector3d::Zero();
};

}  // namespace

SdEpochResult runClockFreeSdEpoch(
    SdFilterState& state,
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const Vector3d& seed_position,
    double dt,
    bool debug_enabled) {

    SdEpochResult result;

    // Compute zdres at current linearization point
    const Vector3d lin_pos = state.initialized ? state.x.head(3) : seed_position;
    std::map<SatelliteId, SatObs> sat_obs;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies < 2) continue;
        if (osr.elevation < kMinElevationRad) continue;

        SatObs so;
        so.osr = &osr;
        const double geo = geodist(osr.satellite_position, lin_pos);
        so.los = (osr.satellite_position - lin_pos).normalized();
        const double sat_clk_m = constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;

        for (int f = 0; f < std::min(osr.num_frequencies, 2); ++f) {
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid) continue;
            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                so.y_code[f] = raw->pseudorange - geo + sat_clk_m - osr.PRC[f];
                so.valid_code[f] = true;
            }
            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                so.y_phase[f] = raw->carrier_phase * osr.wavelengths[f]
                              - geo + sat_clk_m - osr.CPC[f];
                so.valid_phase[f] = true;
            }
        }
        sat_obs[osr.satellite] = so;
    }

    if (sat_obs.size() < 4) return result;

    // Reference satellite per system (highest elevation)
    std::map<GNSSSystem, SatelliteId> new_refs;
    for (const auto& [sat, so] : sat_obs) {
        auto it = new_refs.find(sat.system);
        if (it == new_refs.end() || so.osr->elevation > sat_obs[it->second].osr->elevation)
            new_refs[sat.system] = sat;
    }

    // Detect reference satellite change → reset affected ambiguities
    if (state.initialized) {
        for (const auto& [sys, new_ref] : new_refs) {
            auto old_it = state.reference_sats.find(sys);
            if (old_it != state.reference_sats.end() &&
                !(old_it->second == new_ref)) {
                // Ref changed: reset all ambiguities for this system
                for (auto& [sat, idx] : state.amb_indices) {
                    if (sat.system == sys) {
                        state.x(idx) = 0.0;
                        state.P(idx, idx) = kAmbiguityInitialVariance;
                    }
                }
                for (auto& [sat, idx] : state.amb2_indices) {
                    if (sat.system == sys) {
                        state.x(idx) = 0.0;
                        state.P(idx, idx) = kAmbiguityInitialVariance;
                    }
                }
                if (debug_enabled) {
                    std::cerr << "[CLAS-SD] ref change: "
                              << old_it->second.toString()
                              << " -> " << new_ref.toString() << "\n";
                }
            }
        }
    }
    state.reference_sats = new_refs;

    // Initialize from KF-converged position (small initial variance)
    if (!state.initialized) {
        state.x = VectorXd::Zero(3);
        state.P = MatrixXd::Identity(3, 3) * 0.25;  // 0.5m sigma
        state.x.head(3) = seed_position;
        state.nx = 3;
        state.amb_indices.clear();
        state.amb2_indices.clear();
        state.initialized = true;
    }

    // Allocate ambiguity states
    auto ensure_amb = [&](const SatelliteId& sat,
                          std::map<SatelliteId, int>& indices) {
        if (indices.count(sat)) return;
        const int idx = state.nx++;
        indices[sat] = idx;
        VectorXd new_x = VectorXd::Zero(state.nx);
        new_x.head(idx) = state.x;
        state.x = new_x;
        MatrixXd new_P = MatrixXd::Zero(state.nx, state.nx);
        new_P.topLeftCorner(idx, idx) = state.P;
        new_P(idx, idx) = kAmbiguityInitialVariance;
        state.P = new_P;
    };
    for (const auto& [sat, so] : sat_obs) {
        auto ref_it = new_refs.find(sat.system);
        if (ref_it == new_refs.end() || ref_it->second == sat) continue;
        ensure_amb(sat, state.amb_indices);
        if (so.osr->num_frequencies >= 2) ensure_amb(sat, state.amb2_indices);
    }

    // Prediction (static: no position process noise)
    MatrixXd Q = MatrixXd::Zero(state.nx, state.nx);
    for (const auto& [_, idx] : state.amb_indices) { (void)_; Q(idx, idx) = kAmbiguityProcessNoise * dt; }
    for (const auto& [_, idx] : state.amb2_indices) { (void)_; Q(idx, idx) = kAmbiguityProcessNoise * dt; }
    state.P += Q;

    // Form SD observations (code + phase)
    struct SdObs { Eigen::RowVectorXd H; double z; double R; bool is_phase; };
    std::vector<SdObs> sd_obs_list;

    for (const auto& [sat, so] : sat_obs) {
        auto ref_it = new_refs.find(sat.system);
        if (ref_it == new_refs.end() || ref_it->second == sat) continue;
        const auto& ref_so = sat_obs[ref_it->second];
        const double sin_r = std::sin(std::max(ref_so.osr->elevation, 0.1));
        const double sin_s = std::sin(std::max(so.osr->elevation, 0.1));
        const double ew_r = 1.0 / (sin_r * sin_r);
        const double ew_s = 1.0 / (sin_s * sin_s);

        for (int f = 0; f < 2; ++f) {
            const auto& amb_map = (f == 0) ? state.amb_indices : state.amb2_indices;
            auto amb_it = amb_map.find(sat);
            if (amb_it == amb_map.end()) continue;
            const int amb_idx = amb_it->second;

            // SD code
            if (so.valid_code[f] && ref_so.valid_code[f]) {
                const double v = ref_so.y_code[f] - so.y_code[f];
                Eigen::RowVectorXd h = Eigen::RowVectorXd::Zero(state.nx);
                h.head(3) = (-ref_so.los + so.los).transpose();
                sd_obs_list.push_back({h, v, kCodeVarianceScale * (ew_r + ew_s), false});
            }

            // SD phase
            if (so.valid_phase[f] && ref_so.valid_phase[f] && so.osr->wavelengths[f] > 0.0) {
                double v = ref_so.y_phase[f] - so.y_phase[f];
                Eigen::RowVectorXd h = Eigen::RowVectorXd::Zero(state.nx);
                h.head(3) = (-ref_so.los + so.los).transpose();
                h(amb_idx) = -so.osr->wavelengths[f];

                // Initialize ambiguity from well-converged position
                if (state.P(amb_idx, amb_idx) >= kAmbiguityInitialVariance * 0.9) {
                    state.x(amb_idx) = v / so.osr->wavelengths[f];
                    state.P(amb_idx, amb_idx) = 100.0;
                }

                // Subtract current ambiguity from residual
                v += so.osr->wavelengths[f] * state.x(amb_idx);
                sd_obs_list.push_back({h, v, kPhaseVariance * (ew_r + ew_s), true});
            }
        }
    }

    const int nobs = static_cast<int>(sd_obs_list.size());
    if (nobs < 4) return result;

    // Kalman update
    MatrixXd H = MatrixXd::Zero(nobs, state.nx);
    VectorXd z = VectorXd::Zero(nobs);
    VectorXd R_diag = VectorXd::Zero(nobs);
    for (int i = 0; i < nobs; ++i) {
        H.row(i) = sd_obs_list[static_cast<size_t>(i)].H;
        z(i) = sd_obs_list[static_cast<size_t>(i)].z;
        R_diag(i) = sd_obs_list[static_cast<size_t>(i)].R;
        if (std::abs(z(i)) > kOutlierSigmaScale * std::sqrt(R_diag(i)))
            R_diag(i) = 1e10;
    }

    const MatrixXd R = R_diag.asDiagonal();
    const MatrixXd S = H * state.P * H.transpose() + R;
    const MatrixXd K = state.P * H.transpose() * S.inverse();
    const VectorXd dx = K * z;
    if (!dx.allFinite()) return result;

    state.x += dx;
    const MatrixXd I_KH = MatrixXd::Identity(state.nx, state.nx) - K * H;
    state.P = I_KH * state.P * I_KH.transpose() + K * R * K.transpose();

    // Residual statistics
    double code_ss = 0.0, phase_ss = 0.0;
    int n_code = 0, n_phase = 0;
    for (int i = 0; i < nobs; ++i) {
        if (R_diag(i) > 1e9) continue;
        const double post_res = z(i) - H.row(i).dot(dx);
        if (sd_obs_list[static_cast<size_t>(i)].is_phase) {
            phase_ss += post_res * post_res; ++n_phase;
        } else {
            code_ss += post_res * post_res; ++n_code;
        }
    }

    result.valid = true;
    result.position = state.x.head(3);
    result.num_satellites = static_cast<int>(sat_obs.size());
    result.num_observations = nobs;
    result.code_rms = n_code > 0 ? std::sqrt(code_ss / n_code) : 0.0;
    result.phase_rms = n_phase > 0 ? std::sqrt(phase_ss / n_phase) : 0.0;
    state.position_sigma_m = std::sqrt(state.P.block(0, 0, 3, 3).trace() / 3);

    if (debug_enabled) {
        std::cerr << "[CLAS-SD] ep=" << state.epoch_count
                  << " nobs=" << nobs << " sats=" << sat_obs.size()
                  << " pos_shift=" << dx.head(3).norm()
                  << " code_rms=" << result.code_rms
                  << " phase_rms=" << result.phase_rms
                  << " pos_sigma=" << state.position_sigma_m << "\n";
    }

    ++state.epoch_count;
    return result;
}

// ============================================================================
// Multi-epoch SD AR: accumulate float DD ambiguities, then LAMBDA
// ============================================================================

namespace {

struct SdSatObs {
    const OSRCorrection* osr = nullptr;
    double y_code[2] = {};
    double y_phase[2] = {};
    bool valid_code[2] = {};
    bool valid_phase[2] = {};
    Vector3d los = Vector3d::Zero();
};

std::map<SatelliteId, SdSatObs> buildSdZdres(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const Vector3d& position) {
    std::map<SatelliteId, SdSatObs> sat_obs;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies < 2 || osr.elevation < 0.26) continue;
        SdSatObs so;
        so.osr = &osr;
        const double geo = geodist(osr.satellite_position, position);
        so.los = (osr.satellite_position - position).normalized();
        const double sat_clk_m = constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        for (int f = 0; f < std::min(osr.num_frequencies, 2); ++f) {
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid) continue;
            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                so.y_code[f] = raw->pseudorange - geo + sat_clk_m - osr.PRC[f];
                so.valid_code[f] = true;
            }
            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                so.y_phase[f] = raw->carrier_phase * osr.wavelengths[f]
                              - geo + sat_clk_m - osr.CPC[f];
                so.valid_phase[f] = true;
            }
        }
        sat_obs[osr.satellite] = so;
    }
    return sat_obs;
}

std::map<GNSSSystem, SatelliteId> selectRefSats(
    const std::map<SatelliteId, SdSatObs>& sat_obs) {
    std::map<GNSSSystem, SatelliteId> refs;
    for (const auto& [sat, so] : sat_obs) {
        auto it = refs.find(sat.system);
        if (it == refs.end() || so.osr->elevation > sat_obs.at(it->second).osr->elevation)
            refs[sat.system] = sat;
    }
    return refs;
}

Vector3d solveCodeSdPosition(
    const std::map<SatelliteId, SdSatObs>& sat_obs,
    const std::map<GNSSSystem, SatelliteId>& refs,
    const Vector3d& seed) {
    struct CodeObs { Eigen::RowVectorXd H; double z; double R; };
    std::vector<CodeObs> obs;
    for (const auto& [sat, so] : sat_obs) {
        auto ref_it = refs.find(sat.system);
        if (ref_it == refs.end() || ref_it->second == sat) continue;
        const auto& ref_so = sat_obs.at(ref_it->second);
        const double sin_r = std::sin(std::max(ref_so.osr->elevation, 0.1));
        const double sin_s = std::sin(std::max(so.osr->elevation, 0.1));
        const double ew = 1.0 / (sin_r * sin_r) + 1.0 / (sin_s * sin_s);
        for (int f = 0; f < 2; ++f) {
            if (so.valid_code[f] && ref_so.valid_code[f]) {
                Eigen::RowVectorXd h = (-ref_so.los + so.los).transpose();
                obs.push_back({h, ref_so.y_code[f] - so.y_code[f], 8.0 * ew});
            }
        }
    }
    if (obs.size() < 4) return seed;
    const int n = static_cast<int>(obs.size());
    MatrixXd H(n, 3); VectorXd z(n); MatrixXd W = MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i) {
        H.row(i) = obs[static_cast<size_t>(i)].H;
        z(i) = obs[static_cast<size_t>(i)].z;
        W(i, i) = 1.0 / obs[static_cast<size_t>(i)].R;
    }
    Eigen::LDLT<MatrixXd> ldlt(H.transpose() * W * H);
    if (ldlt.info() != Eigen::Success) return seed;
    VectorXd dx = ldlt.solve(H.transpose() * W * z);
    return dx.allFinite() ? (seed + dx).eval() : seed;
}

}  // namespace

SdEpochResult solveMultiEpochSdAr(
    DdAmbAccumulator& acc,
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const Vector3d& seed_position,
    double ar_ratio_threshold,
    int min_accumulation_epochs,
    bool debug_enabled) {

    SdEpochResult result;
    auto sat_obs = buildSdZdres(obs, osr_corrections, seed_position);
    if (sat_obs.size() < 5) return result;
    const auto refs = selectRefSats(sat_obs);

    // Code-only position refinement
    const Vector3d refined = solveCodeSdPosition(sat_obs, refs, seed_position);

    // Recompute zdres at refined position
    sat_obs = buildSdZdres(obs, osr_corrections, refined);

    // Compute single-epoch DD float ambiguities (L1 only)
    for (const auto& [sat, so] : sat_obs) {
        auto ref_it = refs.find(sat.system);
        if (ref_it == refs.end() || ref_it->second == sat) continue;
        const auto& ref_so = sat_obs.at(ref_it->second);
        if (!so.valid_phase[0] || !ref_so.valid_phase[0]) continue;
        if (so.osr->wavelengths[0] <= 0.0) continue;

        const double sd_phase = ref_so.y_phase[0] - so.y_phase[0];
        const double sd_geo = (-ref_so.los + so.los).transpose().dot(Eigen::Vector3d::Zero());
        // DD amb = SD_phase / wavelength (position contribution is tiny at refined pos)
        const double dd_amb_cycles = sd_phase / so.osr->wavelengths[0];

        auto& entry = acc.l1_ambs[sat];
        entry.sum += dd_amb_cycles;
        entry.sum_sq += dd_amb_cycles * dd_amb_cycles;
        entry.count += 1;
    }
    acc.total_epochs += 1;

    // Not enough epochs accumulated yet
    if (acc.total_epochs < min_accumulation_epochs) {
        result.valid = true;
        result.position = refined;
        result.num_satellites = static_cast<int>(sat_obs.size());
        return result;
    }

    // Build LAMBDA input from accumulated DD ambiguities
    std::vector<std::pair<SatelliteId, double>> amb_list;
    std::vector<double> amb_vars;
    for (const auto& [sat, entry] : acc.l1_ambs) {
        if (entry.count < min_accumulation_epochs) continue;
        // Check satellite still visible
        if (sat_obs.find(sat) == sat_obs.end()) continue;
        amb_list.push_back({sat, entry.mean()});
        amb_vars.push_back(entry.variance());
    }

    const int n_amb = static_cast<int>(amb_list.size());
    if (n_amb < 4) {
        result.valid = true;
        result.position = refined;
        return result;
    }

    VectorXd float_amb(n_amb);
    MatrixXd Q_amb = MatrixXd::Zero(n_amb, n_amb);
    for (int i = 0; i < n_amb; ++i) {
        float_amb(i) = amb_list[static_cast<size_t>(i)].second;
        Q_amb(i, i) = std::max(amb_vars[static_cast<size_t>(i)], 1e-6);
    }

    VectorXd fixed_amb;
    double ratio = 0.0;
    const bool fixed = lambdaSearch(float_amb, Q_amb, fixed_amb, ratio);

    if (debug_enabled) {
        std::cerr << "[CLAS-SD-MAR] ep=" << acc.total_epochs
                  << " namb=" << n_amb << " fixed=" << fixed
                  << " ratio=" << ratio
                  << " mean_var=" << Q_amb.diagonal().mean()
                  << " min_count=" << acc.l1_ambs.begin()->second.count
                  << "\n";
    }

    if (!fixed || ratio < ar_ratio_threshold) {
        result.valid = true;
        result.position = refined;
        return result;
    }

    // Fixed: solve position from fixed phase SD
    std::vector<std::pair<Eigen::RowVectorXd, double>> fixed_obs;
    for (int i = 0; i < n_amb; ++i) {
        const auto& sat = amb_list[static_cast<size_t>(i)].first;
        auto sat_it = sat_obs.find(sat);
        if (sat_it == sat_obs.end()) continue;
        const auto& so = sat_it->second;
        auto ref_it = refs.find(sat.system);
        if (ref_it == refs.end()) continue;
        const auto& ref_so = sat_obs.at(ref_it->second);
        if (!so.valid_phase[0] || !ref_so.valid_phase[0]) continue;

        const double sd_phase = ref_so.y_phase[0] - so.y_phase[0];
        const double v = sd_phase - (-so.osr->wavelengths[0]) * fixed_amb(i);
        Eigen::RowVectorXd h = (-ref_so.los + so.los).transpose();
        fixed_obs.push_back({h, v});
    }

    if (fixed_obs.size() < 4) {
        result.valid = true;
        result.position = refined;
        return result;
    }

    const int nfix = static_cast<int>(fixed_obs.size());
    MatrixXd H_fix(nfix, 3);
    VectorXd z_fix(nfix);
    MatrixXd W_fix = MatrixXd::Identity(nfix, nfix);  // unit weight for phase
    for (int i = 0; i < nfix; ++i) {
        H_fix.row(i) = fixed_obs[static_cast<size_t>(i)].first;
        z_fix(i) = fixed_obs[static_cast<size_t>(i)].second;
    }

    Eigen::LDLT<MatrixXd> ldlt(H_fix.transpose() * W_fix * H_fix);
    if (ldlt.info() != Eigen::Success) {
        result.valid = true;
        result.position = refined;
        return result;
    }
    const VectorXd dx = ldlt.solve(H_fix.transpose() * W_fix * z_fix);

    result.valid = true;
    result.position = refined + dx;
    result.num_satellites = static_cast<int>(sat_obs.size());
    result.code_rms = ratio;  // Store ratio
    result.phase_rms = 0.0;

    if (debug_enabled) {
        std::cerr << "[CLAS-SD-MAR] FIXED ratio=" << ratio
                  << " pos_shift=" << dx.norm() << "\n";
    }

    return result;
}

// ============================================================================
// Single-epoch SD least-squares + LAMBDA ambiguity resolution
// ============================================================================

SdEpochResult solveSingleEpochSdAr(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const Vector3d& seed_position,
    double ar_ratio_threshold,
    bool debug_enabled) {

    SdEpochResult result;

    // Build zero-difference residuals at seed position
    struct SatObs {
        const OSRCorrection* osr = nullptr;
        double y_code[2] = {};
        double y_phase[2] = {};
        bool valid_code[2] = {};
        bool valid_phase[2] = {};
        Vector3d los = Vector3d::Zero();
    };

    std::map<SatelliteId, SatObs> sat_obs;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies < 2) continue;
        if (osr.elevation < 0.26) continue;

        SatObs so;
        so.osr = &osr;
        const double geo = geodist(osr.satellite_position, seed_position);
        so.los = (osr.satellite_position - seed_position).normalized();
        const double sat_clk_m = constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;

        for (int f = 0; f < std::min(osr.num_frequencies, 2); ++f) {
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid) continue;
            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                so.y_code[f] = raw->pseudorange - geo + sat_clk_m - osr.PRC[f];
                so.valid_code[f] = true;
            }
            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                so.y_phase[f] = raw->carrier_phase * osr.wavelengths[f]
                              - geo + sat_clk_m - osr.CPC[f];
                so.valid_phase[f] = true;
            }
        }
        sat_obs[osr.satellite] = so;
    }

    if (sat_obs.size() < 5) return result;

    // Reference satellite per system
    std::map<GNSSSystem, SatelliteId> refs;
    for (const auto& [sat, so] : sat_obs) {
        auto it = refs.find(sat.system);
        if (it == refs.end() || so.osr->elevation > sat_obs[it->second].osr->elevation)
            refs[sat.system] = sat;
    }

    // Build observation list: SD code + SD phase
    // State: pos(3) + DD_amb(n_amb) in cycles
    struct AmbInfo { SatelliteId sat; int freq; double wavelength; int state_idx; };
    std::vector<AmbInfo> ambs;

    // Count non-reference satellites with valid dual-freq
    for (const auto& [sat, so] : sat_obs) {
        auto ref_it = refs.find(sat.system);
        if (ref_it == refs.end() || ref_it->second == sat) continue;
        // L1 only for AR (fewer ambiguities → better ratio)
        const int f = 0;
        if (!so.valid_phase[f] || !sat_obs[ref_it->second].valid_phase[f]) continue;
        if (so.osr->wavelengths[f] <= 0.0) continue;
        ambs.push_back({sat, f, so.osr->wavelengths[f],
                        3 + static_cast<int>(ambs.size())});
    }

    const int n_amb = static_cast<int>(ambs.size());
    if (n_amb < 4) return result;
    const int nx = 3 + n_amb;

    // Step 1: Code-only SD least squares for position
    struct Obs { Eigen::RowVectorXd H; double z; double R; bool is_phase; };
    std::vector<Obs> code_obs;

    for (const auto& [sat, so] : sat_obs) {
        auto ref_it = refs.find(sat.system);
        if (ref_it == refs.end() || ref_it->second == sat) continue;
        const auto& ref_so = sat_obs[ref_it->second];
        const double sin_r = std::sin(std::max(ref_so.osr->elevation, 0.1));
        const double sin_s = std::sin(std::max(so.osr->elevation, 0.1));
        const double ew = 1.0 / (sin_r * sin_r) + 1.0 / (sin_s * sin_s);

        for (int f = 0; f < 2; ++f) {
            if (so.valid_code[f] && ref_so.valid_code[f]) {
                const double v = ref_so.y_code[f] - so.y_code[f];
                Eigen::RowVectorXd h = Eigen::RowVectorXd::Zero(3);
                h = (-ref_so.los + so.los).transpose();
                code_obs.push_back({h, v, 8.0 * ew, false});
            }
        }
    }

    if (code_obs.size() < 4) return result;

    // WLS for position from code SD only
    const int n_code = static_cast<int>(code_obs.size());
    MatrixXd H_code = MatrixXd::Zero(n_code, 3);
    VectorXd z_code = VectorXd::Zero(n_code);
    MatrixXd W_code = MatrixXd::Zero(n_code, n_code);
    for (int i = 0; i < n_code; ++i) {
        H_code.row(i) = code_obs[static_cast<size_t>(i)].H;
        z_code(i) = code_obs[static_cast<size_t>(i)].z;
        W_code(i, i) = 1.0 / code_obs[static_cast<size_t>(i)].R;
    }
    const MatrixXd N_code = H_code.transpose() * W_code * H_code;
    const Eigen::LDLT<MatrixXd> ldlt_code(N_code);
    if (ldlt_code.info() != Eigen::Success) return result;
    const VectorXd dx_code = ldlt_code.solve(H_code.transpose() * W_code * z_code);
    if (!dx_code.allFinite()) return result;

    // Step 2: Compute DD ambiguities from code-refined position
    const Vector3d refined_pos = seed_position + dx_code;

    // Recompute zdres at refined position for phase amb extraction
    for (auto& [sat, so] : sat_obs) {
        const double geo = geodist(so.osr->satellite_position, refined_pos);
        so.los = (so.osr->satellite_position - refined_pos).normalized();
        const double sat_clk_m = constants::SPEED_OF_LIGHT * so.osr->satellite_clock_bias_s;
        for (int f = 0; f < std::min(so.osr->num_frequencies, 2); ++f) {
            const Observation* raw = obs.getObservation(so.osr->satellite, so.osr->signals[f]);
            if (!raw || !raw->valid) continue;
            if (raw->has_pseudorange && std::isfinite(raw->pseudorange))
                so.y_code[f] = raw->pseudorange - geo + sat_clk_m - so.osr->PRC[f];
            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase))
                so.y_phase[f] = raw->carrier_phase * so.osr->wavelengths[f]
                              - geo + sat_clk_m - so.osr->CPC[f];
        }
    }

    // Step 3: Full LS with code+phase at refined position
    std::vector<Obs> obs_list;
    for (const auto& [sat, so] : sat_obs) {
        auto ref_it = refs.find(sat.system);
        if (ref_it == refs.end() || ref_it->second == sat) continue;
        const auto& ref_so = sat_obs[ref_it->second];
        const double sin_r = std::sin(std::max(ref_so.osr->elevation, 0.1));
        const double sin_s = std::sin(std::max(so.osr->elevation, 0.1));
        const double ew = 1.0 / (sin_r * sin_r) + 1.0 / (sin_s * sin_s);

        for (int f = 0; f < 2; ++f) {
            if (so.valid_code[f] && ref_so.valid_code[f]) {
                const double v = ref_so.y_code[f] - so.y_code[f];
                Eigen::RowVectorXd h = Eigen::RowVectorXd::Zero(nx);
                h.head(3) = (-ref_so.los + so.los).transpose();
                obs_list.push_back({h, v, 8.0 * ew, false});
            }

            if (!so.valid_phase[f] || !ref_so.valid_phase[f]) continue;
            if (so.osr->wavelengths[f] <= 0.0) continue;
            int amb_idx = -1;
            for (const auto& ai : ambs) {
                if (ai.sat == sat && ai.freq == f) { amb_idx = ai.state_idx; break; }
            }
            if (amb_idx < 0) continue;

            const double v = ref_so.y_phase[f] - so.y_phase[f];
            Eigen::RowVectorXd h = Eigen::RowVectorXd::Zero(nx);
            h.head(3) = (-ref_so.los + so.los).transpose();
            h(amb_idx) = -so.osr->wavelengths[f];
            obs_list.push_back({h, v, 0.01 * ew, true});
        }
    }

    const int nobs = static_cast<int>(obs_list.size());
    if (nobs < nx + 1) return result;

    // WLS for pos + amb
    MatrixXd H = MatrixXd::Zero(nobs, nx);
    VectorXd z = VectorXd::Zero(nobs);
    MatrixXd W = MatrixXd::Zero(nobs, nobs);
    for (int i = 0; i < nobs; ++i) {
        H.row(i) = obs_list[static_cast<size_t>(i)].H;
        z(i) = obs_list[static_cast<size_t>(i)].z;
        W(i, i) = 1.0 / obs_list[static_cast<size_t>(i)].R;
    }

    const MatrixXd N = H.transpose() * W * H;
    const Eigen::LDLT<MatrixXd> ldlt(N);
    if (ldlt.info() != Eigen::Success) return result;
    const VectorXd x_float = ldlt.solve(H.transpose() * W * z);
    if (!x_float.allFinite()) return result;
    const MatrixXd Q_full = ldlt.solve(MatrixXd::Identity(nx, nx));

    // Extract ambiguity float values (in cycles) and covariance
    VectorXd amb_float = VectorXd::Zero(n_amb);
    for (int i = 0; i < n_amb; ++i) {
        // Convert from meters to cycles
        amb_float(i) = x_float(3 + i);
    }
    // Ambiguity states are already in cycles (H has wavelength factor)
    // Wait — H(amb_idx) = -wavelength, so x(amb_idx) is in METERS / wavelength?
    // No: H * x = z. z is in meters. H(amb) = -wavelength. x(amb) * (-wavelength) = phase_contribution_m.
    // So x(amb) is in cycles (because wavelength * cycles = meters).
    // Actually: H(amb) = -lambda, x(amb) in cycles → H*x = -lambda * N_cycles = meters. Correct.

    const MatrixXd Q_amb = Q_full.block(3, 3, n_amb, n_amb);

    // LAMBDA search
    VectorXd amb_fixed;
    double ratio = 0.0;
    const bool fixed = lambdaSearch(amb_float, Q_amb, amb_fixed, ratio);

    if (debug_enabled) {
        std::cerr << "[CLAS-SD-AR] nobs=" << nobs << " namb=" << n_amb
                  << " fixed=" << fixed << " ratio=" << ratio
                  << " pos_shift=" << x_float.head(3).norm() << "\n";
    }

    if (!fixed || ratio < ar_ratio_threshold) {
        // Return float solution (code-refined + float LS correction)
        result.valid = true;
        result.position = refined_pos + x_float.head(3);
        result.num_satellites = static_cast<int>(sat_obs.size());
        result.num_observations = nobs;
        return result;
    }

    // Fixed solution: re-solve position with fixed ambiguities (phase only)
    // Phase SD: v - lambda*N_fixed = H_pos * dx
    std::vector<Obs> phase_fixed_obs;
    for (size_t i = 0; i < obs_list.size(); ++i) {
        if (!obs_list[i].is_phase) continue;
        double v = obs_list[i].z;
        // Remove fixed ambiguity from observation
        for (int j = 0; j < n_amb; ++j) {
            v -= obs_list[i].H(3 + j) * amb_fixed(j);
        }
        Obs fixed_obs;
        fixed_obs.H = Eigen::RowVectorXd::Zero(3);
        fixed_obs.H = obs_list[i].H.head(3);
        fixed_obs.z = v;
        fixed_obs.R = obs_list[i].R;
        fixed_obs.is_phase = true;
        phase_fixed_obs.push_back(fixed_obs);
    }

    if (phase_fixed_obs.size() < 4) {
        result.valid = true;
        result.position = refined_pos + x_float.head(3);
        return result;
    }

    // WLS for position only
    const int nfix = static_cast<int>(phase_fixed_obs.size());
    MatrixXd H_fix = MatrixXd::Zero(nfix, 3);
    VectorXd z_fix = VectorXd::Zero(nfix);
    MatrixXd W_fix = MatrixXd::Zero(nfix, nfix);
    for (int i = 0; i < nfix; ++i) {
        H_fix.row(i) = phase_fixed_obs[static_cast<size_t>(i)].H;
        z_fix(i) = phase_fixed_obs[static_cast<size_t>(i)].z;
        W_fix(i, i) = 1.0 / phase_fixed_obs[static_cast<size_t>(i)].R;
    }

    const MatrixXd N_fix = H_fix.transpose() * W_fix * H_fix;
    const Eigen::LDLT<MatrixXd> ldlt_fix(N_fix);
    if (ldlt_fix.info() != Eigen::Success) {
        result.valid = true;
        result.position = refined_pos + x_float.head(3);
        return result;
    }
    const VectorXd dx_fix = ldlt_fix.solve(H_fix.transpose() * W_fix * z_fix);

    result.valid = true;
    result.position = refined_pos + dx_fix;
    result.num_satellites = static_cast<int>(sat_obs.size());
    result.num_observations = nobs;
    result.code_rms = ratio;  // Store AR ratio in code_rms field
    result.phase_rms = 0.0;

    if (debug_enabled) {
        std::cerr << "[CLAS-SD-AR] FIXED pos_shift=" << dx_fix.norm()
                  << " ratio=" << ratio
                  << " float_shift=" << x_float.head(3).norm() << "\n";
    }

    return result;
}

}  // namespace libgnss::ppp_clas_sd
