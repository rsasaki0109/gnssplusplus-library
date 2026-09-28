// Clock-free single-difference CLAS helpers (CLASLIB ppprtk.c approach).
//
// Uses full SD (code + phase) with raw PRC/CPC (trop + iono included) so the
// receiver clock bias cancels; see solveMultiEpochSdAr().

#include <libgnss++/algorithms/ppp_clas_sd.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
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
            const Observation* raw = findOsrFrequencyObservation(obs, osr, f);
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
    result.code_rms = ratio;
    result.ar_ratio = ratio;
    result.position_shift_m = dx.norm();
    result.phase_rms = 0.0;

    if (debug_enabled) {
        std::cerr << "[CLAS-SD-MAR] FIXED ratio=" << ratio
                  << " pos_shift=" << result.position_shift_m << "\n";
    }

    return result;
}

}  // namespace libgnss::ppp_clas_sd
