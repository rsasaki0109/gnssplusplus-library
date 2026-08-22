#include <libgnss++/algorithms/ppp_clas_dd.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include "ppp_clas_dd_internal.hpp"

namespace libgnss::ppp_clas_dd {
using namespace internal;


int StateLayout::np() const {
    return options.dynamics ? 9 : 3;
}

int StateLayout::nf() const {
    return std::max(1, options.frequencies);
}

int StateLayout::ni() const {
    return options.ionosphere_mode == IonosphereMode::Off ? 0 : options.max_satellites;
}

int StateLayout::nt() const {
    switch (options.troposphere_mode) {
        case TroposphereMode::Off:
            return 0;
        case TroposphereMode::EstimateZtd:
            return 1;
        case TroposphereMode::EstimateZtdGradients:
            return 3;
    }
    return 0;
}

int StateLayout::nl() const {
    return options.glonass_frequency_bias_states
        ? kClaslibGlonassFrequencyBiasStates
        : 0;
}

int StateLayout::nb() const {
    return options.max_satellites * nf();
}

int StateLayout::nr() const {
    return np() + ni() + nt() + nl();
}

int StateLayout::nx() const {
    return nr() + nb();
}

int StateLayout::receiverClockIndex(int system_index) const {
    return np() + system_index;
}

int StateLayout::macroTroposphereIndexAfterClocks() const {
    return receiverClockIndex(0) + options.system_count;
}

int StateLayout::ionosphereIndex(int satno_one_based) const {
    return validSatelliteNumber(satno_one_based) && ni() > 0
        ? np() + satno_one_based - 1
        : -1;
}

int StateLayout::troposphereIndex() const {
    return nt() > 0 ? np() + ni() : -1;
}

int StateLayout::ambiguityIndex(int satno_one_based, int frequency_index) const {
    if (!validSatelliteNumber(satno_one_based) ||
        frequency_index < 0 ||
        frequency_index >= nf()) {
        return -1;
    }
    return nr() + options.max_satellites * frequency_index + satno_one_based - 1;
}

bool StateLayout::validSatelliteNumber(int satno_one_based) const {
    return satno_one_based >= 1 && satno_one_based <= options.max_satellites;
}

int claslibSatelliteNumber(const SatelliteId& satellite) {
    const int prn = static_cast<int>(satellite.prn);
    if (prn <= 0) {
        return 0;
    }

    switch (satellite.system) {
        case GNSSSystem::GPS:
            return prn <= kMaxPrnGps ? prn : 0;
        case GNSSSystem::GLONASS:
            return prn <= kMaxPrnGlo ? kNsatGps + prn : 0;
        case GNSSSystem::Galileo:
            return prn <= kMaxPrnGal ? kNsatGps + kNsatGlo + prn : 0;
        case GNSSSystem::QZSS: {
            const int claslib_prn = qzssPrnForClaslib(prn);
            return claslib_prn == 0
                ? 0
                : kNsatGps + kNsatGlo + kNsatGal +
                      (claslib_prn - kMinPrnQzsClaslib + 1);
        }
        case GNSSSystem::BeiDou:
            return prn <= kMaxPrnBds
                ? kNsatGps + kNsatGlo + kNsatGal + kNsatQzs + prn
                : 0;
        case GNSSSystem::SBAS:
            return prn >= kMinPrnSbas && prn <= kMaxPrnSbas
                ? kNsatGps + kNsatGlo + kNsatGal + kNsatQzs + kNsatBds +
                      kNsatNavic + (prn - kMinPrnSbas + 1)
                : 0;
        default:
            return 0;
    }
}

StateLayoutOptions layoutOptionsFromConfig(
    const ppp_shared::PPPConfig& config,
    const std::vector<OSRCorrection>& osr_corrections) {
    StateLayoutOptions options;
    // The canonical CLASLIB profile runs PPP-RTK kinematically with
    // pos1-dynamics=off (NP=3).  The outer native PPP still needs its
    // dynamics model for a robust seed, but those velocity/acceleration
    // states must not leak into the CLASLIB-parity DD filter layout.
    options.dynamics =
        !config.clas_mrtklib_float_parity &&
        config.kinematic_mode && config.use_dynamics_model;
    options.ionosphere_mode = config.estimate_ionosphere
        ? IonosphereMode::EstimateAdaptive
        : IonosphereMode::Off;
    // CLASLIB static_linux.conf uses pos1-tropopt=off because the compact
    // correction already supplies the troposphere term.  Do not add the
    // outer native PPP's residual-ZTD state to the parity scaffold.
    options.troposphere_mode =
        config.clas_mrtklib_float_parity
            ? TroposphereMode::Off
            : (config.estimate_troposphere
                   ? TroposphereMode::EstimateZtd
                   : TroposphereMode::Off);

    int frequencies = config.use_ionosphere_free ? 1 : 2;
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            frequencies = std::max(frequencies, osr.num_frequencies);
            for (int f = 0; f < osr.num_frequencies && f < OSR_MAX_FREQ; ++f) {
                frequencies = std::max(
                    frequencies,
                    claslibFrequencyIndex(osr.satellite, osr.signals[f], f) + 1);
            }
        }
    }
    options.frequencies = std::max(1, frequencies);
    return options;
}

DdMeasurementBuildResult buildDdMeasurementSystem(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const StateLayout& layout,
    const VectorXd& state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function,
    const Vector3d& receiver_geometry_displacement) {
    DdMeasurementBuildResult result;
    const int nx = layout.nx();
    if (state.size() < nx || nx < 3 || trop_mapping_function == nullptr) {
        return result;
    }

    const Vector3d linearization_position = state.segment(0, 3);
    const Vector3d receiver_position =
        linearization_position + receiver_geometry_displacement;
    if (!receiver_position.allFinite() || receiver_position.norm() <= 0.0) {
        return result;
    }
    result.linearization_position_ecef = linearization_position;

    double rx_lat = 0.0;
    double rx_lon = 0.0;
    double rx_height = 0.0;
    ecef2geodetic(receiver_position, rx_lat, rx_lon, rx_height);

    std::map<DdGroupKey, std::vector<ZeroDiffMeasurement>> groups;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies <= 0) {
            continue;
        }
        const int satno = claslibSatelliteNumber(osr.satellite);
        if (!layout.validSatelliteNumber(satno)) {
            continue;
        }
        const Vector3d range_vector = osr.satellite_position - receiver_position;
        const double range_norm = range_vector.norm();
        const double geo = geodist(osr.satellite_position, receiver_position);
        if (!std::isfinite(geo) || range_norm <= 0.0) {
            continue;
        }
        const Vector3d los = range_vector / range_norm;
        const Vector3d los_enu = ecef2enu(range_vector, rx_lat, rx_lon);
        const double elevation =
            std::atan2(los_enu.z(), std::hypot(los_enu.x(), los_enu.y()));
        const double sat_clock_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double ion_map = ionosphereMapFactor(receiver_position, elevation);
        const double trop_mapping =
            config.estimate_troposphere
                ? trop_mapping_function(receiver_position, elevation, obs.time)
                : 0.0;

        for (int f = 0; f < osr.num_frequencies && f < layout.nf(); ++f) {
            if (osr.wavelengths[f] <= 0.0 || osr.wavelengths[0] <= 0.0) {
                continue;
            }
            // CLASLIB validobs() requires the carrier row even while forming
            // the paired code DD. corrmeas() zeroes that carrier row when the
            // selected compact phase-bias cell is invalid, so neither phase
            // nor code from this satellite/frequency reaches ddres(). Keep
            // the rule typed to the literal MRTKLIB parity profile; ordinary
            // SSR/MADOCA and the historical CLAS filter remain unchanged.
            if (config.clas_mrtklib_float_parity &&
                !osr.phase_bias_present[f]) {
                continue;
            }
            const int frequency_index =
                claslibFrequencyIndex(osr.satellite, osr.signals[f], f);
            if (frequency_index < 0 || frequency_index >= layout.nf()) {
                continue;
            }
            const int group = systemGroup(osr.satellite, osr.signals[f]);
            if (group < 0) {
                continue;
            }
            const Observation* raw = findOsrFrequencyObservation(obs, osr, f);
            if (raw == nullptr || !raw->valid) {
                continue;
            }
            const bool phase_observable =
                raw->has_carrier_phase &&
                std::isfinite(raw->carrier_phase);
            const bool code_observable =
                raw->has_pseudorange &&
                std::isfinite(raw->pseudorange);
            const double ion_scale =
                std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2);

            if (phase_observable) {
                ZeroDiffMeasurement zd;
                zd.satellite = osr.satellite;
                zd.satno = satno;
                zd.system_group = group;
                zd.is_phase = true;
                zd.frequency_index = frequency_index;
                zd.residual_m =
                    raw->carrier_phase * osr.wavelengths[f] -
                    (geo - sat_clock_m + osr.CPC[f]);
                if (ppp_shared::pppDebugEnabled()) {
                    std::cerr << std::setprecision(15)
                              << "[CLAS-DD-ZD-NATIVE] tow=" << obs.time.tow
                              << " sat=" << osr.satellite.toString()
                              << " f=" << frequency_index
                              << " phase=1 zd=" << zd.residual_m
                              << " geo=" << geo
                              << " clock_m=" << sat_clock_m
                              << " satpos="
                              << osr.satellite_position.transpose()
                              << " corr=" << osr.CPC[f] << '\n';
                }
                zd.los = los;
                zd.elevation_rad = elevation;
                zd.ionosphere_map = ion_map;
                zd.trop_mapping = trop_mapping;
                zd.wavelength_m = osr.wavelengths[f];
                zd.ionosphere_scale = ion_scale;
                zd.variance_m2 =
                    claslibVarerr(
                        osr.satellite.system, elevation, true, frequency_index);
                groups[{group, frequency_index, true}].push_back(zd);
            }

            if (code_observable) {
                ZeroDiffMeasurement zd;
                zd.satellite = osr.satellite;
                zd.satno = satno;
                zd.system_group = group;
                zd.is_phase = false;
                zd.frequency_index = frequency_index;
                zd.residual_m =
                    raw->pseudorange - (geo - sat_clock_m + osr.PRC[f]);
                if (ppp_shared::pppDebugEnabled()) {
                    std::cerr << std::setprecision(15)
                              << "[CLAS-DD-ZD-NATIVE] tow=" << obs.time.tow
                              << " sat=" << osr.satellite.toString()
                              << " f=" << frequency_index
                              << " phase=0 zd=" << zd.residual_m
                              << " geo=" << geo
                              << " clock_m=" << sat_clock_m
                              << " corr=" << osr.PRC[f] << '\n';
                }
                zd.los = los;
                zd.elevation_rad = elevation;
                zd.ionosphere_map = ion_map;
                zd.trop_mapping = trop_mapping;
                zd.wavelength_m = osr.wavelengths[f];
                zd.ionosphere_scale = ion_scale;
                zd.variance_m2 =
                    claslibVarerr(
                        osr.satellite.system, elevation, false, frequency_index);
                groups[{group, frequency_index, false}].push_back(zd);
            }
        }
    }

    std::map<DdGroupKey, std::vector<DdRow>> rows_by_group;
    for (const auto& [key, measurements] : groups) {
        if (measurements.size() < 2) {
            continue;
        }

        const auto ref_it = std::max_element(
            measurements.begin(),
            measurements.end(),
            [](const ZeroDiffMeasurement& lhs, const ZeroDiffMeasurement& rhs) {
                return lhs.elevation_rad < rhs.elevation_rad;
            });
        if (ref_it == measurements.end()) {
            continue;
        }
        ++result.reference_groups;
        const ZeroDiffMeasurement& ref = *ref_it;
        result.reference_groups_detail.push_back(
            {key.system_group,
             key.frequency_index,
             key.is_phase,
             ref.satellite,
             ref.elevation_rad});

        for (const auto& sat : measurements) {
            if (sat.satellite == ref.satellite) {
                continue;
            }

            DdRow row;
            row.reference_satellite = ref.satellite;
            row.target_satellite = sat.satellite;
            row.is_phase = key.is_phase;
            row.frequency_index = key.frequency_index;
            row.system_group = key.system_group;
            row.raw_dd_m = ref.residual_m - sat.residual_m;
            row.residual_m = row.raw_dd_m;
            row.reference_elevation_rad = ref.elevation_rad;
            row.target_elevation_rad = sat.elevation_rad;
            row.position_coefficients = -ref.los + sat.los;
            row.reference_variance_m2 = ref.variance_m2;
            row.target_variance_m2 = sat.variance_m2;

            if (config.estimate_ionosphere &&
                layout.options.ionosphere_mode != IonosphereMode::Off) {
                const double sign = row.is_phase ? -1.0 : 1.0;
                const double ref_coeff =
                    sign * ref.ionosphere_scale * ref.ionosphere_map;
                const double sat_coeff =
                    sign * sat.ionosphere_scale * sat.ionosphere_map;
                const int ref_iono = layout.ionosphereIndex(ref.satno);
                const int sat_iono = layout.ionosphereIndex(sat.satno);
                row.residual_m -=
                    ref_coeff * state(ref_iono) - sat_coeff * state(sat_iono);
                addStateCoefficient(row.state_coefficients, ref_iono, ref_coeff);
                addStateCoefficient(row.state_coefficients, sat_iono, -sat_coeff);
            }

            const int trop_index = layout.troposphereIndex();
            if (trop_index >= 0 && config.estimate_troposphere) {
                const double trop_coeff = ref.trop_mapping - sat.trop_mapping;
                row.residual_m -= trop_coeff * state(trop_index);
                addStateCoefficient(row.state_coefficients, trop_index, trop_coeff);
            }

            if (row.is_phase) {
                const int ref_ambiguity =
                    layout.ambiguityIndex(ref.satno, key.frequency_index);
                const int sat_ambiguity =
                    layout.ambiguityIndex(sat.satno, key.frequency_index);
                row.residual_m -=
                    ref.wavelength_m * state(ref_ambiguity) -
                    sat.wavelength_m * state(sat_ambiguity);
                addStateCoefficient(
                    row.state_coefficients, ref_ambiguity, ref.wavelength_m);
                addStateCoefficient(
                    row.state_coefficients, sat_ambiguity, -sat.wavelength_m);
                ++result.phase_rows;
            } else {
                ++result.code_rows;
            }

            rows_by_group[key].push_back(row);
            result.rows.push_back(std::move(row));
        }
    }

    const auto blocks = rowsToBlocks(rows_by_group);
    result.measurement_system =
        rtk_measurement::assembleMeasurementSystem(blocks, nx);
    return result;
}

void appendDdMeasurementRowsCsv(
    const std::string& path,
    const GNSSTime& time,
    const DdMeasurementBuildResult& build,
    const std::string& stage) {
    if (path.empty()) {
        return;
    }
    const bool needs_header = !std::ifstream(path).good();
    std::ofstream output(path, std::ios::app);
    if (!output.is_open()) {
        return;
    }
    output.precision(17);
    if (needs_header) {
        output
            << "schema,week,tow,stage,system_group,frequency_index,"
            << "measurement_type,reference_satellite,target_satellite,raw_dd_m,residual_m,"
            << "reference_variance_m2,target_variance_m2,"
            << "reference_elevation_rad,target_elevation_rad,"
            << "linearization_x_m,linearization_y_m,linearization_z_m,"
            << "position_coeff_x,position_coeff_y,position_coeff_z\n";
    }
    for (const auto& row : build.rows) {
        output << "clas_dd_measurement.v3," << time.week << ',' << time.tow << ','
               << stage << ',' << row.system_group << ',' << row.frequency_index << ','
               << (row.is_phase ? "phase" : "code") << ','
               << row.reference_satellite.toString() << ','
               << row.target_satellite.toString() << ',' << row.raw_dd_m << ','
               << row.residual_m << ','
               << row.reference_variance_m2 << ',' << row.target_variance_m2 << ','
               << row.reference_elevation_rad << ',' << row.target_elevation_rad << ','
               << build.linearization_position_ecef.x() << ','
               << build.linearization_position_ecef.y() << ','
               << build.linearization_position_ecef.z() << ','
               << row.position_coefficients.x() << ','
               << row.position_coefficients.y() << ','
               << row.position_coefficients.z() << '\n';
    }
}

DdPostfitValidationResult validateDdPostfitResiduals(
    const DdMeasurementBuildResult& postfit_build,
    const StateLayout& layout,
    const MatrixXd& covariance) {
    DdPostfitValidationResult validation;
    validation.row_count = static_cast<int>(postfit_build.rows.size());
    if (postfit_build.rows.empty()) {
        validation.reject_reason = "postfit_no_rows";
        return validation;
    }

    double all_sum_sq = 0.0;
    double phase_sum_sq = 0.0;
    for (const auto& row : postfit_build.rows) {
        const double abs_residual = std::abs(row.residual_m);
        all_sum_sq += row.residual_m * row.residual_m;
        validation.residual_max_abs_m =
            std::max(validation.residual_max_abs_m, abs_residual);
        if (row.is_phase) {
            ++validation.phase_row_count;
            phase_sum_sq += row.residual_m * row.residual_m;
            validation.phase_residual_max_abs_m =
                std::max(validation.phase_residual_max_abs_m, abs_residual);
            if (abs_residual >= std::abs(validation.worst_phase_residual_m)) {
                validation.worst_phase_residual_m = row.residual_m;
                validation.worst_phase_system_group = row.system_group;
                validation.worst_phase_frequency_index = row.frequency_index;
                validation.worst_phase_pair =
                    row.reference_satellite.toString() + ">" +
                    row.target_satellite.toString();
            }
        }
    }
    validation.residual_rms_m =
        std::sqrt(all_sum_sq / static_cast<double>(validation.row_count));
    if (validation.phase_row_count > 0) {
        validation.phase_residual_rms_m =
            std::sqrt(phase_sum_sq /
                      static_cast<double>(validation.phase_row_count));
    }

    if (validation.phase_row_count < std::max(2, layout.np())) {
        validation.reject_reason = "postfit_min_rows";
        return validation;
    }
    if (validation.phase_residual_rms_m > kDdFixedPostfitPhaseRmsLimitM) {
        validation.reject_reason = "postfit_rms";
        return validation;
    }
    if (validation.phase_residual_max_abs_m > kDdFixedPostfitPhaseMaxLimitM) {
        validation.reject_reason = "postfit_max";
        return validation;
    }

    const auto& system = postfit_build.measurement_system;
    if (system.residuals.size() != validation.row_count ||
        system.design_matrix.rows() != validation.row_count ||
        system.covariance.rows() != validation.row_count ||
        system.covariance.cols() != validation.row_count ||
        covariance.rows() != layout.nx() ||
        covariance.cols() != layout.nx()) {
        validation.reject_reason = "postfit_invalid_system";
        return validation;
    }

    const MatrixXd innovation_covariance =
        system.design_matrix * covariance * system.design_matrix.transpose() +
        system.covariance;

    double weighted_phase_sum = 0.0;
    int accepted_phase_rows = 0;
    for (int row_index = 0; row_index < validation.row_count; ++row_index) {
        if (!postfit_build.rows[static_cast<size_t>(row_index)].is_phase) {
            continue;
        }
        const double sigma2 = innovation_covariance(row_index, row_index);
        if (!std::isfinite(sigma2) || sigma2 <= 0.0) {
            validation.reject_reason = "postfit_nonfinite_sigma";
            return validation;
        }
        const double residual = system.residuals(row_index);
        const double sigma = std::sqrt(sigma2);
        if (std::abs(residual) > kClaslibResidualSigmaGate * sigma) {
            validation.reject_reason = "postfit_sigma";
            return validation;
        }
        weighted_phase_sum += residual * residual / sigma2;
        ++accepted_phase_rows;
    }

    const int dof = accepted_phase_rows - layout.np();
    if (dof <= 0) {
        validation.reject_reason = "postfit_dof";
        return validation;
    }
    validation.chi_square_ratio =
        weighted_phase_sum / static_cast<double>(dof);
    if (!std::isfinite(validation.chi_square_ratio) ||
        validation.chi_square_ratio > kClaslibFixedChiSquareGate) {
        validation.reject_reason = "postfit_chi2";
        return validation;
    }

    validation.accepted = true;
    return validation;
}
}  // namespace libgnss::ppp_clas_dd
