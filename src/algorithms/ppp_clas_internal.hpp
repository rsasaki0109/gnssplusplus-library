#pragma once

// Shared file-local helpers for the PPP-CLAS implementation TUs;
// extracted from the former monolithic ppp_clas.cpp anonymous namespace.

#include <libgnss++/algorithms/ppp_clas.hpp>

#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_bias_identity.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include "ppp_internal.hpp"
#include "ppp_clas_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace libgnss::ppp_clas {
namespace internal {


constexpr std::array<double, 100> kClaslibChiSquare001 = {
    10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
    31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
    46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
    61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
    74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
    88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100.0,
    101.0,102.0,103.0,104.0,105.0,107.0,108.0,109.0,110.0,112.0,
    113.0,114.0,115.0,116.0,118.0,119.0,120.0,122.0,123.0,125.0,
    126.0,127.0,128.0,129.0,131.0,132.0,133.0,134.0,135.0,137.0,
    138.0,139.0,140.0,142.0,143.0,144.0,145.0,147.0,148.0,149.0
};

inline double claslibChiSquare001ForDof(int dof) {
    if (dof <= 0) {
        return 0.0;
    }
    if (dof <= static_cast<int>(kClaslibChiSquare001.size())) {
        return kClaslibChiSquare001[static_cast<size_t>(dof - 1)];
    }
    const double tail = std::max(0, dof - static_cast<int>(kClaslibChiSquare001.size()));
    return kClaslibChiSquare001.back() + 1.1 * tail;
}

struct PhaseResidualInfo {
    SatelliteId ambiguity_satellite;
    SatelliteId real_satellite;
    double residual_m = 0.0;
    double variance_m2 = 0.0;
    double frequency_hz = 0.0;
    double wavelength_m = 0.0;
    double elevation_rad = 0.0;
    // Sparse measurement partials for the MRTKLIB innovation-variance basis
    // (h-row entries; the receiver clock is omitted because it cancels in
    // the same-system DDs where these are consumed).
    Vector3d unit_vector = Vector3d::Zero();  ///< d(range)/d(receiver pos)
    double trop_mapping = 0.0;
    int iono_index = -1;
    double iono_scale = 0.0;  ///< mu_f = (lambda_f/lambda_1)^2, phase sign applied at use
    int ambiguity_index = -1;
};

struct PhasePairInfo {
    std::array<double, 2> residual_m{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 2> variance_m2{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 2> frequency_hz{0.0, 0.0};
    std::array<double, 2> wavelength_m{0.0, 0.0};
};

inline double elevationWeight(double elevation_rad) {
    const double s = std::sin(elevation_rad);
    return 1.0 / (s * s);
}

// ---------------------------------------------------------------------------
// MRTKLIB varerr() parity (mrtk_ppp_rtk.c:322) with the CLAS benchmark
// constants from conf/benchmark/clas.toml [kalman_filter.measurement_error]:
//   phase = 0.01 m (err[1]), phase_elevation = 0.005 m (err[2]),
//   code_phase_ratio_L1 = 50 (eratio[0]);
//   var = a^2 + b^2 / sin^2(el),  code rows multiply a,b by eratio;
//   GPS/QZS L2-slot phase rows get x(2.55/1.55)^2 (mrtk_ppp_rtk.c:1633);
//   EFACT is 1.0 for GPS/Galileo/QZSS (mrtk_const.h) so it is omitted.
// Only active on the dynamics-model kinematic CLAS path (the MRTKLIB
// equivalence track); white-noise/static keep the historical flat
// clas_code_variance_scale / clas_phase_variance / (1/sin^2 el) model.
constexpr double kMrtklibPhaseErrConstM = 0.01;
constexpr double kMrtklibPhaseErrElevM = 0.005;
constexpr double kMrtklibCodePhaseRatio = 50.0;
constexpr double kMrtklibL2PhaseVarFactor = (2.55 / 1.55) * (2.55 / 1.55);
constexpr double kL2FrequencyHz = 1227.60e6;

inline bool clasMrtklibFloatParity(const ppp_shared::PPPConfig& config) {
    return config.clas_mrtklib_float_parity &&
           config.kinematic_mode && !config.low_dynamics_mode &&
           config.use_clas_osr_filter && config.use_dynamics_model;
}


inline bool isMrtklibL2Slot(int freq_index, double frequency_hz) {
    return freq_index > 0 && std::abs(frequency_hz - kL2FrequencyHz) < 1.0e6;
}

inline double clasPhaseVariance(const ppp_shared::PPPConfig& config,
                         double elevation_rad,
                         int freq_index,
                         double frequency_hz) {
    if (!clasMrtklibFloatParity(config)) {
        return config.clas_phase_variance * elevationWeight(elevation_rad);
    }
    const double s = std::sin(elevation_rad);
    double var = kMrtklibPhaseErrConstM * kMrtklibPhaseErrConstM +
                 (kMrtklibPhaseErrElevM * kMrtklibPhaseErrElevM) / (s * s);
    if (isMrtklibL2Slot(freq_index, frequency_hz)) {
        var *= kMrtklibL2PhaseVarFactor;
    }
    return var;
}

inline double clasCodeVariance(const ppp_shared::PPPConfig& config,
                        double elevation_rad) {
    if (!clasMrtklibFloatParity(config)) {
        return config.clas_code_variance_scale * elevationWeight(elevation_rad);
    }
    const double s = std::sin(elevation_rad);
    const double a = kMrtklibCodePhaseRatio * kMrtklibPhaseErrConstM;
    const double b = kMrtklibCodePhaseRatio * kMrtklibPhaseErrElevM;
    return a * a + (b * b) / (s * s);
}

inline int receiverClockStateIndex(const ppp_shared::PPPState& filter_state,
                            const SatelliteId& satellite) {
    switch (satellite.system) {
        case GNSSSystem::GLONASS:
            return filter_state.glo_clock_index;
        case GNSSSystem::Galileo:
            return filter_state.gal_clock_index >= 0
                ? filter_state.gal_clock_index : filter_state.clock_index;
        case GNSSSystem::QZSS:
            return filter_state.qzs_clock_index >= 0
                ? filter_state.qzs_clock_index : filter_state.clock_index;
        case GNSSSystem::BeiDou:
            return filter_state.bds_clock_index >= 0
                ? filter_state.bds_clock_index : filter_state.clock_index;
        default:
            return filter_state.clock_index;
    }
}

inline double receiverClockBiasMeters(const ppp_shared::PPPState& filter_state,
                               const SatelliteId& satellite) {
    const int clock_index = receiverClockStateIndex(filter_state, satellite);
    return filter_state.state(clock_index);
}

inline bool usesClasStecConstraint(const ppp_shared::PPPConfig& config) {
    return pppEnvOverrides().clas_stec_constraint &&
           config.estimate_ionosphere &&
           config.use_clas_osr_filter;
}

inline double cssrStecQualityStdTecu(int quality) {
    const int cls = (quality >> 3) & 0x7;
    const int val = quality & 0x7;
    if ((cls == 0 && val == 0) || (cls == 7 && val == 7)) {
        return std::numeric_limits<double>::infinity();
    }
    return (1.0 + 0.25 * static_cast<double>(val)) *
               std::pow(3.0, static_cast<double>(cls)) -
           1.0;
}

inline double tecuStdToDelayVarianceM2(double sigma_tecu, double frequency_hz) {
    if (!std::isfinite(sigma_tecu) || sigma_tecu < 0.0 || frequency_hz <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    constexpr double kTecuToElectrons = 1e16;
    const double sigma_m =
        40.3 * kTecuToElectrons * sigma_tecu / (frequency_hz * frequency_hz);
    return sigma_m * sigma_m;
}

inline double clasStecConstraintVariance(
    const std::map<std::string, std::string>& epoch_atmos,
    const OSRCorrection& osr,
    const ppp_shared::PPPConfig& config) {
    constexpr double kClaslibStdIonoM = 0.010;
    constexpr double kClaslibMinVarianceM2 = kClaslibStdIonoM * kClaslibStdIonoM;
    const std::string key = "atmos_stec_quality:" + osr.satellite.toString();
    const auto quality_it = epoch_atmos.find(key);
    if (quality_it != epoch_atmos.end()) {
        const int quality = std::atoi(quality_it->second.c_str());
        const double quality_variance =
            tecuStdToDelayVarianceM2(
                cssrStecQualityStdTecu(quality),
                osr.frequencies[0]);
        if (std::isfinite(quality_variance) && quality_variance > 0.0) {
            return std::max(kClaslibMinVarianceM2, quality_variance);
        }
    }
    if (config.clas_iono_prior_variance > 0.0) {
        return std::min(config.clas_iono_prior_variance, kClaslibMinVarianceM2);
    }
    return kClaslibMinVarianceM2;
}

inline double effectiveClasIonoProcessNoise(const ppp_shared::PPPConfig& config) {
    if (usesClasStecConstraint(config)) {
        constexpr double kClaslibPrnIonoStdM = 0.001;
        return kClaslibPrnIonoStdM * kClaslibPrnIonoStdM;
    }
    return config.process_noise_ionosphere;
}

inline std::ofstream* ambDatumDumpStream() {
    return ppp_clas_diagnostics::ambiguityDatumStream();
}

inline std::ofstream* clasGeometryDumpStream() {
    return ppp_clas_diagnostics::geometryDumpStream();
}

inline bool clasPhaseRowDumpEnabled() {
    return ppp_clas_diagnostics::phaseRowDumpEnabled();
}

inline void writeClasCodeDumpCodePhaseExtension(std::ostream& out) {
    ppp_clas_diagnostics::writeCodePhaseExtension(out);
}

inline void writeClasCodeDumpPhasePhaseExtension(
    std::ostream& out,
    double cpc_m,
    double carrier_correction_m,
    double cpc_minus_trop_m,
    double phase_bias_m,
    double windup_m,
    double phase_compensation_m,
    double raw_l_m,
    double corrected_l_m) {
    ppp_clas_diagnostics::writePhasePhaseExtension(
        out,
        cpc_m,
        carrier_correction_m,
        cpc_minus_trop_m,
        phase_bias_m,
        windup_m,
        phase_compensation_m,
        raw_l_m,
        corrected_l_m);
}

inline std::ofstream* clasCodeDumpStream() {
    return ppp_clas_diagnostics::codeDumpStream();
}

inline bool selectedClasGeometryDumpTow(double tow) {
    return ppp_clas_diagnostics::selectedGeometryDumpTow(tow);
}

inline Vector3d clasGeometryDumpReceiverPosition(const Vector3d& fallback) {
    return ppp_clas_diagnostics::geometryDumpReceiverPosition(fallback);
}

inline bool usesClaslibCodePrcRows(const ppp_shared::PPPConfig& config) {
    return pppEnvOverrides().clas_code_row_full_prc &&
           config.clas_correction_application_policy ==
               ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

inline bool usesClaslibBiasValidityRows(const ppp_shared::PPPConfig& config) {
    const auto& env = pppEnvOverrides();
    return config.use_clas_osr_filter && env.clas_dd_filter &&
           env.clas_code_row_bias_identity;
}

inline void dumpClasCodeRows(
    const char* stage,
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function) {
    auto* dump = clasCodeDumpStream();
    if (dump == nullptr) {
        return;
    }
    if (filter_state.total_states <= filter_state.trop_index ||
        filter_state.state.size() < filter_state.total_states) {
        return;
    }

    const Vector3d receiver_position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    double rx_lat = 0.0;
    double rx_lon = 0.0;
    double rx_h = 0.0;
    ecef2geodetic(receiver_position, rx_lat, rx_lon, rx_h);

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const double geo = geodist(osr.satellite_position, receiver_position);
        const Vector3d range_vector = osr.satellite_position - receiver_position;
        if (!std::isfinite(geo) || range_vector.norm() <= 0.0) {
            continue;
        }
        const Vector3d los = range_vector.normalized();
        const Vector3d los_enu = ecef2enu(los, rx_lat, rx_lon);
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double receiver_clock_m =
            receiverClockBiasMeters(filter_state, osr.satellite);
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_model = trop_mapping * trop_zenith;
        const bool claslib_code_prc = usesClaslibCodePrcRows(config);
        const double code_trop_model = claslib_code_prc ? 0.0 : trop_model;
        const auto iono_state_it = filter_state.ionosphere_indices.find(osr.satellite);
        const bool have_iono_state =
            config.estimate_ionosphere &&
            iono_state_it != filter_state.ionosphere_indices.end() &&
            iono_state_it->second >= 0 &&
            iono_state_it->second < filter_state.total_states;
        const double iono_state_l1_m =
            have_iono_state ? filter_state.state(iono_state_it->second) : 0.0;

        for (int f = 0; f < std::max(osr.num_frequencies,
                                     osr.num_output_frequencies); ++f) {
            if (usesClaslibBiasValidityRows(config) &&
                (!osr.code_bias_present[f] || !osr.phase_bias_present[f])) {
                continue;
            }
            const auto lookup = findOsrFrequencyObservationWithProvenance(obs, osr, f);
            const Observation* raw = lookup.observation;
            if (raw == nullptr || !raw->valid || !raw->has_pseudorange ||
                !std::isfinite(raw->pseudorange)) {
                continue;
            }
            const auto applied = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);
            const double corrected_p =
                raw->pseudorange - applied.pseudorange_correction_m;
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            const double iono_scaled = iono_scale * osr.iono_l1_m;
            const double predicted =
                geo - sat_clk_m + receiver_clock_m + code_trop_model +
                iono_scale * iono_state_l1_m;
            const double residual = corrected_p - predicted;
            const double variance = clasCodeVariance(config, osr.elevation);
            const bool have_atmos_ref =
                osr.atmos_reference_time.week != 0 ||
                std::abs(osr.atmos_reference_time.tow) > 0.0;
            const bool have_clock_ref =
                osr.clock_reference_time.week != 0 ||
                std::abs(osr.clock_reference_time.tow) > 0.0;
            const bool have_code_bias_ref =
                osr.code_bias_reference_time.week != 0 ||
                std::abs(osr.code_bias_reference_time.tow) > 0.0;
            const double atmos_clock_gap_s =
                (have_atmos_ref && have_clock_ref)
                    ? osr.atmos_reference_time - osr.clock_reference_time
                    : 0.0;
            auto timeWeekField = [](const GNSSTime& time, bool have_time) {
                return have_time ? std::to_string(time.week) : std::string();
            };
            auto timeTowField = [](const GNSSTime& time, bool have_time) {
                if (!have_time) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << time.tow;
                return stream.str();
            };
            auto secondsField = [](double value, bool have_value) {
                if (!have_value) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                return stream.str();
            };
            auto doubleField = [](double value, bool have_value) {
                if (!have_value) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                return stream.str();
            };

            *dump << std::setprecision(17)
                  << "CODE,"
                  << stage << ','
                  << obs.time.week << ','
                  << obs.time.tow << ','
                  << osr.satellite.toString() << ','
                  << f << ','
                  << static_cast<int>(raw->signal) << ','
                  << raw->pseudorange_observation_type << ','
                  << raw->carrier_phase_observation_type << ','
                  << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                         raw->pseudorange_observation_type) << ','
                  << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                         raw->carrier_phase_observation_type) << ','
                  << ppp_internal::signalFamilyName(raw->signal) << ','
                  << osr.pseudorange_rinex_codes[f] << ','
                  << osr.carrier_rinex_codes[f] << ','
                  << (lookup.exact_identity_requested ? 1 : 0) << ','
                  << (lookup.exact_identity_matched ? 1 : 0) << ','
                  << (lookup.family_fallback ? 1 : 0) << ','
                  << static_cast<int>(osr.code_bias_signal_ids[f]) << ','
                  << static_cast<int>(osr.phase_bias_signal_ids[f]) << ','
                  << (osr.bias_exact_identity[f] ? 1 : 0) << ','
                  << static_cast<int>(osr.code_bias_source_signal_ids[f]) << ','
                  << static_cast<int>(osr.phase_bias_source_signal_ids[f]) << ','
                  << (osr.code_bias_present[f] ? 1 : 0) << ','
                  << (osr.phase_bias_present[f] ? 1 : 0) << ','
                  << (osr.code_bias_fallback[f] ? 1 : 0) << ','
                  << (osr.phase_bias_fallback[f] ? 1 : 0) << ','
                  << raw->pseudorange << ','
                  << corrected_p << ','
                  << applied.pseudorange_correction_m << ','
                  << osr.PRC[f] << ','
                  << (osr.PRC[f] - osr.trop_correction_m) << ','
                  << osr.trop_correction_m << ','
                  << osr.iono_l1_m << ','
                  << osr.stec_tecu << ','
                  << iono_scaled << ','
                  << osr.code_bias_m[f] << ','
                  << osr.network_compensation_m << ','
                  << osr.iode_geometry_compensation_m << ','
                  << osr.receiver_antenna_m[f] << ','
                  << osr.relativity_correction_m << ','
                  << timeWeekField(osr.atmos_reference_time, have_atmos_ref) << ','
                  << timeTowField(osr.atmos_reference_time, have_atmos_ref) << ','
                  << timeWeekField(osr.clock_reference_time, have_clock_ref) << ','
                  << timeTowField(osr.clock_reference_time, have_clock_ref) << ','
                  << timeWeekField(osr.code_bias_reference_time, have_code_bias_ref) << ','
                  << timeTowField(osr.code_bias_reference_time, have_code_bias_ref) << ','
                  << secondsField(atmos_clock_gap_s, have_atmos_ref && have_clock_ref) << ','
                  << osr.atmos_network_id << ','
                  << osr.atmos_grid_no << ','
                  << osr.atmos_nearest_grid_distance_m << ','
                  << osr.atmos_interpolation_grid_count << ','
                  << osr.atmos_interpolation_grid_no[0] << ','
                  << osr.atmos_interpolation_weights[0] << ','
                  << osr.atmos_interpolation_grid_no[1] << ','
                  << osr.atmos_interpolation_weights[1] << ','
                  << osr.atmos_interpolation_grid_no[2] << ','
                  << osr.atmos_interpolation_weights[2] << ','
                  << osr.atmos_interpolation_grid_no[3] << ','
                  << osr.atmos_interpolation_weights[3] << ','
                  << (osr.atmos_lifecycle ? 1 : 0) << ','
                  << doubleField(
                         osr.atmos_lifecycle_tow,
                         osr.has_atmos_lifecycle_tow) << ','
                  << osr.atmos_selected_satellite_count << ','
                  << osr.atmos_valid_grid_count << ','
                  << osr.atmos_stec_grid_value_count << ','
                  << osr.atmos_selected_grid_stec_value_count << ','
                  << geo << ','
                  << sat_clk_m << ','
                  << receiver_clock_m << ','
                  << code_trop_model << ','
                  << iono_state_l1_m << ','
                  << iono_scale << ','
                  << predicted << ','
                  << residual << ','
                  << variance << ','
                  << los_enu.x() << ','
                  << los_enu.y() << ','
                  << los_enu.z() << ','
                  << osr.azimuth << ','
                  << osr.elevation << ','
                  << receiver_position.x() << ','
                  << receiver_position.y() << ','
                  << receiver_position.z();
            writeClasCodeDumpCodePhaseExtension(*dump);
            *dump << '\n';
        }
    }
}

inline void dumpClasPhaseRows(
    const char* stage,
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function) {
    if (!clasPhaseRowDumpEnabled()) {
        return;
    }
    auto* dump = clasCodeDumpStream();
    if (dump == nullptr) {
        return;
    }
    if (filter_state.total_states <= filter_state.trop_index ||
        filter_state.state.size() < filter_state.total_states) {
        return;
    }

    const Vector3d receiver_position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    double rx_lat = 0.0;
    double rx_lon = 0.0;
    double rx_h = 0.0;
    ecef2geodetic(receiver_position, rx_lat, rx_lon, rx_h);

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const double geo = geodist(osr.satellite_position, receiver_position);
        const Vector3d range_vector = osr.satellite_position - receiver_position;
        if (!std::isfinite(geo) || range_vector.norm() <= 0.0) {
            continue;
        }
        const Vector3d los = range_vector.normalized();
        const Vector3d los_enu = ecef2enu(los, rx_lat, rx_lon);
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double receiver_clock_m =
            receiverClockBiasMeters(filter_state, osr.satellite);
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_model = trop_mapping * trop_zenith;
        const auto iono_state_it = filter_state.ionosphere_indices.find(osr.satellite);
        const bool have_iono_state =
            config.estimate_ionosphere &&
            iono_state_it != filter_state.ionosphere_indices.end() &&
            iono_state_it->second >= 0 &&
            iono_state_it->second < filter_state.total_states;
        const double iono_state_l1_m =
            have_iono_state ? filter_state.state(iono_state_it->second) : 0.0;

        for (int f = 0; f < std::max(osr.num_frequencies,
                                     osr.num_output_frequencies); ++f) {
            if (usesClaslibBiasValidityRows(config) &&
                (!osr.code_bias_present[f] || !osr.phase_bias_present[f])) {
                continue;
            }
            const auto lookup = findOsrFrequencyObservationWithProvenance(obs, osr, f);
            const Observation* raw = lookup.observation;
            if (raw == nullptr || !raw->valid || !raw->has_carrier_phase ||
                !std::isfinite(raw->carrier_phase)) {
                continue;
            }
            const auto applied = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);
            const double l_m = raw->carrier_phase * osr.wavelengths[f];
            const double l_corr = l_m - applied.carrier_phase_correction_m;
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            const double iono_scaled = iono_scale * osr.iono_l1_m;
            const auto& env = pppEnvOverrides();
            const bool claslib_amb_datum_phase =
                env.clas_amb_datum &&
                !env.clas_amb_datum_residual_phase_trop &&
                config.clas_correction_application_policy ==
                    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
            const bool residual_amb_datum_phase_trop =
                env.clas_amb_datum &&
                env.clas_amb_datum_residual_phase_trop &&
                config.clas_correction_application_policy ==
                    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
            const double phase_trop_model = claslib_amb_datum_phase
                ? 0.0
                : (residual_amb_datum_phase_trop
                      ? trop_model - osr.trop_correction_m
                      : trop_model);
            const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId amb_sat(osr.satellite.system, amb_prn);
            const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
            if (amb_it == filter_state.ambiguity_indices.end()) {
                continue;
            }
            const int amb_idx = amb_it->second;
            const double predicted_no_amb =
                geo - sat_clk_m + receiver_clock_m + phase_trop_model
                - iono_scale * iono_state_l1_m;
            const double predicted =
                predicted_no_amb + filter_state.state(amb_idx);
            const double residual = l_corr - predicted;
            const double variance =
                clasPhaseVariance(config, osr.elevation, f, osr.frequencies[f]);
            const bool have_atmos_ref =
                osr.atmos_reference_time.week != 0 ||
                std::abs(osr.atmos_reference_time.tow) > 0.0;
            const bool have_clock_ref =
                osr.clock_reference_time.week != 0 ||
                std::abs(osr.clock_reference_time.tow) > 0.0;
            const bool have_code_bias_ref =
                osr.code_bias_reference_time.week != 0 ||
                std::abs(osr.code_bias_reference_time.tow) > 0.0;
            const double atmos_clock_gap_s =
                (have_atmos_ref && have_clock_ref)
                    ? osr.atmos_reference_time - osr.clock_reference_time
                    : 0.0;
            auto timeWeekField = [](const GNSSTime& time, bool have_time) {
                return have_time ? std::to_string(time.week) : std::string();
            };
            auto timeTowField = [](const GNSSTime& time, bool have_time) {
                if (!have_time) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << time.tow;
                return stream.str();
            };
            auto secondsField = [](double value, bool have_value) {
                if (!have_value) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                return stream.str();
            };
            auto doubleField = [](double value, bool have_value) {
                if (!have_value) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                return stream.str();
            };

            *dump << std::setprecision(17)
                  << "PHASE,"
                  << stage << ','
                  << obs.time.week << ','
                  << obs.time.tow << ','
                  << osr.satellite.toString() << ','
                  << f << ','
                  << static_cast<int>(raw->signal) << ','
                  << raw->pseudorange_observation_type << ','
                  << raw->carrier_phase_observation_type << ','
                  << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                         raw->pseudorange_observation_type) << ','
                  << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                         raw->carrier_phase_observation_type) << ','
                  << ppp_internal::signalFamilyName(raw->signal) << ','
                  << osr.pseudorange_rinex_codes[f] << ','
                  << osr.carrier_rinex_codes[f] << ','
                  << (lookup.exact_identity_requested ? 1 : 0) << ','
                  << (lookup.exact_identity_matched ? 1 : 0) << ','
                  << (lookup.family_fallback ? 1 : 0) << ','
                  << static_cast<int>(osr.code_bias_signal_ids[f]) << ','
                  << static_cast<int>(osr.phase_bias_signal_ids[f]) << ','
                  << (osr.bias_exact_identity[f] ? 1 : 0) << ','
                  << static_cast<int>(osr.code_bias_source_signal_ids[f]) << ','
                  << static_cast<int>(osr.phase_bias_source_signal_ids[f]) << ','
                  << (osr.code_bias_present[f] ? 1 : 0) << ','
                  << (osr.phase_bias_present[f] ? 1 : 0) << ','
                  << (osr.code_bias_fallback[f] ? 1 : 0) << ','
                  << (osr.phase_bias_fallback[f] ? 1 : 0) << ','
                  << ','
                  << ','
                  << applied.carrier_phase_correction_m << ','
                  << ','
                  << ','
                  << osr.trop_correction_m << ','
                  << osr.iono_l1_m << ','
                  << osr.stec_tecu << ','
                  << iono_scaled << ','
                  << ','
                  << osr.network_compensation_m << ','
                  << osr.iode_geometry_compensation_m << ','
                  << osr.receiver_antenna_m[f] << ','
                  << osr.relativity_correction_m << ','
                  << timeWeekField(osr.atmos_reference_time, have_atmos_ref) << ','
                  << timeTowField(osr.atmos_reference_time, have_atmos_ref) << ','
                  << timeWeekField(osr.clock_reference_time, have_clock_ref) << ','
                  << timeTowField(osr.clock_reference_time, have_clock_ref) << ','
                  << timeWeekField(osr.code_bias_reference_time, have_code_bias_ref) << ','
                  << timeTowField(osr.code_bias_reference_time, have_code_bias_ref) << ','
                  << secondsField(atmos_clock_gap_s, have_atmos_ref && have_clock_ref) << ','
                  << osr.atmos_network_id << ','
                  << osr.atmos_grid_no << ','
                  << osr.atmos_nearest_grid_distance_m << ','
                  << osr.atmos_interpolation_grid_count << ','
                  << osr.atmos_interpolation_grid_no[0] << ','
                  << osr.atmos_interpolation_weights[0] << ','
                  << osr.atmos_interpolation_grid_no[1] << ','
                  << osr.atmos_interpolation_weights[1] << ','
                  << osr.atmos_interpolation_grid_no[2] << ','
                  << osr.atmos_interpolation_weights[2] << ','
                  << osr.atmos_interpolation_grid_no[3] << ','
                  << osr.atmos_interpolation_weights[3] << ','
                  << (osr.atmos_lifecycle ? 1 : 0) << ','
                  << doubleField(
                         osr.atmos_lifecycle_tow,
                         osr.has_atmos_lifecycle_tow) << ','
                  << osr.atmos_selected_satellite_count << ','
                  << osr.atmos_valid_grid_count << ','
                  << osr.atmos_stec_grid_value_count << ','
                  << osr.atmos_selected_grid_stec_value_count << ','
                  << geo << ','
                  << sat_clk_m << ','
                  << receiver_clock_m << ','
                  << phase_trop_model << ','
                  << iono_state_l1_m << ','
                  << iono_scale << ','
                  << predicted << ','
                  << residual << ','
                  << variance << ','
                  << los_enu.x() << ','
                  << los_enu.y() << ','
                  << los_enu.z() << ','
                  << osr.azimuth << ','
                  << osr.elevation << ','
                  << receiver_position.x() << ','
                  << receiver_position.y() << ','
                  << receiver_position.z();
            writeClasCodeDumpPhasePhaseExtension(
                *dump,
                osr.CPC[f],
                applied.carrier_phase_correction_m,
                osr.CPC[f] - osr.trop_correction_m,
                osr.phase_bias_m[f],
                osr.windup_m[f],
                osr.phase_compensation_m[f],
                l_m,
                l_corr);
            *dump << '\n';
        }
    }
}

inline 
void reinitializePositionState(ppp_shared::PPPState& filter_state,
                               const Vector3d& position,
                               double variance) {
    for (int axis = 0; axis < 3; ++axis) {
        const int index = filter_state.pos_index + axis;
        filter_state.state(index) = position(axis);
        filter_state.covariance.row(index).setZero();
        filter_state.covariance.col(index).setZero();
        filter_state.covariance(index, index) = variance;
    }
}


constexpr double kClasNominalEpochIntervalS = 0.2;
constexpr double kClasOutageGapResetS = 2.0;
// Historical native fallback for paths that do not use MRTKLIB's explicit
// per-frequency observation-outage counter.
constexpr double kClasPerSatOutageEpochs = 2.0;
constexpr double kMinimumGeometryFreeSlipThresholdMeters = 0.05;
// MRTKLIB uses GF-only slip at 0.05 m; MW-mean is native-only. Raised from 0.5
inline // to reduce Galileo burst false slips (see gnss_mrtklib_diff_report.md Q4).
constexpr double kMinimumMwSlipThresholdCycles = 1.0;
constexpr double kMwSlipFallbackThresholdMeters = 10.0;

inline double clasSlipThresholdScale(double dt_seconds) {
    return std::max(1.0, dt_seconds / kClasNominalEpochIntervalS);
}

inline void clearClasWlnlMwState(ppp_shared::PPPAmbiguityInfo& ambiguity) {
    ambiguity.wl_is_fixed = false;
    ambiguity.wl_fixed_integer = 0;
    ambiguity.nl_is_fixed = false;
    ambiguity.nl_fixed_cycles = 0.0;
    ambiguity.mw_sum_cycles = 0.0;
    ambiguity.mw_count = 0;
    ambiguity.mw_mean_cycles = 0.0;
}

inline void resetClasPhaseBiasRepair(CLASPhaseBiasRepairInfo& repair) {
    repair.reference_time = GNSSTime();
    repair.last_continuity_m = {0.0, 0.0, 0.0};
    repair.offset_cycles = {0.0, 0.0, 0.0};
    repair.pending_state_shift_cycles = {0.0, 0.0, 0.0};
    repair.has_last = {false, false, false};
}

inline const Observation* findMrtklibParityRawObservation(
    const ObservationData& obs,
    const OSRCorrection& osr,
    int frequency_index) {
    if (osr.satellite.system != GNSSSystem::GPS) {
        return findOsrFrequencyObservation(obs, osr, frequency_index);
    }
    static constexpr char kGpsL2Priority[] = "PYWCMNDLXS";

    // MRTKLIB v0.5.1 getcodepri(): GPS slot 0 is L1C/A and slot 1 uses
    // "PYWCMNDLXS" priority.  The RINEX reader stores all tracking codes so
    // this lookup does not silently substitute the native L2C selection for
    // the L2W slot used by the reference run.
    if (const std::string* slot = obs.getRinexFrequencySlot(
            osr.satellite.system, frequency_index)) {
        return obs.getRinexTrackingObservation(osr.satellite, *slot);
    }
    if (frequency_index == 0) {
        if (const Observation* exact =
                obs.getRinexTrackingObservation(osr.satellite, "1C")) {
            return exact;
        }
    } else if (frequency_index == 1) {
        for (const char* priority = kGpsL2Priority; *priority != '\0'; ++priority) {
            const char tracking = *priority;
            const std::string key = std::string("2") + tracking;
            const Observation* exact =
                obs.getRinexTrackingObservation(osr.satellite, key);
            if (exact != nullptr && exact->has_pseudorange &&
                exact->has_carrier_phase) {
                return exact;
            }
        }
    }

    // Hand-built ObservationData in callers predating the complete RINEX
    // store has no auxiliary entries.  Accept it only when its provenance is
    // already the code that MRTKLIB would place in the slot.
    const Observation* selected =
        findOsrFrequencyObservation(obs, osr, frequency_index);
    if (selected == nullptr) return nullptr;
    if (frequency_index == 0) {
        if (obs.rinex_tracking_observations.empty()) return selected;
        return selected->pseudorange_observation_type == "C1C" &&
                       selected->carrier_phase_observation_type == "L1C"
                   ? selected
                   : nullptr;
    }
    if (frequency_index == 1) {
        const std::string& code = selected->pseudorange_observation_type;
        const std::string& phase = selected->carrier_phase_observation_type;
        if (obs.rinex_tracking_observations.empty() && code.empty() && phase.empty()) {
            return selected;
        }
        for (const char* priority = kGpsL2Priority; *priority != '\0'; ++priority) {
            const char tracking = *priority;
            const std::string suffix = std::string("2") + tracking;
            if (code == "C" + suffix && phase == "L" + suffix) {
                return selected;
            }
        }
        return nullptr;
    }
    return selected;
}

}  // namespace internal

// Cross-TU free functions: definitions live in ppp_clas.cpp /
// ppp_clas_measurements.cpp; declared here for the sibling TUs.
bool suppressesClasAmbDatumPhaseTrop(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy);
bool usesResidualClasAmbDatumPhaseTrop(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy);
double effectiveClasTropPriorVariance(const ppp_shared::PPPConfig& config);
}  // namespace libgnss::ppp_clas
