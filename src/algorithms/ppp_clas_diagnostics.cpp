#include "ppp_clas_diagnostics.hpp"

#include <libgnss++/algorithms/ppp_env_overrides.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <string>

namespace libgnss::ppp_clas_diagnostics {

namespace {

const char* phaseExtensionHeader() {
    return phaseRowDumpEnabled()
        ? ",row_type,cpc_m,carrier_correction_m,cpc_minus_trop_m,phase_bias_m,"
          "windup_m,phase_compensation_m,raw_l_m,corrected_l_m"
        : "";
}

}  // namespace

std::ofstream* ambiguityDatumStream() {
    const auto& path = pppEnvOverrides().clas_amb_datum_dump_path;
    if (path.empty()) {
        return nullptr;
    }
    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,sat,freq,signal,carrier_rinex_code,"
                   << "carrier_rtklib_code,signal_family,"
                   << "requested_pseudorange_rinex_code,"
                   << "requested_carrier_rinex_code,"
                   << "observation_exact_identity_requested,"
                   << "observation_exact_match,observation_family_fallback,"
                   << "phase_bias_signal_id,bias_exact_identity,"
                   << "phase_bias_source_signal_id,phase_bias_present,"
                   << "phase_bias_fallback,lambda_m,"
                   << "raw_phase_cycles,raw_phase_m,carrier_correction_m,"
                   << "cpc_m,cpc_minus_trop_m,trop_correction_m,relativity_m,"
                   << "receiver_antenna_m,iono_cpc_m,phase_bias_m,windup_m,"
                   << "phase_compensation_m,l_corr_m,predicted_no_amb_m,"
                   << "amb_state_m,amb_state_cycles,residual_m\n";
        }
    }
    return stream ? &stream : nullptr;
}

std::ofstream* geometryDumpStream() {
    const auto& path = pppEnvOverrides().clas_geom_dump_path;
    if (path.empty()) {
        return nullptr;
    }
    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,tx_tow,sat,freq,signal,"
                   << "rx_state_x_m,rx_state_y_m,rx_state_z_m,"
                   << "rx_row_x_m,rx_row_y_m,rx_row_z_m,"
                   << "rx_forced_x_m,rx_forced_y_m,rx_forced_z_m,"
                   << "tide_x_m,tide_y_m,tide_z_m,"
                   << "sat_x_m,sat_y_m,sat_z_m,sat_vx_mps,sat_vy_mps,sat_vz_mps,"
                   << "sat_clk_m,euclidean_m,sagnac_m,rho_m,az_rad,el_rad,"
                   << "cpc_m,model_phase_m\n";
        }
    }
    return stream ? &stream : nullptr;
}

bool phaseRowDumpEnabled() {
    return pppEnvOverrides().clas_phase_row_dump;
}

void writeCodePhaseExtension(std::ostream& out) {
    if (phaseRowDumpEnabled()) {
        out << ",code,,,,,,,,";
    }
}

void writePhasePhaseExtension(
    std::ostream& out,
    double cpc_m,
    double carrier_correction_m,
    double cpc_minus_trop_m,
    double phase_bias_m,
    double windup_m,
    double phase_compensation_m,
    double raw_l_m,
    double corrected_l_m) {
    if (!phaseRowDumpEnabled()) {
        return;
    }
    out << ",phase," << cpc_m << ',' << carrier_correction_m << ','
        << cpc_minus_trop_m << ',' << phase_bias_m << ',' << windup_m << ','
        << phase_compensation_m << ',' << raw_l_m << ',' << corrected_l_m;
}

std::ofstream* codeDumpStream() {
    const auto& path = pppEnvOverrides().clas_code_dump_path;
    if (path.empty()) {
        return nullptr;
    }
    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,stage,week,tow,sat,freq,signal,"
                   << "pseudorange_rinex_code,carrier_rinex_code,"
                   << "pseudorange_rtklib_code,carrier_rtklib_code,signal_family,"
                   << "requested_pseudorange_rinex_code,"
                   << "requested_carrier_rinex_code,"
                   << "observation_exact_identity_requested,"
                   << "observation_exact_match,observation_family_fallback,"
                   << "code_bias_signal_id,phase_bias_signal_id,"
                   << "bias_exact_identity,code_bias_source_signal_id,"
                   << "phase_bias_source_signal_id,code_bias_present,"
                   << "phase_bias_present,code_bias_fallback,phase_bias_fallback,"
                   << "raw_p_m,corrected_p_m,applied_pr_corr_m,prc_m,"
                   << "prc_minus_trop_m,trop_correction_m,iono_l1_m,"
                   << "stec_tecu,iono_scaled_m,code_bias_m,network_compensation_m,"
                   << "iode_geometry_compensation_m,"
                   << "receiver_ant_m,relativity_m,"
                   << "atmos_ref_week,atmos_ref_tow,clock_ref_week,clock_ref_tow,"
                   << "code_bias_ref_week,code_bias_ref_tow,"
                   << "atmos_clock_gap_s,atmos_network_id,atmos_grid_no,"
                   << "atmos_grid_distance_m,atmos_grid_count,"
                   << "atmos_grid1_no,atmos_grid1_weight,"
                   << "atmos_grid2_no,atmos_grid2_weight,"
                   << "atmos_grid3_no,atmos_grid3_weight,"
                   << "atmos_grid4_no,atmos_grid4_weight,"
                   << "atmos_lifecycle,atmos_lifecycle_tow,"
                   << "atmos_selected_satellite_count,"
                   << "atmos_valid_grid_count,atmos_stec_grid_value_count,"
                   << "atmos_selected_grid_stec_value_count,"
                   << "geo_m,sat_clk_m,receiver_clock_m,trop_model_m,"
                   << "iono_state_m,iono_scale,predicted_m,residual_m,"
                   << "variance_m2,los_e_m,los_n_m,los_u_m,az_rad,el_rad,"
                   << "rx_x_m,rx_y_m,rx_z_m" << phaseExtensionHeader() << '\n';
        }
    }
    return stream ? &stream : nullptr;
}

bool selectedGeometryDumpTow(double tow) {
    constexpr std::array<double, 3> targets{230572.0, 232034.0, 234018.0};
    return std::any_of(targets.begin(), targets.end(), [tow](double target) {
        return std::abs(tow - target) < 0.01;
    });
}

Eigen::Vector3d geometryDumpReceiverPosition(const Eigen::Vector3d& fallback) {
    const auto& spec = pppEnvOverrides().clas_geom_dump_rx_xyz;
    if (spec.empty()) {
        return fallback;
    }
    std::string normalized = spec;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::istringstream input(normalized);
    Eigen::Vector3d parsed = fallback;
    if (input >> parsed.x() >> parsed.y() >> parsed.z()) {
        return parsed;
    }
    return fallback;
}

}  // namespace libgnss::ppp_clas_diagnostics
