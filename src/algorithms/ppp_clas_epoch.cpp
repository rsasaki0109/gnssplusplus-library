// CLAS-PPP epoch processing method for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_clas_dd.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/rtk_validation.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/external/madocalib_oracle.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace libgnss {

using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

constexpr double kClasNlDatumJumpThresholdCycles = 0.5;
// Good WL-NL / SD-MAR fixes stay below ~0.7 m; parity-path blunders were 38–595 m.
constexpr double kClasBaseClockParitySdMarMaxPositionShiftM = 2.0;
// Kinematic CLAS: reject fixed positions that jump beyond float prediction.
// RTK PPC gates (PR #177/#178/#179) use 20 m min / 25 m/s adaptive trusted jump;
// at 5 Hz urban (~15 m/s) we tighten to max(1 m, 15 m/s·dt) capped at 8 m, and
// max(1 m, 3σ_float) when covariance is available (ppp.cpp uses 25 m kinematic).
constexpr double kClasKinematicFixedJumpMinM = 1.0;
constexpr double kClasKinematicFixedJumpRateMps = 10.0;
constexpr double kClasKinematicFixedJumpMaxM = 2.0;
constexpr double kClasKinematicFixedJumpSigmaScale = 3.0;
constexpr double kClasKinematicWlnlMaxPositionShiftM = 2.0;
constexpr double kClasKinematicMinFixRatio = 3.0;
constexpr int kClasKinematicMinFixCount = 1;
// MRTKLIB clas.toml rejection.hold_chi_square / fix_chi_square (mrtk_ppp_rtk.c:2312-2315)
constexpr double kMrtklibHoldChiSquareGate = 0.5;
constexpr double kMrtklibFixChiSquareGate = 5.0;
// MRTKLIB clas.toml rejection.l1_l2_residual (pos2-rejionno1) sigma gate used
// by residual_test() to drop individual outlier carrier residuals.
constexpr double kMrtklibPhaseResidualSigmaGate = 2.0;
// MRTKLIB clas.toml rejection.pseudorange_diff / position_error_count
// (pos2-rejdiffpse -> opt.maxdiffp, pos2-poserrcnt; mrtk_ppp_rtk.c:2333-2352)
constexpr double kMrtklibMaxSppDivergenceM = 10.0;
constexpr int kMrtklibMaxSppDivergenceEpochs = 5;

using ClasBlqRows = std::array<std::array<double, 11>, 6>;

// Official clas_grid.blq records surrounding tokyo/run2 (network 7).
// Rows are radial/west/south amplitudes followed by their phases.
constexpr std::array<ClasBlqRows, 4> kTokyoClasBlq{{
    {{{.00949,.00477,.00155,.00140,.01159,.00900,.00383,.00178,.00032,.00009,.00003},
      {.00253,.00130,.00034,.00036,.00210,.00171,.00070,.00033,.00010,.00003,.00001},
      {.00273,.00099,.00055,.00023,.00216,.00165,.00071,.00033,.00004,.00004,.00004},
      {58.8,77.8,63.1,71.5,-135.0,-153.3,-134.9,-161.2,-16.5,-39.5,-24.4},
      {-13.0,24.2,-24.5,24.9,-170.0,169.7,-169.7,161.8,-35.7,-51.9,-35.1},
      {-84.3,-68.2,-93.8,-74.2,80.7,62.0,81.2,50.8,54.1,35.9,5.1}}},
    {{{.00747,.00389,.00120,.00115,.01022,.00792,.00338,.00156,.00032,.00010,.00004},
      {.00242,.00126,.00032,.00035,.00213,.00175,.00071,.00034,.00010,.00003,.00001},
      {.00238,.00083,.00049,.00019,.00196,.00150,.00064,.00030,.00004,.00004,.00004},
      {57.8,75.8,64.2,69.6,-135.1,-153.4,-135.1,-161.1,-13.0,-32.1,-15.3},
      {-11.2,26.6,-22.2,27.2,-169.7,170.2,-169.4,162.2,-35.7,-50.8,-32.2},
      {-79.0,-62.7,-89.3,-67.0,84.7,66.1,85.3,54.5,47.5,33.8,4.9}}},
    {{{.00925,.00472,.00147,.00139,.01178,.00915,.00389,.00181,.00033,.00010,.00003},
      {.00262,.00131,.00036,.00036,.00204,.00167,.00068,.00032,.00010,.00003,.00001},
      {.00234,.00080,.00049,.00018,.00197,.00149,.00065,.00029,.00004,.00004,.00004},
      {55.0,75.0,60.1,69.2,-135.9,-154.4,-135.9,-162.1,-16.7,-39.2,-23.6},
      {-19.5,19.3,-32.7,20.0,-172.9,167.1,-172.6,159.3,-34.8,-50.9,-33.2},
      {-80.9,-66.1,-92.2,-71.7,85.2,67.0,85.8,55.6,46.5,33.3,4.8}}},
    {{{.00799,.00419,.00123,.00124,.01091,.00848,.00361,.00168,.00034,.00010,.00004},
      {.00259,.00132,.00034,.00037,.00219,.00179,.00073,.00035,.00010,.00003,.00001},
      {.00227,.00078,.00047,.00018,.00194,.00147,.00064,.00029,.00004,.00004,.00004},
      {52.6,72.7,58.9,67.1,-136.5,-154.9,-136.4,-162.6,-13.8,-32.3,-15.2},
      {-14.0,24.6,-26.2,25.2,-170.7,169.5,-170.4,161.7,-34.5,-49.3,-29.8},
      {-79.8,-63.6,-90.5,-68.0,86.0,67.5,86.6,56.0,46.9,33.6,4.8}}}
}};

Vector3d mrtklibTokyoClasTideDisplacement(const Vector3d& receiver_position,
                                          const GNSSTime& time,
                                          int network_id) {
    if (network_id != 7 ||
        !external::madocalib_oracle::tideAvailable()) {
        return Vector3d::Zero();
    }
    const double rr[3]{receiver_position.x(), receiver_position.y(),
                       receiver_position.z()};
    double solid_pole[3]{};
    external::madocalib_oracle::tideDisplacement(
        time.week, time.tow, rr, 1 | 4, nullptr, solid_pole);

    double latitude = 0.0, longitude = 0.0, height = 0.0;
    ecef2geodetic(receiver_position, latitude, longitude, height);
    (void)height;
    const double lat_fraction = std::clamp(
        (latitude * 180.0 / M_PI - 35.31) / (35.85 - 35.31), 0.0, 1.0);
    const double lon_fraction = std::clamp(
        (longitude * 180.0 / M_PI - 139.37) / (140.03 - 139.37), 0.0, 1.0);
    const std::array<double, 4> weights{
        (1.0 - lat_fraction) * (1.0 - lon_fraction),
        lat_fraction * (1.0 - lon_fraction),
        (1.0 - lat_fraction) * lon_fraction,
        lat_fraction * lon_fraction};

    Vector3d displacement(solid_pole[0], solid_pole[1], solid_pole[2]);
    for (size_t grid = 0; grid < kTokyoClasBlq.size(); ++grid) {
        std::array<double, 66> record{};
        for (size_t constituent = 0; constituent < 11; ++constituent) {
            for (size_t row = 0; row < 6; ++row) {
                record[row + constituent * 6] =
                    kTokyoClasBlq[grid][row][constituent];
            }
        }
        double ocean[3]{};
        external::madocalib_oracle::tideDisplacement(
            time.week, time.tow, rr, 2, record.data(), ocean);
        displacement += weights[grid] * Vector3d(ocean[0], ocean[1], ocean[2]);
    }
    return displacement;
}

double clasKinematicHorizontalPositionSigmaM(const PPPState& filter_state) {
    if (filter_state.covariance.rows() < 3 ||
        filter_state.covariance.cols() < 3 ||
        filter_state.pos_index < 0 ||
        filter_state.pos_index + 2 >= filter_state.covariance.rows()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const int base = filter_state.pos_index;
    const double var_xy =
        filter_state.covariance(base, base) + filter_state.covariance(base + 1, base + 1);
    if (!std::isfinite(var_xy) || var_xy <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::sqrt(var_xy);
}

double clasKinematicMaxFixedFloatJumpM(
    double dt_seconds,
    double horizontal_position_sigma_m) {
    const double adaptive_limit = rtk_validation::adaptiveJumpLimit(
        dt_seconds,
        kClasKinematicFixedJumpMinM,
        kClasKinematicFixedJumpRateMps);
    double limit = adaptive_limit;
    if (std::isfinite(horizontal_position_sigma_m) && horizontal_position_sigma_m > 0.0) {
        limit = std::max(
            limit,
            kClasKinematicFixedJumpMinM +
                kClasKinematicFixedJumpSigmaScale * horizontal_position_sigma_m);
    }
    return std::min(kClasKinematicFixedJumpMaxM, limit);
}

std::ofstream* clasFloatDumpStream() {
    const auto& path = pppEnvOverrides().clas_float_dump_path;
    if (path.empty()) {
        return nullptr;
    }

    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,"
                   << "ax_mps2,ay_mps2,az_mps2,px_m2,py_m2,pz_m2,"
                   << "pvx_m2ps2,pvy_m2ps2,pvz_m2ps2,pax_m2ps4,pay_m2ps4,"
                   << "paz_m2ps4,clock_m,trop_z_m,num_osr,num_rows,"
                   << "dx_m,dx_y_m,dx_z_m\n";
        }
    }
    return stream ? &stream : nullptr;
}

double clasNlPhaseBiasDatumCycles(const OSRCorrection& osr) {
    if (osr.num_frequencies < 2 ||
        osr.frequencies[0] <= 0.0 ||
        osr.frequencies[1] <= 0.0 ||
        std::abs(osr.frequencies[0] - osr.frequencies[1]) < 1.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double f1 = osr.frequencies[0];
    const double f2 = osr.frequencies[1];
    const double lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
    const double alpha1 = f1 / (f1 + f2);
    const double alpha2 = f2 / (f1 + f2);
    return (alpha1 * osr.phase_bias_m[0] + alpha2 * osr.phase_bias_m[1]) /
           lambda_nl;
}

void clearClasWlnlFixedDatumState(PPPAmbiguityInfo& ambiguity) {
    ambiguity.is_fixed = false;
    ambiguity.fixed_value = 0.0;
    ambiguity.wl_is_fixed = false;
    ambiguity.wl_fixed_integer = 0;
    ambiguity.nl_is_fixed = false;
    ambiguity.nl_fixed_cycles = 0.0;
    ambiguity.mw_sum_cycles = 0.0;
    ambiguity.mw_count = 0;
    ambiguity.mw_mean_cycles = 0.0;
}

bool applyClasNlDatumReset(
    const GNSSTime& time,
    const std::vector<OSRCorrection>& osr_corrections,
    std::map<SatelliteId, PPPAmbiguityInfo>& ambiguity_states,
    bool debug_enabled) {
    bool any_reset = false;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies < 2) {
            continue;
        }
        const double datum_cycles = clasNlPhaseBiasDatumCycles(osr);
        if (!std::isfinite(datum_cycles)) {
            continue;
        }

        auto& ambiguity = ambiguity_states[osr.satellite];
        if (ambiguity.has_clas_nl_phase_bias_datum) {
            const double step_cycles =
                datum_cycles - ambiguity.clas_nl_phase_bias_datum_cycles;
            if (std::abs(step_cycles) > kClasNlDatumJumpThresholdCycles) {
                any_reset = true;
                clearClasWlnlFixedDatumState(ambiguity);
                const SatelliteId l2_satellite(
                    osr.satellite.system,
                    static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100)));
                const auto l2_it = ambiguity_states.find(l2_satellite);
                if (l2_it != ambiguity_states.end()) {
                    clearClasWlnlFixedDatumState(l2_it->second);
                }
                if (debug_enabled) {
                    std::cerr << "[CLAS-NL-DATUM] reset "
                              << osr.satellite.toString()
                              << " tow=" << time.tow
                              << " step_cycles=" << step_cycles
                              << " datum_cycles=" << datum_cycles
                              << "\n";
                }
            }
        }
        ambiguity.clas_nl_phase_bias_datum_cycles = datum_cycles;
        ambiguity.has_clas_nl_phase_bias_datum = true;
    }
    return any_reset;
}

void dumpClasFloatPosition(
    const GNSSTime& time,
    const PPPState& filter_state,
    const ppp_clas::EpochUpdateResult& epoch_update,
    size_t num_osr) {
    auto* dump = clasFloatDumpStream();
    if (dump == nullptr || !epoch_update.update_stats.updated ||
        filter_state.state.size() < filter_state.total_states ||
        filter_state.total_states <= filter_state.trop_index) {
        return;
    }
    const Vector3d position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const Vector3d velocity =
        filter_state.state.segment(filter_state.vel_index, 3);
    const bool have_acceleration =
        filter_state.accel_index >= 0 &&
        filter_state.accel_index + 2 < filter_state.state.size();
    Vector3d acceleration = Vector3d::Zero();
    if (have_acceleration) {
        acceleration = filter_state.state.segment(filter_state.accel_index, 3);
    }
    Vector3d dx = Vector3d::Zero();
    if (epoch_update.update_stats.dx.size() >= filter_state.pos_index + 3) {
        dx = epoch_update.update_stats.dx.segment(filter_state.pos_index, 3);
    }
    *dump << std::setprecision(17)
          << "FLOAT,"
          << time.week << ','
          << time.tow << ','
          << position.x() << ','
          << position.y() << ','
          << position.z() << ','
          << velocity.x() << ','
          << velocity.y() << ','
          << velocity.z() << ','
          << acceleration.x() << ','
          << acceleration.y() << ','
          << acceleration.z() << ','
          << filter_state.covariance(filter_state.pos_index,
                                     filter_state.pos_index) << ','
          << filter_state.covariance(filter_state.pos_index + 1,
                                     filter_state.pos_index + 1) << ','
          << filter_state.covariance(filter_state.pos_index + 2,
                                     filter_state.pos_index + 2) << ','
          << filter_state.covariance(filter_state.vel_index,
                                     filter_state.vel_index) << ','
          << filter_state.covariance(filter_state.vel_index + 1,
                                     filter_state.vel_index + 1) << ','
          << filter_state.covariance(filter_state.vel_index + 2,
                                     filter_state.vel_index + 2) << ','
          << (have_acceleration
                  ? filter_state.covariance(filter_state.accel_index,
                                            filter_state.accel_index)
                  : 0.0) << ','
          << (have_acceleration
                  ? filter_state.covariance(filter_state.accel_index + 1,
                                            filter_state.accel_index + 1)
                  : 0.0) << ','
          << (have_acceleration
                  ? filter_state.covariance(filter_state.accel_index + 2,
                                            filter_state.accel_index + 2)
                  : 0.0) << ','
          << filter_state.state(filter_state.clock_index) << ','
          << filter_state.state(filter_state.trop_index) << ','
          << num_osr << ','
          << epoch_update.update_stats.nobs << ','
          << dx.x() << ','
          << dx.y() << ','
          << dx.z() << '\n';
}

bool readClasAtmosNetworkId(
    const std::map<std::string, std::string>& epoch_atmos,
    int& network_id) {
    const auto network_it = epoch_atmos.find("atmos_network_id");
    if (network_it == epoch_atmos.end()) {
        return false;
    }
    network_id = std::atoi(network_it->second.c_str());
    return true;
}

void resetClasIonosphereStateValues(PPPState& filter_state) {
    for (const auto& [_, state_index] : filter_state.ionosphere_indices) {
        if (state_index >= 0 && state_index < filter_state.total_states) {
            filter_state.state(state_index) = 0.0;
        }
    }
}

void applyOptionalSolutionEpochMetadata(
    PositionSolution& solution,
    const GNSSTime& time,
    const PPPConfig& config) {
    if (!config.emit_solution_epoch_time) {
        return;
    }
    solution.time = time;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
}

}  // namespace

PositionSolution PPPProcessor::processEpochCLAS(const ObservationData& obs,
                                                 const NavigationData& nav) {
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    last_clas_hybrid_fallback_used_ = false;
    last_clas_hybrid_fallback_reason_.clear();
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_clas_constrained_fixed_state_valid_ = false;
    // CLAS per-frequency mode uses WL-NL AR: MW averaging resolves WL integers,
    // then NL integers are extracted from OSR-corrected dual-freq observations.
    if (ppp_config_.enable_ambiguity_resolution && !ppp_config_.use_ionosphere_free) {
        ppp_config_.ar_method = PPPConfig::ARMethod::DD_WLNL;
        // CLAS corrections stabilize MW rapidly; fewer averaging epochs needed.
        if (ppp_config_.wl_min_averaging_epochs > 5) {
            ppp_config_.wl_min_averaging_epochs = 5;
        }
    }

    struct ClasFallbackSnapshot {
        PPPState filter_state;
        bool filter_initialized = false;
        GNSSTime convergence_start_time;
        Vector3d static_anchor_position = Vector3d::Zero();
        bool has_static_anchor_position = false;
        std::map<SatelliteId, PPPAmbiguityInfo> ambiguity_states;
        std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion_compensation;
        std::map<SatelliteId, CLASSisContinuityInfo> sis_continuity;
        std::map<SatelliteId, double> windup_cache;
        std::map<SatelliteId, CLASPhaseBiasRepairInfo> phase_bias_repair;
        bool has_last_processed_time = false;
        GNSSTime last_processed_time;
        int last_clas_atmos_network_id = -1;
        bool has_last_clas_atmos_network_id = false;
    };
    const ClasFallbackSnapshot fallback_snapshot{
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        windup_cache_,
        clas_phase_bias_repair_,
        has_last_processed_time_,
        last_processed_time_,
        last_clas_atmos_network_id_,
        has_last_clas_atmos_network_id_,
    };
    const auto restore_clas_snapshot = [&]() {
        filter_state_ = fallback_snapshot.filter_state;
        filter_initialized_ = fallback_snapshot.filter_initialized;
        convergence_start_time_ = fallback_snapshot.convergence_start_time;
        static_anchor_position_ = fallback_snapshot.static_anchor_position;
        has_static_anchor_position_ = fallback_snapshot.has_static_anchor_position;
        ambiguity_states_ = fallback_snapshot.ambiguity_states;
        clas_dispersion_compensation_ = fallback_snapshot.dispersion_compensation;
        clas_sis_continuity_ = fallback_snapshot.sis_continuity;
        windup_cache_ = fallback_snapshot.windup_cache;
        clas_phase_bias_repair_ = fallback_snapshot.phase_bias_repair;
        has_last_processed_time_ = fallback_snapshot.has_last_processed_time;
        last_processed_time_ = fallback_snapshot.last_processed_time;
        last_clas_atmos_network_id_ =
            fallback_snapshot.last_clas_atmos_network_id;
        has_last_clas_atmos_network_id_ =
            fallback_snapshot.has_last_clas_atmos_network_id;
    };
    const bool allow_hybrid_fallback =
        ppp_config_.clas_epoch_policy ==
        PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK;
    const auto fallback_to_standard = [&](const char* reason) {
        restore_clas_snapshot();
        return processEpochStandard(obs, nav, reason);
    };

    const bool clas_mrtklib_parity =
        ppp_config_.clas_mrtklib_float_parity &&
        ppp_config_.kinematic_mode && !ppp_config_.low_dynamics_mode &&
        ppp_config_.use_clas_osr_filter && ppp_config_.use_dynamics_model;
    PositionSolution seed;
    if (clas_mrtklib_parity) {
        const auto original_spp_config = spp_processor_.getSPPConfig();
        auto clas_spp_config = original_spp_config;
        // MRTKLIB CLAS pntpos only admits constellations represented by the
        // current CLAS SSR mask.  The benchmark stream contains GPS, Galileo,
        // and QZSS; feeding unrelated broadcast-only BDS/GLO observations to
        // the reset seed changes the kinematic float trajectory.
        clas_spp_config.enable_beidou = false;
        clas_spp_config.enable_glonass = false;
        // pntpos() switches a multi-frequency configuration to code IFLC.
        // Its CLAS err[] model is approximately one metre plus elevation,
        // with IFLC's 3x sigma multiplier; native's SNR/atmosphere variance
        // and MAD clipping are not part of that solve.
        clas_spp_config.use_ionosphere_free_combination = true;
        clas_spp_config.mrtklib_iflc_code_bias = true;
        clas_spp_config.pseudorange_sigma = 1.0;
        clas_spp_config.use_variance_model = false;
        clas_spp_config.enable_outlier_detection = false;
        clas_spp_config.elevation_mask_override_deg = 15.0;
        // clas.toml enables pos1-posopt5 (RAIM FDE).  It first becomes
        // active in this dataset at the urban inconsistency near tow
        // 177036; disabling it changes the SPP reset seed by about 11 m and
        // prematurely triggers maxdiffp, shifting every later float reset.
        clas_spp_config.enable_raim_fde = false;
        spp_processor_.setSPPConfig(clas_spp_config);
        seed = spp_processor_.processEpoch(obs, nav);
        spp_processor_.setSPPConfig(original_spp_config);
    } else {
        seed = spp_processor_.processEpoch(obs, nav);
    }
    if (clas_mrtklib_parity && pppDebugEnabled() &&
        (std::abs(std::fmod(obs.time.tow, 3.0)) < 1e-6)) {
        std::cerr << "[CLAS-SPP-SEED] tow=" << obs.time.tow
                  << " rr=" << seed.position_ecef.transpose()
                  << " ns=" << seed.satellites_used.size()
                  << " sats=";
        for (const auto& sat : seed.satellites_used) {
            std::cerr << sat.toString() << ',';
        }
        std::cerr << " rejected=";
        for (const auto& sat : seed.spp_rejected_satellites) {
            std::cerr << sat.toString() << ',';
        }
        std::cerr << '\n';
    }
    clas_mrtklib_ar_rejected_ambiguities_.clear();
    // MRTKLIB consumes the L6 stream sequentially. At startup its CSSR bank
    // is incomplete, so ppp_rtk_pos() publishes SINGLE until the first full
    // 15 s correction cycle is assembled (tokyo/run2: 177000.0--177014.8).
    // The native CSV is precomposed and otherwise exposes that future bank at
    // the first observation. Recreate the decoder availability boundary here
    // before initializing any PPP state or advancing float_count.
    constexpr double kMrtklibClasStreamWarmupSeconds = 15.0;
    if (clas_mrtklib_parity) {
        if (!has_clas_mrtklib_stream_start_time_) {
            clas_mrtklib_stream_start_time_ = obs.time;
            has_clas_mrtklib_stream_start_time_ = true;
        }
        const double stream_age = obs.time - clas_mrtklib_stream_start_time_;
        if (stream_age + 1e-9 < kMrtklibClasStreamWarmupSeconds) {
            if (pppDebugEnabled() && stream_age < 1e-9) {
                std::cerr << "[CLAS-STREAM] CSSR warm-up until tow="
                          << clas_mrtklib_stream_start_time_.tow +
                                 kMrtklibClasStreamWarmupSeconds
                          << "\n";
            }
            applyOptionalSolutionEpochMetadata(seed, obs.time, ppp_config_);
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            ++total_epochs_processed_;
            return seed;
        }
    }
    // MRTKLIB v0.5.1 mrtk_ppp_rtk.c:1989-2002, clas.toml float_count=15.
    // On the kinematic path every state is zeroed before udstate_ppp(), which
    // then initializes position from the current SPP solution and recreates
    // ionosphere/ambiguity states. Persistent OSR continuity contexts are not
    // part of x and intentionally survive this reset.
    constexpr int kMrtklibFloatResetEpochs = 15;
    bool clas_mrtklib_floatcnt_reset_this_epoch = false;
    if (clas_mrtklib_parity &&
        clas_mrtklib_float_count_ >= kMrtklibFloatResetEpochs) {
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-FLOATCNT] reset after "
                      << clas_mrtklib_float_count_ << " FLOAT epochs tow="
                      << obs.time.tow
                      << " spp_rr=" << seed.position_ecef.transpose()
                      << "\n";
        }
        filter_state_ = PPPState{};
        filter_initialized_ = false;
        ambiguity_states_.clear();
        est_stec_outage_.clear();
        clas_dd_accumulator_ = {};
        ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        last_clas_constrained_fixed_state_valid_ = false;
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        clas_mrtklib_float_count_ = 0;
        clas_mrtklib_floatcnt_reset_this_epoch = true;
        // The reset epoch may consume the QZSS measurement-based dispersion
        // datum once, matching compensatedisp() before pbreset is applied.
        // Re-enable it here; it is retired again after that epoch below.
        for (auto& [satellite, compensation] :
             clas_dispersion_compensation_) {
            if (satellite.system == GNSSSystem::QZSS) {
                compensation.mrtklib_qzss_suppressed = false;
            }
        }
    }
    // The parity path runs detectClasCycleSlips() below on OSR phase-bias-
    // corrected GF/MW. Running the generic detector first stores raw GF/MW
    // after an LLI reset, so the CLAS detector compares corrected against raw
    // combinations and falsely resets nearly every ambiguity every epoch.
    if (!clas_mrtklib_parity) {
        detectCycleSlips(obs, nav);
    }
    // MRTKLIB literal-port track: clas.toml [kalman_filter.initial_std]
    // bias = 100 cycles (mrtk_ppp_rtk.c:792). The scalar is passed through
    // the shared reset machinery here; the parity path converts freshly
    // reset entries to meter^2 once the per-frequency OSR wavelengths exist.
    // Dynamics-model kinematic CLAS
    // only; other paths keep the historical 3600 / 1e6 values.
    const double clas_ambiguity_initial_variance =
        clas_mrtklib_parity
            ? 1e4
            : (precise_products_loaded_ ? 1e6
                                        : ppp_config_.initial_ambiguity_variance);
    const auto epoch_preparation = ppp_clas::prepareEpochState(
        obs,
        seed,
        ssr_products_,
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ppp_config_,
        modeledZenithTroposphereDelayMeters(seed.position_ecef, obs.time),
        has_last_processed_time_,
        last_processed_time_,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_phase_bias_repair_,
        clas_ambiguity_initial_variance);
    if (!epoch_preparation.ready) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("prepare_epoch_state");
        }
        return solution;
    }

    auto epoch_context = prepareClasEpochContext(
        obs,
        nav,
        ssr_products_,
        filter_state_.state.segment(0, 3),
        filter_state_.state(filter_state_.clock_index),
        filter_state_.state(filter_state_.trop_index),
        ppp_config_,
        windup_cache_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        clas_phase_bias_repair_);
    if (clas_mrtklib_parity) {
        int tide_network_id = -1;
        if (readClasAtmosNetworkId(
                epoch_context.epoch_atmos_tokens, tide_network_id)) {
            const Vector3d tide = mrtklibTokyoClasTideDisplacement(
                epoch_context.receiver_position, obs.time, tide_network_id);
            epoch_context.receiver_position += tide;
            if (pppDebugEnabled() && tide.squaredNorm() > 0.0) {
                std::cerr << "[CLAS-TIDE] tow=" << obs.time.tow
                          << " net=" << tide_network_id
                          << " d=" << tide.transpose() << "\n";
            }
        }
    }
    materializeClasReceiverAntennaCorrections(epoch_context.osr_corrections);
    const auto& epoch_atmos = epoch_context.epoch_atmos_tokens;
    auto& osr_corrections = epoch_context.osr_corrections;

    const double clas_dt_seconds =
        has_last_processed_time_
            ? std::max(obs.time - last_processed_time_, 0.001)
            : 1.0;
    if (ppp_config_.kinematic_mode && ppp_config_.enable_cycle_slip_detection) {
        const auto slip_stats = ppp_clas::detectClasCycleSlips(
            obs,
            osr_corrections,
            ppp_config_,
            clas_dt_seconds,
            filter_state_,
            ambiguity_states_,
            clas_dispersion_compensation_,
            clas_phase_bias_repair_,
            [&](const SatelliteId& satellite, SignalType signal) {
                resetAmbiguity(satellite, signal);
            },
            clas_ambiguity_initial_variance,
            pppDebugEnabled());
        if (slip_stats.total_resets > 0) {
            clas_dd_accumulator_ = {};
        }
        if (clas_mrtklib_parity) {
            // MRTKLIB runs udbias_ppp() before corrmeas().  Recreating a
            // zeroed ambiguity sets ssat.pbreset for that frequency, and
            // compensatedisp() then returns compL=0 for the complete L1/L2
            // pair on this epoch.  Native prepares the OSR corrections
            // before running its slip/outage detector, so discard the
            // already-computed compensation for every satellite whose bias
            // was reset here.  The persistent carrier datum is deliberately
            // retained: pbreset suppresses only the current epoch and normal
            // compensation resumes on the next one.
            for (auto& osr : osr_corrections) {
                if (slip_stats.reset_satellites.count(osr.satellite) != 0) {
                    for (int frequency = 0;
                         frequency < osr.num_frequencies &&
                         frequency < OSR_MAX_FREQ;
                         ++frequency) {
                        // CPC was aggregated while preparing the OSR, before
                        // the later udbias-equivalent reset became known.
                        osr.CPC[frequency] -=
                            osr.phase_compensation_m[frequency];
                    }
                    std::fill(std::begin(osr.phase_compensation_m),
                              std::end(osr.phase_compensation_m), 0.0);
                }
            }
        }
    }

    if (pppEnvOverrides().clas_stec_constraint &&
        ppp_config_.estimate_ionosphere &&
        ppp_config_.use_clas_osr_filter) {
        int network_id = -1;
        if (readClasAtmosNetworkId(epoch_atmos, network_id)) {
            if (has_last_clas_atmos_network_id_ &&
                network_id != last_clas_atmos_network_id_) {
                resetClasIonosphereStateValues(filter_state_);
            }
            last_clas_atmos_network_id_ = network_id;
            has_last_clas_atmos_network_id_ = true;
        }
    }

    if (osr_corrections.size() < 4) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("insufficient_osr");
        }
        // Dynamics mode: prepareEpochState already ran predictFilterState,
        // which propagated pos += vel*dt and inflated the covariance for
        // this interval, so the processed-time bookkeeping MUST advance
        // before this early return. Otherwise the next epoch's dt spans
        // this interval AGAIN and the same coast is re-applied every epoch:
        // across a low-satellite stretch dt grows unboundedly (observed
        // 2.2 s -> 15+ s at 5 Hz approaching the tokyo_run2 bridge outage)
        // and the repeated vel*dt over-propagation drags the float
        // kilometers away (the 4.5 km post-bridge blunder). White-noise
        // mode keeps historical behavior (its per-epoch SPP re-anchor of
        // position and clock bounds the damage of a stale dt).
        if (ppp_config_.use_dynamics_model) {
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
        }
        solution = seed;
        return solution;
    }

    ppp_clas::ensureAmbiguityStates(
        filter_state_, osr_corrections, clas_ambiguity_initial_variance);
    if (clas_mrtklib_parity) {
        constexpr double kMrtklibBiasVarianceCycles2 = 100.0 * 100.0;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) continue;
            for (int frequency = 0; frequency < osr.num_frequencies && frequency < 2;
                 ++frequency) {
                const double wavelength = osr.wavelengths[frequency];
                if (!(wavelength > 0.0)) continue;
                const SatelliteId ambiguity_satellite(
                    osr.satellite.system,
                    static_cast<uint8_t>(std::min(
                        255, static_cast<int>(osr.satellite.prn) +
                                 (frequency == 0 ? 0 : 100))));
                const int ambiguity_index = ambiguityStateIndex(ambiguity_satellite);
                if (ambiguity_index < 0) continue;
                double& variance =
                    filter_state_.covariance(ambiguity_index, ambiguity_index);
                // A reset can precede predictFilterState(), which adds the
                // tiny bias random walk and makes the sentinel slightly
                // larger than exactly 10000.
                if (variance > 0.9 * kMrtklibBiasVarianceCycles2) {
                    variance = kMrtklibBiasVarianceCycles2 *
                               wavelength * wavelength;
                }
            }
        }
    }
    if (clas_mrtklib_parity &&
        (clas_mrtklib_floatcnt_reset_this_epoch ||
         epoch_preparation.initialized_this_epoch)) {
        // udbias_ppp() recreates every zeroed phase-bias state with
        // lock=-minlock. The accepted observation at the end of this epoch
        // increments it once, so the published lock is -4 for minlock=5.
        // This happens on every whole-filter initialization, not only at a
        // 30-second CSSR phase-bias boundary: udbias_ppp() keys the reset to
        // x[IB]==0 after udpos/udstate, independently of correction timing.
        constexpr int kMrtklibMinLock = 5;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) continue;
            ambiguity_states_[osr.satellite].lock_count = -kMrtklibMinLock;
            ambiguity_states_[osr.satellite].outage_count = 0;
            const SatelliteId l2_satellite(
                osr.satellite.system,
                static_cast<uint8_t>(std::min(
                    255, static_cast<int>(osr.satellite.prn) + 100)));
            ambiguity_states_[l2_satellite].lock_count = -kMrtklibMinLock;
            ambiguity_states_[l2_satellite].outage_count = 0;
        }
    }
    if (pppEnvOverrides().clas_nl_datum_reset &&
        ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        ppp_config_.use_clas_osr_filter &&
        !ppp_config_.use_ionosphere_free &&
        ppp_config_.estimate_ionosphere) {
        if (applyClasNlDatumReset(
                obs.time, osr_corrections, ambiguity_states_, pppDebugEnabled())) {
            clas_dd_accumulator_ = {};
        }
    }
    if (clas_mrtklib_parity &&
        (clas_mrtklib_floatcnt_reset_this_epoch ||
         epoch_preparation.initialized_this_epoch)) {
        // Fresh udbias_ppp() states must be seeded from raw L*lambda-P.
        // A continuity shift queued for the discarded filter would make a
        // zero state non-zero before seeding and falsely mark it initialized.
        for (auto& [_, repair] : clas_phase_bias_repair_) {
            repair.pending_state_shift_cycles = {0.0, 0.0, 0.0};
        }
    }
    ppp_clas::applyPendingPhaseBiasStateShifts(
        filter_state_, osr_corrections, clas_phase_bias_repair_, pppDebugEnabled());

    const auto epoch_update = ppp_clas::runEpochMeasurementUpdate(
        obs,
        epoch_context,
        filter_state_,
        ppp_config_,
        seed,
        ambiguity_states_,
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        },
        [&](const SatelliteId& satellite, SignalType signal) {
            resetAmbiguity(satellite, signal);
        },
        [&](const SatelliteId& satellite) {
            return ambiguityStateIndex(satellite);
        },
        pppDebugEnabled());
    if (clas_mrtklib_parity &&
        (clas_mrtklib_floatcnt_reset_this_epoch ||
         epoch_preparation.initialized_this_epoch)) {
        // MRTKLIB's QZSS CLAS row is L1-only (the L2 phase-bias slot is
        // invalid). compensatedisp() may contribute the carried pair once on
        // the reset epoch, then pbreset clears that datum; subsequent epochs
        // rebase and therefore contribute zero. Native retains an L2 raw
        // observation for slip detection, so explicitly retire only the
        // QZSS dispersion datum after that first reset-epoch use. Merely
        // clearing has_base is insufficient: the raw L1/L2 pair would rebase
        // on the next epoch and start accumulating again, while MRTKLIB's
        // invalid QZSS L2 phase-bias slot keeps pbreset asserted and compL=0.
        for (const auto& osr : osr_corrections) {
            if (osr.satellite.system != GNSSSystem::QZSS) continue;
            auto& compensation =
                clas_dispersion_compensation_[osr.satellite];
            compensation.has_base = {false, false};
            compensation.slip = {false, false};
            compensation.mrtklib_qzss_suppressed = true;
        }
    }
    if (!epoch_update.updated) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("measurement_update");
        }
        if (clas_mrtklib_parity &&
            (epoch_update.insufficient_valid_satellites ||
             epoch_preparation.initialized_this_epoch)) {
            // mrtk_ppp_rtk.c resets every kinematic state when stat remains
            // SOLQ_NONE after the ssat.vsat L1 count check. float_count is
            // neither incremented nor cleared on this SINGLE epoch.
            filter_state_ = PPPState{};
            filter_initialized_ = false;
            ambiguity_states_.clear();
            est_stec_outage_.clear();
            clas_dd_accumulator_ = {};
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            last_clas_constrained_fixed_state_valid_ = false;
            solution = seed;
            applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            ++total_epochs_processed_;
            return solution;
        }
        // MRTKLIB keeps publishing the predicted FLOAT state when all
        // post-fit DD reference trials are rejected; the epoch is not
        // reclassified as SINGLE merely because filter2_ supplied no new
        // posterior. Keeping the float status also advances float_count and
        // preserves the literal 15-epoch reset cadence.
        if (clas_mrtklib_parity && filter_initialized_) {
            solution = ppp_clas::finalizeEpochSolution(
                filter_state_, obs.time, false, 0.0, 0,
                static_cast<int>(epoch_context.osr_corrections.size()));
            ++clas_mrtklib_float_count_;
            had_fixed_last_epoch_ = false;
            applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            ++total_epochs_processed_;
            if (solution.isValid()) {
                last_published_solution_position_ecef_ = solution.position_ecef;
                has_last_published_solution_position_ = true;
            }
            return solution;
        }
        // Same dt-bookkeeping requirement as the insufficient_osr early
        // return above: the predict for this interval already happened.
        if (ppp_config_.use_dynamics_model) {
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
        }
        solution = seed;
        return solution;
    }
    dumpClasFloatPosition(obs.time, filter_state_, epoch_update, osr_corrections.size());
    const auto& update_stats = epoch_update.update_stats;
    if (clas_mrtklib_parity) {
        clas_mrtklib_ar_rejected_ambiguities_ =
            update_stats.rejected_phase_ambiguities;
    }
    pre_anchor_covariance_ = update_stats.pre_anchor_covariance;

    if (ppp_config_.use_clas_dd_filter) {
        if (!clas_dd_filter_) {
            clas_dd_filter_ = std::make_unique<ppp_clas_dd::DdFilterScaffold>();
        }
        const PositionSolution native_float_solution = ppp_clas::finalizeEpochSolution(
            filter_state_,
            obs.time,
            false,
            0.0,
            0,
            static_cast<int>(osr_corrections.size()));
        solution = clas_dd_filter_->processFloatUpdate(
            obs,
            epoch_context,
            filter_state_,
            native_float_solution,
            ppp_config_,
            [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
                return calculateMappingFunction(receiver_pos, elevation, time);
            });
        applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
        has_last_processed_time_ = true;
        last_processed_time_ = obs.time;
        ++total_epochs_processed_;
        return solution;
    }

    // Accumulate Melbourne-Wübbena for WL-NL AR in CLAS per-frequency mode.
    if (ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid || osr.num_frequencies < 2) continue;
            const Observation* l1_raw = findOsrFrequencyObservation(obs, osr, 0);
            const Observation* l2_raw = findOsrFrequencyObservation(obs, osr, 1);
            if (!l1_raw || !l2_raw || !l1_raw->valid || !l2_raw->valid) continue;
            if (!l1_raw->has_carrier_phase || !l2_raw->has_carrier_phase) continue;
            if (!l1_raw->has_pseudorange || !l2_raw->has_pseudorange) continue;
            if (ppp_config_.kinematic_mode &&
                ppp_config_.use_clas_osr_filter &&
                ((l1_raw->snr > 0.0 &&
                  ppp_ar::clasKinematicSnrMasked(0, osr.elevation, l1_raw->snr)) ||
                 (l2_raw->snr > 0.0 &&
                  ppp_ar::clasKinematicSnrMasked(1, osr.elevation, l2_raw->snr)))) {
                continue;
            }
            const double f1 = osr.frequencies[0];
            const double f2 = osr.frequencies[1];
            if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1e6) continue;
            const double l1_m = l1_raw->carrier_phase * osr.wavelengths[0]
                              - osr.phase_bias_m[0];
            const double l2_m = l2_raw->carrier_phase * osr.wavelengths[1]
                              - osr.phase_bias_m[1];
            const double p1 = l1_raw->pseudorange - osr.code_bias_m[0];
            const double p2 = l2_raw->pseudorange - osr.code_bias_m[1];
            const double mw_m = (f1 * l1_m - f2 * l2_m) / (f1 - f2)
                              - (f1 * p1 + f2 * p2) / (f1 + f2);
            constexpr double lambda_wl_gps = constants::SPEED_OF_LIGHT / (1575.42e6 - 1227.60e6);
            const double mw_cycles = mw_m / lambda_wl_gps;
            auto& amb = ambiguity_states_[osr.satellite];
            const bool mw_slip = amb.needs_reinitialization;
            if (!mw_slip && amb.mw_count > 0) {
                amb.mw_sum_cycles += mw_cycles;
                amb.mw_count += 1;
                amb.mw_mean_cycles = amb.mw_sum_cycles / amb.mw_count;
            } else {
                amb.mw_sum_cycles = mw_cycles;
                amb.mw_count = 1;
                amb.mw_mean_cycles = mw_cycles;
                amb.wl_is_fixed = false;
            }
        }
    }

    const auto trop_mapping_for_validation =
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        };
    const auto ambiguity_index_for_validation = [&](const SatelliteId& satellite) {
        return ambiguityStateIndex(satellite);
    };

    const Vector3d clas_float_position_ecef =
        filter_state_.state.segment(filter_state_.pos_index, 3);
    const double clas_float_horizontal_sigma_m =
        clasKinematicHorizontalPositionSigmaM(filter_state_);
    const PPPState clas_float_filter_state = filter_state_;
    const auto clas_float_ambiguity_states = ambiguity_states_;

    const auto ambiguity_resolution =
        ppp_clas::resolveAndValidateAmbiguities(
            filter_state_,
            ambiguity_states_,
            [&]() {
                return ppp_config_.enable_ambiguity_resolution &&
                       resolveAmbiguities(obs, nav);
            },
            (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL)
                ? ppp_clas::ValidateFixedSolutionFunction{}
                : ppp_clas::ValidateFixedSolutionFunction{[&]() {
                      return ppp_clas::validateFixedSolution(
                          obs, osr_corrections, filter_state_, ppp_config_,
                          trop_mapping_for_validation,
                          ambiguity_index_for_validation, pppDebugEnabled());
                  }},
            pppDebugEnabled());

    // MRTKLIB mrtk_ppp_rtk.c:2296-2330 parity: validate the fixed solution with
    // the post-fix DD phase chi-square. Publish FIX only when chisq < thres_fix
    // (5.0) and hold (constrain the float filter toward the fixed DD
    // ambiguities, holdamb()) only when chisq < thres_hold (0.5). AR-failed
    // epochs publish FLOAT and reset the nfix counter; there is no hold-driven
    // FIX publication.
    const bool kinematic_clas_wlnl_hold_path =
        ppp_config_.kinematic_mode &&
        ppp_config_.use_clas_osr_filter &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL;
    bool clas_kinematic_chisq_rejected = false;
    if (kinematic_clas_wlnl_hold_path &&
        ppp_config_.enable_ambiguity_resolution) {
        if (!ppp_ar::wlnlHoldStillValid(clas_wlnl_hold_, ambiguity_states_)) {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }

        if (ambiguity_resolution.accepted &&
            !last_clas_constrained_fixed_state_valid_) {
            // MRTKLIB publishes FIX only from the constrained xa solution
            // (sol.rr = xa). Without a validated state-DD LAMBDA fix the epoch
            // stays FLOAT instead of labelling the float state as fixed.
            clas_kinematic_chisq_rejected = true;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-CHISQ] no constrained state, demote"
                          << " ratio=" << last_ar_ratio_ << "\n";
            }
        } else if (ambiguity_resolution.accepted) {
            const PPPState& fixed_state_for_validation =
                last_clas_constrained_fixed_state_;
            ppp_clas::FixValidationOptions fix_validation_options;
            fix_validation_options.outlier_sigma_gate =
                kMrtklibPhaseResidualSigmaGate;
            fix_validation_options.mrtklib_chisq_fallback = true;
            // MRTKLIB parity (dynamics path only): the post-fix residual
            // gate/chi-square normalize by the innovation covariance
            // H'*P*H + R formed from the FLOAT posterior covariance
            // (mrtk_ppp_rtk.c:2296-2313 -> filter2_ -> residual_test), not
            // by the measurement variance alone. Under the tight Stage-2
            // varerr an R-only basis rejects every centimeter-level DD
            // residual and drives fix% to zero.
            if (ppp_config_.clas_mrtklib_float_parity &&
                ppp_config_.use_dynamics_model) {
                fix_validation_options.innovation_covariance =
                    &clas_float_filter_state.covariance;
            }
            const auto fix_validation = ppp_clas::validateFixedSolution(
                obs,
                osr_corrections,
                fixed_state_for_validation,
                ppp_config_,
                trop_mapping_for_validation,
                ambiguity_index_for_validation,
                pppDebugEnabled(),
                fix_validation_options);
            const double phase_chisq = fix_validation.phase_chisq;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-CHISQ] phase_chisq=" << phase_chisq
                          << " rows=" << fix_validation.phase_rows
                          << " outliers=" << fix_validation.phase_outlier_rows
                          << " ratio=" << last_ar_ratio_ << "\n";
            }
            if (!std::isfinite(phase_chisq) ||
                phase_chisq >= kMrtklibFixChiSquareGate) {
                clas_kinematic_chisq_rejected = true;
            } else if (phase_chisq < kMrtklibHoldChiSquareGate) {
                std::map<SatelliteId, double> clas_satellite_elevations_rad;
                for (const auto& osr : osr_corrections) {
                    if (osr.valid) {
                        clas_satellite_elevations_rad[osr.satellite] =
                            osr.elevation;
                    }
                }
                ++clas_wlnl_hold_.consecutive_fix_count;
                std::vector<ppp_ar::WlnlHoldConstraint> hold_constraints;
                if (clas_wlnl_hold_.consecutive_fix_count >=
                        ppp_ar::kMrtklibMinFixCount &&
                    ppp_ar::buildWlnlHoldConstraints(
                        last_clas_constrained_fixed_state_,
                        ambiguity_states_,
                        clas_satellite_elevations_rad,
                        hold_constraints,
                        ppp_config_.use_dynamics_model &&
                            !ppp_config_.low_dynamics_mode)) {
                    clas_wlnl_hold_.constraints = std::move(hold_constraints);
                    clas_wlnl_hold_.active = true;
                    const bool hold_applied = ppp_ar::applyWlnlHoldAmbiguity(
                        filter_state_, clas_wlnl_hold_.constraints);
                    if (pppDebugEnabled()) {
                        std::cerr << "[CLAS-WLNL-HOLD] applied=" << hold_applied
                                  << " constraints="
                                  << clas_wlnl_hold_.constraints.size()
                                  << " nfix="
                                  << clas_wlnl_hold_.consecutive_fix_count
                                  << " chisq=" << phase_chisq << "\n";
                    }
                } else if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-WLNL-HOLD] skipped nfix="
                              << clas_wlnl_hold_.consecutive_fix_count
                              << " chisq=" << phase_chisq << "\n";
                }
            }
        } else {
            // MRTKLIB resets rtk->nfix on every non-FIX epoch
            // (mrtk_ppp_rtk.c:2371).
            clas_wlnl_hold_.consecutive_fix_count = 0;
        }
    }

    bool ambiguity_fixed_epoch =
        ambiguity_resolution.accepted && !clas_kinematic_chisq_rejected;
    if (clas_kinematic_chisq_rejected) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        ambiguity_states_ = clas_float_ambiguity_states;
        ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-KIN-CHISQ] reject fix (chisq >= "
                      << kMrtklibFixChiSquareGate << ")\n";
        }
    }
    if (ambiguity_resolution.rejected_after_fix) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        if (ppp_config_.kinematic_mode && ppp_config_.use_clas_osr_filter) {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }
    }

    if (pppDebugEnabled()) {
        ppp_clas::logUpdateSummary(update_stats, osr_corrections.size());
    }

    bool wlnl_fixed_position_ok = false;
    Vector3d wlnl_fixed_position = Vector3d::Zero();
    if (ambiguity_fixed_epoch &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        !kinematic_clas_wlnl_hold_path) {
        wlnl_fixed_position_ok = solveFixedPosition(obs, nav, wlnl_fixed_position);
        if (pppDebugEnabled() && wlnl_fixed_position_ok) {
            const double shift = (wlnl_fixed_position -
                filter_state_.state.segment(filter_state_.pos_index, 3)).norm();
            std::cerr << "[CLAS-WLNL-FIX] pos_shift=" << shift << "m\n";
        }
    }

    // MRTKLIB publishes the fixed solution from xa/Pa while the float filter
    // x/P is only nudged by holdamb() (mrtk_ppp_rtk.c:2355-2361).
    const bool use_constrained_fixed_state =
        ambiguity_fixed_epoch &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        last_clas_constrained_fixed_state_valid_ &&
        (env_overrides_.clas_resamb ||
         (ppp_config_.kinematic_mode && ppp_config_.use_clas_osr_filter));
    const PPPState& solution_filter_state =
        use_constrained_fixed_state ? last_clas_constrained_fixed_state_ : filter_state_;

    solution = ppp_clas::finalizeEpochSolution(
        solution_filter_state,
        obs.time,
        ambiguity_fixed_epoch,
        last_ar_ratio_,
        last_fixed_ambiguities_,
        static_cast<int>(osr_corrections.size()));

    if (wlnl_fixed_position_ok &&
        !ppp_config_.kinematic_mode &&
        !env_overrides_.clas_fixed_state_output &&
        !use_constrained_fixed_state) {
        solution.position_ecef = wlnl_fixed_position;
    }

    // Multi-epoch SD AR: accumulate DD float ambiguities over epochs,
    // then fix with LAMBDA when variance is small enough.
    {
        const auto sd_ar_result = ppp_clas_sd::solveMultiEpochSdAr(
            clas_dd_accumulator_,
            obs,
            osr_corrections,
            solution.position_ecef,
            3.0,   // AR ratio threshold
            20,    // Min accumulation epochs before attempting LAMBDA
            pppDebugEnabled());
        const bool sd_ar_fixed =
            sd_ar_result.valid && sd_ar_result.ar_ratio >= 3.0;
        const bool apply_sd_mar_shift_gate =
            ppp_config_.kinematic_mode ||
            env_overrides_.clas_base_clock_parity;
        const bool sd_ar_position_ok =
            !apply_sd_mar_shift_gate ||
            sd_ar_result.position_shift_m <=
                kClasBaseClockParitySdMarMaxPositionShiftM;
        if (sd_ar_fixed && sd_ar_position_ok && !ppp_config_.kinematic_mode) {
            solution.position_ecef = sd_ar_result.position;
            solution.status = SolutionStatus::PPP_FIXED;
        } else if (sd_ar_fixed && pppDebugEnabled()) {
            std::cerr << "[CLAS-SD-MAR] reject"
                      << (ppp_config_.kinematic_mode ? " kinematic" : " parity")
                      << " pos_shift="
                      << sd_ar_result.position_shift_m
                      << " ratio=" << sd_ar_result.ar_ratio << "\n";
        }
    }

    // On the kinematic CLAS WLNL path the MRTKLIB-parity post-fix chi-square
    // gate (plus the maxdiffp guard below) already validates every fixed
    // publication, so the custom float-jump/continuity gates are skipped: the
    // float filter itself carries meter-level error, and a correct fix
    // legitimately jumps away from the previously published float position.
    bool clas_kinematic_fix_rejected = false;
    if (ppp_config_.kinematic_mode &&
        solution.status == SolutionStatus::PPP_FIXED &&
        !(kinematic_clas_wlnl_hold_path && ambiguity_fixed_epoch)) {
        const double dt_seconds =
            has_last_processed_time_ ? obs.time - last_processed_time_ : 0.2;
        const double max_fixed_float_jump_m = clasKinematicMaxFixedFloatJumpM(
            dt_seconds,
            clas_float_horizontal_sigma_m);
        const Vector3d post_ar_filter_position =
            solution_filter_state.state.segment(solution_filter_state.pos_index, 3);
        double fixed_float_jump_m =
            (solution.position_ecef - clas_float_position_ecef).norm();
        if (wlnl_fixed_position_ok) {
            fixed_float_jump_m = std::max(
                fixed_float_jump_m,
                (wlnl_fixed_position - post_ar_filter_position).norm());
        }
        const bool ratio_ok =
            solution.ratio <= 0.0 ||
            solution.ratio >= (kinematic_clas_wlnl_hold_path
                 ? ppp_config_.ar_ratio_threshold
                 : std::max(
                       kClasKinematicMinFixRatio,
                       ppp_config_.ar_ratio_threshold + 0.5));
        const bool wlnl_shift_ok =
            !wlnl_fixed_position_ok ||
            (wlnl_fixed_position - post_ar_filter_position).norm() <=
                kClasKinematicWlnlMaxPositionShiftM;
        double continuity_jump_m = 0.0;
        if (has_last_published_solution_position_) {
            continuity_jump_m = (solution.position_ecef -
                last_published_solution_position_ecef_).norm();
        }
        const bool continuity_ok =
            !has_last_published_solution_position_ ||
            continuity_jump_m <= max_fixed_float_jump_m;
        const bool float_jump_ok =
            std::isfinite(fixed_float_jump_m) &&
            fixed_float_jump_m <= max_fixed_float_jump_m;
        const bool jump_ok =
            wlnl_shift_ok &&
            continuity_ok &&
            float_jump_ok &&
            ratio_ok;
        if (jump_ok) {
            ++clas_kinematic_fix_candidate_streak_;
        } else {
            clas_kinematic_fix_candidate_streak_ = 0;
        }
        const bool minfix_ok =
            clas_kinematic_fix_candidate_streak_ >= kClasKinematicMinFixCount;
        if (!jump_ok || !minfix_ok) {
            clas_kinematic_fix_rejected = true;
            solution.position_ecef = clas_float_position_ecef;
            solution.status = SolutionStatus::PPP_FLOAT;
            solution.ratio = 0.0;
            solution.num_fixed_ambiguities = 0;
            filter_state_ = clas_float_filter_state;
            ambiguity_states_ = clas_float_ambiguity_states;
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            if (!jump_ok) {
                clas_kinematic_fix_candidate_streak_ = 0;
            }
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-FIX] reject"
                          << (!wlnl_shift_ok ? " wlnl_shift" :
                              !continuity_ok ? " continuity" :
                              !float_jump_ok ? " float_jump" :
                              !ratio_ok ? " ratio" :
                              " minfix")
                          << " jump=" << fixed_float_jump_m
                          << " continuity=" << continuity_jump_m
                          << " limit=" << max_fixed_float_jump_m
                          << " ratio=" << solution.ratio
                          << " streak=" << clas_kinematic_fix_candidate_streak_
                          << "\n";
            }
        }
    } else if (ppp_config_.kinematic_mode) {
        clas_kinematic_fix_candidate_streak_ = 0;
    }

    // MRTKLIB maxdiffp guard (mrtk_ppp_rtk.c:2333-2352). MRTKLIB applies it
    // with dynamics on (benchmark clas.toml: dynamics=true + pseudorange_diff);
    // without it the dynamics filter can enter a rejection spiral and coast
    // kilometers away on the velocity states.
    if (ppp_config_.kinematic_mode &&
        ppp_config_.use_clas_osr_filter &&
        seed.isValid()) {
        const Vector3d float_position =
            filter_state_.state.segment(filter_state_.pos_index, 3);
        const double spp_divergence_m =
            (float_position - seed.position_ecef).norm();
        if (spp_divergence_m > kMrtklibMaxSppDivergenceM) {
            ++clas_kinematic_spp_divergence_count_;
            if (clas_kinematic_spp_divergence_count_ >
                    kMrtklibMaxSppDivergenceEpochs) {
                if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-KIN-MAXDIFFP] reset to SPP dist="
                              << spp_divergence_m << "\n";
                }
                filter_state_.state.segment(filter_state_.pos_index, 3) =
                    seed.position_ecef;
                for (int i = 0; i < 3; ++i) {
                    const int idx = filter_state_.pos_index + i;
                    filter_state_.covariance.row(idx).setZero();
                    filter_state_.covariance.col(idx).setZero();
                    const double spp_variance =
                        seed.position_covariance(i, i) > 0.0
                            ? seed.position_covariance(i, i)
                            : 100.0;
                    filter_state_.covariance(idx, idx) = spp_variance;
                }
                if (ppp_config_.use_dynamics_model &&
                    filter_state_.vel_index >= 0 &&
                    filter_state_.vel_index + 2 < filter_state_.covariance.rows()) {
                    filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
                    for (int i = 0; i < 3; ++i) {
                        const int idx = filter_state_.vel_index + i;
                        filter_state_.covariance.row(idx).setZero();
                        filter_state_.covariance.col(idx).setZero();
                        filter_state_.covariance(idx, idx) =
                            clas_mrtklib_parity
                                ? 1.0
                                : ppp_config_.initial_velocity_variance;
                    }
                }
                if (clas_mrtklib_parity &&
                    filter_state_.accel_index >= 0 &&
                    filter_state_.accel_index + 2 <
                        filter_state_.covariance.rows()) {
                    filter_state_.state.segment(filter_state_.accel_index, 3)
                        .setConstant(1e-6);
                    for (int i = 0; i < 3; ++i) {
                        const int idx = filter_state_.accel_index + i;
                        filter_state_.covariance.row(idx).setZero();
                        filter_state_.covariance.col(idx).setZero();
                        filter_state_.covariance(idx, idx) = 1.0;
                    }
                }
                solution.position_ecef = seed.position_ecef;
                solution.status = SolutionStatus::SPP;
                solution.ratio = 0.0;
                solution.num_fixed_ambiguities = 0;
                clas_kinematic_fix_rejected = false;
                clas_kinematic_fix_candidate_streak_ = 0;
                clas_kinematic_spp_divergence_count_ = 0;
                ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            }
        } else {
            clas_kinematic_spp_divergence_count_ = 0;
        }
    }

    had_fixed_last_epoch_ =
        solution.status == SolutionStatus::PPP_FIXED && !clas_kinematic_fix_rejected;

    if (clas_mrtklib_parity) {
        if (solution.status == SolutionStatus::PPP_FIXED) {
            clas_mrtklib_float_count_ = 0;
        } else if (solution.status == SolutionStatus::PPP_FLOAT) {
            ++clas_mrtklib_float_count_;
        }
    }

    has_last_processed_time_ = true;
    last_processed_time_ = obs.time;
    ++total_epochs_processed_;

    applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
    if (ppp_config_.kinematic_mode && solution.isValid()) {
        last_published_solution_position_ecef_ = solution.position_ecef;
        has_last_published_solution_position_ = true;
    }
    return solution;
}

}  // namespace libgnss
