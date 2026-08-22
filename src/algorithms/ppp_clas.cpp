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

#include "ppp_clas_internal.hpp"

namespace libgnss::ppp_clas {
using namespace internal;

AppliedOsrCorrections selectAppliedOsrCorrections(
    const OSRCorrection& osr,
    int freq_index,
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    AppliedOsrCorrections corrections;
    if (freq_index < 0 ||
        freq_index >= std::max(osr.num_frequencies,
                               osr.num_output_frequencies)) {
        return corrections;
    }

    const double relativity = osr.relativity_correction_m;
    const double receiver_antenna = osr.receiver_antenna_m[freq_index];
    const double code_bias = osr.code_bias_m[freq_index];
    const double phase_bias = osr.phase_bias_m[freq_index];
    const double windup = osr.windup_m[freq_index];
    const double phase_compensation = osr.phase_compensation_m[freq_index];

    switch (policy) {
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR:
        corrections.pseudorange_correction_m =
            pppEnvOverrides().clas_code_row_full_prc ?
                osr.PRC[freq_index] :
                osr.PRC[freq_index] - osr.trop_correction_m;
        corrections.carrier_phase_correction_m =
            pppEnvOverrides().clas_amb_datum
                ? osr.CPC[freq_index]
                : osr.CPC[freq_index] - osr.trop_correction_m;
        break;
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS:
        corrections.pseudorange_correction_m =
            relativity + receiver_antenna + code_bias;
        corrections.carrier_phase_correction_m =
            relativity + receiver_antenna + phase_bias + windup + phase_compensation;
        break;
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY:
        corrections.pseudorange_correction_m =
            relativity + receiver_antenna;
        corrections.carrier_phase_correction_m =
            relativity + receiver_antenna + windup;
        break;
    }

    return corrections;
}

bool usesClasTropospherePrior(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    return policy ==
           ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

bool suppressesClasAmbDatumPhaseTrop(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    const auto& env = pppEnvOverrides();
    return env.clas_amb_datum &&
           !env.clas_amb_datum_residual_phase_trop &&
           policy ==
               ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

bool usesResidualClasAmbDatumPhaseTrop(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    const auto& env = pppEnvOverrides();
    return env.clas_amb_datum &&
           env.clas_amb_datum_residual_phase_trop &&
           policy ==
               ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

double effectiveClasTropPriorVariance(const ppp_shared::PPPConfig& config) {
    const double value = pppEnvOverrides().clas_trop_prior_variance;
    return value > 0.0 ? value : config.clas_trop_prior_variance;
}

double effectiveClasTropInitialVariance(const ppp_shared::PPPConfig& config) {
    const double value = pppEnvOverrides().clas_trop_initial_variance;
    return value > 0.0 ? value : config.clas_trop_initial_variance;
}

double effectiveClasTropProcessNoise(const ppp_shared::PPPConfig& config) {
    const double value = pppEnvOverrides().clas_trop_process_noise;
    return value > 0.0 ? value : config.clas_trop_process_noise;
}

std::vector<SatelliteId> collectResidualIonoSatellites(
    const ObservationData& obs,
    const SSRProducts& ssr_products) {
    auto supports_clas_residual_iono = [](const SatelliteId& satellite) {
        return satellite.system == GNSSSystem::GPS ||
               satellite.system == GNSSSystem::Galileo ||
               satellite.system == GNSSSystem::QZSS;
    };

    std::set<SatelliteId> unique_satellites;
    for (const auto& satellite : obs.getSatellites()) {
        if (supports_clas_residual_iono(satellite)) {
            unique_satellites.insert(satellite);
        }
    }
    for (const auto& [satellite, _] : ssr_products.orbit_clock_corrections) {
        if (supports_clas_residual_iono(satellite)) {
            unique_satellites.insert(satellite);
        }
    }
    return std::vector<SatelliteId>(unique_satellites.begin(), unique_satellites.end());
}

EpochPreparationResult prepareEpochState(
    const ObservationData& obs,
    const PositionSolution& seed_solution,
    const SSRProducts& ssr_products,
    ppp_shared::PPPState& filter_state,
    bool& filter_initialized,
    GNSSTime& convergence_start_time,
    Vector3d& static_anchor_position,
    bool& has_static_anchor_position,
    const ppp_shared::PPPConfig& config,
    double modeled_zenith_troposphere_delay_m,
    bool has_last_processed_time,
    const GNSSTime& last_processed_time,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance) {
    EpochPreparationResult result;
    bool initialized_this_epoch = false;
    if (!filter_initialized) {
        if (!seed_solution.isValid()) {
            return result;
        }
        const auto iono_satellites =
            config.estimate_ionosphere
                ? collectResidualIonoSatellites(obs, ssr_products)
                : std::vector<SatelliteId>{};
        bool qzss_visible = false;
        for (const auto& satellite : obs.getSatellites()) {
            if (satellite.system == GNSSSystem::QZSS) {
                qzss_visible = true;
                break;
            }
        }
        initializeFilterState(
            filter_state,
            seed_solution,
            obs.time,
            iono_satellites,
            config,
            modeled_zenith_troposphere_delay_m,
            qzss_visible);
        filter_initialized = true;
        initialized_this_epoch = true;
        convergence_start_time = obs.time;
        if (!config.kinematic_mode || config.low_dynamics_mode) {
            static_anchor_position = seed_solution.position_ecef;
            has_static_anchor_position = true;
        }
    }

    const double dt =
        has_last_processed_time ? std::max(obs.time - last_processed_time, 0.001) : 1.0;
    syncSlipState(
        obs,
        filter_state,
        ambiguity_states,
        dispersion_compensation,
        phase_bias_repair,
        ambiguity_reset_variance,
        !clasMrtklibFloatParity(config));
    // MRTKLIB udpos_ppp() returns immediately after initializing x from the
    // current SPP solution. Applying F(dt) in that same epoch advances the
    // freshly seeded position by one extra velocity interval (~1 m at urban
    // driving speed) after every floatcnt mass reset.
    if (!(initialized_this_epoch && clasMrtklibFloatParity(config))) {
        const auto observed_satellite_list = obs.getSatellites();
        const std::set<SatelliteId> observed_satellites(
            observed_satellite_list.begin(), observed_satellite_list.end());
        predictFilterState(
            filter_state,
            config,
            dt,
            seed_solution.position_ecef,
            seed_solution.receiver_clock_bias,
            seed_solution.isValid(),
            &observed_satellites);
    }
    if (!clasMrtklibFloatParity(config)) {
        markSlipCompensationFromAmbiguities(
            obs, ambiguity_states, dispersion_compensation);
    }
    result.ready = true;
    result.initialized_this_epoch = initialized_this_epoch;
    return result;
}

void initializeFilterState(
    ppp_shared::PPPState& filter_state,
    const PositionSolution& seed_solution,
    const GNSSTime& /*time*/,
    const std::vector<SatelliteId>& iono_satellites,
    const ppp_shared::PPPConfig& config,
    double modeled_zenith_troposphere_delay_m,
    bool qzss_visible) {
    const bool mrtklib_pva = clasMrtklibFloatParity(config);
    filter_state.accel_index = mrtklib_pva ? 6 : -1;
    filter_state.clock_index = mrtklib_pva ? 9 : 6;
    filter_state.glo_clock_index = mrtklib_pva ? 10 : 7;
    filter_state.trop_index = mrtklib_pva ? 11 : 8;
    const int fixed_state_count = mrtklib_pva ? 12 : 9;
    filter_state.gal_clock_index = -1;
    filter_state.qzs_clock_index = -1;
    filter_state.bds_clock_index = -1;
    int isb_start = fixed_state_count;
    if (pppEnvOverrides().clas_qzss_s_prn_fix && config.use_clas_osr_filter &&
        qzss_visible) {
        filter_state.qzs_clock_index = isb_start++;
    }
    const int n_isb = isb_start - fixed_state_count;
    const int n_iono = static_cast<int>(iono_satellites.size());
    const int base = fixed_state_count + n_isb + n_iono;
    filter_state.ionosphere_indices.clear();
    filter_state.adaptive_ionosphere_process_noise.clear();
    filter_state.state = VectorXd::Zero(base);
    filter_state.covariance = MatrixXd::Identity(base, base);
    filter_state.state.segment(0, 3) = seed_solution.position_ecef;
    if (mrtklib_pva) {
        filter_state.state.segment(filter_state.vel_index, 3) =
            seed_solution.velocity_ecef;
        filter_state.state.segment(filter_state.accel_index, 3).setConstant(1e-6);
    }
    filter_state.state(filter_state.clock_index) = seed_solution.receiver_clock_bias;
    filter_state.state(filter_state.glo_clock_index) = seed_solution.receiver_clock_bias;
    if (filter_state.qzs_clock_index >= 0) {
        filter_state.state(filter_state.qzs_clock_index) =
            seed_solution.receiver_clock_bias;
    }
    // benchmark/clas.toml uses troposphere="off". Keep the architectural
    // placeholder zero so MRTKLIB filter2 state compaction excludes it.
    filter_state.state(filter_state.trop_index) = mrtklib_pva
        ? 0.0
        : modeled_zenith_troposphere_delay_m;
    // MRTKLIB udpos_ppp()/udtrop() literal initial covariance.  The legacy
    // native CLAS path uses its independently tuned 10 m position sigma and
    // configurable wet-trop prior; the parity path must use VAR_POS=30^2 and
    // clas.toml initial_std.troposphere=0.005 m.
    constexpr double kMrtklibInitialPositionVariance = 30.0 * 30.0;
    constexpr double kMrtklibInitialTropVariance = 0.005 * 0.005;
    filter_state.covariance.block(0, 0, 3, 3) *=
        mrtklib_pva ? kMrtklibInitialPositionVariance
                    : config.clas_initial_position_variance;
    if (config.kinematic_mode && config.use_dynamics_model) {
        filter_state.covariance.block(
            filter_state.vel_index, filter_state.vel_index, 3, 3) *=
            mrtklib_pva ? 1.0 : config.initial_velocity_variance;
        if (mrtklib_pva) {
            filter_state.covariance.block(
                filter_state.accel_index, filter_state.accel_index, 3, 3) *= 1.0;
        }
    }
    filter_state.covariance(filter_state.clock_index, filter_state.clock_index) =
        config.clas_clock_variance;
    filter_state.covariance(filter_state.glo_clock_index,
                            filter_state.glo_clock_index) = config.clas_clock_variance;
    filter_state.covariance(filter_state.trop_index, filter_state.trop_index) =
        mrtklib_pva ? kMrtklibInitialTropVariance
                    : effectiveClasTropInitialVariance(config);
    if (filter_state.qzs_clock_index >= 0) {
        filter_state.covariance(filter_state.qzs_clock_index,
                                filter_state.qzs_clock_index) =
            config.clas_clock_variance;
    }
    filter_state.iono_index = isb_start;
    // MRTKLIB literal-port track: clas.toml [kalman_filter.initial_std]
    // ionosphere = 0.01 m (the state is residual iono after the CLAS grid
    // STEC correction; mrtk_ppp_rtk.c:498 initx(1e-6, SQR(std[1]))).
    const double iono_initial_variance = clasMrtklibFloatParity(config)
        ? 0.01 * 0.01
        : std::min(config.initial_ionosphere_variance, 1.0);
    for (size_t index = 0; index < iono_satellites.size(); ++index) {
        const int state_index = filter_state.iono_index + static_cast<int>(index);
        filter_state.ionosphere_indices[iono_satellites[index]] = state_index;
        filter_state.adaptive_ionosphere_process_noise[iono_satellites[index]] = 0.0;
        filter_state.state(state_index) = mrtklib_pva ? 1e-6 : 0.0;
        filter_state.covariance(state_index, state_index) = iono_initial_variance;
    }
    filter_state.amb_index = base;
    filter_state.total_states = base;
}


void predictFilterState(
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double dt,
    const Vector3d& seed_position_ecef,
    double seed_receiver_clock_bias_m,
    bool seed_valid,
    const std::set<SatelliteId>* observed_satellites) {
    const int nx = filter_state.total_states;
    const bool kinematic_white_noise =
        config.kinematic_mode && !config.low_dynamics_mode;
    const bool use_dynamic_prediction =
        kinematic_white_noise && config.use_dynamics_model;

    MatrixXd F = MatrixXd::Identity(nx, nx);
    MatrixXd covariance_F = MatrixXd::Identity(nx, nx);
    if (use_dynamic_prediction) {
        F.block(filter_state.pos_index, filter_state.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * dt;
        covariance_F.block(
            filter_state.pos_index, filter_state.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * dt;
        if (clasMrtklibFloatParity(config) && filter_state.accel_index >= 0) {
            F.block(filter_state.vel_index, filter_state.accel_index, 3, 3) =
                MatrixXd::Identity(3, 3) * dt;
            F.block(filter_state.pos_index, filter_state.accel_index, 3, 3) =
                MatrixXd::Identity(3, 3) * (dt * dt / 2.0);
            covariance_F.block(
                filter_state.vel_index, filter_state.accel_index, 3, 3) =
                MatrixXd::Identity(3, 3) * dt;
        }
        filter_state.state = F * filter_state.state;
    } else if (kinematic_white_noise) {
        if (config.reset_kinematic_position_to_spp_each_epoch && seed_valid) {
            reinitializePositionState(
                filter_state,
                seed_position_ecef,
                config.initial_position_variance);
        }
    } else if (config.process_noise_position > 0.0) {
        const double position_q = config.process_noise_position * dt;
        for (int axis = 0; axis < 3; ++axis) {
            filter_state.covariance(filter_state.pos_index + axis,
                                    filter_state.pos_index + axis) += position_q;
        }
    }

    MatrixXd Q = MatrixXd::Zero(nx, nx);
    if (use_dynamic_prediction && clasMrtklibFloatParity(config) &&
        filter_state.accel_index >= 0) {
        // MRTKLIB v0.5.1 udpos_ppp(): acceleration random walk is defined
        // in local ENU (horizontal prn[3]=0.2, vertical prn[4]=0.1) and
        // rotated to ECEF before being added to P[6:9,6:9].
        double lat = 0.0, lon = 0.0, height = 0.0;
        ecef2geodetic(filter_state.state.segment(filter_state.pos_index, 3),
                      lat, lon, height);
        (void)height;
        Matrix3d enu_to_ecef;
        const Vector3d east = enu2ecef(Vector3d::UnitX(), lat, lon);
        const Vector3d north = enu2ecef(Vector3d::UnitY(), lat, lon);
        const Vector3d up = enu2ecef(Vector3d::UnitZ(), lat, lon);
        enu_to_ecef.col(0) = east;
        enu_to_ecef.col(1) = north;
        enu_to_ecef.col(2) = up;
        Matrix3d q_enu = Matrix3d::Zero();
        q_enu(0, 0) = q_enu(1, 1) = 0.2 * 0.2 * std::abs(dt);
        q_enu(2, 2) = 0.1 * 0.1 * std::abs(dt);
        Q.block(filter_state.accel_index, filter_state.accel_index, 3, 3) =
            enu_to_ecef * q_enu * enu_to_ecef.transpose();
    } else if (use_dynamic_prediction) {
        const double position_q =
            std::max(0.0, config.process_noise_position) * dt;
        const double velocity_q =
            std::max(0.0, config.process_noise_velocity) * dt;
        for (int axis = 0; axis < 3; ++axis) {
            Q(filter_state.pos_index + axis, filter_state.pos_index + axis) =
                position_q;
            Q(filter_state.vel_index + axis, filter_state.vel_index + axis) =
                velocity_q;
        }
    }
    // Receiver clock temporal update. Both modes treat the clock as WHITE
    // NOISE re-initialized from the SPP solution each epoch, mirroring
    // RTKLIB/MRTKLIB PPP udclk_ppp (mrtk_ppp.c:822: initx(CLIGHT*dtr,
    // VAR_CLK) every epoch; VAR_CLK = 60^2 m^2). A random-walk clock model
    // is NOT viable for consumer receivers: this class of hardware drifts
    // ~150 m/s (measured -30.5 m per 0.2 s epoch on the PPC tokyo_run2
    // rover), so any Q small enough to keep the ambiguity float covariance
    // well-conditioned lags the true clock, and a multi-second data outage
    // (bridge) accumulates hundreds of meters of clock error against a
    // few-meter sigma -- a guaranteed rejection spiral.
    //
    // When the SPP seed is unavailable (deep urban canyon, <4 usable
    // satellites), the dynamics filter coasts the clock state and must
    // inflate its variance by the drift accumulated over dt: the drift is
    // quasi-deterministic, so the coast variance grows with dt^2, not dt.
    const bool clock_coasting = use_dynamic_prediction && !seed_valid;
    const double clock_coast_drift_m = config.clas_dynamic_clock_coast_drift_mps * dt;
    const double clock_process_noise = clock_coasting
        ? clock_coast_drift_m * clock_coast_drift_m
        : config.clas_clock_variance;
    Q(filter_state.clock_index, filter_state.clock_index) = clock_process_noise;
    Q(filter_state.glo_clock_index, filter_state.glo_clock_index) = clock_process_noise;
    if (filter_state.qzs_clock_index >= 0) {
        Q(filter_state.qzs_clock_index, filter_state.qzs_clock_index) =
            pppEnvOverrides().isb_process_noise * dt;
    }
    Q(filter_state.trop_index, filter_state.trop_index) =
        effectiveClasTropProcessNoise(config) * dt;
    // MRTKLIB literal-port track: ionosphere="est-adaptive". udion() keeps a
    // persistent per-satellite Q rate, clamps it to [0.001^2, 0.05^2], and
    // adds Q*dt only for satellites observed in the current epoch. filter2()
    // refreshes that rate after the measurement update below. Bias remains a
    // fixed 0.001^2 cycle^2/s random walk.
    const bool mrtklib_parity = clasMrtklibFloatParity(config);
    constexpr double kMrtklibIonoProcessNoise = 0.001 * 0.001;   // (m^2/s)
    constexpr double kMrtklibBiasProcessNoise = 0.001 * 0.001;   // (m^2/s)
    if (config.estimate_ionosphere) {
        for (const auto& [satellite, state_index] :
             filter_state.ionosphere_indices) {
            if (state_index >= 0 && state_index < nx) {
                if (mrtklib_parity) {
                    if (observed_satellites != nullptr &&
                        observed_satellites->count(satellite) == 0) {
                        continue;
                    }
                    double& iono_process_noise =
                        filter_state.adaptive_ionosphere_process_noise[satellite];
                    if (iono_process_noise == 0.0) {
                        iono_process_noise = kMrtklibIonoProcessNoise;
                    } else {
                        iono_process_noise = std::clamp(
                            iono_process_noise,
                            kMrtklibIonoProcessNoise,
                            0.05 * 0.05);
                    }
                    Q(state_index, state_index) =
                        iono_process_noise * dt;
                } else {
                    Q(state_index, state_index) =
                        effectiveClasIonoProcessNoise(config) * dt;
                }
            }
        }
    }
    const double bias_process_noise = mrtklib_parity
        ? kMrtklibBiasProcessNoise
        : config.process_noise_ambiguity;
    for (const auto& [ambiguity_satellite, state_index] :
         filter_state.ambiguity_indices) {
        if (state_index >= 0 && state_index < nx) {
            double scale2 = 1.0;
            if (mrtklib_parity) {
                const auto wavelength_it =
                    filter_state.ambiguity_wavelengths_m.find(ambiguity_satellite);
                if (wavelength_it != filter_state.ambiguity_wavelengths_m.end() &&
                    wavelength_it->second > 0.0) {
                    scale2 = wavelength_it->second * wavelength_it->second;
                }
            }
            Q(state_index, state_index) = bias_process_noise * scale2 * dt;
        }
    }
    // MRTKLIB v0.5.1 udpos_ppp() deliberately does not use the F employed
    // above for its covariance propagation.  Its two in-place loops add
    // pos<-vel and vel<-acc on each side of P, but omit the dt^2/2
    // pos<-acc element that is present in the state prediction.  Preserve
    // that literal asymmetry on the parity path; using the mathematically
    // complete F here changes the carrier Kalman gains enough to alter the
    // postfit vsat admission and therefore the float-reset cadence.
    filter_state.covariance =
        covariance_F * filter_state.covariance * covariance_F.transpose() + Q;
    // White-noise clock: re-initialize the clock STATE from the SPP
    // solution every epoch in both position models (RTKLIB PPP udclk_ppp
    // semantics -- dynamics in udpos_ppp only ever applies to pos/vel/acc,
    // never to the clock). In dynamics mode the measurement update then
    // refines the clock within the reseed variance set below.
    if (seed_valid) {
        const double gps_clock_before = filter_state.state(filter_state.clock_index);
        filter_state.state(filter_state.clock_index) = seed_receiver_clock_bias_m;
        filter_state.state(filter_state.glo_clock_index) = seed_receiver_clock_bias_m;
        if (filter_state.qzs_clock_index >= 0) {
            const double isb_offset =
                filter_state.state(filter_state.qzs_clock_index) - gps_clock_before;
            filter_state.state(filter_state.qzs_clock_index) =
                seed_receiver_clock_bias_m + isb_offset;
        }
    }
    // Decouple clock from position/ambiguities: zero cross-covariance to
    // prevent code observation noise from leaking into position (and to keep
    // the ambiguity float covariance well-conditioned for AR/LAMBDA search)
    // via KF coupling. This cross-term zeroing is safe and desirable in both
    // modes: any correlation useful for a single measurement update is
    // re-derived within that same update from the current epoch's H/R, so
    // zeroing the *carried-forward* cross-covariance each predict step only
    // discards stale, epoch-old correlation.
    //
    // The diagonal (clock's own variance) differs by mode:
    //  - White-noise mode: the clock was just hard-reseeded from SPP, and
    //    position is also re-anchored to SPP each epoch, so the clock is
    //    treated as exactly known (variance 0; measurement update leaves it
    //    at the SPP value).
    //  - Dynamics mode with a valid seed: RTKLIB VAR_CLK semantics -- the
    //    reseeded SPP clock is a prior with ~60 m sigma and the measurement
    //    update refines it each epoch. The variance must NOT be zeroed
    //    (that would freeze the clock at the SPP value; worse, on epochs
    //    where the reseed is skipped it would freeze the clock entirely,
    //    which was the original divergence root cause) and must not be left
    //    at clas_clock_variance=1e8 either (destroys LAMBDA conditioning).
    //  - Dynamics mode coasting (no SPP seed): keep the propagated
    //    variance, which already includes the dt^2 drift inflation from Q.
    if (config.clas_decouple_clock_position) {
        const int ci = filter_state.clock_index;
        const int gi = filter_state.glo_clock_index;
        const int qi = filter_state.qzs_clock_index;
        for (int i = 0; i < nx; ++i) {
            if (i != ci) { filter_state.covariance(ci, i) = 0; filter_state.covariance(i, ci) = 0; }
            if (i != gi) { filter_state.covariance(gi, i) = 0; filter_state.covariance(i, gi) = 0; }
            if (qi >= 0 && i != qi) {
                filter_state.covariance(qi, i) = 0;
                filter_state.covariance(i, qi) = 0;
            }
        }
        if (!use_dynamic_prediction) {
            filter_state.covariance(ci, ci) = 0;
            filter_state.covariance(gi, gi) = 0;
        } else if (seed_valid) {
            filter_state.covariance(ci, ci) = config.clas_dynamic_clock_reseed_variance;
            filter_state.covariance(gi, gi) = config.clas_dynamic_clock_reseed_variance;
        }
    }
}


void ensureAmbiguityStates(
    ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    double initial_variance) {
    auto allocate_ambiguity = [&](const SatelliteId& ambiguity_satellite) {
        if (filter_state.ambiguity_indices.find(ambiguity_satellite) !=
            filter_state.ambiguity_indices.end()) {
            return;
        }
        const int new_index = filter_state.total_states;
        filter_state.ambiguity_indices[ambiguity_satellite] = new_index;
        filter_state.total_states++;

        VectorXd new_state = VectorXd::Zero(filter_state.total_states);
        new_state.head(new_index) = filter_state.state;
        filter_state.state = new_state;

        MatrixXd new_covariance =
            MatrixXd::Zero(filter_state.total_states, filter_state.total_states);
        new_covariance.topLeftCorner(new_index, new_index) = filter_state.covariance;
        new_covariance(new_index, new_index) = initial_variance;
        filter_state.covariance = new_covariance;
    };

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        allocate_ambiguity(osr.satellite);
        if (osr.wavelengths[0] > 0.0) {
            filter_state.ambiguity_wavelengths_m[osr.satellite] =
                osr.wavelengths[0];
        }
        if (osr.num_frequencies >= 2) {
            const uint8_t l2_prn =
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId l2_satellite(osr.satellite.system, l2_prn);
            allocate_ambiguity(l2_satellite);
            if (osr.wavelengths[1] > 0.0) {
                filter_state.ambiguity_wavelengths_m[l2_satellite] =
                    osr.wavelengths[1];
            }
        }
    }
}

void applyPendingPhaseBiasStateShifts(
    ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    bool debug_enabled) {
    for (const auto& osr : osr_corrections) {
        auto repair_it = phase_bias_repair.find(osr.satellite);
        if (repair_it == phase_bias_repair.end()) {
            continue;
        }
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const double shift_cycles =
                repair_it->second.pending_state_shift_cycles[static_cast<size_t>(f)];
            if (shift_cycles == 0.0 || osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const uint8_t ambiguity_prn = f == 0 ? osr.satellite.prn :
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId ambiguity_satellite(osr.satellite.system, ambiguity_prn);
            const auto ambiguity_it =
                filter_state.ambiguity_indices.find(ambiguity_satellite);
            if (ambiguity_it == filter_state.ambiguity_indices.end()) {
                continue;
            }
            filter_state.state(ambiguity_it->second) += shift_cycles * osr.wavelengths[f];
            repair_it->second.pending_state_shift_cycles[static_cast<size_t>(f)] = 0.0;
            if (debug_enabled) {
                std::cerr << "[CLAS-PBIAS] state shift "
                          << ambiguity_satellite.toString()
                          << " f=" << f
                          << " cycles=" << shift_cycles
                          << " meters=" << shift_cycles * osr.wavelengths[f]
                          << "\n";
            }
        }
    }
}


KalmanUpdateStats applyMeasurementUpdate(
    ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const ppp_shared::PPPConfig& config,
    const PositionSolution* seed_solution) {
    KalmanUpdateStats stats;
    stats.nobs = static_cast<int>(measurements.size());
    if (stats.nobs < 4) {
        return stats;
    }

    MatrixXd H = MatrixXd::Zero(stats.nobs, filter_state.total_states);
    VectorXd z = VectorXd::Zero(stats.nobs);
    MatrixXd R = MatrixXd::Zero(stats.nobs, stats.nobs);

    for (int i = 0; i < stats.nobs; ++i) {
        H.row(i) = measurements[static_cast<size_t>(i)].H;
        z(i) = measurements[static_cast<size_t>(i)].residual;
        R(i, i) = measurements[static_cast<size_t>(i)].variance;
    }

    // MRTKLIB ddcov() (mrtk_ppp_rtk.c:914-927): all DD rows in a
    // system/frequency/type block share the reference-satellite error Ri,
    // while only each diagonal adds its target error Rj. Keep this strictly
    // parity-gated so legacy CLAS SD rows retain their diagonal covariance.
    if (clasMrtklibFloatParity(config)) {
        for (int i = 0; i < stats.nobs; ++i) {
            const auto& row_i = measurements[static_cast<size_t>(i)];
            if (row_i.dd_covariance_block < 0) {
                continue;
            }
            for (int j = i + 1; j < stats.nobs; ++j) {
                const auto& row_j = measurements[static_cast<size_t>(j)];
                if (row_j.dd_covariance_block != row_i.dd_covariance_block) {
                    continue;
                }
                R(i, j) = row_i.reference_variance;
                R(j, i) = row_i.reference_variance;
            }
        }
    }

    const bool mrtklib_parity = clasMrtklibFloatParity(config);
    MatrixXd S;
    if (mrtklib_parity) {
        // MRTKLIB filter2() compacts the state first, then filter2_() forms
        // Q=H'*P*H+R and runs residual_test().  Inactive zero-valued states
        // can still have a positive covariance in the native full matrix;
        // allowing those states into S changes which DD observations survive
        // the prefit gate (and consequently which ambiguities reach PAR).
        std::vector<int> gate_active;
        gate_active.reserve(static_cast<size_t>(filter_state.total_states));
        for (int i = 0; i < filter_state.total_states; ++i) {
            if (filter_state.state(i) != 0.0 &&
                filter_state.covariance(i, i) > 0.0) {
                gate_active.push_back(i);
            }
        }
        const int gate_states = static_cast<int>(gate_active.size());
        MatrixXd H_gate(stats.nobs, gate_states);
        MatrixXd P_gate(gate_states, gate_states);
        for (int i = 0; i < gate_states; ++i) {
            const int source_i = gate_active[static_cast<size_t>(i)];
            H_gate.col(i) = H.col(source_i);
            for (int j = 0; j < gate_states; ++j) {
                P_gate(i, j) = filter_state.covariance(
                    source_i, gate_active[static_cast<size_t>(j)]);
            }
        }
        S = H_gate * P_gate * H_gate.transpose() + R;

        // MRTKLIB v0.5.1 filter2_()/residual_test() prefit stage
        // (mrtk_ppp_rtk.c:955-1185): reject against the full innovation
        // covariance H'PH+R, compact the surviving rows, and do not update
        // when every actual DD observation was rejected. clas.toml uses
        // l1_l2_residual=2 sigma; D2 permits 10x for a newly initialized
        // phase bias (P == 1e4 m^2).
        std::vector<int> accepted;
        int accepted_dd_observations = 0;
        std::set<int> pair_rejected_rows;
        std::map<SatelliteId, std::array<int, 2>> phase_pair_rows;
        for (int i = 0; i < stats.nobs; ++i) {
            const auto& row = measurements[static_cast<size_t>(i)];
            if (!row.is_phase || row.freq_index < 0 || row.freq_index > 1) {
                continue;
            }
            auto result = phase_pair_rows.try_emplace(
                row.satellite, std::array<int, 2>{-1, -1});
            result.first->second[static_cast<size_t>(row.freq_index)] = i;
        }
        constexpr double kF1F2 = 1575.42e6 / 1227.60e6;
        constexpr double kGamma = kF1F2 * kF1F2;
        for (const auto& [satellite, rows] : phase_pair_rows) {
            (void)satellite;
            if (rows[0] < 0 || rows[1] < 0) {
                continue;
            }
            const double v0 = z(rows[0]);
            const double v1 = z(rows[1]);
            const double dispersive =
                kF1F2 * (v0 - v1) / (1.0 - kGamma);
            const double nondispersive =
                (kGamma * v0 - v1) / (kGamma - 1.0);
            const double variance = std::max(S(rows[0], rows[0]),
                                             S(rows[1], rows[1]));
            if (variance > 0.0 &&
                (dispersive * dispersive > 9.0 * variance ||
                 nondispersive * nondispersive > 9.0 * variance)) {
                pair_rejected_rows.insert(rows[0]);
                pair_rejected_rows.insert(rows[1]);
            }
        }
        for (int i = 0; i < stats.nobs; ++i) {
            const auto& row = measurements[static_cast<size_t>(i)];
            if (row.freq_index < 0) {
                accepted.push_back(i);  // native-only atmosphere constraints
                continue;
            }
            double threshold = 2.0;
            if (row.is_phase && row.ambiguity_fresh) {
                threshold *= 10.0;
            }
            const bool row_accepted =
                pair_rejected_rows.count(i) == 0 &&
                S(i, i) > 0.0 &&
                z(i) * z(i) < S(i, i) * threshold * threshold;
            if (ppp_internal::pppDebugEnabled() && row.is_phase &&
                (!row_accepted ||
                 row.satellite.system == GNSSSystem::QZSS)) {
                std::cerr << "[CLAS-PHASE-GATE] tow="
                          << (seed_solution != nullptr
                                  ? seed_solution->time.tow
                                  : -1.0)
                          << " sat="
                          << row.satellite.toString()
                          << " f=" << row.freq_index
                          << " v=" << z(i)
                          << " S=" << S(i, i)
                          << " th=" << threshold
                          << " pair_rej=" << pair_rejected_rows.count(i)
                          << " accepted=" << row_accepted << '\n';
            }
            if (row_accepted) {
                accepted.push_back(i);
                ++accepted_dd_observations;
            }
            if (row.is_phase && !row_accepted) {
                const uint8_t ambiguity_prn =
                    row.freq_index == 0
                        ? row.satellite.prn
                        : static_cast<uint8_t>(std::min(
                              255, static_cast<int>(row.satellite.prn) + 100));
                stats.rejected_phase_ambiguities.insert(
                    SatelliteId(row.satellite.system, ambiguity_prn));
            }
        }
        if (accepted_dd_observations == 0) {
            if (ppp_internal::pppDebugEnabled()) {
                std::cerr << "[CLAS-FILTER2] all DD measurements rejected total="
                          << stats.nobs << "\n";
            }
            stats.nobs = 0;
            return stats;
        }
        if (ppp_internal::pppDebugEnabled()) {
            std::cerr << "[CLAS-FILTER2] accepted_dd="
                      << accepted_dd_observations
                      << " total=" << stats.nobs << "\n";
        }
        if (accepted.size() != static_cast<size_t>(stats.nobs)) {
            const int kept = static_cast<int>(accepted.size());
            MatrixXd kept_H(kept, filter_state.total_states);
            VectorXd kept_z(kept);
            MatrixXd kept_R(kept, kept);
            for (int i = 0; i < kept; ++i) {
                kept_H.row(i) = H.row(accepted[static_cast<size_t>(i)]);
                kept_z(i) = z(accepted[static_cast<size_t>(i)]);
                for (int j = 0; j < kept; ++j) {
                    kept_R(i, j) = R(accepted[static_cast<size_t>(i)],
                                     accepted[static_cast<size_t>(j)]);
                }
            }
            H = std::move(kept_H);
            z = std::move(kept_z);
            R = std::move(kept_R);
            stats.nobs = kept;
            S = H * filter_state.covariance * H.transpose() + R;
        }
    } else {
        for (int i = 0; i < stats.nobs; ++i) {
            const double sigma = std::sqrt(R(i, i));
            if (std::abs(z(i)) > config.clas_outlier_sigma_scale * sigma) {
                R(i, i) = 1e10;
            }
        }
        S = H * filter_state.covariance * H.transpose() + R;
    }
    if (mrtklib_parity) {
        // MRTKLIB stores phase-bias states in cycles and applies wavelength
        // coefficients in H. Native stores the same states in metres. Work in
        // MRTKLIB's cycle coordinates for the parity update, then transform
        // the posterior back to metres. Although this is an exact similarity
        // transform algebraically, doing the inversion in metre coordinates
        // changes Qb by several percent at fresh 1e4-cycle states.
        VectorXd state_scale = VectorXd::Ones(filter_state.total_states);
        for (const auto& [satellite, state_index] :
             filter_state.ambiguity_indices) {
            const auto wavelength_it =
                filter_state.ambiguity_wavelengths_m.find(satellite);
            if (state_index >= 0 && state_index < filter_state.total_states &&
                wavelength_it != filter_state.ambiguity_wavelengths_m.end() &&
                wavelength_it->second > 0.0) {
                state_scale(state_index) = wavelength_it->second;
            }
        }
        const VectorXd inverse_scale = state_scale.cwiseInverse();
        const VectorXd state_cycles =
            inverse_scale.array() * filter_state.state.array();

        // MRTKLIB filter2() (mrtk_ppp_rtk.c:1210-1244) compacts x/P/H to
        // states satisfying x[i] != 0 && P[i,i] > 0 before filter2_(), then
        // copies only that posterior submatrix back.  Preserve that exact
        // lifecycle here: zero-valued, not-yet-initialized states must not
        // enter H'PH or acquire cross-covariance through this update.
        std::vector<int> active;
        active.reserve(static_cast<size_t>(filter_state.total_states));
        for (int i = 0; i < filter_state.total_states; ++i) {
            if (state_cycles(i) != 0.0 &&
                filter_state.covariance(i, i) > 0.0) {
                active.push_back(i);
            }
        }
        const int na = static_cast<int>(active.size());
        if (ppp_internal::pppDebugEnabled()) {
            std::cerr << "[CLAS-FILTER2] active_states=" << na
                      << " total_states=" << filter_state.total_states << '\n';
        }
        if (na == 0) {
            stats.nobs = 0;
            return stats;
        }
        VectorXd x_active(na);
        MatrixXd P_active(na, na);
        MatrixXd H_active(stats.nobs, na);
        for (int i = 0; i < na; ++i) {
            const int source_i = active[static_cast<size_t>(i)];
            x_active(i) = state_cycles(source_i);
            H_active.col(i) = H.col(source_i) * state_scale(source_i);
            for (int j = 0; j < na; ++j) {
                const int source_j = active[static_cast<size_t>(j)];
                P_active(i, j) =
                    filter_state.covariance(source_i, source_j) *
                    inverse_scale(source_i) * inverse_scale(source_j);
            }
        }
        const MatrixXd innovation =
            H_active * P_active * H_active.transpose() + R;
        const MatrixXd K =
            P_active * H_active.transpose() * innovation.inverse();
        const VectorXd dx_active = K * z;
        const MatrixXd posterior =
            (MatrixXd::Identity(na, na) - K * H_active) * P_active;
        x_active += dx_active;

        VectorXd dx = VectorXd::Zero(filter_state.total_states);
        for (int i = 0; i < na; ++i) {
            const int target_i = active[static_cast<size_t>(i)];
            filter_state.state(target_i) =
                x_active(i) * state_scale(target_i);
            dx(target_i) = dx_active(i) * state_scale(target_i);
            for (int j = 0; j < na; ++j) {
                const int target_j = active[static_cast<size_t>(j)];
                filter_state.covariance(target_i, target_j) =
                    posterior(i, j) * state_scale(target_i) *
                    state_scale(target_j);
            }
        }
        stats.dx = dx;
    } else {
        const MatrixXd K =
            filter_state.covariance * H.transpose() * S.inverse();
        const VectorXd dx = K * z;
        filter_state.state += dx;
        const MatrixXd I_KH =
            MatrixXd::Identity(filter_state.total_states,
                               filter_state.total_states) - K * H;
        filter_state.covariance =
            I_KH * filter_state.covariance * I_KH.transpose() +
            K * R * K.transpose();
        stats.dx = dx;
    }

    stats.updated = true;
    stats.residuals = z;
    stats.variances = R.diagonal();
    stats.pre_anchor_covariance = filter_state.covariance;

    const bool apply_spp_position_anchor =
        seed_solution != nullptr && seed_solution->isValid() &&
        (!config.kinematic_mode || config.low_dynamics_mode);
    if (apply_spp_position_anchor) {
        const double anchor_sigma = config.clas_anchor_sigma;
        for (int axis = 0; axis < 3; ++axis) {
            const int idx = axis;
            const double innovation =
                seed_solution->position_ecef(axis) - filter_state.state(idx);
            const double innovation_covariance =
                filter_state.covariance(idx, idx) + anchor_sigma * anchor_sigma;
            if (innovation_covariance > 0.0) {
                VectorXd K_col = filter_state.covariance.col(idx) / innovation_covariance;
                filter_state.state += K_col * innovation;
                filter_state.covariance -= K_col * filter_state.covariance.row(idx);
            }
        }
    }

    return stats;
}


PositionSolution finalizeEpochSolution(
    const ppp_shared::PPPState& filter_state,
    const GNSSTime& /*time*/,
    bool fixed,
    double ar_ratio,
    int fixed_ambiguities,
    int num_satellites) {
    PositionSolution solution;
    solution.position_ecef = filter_state.state.segment(0, 3);
    solution.receiver_clock_bias = filter_state.state(filter_state.clock_index);
    solution.status = fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT;
    solution.ratio = fixed ? ar_ratio : 0.0;
    solution.num_fixed_ambiguities = fixed ? fixed_ambiguities : 0;
    solution.num_satellites = num_satellites;
    return solution;
}

}  // namespace libgnss::ppp_clas
