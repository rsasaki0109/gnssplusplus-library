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

MeasurementBuildResult buildEpochMeasurements(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    double receiver_clock_m,
    double trop_zenith,
    const std::map<std::string, std::string>& epoch_atmos,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function,
    bool debug_enabled,
    int reference_rank) {
    (void)receiver_clock_m;
    MeasurementBuildResult result;
    const bool stec_constraint = usesClasStecConstraint(config);
    const bool mrtklib_parity = clasMrtklibFloatParity(config);
    std::map<SatelliteId, double> iono_state_targets_m;
    std::map<SatelliteId, double> iono_state_variances_m2;
    std::set<int> fresh_ambiguity_indices;
    // MRTKLIB v0.5.1 udbias_ppp() (mrtk_ppp_rtk.c:761-797): initialize
    // each phase-bias state from raw phase-minus-code, after removing the
    // common offset inferred from already-initialized satellites. Native
    // ambiguity states are stored in metres (MRTKLIB stores cycles), so the
    // literal equivalent is L*lambda-P-com_bias. initx() also clears every
    // cross-covariance of a newly initialized state.
    if (mrtklib_parity && ambiguity_reset_function) {
        struct BiasSeed {
            SatelliteId satellite;
            int state_index = -1;
            double phase_minus_code_m = 0.0;
            double wavelength_m = 0.0;
        };
        std::array<std::vector<BiasSeed>, OSR_MAX_FREQ> bias_seeds;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) continue;
            // The v0.5.1 CLAS residual path leaves Galileo unusable on this
            // dataset (ssat.vsat remains zero); retain those states for the
            // common layout but do not initialize them from absent rows.
            if (osr.satellite.system == GNSSSystem::Galileo) continue;
            for (int f = 0; f < osr.num_frequencies; ++f) {
                if (osr.satellite.system == GNSSSystem::QZSS && f > 0) {
                    continue;
                }
                const Observation* raw =
                    findMrtklibParityRawObservation(obs, osr, f);
                if (raw == nullptr || !raw->valid || !raw->has_carrier_phase ||
                    !raw->has_pseudorange || !std::isfinite(raw->carrier_phase) ||
                    !std::isfinite(raw->pseudorange) || osr.wavelengths[f] <= 0.0) {
                    continue;
                }
                const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                    : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
                const auto amb_it = filter_state.ambiguity_indices.find(
                    SatelliteId(osr.satellite.system, amb_prn));
                if (amb_it == filter_state.ambiguity_indices.end()) continue;
                bias_seeds[static_cast<size_t>(f)].push_back({
                    SatelliteId(osr.satellite.system, amb_prn),
                    amb_it->second,
                    raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange,
                    osr.wavelengths[f]});
            }
        }
        for (auto& frequency_seeds : bias_seeds) {
            double offset_sum_m = 0.0;
            int offset_count = 0;
            for (const auto& seed : frequency_seeds) {
                if (filter_state.state(seed.state_index) != 0.0 &&
                    filter_state.covariance(seed.state_index, seed.state_index) <
                        config.clas_ambiguity_reinit_threshold) {
                    offset_sum_m += seed.phase_minus_code_m -
                                    filter_state.state(seed.state_index);
                    ++offset_count;
                }
            }
            const double common_bias_m = offset_count > 0
                ? offset_sum_m / static_cast<double>(offset_count)
                : 0.0;
            if (debug_enabled &&
                std::abs(obs.time.tow - 177091.4) < 0.01) {
                std::cerr << "[CLAS-BIAS-SEED] tow=" << obs.time.tow
                          << " common=" << common_bias_m
                          << " count=" << offset_count << "\n";
                for (const auto& seed : frequency_seeds) {
                    std::cerr << "[CLAS-BIAS-SEED] sat="
                              << seed.satellite.toString()
                              << " raw=" << seed.phase_minus_code_m
                              << " state=" << filter_state.state(seed.state_index)
                              << " var="
                              << filter_state.covariance(seed.state_index,
                                                         seed.state_index)
                              << "\n";
                }
            }
            for (const auto& seed : frequency_seeds) {
                if (filter_state.state(seed.state_index) != 0.0 &&
                    filter_state.covariance(seed.state_index, seed.state_index) <
                        config.clas_ambiguity_reinit_threshold) {
                    continue;
                }
                filter_state.state(seed.state_index) =
                    seed.phase_minus_code_m - common_bias_m;
                filter_state.covariance.row(seed.state_index).setZero();
                filter_state.covariance.col(seed.state_index).setZero();
                filter_state.covariance(seed.state_index, seed.state_index) =
                    1e4 * seed.wavelength_m * seed.wavelength_m;
                fresh_ambiguity_indices.insert(seed.state_index);
            }
        }
    }

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        if (mrtklib_parity &&
            osr.satellite.system == GNSSSystem::Galileo) {
            continue;
        }
        // MRTKLIB clas_osr_corrmeas() requires STEC data for this satellite
        // at the selected CLAS grid and returns 0 on search_data "no sat
        // data" (mrtk_clas_osr.c:575-584). Native historically falls back
        // across grids/means, which admitted extra DD rows (G03 at tow
        // 177000). Preserve that fallback outside the literal parity path.
        if (mrtklib_parity &&
            ((osr.atmos_stec_satellite_membership_known &&
              !osr.atmos_stec_has_satellite) ||
             (osr.atmos_grid_satellite_membership_known &&
              !osr.atmos_grid_has_satellite))) {
            continue;
        }

        const double geo = geodist(osr.satellite_position, receiver_position);
        const Vector3d los =
            (osr.satellite_position - receiver_position).normalized();
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double receiver_clock_m =
            receiverClockBiasMeters(filter_state, osr.satellite);
        const int receiver_clock_index =
            receiverClockStateIndex(filter_state, osr.satellite);
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_modeled = trop_mapping * trop_zenith;
        // MRTKLIB ionmapf(): single-layer shell at HION=350 km. ddres()
        // multiplies every estimated vertical L1 ionosphere state by this
        // satellite-specific slant factor before applying the frequency
        // ratio. Native historically used only the frequency ratio.
        double receiver_lat = 0.0;
        double receiver_lon = 0.0;
        double receiver_height = 0.0;
        ecef2geodetic(receiver_position, receiver_lat, receiver_lon,
                     receiver_height);
        (void)receiver_lat;
        (void)receiver_lon;
        constexpr double kMrtklibIonosphereHeightM = 350000.0;
        const double ionosphere_mapping =
            receiver_height >= kMrtklibIonosphereHeightM
                ? 1.0
                : 1.0 / std::cos(std::asin(
                      (constants::WGS84_A + receiver_height) /
                      (constants::WGS84_A + kMrtklibIonosphereHeightM) *
                      std::cos(osr.elevation)));

        std::array<const Observation*, OSR_MAX_FREQ> raw_observations{};
        const auto iono_state_it = filter_state.ionosphere_indices.find(osr.satellite);
        const int iono_state_index =
            iono_state_it != filter_state.ionosphere_indices.end() ?
                iono_state_it->second : -1;
        const bool use_residual_iono_state =
            config.estimate_ionosphere &&
            iono_state_index >= 0 &&
            iono_state_index < filter_state.total_states;
        for (int f = 0; f < osr.num_frequencies; ++f) {
            if (mrtklib_parity &&
                osr.satellite.system == GNSSSystem::QZSS && f > 0) {
                continue;
            }
            raw_observations[static_cast<size_t>(f)] =
                findOsrFrequencyObservation(obs, osr, f);
        }

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* raw = raw_observations[static_cast<size_t>(f)];
            if (!raw || !raw->valid) {
                continue;
            }
            // clas_osr_zdres() skips the complete frequency cell when either
            // satellite code bias or phase bias is CSSRINVALID.  Applying a
            // sibling signal's collapsed RTCM-band value would admit rows
            // that do not exist in MRTKLIB (for example G04 C2L at 177067.2).
            if ((mrtklib_parity || usesClaslibBiasValidityRows(config)) &&
                (!osr.code_bias_present[f] || !osr.phase_bias_present[f])) {
                continue;
            }
            // MRTKLIB v0.5.1 clas_osr_zdres() multiplies its uint16 SNR
            // (stored in 0.001 dB-Hz units) by the legacy 0.25 factor before
            // testsnr().  The resulting thousands-of-dB-Hz value means the
            // configured mask never rejects a CLAS measurement.  Preserve
            // that observable behavior on the literal parity path.
            if (!mrtklib_parity &&
                config.kinematic_mode &&
                config.use_clas_osr_filter &&
                raw->snr > 0.0 &&
                ppp_ar::clasKinematicSnrMasked(f, osr.elevation, raw->snr)) {
                continue;
            }
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            auto applied_corrections = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);
            const double effective_iono_scale =
                iono_scale * (mrtklib_parity ? ionosphere_mapping : 1.0);
            if (mrtklib_parity) {
                applied_corrections.pseudorange_correction_m = osr.PRC[f];
                applied_corrections.carrier_phase_correction_m = osr.CPC[f];
            }
            if (stec_constraint && use_residual_iono_state &&
                osr.has_iono && std::isfinite(osr.iono_l1_m)) {
                const double iono_scaled = iono_scale * osr.iono_l1_m;
                applied_corrections.pseudorange_correction_m -= iono_scaled;
                applied_corrections.carrier_phase_correction_m += iono_scaled;
                iono_state_targets_m[osr.satellite] = osr.iono_l1_m;
                iono_state_variances_m2[osr.satellite] =
                    clasStecConstraintVariance(epoch_atmos, osr, config);
            }
            const double iono_state_m =
                use_residual_iono_state ? filter_state.state(iono_state_index) : 0.0;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const bool claslib_code_prc = usesClaslibCodePrcRows(config);
                const double code_trop_modeled =
                    (mrtklib_parity || claslib_code_prc) ? 0.0 : trop_modeled;
                const double code_trop_mapping =
                    (mrtklib_parity || claslib_code_prc) ? 0.0 : trop_mapping;
                const double p_corr =
                    raw->pseudorange - applied_corrections.pseudorange_correction_m;
                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + code_trop_modeled +
                    effective_iono_scale * iono_state_m;
                const double residual = p_corr - predicted;
                const double el_weight = elevationWeight(osr.elevation);

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(receiver_clock_index) = 1.0;
                row.H(filter_state.trop_index) = code_trop_mapping;
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = effective_iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.residual = residual;
                row.variance = clasCodeVariance(config, osr.elevation);
                row.satellite = osr.satellite;
                row.is_phase = false;
                row.freq_index = f;
                result.measurements.push_back(row);
            }

            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                const double l_m = raw->carrier_phase * osr.wavelengths[f];
                const double l_corr =
                    l_m - applied_corrections.carrier_phase_correction_m;
                if (auto* dump = clasGeometryDumpStream();
                    dump != nullptr && selectedClasGeometryDumpTow(obs.time.tow)) {
                    const Vector3d forced_receiver_position =
                        clasGeometryDumpReceiverPosition(receiver_position);
                    const double forced_rho =
                        geodist(osr.satellite_position, forced_receiver_position);
                    const double euclidean =
                        (osr.satellite_position - forced_receiver_position).norm();
                    const double forced_sagnac = forced_rho - euclidean;
                    const double forced_sat_clk_m =
                        constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
                    double forced_lat = 0.0;
                    double forced_lon = 0.0;
                    double forced_h = 0.0;
                    ecef2geodetic(
                        forced_receiver_position, forced_lat, forced_lon, forced_h);
                    const Vector3d forced_los_enu = ecef2enu(
                        osr.satellite_position - forced_receiver_position,
                        forced_lat,
                        forced_lon);
                    const double forced_el = std::atan2(
                        forced_los_enu.z(),
                        std::hypot(forced_los_enu.x(), forced_los_enu.y()));
                    const double forced_az =
                        std::atan2(forced_los_enu.x(), forced_los_enu.y());
                    *dump << std::setprecision(17)
                          << "GEOM,"
                          << obs.time.week << ','
                          << obs.time.tow << ','
                          << osr.signal_transmit_time.tow << ','
                          << osr.satellite.toString() << ','
                          << f << ','
                          << static_cast<int>(raw->signal) << ','
                          << receiver_position.x() << ','
                          << receiver_position.y() << ','
                          << receiver_position.z() << ','
                          << receiver_position.x() << ','
                          << receiver_position.y() << ','
                          << receiver_position.z() << ','
                          << forced_receiver_position.x() << ','
                          << forced_receiver_position.y() << ','
                          << forced_receiver_position.z() << ','
                          << 0.0 << ','
                          << 0.0 << ','
                          << 0.0 << ','
                          << osr.satellite_position.x() << ','
                          << osr.satellite_position.y() << ','
                          << osr.satellite_position.z() << ','
                          << osr.satellite_velocity.x() << ','
                          << osr.satellite_velocity.y() << ','
                          << osr.satellite_velocity.z() << ','
                          << forced_sat_clk_m << ','
                          << euclidean << ','
                          << forced_sagnac << ','
                          << forced_rho << ','
                          << forced_az << ','
                          << forced_el << ','
                          << osr.CPC[f] << ','
                          << forced_rho - forced_sat_clk_m + osr.CPC[f]
                          << '\n';
                }
                const bool claslib_amb_datum_phase =
                    !mrtklib_parity && suppressesClasAmbDatumPhaseTrop(
                        config.clas_correction_application_policy);
                const bool residual_amb_datum_phase_trop =
                    !mrtklib_parity && usesResidualClasAmbDatumPhaseTrop(
                        config.clas_correction_application_policy);
                const double phase_trop_modeled = mrtklib_parity
                    ? 0.0
                    : (claslib_amb_datum_phase
                    ? 0.0
                    : (residual_amb_datum_phase_trop
                          ? trop_modeled - osr.trop_correction_m
                          : trop_modeled));
                const double phase_trop_partial =
                    (mrtklib_parity || claslib_amb_datum_phase)
                        ? 0.0 : trop_mapping;

                const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                    : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
                const SatelliteId amb_sat(osr.satellite.system, amb_prn);
                const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
                if (amb_it == filter_state.ambiguity_indices.end()) {
                    continue;
                }
                const int amb_idx = amb_it->second;

                // On the MRTKLIB parity path slip/outage handling has already
                // reset the bias before the udbias_ppp-style seeding pass at
                // the top of this function.  Resetting it again here would
                // erase the freshly seeded L*lambda-P state, leave x[IB]==0
                // (therefore inactive in filter2()), and force the several-
                // metre phase innovation into position/ionosphere instead.
                // Legacy CLAS keeps its historical late-reset ordering.
                if (!mrtklib_parity && raw->loss_of_lock &&
                    ambiguity_reset_function) {
                    ambiguity_reset_function(amb_sat, raw->signal);
                }

                if (!mrtklib_parity &&
                    filter_state.covariance(amb_idx, amb_idx) >=
                        config.clas_ambiguity_reinit_threshold) {
                    filter_state.state(amb_idx) =
                        l_corr - (geo - sat_clk_m + receiver_clock_m + phase_trop_modeled
                                  - effective_iono_scale * iono_state_m);
                }

                const double predicted_no_amb =
                    geo - sat_clk_m + receiver_clock_m + phase_trop_modeled
                    - effective_iono_scale * iono_state_m;
                const double predicted =
                    predicted_no_amb + filter_state.state(amb_idx);
                const double residual = l_corr - predicted;
                const double el_weight = elevationWeight(osr.elevation);

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(receiver_clock_index) = 1.0;
                row.H(filter_state.trop_index) = phase_trop_partial;
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = -effective_iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.H(amb_idx) = 1.0;
                row.residual = residual;
                row.variance =
                    clasPhaseVariance(config, osr.elevation, f, osr.frequencies[f]);
                row.satellite = osr.satellite;
                row.is_phase = true;
                row.ambiguity_fresh = fresh_ambiguity_indices.contains(amb_idx);
                row.freq_index = f;
                row.ambiguity_index = amb_idx;
                result.measurements.push_back(row);
                result.observed_ambiguities.push_back(
                    {amb_sat, raw->signal, osr.wavelengths[f], raw->carrier_phase, raw->snr});
                if (auto* dump = ambDatumDumpStream(); dump != nullptr) {
                    const auto lookup =
                        findOsrFrequencyObservationWithProvenance(obs, osr, f);
                    const double ambiguity_state_m = filter_state.state(amb_idx);
                    const double ambiguity_state_cycles =
                        osr.wavelengths[f] > 0.0
                            ? ambiguity_state_m / osr.wavelengths[f]
                            : 0.0;
                    const double iono_cpc_m = -iono_scale * osr.iono_l1_m;
                    *dump << std::setprecision(17)
                          << "PHASE,"
                          << obs.time.week << ','
                          << obs.time.tow << ','
                          << osr.satellite.toString() << ','
                          << f << ','
                          << static_cast<int>(raw->signal) << ','
                          << raw->carrier_phase_observation_type << ','
                          << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                                 raw->carrier_phase_observation_type) << ','
                          << ppp_internal::signalFamilyName(raw->signal) << ','
                          << osr.pseudorange_rinex_codes[f] << ','
                          << osr.carrier_rinex_codes[f] << ','
                          << (lookup.exact_identity_requested ? 1 : 0) << ','
                          << (lookup.exact_identity_matched ? 1 : 0) << ','
                          << (lookup.family_fallback ? 1 : 0) << ','
                          << static_cast<int>(osr.phase_bias_signal_ids[f]) << ','
                          << (osr.bias_exact_identity[f] ? 1 : 0) << ','
                          << static_cast<int>(osr.phase_bias_source_signal_ids[f]) << ','
                          << (osr.phase_bias_present[f] ? 1 : 0) << ','
                          << (osr.phase_bias_fallback[f] ? 1 : 0) << ','
                          << osr.wavelengths[f] << ','
                          << raw->carrier_phase << ','
                          << l_m << ','
                          << applied_corrections.carrier_phase_correction_m << ','
                          << osr.CPC[f] << ','
                          << (osr.CPC[f] - osr.trop_correction_m) << ','
                          << osr.trop_correction_m << ','
                          << osr.relativity_correction_m << ','
                          << osr.receiver_antenna_m[f] << ','
                          << iono_cpc_m << ','
                          << osr.phase_bias_m[f] << ','
                          << osr.windup_m[f] << ','
                          << osr.phase_compensation_m[f] << ','
                          << l_corr << ','
                          << predicted_no_amb << ','
                          << ambiguity_state_m << ','
                          << ambiguity_state_cycles << ','
                          << residual << '\n';
                }
            }
        }
    }

    if (!mrtklib_parity &&
        config.estimate_troposphere && !epoch_atmos.empty() &&
        usesClasTropospherePrior(config.clas_correction_application_policy)) {
        const double clas_trop_zenith =
            ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                epoch_atmos,
                receiver_position,
                obs.time,
                M_PI_2,
                config.clas_expanded_value_construction_policy,
                config.clas_subtype12_value_construction_policy,
                config.clas_expanded_residual_sampling_policy);
        if (std::isfinite(clas_trop_zenith) && clas_trop_zenith > 0.0) {
            MeasurementRow row;
            row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
            row.H(filter_state.trop_index) = 1.0;
            row.residual = clas_trop_zenith - trop_zenith;
            row.variance = effectiveClasTropPriorVariance(config);
            row.satellite = SatelliteId{};
            row.is_phase = false;
            row.freq_index = -1;
            result.measurements.push_back(row);
            if (debug_enabled) {
                std::cerr << "[CLAS-TROP] prior=" << clas_trop_zenith
                          << " state=" << trop_zenith << "\n";
            }
        }
    }

    if (config.estimate_ionosphere && !mrtklib_parity) {
        for (const auto& satellite : result.observed_iono_states) {
            const auto iono_it = filter_state.ionosphere_indices.find(satellite);
            if (iono_it == filter_state.ionosphere_indices.end()) {
                continue;
            }
            const int iono_state_index = iono_it->second;
            if (iono_state_index < 0 || iono_state_index >= filter_state.total_states) {
                continue;
            }
            MeasurementRow row;
            row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
            row.H(iono_state_index) = 1.0;
            const auto target_it = iono_state_targets_m.find(satellite);
            const double target_m =
                target_it != iono_state_targets_m.end() ? target_it->second : 0.0;
            row.residual = target_m - filter_state.state(iono_state_index);
            const auto variance_it = iono_state_variances_m2.find(satellite);
            row.variance = variance_it != iono_state_variances_m2.end()
                ? variance_it->second
                : config.clas_iono_prior_variance;
            row.satellite = satellite;
            row.is_phase = false;
            row.freq_index = -1;
            result.measurements.push_back(row);
        }
    }

    // Same-system differencing of the zero-difference rows. MRTKLIB ddres()
    // differences both phase and code by system/frequency, cancelling the
    // receiver clock exactly (mrtk_ppp_rtk.c:1295-1451). The legacy native
    // path keeps code undifferenced; the literal-port parity path differences
    // both families and carries the shared-reference covariance into ddcov.
    if (config.use_clas_osr_filter) {
        // Build elevation map for reference satellite selection
        std::map<SatelliteId, double> elevation_map;
        for (const auto& osr : osr_corrections) {
            if (osr.valid) {
                elevation_map[osr.satellite] = osr.elevation;
            }
        }

        // Group same-system measurements by signal. MRTKLIB validobs()
        // requires the corresponding phase observation before admitting a
        // code row (mrtk_ppp_rtk.c:896-899).
        struct SdGroupKey {
            GNSSSystem system;
            int freq_index;
            bool is_phase;
            bool operator<(const SdGroupKey& rhs) const {
                if (system != rhs.system) return system < rhs.system;
                // MRTKLIB ddres() iterates f=0..2*nf: every carrier-phase
                // frequency first, followed by every code frequency.  This
                // order also fixes the innovation/LU pivot order used by the
                // literal filter2 update.
                if (is_phase != rhs.is_phase) return is_phase > rhs.is_phase;
                return freq_index < rhs.freq_index;
            }
        };
        const bool mrtklib_dd = clasMrtklibFloatParity(config);
        const bool code_sd = mrtklib_dd || pppEnvOverrides().clas_code_sd;
        std::set<std::pair<SatelliteId, int>> phase_observations;
        if (mrtklib_dd) {
            for (const auto& measurement : result.measurements) {
                if (measurement.is_phase && measurement.freq_index >= 0) {
                    phase_observations.insert(
                        {measurement.satellite, measurement.freq_index});
                }
            }
        }
        std::map<SdGroupKey, std::vector<size_t>> sd_groups;
        for (size_t i = 0; i < result.measurements.size(); ++i) {
            const auto& m = result.measurements[i];
            if (m.freq_index < 0) continue;
            if (!m.is_phase && !code_sd) continue;
            if (mrtklib_dd && !m.is_phase &&
                phase_observations.count({m.satellite, m.freq_index}) == 0) {
                continue;
            }
            sd_groups[{m.satellite.system, m.freq_index, m.is_phase}].push_back(i);
        }

        // Start with prior constraints, plus undifferenced code rows on the
        // default path. The gated path lets code rows be rebuilt as SD below.
        std::vector<MeasurementRow> sd_measurements;
        for (size_t i = 0; i < result.measurements.size(); ++i) {
            const auto& m = result.measurements[i];
            if (m.freq_index < 0 || (!m.is_phase && !code_sd)) {
                sd_measurements.push_back(m);
            }
        }

        // Form DD rows for each selected group. MRTKLIB searches with >=, so
        // equal-elevation ties select the later observation (lines 1303-1321).
        int dd_covariance_block = 0;
        for (auto& [group_key, member_indices] : sd_groups) {
            if (member_indices.size() < 2) continue;

            if (mrtklib_dd && ppp_internal::pppDebugEnabled()) {
                std::cerr << "[CLAS-DD-GROUP] sys="
                          << static_cast<int>(group_key.system)
                          << " f=" << group_key.freq_index
                          << " phase=" << group_key.is_phase
                          << " sats=";
                for (size_t idx : member_indices) {
                    std::cerr << result.measurements[idx].satellite.toString() << ' ';
                }
                std::cerr << "\n";
            }

            // MRTKLIB ddres(niter): first use the highest-elevation member;
            // on the second reference pass exclude that member and select the
            // next highest. Equal-elevation ties prefer the later observation.
            auto ranked_members = member_indices;
            std::stable_sort(
                ranked_members.begin(), ranked_members.end(),
                [&](size_t lhs, size_t rhs) {
                    const double lhs_el = elevation_map.count(
                                              result.measurements[lhs].satellite)
                                              ? elevation_map.at(
                                                    result.measurements[lhs].satellite)
                                              : -1.0;
                    const double rhs_el = elevation_map.count(
                                              result.measurements[rhs].satellite)
                                              ? elevation_map.at(
                                                    result.measurements[rhs].satellite)
                                              : -1.0;
                    if (lhs_el != rhs_el) {
                        return lhs_el > rhs_el;
                    }
                    return lhs > rhs;
                });
            const size_t ref_index = ranked_members[static_cast<size_t>(
                std::min(std::max(reference_rank, 0),
                         static_cast<int>(ranked_members.size()) - 1))];

            // Form SD: reference - satellite
            const auto& ref_row = result.measurements[ref_index];
            for (size_t idx : member_indices) {
                if (idx == ref_index) continue;
                const auto& sat_row = result.measurements[idx];
                MeasurementRow sd_row;
                sd_row.H = ref_row.H - sat_row.H;
                sd_row.residual = ref_row.residual - sat_row.residual;
                sd_row.variance = ref_row.variance + sat_row.variance;
                if (mrtklib_dd) {
                    sd_row.reference_variance = ref_row.variance;
                    sd_row.dd_covariance_block = dd_covariance_block;
                    sd_row.reference_satellite = ref_row.satellite;
                }
                sd_row.satellite = sat_row.satellite;
                sd_row.is_phase = group_key.is_phase;
                sd_row.ambiguity_fresh = sat_row.ambiguity_fresh;
                sd_row.freq_index = group_key.freq_index;
                sd_row.ambiguity_index = sat_row.ambiguity_index;
                if (mrtklib_dd && ppp_internal::pppDebugEnabled()) {
                    std::cerr << std::setprecision(15)
                              << "[CLAS-DD-ROW] ref="
                              << ref_row.satellite.toString()
                              << " sat=" << sat_row.satellite.toString()
                              << " phase=" << group_key.is_phase
                              << " f=" << group_key.freq_index
                              << " v=" << sd_row.residual
                              << " ref_amb=" << ref_row.ambiguity_index
                              << " sat_amb=" << sat_row.ambiguity_index
                              << " ref_p="
                              << (ref_row.ambiguity_index >= 0
                                      ? filter_state.covariance(
                                            ref_row.ambiguity_index,
                                            ref_row.ambiguity_index)
                                      : -1.0)
                              << " sat_p="
                              << (sat_row.ambiguity_index >= 0
                                      ? filter_state.covariance(
                                            sat_row.ambiguity_index,
                                            sat_row.ambiguity_index)
                                      : -1.0)
                              << " Ri=" << ref_row.variance
                              << " Rj=" << sat_row.variance << '\n';
                }
                sd_measurements.push_back(sd_row);
            }
            if (mrtklib_dd) {
                ++dd_covariance_block;
            }
        }
        result.measurements = std::move(sd_measurements);
    }

    return result;
}

}  // namespace libgnss::ppp_clas
