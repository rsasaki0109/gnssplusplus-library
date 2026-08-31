#include <libgnss++/algorithms/fgo.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/doppler_contract.hpp>
#include <libgnss++/algorithms/tdcp_contract.hpp>
#include <libgnss++/algorithms/signal_bias_contract.hpp>
#include <libgnss++/algorithms/residual_ionosphere_contract.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#ifdef GNSSPP_HAS_CHOLMOD
#include <Eigen/CholmodSupport>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <tuple>


#include "fgo_internal.hpp"

namespace libgnss {

using namespace fgo_internal;
namespace upstream = observable_upstream;

FGOProcessor::FGOProblem FGOProcessor::buildPseudorangeProblem(
    const std::vector<ObservationData>& input_epochs,
    const NavigationData& nav) const {
    FGOProblem problem;
    problem.diagnostics.input_epochs = input_epochs.size();
    if (input_epochs.empty()) {
        return problem;
    }

    // MF hygiene: one observation code per (system, secondary band) for this
    // receiver (cssrlib/RTKLIB-style). Only meaningful when secondary bands
    // are ingested at all.
    SecondaryCodeTable rover_secondary_codes;
    const SecondaryCodeTable* rover_secondary_codes_ptr = nullptr;
    if (config_.use_multi_frequency_double_difference &&
        config_.use_double_difference_secondary_code_alignment &&
        config_.use_multi_constellation) {
        rover_secondary_codes = buildSecondaryCodeTable(input_epochs);
        rover_secondary_codes_ptr = &rover_secondary_codes;
    }

    ProcessorConfig spp_processor_config;
    spp_processor_config.mode = PositioningMode::SPP;
    spp_processor_config.elevation_mask = config_.min_elevation_deg;
    spp_processor_config.snr_mask = config_.min_snr_dbhz;
    spp_processor_config.use_ionosphere_model = config_.use_ionosphere_model;
    spp_processor_config.use_troposphere_model = config_.use_troposphere_model;

    SPPProcessor::SPPConfig spp_config;
    spp_config.apply_atmospheric_corrections =
        config_.use_ionosphere_model || config_.use_troposphere_model;
    spp_config.use_multi_constellation = config_.use_multi_constellation;
    spp_config.model_intersystem_bias = config_.spp_model_intersystem_bias;
    spp_config.use_signal_specific_galileo_group_delay =
        config_.use_signal_specific_galileo_group_delay;

    SPPProcessor spp_processor(spp_config);
    spp_processor.initialize(spp_processor_config);

    const double effective_min_elevation_deg =
        config_.use_upstream_observable_quality
            ? config_.upstream_min_elevation_deg
            : config_.min_elevation_deg;
    const double min_elevation_rad = effective_min_elevation_deg * M_PI / 180.0;
    const double dd_reference_min_snr_dbhz =
        config_.double_difference_reference_min_snr_dbhz >= 0.0
            ? config_.double_difference_reference_min_snr_dbhz
            : config_.min_snr_dbhz;
    upstream::SnrPercentiles upstream_snr_percentiles;
    std::vector<upstream::EpochMask> upstream_epoch_masks;
    if (config_.use_upstream_observable_quality) {
        upstream_snr_percentiles = upstream::collectSnrPercentiles(
            input_epochs, config_.upstream_snr_percentile);
        problem.diagnostics.upstream_snr_l1_dbhz =
            upstream_snr_percentiles.l1_dbhz;
        problem.diagnostics.upstream_snr_l5_dbhz =
            upstream_snr_percentiles.l5_dbhz;
        upstream::applyAdjacentMasks(
            input_epochs, config_.upstream_device_model, upstream_epoch_masks,
            problem.diagnostics.upstream_pd_pair_rejections,
            problem.diagnostics.upstream_ld_pair_rejections,
            config_.upstream_max_adjacent_gap_s);
    }
    const auto finiteMedian = [](std::vector<double> values) {
        values.erase(std::remove_if(values.begin(), values.end(),
                                    [](double value) {
                                        return !std::isfinite(value);
                                    }),
                      values.end());
        if (values.empty()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        std::sort(values.begin(), values.end());
        const std::size_t middle = values.size() / 2U;
        return (values.size() & 1U) != 0U
                   ? values[middle]
                   : 0.5 * (values[middle - 1U] + values[middle]);
    };
    std::vector<std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>>
        carrier_by_problem_epoch;
    std::vector<std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>>
        dd_pseudorange_by_problem_epoch;
    std::vector<std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>>
        dd_reference_carrier_by_problem_epoch;
    std::map<SatelliteId, double> previous_gps_pseudorange_by_satellite;

    // Seed continuity across a brief SPP outage (see FGOConfig::use_spp_seed):
    // a tunnel/underpass/urban-canyon gap can drop the rover to near-zero
    // tracked satellites for several seconds. The RINEX header's static
    // approximate position can be kilometres from the true kinematic position
    // at that point in the trajectory; seeding elevation/geometry from it
    // spuriously fails the elevation mask for whatever few satellites ARE
    // still tracked during recovery, rejecting far more marginal epochs than
    // the min_satellites_per_epoch gate actually requires. Coast on the last
    // valid SPP fix instead -- at the rover's epoch-to-epoch cadence this
    // stays far closer to truth through a brief outage than the static
    // fallback, and degrades no worse than the static fallback once the
    // outage is long enough that both are stale.
    Vector3d last_valid_seed_position_ecef = Vector3d::Zero();
    double last_valid_seed_clock_bias_m = 0.0;
    bool have_last_valid_seed = false;

    for (std::size_t input_epoch_index = 0;
         input_epoch_index < input_epochs.size(); ++input_epoch_index) {
        const auto& epoch = input_epochs[input_epoch_index];
        const upstream::EpochMask* upstream_mask =
            config_.use_upstream_observable_quality &&
                    input_epoch_index < upstream_epoch_masks.size()
                ? &upstream_epoch_masks[input_epoch_index]
                : nullptr;
        EpochSeed seed;
        seed.time = epoch.time;

        PositionSolution spp_solution = makeInvalidSolution(epoch.time);
        if (config_.use_spp_seed) {
            spp_solution = spp_processor.processEpoch(epoch, nav);
        }

        if (spp_solution.isValid()) {
            seed.position_ecef = spp_solution.position_ecef;
            seed.receiver_clock_bias_m = spp_solution.receiver_clock_bias;
            // SPP's internal/returned clock state is a range bias in metres
            // (see SPPProcessor::calculatePredictedRange), despite the legacy
            // PositionSolution field comment saying seconds.  Mark it
            // explicitly so the opt-in PDC bridge cannot multiply it by c.
            seed.receiver_clock_bias_is_meters = true;
            seed.fresh_spp_solution = true;
            last_valid_seed_position_ecef = seed.position_ecef;
            last_valid_seed_clock_bias_m = seed.receiver_clock_bias_m;
            have_last_valid_seed = true;
        } else if (have_last_valid_seed) {
            seed.position_ecef = last_valid_seed_position_ecef;
            seed.receiver_clock_bias_m = last_valid_seed_clock_bias_m;
            seed.receiver_clock_bias_is_meters = true;
        } else if (epoch.receiver_position.norm() > 1e6) {
            seed.position_ecef = epoch.receiver_position;
            seed.receiver_clock_bias_m = epoch.receiver_clock_bias * constants::SPEED_OF_LIGHT;
            seed.receiver_clock_bias_is_meters = true;
        } else {
            ++problem.diagnostics.skipped_epochs_without_seed;
            continue;
        }

        std::vector<PseudorangeFactor> epoch_factors;
        epoch_factors.reserve(epoch.observations.size());
        std::vector<UndifferencedDopplerFactor> epoch_doppler_factors;
        if (config_.use_undifferenced_doppler_factors) {
            epoch_doppler_factors.reserve(epoch.observations.size());
        }
        std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation> epoch_carriers;
        std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>
            epoch_dd_pseudorange_observations;
        std::map<std::pair<SatelliteId, SignalType>, PreparedCarrierObservation>
            epoch_dd_reference_carriers;
        std::map<SatelliteId, double> gps_pseudorange_by_satellite;

        double receiver_lat = 0.0;
        double receiver_lon = 0.0;
        double receiver_height = 0.0;
        ecef2geodetic(seed.position_ecef, receiver_lat, receiver_lon, receiver_height);

        for (const auto& observation : epoch.observations) {
            if (!isEligibleFgoSignal(observation.satellite,
                                     observation.signal,
                                     config_.use_multi_constellation,
                                     config_.use_multi_frequency_double_difference)) {
                continue;
            }
            if (!passesSecondaryCodeAlignment(observation,
                                              rover_secondary_codes_ptr)) {
                continue;
            }
            if (!observation.valid || !observation.has_pseudorange ||
                observation.pseudorange <= 0.0) {
                continue;
            }
            const std::pair<SatelliteId, SignalType> observation_key{
                observation.satellite, observation.signal};
            const bool upstream_pseudorange_masked =
                upstream_mask != nullptr &&
                upstream_mask->pseudorange.count(observation_key) != 0U;
            const bool upstream_carrier_masked =
                upstream_mask != nullptr &&
                upstream_mask->carrier.count(observation_key) != 0U;
            const upstream::ObservationBand upstream_band =
                upstream::bandForSignal(observation.signal);
            const double upstream_pseudorange_sigma =
                config_.use_upstream_observable_quality
                    ? upstream::snrPercentileSigma(
                          observation.signal, observation.snr,
                          upstream_snr_percentiles, 'P')
                    : std::numeric_limits<double>::quiet_NaN();
            const double upstream_doppler_sigma =
                config_.use_upstream_observable_quality
                    ? upstream::snrPercentileSigma(
                          observation.signal, observation.snr,
                          upstream_snr_percentiles, 'D')
                    : std::numeric_limits<double>::quiet_NaN();
            const bool upstream_snr_ok =
                !config_.use_upstream_observable_quality ||
                (std::isfinite(observation.snr) &&
                 observation.snr >= config_.upstream_min_snr_dbhz &&
                 upstream_band != upstream::ObservationBand::Unknown &&
                 upstream::finitePositive(upstream_pseudorange_sigma) &&
                 upstream::finitePositive(upstream_doppler_sigma));
            const bool passes_snr_mask = config_.use_upstream_observable_quality
                                             ? upstream_snr_ok
                                             : observation.snr >= config_.min_snr_dbhz;
            const bool passes_dd_reference_snr_mask =
                dd_reference_min_snr_dbhz <= 0.0 ||
                observation.snr >= dd_reference_min_snr_dbhz;

            Vector3d satellite_position;
            Vector3d satellite_velocity;
            double satellite_clock_bias = 0.0;
            double satellite_clock_drift = 0.0;

            GNSSTime transmit_time =
                epoch.time - observation.pseudorange / constants::SPEED_OF_LIGHT;
            if (!nav.calculateSatelliteState(observation.satellite,
                                             transmit_time,
                                             satellite_position,
                                             satellite_velocity,
                                             satellite_clock_bias,
                                             satellite_clock_drift)) {
                continue;
            }

            transmit_time = transmit_time - satellite_clock_bias;
            if (!nav.calculateSatelliteState(observation.satellite,
                                             transmit_time,
                                             satellite_position,
                                             satellite_velocity,
                                             satellite_clock_bias,
                                             satellite_clock_drift)) {
                continue;
            }

            const Ephemeris* eph = nav.getEphemeris(observation.satellite, transmit_time);
            if (!eph || !isHealthyForPositioning(observation, *eph)) {
                continue;
            }

            const Vector3d corrected_satellite_position =
                earthRotationCorrected(satellite_position, seed.position_ecef);
            const auto geometry =
                nav.calculateGeometry(seed.position_ecef, corrected_satellite_position);
            const bool passes_elevation_mask =
                geometry.elevation >= min_elevation_rad;

            double ionosphere_delay = 0.0;
            if (config_.use_ionosphere_model && nav.ionosphere_model.valid) {
                ionosphere_delay = models::ionoDelayKlobuchar(
                    receiver_lat,
                    receiver_lon,
                    geometry.azimuth,
                    geometry.elevation,
                    epoch.time.tow,
                    nav.ionosphere_model.alpha,
                    nav.ionosphere_model.beta);

                const double frequency_hz = signalFrequencyHz(observation.signal, eph);
                if (frequency_hz > 0.0) {
                    const double scale = constants::GPS_L1_FREQ / frequency_hz;
                    ionosphere_delay *= scale * scale;
                }
            }

            double troposphere_delay = 0.0;
            if (config_.use_troposphere_model) {
                troposphere_delay =
                    models::tropDelaySaastamoinen(seed.position_ecef, geometry.elevation);
            }

            const double satellite_clock_m =
                satellite_clock_bias * constants::SPEED_OF_LIGHT;
            galileo_group_delay::Selection group_delay_selection;
            const double group_delay_m = groupDelayCorrectionMeters(
                observation, *eph,
                config_.use_signal_specific_galileo_group_delay);
            if (config_.use_signal_specific_galileo_group_delay &&
                observation.satellite.system == GNSSSystem::Galileo &&
                observation.signal == SignalType::GAL_E1) {
                group_delay_selection = galileo_group_delay::select(
                    observation, *eph, true);
                if (!group_delay_selection.valid) {
                    ++problem.diagnostics.galileo_e1_group_delay_invalid_rows;
                    continue;
                }
                if (group_delay_selection.reference ==
                    galileo_group_delay::Reference::FNavE1E5a) {
                    ++problem.diagnostics.galileo_e1_fnav_group_delay_rows;
                } else if (group_delay_selection.reference ==
                           galileo_group_delay::Reference::INavE1E5b) {
                    ++problem.diagnostics.galileo_e1_inav_group_delay_rows;
                } else if (group_delay_selection.reference ==
                           galileo_group_delay::Reference::Ambiguous) {
                    ++problem.diagnostics.galileo_e1_group_delay_source_fallback_rows;
                }
            }
            const double corrected_pseudorange =
                observation.pseudorange +
                satellite_clock_m -
                ionosphere_delay -
                troposphere_delay -
                group_delay_m;

            const double sin_el = std::max(0.1, std::sin(geometry.elevation));
            const double pseudorange_elevation_scale = std::pow(
                sin_el,
                std::max(0.0, config_.pseudorange_elevation_sigma_power));
            if (passes_snr_mask && passes_elevation_mask &&
                !upstream_pseudorange_masked) {
                PseudorangeFactor factor;
                factor.satellite = observation.satellite;
                factor.signal = observation.signal;
                factor.clock_group = clockBiasGroup(observation.satellite.system);
                factor.satellite_position_ecef = corrected_satellite_position;
                factor.corrected_pseudorange_m = corrected_pseudorange;
                factor.sigma_m = config_.use_upstream_observable_quality
                                     ? upstream_pseudorange_sigma
                                     : std::max(
                                           1e-3,
                                           config_.pseudorange_sigma_m /
                                               std::max(1e-6,
                                                        pseudorange_elevation_scale));
                if (!upstream::finitePositive(factor.sigma_m)) {
                    continue;
                }
                factor.snr_dbhz = observation.snr;
                const double seed_range =
                    (corrected_satellite_position - seed.position_ecef).norm();
                factor.upstream_seed_residual_m =
                    corrected_pseudorange - seed_range -
                    seed.receiver_clock_bias_m;
                factor.elevation_rad = geometry.elevation;
                factor.residual_ionosphere_coefficient =
                    residual_ionosphere::signalCoefficient(
                        geometry.elevation,
                        signalFrequencyHz(observation.signal, eph));
                if (config_.use_residual_ionosphere_states) {
                    ++problem.diagnostics.residual_ionosphere_candidate_rows;
                    if (!residual_ionosphere::finiteCoefficient(
                            factor.residual_ionosphere_coefficient)) {
                        ++problem.diagnostics.residual_ionosphere_invalid_coefficients;
                    }
                }
                epoch_factors.push_back(factor);
                if (config_.use_upstream_observable_quality) {
                    ++problem.diagnostics.upstream_pseudorange_factors;
                }
                if (observation.satellite.system == GNSSSystem::GPS) {
                    gps_pseudorange_by_satellite[observation.satellite] =
                        observation.pseudorange;
                }
            }

            const bool carrier_loss_of_lock =
                observation.loss_of_lock || ((observation.lli & 0x01U) != 0);
            double wavelength = signalWavelengthMeters(observation);
            const bool has_carrier_phase =
                observation.has_carrier_phase && observation.carrier_phase != 0.0;
            if (wavelength <= 0.0) {
                wavelength = signalWavelengthMeters(observation.signal, eph);
            }
            const bool usable_carrier = has_carrier_phase && wavelength > 0.0;
            const double corrected_carrier =
                usable_carrier
                    ? observation.carrier_phase * wavelength +
                          satellite_clock_m -
                          troposphere_delay +
                          ionosphere_delay
                    : 0.0;

            FGOProcessor::ObservationModelDebug model_debug;
            model_debug.raw_pseudorange_m = observation.pseudorange;
            model_debug.raw_carrier_m =
                usable_carrier ? observation.carrier_phase * wavelength : 0.0;
            model_debug.satellite_clock_m = satellite_clock_m;
            model_debug.ionosphere_delay_m = ionosphere_delay;
            model_debug.troposphere_delay_m = troposphere_delay;
            model_debug.group_delay_m = group_delay_m;
            model_debug.corrected_pseudorange_m = corrected_pseudorange;
            model_debug.corrected_carrier_m = corrected_carrier;
            const Vector3d corrected_delta =
                corrected_satellite_position - seed.position_ecef;
            const double corrected_range = corrected_delta.norm();
            if (corrected_range <= 0.0) {
                continue;
            }
            model_debug.geometric_range_m = corrected_range;
            model_debug.elevation_rad = geometry.elevation;
            model_debug.azimuth_rad = geometry.azimuth;
            model_debug.snr_dbhz = observation.snr;

            PreparedCarrierObservation carrier;
            carrier.satellite = observation.satellite;
            carrier.signal = observation.signal;
            carrier.satellite_position_ecef = corrected_satellite_position;
            carrier.corrected_pseudorange_m = corrected_pseudorange;
            carrier.corrected_carrier_m = corrected_carrier;
            carrier.wavelength_m = wavelength;
            carrier.sigma_m =
                std::max(1e-4, config_.carrier_phase_sigma_m / sin_el);
            carrier.elevation_rad = geometry.elevation;
            carrier.loss_of_lock = carrier_loss_of_lock;
            carrier.has_carrier_phase = usable_carrier;
            carrier.los = -corrected_delta / corrected_range;
            carrier.doppler_sigma_mps =
                std::max(1e-4,
                         config_.single_difference_doppler_sigma_mps /
                             std::sqrt(sin_el));
            if (observation.has_doppler && wavelength > 0.0) {
                Vector3d doppler_los = Vector3d::Zero();
                double modeled_range_rate = 0.0;
                bool doppler_geometry_valid = false;
                if (config_.use_corrected_undifferenced_doppler_factors) {
                    // Android reports an uncorrected pseudorange rate.  Use
                    // the same transmit-time Earth-rotation correction for
                    // satellite position and velocity; the receiver clock
                    // drift is added later as a difference of graph clock
                    // bias states (see fgo.cpp).
                    doppler_geometry_valid =
                        doppler_contract::knownSatelliteRangeRate(
                            satellite_position, satellite_velocity,
                            seed.position_ecef, true, doppler_los,
                            modeled_range_rate);
                } else {
                    const Vector3d doppler_delta =
                        satellite_position - seed.position_ecef;
                    const double doppler_range = doppler_delta.norm();
                    if (doppler_range > 0.0 && doppler_delta.allFinite()) {
                        doppler_los = doppler_delta / doppler_range;
                        const double sagnac_rate =
                            constants::OMEGA_E / constants::SPEED_OF_LIGHT *
                            (satellite_velocity(1) * seed.position_ecef(0) -
                             satellite_velocity(0) * seed.position_ecef(1));
                        modeled_range_rate =
                            satellite_velocity.dot(doppler_los) + sagnac_rate;
                        doppler_geometry_valid =
                            std::isfinite(modeled_range_rate);
                    }
                }
                if (doppler_geometry_valid) {
                    const double satellite_clock_drift_mps =
                        satellite_clock_drift * constants::SPEED_OF_LIGHT;
                    const double measured_range_rate =
                        doppler_contract::rinexDopplerToRangeRate(
                            observation.doppler,
                            constants::SPEED_OF_LIGHT / wavelength);
                    carrier.doppler_residual_mps =
                        doppler_contract::receiverOnlyResidual(
                            measured_range_rate, modeled_range_rate,
                            satellite_clock_drift);
                    carrier.has_doppler_residual =
                        std::isfinite(carrier.doppler_residual_mps) &&
                        std::isfinite(measured_range_rate);
                    model_debug.has_doppler_residual =
                        carrier.has_doppler_residual;
                    model_debug.doppler_residual_mps =
                        carrier.doppler_residual_mps;
                    model_debug.doppler_measured_range_rate_mps =
                        measured_range_rate;
                    model_debug.doppler_satellite_range_rate_mps =
                        modeled_range_rate;
                    model_debug.doppler_satellite_clock_drift_mps =
                        satellite_clock_drift_mps;
                    model_debug.doppler_uses_rotated_satellite_state =
                        config_.use_corrected_undifferenced_doppler_factors;
                    if (config_.use_corrected_undifferenced_doppler_factors) {
                        // Keep all receiver-only factors on the corrected
                        // state geometry.  The legacy branch intentionally
                        // retains carrier.los from the existing PR geometry.
                        carrier.los = -doppler_los;
                    }
                }
            }
            carrier.model_debug = model_debug;
            if (config_.use_undifferenced_doppler_factors &&
                carrier.has_doppler_residual && passes_snr_mask &&
                passes_elevation_mask) {
                UndifferencedDopplerFactor factor;
                factor.satellite = observation.satellite;
                factor.signal = observation.signal;
                factor.los = carrier.los;
                factor.residual_mps = carrier.doppler_residual_mps;
                factor.sigma_mps = config_.use_upstream_observable_quality
                                       ? upstream_doppler_sigma
                                       : std::max(
                                             1e-4,
                                             config_.undifferenced_doppler_sigma_mps /
                                                 std::sqrt(sin_el));
                if (!upstream::finitePositive(factor.sigma_mps)) {
                    continue;
                }
                factor.elevation_rad = carrier.elevation_rad;
                factor.measured_range_rate_mps =
                    model_debug.doppler_measured_range_rate_mps;
                factor.satellite_range_rate_mps =
                    model_debug.doppler_satellite_range_rate_mps;
                factor.satellite_clock_drift_mps =
                    model_debug.doppler_satellite_clock_drift_mps;
                factor.uses_rotated_satellite_state =
                    model_debug.doppler_uses_rotated_satellite_state;
                epoch_doppler_factors.push_back(factor);
            }
            if (passes_dd_reference_snr_mask && !upstream_carrier_masked) {
                epoch_dd_reference_carriers[
                    {observation.satellite, observation.signal}] = carrier;
            }
            if (passes_snr_mask && passes_elevation_mask &&
                !upstream_pseudorange_masked) {
                epoch_dd_pseudorange_observations[
                    {observation.satellite, observation.signal}] = carrier;
            }
            if (usable_carrier && passes_snr_mask && passes_elevation_mask &&
                !upstream_carrier_masked) {
                if (!(config_.reject_rover_carrier_loss_of_lock &&
                      carrier_loss_of_lock)) {
                    epoch_carriers[{observation.satellite, observation.signal}] =
                        carrier;
                }
            }
        }

        if (config_.use_upstream_observable_quality) {
            // exobs_residuals.m removes Doppler rows whose modeled residual
            // differs from the epoch median by more than 3 m/s.  Apply this
            // only to the receiver-only D factors; TDCP keeps the frozen
            // Phase12 0.03 m contract and is screened by its own arc rules.
            problem.diagnostics.upstream_doppler_candidates +=
                epoch_doppler_factors.size();
            const std::vector<double> residuals = [&]() {
                std::vector<double> values;
                values.reserve(epoch_doppler_factors.size());
                for (const auto& factor : epoch_doppler_factors) {
                    values.push_back(factor.residual_mps);
                }
                return values;
            }();
            const double median = finiteMedian(residuals);
            std::vector<UndifferencedDopplerFactor> filtered_doppler;
            filtered_doppler.reserve(epoch_doppler_factors.size());
            for (const auto& factor : epoch_doppler_factors) {
                const double threshold = upstream::residualThreshold(
                    upstream::bandForSignal(factor.signal), 'D');
                if (std::isfinite(median) && std::isfinite(threshold) &&
                    std::isfinite(factor.residual_mps) &&
                    std::abs(factor.residual_mps - median) <= threshold) {
                    filtered_doppler.push_back(factor);
                } else {
                    ++problem.diagnostics.upstream_doppler_residual_rejections;
                }
            }
            epoch_doppler_factors.swap(filtered_doppler);
            problem.diagnostics.upstream_doppler_factors +=
                epoch_doppler_factors.size();
        }

        const std::size_t usable_epoch_measurements =
            config_.use_pseudorange_factors ? epoch_factors.size()
                                            : epoch_carriers.size();
        if (static_cast<int>(usable_epoch_measurements) <
            config_.min_satellites_per_epoch) {
            if (!config_.retain_sparse_epochs_for_imu) {
                continue;
            }
            ++problem.diagnostics.sparse_epochs_retained;
            if (usable_epoch_measurements == 0) {
                ++problem.diagnostics.sparse_empty_epochs_retained;
            }
        }

        const std::size_t epoch_index = problem.epochs.size();
        problem.epochs.push_back(seed);
        bool clock_jump = false;
        double gps_pseudorange_delta_sum = 0.0;
        std::size_t gps_pseudorange_delta_count = 0;
        for (const auto& [satellite, pseudorange] : gps_pseudorange_by_satellite) {
            const auto previous_it =
                previous_gps_pseudorange_by_satellite.find(satellite);
            if (previous_it == previous_gps_pseudorange_by_satellite.end()) {
                continue;
            }
            gps_pseudorange_delta_sum += pseudorange - previous_it->second;
            ++gps_pseudorange_delta_count;
        }
        if (gps_pseudorange_delta_count > 0) {
            const double mean_delta =
                gps_pseudorange_delta_sum /
                static_cast<double>(gps_pseudorange_delta_count);
            clock_jump = mean_delta > 1e5;
            problem.gps_common_pseudorange_delta_m.push_back(mean_delta);
        } else {
            problem.gps_common_pseudorange_delta_m.push_back(0.0);
        }
        problem.gps_common_pseudorange_delta_satellites.push_back(
            static_cast<int>(gps_pseudorange_delta_count));
        problem.clock_jumps.push_back(clock_jump);
        previous_gps_pseudorange_by_satellite =
            std::move(gps_pseudorange_by_satellite);
        problem.diagnostics.seeded_epochs = problem.epochs.size();
        carrier_by_problem_epoch.push_back(std::move(epoch_carriers));
        dd_pseudorange_by_problem_epoch.push_back(
            std::move(epoch_dd_pseudorange_observations));
        dd_reference_carrier_by_problem_epoch.push_back(
            std::move(epoch_dd_reference_carriers));
        if (config_.use_pseudorange_factors) {
            for (auto& factor : epoch_factors) {
                factor.epoch_index = epoch_index;
                problem.pseudorange_factors.push_back(factor);
            }
        }
        for (auto& factor : epoch_doppler_factors) {
            factor.epoch_index = epoch_index;
            if (config_.use_corrected_undifferenced_doppler_factors) {
                if (epoch_index == 0) {
                    // A single epoch cannot identify receiver clock drift;
                    // do not silently absorb it into velocity.
                    continue;
                }
                factor.previous_epoch_index = epoch_index - 1;
                factor.dt_s = problem.epochs[epoch_index].time -
                              problem.epochs[epoch_index - 1].time;
                if (!(factor.dt_s > 0.0) ||
                    (config_.max_tdcp_gap_s > 0.0 &&
                     factor.dt_s > config_.max_tdcp_gap_s)) {
                    continue;
                }
                factor.includes_receiver_clock_drift = true;
            }
            problem.undifferenced_doppler_factors.push_back(factor);
        }
    }

    if (config_.use_upstream_observable_quality) {
        // The published residual mask uses one median per system/frequency
        // over the whole observation matrix.  Reconstruct the same
        // truth-free center from native SPP-seed residuals; no enriched
        // receiver/satellite coordinate is consulted.
        using ResidualGroup =
            std::pair<GNSSSystem, upstream::ObservationBand>;
        std::map<ResidualGroup, std::vector<double>> residuals_by_group;
        for (const auto& factor : problem.pseudorange_factors) {
            const ResidualGroup group{
                factor.clock_group, upstream::bandForSignal(factor.signal)};
            if (group.second == upstream::ObservationBand::Unknown ||
                !std::isfinite(factor.upstream_seed_residual_m)) {
                continue;
            }
            residuals_by_group[group].push_back(factor.upstream_seed_residual_m);
        }
        std::map<ResidualGroup, double> residual_medians;
        for (const auto& [group, values] : residuals_by_group) {
            residual_medians[group] = finiteMedian(values);
        }
        problem.diagnostics.upstream_pseudorange_candidates =
            problem.pseudorange_factors.size();
        std::vector<PseudorangeFactor> filtered_pseudorange;
        filtered_pseudorange.reserve(problem.pseudorange_factors.size());
        for (const auto& factor : problem.pseudorange_factors) {
            const ResidualGroup group{
                factor.clock_group, upstream::bandForSignal(factor.signal)};
            const auto median_it = residual_medians.find(group);
            const double median = median_it == residual_medians.end()
                                      ? std::numeric_limits<double>::quiet_NaN()
                                      : median_it->second;
            const double threshold = upstream::residualThreshold(group.second, 'P');
            if (std::isfinite(median) && std::isfinite(threshold) &&
                std::isfinite(factor.upstream_seed_residual_m) &&
                std::abs(factor.upstream_seed_residual_m - median) <= threshold) {
                filtered_pseudorange.push_back(factor);
            } else {
                ++problem.diagnostics.upstream_pseudorange_residual_rejections;
            }
        }
        problem.pseudorange_factors.swap(filtered_pseudorange);
        problem.diagnostics.upstream_pseudorange_factors =
            problem.pseudorange_factors.size();
    }

    if (config_.use_doppler_velocity_wls_initialization) {
        // Build the initializer only from the corrected receiver-only rows.
        // The grouping is deliberately performed after factor construction so
        // the WLS and graph consume the exact same LOS, known-satellite term,
        // and uncertainty values.
        problem.doppler_velocity_wls_estimates.resize(problem.epochs.size());
        doppler_velocity_wls::Options wls_options;
        wls_options.min_rows = config_.doppler_velocity_wls_min_rows;
        wls_options.max_condition_number =
            config_.doppler_velocity_wls_max_condition_number;
        wls_options.huber_threshold_sigma =
            config_.doppler_velocity_wls_huber_threshold_sigma;
        wls_options.max_irls_iterations =
            config_.doppler_velocity_wls_max_irls_iterations;
        wls_options.max_velocity_mps =
            config_.doppler_velocity_wls_max_velocity_mps;
        wls_options.max_clock_rate_mps =
            config_.doppler_velocity_wls_max_clock_rate_mps;
        wls_options.max_normalized_rms =
            config_.doppler_velocity_wls_max_normalized_rms;
        wls_options.max_abs_normalized_residual =
            config_.doppler_velocity_wls_max_abs_normalized_residual;

        std::vector<std::vector<doppler_velocity_wls::ObservationRow>> rows_by_epoch(
            problem.epochs.size());
        for (const auto& factor : problem.undifferenced_doppler_factors) {
            if (!factor.includes_receiver_clock_drift ||
                factor.epoch_index >= rows_by_epoch.size()) {
                continue;
            }
            rows_by_epoch[factor.epoch_index].push_back({
                factor.los, factor.residual_mps, factor.sigma_mps});
        }
        for (std::size_t epoch_index = 0;
             epoch_index < problem.epochs.size(); ++epoch_index) {
            auto& estimate = problem.doppler_velocity_wls_estimates[epoch_index];
            if (epoch_index < problem.clock_jumps.size() &&
                problem.clock_jumps[epoch_index]) {
                estimate.reset = true;
                estimate.reason = "clock-discontinuity-reset";
                continue;
            }
            estimate = doppler_velocity_wls::solve(rows_by_epoch[epoch_index],
                                                   wls_options);
        }

        // A missing row set is never replaced with zero.  Complete only short
        // same-lane gaps using already-valid estimates: first try a bounded
        // two-sided interpolation, then a one-sided constant hold.  A clock
        // jump or a gap beyond max_tdcp_gap_s blocks both operations.
        std::vector<GNSSTime> times;
        times.reserve(problem.epochs.size());
        for (const auto& epoch : problem.epochs) {
            times.push_back(epoch.time);
        }
        const double max_gap = std::max(0.0, config_.max_tdcp_gap_s);
        const double edge_hold =
            std::max(0.0, config_.doppler_velocity_wls_edge_hold_max_s);
        auto has_clock_jump_between = [&](std::size_t first,
                                          std::size_t last) {
            if (first > last) {
                std::swap(first, last);
            }
            for (std::size_t i = first + 1; i <= last &&
                                             i < problem.clock_jumps.size();
                 ++i) {
                if (problem.clock_jumps[i]) {
                    return true;
                }
            }
            return false;
        };
        for (std::size_t i = 0; i < problem.doppler_velocity_wls_estimates.size();
             ++i) {
            auto& estimate = problem.doppler_velocity_wls_estimates[i];
            if (estimate.valid) {
                continue;
            }
            std::size_t previous = i;
            while (previous > 0) {
                --previous;
                if (problem.doppler_velocity_wls_estimates[previous].valid) {
                    break;
                }
            }
            const bool have_previous =
                problem.doppler_velocity_wls_estimates[previous].valid;
            std::size_t next = i;
            while (next + 1 < problem.doppler_velocity_wls_estimates.size()) {
                ++next;
                if (problem.doppler_velocity_wls_estimates[next].valid) {
                    break;
                }
            }
            const bool have_next =
                problem.doppler_velocity_wls_estimates[next].valid;

            const double previous_dt =
                have_previous ? times[i] - times[previous] : 0.0;
            const double next_dt = have_next ? times[next] - times[i] : 0.0;
            if (have_previous && have_next && previous < i && i < next &&
                previous_dt > 0.0 && next_dt > 0.0 &&
                (max_gap <= 0.0 ||
                 (previous_dt <= max_gap && next_dt <= max_gap)) &&
                !has_clock_jump_between(previous, next)) {
                const auto& before =
                    problem.doppler_velocity_wls_estimates[previous];
                const auto& after = problem.doppler_velocity_wls_estimates[next];
                const double alpha = previous_dt / (previous_dt + next_dt);
                estimate = before;
                estimate.direct = false;
                estimate.propagated = true;
                estimate.reason = "bounded-interpolation";
                estimate.velocity_ecef_mps =
                    (1.0 - alpha) * before.velocity_ecef_mps +
                    alpha * after.velocity_ecef_mps;
                estimate.clock_rate_mps =
                    (1.0 - alpha) * before.clock_rate_mps +
                    alpha * after.clock_rate_mps;
                estimate.covariance =
                    (1.0 - alpha) * before.covariance + alpha * after.covariance;
                continue;
            }
            if (have_previous && previous_dt > 0.0 && previous_dt <= edge_hold &&
                (max_gap <= 0.0 || previous_dt <= max_gap) &&
                !has_clock_jump_between(previous, i)) {
                const auto& before =
                    problem.doppler_velocity_wls_estimates[previous];
                estimate = before;
                estimate.direct = false;
                estimate.propagated = true;
                estimate.reason = "bounded-edge-hold";
                continue;
            }
            if (have_next && next_dt > 0.0 && next_dt <= edge_hold &&
                (max_gap <= 0.0 || next_dt <= max_gap) &&
                !has_clock_jump_between(i, next)) {
                const auto& after = problem.doppler_velocity_wls_estimates[next];
                estimate = after;
                estimate.direct = false;
                estimate.propagated = true;
                estimate.reason = "bounded-edge-hold";
            }
        }
    }

    if ((config_.use_carrier_phase_factors || config_.use_double_difference_factors) &&
        !carrier_by_problem_epoch.empty()) {
        using CarrierKey = std::pair<SatelliteId, SignalType>;
        std::map<CarrierKey, ActiveCarrierSegment> active_segments;
        std::map<CarrierKey, std::size_t> next_segment_indices;
        const double max_gap = std::max(0.0, config_.max_tdcp_gap_s);

        for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size(); ++epoch_index) {
            const auto& current_carriers = carrier_by_problem_epoch[epoch_index];
            for (const auto& [key, carrier] : current_carriers) {
                auto active_it = active_segments.find(key);
                bool start_new_segment = active_it == active_segments.end();
                if (active_it != active_segments.end()) {
                    const double dt = problem.epochs[epoch_index].time -
                                      problem.epochs[active_it->second.last_epoch_index].time;
                    if (dt <= 0.0 || (max_gap > 0.0 && dt > max_gap) ||
                        carrier.loss_of_lock) {
                        start_new_segment = true;
                    }
                }

                if (start_new_segment) {
                    const std::size_t segment_index = next_segment_indices[key]++;
                    std::size_t carrier_segment_index = segment_index;
                    if (config_.use_carrier_phase_factors) {
                        AmbiguityState ambiguity;
                        ambiguity.satellite = carrier.satellite;
                        ambiguity.signal = carrier.signal;
                        ambiguity.segment_index = segment_index;
                        ambiguity.wavelength_m = carrier.wavelength_m;
                        ambiguity.initial_ambiguity_m =
                            carrier.corrected_carrier_m -
                            carrier.corrected_pseudorange_m;
                        carrier_segment_index = problem.ambiguity_states.size();
                        problem.ambiguity_states.push_back(ambiguity);
                    }
                    active_it = active_segments
                                    .insert_or_assign(
                                        key,
                                        ActiveCarrierSegment{carrier_segment_index,
                                                             epoch_index})
                                    .first;
                } else {
                    active_it->second.last_epoch_index = epoch_index;
                }

                CarrierPhaseFactor carrier_observation;
                carrier_observation.epoch_index = epoch_index;
                carrier_observation.ambiguity_index = active_it->second.ambiguity_index;
                carrier_observation.satellite = carrier.satellite;
                carrier_observation.clock_group =
                    clockBiasGroup(carrier.satellite.system);
                carrier_observation.signal = carrier.signal;
                carrier_observation.satellite_position_ecef =
                    carrier.satellite_position_ecef;
                carrier_observation.corrected_pseudorange_m =
                    carrier.corrected_pseudorange_m;
                carrier_observation.corrected_carrier_m =
                    carrier.corrected_carrier_m;
                carrier_observation.wavelength_m = carrier.wavelength_m;
                carrier_observation.sigma_m = carrier.sigma_m;
                carrier_observation.elevation_rad = carrier.elevation_rad;
                carrier_observation.has_carrier_phase = carrier.has_carrier_phase;
                carrier_observation.loss_of_lock = carrier.loss_of_lock;
                carrier_observation.has_doppler_residual =
                    carrier.has_doppler_residual;
                carrier_observation.doppler_residual_mps =
                    carrier.doppler_residual_mps;
                carrier_observation.doppler_sigma_mps =
                    carrier.doppler_sigma_mps;
                carrier_observation.los = carrier.los;
                carrier_observation.model_debug = carrier.model_debug;
                problem.carrier_observations.push_back(carrier_observation);
                if (config_.use_carrier_phase_factors) {
                    problem.carrier_phase_factors.push_back(carrier_observation);
                }
            }
        }
    }

    if (config_.use_double_difference_factors) {
        for (std::size_t epoch_index = 0;
             epoch_index < dd_pseudorange_by_problem_epoch.size();
             ++epoch_index) {
            for (const auto& [key, carrier] :
                 dd_pseudorange_by_problem_epoch[epoch_index]) {
                CarrierPhaseFactor pseudorange_observation;
                pseudorange_observation.epoch_index = epoch_index;
                pseudorange_observation.satellite = carrier.satellite;
                pseudorange_observation.clock_group =
                    clockBiasGroup(carrier.satellite.system);
                pseudorange_observation.signal = carrier.signal;
                pseudorange_observation.satellite_position_ecef =
                    carrier.satellite_position_ecef;
                pseudorange_observation.corrected_pseudorange_m =
                    carrier.corrected_pseudorange_m;
                pseudorange_observation.corrected_carrier_m =
                    carrier.corrected_carrier_m;
                pseudorange_observation.wavelength_m = carrier.wavelength_m;
                pseudorange_observation.sigma_m = carrier.sigma_m;
                pseudorange_observation.elevation_rad = carrier.elevation_rad;
                pseudorange_observation.has_carrier_phase =
                    carrier.has_carrier_phase;
                pseudorange_observation.loss_of_lock = carrier.loss_of_lock;
                pseudorange_observation.has_doppler_residual =
                    carrier.has_doppler_residual;
                pseudorange_observation.doppler_residual_mps =
                    carrier.doppler_residual_mps;
                pseudorange_observation.doppler_sigma_mps =
                    carrier.doppler_sigma_mps;
                pseudorange_observation.los = carrier.los;
                pseudorange_observation.model_debug = carrier.model_debug;
                problem.double_difference_pseudorange_observations.push_back(
                    pseudorange_observation);
            }
        }
    }

    if (config_.use_tdcp_factors && problem.epochs.size() >= 2) {
        const double sigma = std::max(1e-4, config_.tdcp_sigma_m);
        const double max_gap = std::max(0.0, config_.max_tdcp_gap_s);
        for (std::size_t epoch_index = 1; epoch_index < problem.epochs.size(); ++epoch_index) {
            const double dt = problem.epochs[epoch_index].time -
                              problem.epochs[epoch_index - 1].time;
            if (dt <= 0.0 || (max_gap > 0.0 && dt > max_gap)) {
                problem.diagnostics.tdcp_rejected_gap +=
                    carrier_by_problem_epoch[epoch_index].size();
                continue;
            }

            const auto& previous_carriers = carrier_by_problem_epoch[epoch_index - 1];
            const auto& current_carriers = carrier_by_problem_epoch[epoch_index];
            for (const auto& [key, current] : current_carriers) {
                const auto previous_it = previous_carriers.find(key);
                if (previous_it == previous_carriers.end()) {
                    ++problem.diagnostics.tdcp_rejected_missing_previous;
                    continue;
                }
                const auto& previous = previous_it->second;
                ++problem.diagnostics.tdcp_candidate_pairs;
                const double delta_carrier_m =
                    current.corrected_carrier_m - previous.corrected_carrier_m;
                const double delta_code_m =
                    current.corrected_pseudorange_m - previous.corrected_pseudorange_m;
                const auto decision = tdcp_contract::evaluateAdjacentPair(
                    dt, previous.loss_of_lock, current.loss_of_lock,
                    delta_carrier_m, delta_code_m, max_gap,
                    config_.reject_tdcp_loss_of_lock,
                    config_.reject_tdcp_code_phase_jump,
                    config_.tdcp_code_phase_jump_threshold_m,
                    epoch_index > 0 && epoch_index - 1 < problem.clock_jumps.size() &&
                        problem.clock_jumps[epoch_index - 1],
                    epoch_index < problem.clock_jumps.size() &&
                        problem.clock_jumps[epoch_index]);
                switch (decision.reason) {
                    case tdcp_contract::PairRejectReason::Accepted:
                        break;
                    case tdcp_contract::PairRejectReason::Gap:
                        ++problem.diagnostics.tdcp_rejected_gap;
                        continue;
                    case tdcp_contract::PairRejectReason::ClockDiscontinuity:
                        ++problem.diagnostics.tdcp_rejected_clock_discontinuity;
                        continue;
                    case tdcp_contract::PairRejectReason::LossOfLock:
                        ++problem.diagnostics.tdcp_rejected_loss_of_lock;
                        continue;
                    case tdcp_contract::PairRejectReason::NonFiniteMeasurement:
                        ++problem.diagnostics.tdcp_rejected_invalid_measurement;
                        continue;
                    case tdcp_contract::PairRejectReason::CodePhaseJump:
                        ++problem.diagnostics.tdcp_rejected_code_phase_jump;
                        continue;
                }

                TimeDifferencedCarrierFactor factor;
                factor.previous_epoch_index = epoch_index - 1;
                factor.current_epoch_index = epoch_index;
                factor.satellite = current.satellite;
                factor.signal = current.signal;
                factor.previous_satellite_position_ecef = previous.satellite_position_ecef;
                factor.current_satellite_position_ecef = current.satellite_position_ecef;
                factor.delta_carrier_m = delta_carrier_m;
                factor.sigma_m = sigma;
                factor.dt_s = dt;
                problem.tdcp_factors.push_back(factor);
            }
        }
    }

    if (config_.use_double_difference_factors) {
        for (std::size_t epoch_index = 0;
             epoch_index < dd_reference_carrier_by_problem_epoch.size();
             ++epoch_index) {
            for (const auto& [key, carrier] :
                 dd_reference_carrier_by_problem_epoch[epoch_index]) {
                CarrierPhaseFactor reference_observation;
                reference_observation.epoch_index = epoch_index;
                reference_observation.satellite = carrier.satellite;
                reference_observation.clock_group =
                    clockBiasGroup(carrier.satellite.system);
                reference_observation.signal = carrier.signal;
                reference_observation.satellite_position_ecef =
                    carrier.satellite_position_ecef;
                reference_observation.corrected_pseudorange_m =
                    carrier.corrected_pseudorange_m;
                reference_observation.corrected_carrier_m =
                    carrier.corrected_carrier_m;
                reference_observation.wavelength_m = carrier.wavelength_m;
                reference_observation.sigma_m = carrier.sigma_m;
                reference_observation.elevation_rad = carrier.elevation_rad;
                reference_observation.has_carrier_phase = carrier.has_carrier_phase;
                reference_observation.loss_of_lock = carrier.loss_of_lock;
                reference_observation.has_doppler_residual =
                    carrier.has_doppler_residual;
                reference_observation.doppler_residual_mps =
                    carrier.doppler_residual_mps;
                reference_observation.doppler_sigma_mps =
                    carrier.doppler_sigma_mps;
                reference_observation.los = carrier.los;
                reference_observation.model_debug = carrier.model_debug;
                problem.double_difference_reference_observations.push_back(
                    reference_observation);
            }
        }
    }

    return problem;
}

FGOProcessor::FGOProblem FGOProcessor::buildDoubleDifferenceProblem(
    const std::vector<ObservationData>& rover_epochs,
    const std::vector<ObservationData>& base_epochs,
    const NavigationData& nav,
    const Vector3d& base_position_ecef) const {
    FGOProblem problem = buildPseudorangeProblem(rover_epochs, nav);
    if (!config_.use_double_difference_factors ||
        problem.carrier_observations.empty() ||
        base_epochs.empty() ||
        base_position_ecef.norm() <= 1e6) {
        return problem;
    }

    std::map<std::size_t, std::vector<const CarrierPhaseFactor*>>
        rover_carriers_by_epoch;
    for (const auto& factor : problem.carrier_observations) {
        if (factor.epoch_index < problem.epochs.size()) {
            rover_carriers_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }
    std::map<std::size_t, std::vector<const CarrierPhaseFactor*>>
        rover_pseudoranges_by_epoch;
    for (const auto& factor :
         problem.double_difference_pseudorange_observations) {
        if (factor.epoch_index < problem.epochs.size()) {
            rover_pseudoranges_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }
    std::map<std::size_t, std::vector<const CarrierPhaseFactor*>>
        rover_reference_carriers_by_epoch;
    for (const auto& factor : problem.double_difference_reference_observations) {
        if (factor.epoch_index < problem.epochs.size()) {
            rover_reference_carriers_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }

    // MF hygiene: the base receiver gets its own per-(system, secondary band)
    // code table, mirroring the rover-side table in buildPseudorangeProblem.
    SecondaryCodeTable base_secondary_codes;
    const SecondaryCodeTable* base_secondary_codes_ptr = nullptr;
    if (config_.use_multi_frequency_double_difference &&
        config_.use_double_difference_secondary_code_alignment &&
        config_.use_multi_constellation) {
        base_secondary_codes = buildSecondaryCodeTable(base_epochs);
        base_secondary_codes_ptr = &base_secondary_codes;
    }

    std::size_t base_cursor = 0;
    const double match_tolerance = std::max(0.0, config_.base_epoch_match_tolerance_s);
    const double pseudorange_sigma =
        std::max(1e-3, config_.double_difference_pseudorange_sigma_m);
    const double carrier_sigma =
        std::max(1e-4, config_.double_difference_carrier_sigma_m);
    // Elevation-dependent DD sigma ("varerr", FGOConfig::use_elevation_
    // dependent_sigma): el_min_rad is the pair-elevation floor (reference:
    // np.radians(max(1.0, cfg.el_mask_deg))); epoch_dt_s is the rover
    // epoch-to-epoch interval fed to the clock-stability term, persisted
    // across the epoch loop below and only updated on a positive delta --
    // mirroring the reference's tc._epoch_dt / _update_epoch_dt exactly
    // (state carried across epochs, default 0.2 s until the first update;
    // see the knob's doc comment in fgo.hpp for why 0.2 s is also the right
    // fallback for this dataset).
    const double el_min_rad = M_PI / 180.0 * std::max(1.0, config_.min_elevation_deg);
    double epoch_dt_s = 0.2;
    const double max_segment_gap = std::max(0.0, config_.max_tdcp_gap_s);
    std::map<DoubleDifferenceAmbiguityKey, ActiveCarrierSegment>
        active_dd_segments;
    std::map<DoubleDifferenceSegmentKey, std::size_t> next_dd_segment_indices;
    using DdFactorKey =
        std::tuple<std::size_t, SatelliteId, SatelliteId, SignalType>;
    std::map<SingleDifferenceReferenceKey, SatelliteId> sd_reference_by_group;
    std::map<SingleDifferenceDefaultReferenceKey, std::map<SatelliteId, std::size_t>>
        sd_default_reference_counts;

    // --- CMC screening state (function-scope, persists across the epoch
    // loop below; see FGOConfig::use_code_minus_carrier_screening). ---
    std::map<CarrierKey, CmcState> cmc_states;
    // Last rover-side (single-receiver) ambiguity_index seen per (satellite,
    // signal); a change here (cycle slip / loss-of-lock / outage causing the
    // rover carrier arc in buildPseudorangeProblem to restart) is this port's
    // signal to also reset the CMC baseline for that key (deviation from the
    // reference documented on the config knob).
    std::map<CarrierKey, std::size_t> cmc_last_rover_ambiguity_index;
    std::size_t cmc_jump_reset_total = 0;
    using GeometryFreePairKey =
        std::tuple<SatelliteId, SignalType, SignalType>;
    std::map<GeometryFreePairKey, GeometryFreeSlipState>
        geometry_free_slip_states;
    std::map<CarrierKey, GeometryFreePendingReset>
        geometry_free_pending_resets;
    std::size_t geometry_free_slip_reset_total = 0;
    std::size_t cmc_level_exclusion_total = 0;
    // FGOConfig::cmc_aware_reference_selection: count of DD reference
    // picks steered away from a CMC-level-excluded candidate this run.
    std::size_t cmc_ref_avoided_total = 0;

    for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size(); ++epoch_index) {
        // Reference: tc._update_epoch_dt(obs) runs unconditionally at the top
        // of every epoch's processing, before any DD-eligibility gates below
        // -- mirrored here the same way (only meaningful when varerr is on).
        if (config_.use_elevation_dependent_sigma && epoch_index > 0) {
            const double dt = problem.epochs[epoch_index].time -
                              problem.epochs[epoch_index - 1].time;
            if (dt > 0.0) {
                epoch_dt_s = dt;
            }
        }
        const auto rover_it = rover_carriers_by_epoch.find(epoch_index);
        const auto rover_pseudorange_it =
            rover_pseudoranges_by_epoch.find(epoch_index);
        const auto rover_reference_it =
            rover_reference_carriers_by_epoch.find(epoch_index);
        if ((rover_it == rover_carriers_by_epoch.end() ||
             rover_it->second.empty()) &&
            (rover_pseudorange_it == rover_pseudoranges_by_epoch.end() ||
             rover_pseudorange_it->second.empty())) {
            continue;
        }

        ObservationData matched_base_epoch;
        bool interpolated_base_epoch = false;
        const bool has_base_epoch =
            findMatchedBaseEpoch(base_epochs,
                                 problem.epochs[epoch_index].time,
                                 base_cursor,
                                 match_tolerance,
                                 config_.base_interpolation_max_gap_s,
                                 matched_base_epoch,
                                 interpolated_base_epoch);
        if (!has_base_epoch) {
            problem.diagnostics.double_difference_rejected_no_base_epoch +=
                rover_it == rover_carriers_by_epoch.end()
                    ? 0
                    : rover_it->second.size();
            continue;
        }
        ++problem.diagnostics.double_difference_matched_base_epochs;
        if (interpolated_base_epoch) {
            ++problem.diagnostics.double_difference_interpolated_base_epochs;
        }

        std::map<std::pair<GNSSSystem, SignalType>, std::vector<const CarrierPhaseFactor*>>
            grouped_rover_pseudoranges;
        std::map<std::pair<GNSSSystem, SignalType>, std::vector<const CarrierPhaseFactor*>>
            grouped_rover_carriers;
        std::map<CarrierKey, const CarrierPhaseFactor*> rover_references_by_key;
        if (rover_reference_it != rover_reference_carriers_by_epoch.end()) {
            for (const auto* reference_factor : rover_reference_it->second) {
                rover_references_by_key[{reference_factor->satellite,
                                          reference_factor->signal}] =
                    reference_factor;
            }
        }
        std::map<std::pair<GNSSSystem, SignalType>,
                 std::vector<const PreparedCarrierObservation*>>
            grouped_reference_observations;
        const double base_min_snr_dbhz =
            config_.double_difference_base_min_snr_dbhz >= 0.0
                ? config_.double_difference_base_min_snr_dbhz
                : config_.min_snr_dbhz;
        const double reference_min_snr_dbhz =
            config_.double_difference_reference_min_snr_dbhz >= 0.0
                ? config_.double_difference_reference_min_snr_dbhz
                : config_.min_snr_dbhz;
        const auto base_carriers =
            prepareCarrierObservationsForReceiver(
                matched_base_epoch,
                nav,
                base_position_ecef,
                config_,
                base_min_snr_dbhz,
                true,
                false,
                base_secondary_codes_ptr);
        const auto base_pseudorange_observations =
            prepareCarrierObservationsForReceiver(
                matched_base_epoch,
                nav,
                base_position_ecef,
                config_,
                base_min_snr_dbhz,
                false,
                false,
                base_secondary_codes_ptr);
        const auto base_reference_observations =
            prepareCarrierObservationsForReceiver(
                matched_base_epoch,
                nav,
                base_position_ecef,
                config_,
                reference_min_snr_dbhz,
                false,
                false,
                base_secondary_codes_ptr);

        // --- CMC screening pre-pass (this epoch) ---
        //
        // Runs once per (satellite, signal) that has usable carrier at BOTH
        // rover and base this epoch (same requirement as the reference:
        // pr/cp nonzero on both sides), independent of which satellite ends
        // up the DD reference for its (system, signal) group below. Results
        // gate the PR-factor loop (level exclusion, which -- because the CP
        // loop only builds a factor when a matching PR factor already exists
        // -- transitively also excludes the CP factor for the same epoch,
        // matching the reference's single `continue` before either) and the
        // CP loop's ambiguity-arc bookkeeping (jump forces a new arc).
        std::set<CarrierKey> cmc_level_exclude_this_epoch;
        std::set<CarrierKey> cmc_jump_reset_this_epoch;
        std::set<CarrierKey> geometry_free_reset_this_epoch;
        if (config_.use_code_minus_carrier_screening &&
            rover_it != rover_carriers_by_epoch.end()) {
            for (const auto* rover_factor : rover_it->second) {
                const CarrierKey key{rover_factor->satellite, rover_factor->signal};
                const auto base_it = base_carriers.find(key);
                if (base_it == base_carriers.end()) {
                    continue;
                }
                const auto& base_carrier = base_it->second;
                const double pr_rover = rover_factor->model_debug.raw_pseudorange_m;
                const double cp_rover = rover_factor->model_debug.raw_carrier_m;
                const double pr_base = base_carrier.model_debug.raw_pseudorange_m;
                const double cp_base = base_carrier.model_debug.raw_carrier_m;
                if (pr_rover == 0.0 || cp_rover == 0.0 || pr_base == 0.0 ||
                    cp_base == 0.0) {
                    continue;
                }
                const double cmc = (pr_rover - pr_base) - (cp_rover - cp_base);

                // Deviation from the reference: any rover-side arc restart
                // (cycle slip / loss-of-lock / outage -- surfaced here as a
                // change in the rover carrier's own ambiguity_index) resets
                // this key's CMC baseline/prev/warmup state, since CMC
                // embeds a -wavelength*N term that a new ambiguity
                // invalidates.
                const auto last_arc_it =
                    cmc_last_rover_ambiguity_index.find(key);
                const bool rover_arc_restarted =
                    last_arc_it == cmc_last_rover_ambiguity_index.end() ||
                    last_arc_it->second != rover_factor->ambiguity_index;
                cmc_last_rover_ambiguity_index[key] = rover_factor->ambiguity_index;

                CmcState& state = cmc_states[key];
                if (rover_arc_restarted) {
                    state = CmcState{};
                } else if (state.has_prev &&
                           std::abs(cmc - state.prev_cmc_m) >
                               config_.code_minus_carrier_jump_threshold_m) {
                    cmc_jump_reset_this_epoch.insert(key);
                    ++cmc_jump_reset_total;
                    // Same reasoning as above: the jump itself is a fresh
                    // arc from this epoch on, so the baseline resets too.
                    state = CmcState{};
                }

                state.prev_cmc_m = cmc;
                state.has_prev = true;

                const double level_threshold =
                    config_.code_minus_carrier_level_threshold_m;
                if (level_threshold > 0.0) {
                    if (!state.has_baseline) {
                        state.baseline_m = cmc;
                        state.warmup_count = 1;
                        state.has_baseline = true;
                    } else if (state.warmup_count <
                               config_.code_minus_carrier_warmup_epochs) {
                        state.baseline_m =
                            (state.baseline_m * state.warmup_count + cmc) /
                            (state.warmup_count + 1);
                        ++state.warmup_count;
                    } else if (std::abs(cmc - state.baseline_m) > level_threshold) {
                        cmc_level_exclude_this_epoch.insert(key);
                        ++cmc_level_exclusion_total;
                    } else {
                        const double alpha = config_.code_minus_carrier_baseline_alpha;
                        state.baseline_m =
                            (1.0 - alpha) * state.baseline_m + alpha * cmc;
                    }
                }
            }
        }

        if (config_.use_geometry_free_cycle_slip_reset &&
            rover_it != rover_carriers_by_epoch.end()) {
            using GeometryFreeSample =
                std::pair<const CarrierPhaseFactor*,
                          const PreparedCarrierObservation*>;
            std::map<SatelliteId, std::map<SignalType, GeometryFreeSample>>
                samples_by_satellite;
            std::map<CarrierKey, GeometryFreeSample> samples_by_key;
            for (const auto* rover_factor : rover_it->second) {
                const CarrierKey key{rover_factor->satellite,
                                     rover_factor->signal};
                const auto base_it = base_carriers.find(key);
                if (base_it != base_carriers.end()) {
                    const GeometryFreeSample sample{rover_factor,
                                                    &base_it->second};
                    samples_by_satellite[rover_factor->satellite]
                                        [rover_factor->signal] = sample;
                    samples_by_key[key] = sample;
                }
            }
            const double confirmation_s = std::max(
                0.0, config_.geometry_free_cycle_slip_confirmation_s);
            for (auto pending = geometry_free_pending_resets.begin();
                 pending != geometry_free_pending_resets.end();) {
                const auto sample = samples_by_key.find(pending->first);
                if (sample == samples_by_key.end() ||
                    sample->second.first->ambiguity_index !=
                        pending->second.rover_ambiguity_index) {
                    pending = geometry_free_pending_resets.erase(pending);
                    continue;
                }
                const double age_s = problem.epochs[epoch_index].time -
                                     pending->second.time;
                if (age_s >= confirmation_s) {
                    geometry_free_reset_this_epoch.insert(pending->first);
                    ++geometry_free_slip_reset_total;
                    pending = geometry_free_pending_resets.erase(pending);
                    continue;
                }
                ++pending;
            }
            std::set<CarrierKey> slip_bands_this_epoch;
            for (const auto& [satellite, samples_by_signal] :
                 samples_by_satellite) {
                std::vector<GeometryFreeSample> samples;
                samples.reserve(samples_by_signal.size());
                for (const auto& [signal, sample] : samples_by_signal) {
                    (void)signal;
                    samples.push_back(sample);
                }
                for (std::size_t first = 0; first < samples.size(); ++first) {
                    for (std::size_t second = first + 1;
                         second < samples.size(); ++second) {
                        const auto* first_rover = samples[first].first;
                        const auto* second_rover = samples[second].first;
                        const auto* first_base = samples[first].second;
                        const auto* second_base = samples[second].second;
                        const double first_sd_phase_m =
                            first_rover->model_debug.raw_carrier_m -
                            first_base->model_debug.raw_carrier_m;
                        const double second_sd_phase_m =
                            second_rover->model_debug.raw_carrier_m -
                            second_base->model_debug.raw_carrier_m;
                        const double geometry_free_m =
                            first_sd_phase_m - second_sd_phase_m;
                        const GeometryFreePairKey pair_key{
                            satellite, first_rover->signal,
                            second_rover->signal};
                        const auto previous =
                            geometry_free_slip_states.find(pair_key);
                        if (previous != geometry_free_slip_states.end()) {
                            const double dt = problem.epochs[epoch_index].time -
                                              previous->second.time;
                            const bool same_rover_arcs =
                                previous->second.first_rover_ambiguity_index ==
                                    first_rover->ambiguity_index &&
                                previous->second.second_rover_ambiguity_index ==
                                    second_rover->ambiguity_index;
                            if (same_rover_arcs && dt > 0.0 &&
                                (max_segment_gap <= 0.0 ||
                                 dt <= max_segment_gap) &&
                                std::abs(geometry_free_m -
                                         previous->second.geometry_free_m) >
                                    std::max(
                                        0.0,
                                        config_.geometry_free_cycle_slip_threshold_m)) {
                                const CarrierKey first_key{
                                    satellite, first_rover->signal};
                                const CarrierKey second_key{
                                    satellite, second_rover->signal};
                                slip_bands_this_epoch.insert(first_key);
                                slip_bands_this_epoch.insert(second_key);
                            }
                        }
                        geometry_free_slip_states[pair_key] = {
                            problem.epochs[epoch_index].time,
                            geometry_free_m,
                            first_rover->ambiguity_index,
                            second_rover->ambiguity_index};
                    }
                }
            }
            for (const auto& key : slip_bands_this_epoch) {
                if (geometry_free_reset_this_epoch.count(key) != 0) {
                    continue;
                }
                const auto sample = samples_by_key.find(key);
                if (sample == samples_by_key.end()) {
                    continue;
                }
                const auto pending = geometry_free_pending_resets.find(key);
                if (pending != geometry_free_pending_resets.end()) {
                    // A second discontinuity before confirmation is
                    // ambiguous (often an isolated outlier returning to its
                    // former level), so fail closed instead of restarting the
                    // timer.
                    geometry_free_pending_resets.erase(pending);
                    continue;
                }
                geometry_free_pending_resets[key] = {
                    problem.epochs[epoch_index].time,
                    sample->second.first->ambiguity_index};
            }
        }

        if (rover_pseudorange_it != rover_pseudoranges_by_epoch.end()) {
            for (const auto* rover_factor : rover_pseudorange_it->second) {
                const CarrierKey key{rover_factor->satellite,
                                     rover_factor->signal};
                if (base_pseudorange_observations.find(key) ==
                    base_pseudorange_observations.end()) {
                    continue;
                }
                grouped_rover_pseudoranges[{rover_factor->satellite.system,
                                            rover_factor->signal}]
                    .push_back(rover_factor);
            }
        }
        if (rover_it != rover_carriers_by_epoch.end()) {
            for (const auto* rover_factor : rover_it->second) {
                const CarrierKey key{rover_factor->satellite,
                                     rover_factor->signal};
                if (base_carriers.find(key) == base_carriers.end()) {
                    continue;
                }
                grouped_rover_carriers[{rover_factor->satellite.system,
                                        rover_factor->signal}]
                    .push_back(rover_factor);
            }
        }
        for (const auto& [key, reference_observation] :
             base_reference_observations) {
            grouped_reference_observations[{reference_observation.satellite.system,
                                            reference_observation.signal}]
                .push_back(&reference_observation);
        }

        const auto select_reference =
            [&](const std::pair<GNSSSystem, SignalType>& group_key,
                std::size_t rejected_count,
                const CarrierPhaseFactor*& reference,
                const PreparedCarrierObservation*& base_reference) {
                if (group_key.first == GNSSSystem::GLONASS) {
                    problem.diagnostics.double_difference_rejected_no_reference +=
                        rejected_count;
                    return false;
                }

                const auto reference_group_it =
                    grouped_reference_observations.find(group_key);
                if (reference_group_it == grouped_reference_observations.end() ||
                    reference_group_it->second.empty()) {
                    problem.diagnostics.double_difference_rejected_no_reference +=
                        rejected_count;
                    return false;
                }

                const PreparedCarrierObservation* base_reference_selection =
                    *std::max_element(
                        reference_group_it->second.begin(),
                        reference_group_it->second.end(),
                        [](const PreparedCarrierObservation* lhs,
                           const PreparedCarrierObservation* rhs) {
                            return lhs->elevation_rad < rhs->elevation_rad;
                        });

                // --- CMC-aware reference selection
                // (FGOConfig::cmc_aware_reference_selection) ---
                // The plain max-elevation pick above can land on a satellite
                // the CMC screening pre-pass above already flagged as
                // sustained multipath/NLOS this epoch
                // (cmc_level_exclude_this_epoch). Every DD pair in the group
                // is formed against this ONE reference, so a multipath-
                // biased reference poisons every DD residual in the group at
                // once (see the config knob's doc comment for the tokyo
                // run1 episode that motivated this). When the knob is on and
                // CMC screening is active, prefer the highest-elevation
                // NON-excluded candidate instead; if every candidate in the
                // group is CMC-excluded this epoch, keep the original
                // max-elevation choice so the group is never dropped.
                if (config_.cmc_aware_reference_selection &&
                    config_.use_code_minus_carrier_screening &&
                    !cmc_level_exclude_this_epoch.empty()) {
                    const CarrierKey default_key{
                        base_reference_selection->satellite,
                        base_reference_selection->signal};
                    if (cmc_level_exclude_this_epoch.count(default_key) > 0) {
                        const PreparedCarrierObservation* best_clean = nullptr;
                        for (const auto* candidate : reference_group_it->second) {
                            const CarrierKey candidate_key{candidate->satellite,
                                                            candidate->signal};
                            if (cmc_level_exclude_this_epoch.count(candidate_key) >
                                0) {
                                continue;
                            }
                            if (!best_clean || candidate->elevation_rad >
                                                    best_clean->elevation_rad) {
                                best_clean = candidate;
                            }
                        }
                        if (best_clean) {
                            base_reference_selection = best_clean;
                            ++cmc_ref_avoided_total;
                        }
                        // else: every candidate in the group is CMC-excluded
                        // -- fall back to the original max-elevation choice
                        // (base_reference_selection unchanged) rather than
                        // dropping the whole group.
                    }
                }

                const CarrierKey reference_key{
                    base_reference_selection->satellite,
                    base_reference_selection->signal};
                const auto rover_reference_by_key_it =
                    rover_references_by_key.find(reference_key);
                if (rover_reference_by_key_it == rover_references_by_key.end()) {
                    problem.diagnostics.double_difference_rejected_no_reference +=
                        rejected_count;
                    return false;
                }
                reference = rover_reference_by_key_it->second;
                base_reference = base_reference_selection;
                return true;
            };

        std::map<DdFactorKey, DoubleDifferencePseudorangeFactor>
            pseudorange_factors_by_key;
        // Includes CMC-level-excluded code observations solely for carrier
        // ambiguity initialization when code-only exclusion is requested.
        std::map<DdFactorKey, DoubleDifferencePseudorangeFactor>
            pseudorange_seed_factors_by_key;
        for (const auto& [group_key, group] : grouped_rover_pseudoranges) {
            if (group_key.first == GNSSSystem::GLONASS) {
                problem.diagnostics.double_difference_rejected_no_reference +=
                    group.size();
                continue;
            }

            const CarrierPhaseFactor* reference = nullptr;
            const PreparedCarrierObservation* base_reference_ptr = nullptr;
            if (!select_reference(group_key,
                                  group.size(),
                                  reference,
                                  base_reference_ptr)) {
                continue;
            }
            const auto& base_reference = *base_reference_ptr;
            for (const auto* satellite : group) {
                if (satellite->satellite == reference->satellite &&
                    satellite->signal == reference->signal) {
                    continue;
                }
                const CarrierKey satellite_key{satellite->satellite, satellite->signal};
                const bool cmc_level_excluded =
                    cmc_level_exclude_this_epoch.count(satellite_key) > 0;
                if (cmc_level_excluded &&
                    !config_.code_minus_carrier_level_pseudorange_only) {
                    // Sustained CMC multipath this epoch: skip the DD
                    // pseudorange factor. The CP loop below only builds a
                    // carrier factor when it finds a matching entry in
                    // pseudorange_factors_by_key, so omitting the PR factor
                    // here transitively excludes the CP factor too (matches
                    // the reference's single `continue` before either).
                    continue;
                }
                const auto base_satellite_it =
                    base_pseudorange_observations.find(satellite_key);
                if (base_satellite_it == base_pseudorange_observations.end()) {
                    continue;
                }

                const auto& base_satellite = base_satellite_it->second;
                const double satellite_sin_el =
                    std::max(0.1, std::sin(satellite->elevation_rad));
                const double satellite_sqrt_sin_el =
                    std::sqrt(satellite_sin_el);

                DoubleDifferencePseudorangeFactor pseudorange_factor;
                pseudorange_factor.epoch_index = epoch_index;
                pseudorange_factor.satellite = satellite->satellite;
                pseudorange_factor.reference_satellite = reference->satellite;
                pseudorange_factor.signal = satellite->signal;
                pseudorange_factor.rover_satellite_position_ecef =
                    satellite->satellite_position_ecef;
                pseudorange_factor.rover_reference_position_ecef =
                    reference->satellite_position_ecef;
                pseudorange_factor.base_satellite_position_ecef =
                    base_satellite.satellite_position_ecef;
                pseudorange_factor.base_reference_position_ecef =
                    base_reference.satellite_position_ecef;
                pseudorange_factor.base_position_ecef = base_position_ecef;
                pseudorange_factor.observed_dd_pseudorange_m =
                    (satellite->corrected_pseudorange_m -
                     base_satellite.corrected_pseudorange_m) -
                    (reference->corrected_pseudorange_m -
                     base_reference.corrected_pseudorange_m);
                const double pseudorange_band_scale =
                    signal_policy::isSecondarySignal(satellite->satellite.system,
                                                     satellite->signal)
                        ? std::max(1.0,
                                   config_
                                       .double_difference_secondary_pseudorange_sigma_scale)
                        : 1.0;
                // Base DD-PR sigma: varerr (FGOConfig::use_elevation_
                // dependent_sigma) when enabled, else the pre-existing flat/
                // elevation-power fallback -- see the knob's doc comment in
                // fgo.hpp for the full rationale/composition order. The
                // secondary-band scale above still multiplies on top either
                // way (unchanged).
                double pseudorange_base_sigma = pseudorange_sigma / satellite_sqrt_sin_el;
                if (config_.use_elevation_dependent_sigma) {
                    const double el_pair_rad = std::max(
                        std::min(reference->elevation_rad, satellite->elevation_rad),
                        el_min_rad);
                    pseudorange_base_sigma = varerrDdSigma(
                        /*is_pseudorange=*/true, el_pair_rad, epoch_dt_s, config_);
                }
                pseudorange_factor.sigma_m =
                    pseudorange_band_scale * pseudorange_base_sigma;
                pseudorange_factor.elevation_rad = satellite->elevation_rad;
                pseudorange_factor.rover_satellite_model =
                    satellite->model_debug;
                pseudorange_factor.rover_reference_model =
                    reference->model_debug;
                pseudorange_factor.base_satellite_model =
                    base_satellite.model_debug;
                pseudorange_factor.base_reference_model =
                    base_reference.model_debug;
                const DdFactorKey factor_key{
                    epoch_index,
                    satellite->satellite,
                    reference->satellite,
                    satellite->signal,
                };
                pseudorange_seed_factors_by_key[factor_key] = pseudorange_factor;
                if (!cmc_level_excluded) {
                    problem.double_difference_pseudorange_factors.push_back(
                        pseudorange_factor);
                    pseudorange_factors_by_key[factor_key] = pseudorange_factor;
                }
            }
        }

        for (const auto& [group_key, group] : grouped_rover_carriers) {
            const CarrierPhaseFactor* reference = nullptr;
            const PreparedCarrierObservation* base_reference_ptr = nullptr;
            if (!select_reference(group_key,
                                  group.size(),
                                  reference,
                                  base_reference_ptr)) {
                continue;
            }
            const auto& base_reference = *base_reference_ptr;

            for (const auto* satellite : group) {
                if (satellite->satellite == reference->satellite &&
                    satellite->signal == reference->signal) {
                    continue;
                }
                const CarrierKey satellite_key{satellite->satellite, satellite->signal};
                const auto base_satellite_it = base_carriers.find(satellite_key);
                if (base_satellite_it == base_carriers.end()) {
                    continue;
                }
                const DdFactorKey factor_key{
                        epoch_index,
                        satellite->satellite,
                        reference->satellite,
                        satellite->signal,
                    };
                const auto pseudorange_factor_it = pseudorange_factors_by_key.find(factor_key);
                const DoubleDifferencePseudorangeFactor* pseudorange_factor_ptr =
                    pseudorange_factor_it != pseudorange_factors_by_key.end()
                        ? &pseudorange_factor_it->second
                        : nullptr;
                if (!pseudorange_factor_ptr &&
                    config_.code_minus_carrier_level_pseudorange_only) {
                    const auto seed_it = pseudorange_seed_factors_by_key.find(factor_key);
                    if (seed_it != pseudorange_seed_factors_by_key.end()) {
                        pseudorange_factor_ptr = &seed_it->second;
                    }
                }
                if (!pseudorange_factor_ptr) {
                    // Sustained CMC multipath dropped this satellite's DD-PR
                    // factor at problem-build time (and, transitively, this
                    // DD-CP row -- code_minus_carrier_level_pseudorange_only
                    // is false, else pseudorange_factor_ptr would have come
                    // from the seed map above). Retain a minimal excluded
                    // row -- geometry + observed DD carrier, no ambiguity/
                    // segment tracking (ambiguity_index is the sentinel
                    // max()) -- solely so the surplus-satellite independent
                    // integrity validator (FGOConfig::use_surplus_
                    // satellite_validation) can see observations that never
                    // entered ambiguity resolution at all. NEVER added to
                    // double_difference_carrier_factors / the solved graph.
                    if (cmc_level_exclude_this_epoch.count(satellite_key) > 0 &&
                        reference->has_carrier_phase && base_reference.has_carrier_phase) {
                        const auto& base_satellite_excl = base_satellite_it->second;
                        DoubleDifferenceCarrierFactor excluded;
                        excluded.epoch_index = epoch_index;
                        excluded.use_ambiguity_difference = false;
                        excluded.ambiguity_index = std::numeric_limits<std::size_t>::max();
                        excluded.satellite = satellite->satellite;
                        excluded.reference_satellite = reference->satellite;
                        excluded.signal = satellite->signal;
                        excluded.rover_satellite_position_ecef = satellite->satellite_position_ecef;
                        excluded.rover_reference_position_ecef = reference->satellite_position_ecef;
                        excluded.base_satellite_position_ecef =
                            base_satellite_excl.satellite_position_ecef;
                        excluded.base_reference_position_ecef =
                            base_reference.satellite_position_ecef;
                        excluded.base_position_ecef = base_position_ecef;
                        excluded.observed_dd_carrier_m =
                            (satellite->corrected_carrier_m -
                             base_satellite_excl.corrected_carrier_m) -
                            (reference->corrected_carrier_m -
                             base_reference.corrected_carrier_m);
                        excluded.sigma_m = carrier_sigma;  // unused by the surplus validator
                        excluded.elevation_rad = satellite->elevation_rad;
                        problem.excluded_double_difference_carrier_factors.push_back(excluded);
                    }
                    continue;
                }
                ++problem.diagnostics.double_difference_candidate_pairs;

                const auto& base_satellite = base_satellite_it->second;
                const auto& pseudorange_factor = *pseudorange_factor_ptr;
                const double satellite_sin_el =
                    std::max(0.1, std::sin(satellite->elevation_rad));
                const double satellite_sqrt_sin_el =
                    std::sqrt(satellite_sin_el);

                if (!reference->has_carrier_phase ||
                    !base_reference.has_carrier_phase) {
                    continue;
                }

                DoubleDifferenceCarrierFactor factor;
                factor.epoch_index = epoch_index;
                factor.use_ambiguity_difference = false;
                factor.satellite = satellite->satellite;
                factor.reference_satellite = reference->satellite;
                factor.signal = satellite->signal;
                factor.rover_satellite_position_ecef =
                    satellite->satellite_position_ecef;
                factor.rover_reference_position_ecef =
                    reference->satellite_position_ecef;
                factor.base_satellite_position_ecef =
                    base_satellite.satellite_position_ecef;
                factor.base_reference_position_ecef =
                    base_reference.satellite_position_ecef;
                factor.base_position_ecef = base_position_ecef;
                factor.observed_dd_carrier_m =
                    (satellite->corrected_carrier_m -
                     base_satellite.corrected_carrier_m) -
                    (reference->corrected_carrier_m -
                     base_reference.corrected_carrier_m);
                const double carrier_band_scale =
                    signal_policy::isSecondarySignal(satellite->satellite.system,
                                                     satellite->signal)
                        ? std::max(1.0,
                                   config_
                                       .double_difference_secondary_carrier_sigma_scale)
                        : 1.0;
                // Base DD-CP sigma: varerr when enabled, else the
                // pre-existing flat/elevation-power fallback -- same source
                // switch as the DD-PR site above (see that comment and the
                // knob's doc comment in fgo.hpp). Secondary-band scale still
                // multiplies on top either way.
                double carrier_base_sigma = carrier_sigma / satellite_sqrt_sin_el;
                if (config_.use_elevation_dependent_sigma) {
                    const double el_pair_rad = std::max(
                        std::min(reference->elevation_rad, satellite->elevation_rad),
                        el_min_rad);
                    carrier_base_sigma = varerrDdSigma(
                        /*is_pseudorange=*/false, el_pair_rad, epoch_dt_s, config_);
                }
                factor.sigma_m = carrier_band_scale * carrier_base_sigma;
                factor.elevation_rad = satellite->elevation_rad;
                factor.rover_satellite_model = satellite->model_debug;
                factor.rover_reference_model = reference->model_debug;
                factor.base_satellite_model = base_satellite.model_debug;
                factor.base_reference_model = base_reference.model_debug;

                const Vector3d seed_position =
                    problem.epochs[epoch_index].position_ecef;
                const double rover_satellite_range =
                    (factor.rover_satellite_position_ecef - seed_position).norm();
                const double rover_reference_range =
                    (factor.rover_reference_position_ecef - seed_position).norm();
                const double base_satellite_range =
                    (factor.base_satellite_position_ecef - base_position_ecef).norm();
                const double base_reference_range =
                    (factor.base_reference_position_ecef - base_position_ecef).norm();

                const DoubleDifferenceAmbiguityKey ambiguity_key{
                    satellite->satellite,
                    reference->satellite,
                    satellite->signal,
                    satellite->ambiguity_index,
                    0,
                };
                auto active_it = active_dd_segments.find(ambiguity_key);
                bool start_new_segment =
                    config_.reset_double_difference_ambiguities_each_epoch ||
                    active_it == active_dd_segments.end();
                if (!start_new_segment) {
                    const double dt = problem.epochs[epoch_index].time -
                                      problem.epochs[active_it->second.last_epoch_index].time;
                    if (dt <= 0.0 ||
                        (max_segment_gap > 0.0 && dt > max_segment_gap)) {
                        start_new_segment = true;
                    }
                }
                if (cmc_jump_reset_this_epoch.count(
                        CarrierKey{satellite->satellite, satellite->signal}) > 0) {
                    // CMC jump: force the same arc break the loss-of-lock /
                    // gap checks above already perform (multipath jump
                    // breaks the integer ambiguity).
                    start_new_segment = true;
                }
                if (geometry_free_reset_this_epoch.count(
                        CarrierKey{satellite->satellite, satellite->signal}) > 0 ||
                    geometry_free_reset_this_epoch.count(
                        CarrierKey{reference->satellite, reference->signal}) > 0) {
                    start_new_segment = true;
                }

                if (start_new_segment) {
                    AmbiguityState ambiguity;
                    ambiguity.satellite = satellite->satellite;
                    ambiguity.reference_satellite = reference->satellite;
                    ambiguity.signal = satellite->signal;
                    ambiguity.is_double_difference = true;
                    ambiguity.segment_index =
                        next_dd_segment_indices[DoubleDifferenceSegmentKey{
                            satellite->satellite,
                            reference->satellite,
                            satellite->signal,
                        }]++;
                    ambiguity.wavelength_m = satellite->wavelength_m;
                    const double dd_geometry_at_seed =
                        (rover_satellite_range - base_satellite_range) -
                        (rover_reference_range - base_reference_range);
                    const double carrier_residual_at_seed =
                        factor.observed_dd_carrier_m - dd_geometry_at_seed;
                    const double pseudorange_residual_at_seed =
                        pseudorange_factor.observed_dd_pseudorange_m -
                        dd_geometry_at_seed;
                    ambiguity.initial_ambiguity_m =
                        carrier_residual_at_seed -
                        pseudorange_residual_at_seed;
                    active_it =
                        active_dd_segments
                            .insert_or_assign(
                                ambiguity_key,
                                ActiveCarrierSegment{problem.ambiguity_states.size(),
                                                     epoch_index})
                            .first;
                    problem.ambiguity_states.push_back(ambiguity);
                } else {
                    active_it->second.last_epoch_index = epoch_index;
                }

                factor.ambiguity_index = active_it->second.ambiguity_index;
                problem.double_difference_carrier_factors.push_back(factor);
            }
        }
    }

    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        sd_reference_by_group[SingleDifferenceReferenceKey{
            factor.epoch_index,
            factor.satellite.system,
            factor.signal,
        }] = factor.reference_satellite;
        ++sd_default_reference_counts[SingleDifferenceDefaultReferenceKey{
            factor.satellite.system,
            factor.signal,
        }][factor.reference_satellite];
    }
    for (const auto& factor : problem.double_difference_carrier_factors) {
        sd_reference_by_group[SingleDifferenceReferenceKey{
            factor.epoch_index,
            factor.satellite.system,
            factor.signal,
        }] = factor.reference_satellite;
        ++sd_default_reference_counts[SingleDifferenceDefaultReferenceKey{
            factor.satellite.system,
            factor.signal,
        }][factor.reference_satellite];
    }

    if (config_.use_ambiguity_between_factors) {
        using AmbiguityTrackKey = std::pair<SatelliteId, SignalType>;
        std::map<AmbiguityTrackKey, const DoubleDifferenceCarrierFactor*>
            previous_by_track;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
                continue;
            }
            const AmbiguityTrackKey track_key{factor.satellite, factor.signal};
            const auto previous_it = previous_by_track.find(track_key);
            if (previous_it != previous_by_track.end()) {
                const auto* previous = previous_it->second;
                const double dt =
                    problem.epochs[factor.epoch_index].time -
                    problem.epochs[previous->epoch_index].time;
                if (factor.epoch_index == previous->epoch_index + 1 &&
                    dt > 0.0 &&
                    (config_.max_tdcp_gap_s <= 0.0 ||
                     dt <= config_.max_tdcp_gap_s) &&
                    previous->ambiguity_index != factor.ambiguity_index) {
                    const auto& ambiguity =
                        problem.ambiguity_states[factor.ambiguity_index];
                    if (ambiguity.wavelength_m > 0.0) {
                        AmbiguityBetweenFactor between;
                        between.previous_epoch_index = previous->epoch_index;
                        between.current_epoch_index = factor.epoch_index;
                        between.previous_ambiguity_index =
                            previous->ambiguity_index;
                        between.current_ambiguity_index = factor.ambiguity_index;
                        between.satellite = factor.satellite;
                        between.signal = factor.signal;
                        between.sigma_m =
                            std::max(1e-9,
                                     config_.ambiguity_between_sigma_cycles *
                                         ambiguity.wavelength_m);
                        problem.ambiguity_between_factors.push_back(between);
                    }
                }
            }
            previous_by_track[track_key] = &factor;
        }
    }

    std::map<SingleDifferenceDefaultReferenceKey, SatelliteId>
        sd_default_reference_by_group;
    for (const auto& [group_key, counts] : sd_default_reference_counts) {
        const auto best_it =
            std::max_element(counts.begin(),
                             counts.end(),
                             [](const auto& lhs, const auto& rhs) {
                                 return lhs.second < rhs.second;
                             });
        if (best_it != counts.end()) {
            sd_default_reference_by_group[group_key] = best_it->first;
        }
    }

    if (config_.use_single_difference_doppler_factors ||
        config_.monitor_external_doppler_dr ||
        config_.use_external_doppler_dr_validation ||
        config_.monitor_candidate_integrity_witness ||
        config_.use_single_difference_tdcp_factors) {
        std::map<CarrierKey, SingleDifferenceCarrierResidual>
            previous_sd_carrier_residuals;
        for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
             ++epoch_index) {
            std::map<CarrierKey, const CarrierPhaseFactor*> rover_by_key;
            std::vector<const CarrierPhaseFactor*> rover_factor_rows;
            const auto rover_pseudorange_it =
                rover_pseudoranges_by_epoch.find(epoch_index);
            if (rover_pseudorange_it != rover_pseudoranges_by_epoch.end()) {
                for (const auto* rover : rover_pseudorange_it->second) {
                    rover_by_key[{rover->satellite, rover->signal}] = rover;
                    rover_factor_rows.push_back(rover);
                }
            }

            std::map<CarrierKey, SingleDifferenceCarrierResidual>
                current_sd_carrier_residuals;
            for (const auto* rover : rover_factor_rows) {
                const CarrierKey carrier_key{rover->satellite, rover->signal};
                const auto reference_it =
                    sd_reference_by_group.find(SingleDifferenceReferenceKey{
                        epoch_index,
                        rover->satellite.system,
                        rover->signal,
                    });
                SatelliteId reference_satellite;
                if (reference_it != sd_reference_by_group.end()) {
                    reference_satellite = reference_it->second;
                } else {
                    const auto default_reference_it =
                        sd_default_reference_by_group.find(
                            SingleDifferenceDefaultReferenceKey{
                                rover->satellite.system,
                                rover->signal,
                            });
                    if (default_reference_it ==
                            sd_default_reference_by_group.end() ||
                        !(rover->satellite == default_reference_it->second)) {
                        continue;
                    }
                    reference_satellite = default_reference_it->second;
                }

                const auto reference_model_it =
                    rover_by_key.find({reference_satellite, rover->signal});
                if (reference_model_it == rover_by_key.end()) {
                    continue;
                }
                const auto* reference = reference_model_it->second;
                // A satellite differenced with itself is identically zero
                // and adds no information (but can distort factor counts and
                // residual diagnostics).
                if (rover->satellite == reference_satellite) {
                    continue;
                }
                const Vector3d sd_los = rover->los - reference->los;

                if ((config_.use_single_difference_doppler_factors ||
                     config_.monitor_external_doppler_dr ||
                     config_.use_external_doppler_dr_validation ||
                     config_.monitor_candidate_integrity_witness) &&
                    rover->has_doppler_residual &&
                    reference->has_doppler_residual) {
                    SingleDifferenceDopplerFactor factor;
                    factor.epoch_index = epoch_index;
                    factor.satellite = rover->satellite;
                    factor.reference_satellite = reference_satellite;
                    factor.signal = rover->signal;
                    factor.los = sd_los;
                    factor.residual_mps =
                        rover->doppler_residual_mps -
                        reference->doppler_residual_mps;
                    factor.sigma_mps = std::hypot(rover->doppler_sigma_mps,
                                                  reference->doppler_sigma_mps);
                    factor.elevation_rad = rover->elevation_rad;
                    if (std::isfinite(factor.residual_mps) &&
                        std::isfinite(factor.sigma_mps) &&
                        factor.sigma_mps > 0.0) {
                        problem.single_difference_doppler_factors.push_back(factor);
                    }
                }

                if (rover->has_carrier_phase &&
                    reference->has_carrier_phase &&
                    !rover->loss_of_lock &&
                    !reference->loss_of_lock) {
                    const double rover_carrier_residual =
                        rover->corrected_carrier_m -
                        rover->model_debug.geometric_range_m;
                    const double reference_carrier_residual =
                        reference->corrected_carrier_m -
                        reference->model_debug.geometric_range_m;
                    const double sd_carrier_residual =
                        rover_carrier_residual - reference_carrier_residual;
                    current_sd_carrier_residuals[carrier_key] =
                        SingleDifferenceCarrierResidual{
                            epoch_index,
                            sd_carrier_residual,
                            sd_los,
                        };

                    const auto previous_it =
                        previous_sd_carrier_residuals.find(carrier_key);
                    if (config_.use_single_difference_tdcp_factors &&
                        previous_it != previous_sd_carrier_residuals.end()) {
                        const double tdcp =
                            sd_carrier_residual - previous_it->second.residual_m;
                        if (std::isfinite(tdcp) && tdcp != 0.0) {
                            SingleDifferenceTdcpFactor factor;
                            factor.previous_epoch_index =
                                previous_it->second.epoch_index;
                            factor.current_epoch_index = epoch_index;
                            factor.satellite = rover->satellite;
                            factor.reference_satellite = reference_satellite;
                            factor.signal = rover->signal;
                            factor.previous_los = sd_los;
                            factor.los = sd_los;
                            factor.delta_carrier_m = tdcp;
                            const double sin_el =
                                std::max(0.1, std::sin(rover->elevation_rad));
                            factor.sigma_m =
                                std::max(1e-4,
                                         config_.single_difference_tdcp_sigma_m /
                                             std::sqrt(sin_el));
                            factor.elevation_rad = rover->elevation_rad;
                            problem.single_difference_tdcp_factors.push_back(factor);
                        }
                    }
                }
            }
            previous_sd_carrier_residuals =
                std::move(current_sd_carrier_residuals);
        }
    }

    problem.diagnostics.code_minus_carrier_jump_resets = cmc_jump_reset_total;
    problem.diagnostics.geometry_free_cycle_slip_resets =
        geometry_free_slip_reset_total;
    problem.diagnostics.code_minus_carrier_level_exclusions =
        cmc_level_exclusion_total;
    problem.diagnostics.cmc_ref_avoided_count = cmc_ref_avoided_total;

    return problem;
}

}  // namespace libgnss
