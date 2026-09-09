#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/phase126_raw_base_compound.hpp>
#include <libgnss++/algorithms/source_epoch_states.hpp>
#include <libgnss++/algorithms/phase127_glonass_channel_provenance.hpp>
#include <libgnss++/algorithms/phase128_glonass_provenance.hpp>
#include <libgnss++/algorithms/phase129_glonass_local_miss.hpp>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/algorithms/galileo_group_delay.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <iterator>

namespace libgnss::base_pseudorange_compensation {
namespace {

bool finiteTime(const GNSSTime& time) {
    return std::isfinite(time.tow) && time.week >= -10000 && time.week <= 100000;
}

Vector3d earthRotationCorrected(const Vector3d& satellite_position,
                                const Vector3d& receiver_position) {
    const double travel_time =
        (satellite_position - receiver_position).norm() /
        constants::SPEED_OF_LIGHT;
    const double angle = constants::OMEGA_E * travel_time;
    Eigen::Matrix3d rotation;
    rotation << std::cos(angle), std::sin(angle), 0.0,
                -std::sin(angle), std::cos(angle), 0.0,
                0.0, 0.0, 1.0;
    return rotation * satellite_position;
}

bool isHealthyForPositioning(const Observation& observation,
                             const Ephemeris& ephemeris) {
    int health = static_cast<int>(ephemeris.health);
    if (observation.satellite.system == GNSSSystem::QZSS) {
        health &= 0xFE;
    }
    return health == 0;
}

double groupDelayCorrectionMeters(const Observation& observation,
                                  const Ephemeris& ephemeris,
                                  bool use_signal_specific_e1) {
    if (observation.satellite.system == GNSSSystem::Galileo) {
        return galileo_group_delay::correctionMeters(
            observation, ephemeris, use_signal_specific_e1);
    }
    switch (observation.satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return ephemeris.tgd * constants::SPEED_OF_LIGHT;
        case GNSSSystem::BeiDou:
            switch (observation.signal) {
                case SignalType::BDS_B1I:
                case SignalType::BDS_B1C:
                    return ephemeris.tgd * constants::SPEED_OF_LIGHT;
                case SignalType::BDS_B2I:
                case SignalType::BDS_B2A:
                    return ephemeris.tgd_secondary * constants::SPEED_OF_LIGHT;
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
}

double median(std::vector<double> values) {
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
    return values.size() % 2U == 0U
               ? 0.5 * (values[middle - 1U] + values[middle])
               : values[middle];
}

double percentile(std::vector<double> values, double p) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                  values.end());
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    if (values.size() == 1U) return values.front();
    const double rank = 0.5 + p / 100.0 * static_cast<double>(values.size());
    if (rank <= 1.0) return values.front();
    if (rank >= static_cast<double>(values.size())) return values.back();
    const double lower_rank = std::floor(rank);
    const std::size_t lower = static_cast<std::size_t>(lower_rank - 1.0);
    const std::size_t upper = lower + 1U;
    return values[lower] + (rank - lower_rank) * (values[upper] - values[lower]);
}

}  // namespace

std::vector<double> centeredMovingMean(const std::vector<double>& values,
                                       std::size_t window_samples) {
    if (values.empty() || window_samples == 0U) return {};
    std::vector<double> result(values.size(),
                               std::numeric_limits<double>::quiet_NaN());
    const std::size_t left = window_samples / 2U;
    const std::size_t right = window_samples - left - 1U;
    for (std::size_t index = 0U; index < values.size(); ++index) {
        const std::size_t begin = index > left ? index - left : 0U;
        const std::size_t end = std::min(values.size() - 1U, index + right);
        double sum = 0.0;
        std::size_t count = 0U;
        for (std::size_t cursor = begin; cursor <= end; ++cursor) {
            if (std::isfinite(values[cursor])) {
                sum += values[cursor];
                ++count;
            }
        }
        if (count != 0U) result[index] = sum / static_cast<double>(count);
    }
    return result;
}

bool Model::build(const ObservationSeries& base_epochs,
                  const NavigationData& nav,
                  const Config& config) {
    streams_.clear();
    canonical_streams_.clear();
    // Build into a private candidate map.  The source-complete selector is a
    // three-step compound operation; a failed ingress/state/stream step must
    // never leave a partially usable correction map behind.
    std::map<ObservationKey, std::vector<Sample>> candidate_streams;
    std::map<phase131_canonical::Key,
             std::map<SignalType, std::vector<Sample>>>
        canonical_candidates;
    std::set<ObservationKey> local_miss_stream_keys;
    diagnostics_ = Diagnostics{};
    diagnostics_.enabled = true;
    diagnostics_.source_complete = config.source_complete;
    diagnostics_.source_epoch_states_requested = config.use_source_epoch_states;
    if (config.use_dense_epoch_smoothing &&
        (!config.use_source_epoch_states || config.use_phase131_canonical_correction_band_key)) {
        diagnostics_.failure = "dense smoothing requires source epoch states and typed stream keys";
        return false;
    }
    if (config.use_source_fgo_frequency_slots && !config.use_source_epoch_states) {
        diagnostics_.failure = "source frequency slots require source epoch states";
        return false;
    }
    if (config.use_source_epoch_states && !config.source_complete) {
        diagnostics_.failure = "source epoch states require source-complete base equations";
        return false;
    }
    diagnostics_.phase127_enabled =
        config.use_phase127_glonass_channel_provenance;
    diagnostics_.phase128_enabled =
        config.use_phase128_glonass_provenance_parser_admission;
    diagnostics_.phase129_enabled =
        config.use_phase129_glonass_local_miss_mask;
    diagnostics_.phase131_enabled =
        config.use_phase131_canonical_correction_band_key;
    diagnostics_.phase128_header_status =
        glonassFrequencyChannelHeaderStatusName(
            config.glonass_frequency_channel_header_status);
    if (config.use_phase127_glonass_channel_provenance &&
        !config.source_complete) {
        diagnostics_.failure =
            "Phase127 GLONASS provenance requires Phase126 source-complete mode";
        return false;
    }
    if (config.use_phase128_glonass_provenance_parser_admission &&
        (!config.use_phase127_glonass_channel_provenance ||
         !config.source_complete)) {
        diagnostics_.failure =
            "Phase128 GLONASS parser/admission requires the Phase126/127 "
            "composed source-complete mode";
        return false;
    }
    if (config.use_phase129_glonass_local_miss_mask &&
        (!config.source_complete ||
         !config.use_phase127_glonass_channel_provenance ||
         !config.use_phase128_glonass_provenance_parser_admission)) {
        diagnostics_.phase129_configuration_valid = false;
        diagnostics_.phase129_configuration_failure =
            "Phase129 GLONASS local miss mask requires the composed "
            "Phase126/127/128 source-complete selectors";
        diagnostics_.failure = diagnostics_.phase129_configuration_failure;
        return false;
    }
    if (config.use_phase131_canonical_correction_band_key &&
        (!config.source_complete ||
         !config.use_phase127_glonass_channel_provenance ||
         !config.use_phase128_glonass_provenance_parser_admission ||
         !config.use_phase129_glonass_local_miss_mask)) {
        diagnostics_.phase131_configuration_valid = false;
        diagnostics_.phase131_configuration_failure =
            "Phase131 canonical correction key requires the composed "
            "Phase126/127/128/129 source-complete selectors";
        diagnostics_.failure = diagnostics_.phase131_configuration_failure;
        return false;
    }
    diagnostics_.station_reference_verified =
        config.station_reference_verified;
    diagnostics_.official_no_explicit_tgd_bgd = config.source_complete;
    diagnostics_.base_epochs = base_epochs.epochs.size();
    diagnostics_.moving_mean_samples = config.moving_mean_samples;
    Vector3d reference_position = config.base_position_ecef;
    if (config.source_complete) {
        phase126_raw_base::StationReference station_reference;
        station_reference.approximate_position_ecef =
            config.base_position_ecef;
        station_reference.antenna_delta_enu = config.antenna_delta_enu;
        station_reference.has_approximate_position =
            config.approximate_position_present;
        station_reference.has_antenna_delta = config.antenna_delta_present;
        station_reference.antenna_reference_convention_proven =
            config.station_reference_verified &&
            config.antenna_reference_is_approx_position;
        std::string station_failure;
        if (!phase126_raw_base::validateStationReference(
                station_reference, station_failure)) {
            diagnostics_.failure = station_failure;
            return false;
        }
        reference_position = phase126_raw_base::antennaReferenceEcef(
            station_reference, false);
        if (!reference_position.allFinite()) {
            diagnostics_.failure =
                "source-complete antenna reference is non-finite";
            return false;
        }
    }
    if (!reference_position.allFinite() || reference_position.norm() < 6.0e6 ||
        reference_position.norm() > 7.0e6) {
        diagnostics_.failure = "base coordinate is not finite/Earth-valid";
        return false;
    }
    if (!(config.expected_interval_s == 1.0 || config.expected_interval_s == 15.0) ||
        config.moving_mean_samples == 0U) {
        diagnostics_.failure = "unsupported base interval or moving-mean window";
        return false;
    }
    if (config.source_complete &&
        !((config.expected_interval_s == 1.0 &&
           config.moving_mean_samples == 151U) ||
          (config.expected_interval_s == 15.0 &&
           config.moving_mean_samples == 11U))) {
        diagnostics_.failure =
            "source-complete moving-mean window does not match frozen route";
        return false;
    }
    double base_lat = 0.0;
    double base_lon = 0.0;
    double base_height = 0.0;
    ecef2geodetic(reference_position, base_lat, base_lon, base_height);
    if (!std::isfinite(base_lat) || !std::isfinite(base_lon) ||
        !std::isfinite(base_height)) {
        diagnostics_.failure = "base coordinate has no finite geodetic form";
        return false;
    }
    GNSSTime previous_time;
    bool have_previous_time = false;
    std::vector<double> observed_intervals;
    for (const auto& epoch : base_epochs.epochs) {
        if (!finiteTime(epoch.time)) {
            diagnostics_.failure = "base epoch time is non-finite";
            return false;
        }
        if (have_previous_time) {
            const double dt = epoch.time - previous_time;
            if (!(dt > 0.0)) {
                diagnostics_.failure = "base epochs are not strictly monotonic";
                return false;
            }
            observed_intervals.push_back(dt);
        }
        previous_time = epoch.time;
        have_previous_time = true;
        std::map<SatelliteId, source_transmission_clock::EpochSatelliteState> epoch_states;
        if (config.use_source_epoch_states) {
            try {
                epoch_states = source_transmission_clock::buildEpochStates(
                    epoch.time, epoch.observations, nav);
                diagnostics_.source_epoch_states_built += epoch_states.size();
            } catch (const std::exception& error) {
                diagnostics_.failure = std::string("source epoch states: ") + error.what();
                return false;
            }
        }
        for (const auto& observation : epoch.observations) {
            ++diagnostics_.base_observation_rows;
            if (!observation.valid || !observation.has_pseudorange ||
                !(observation.pseudorange > 0.0) ||
                !std::isfinite(observation.pseudorange)) {
                continue;
            }
            ++diagnostics_.matched_base_rows;
            if (config.use_source_fgo_frequency_slots) {
                const auto& code = observation.pseudorange_observation_type;
                const auto slot = code.size() == 3
                    ? source_transmission_clock::slotForRinexBand(
                          observation.satellite.system, code[1]-'0')
                    : std::nullopt;
                if (!slot) {
                    diagnostics_.failure = "source correction row has no frequency slot";
                    return false;
                }
                if (*slot != source_transmission_clock::Slot::L1 &&
                    *slot != source_transmission_clock::Slot::L5) {
                    ++diagnostics_.source_frequency_rows_excluded;
                    continue;
                }
            }
            GNSSTime transmit_time =
                epoch.time - observation.pseudorange / constants::SPEED_OF_LIGHT;
            Vector3d satellite_position;
            Vector3d satellite_velocity;
            double satellite_clock_bias = 0.0;
            double satellite_clock_drift = 0.0;
            const Ephemeris* ephemeris = nullptr;
            if (config.use_source_epoch_states) {
                const auto state = epoch_states.find(observation.satellite);
                if (state == epoch_states.end()) {
                    diagnostics_.failure = "source epoch state missing for base row";
                    return false;
                }
                const auto records = nav.ephemeris_data.find(observation.satellite);
                if (records == nav.ephemeris_data.end()) return false;
                ephemeris = source_transmission_clock::selectBroadcastMessage(
                    records->second, observation.satellite, epoch.time);
                if (!ephemeris) return false;
                transmit_time = state->second.state.transmit_time;
                satellite_position = state->second.state.position_ecef;
                satellite_velocity = state->second.state.velocity_ecef;
                satellite_clock_bias = state->second.state.clock_seconds;
                satellite_clock_drift = state->second.state.clock_drift;
            } else {
            if (!nav.calculateSatelliteState(observation.satellite,
                                             transmit_time,
                                             satellite_position,
                                             satellite_velocity,
                                             satellite_clock_bias,
                                             satellite_clock_drift)) {
                if (config.source_complete) {
                    diagnostics_.failure =
                        "source-complete transmission-time satellite state unavailable";
                    return false;
                }
                continue;
            }
            transmit_time = transmit_time - satellite_clock_bias;
            if (!nav.calculateSatelliteState(observation.satellite,
                                             transmit_time,
                                             satellite_position,
                                             satellite_velocity,
                                             satellite_clock_bias,
                                             satellite_clock_drift)) {
                if (config.source_complete) {
                    diagnostics_.failure =
                        "source-complete transmission-time satellite state unavailable";
                    return false;
                }
                continue;
            }
            ephemeris = nav.getEphemeris(observation.satellite, transmit_time);
            }
            if (ephemeris == nullptr) {
                if (config.source_complete) {
                    diagnostics_.failure =
                        "source-complete ephemeris unavailable";
                    return false;
                }
                continue;
            }
            if (!isHealthyForPositioning(observation, *ephemeris)) {
                continue;
            }
            Observation frequency_observation = observation;
            phase131_canonical::Canonicalization canonicalization;
            if ((config.use_phase127_glonass_channel_provenance ||
                 config.use_phase128_glonass_provenance_parser_admission) &&
                observation.satellite.system == GNSSSystem::GLONASS) {
                ++diagnostics_.phase127_glonass_rows;
                phase127_glonass::Result provenance;
                if (config.use_phase128_glonass_provenance_parser_admission) {
                    provenance = phase128_glonass::resolve(
                        observation.satellite, transmit_time, nav,
                        config.glonass_frequency_channel_header_status,
                        config.glonass_frequency_channel_entries,
                        config.glonass_frequency_channel_malformed_entries);
                    ++diagnostics_.phase128_canonical_records;
                } else {
                    provenance = phase127_glonass::resolve(
                        observation.satellite, transmit_time, nav,
                        config.glonass_frequency_channel_entries,
                        config.glonass_frequency_channel_malformed_entries);
                }
                const auto& provenance_diagnostics = provenance.diagnostics;
                diagnostics_.phase127_header_entries_seen +=
                    provenance_diagnostics.header_entries_seen;
                diagnostics_.phase127_header_duplicate_entries +=
                    provenance_diagnostics.header_duplicate_entries;
                diagnostics_.phase127_header_conflict_entries +=
                    provenance_diagnostics.header_conflict_entries;
                diagnostics_.phase127_header_malformed_entries +=
                    provenance_diagnostics.header_malformed_entries;
                diagnostics_.phase127_ephemeris_candidates +=
                    provenance_diagnostics.ephemeris_candidates;
                diagnostics_.phase127_ephemeris_ties +=
                    provenance_diagnostics.ephemeris_ties;
                diagnostics_.phase127_ephemeris_duplicate_entries +=
                    provenance_diagnostics.ephemeris_duplicate_entries;
                diagnostics_.phase127_ephemeris_conflict_entries +=
                    provenance_diagnostics.ephemeris_conflict_entries;
                diagnostics_.phase127_query_time_coverage_gaps +=
                    provenance_diagnostics.query_time_coverage_gaps;
                diagnostics_.phase127_invalid_channels +=
                    provenance_diagnostics.invalid_channel_entries;
                if (!provenance.accepted) {
                    if (config.use_phase128_glonass_provenance_parser_admission) {
                        ++diagnostics_.phase128_canonical_rejected_records;
                    }
                    const auto classification =
                        phase129_glonass_local_miss::classify(
                            provenance,
                            config.use_phase129_glonass_local_miss_mask);
                    ++diagnostics_.phase127_failure_counts[
                        classification.reason_code];
                    if (classification.local_miss) {
                        // Keep this row out of the correction stream.  The
                        // exact Phase127 reason is retained in a local-miss
                        // ledger, while global source-complete failures below
                        // still abort the whole compound build.
                        ++diagnostics_.phase129_glonass_local_miss_rows;
                        local_miss_stream_keys.insert(
                            {observation.satellite, observation.signal});
                        ++diagnostics_.phase129_glonass_local_miss_counts[
                            classification.reason_code];
                        continue;
                    }
                    diagnostics_.failure =
                        "phase127-glonass-channel-provenance: " +
                        (provenance_diagnostics.failure.empty()
                             ? classification.reason_code
                             : provenance_diagnostics.failure);
                    return false;
                }
                ++diagnostics_.phase127_accepted_rows;
                if (provenance.source == phase127_glonass::ChannelSource::Header) {
                    ++diagnostics_.phase127_header_primary_rows;
                } else if (provenance.source ==
                           phase127_glonass::ChannelSource::BroadcastEphemeris) {
                    ++diagnostics_.phase127_ephemeris_fallback_rows;
                }
                // This local annotation is the exact source-certified value
                // consumed by the existing signal-frequency helper.  It is
                // not written back to the source stream or used to add a
                // factor/state, so Phase126 equations remain unchanged.
                frequency_observation.has_glonass_frequency_channel = true;
                frequency_observation.glonass_frequency_channel =
                    provenance.channel;
            }
            if (config.use_phase131_canonical_correction_band_key) {
                canonicalization = phase131_canonical::canonicalize(
                    observation.satellite, observation.signal,
                    frequency_observation.has_glonass_frequency_channel,
                    frequency_observation.glonass_frequency_channel);
                if (!canonicalization.accepted) {
                    ++diagnostics_.phase131_canonical_rejected_rows;
                    ++diagnostics_.phase131_failure_counts[
                        canonicalization.reason];
                    if (canonicalization.reason ==
                        "unknown-physical-frequency-family") {
                        ++diagnostics_.phase131_unknown_band_rows;
                    }
                    // Unsupported/uncertified rows are explicit local
                    // misses.  The source-complete stream remains usable if
                    // another certified family has support; no raw or zero
                    // correction is inserted for this row.
                    continue;
                }
                ++diagnostics_.phase131_canonical_rows;
            }
            phase126_raw_base::Geometry source_geometry;
            NavigationData::SatelliteGeometry legacy_geometry;
            if (config.source_complete) {
                source_geometry = phase126_raw_base::geodistWithSagnac(
                    reference_position, satellite_position);
                ++diagnostics_.sagnac_evaluations;
                if (!source_geometry.line_of_sight.allFinite() ||
                    !std::isfinite(source_geometry.range_m) ||
                    !std::isfinite(source_geometry.elevation_rad) ||
                    !std::isfinite(source_geometry.azimuth_rad)) {
                    diagnostics_.failure =
                        "source-complete geometry/Sagnac is non-finite";
                    return false;
                }
            } else {
                const Vector3d rotated_satellite = earthRotationCorrected(
                    satellite_position, config.base_position_ecef);
                legacy_geometry = nav.calculateGeometry(
                    config.base_position_ecef, rotated_satellite);
            }
            const double geometric_range = config.source_complete
                                               ? source_geometry.range_m
                                               : legacy_geometry.distance;
            if (!(geometric_range > 0.0) || !std::isfinite(geometric_range)) {
                if (config.source_complete) {
                    diagnostics_.failure =
                        "source-complete geometric range is invalid";
                    return false;
                }
                continue;
            }
            const double frequency_hz =
                config.use_phase127_glonass_channel_provenance &&
                        observation.satellite.system == GNSSSystem::GLONASS
                    ? signalFrequencyHz(frequency_observation)
                    : signalFrequencyHz(observation.signal, ephemeris);
            if (config.source_complete &&
                (!(frequency_hz > 0.0) || !std::isfinite(frequency_hz))) {
                diagnostics_.failure =
                    "source-complete signal frequency is unavailable";
                return false;
            }
            double ionosphere_delay = 0.0;
            if (config.use_ionosphere_model && nav.ionosphere_model.valid) {
                ionosphere_delay = models::ionoDelayKlobuchar(
                    base_lat, base_lon,
                    config.source_complete ? source_geometry.azimuth_rad
                                           : legacy_geometry.azimuth,
                    config.source_complete ? source_geometry.elevation_rad
                                           : legacy_geometry.elevation,
                    epoch.time.tow, nav.ionosphere_model.alpha,
                    nav.ionosphere_model.beta);
                if (frequency_hz > 0.0) {
                    const double scale = constants::GPS_L1_FREQ / frequency_hz;
                    ionosphere_delay *= scale * scale;
                }
            }
            const double troposphere_delay =
                config.use_troposphere_model
                    ? models::tropDelaySaastamoinen(
                          reference_position,
                          config.source_complete ? source_geometry.elevation_rad
                                                 : legacy_geometry.elevation)
                    : 0.0;
            if (config.source_complete &&
                (!std::isfinite(ionosphere_delay) ||
                 !std::isfinite(troposphere_delay))) {
                diagnostics_.failure =
                    "source-complete atmosphere model is non-finite";
                return false;
            }
            const double satellite_clock_m =
                satellite_clock_bias * constants::SPEED_OF_LIGHT;
            // Gobs.resPc deliberately excludes explicit TGD/BGD.  In the
            // source-complete branch this zero policy is symmetric with the
            // rover FGO pseudorange path; the legacy native group-delay
            // operator remains untouched when the selector is off.
            const double group_delay_m =
                config.source_complete
                    ? phase126_raw_base::officialExplicitCodeBiasMeters()
                    : groupDelayCorrectionMeters(
                          observation, *ephemeris,
                          config.use_signal_specific_galileo_group_delay);
            const double residual =
                config.source_complete
                    ? phase126_raw_base::officialBaseCodeResidual(
                          observation.pseudorange, satellite_clock_m,
                          geometric_range, ionosphere_delay, troposphere_delay)
                    : observation.pseudorange + satellite_clock_m -
                          ionosphere_delay - troposphere_delay - group_delay_m -
                          geometric_range;
            if (!std::isfinite(residual)) {
                if (config.source_complete) {
                    diagnostics_.failure =
                        "source-complete base residual is non-finite";
                    return false;
                }
                continue;
            }
            ++diagnostics_.finite_base_residual_rows;
            if (config.source_complete) ++diagnostics_.source_complete_signal_rows;
            candidate_streams[{observation.satellite, observation.signal}].push_back(
                {epoch.time, residual});
            if (config.use_phase131_canonical_correction_band_key) {
                canonical_candidates[canonicalization.key][observation.signal]
                    .push_back({epoch.time, residual});
            }
        }
    }
    if (config.use_phase129_glonass_local_miss_mask) {
        diagnostics_.phase129_glonass_local_miss_streams =
            local_miss_stream_keys.size();
        diagnostics_.phase129_glonass_row_count_consistent =
            phase129_glonass_local_miss::rowLedgerConsistent(
                diagnostics_.phase127_glonass_rows,
                diagnostics_.phase127_accepted_rows,
                diagnostics_.phase129_glonass_local_miss_rows);
        if (!diagnostics_.phase129_glonass_row_count_consistent) {
            diagnostics_.phase129_configuration_valid = false;
            diagnostics_.phase129_configuration_failure =
                "Phase129 GLONASS row ledger is inconsistent";
            diagnostics_.failure = diagnostics_.phase129_configuration_failure;
            candidate_streams.clear();
            return false;
        }
    }
    // The published MATLAB path selects obsb.dt, which is a route-level
    // sampling interval rather than a requirement that every adjacent epoch
    // be present.  Use the observed median and retain strictly-positive
    // monotonicity above; isolated gaps therefore do not silently change the
    // 1-Hz/15-s window contract.
    if (!observed_intervals.empty()) {
        diagnostics_.base_interval_s = median(observed_intervals);
    }
    if (diagnostics_.base_epochs > 1U &&
        (!std::isfinite(diagnostics_.base_interval_s) ||
         std::abs(diagnostics_.base_interval_s - config.expected_interval_s) > 1.0e-6)) {
        diagnostics_.failure = "observed base interval differs from frozen dt";
        return false;
    }
    diagnostics_.base_interval_s = config.expected_interval_s;
    std::vector<double> absolute_corrections;
    for (auto& [key, samples] : candidate_streams) {
        std::sort(samples.begin(), samples.end(),
                  [](const Sample& lhs, const Sample& rhs) {
                      return lhs.time < rhs.time;
                  });
        if (samples.empty()) continue;
        if (config.source_complete &&
            std::adjacent_find(
                samples.begin(), samples.end(),
                [](const Sample& lhs, const Sample& rhs) {
                    return !(rhs.time > lhs.time);
                }) != samples.end()) {
            diagnostics_.failure =
                "source-complete correction stream has duplicate/non-monotonic time";
            streams_.clear();
            return false;
        }
        ++diagnostics_.matching_streams;
        if (config.use_dense_epoch_smoothing) {
            // Window width counts base epochs, not only finite observations.
            // Retain NaNs after smoothing as interpolation barriers as well.
            std::vector<Sample> dense;
            dense.reserve(base_epochs.epochs.size());
            std::size_t index = 0;
            for (const auto& epoch : base_epochs.epochs) {
                double residual = std::numeric_limits<double>::quiet_NaN();
                if (index < samples.size() && samples[index].time == epoch.time) {
                    residual = samples[index++].residual_m;
                }
                dense.push_back({epoch.time, residual});
            }
            if (index != samples.size()) {
                diagnostics_.failure = "correction sample absent from base epoch grid";
                return false;
            }
            samples.swap(dense);
        }
        std::vector<double> values;
        values.reserve(samples.size());
        for (const auto& sample : samples) values.push_back(sample.residual_m);
        const std::vector<double> smoothed = centeredMovingMean(
            values, config.moving_mean_samples);
        for (std::size_t index = 0U; index < samples.size(); ++index) {
            if (config.use_dense_epoch_smoothing) samples[index].residual_m = smoothed[index];
            if (!std::isfinite(smoothed[index])) continue;
            samples[index].residual_m = smoothed[index];
            ++diagnostics_.smoothed_rows;
            absolute_corrections.push_back(std::abs(smoothed[index]));
        }
    }
    if (config.use_phase131_canonical_correction_band_key) {
        // Collapse only after the source typed streams have been built.  A
        // physical family may have more than one declared tracking code; the
        // source-defined SignalType priority selects one stream, while an
        // equal-priority ambiguity is a hard failure.  No first/last/nearest
        // sample is ever selected implicitly.
        for (const auto& [canonical_key, by_signal] : canonical_candidates) {
            diagnostics_.phase131_canonical_streams += by_signal.size();
            if (by_signal.empty()) continue;
            auto selected = by_signal.begin();
            int selected_priority = phase131_canonical::sourcePriority(
                canonical_key.satellite.system, selected->first);
            bool ambiguous = false;
            for (auto candidate = std::next(by_signal.begin());
                 candidate != by_signal.end(); ++candidate) {
                const int candidate_priority =
                    phase131_canonical::sourcePriority(
                        canonical_key.satellite.system, candidate->first);
                if (candidate_priority < selected_priority) {
                    selected = candidate;
                    selected_priority = candidate_priority;
                    ambiguous = false;
                } else if (candidate_priority == selected_priority) {
                    ambiguous = true;
                }
            }
            if (ambiguous) {
                ++diagnostics_.phase131_canonical_key_conflicts;
                ++diagnostics_.phase131_failure_counts[
                    "canonical-key-conflict"];
                diagnostics_.phase131_configuration_valid = false;
                diagnostics_.phase131_configuration_failure =
                    "ambiguous finite source streams share one canonical key";
                diagnostics_.failure =
                    diagnostics_.phase131_configuration_failure;
                candidate_streams.clear();
                canonical_streams_.clear();
                return false;
            }
            if (by_signal.size() > 1U) {
                diagnostics_.phase131_canonical_merged_streams +=
                    by_signal.size() - 1U;
            }
            std::vector<Sample> samples = selected->second;
            std::sort(samples.begin(), samples.end(),
                      [](const Sample& lhs, const Sample& rhs) {
                          return lhs.time < rhs.time;
                      });
            if (std::adjacent_find(
                    samples.begin(), samples.end(),
                    [](const Sample& lhs, const Sample& rhs) {
                        return !(rhs.time > lhs.time);
                    }) != samples.end()) {
                ++diagnostics_.phase131_canonical_duplicate_rows;
                ++diagnostics_.phase131_failure_counts[
                    "canonical-duplicate-or-nonmonotonic-time"];
                diagnostics_.phase131_configuration_valid = false;
                diagnostics_.phase131_configuration_failure =
                    "canonical correction stream has duplicate/non-monotonic time";
                diagnostics_.failure =
                    diagnostics_.phase131_configuration_failure;
                candidate_streams.clear();
                canonical_streams_.clear();
                return false;
            }
            if (samples.empty()) continue;
            std::vector<double> values;
            values.reserve(samples.size());
            for (const auto& sample : samples) values.push_back(sample.residual_m);
            const std::vector<double> smoothed = centeredMovingMean(
                values, config.moving_mean_samples);
            std::vector<Sample> selected_samples;
            selected_samples.reserve(samples.size());
            for (std::size_t index = 0U; index < samples.size(); ++index) {
                if (!std::isfinite(smoothed[index])) continue;
                selected_samples.push_back({samples[index].time, smoothed[index]});
            }
            if (!selected_samples.empty()) {
                canonical_streams_[canonical_key] = std::move(selected_samples);
                ++diagnostics_.phase131_canonical_selected_streams;
            }
        }
        if (diagnostics_.phase131_canonical_selected_streams == 0U) {
            diagnostics_.phase131_configuration_valid = false;
            diagnostics_.phase131_configuration_failure =
                "no finite canonical correction stream";
            diagnostics_.failure = diagnostics_.phase131_configuration_failure;
            candidate_streams.clear();
            canonical_streams_.clear();
            return false;
        }
    }
    diagnostics_.correction_abs_p50_m = median(absolute_corrections);
    diagnostics_.correction_abs_p95_m = percentile(absolute_corrections, 95.0);
    diagnostics_.correction_abs_max_m = absolute_corrections.empty()
                                            ? std::numeric_limits<double>::quiet_NaN()
                                            : *std::max_element(absolute_corrections.begin(), absolute_corrections.end());
    diagnostics_.built = diagnostics_.matching_streams != 0U &&
                         diagnostics_.smoothed_rows != 0U;
    if (!diagnostics_.built && diagnostics_.failure.empty()) {
        diagnostics_.failure = "no finite same-satellite/signal base residual stream";
    }
    if (diagnostics_.built) {
        streams_.swap(candidate_streams);
        if (!config.use_phase131_canonical_correction_band_key) {
            canonical_streams_.clear();
        }
    } else {
        streams_.clear();
        canonical_streams_.clear();
    }
    return diagnostics_.built;
}

bool Model::streamMedian(const SatelliteId& satellite, SignalType signal,
                         double& median_m) const {
    const auto found=streams_.find({satellite,signal});
    if (found==streams_.end()) return false;
    std::vector<double> values;
    for (const auto& sample : found->second)
        if (std::isfinite(sample.residual_m)) values.push_back(sample.residual_m);
    if (values.empty()) return false;
    std::sort(values.begin(),values.end());
    const auto n=values.size();
    median_m=n%2 ? values[n/2] : values[n/2-1]/2+values[n/2]/2;
    return true;
}

bool Model::correctionAt(const GNSSTime& time,
                         const SatelliteId& satellite,
                         SignalType signal,
                         double& correction_m) const {
    correction_m = std::numeric_limits<double>::quiet_NaN();
    const auto it = streams_.find({satellite, signal});
    if (it == streams_.end() || it->second.empty() || !finiteTime(time)) {
        return false;
    }
    const auto& samples = it->second;
    if (time < samples.front().time || time > samples.back().time) {
        return false;
    }
    const auto upper = std::lower_bound(
        samples.begin(), samples.end(), time,
        [](const Sample& sample, const GNSSTime& value) {
            return sample.time < value;
        });
    if (upper == samples.begin()) {
        correction_m = upper->residual_m;
        return std::isfinite(correction_m);
    }
    if (upper == samples.end()) {
        correction_m = samples.back().residual_m;
        return std::isfinite(correction_m);
    }
    const Sample& right = *upper;
    const Sample& left = *(upper - 1);
    // A finite exact sample after a NaN interval remains usable. Interior
    // queries touching that NaN must still return unavailable, not bridge it.
    if (!std::isfinite(left.residual_m) && time == right.time) {
        correction_m = right.residual_m;
        return std::isfinite(correction_m);
    }
    const double dt = right.time - left.time;
    if (!(dt > 0.0) || !std::isfinite(dt)) return false;
    const double fraction = (time - left.time) / dt;
    correction_m = left.residual_m + fraction * (right.residual_m - left.residual_m);
    return std::isfinite(correction_m);
}

bool Model::hasStream(const SatelliteId& satellite, SignalType signal) const {
    const auto it = streams_.find({satellite, signal});
    return it != streams_.end() && !it->second.empty();
}

bool Model::hasCanonicalStream(const SatelliteId& satellite,
                               SignalType signal,
                               bool has_glonass_frequency_channel,
                               int glonass_frequency_channel) const {
    const auto canonical = phase131_canonical::canonicalize(
        satellite, signal, has_glonass_frequency_channel,
        glonass_frequency_channel);
    if (!canonical.accepted) return false;
    const auto it = canonical_streams_.find(canonical.key);
    return it != canonical_streams_.end() && !it->second.empty();
}

bool Model::correctionAtCanonical(
    const GNSSTime& time,
    const SatelliteId& satellite,
    SignalType signal,
    bool has_glonass_frequency_channel,
    int glonass_frequency_channel,
    double& correction_m) const {
    correction_m = std::numeric_limits<double>::quiet_NaN();
    const auto canonical = phase131_canonical::canonicalize(
        satellite, signal, has_glonass_frequency_channel,
        glonass_frequency_channel);
    if (!canonical.accepted || !finiteTime(time)) return false;
    const auto it = canonical_streams_.find(canonical.key);
    if (it == canonical_streams_.end() || it->second.empty()) return false;
    const auto& samples = it->second;
    if (time < samples.front().time || time > samples.back().time) return false;
    const auto upper = std::lower_bound(
        samples.begin(), samples.end(), time,
        [](const Sample& sample, const GNSSTime& value) {
            return sample.time < value;
        });
    if (upper == samples.begin()) {
        correction_m = upper->residual_m;
        return std::isfinite(correction_m);
    }
    if (upper == samples.end()) {
        correction_m = samples.back().residual_m;
        return std::isfinite(correction_m);
    }
    const Sample& right = *upper;
    const Sample& left = *(upper - 1);
    const double dt = right.time - left.time;
    if (!(dt > 0.0) || !std::isfinite(dt)) return false;
    const double fraction = (time - left.time) / dt;
    correction_m = left.residual_m + fraction * (right.residual_m - left.residual_m);
    return std::isfinite(correction_m);
}

}  // namespace libgnss::base_pseudorange_compensation
