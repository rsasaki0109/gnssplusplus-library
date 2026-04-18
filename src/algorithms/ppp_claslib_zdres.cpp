#include <libgnss++/algorithms/ppp_claslib_zdres.hpp>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

namespace libgnss::ppp_claslib_zdres {

namespace {

using ppp_clas::MeasurementBuildResult;
using ppp_clas::MeasurementRow;

constexpr int kMaxClasFreq = OSR_MAX_FREQ;

double elevationWeight(double elevation_rad) {
    const double sin_el = std::max(std::sin(elevation_rad), 1e-3);
    return 1.0 / (sin_el * sin_el);
}

double clasCodeVarianceForRow(GNSSSystem system, double elevation_rad) {
    constexpr double kCodePhaseRatio = 50.0;
    constexpr double kPhaseSigmaM = 0.01;
    constexpr double kPhaseElevationSigmaM = 0.005;
    const double system_factor = system == GNSSSystem::GLONASS ? 1.5 : 1.0;
    const double a = system_factor * kCodePhaseRatio * kPhaseSigmaM;
    const double b = system_factor * kCodePhaseRatio * kPhaseElevationSigmaM;
    return a * a + b * b * elevationWeight(elevation_rad);
}

double clasPhaseVarianceForRow(GNSSSystem system, double elevation_rad, int freq_index) {
    constexpr double kPhaseSigmaM = 0.01;
    constexpr double kPhaseElevationSigmaM = 0.005;
    const double system_factor = system == GNSSSystem::GLONASS ? 1.5 : 1.0;
    const double a = system_factor * kPhaseSigmaM;
    const double b = system_factor * kPhaseElevationSigmaM;
    double variance = a * a + b * b * elevationWeight(elevation_rad);
    if (freq_index == 1) {
        variance *= std::pow(2.55 / 1.55, 2);
    }
    return variance;
}

double claslibIonoMappingFunction(const Vector3d& receiver_position, double elevation_rad) {
    constexpr double kIonosphereHeightM = 350000.0;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver_position, lat, lon, height);
    (void)lat;
    (void)lon;
    if (height >= kIonosphereHeightM) {
        return 1.0;
    }
    const double ratio =
        (constants::WGS84_A + height) /
        (constants::WGS84_A + kIonosphereHeightM);
    const double arg = std::clamp(ratio * std::cos(elevation_rad), -1.0, 1.0);
    const double denom = std::cos(std::asin(arg));
    return denom > 0.0 ? 1.0 / denom : 1.0;
}

int clasSystemOrder(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::GLONASS: return 1;
        case GNSSSystem::Galileo: return 2;
        case GNSSSystem::BeiDou: return 3;
        case GNSSSystem::QZSS: return 4;
        default: return 6;
    }
}

int clasResidualGroup(const OSRCorrection& osr, int freq_index) {
    if (osr.satellite.system == GNSSSystem::GPS && freq_index == 1 &&
        osr.signals[freq_index] == SignalType::GPS_L2C) {
        return 5;
    }
    return clasSystemOrder(osr.satellite.system);
}

SatelliteId ambiguitySatelliteForFreq(const SatelliteId& satellite, int freq_index) {
    if (freq_index == 0) {
        return satellite;
    }
    return SatelliteId(
        satellite.system,
        static_cast<uint8_t>(std::min(255, static_cast<int>(satellite.prn) + 100)));
}

bool shouldDumpClasTime(const GNSSTime& time) {
    return time.week == 2068 &&
           ((time.tow >= 230420.0 - 1e-6 && time.tow <= 230425.0 + 1e-6) ||
            std::abs(time.tow - 230620.0) <= 1e-6 ||
            std::abs(time.tow - 231420.0) <= 1e-6 ||
            std::abs(time.tow - 232419.0) <= 1e-6);
}

bool shouldDumpSat(const SatelliteId& satellite) {
    const int prn = satellite.prn > 100 ? satellite.prn - 100 : satellite.prn;
    if (satellite.system == GNSSSystem::GPS) {
        return prn == 14 || prn == 25 || prn == 26 ||
               prn == 29 || prn == 31 || prn == 32;
    }
    if (satellite.system == GNSSSystem::Galileo) {
        return prn == 7 || prn == 27 || prn == 30;
    }
    return satellite.system == GNSSSystem::QZSS &&
           (prn == 1 || prn == 2 || prn == 3);
}

double finiteOrZero(double value) {
    return std::isfinite(value) ? value : 0.0;
}

struct AppliedCorrections {
    double code_m = 0.0;
    double phase_m = 0.0;
};

AppliedCorrections selectAppliedCorrections(
    const OSRCorrection& osr,
    int freq_index,
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    AppliedCorrections corrections;
    if (freq_index < 0 || freq_index >= osr.num_frequencies) {
        return corrections;
    }

    switch (policy) {
        case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR:
            corrections.code_m = osr.PRC[freq_index] - osr.trop_correction_m;
            corrections.phase_m = osr.CPC[freq_index] - osr.trop_correction_m;
            break;
        case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS:
            corrections.code_m =
                osr.relativity_correction_m +
                osr.receiver_antenna_m[freq_index] +
                osr.code_bias_m[freq_index] +
                osr.solid_earth_tide_m;
            corrections.phase_m =
                osr.relativity_correction_m +
                osr.receiver_antenna_m[freq_index] +
                osr.phase_bias_m[freq_index] +
                osr.windup_m[freq_index] +
                osr.phase_compensation_m[freq_index] +
                osr.solid_earth_tide_m;
            break;
        case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY:
            corrections.code_m =
                osr.relativity_correction_m +
                osr.receiver_antenna_m[freq_index] +
                osr.solid_earth_tide_m;
            corrections.phase_m = corrections.code_m;
            break;
    }
    return corrections;
}

MeasurementRow::ModelComponents subtractModelComponents(
    const MeasurementRow::ModelComponents& lhs,
    const MeasurementRow::ModelComponents& rhs) {
    MeasurementRow::ModelComponents out;
    out.valid = lhs.valid && rhs.valid;
    auto sub = [](double a, double b) {
        return std::isfinite(a) && std::isfinite(b)
                   ? a - b
                   : std::numeric_limits<double>::quiet_NaN();
    };
    out.y = sub(lhs.y, rhs.y);
    out.obs = sub(lhs.obs, rhs.obs);
    out.rho = sub(lhs.rho, rhs.rho);
    out.dts = sub(lhs.dts, rhs.dts);
    out.dtr = sub(lhs.dtr, rhs.dtr);
    out.rel = sub(lhs.rel, rhs.rel);
    out.sagnac = sub(lhs.sagnac, rhs.sagnac);
    out.trop_dry = sub(lhs.trop_dry, rhs.trop_dry);
    out.trop_wet = sub(lhs.trop_wet, rhs.trop_wet);
    out.trop_model = sub(lhs.trop_model, rhs.trop_model);
    out.trop_grid = sub(lhs.trop_grid, rhs.trop_grid);
    out.iono_grid = sub(lhs.iono_grid, rhs.iono_grid);
    out.iono_state_term = sub(lhs.iono_state_term, rhs.iono_state_term);
    out.prc = sub(lhs.prc, rhs.prc);
    out.cpc = sub(lhs.cpc, rhs.cpc);
    out.osr_corr = sub(lhs.osr_corr, rhs.osr_corr);
    out.code_bias = sub(lhs.code_bias, rhs.code_bias);
    out.phase_bias = sub(lhs.phase_bias, rhs.phase_bias);
    out.phase_comp = sub(lhs.phase_comp, rhs.phase_comp);
    out.windup = sub(lhs.windup, rhs.windup);
    out.pco_rcv = sub(lhs.pco_rcv, rhs.pco_rcv);
    out.pco_sat = sub(lhs.pco_sat, rhs.pco_sat);
    out.pcv_rcv = sub(lhs.pcv_rcv, rhs.pcv_rcv);
    out.pcv_sat = sub(lhs.pcv_sat, rhs.pcv_sat);
    out.tide_solid = sub(lhs.tide_solid, rhs.tide_solid);
    out.tide_ocean = sub(lhs.tide_ocean, rhs.tide_ocean);
    out.tide_pole = sub(lhs.tide_pole, rhs.tide_pole);
    out.amb_m = sub(lhs.amb_m, rhs.amb_m);
    out.modeled_sum = sub(lhs.modeled_sum, rhs.modeled_sum);
    return out;
}

struct ZdRow {
    const OSRCorrection* osr = nullptr;
    SatelliteId satellite;
    SatelliteId ambiguity_satellite;
    int freq_index = 0;
    int group = 0;
    bool is_phase = false;
    bool valid = false;
    double y = 0.0;
    double variance = 0.0;
    Eigen::RowVectorXd H_base;
    MeasurementRow::ModelComponents components;
};

bool rowLessClasOrder(const ZdRow& lhs, const ZdRow& rhs) {
    if (lhs.group != rhs.group) return lhs.group < rhs.group;
    if (lhs.freq_index != rhs.freq_index) return lhs.freq_index < rhs.freq_index;
    if (lhs.is_phase != rhs.is_phase) return lhs.is_phase && !rhs.is_phase;
    if (lhs.satellite.system != rhs.satellite.system) {
        return lhs.satellite.system < rhs.satellite.system;
    }
    return lhs.satellite.prn < rhs.satellite.prn;
}

void dumpZdRow(const GNSSTime& time, const ZdRow& row) {
    if (!ppp_shared::pppDebugEnabled() || !shouldDumpClasTime(time) ||
        !shouldDumpSat(row.satellite)) {
        return;
    }
    std::cerr << "[CLAS-ZDRES] source=lib port=ported_zdres"
              << " week=" << time.week
              << " tow=" << time.tow
              << " sat=" << row.satellite.toString()
              << " f=" << row.freq_index
              << " freq_group=" << row.freq_index
              << " type=" << (row.is_phase ? "phase" : "code")
              << std::setprecision(12)
              << " y=" << row.y
              << " obs=" << row.components.obs
              << " model=" << row.components.modeled_sum
              << " rho=" << row.components.rho
              << " dts=" << row.components.dts
              << " sagnac=" << row.components.sagnac
              << " trop_grid=" << row.components.trop_grid
              << " iono_grid=" << row.components.iono_grid
              << " prc=" << row.components.prc
              << " cpc=" << row.components.cpc
              << " osr_corr=" << row.components.osr_corr
              << " windup=" << row.components.windup
              << " ant_rcv=" << row.components.pcv_rcv
              << " tide=" << row.components.tide_solid
              << "\n";
}

std::vector<ZdRow> buildZeroDifferenceRows(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    double trop_zenith_m,
    const TropMappingFunction& trop_mapping_function) {
    std::vector<ZdRow> rows;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }

        const double rho_sagnac = geodist(osr.satellite_position, receiver_position);
        const double rho_no_sagnac =
            (osr.satellite_position - receiver_position).norm();
        if (rho_sagnac <= 0.0 || rho_no_sagnac <= 0.0) {
            continue;
        }
        const Vector3d los =
            (osr.satellite_position - receiver_position).normalized();
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_modeled =
            std::isfinite(trop_zenith_m) ? trop_mapping * trop_zenith_m : 0.0;

        for (int f = 0; f < osr.num_frequencies && f < kMaxClasFreq; ++f) {
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid ||
                !raw->has_carrier_phase || !raw->has_pseudorange ||
                !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange) ||
                raw->carrier_phase == 0.0 || raw->pseudorange == 0.0 ||
                osr.wavelengths[f] <= 0.0) {
                continue;
            }

            const double fi =
                osr.wavelengths[0] > 0.0 ? osr.wavelengths[f] / osr.wavelengths[0] : 1.0;
            const double iono_scaled =
                fi * fi * (constants::GPS_L2_FREQ / constants::GPS_L1_FREQ) *
                osr.iono_l1_m;
            const double tide_component =
                osr.tide_geometry_m != 0.0 ? osr.tide_geometry_m : osr.solid_earth_tide_m;
            const AppliedCorrections applied =
                selectAppliedCorrections(
                    osr, f, config.clas_correction_application_policy);

            Eigen::RowVectorXd h_base =
                Eigen::RowVectorXd::Zero(filter_state.total_states);
            h_base.segment(0, 3) = -los.transpose();
            if (config.estimate_troposphere &&
                filter_state.trop_index >= 0 &&
                filter_state.trop_index < filter_state.total_states) {
                h_base(filter_state.trop_index) = trop_mapping;
            }

            auto make_components = [&](bool is_phase, double obs_m, double model_m) {
                MeasurementRow::ModelComponents c;
                c.valid = true;
                c.y = obs_m - model_m;
                c.obs = obs_m;
                c.rho = rho_no_sagnac;
                c.dts = sat_clk_m;
                c.rel = osr.relativity_correction_m;
                c.sagnac = rho_sagnac - rho_no_sagnac;
                c.trop_dry = trop_modeled;
                c.trop_model = trop_modeled;
                c.trop_grid = osr.trop_correction_m;
                c.iono_grid = is_phase ? -iono_scaled : iono_scaled;
                c.prc = is_phase ? 0.0 : osr.PRC[f];
                c.cpc = is_phase ? osr.CPC[f] : 0.0;
                c.osr_corr = is_phase ? applied.phase_m : applied.code_m;
                c.code_bias = is_phase ? 0.0 : osr.code_bias_m[f];
                c.phase_bias = is_phase ? osr.phase_bias_m[f] : 0.0;
                c.phase_comp = is_phase ? osr.phase_compensation_m[f] : 0.0;
                c.windup = is_phase ? osr.windup_m[f] : 0.0;
                c.pcv_rcv = osr.receiver_antenna_m[f];
                c.tide_solid = tide_component;
                c.modeled_sum = model_m;
                return c;
            };

            const double phase_obs_m = raw->carrier_phase * osr.wavelengths[f];
            const double phase_model_m =
                rho_sagnac - sat_clk_m + trop_modeled + applied.phase_m;
            ZdRow phase_row;
            phase_row.osr = &osr;
            phase_row.satellite = osr.satellite;
            phase_row.ambiguity_satellite = ambiguitySatelliteForFreq(osr.satellite, f);
            phase_row.freq_index = f;
            phase_row.group = clasResidualGroup(osr, f);
            phase_row.is_phase = true;
            phase_row.valid = true;
            phase_row.y = phase_obs_m - phase_model_m;
            phase_row.variance =
                clasPhaseVarianceForRow(osr.satellite.system, osr.elevation, f);
            phase_row.H_base = h_base;
            phase_row.components = make_components(true, phase_obs_m, phase_model_m);
            rows.push_back(phase_row);
            dumpZdRow(obs.time, phase_row);

            const double code_model_m =
                rho_sagnac - sat_clk_m + trop_modeled + applied.code_m;
            ZdRow code_row;
            code_row.osr = &osr;
            code_row.satellite = osr.satellite;
            code_row.ambiguity_satellite = SatelliteId{};
            code_row.freq_index = f;
            code_row.group = clasResidualGroup(osr, f);
            code_row.is_phase = false;
            code_row.valid = true;
            code_row.y = raw->pseudorange - code_model_m;
            code_row.variance =
                clasCodeVarianceForRow(osr.satellite.system, osr.elevation);
            code_row.H_base = h_base;
            code_row.components = make_components(false, raw->pseudorange, code_model_m);
            rows.push_back(code_row);
            dumpZdRow(obs.time, code_row);
        }
    }
    std::sort(rows.begin(), rows.end(), rowLessClasOrder);
    return rows;
}

void seedMissingAmbiguitiesFromPhaseCode(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config) {
    if (config.use_ported_udstate) {
        return;
    }
    std::array<double, OSR_MAX_FREQ> bias_sum{};
    std::array<int, OSR_MAX_FREQ> bias_count{};
    std::map<SatelliteId, double> raw_bias_m;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) continue;
        for (int f = 0; f < osr.num_frequencies && f < OSR_MAX_FREQ; ++f) {
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid || !raw->has_carrier_phase ||
                !raw->has_pseudorange || osr.wavelengths[f] <= 0.0 ||
                !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange)) {
                continue;
            }
            const SatelliteId amb_sat = ambiguitySatelliteForFreq(osr.satellite, f);
            const double bias = raw->carrier_phase * osr.wavelengths[f] -
                                raw->pseudorange;
            raw_bias_m[amb_sat] = bias;
            const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
            if (amb_it == filter_state.ambiguity_indices.end()) {
                continue;
            }
            const int amb_idx = amb_it->second;
            if (amb_idx < 0 || amb_idx >= filter_state.total_states ||
                filter_state.state(amb_idx) == 0.0 ||
                !std::isfinite(filter_state.state(amb_idx))) {
                continue;
            }
            bias_sum[static_cast<size_t>(f)] += bias - filter_state.state(amb_idx);
            bias_count[static_cast<size_t>(f)]++;
        }
    }

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) continue;
        for (int f = 0; f < osr.num_frequencies && f < OSR_MAX_FREQ; ++f) {
            const SatelliteId amb_sat = ambiguitySatelliteForFreq(osr.satellite, f);
            const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
            const auto bias_it = raw_bias_m.find(amb_sat);
            if (amb_it == filter_state.ambiguity_indices.end() ||
                bias_it == raw_bias_m.end()) {
                continue;
            }
            const int amb_idx = amb_it->second;
            if (amb_idx < 0 || amb_idx >= filter_state.total_states ||
                (std::isfinite(filter_state.state(amb_idx)) &&
                 filter_state.state(amb_idx) != 0.0 &&
                 filter_state.covariance(amb_idx, amb_idx) <
                     config.clas_ambiguity_reinit_threshold)) {
                continue;
            }
            const int count = bias_count[static_cast<size_t>(f)];
            const double common_bias =
                count > 0 ? bias_sum[static_cast<size_t>(f)] / count : 0.0;
            filter_state.state(amb_idx) = bias_it->second - common_bias;
            if (osr.wavelengths[f] > 0.0) {
                filter_state.covariance(amb_idx, amb_idx) =
                    std::pow(100.0 * osr.wavelengths[f], 2);
            }
        }
    }
}

std::vector<MeasurementRow> formSingleDifferenceRows(
    const ObservationData& obs,
    const std::vector<ZdRow>& zd_rows,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    const AmbiguityResetFunction& ambiguity_reset_function,
    MeasurementBuildResult& result) {
    std::vector<MeasurementRow> sd_rows;

    struct GroupKey {
        int group = 0;
        int freq_index = 0;
        bool is_phase = false;
        bool operator<(const GroupKey& rhs) const {
            if (group != rhs.group) return group < rhs.group;
            if (freq_index != rhs.freq_index) return freq_index < rhs.freq_index;
            return is_phase > rhs.is_phase;
        }
    };

    std::map<GroupKey, std::vector<const ZdRow*>> groups;
    for (const ZdRow& row : zd_rows) {
        if (!row.valid) continue;
        groups[{row.group, row.freq_index, row.is_phase}].push_back(&row);
    }

    for (auto& [key, members] : groups) {
        if (members.size() < 2) {
            continue;
        }

        const ZdRow* ref = nullptr;
        for (const ZdRow* row : members) {
            if (ref == nullptr ||
                row->osr->elevation >= ref->osr->elevation) {
                ref = row;
            }
        }
        if (ref == nullptr) {
            continue;
        }

        for (const ZdRow* sat : members) {
            if (sat == ref) {
                continue;
            }

            MeasurementRow row;
            row.H = ref->H_base - sat->H_base;
            row.residual = ref->y - sat->y;
            row.variance = ref->variance + sat->variance;
            row.reference_variance = ref->variance;
            row.satellite = sat->satellite;
            row.ambiguity_satellite =
                key.is_phase ? sat->ambiguity_satellite : SatelliteId{};
            row.reference_satellite =
                key.is_phase ? ref->ambiguity_satellite : ref->satellite;
            row.is_phase = key.is_phase;
            row.freq_index = key.freq_index;
            row.components =
                subtractModelComponents(ref->components, sat->components);

            if (config.estimate_ionosphere) {
                const double ref_fi =
                    ref->osr->wavelengths[key.freq_index] / constants::GPS_L1_WAVELENGTH;
                const double sat_fi =
                    sat->osr->wavelengths[key.freq_index] / constants::GPS_L1_WAVELENGTH;
                const double sign = key.is_phase ? -1.0 : 1.0;
                const double ref_coeff =
                    sign * ref_fi * ref_fi *
                    claslibIonoMappingFunction(receiver_position, ref->osr->elevation);
                const double sat_coeff =
                    sign * sat_fi * sat_fi *
                    claslibIonoMappingFunction(receiver_position, sat->osr->elevation);
                const auto ref_iono_it =
                    filter_state.ionosphere_indices.find(ref->satellite);
                const auto sat_iono_it =
                    filter_state.ionosphere_indices.find(sat->satellite);
                const bool ref_has_iono =
                    ref_iono_it != filter_state.ionosphere_indices.end() &&
                    ref_iono_it->second >= 0 &&
                    ref_iono_it->second < filter_state.total_states;
                const bool sat_has_iono =
                    sat_iono_it != filter_state.ionosphere_indices.end() &&
                    sat_iono_it->second >= 0 &&
                    sat_iono_it->second < filter_state.total_states;
                const double ref_state =
                    ref_has_iono ? filter_state.state(ref_iono_it->second) : 0.0;
                const double sat_state =
                    sat_has_iono ? filter_state.state(sat_iono_it->second) : 0.0;
                const double iono_state_term =
                    ref_coeff * ref_state - sat_coeff * sat_state;
                row.residual -= iono_state_term;
                row.components.iono_state_term =
                    finiteOrZero(row.components.iono_state_term) + iono_state_term;
                if (ref_has_iono) {
                    row.H(ref_iono_it->second) = ref_coeff;
                    result.observed_iono_states.insert(ref->satellite);
                }
                if (sat_has_iono) {
                    row.H(sat_iono_it->second) = -sat_coeff;
                    result.observed_iono_states.insert(sat->satellite);
                }
            }

            if (key.is_phase) {
                const auto ref_amb_it =
                    filter_state.ambiguity_indices.find(ref->ambiguity_satellite);
                const auto sat_amb_it =
                    filter_state.ambiguity_indices.find(sat->ambiguity_satellite);
                const bool ref_has_amb =
                    ref_amb_it != filter_state.ambiguity_indices.end() &&
                    ref_amb_it->second >= 0 &&
                    ref_amb_it->second < filter_state.total_states;
                const bool sat_has_amb =
                    sat_amb_it != filter_state.ambiguity_indices.end() &&
                    sat_amb_it->second >= 0 &&
                    sat_amb_it->second < filter_state.total_states;
                if (!sat_has_amb) {
                    continue;
                }
                const double ref_amb_state =
                    ref_has_amb ? filter_state.state(ref_amb_it->second) : 0.0;
                const double sat_amb_state =
                    filter_state.state(sat_amb_it->second);
                const double amb_state_term = ref_amb_state - sat_amb_state;
                row.residual -= amb_state_term;
                if (ref_has_amb) {
                    row.H(ref_amb_it->second) = 1.0;
                }
                row.H(sat_amb_it->second) = -1.0;
                row.components.amb_m =
                    finiteOrZero(row.components.amb_m) + amb_state_term;

                const Observation* raw =
                    obs.getObservation(sat->satellite, sat->osr->signals[key.freq_index]);
                if (raw && raw->loss_of_lock && ambiguity_reset_function) {
                    ambiguity_reset_function(sat->ambiguity_satellite, raw->signal);
                }
                if (raw) {
                    result.observed_ambiguities.push_back({
                        sat->ambiguity_satellite,
                        raw->signal,
                        sat->osr->wavelengths[key.freq_index],
                        raw->carrier_phase,
                        raw->snr});
                }
                const Observation* ref_raw =
                    obs.getObservation(ref->satellite, ref->osr->signals[key.freq_index]);
                if (ref_raw) {
                    result.observed_ambiguities.push_back({
                        ref->ambiguity_satellite,
                        ref_raw->signal,
                        ref->osr->wavelengths[key.freq_index],
                        ref_raw->carrier_phase,
                        ref_raw->snr});
                }
            }

            row.components.y = row.residual;
            row.components.modeled_sum =
                finiteOrZero(row.components.obs) - row.residual;
            sd_rows.push_back(row);
        }
    }

    return sd_rows;
}

}  // namespace

MeasurementBuildResult buildEpochMeasurementsPortedZdres(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function,
    bool debug_enabled) {
    (void)debug_enabled;
    MeasurementBuildResult result;
    seedMissingAmbiguitiesFromPhaseCode(
        obs, epoch_context.osr_corrections, filter_state, config);
    const std::vector<ZdRow> zd_rows =
        buildZeroDifferenceRows(
            obs,
            epoch_context.osr_corrections,
            filter_state,
            config,
            epoch_context.receiver_position,
            epoch_context.trop_zenith_m,
            trop_mapping_function);
    result.measurements =
        formSingleDifferenceRows(
            obs,
            zd_rows,
            filter_state,
            config,
            epoch_context.receiver_position,
            ambiguity_reset_function,
            result);
    return result;
}

}  // namespace libgnss::ppp_claslib_zdres
