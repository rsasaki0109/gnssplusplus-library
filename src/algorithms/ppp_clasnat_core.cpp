#include <libgnss++/algorithms/ppp_clasnat_core.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_claslib_pntpos.hpp>
#include <libgnss++/algorithms/ppp_clasnat_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <vector>

namespace libgnss::ppp_clasnat_core {

namespace {

constexpr double kIonoInitM = 1e-6;
constexpr double kIonoInitVariance = 0.010 * 0.010;
constexpr double kIonoProcessMPerSqrtS = 1e-3;
constexpr double kIonoAdaptiveProcessMaxM = 0.050;
constexpr double kIonoAdaptiveForget = 0.3;
constexpr double kIonoAdaptiveGain = 3.0;
constexpr double kIonoTimeConstantS = 10.0;
constexpr double kBiasInitStdCycles = 100.0;
constexpr double kBiasProcessCyclesPerSqrtS = 1e-3;
constexpr double kClasnatPositionInitVariance = 30.0 * 30.0;
constexpr int kMinLockForAr = 1;
constexpr int kMaxAmbOutage = 90;
constexpr double kElevationMaskRad = 15.0 * M_PI / 180.0;
constexpr int kResidualFreqCount = 2;
constexpr double kResidualRejectSigma = 2.0;
constexpr double kDispersiveRejectSigma = 3.0;
constexpr double kNonDispersiveRejectSigma = 3.0;

struct AppliedCorrections {
    double code_m = 0.0;
    double phase_m = 0.0;
};

struct ZdRow {
    const OSRCorrection* osr = nullptr;
    SatelliteId satellite;
    SatelliteId ambiguity_satellite;
    int signal_index = 0;
    int freq_index = 0;
    int group = 0;
    bool is_phase = false;
    bool valid = false;
    double y = 0.0;
    double variance = 0.0;
    Eigen::RowVectorXd H_base;
    ppp_clas::MeasurementRow::ModelComponents components;
};

struct FullMeasurementRow {
    Eigen::RowVectorXd H;
    double residual = 0.0;
    double variance = 0.0;
    double reference_variance = 0.0;
    SatelliteId satellite;
    SatelliteId ambiguity_satellite;
    SatelliteId reference_satellite;
    bool is_phase = false;
    int signal_index = 0;
    int freq_index = 0;
    ppp_clas::MeasurementRow::ModelComponents components;
};

struct MeasurementBuild {
    std::vector<FullMeasurementRow> rows;
    std::set<SatelliteId> observed_ambiguities;
};

struct UpdateStats {
    bool updated = false;
    int nobs = 0;
    int active_states = 0;
    VectorXd residuals;
    VectorXd variances;
    VectorXd state_before;
    VectorXd state_after;
    MatrixXd pre_update_covariance;
    std::vector<int> selected_rows;
};

// -----------------------------------------------------------------------------
// CLASNAT configuration and CLASLIB-compatible scalar helpers
// -----------------------------------------------------------------------------

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

// CLASLIB correspondence: ionmapf() in rtkcmn.c.
double clasnatIonoMappingFunction(const Vector3d& receiver_position, double elevation_rad) {
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

int dayOfYearFromTime(const GNSSTime& time) {
    const auto system_time = time.toSystemTime();
    const std::time_t utc_seconds = std::chrono::system_clock::to_time_t(system_time);
    std::tm utc_tm{};
    gmtime_r(&utc_seconds, &utc_tm);
    return utc_tm.tm_yday + 1;
}

double modeledZenithTroposphereDelayMeters(const Vector3d& receiver_position,
                                           const GNSSTime& time) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    const auto delay = models::estimateZenithTroposphereClimatology(
        latitude_rad, height_m, dayOfYearFromTime(time));
    const double total_delay_m = delay.totalDelayMeters();
    return std::isfinite(total_delay_m) && total_delay_m > 0.0
               ? total_delay_m
               : 2.3;
}

// CLASLIB correspondence: tropmapf() used by prectrop2() in ppprtk.c.
double clasnatTropMappingFunction(const Vector3d& receiver_position,
                                  double elevation_rad,
                                  const GNSSTime& time) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    return models::niellHydrostaticMapping(
        latitude_rad, height_m, elevation_rad, dayOfYearFromTime(time));
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
    (void)freq_index;
    return clasSystemOrder(osr.satellite.system);
}

int clasSatNo(const SatelliteId& satellite) {
    const int prn = static_cast<int>(satellite.prn);
    switch (satellite.system) {
        case GNSSSystem::GPS:
            return prn >= 1 && prn <= 32 ? prn : 0;
        case GNSSSystem::Galileo:
            return prn >= 1 && prn <= 36 ? 32 + prn : 0;
        case GNSSSystem::QZSS: {
            const int qprn = prn >= 193 ? prn - 192 : prn;
            return qprn >= 1 && qprn <= 7 ? 68 + qprn : 0;
        }
        default:
            return 0;
    }
}

SatelliteId realSatelliteForAmbiguity(const SatelliteId& satellite) {
    if (satellite.system == GNSSSystem::QZSS &&
        satellite.prn >= 193 && satellite.prn <= 199) {
        return satellite;
    }
    int prn = static_cast<int>(satellite.prn);
    while (prn > 100) {
        prn -= 100;
    }
    return SatelliteId(satellite.system, static_cast<uint8_t>(std::max(1, prn)));
}

SatelliteId ambiguitySatelliteForFreq(const SatelliteId& satellite, int freq_index) {
    int base_prn = static_cast<int>(satellite.prn);
    if (satellite.system == GNSSSystem::QZSS && base_prn >= 193 && base_prn <= 199) {
        base_prn -= 192;
    }
    if (freq_index <= 0) {
        return SatelliteId(satellite.system, static_cast<uint8_t>(base_prn));
    }
    const int prn = base_prn + 100 * freq_index;
    return SatelliteId(
        satellite.system,
        static_cast<uint8_t>(std::min(255, std::max(0, prn))));
}

int ionoIndex(const SatelliteId& satellite) {
    const int satno = clasSatNo(satellite);
    return satno > 0 ? kClasNp + satno - 1 : -1;
}

int ambIndex(const SatelliteId& satellite, int freq_index) {
    const int satno = clasSatNo(satellite);
    if (satno <= 0 || freq_index < 0 || freq_index >= kClasNfreq) {
        return -1;
    }
    return kClasAmbStart + kClasMaxSat * freq_index + satno - 1;
}

int clasFrequencySlot(const OSRCorrection& osr, int signal_index) {
    if (signal_index < 0 || signal_index >= osr.num_frequencies ||
        signal_index >= OSR_MAX_FREQ) {
        return -1;
    }
    if (osr.satellite.system == GNSSSystem::Galileo &&
        osr.signals[signal_index] == SignalType::GAL_E5A) {
        return 2;
    }
    return signal_index;
}

int freqIndexFromAmbIndex(int index) {
    if (index < kClasAmbStart || index >= kClasNx) {
        return -1;
    }
    return (index - kClasAmbStart) / kClasMaxSat;
}

double clasCarrierWavelength(int freq_index) {
    switch (freq_index) {
        case 0: return constants::GPS_L1_WAVELENGTH;
        case 1: return constants::GPS_L2_WAVELENGTH;
        case 2: return constants::GPS_L5_WAVELENGTH;
        default: return 0.0;
    }
}

bool rowLessClasOrder(const ZdRow& lhs, const ZdRow& rhs) {
    if (lhs.group != rhs.group) return lhs.group < rhs.group;
    const int lhs_loop = lhs.is_phase ? lhs.freq_index : lhs.freq_index + kClasNfreq;
    const int rhs_loop = rhs.is_phase ? rhs.freq_index : rhs.freq_index + kClasNfreq;
    if (lhs_loop != rhs_loop) return lhs_loop < rhs_loop;
    if (lhs.satellite.system != rhs.satellite.system) {
        return lhs.satellite.system < rhs.satellite.system;
    }
    return lhs.satellite.prn < rhs.satellite.prn;
}

// CLASLIB correspondence: prcopt_default CLAS PPP-RTK option shaping.
ppp_shared::PPPConfig clasnatConfig(const ppp_shared::PPPConfig& base) {
    ppp_shared::PPPConfig config = base;
    config.use_ported_clasnat = true;
    config.use_ported_full = true;
    config.use_clas_osr_filter = true;
    config.wlnl_strict_claslib_parity = true;
    config.use_ionosphere_free = false;
    config.estimate_ionosphere = true;
    config.estimate_troposphere = false;
    config.ar_method = ppp_shared::PPPConfig::ARMethod::DD_PER_FREQ;
    config.clas_epoch_policy = ppp_shared::PPPConfig::ClasEpochPolicy::STRICT_OSR;
    config.clas_correction_application_policy =
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
    config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_FIRST;
    config.clas_decouple_clock_position = false;
    config.clas_initial_position_variance = kClasnatPositionInitVariance;
    config.initial_ionosphere_variance = kIonoInitVariance;
    config.process_noise_ionosphere = 1e-6;
    config.apply_tides_to_receiver_position = true;
    config.apply_tide_as_osr = false;
    return config;
}

// -----------------------------------------------------------------------------
// Debug and parity dump helpers
// -----------------------------------------------------------------------------

bool shouldDumpFullStateTime(const GNSSTime& time) {
    if (time.week != 2068) {
        return false;
    }
    return (time.tow >= 230420.0 - 1e-6 &&
            time.tow <= 230425.0 + 1e-6) ||
           std::abs(time.tow - 231420.0) <= 1e-6 ||
           std::abs(time.tow - 232419.0) <= 1e-6;
}

bool shouldDumpFullStateTarget(const SatelliteId& satellite) {
    const SatelliteId real_satellite = realSatelliteForAmbiguity(satellite);
    if (real_satellite.system == GNSSSystem::GPS) {
        return real_satellite.prn == 14 || real_satellite.prn == 25 ||
               real_satellite.prn == 26 || real_satellite.prn == 29 ||
               real_satellite.prn == 31 || real_satellite.prn == 32;
    }
    if (real_satellite.system == GNSSSystem::Galileo) {
        return real_satellite.prn == 7 || real_satellite.prn == 27 ||
               real_satellite.prn == 30;
    }
    if (real_satellite.system == GNSSSystem::QZSS) {
        return real_satellite.prn == 1 || real_satellite.prn == 2 ||
               real_satellite.prn == 3;
    }
    return false;
}

const char* fullStateDumpPath() {
    const char* path = std::getenv("CLAS_FULL_STATE_DUMP");
    return (path != nullptr && *path != '\0') ? path : nullptr;
}

bool fullStateDumpAllTimes() {
    const char* value = std::getenv("CLAS_FULL_STATE_DUMP_ALL");
    return value != nullptr && *value != '\0' && std::string(value) != "0";
}

bool fullStateDumpEnabled(const GNSSTime& time) {
    return (fullStateDumpAllTimes() || shouldDumpFullStateTime(time)) &&
           (ppp_shared::pppDebugEnabled() || fullStateDumpPath() != nullptr);
}

bool shouldDumpFullItemizedTarget(const SatelliteId& satellite, int freq_index, bool is_phase) {
    if (is_phase || freq_index != 0 || satellite.system != GNSSSystem::QZSS) {
        return false;
    }
    return satellite.prn == 1 || satellite.prn == 2;
}

void emitFullStateLine(const std::string& line) {
    if (ppp_shared::pppDebugEnabled()) {
        std::cerr << line << "\n";
    }
    if (const char* path = fullStateDumpPath()) {
        std::ofstream file(path, std::ios::app);
        if (file.is_open()) {
            file << line << "\n";
        }
    }
}

void dumpFullItemizedZeroDiff(const GNSSTime& time,
                              const OSRCorrection& osr,
                              int signal_index,
                              int freq_slot,
                              const Observation& raw,
                              const Vector3d& receiver_position,
                              double receiver_clock_bias_m,
                              double trop_zenith_m,
                              double trop_mapping,
                              double trop_modeled,
                              double rho_no_sagnac,
                              double rho_sagnac,
                              double sat_clk_m,
                              double iono_grid_m,
                              double tide_applied_m,
                              const AppliedCorrections& applied,
                              double code_model_m) {
    if (!fullStateDumpEnabled(time) ||
        !shouldDumpFullItemizedTarget(osr.satellite, freq_slot, false)) {
        return;
    }
    const double cp_lambda =
        raw.has_carrier_phase && osr.wavelengths[signal_index] > 0.0
            ? raw.carrier_phase * osr.wavelengths[signal_index]
            : 0.0;
    const double sagnac = rho_sagnac - rho_no_sagnac;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(12)
        << "[CLAS-FULL-ITEMIZED] source=lib"
        << " week=" << time.week
        << " tow=" << time.tow
        << " sat=" << osr.satellite.toString()
        << " f=" << freq_slot
        << " freq_group=" << freq_slot
        << " type=code"
        << " pr=" << raw.pseudorange
        << " cp_lambda=" << cp_lambda
        << " rs_x=" << osr.satellite_position.x()
        << " rs_y=" << osr.satellite_position.y()
        << " rs_z=" << osr.satellite_position.z()
        << " dts_m=" << sat_clk_m
        << " rr_x=" << receiver_position.x()
        << " rr_y=" << receiver_position.y()
        << " rr_z=" << receiver_position.z()
        << " dtr_m=" << receiver_clock_bias_m
        << " rho=" << rho_no_sagnac
        << " rho_sagnac=" << rho_sagnac
        << " sagnac=" << sagnac
        << " rcv_ant=" << osr.receiver_antenna_m[signal_index]
        << " pco_rcv=0.000000000000"
        << " pcv_rcv=" << osr.receiver_antenna_m[signal_index]
        << " tide=" << tide_applied_m
        << " tide_solid=" << osr.solid_earth_tide_m
        << " tide_geom=" << osr.tide_geometry_m
        << " windup=0.000000000000"
        << " trop_zenith=" << trop_zenith_m
        << " trop_mapping=" << trop_mapping
        << " trop_mapped=" << trop_modeled
        << " trop_grid=" << osr.trop_correction_m
        << " iono_l1=" << osr.iono_l1_m
        << " iono_grid=" << iono_grid_m
        << " iono_gradient=0.000000000000"
        << " prc=" << osr.PRC[signal_index]
        << " cpc=" << osr.CPC[signal_index]
        << " sat_ant=0.000000000000"
        << " pco_sat=0.000000000000"
        << " pcv_sat=0.000000000000"
        << " rel=" << osr.relativity_correction_m
        << " code_bias=" << osr.code_bias_m[signal_index]
        << " phase_bias=0.000000000000"
        << " phase_comp=0.000000000000"
        << " osr_corr=" << applied.code_m
        << " modeled_sum=" << code_model_m
        << " modeled_sum_nodtr=" << (code_model_m - receiver_clock_bias_m)
        << " y=" << (raw.pseudorange - code_model_m)
        << " y_nodtr=" << (raw.pseudorange - (code_model_m - receiver_clock_bias_m));
    emitFullStateLine(oss.str());
}

double vectorAt(const VectorXd& values, int index) {
    if (index < 0 || index >= values.size()) {
        return 0.0;
    }
    return values(index);
}

double rowAt(const Eigen::RowVectorXd& row, int index) {
    if (index < 0 || index >= row.size()) {
        return 0.0;
    }
    return row(index);
}

ppp_clas::MeasurementRow::ModelComponents subtractComponents(
    const ppp_clas::MeasurementRow::ModelComponents& lhs,
    const ppp_clas::MeasurementRow::ModelComponents& rhs) {
    ppp_clas::MeasurementRow::ModelComponents out;
    out.valid = lhs.valid && rhs.valid;
    auto sub = [](double a, double b) {
        return (std::isfinite(a) && std::isfinite(b))
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

double wavelengthForAmbiguity(
    const SatelliteId& ambiguity_satellite,
    const std::map<SatelliteId, std::array<double, OSR_MAX_FREQ>>& wavelengths_by_sat) {
    const SatelliteId real_satellite = realSatelliteForAmbiguity(ambiguity_satellite);
    int freq_index = 0;
    if (!(ambiguity_satellite.system == GNSSSystem::QZSS &&
          ambiguity_satellite.prn >= 193 && ambiguity_satellite.prn <= 199)) {
        int prn = static_cast<int>(ambiguity_satellite.prn);
        while (prn > 100) {
            prn -= 100;
            ++freq_index;
        }
    }
    const auto it = wavelengths_by_sat.find(real_satellite);
    if (it == wavelengths_by_sat.end() ||
        freq_index < 0 || freq_index >= OSR_MAX_FREQ) {
        return 0.0;
    }
    return it->second[static_cast<size_t>(freq_index)];
}

void dumpFullState(const ClasnatRtkState& rtk,
                   const GNSSTime& time,
                   const char* stage,
                   double receiver_clock_bias_m,
                   double trop_zenith_m,
                   const std::vector<OSRCorrection>& osr_corrections) {
    if (!fullStateDumpEnabled(time) || rtk.x.size() < 3) {
        return;
    }

    std::map<SatelliteId, std::array<double, OSR_MAX_FREQ>> wavelengths_by_sat;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        auto& wavelengths = wavelengths_by_sat[osr.satellite];
        for (int f = 0; f < osr.num_frequencies && f < OSR_MAX_FREQ; ++f) {
            const int freq_slot = clasFrequencySlot(osr, f);
            if (freq_slot >= 0 && freq_slot < OSR_MAX_FREQ) {
                wavelengths[static_cast<size_t>(freq_slot)] = osr.wavelengths[f];
            }
        }
    }

    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(12)
            << "[CLAS-FULL-STATE] source=lib kind=rcv"
            << " week=" << time.week
            << " tow=" << time.tow
            << " stage=" << (stage != nullptr ? stage : "")
            << " pos_x=" << rtk.x(0)
            << " pos_y=" << rtk.x(1)
            << " pos_z=" << rtk.x(2)
            << " receiver_clock_m=" << receiver_clock_bias_m
            << " trop_model_m=" << trop_zenith_m
            << " nx=" << kClasNx
            << " initialized=" << (rtk.initialized ? 1 : 0);
        emitFullStateLine(oss.str());
    }

    for (const auto& [satellite, state_index] : rtk.ionosphere_indices) {
        if (!shouldDumpFullStateTarget(satellite) ||
            state_index < 0 || state_index >= rtk.x.size()) {
            continue;
        }
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(12)
            << "[CLAS-FULL-STATE] source=lib kind=iono"
            << " week=" << time.week
            << " tow=" << time.tow
            << " stage=" << (stage != nullptr ? stage : "")
            << " sat=" << satellite.toString()
            << " idx=" << state_index
            << " iono=" << rtk.x(state_index)
            << " var=" << rtk.P(state_index, state_index);
        emitFullStateLine(oss.str());
    }

    for (const auto& [amb_satellite, state_index] : rtk.ambiguity_indices) {
        const SatelliteId real_satellite = realSatelliteForAmbiguity(amb_satellite);
        if (!shouldDumpFullStateTarget(real_satellite) ||
            state_index < 0 || state_index >= rtk.x.size()) {
            continue;
        }
        const double lambda = wavelengthForAmbiguity(amb_satellite, wavelengths_by_sat);
        const double x_cycles = rtk.x(state_index);
        const double pdiag_cycles2 = rtk.P(state_index, state_index);
        const auto amb_it = rtk.ambiguity_states.find(amb_satellite);
        const int lock =
            amb_it != rtk.ambiguity_states.end() ? amb_it->second.lock_count : 0;
        const int outc =
            amb_it != rtk.ambiguity_states.end() ? amb_it->second.outage_count : 0;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(12)
            << "[CLAS-FULL-STATE] source=lib kind=amb"
            << " week=" << time.week
            << " tow=" << time.tow
            << " stage=" << (stage != nullptr ? stage : "")
            << " sat=" << real_satellite.toString()
            << " amb_sat=" << amb_satellite.toString()
            << " freq_group=" << std::max(0, freqIndexFromAmbIndex(state_index))
            << " idx=" << state_index
            << " x_cycles=" << x_cycles
            << " x_m=" << x_cycles * lambda
            << " pdiag_cycles2=" << pdiag_cycles2
            << " pdiag_m2=" << pdiag_cycles2 * lambda * lambda
            << " lambda=" << lambda
            << " lock=" << lock
            << " outc=" << outc;
        emitFullStateLine(oss.str());
    }
}

void dumpFullModelComponentRow(int row_index,
                               const FullMeasurementRow& measurement,
                               const GNSSTime& time) {
    if (!measurement.components.valid ||
        (!shouldDumpFullStateTarget(measurement.satellite) &&
         !shouldDumpFullStateTarget(measurement.reference_satellite))) {
        return;
    }
    const auto& c = measurement.components;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(12)
        << "[CLAS-FULL-STATE] source=lib kind=comp"
        << " week=" << time.week
        << " tow=" << time.tow
        << " row=" << row_index
        << " sat=" << measurement.satellite.toString()
        << " ref=" << measurement.reference_satellite.toString()
        << " f=" << measurement.freq_index
        << " freq_group=" << measurement.freq_index
        << " type=" << (measurement.is_phase ? "phase" : "code")
        << " y=" << c.y
        << " obs=" << c.obs
        << " rho=" << c.rho
        << " dts=" << c.dts
        << " dtr=" << c.dtr
        << " rel=" << c.rel
        << " sagnac=" << c.sagnac
        << " trop_dry=" << c.trop_dry
        << " trop_wet=" << c.trop_wet
        << " trop_model=" << c.trop_model
        << " trop_grid=" << c.trop_grid
        << " iono_grid=" << c.iono_grid
        << " iono_state_term=" << c.iono_state_term
        << " prc=" << c.prc
        << " cpc=" << c.cpc
        << " osr_corr=" << c.osr_corr
        << " code_bias=" << c.code_bias
        << " phase_bias=" << c.phase_bias
        << " phase_comp=" << c.phase_comp
        << " windup=" << c.windup
        << " pco_rcv=" << c.pco_rcv
        << " pco_sat=" << c.pco_sat
        << " pcv_rcv=" << c.pcv_rcv
        << " pcv_sat=" << c.pcv_sat
        << " tide_solid=" << c.tide_solid
        << " tide_ocean=" << c.tide_ocean
        << " tide_pole=" << c.tide_pole
        << " amb_m=" << c.amb_m
        << " modeled_sum=" << c.modeled_sum;
    emitFullStateLine(oss.str());
}

void dumpSelectedUpdateRows(const std::vector<FullMeasurementRow>& rows,
                            const std::vector<int>& selected_rows,
                            const MatrixXd& measurement_covariance,
                            const MatrixXd& innovation_covariance,
                            const GNSSTime& time) {
    if (!fullStateDumpEnabled(time)) {
        return;
    }
    for (int selected_index = 0;
         selected_index < static_cast<int>(selected_rows.size());
         ++selected_index) {
        const int row_index = selected_rows[static_cast<size_t>(selected_index)];
        if (row_index < 0 || row_index >= static_cast<int>(rows.size())) {
            continue;
        }
        const auto& row = rows[static_cast<size_t>(row_index)];
        const double measurement_variance =
            row_index < measurement_covariance.rows() &&
                    row_index < measurement_covariance.cols()
                ? measurement_covariance(row_index, row_index)
                : std::numeric_limits<double>::quiet_NaN();
        const double innovation_variance =
            row_index < innovation_covariance.rows() &&
                    row_index < innovation_covariance.cols()
                ? innovation_covariance(row_index, row_index)
                : std::numeric_limits<double>::quiet_NaN();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(12)
            << "[CLAS-ROWS-SELECTED] source=lib"
            << " week=" << time.week
            << " tow=" << time.tow
            << " stage=0"
            << " selected_row=" << selected_index
            << " row=" << row_index
            << " sat=" << row.satellite.toString()
            << " ref=" << row.reference_satellite.toString()
            << " f=" << row.freq_index
            << " freq_group=" << row.freq_index
            << " type=" << (row.is_phase ? "phase" : "code")
            << " y=" << row.residual
            << " R=" << measurement_variance
            << " S=" << innovation_variance
            << " threshold_l1l2=" << kResidualRejectSigma
            << " threshold_disp=" << kDispersiveRejectSigma
            << " threshold_l0=" << kNonDispersiveRejectSigma
            << " selected_count=" << selected_rows.size()
            << " total_rows=" << rows.size();
        emitFullStateLine(oss.str());
    }
}

void dumpFullUpdateRows(const std::vector<FullMeasurementRow>& rows,
                        const UpdateStats& update,
                        const ClasnatRtkState& rtk,
                        const GNSSTime& time) {
    if (!fullStateDumpEnabled(time) ||
        update.state_before.size() != kClasNx ||
        update.state_after.size() != kClasNx) {
        return;
    }

    struct Focus {
        SatelliteId real_satellite;
        SatelliteId ambiguity_satellite;
        const char* role = "";
    };

    for (int row_index = 0; row_index < static_cast<int>(rows.size()); ++row_index) {
        const auto& measurement = rows[static_cast<size_t>(row_index)];
        if (measurement.freq_index < 0 || measurement.freq_index >= kClasNfreq) {
            continue;
        }
        dumpFullModelComponentRow(row_index, measurement, time);

        std::vector<Focus> foci;
        if (measurement.satellite.prn != 0) {
            const SatelliteId real_satellite =
                realSatelliteForAmbiguity(measurement.satellite);
            if (shouldDumpFullStateTarget(real_satellite)) {
                const SatelliteId amb_satellite =
                    measurement.is_phase && measurement.ambiguity_satellite.prn != 0
                        ? measurement.ambiguity_satellite
                        : ambiguitySatelliteForFreq(real_satellite, measurement.freq_index);
                foci.push_back({real_satellite, amb_satellite, "sat"});
            }
        }
        if (measurement.reference_satellite.prn != 0) {
            const SatelliteId real_reference =
                realSatelliteForAmbiguity(measurement.reference_satellite);
            if (shouldDumpFullStateTarget(real_reference)) {
                const SatelliteId amb_satellite =
                    ambiguitySatelliteForFreq(real_reference, measurement.freq_index);
                foci.push_back({real_reference, amb_satellite, "ref"});
            }
        }
        if (foci.empty()) {
            continue;
        }

        const double h_pos_x = rowAt(measurement.H, 0);
        const double h_pos_y = rowAt(measurement.H, 1);
        const double h_pos_z = rowAt(measurement.H, 2);
        const double h_pos_norm =
            std::sqrt(h_pos_x * h_pos_x + h_pos_y * h_pos_y + h_pos_z * h_pos_z);

        for (const auto& focus : foci) {
            const auto iono_it = rtk.ionosphere_indices.find(focus.real_satellite);
            const int iono_index =
                iono_it != rtk.ionosphere_indices.end() ? iono_it->second : -1;
            const auto amb_it = rtk.ambiguity_indices.find(focus.ambiguity_satellite);
            const int amb_index =
                amb_it != rtk.ambiguity_indices.end() ? amb_it->second : -1;
            const double h_iono = rowAt(measurement.H, iono_index);
            const double h_amb = measurement.is_phase ? rowAt(measurement.H, amb_index) : 0.0;
            const double state_before = vectorAt(update.state_before, amb_index);
            const double state_after = vectorAt(update.state_after, amb_index);
            const double dx_state = state_after - state_before;
            const double lambda =
                measurement.is_phase && std::abs(h_amb) > 0.0
                    ? std::abs(h_amb)
                    : 0.0;

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(12)
                << "[CLAS-FULL-STATE] source=lib kind=row"
                << " week=" << time.week
                << " tow=" << time.tow
                << " row=" << row_index
                << " focus_sat=" << focus.real_satellite.toString()
                << " focus_amb_sat=" << focus.ambiguity_satellite.toString()
                << " focus_role=" << focus.role
                << " sat=" << measurement.satellite.toString()
                << " ref=" << measurement.reference_satellite.toString()
                << " f=" << measurement.freq_index
                << " freq_group=" << measurement.freq_index
                << " type=" << (measurement.is_phase ? "phase" : "code")
                << " h_pos_x=" << h_pos_x
                << " h_pos_y=" << h_pos_y
                << " h_pos_z=" << h_pos_z
                << " h_pos_norm=" << h_pos_norm
                << " h_iono=" << h_iono
                << " h_trop=0.000000000000"
                << " h_amb=" << h_amb
                << " h_amb_eff=" << h_amb
                << " y=" << measurement.residual
                << " R=" << measurement.variance
                << " dx_state=" << dx_state
                << " dx_state_cycles=" << dx_state
                << " lambda=" << lambda
                << " state_before_cycles=" << state_before
                << " state_after_cycles=" << state_after;
            emitFullStateLine(oss.str());
        }
    }
}

// -----------------------------------------------------------------------------
// Time update and state prediction
// -----------------------------------------------------------------------------

void ensureStorage(ClasnatRtkState& rtk) {
    if (rtk.x.size() == kClasNx &&
        rtk.P.rows() == kClasNx && rtk.P.cols() == kClasNx &&
        rtk.Q.rows() == kClasNx && rtk.Q.cols() == kClasNx) {
        if (rtk.xa.size() != kClasNx ||
            rtk.Pa.rows() != kClasNx ||
            rtk.Pa.cols() != kClasNx) {
            rtk.xa = VectorXd::Zero(kClasNx);
            rtk.Pa = MatrixXd::Zero(kClasNx, kClasNx);
            rtk.has_fixed_solution = false;
        }
        return;
    }
    rtk.x = VectorXd::Zero(kClasNx);
    rtk.P = MatrixXd::Zero(kClasNx, kClasNx);
    rtk.Q = MatrixXd::Zero(kClasNx, kClasNx);
    rtk.xa = VectorXd::Zero(kClasNx);
    rtk.Pa = MatrixXd::Zero(kClasNx, kClasNx);
    rtk.has_fixed_solution = false;
    rtk.ionosphere_indices.clear();
    rtk.ambiguity_indices.clear();
    rtk.ambiguity_states.clear();
}

void initx(ClasnatRtkState& rtk, int index, double value, double variance) {
    if (index < 0 || index >= kClasNx) {
        return;
    }
    rtk.x(index) = value;
    rtk.P.row(index).setZero();
    rtk.P.col(index).setZero();
    rtk.P(index, index) = variance;
    if (rtk.Q.rows() == kClasNx && rtk.Q.cols() == kClasNx) {
        rtk.Q.row(index).setZero();
        rtk.Q.col(index).setZero();
    }
}

void ensureStateMaps(ClasnatRtkState& rtk, const std::vector<OSRCorrection>& osr_corrections) {
    ensureStorage(rtk);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const int ii = ionoIndex(osr.satellite);
        if (ii >= 0) {
            rtk.ionosphere_indices[osr.satellite] = ii;
        }
        for (int f = 0; f < std::min(osr.num_frequencies, kResidualFreqCount); ++f) {
            const int freq_slot = clasFrequencySlot(osr, f);
            if (freq_slot < 0 || freq_slot >= kClasNfreq) {
                continue;
            }
            const SatelliteId amb_sat =
                ambiguitySatelliteForFreq(osr.satellite, freq_slot);
            const int ai = ambIndex(osr.satellite, freq_slot);
            if (ai < 0) {
                continue;
            }
            rtk.ambiguity_indices[amb_sat] = ai;
            auto& ambiguity = rtk.ambiguity_states[amb_sat];
            ambiguity.ambiguity_scale_m = 1.0;
        }
    }
}

// CLASLIB correspondence: pntpos() seed before pppos() refinement.
bool solveSeed(const ObservationData& obs,
               const NavigationData& nav,
               const ClasnatRtkState& rtk,
               PositionSolution& seed) {
    ppp_claslib_pntpos::PntposOptions options;
    options.mode = ppp_claslib_pntpos::PntposOptions::Mode::PppRtk;
    options.debug_dump = ppp_shared::pppDebugEnabled();
    Vector3d initial_rr = Vector3d::Zero();
    if (rtk.initialized && rtk.x.size() >= 3 && rtk.x.head<3>().norm() > 0.0) {
        initial_rr = rtk.x.head<3>();
    }
    return ppp_claslib_pntpos::solvePntposSeed(
        obs, nav, initial_rr, seed, nullptr, options);
}

// CLASLIB correspondence: udpos_ppp() in ppprtk.c.
void predictPosition(ClasnatRtkState& rtk,
                     const PositionSolution& seed,
                     const ppp_shared::PPPConfig& config) {
    ensureStorage(rtk);
    if (!rtk.initialized || rtk.x.head<3>().norm() == 0.0) {
        rtk.x.head<3>() = seed.position_ecef;
        const double variance = std::max(1.0, config.clas_initial_position_variance);
        rtk.P.block<3, 3>(0, 0).setIdentity();
        rtk.P.block<3, 3>(0, 0) *= variance;
        rtk.initialized = true;
        return;
    }
    if (config.kinematic_mode) {
        rtk.x.head<3>() = seed.position_ecef;
    }
}

// CLASLIB correspondence: ionosphere state propagation used by udstate_ppp()/ddres().
void predictIonosphere(ClasnatRtkState& rtk,
                       const std::vector<OSRCorrection>& osr_corrections,
                       double dt) {
    if (dt > 0.0 && kIonoTimeConstantS > 0.0) {
        const double ki = std::exp(-dt / kIonoTimeConstantS);
        for (const auto& [_, idx] : rtk.ionosphere_indices) {
            if (idx < 0 || idx >= kClasNx ||
                rtk.x(idx) == 0.0 || rtk.P(idx, idx) <= 0.0) {
                continue;
            }
            rtk.x(idx) *= ki;
            rtk.P.row(idx) *= ki;
            rtk.P.col(idx) *= ki;
        }
    }
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const auto it = rtk.ionosphere_indices.find(osr.satellite);
        if (it == rtk.ionosphere_indices.end()) {
            continue;
        }
        const int idx = it->second;
        if (idx < 0 || idx >= kClasNx) {
            continue;
        }
        if (rtk.x(idx) == 0.0 || rtk.P(idx, idx) <= 0.0) {
            initx(rtk, idx, kIonoInitM, kIonoInitVariance);
        } else if (dt > 0.0) {
            double q = rtk.Q(idx, idx);
            if (q == 0.0) {
                const double factor = std::cos(osr.elevation);
                q = std::pow(kIonoProcessMPerSqrtS * factor, 2);
                rtk.Q(idx, idx) = q;
            } else if (q > std::pow(kIonoAdaptiveProcessMaxM, 2)) {
                q = std::pow(kIonoAdaptiveProcessMaxM, 2);
                rtk.Q(idx, idx) = q;
            } else if (q < std::pow(kIonoProcessMPerSqrtS, 2)) {
                q = std::pow(kIonoProcessMPerSqrtS, 2);
                rtk.Q(idx, idx) = q;
            }
            rtk.P(idx, idx) += q * dt;
        }
    }
}

// CLASLIB correspondence: udbias_ppp() in ppprtk.c.
void predictAmbiguities(ClasnatRtkState& rtk,
                        const ObservationData& obs,
                        const std::vector<OSRCorrection>& osr_corrections,
                        double dt) {
    for (auto& [_, ambiguity] : rtk.ambiguity_states) {
        ++ambiguity.outage_count;
    }

    for (auto& [amb_sat, idx] : rtk.ambiguity_indices) {
        auto& ambiguity = rtk.ambiguity_states[amb_sat];
        if (ambiguity.outage_count > kMaxAmbOutage && idx >= 0 && idx < kClasNx) {
            initx(rtk, idx, 0.0, 0.0);
            ambiguity = ppp_shared::PPPAmbiguityInfo{};
            ambiguity.ambiguity_scale_m = 1.0;
            ambiguity.lock_count = -5;
        }
    }

    struct RawBiasSeed {
        SatelliteId ambiguity_satellite;
        double bias_m = 0.0;
        double wavelength_m = 0.0;
        int freq_slot = -1;
        int state_index = -1;
    };

    std::array<double, kClasNfreq> bias_sum{};
    std::array<int, kClasNfreq> bias_count{};
    std::vector<RawBiasSeed> raw_bias;

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        for (int f = 0; f < std::min(osr.num_frequencies, kResidualFreqCount); ++f) {
            const int freq_slot = clasFrequencySlot(osr, f);
            if (freq_slot < 0 || freq_slot >= kClasNfreq) {
                continue;
            }
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid || !raw->has_carrier_phase ||
                !raw->has_pseudorange || osr.wavelengths[f] <= 0.0 ||
                !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange)) {
                continue;
            }

            const SatelliteId amb_sat =
                ambiguitySatelliteForFreq(osr.satellite, freq_slot);
            const auto idx_it = rtk.ambiguity_indices.find(amb_sat);
            if (idx_it == rtk.ambiguity_indices.end()) {
                continue;
            }
            const int idx = idx_it->second;
            auto& ambiguity = rtk.ambiguity_states[amb_sat];
            if (raw->loss_of_lock && idx >= 0 && idx < kClasNx) {
                initx(rtk, idx, 0.0, 0.0);
                ambiguity.lock_count = -5;
                ambiguity.outage_count = 0;
                ambiguity.needs_reinitialization = true;
            }
            if (idx >= 0 && idx < kClasNx && rtk.P(idx, idx) > 0.0 && dt > 0.0) {
                rtk.P(idx, idx) += kBiasProcessCyclesPerSqrtS *
                                   kBiasProcessCyclesPerSqrtS * dt;
            }

            const double bias_m =
                raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange;
            raw_bias.push_back(
                {amb_sat, bias_m, osr.wavelengths[f], freq_slot, idx});
            if (idx >= 0 && idx < kClasNx &&
                rtk.x(idx) != 0.0 && std::isfinite(rtk.x(idx)) &&
                osr.wavelengths[f] > 0.0) {
                bias_sum[static_cast<size_t>(freq_slot)] +=
                    bias_m - rtk.x(idx) * osr.wavelengths[f];
                bias_count[static_cast<size_t>(freq_slot)]++;
            }
        }
    }

    for (const auto& seed : raw_bias) {
        const int idx = seed.state_index;
        if (idx < 0 || idx >= kClasNx ||
            (rtk.x(idx) != 0.0 && std::isfinite(rtk.x(idx)) && rtk.P(idx, idx) > 0.0)) {
            continue;
        }
        if (seed.freq_slot < 0 || seed.freq_slot >= kClasNfreq ||
            seed.wavelength_m <= 0.0) {
            continue;
        }
        const int count = bias_count[static_cast<size_t>(seed.freq_slot)];
        const double common_bias_m =
            count > 0 ? bias_sum[static_cast<size_t>(seed.freq_slot)] / count : 0.0;
        const double value_cycles =
            (seed.bias_m - common_bias_m) / seed.wavelength_m;
        initx(rtk, idx, value_cycles, kBiasInitStdCycles * kBiasInitStdCycles);
        auto& ambiguity = rtk.ambiguity_states[seed.ambiguity_satellite];
        ambiguity.lock_count = -5;
        ambiguity.outage_count = 0;
        ambiguity.needs_reinitialization = true;
        ambiguity.ambiguity_scale_m = 1.0;
        ambiguity.float_value = value_cycles;
    }
}

// CLASLIB correspondence: udstate_ppp() update sequence in ppprtk.c.
void propagateState(ClasnatRtkState& rtk,
                    const ObservationData& obs,
                    const std::vector<OSRCorrection>& osr_corrections,
                    const PositionSolution& seed,
                    const ppp_shared::PPPConfig& config) {
    const double dt =
        rtk.has_last_time ? std::max(0.0, std::abs(obs.time - rtk.last_time)) : 0.0;
    predictPosition(rtk, seed, config);
    ensureStateMaps(rtk, osr_corrections);
    predictIonosphere(rtk, osr_corrections, dt);
    predictAmbiguities(rtk, obs, osr_corrections, dt);
}

// -----------------------------------------------------------------------------
// Measurement construction
// -----------------------------------------------------------------------------

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
            corrections.code_m =
                osr.PRC[freq_index] - osr.trop_correction_m;
            corrections.phase_m =
                osr.CPC[freq_index] - osr.trop_correction_m;
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

// CLASLIB correspondence: zdres() zero-difference residual construction.
std::vector<ZdRow> buildZeroDifferenceRows(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    double receiver_clock_bias_m,
    double trop_zenith_m) {
    std::vector<ZdRow> rows;
    rows.reserve(osr_corrections.size() * 2 * kClasNfreq);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.elevation < kElevationMaskRad) {
            continue;
        }
        const double rho_sagnac = geodist(osr.satellite_position, receiver_position);
        const double rho_no_sagnac =
            (osr.satellite_position - receiver_position).norm();
        if (rho_sagnac <= 0.0 || rho_no_sagnac <= 0.0) {
            continue;
        }
        const Vector3d los = (osr.satellite_position - receiver_position).normalized();
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double trop_mapping =
            clasnatTropMappingFunction(receiver_position, osr.elevation, obs.time);
        const double trop_modeled =
            std::isfinite(trop_zenith_m) && trop_zenith_m > 0.0
                ? trop_mapping * trop_zenith_m
                : 0.0;

        for (int f = 0; f < std::min({osr.num_frequencies, kClasNfreq, kResidualFreqCount}); ++f) {
            const int freq_slot = clasFrequencySlot(osr, f);
            if (freq_slot < 0 || freq_slot >= kClasNfreq) {
                continue;
            }
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid ||
                !raw->has_carrier_phase || !raw->has_pseudorange ||
                !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange) ||
                raw->carrier_phase == 0.0 || raw->pseudorange == 0.0 ||
                osr.wavelengths[f] <= 0.0) {
                continue;
            }
            AppliedCorrections applied = selectAppliedCorrections(
                osr, f, config.clas_correction_application_policy);
            double row_trop_modeled = trop_modeled;
            if (config.use_ported_clasnat &&
                config.clas_correction_application_policy ==
                    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR) {
                applied.code_m = osr.PRC[f];
                applied.phase_m = osr.CPC[f];
                row_trop_modeled = 0.0;
            }
            const double fi_for_grid =
                (osr.wavelengths[0] > 0.0 && osr.wavelengths[f] > 0.0)
                    ? osr.wavelengths[f] / osr.wavelengths[0]
                    : 1.0;
            const double iono_grid_m =
                fi_for_grid * fi_for_grid *
                (constants::GPS_L2_FREQ / constants::GPS_L1_FREQ) *
                osr.iono_l1_m;
            const double tide_applied_m =
                osr.tide_geometry_m != 0.0 ? osr.tide_geometry_m
                                            : osr.solid_earth_tide_m;
            Eigen::RowVectorXd h_base = Eigen::RowVectorXd::Zero(kClasNx);
            h_base.segment(0, 3) = -los.transpose();

            const double phase_obs_m = raw->carrier_phase * osr.wavelengths[f];
            const double phase_model_m =
                rho_sagnac - sat_clk_m + receiver_clock_bias_m +
                row_trop_modeled + applied.phase_m;
            ZdRow phase_row;
            phase_row.osr = &osr;
            phase_row.satellite = osr.satellite;
            phase_row.ambiguity_satellite =
                ambiguitySatelliteForFreq(osr.satellite, freq_slot);
            phase_row.signal_index = f;
            phase_row.freq_index = freq_slot;
            phase_row.group = clasResidualGroup(osr, f);
            phase_row.is_phase = true;
            phase_row.valid = true;
            phase_row.y = phase_obs_m - phase_model_m;
            phase_row.variance =
                clasPhaseVarianceForRow(osr.satellite.system, osr.elevation, freq_slot);
            phase_row.H_base = h_base;
            phase_row.components.valid = true;
            phase_row.components.y = phase_row.y;
            phase_row.components.obs = phase_obs_m;
            phase_row.components.rho = rho_no_sagnac;
            phase_row.components.dts = sat_clk_m;
            phase_row.components.dtr = receiver_clock_bias_m;
            phase_row.components.rel = osr.relativity_correction_m;
            phase_row.components.sagnac = rho_sagnac - rho_no_sagnac;
            phase_row.components.trop_dry = row_trop_modeled;
            phase_row.components.trop_model = row_trop_modeled;
            phase_row.components.trop_grid = osr.trop_correction_m;
            phase_row.components.iono_grid = -iono_grid_m;
            phase_row.components.cpc = osr.CPC[f];
            phase_row.components.osr_corr = applied.phase_m;
            phase_row.components.phase_bias = osr.phase_bias_m[f];
            phase_row.components.phase_comp = osr.phase_compensation_m[f];
            phase_row.components.windup = osr.windup_m[f];
            phase_row.components.pcv_rcv = osr.receiver_antenna_m[f];
            phase_row.components.tide_solid = tide_applied_m;
            phase_row.components.modeled_sum = phase_model_m;
            rows.push_back(phase_row);

            const double code_model_m =
                rho_sagnac - sat_clk_m + receiver_clock_bias_m +
                row_trop_modeled + applied.code_m;
            ZdRow code_row;
            code_row.osr = &osr;
            code_row.satellite = osr.satellite;
            code_row.signal_index = f;
            code_row.freq_index = freq_slot;
            code_row.group = clasResidualGroup(osr, f);
            code_row.is_phase = false;
            code_row.valid = true;
            code_row.y = raw->pseudorange - code_model_m;
            code_row.variance =
                clasCodeVarianceForRow(osr.satellite.system, osr.elevation);
            code_row.H_base = h_base;
            code_row.components.valid = true;
            code_row.components.y = code_row.y;
            code_row.components.obs = raw->pseudorange;
            code_row.components.rho = rho_no_sagnac;
            code_row.components.dts = sat_clk_m;
            code_row.components.dtr = receiver_clock_bias_m;
            code_row.components.rel = osr.relativity_correction_m;
            code_row.components.sagnac = rho_sagnac - rho_no_sagnac;
            code_row.components.trop_dry = row_trop_modeled;
            code_row.components.trop_model = row_trop_modeled;
            code_row.components.trop_grid = osr.trop_correction_m;
            code_row.components.iono_grid = iono_grid_m;
            code_row.components.prc = osr.PRC[f];
            code_row.components.osr_corr = applied.code_m;
            code_row.components.code_bias = osr.code_bias_m[f];
            code_row.components.pcv_rcv = osr.receiver_antenna_m[f];
            code_row.components.tide_solid = tide_applied_m;
            code_row.components.modeled_sum = code_model_m;
            dumpFullItemizedZeroDiff(
                obs.time,
                osr,
                f,
                freq_slot,
                *raw,
                receiver_position,
                receiver_clock_bias_m,
                trop_zenith_m,
                trop_mapping,
                row_trop_modeled,
                rho_no_sagnac,
                rho_sagnac,
                sat_clk_m,
                iono_grid_m,
                tide_applied_m,
                applied,
                code_model_m);
            rows.push_back(code_row);
        }
    }
    std::sort(rows.begin(), rows.end(), rowLessClasOrder);
    return rows;
}

// CLASLIB correspondence: ddres() single-difference residual assembly.
MeasurementBuild formSingleDifferenceRows(
    const ObservationData& obs,
    const std::vector<ZdRow>& zd_rows,
    ClasnatRtkState& rtk,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position) {
    MeasurementBuild build;
    struct GroupKey {
        int group = 0;
        int freq_index = 0;
        bool is_phase = false;
        bool operator<(const GroupKey& rhs) const {
            if (group != rhs.group) return group < rhs.group;
            const int lhs_loop = is_phase ? freq_index : freq_index + kClasNfreq;
            const int rhs_loop = rhs.is_phase ? rhs.freq_index : rhs.freq_index + kClasNfreq;
            return lhs_loop < rhs_loop;
        }
    };
    std::map<GroupKey, std::vector<const ZdRow*>> groups;
    for (const auto& row : zd_rows) {
        if (row.valid) {
            groups[{row.group, row.freq_index, row.is_phase}].push_back(&row);
        }
    }

    for (auto& [key, members] : groups) {
        if (members.size() < 2) {
            continue;
        }
        const ZdRow* ref = nullptr;
        for (const ZdRow* row : members) {
            if (ref == nullptr || row->osr->elevation >= ref->osr->elevation) {
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
            FullMeasurementRow row;
            row.H = ref->H_base - sat->H_base;
            row.residual = ref->y - sat->y;
            row.variance = ref->variance + sat->variance;
            row.reference_variance = ref->variance;
            row.satellite = sat->satellite;
            row.ambiguity_satellite =
                key.is_phase ? sat->ambiguity_satellite : SatelliteId{};
            row.reference_satellite = ref->satellite;
            row.is_phase = key.is_phase;
            row.signal_index = sat->signal_index;
            row.freq_index = key.freq_index;
            row.components = subtractComponents(ref->components, sat->components);

            if (config.estimate_ionosphere) {
                const double ref_fi =
                    ref->osr->wavelengths[ref->signal_index] / constants::GPS_L1_WAVELENGTH;
                const double sat_fi =
                    sat->osr->wavelengths[sat->signal_index] / constants::GPS_L1_WAVELENGTH;
                const double sign = key.is_phase ? -1.0 : 1.0;
                const double ref_coeff =
                    sign * ref_fi * ref_fi *
                    clasnatIonoMappingFunction(receiver_position, ref->osr->elevation);
                const double sat_coeff =
                    sign * sat_fi * sat_fi *
                    clasnatIonoMappingFunction(receiver_position, sat->osr->elevation);
                const auto ref_iono_it =
                    rtk.ionosphere_indices.find(ref->satellite);
                const auto sat_iono_it =
                    rtk.ionosphere_indices.find(sat->satellite);
                const bool ref_has_iono =
                    ref_iono_it != rtk.ionosphere_indices.end() &&
                    ref_iono_it->second >= 0 &&
                    ref_iono_it->second < kClasNx;
                const bool sat_has_iono =
                    sat_iono_it != rtk.ionosphere_indices.end() &&
                    sat_iono_it->second >= 0 &&
                    sat_iono_it->second < kClasNx;
                const double ref_state =
                    ref_has_iono ? rtk.x(ref_iono_it->second) : 0.0;
                const double sat_state =
                    sat_has_iono ? rtk.x(sat_iono_it->second) : 0.0;
                const double iono_state_term =
                    ref_coeff * ref_state - sat_coeff * sat_state;
                row.residual -= iono_state_term;
                row.components.iono_state_term += iono_state_term;
                if (ref_has_iono) {
                    row.H(ref_iono_it->second) = ref_coeff;
                }
                if (sat_has_iono) {
                    row.H(sat_iono_it->second) = -sat_coeff;
                }
            }

            if (key.is_phase) {
                const auto ref_amb_it =
                    rtk.ambiguity_indices.find(ref->ambiguity_satellite);
                const auto sat_amb_it =
                    rtk.ambiguity_indices.find(sat->ambiguity_satellite);
                const bool ref_has_amb =
                    ref_amb_it != rtk.ambiguity_indices.end() &&
                    ref_amb_it->second >= 0 &&
                    ref_amb_it->second < kClasNx;
                const bool sat_has_amb =
                    sat_amb_it != rtk.ambiguity_indices.end() &&
                    sat_amb_it->second >= 0 &&
                    sat_amb_it->second < kClasNx;
                if (!sat_has_amb) {
                    continue;
                }
                const double ref_amb_state =
                    ref_has_amb ? rtk.x(ref_amb_it->second) : 0.0;
                const double sat_amb_state = rtk.x(sat_amb_it->second);
                const double ref_lambda = ref->osr->wavelengths[ref->signal_index];
                const double sat_lambda = sat->osr->wavelengths[sat->signal_index];
                const double amb_state_term =
                    ref_lambda * ref_amb_state - sat_lambda * sat_amb_state;
                row.residual -= amb_state_term;
                row.components.amb_m += amb_state_term;
                if (ref_has_amb) {
                    row.H(ref_amb_it->second) = ref_lambda;
                    build.observed_ambiguities.insert(ref->ambiguity_satellite);
                }
                row.H(sat_amb_it->second) = -sat_lambda;
                build.observed_ambiguities.insert(sat->ambiguity_satellite);

                const Observation* raw =
                    obs.getObservation(
                        sat->satellite, sat->osr->signals[sat->signal_index]);
                if (raw && raw->loss_of_lock) {
                    initx(rtk, sat_amb_it->second, 0.0, 0.0);
                    auto& ambiguity = rtk.ambiguity_states[sat->ambiguity_satellite];
                    ambiguity.lock_count = -5;
                    ambiguity.needs_reinitialization = true;
                }
            }
            row.components.y = row.residual;
            row.components.modeled_sum = row.components.obs - row.residual;
            build.rows.push_back(row);
        }
    }
    return build;
}

MeasurementBuild buildMeasurements(const ObservationData& obs,
                                   const std::vector<OSRCorrection>& osr_corrections,
                                   ClasnatRtkState& rtk,
                                   const ppp_shared::PPPConfig& config,
                                   const Vector3d& receiver_position,
                                   double receiver_clock_bias_m,
                                   double trop_zenith_m) {
    const auto zd_rows =
        buildZeroDifferenceRows(
            obs,
            osr_corrections,
            config,
            receiver_position,
            receiver_clock_bias_m,
            trop_zenith_m);
    return formSingleDifferenceRows(obs, zd_rows, rtk, config, receiver_position);
}

// -----------------------------------------------------------------------------
// Measurement update
// -----------------------------------------------------------------------------

// CLASLIB correspondence: residual gating in ddres().
std::vector<int> selectClasnatResidualRows(
    const std::vector<FullMeasurementRow>& rows,
    const VectorXd& residuals,
    const MatrixXd& innovation_covariance) {
    struct PhaseKey {
        SatelliteId satellite;
        int freq_index = 0;

        bool operator<(const PhaseKey& rhs) const {
            if (satellite < rhs.satellite) {
                return true;
            }
            if (rhs.satellite < satellite) {
                return false;
            }
            return freq_index < rhs.freq_index;
        }
    };

    std::map<PhaseKey, double> phase_residuals;
    std::map<PhaseKey, double> phase_variances;
    std::map<PhaseKey, bool> phase_valid;
    std::set<SatelliteId> phase_satellites;

    for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
        const auto& row = rows[static_cast<size_t>(i)];
        if (!row.is_phase ||
            row.freq_index < 0 || row.freq_index >= kClasNfreq) {
            continue;
        }
        const PhaseKey key{row.satellite, row.freq_index};
        phase_residuals[key] = residuals(i);
        phase_variances[key] = innovation_covariance(i, i);
        phase_valid[key] = true;
        phase_satellites.insert(row.satellite);
    }

    for (const auto& satellite : phase_satellites) {
        const PhaseKey l1_key{satellite, 0};
        const auto l1_residual_it = phase_residuals.find(l1_key);
        const auto l1_variance_it = phase_variances.find(l1_key);
        if (l1_residual_it == phase_residuals.end() ||
            l1_variance_it == phase_variances.end() ||
            l1_variance_it->second == 0.0) {
            continue;
        }
        for (int freq = 1; freq < kClasNfreq; ++freq) {
            const PhaseKey other_key{satellite, freq};
            const auto other_residual_it = phase_residuals.find(other_key);
            const auto other_variance_it = phase_variances.find(other_key);
            if (other_residual_it == phase_residuals.end() ||
                other_variance_it == phase_variances.end() ||
                other_variance_it->second == 0.0) {
                continue;
            }
            const double lam_l1 = clasCarrierWavelength(0);
            const double lam_other = clasCarrierWavelength(freq);
            if (lam_l1 <= 0.0 || lam_other <= 0.0) {
                continue;
            }
            const double gamma = (lam_other * lam_other) / (lam_l1 * lam_l1);
            const double denom_disp = 1.0 - gamma;
            const double denom_l0 = gamma - 1.0;
            if (denom_disp == 0.0 || denom_l0 == 0.0) {
                continue;
            }

            const double l1_residual = l1_residual_it->second;
            const double other_residual = other_residual_it->second;
            const double sig2 =
                std::max(l1_variance_it->second, other_variance_it->second);
            const double dispersive =
                (constants::GPS_L1_FREQ / constants::GPS_L2_FREQ) *
                (l1_residual - other_residual) / denom_disp;
            const double non_dispersive =
                (gamma * l1_residual - other_residual) / denom_l0;

            if (kDispersiveRejectSigma > 0.0 &&
                dispersive * dispersive >
                    kDispersiveRejectSigma * kDispersiveRejectSigma * sig2) {
                phase_valid[l1_key] = false;
                phase_valid[other_key] = false;
                continue;
            }
            if (kNonDispersiveRejectSigma > 0.0 &&
                non_dispersive * non_dispersive >
                    kNonDispersiveRejectSigma * kNonDispersiveRejectSigma * sig2) {
                phase_valid[l1_key] = false;
                phase_valid[other_key] = false;
                continue;
            }
        }
    }

    std::vector<int> selected_rows;
    selected_rows.reserve(rows.size());
    for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
        const auto& row = rows[static_cast<size_t>(i)];
        const double residual = residuals(i);
        const double variance = innovation_covariance(i, i);
        const bool passes_prefit =
            variance == 0.0 ||
            kResidualRejectSigma == 0.0 ||
            (std::isfinite(residual) && std::isfinite(variance) &&
             residual * residual <
                 variance * kResidualRejectSigma * kResidualRejectSigma);
        if (!passes_prefit) {
            continue;
        }
        if (row.is_phase) {
            const auto valid_it =
                phase_valid.find(PhaseKey{row.satellite, row.freq_index});
            if (valid_it != phase_valid.end() && !valid_it->second) {
                continue;
            }
        }
        selected_rows.push_back(i);
    }
    return selected_rows;
}

// CLASLIB correspondence: filter() in rtkcmn.c.
UpdateStats applyMeasurementUpdate(ClasnatRtkState& rtk,
                                   const std::vector<FullMeasurementRow>& rows,
                                   bool debug_enabled,
                                   const GNSSTime& time) {
    UpdateStats stats;
    stats.nobs = static_cast<int>(rows.size());
    if (stats.nobs < 4) {
        return stats;
    }
    MatrixXd H = MatrixXd::Zero(stats.nobs, kClasNx);
    VectorXd z = VectorXd::Zero(stats.nobs);
    MatrixXd R = MatrixXd::Zero(stats.nobs, stats.nobs);
    for (int i = 0; i < stats.nobs; ++i) {
        const auto& row = rows[static_cast<size_t>(i)];
        H.row(i) = row.H;
        z(i) = row.residual;
        R(i, i) = std::max(row.variance, 1e-8);
    }

    for (int i = 0; i < stats.nobs; ++i) {
        const auto& lhs = rows[static_cast<size_t>(i)];
        if (lhs.reference_variance <= 0.0) {
            continue;
        }
        for (int j = i + 1; j < stats.nobs; ++j) {
            const auto& rhs = rows[static_cast<size_t>(j)];
            if (rhs.reference_variance <= 0.0 ||
                lhs.is_phase != rhs.is_phase ||
                lhs.freq_index != rhs.freq_index ||
                !(lhs.reference_satellite == rhs.reference_satellite)) {
                continue;
            }
            const double common_reference_variance =
                0.5 * (lhs.reference_variance + rhs.reference_variance);
            R(i, j) = common_reference_variance;
            R(j, i) = common_reference_variance;
        }
    }

    std::vector<int> active_indices;
    active_indices.reserve(kClasNx);
    for (int i = 0; i < kClasNx; ++i) {
        if (std::isfinite(rtk.x(i)) && rtk.x(i) != 0.0 &&
            std::isfinite(rtk.P(i, i)) && rtk.P(i, i) > 0.0) {
            active_indices.push_back(i);
        }
    }
    stats.active_states = static_cast<int>(active_indices.size());
    if (active_indices.empty()) {
        return stats;
    }

    stats.state_before = rtk.x;
    stats.pre_update_covariance = rtk.P;
    const int na = static_cast<int>(active_indices.size());
    MatrixXd Pa = MatrixXd::Zero(na, na);
    MatrixXd Ha = MatrixXd::Zero(stats.nobs, na);
    for (int i = 0; i < na; ++i) {
        const int src_i = active_indices[static_cast<size_t>(i)];
        Ha.col(i) = H.col(src_i);
        for (int j = 0; j < na; ++j) {
            Pa(i, j) = rtk.P(src_i, active_indices[static_cast<size_t>(j)]);
        }
    }

    const MatrixXd PHt = Pa * Ha.transpose();
    const MatrixXd S = Ha * PHt + R;
    stats.selected_rows = selectClasnatResidualRows(rows, z, S);
    dumpSelectedUpdateRows(rows, stats.selected_rows, R, S, time);
    if (stats.selected_rows.empty()) {
        return stats;
    }

    const int ns = static_cast<int>(stats.selected_rows.size());
    MatrixXd Ha_selected = MatrixXd::Zero(ns, na);
    VectorXd z_selected = VectorXd::Zero(ns);
    MatrixXd S_selected = MatrixXd::Zero(ns, ns);
    MatrixXd PHt_selected = MatrixXd::Zero(na, ns);
    for (int i = 0; i < ns; ++i) {
        const int src_i = stats.selected_rows[static_cast<size_t>(i)];
        Ha_selected.row(i) = Ha.row(src_i);
        z_selected(i) = z(src_i);
        PHt_selected.col(i) = PHt.col(src_i);
        for (int j = 0; j < ns; ++j) {
            const int src_j = stats.selected_rows[static_cast<size_t>(j)];
            S_selected(i, j) = S(src_i, src_j);
        }
    }

    Eigen::FullPivLU<MatrixXd> lu(S_selected);
    if (!lu.isInvertible()) {
        return stats;
    }
    const MatrixXd K = PHt_selected * lu.solve(MatrixXd::Identity(ns, ns));
    const VectorXd dxa = K * z_selected;
    if (!dxa.allFinite()) {
        return stats;
    }
    const MatrixXd I_KH = MatrixXd::Identity(na, na) - K * Ha_selected;
    MatrixXd Pa_after = I_KH * Pa;
    Pa_after = 0.5 * (Pa_after + Pa_after.transpose());

    for (int i = 0; i < na; ++i) {
        const int dst_i = active_indices[static_cast<size_t>(i)];
        rtk.x(dst_i) += dxa(i);
        for (int j = 0; j < na; ++j) {
            rtk.P(dst_i, active_indices[static_cast<size_t>(j)]) = Pa_after(i, j);
        }
    }

    std::set<SatelliteId> observed_ionosphere_satellites;
    for (const auto& row : rows) {
        observed_ionosphere_satellites.insert(row.satellite);
        observed_ionosphere_satellites.insert(row.reference_satellite);
    }
    for (const auto& satellite : observed_ionosphere_satellites) {
        const auto idx_it = rtk.ionosphere_indices.find(satellite);
        if (idx_it == rtk.ionosphere_indices.end()) {
            continue;
        }
        const int state_index = idx_it->second;
        if (state_index < 0 || state_index >= kClasNx) {
            continue;
        }
        const auto active_it =
            std::find(active_indices.begin(), active_indices.end(), state_index);
        if (active_it == active_indices.end()) {
            continue;
        }
        const int active_index =
            static_cast<int>(std::distance(active_indices.begin(), active_it));
        const double qp_diag = dxa(active_index) * dxa(active_index);
        rtk.Q(state_index, state_index) =
            kIonoAdaptiveForget * rtk.Q(state_index, state_index) +
            (1.0 - kIonoAdaptiveForget) *
                kIonoAdaptiveGain * kIonoAdaptiveGain * qp_diag;
    }
    stats.updated = true;
    stats.state_after = rtk.x;
    stats.residuals = z;
    stats.variances = R.diagonal();

    if (debug_enabled) {
        std::cerr << "[CLAS-FULL] tow=" << time.tow
                  << " stage=filter updated=1"
                  << " nobs=" << stats.nobs
                  << " active=" << stats.active_states
                  << " dx_pos=" << dxa.head(std::min<Eigen::Index>(3, dxa.size())).norm()
                  << " rms=" << std::sqrt(z.squaredNorm() / stats.nobs)
                  << "\n";
    }
    return stats;
}

// -----------------------------------------------------------------------------
// Ambiguity resolution
// -----------------------------------------------------------------------------

// CLASLIB correspondence: ssat lock/outage bookkeeping after ddres()/filter().
void markObservedAmbiguities(ClasnatRtkState& rtk,
                             const MeasurementBuild& build,
                             const ObservationData& obs) {
    for (const auto& amb_sat : build.observed_ambiguities) {
        const auto idx_it = rtk.ambiguity_indices.find(amb_sat);
        if (idx_it == rtk.ambiguity_indices.end() ||
            idx_it->second < 0 || idx_it->second >= kClasNx) {
            continue;
        }
        auto& ambiguity = rtk.ambiguity_states[amb_sat];
        ambiguity.float_value = rtk.x(idx_it->second);
        ambiguity.ambiguity_scale_m = 1.0;
        ambiguity.last_time = obs.time;
        ambiguity.outage_count = 0;
        ambiguity.lock_count += 1;
        if (ambiguity.lock_count > 0) {
            ambiguity.needs_reinitialization = false;
        }
    }
}

ppp_shared::PPPState makeArState(const ClasnatRtkState& rtk, bool include_l5) {
    ppp_shared::PPPState state;
    state.state = rtk.x;
    state.covariance = rtk.P;
    state.pos_index = 0;
    state.vel_index = -1;
    state.clock_index = 0;
    state.glo_clock_index = 0;
    state.gal_clock_index = 0;
    state.qzs_clock_index = 0;
    state.bds_clock_index = 0;
    state.trop_index = 0;
    state.iono_index = kClasNp;
    state.amb_index = kClasAmbStart;
    state.ionosphere_indices = rtk.ionosphere_indices;
    state.total_states = kClasNx;
    for (const auto& [satellite, idx] : rtk.ambiguity_indices) {
        if (!include_l5 && freqIndexFromAmbIndex(idx) >= 2) {
            continue;
        }
        state.ambiguity_indices[satellite] = idx;
    }
    return state;
}

void copyFixedFromArState(const ppp_shared::PPPState& state, ClasnatRtkState& rtk) {
    if (state.state.size() == kClasNx &&
        state.covariance.rows() == kClasNx &&
        state.covariance.cols() == kClasNx) {
        rtk.xa = state.state;
        rtk.Pa = state.covariance;
        rtk.has_fixed_solution = true;
    }
}

void copyFilterFromArState(const ppp_shared::PPPState& state, ClasnatRtkState& rtk) {
    if (state.state.size() == kClasNx &&
        state.covariance.rows() == kClasNx &&
        state.covariance.cols() == kClasNx) {
        rtk.x = state.state;
        rtk.P = state.covariance;
    }
}

// CLASLIB correspondence: resamb_LAMBDA() in ppprtk.c.
bool tryAmbiguityResolution(ClasnatRtkState& rtk,
                            const std::vector<OSRCorrection>& osr_corrections,
                            const ppp_shared::PPPConfig& config,
                            const MatrixXd& pre_update_covariance,
                            const GNSSTime& time,
                            Vector3d& fixed_position) {
    rtk.last_ar_ratio = 0.0;
    rtk.last_fixed_ambiguities = 0;
    rtk.has_fixed_solution = false;
    if (!config.enable_ambiguity_resolution) {
        return false;
    }

    ppp_shared::PPPState ar_state = makeArState(rtk, true);
    std::map<SatelliteId, double> real_elevations;
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            real_elevations[osr.satellite] = osr.elevation;
        }
    }

    auto eligible = ppp_ar::collectEligibleAmbiguities(
        ar_state, rtk.ambiguity_states, kMinLockForAr, &time);
    if (eligible.satellites.size() < 4) {
        return false;
    }

    (void)pre_update_covariance;
    MatrixXd ar_cross_covariance;
    const auto attempt = ppp_ar::tryDirectDdFixWithPar(
        config,
        ar_state,
        ar_cross_covariance,
        rtk.ambiguity_states,
        eligible,
        real_elevations,
        ppp_shared::pppDebugEnabled(),
        &time);
    rtk.last_ar_ratio = attempt.ratio;
    rtk.last_fixed_ambiguities = attempt.nb;
    if (!attempt.fixed) {
        return false;
    }
    copyFixedFromArState(attempt.state, rtk);
    if (!rtk.has_fixed_solution) {
        return false;
    }
    fixed_position = rtk.xa.head<3>();
    rtk.ambiguity_states = attempt.ambiguities;
    if (attempt.has_hold_state) {
        copyFilterFromArState(attempt.hold_state, rtk);
    }
    return true;
}

// -----------------------------------------------------------------------------
// Output helpers
// -----------------------------------------------------------------------------

// CLASLIB correspondence: sol_t output update at the end of pppos().
PositionSolution makeSolution(const ObservationData& obs,
                              const Vector3d& position,
                              const MatrixXd& covariance,
                              SolutionStatus status,
                              int num_satellites,
                              int num_measurements,
                              double residual_rms,
                              double ratio,
                              int fixed_ambiguities,
                              double receiver_clock_bias_m) {
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = status;
    solution.position_ecef = position;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, lat, lon, height);
    solution.position_geodetic = GeodeticCoord(lat, lon, height);
    if (covariance.rows() >= 3 && covariance.cols() >= 3) {
        solution.position_covariance = covariance.block<3, 3>(0, 0);
    } else {
        solution.position_covariance.setIdentity();
        solution.position_covariance *= 100.0;
    }
    solution.receiver_clock_bias = receiver_clock_bias_m;
    solution.num_satellites = num_satellites;
    solution.num_frequencies = kClasNfreq;
    solution.iterations = 1;
    solution.residual_rms = residual_rms;
    solution.ratio = ratio;
    solution.num_fixed_ambiguities = fixed_ambiguities;
    solution.satellites_used = obs.getSatellites();
    (void)num_measurements;
    return solution;
}

double residualRms(const VectorXd& residuals) {
    return residuals.size() > 0
               ? std::sqrt(residuals.squaredNorm() / static_cast<double>(residuals.size()))
               : 0.0;
}

}  // namespace

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

/// @brief Reset all CLASNAT PPP-RTK filter state.
void reset(ClasnatRtkState& state) {
    state = ClasnatRtkState{};
}

/// @brief Run one CLASNAT epoch; CLASLIB correspondence: pppos() in ppprtk.c.
EpochResult runEpoch(const ObservationData& obs,
                     const NavigationData& nav,
                     const SSRProducts& ssr,
                     ClasnatRtkState& rtk,
                     const ppp_shared::PPPConfig& config) {
    EpochResult result;
    const ppp_shared::PPPConfig epoch_config = clasnatConfig(config);

    PositionSolution seed;
    const bool seed_ok = solveSeed(obs, nav, rtk, seed);
    if (!seed_ok && (!rtk.initialized || rtk.x.size() < 3)) {
        return result;
    }
    if (!seed_ok) {
        seed = makeSolution(
            obs,
            rtk.x.head<3>(),
            rtk.P,
            SolutionStatus::PPP_FLOAT,
            static_cast<int>(obs.getNumSatellites()),
            0,
            0.0,
            rtk.last_ar_ratio,
            rtk.last_fixed_ambiguities,
            0.0);
    }

    const Vector3d context_position =
        rtk.initialized && rtk.x.size() >= 3 && rtk.x.head<3>().norm() > 0.0
            ? rtk.x.head<3>()
            : seed.position_ecef;
    const double trop_zenith_m =
        modeledZenithTroposphereDelayMeters(context_position, obs.time);
    const auto epoch_context = ppp_clasnat_osr::prepareClasnatEpochContext(
        obs,
        nav,
        ssr,
        context_position,
        seed.receiver_clock_bias,
        trop_zenith_m,
        epoch_config,
        rtk.windup_cache,
        rtk.dispersion_compensation,
        rtk.sis_continuity,
        rtk.phase_bias_repair);

    if (epoch_context.osr_corrections.size() < 4) {
        result.solution = seed;
        rtk.has_last_time = true;
        rtk.last_time = obs.time;
        return result;
    }

    propagateState(rtk, obs, epoch_context.osr_corrections, seed, epoch_config);
    dumpFullState(
        rtk,
        obs.time,
        "after_udstate",
        seed.receiver_clock_bias,
        trop_zenith_m,
        epoch_context.osr_corrections);
    const Vector3d measurement_receiver_position =
        epoch_config.use_ported_clasnat ? epoch_context.receiver_position : rtk.x.head<3>();
    const auto measurement_build = buildMeasurements(
        obs,
        epoch_context.osr_corrections,
        rtk,
        epoch_config,
        measurement_receiver_position,
        seed.receiver_clock_bias,
        trop_zenith_m);
    dumpFullState(
        rtk,
        obs.time,
        "before_first_filter",
        seed.receiver_clock_bias,
        trop_zenith_m,
        epoch_context.osr_corrections);
    result.measurement_count =
        static_cast<int>(measurement_build.rows.size());
    const auto update = applyMeasurementUpdate(
        rtk, measurement_build.rows, ppp_shared::pppDebugEnabled(), obs.time);
    dumpFullUpdateRows(measurement_build.rows, update, rtk, obs.time);
    result.updated = update.updated;
    result.active_state_count = update.active_states;
    if (!update.updated) {
        result.solution = seed;
        rtk.has_last_time = true;
        rtk.last_time = obs.time;
        return result;
    }
    markObservedAmbiguities(rtk, measurement_build, obs);
    dumpFullState(
        rtk,
        obs.time,
        "after_filter",
        seed.receiver_clock_bias,
        trop_zenith_m,
        epoch_context.osr_corrections);

    Vector3d output_position = rtk.x.head<3>();
    result.fixed = tryAmbiguityResolution(
        rtk,
        epoch_context.osr_corrections,
        epoch_config,
        update.pre_update_covariance,
        obs.time,
        output_position);
    const MatrixXd& output_covariance =
        result.fixed && rtk.has_fixed_solution ? rtk.Pa : rtk.P;

    const auto valid_osr_count = std::count_if(
        epoch_context.osr_corrections.begin(),
        epoch_context.osr_corrections.end(),
        [](const OSRCorrection& osr) { return osr.valid; });
    result.solution = makeSolution(
        obs,
        output_position,
        output_covariance,
        result.fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT,
        static_cast<int>(valid_osr_count),
        result.measurement_count,
        residualRms(update.residuals),
        rtk.last_ar_ratio,
        rtk.last_fixed_ambiguities,
        seed.receiver_clock_bias);

    if (ppp_shared::pppDebugEnabled()) {
        std::cerr << "[CLAS-FULL] tow=" << obs.time.tow
                  << " nx=" << kClasNx
                  << " nobs=" << result.measurement_count
                  << " updated=" << (result.updated ? 1 : 0)
                  << " fixed=" << (result.fixed ? 1 : 0)
                  << " ratio=" << rtk.last_ar_ratio
                  << " nb=" << rtk.last_fixed_ambiguities
                  << " pos=" << output_position.transpose()
                  << "\n";
    }

    ++rtk.epoch_count;
    rtk.has_last_time = true;
    rtk.last_time = obs.time;
    return result;
}

}  // namespace libgnss::ppp_clasnat_core
