#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/tidal.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

namespace libgnss {

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

bool gnsstimeIsSet(const GNSSTime& time);

std::string trimCopy(const std::string& text) {
    const auto first = std::find_if_not(
        text.begin(), text.end(),
        [](unsigned char c) { return std::isspace(c) != 0; });
    const auto last = std::find_if_not(
        text.rbegin(), text.rend(),
        [](unsigned char c) { return std::isspace(c) != 0; }).base();
    if (first >= last) {
        return "";
    }
    return std::string(first, last);
}

std::string normalizeAntennaType(const std::string& antenna_type) {
    std::string normalized = trimCopy(antenna_type);
    std::replace(normalized.begin(), normalized.end(), '_', ' ');
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    while (normalized.find("  ") != std::string::npos) {
        normalized.erase(normalized.find("  "), 1);
    }
    return normalized;
}

SignalType antexSignalType(const std::string& code) {
    const std::string trimmed = trimCopy(code);
    if (trimmed == "G01") return SignalType::GPS_L1CA;
    if (trimmed == "G02") return SignalType::GPS_L2C;
    if (trimmed == "G05") return SignalType::GPS_L5;
    if (trimmed == "R01") return SignalType::GLO_L1CA;
    if (trimmed == "R02") return SignalType::GLO_L2CA;
    if (trimmed == "E01") return SignalType::GAL_E1;
    if (trimmed == "E05") return SignalType::GAL_E5A;
    if (trimmed == "E07") return SignalType::GAL_E5B;
    if (trimmed == "E06") return SignalType::GAL_E6;
    if (trimmed == "C02") return SignalType::BDS_B1I;
    if (trimmed == "C07") return SignalType::BDS_B2I;
    if (trimmed == "C06") return SignalType::BDS_B3I;
    if (trimmed == "C01") return SignalType::BDS_B1C;
    if (trimmed == "C05") return SignalType::BDS_B2A;
    if (trimmed == "J01") return SignalType::QZS_L1CA;
    if (trimmed == "J02") return SignalType::QZS_L2C;
    if (trimmed == "J05") return SignalType::QZS_L5;
    return SignalType::SIGNAL_TYPE_COUNT;
}

SignalType claslibAntexSignalType(const std::string& code) {
    const std::string trimmed = trimCopy(code);
    if (trimmed.size() < 3 ||
        std::isdigit(static_cast<unsigned char>(trimmed[1])) == 0 ||
        std::isdigit(static_cast<unsigned char>(trimmed[2])) == 0) {
        return SignalType::SIGNAL_TYPE_COUNT;
    }
    const int frequency_number =
        (trimmed[1] - '0') * 10 + (trimmed[2] - '0');
    switch (frequency_number) {
        case 1: return SignalType::GPS_L1CA;
        case 2: return SignalType::GPS_L2C;
        case 5: return SignalType::GPS_L5;
        default:
            return SignalType::SIGNAL_TYPE_COUNT;
    }
}

SignalType receiverAntennaLookupSignal(SignalType signal,
                                       bool claslib_frequency_collapse) {
    if (!claslib_frequency_collapse) {
        return signal;
    }
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GPS_L1P:
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
        case SignalType::GAL_E1:
        case SignalType::BDS_B1I:
        case SignalType::BDS_B1C:
        case SignalType::QZS_L1CA:
            return SignalType::GPS_L1CA;
        case SignalType::GPS_L2P:
        case SignalType::GPS_L2C:
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
        case SignalType::QZS_L2C:
            return SignalType::GPS_L2C;
        case SignalType::GPS_L5:
        case SignalType::GAL_E5A:
        case SignalType::BDS_B2A:
        case SignalType::QZS_L5:
            return SignalType::GPS_L5;
        default:
            return signal;
    }
}

std::string extractAntexFrequencyCode(const std::string& line) {
    for (size_t index = 0; index + 2 < std::min<size_t>(line.size(), 20U); ++index) {
        if (std::isalpha(static_cast<unsigned char>(line[index])) != 0 &&
            std::isdigit(static_cast<unsigned char>(line[index + 1])) != 0 &&
            std::isdigit(static_cast<unsigned char>(line[index + 2])) != 0) {
            return line.substr(index, 3);
        }
    }
    return "";
}

struct ReceiverAntennaFrequencyModel {
    Vector3d offset_enu = Vector3d::Zero();
    std::array<double, 19> noazi_pcv_m{};
    bool has_offset = false;
    bool has_noazi_pcv = false;
};

using ReceiverAntennaModel =
    std::map<SignalType, ReceiverAntennaFrequencyModel>;
using ReceiverAntennaModelMap =
    std::map<std::string, ReceiverAntennaModel>;

bool loadReceiverAntennaModels(
    const std::string& filename,
    ReceiverAntennaModelMap& receiver_models,
    bool claslib_frequency_collapse = false) {
    receiver_models.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_antenna = false;
    bool receiver_entry = false;
    SignalType current_signal = SignalType::SIGNAL_TYPE_COUNT;
    std::string current_type;
    ReceiverAntennaModel current_model;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() >= 60 ? trimCopy(line.substr(60)) : "";
        if (label == "START OF ANTENNA") {
            in_antenna = true;
            receiver_entry = false;
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            current_type.clear();
            current_model.clear();
            continue;
        }
        if (!in_antenna) {
            continue;
        }
        if (label == "END OF ANTENNA") {
            if (receiver_entry && !current_type.empty() && !current_model.empty()) {
                receiver_models[current_type] = current_model;
            }
            in_antenna = false;
            continue;
        }
        if (label == "TYPE / SERIAL NO") {
            current_type = normalizeAntennaType(line.substr(0, std::min<size_t>(line.size(), 20U)));
            const std::string serial =
                line.size() > 20 ? trimCopy(line.substr(20, std::min<size_t>(20U, line.size() - 20))) : "";
            receiver_entry = !(serial.size() == 3 &&
                               std::isalpha(static_cast<unsigned char>(serial[0])) != 0 &&
                               std::isdigit(static_cast<unsigned char>(serial[1])) != 0 &&
                               std::isdigit(static_cast<unsigned char>(serial[2])) != 0);
            continue;
        }
        if (!receiver_entry) {
            continue;
        }
        if (label == "START OF FREQUENCY") {
            const std::string frequency_code = extractAntexFrequencyCode(line);
            current_signal = claslib_frequency_collapse ?
                claslibAntexSignalType(frequency_code) :
                antexSignalType(frequency_code);
            continue;
        }
        if (label == "END OF FREQUENCY") {
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            continue;
        }
        if (current_signal == SignalType::SIGNAL_TYPE_COUNT) {
            continue;
        }
        if (label == "NORTH / EAST / UP") {
            std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 30U)));
            double north_mm = 0.0;
            double east_mm = 0.0;
            double up_mm = 0.0;
            if (iss >> north_mm >> east_mm >> up_mm) {
                auto& model = current_model[current_signal];
                model.offset_enu =
                    Vector3d(east_mm * 1e-3, north_mm * 1e-3, up_mm * 1e-3);
                model.has_offset = true;
            }
            continue;
        }
        const size_t noazi_pos = line.find("NOAZI");
        if (noazi_pos != std::string::npos) {
            std::istringstream iss(line.substr(noazi_pos + 5));
            auto& model = current_model[current_signal];
            double value_mm = 0.0;
            int count = 0;
            for (; count < static_cast<int>(model.noazi_pcv_m.size()) &&
                   (iss >> value_mm); ++count) {
                model.noazi_pcv_m[static_cast<size_t>(count)] = value_mm * 1e-3;
            }
            if (count > 0) {
                for (size_t i = static_cast<size_t>(count); i < model.noazi_pcv_m.size(); ++i) {
                    model.noazi_pcv_m[i] = model.noazi_pcv_m[i - 1];
                }
                model.has_noazi_pcv = true;
            }
        }
    }

    return !receiver_models.empty();
}

const ReceiverAntennaModel* findReceiverAntennaModel(
    const ppp_shared::PPPConfig& config) {
    if (config.antex_file_path.empty() ||
        config.receiver_antenna_type.empty()) {
        return nullptr;
    }

    const bool claslib_frequency_collapse =
        config.wlnl_strict_claslib_parity;
    using CacheKey = std::pair<std::string, bool>;
    static std::map<CacheKey, ReceiverAntennaModelMap> cache;
    const CacheKey cache_key{
        config.antex_file_path,
        claslib_frequency_collapse};
    auto cache_it = cache.find(cache_key);
    if (cache_it == cache.end()) {
        ReceiverAntennaModelMap models;
        if (!loadReceiverAntennaModels(
                config.antex_file_path,
                models,
                claslib_frequency_collapse)) {
            cache[cache_key] = ReceiverAntennaModelMap{};
        } else {
            cache[cache_key] = std::move(models);
        }
        cache_it = cache.find(cache_key);
    }
    if (cache_it == cache.end()) {
        return nullptr;
    }

    const std::string normalized_type =
        normalizeAntennaType(config.receiver_antenna_type);
    if (normalized_type.empty()) {
        return nullptr;
    }
    const auto model_it = cache_it->second.find(normalized_type);
    if (model_it != cache_it->second.end()) {
        return &model_it->second;
    }

    // Match RTKLIB's useful fallback for descriptors without an exact radome.
    const std::string antenna_without_radome =
        trimCopy(normalized_type.substr(0, std::min<size_t>(20U, normalized_type.size())));
    for (const auto& [type, model] : cache_it->second) {
        if (type.rfind(antenna_without_radome, 0) == 0) {
            return &model;
        }
    }
    return nullptr;
}

double interpolateReceiverNoaziPcv(
    const ReceiverAntennaFrequencyModel& model,
    double elevation_rad) {
    if (!model.has_noazi_pcv) {
        return 0.0;
    }
    const double zenith_deg =
        90.0 - elevation_rad * 180.0 / M_PI;
    const double a = zenith_deg / 5.0;
    const int i = static_cast<int>(a);
    if (i < 0) {
        return model.noazi_pcv_m[0];
    }
    if (i >= 18) {
        return model.noazi_pcv_m[18];
    }
    return model.noazi_pcv_m[static_cast<size_t>(i)] * (1.0 - a + i) +
           model.noazi_pcv_m[static_cast<size_t>(i + 1)] * (a - i);
}

struct ReceiverAntennaCorrectionDetails {
    Vector3d offset_enu = Vector3d::Zero();
    Vector3d offset_ecef = Vector3d::Zero();
    double pco_m = 0.0;
    double pcv_m = 0.0;
    double total_m = 0.0;
    bool has_antex = false;
};

ReceiverAntennaCorrectionDetails receiverAntennaCorrection(
    const ppp_shared::PPPConfig& config,
    const ReceiverAntennaModel* antenna_model,
    SignalType signal,
    double azimuth_rad,
    double elevation_rad,
    double receiver_lat_rad,
    double receiver_lon_rad) {
    ReceiverAntennaCorrectionDetails details;
    details.offset_enu = config.receiver_antenna_delta_enu;
    if (antenna_model != nullptr) {
        const SignalType lookup_signal = receiverAntennaLookupSignal(
            signal,
            config.wlnl_strict_claslib_parity);
        const auto signal_it = antenna_model->find(lookup_signal);
        if (signal_it != antenna_model->end()) {
            const auto& model = signal_it->second;
            if (model.has_offset) {
                details.offset_enu += model.offset_enu;
                details.has_antex = true;
            }
            details.pcv_m = interpolateReceiverNoaziPcv(model, elevation_rad);
        }
    }

    const double cos_el = std::cos(elevation_rad);
    const Vector3d e_enu(
        std::sin(azimuth_rad) * cos_el,
        std::cos(azimuth_rad) * cos_el,
        std::sin(elevation_rad));
    details.pco_m = -details.offset_enu.dot(e_enu);
    details.total_m = details.pco_m + details.pcv_m;
    details.offset_ecef =
        enu2ecef(details.offset_enu, receiver_lat_rad, receiver_lon_rad);
    return details;
}

const tidal::EarthRotationParameters* cachedEarthRotationParameters(
    const std::string& path) {
    if (path.empty()) {
        return nullptr;
    }
    struct CacheEntry {
        bool loaded = false;
        tidal::EarthRotationParameters erp;
    };
    static std::map<std::string, CacheEntry> cache;
    auto it = cache.find(path);
    if (it == cache.end()) {
        CacheEntry entry;
        entry.loaded = tidal::loadEarthRotationParameters(path, entry.erp);
        it = cache.emplace(path, std::move(entry)).first;
    }
    return it->second.loaded ? &it->second.erp : nullptr;
}

const tidal::OceanLoadingGrid* cachedClasGridOceanLoading(
    const std::string& path) {
    if (path.empty()) {
        return nullptr;
    }
    struct CacheEntry {
        bool loaded = false;
        tidal::OceanLoadingGrid grid;
    };
    static std::map<std::string, CacheEntry> cache;
    auto it = cache.find(path);
    if (it == cache.end()) {
        CacheEntry entry;
        entry.loaded = tidal::loadClasGridOceanLoading(path, entry.grid);
        it = cache.emplace(path, std::move(entry)).first;
    }
    return it->second.loaded ? &it->second.grid : nullptr;
}

bool buildClasGridTideInterpolation(
    const std::map<std::string, std::string>& atmos_tokens,
    const Vector3d& receiver_position,
    tidal::ClasGridInterpolation& interpolation) {
    ppp_atmosphere::ClasGridReference reference;
    if (!ppp_atmosphere::resolveClasGridReference(
            atmos_tokens, receiver_position, reference)) {
        return false;
    }

    interpolation = tidal::ClasGridInterpolation{};
    interpolation.network_id = reference.network_id;
    if (reference.has_model_interpolation) {
        interpolation.grid_count = 4;
        interpolation.use_model_interpolation = true;
        for (int i = 0; i < 4; ++i) {
            interpolation.grid_numbers[i] =
                static_cast<int>(reference.model_grid_indices[i]) + 1;
            interpolation.weights[i] = 0.0;
        }
        for (int i = 0; i < 16; ++i) {
            interpolation.model_gmat[i] = reference.model_gmat[i];
        }
        for (int i = 0; i < 4; ++i) {
            interpolation.model_emat[i] = reference.model_emat[i];
        }
        return true;
    }

    interpolation.grid_count = 1;
    interpolation.grid_numbers[0] = reference.grid_no;
    interpolation.weights[0] = 1.0;
    return true;
}

tidal::TideDisplacementComponents calculateClasTideComponents(
    const Vector3d& receiver_position,
    const GNSSTime& time,
    const std::map<std::string, std::string>& atmos_tokens,
    const ppp_shared::PPPConfig& config) {
    const tidal::EarthRotationParameters* erp =
        cachedEarthRotationParameters(config.eop_file_path);
    const tidal::OceanLoadingGrid* ocean_grid =
        cachedClasGridOceanLoading(config.clas_grid_blq_file_path);

    tidal::ClasGridInterpolation interpolation;
    const bool have_ocean_interpolation =
        config.apply_clas_grid_ocean_loading &&
        ocean_grid != nullptr &&
        buildClasGridTideInterpolation(atmos_tokens, receiver_position, interpolation);

    return tidal::calculateTideDisplacement(
        receiver_position,
        time,
        erp,
        ocean_grid,
        have_ocean_interpolation ? &interpolation : nullptr,
        config.apply_solid_earth_tides,
        config.apply_clas_grid_ocean_loading && have_ocean_interpolation,
        config.apply_pole_tide);
}

Vector3d orbitCorrectionToEcef(const Vector3d& orbit_correction,
                               const Vector3d& sat_pos,
                               const Vector3d& sat_vel,
                               bool corrections_are_rac) {
    if (!corrections_are_rac ||
        orbit_correction.squaredNorm() <= 0.0 ||
        sat_pos.squaredNorm() <= 0.0 ||
        sat_vel.squaredNorm() <= 0.0) {
        return orbit_correction;
    }

    // RAC frame follows RTCM-10403.1 / CLASLIB convention:
    //   Along-track  = normalize(velocity)
    //   Cross-track  = normalize(position × velocity)
    //   Radial       = along × cross  (outward from Earth center)
    const Vector3d ea = sat_vel.normalized();
    Vector3d c_unit = sat_pos.cross(sat_vel);
    if (c_unit.squaredNorm() > 0.0) {
        c_unit.normalize();
    } else {
        c_unit = Vector3d(0, 0, 1);
    }
    const Vector3d er = ea.cross(c_unit);
    return -(er * orbit_correction(0)
           + ea * orbit_correction(1)
           + c_unit * orbit_correction(2));
}

bool ssrEntryMatchesPreferredNetwork(const SSROrbitClockCorrection& entry,
                                     int preferred_network_id) {
    if (preferred_network_id <= 0) {
        return true;
    }
    return entry.bias_network_id == preferred_network_id ||
           entry.bias_network_id == 0 ||
           entry.atmos_network_id == preferred_network_id;
}

const SSROrbitClockCorrection* pickHeldOrbitCorrection(
    const SSRProducts& ssr,
    const SatelliteId& sat,
    const GNSSTime& time,
    int preferred_network_id) {
    constexpr double kSsrLookupMaxAgeSeconds = 900.0;

    const auto sat_it = ssr.orbit_clock_corrections.find(sat);
    if (sat_it == ssr.orbit_clock_corrections.end() || sat_it->second.empty()) {
        return nullptr;
    }

    const auto& entries = sat_it->second;
    auto upper = std::upper_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
            return lhs < rhs.time;
        });
    size_t group_end = static_cast<size_t>(upper - entries.begin());
    while (group_end > 0) {
        const size_t group_last = group_end - 1;
        const GNSSTime group_time = entries[group_last].time;
        if (std::abs(time - group_time) > kSsrLookupMaxAgeSeconds) {
            break;
        }
        size_t group_begin = group_last;
        while (group_begin > 0 && entries[group_begin - 1].time == group_time) {
            --group_begin;
        }

        const SSROrbitClockCorrection* fallback = nullptr;
        for (size_t index = group_begin; index < group_end; ++index) {
            const SSROrbitClockCorrection& entry = entries[index];
            if (!entry.orbit_valid) {
                continue;
            }
            if (fallback == nullptr) {
                fallback = &entry;
            }
            if (ssrEntryMatchesPreferredNetwork(entry, preferred_network_id)) {
                return &entry;
            }
        }
        if (fallback != nullptr) {
            return fallback;
        }
        group_end = group_begin;
    }
    return nullptr;
}

const SSROrbitClockCorrection* pickHeldClockCorrection(
    const SSRProducts& ssr,
    const SatelliteId& sat,
    const GNSSTime& time,
    int preferred_network_id) {
    constexpr double kSsrLookupMaxAgeSeconds = 900.0;

    const auto sat_it = ssr.orbit_clock_corrections.find(sat);
    if (sat_it == ssr.orbit_clock_corrections.end() || sat_it->second.empty()) {
        return nullptr;
    }

    const auto& entries = sat_it->second;
    auto upper = std::upper_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
            return lhs < rhs.time;
        });
    size_t group_end = static_cast<size_t>(upper - entries.begin());
    while (group_end > 0) {
        const size_t group_last = group_end - 1;
        const GNSSTime group_time = entries[group_last].time;
        if (std::abs(time - group_time) > kSsrLookupMaxAgeSeconds) {
            break;
        }
        size_t group_begin = group_last;
        while (group_begin > 0 && entries[group_begin - 1].time == group_time) {
            --group_begin;
        }

        const SSROrbitClockCorrection* fallback = nullptr;
        for (size_t index = group_begin; index < group_end; ++index) {
            const SSROrbitClockCorrection& entry = entries[index];
            if (!entry.clock_valid) {
                continue;
            }
            if (fallback == nullptr) {
                fallback = &entry;
            }
            if (ssrEntryMatchesPreferredNetwork(entry, preferred_network_id)) {
                return &entry;
            }
        }
        if (fallback != nullptr) {
            return fallback;
        }
        group_end = group_begin;
    }
    return nullptr;
}

struct RacToEcefDump {
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    Vector3d er = Vector3d::Zero();
    Vector3d ea = Vector3d::Zero();
    Vector3d ec = Vector3d::Zero();
};

RacToEcefDump makeRacToEcefDump(
    const Vector3d& sat_pos,
    const Vector3d& sat_vel,
    bool corrections_are_rac) {
    RacToEcefDump out;
    if (!corrections_are_rac ||
        sat_pos.squaredNorm() <= 0.0 ||
        sat_vel.squaredNorm() <= 0.0) {
        return out;
    }
    out.ea = sat_vel.normalized();
    out.ec = sat_pos.cross(sat_vel);
    if (out.ec.squaredNorm() > 0.0) {
        out.ec.normalize();
    } else {
        out.ec = Vector3d(0, 0, 1);
    }
    out.er = out.ea.cross(out.ec);
    out.matrix.col(0) = -out.er;
    out.matrix.col(1) = -out.ea;
    out.matrix.col(2) = -out.ec;
    return out;
}

int cssrPrnForDump(const SatelliteId& sat) {
    if (sat.system == GNSSSystem::QZSS) {
        return 193 + static_cast<int>(sat.prn) - 1;
    }
    return static_cast<int>(sat.prn);
}

double claslibMaxToeAgeSeconds(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::Galileo:
            return 10801.0;
        case GNSSSystem::BeiDou:
            return 21601.0;
        case GNSSSystem::QZSS:
            return 7201.0;
        case GNSSSystem::GLONASS:
            return 1800.0;
        default:
            return 7201.0;
    }
}

const Ephemeris* selectSsrEphemerisByIode(
    const NavigationData& nav,
    const SatelliteId& sat,
    const GNSSTime& teph,
    int iode) {
    if (iode < 0) {
        return nullptr;
    }
    const auto sat_it = nav.ephemeris_data.find(sat);
    if (sat_it == nav.ephemeris_data.end()) {
        return nullptr;
    }
    const double max_age = claslibMaxToeAgeSeconds(sat.system);
    for (const auto& eph : sat_it->second) {
        if (!eph.valid || static_cast<int>(eph.iode) != iode) {
            continue;
        }
        if (std::abs(eph.toe - teph) > max_age) {
            continue;
        }
        return &eph;
    }
    return nullptr;
}

bool isGpsClasParityTarget(const SatelliteId& sat) {
    return sat.system == GNSSSystem::GPS &&
           (sat.prn == 14 || sat.prn == 25 || sat.prn == 26 ||
            sat.prn == 29 || sat.prn == 31 || sat.prn == 32);
}

bool shouldDumpClasSatSsr(
    const ppp_shared::PPPConfig& config,
    const GNSSTime& time,
    const SatelliteId& sat) {
    return config.wlnl_strict_claslib_parity &&
           time.week == 2068 &&
           time.tow >= 230420.0 - 1e-6 &&
           time.tow <= 230425.0 + 1e-6 &&
           (isGpsClasParityTarget(sat) ||
            (sat.system == GNSSSystem::Galileo &&
             (sat.prn == 7 || sat.prn == 27)) ||
            (sat.system == GNSSSystem::QZSS &&
             (sat.prn == 1 || sat.prn == 2 || sat.prn == 3)));
}

const char* libSatSsrDumpPath() {
    const char* path = std::getenv("LIB_SAT_SSR_DUMP");
    return (path != nullptr && *path != '\0') ? path : "/tmp/lib_sat_ssr_dump.rerun.txt";
}

void writeLibSatSsrDumpLine(const std::string& line) {
    std::cerr << line << "\n";
    std::ofstream file(libSatSsrDumpPath(), std::ios::app);
    if (file.is_open()) {
        file << line << "\n";
    }
}

void dumpClasSatSsr(
    const ppp_shared::PPPConfig& config,
    const ObservationData& obs,
    const SatelliteId& sat,
    const GNSSTime& satellite_time,
    const Ephemeris* eph,
    const Vector3d& receiver_pos,
    const Vector3d& rs_bcast,
    const Vector3d& vs_bcast,
    double dts_bcast_s,
    const GNSSTime& rac_ref_time,
    const Vector3d& orbit_corr_rac,
    int ssr_iode,
    const Vector3d& delta_ecef,
    const GNSSTime& clk_ref_time,
    double delta_clk_m,
    const Vector3d& rs_final,
    double dts_final_s,
    double rho_to_rcv,
    double sagnac) {
    if (!shouldDumpClasSatSsr(config, obs.time, sat)) {
        return;
    }

    const RacToEcefDump rac = makeRacToEcefDump(
        rs_bcast,
        vs_bcast,
        true);
    const double dt_ssr = gnsstimeIsSet(rac_ref_time) ? (satellite_time - rac_ref_time) :
        std::numeric_limits<double>::quiet_NaN();
    const double clk_dt_ssr = gnsstimeIsSet(clk_ref_time) ? (satellite_time - clk_ref_time) :
        std::numeric_limits<double>::quiet_NaN();
    const double toe = eph != nullptr ? eph->toe.tow : std::numeric_limits<double>::quiet_NaN();
    const double toc = eph != nullptr ? eph->toc.tow : std::numeric_limits<double>::quiet_NaN();
    const int iode = eph != nullptr ? static_cast<int>(eph->iode) : -1;
    const double tgd_primary_s = eph != nullptr ? eph->tgd : std::numeric_limits<double>::quiet_NaN();
    const double tgd_secondary_s =
        eph != nullptr ? eph->tgd_secondary : std::numeric_limits<double>::quiet_NaN();
    const double dts_bcast_m = dts_bcast_s * constants::SPEED_OF_LIGHT;
    const double dts_final_m = dts_final_s * constants::SPEED_OF_LIGHT;
    const double geom_clock = rho_to_rcv + sagnac - dts_final_m;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(12)
        << "[CLAS-SAT-SSR] source=lib"
        << " week=" << obs.time.week
        << " tow=" << obs.time.tow
        << " sat=" << sat.toString()
        << " prn=" << cssrPrnForDump(sat)
        << " sat_time_gps=" << satellite_time.tow
        << " t_broadcast_toe=" << toe
        << " t_broadcast_toc=" << toc
        << " iode=" << iode
        << " ssr_iode=" << ssr_iode
        << " bgd_e1e5a_s=" << tgd_primary_s
        << " bgd_e1e5a_m=" << (tgd_primary_s * constants::SPEED_OF_LIGHT)
        << " bgd_e1e5b_s=" << tgd_secondary_s
        << " bgd_e1e5b_m=" << (tgd_secondary_s * constants::SPEED_OF_LIGHT)
        << " dts_bcast_bgd_applied=0"
        << " gal_frequency_convention=E1-E5a"
        << " rs_bcast[0]=" << rs_bcast(0)
        << " rs_bcast[1]=" << rs_bcast(1)
        << " rs_bcast[2]=" << rs_bcast(2)
        << " dts_bcast=" << dts_bcast_m
        << " rac_ref_time=" << (gnsstimeIsSet(rac_ref_time) ? rac_ref_time.tow : std::numeric_limits<double>::quiet_NaN())
        << " dt_ssr=" << dt_ssr
        << " dt_ssr_effective=" << dt_ssr
        << " udi_orbit=0.000000000000"
        << " delta_r=" << orbit_corr_rac(0)
        << " delta_a=" << orbit_corr_rac(1)
        << " delta_c=" << orbit_corr_rac(2)
        << " delta_r_rate=0.000000000000"
        << " delta_a_rate=0.000000000000"
        << " delta_c_rate=0.000000000000";
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            oss << " R_rac_to_ecef[" << row << "][" << col << "]=" << rac.matrix(row, col);
        }
    }
    oss << " delta_ecef[0]=" << delta_ecef(0)
        << " delta_ecef[1]=" << delta_ecef(1)
        << " delta_ecef[2]=" << delta_ecef(2)
        << " clk_ref_time=" << (gnsstimeIsSet(clk_ref_time) ? clk_ref_time.tow : std::numeric_limits<double>::quiet_NaN())
        << " clk_dt_ssr=" << clk_dt_ssr
        << " clk_dt_ssr_effective=" << clk_dt_ssr
        << " udi_clock=0.000000000000"
        << " c0=" << delta_clk_m
        << " c1=0.000000000000"
        << " c2=0.000000000000"
        << " delta_clk=" << delta_clk_m
        << " rs_final[0]=" << rs_final(0)
        << " rs_final[1]=" << rs_final(1)
        << " rs_final[2]=" << rs_final(2)
        << " dts_final=" << dts_final_m
        << " rho_to_rcv=" << rho_to_rcv
        << " sagnac=" << sagnac
        << " geom_clock=" << geom_clock;
    writeLibSatSsrDumpLine(oss.str());
}

const char* clasAtmosSelectionPolicyName(
    ppp_shared::PPPConfig::ClasAtmosSelectionPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_FIRST:
            return "grid-first";
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED:
            return "grid-guarded";
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::BALANCED:
            return "balanced";
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::FRESHNESS_FIRST:
            return "freshness-first";
    }
    return "grid-first";
}

void resetPhaseBiasRepairInfo(CLASPhaseBiasRepairInfo& info) {
    info.reference_time = GNSSTime();
    info.last_continuity_m = {0.0, 0.0, 0.0};
    info.offset_cycles = {0.0, 0.0, 0.0};
    info.pending_state_shift_cycles = {0.0, 0.0, 0.0};
    info.has_last = {false, false, false};
}

struct ClasAtmosCandidate {
    std::map<std::string, std::string> tokens;
    bool has_grid = false;
    double grid_distance_sq = std::numeric_limits<double>::infinity();
    double time_gap = std::numeric_limits<double>::infinity();
    bool is_future = false;
    int token_count = -1;
};

bool isBetterGridFirstCandidate(const ClasAtmosCandidate& candidate,
                                const ClasAtmosCandidate& best) {
    return (candidate.has_grid && !best.has_grid) ||
           (candidate.has_grid == best.has_grid &&
            candidate.is_future != best.is_future &&
            !candidate.is_future) ||
           (candidate.has_grid == best.has_grid &&
            candidate.is_future == best.is_future &&
            candidate.grid_distance_sq + 1e-9 < best.grid_distance_sq) ||
           (candidate.has_grid == best.has_grid &&
            candidate.is_future == best.is_future &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            candidate.time_gap + 1e-9 < best.time_gap) ||
           (candidate.has_grid == best.has_grid &&
            candidate.is_future == best.is_future &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
           candidate.token_count > best.token_count);
}

bool isBetterGridGuardedCandidate(const ClasAtmosCandidate& candidate,
                                  const ClasAtmosCandidate& best,
                                  double stale_after_seconds) {
    const bool candidate_stale = candidate.time_gap > stale_after_seconds;
    const bool best_stale = best.time_gap > stale_after_seconds;
    return (candidate_stale != best_stale && !candidate_stale) ||
           (candidate_stale == best_stale &&
            isBetterGridFirstCandidate(candidate, best));
}

bool isBetterFreshnessFirstCandidate(const ClasAtmosCandidate& candidate,
                                     const ClasAtmosCandidate& best) {
    return (candidate.has_grid && !best.has_grid) ||
           (candidate.has_grid == best.has_grid &&
            candidate.is_future != best.is_future &&
            !candidate.is_future) ||
           (candidate.has_grid == best.has_grid &&
            candidate.is_future == best.is_future &&
            candidate.time_gap + 1e-9 < best.time_gap) ||
           (candidate.has_grid == best.has_grid &&
            candidate.is_future == best.is_future &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            candidate.grid_distance_sq + 1e-9 < best.grid_distance_sq) ||
           (candidate.has_grid == best.has_grid &&
            candidate.is_future == best.is_future &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            candidate.token_count > best.token_count);
}

bool isBetterBalancedCandidate(const ClasAtmosCandidate& candidate,
                               const ClasAtmosCandidate& best,
                               double stale_after_seconds) {
    const bool candidate_stale = candidate.time_gap > stale_after_seconds;
    const bool best_stale = best.time_gap > stale_after_seconds;
    return (candidate.has_grid && !best.has_grid) ||
           (candidate.has_grid == best.has_grid && candidate_stale != best_stale &&
            !candidate_stale) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            candidate.is_future != best.is_future &&
            !candidate.is_future) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            candidate.is_future == best.is_future &&
            !candidate_stale &&
            candidate.grid_distance_sq + 1e-9 < best.grid_distance_sq) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            candidate.is_future == best.is_future &&
            candidate_stale &&
            candidate.time_gap + 1e-9 < best.time_gap) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            candidate.is_future == best.is_future &&
            !candidate_stale &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            candidate.time_gap + 1e-9 < best.time_gap) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            candidate.is_future == best.is_future &&
            candidate_stale &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            candidate.grid_distance_sq + 1e-9 < best.grid_distance_sq) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            candidate.is_future == best.is_future &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            candidate.token_count > best.token_count);
}

bool isBetterClasAtmosCandidate(
    const ClasAtmosCandidate& candidate,
    const ClasAtmosCandidate& best,
    const ppp_shared::PPPConfig& config) {
    switch (config.clas_atmos_selection_policy) {
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_FIRST:
            return isBetterGridFirstCandidate(candidate, best);
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED:
            return isBetterGridGuardedCandidate(
                candidate, best, config.clas_atmos_stale_after_seconds);
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::BALANCED:
            return isBetterBalancedCandidate(
                candidate, best, config.clas_atmos_stale_after_seconds);
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::FRESHNESS_FIRST:
            return isBetterFreshnessFirstCandidate(candidate, best);
    }
    return isBetterGridFirstCandidate(candidate, best);
}

/// Saastamoinen troposphere model with Niell mapping function
double troposphereDelay(const Vector3d& receiver_pos, double elevation, double trop_zenith) {
    if (elevation < 0.05) return trop_zenith * 10.0;  // near-horizon penalty
    // Simplified Niell dry mapping
    const double m = 1.0 / std::sin(std::max(elevation, 0.05));
    return trop_zenith * m;
}

/// Relativistic correction: only Shapiro delay.
/// The periodic relativity and gravitational redshift are already in the
/// broadcast clock polynomial, so we only need the signal propagation delay
/// due to the gravitational field (Shapiro effect).
double relativisticCorrection(const Vector3d& sat_pos, const Vector3d& /* sat_vel */,
                               const Vector3d& rcv_pos) {
    constexpr double GM = 3.986005e14;
    constexpr double c = constants::SPEED_OF_LIGHT;
    const double rs = sat_pos.norm();
    const double rr = rcv_pos.norm();
    const double rho = (sat_pos - rcv_pos).norm();
    const double arg = (rs + rr + rho) / std::max(rs + rr - rho, 1.0);
    // Shapiro delay in meters: 2*GM/c² * ln(...)
    return 2.0 * GM / (c * c) * std::log(std::max(arg, 1.0));
}

/// Phase wind-up (Wu et al. 1993) using the same satellite/receiver frame as
/// CLASLIB's windupcorr().
double phaseWindup(const Vector3d& sat_pos,
                   const Vector3d& sat_vel,
                   const Vector3d& rcv_pos,
                   double prev) {
    if (sat_pos.squaredNorm() <= 0.0 ||
        sat_vel.squaredNorm() <= 0.0 ||
        rcv_pos.squaredNorm() <= 0.0) {
        return prev;
    }

    const Vector3d ek_vec = rcv_pos - sat_pos;
    if (ek_vec.norm() < 1e-10) return prev;
    const Vector3d ek = ek_vec.normalized();

    const Vector3d ezs_vec = -sat_pos;
    if (ezs_vec.norm() < 1e-10) return prev;
    const Vector3d ezs = ezs_vec.normalized();

    const Vector3d omega(0.0, 0.0, constants::OMEGA_E);
    const Vector3d ess_vec = sat_vel + omega.cross(sat_pos);
    if (ess_vec.norm() < 1e-10) return prev;
    const Vector3d ess = ess_vec.normalized();

    Vector3d eys_vec = ezs.cross(ess);
    if (eys_vec.norm() < 1e-10) return prev;
    const Vector3d eys = eys_vec.normalized();
    const Vector3d exs = eys.cross(ezs);

    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(rcv_pos, lat, lon, h);
    (void)h;
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);
    const Vector3d exr(-sin_lon, cos_lon, 0.0);
    const Vector3d eyr(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);

    const Vector3d eks = ek.cross(eys);
    const Vector3d ekr = ek.cross(eyr);
    const Vector3d ds = exs - ek * ek.dot(exs) - eks;
    const Vector3d dr = exr - ek * ek.dot(exr) + ekr;
    const double denom = ds.norm() * dr.norm();
    if (denom < 1e-20) return prev;

    const double cosp = std::clamp(ds.dot(dr) / denom, -1.0, 1.0);
    double ph = std::acos(cosp) / (2.0 * M_PI);
    if (ek.dot(ds.cross(dr)) < 0.0) {
        ph = -ph;
    }

    return ph + std::floor(prev - ph + 0.5);
}

bool gnsstimeIsSet(const GNSSTime& time) {
    return time.week != 0 || std::abs(time.tow) > 0.0;
}

void updateDispersionCompensation(
    OSRCorrection& osr,
    CLASDispersionCompensationInfo& compensation,
    const Observation* l1_obs,
    const Observation* l2_obs,
    const GNSSTime& obs_time,
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    if (usesClasPhaseBiasRepair(policy)) {
        compensation.reference_time = GNSSTime();
        compensation.base_phase_m = {0.0, 0.0};
        compensation.has_base = {false, false};
        compensation.slip = {false, false};
        return;
    }
    const GNSSTime interval_reference =
        osr.atmos_reference_time.week != 0 ? osr.atmos_reference_time : obs_time;
    if (compensation.reference_time.week == 0 ||
        compensation.reference_time != interval_reference) {
        compensation.reference_time = interval_reference;
        compensation.base_phase_m = {0.0, 0.0};
        compensation.has_base = {false, false};
        compensation.slip = {false, false};
        const Observation* freq_obs[2] = {l1_obs, l2_obs};
        for (int f = 0; f < 2; ++f) {
            const Observation* raw = freq_obs[f];
            if (raw != nullptr && raw->valid && raw->has_carrier_phase &&
                std::isfinite(raw->carrier_phase) && osr.wavelengths[f] > 0.0) {
                compensation.base_phase_m[static_cast<size_t>(f)] =
                    raw->carrier_phase * osr.wavelengths[f];
                compensation.has_base[static_cast<size_t>(f)] = true;
            }
        }
    }

    auto checkSlip = [&](const Observation* obs_ptr, int freq_idx) {
        if ((obs_ptr == nullptr || !obs_ptr->valid || !obs_ptr->has_carrier_phase ||
             !std::isfinite(obs_ptr->carrier_phase) || obs_ptr->loss_of_lock) &&
            compensation.reference_time.week != 0) {
            compensation.slip[static_cast<size_t>(freq_idx)] = true;
        }
    };
    checkSlip(l1_obs, 0);
    checkSlip(l2_obs, 1);

    if (!compensation.slip[0] && !compensation.slip[1] &&
        compensation.has_base[0] && compensation.has_base[1] &&
        l1_obs != nullptr && l2_obs != nullptr &&
        l1_obs->has_carrier_phase && l2_obs->has_carrier_phase &&
        std::isfinite(l1_obs->carrier_phase) &&
        std::isfinite(l2_obs->carrier_phase) &&
        osr.wavelengths[0] > 0.0 && osr.wavelengths[1] > 0.0) {
        const double l1_phase_m = l1_obs->carrier_phase * osr.wavelengths[0];
        const double l2_phase_m = l2_obs->carrier_phase * osr.wavelengths[1];
        const double fi = osr.wavelengths[1] / osr.wavelengths[0];
        const double denom = 1.0 - fi * fi;
        if (std::abs(denom) > 1e-9) {
            const double dgf = l1_phase_m - l2_phase_m -
                (compensation.base_phase_m[0] - compensation.base_phase_m[1]);
            osr.phase_compensation_m[0] = dgf / denom;
            osr.phase_compensation_m[1] = (fi * fi * dgf) / denom;
        }
    }
}

// Minimum elevation angle for satellite inclusion (radians, ~15 degrees)
constexpr double kElevationMaskRad = 0.26;
// Maximum time gap for phase bias repair before resetting (seconds)
constexpr double kPhaseBiasRepairTimeoutSeconds = 120.0;
// Expected SSR clock interval for SIS continuity detection (seconds)
constexpr double kSsrClockIntervalSeconds = 5.0;
// Expected phase bias lag for SIS continuity correction (seconds)
constexpr double kPhaseBiasLagSeconds = 30.0;
// Phase bias 100-cycle jump detection window (cycles)
constexpr double kPhaseBiasJumpLowerCycles = 95.0;
constexpr double kPhaseBiasJumpUpperCycles = 105.0;
constexpr double kPhaseBiasJumpCorrectionCycles = 100.0;

/// Update SIS (Signal-In-Space) continuity tracking for a satellite.
/// Detects clock epoch transitions and computes delta for SIS correction.
void updateSisContinuity(
    CLASSisContinuityInfo& info,
    const OSRCorrection& osr,
    bool clock_time_valid) {
    const double current_sis_m =
        -osr.ssr_exact_clock_m + osr.ssr_exact_orbit_los_m;
    if (!clock_time_valid) {
        info = CLASSisContinuityInfo{};
    } else if (!info.has_current) {
        info.current_time = osr.clock_reference_time;
        info.current_sis_m = current_sis_m;
        info.has_current = true;
    } else if (info.current_time != osr.clock_reference_time) {
        const double dt_clock = osr.clock_reference_time - info.current_time;
        info.previous_time = info.current_time;
        info.previous_sis_m = info.current_sis_m;
        info.current_time = osr.clock_reference_time;
        info.current_sis_m = current_sis_m;
        info.has_previous = true;
        if (std::abs(dt_clock - kSsrClockIntervalSeconds) < 0.5) {
            info.last_delta_m = info.current_sis_m - info.previous_sis_m;
            info.has_last_delta = true;
            if (pppDebugEnabled()) {
                std::cerr << "[OSR-SIS] " << osr.satellite.toString()
                          << " clk_ref_tow=" << osr.clock_reference_time.tow
                          << " sis_prev_m=" << info.previous_sis_m
                          << " sis_curr_m=" << info.current_sis_m
                          << " sis_delta_m=" << info.last_delta_m
                          << " exact_orb_los=" << osr.ssr_exact_orbit_los_m
                          << " exact_clk=" << osr.ssr_exact_clock_m
                          << "\n";
            }
        } else {
            info.last_delta_m = 0.0;
            info.has_last_delta = false;
        }
    }
}

/// Detect phase bias epoch change and update repair tracking state.
/// Returns {epoch_changed, phase_bias_dt} for use in PRC/CPC aggregation.
struct PhaseBiasEpochStatus {
    bool epoch_changed = false;
    double dt = 0.0;
};

PhaseBiasEpochStatus updatePhaseBiasRepairState(
    CLASPhaseBiasRepairInfo& repair_info,
    const GNSSTime& effective_reference_time,
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    PhaseBiasEpochStatus status;
    if (!usesClasPhaseBiasRepair(policy)) {
        resetPhaseBiasRepairInfo(repair_info);
    } else if (!gnsstimeIsSet(effective_reference_time)) {
        resetPhaseBiasRepairInfo(repair_info);
    } else if (repair_info.reference_time.week == 0 &&
               std::abs(repair_info.reference_time.tow) == 0.0) {
        repair_info.reference_time = effective_reference_time;
    } else if (repair_info.reference_time != effective_reference_time) {
        status.epoch_changed = true;
        status.dt = effective_reference_time - repair_info.reference_time;
        if (std::abs(status.dt) >= kPhaseBiasRepairTimeoutSeconds) {
            repair_info.offset_cycles = {0.0, 0.0, 0.0};
            repair_info.pending_state_shift_cycles = {0.0, 0.0, 0.0};
            repair_info.has_last = {false, false, false};
        }
        repair_info.reference_time = effective_reference_time;
    }
    return status;
}

}  // anonymous namespace

const char* clasPhaseContinuityPolicyName(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR:
            return "full-repair";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::SIS_CONTINUITY_ONLY:
            return "sis-continuity-only";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::REPAIR_ONLY:
            return "repair-only";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::RAW_PHASE_BIAS:
            return "raw-phase-bias";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::NO_PHASE_BIAS:
            return "no-phase-bias";
    }
    return "full-repair";
}

const char* clasPhaseBiasValuePolicyName(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::FULL:
            return "full";
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::PHASE_BIAS_ONLY:
            return "phase-bias-only";
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::COMPENSATION_ONLY:
            return "compensation-only";
    }
    return "full";
}

const char* clasPhaseBiasReferenceTimePolicyName(
    ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::PHASE_BIAS_REFERENCE:
            return "phase-bias-reference";
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::CLOCK_REFERENCE:
            return "clock-reference";
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::OBSERVATION_EPOCH:
            return "observation-epoch";
    }
    return "phase-bias-reference";
}

const char* clasSsrTimingPolicyName(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::LAG_TOLERANT:
            return "lag-tolerant";
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_PHASE_BIAS:
            return "clock-bound-phase-bias";
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS:
            return "clock-bound-atmos-and-phase-bias";
    }
    return "lag-tolerant";
}

const char* clasExpandedValueConstructionPolicyName(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::FULL_COMPOSED:
            return "full-composed";
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::RESIDUAL_ONLY:
            return "residual-only";
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::POLYNOMIAL_ONLY:
            return "polynomial-only";
    }
    return "full-composed";
}

bool usesClasPhaseBiasTerms(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::NO_PHASE_BIAS;
}

bool usesClasRawPhaseBiasValues(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::COMPENSATION_ONLY;
}

bool usesClasPhaseCompensationValues(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::PHASE_BIAS_ONLY;
}

GNSSTime selectClasPhaseBiasReferenceTime(
    ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy policy,
    const GNSSTime& phase_bias_reference_time,
    const GNSSTime& clock_reference_time,
    const GNSSTime& observation_time) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::PHASE_BIAS_REFERENCE:
            return phase_bias_reference_time;
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::CLOCK_REFERENCE:
            return gnsstimeIsSet(clock_reference_time) ? clock_reference_time
                                                       : phase_bias_reference_time;
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::OBSERVATION_EPOCH:
            return observation_time;
    }
    return phase_bias_reference_time;
}

bool usesClasSisContinuity(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR ||
           policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::SIS_CONTINUITY_ONLY;
}

bool usesClasPhaseBiasRepair(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR ||
           policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::REPAIR_ONLY;
}

bool usesClasClockBoundPhaseBias(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy) {
    return policy ==
               ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_PHASE_BIAS ||
           policy ==
               ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS;
}

bool usesClasClockBoundAtmos(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy) {
    return policy ==
           ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS;
}

bool usesClasExpandedPolynomialTerms(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::RESIDUAL_ONLY;
}

bool usesClasExpandedResidualTerms(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::POLYNOMIAL_ONLY;
}

int satelliteStecTokenCount(const std::map<std::string, std::string>& atmos_tokens,
                            const SatelliteId& satellite) {
    int count = 0;
    for (const auto& [key, _value] : atmos_tokens) {
        if (key.find("atmos_stec") != std::string::npos &&
            key.find(satellite.toString()) != std::string::npos) {
            ++count;
        }
    }
    return count;
}

int preferredClasNetworkId(const std::map<std::string, std::string>& atmos_tokens) {
    int network_id = 0;
    if (!ppp_atmosphere::parseAtmosTokenInt(atmos_tokens, "atmos_network_id", network_id)) {
        return 0;
    }
    return std::max(network_id, 0);
}

std::map<std::string, std::string> selectClasEpochAtmosTokens(
    const SSRProducts& ssr_products,
    const std::vector<SatelliteId>& satellites,
    const GNSSTime& time,
    const Vector3d& receiver_position,
    const ppp_shared::PPPConfig& config) {
    constexpr double kAtmosSelectionGapSeconds = 120.0;

    ClasAtmosCandidate best;

    for (const auto& satellite : satellites) {
        const auto sat_it = ssr_products.orbit_clock_corrections.find(satellite);
        if (sat_it == ssr_products.orbit_clock_corrections.end()) {
            continue;
        }
        for (const auto& correction : sat_it->second) {
            if (!correction.atmos_valid || correction.atmos_tokens.empty()) {
                continue;
            }
            const double time_offset = correction.time - time;
            const double time_gap = std::abs(time_offset);
            if (time_gap > kAtmosSelectionGapSeconds) {
                continue;
            }

            ppp_atmosphere::ClasGridReference grid_reference;
            const bool has_grid = ppp_atmosphere::resolveClasGridReference(
                correction.atmos_tokens, receiver_position, grid_reference);
            const double grid_distance_sq =
                has_grid
                    ? (grid_reference.dlat_deg * grid_reference.dlat_deg +
                       grid_reference.dlon_deg * grid_reference.dlon_deg)
                    : std::numeric_limits<double>::infinity();
            const ClasAtmosCandidate candidate{
                correction.atmos_tokens,
                has_grid,
                grid_distance_sq,
                time_gap,
                time_offset > 1e-9,
                static_cast<int>(correction.atmos_tokens.size()),
            };

            if (pppDebugEnabled() && std::abs(time.tow - 230420.0) < 1e-6) {
                int stec_token_count = 0;
                const int sat_token_count =
                    satelliteStecTokenCount(correction.atmos_tokens, satellite);
                for (const auto& [key, _value] : correction.atmos_tokens) {
                    if (key.find("atmos_stec") != std::string::npos) {
                        ++stec_token_count;
                    }
                }
                std::cerr << "[PPP-ATMOS-CAND] sat=" << satellite.toString()
                          << " corr_tow=" << correction.time.tow
                          << " tokens=" << correction.atmos_tokens.size()
                          << " stec_tokens=" << stec_token_count
                          << " sat_tokens=" << sat_token_count
                          << " has_grid=" << static_cast<int>(has_grid)
                          << " grid_dist2=" << grid_distance_sq
                          << " dt=" << time_gap
                          << "\n";
            }

            if (!isBetterClasAtmosCandidate(candidate, best, config)) {
                continue;
            }

            best = candidate;
        }
    }

    if (config.clas_atmos_selection_policy ==
            ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED &&
        !best.tokens.empty() &&
        best.time_gap > config.clas_atmos_stale_after_seconds) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-ATMOS] rejected stale nearest-grid network="
                      << preferredClasNetworkId(best.tokens)
                      << " dt=" << best.time_gap
                      << " stale_after_s=" << config.clas_atmos_stale_after_seconds << "\n";
        }
        return {};
    }

    if (pppDebugEnabled() && !best.tokens.empty()) {
        std::cerr << "[PPP-ATMOS] selected network=" << preferredClasNetworkId(best.tokens)
                  << " tokens=" << best.tokens.size()
                  << " grid_selected=" << static_cast<int>(best.has_grid)
                  << " dt=" << best.time_gap
                  << " policy=" << clasAtmosSelectionPolicyName(config.clas_atmos_selection_policy)
                  << " stale_after_s=" << config.clas_atmos_stale_after_seconds << "\n";
    }

    return best.tokens;
}

std::map<std::string, std::string> selectClasSatelliteAtmosTokens(
    const SSRProducts& ssr_products,
    const SatelliteId& satellite,
    const GNSSTime& time,
    const Vector3d& receiver_position,
    const ppp_shared::PPPConfig& config) {
    constexpr double kAtmosSelectionGapSeconds = 120.0;

    const auto sat_it = ssr_products.orbit_clock_corrections.find(satellite);
    if (sat_it == ssr_products.orbit_clock_corrections.end()) {
        return {};
    }

    ClasAtmosCandidate best;
    for (const auto& correction : sat_it->second) {
        if (!correction.atmos_valid || correction.atmos_tokens.empty()) {
            continue;
        }
        if (satelliteStecTokenCount(correction.atmos_tokens, satellite) <= 0) {
            continue;
        }
        const double time_offset = correction.time - time;
        const double time_gap = std::abs(time_offset);
        if (time_gap > kAtmosSelectionGapSeconds) {
            continue;
        }

        ppp_atmosphere::ClasGridReference grid_reference;
        const bool has_grid = ppp_atmosphere::resolveClasGridReference(
            correction.atmos_tokens, receiver_position, grid_reference);
        const double grid_distance_sq =
            has_grid
                ? (grid_reference.dlat_deg * grid_reference.dlat_deg +
                   grid_reference.dlon_deg * grid_reference.dlon_deg)
                : std::numeric_limits<double>::infinity();
        const ClasAtmosCandidate candidate{
            correction.atmos_tokens,
            has_grid,
            grid_distance_sq,
            time_gap,
            time_offset > 1e-9,
            static_cast<int>(correction.atmos_tokens.size()),
        };

        if (!isBetterClasAtmosCandidate(candidate, best, config)) {
            continue;
        }
        best = candidate;
    }

    if (pppDebugEnabled() && !best.tokens.empty() &&
        std::abs(time.tow - 230420.0) < 1e-6) {
        if (config.clas_atmos_selection_policy ==
                ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED &&
            best.time_gap > config.clas_atmos_stale_after_seconds) {
            std::cerr << "[PPP-ATMOS-SAT] rejected stale nearest-grid network="
                      << preferredClasNetworkId(best.tokens)
                      << " dt=" << best.time_gap
                      << " stale_after_s=" << config.clas_atmos_stale_after_seconds << "\n";
            return {};
        }
        std::cerr << "[PPP-ATMOS-SAT] sat=" << satellite.toString()
                  << " network=" << preferredClasNetworkId(best.tokens)
                  << " tokens=" << best.tokens.size()
                  << " dt=" << best.time_gap
                  << " grid_selected=" << static_cast<int>(best.has_grid)
                  << " policy="
                  << clasAtmosSelectionPolicyName(config.clas_atmos_selection_policy)
                  << "\n";
    }

    if (config.clas_atmos_selection_policy ==
            ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED &&
        !best.tokens.empty() &&
        best.time_gap > config.clas_atmos_stale_after_seconds) {
        return {};
    }

    return best.tokens;
}

CLASEpochContext prepareClasEpochContext(
    const ObservationData& obs,
    const NavigationData& nav,
    const SSRProducts& ssr,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, double>& prev_windup,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASSisContinuityInfo>& sis_continuity,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair) {
    CLASEpochContext context;
    context.receiver_clock_m = receiver_clk;
    context.trop_zenith_m = trop_zenith;
    context.epoch_atmos_tokens =
        selectClasEpochAtmosTokens(ssr, obs.getSatellites(), obs.time, receiver_pos, config);

    if (config.apply_tides_to_receiver_position) {
        const auto tide_components = calculateClasTideComponents(
            receiver_pos, obs.time, context.epoch_atmos_tokens, config);
        context.tide_displacement_ecef = tide_components.total();
    }
    context.receiver_position = receiver_pos + context.tide_displacement_ecef;

    context.osr_corrections = computeOSR(
        obs,
        nav,
        ssr,
        context.epoch_atmos_tokens,
        context.receiver_position,
        receiver_clk,
        trop_zenith,
        config,
        prev_windup,
        dispersion_compensation,
        sis_continuity,
        phase_bias_repair,
        context.tide_displacement_ecef);
    return context;
}

std::vector<OSRCorrection> computeOSR(
    const ObservationData& obs,
    const NavigationData& nav,
    const SSRProducts& ssr,
    const std::map<std::string, std::string>& epoch_atmos_tokens,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, double>& prev_windup,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASSisContinuityInfo>& sis_continuity,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    const Vector3d& receiver_tide_displacement) {

    std::vector<OSRCorrection> corrections;
    int preferred_network_id = 0;
    ppp_atmosphere::parseAtmosTokenInt(
        epoch_atmos_tokens, "atmos_network_id", preferred_network_id);
    ppp_shared::PPPConfig trop_config = config;
    trop_config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::FRESHNESS_FIRST;
    const std::map<std::string, std::string> fresh_epoch_atmos_tokens =
        selectClasEpochAtmosTokens(
            ssr, obs.getSatellites(), obs.time, receiver_pos, trop_config);

    const bool debug_enabled = pppDebugEnabled();
    const ReceiverAntennaModel* receiver_antenna_model =
        findReceiverAntennaModel(config);

    // Receiver tide displacement is either supplied by the CLASLIB parity
    // context or computed on demand for observation-side tide correction.
    Vector3d tide_displacement_model = receiver_tide_displacement;
    if (tide_displacement_model.squaredNorm() <= 0.0) {
        if (config.apply_tide_as_osr &&
            (config.apply_clas_grid_ocean_loading || config.apply_pole_tide ||
             !config.eop_file_path.empty() || !config.clas_grid_blq_file_path.empty())) {
            tide_displacement_model =
                calculateClasTideComponents(
                    receiver_pos, obs.time, epoch_atmos_tokens, config).total();
        } else if (config.apply_tide_as_osr || debug_enabled) {
            tide_displacement_model =
                tidal::calculateSolidEarthTides(receiver_pos, obs.time);
        }
    }
    const Vector3d tide_displacement =
        config.apply_tide_as_osr ? tide_displacement_model : Vector3d::Zero();

    for (const auto& sat : obs.getSatellites()) {
        OSRCorrection osr;
        osr.satellite = sat;
        const std::string sat_id = sat.toString();
        const bool focused_g14_dump =
            obs.time.week == 2068 &&
            std::abs(obs.time.tow - 230420.0) <= 1e-6 &&
            sat_id == "G14";

        // --- 1. Find L1/L2 observations ---
        const auto findSignal = [&](const std::vector<SignalType>& candidates)
            -> const Observation* {
            for (auto sig : candidates) {
                const Observation* o = obs.getObservation(sat, sig);
                if (o && o->valid && o->has_carrier_phase && o->has_pseudorange) return o;
            }
            return nullptr;
        };

        std::vector<SignalType> l1_cands, l2_cands;
        switch (sat.system) {
            case GNSSSystem::GPS:
                l1_cands = {SignalType::GPS_L1CA, SignalType::GPS_L1P};
                l2_cands = {SignalType::GPS_L2C, SignalType::GPS_L2P, SignalType::GPS_L5};
                break;
            case GNSSSystem::Galileo:
                l1_cands = {SignalType::GAL_E1};
                l2_cands = {SignalType::GAL_E5A, SignalType::GAL_E5B};
                break;
            case GNSSSystem::QZSS:
                l1_cands = {SignalType::QZS_L1CA};
                l2_cands = {SignalType::QZS_L2C, SignalType::QZS_L5};
                break;
            default:
                continue;
        }

        const Observation* l1_obs = findSignal(l1_cands);
        const Observation* l2_obs = findSignal(l2_cands);
        if (!l1_obs) continue;

        // --- 2. Satellite position/clock from broadcast + SSR ---
        const SSROrbitClockCorrection* orbit_source =
            pickHeldOrbitCorrection(ssr, sat, obs.time, preferred_network_id);
        const int ssr_iode =
            (orbit_source != nullptr && orbit_source->orbit_valid) ? orbit_source->iode : -1;
        Vector3d sat_pos, sat_vel;
        double sat_clk = 0.0, sat_drift = 0.0;
        GNSSTime satellite_time = obs.time;
        const Ephemeris* eph_for_sat_time = nullptr;
        if (config.wlnl_strict_claslib_parity && l1_obs->pseudorange > 0.0) {
            const Ephemeris* clock_eph = nav.getEphemeris(sat, obs.time);
            if (clock_eph == nullptr) {
                continue;
            }
            const double travel_time = l1_obs->pseudorange / constants::SPEED_OF_LIGHT;
            GNSSTime raw_transmit_time = obs.time - travel_time;
            Vector3d unused_pos, unused_vel;
            if (!clock_eph->calculateSatelliteState(
                    raw_transmit_time, unused_pos, unused_vel, sat_clk, sat_drift)) {
                continue;
            }
            satellite_time = raw_transmit_time - sat_clk;
            eph_for_sat_time =
                selectSsrEphemerisByIode(nav, sat, obs.time, ssr_iode);
            if (eph_for_sat_time == nullptr) {
                eph_for_sat_time = nav.getEphemeris(sat, obs.time);
            }
            if (eph_for_sat_time == nullptr ||
                !eph_for_sat_time->calculateSatelliteState(
                    satellite_time, sat_pos, sat_vel, sat_clk, sat_drift)) {
                continue;
            }
        } else {
            if (!nav.calculateSatelliteState(sat, obs.time, sat_pos, sat_vel, sat_clk, sat_drift)) {
                continue;
            }

            // Re-evaluate satellite state at signal transmission time. Without
            // this, CLAS per-frequency residuals stay at the 20-40 m level.
            if (l1_obs->pseudorange > 0.0) {
                const double travel_time = l1_obs->pseudorange / constants::SPEED_OF_LIGHT;
                satellite_time = obs.time - travel_time + sat_clk;
                if (!nav.calculateSatelliteState(
                        sat, satellite_time, sat_pos, sat_vel, sat_clk, sat_drift)) {
                    continue;
                }
            }
            eph_for_sat_time = nav.getEphemeris(sat, satellite_time);
        }
        const Vector3d sat_pos_for_ssr_frame = sat_pos;
        const Vector3d sat_vel_for_ssr_frame = sat_vel;
        const double sat_clk_bcast_s = sat_clk;

        // Apply SSR orbit/clock corrections
        Vector3d orbit_corr = Vector3d::Zero();
        Vector3d orbit_corr_rac = Vector3d::Zero();
        double clock_corr = 0.0;
        double ura_sigma = 0.0;
        std::map<uint8_t, double> ssr_cbias, ssr_pbias;
        std::map<std::string, std::string> sat_atmos_tokens;
        GNSSTime atmos_reference_time;
        GNSSTime phase_bias_reference_time;
        GNSSTime orbit_reference_time;
        GNSSTime clock_reference_time;
        if (ssr.interpolateCorrection(sat, obs.time, orbit_corr, clock_corr,
                                       &ura_sigma, &ssr_cbias, &ssr_pbias,
                                       &sat_atmos_tokens,
                                       &atmos_reference_time,
                                       &phase_bias_reference_time,
                                       &clock_reference_time,
                                       preferred_network_id)) {
            orbit_corr_rac = orbit_corr;
            orbit_reference_time =
                orbit_source != nullptr ? orbit_source->time : GNSSTime();
            const SSROrbitClockCorrection* clock_source =
                pickHeldClockCorrection(ssr, sat, obs.time, preferred_network_id);
            if (!gnsstimeIsSet(clock_reference_time) && clock_source != nullptr) {
                clock_reference_time = clock_source->time;
            }
            orbit_corr = orbitCorrectionToEcef(
                orbit_corr,
                sat_pos_for_ssr_frame,
                sat_vel_for_ssr_frame,
                ssr.orbitCorrectionsAreRac());
            if (pppDebugEnabled() && corrections.size() < 20) {
                std::cerr << "[OSR-SSR] " << sat.toString()
                          << " orbit_ecef=" << orbit_corr.transpose()
                          << " clk_m=" << clock_corr
                          << " brdc_clk_s=" << sat_clk
                          << " sat_clk_total_s=" << sat_clk + clock_corr / constants::SPEED_OF_LIGHT
                          << " cbias_n=" << ssr_cbias.size()
                          << " pbias_n=" << ssr_pbias.size() << "\n";
            }
            sat_pos += orbit_corr;
            sat_clk += clock_corr / constants::SPEED_OF_LIGHT;
            osr.has_code_bias = !ssr_cbias.empty();
            osr.has_phase_bias = !ssr_pbias.empty();
            osr.atmos_reference_time = atmos_reference_time;
            osr.phase_bias_reference_time = phase_bias_reference_time;
            osr.clock_reference_time = clock_reference_time;
            osr.clock_correction_m = clock_corr;
        } else {
            continue;
        }

        const auto ssr_timing_policy = config.clas_ssr_timing_policy;
        const bool clock_ref_valid = gnsstimeIsSet(osr.clock_reference_time);
        const bool phase_bias_ref_valid = gnsstimeIsSet(osr.phase_bias_reference_time);
        const bool atmos_ref_valid = gnsstimeIsSet(osr.atmos_reference_time);
        if (usesClasClockBoundPhaseBias(ssr_timing_policy) &&
            clock_ref_valid &&
            phase_bias_ref_valid &&
            osr.phase_bias_reference_time != osr.clock_reference_time) {
            ssr_pbias.clear();
            osr.has_phase_bias = false;
        }
        if (usesClasClockBoundAtmos(ssr_timing_policy) &&
            clock_ref_valid &&
            atmos_ref_valid &&
            osr.atmos_reference_time != osr.clock_reference_time) {
            sat_atmos_tokens.clear();
            osr.atmos_reference_time = GNSSTime();
        }

        std::map<std::string, std::string> trop_atmos_tokens = fresh_epoch_atmos_tokens;
        if (trop_atmos_tokens.empty()) {
            trop_atmos_tokens = epoch_atmos_tokens;
        }
        if (trop_atmos_tokens.empty()) {
            trop_atmos_tokens = sat_atmos_tokens;
        }
        std::map<std::string, std::string> iono_atmos_tokens;
        if (config.wlnl_strict_claslib_parity) {
            if (satelliteStecTokenCount(epoch_atmos_tokens, sat) > 0) {
                iono_atmos_tokens = epoch_atmos_tokens;
            }
        } else {
            const std::map<std::string, std::string> sat_specific_iono_tokens =
                selectClasSatelliteAtmosTokens(ssr, sat, obs.time, receiver_pos, config);
            if (!sat_specific_iono_tokens.empty()) {
                iono_atmos_tokens = sat_specific_iono_tokens;
            } else if (satelliteStecTokenCount(sat_atmos_tokens, sat) > 0) {
                iono_atmos_tokens = sat_atmos_tokens;
            } else if (satelliteStecTokenCount(epoch_atmos_tokens, sat) > 0) {
                iono_atmos_tokens = epoch_atmos_tokens;
            }
        }
        if ((!trop_atmos_tokens.empty() || !iono_atmos_tokens.empty()) &&
            !gnsstimeIsSet(osr.atmos_reference_time)) {
            osr.atmos_reference_time = obs.time;
        }

        osr.satellite_position = sat_pos;
        osr.satellite_velocity = sat_vel;
        osr.satellite_clock_bias_s = sat_clk;

        // --- 3. Geometry ---
        const double geo_range = geodist(sat_pos, receiver_pos);
        const double rho_no_sagnac = (sat_pos - receiver_pos).norm();
        const double sagnac = geo_range - rho_no_sagnac;
        dumpClasSatSsr(
            config,
            obs,
            sat,
            satellite_time,
            eph_for_sat_time,
            receiver_pos,
            sat_pos_for_ssr_frame,
            sat_vel_for_ssr_frame,
            sat_clk_bcast_s,
            orbit_reference_time,
            orbit_corr_rac,
            ssr_iode,
            orbit_corr,
            clock_reference_time,
            clock_corr,
            sat_pos,
            sat_clk,
            rho_no_sagnac,
            sagnac);
        const Vector3d los = (sat_pos - receiver_pos) / geo_range;
        osr.orbit_projection_m = los.dot(orbit_corr);
        osr.ssr_exact_orbit_los_m = osr.orbit_projection_m;
        osr.ssr_exact_clock_m = osr.clock_correction_m;
        if (gnsstimeIsSet(osr.clock_reference_time)) {
            double exact_clock_corr = 0.0;
            GNSSTime exact_clock_reference_time;
            Vector3d unused_orbit_corr = Vector3d::Zero();
            if (ssr.interpolateCorrection(sat,
                                          osr.clock_reference_time,
                                          unused_orbit_corr,
                                          exact_clock_corr,
                                          nullptr,
                                          nullptr,
                                          nullptr,
                                          nullptr,
                                          nullptr,
                                          nullptr,
                                          &exact_clock_reference_time,
                                          preferred_network_id)) {
                if (gnsstimeIsSet(exact_clock_reference_time) &&
                    exact_clock_reference_time == osr.clock_reference_time) {
                    const SSROrbitClockCorrection* exact_orbit_source =
                        pickHeldOrbitCorrection(
                            ssr, sat, obs.time, preferred_network_id);
                    if (exact_orbit_source == nullptr) {
                        osr.ssr_exact_clock_m = exact_clock_corr;
                    } else {
                        Vector3d exact_orbit_corr =
                            exact_orbit_source->orbit_correction_ecef;
                        osr.ssr_exact_clock_m = exact_clock_corr;
                        exact_orbit_corr = orbitCorrectionToEcef(
                            exact_orbit_corr,
                            sat_pos_for_ssr_frame,
                            sat_vel_for_ssr_frame,
                            ssr.orbitCorrectionsAreRac());
                        osr.ssr_exact_orbit_los_m = los.dot(exact_orbit_corr);
                    }
                }
            }
        }
        double lat = 0.0, lon = 0.0, h = 0.0;
        ecef2geodetic(receiver_pos, lat, lon, h);
        const Vector3d los_enu = ecef2enu(sat_pos - receiver_pos, lat, lon);
        const double elev = std::atan2(los_enu.z(), std::hypot(los_enu.x(), los_enu.y()));
        const double azim = std::atan2(los_enu.x(), los_enu.y());
        if (debug_enabled && focused_g14_dump) {
            const Vector3d tide_enu = ecef2enu(tide_displacement_model, lat, lon);
            const double tide_los_m = los.dot(tide_displacement_model);
            std::cerr << std::setprecision(15)
                      << "[OSR-TIDE] source=lib"
                      << " week=" << obs.time.week
                      << " tow=" << obs.time.tow
                      << " sat=" << sat_id
                      << " tide_ecef=" << tide_displacement_model.transpose()
                      << " tide_enu=" << tide_enu.transpose()
                      << " tide_los_m=" << tide_los_m
                      << " applied=" << (config.apply_tide_as_osr ? 1 : 0)
                      << "\n";
        }

        if (elev < kElevationMaskRad) {
            if (debug_enabled && corrections.size() < 20) {
                std::cerr << "[OSR-DROP] " << sat_id
                          << " reason=elevation"
                          << " elev_deg=" << (elev * 180.0 / M_PI)
                          << " recv_pos=" << receiver_pos.transpose()
                          << "\n";
            }
            continue;
        }

        osr.elevation = elev;
        osr.azimuth = azim;

        // --- 3b. Solid earth tide (observation-side) ---
        // Project the epoch-wide tide displacement onto this satellite's LOS.
        if (config.apply_tide_as_osr) {
            osr.solid_earth_tide_m = los.dot(tide_displacement);
        }
        if (receiver_tide_displacement.squaredNorm() > 0.0) {
            osr.tide_geometry_m = -los.dot(receiver_tide_displacement);
        }

        const Ephemeris* eph = nav.getEphemeris(sat, obs.time);
        osr.signals[0] = l1_obs->signal;
        osr.frequencies[0] = signalFrequencyHz(l1_obs->signal, eph);
        osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
        osr.num_frequencies = 1;
        if (l2_obs) {
            osr.signals[1] = l2_obs->signal;
            osr.frequencies[1] = signalFrequencyHz(l2_obs->signal, eph);
            osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
            osr.num_frequencies = 2;
        }
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const auto ant = receiverAntennaCorrection(
                config,
                receiver_antenna_model,
                osr.signals[f],
                azim,
                elev,
                lat,
                lon);
            osr.receiver_antenna_m[f] = ant.total_m;
            if (debug_enabled && focused_g14_dump) {
                std::cerr << std::setprecision(15)
                          << "[OSR-RCV-ANT] source=lib"
                          << " week=" << obs.time.week
                          << " tow=" << obs.time.tow
                          << " sat=" << sat_id
                          << " f=" << f
                          << " signal=" << static_cast<int>(osr.signals[f])
                          << " antenna=\"" << normalizeAntennaType(config.receiver_antenna_type) << "\""
                          << " has_antex=" << (ant.has_antex ? 1 : 0)
                          << " offset_enu=" << ant.offset_enu.transpose()
                          << " offset_ecef=" << ant.offset_ecef.transpose()
                          << " los_enu=" << los_enu.normalized().transpose()
                          << " pco_m=" << ant.pco_m
                          << " pcv_m=" << ant.pcv_m
                          << " total_m=" << ant.total_m
                          << "\n";
            }
        }

        // --- 4. Troposphere ---
        // Use Saastamoinen as trop fallback when CLAS grid trop is unavailable.
        osr.trop_correction_m = models::tropDelaySaastamoinen(receiver_pos, elev);

        // CLAS troposphere grid correction (if available)
        if (!trop_atmos_tokens.empty()) {
            const double clas_trop = ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                trop_atmos_tokens,
                receiver_pos,
                obs.time,
                elev,
                config.clas_expanded_value_construction_policy,
                config.clas_subtype12_value_construction_policy,
                config.clas_expanded_residual_sampling_policy);
            if (std::isfinite(clas_trop) && std::abs(clas_trop) > 0.0) {
                // Sanity check: CLAS grid trop should be within 30% of Saastamoinen.
                // Distant networks produce unrealistic trop values.
                const double saastamoinen = osr.trop_correction_m;
                if (saastamoinen > 0.1 &&
                    std::abs(clas_trop - saastamoinen) / saastamoinen < 0.3) {
                    osr.trop_correction_m = clas_trop;
                }
            }
        }

        // --- 5. Relativity ---
        osr.relativity_correction_m = relativisticCorrection(sat_pos, sat_vel, receiver_pos);

        // --- 6. Ionosphere (STEC) ---
        if (!iono_atmos_tokens.empty()) {
            const double stec_tecu = ppp_atmosphere::atmosphericStecTecu(
                iono_atmos_tokens,
                sat,
                receiver_pos,
                config.clas_expanded_value_construction_policy,
                config.clas_subtype12_value_construction_policy,
                config.clas_expanded_residual_sampling_policy);
            if (pppDebugEnabled() && corrections.size() < 20) {
                const int sat_token_count =
                    satelliteStecTokenCount(iono_atmos_tokens, sat);
                int stec_token_count = 0;
                for (const auto& [key, _value] : iono_atmos_tokens) {
                    if (key.find("atmos_stec") != std::string::npos) {
                        ++stec_token_count;
                    }
                }
                std::cerr << "[OSR-IONO] " << sat.toString()
                          << " stec_tecu=" << stec_tecu
                          << " token_count=" << iono_atmos_tokens.size()
                          << " stec_tokens=" << stec_token_count
                          << " sat_tokens=" << sat_token_count
                          << "\n";
            }
            if (std::isfinite(stec_tecu) && std::abs(stec_tecu) > 0.001) {
                // CLASLIB stores the ionosphere term as 40.3e16/(FREQ1*FREQ2)*STEC
                // and applies the per-frequency scaling later in PRC/CPC.
                osr.iono_l1_m =
                    40.3e16 * stec_tecu /
                    (constants::GPS_L1_FREQ * constants::GPS_L2_FREQ);
                osr.has_iono = true;
            }
        }
        if (!osr.has_iono) {
            if (pppDebugEnabled() && corrections.size() < 20) {
                std::cerr << "[OSR-DROP] " << sat.toString()
                          << " reason=no_iono"
                          << " has_tokens=" << (!iono_atmos_tokens.empty() ? 1 : 0)
                          << "\n";
            }
            continue;  // CLASLIB rejects satellites without STEC
        }

        // --- 7. Code/Phase bias ---
        // L2C(8) and L2P(9) are interchangeable for bias lookup: some satellites
        // only broadcast one of the two, so fall back to the other if not found.
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const uint8_t sid = rtcmSsrSignalId(sat.system, osr.signals[f]);
            uint8_t preferred_sid = sid;
            uint8_t alt_sid = 0U;
            if (sat.system == GNSSSystem::GPS &&
                osr.signals[f] == SignalType::GPS_L2C) {
                preferred_sid = 9U;
                alt_sid = 8U;
            } else if (sid == 8U || sid == 9U) {
                alt_sid = (sid == 8U) ? 9U : 8U;
            }
            auto cb_it = ssr_cbias.find(preferred_sid);
            auto pb_it = ssr_pbias.find(preferred_sid);
            if (alt_sid != 0U &&
                (cb_it == ssr_cbias.end() || pb_it == ssr_pbias.end())) {
                if (cb_it == ssr_cbias.end()) {
                    cb_it = ssr_cbias.find(alt_sid);
                }
                if (pb_it == ssr_pbias.end()) {
                    pb_it = ssr_pbias.find(alt_sid);
                }
            }
            osr.code_bias_m[f] = (cb_it != ssr_cbias.end()) ? cb_it->second : 0.0;
            osr.phase_bias_m[f] = (pb_it != ssr_pbias.end()) ? pb_it->second : 0.0;
        }

        const auto phase_continuity_policy =
            config.clas_phase_continuity_policy;
        if (osr.num_frequencies >= 2) {
            updateDispersionCompensation(
                osr, dispersion_compensation[sat], l1_obs, l2_obs, obs.time,
                phase_continuity_policy);
        }

        // --- 8. Phase wind-up ---
        double& wu = prev_windup[sat];
        wu = phaseWindup(sat_pos, sat_vel, receiver_pos, wu);
        osr.windup_cycles = wu;
        for (int f = 0; f < osr.num_frequencies; ++f) {
            osr.windup_m[f] = wu * osr.wavelengths[f];
        }

        auto& phase_bias_repair_info = phase_bias_repair[sat];
        auto& sis_continuity_info = sis_continuity[sat];
        const bool clock_time_valid = gnsstimeIsSet(osr.clock_reference_time);
        updateSisContinuity(sis_continuity_info, osr, clock_time_valid);
        const GNSSTime effective_phase_bias_reference_time =
            selectClasPhaseBiasReferenceTime(
                config.clas_phase_bias_reference_time_policy,
                osr.phase_bias_reference_time,
                osr.clock_reference_time,
                obs.time);
        const auto pbias_status = updatePhaseBiasRepairState(
            phase_bias_repair_info, effective_phase_bias_reference_time,
            phase_continuity_policy);
        const bool phase_bias_epoch_changed = pbias_status.epoch_changed;
        const double phase_bias_dt = pbias_status.dt;

        // --- 8b. GF-based dispersion compensation (CLASLIB compensatedisp) ---
        if (phase_bias_epoch_changed &&
            osr.num_frequencies >= 2 &&
            usesClasPhaseBiasRepair(phase_continuity_policy) &&
            phase_bias_repair_info.has_prev_ssr_epoch &&
            osr.frequencies[0] > 0.0 && osr.frequencies[1] > 0.0) {
            const double f1 = osr.frequencies[0];
            const double f2 = osr.frequencies[1];
            const double fi = osr.wavelengths[1] / osr.wavelengths[0];  // λ2/λ1
            const double fi2 = fi * fi;
            const double iono_factor = -(f2 / f1) * (1.0 - fi2);
            const double disp_curr = iono_factor * osr.iono_l1_m
                + osr.phase_bias_m[0] - osr.phase_bias_m[1];
            const double disp_prev = iono_factor * phase_bias_repair_info.prev_iono_l1_m
                + phase_bias_repair_info.prev_phase_bias_m[0]
                - phase_bias_repair_info.prev_phase_bias_m[1];
            const double dt_ssr = phase_bias_dt;
            if (std::abs(dt_ssr) > 0.1 && std::abs(dt_ssr) < kPhaseBiasRepairTimeoutSeconds) {
                const double coef = (disp_curr - disp_prev) / dt_ssr;
                const double norm = std::abs(iono_factor);
                // Sanity check: reject rates that are too large (adapted from CLASLIB)
                if (norm > 0.0 && std::abs(coef / norm) <= 1.0) {
                    phase_bias_repair_info.dispersion_rate_m_per_s[0] =
                        coef / (1.0 - fi2);
                    phase_bias_repair_info.dispersion_rate_m_per_s[1] =
                        fi2 / (1.0 - fi2) * coef;
                } else {
                    phase_bias_repair_info.dispersion_rate_m_per_s[0] = 0.0;
                    phase_bias_repair_info.dispersion_rate_m_per_s[1] = 0.0;
                }
            }
        }
        if (phase_bias_epoch_changed) {
            for (int ff = 0; ff < osr.num_frequencies && ff < OSR_MAX_FREQ; ++ff) {
                phase_bias_repair_info.prev_phase_bias_m[static_cast<size_t>(ff)] =
                    osr.phase_bias_m[ff];
            }
            phase_bias_repair_info.prev_iono_l1_m = osr.iono_l1_m;
            phase_bias_repair_info.prev_ssr_epoch_time = effective_phase_bias_reference_time;
            phase_bias_repair_info.has_prev_ssr_epoch = true;
        }
        // Apply SSR-based dispersion compensation to phase_compensation_m
        if (phase_bias_repair_info.has_prev_ssr_epoch &&
            usesClasPhaseBiasRepair(phase_continuity_policy)) {
            const double dt_from_ssr =
                obs.time - phase_bias_repair_info.prev_ssr_epoch_time;
            for (int ff = 0; ff < osr.num_frequencies && ff < OSR_MAX_FREQ; ++ff) {
                osr.phase_compensation_m[ff] +=
                    phase_bias_repair_info.dispersion_rate_m_per_s[static_cast<size_t>(ff)]
                    * dt_from_ssr;
            }
        }

        // --- 9. Aggregate PRC/CPC (CLASLIB L282-285) ---
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const double fi = osr.frequencies[f] > 0.0 ? osr.wavelengths[f] / osr.wavelengths[0] : 1.0;
            const double iono_scaled =
                fi * fi * (constants::GPS_L2_FREQ / constants::GPS_L1_FREQ) * osr.iono_l1_m;
            const auto phase_bias_value_policy =
                config.clas_phase_bias_value_policy;
            const double phase_bias_term =
                usesClasPhaseBiasTerms(phase_continuity_policy) &&
                        usesClasRawPhaseBiasValues(phase_bias_value_policy) ?
                    osr.phase_bias_m[f] :
                    0.0;
            const double phase_compensation_term =
                usesClasPhaseBiasTerms(phase_continuity_policy) &&
                        usesClasPhaseCompensationValues(phase_bias_value_policy) ?
                    osr.phase_compensation_m[f] :
                    0.0;

            osr.PRC[f] = osr.trop_correction_m + osr.relativity_correction_m
                       + osr.receiver_antenna_m[f] + iono_scaled + osr.code_bias_m[f]
                       + osr.solid_earth_tide_m;

            osr.CPC[f] = osr.trop_correction_m + osr.relativity_correction_m
                       + osr.receiver_antenna_m[f] - iono_scaled
                       + phase_bias_term + osr.windup_m[f]
                       + phase_compensation_term
                       + osr.solid_earth_tide_m;

            if (clock_time_valid && gnsstimeIsSet(effective_phase_bias_reference_time) &&
                sis_continuity_info.has_last_delta &&
                usesClasSisContinuity(phase_continuity_policy)) {
                const double pbias_lag =
                    osr.clock_reference_time - effective_phase_bias_reference_time;
                if (std::abs(pbias_lag - kPhaseBiasLagSeconds) < 0.5) {
                    osr.CPC[f] -= sis_continuity_info.last_delta_m;
                    osr.PRC[f] -= sis_continuity_info.last_delta_m;
                    if (debug_enabled && f == 0) {
                        std::cerr << "[OSR-SIS] " << sat_id
                                  << " lag_s=" << pbias_lag
                                  << " sis_delta_m=" << sis_continuity_info.last_delta_m
                                  << " ref_policy="
                                  << clasPhaseBiasReferenceTimePolicyName(
                                         config.clas_phase_bias_reference_time_policy)
                                  << "\n";
                    }
                }
            }

            const double continuity_term =
                osr.orbit_projection_m - osr.clock_correction_m + osr.CPC[f];
            if (usesClasPhaseBiasRepair(phase_continuity_policy)) {
                phase_bias_repair_info.last_continuity_m[static_cast<size_t>(f)] =
                    continuity_term;
                phase_bias_repair_info.has_last[static_cast<size_t>(f)] = true;
            }

            // Absorb phase bias changes across SSR epochs into the ambiguity
            // state.  When phase_bias_m changes between SSR updates, CPC jumps
            // by the delta.  Without compensation, the filter treats this as
            // an ambiguity change, resetting convergence.
        }

        osr.valid = true;
        const bool focused_gps_bias_dump =
            obs.time.week == 2068 &&
            obs.time.tow >= 230420.0 - 1e-6 &&
            obs.time.tow <= 230425.0 + 1e-6 &&
            isGpsClasParityTarget(sat);
        if (debug_enabled &&
            (corrections.size() < 3 || sat_id == "G31" || focused_gps_bias_dump || focused_g14_dump)) {
            std::cerr << "[OSR] " << sat_id
                      << " sig0=" << static_cast<int>(osr.signals[0])
                      << " sig1=" << (osr.num_frequencies >= 2 ?
                          static_cast<int>(osr.signals[1]) : -1)
                      << " trop=" << osr.trop_correction_m
                      << " rel=" << osr.relativity_correction_m
                      << " iono_l1=" << osr.iono_l1_m
                      << " cbias0=" << osr.code_bias_m[0]
                      << " cbias1=" << (osr.num_frequencies >= 2 ? osr.code_bias_m[1] : 0.0)
                      << " pbias0=" << osr.phase_bias_m[0]
                      << " pbias1=" << (osr.num_frequencies >= 2 ? osr.phase_bias_m[1] : 0.0)
                      << " windup=" << osr.windup_cycles
                      << " rcv_ant0=" << osr.receiver_antenna_m[0]
                      << " rcv_ant1=" << (osr.num_frequencies >= 2 ? osr.receiver_antenna_m[1] : 0.0)
                      << " tide=" << osr.solid_earth_tide_m
                      << " tide_geom=" << osr.tide_geometry_m
                      << " pbias_ref_tow=" << osr.phase_bias_reference_time.tow
                      << " eff_pbias_ref_tow=" << effective_phase_bias_reference_time.tow
                      << " orb_los=" << osr.orbit_projection_m
                      << " clk_corr=" << osr.clock_correction_m
                      << " PRC0=" << osr.PRC[0]
                      << " CPC0=" << osr.CPC[0]
                      << " CPC1=" << (osr.num_frequencies >= 2 ? osr.CPC[1] : 0.0)
                      << "\n";
        }
        corrections.push_back(osr);
    }

    return corrections;
}

}  // namespace libgnss
