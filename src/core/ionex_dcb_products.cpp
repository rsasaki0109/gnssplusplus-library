#include <libgnss++/core/navigation.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "navigation_internal.hpp"

namespace libgnss {

using namespace navigation_internal;

bool IONEXProducts::loadIONEXFile(const std::string& filename) {
    clear();

    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_header = true;
    bool loaded_any = false;
    bool current_is_rms = false;
    IONEXMap current_map;
    std::string line;
    while (std::getline(input, line)) {
        if (in_header) {
            if (line.find("IONEX VERSION / TYPE") != std::string::npos) {
                const std::string version_text = trimCopy(line.substr(0, std::min<size_t>(20U, line.size())));
                if (!version_text.empty()) {
                    std::istringstream version_stream(version_text);
                    version_stream >> version;
                }
                const auto type_fields = trimCopy(line.substr(20, std::min<size_t>(40U, line.size() > 20 ? line.size() - 20U : 0U)));
                if (!type_fields.empty()) {
                    std::istringstream type_stream(type_fields);
                    std::string unused_type;
                    type_stream >> unused_type >> system;
                }
            } else if (line.find("INTERVAL") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    interval_s = static_cast<int>(std::llround(values.front()));
                }
            } else if (line.find("MAP DIMENSION") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    map_dimension = static_cast<int>(std::llround(values.front()));
                }
            } else if (line.find("BASE RADIUS") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    base_radius_km = values.front();
                }
            } else if (line.find("ELEVATION CUTOFF") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    elevation_cutoff_deg = values.front();
                }
            } else if (line.find("MAPPING FUNCTION") != std::string::npos) {
                mapping_function = trimCopy(line.substr(0, std::min<size_t>(20U, line.size())));
            } else if (line.find("EXPONENT") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    exponent = static_cast<int>(std::llround(values.front()));
                }
            } else if (line.find("LAT1 / LAT2 / DLAT") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 30U, values) && values.size() >= 3U) {
                    latitude_grid = {values[0], values[1], values[2]};
                }
            } else if (line.find("LON1 / LON2 / DLON") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 30U, values) && values.size() >= 3U) {
                    longitude_grid = {values[0], values[1], values[2]};
                }
            } else if (line.find("HGT1 / HGT2 / DHGT") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 30U, values) && values.size() >= 3U) {
                    height_grid = {values[0], values[1], values[2]};
                }
            } else if (line.find("PRN / BIAS / RMS") != std::string::npos) {
                ++auxiliary_dcb_entries;
            } else if (line.find("END OF HEADER") != std::string::npos) {
                in_header = false;
            }
            continue;
        }

        if (line.find("START OF TEC MAP") != std::string::npos) {
            current_map = IONEXMap{};
            current_is_rms = false;
            continue;
        }
        if (line.find("START OF RMS MAP") != std::string::npos) {
            current_map = IONEXMap{};
            current_is_rms = true;
            continue;
        }
        if (line.find("END OF TEC MAP") != std::string::npos ||
            line.find("END OF RMS MAP") != std::string::npos) {
            if (!current_map.rows.empty()) {
                auto& target = current_is_rms ? rms_maps : tec_maps;
                target.push_back(current_map);
                loaded_any = true;
            }
            current_map = IONEXMap{};
            current_is_rms = false;
            continue;
        }
        if (line.find("EPOCH OF CURRENT MAP") != std::string::npos) {
            parseIonexEpochLine(line, current_map.time);
            continue;
        }
        if (line.find("LAT/LON1/LON2/DLON/H") == std::string::npos) {
            continue;
        }

        std::vector<double> row_header;
        if (!parseFixedFieldDoubles(line, 40U, row_header) || row_header.size() < 5U) {
            continue;
        }

        IONEXLatitudeRow row;
        row.latitude_deg = row_header[0];
        row.longitude_start_deg = row_header[1];
        row.longitude_end_deg = row_header[2];
        row.longitude_step_deg = row_header[3];
        row.height_km = row_header[4];

        if (std::abs(row.longitude_step_deg) < 1e-12) {
            continue;
        }

        const int longitude_count = static_cast<int>(
            std::llround((row.longitude_end_deg - row.longitude_start_deg) / row.longitude_step_deg)) + 1;
        if (longitude_count <= 0) {
            continue;
        }

        while (static_cast<int>(row.values_tecu.size()) < longitude_count && std::getline(input, line)) {
            if (line.find("END OF TEC MAP") != std::string::npos ||
                line.find("END OF RMS MAP") != std::string::npos ||
                line.find("LAT/LON1/LON2/DLON/H") != std::string::npos) {
                break;
            }
            // IONEX data values are 5-char wide (I5 in the spec) and
            // packed up to 16 values per 80-character line. The
            // previous 60-char window dropped the last 4 values on
            // every line, leaving every TEC row 16-values short and
            // every map empty — silently rejecting CODE / IGS final
            // IONEX. Use the full line.
            std::istringstream value_stream(line);
            double raw_value = 0.0;
            while (value_stream >> raw_value) {
                if (std::abs(raw_value - 9999.0) < 1e-6) {
                    row.values_tecu.push_back(std::numeric_limits<double>::quiet_NaN());
                } else {
                    row.values_tecu.push_back(raw_value * std::pow(10.0, exponent));
                }
                if (static_cast<int>(row.values_tecu.size()) >= longitude_count) {
                    break;
                }
            }
        }
        if (static_cast<int>(row.values_tecu.size()) == longitude_count) {
            current_map.rows.push_back(std::move(row));
        }
    }

    auto sort_by_time = [](std::vector<IONEXMap>& maps) {
        std::sort(
            maps.begin(),
            maps.end(),
            [](const IONEXMap& lhs, const IONEXMap& rhs) {
                return lhs.time < rhs.time;
            });
    };
    sort_by_time(tec_maps);
    sort_by_time(rms_maps);
    return loaded_any;
}

bool IONEXProducts::interpolateTecu(const GNSSTime& time,
                                    double latitude_deg,
                                    double longitude_deg,
                                    double& tecu,
                                    double* rms_tecu) const {
    const auto interpolate_map_series =
        [&](const std::vector<IONEXMap>& maps, double& output_value) -> bool {
            if (maps.empty()) {
                return false;
            }

            const IONEXMap* before = findEntryAtOrBefore(maps, time);
            const IONEXMap* after = findEntryAtOrAfter(maps, time);
            if (before == nullptr && after == nullptr) {
                return false;
            }
            if (before != nullptr && after != nullptr && before != after) {
                double left = 0.0;
                double right = 0.0;
                if (!interpolateIonexMap(*before, latitude_deg, longitude_deg, left) ||
                    !interpolateIonexMap(*after, latitude_deg, longitude_deg, right)) {
                    return false;
                }
                const double dt_total = after->time - before->time;
                if (std::abs(dt_total) < 1e-9) {
                    output_value = left;
                    return true;
                }
                const double alpha = (time - before->time) / dt_total;
                output_value = left + alpha * (right - left);
                return true;
            }

            const IONEXMap* sample = before != nullptr ? before : after;
            return sample != nullptr && interpolateIonexMap(*sample, latitude_deg, longitude_deg, output_value);
        };

    if (!interpolate_map_series(tec_maps, tecu)) {
        return false;
    }
    if (rms_tecu != nullptr) {
        double rms_value = 0.0;
        if (interpolate_map_series(rms_maps, rms_value)) {
            *rms_tecu = rms_value;
        } else {
            *rms_tecu = 0.0;
        }
    }
    return true;
}

bool IONEXProducts::hasData(const GNSSTime& time) const {
    return findEntryAtOrBefore(tec_maps, time) != nullptr ||
           findEntryAtOrAfter(tec_maps, time) != nullptr;
}

void IONEXProducts::clear() {
    version.clear();
    system.clear();
    interval_s = 0;
    map_dimension = 0;
    exponent = 0;
    base_radius_km = 0.0;
    elevation_cutoff_deg = 0.0;
    mapping_function.clear();
    latitude_grid.clear();
    longitude_grid.clear();
    height_grid.clear();
    auxiliary_dcb_entries = 0;
    tec_maps.clear();
    rms_maps.clear();
}

bool DCBProducts::loadFile(const std::string& filename) {
    clear();

    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_bias_solution = false;
    bool loaded_any = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty()) {
            continue;
        }
        if (trimmed.rfind("+BIAS/SOLUTION", 0) == 0) {
            in_bias_solution = true;
            continue;
        }
        if (trimmed.rfind("-BIAS/SOLUTION", 0) == 0) {
            in_bias_solution = false;
            continue;
        }

        if (in_bias_solution) {
            if (trimmed[0] == '*') {
                continue;
            }
            std::vector<std::string> fields;
            std::istringstream stream(trimmed);
            std::string token;
            while (stream >> token) {
                fields.push_back(token);
            }
            if (fields.size() < 9U) {
                continue;
            }

            // Bias-SINEX rows are `BIAS SVN PRN [STATION] OBS1 OBS2
            // START END UNIT VALUE STDEV`. Some emitters omit the
            // optional STATION column entirely; some include it as
            // a 9-char field that whitespace-tokenization collapses
            // away. After collapsing, the PRN column may therefore
            // land at fields[1] (no SVN, e.g. CAS) or fields[2]
            // (BSX with SVN, e.g. GFZ/GBM). Pick whichever parses
            // as a valid PRN (3-char `<sys><digit><digit>`); SVNs
            // are 4 chars (`<sys><digit><digit><digit>`) so
            // length disambiguates without needing brittle field-
            // count heuristics.
            SatelliteId satellite;
            std::size_t obs_offset = 0;
            if (fields[1].size() == 3U &&
                parseSatelliteToken(fields[1], satellite)) {
                obs_offset = 2;
            } else if (fields[2].size() == 3U &&
                       parseSatelliteToken(fields[2], satellite)) {
                obs_offset = 3;
            } else {
                continue;
            }
            // Required columns (relative to obs_offset): OBS1,
            // OBS2, START, END, UNIT, VALUE, STDEV — 7 more.
            if (fields.size() < obs_offset + 7U) {
                continue;
            }

            try {
                DCBEntry entry;
                entry.bias_type = fields[0];
                entry.satellite = satellite;
                entry.observation_1 = fields[obs_offset];
                entry.observation_2 = fields[obs_offset + 1];
                entry.unit = fields[obs_offset + 4];
                entry.bias = std::stod(fields[obs_offset + 5]);
                entry.sigma = std::stod(fields[obs_offset + 6]);
                entry.valid = std::isfinite(entry.bias);
                entries.push_back(entry);
                loaded_any = true;
            } catch (const std::exception&) {
            }
            continue;
        }

        if (line.find("PRN / BIAS / RMS") != std::string::npos) {
            std::vector<double> values;
            std::vector<std::string> fields;
            std::istringstream stream(line.substr(0, std::min<size_t>(40U, line.size())));
            std::string token;
            while (stream >> token) {
                fields.push_back(token);
            }
            if (fields.size() < 3U) {
                continue;
            }
            SatelliteId satellite;
            if (!parseSatelliteToken(fields[0], satellite)) {
                continue;
            }
            try {
                DCBEntry entry;
                entry.bias_type = "DCB";
                entry.satellite = satellite;
                entry.unit = "ns";
                entry.bias = std::stod(fields[1]);
                entry.sigma = std::stod(fields[2]);
                entry.valid = std::isfinite(entry.bias);
                entries.push_back(entry);
                loaded_any = true;
            } catch (const std::exception&) {
            }
        }
    }

    return loaded_any;
}

bool DCBProducts::getBias(const SatelliteId& sat,
                          const std::string& bias_type,
                          const std::string& observation_1,
                          const std::string& observation_2,
                          double& bias,
                          double* sigma) const {
    for (const auto& entry : entries) {
        if (!entry.valid || !(entry.satellite == sat) || entry.bias_type != bias_type) {
            continue;
        }
        if (!observation_1.empty() && entry.observation_1 != observation_1) {
            continue;
        }
        if (!observation_2.empty() && entry.observation_2 != observation_2) {
            continue;
        }
        bias = entry.bias;
        if (sigma != nullptr) {
            *sigma = entry.sigma;
        }
        return true;
    }
    return false;
}

void DCBProducts::clear() {
    entries.clear();
}
} // namespace libgnss
