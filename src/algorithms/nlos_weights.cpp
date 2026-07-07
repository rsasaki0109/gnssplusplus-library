#include <libgnss++/algorithms/nlos_weights.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace libgnss {
namespace nlos_weights {

namespace {

std::string toLower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        out.push_back(trim(cell));
    }
    return out;
}

int findColumn(const std::vector<std::string>& header,
               const std::vector<std::string>& candidate_names) {
    for (const auto& name : candidate_names) {
        for (size_t i = 0; i < header.size(); ++i) {
            if (toLower(header[i]) == name) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

}  // namespace

NlosWeightTable loadNlosWeightsCsv(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("failed to open NLOS weights CSV: " + path);
    }

    std::string header_line;
    if (!std::getline(input, header_line)) {
        throw std::runtime_error("NLOS weights CSV is empty: " + path);
    }
    const auto header = splitCsvLine(header_line);

    const int tow_col = findColumn(header, {"tow"});
    const int sat_col = findColumn(header, {"sat", "prn", "satellite"});
    // "los_prob" (this module's native contract) takes precedence; fall
    // back to the boolean "is_los" contract emitted by
    // build_per_epoch_nlos_csv.py (0/1 -> 0.0/1.0).
    int prob_col = findColumn(header, {"los_prob"});
    bool prob_is_boolean_is_los = false;
    if (prob_col < 0) {
        prob_col = findColumn(header, {"is_los"});
        prob_is_boolean_is_los = prob_col >= 0;
    }

    if (tow_col < 0 || sat_col < 0 || prob_col < 0) {
        throw std::runtime_error(
            "NLOS weights CSV missing required columns (need tow + sat/prn + "
            "los_prob/is_los): " + path);
    }

    NlosWeightTable table;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        const auto cells = splitCsvLine(line);
        const size_t max_needed =
            static_cast<size_t>(std::max({tow_col, sat_col, prob_col})) + 1;
        if (cells.size() < max_needed) continue;

        double tow = 0.0;
        double prob_raw = 0.0;
        try {
            tow = std::stod(cells[static_cast<size_t>(tow_col)]);
            prob_raw = std::stod(cells[static_cast<size_t>(prob_col)]);
        } catch (const std::exception&) {
            continue;
        }
        const std::string sat_id = trim(cells[static_cast<size_t>(sat_col)]);
        if (sat_id.empty() || !std::isfinite(tow) || !std::isfinite(prob_raw)) continue;

        const double los_prob =
            prob_is_boolean_is_los ? (prob_raw != 0.0 ? 1.0 : 0.0)
                                    : std::clamp(prob_raw, 0.0, 1.0);

        table.by_tow[tow][sat_id] = los_prob;
    }

    return table;
}

double lookupLosProb(const NlosWeightTable& table,
                      double tow,
                      const std::string& sat_id,
                      double tow_tolerance_s) {
    if (table.empty() || sat_id.empty()) {
        return 1.0;
    }

    const double tolerance = std::isfinite(tow_tolerance_s) ? std::max(0.0, tow_tolerance_s) : 0.0;

    // Nearest-tow lookup (like Python's bisect_left in gnss_gpu.nlos_mask):
    // check the first key >= tow and the key just before it, pick whichever
    // is within tolerance and closer.
    auto it = table.by_tow.lower_bound(tow);
    const double* best_prob = nullptr;
    double best_delta = tolerance + 1.0;

    auto consider = [&](const std::map<double, std::map<std::string, double>>::const_iterator& candidate) {
        if (candidate == table.by_tow.end()) return;
        const double delta = std::abs(candidate->first - tow);
        if (delta > tolerance || delta >= best_delta) return;
        const auto sat_it = candidate->second.find(sat_id);
        if (sat_it == candidate->second.end()) return;
        best_prob = &sat_it->second;
        best_delta = delta;
    };

    consider(it);
    if (it != table.by_tow.begin()) {
        consider(std::prev(it));
    }

    return best_prob != nullptr ? *best_prob : 1.0;
}

double nlosVarianceInflationFactor(double los_prob,
                                    NlosWeightMode mode,
                                    double continuous_floor,
                                    double two_tier_los_threshold,
                                    double two_tier_sigma_inflation) {
    if (mode == NlosWeightMode::OFF || mode == NlosWeightMode::EXCLUDE) {
        // EXCLUDE drops flagged satellites from DD formation entirely
        // (nlosShouldExclude / RTKProcessor::buildSelectionSnapshot); any
        // satellite that survives that filter gets no sigma inflation on
        // top, so this mapping is a no-op for EXCLUDE, same as OFF.
        return 1.0;
    }
    if (!std::isfinite(los_prob)) {
        return 1.0;
    }
    const double prob = std::clamp(los_prob, 0.0, 1.0);

    if (mode == NlosWeightMode::TWO_TIER) {
        if (prob >= two_tier_los_threshold) {
            return 1.0;
        }
        const double inflation = std::isfinite(two_tier_sigma_inflation)
                                      ? std::max(1.0, two_tier_sigma_inflation)
                                      : 1.0;
        return inflation * inflation;
    }

    // CONTINUOUS
    const double floor = std::isfinite(continuous_floor)
                              ? std::clamp(continuous_floor, 1e-6, 1.0)
                              : 1e-3;
    return 1.0 / std::max(prob, floor);
}

bool nlosShouldExclude(double los_prob, NlosWeightMode mode, double exclude_threshold) {
    if (mode != NlosWeightMode::EXCLUDE) return false;
    if (!std::isfinite(los_prob)) return false;
    const double threshold = std::isfinite(exclude_threshold) ? exclude_threshold : 0.5;
    return los_prob < threshold;
}

bool nlosExclusionGuardAllows(int total_sats, int excluded_count, int min_sats) {
    if (excluded_count <= 0) return false;
    const int floor = std::max(0, min_sats);
    const int surviving = total_sats - excluded_count;
    return surviving >= floor;
}

bool nlosMinLosSatsGateAllows(int los_sat_count, int min_los_sats) {
    if (min_los_sats <= 0) return true;
    return los_sat_count >= min_los_sats;
}

}  // namespace nlos_weights
}  // namespace libgnss
