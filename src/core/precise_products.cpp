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

namespace {

// RTKLIB-style Neville's algorithm for polynomial interpolation. Returns the
// interpolated value at t=0 given samples at offsets x[i] with values y[i].
// y[] is modified in place. Numerically stable for closely-spaced samples.
double nevillePolynomial(const std::vector<double>& x,
                         std::vector<double>& y,
                         int n) {
    for (int j = 1; j < n; ++j) {
        for (int i = 0; i < n - j; ++i) {
            const double denom = x[i + j] - x[i];
            if (std::abs(denom) < 1e-12) {
                return y[i];
            }
            y[i] = (x[i + j] * y[i] - x[i] * y[i + 1]) / denom;
        }
    }
    return y[0];
}

constexpr int kLagrangeOrder = 10;  ///< number of SP3 samples (degree 9)

}  // namespace

bool PreciseProducts::loadSP3File(const std::string& filename) {
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    GNSSTime current_time;
    bool have_epoch = false;
    bool loaded_any = false;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        if (line[0] == '*') {
            have_epoch = parseSp3EpochLine(line, current_time);
            continue;
        }
        if (!have_epoch || line[0] != 'P') {
            continue;
        }

        std::istringstream stream(line.substr(1));
        std::string sat_token;
        double x_km = 0.0;
        double y_km = 0.0;
        double z_km = 0.0;
        double clock_microseconds = 0.0;
        if (!(stream >> sat_token >> x_km >> y_km >> z_km >> clock_microseconds)) {
            continue;
        }

        SatelliteId satellite;
        if (!parseSatelliteToken(sat_token, satellite)) {
            continue;
        }

        PreciseOrbitClock sample;
        sample.satellite = satellite;
        sample.time = current_time;
        sample.position = Vector3d(x_km * 1000.0, y_km * 1000.0, z_km * 1000.0);
        sample.position_valid = std::isfinite(sample.position.x()) &&
            std::isfinite(sample.position.y()) &&
            std::isfinite(sample.position.z());
        if (std::isfinite(clock_microseconds) && std::abs(clock_microseconds) < 999999.0) {
            sample.clock_bias = clock_microseconds * 1e-6;
            sample.clock_valid = true;
        }
        addOrbitClock(sample);
        loaded_any = true;
    }
    return loaded_any;
}

bool PreciseProducts::loadClockFile(const std::string& filename) {
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_header = true;
    bool loaded_any = false;
    std::string line;
    while (std::getline(input, line)) {
        if (in_header) {
            if (line.find("END OF HEADER") != std::string::npos) {
                in_header = false;
            }
            continue;
        }
        if (line.size() < 2 || line[0] != 'A' || line[1] != 'S') {
            continue;
        }

        std::istringstream stream(line);
        std::string record_type;
        std::string sat_token;
        if (!(stream >> record_type >> sat_token)) {
            continue;
        }

        SatelliteId satellite;
        if (!parseSatelliteToken(sat_token, satellite)) {
            continue;
        }

        GNSSTime current_time;
        if (!parseClockEpochTokens(stream, current_time)) {
            continue;
        }

        int value_count = 0;
        double clock_bias = 0.0;
        double clock_sigma = 0.0;
        if (!(stream >> value_count >> clock_bias)) {
            continue;
        }
        if (value_count > 1) {
            stream >> clock_sigma;
        }

        PreciseOrbitClock sample;
        sample.satellite = satellite;
        sample.time = current_time;
        sample.clock_bias = clock_bias;
        sample.clock_sigma = clock_sigma;
        sample.clock_valid = std::isfinite(clock_bias);
        addOrbitClock(sample);
        loaded_any = true;
    }
    return loaded_any;
}

void PreciseProducts::addOrbitClock(const PreciseOrbitClock& data) {
    auto& entries = orbit_clock_data[data.satellite];
    auto it = std::lower_bound(
        entries.begin(),
        entries.end(),
        data.time,
        [](const PreciseOrbitClock& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });

    if (it != entries.end() && it->time == data.time) {
        if (data.position_valid) {
            it->position = data.position;
            it->velocity = data.velocity;
            it->position_sigma = data.position_sigma;
            it->position_valid = true;
        }
        if (data.clock_valid) {
            it->clock_bias = data.clock_bias;
            it->clock_drift = data.clock_drift;
            it->clock_sigma = data.clock_sigma;
            it->clock_valid = true;
        }
        return;
    }

    entries.insert(it, data);
}

bool PreciseProducts::interpolateOrbitClock(const SatelliteId& sat,
                                            const GNSSTime& time,
                                            Vector3d& position,
                                            Vector3d& velocity,
                                            double& clock_bias,
                                            double& clock_drift) const {
    const auto sat_it = orbit_clock_data.find(sat);
    if (sat_it == orbit_clock_data.end() || sat_it->second.empty()) {
        return false;
    }

    const auto& entries = sat_it->second;

    // Build position- and clock-sample index vectors so polynomial
    // interpolation can pick a contiguous window of N samples around the
    // query time. SP3 (orbit) and CLK rows are stored in the same sorted
    // list; position_valid and clock_valid flag which records contribute
    // to each grid.
    std::vector<int> pos_indices;
    std::vector<int> clk_indices;
    int pos_at_or_before = -1;  // last position-valid index with time <= query
    int clk_at_or_before = -1;
    pos_indices.reserve(entries.size());
    clk_indices.reserve(entries.size());
    for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
        if (entries[i].position_valid) {
            pos_indices.push_back(i);
            if (entries[i].time <= time) {
                pos_at_or_before = static_cast<int>(pos_indices.size()) - 1;
            }
        }
        if (entries[i].clock_valid) {
            clk_indices.push_back(i);
            if (entries[i].time <= time) {
                clk_at_or_before = static_cast<int>(clk_indices.size()) - 1;
            }
        }
    }

    if (pos_indices.empty()) {
        return false;
    }

    // Pick an N-sample window centred on the query, clamped to the array
    // ends so the same code path serves boundary queries with extrapolation
    // (RTKLIB pephpos does the same).
    auto interpolateLagrange = [&entries, &time](
                                    const std::vector<int>& indices,
                                    int at_or_before,
                                    auto&& accessor,
                                    auto& out_value,
                                    auto& out_rate) -> bool {
        const int n_avail = static_cast<int>(indices.size());
        if (n_avail == 0) {
            return false;
        }
        const int n_use = std::min(kLagrangeOrder, n_avail);
        int start = at_or_before - (n_use - 1) / 2;
        if (start < 0) start = 0;
        if (start + n_use > n_avail) start = n_avail - n_use;
        if (start < 0) start = 0;

        // Reference the times relative to the first picked sample to keep
        // the polynomial coefficients well-scaled (15-min spacing × 9 ≈ 8100 s
        // dynamic range; relative differences fit cleanly in double).
        const GNSSTime& t_ref = entries[indices[start]].time;
        const double dt_query = time - t_ref;
        if (std::abs(dt_query) > kPreciseInterpolationGapSeconds * n_use) {
            return false;  // sanity guard: query far outside sample span
        }

        std::vector<double> xs;
        xs.reserve(n_use);
        for (int k = 0; k < n_use; ++k) {
            xs.push_back(entries[indices[start + k]].time - t_ref - dt_query);
        }

        // For derivative computation: evaluate the same polynomial at query+h
        // (i.e. shift the x-axis by -h so the new origin is at original x=h).
        // dp/dt = (P(h) - P(0))/h.
        const double h_deriv = 1.0;
        std::vector<double> xs_deriv = xs;
        for (auto& x : xs_deriv) x -= h_deriv;

        using value_t = decltype(accessor(entries[indices[0]]));
        if constexpr (std::is_same_v<value_t, Vector3d>) {
            for (int axis = 0; axis < 3; ++axis) {
                std::vector<double> ys_value;
                std::vector<double> ys_deriv;
                ys_value.reserve(n_use);
                ys_deriv.reserve(n_use);
                for (int k = 0; k < n_use; ++k) {
                    const double v = accessor(entries[indices[start + k]])(axis);
                    ys_value.push_back(v);
                    ys_deriv.push_back(v);
                }
                out_value(axis) = nevillePolynomial(xs, ys_value, n_use);
                const double v_shift = nevillePolynomial(xs_deriv, ys_deriv, n_use);
                out_rate(axis) = (v_shift - out_value(axis)) / h_deriv;
            }
        } else {
            std::vector<double> ys_value;
            std::vector<double> ys_deriv;
            ys_value.reserve(n_use);
            ys_deriv.reserve(n_use);
            for (int k = 0; k < n_use; ++k) {
                const double v = accessor(entries[indices[start + k]]);
                ys_value.push_back(v);
                ys_deriv.push_back(v);
            }
            out_value = nevillePolynomial(xs, ys_value, n_use);
            const double v_shift = nevillePolynomial(xs_deriv, ys_deriv, n_use);
            out_rate = (v_shift - out_value) / h_deriv;
        }
        return true;
    };

    if (!interpolateLagrange(
            pos_indices, pos_at_or_before,
            [](const PreciseOrbitClock& e) -> Vector3d { return e.position; },
            position, velocity)) {
        return false;
    }

    if (!interpolateLagrange(
            clk_indices, clk_at_or_before,
            [](const PreciseOrbitClock& e) -> double { return e.clock_bias; },
            clock_bias, clock_drift)) {
        clock_bias = 0.0;
        clock_drift = 0.0;
    }
    return true;
}

bool PreciseProducts::hasData(const SatelliteId& sat, const GNSSTime& time) const {
    Vector3d position;
    Vector3d velocity;
    double clock_bias = 0.0;
    double clock_drift = 0.0;
    return interpolateOrbitClock(sat, time, position, velocity, clock_bias, clock_drift);
}

void PreciseProducts::clear() {
    orbit_clock_data.clear();
}
} // namespace libgnss
