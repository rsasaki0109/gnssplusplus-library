#pragma once

#include "../core/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <tuple>

namespace libgnss::causal_ambiguity_arc {

struct Config {
    int minimum_samples = 5;
    int maximum_effective_samples = 50;
    double maximum_gap_s = 1.0;
};

struct Result {
    bool valid = false;
    bool ready = false;
    bool reset = false;
    int samples = 0;
    std::uint64_t reference_generation = 0;
    double smoothed_value = NAN;
    double smoothed_value_variance = INFINITY;
};

/// A causal, reference-generation-aware bank for ambiguity-domain observables.
///
/// Values are never held as fixed integers.  The bank only smooths the
/// current/past floating observable used to validate a fresh integer search.
class Bank {
public:
    explicit Bank(Config config = {}) : config_(config) {}

    Result update(const SatelliteId& satellite,
                  const SatelliteId& reference,
                  int frequency_pair,
                  double time_s,
                  double value,
                  bool slip) {
        Result result;
        if (!std::isfinite(time_s) || !std::isfinite(value) ||
            satellite == reference) {
            return result;
        }
        const std::uint64_t generation =
            observeReference(
                satellite, frequency_pair, reference);
        const Key key{
            satellite, reference, frequency_pair, generation};
        auto& state = states_[key];
        const double gap_s =
            state.initialized ? time_s - state.last_time_s : NAN;
        const bool discontinuity =
            slip || !state.initialized || !std::isfinite(gap_s) ||
            gap_s <= 0.0 || gap_s > config_.maximum_gap_s;
        if (discontinuity) {
            state = State{};
            state.initialized = true;
            state.samples = 1;
            state.mean = value;
            state.observation_variance = 0.0;
            state.rounded_integer =
                static_cast<long long>(std::llround(value));
            state.stable_integer_samples = 1;
            state.last_time_s = time_s;
            result.reset = true;
        } else {
            state.samples++;
            const int effective_samples = std::clamp(
                state.samples, 1,
                std::max(1, config_.maximum_effective_samples));
            const double alpha =
                1.0 / static_cast<double>(effective_samples);
            const double delta = value - state.mean;
            state.mean += alpha * delta;
            state.observation_variance =
                (1.0 - alpha) *
                (state.observation_variance + alpha * delta * delta);
            const auto rounded =
                static_cast<long long>(std::llround(state.mean));
            if (rounded == state.rounded_integer) {
                state.stable_integer_samples++;
            } else {
                state.rounded_integer = rounded;
                state.stable_integer_samples = 1;
            }
            state.last_time_s = time_s;
        }
        result.valid = true;
        result.samples = state.samples;
        result.reference_generation = generation;
        result.smoothed_value = state.mean;
        if (state.samples >= 2) {
            const int effective_samples = std::clamp(
                state.samples, 1,
                std::max(1, config_.maximum_effective_samples));
            result.smoothed_value_variance =
                state.observation_variance /
                static_cast<double>(effective_samples);
        }
        result.ready =
            state.samples >= std::max(1, config_.minimum_samples) &&
            state.stable_integer_samples >=
                std::max(1, config_.minimum_samples);
        return result;
    }

    /// Update a reference-invariant single-difference signal arc.  A current
    /// DD validation can be formed by subtracting two ready SD results; no DD
    /// integer or past reference relation is retained.
    Result updateSignal(const SatelliteId& satellite,
                        int frequency_pair,
                        double time_s,
                        double value,
                        bool slip) {
        Result result;
        if (!std::isfinite(time_s) || !std::isfinite(value)) {
            return result;
        }
        const TrackKey key{satellite, frequency_pair};
        auto& state = signal_states_[key];
        if (state.initialized &&
            std::abs(time_s - state.last_time_s) <= 1e-9) {
            result.valid = true;
            result.samples = state.samples;
            result.smoothed_value = state.mean;
            if (state.samples >= 2) {
                const int effective_samples = std::clamp(
                    state.samples, 1,
                    std::max(1, config_.maximum_effective_samples));
                result.smoothed_value_variance =
                    state.observation_variance /
                    static_cast<double>(effective_samples);
            }
            result.ready =
                state.samples >= std::max(1, config_.minimum_samples) &&
                state.stable_integer_samples >=
                    std::max(1, config_.minimum_samples);
            return result;
        }
        const double gap_s =
            state.initialized ? time_s - state.last_time_s : NAN;
        const bool discontinuity =
            slip || !state.initialized || !std::isfinite(gap_s) ||
            gap_s <= 0.0 || gap_s > config_.maximum_gap_s;
        if (discontinuity) {
            state = State{};
            state.initialized = true;
            state.samples = 1;
            state.mean = value;
            state.observation_variance = 0.0;
            state.rounded_integer =
                static_cast<long long>(std::llround(value));
            state.stable_integer_samples = 1;
            state.last_time_s = time_s;
            result.reset = true;
        } else {
            state.samples++;
            const int effective_samples = std::clamp(
                state.samples, 1,
                std::max(1, config_.maximum_effective_samples));
            const double alpha =
                1.0 / static_cast<double>(effective_samples);
            const double delta = value - state.mean;
            state.mean += alpha * delta;
            state.observation_variance =
                (1.0 - alpha) *
                (state.observation_variance + alpha * delta * delta);
            const auto rounded =
                static_cast<long long>(std::llround(state.mean));
            if (rounded == state.rounded_integer) {
                state.stable_integer_samples++;
            } else {
                state.rounded_integer = rounded;
                state.stable_integer_samples = 1;
            }
            state.last_time_s = time_s;
        }
        result.valid = true;
        result.samples = state.samples;
        result.smoothed_value = state.mean;
        if (state.samples >= 2) {
            const int effective_samples = std::clamp(
                state.samples, 1,
                std::max(1, config_.maximum_effective_samples));
            result.smoothed_value_variance =
                state.observation_variance /
                static_cast<double>(effective_samples);
        }
        result.ready =
            state.samples >= std::max(1, config_.minimum_samples) &&
            state.stable_integer_samples >=
                std::max(1, config_.minimum_samples);
        return result;
    }

    void reset() {
        states_.clear();
        signal_states_.clear();
        references_.clear();
        generations_.clear();
    }

    std::size_t size() const {
        return states_.size() + signal_states_.size();
    }

private:
    struct Key {
        SatelliteId satellite;
        SatelliteId reference;
        int frequency_pair = 0;
        std::uint64_t reference_generation = 0;

        bool operator<(const Key& other) const {
            return std::tie(
                       satellite, reference, frequency_pair,
                       reference_generation) <
                   std::tie(
                       other.satellite, other.reference,
                       other.frequency_pair,
                       other.reference_generation);
        }
    };

    struct State {
        bool initialized = false;
        int samples = 0;
        int stable_integer_samples = 0;
        long long rounded_integer = 0;
        double mean = 0.0;
        double observation_variance = 0.0;
        double last_time_s = NAN;
    };

    struct TrackKey {
        SatelliteId satellite;
        int frequency_pair = 0;

        bool operator<(const TrackKey& other) const {
            return std::tie(satellite, frequency_pair) <
                   std::tie(
                       other.satellite, other.frequency_pair);
        }
    };

    std::uint64_t observeReference(
        const SatelliteId& satellite,
        int frequency_pair,
        const SatelliteId& reference) {
        const TrackKey track{satellite, frequency_pair};
        auto current = references_.find(track);
        if (current == references_.end() ||
            current->second != reference) {
            references_[track] = reference;
            return ++generations_[track];
        }
        return generations_[track];
    }

    Config config_;
    std::map<Key, State> states_;
    std::map<TrackKey, State> signal_states_;
    std::map<TrackKey, SatelliteId> references_;
    std::map<TrackKey, std::uint64_t> generations_;
};

}  // namespace libgnss::causal_ambiguity_arc
