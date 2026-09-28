#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

#include <map>
#include <vector>

namespace libgnss::ppp_clas_sd {

struct SdEpochResult {
    bool valid = false;
    Vector3d position = Vector3d::Zero();
    int num_satellites = 0;
    int num_observations = 0;
    double code_rms = 0.0;
    double phase_rms = 0.0;
    /// LAMBDA ratio when multi-epoch SD AR fixes ambiguities (0 otherwise).
    double ar_ratio = 0.0;
    /// Position shift from the seed when SD AR applies a fixed solution.
    double position_shift_m = 0.0;
};

/// Accumulated DD ambiguity state for multi-epoch AR.
struct DdAmbAccumulator {
    /// Per-DD-ambiguity accumulation (keyed by satellite + freq)
    struct Entry {
        double sum = 0.0;
        double sum_sq = 0.0;
        int count = 0;
        double mean() const { return count > 0 ? sum / count : 0.0; }
        double variance() const {
            if (count < 2) return 1e6;
            const double m = mean();
            return sum_sq / count - m * m;
        }
    };
    std::map<SatelliteId, Entry> l1_ambs;
    int total_epochs = 0;
};

/// Multi-epoch SD AR: accumulates float DD ambiguities across epochs,
/// then fixes with LAMBDA when accumulated variance is small enough.
SdEpochResult solveMultiEpochSdAr(
    DdAmbAccumulator& accumulator,
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const Vector3d& seed_position,
    double ar_ratio_threshold,
    int min_accumulation_epochs,
    bool debug_enabled);

}  // namespace libgnss::ppp_clas_sd
