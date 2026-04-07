#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

#include <map>
#include <vector>

namespace libgnss::ppp_clas_sd {

/// Clock-free single-difference filter state.
/// State vector: pos(3) + SD ambiguities (variable length).
/// No clock, no trop — full SD with PRC/CPC (which include trop + iono)
/// cancels both receiver clock and atmospheric terms.
struct SdFilterState {
    VectorXd x;
    MatrixXd P;
    /// Map from {system, freq_index, satellite} → state index for SD ambiguity
    std::map<SatelliteId, int> amb_indices;  // L1 ambiguities
    std::map<SatelliteId, int> amb2_indices; // L2 ambiguities
    int nx = 3;  // starts with pos(3), grows with ambiguities
    bool initialized = false;
    int epoch_count = 0;
    double position_sigma_m = 100.0;  // current position uncertainty
    /// Reference satellites per system (highest elevation)
    std::map<GNSSSystem, SatelliteId> reference_sats;
};

struct SdEpochResult {
    bool valid = false;
    Vector3d position = Vector3d::Zero();
    int num_satellites = 0;
    int num_observations = 0;
    double code_rms = 0.0;
    double phase_rms = 0.0;
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

/// Single-epoch SD least-squares + LAMBDA ambiguity resolution.
/// Solves position and DD ambiguities from SD code+phase in one epoch,
/// then fixes ambiguities using LAMBDA. No KF state required.
SdEpochResult solveSingleEpochSdAr(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const Vector3d& seed_position,
    double ar_ratio_threshold,
    bool debug_enabled);

/// Run one epoch of the clock-free SD filter.
/// Uses raw PRC/CPC (with trop + iono) so that full SD cancels clock.
SdEpochResult runClockFreeSdEpoch(
    SdFilterState& state,
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const Vector3d& seed_position,
    double dt,
    bool debug_enabled);

}  // namespace libgnss::ppp_clas_sd
