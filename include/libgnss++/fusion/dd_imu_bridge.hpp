#pragma once

// Historical Stage 2 design sketch: parallel DD-RTK + IMU state.
//
// This header intentionally declares NO functions and has NO corresponding
// .cpp. The active design is docs/tight_coupling.md: RTKProcessor remains the
// owner of baseline/ambiguity/AR state while a thin TightCouplingProcessor
// owns IMU propagation and feeds an opt-in time update into RTKProcessor.
//
// These declarations are retained for source compatibility and historical
// context only. New code must not duplicate RTKProcessor's ambiguity states
// into TightlyCoupledState; doing so would fork the existing lock, hold,
// slip, validation, and ambiguity-resolution lifecycle.

#include <vector>

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_types.hpp>

namespace libgnss {
namespace dd_imu_bridge {

/**
 * @brief Legacy sketch of an ambiguity duplicated into a parallel ESKF.
 *
 * Deprecated as an implementation direction; RTKProcessor exclusively owns
 * live ambiguities in the RTK-hosted design.
 */
struct AmbiguityErrorState {
    int satellite_prn = 0;
    int frequency_index = -1;
    int generation = 0;      ///< bumped on cycle slip / forced reset, forces a fresh scalar slot
    double float_value_cycles = 0.0;
    double variance_cycles2 = 0.0;
    bool held = false;       ///< fixed-and-held: folded into the DD measurement as a constant
};

/**
 * @brief Legacy parallel-filter state retained for source compatibility.
 *
 * The active design does not instantiate this type.
 */
struct TightlyCoupledState {
    FusionState eskf;
    std::vector<AmbiguityErrorState> ambiguities;
};

}  // namespace dd_imu_bridge
}  // namespace libgnss
