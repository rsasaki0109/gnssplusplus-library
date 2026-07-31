#pragma once

namespace libgnss::rtk_tdcp_diagnostics {

struct Config {
    double max_gap_s = 2.0;
};

enum class Status {
    VALID,
    INVALID_INPUT,
    INVALID_GAP,
    LOSS_OF_LOCK,
};

struct Result {
    Status status = Status::INVALID_INPUT;
    double residual_m = 0.0;
};

/**
 * Compare a single-difference carrier-phase change with trapezoid-integrated
 * single-difference Doppler range rate. Carrier phase and range rate are in
 * meters and meters/second respectively. This function is diagnostic only.
 */
Result evaluate(double previous_phase_m,
                double current_phase_m,
                double previous_range_rate_mps,
                double current_range_rate_mps,
                double dt_s,
                bool loss_of_lock,
                const Config& config = {});

}  // namespace libgnss::rtk_tdcp_diagnostics
