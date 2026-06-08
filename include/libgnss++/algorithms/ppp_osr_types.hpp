#pragma once

#include <libgnss++/core/types.hpp>

#include <array>
#include <map>
#include <string>
#include <vector>

namespace libgnss {

constexpr int OSR_MAX_FREQ = 3;

struct OSRCorrection {
    SatelliteId satellite;

    double trop_correction_m = 0.0;
    double relativity_correction_m = 0.0;
    double receiver_antenna_m[OSR_MAX_FREQ] = {};
    double iono_l1_m = 0.0;
    double code_bias_m[OSR_MAX_FREQ] = {};
    double phase_bias_m[OSR_MAX_FREQ] = {};
    double windup_cycles = 0.0;
    double windup_m[OSR_MAX_FREQ] = {};

    double PRC[OSR_MAX_FREQ] = {};
    double CPC[OSR_MAX_FREQ] = {};
    double phase_compensation_m[OSR_MAX_FREQ] = {};
    double orbit_projection_m = 0.0;
    double clock_correction_m = 0.0;

    Vector3d satellite_position = Vector3d::Zero();
    Vector3d satellite_velocity = Vector3d::Zero();
    double satellite_clock_bias_s = 0.0;
    double elevation = 0.0;
    double azimuth = 0.0;

    SignalType signals[OSR_MAX_FREQ] = {};
    double wavelengths[OSR_MAX_FREQ] = {};
    double frequencies[OSR_MAX_FREQ] = {};
    int num_frequencies = 0;

    bool valid = false;
    bool has_iono = false;
    bool has_code_bias = false;
    bool has_phase_bias = false;
    GNSSTime atmos_reference_time;
    GNSSTime phase_bias_reference_time;
    GNSSTime clock_reference_time;
};

struct CLASPhaseBiasRepairInfo {
    GNSSTime reference_time;
    std::array<double, OSR_MAX_FREQ> last_continuity_m{{0.0, 0.0, 0.0}};
    std::array<double, OSR_MAX_FREQ> offset_cycles{{0.0, 0.0, 0.0}};
    std::array<double, OSR_MAX_FREQ> pending_state_shift_cycles{{0.0, 0.0, 0.0}};
    std::array<bool, OSR_MAX_FREQ> has_last{{false, false, false}};
};

struct CLASDispersionCompensationInfo {
    GNSSTime reference_time;
    std::array<double, 2> base_phase_m{{0.0, 0.0}};
    std::array<bool, 2> has_base{{false, false}};
    std::array<bool, 2> slip{{false, false}};
};

struct CLASSisContinuityInfo {
    GNSSTime current_time;
    GNSSTime previous_time;
    double current_sis_m = 0.0;
    double previous_sis_m = 0.0;
    double last_delta_m = 0.0;
    bool has_current = false;
    bool has_previous = false;
    bool has_last_delta = false;
};

struct CLASEpochContext {
    Vector3d receiver_position = Vector3d::Zero();
    double receiver_clock_m = 0.0;
    double trop_zenith_m = 0.0;
    std::map<std::string, std::string> epoch_atmos_tokens;
    std::vector<OSRCorrection> osr_corrections;
};

}  // namespace libgnss
