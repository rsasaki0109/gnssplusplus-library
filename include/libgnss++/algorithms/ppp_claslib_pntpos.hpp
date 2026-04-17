#pragma once

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>

#include <array>
#include <string>
#include <vector>

namespace libgnss::ppp_claslib_pntpos {

struct PntposOptions {
    enum class Mode {
        Single,
        Precise,
        PppRtk,
    };

    Mode mode = Mode::PppRtk;
    double elmin_rad = 0.2617993877991494;
    std::array<double, 7> err = {100.0, 0.010, 0.005, 0.0, 1.0, 0.0, 0.0};
    double rejethres_m = 20.0;
    int rejeminsat = 7;
    double maxgdop = 30.0;
    bool raim_fde = true;
    bool enable_gps = true;
    bool enable_galileo = true;
    bool enable_qzss = true;
    bool enable_glonass = false;
    bool enable_beidou = false;
    bool snr_mask_rover = true;
    std::array<std::array<double, 9>, 3> snr_mask = {{
        {{10.0, 10.0, 10.0, 10.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
        {{10.0, 10.0, 10.0, 10.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
        {{10.0, 10.0, 10.0, 10.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
    }};
    bool debug_dump = false;
};

struct SatelliteStatus {
    SatelliteId satellite;
    bool valid = false;
    double azimuth_rad = 0.0;
    double elevation_rad = 0.0;
    double pseudorange_residual_m = 0.0;
};

struct PntposResult {
    bool valid = false;
    int stat = 0;  // CLASLIB sol_t.stat value (SOLQ_SINGLE=5 for valid SPP).
    int ns = 0;
    std::array<double, 6> rr = {};
    std::array<double, 6> dtr = {};
    std::array<float, 6> qr = {};
    double gdop = 0.0;
    double pdop = 0.0;
    double hdop = 0.0;
    double vdop = 0.0;
    int iterations = 0;
    double residual_rms_m = 0.0;
    std::string message;
    PositionSolution solution;
    std::vector<SatelliteStatus> satellites;
};

PntposResult pntpos(const ObservationData& obs,
                    const NavigationData& nav,
                    const Vector3d& initial_rr,
                    const PntposOptions& options = PntposOptions{});

bool solvePntposSeed(const ObservationData& obs,
                     const NavigationData& nav,
                     const Vector3d& initial_rr,
                     PositionSolution& solution,
                     PntposResult* result = nullptr,
                     const PntposOptions& options = PntposOptions{});

}  // namespace libgnss::ppp_claslib_pntpos
