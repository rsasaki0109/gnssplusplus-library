#pragma once

#include <cstdint>
#include <vector>

#include "rtklib2/common/version.hpp"
#include "rtklib2/core/system_context.hpp"
#include "rtklib2/telemetry/logger.hpp"

namespace rtklib2::solvers::spp {

enum class SolutionQuality : std::uint8_t {
    None = 0,
    SPP,
    DGPS,
    Float,
    Fixed,
    PPPFloat,
    PPPFixed
};

struct SolutionMetrics {
    double pdop{999.9};
    double hdop{999.9};
    double vdop{999.9};
    double gdop{999.9};
    double residual_rms{0.0};
    std::int32_t iterations{0};
};

struct SolutionSummary {
    SolutionQuality quality{SolutionQuality::None};
    double receiver_clock_bias{0.0};
    double receiver_clock_drift{0.0};
    std::int32_t satellites_used{0};
    std::vector<double> satellite_elevations;
};

}  // namespace rtklib2::solvers::spp
