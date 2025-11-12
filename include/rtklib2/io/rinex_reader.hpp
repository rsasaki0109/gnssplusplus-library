#pragma once

#include <string>
#include <vector>

#include "libgnss++/core/navigation.hpp"
#include "libgnss++/core/observation.hpp"

namespace rtklib2::io::rinex {

struct ObservationHeader {
    double version{0.0};
    std::string marker_name;
    libgnss::Vector3d approximate_position{libgnss::Vector3d::Zero()};
};

bool read_observation_file(const std::string& file_path,
                           std::vector<libgnss::ObservationData>& epochs,
                           ObservationHeader* header = nullptr);

bool read_navigation_file(const std::string& file_path,
                          libgnss::NavigationData& navigation_data);

}  // namespace rtklib2::io::rinex
