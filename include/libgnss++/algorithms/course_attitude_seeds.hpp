#pragma once

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace libgnss::course_attitude_seeds {

// Per-epoch GNSS course (ENU yaw, rad, atan2(north, east)) for IMU attitude
// seeds.  Epochs slower than min_speed_mps take the course of the nearest
// earlier moving epoch, and leading slow epochs take the first moving course.
// A seed of course + a constant phone mount yaw keeps the optimizer out of the
// basin where the yaw slips a full turn against the gyro (2023-05-09 sjc-r
// A505U, whose dead-reckoned and velocity-heading seeds both slip).
inline std::vector<double> filledCourse(const std::vector<Eigen::Vector3d>& velocity_enu,
                                        double min_speed_mps = 3.0) {
    std::vector<double> course(velocity_enu.size(), std::numeric_limits<double>::quiet_NaN());
    bool moving = false;
    for (std::size_t i = 0; i < velocity_enu.size(); ++i) {
        const auto& v = velocity_enu[i];
        if (!v.allFinite()) throw std::invalid_argument("course attitude seeds need finite velocities");
        if (std::hypot(v.x(), v.y()) >= min_speed_mps) {
            course[i] = std::atan2(v.y(), v.x());
            moving = true;
        }
    }
    if (!moving) throw std::invalid_argument("course attitude seeds need at least one moving epoch");
    double last = std::numeric_limits<double>::quiet_NaN();
    for (double& c : course) {
        if (std::isfinite(c)) last = c; else c = last;
    }
    last = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t k = course.size(); k-- > 0;) {
        if (std::isfinite(course[k])) last = course[k]; else course[k] = last;
    }
    return course;
}

}  // namespace libgnss::course_attitude_seeds
