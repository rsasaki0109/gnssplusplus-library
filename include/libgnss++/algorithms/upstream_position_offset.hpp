#pragma once

// Raw-only port of the pinned taroz `add_position_offset.m` post-processing
// contract.  This header intentionally contains no observation, truth, or
// trajectory I/O: it maps a fixed phone-local offset through the optimized
// Pose3 rpy and returns a finite local ENU displacement.

#include <Eigen/Dense>

#include <cmath>
#include <string_view>

namespace libgnss::upstream_position_offset {

struct PhoneOffset {
    double offset_rl_m = 0.0;
    double offset_ud_m = 0.0;
};

struct Result {
    bool ok = false;
    PhoneOffset phone_offset;
    Eigen::Vector3d offset_enu_m = Eigen::Vector3d::Zero();
};

inline bool finite(const Eigen::Vector3d& value) {
    return value.allFinite();
}

/**
 * Match the upstream phone branches in their original order.
 *
 * `sm-g988` is checked before the generic `sm`/`samsung` branch, and the
 * pixel6pro exact branch is checked before the pixel7/pixel4/pixel5
 * substring branches.  Unknown phone families are deliberately rejected.
 */
inline bool phoneOffset(std::string_view phone, PhoneOffset& offset) {
    if (phone.find("mi8") != std::string_view::npos) {
        offset = {0.25, -0.35};
        return true;
    }
    if (phone == "sm-g988") {
        offset = {0.20, -0.05};
        return true;
    }
    if (phone.find("sm") != std::string_view::npos ||
        phone.find("samsung") != std::string_view::npos) {
        offset = {0.30, -0.25};
        return true;
    }
    if (phone == "pixel6pro") {
        offset = {-0.20, -0.15};
        return true;
    }
    if (phone.find("pixel7") != std::string_view::npos) {
        offset = {-0.10, -0.20};
        return true;
    }
    if (phone.find("pixel4") != std::string_view::npos) {
        offset = {-0.00, -0.15};
        return true;
    }
    if (phone.find("pixel5") != std::string_view::npos) {
        offset = {-0.10, -0.30};
        return true;
    }
    return false;
}

/**
 * Port MATLAB eul2rotm exactly: Rx * Ry * Rz, after the upstream yaw shift
 * rpy-[0 0 pi].  `rpy_rad` is [roll,pitch,yaw] in radians.
 */
inline Eigen::Matrix3d eul2rotm(const Eigen::Vector3d& rpy_rad) {
    const Eigen::Vector3d eul =
        rpy_rad - Eigen::Vector3d(0.0, 0.0, 3.1415926535897932384626433832795);
    const double cx = std::cos(eul.x());
    const double sx = std::sin(eul.x());
    const double cy = std::cos(eul.y());
    const double sy = std::sin(eul.y());
    const double cz = std::cos(eul.z());
    const double sz = std::sin(eul.z());
    Eigen::Matrix3d rx;
    rx << 1.0, 0.0, 0.0,
          0.0, cx, -sx,
          0.0, sx, cx;
    Eigen::Matrix3d ry;
    ry << cy, 0.0, sy,
          0.0, 1.0, 0.0,
          -sy, 0.0, cy;
    Eigen::Matrix3d rz;
    rz << cz, -sz, 0.0,
          sz, cz, 0.0,
          0.0, 0.0, 1.0;
    return rx * ry * rz;
}

inline Result offsetFromRpy(std::string_view phone,
                            const Eigen::Vector3d& rpy_rad) {
    Result result;
    if (!finite(rpy_rad) || !phoneOffset(phone, result.phone_offset)) {
        return result;
    }
    // Upstream names the local components UD then RL in this order.
    result.offset_enu_m = eul2rotm(rpy_rad) *
                          Eigen::Vector3d(result.phone_offset.offset_ud_m,
                                          result.phone_offset.offset_rl_m, 0.0);
    result.ok = finite(result.offset_enu_m);
    if (!result.ok) result.offset_enu_m.setZero();
    return result;
}

}  // namespace libgnss::upstream_position_offset
