#pragma once

#include <libgnss++/core/types.hpp>

namespace libgnss {
namespace tidal {

/// Julian date from GPS time.
double julianDateFromTime(const GNSSTime& time);

/// Greenwich Mean Sidereal Time (radians).
double greenwichMeanSiderealTime(const GNSSTime& time);

/// Rotate a vector from ECI to ECEF frame.
Vector3d rotateEciToEcef(const Vector3d& eci, double gmst_rad);

/// Approximate Sun position in ECEF (meters).
Vector3d approximateSunPositionEcef(const GNSSTime& time);

/// Approximate Moon position in ECEF (meters).
Vector3d approximateMoonPositionEcef(const GNSSTime& time);

/// Surface-up unit vector at a given ECEF position.
Vector3d surfaceUpVector(const Vector3d& receiver_position);

/// Degree-2 body tide displacement at the receiver due to one celestial body.
Vector3d bodyTideDisplacement(const Vector3d& receiver_position,
                              const Vector3d& body_position_ecef,
                              double body_gm_m3_s2);

/// Total solid Earth tide displacement at the receiver (meters, ECEF).
/// This is the sum of Sun and Moon degree-2 body tide contributions.
Vector3d calculateSolidEarthTides(const Vector3d& receiver_position,
                                  const GNSSTime& time);

}  // namespace tidal
}  // namespace libgnss
