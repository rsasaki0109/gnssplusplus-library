#include <libgnss++/core/navigation.hpp>
#include <cmath>

namespace libgnss {

bool Ephemeris::calculateSatelliteState(const GNSSTime& time,
                                       Vector3d& pos,
                                       Vector3d& vel,
                                       double& clock_bias,
                                       double& clock_drift) const {
    if (!valid) return false;
    
    // Simplified satellite position calculation
    // In real implementation, would use full Kepler orbit calculation
    
    double dt = time - toe;
    if (dt > 302400.0) {
        dt -= 604800.0;
    } else if (dt < -302400.0) {
        dt += 604800.0;
    }
    
    // Mean motion
    const double GM = 3.986005e14;      // WGS84 gravitational parameter (m^3/s^2) - per IS-GPS-200
    double a = sqrt_a * sqrt_a; // Semi-major axis
    double n0 = std::sqrt(GM / (a * a * a));
    double n = n0 + delta_n;
    
    // Mean anomaly
    double M = m0 + n * dt;
    
    // Eccentric anomaly (iterative solution for Kepler's equation)
    double E = M;
    for (int i = 0; i < 10; ++i) {
        double E_old = E;
        E = M + e * std::sin(E_old);
        if (std::abs(E - E_old) < 1e-12) {
            break;
        }
    }
    
    // True anomaly
    double nu = std::atan2(std::sqrt(1 - e*e) * std::sin(E), std::cos(E) - e);
    
    // Second harmonic perturbations
    double sin2u = std::sin(2.0 * (omega + nu));
    double cos2u = std::cos(2.0 * (omega + nu));
    double du = cus * sin2u + cuc * cos2u; // argument of latitude correction
    double dr = crs * sin2u + crc * cos2u; // radius correction
    double di = cis * sin2u + cic * cos2u; // inclination correction

    // Corrected argument of latitude, radius, and inclination
    double u = omega + nu + du;
    double r = a * (1.0 - e * std::cos(E)) + dr;
    double i = i0 + idot * dt + di;

    // Position in orbital plane
    double x_orb = r * std::cos(u);
    double y_orb = r * std::sin(u);
    const double OMEGA_E = 7.2921150e-5;    // WGS84 Earth rotation rate (rad/s) - per IS-GPS-200
    double Omega = omega0 + (omega_dot - OMEGA_E) * dt - OMEGA_E * toe.tow;
    
    // ECEF coordinates
    pos(0) = x_orb * std::cos(Omega) - y_orb * std::cos(i) * std::sin(Omega);
    pos(1) = x_orb * std::sin(Omega) + y_orb * std::cos(i) * std::cos(Omega);
    pos(2) = y_orb * std::sin(i);
    
    // Simplified velocity (would be more complex in real implementation)
    vel.setZero();
    
    // Clock correction
    double dt_clock = time - toc;
    clock_bias = af0 + af1 * dt_clock + af2 * dt_clock * dt_clock;

    // Relativistic correction
    const double F = -4.442807633e-10; // s/m^(1/2)
    clock_bias += F * e * sqrt_a * std::sin(E);

    clock_drift = af1 + 2.0 * af2 * dt_clock;
    
    return true;
}

bool Ephemeris::isValid(const GNSSTime& time) const {
    if (!valid) return false;
    double age = std::abs(time - toe);
    return age < 7200.0; // 2 hours
}

double Ephemeris::getAge(const GNSSTime& time) const {
    return std::abs(time - toe);
}

void NavigationData::addEphemeris(const Ephemeris& eph) {
    ephemeris_data[eph.satellite].push_back(eph);
    
    // Sort by time of ephemeris
    auto& eph_list = ephemeris_data[eph.satellite];
    std::sort(eph_list.begin(), eph_list.end(), 
              [](const Ephemeris& a, const Ephemeris& b) {
                  return a.toe < b.toe;
              });
}

const Ephemeris* NavigationData::getEphemeris(const SatelliteId& sat, const GNSSTime& time) const {
    auto it = ephemeris_data.find(sat);
    if (it == ephemeris_data.end()) {
        return nullptr;
    }
    
    const Ephemeris* best_eph = nullptr;
    double min_age = 1e9;
    
    for (const auto& eph : it->second) {
        if (eph.isValid(time)) {
            double age = eph.getAge(time);
            if (age < min_age) {
                min_age = age;
                best_eph = &eph;
            }
        }
    }
    
    return best_eph;
}

bool NavigationData::calculateSatelliteState(const SatelliteId& sat,
                                           const GNSSTime& time,
                                           Vector3d& position,
                                           Vector3d& velocity,
                                           double& clock_bias,
                                           double& clock_drift) const {
    const Ephemeris* eph = getEphemeris(sat, time);
    if (!eph) {
        return false;
    }
    
    return eph->calculateSatelliteState(time, position, velocity, clock_bias, clock_drift);
}

std::map<SatelliteId, Vector3d> NavigationData::calculateSatellitePositions(
    const std::vector<SatelliteId>& satellites,
    const GNSSTime& time) const {
    
    std::map<SatelliteId, Vector3d> positions;
    
    for (const auto& sat : satellites) {
        Vector3d pos, vel;
        double clock_bias, clock_drift;
        
        if (calculateSatelliteState(sat, time, pos, vel, clock_bias, clock_drift)) {
            positions[sat] = pos;
        }
    }
    
    return positions;
}

NavigationData::SatelliteGeometry NavigationData::calculateGeometry(
    const Vector3d& receiver_pos,
    const Vector3d& satellite_pos) const {
    
    SatelliteGeometry geom;
    
    Vector3d los = satellite_pos - receiver_pos;
    geom.distance = los.norm();
    
    // Convert to local coordinates for elevation/azimuth
    Vector3d up = receiver_pos.normalized();
    Vector3d east = Vector3d(-receiver_pos(1), receiver_pos(0), 0.0).normalized();
    Vector3d north = up.cross(east);
    
    Vector3d los_local;
    los_local(0) = los.dot(east);
    los_local(1) = los.dot(north);
    los_local(2) = los.dot(up);
    
    geom.elevation = std::atan2(los_local(2), std::sqrt(los_local(0)*los_local(0) + los_local(1)*los_local(1)));
    geom.azimuth = std::atan2(los_local(0), los_local(1));
    
    if (geom.azimuth < 0) {
        geom.azimuth += 2.0 * M_PI;
    }
    
    return geom;
}

bool NavigationData::hasEphemeris(const SatelliteId& sat, const GNSSTime& time) const {
    return getEphemeris(sat, time) != nullptr;
}

std::vector<SatelliteId> NavigationData::getAvailableSatellites(const GNSSTime& time) const {
    std::vector<SatelliteId> satellites;
    
    for (const auto& pair : ephemeris_data) {
        if (hasEphemeris(pair.first, time)) {
            satellites.push_back(pair.first);
        }
    }
    
    return satellites;
}

NavigationData::NavigationData() {
    clear();
}

void NavigationData::clear() {
    ephemeris_data.clear();
}

bool NavigationData::isEmpty() const {
    return ephemeris_data.empty();
}

double IonosphereModel::calculateDelay(const GNSSTime& time,
                                     const GeodeticCoord& user_pos,
                                     const Vector3d& sat_pos,
                                     double frequency) const {
    if (!valid) return 0.0;
    
    // Simplified Klobuchar model
    double delay = 0.0;
    
    // Would implement full ionospheric delay calculation here
    // For now, return a simple frequency-dependent delay
    double f1 = constants::GPS_L1_FREQ;
    delay = 5.0 * (f1 * f1) / (frequency * frequency); // meters
    
    return delay;
}

double TroposphereModel::calculateDelay(const GeodeticCoord& user_pos,
                                      double elevation,
                                      const GNSSTime& time) const {
    // Simplified Saastamoinen model
    double height = user_pos.height;
    double lat = user_pos.latitude;
    
    // Pressure at sea level
    double P0 = 1013.25; // mbar
    
    // Pressure at user height
    double P = P0 * std::pow(1.0 - 2.26e-5 * height, 5.225);
    
    // Temperature
    double T = 288.15 - 6.5e-3 * height; // K
    
    // Water vapor pressure (simplified)
    double e = 6.108 * std::exp(17.15 * T / (234.7 + T));
    
    // Zenith delays
    double Zd = 0.002277 * (P + (1255.0/T + 0.05) * e);
    
    // Mapping function (simplified)
    double mapping = 1.0 / std::sin(elevation);
    if (mapping > 10.0) mapping = 10.0;
    
    return Zd * mapping;
}

bool PreciseProducts::loadSP3File(const std::string& filename) {
    // Stub implementation - would parse SP3 format
    (void)filename;
    return false;
}

bool PreciseProducts::loadClockFile(const std::string& filename) {
    // Stub implementation - would parse clock RINEX format
    (void)filename;
    return false;
}

} // namespace libgnss
