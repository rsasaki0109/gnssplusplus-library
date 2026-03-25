#include <libgnss++/core/navigation.hpp>
#include <cmath>

namespace libgnss {

namespace {

constexpr double kGpsWeekSeconds = 604800.0;
constexpr double kHalfGpsWeekSeconds = 302400.0;
constexpr double kEarthRotationRate = 7.2921151467e-5;
constexpr double kEarthMu = 3.986005e14;
constexpr double kGlonassEarthMu = 3.9860044e14;
constexpr double kGlonassJ2 = 1.0826257e-3;
constexpr double kGlonassEarthRadius = 6378136.0;
constexpr double kGlonassEarthRotationRate = 7.292115e-5;
constexpr double kGlonassIntegrationStep = 60.0;
constexpr double kBeiDouEarthRotationRate = 7.292115e-5;
constexpr double kBeiDouEarthMu = 3.986004418e14;
constexpr double kRelativityF = -4.442807633e-10;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84E2 = 6.69437999014e-3;
constexpr double kBeiDouGeoSin5Deg = -0.0871557427476582;
constexpr double kBeiDouGeoCos5Deg = 0.9961946980917456;

double normalizeGpsTime(double dt) {
    while (dt > kHalfGpsWeekSeconds) {
        dt -= kGpsWeekSeconds;
    }
    while (dt < -kHalfGpsWeekSeconds) {
        dt += kGpsWeekSeconds;
    }
    return dt;
}

double solveKepler(double mean_anomaly, double eccentricity) {
    double E = mean_anomaly;
    for (int i = 0; i < 30; ++i) {  // RTKLIB: MAX_ITER_KEPLER=30
        double Ek = E;
        E -= (E - eccentricity * std::sin(E) - mean_anomaly) / (1.0 - eccentricity * std::cos(E));
        if (std::abs(E - Ek) < 1e-14) break;  // RTKLIB: RTOL_KEPLER=1e-14
    }
    return E;
}

bool computeBroadcastState(const Ephemeris& eph,
                           const GNSSTime& time,
                           Vector3d& pos,
                           double& clock_bias,
                           double& clock_drift) {
    if (!eph.valid || eph.sqrt_a <= 0.0) {
        return false;
    }

    const double a = eph.sqrt_a * eph.sqrt_a;
    const double tk = normalizeGpsTime(time - eph.toe);
    const double tc = normalizeGpsTime(time - eph.toc);
    const bool is_beidou = eph.satellite.system == GNSSSystem::BeiDou;
    const double earth_mu = is_beidou ? kBeiDouEarthMu : kEarthMu;
    const double earth_rotation = is_beidou ? kBeiDouEarthRotationRate : kEarthRotationRate;

    const double n0 = std::sqrt(earth_mu / (a * a * a));
    const double n = n0 + eph.delta_n;
    const double mean_anomaly = eph.m0 + n * tk;
    const double eccentric_anomaly = solveKepler(mean_anomaly, eph.e);

    const double sin_e = std::sin(eccentric_anomaly);
    const double cos_e = std::cos(eccentric_anomaly);
    const double sqrt_one_minus_e2 = std::sqrt(1.0 - eph.e * eph.e);
    const double true_anomaly = std::atan2(sqrt_one_minus_e2 * sin_e, cos_e - eph.e);
    const double phi = true_anomaly + eph.omega;

    const double two_phi = 2.0 * phi;
    const double du = eph.cus * std::sin(two_phi) + eph.cuc * std::cos(two_phi);
    const double dr = eph.crs * std::sin(two_phi) + eph.crc * std::cos(two_phi);
    const double di = eph.cis * std::sin(two_phi) + eph.cic * std::cos(two_phi);

    const double u = phi + du;
    const double r = a * (1.0 - eph.e * cos_e) + dr;
    const double inclination = eph.i0 + eph.idot * tk + di;
    const double toe_seconds = eph.toes != 0.0 ? eph.toes : eph.toe.tow;
    const double omega = eph.omega0 + (eph.omega_dot - earth_rotation) * tk
        - earth_rotation * toe_seconds;

    const double x_orb = r * std::cos(u);
    const double y_orb = r * std::sin(u);

    const double cos_omega = std::cos(omega);
    const double sin_omega = std::sin(omega);
    const double cos_i = std::cos(inclination);
    const double sin_i = std::sin(inclination);

    if (is_beidou && eph.satellite.prn <= 5) {
        const double xg = x_orb * cos_omega - y_orb * cos_i * sin_omega;
        const double yg = x_orb * sin_omega + y_orb * cos_i * cos_omega;
        const double zg = y_orb * sin_i;
        const double sin_rot = std::sin(earth_rotation * tk);
        const double cos_rot = std::cos(earth_rotation * tk);
        pos(0) = xg * cos_rot + yg * sin_rot * kBeiDouGeoCos5Deg +
                 zg * sin_rot * kBeiDouGeoSin5Deg;
        pos(1) = -xg * sin_rot + yg * cos_rot * kBeiDouGeoCos5Deg +
                 zg * cos_rot * kBeiDouGeoSin5Deg;
        pos(2) = -yg * kBeiDouGeoSin5Deg + zg * kBeiDouGeoCos5Deg;
    } else {
        pos(0) = x_orb * cos_omega - y_orb * cos_i * sin_omega;
        pos(1) = x_orb * sin_omega + y_orb * cos_i * cos_omega;
        pos(2) = y_orb * sin_i;
    }

    clock_bias = eph.af0 + eph.af1 * tc + eph.af2 * tc * tc;
    clock_bias += kRelativityF * eph.e * eph.sqrt_a * sin_e;
    // TGD NOT applied here — it's applied in SPP only (DD cancels TGD)
    // clock_bias -= eph.tgd;
    clock_drift = eph.af1 + 2.0 * eph.af2 * tc;
    return true;
}

void computeGlonassDynamics(const double* state, double* derivative, const Vector3d& acceleration) {
    const double r2 = state[0] * state[0] + state[1] * state[1] + state[2] * state[2];
    if (r2 <= 0.0) {
        for (int i = 0; i < 6; ++i) derivative[i] = 0.0;
        return;
    }

    const double r3 = r2 * std::sqrt(r2);
    const double omg2 = kGlonassEarthRotationRate * kGlonassEarthRotationRate;
    const double a = 1.5 * kGlonassJ2 * kGlonassEarthMu *
                     (kGlonassEarthRadius * kGlonassEarthRadius) / (r2 * r3);
    const double b = 5.0 * state[2] * state[2] / r2;
    const double c = -kGlonassEarthMu / r3 - a * (1.0 - b);

    derivative[0] = state[3];
    derivative[1] = state[4];
    derivative[2] = state[5];
    derivative[3] = (c + omg2) * state[0] + 2.0 * kGlonassEarthRotationRate * state[4] + acceleration(0);
    derivative[4] = (c + omg2) * state[1] - 2.0 * kGlonassEarthRotationRate * state[3] + acceleration(1);
    derivative[5] = (c - 2.0 * a) * state[2] + acceleration(2);
}

void propagateGlonassOrbit(double dt, double* state, const Vector3d& acceleration) {
    double k1[6], k2[6], k3[6], k4[6], work[6];

    computeGlonassDynamics(state, k1, acceleration);
    for (int i = 0; i < 6; ++i) work[i] = state[i] + k1[i] * dt / 2.0;
    computeGlonassDynamics(work, k2, acceleration);
    for (int i = 0; i < 6; ++i) work[i] = state[i] + k2[i] * dt / 2.0;
    computeGlonassDynamics(work, k3, acceleration);
    for (int i = 0; i < 6; ++i) work[i] = state[i] + k3[i] * dt;
    computeGlonassDynamics(work, k4, acceleration);

    for (int i = 0; i < 6; ++i) {
        state[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * dt / 6.0;
    }
}

bool computeGlonassState(const Ephemeris& eph,
                         const GNSSTime& time,
                         Vector3d& pos,
                         Vector3d& vel,
                         double& clock_bias,
                         double& clock_drift) {
    if (!eph.valid) {
        return false;
    }

    double state[6] = {
        eph.glonass_position(0), eph.glonass_position(1), eph.glonass_position(2),
        eph.glonass_velocity(0), eph.glonass_velocity(1), eph.glonass_velocity(2),
    };

    double t = time - eph.toe;
    clock_bias = -eph.glonass_taun + eph.glonass_gamn * t;
    clock_drift = eph.glonass_gamn;

    for (double step = t < 0.0 ? -kGlonassIntegrationStep : kGlonassIntegrationStep;
         std::abs(t) > 1e-9;
         t -= step) {
        if (std::abs(t) < kGlonassIntegrationStep) {
            step = t;
        }
        propagateGlonassOrbit(step, state, eph.glonass_acceleration);
    }

    pos = Vector3d(state[0], state[1], state[2]);
    vel = Vector3d(state[3], state[4], state[5]);
    return true;
}

void ecefToGeodetic(const Vector3d& ecef, double& lat, double& lon, double& h) {
    lon = std::atan2(ecef(1), ecef(0));
    const double p = std::sqrt(ecef(0) * ecef(0) + ecef(1) * ecef(1));
    lat = std::atan2(ecef(2), p * (1.0 - kWgs84E2));

    for (int i = 0; i < 8; ++i) {
        const double sin_lat = std::sin(lat);
        const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
        h = p / std::cos(lat) - n;
        lat = std::atan2(ecef(2), p * (1.0 - kWgs84E2 * n / (n + h)));
    }

    const double sin_lat = std::sin(lat);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    h = p / std::cos(lat) - n;
}

} // namespace

bool Ephemeris::calculateSatelliteState(const GNSSTime& time,
                                       Vector3d& pos,
                                       Vector3d& vel,
                                       double& clock_bias,
                                       double& clock_drift) const {
    if (satellite.system == GNSSSystem::GLONASS) {
        return computeGlonassState(*this, time, pos, vel, clock_bias, clock_drift);
    }

    if (!computeBroadcastState(*this, time, pos, clock_bias, clock_drift)) {
        return false;
    }

    Vector3d pos_forward;
    Vector3d pos_backward;
    double clk_forward = 0.0;
    double clk_backward = 0.0;
    double drift_dummy = 0.0;
    const double dt = 0.5;

    if (computeBroadcastState(*this, time + dt, pos_forward, clk_forward, drift_dummy) &&
        computeBroadcastState(*this, time - dt, pos_backward, clk_backward, drift_dummy)) {
        vel = (pos_forward - pos_backward) / (2.0 * dt);
        clock_drift = (clk_forward - clk_backward) / (2.0 * dt);
    } else {
        vel.setZero();
    }

    return true;
}

bool Ephemeris::isValid(const GNSSTime& time) const {
    if (!valid) return false;
    if (satellite.system == GNSSSystem::GLONASS) {
        return std::abs(time - toe) <= 1800.0;
    }
    double age = std::abs(time - toe);
    return age <= 14400.0; // 4 hours (RTKLIB MAXDTOE=7200 but relaxed for sparse nav data)
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
    
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecefToGeodetic(receiver_pos, lat, lon, height);

    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    Vector3d east(-sin_lon, cos_lon, 0.0);
    Vector3d north(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    Vector3d up(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);

    Vector3d los_local(los.dot(east), los.dot(north), los.dot(up));
    
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
