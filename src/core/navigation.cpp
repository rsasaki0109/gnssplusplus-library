#include <libgnss++/core/navigation.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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
constexpr double kPreciseInterpolationGapSeconds = 900.0;

std::string trimCopy(const std::string& value) {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

GNSSSystem systemFromChar(char system_char) {
    switch (system_char) {
        case 'G': return GNSSSystem::GPS;
        case 'R': return GNSSSystem::GLONASS;
        case 'E': return GNSSSystem::Galileo;
        case 'C': return GNSSSystem::BeiDou;
        case 'J': return GNSSSystem::QZSS;
        case 'S': return GNSSSystem::SBAS;
        case 'I': return GNSSSystem::NavIC;
        default: return GNSSSystem::UNKNOWN;
    }
}

bool parseSatelliteToken(const std::string& token, SatelliteId& satellite) {
    if (token.size() < 2) {
        return false;
    }
    const GNSSSystem system = systemFromChar(token[0]);
    if (system == GNSSSystem::UNKNOWN) {
        return false;
    }
    try {
        satellite = SatelliteId(system, static_cast<uint8_t>(std::stoi(token.substr(1))));
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseEpochFields(int year,
                      int month,
                      int day,
                      int hour,
                      int minute,
                      double second,
                      GNSSTime& time) {
    std::tm epoch_tm{};
    epoch_tm.tm_year = year - 1900;
    epoch_tm.tm_mon = month - 1;
    epoch_tm.tm_mday = day;
    epoch_tm.tm_hour = hour;
    epoch_tm.tm_min = minute;
    epoch_tm.tm_sec = static_cast<int>(std::floor(second));
    const time_t seconds_since_unix = timegm(&epoch_tm);
    if (seconds_since_unix == static_cast<time_t>(-1)) {
        return false;
    }
    const auto tp = std::chrono::system_clock::from_time_t(seconds_since_unix) +
        std::chrono::microseconds(
            static_cast<long long>(std::llround((second - std::floor(second)) * 1e6)));
    time = GNSSTime::fromSystemTime(tp);
    return true;
}

bool parseSp3EpochLine(const std::string& line, GNSSTime& time) {
    if (line.empty() || line[0] != '*') {
        return false;
    }
    std::istringstream stream(line.substr(1));
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!(stream >> year >> month >> day >> hour >> minute >> second)) {
        return false;
    }
    return parseEpochFields(year, month, day, hour, minute, second, time);
}

bool parseClockEpochTokens(std::istringstream& stream, GNSSTime& time) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!(stream >> year >> month >> day >> hour >> minute >> second)) {
        return false;
    }
    return parseEpochFields(year, month, day, hour, minute, second, time);
}

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
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    GNSSTime current_time;
    bool have_epoch = false;
    bool loaded_any = false;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        if (line[0] == '*') {
            have_epoch = parseSp3EpochLine(line, current_time);
            continue;
        }
        if (!have_epoch || line[0] != 'P') {
            continue;
        }

        std::istringstream stream(line.substr(1));
        std::string sat_token;
        double x_km = 0.0;
        double y_km = 0.0;
        double z_km = 0.0;
        double clock_microseconds = 0.0;
        if (!(stream >> sat_token >> x_km >> y_km >> z_km >> clock_microseconds)) {
            continue;
        }

        SatelliteId satellite;
        if (!parseSatelliteToken(sat_token, satellite)) {
            continue;
        }

        PreciseOrbitClock sample;
        sample.satellite = satellite;
        sample.time = current_time;
        sample.position = Vector3d(x_km * 1000.0, y_km * 1000.0, z_km * 1000.0);
        sample.position_valid = std::isfinite(sample.position.x()) &&
            std::isfinite(sample.position.y()) &&
            std::isfinite(sample.position.z());
        if (std::isfinite(clock_microseconds) && std::abs(clock_microseconds) < 999999.0) {
            sample.clock_bias = clock_microseconds * 1e-6;
            sample.clock_valid = true;
        }
        addOrbitClock(sample);
        loaded_any = true;
    }
    return loaded_any;
}

bool PreciseProducts::loadClockFile(const std::string& filename) {
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_header = true;
    bool loaded_any = false;
    std::string line;
    while (std::getline(input, line)) {
        if (in_header) {
            if (line.find("END OF HEADER") != std::string::npos) {
                in_header = false;
            }
            continue;
        }
        if (line.size() < 2 || line[0] != 'A' || line[1] != 'S') {
            continue;
        }

        std::istringstream stream(line);
        std::string record_type;
        std::string sat_token;
        if (!(stream >> record_type >> sat_token)) {
            continue;
        }

        SatelliteId satellite;
        if (!parseSatelliteToken(sat_token, satellite)) {
            continue;
        }

        GNSSTime current_time;
        if (!parseClockEpochTokens(stream, current_time)) {
            continue;
        }

        int value_count = 0;
        double clock_bias = 0.0;
        double clock_sigma = 0.0;
        if (!(stream >> value_count >> clock_bias)) {
            continue;
        }
        if (value_count > 1) {
            stream >> clock_sigma;
        }

        PreciseOrbitClock sample;
        sample.satellite = satellite;
        sample.time = current_time;
        sample.clock_bias = clock_bias;
        sample.clock_sigma = clock_sigma;
        sample.clock_valid = std::isfinite(clock_bias);
        addOrbitClock(sample);
        loaded_any = true;
    }
    return loaded_any;
}

void PreciseProducts::addOrbitClock(const PreciseOrbitClock& data) {
    auto& entries = orbit_clock_data[data.satellite];
    auto it = std::lower_bound(
        entries.begin(),
        entries.end(),
        data.time,
        [](const PreciseOrbitClock& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });

    if (it != entries.end() && it->time == data.time) {
        if (data.position_valid) {
            it->position = data.position;
            it->velocity = data.velocity;
            it->position_sigma = data.position_sigma;
            it->position_valid = true;
        }
        if (data.clock_valid) {
            it->clock_bias = data.clock_bias;
            it->clock_drift = data.clock_drift;
            it->clock_sigma = data.clock_sigma;
            it->clock_valid = true;
        }
        return;
    }

    entries.insert(it, data);
}

bool PreciseProducts::interpolateOrbitClock(const SatelliteId& sat,
                                            const GNSSTime& time,
                                            Vector3d& position,
                                            Vector3d& velocity,
                                            double& clock_bias,
                                            double& clock_drift) const {
    const auto sat_it = orbit_clock_data.find(sat);
    if (sat_it == orbit_clock_data.end() || sat_it->second.empty()) {
        return false;
    }

    const auto& entries = sat_it->second;
    auto upper = std::lower_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const PreciseOrbitClock& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });

    const PreciseOrbitClock* before = nullptr;
    const PreciseOrbitClock* after = nullptr;
    if (upper != entries.end()) {
        after = &(*upper);
        if (upper != entries.begin()) {
            before = &(*(upper - 1));
        }
        if (upper->time == time) {
            before = after;
        }
    } else {
        before = &entries.back();
    }

    if (before == nullptr && after == nullptr) {
        return false;
    }

    if (before != nullptr && after != nullptr && before != after) {
        const double dt_total = after->time - before->time;
        const double dt_before = time - before->time;
        if (std::abs(dt_total) < 1e-9 || std::abs(dt_total) > kPreciseInterpolationGapSeconds) {
            return false;
        }
        const double alpha = dt_before / dt_total;
        if ((before->position_valid || after->position_valid) &&
            before->position_valid && after->position_valid) {
            position = before->position + alpha * (after->position - before->position);
            velocity = (after->position - before->position) / dt_total;
        } else if (before->position_valid) {
            position = before->position;
            velocity = before->velocity;
        } else if (after->position_valid) {
            position = after->position;
            velocity = after->velocity;
        } else {
            return false;
        }

        if (before->clock_valid && after->clock_valid) {
            clock_bias = before->clock_bias + alpha * (after->clock_bias - before->clock_bias);
            clock_drift = (after->clock_bias - before->clock_bias) / dt_total;
        } else if (before->clock_valid) {
            clock_bias = before->clock_bias;
            clock_drift = before->clock_drift;
        } else if (after->clock_valid) {
            clock_bias = after->clock_bias;
            clock_drift = after->clock_drift;
        } else {
            clock_bias = 0.0;
            clock_drift = 0.0;
        }
        return true;
    }

    const PreciseOrbitClock* sample = before != nullptr ? before : after;
    if (sample == nullptr) {
        return false;
    }
    if (std::abs(sample->time - time) > kPreciseInterpolationGapSeconds) {
        return false;
    }
    if (!sample->position_valid) {
        return false;
    }
    position = sample->position;
    velocity = sample->velocity;
    clock_bias = sample->clock_valid ? sample->clock_bias : 0.0;
    clock_drift = sample->clock_valid ? sample->clock_drift : 0.0;
    return true;
}

bool PreciseProducts::hasData(const SatelliteId& sat, const GNSSTime& time) const {
    Vector3d position;
    Vector3d velocity;
    double clock_bias = 0.0;
    double clock_drift = 0.0;
    return interpolateOrbitClock(sat, time, position, velocity, clock_bias, clock_drift);
}

void PreciseProducts::clear() {
    orbit_clock_data.clear();
}

void SSRProducts::addCorrection(const SSROrbitClockCorrection& correction) {
    auto& entries = orbit_clock_corrections[correction.satellite];
    auto it = std::lower_bound(
        entries.begin(),
        entries.end(),
        correction.time,
        [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });

    if (it != entries.end() && it->time == correction.time) {
        if (correction.orbit_valid) {
            it->orbit_correction_ecef = correction.orbit_correction_ecef;
            it->orbit_valid = true;
        }
        if (correction.clock_valid) {
            it->clock_correction_m = correction.clock_correction_m;
            it->clock_valid = true;
        }
        if (correction.ura_valid) {
            it->ura_sigma_m = correction.ura_sigma_m;
            it->ura_valid = true;
        }
        if (correction.code_bias_valid) {
            it->code_bias_m = correction.code_bias_m;
            it->code_bias_valid = !it->code_bias_m.empty();
        }
        if (correction.phase_bias_valid) {
            it->phase_bias_m = correction.phase_bias_m;
            it->phase_bias_valid = !it->phase_bias_m.empty();
        }
        if (correction.atmos_valid) {
            it->atmos_tokens = correction.atmos_tokens;
            it->atmos_valid = !it->atmos_tokens.empty();
        }
        return;
    }

    entries.insert(it, correction);
}

bool SSRProducts::interpolateCorrection(const SatelliteId& sat,
                                        const GNSSTime& time,
                                        Vector3d& orbit_correction_ecef,
                                        double& clock_correction_m,
                                        double* ura_sigma_m,
                                        std::map<uint8_t, double>* code_bias_m,
                                        std::map<uint8_t, double>* phase_bias_m,
                                        std::map<std::string, std::string>* atmos_tokens) const {
    const auto sat_it = orbit_clock_corrections.find(sat);
    if (sat_it == orbit_clock_corrections.end() || sat_it->second.empty()) {
        return false;
    }
    if (ura_sigma_m != nullptr) {
        *ura_sigma_m = 0.0;
    }
    if (code_bias_m != nullptr) {
        code_bias_m->clear();
    }
    if (phase_bias_m != nullptr) {
        phase_bias_m->clear();
    }
    if (atmos_tokens != nullptr) {
        atmos_tokens->clear();
    }

    const auto& entries = sat_it->second;
    auto upper = std::lower_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });

    const SSROrbitClockCorrection* before = nullptr;
    const SSROrbitClockCorrection* after = nullptr;
    if (upper != entries.end()) {
        after = &(*upper);
        if (upper != entries.begin()) {
            before = &(*(upper - 1));
        }
        if (upper->time == time) {
            before = after;
        }
    } else {
        before = &entries.back();
    }

    if (before == nullptr && after == nullptr) {
        return false;
    }

    if (before != nullptr && after != nullptr && before != after) {
        const double dt_total = after->time - before->time;
        const double dt_before = time - before->time;
        if (std::abs(dt_total) < 1e-9 || std::abs(dt_total) > kPreciseInterpolationGapSeconds) {
            return false;
        }
        const double alpha = dt_before / dt_total;
        if (before->orbit_valid && after->orbit_valid) {
            orbit_correction_ecef =
                before->orbit_correction_ecef +
                alpha * (after->orbit_correction_ecef - before->orbit_correction_ecef);
        } else if (before->orbit_valid) {
            orbit_correction_ecef = before->orbit_correction_ecef;
        } else if (after->orbit_valid) {
            orbit_correction_ecef = after->orbit_correction_ecef;
        } else {
            orbit_correction_ecef.setZero();
        }

        if (before->clock_valid && after->clock_valid) {
            clock_correction_m =
                before->clock_correction_m +
                alpha * (after->clock_correction_m - before->clock_correction_m);
        } else if (before->clock_valid) {
            clock_correction_m = before->clock_correction_m;
        } else if (after->clock_valid) {
            clock_correction_m = after->clock_correction_m;
        } else {
            clock_correction_m = 0.0;
        }
        if (ura_sigma_m != nullptr) {
            if (before->ura_valid && after->ura_valid) {
                *ura_sigma_m =
                    before->ura_sigma_m + alpha * (after->ura_sigma_m - before->ura_sigma_m);
            } else if (before->ura_valid) {
                *ura_sigma_m = before->ura_sigma_m;
            } else if (after->ura_valid) {
                *ura_sigma_m = after->ura_sigma_m;
            }
        }
        if (code_bias_m != nullptr) {
            if (before->code_bias_valid && !before->code_bias_m.empty()) {
                *code_bias_m = before->code_bias_m;
            } else if (after->code_bias_valid && !after->code_bias_m.empty()) {
                *code_bias_m = after->code_bias_m;
            }
        }
        if (phase_bias_m != nullptr) {
            if (before->phase_bias_valid && !before->phase_bias_m.empty()) {
                *phase_bias_m = before->phase_bias_m;
            } else if (after->phase_bias_valid && !after->phase_bias_m.empty()) {
                *phase_bias_m = after->phase_bias_m;
            }
        }
        if (atmos_tokens != nullptr) {
            if (before->atmos_valid && !before->atmos_tokens.empty()) {
                *atmos_tokens = before->atmos_tokens;
            } else if (after->atmos_valid && !after->atmos_tokens.empty()) {
                *atmos_tokens = after->atmos_tokens;
            }
        }
        return before->orbit_valid || after->orbit_valid ||
               before->clock_valid || after->clock_valid ||
               before->ura_valid || after->ura_valid ||
               before->code_bias_valid || after->code_bias_valid ||
               before->phase_bias_valid || after->phase_bias_valid ||
               before->atmos_valid || after->atmos_valid;
    }

    const SSROrbitClockCorrection* sample = before != nullptr ? before : after;
    if (sample == nullptr) {
        return false;
    }
    if (std::abs(sample->time - time) > kPreciseInterpolationGapSeconds) {
        return false;
    }

    orbit_correction_ecef =
        sample->orbit_valid ? sample->orbit_correction_ecef : Vector3d::Zero();
    clock_correction_m = sample->clock_valid ? sample->clock_correction_m : 0.0;
    if (ura_sigma_m != nullptr) {
        *ura_sigma_m = sample->ura_valid ? sample->ura_sigma_m : 0.0;
    }
    if (code_bias_m != nullptr && sample->code_bias_valid) {
        *code_bias_m = sample->code_bias_m;
    }
    if (phase_bias_m != nullptr && sample->phase_bias_valid) {
        *phase_bias_m = sample->phase_bias_m;
    }
    if (atmos_tokens != nullptr && sample->atmos_valid) {
        *atmos_tokens = sample->atmos_tokens;
    }
    return sample->orbit_valid || sample->clock_valid || sample->ura_valid ||
           sample->code_bias_valid || sample->phase_bias_valid || sample->atmos_valid;
}

bool SSRProducts::loadCSVFile(const std::string& filename) {
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool loaded_any = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        std::vector<std::string> columns;
        std::istringstream csv_stream(trimmed);
        std::string token;
        while (std::getline(csv_stream, token, ',')) {
            columns.push_back(trimCopy(token));
        }
        if (columns.size() < 7U) {
            continue;
        }

        int week = 0;
        double tow = 0.0;
        double dx = 0.0;
        double dy = 0.0;
        double dz = 0.0;
        double dclock_m = 0.0;
        const std::string& sat_token = columns[2];
        try {
            week = std::stoi(columns[0]);
            tow = std::stod(columns[1]);
            dx = std::stod(columns[3]);
            dy = std::stod(columns[4]);
            dz = std::stod(columns[5]);
            dclock_m = std::stod(columns[6]);
        } catch (const std::exception&) {
            continue;
        }

        SatelliteId satellite;
        if (!parseSatelliteToken(sat_token, satellite)) {
            continue;
        }

        SSROrbitClockCorrection correction;
        correction.satellite = satellite;
        correction.time = GNSSTime(week, tow);
        correction.orbit_correction_ecef = Vector3d(dx, dy, dz);
        correction.clock_correction_m = dclock_m;
        correction.orbit_valid =
            std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz);
        correction.clock_valid = std::isfinite(dclock_m);
        for (size_t index = 7; index < columns.size(); ++index) {
            const std::string& extra = columns[index];
            if (extra.empty()) {
                continue;
            }
            if (extra.rfind("ura_sigma_m=", 0) == 0) {
                try {
                    correction.ura_sigma_m = std::stod(extra.substr(12));
                    correction.ura_valid = std::isfinite(correction.ura_sigma_m);
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("cbias:", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 6) {
                    continue;
                }
                try {
                    const int signal_id = std::stoi(extra.substr(6, equal_pos - 6));
                    const double bias_m = std::stod(extra.substr(equal_pos + 1));
                    if (signal_id >= 0 && signal_id <= 255 && std::isfinite(bias_m)) {
                        correction.code_bias_m[static_cast<uint8_t>(signal_id)] = bias_m;
                    }
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("pbias:", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 6) {
                    continue;
                }
                try {
                    const int signal_id = std::stoi(extra.substr(6, equal_pos - 6));
                    const double bias_m = std::stod(extra.substr(equal_pos + 1));
                    if (signal_id >= 0 && signal_id <= 255 && std::isfinite(bias_m)) {
                        correction.phase_bias_m[static_cast<uint8_t>(signal_id)] = bias_m;
                    }
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("atmos_", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 6) {
                    continue;
                }
                correction.atmos_tokens[extra.substr(0, equal_pos)] = extra.substr(equal_pos + 1);
            }
        }
        correction.code_bias_valid = !correction.code_bias_m.empty();
        correction.phase_bias_valid = !correction.phase_bias_m.empty();
        correction.atmos_valid = !correction.atmos_tokens.empty();
        addCorrection(correction);
        loaded_any = true;
    }

    return loaded_any;
}

bool SSRProducts::hasData(const SatelliteId& sat, const GNSSTime& time) const {
    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    return interpolateCorrection(sat, time, orbit_correction, clock_correction_m);
}

void SSRProducts::clear() {
    orbit_clock_corrections.clear();
}

} // namespace libgnss
