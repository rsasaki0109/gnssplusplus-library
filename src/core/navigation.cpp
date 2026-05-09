#include <libgnss++/core/navigation.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <limits>
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

int parsePositiveIntToken(const std::map<std::string, std::string>& tokens,
                          const std::string& key) {
    const auto it = tokens.find(key);
    if (it == tokens.end()) {
        return 0;
    }
    try {
        const int value = std::stoi(it->second);
        return value > 0 ? value : 0;
    } catch (const std::exception&) {
        return 0;
    }
}

bool sameCorrectionVariant(const SSROrbitClockCorrection& lhs,
                           const SSROrbitClockCorrection& rhs) {
    return lhs.atmos_network_id == rhs.atmos_network_id &&
           lhs.bias_network_id == rhs.bias_network_id;
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

bool parseIonexEpochLine(const std::string& line, GNSSTime& time) {
    const std::string epoch_text = trimCopy(line.substr(0, std::min<size_t>(43U, line.size())));
    if (epoch_text.empty()) {
        return false;
    }
    std::istringstream stream(epoch_text);
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

bool parseFixedFieldDoubles(const std::string& line,
                            size_t end_column,
                            std::vector<double>& values) {
    values.clear();
    std::istringstream stream(line.substr(0, std::min(end_column, line.size())));
    double value = 0.0;
    while (stream >> value) {
        values.push_back(value);
    }
    return !values.empty();
}

double normalizeLongitudeToGrid(double lon_deg, double lon_start_deg, double lon_end_deg) {
    double normalized = lon_deg;
    const double min_lon = std::min(lon_start_deg, lon_end_deg);
    const double max_lon = std::max(lon_start_deg, lon_end_deg);
    while (normalized < min_lon - 1e-9) {
        normalized += 360.0;
    }
    while (normalized > max_lon + 1e-9) {
        normalized -= 360.0;
    }
    return normalized;
}

bool interpolateIonexRow(const IONEXLatitudeRow& row, double longitude_deg, double& tecu) {
    if (row.values_tecu.empty() || std::abs(row.longitude_step_deg) < 1e-12) {
        return false;
    }
    const double normalized_lon =
        normalizeLongitudeToGrid(longitude_deg, row.longitude_start_deg, row.longitude_end_deg);
    const double index = (normalized_lon - row.longitude_start_deg) / row.longitude_step_deg;
    if (index < -1e-9 || index > static_cast<double>(row.values_tecu.size() - 1U) + 1e-9) {
        return false;
    }

    const double clamped = std::max(0.0, std::min(index, static_cast<double>(row.values_tecu.size() - 1U)));
    const size_t left = static_cast<size_t>(std::floor(clamped));
    const size_t right = std::min(left + 1U, row.values_tecu.size() - 1U);
    const double alpha = clamped - static_cast<double>(left);
    const double left_value = row.values_tecu[left];
    const double right_value = row.values_tecu[right];
    if (!std::isfinite(left_value) || !std::isfinite(right_value)) {
        return false;
    }
    tecu = left_value + alpha * (right_value - left_value);
    return true;
}

bool interpolateIonexMap(const IONEXMap& map,
                         double latitude_deg,
                         double longitude_deg,
                         double& tecu) {
    if (map.rows.empty()) {
        return false;
    }

    const IONEXLatitudeRow* before = nullptr;
    const IONEXLatitudeRow* after = nullptr;
    for (const auto& row : map.rows) {
        if (row.latitude_deg <= latitude_deg) {
            if (before == nullptr || row.latitude_deg > before->latitude_deg) {
                before = &row;
            }
        }
        if (row.latitude_deg >= latitude_deg) {
            if (after == nullptr || row.latitude_deg < after->latitude_deg) {
                after = &row;
            }
        }
    }
    if (before == nullptr) {
        before = &map.rows.front();
    }
    if (after == nullptr) {
        after = &map.rows.back();
    }

    double before_tecu = 0.0;
    double after_tecu = 0.0;
    if (before == after) {
        return interpolateIonexRow(*before, longitude_deg, tecu);
    }
    if (!interpolateIonexRow(*before, longitude_deg, before_tecu) ||
        !interpolateIonexRow(*after, longitude_deg, after_tecu)) {
        return false;
    }

    const double lat_delta = after->latitude_deg - before->latitude_deg;
    if (std::abs(lat_delta) < 1e-12) {
        tecu = before_tecu;
        return true;
    }
    const double alpha = (latitude_deg - before->latitude_deg) / lat_delta;
    tecu = before_tecu + alpha * (after_tecu - before_tecu);
    return true;
}

template <typename EntryType>
const EntryType* findEntryAtOrBefore(const std::vector<EntryType>& entries, const GNSSTime& time) {
    auto upper = std::lower_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const EntryType& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    if (upper == entries.end()) {
        return entries.empty() ? nullptr : &entries.back();
    }
    if (upper->time == time) {
        return &(*upper);
    }
    if (upper == entries.begin()) {
        return nullptr;
    }
    return &(*(upper - 1));
}

template <typename EntryType>
const EntryType* findEntryAtOrAfter(const std::vector<EntryType>& entries, const GNSSTime& time) {
    auto lower = std::lower_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const EntryType& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    if (lower == entries.end()) {
        return nullptr;
    }
    return &(*lower);
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
    auto lower = std::lower_bound(
        entries.begin(),
        entries.end(),
        correction.time,
        [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    auto upper = std::upper_bound(
        lower,
        entries.end(),
        correction.time,
        [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
            return lhs < rhs.time;
        });

    for (auto it = lower; it != upper; ++it) {
        if (!sameCorrectionVariant(*it, correction)) {
            continue;
        }
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
            for (const auto& [signal_id, bias_m] : correction.code_bias_m) {
                it->code_bias_m[signal_id] = bias_m;
            }
            it->code_bias_valid = !it->code_bias_m.empty();
        }
        if (correction.phase_bias_valid) {
            for (const auto& [signal_id, bias_m] : correction.phase_bias_m) {
                it->phase_bias_m[signal_id] = bias_m;
            }
            it->phase_bias_valid = !it->phase_bias_m.empty();
        }
        if (correction.atmos_valid) {
            for (const auto& [token, value] : correction.atmos_tokens) {
                it->atmos_tokens[token] = value;
            }
            it->atmos_valid = !it->atmos_tokens.empty();
        }
        return;
    }

    entries.insert(upper, correction);
}

bool SSRProducts::interpolateCorrection(const SatelliteId& sat,
                                        const GNSSTime& time,
                                        Vector3d& orbit_correction_ecef,
                                        double& clock_correction_m,
                                        double* ura_sigma_m,
                                        std::map<uint8_t, double>* code_bias_m,
                                        std::map<uint8_t, double>* phase_bias_m,
                                        std::map<std::string, std::string>* atmos_tokens,
                                        GNSSTime* atmos_reference_time,
                                        GNSSTime* phase_bias_reference_time,
                                        GNSSTime* clock_reference_time,
                                        int preferred_network_id) const {
    const auto sat_it = orbit_clock_corrections.find(sat);
    if (sat_it == orbit_clock_corrections.end() || sat_it->second.empty()) {
        return false;
    }
    orbit_correction_ecef.setZero();
    clock_correction_m = 0.0;
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
    if (atmos_reference_time != nullptr) {
        *atmos_reference_time = GNSSTime();
    }
    if (phase_bias_reference_time != nullptr) {
        *phase_bias_reference_time = GNSSTime();
    }
    if (clock_reference_time != nullptr) {
        *clock_reference_time = GNSSTime();
    }

    const auto& entries = sat_it->second;
    auto lower = std::lower_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    auto upper = std::upper_bound(
        lower,
        entries.end(),
        time,
        [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
            return lhs < rhs.time;
        });

    const auto biasScore = [&](const SSROrbitClockCorrection& entry, bool phase) -> int {
        const bool valid =
            phase
                ? (entry.phase_bias_valid && !entry.phase_bias_m.empty())
                : (entry.code_bias_valid && !entry.code_bias_m.empty());
        if (!valid) {
            return -1;
        }
        const size_t signal_count = phase ? entry.phase_bias_m.size() : entry.code_bias_m.size();
        if (preferred_network_id > 0) {
            if (entry.bias_network_id == preferred_network_id) {
                return 300000 + static_cast<int>(signal_count);
            }
            if (entry.bias_network_id == 0) {
                return 200000 + static_cast<int>(signal_count);
            }
            return 100000 + static_cast<int>(signal_count);
        }
        if (entry.bias_network_id == 0) {
            return 200000 + static_cast<int>(signal_count);
        }
        return 100000 + static_cast<int>(signal_count);
    };

    const auto atmosScore = [&](const SSROrbitClockCorrection& entry) -> int {
        if (!entry.atmos_valid || entry.atmos_tokens.empty()) {
            return -1;
        }
        int score = static_cast<int>(entry.atmos_tokens.size());
        if (entry.atmos_network_id > 0) {
            score += 100000;
        }
        if (preferred_network_id > 0) {
            if (entry.atmos_network_id == preferred_network_id) {
                score += 200000;
            } else if (entry.atmos_network_id == 0) {
                score += 50000;
            }
        }
        return score;
    };

    if (lower != entries.end() && lower->time == time) {
        const size_t lower_index = static_cast<size_t>(lower - entries.begin());
        size_t exact_begin = lower_index;
        while (exact_begin > 0 && entries[exact_begin - 1].time == time) {
            --exact_begin;
        }
        size_t exact_end = lower_index;
        while (exact_end < entries.size() && entries[exact_end].time == time) {
            ++exact_end;
        }

        const auto findRecent = [&](const auto& picker, size_t scan_end_index)
            -> const SSROrbitClockCorrection* {
            size_t group_end = scan_end_index;
            while (group_end > 0) {
                const size_t group_last = group_end - 1;
                const GNSSTime group_time = entries[group_last].time;
                if (std::abs(group_time - time) > kPreciseInterpolationGapSeconds) {
                    break;
                }
                size_t group_begin = group_last;
                while (group_begin > 0 && entries[group_begin - 1].time == group_time) {
                    --group_begin;
                }
                if (const SSROrbitClockCorrection* picked = picker(group_begin, group_end)) {
                    return picked;
                }
                group_end = group_begin;
            }
            return nullptr;
        };

        const auto pickOrbit = [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
            for (size_t index = begin; index < end; ++index) {
                if (entries[index].orbit_valid) {
                    return &entries[index];
                }
            }
            return nullptr;
        };
        const auto pickClock = [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
            for (size_t index = begin; index < end; ++index) {
                if (entries[index].clock_valid) {
                    return &entries[index];
                }
            }
            return nullptr;
        };
        const auto pickUra = [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
            for (size_t index = begin; index < end; ++index) {
                if (entries[index].ura_valid) {
                    return &entries[index];
                }
            }
            return nullptr;
        };
        const auto pickAtmos = [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
            const SSROrbitClockCorrection* best = nullptr;
            int best_score = -1;
            for (size_t index = begin; index < end; ++index) {
                const int score = atmosScore(entries[index]);
                if (score > best_score) {
                    best_score = score;
                    best = &entries[index];
                }
            }
            return best;
        };

        const SSROrbitClockCorrection* atmos_source = findRecent(pickAtmos, exact_end);
        int selected_network_id = preferred_network_id;
        if (selected_network_id <= 0 && atmos_source != nullptr && atmos_source->atmos_network_id > 0) {
            selected_network_id = atmos_source->atmos_network_id;
        }
        const auto pickBias = [&](bool phase) {
            return [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
                const SSROrbitClockCorrection* best = nullptr;
                int best_score = -1;
                for (size_t index = begin; index < end; ++index) {
                    const SSROrbitClockCorrection& entry = entries[index];
                    const bool valid =
                        phase
                            ? (entry.phase_bias_valid && !entry.phase_bias_m.empty())
                            : (entry.code_bias_valid && !entry.code_bias_m.empty());
                    if (!valid) {
                        continue;
                    }
                    const size_t signal_count =
                        phase ? entry.phase_bias_m.size() : entry.code_bias_m.size();
                    int score = static_cast<int>(signal_count);
                    if (selected_network_id > 0) {
                        if (entry.bias_network_id == selected_network_id) {
                            score += 300000;
                        } else if (entry.bias_network_id == 0) {
                            score += 200000;
                        } else {
                            score += 100000;
                        }
                    } else if (entry.bias_network_id == 0) {
                        score += 200000;
                    } else {
                        score += 100000;
                    }
                    if (score > best_score) {
                        best_score = score;
                        best = &entry;
                    }
                }
                return best;
            };
        };

        const SSROrbitClockCorrection* orbit_source = findRecent(pickOrbit, exact_end);
        const SSROrbitClockCorrection* clock_source = findRecent(pickClock, exact_end);
        const SSROrbitClockCorrection* ura_source = findRecent(pickUra, exact_end);
        const SSROrbitClockCorrection* code_source =
            code_bias_m != nullptr ? findRecent(pickBias(false), exact_end) : nullptr;
        const SSROrbitClockCorrection* phase_source =
            phase_bias_m != nullptr ? findRecent(pickBias(true), exact_end) : nullptr;

        if (orbit_source != nullptr && orbit_source->orbit_valid) {
            orbit_correction_ecef = orbit_source->orbit_correction_ecef;
        }
        if (clock_source != nullptr && clock_source->clock_valid) {
            clock_correction_m = clock_source->clock_correction_m;
            if (clock_reference_time != nullptr) {
                *clock_reference_time = clock_source->time;
            }
        }
        if (ura_sigma_m != nullptr && ura_source != nullptr && ura_source->ura_valid) {
            *ura_sigma_m = ura_source->ura_sigma_m;
        }
        if (code_bias_m != nullptr && code_source != nullptr && code_source->code_bias_valid) {
            *code_bias_m = code_source->code_bias_m;
        }
        if (phase_bias_m != nullptr && phase_source != nullptr && phase_source->phase_bias_valid) {
            *phase_bias_m = phase_source->phase_bias_m;
            if (phase_bias_reference_time != nullptr) {
                *phase_bias_reference_time = phase_source->time;
            }
        }
        if (atmos_tokens != nullptr && atmos_source != nullptr && atmos_source->atmos_valid) {
            *atmos_tokens = atmos_source->atmos_tokens;
            if (atmos_reference_time != nullptr) {
                *atmos_reference_time = atmos_source->time;
            }
        }
        return orbit_source != nullptr || clock_source != nullptr || ura_source != nullptr ||
               code_source != nullptr || phase_source != nullptr || atmos_source != nullptr;
    }

    const SSROrbitClockCorrection* before = nullptr;
    const SSROrbitClockCorrection* after = nullptr;
    if (lower != entries.end()) {
        after = &(*lower);
        if (lower != entries.begin()) {
            before = &(*(lower - 1));
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
        // Bias forward-fill: CLAS broadcasts bias every 30s but clock every 5s.
        // The interpolation neighbours (before/after) are adjacent SSR entries
        // which are typically clock-only rows with no bias.  Scan backwards
        // through older entries to find the most recent bias, matching the
        // CLASLIB behaviour of holding the last received bias indefinitely.
        auto scanBackwardBias = [&](bool phase) -> const SSROrbitClockCorrection* {
            const SSROrbitClockCorrection* best = nullptr;
            int best_score = -1;
            for (auto scan = lower; scan != entries.begin(); ) {
                --scan;
                if (std::abs(scan->time - time) > kPreciseInterpolationGapSeconds) break;
                const int score = biasScore(*scan, phase);
                if (score > best_score) {
                    best_score = score;
                    best = &(*scan);
                }
            }
            return best;
        };
        if (code_bias_m != nullptr) {
            if (biasScore(*before, false) >= biasScore(*after, false) &&
                before->code_bias_valid && !before->code_bias_m.empty()) {
                *code_bias_m = before->code_bias_m;
            } else if (after->code_bias_valid && !after->code_bias_m.empty()) {
                *code_bias_m = after->code_bias_m;
            } else if (const auto* scanned = scanBackwardBias(false)) {
                *code_bias_m = scanned->code_bias_m;
            }
        }
        if (phase_bias_m != nullptr) {
            if (biasScore(*before, true) >= biasScore(*after, true) &&
                before->phase_bias_valid && !before->phase_bias_m.empty()) {
                *phase_bias_m = before->phase_bias_m;
                if (phase_bias_reference_time != nullptr) {
                    *phase_bias_reference_time = before->time;
                }
            } else if (after->phase_bias_valid && !after->phase_bias_m.empty()) {
                *phase_bias_m = after->phase_bias_m;
                if (phase_bias_reference_time != nullptr) {
                    *phase_bias_reference_time = after->time;
                }
            } else if (const auto* scanned = scanBackwardBias(true)) {
                *phase_bias_m = scanned->phase_bias_m;
                if (phase_bias_reference_time != nullptr) {
                    *phase_bias_reference_time = scanned->time;
                }
            }
        }
        if (clock_reference_time != nullptr) {
            if (after->clock_valid) {
                *clock_reference_time = after->time;
            } else if (before->clock_valid) {
                *clock_reference_time = before->time;
            }
        }
        if (atmos_tokens != nullptr) {
            if (atmosScore(*before) >= atmosScore(*after) &&
                before->atmos_valid && !before->atmos_tokens.empty()) {
                *atmos_tokens = before->atmos_tokens;
                if (atmos_reference_time != nullptr) {
                    *atmos_reference_time = before->time;
                }
            } else if (after->atmos_valid && !after->atmos_tokens.empty()) {
                *atmos_tokens = after->atmos_tokens;
                if (atmos_reference_time != nullptr) {
                    *atmos_reference_time = after->time;
                }
            } else {
                // Neither interpolation neighbour has atmos — scan backwards.
                auto scan = lower;
                while (scan != entries.begin()) {
                    --scan;
                    if (scan->atmos_valid && !scan->atmos_tokens.empty() &&
                        std::abs(scan->time - time) <= kPreciseInterpolationGapSeconds) {
                        *atmos_tokens = scan->atmos_tokens;
                        if (atmos_reference_time != nullptr) {
                            *atmos_reference_time = scan->time;
                        }
                        break;
                    }
                }
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

    // If the matched sample lacks orbit or clock, scan backwards for a
    // recent entry that has them.  This handles atmos-only rows that were
    // inserted at intervening TOWs without orbit/clock data.
    const SSROrbitClockCorrection* orbit_source = sample;
    const SSROrbitClockCorrection* clock_source = sample;
    if (!sample->orbit_valid || !sample->clock_valid) {
        auto scan = std::lower_bound(
            entries.begin(), entries.end(), sample->time,
            [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
                return lhs.time < rhs;
            });
        while (scan != entries.begin()) {
            --scan;
            if (std::abs(scan->time - time) > kPreciseInterpolationGapSeconds) break;
            if (!orbit_source->orbit_valid && scan->orbit_valid) orbit_source = &(*scan);
            if (!clock_source->clock_valid && scan->clock_valid) clock_source = &(*scan);
            if (orbit_source->orbit_valid && clock_source->clock_valid) break;
        }
    }
    orbit_correction_ecef =
        orbit_source->orbit_valid ? orbit_source->orbit_correction_ecef : Vector3d::Zero();
    clock_correction_m = clock_source->clock_valid ? clock_source->clock_correction_m : 0.0;
    if (clock_reference_time != nullptr && clock_source->clock_valid) {
        *clock_reference_time = clock_source->time;
    }
    if (ura_sigma_m != nullptr) {
        *ura_sigma_m = sample->ura_valid ? sample->ura_sigma_m : 0.0;
    }
    if (code_bias_m != nullptr) {
        if (sample->code_bias_valid && !sample->code_bias_m.empty()) {
            *code_bias_m = sample->code_bias_m;
        } else if (orbit_source != sample && orbit_source->code_bias_valid && !orbit_source->code_bias_m.empty()) {
            *code_bias_m = orbit_source->code_bias_m;
        }
    }
    if (phase_bias_m != nullptr) {
        if (sample->phase_bias_valid && !sample->phase_bias_m.empty()) {
            *phase_bias_m = sample->phase_bias_m;
            if (phase_bias_reference_time != nullptr) {
                *phase_bias_reference_time = sample->time;
            }
        } else if (orbit_source != sample && orbit_source->phase_bias_valid &&
                   !orbit_source->phase_bias_m.empty()) {
            *phase_bias_m = orbit_source->phase_bias_m;
            if (phase_bias_reference_time != nullptr) {
                *phase_bias_reference_time = orbit_source->time;
            }
        }
    }
    if (atmos_tokens != nullptr) {
        if (sample->atmos_valid) {
            *atmos_tokens = sample->atmos_tokens;
            if (atmos_reference_time != nullptr) {
                *atmos_reference_time = sample->time;
            }
        } else {
            // The exact-match sample has no atmos — scan backwards for the
            // nearest entry that does.  CLAS atmosphere messages arrive at a
            // lower cadence than orbit/clock, so a recent neighbour is valid.
            auto it = std::lower_bound(
                entries.begin(), entries.end(), sample->time,
                [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
                    return lhs.time < rhs;
                });
            while (it != entries.begin()) {
                --it;
                if (it->atmos_valid && !it->atmos_tokens.empty() &&
                    std::abs(it->time - time) <= kPreciseInterpolationGapSeconds) {
                    *atmos_tokens = it->atmos_tokens;
                    if (atmos_reference_time != nullptr) {
                        *atmos_reference_time = it->time;
                    }
                    break;
                }
            }
        }
    }
    return sample->orbit_valid || sample->clock_valid || sample->ura_valid ||
           sample->code_bias_valid || sample->phase_bias_valid || sample->atmos_valid;
}

bool SSRProducts::loadCSVFile(const std::string& filename) {
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    orbit_corrections_are_rac_ = true;
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
        // orbit is valid only if at least one component is non-zero
        correction.orbit_valid =
            std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz) &&
            (std::abs(dx) > 0.0 || std::abs(dy) > 0.0 || std::abs(dz) > 0.0);
        correction.clock_valid = std::isfinite(dclock_m) && std::abs(dclock_m) > 0.0;
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
            if (extra.rfind("bias_network_id=", 0) == 0) {
                try {
                    correction.bias_network_id = std::max(0, std::stoi(extra.substr(16)));
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("atmos_", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 6) {
                    continue;
                }
                const std::string key = extra.substr(0, equal_pos);
                const std::string value = extra.substr(equal_pos + 1);
                correction.atmos_tokens[key] = value;
                if (key == "atmos_network_id") {
                    try {
                        correction.atmos_network_id = std::max(0, std::stoi(value));
                    } catch (const std::exception&) {
                    }
                }
            }
        }
        if (correction.atmos_network_id <= 0 && !correction.atmos_tokens.empty()) {
            correction.atmos_network_id =
                parsePositiveIntToken(correction.atmos_tokens, "atmos_network_id");
        }
        correction.code_bias_valid = !correction.code_bias_m.empty();
        correction.phase_bias_valid = !correction.phase_bias_m.empty();
        correction.atmos_valid = !correction.atmos_tokens.empty();
        // Keep atmos-only rows. QZSS CLAS broadcasts network-wide atmosphere
        // corrections that may arrive without non-zero orbit/clock deltas, and
        // the PPP loader needs those rows to preserve the atmosphere tokens.
        addCorrection(correction);
        loaded_any = true;
    }

    // Debug: count atmos entries after loading
    if (std::getenv("GNSS_PPP_DEBUG") != nullptr) {
        size_t atmos_entries = 0;
        size_t total_entries = 0;
        for (const auto& [sat, entries] : orbit_clock_corrections) {
            for (const auto& e : entries) {
                ++total_entries;
                if (e.atmos_valid) ++atmos_entries;
            }
        }
        std::cerr << "[SSR-LOAD] total_entries=" << total_entries
                  << " atmos_entries=" << atmos_entries
                  << " satellites=" << orbit_clock_corrections.size() << "\n";
    }

    // Forward-fill orbit corrections: CLAS broadcasts orbit every 30s but
    // clock every 5s.  Carry the last valid orbit forward to clock-only epochs
    // so all epochs use SSR-corrected satellite positions.
    for (auto& [sat, entries] : orbit_clock_corrections) {
        (void)sat;
        Vector3d last_orbit = Vector3d::Zero();
        bool has_last_orbit = false;
        for (auto& entry : entries) {
            if (entry.orbit_valid) {
                last_orbit = entry.orbit_correction_ecef;
                has_last_orbit = true;
            } else if (has_last_orbit && entry.clock_valid) {
                entry.orbit_correction_ecef = last_orbit;
                entry.orbit_valid = true;
            }
        }
    }

    // Forward-fill atmosphere tokens: CLAS broadcasts atmos every 30s.
    // Carry the last valid atmos forward so all epochs have STEC/trop data.
    for (auto& [sat, entries] : orbit_clock_corrections) {
        (void)sat;
        std::map<std::string, std::string> last_atmos;
        int last_atmos_network_id = 0;
        bool has_last_atmos = false;
        for (auto& entry : entries) {
            if (entry.atmos_valid && !entry.atmos_tokens.empty()) {
                last_atmos = entry.atmos_tokens;
                last_atmos_network_id = entry.atmos_network_id;
                has_last_atmos = true;
            } else if (has_last_atmos && !entry.atmos_valid && entry.clock_valid) {
                entry.atmos_tokens = last_atmos;
                entry.atmos_network_id = last_atmos_network_id;
                entry.atmos_valid = true;
            }
        }
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
    orbit_corrections_are_rac_ = false;
}

bool IONEXProducts::loadIONEXFile(const std::string& filename) {
    clear();

    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_header = true;
    bool loaded_any = false;
    bool current_is_rms = false;
    IONEXMap current_map;
    std::string line;
    while (std::getline(input, line)) {
        if (in_header) {
            if (line.find("IONEX VERSION / TYPE") != std::string::npos) {
                const std::string version_text = trimCopy(line.substr(0, std::min<size_t>(20U, line.size())));
                if (!version_text.empty()) {
                    std::istringstream version_stream(version_text);
                    version_stream >> version;
                }
                const auto type_fields = trimCopy(line.substr(20, std::min<size_t>(40U, line.size() > 20 ? line.size() - 20U : 0U)));
                if (!type_fields.empty()) {
                    std::istringstream type_stream(type_fields);
                    std::string unused_type;
                    type_stream >> unused_type >> system;
                }
            } else if (line.find("INTERVAL") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    interval_s = static_cast<int>(std::llround(values.front()));
                }
            } else if (line.find("MAP DIMENSION") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    map_dimension = static_cast<int>(std::llround(values.front()));
                }
            } else if (line.find("BASE RADIUS") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    base_radius_km = values.front();
                }
            } else if (line.find("ELEVATION CUTOFF") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    elevation_cutoff_deg = values.front();
                }
            } else if (line.find("MAPPING FUNCTION") != std::string::npos) {
                mapping_function = trimCopy(line.substr(0, std::min<size_t>(20U, line.size())));
            } else if (line.find("EXPONENT") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 20U, values) && !values.empty()) {
                    exponent = static_cast<int>(std::llround(values.front()));
                }
            } else if (line.find("LAT1 / LAT2 / DLAT") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 30U, values) && values.size() >= 3U) {
                    latitude_grid = {values[0], values[1], values[2]};
                }
            } else if (line.find("LON1 / LON2 / DLON") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 30U, values) && values.size() >= 3U) {
                    longitude_grid = {values[0], values[1], values[2]};
                }
            } else if (line.find("HGT1 / HGT2 / DHGT") != std::string::npos) {
                std::vector<double> values;
                if (parseFixedFieldDoubles(line, 30U, values) && values.size() >= 3U) {
                    height_grid = {values[0], values[1], values[2]};
                }
            } else if (line.find("PRN / BIAS / RMS") != std::string::npos) {
                ++auxiliary_dcb_entries;
            } else if (line.find("END OF HEADER") != std::string::npos) {
                in_header = false;
            }
            continue;
        }

        if (line.find("START OF TEC MAP") != std::string::npos) {
            current_map = IONEXMap{};
            current_is_rms = false;
            continue;
        }
        if (line.find("START OF RMS MAP") != std::string::npos) {
            current_map = IONEXMap{};
            current_is_rms = true;
            continue;
        }
        if (line.find("END OF TEC MAP") != std::string::npos ||
            line.find("END OF RMS MAP") != std::string::npos) {
            if (!current_map.rows.empty()) {
                auto& target = current_is_rms ? rms_maps : tec_maps;
                target.push_back(current_map);
                loaded_any = true;
            }
            current_map = IONEXMap{};
            current_is_rms = false;
            continue;
        }
        if (line.find("EPOCH OF CURRENT MAP") != std::string::npos) {
            parseIonexEpochLine(line, current_map.time);
            continue;
        }
        if (line.find("LAT/LON1/LON2/DLON/H") == std::string::npos) {
            continue;
        }

        std::vector<double> row_header;
        if (!parseFixedFieldDoubles(line, 40U, row_header) || row_header.size() < 5U) {
            continue;
        }

        IONEXLatitudeRow row;
        row.latitude_deg = row_header[0];
        row.longitude_start_deg = row_header[1];
        row.longitude_end_deg = row_header[2];
        row.longitude_step_deg = row_header[3];
        row.height_km = row_header[4];

        if (std::abs(row.longitude_step_deg) < 1e-12) {
            continue;
        }

        const int longitude_count = static_cast<int>(
            std::llround((row.longitude_end_deg - row.longitude_start_deg) / row.longitude_step_deg)) + 1;
        if (longitude_count <= 0) {
            continue;
        }

        while (static_cast<int>(row.values_tecu.size()) < longitude_count && std::getline(input, line)) {
            if (line.find("END OF TEC MAP") != std::string::npos ||
                line.find("END OF RMS MAP") != std::string::npos ||
                line.find("LAT/LON1/LON2/DLON/H") != std::string::npos) {
                break;
            }
            // IONEX data values are 5-char wide (I5 in the spec) and
            // packed up to 16 values per 80-character line. The
            // previous 60-char window dropped the last 4 values on
            // every line, leaving every TEC row 16-values short and
            // every map empty — silently rejecting CODE / IGS final
            // IONEX. Use the full line.
            std::istringstream value_stream(line);
            double raw_value = 0.0;
            while (value_stream >> raw_value) {
                if (std::abs(raw_value - 9999.0) < 1e-6) {
                    row.values_tecu.push_back(std::numeric_limits<double>::quiet_NaN());
                } else {
                    row.values_tecu.push_back(raw_value * std::pow(10.0, exponent));
                }
                if (static_cast<int>(row.values_tecu.size()) >= longitude_count) {
                    break;
                }
            }
        }
        if (static_cast<int>(row.values_tecu.size()) == longitude_count) {
            current_map.rows.push_back(std::move(row));
        }
    }

    auto sort_by_time = [](std::vector<IONEXMap>& maps) {
        std::sort(
            maps.begin(),
            maps.end(),
            [](const IONEXMap& lhs, const IONEXMap& rhs) {
                return lhs.time < rhs.time;
            });
    };
    sort_by_time(tec_maps);
    sort_by_time(rms_maps);
    return loaded_any;
}

bool IONEXProducts::interpolateTecu(const GNSSTime& time,
                                    double latitude_deg,
                                    double longitude_deg,
                                    double& tecu,
                                    double* rms_tecu) const {
    const auto interpolate_map_series =
        [&](const std::vector<IONEXMap>& maps, double& output_value) -> bool {
            if (maps.empty()) {
                return false;
            }

            const IONEXMap* before = findEntryAtOrBefore(maps, time);
            const IONEXMap* after = findEntryAtOrAfter(maps, time);
            if (before == nullptr && after == nullptr) {
                return false;
            }
            if (before != nullptr && after != nullptr && before != after) {
                double left = 0.0;
                double right = 0.0;
                if (!interpolateIonexMap(*before, latitude_deg, longitude_deg, left) ||
                    !interpolateIonexMap(*after, latitude_deg, longitude_deg, right)) {
                    return false;
                }
                const double dt_total = after->time - before->time;
                if (std::abs(dt_total) < 1e-9) {
                    output_value = left;
                    return true;
                }
                const double alpha = (time - before->time) / dt_total;
                output_value = left + alpha * (right - left);
                return true;
            }

            const IONEXMap* sample = before != nullptr ? before : after;
            return sample != nullptr && interpolateIonexMap(*sample, latitude_deg, longitude_deg, output_value);
        };

    if (!interpolate_map_series(tec_maps, tecu)) {
        return false;
    }
    if (rms_tecu != nullptr) {
        double rms_value = 0.0;
        if (interpolate_map_series(rms_maps, rms_value)) {
            *rms_tecu = rms_value;
        } else {
            *rms_tecu = 0.0;
        }
    }
    return true;
}

bool IONEXProducts::hasData(const GNSSTime& time) const {
    return findEntryAtOrBefore(tec_maps, time) != nullptr ||
           findEntryAtOrAfter(tec_maps, time) != nullptr;
}

void IONEXProducts::clear() {
    version.clear();
    system.clear();
    interval_s = 0;
    map_dimension = 0;
    exponent = 0;
    base_radius_km = 0.0;
    elevation_cutoff_deg = 0.0;
    mapping_function.clear();
    latitude_grid.clear();
    longitude_grid.clear();
    height_grid.clear();
    auxiliary_dcb_entries = 0;
    tec_maps.clear();
    rms_maps.clear();
}

bool DCBProducts::loadFile(const std::string& filename) {
    clear();

    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_bias_solution = false;
    bool loaded_any = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty()) {
            continue;
        }
        if (trimmed.rfind("+BIAS/SOLUTION", 0) == 0) {
            in_bias_solution = true;
            continue;
        }
        if (trimmed.rfind("-BIAS/SOLUTION", 0) == 0) {
            in_bias_solution = false;
            continue;
        }

        if (in_bias_solution) {
            if (trimmed[0] == '*') {
                continue;
            }
            std::vector<std::string> fields;
            std::istringstream stream(trimmed);
            std::string token;
            while (stream >> token) {
                fields.push_back(token);
            }
            if (fields.size() < 9U) {
                continue;
            }

            // Bias-SINEX rows are `BIAS SVN PRN [STATION] OBS1 OBS2
            // START END UNIT VALUE STDEV`. Some emitters omit the
            // optional STATION column entirely; some include it as
            // a 9-char field that whitespace-tokenization collapses
            // away. After collapsing, the PRN column may therefore
            // land at fields[1] (no SVN, e.g. CAS) or fields[2]
            // (BSX with SVN, e.g. GFZ/GBM). Pick whichever parses
            // as a valid PRN (3-char `<sys><digit><digit>`); SVNs
            // are 4 chars (`<sys><digit><digit><digit>`) so
            // length disambiguates without needing brittle field-
            // count heuristics.
            SatelliteId satellite;
            std::size_t obs_offset = 0;
            if (fields[1].size() == 3U &&
                parseSatelliteToken(fields[1], satellite)) {
                obs_offset = 2;
            } else if (fields[2].size() == 3U &&
                       parseSatelliteToken(fields[2], satellite)) {
                obs_offset = 3;
            } else {
                continue;
            }
            // Required columns (relative to obs_offset): OBS1,
            // OBS2, START, END, UNIT, VALUE, STDEV — 7 more.
            if (fields.size() < obs_offset + 7U) {
                continue;
            }

            try {
                DCBEntry entry;
                entry.bias_type = fields[0];
                entry.satellite = satellite;
                entry.observation_1 = fields[obs_offset];
                entry.observation_2 = fields[obs_offset + 1];
                entry.unit = fields[obs_offset + 4];
                entry.bias = std::stod(fields[obs_offset + 5]);
                entry.sigma = std::stod(fields[obs_offset + 6]);
                entry.valid = std::isfinite(entry.bias);
                entries.push_back(entry);
                loaded_any = true;
            } catch (const std::exception&) {
            }
            continue;
        }

        if (line.find("PRN / BIAS / RMS") != std::string::npos) {
            std::vector<double> values;
            std::vector<std::string> fields;
            std::istringstream stream(line.substr(0, std::min<size_t>(40U, line.size())));
            std::string token;
            while (stream >> token) {
                fields.push_back(token);
            }
            if (fields.size() < 3U) {
                continue;
            }
            SatelliteId satellite;
            if (!parseSatelliteToken(fields[0], satellite)) {
                continue;
            }
            try {
                DCBEntry entry;
                entry.bias_type = "DCB";
                entry.satellite = satellite;
                entry.unit = "ns";
                entry.bias = std::stod(fields[1]);
                entry.sigma = std::stod(fields[2]);
                entry.valid = std::isfinite(entry.bias);
                entries.push_back(entry);
                loaded_any = true;
            } catch (const std::exception&) {
            }
        }
    }

    return loaded_any;
}

bool DCBProducts::getBias(const SatelliteId& sat,
                          const std::string& bias_type,
                          const std::string& observation_1,
                          const std::string& observation_2,
                          double& bias,
                          double* sigma) const {
    for (const auto& entry : entries) {
        if (!entry.valid || !(entry.satellite == sat) || entry.bias_type != bias_type) {
            continue;
        }
        if (!observation_1.empty() && entry.observation_1 != observation_1) {
            continue;
        }
        if (!observation_2.empty() && entry.observation_2 != observation_2) {
            continue;
        }
        bias = entry.bias;
        if (sigma != nullptr) {
            *sigma = entry.sigma;
        }
        return true;
    }
    return false;
}

void DCBProducts::clear() {
    entries.clear();
}

} // namespace libgnss
