#pragma once

#include <cstdint>
#include <string>
#include <chrono>
#include <Eigen/Dense>

namespace libgnss {

/**
 * @brief GNSS system identifiers
 */
enum class GNSSSystem : uint8_t {
    GPS     = 0x01,
    GLONASS = 0x02,
    Galileo = 0x04,
    BeiDou  = 0x08,
    QZSS    = 0x10,
    SBAS    = 0x20,
    NavIC   = 0x40
};

/**
 * @brief Signal types for different GNSS systems
 */
enum class SignalType : uint8_t {
    // GPS
    GPS_L1CA = 0,
    GPS_L1P,
    GPS_L2P,
    GPS_L2C,
    GPS_L5,
    
    // GLONASS
    GLO_L1CA,
    GLO_L1P,
    GLO_L2CA,
    GLO_L2P,
    
    // Galileo
    GAL_E1,
    GAL_E5A,
    GAL_E5B,
    GAL_E6,
    
    // BeiDou
    BDS_B1I,
    BDS_B2I,
    BDS_B3I,
    BDS_B1C,
    BDS_B2A,
    
    // QZSS
    QZS_L1CA,
    QZS_L2C,
    QZS_L5,
    
    SIGNAL_TYPE_COUNT
};

/**
 * @brief Positioning modes
 */
enum class PositioningMode {
    SPP,        ///< Single Point Positioning
    DGPS,       ///< Differential GPS
    RTK_FLOAT,  ///< RTK Float solution
    RTK_FIXED,  ///< RTK Fixed solution
    PPP,        ///< Precise Point Positioning
    PPP_AR      ///< PPP with Ambiguity Resolution
};

/**
 * @brief Solution status
 */
enum class SolutionStatus {
    NONE,       ///< No solution
    SPP,        ///< Single point positioning
    DGPS,       ///< DGPS solution
    FLOAT,      ///< RTK float solution
    FIXED,      ///< RTK fixed solution
    PPP_FLOAT,  ///< PPP float solution
    PPP_FIXED   ///< PPP fixed solution
};

/**
 * @brief Coordinate systems
 */
enum class CoordinateSystem {
    WGS84,      ///< WGS84 geodetic coordinates
    ECEF,       ///< Earth-Centered Earth-Fixed
    ENU,        ///< East-North-Up local coordinates
    UTM         ///< Universal Transverse Mercator
};

/**
 * @brief Time system
 */
struct GNSSTime {
    int week;           ///< GPS week number
    double tow;         ///< Time of week in seconds
    
    GNSSTime() : week(0), tow(0.0) {}
    GNSSTime(int w, double t) : week(w), tow(t) {}
    
    /**
     * @brief Convert to system time
     */
    std::chrono::system_clock::time_point toSystemTime() const;
    
    /**
     * @brief Create from system time
     */
    static GNSSTime fromSystemTime(const std::chrono::system_clock::time_point& tp);
    
    /**
     * @brief Time difference in seconds
     */
    double operator-(const GNSSTime& other) const {
        return (week - other.week) * 604800.0 + (tow - other.tow);
    }
    
    bool operator<(const GNSSTime& other) const {
        return week < other.week || (week == other.week && tow < other.tow);
    }
    
    bool operator==(const GNSSTime& other) const {
        return week == other.week && std::abs(tow - other.tow) < 1e-6;
    }
};

/**
 * @brief Satellite identifier
 */
struct SatelliteId {
    GNSSSystem system;
    uint8_t prn;        ///< Pseudo-random number
    
    SatelliteId() : system(GNSSSystem::GPS), prn(0) {}
    SatelliteId(GNSSSystem sys, uint8_t p) : system(sys), prn(p) {}
    
    std::string toString() const;
    
    bool operator<(const SatelliteId& other) const {
        return system < other.system || (system == other.system && prn < other.prn);
    }
    
    bool operator==(const SatelliteId& other) const {
        return system == other.system && prn == other.prn;
    }
};

/**
 * @brief 3D position vector
 */
using Vector3d = Eigen::Vector3d;

/**
 * @brief 3x3 covariance matrix
 */
using Matrix3d = Eigen::Matrix3d;

/**
 * @brief Dynamic matrix
 */
using MatrixXd = Eigen::MatrixXd;

/**
 * @brief Dynamic vector
 */
using VectorXd = Eigen::VectorXd;

/**
 * @brief Geodetic coordinates
 */
struct GeodeticCoord {
    double latitude;    ///< Latitude in radians
    double longitude;   ///< Longitude in radians
    double height;      ///< Ellipsoidal height in meters
    
    GeodeticCoord() : latitude(0.0), longitude(0.0), height(0.0) {}
    GeodeticCoord(double lat, double lon, double h) 
        : latitude(lat), longitude(lon), height(h) {}
};

/**
 * @brief ECEF coordinates
 */
struct ECEFCoord {
    double x, y, z;     ///< ECEF coordinates in meters
    
    ECEFCoord() : x(0.0), y(0.0), z(0.0) {}
    ECEFCoord(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    Vector3d toVector() const { return Vector3d(x, y, z); }
    void fromVector(const Vector3d& v) { x = v(0); y = v(1); z = v(2); }
};

/**
 * @brief Constants
 */
namespace constants {
    constexpr double SPEED_OF_LIGHT = 299792458.0;     ///< Speed of light in m/s
    constexpr double GPS_L1_FREQ = 1575.42e6;          ///< GPS L1 frequency in Hz
    constexpr double GPS_L2_FREQ = 1227.60e6;          ///< GPS L2 frequency in Hz
    constexpr double GPS_L5_FREQ = 1176.45e6;          ///< GPS L5 frequency in Hz
    constexpr double WGS84_A = 6378137.0;              ///< WGS84 semi-major axis
    constexpr double WGS84_F = 1.0/298.257223563;      ///< WGS84 flattening
    constexpr double WGS84_E2 = 2*WGS84_F - WGS84_F*WGS84_F; ///< WGS84 eccentricity squared
}

} // namespace libgnss
