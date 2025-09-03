#pragma once

#include <vector>
#include <map>
#include <memory>
#include "types.hpp"

namespace libgnss {

/**
 * @brief Satellite ephemeris data
 */
struct Ephemeris {
    SatelliteId satellite;
    GNSSTime toe;           ///< Time of ephemeris
    GNSSTime toc;           ///< Time of clock
    
    // Orbital elements
    double sqrt_a;          ///< Square root of semi-major axis
    double e;               ///< Eccentricity
    double i0;              ///< Inclination at reference time
    double omega0;          ///< Right ascension of ascending node
    double omega;           ///< Argument of perigee
    double m0;              ///< Mean anomaly at reference time
    double delta_n;         ///< Mean motion difference
    double idot;            ///< Rate of inclination angle
    double omega_dot;       ///< Rate of right ascension
    
    // Perturbations
    double cuc, cus;        ///< Cosine/sine terms for argument of latitude
    double crc, crs;        ///< Cosine/sine terms for orbital radius
    double cic, cis;        ///< Cosine/sine terms for inclination
    
    // Clock parameters
    double af0, af1, af2;   ///< Clock bias, drift, drift rate
    double tgd;             ///< Group delay
    
    // Status and accuracy
    uint16_t week;          ///< GPS week number
    uint8_t health;         ///< Satellite health
    uint8_t ura;            ///< User range accuracy index
    uint16_t iodc;          ///< Issue of data clock
    uint16_t iode;          ///< Issue of data ephemeris
    
    bool valid = false;     ///< Ephemeris validity flag
    
    /**
     * @brief Calculate satellite position and velocity
     * @param time Time for calculation
     * @param pos Output satellite position (ECEF)
     * @param vel Output satellite velocity (ECEF)
     * @param clock_bias Output satellite clock bias
     * @param clock_drift Output satellite clock drift
     * @return true if calculation successful
     */
    bool calculateSatelliteState(const GNSSTime& time,
                               Vector3d& pos,
                               Vector3d& vel,
                               double& clock_bias,
                               double& clock_drift) const;
    
    /**
     * @brief Check if ephemeris is valid for given time
     */
    bool isValid(const GNSSTime& time) const;
    
    /**
     * @brief Get age of ephemeris data
     */
    double getAge(const GNSSTime& time) const;
};

/**
 * @brief Ionospheric model parameters
 */
struct IonosphereModel {
    // Klobuchar model parameters
    double alpha[4] = {0};  ///< Alpha coefficients
    double beta[4] = {0};   ///< Beta coefficients
    
    // NeQuick model parameters (Galileo)
    double ai[3] = {0};     ///< Effective ionization level coefficients
    
    bool valid = false;
    
    /**
     * @brief Calculate ionospheric delay
     * @param time GPS time
     * @param user_pos User position (geodetic)
     * @param sat_pos Satellite position (ECEF)
     * @param frequency Signal frequency in Hz
     * @return Ionospheric delay in meters
     */
    double calculateDelay(const GNSSTime& time,
                        const GeodeticCoord& user_pos,
                        const Vector3d& sat_pos,
                        double frequency) const;
};

/**
 * @brief Tropospheric model parameters
 */
struct TroposphereModel {
    enum class ModelType {
        SAASTAMOINEN,
        HOPFIELD,
        NEILL,
        GMF
    };
    
    ModelType type = ModelType::SAASTAMOINEN;
    
    /**
     * @brief Calculate tropospheric delay
     * @param user_pos User position (geodetic)
     * @param elevation Satellite elevation angle in radians
     * @param time Time (for seasonal variations)
     * @return Tropospheric delay in meters
     */
    double calculateDelay(const GeodeticCoord& user_pos,
                        double elevation,
                        const GNSSTime& time = GNSSTime()) const;
};

/**
 * @brief Navigation data collection
 */
class NavigationData {
public:
    std::map<SatelliteId, std::vector<Ephemeris>> ephemeris_data;
    IonosphereModel ionosphere_model;
    TroposphereModel troposphere_model;
    
    /**
     * @brief Add ephemeris data
     */
    void addEphemeris(const Ephemeris& eph);
    
    /**
     * @brief Get best ephemeris for satellite at given time
     */
    const Ephemeris* getEphemeris(const SatelliteId& sat, const GNSSTime& time) const;
    
    /**
     * @brief Get all ephemeris for satellite
     */
    std::vector<Ephemeris> getEphemeris(const SatelliteId& sat) const;
    
    /**
     * @brief Calculate satellite position and clock
     */
    bool calculateSatelliteState(const SatelliteId& sat,
                               const GNSSTime& time,
                               Vector3d& position,
                               Vector3d& velocity,
                               double& clock_bias,
                               double& clock_drift) const;
    
    /**
     * @brief Calculate satellite positions for multiple satellites
     */
    std::map<SatelliteId, Vector3d> calculateSatellitePositions(
        const std::vector<SatelliteId>& satellites,
        const GNSSTime& time) const;
    
    /**
     * @brief Calculate elevation and azimuth angles
     */
    struct SatelliteGeometry {
        double elevation;   ///< Elevation angle in radians
        double azimuth;     ///< Azimuth angle in radians
        double distance;    ///< Geometric distance in meters
    };
    
    SatelliteGeometry calculateGeometry(const Vector3d& receiver_pos,
                                      const Vector3d& satellite_pos) const;
    
    /**
     * @brief Apply atmospheric corrections
     */
    struct AtmosphericCorrections {
        double ionosphere_delay = 0.0;
        double troposphere_delay = 0.0;
        double total_delay = 0.0;
    };
    
    AtmosphericCorrections calculateAtmosphericCorrections(
        const GeodeticCoord& receiver_pos,
        const Vector3d& satellite_pos,
        const GNSSTime& time,
        double frequency) const;
    
    /**
     * @brief Check if navigation data is available for satellite
     */
    bool hasEphemeris(const SatelliteId& sat, const GNSSTime& time) const;
    
    /**
     * @brief Get list of satellites with valid ephemeris
     */
    std::vector<SatelliteId> getAvailableSatellites(const GNSSTime& time) const;
    
    /**
     * @brief Remove old ephemeris data
     */
    void cleanupOldData(const GNSSTime& current_time, double max_age_hours = 4.0);
    
    /**
     * @brief Clear all navigation data
     */
    void clear();
    
    /**
     * @brief Check if navigation data is empty
     */
    bool isEmpty() const;
    
    /**
     * @brief Get statistics
     */
    struct NavigationStats {
        size_t total_ephemeris = 0;
        size_t valid_ephemeris = 0;
        size_t num_satellites = 0;
        std::map<GNSSSystem, size_t> satellites_per_system;
        GNSSTime oldest_ephemeris;
        GNSSTime newest_ephemeris;
    };
    
    NavigationStats getStats(const GNSSTime& current_time) const;
};

/**
 * @brief Precise orbit and clock data
 */
struct PreciseOrbitClock {
    SatelliteId satellite;
    GNSSTime time;
    
    Vector3d position;      ///< Precise position (ECEF)
    Vector3d velocity;      ///< Precise velocity (ECEF)
    double clock_bias;      ///< Precise clock bias
    double clock_drift;     ///< Precise clock drift
    
    // Accuracy indicators
    double position_sigma;  ///< Position accuracy (1-sigma)
    double clock_sigma;     ///< Clock accuracy (1-sigma)
    
    bool position_valid = false;
    bool clock_valid = false;
};

/**
 * @brief Precise products manager
 */
class PreciseProducts {
public:
    std::map<SatelliteId, std::vector<PreciseOrbitClock>> orbit_clock_data;
    
    /**
     * @brief Add precise orbit/clock data
     */
    void addOrbitClock(const PreciseOrbitClock& data);
    
    /**
     * @brief Interpolate precise orbit and clock
     */
    bool interpolateOrbitClock(const SatelliteId& sat,
                             const GNSSTime& time,
                             Vector3d& position,
                             Vector3d& velocity,
                             double& clock_bias,
                             double& clock_drift) const;
    
    /**
     * @brief Load SP3 orbit file
     */
    bool loadSP3File(const std::string& filename);
    
    /**
     * @brief Load precise clock file
     */
    bool loadClockFile(const std::string& filename);
    
    /**
     * @brief Check data availability
     */
    bool hasData(const SatelliteId& sat, const GNSSTime& time) const;
    
    /**
     * @brief Clear all data
     */
    void clear();
};

} // namespace libgnss
