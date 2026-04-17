#pragma once

#include <vector>
#include <map>
#include <memory>
#include <string>
#include "types.hpp"

namespace libgnss {

/**
 * @brief Satellite ephemeris data
 */
struct Ephemeris {
    Ephemeris() : satellite(), toe(), toc(), tof(), toes(0), sqrt_a(0), e(0), i0(0), omega0(0), omega(0), m0(0),
                  delta_n(0), idot(0), omega_dot(0), cuc(0), cus(0), crc(0), crs(0),
                  cic(0), cis(0), af0(0), af1(0), af2(0), tgd(0), tgd_secondary(0),
                  glonass_taun(0), glonass_gamn(0), glonass_frequency_channel(0),
                  week(0), health(0),
                  ura(0), iodc(0), iode(0), valid(false) {}

    SatelliteId satellite;
    GNSSTime toe;           ///< Time of ephemeris
    GNSSTime toc;           ///< Time of clock
    GNSSTime tof;           ///< Time of frame / transmission
    double toes;            ///< Raw broadcast toe seconds within system week
    
    // Orbital elements
    double sqrt_a;          ///< Square root of semi-major axis
    double e;               ///< Eccentricity
    double i0;              ///< Inclination at reference time
    double omega0;          ///< Right ascension of ascending node
    double omega;           ///< Argument of perigee
    double m0;              ///< Mean anomaly at reference time
    double delta_n;         ///< Mean motion difference
    double idot;            ///< Rate of inclination angle
    double i_dot;           ///< Rate of inclination angle (alias)
    double omega_dot;       ///< Rate of right ascension
    
    // Perturbations
    double cuc, cus;        ///< Cosine/sine terms for argument of latitude
    double crc, crs;        ///< Cosine/sine terms for orbital radius
    double cic, cis;        ///< Cosine/sine terms for inclination
    
    // Clock parameters
    double af0, af1, af2;   ///< Clock bias, drift, drift rate
    double tgd;             ///< Primary group delay / bias parameter
    double tgd_secondary;   ///< Secondary group delay / bias parameter
    double glonass_taun;    ///< GLONASS -tau_n clock bias term
    double glonass_gamn;    ///< GLONASS gamma_n relative frequency bias

    // GLONASS state vector broadcast parameters
    Vector3d glonass_position = Vector3d::Zero();
    Vector3d glonass_velocity = Vector3d::Zero();
    Vector3d glonass_acceleration = Vector3d::Zero();
    int glonass_frequency_channel = 0;
    int glonass_age = 0;
    
    // Status and accuracy
    uint16_t week;          ///< GPS week number
    uint8_t health;         ///< Satellite health
    double sv_health;       ///< Satellite health (double format)
    double sv_accuracy;     ///< Satellite accuracy
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

    NavigationData();
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

/**
 * @brief SSR orbit/clock correction sample
 */
struct SSROrbitClockCorrection {
    SatelliteId satellite;
    GNSSTime time;

    Vector3d orbit_correction_ecef = Vector3d::Zero();   ///< Orbit delta in meters (ECEF or RAC per container flag)
    double clock_correction_m = 0.0;                     ///< Clock delta in meters
    double ura_sigma_m = 0.0;                            ///< SSR URA sigma in meters
    int iode = -1;                                       ///< Broadcast ephemeris IODE/IODnav for SSR orbit
    std::map<uint8_t, double> code_bias_m;               ///< SSR code biases keyed by RTCM signal id
    std::map<uint8_t, double> phase_bias_m;              ///< SSR phase biases keyed by RTCM signal id
    int bias_network_id = 0;                             ///< Optional CLAS bias network id (0 when unset)
    int atmos_network_id = 0;                            ///< Optional CLAS atmosphere network id (0 when unset)
    std::map<std::string, std::string> atmos_tokens;     ///< Optional atmospheric metadata tokens

    bool orbit_valid = false;
    bool clock_valid = false;
    bool ura_valid = false;
    bool code_bias_valid = false;
    bool phase_bias_valid = false;
    bool atmos_valid = false;
};

/**
 * @brief Minimal SSR correction manager
 *
 * This stores orbit/clock corrections keyed by satellite and epoch and can
 * linearly interpolate them. The current loader accepts a simple CSV format:
 * `week,tow,sat,dx,dy,dz,dclock_m[,ura_sigma_m=<m>][,cbias:<id>=<m>...][,pbias:<id>=<m>...][,bias_network_id=<n>][,atmos_<name>=<value>...]`
 */
class SSRProducts {
public:
    std::map<SatelliteId, std::vector<SSROrbitClockCorrection>> orbit_clock_corrections;

    void addCorrection(const SSROrbitClockCorrection& correction);

    bool interpolateCorrection(const SatelliteId& sat,
                               const GNSSTime& time,
                               Vector3d& orbit_correction_ecef,
                               double& clock_correction_m,
                               double* ura_sigma_m = nullptr,
                               std::map<uint8_t, double>* code_bias_m = nullptr,
                               std::map<uint8_t, double>* phase_bias_m = nullptr,
                               std::map<std::string, std::string>* atmos_tokens = nullptr,
                               GNSSTime* atmos_reference_time = nullptr,
                               GNSSTime* phase_bias_reference_time = nullptr,
                               GNSSTime* clock_reference_time = nullptr,
                               int preferred_network_id = 0) const;

    bool loadCSVFile(const std::string& filename);

    bool hasData(const SatelliteId& sat, const GNSSTime& time) const;

    bool orbitCorrectionsAreRac() const { return orbit_corrections_are_rac_; }
    void setOrbitCorrectionsAreRac(bool enabled) { orbit_corrections_are_rac_ = enabled; }

    void clear();

private:
    bool orbit_corrections_are_rac_ = false;
};

/**
 * @brief IONEX latitude row stored as a longitude grid.
 */
struct IONEXLatitudeRow {
    double latitude_deg = 0.0;
    double longitude_start_deg = 0.0;
    double longitude_end_deg = 0.0;
    double longitude_step_deg = 0.0;
    double height_km = 0.0;
    std::vector<double> values_tecu;
};

/**
 * @brief Single IONEX TEC/RMS map at an epoch.
 */
struct IONEXMap {
    GNSSTime time;
    std::vector<IONEXLatitudeRow> rows;
};

/**
 * @brief Minimal IONEX product manager.
 *
 * This loader stores TEC and RMS maps and can interpolate a vertical TEC value
 * at a latitude/longitude/time sample. The current implementation is intended
 * as a core product container and future PPP hook point.
 */
class IONEXProducts {
public:
    std::string version;
    std::string system;
    int interval_s = 0;
    int map_dimension = 0;
    int exponent = 0;
    double base_radius_km = 0.0;
    double elevation_cutoff_deg = 0.0;
    std::string mapping_function;
    std::vector<double> latitude_grid;
    std::vector<double> longitude_grid;
    std::vector<double> height_grid;
    int auxiliary_dcb_entries = 0;
    std::vector<IONEXMap> tec_maps;
    std::vector<IONEXMap> rms_maps;

    bool loadIONEXFile(const std::string& filename);

    bool interpolateTecu(const GNSSTime& time,
                         double latitude_deg,
                         double longitude_deg,
                         double& tecu,
                         double* rms_tecu = nullptr) const;

    bool hasData(const GNSSTime& time) const;

    void clear();
};

/**
 * @brief Minimal differential code bias product entry.
 */
struct DCBEntry {
    std::string bias_type;
    SatelliteId satellite;
    std::string observation_1;
    std::string observation_2;
    std::string unit;
    double bias = 0.0;
    double sigma = 0.0;
    bool valid = false;
};

/**
 * @brief Minimal Bias-SINEX / IONEX auxiliary DCB product manager.
 */
class DCBProducts {
public:
    std::vector<DCBEntry> entries;

    bool loadFile(const std::string& filename);

    bool getBias(const SatelliteId& sat,
                 const std::string& bias_type,
                 const std::string& observation_1,
                 const std::string& observation_2,
                 double& bias,
                 double* sigma = nullptr) const;

    void clear();
};

} // namespace libgnss
