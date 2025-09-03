#pragma once

#include <vector>
#include <map>
#include <memory>
#include "types.hpp"

namespace libgnss {

/**
 * @brief Single observation measurement
 */
struct Observation {
    SatelliteId satellite;
    SignalType signal;
    
    double pseudorange = 0.0;       ///< Pseudorange in meters
    double carrier_phase = 0.0;     ///< Carrier phase in cycles
    double doppler = 0.0;           ///< Doppler frequency in Hz
    double snr = 0.0;               ///< Signal-to-noise ratio in dB-Hz
    
    // Quality indicators
    uint8_t lli = 0;                ///< Loss of lock indicator
    uint8_t code = 0;               ///< Code indicator
    bool valid = true;              ///< Observation validity flag
    
    // Corrections
    double ionosphere_delay = 0.0;  ///< Ionospheric delay correction
    double troposphere_delay = 0.0; ///< Tropospheric delay correction
    double antenna_pco = 0.0;       ///< Antenna phase center offset
    double relativity = 0.0;        ///< Relativistic correction
    
    Observation() = default;
    Observation(const SatelliteId& sat, SignalType sig) 
        : satellite(sat), signal(sig) {}
};

/**
 * @brief Collection of observations for a single epoch
 */
class ObservationData {
public:
    GNSSTime time;
    Vector3d receiver_position;     ///< Approximate receiver position (ECEF)
    double receiver_clock_bias = 0.0;
    
    std::vector<Observation> observations;
    
    ObservationData() = default;
    ObservationData(const GNSSTime& t) : time(t) {}
    
    /**
     * @brief Add observation
     */
    void addObservation(const Observation& obs) {
        observations.push_back(obs);
    }
    
    /**
     * @brief Get observations for specific satellite
     */
    std::vector<Observation> getObservations(const SatelliteId& sat) const;
    
    /**
     * @brief Get observations for specific GNSS system
     */
    std::vector<Observation> getObservations(GNSSSystem system) const;
    
    /**
     * @brief Get observations for specific signal type
     */
    std::vector<Observation> getObservations(SignalType signal) const;
    
    /**
     * @brief Filter observations by elevation angle
     */
    std::vector<Observation> filterByElevation(double min_elevation, 
                                             const Vector3d& receiver_pos) const;
    
    /**
     * @brief Filter observations by SNR
     */
    std::vector<Observation> filterBySNR(double min_snr) const;
    
    /**
     * @brief Get number of satellites
     */
    size_t getNumSatellites() const;
    
    /**
     * @brief Get unique satellite list
     */
    std::vector<SatelliteId> getSatellites() const;
    
    /**
     * @brief Check if observation exists for satellite and signal
     */
    bool hasObservation(const SatelliteId& sat, SignalType signal) const;
    
    /**
     * @brief Get observation for satellite and signal
     */
    const Observation* getObservation(const SatelliteId& sat, SignalType signal) const;
    
    /**
     * @brief Remove observations that don't meet quality criteria
     */
    void applyQualityControl(double min_elevation = 15.0, 
                           double min_snr = 35.0,
                           const Vector3d& receiver_pos = Vector3d::Zero());
    
    /**
     * @brief Calculate geometry matrix (H matrix)
     */
    MatrixXd calculateGeometryMatrix(const Vector3d& receiver_pos,
                                   const std::vector<Vector3d>& sat_positions) const;
    
    /**
     * @brief Calculate dilution of precision values
     */
    struct DOPValues {
        double gdop = 999.9;    ///< Geometric DOP
        double pdop = 999.9;    ///< Position DOP
        double hdop = 999.9;    ///< Horizontal DOP
        double vdop = 999.9;    ///< Vertical DOP
        double tdop = 999.9;    ///< Time DOP
    };
    
    DOPValues calculateDOP(const Vector3d& receiver_pos,
                          const std::vector<Vector3d>& sat_positions) const;
    
    /**
     * @brief Clear all observations
     */
    void clear() {
        observations.clear();
        receiver_position.setZero();
        receiver_clock_bias = 0.0;
    }
    
    /**
     * @brief Check if epoch has valid observations
     */
    bool isEmpty() const {
        return observations.empty();
    }
    
    /**
     * @brief Get statistics for this epoch
     */
    struct EpochStats {
        size_t total_observations = 0;
        size_t valid_observations = 0;
        size_t num_satellites = 0;
        size_t num_systems = 0;
        double average_snr = 0.0;
        double min_elevation = 90.0;
        double max_elevation = 0.0;
    };
    
    EpochStats getStats(const Vector3d& receiver_pos = Vector3d::Zero()) const;
};

/**
 * @brief Time series of observation data
 */
class ObservationSeries {
public:
    std::vector<ObservationData> epochs;
    
    /**
     * @brief Add epoch data
     */
    void addEpoch(const ObservationData& epoch) {
        epochs.push_back(epoch);
    }
    
    /**
     * @brief Get epoch by time
     */
    const ObservationData* getEpoch(const GNSSTime& time) const;
    
    /**
     * @brief Get epochs in time range
     */
    std::vector<ObservationData> getEpochs(const GNSSTime& start, 
                                         const GNSSTime& end) const;
    
    /**
     * @brief Sort epochs by time
     */
    void sortByTime();
    
    /**
     * @brief Get time span of observations
     */
    std::pair<GNSSTime, GNSSTime> getTimeSpan() const;
    
    /**
     * @brief Clear all data
     */
    void clear() {
        epochs.clear();
    }
    
    /**
     * @brief Check if series is empty
     */
    bool isEmpty() const {
        return epochs.empty();
    }
    
    /**
     * @brief Get number of epochs
     */
    size_t size() const {
        return epochs.size();
    }
};

} // namespace libgnss
