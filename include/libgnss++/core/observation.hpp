#pragma once

#include <vector>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <limits>
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
    // Optional source-domain Android raw measurement retained for
    // truth-free contract audits.  Estimators continue to consume only
    // `doppler` (the RINEX-convention Hz value) and never this diagnostic
    // field; non-Android/RINEX observations leave it unset.
    double pseudorange_rate_mps = 0.0; ///< Android PseudorangeRate [m/s]
    double source_carrier_frequency_hz = 0.0; ///< Android CarrierFrequencyHz
    double snr = 0.0;               ///< Signal-to-noise ratio in dB-Hz
    std::string pseudorange_observation_type;   ///< RINEX code observation type, e.g. C1C
    std::string carrier_phase_observation_type; ///< RINEX carrier observation type, e.g. L1C

    // Data availability flags
    bool has_pseudorange = false;   ///< Pseudorange data available
    bool has_carrier_phase = false; ///< Carrier phase data available
    bool has_doppler = false;       ///< Doppler data available
    bool has_pseudorange_rate_mps = false; ///< Source raw-rate diagnostic
    bool has_source_carrier_frequency_hz = false; ///< Source frequency diagnostic

    // Raw Android quality provenance retained for truth-free row-attrition
    // audits.  These fields are metadata only; estimator code continues to
    // consume the existing valid/has_* fields above.
    std::size_t raw_row_index = std::numeric_limits<std::size_t>::max();
    bool raw_snr_masked = false;
    bool raw_multipath_masked = false;
    bool raw_code_masked = false;
    bool raw_doppler_masked = false;

    // Quality indicators
    uint8_t lli = 0;                ///< Loss of lock indicator
    uint8_t code = 0;               ///< Code indicator
    bool valid = true;              ///< Observation validity flag
    bool loss_of_lock = false;      ///< Loss of lock flag
    int signal_strength = 0;        ///< Signal strength (0-9)
    bool has_glonass_frequency_channel = false; ///< GLONASS FCN available
    int glonass_frequency_channel = 0;          ///< GLONASS frequency channel (-7..+6)
    
    // Corrections
    double ionosphere_delay = 0.0;  ///< Ionospheric delay correction
    double troposphere_delay = 0.0; ///< Tropospheric delay correction
    double antenna_pco = 0.0;       ///< Antenna phase center offset
    double relativity = 0.0;        ///< Relativistic correction
    
    Observation() = default;
    Observation(const SatelliteId& sat, SignalType sig) 
        : satellite(sat), signal(sig) {}

    const std::string& exactBiasObservationType() const {
        return !pseudorange_observation_type.empty()
                   ? pseudorange_observation_type
                   : carrier_phase_observation_type;
    }
};

/**
 * @brief Collection of observations for a single epoch
 */
class ObservationData {
public:
    GNSSTime time;
    Vector3d receiver_position;     ///< Approximate receiver position (ECEF)
    double receiver_clock_bias = 0.0;
    // Optional raw Android receiver clock rate in range metres/second.  This
    // is populated only when the input explicitly carries
    // DriftNanosPerSecond; NaN means that no rate was available.  Keeping the
    // unit explicit prevents residual filters from estimating a rate from a
    // coordinate or silently treating a bias [s] as a drift [m/s].
    double receiver_clock_drift_mps =
        std::numeric_limits<double>::quiet_NaN();
    
    std::vector<Observation> observations;

    // Complete RINEX tracking-code observations, kept separately from the
    // policy-selected primary/secondary observations above.  A key such as
    // "2W" groups C2W/L2W/D2W/S2W for one satellite.  Normal positioning
    // paths intentionally ignore this collection; literal compatibility
    // paths can use it when an upstream implementation assigns a specific
    // RINEX tracking code to a frequency slot.
    std::map<std::pair<SatelliteId, std::string>, Observation>
        rinex_tracking_observations;
    std::map<std::pair<GNSSSystem, int>, std::string> rinex_frequency_slots;
    
    ObservationData() = default;
    ObservationData(const GNSSTime& t) : time(t) {}
    
    /**
     * @brief Add observation
     */
    void addObservation(const Observation& obs) {
        observations.push_back(obs);
    }

    void addRinexTrackingObservation(const std::string& tracking_code,
                                     const Observation& obs) {
        rinex_tracking_observations[{obs.satellite, tracking_code}] = obs;
    }

    const Observation* getRinexTrackingObservation(
        const SatelliteId& sat,
        const std::string& tracking_code) const {
        const auto it = rinex_tracking_observations.find({sat, tracking_code});
        return it == rinex_tracking_observations.end() ? nullptr : &it->second;
    }

    void setRinexFrequencySlot(GNSSSystem system,
                               int frequency_index,
                               const std::string& tracking_code) {
        rinex_frequency_slots[{system, frequency_index}] = tracking_code;
    }

    const std::string* getRinexFrequencySlot(GNSSSystem system,
                                             int frequency_index) const {
        const auto it = rinex_frequency_slots.find({system, frequency_index});
        return it == rinex_frequency_slots.end() ? nullptr : &it->second;
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
        rinex_tracking_observations.clear();
        rinex_frequency_slots.clear();
        receiver_position.setZero();
        receiver_clock_bias = 0.0;
        receiver_clock_drift_mps = std::numeric_limits<double>::quiet_NaN();
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
