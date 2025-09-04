#pragma once

#include "../core/processor.hpp"
#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"
#include "spp.hpp"
#include <Eigen/Dense>

namespace libgnss {

/**
 * @brief Precise Point Positioning (PPP) processor
 * 
 * Implements PPP using precise orbits and clocks for decimeter to
 * centimeter level positioning without base stations.
 */
class PPPProcessor : public ProcessorBase {
public:
    /**
     * @brief PPP-specific configuration
     */
    struct PPPConfig {
        // Precise products
        bool use_precise_orbits = true;
        bool use_precise_clocks = true;
        std::string orbit_file_path;
        std::string clock_file_path;
        
        // Ambiguity resolution
        bool enable_ambiguity_resolution = false;
        double ar_ratio_threshold = 3.0;
        int min_satellites_for_ar = 6;
        
        // Kalman filter parameters
        double process_noise_position = 0.01;     ///< Position process noise (m²/s)
        double process_noise_velocity = 0.1;      ///< Velocity process noise (m²/s³)
        double process_noise_clock = 100.0;       ///< Clock process noise (m²/s)
        double process_noise_troposphere = 1e-6;  ///< Troposphere process noise (m²/s)
        double process_noise_ambiguity = 1e-8;    ///< Ambiguity process noise (cycles²/s)
        
        // Measurement noise
        double pseudorange_sigma = 1.0;           ///< Pseudorange measurement noise (m)
        double carrier_phase_sigma = 0.01;        ///< Carrier phase measurement noise (m)
        
        // Atmospheric modeling
        bool estimate_troposphere = true;
        bool use_ionosphere_free = true;
        bool apply_ocean_loading = false;
        bool apply_solid_earth_tides = true;
        bool apply_relativity = true;
        
        // Convergence criteria
        double convergence_threshold_horizontal = 0.1;  ///< Horizontal convergence threshold (m)
        double convergence_threshold_vertical = 0.2;    ///< Vertical convergence threshold (m)
        int convergence_min_epochs = 20;               ///< Minimum epochs for convergence check
        
        // Quality control
        bool enable_outlier_detection = true;
        double outlier_threshold = 3.0;
        bool enable_cycle_slip_detection = true;
        double cycle_slip_threshold = 0.05;
    };
    
    PPPProcessor();
    explicit PPPProcessor(const PPPConfig& ppp_config);
    ~PPPProcessor() override = default;
    
    // ProcessorBase interface
    bool initialize(const ProcessorConfig& config) override;
    PositionSolution processEpoch(const ObservationData& obs, const NavigationData& nav) override;
    ProcessorStats getStats() const override;
    void reset() override;
    
    /**
     * @brief Set PPP configuration
     */
    void setPPPConfig(const PPPConfig& config) { ppp_config_ = config; }
    
    /**
     * @brief Get PPP configuration
     */
    const PPPConfig& getPPPConfig() const { return ppp_config_; }
    
    /**
     * @brief Load precise products
     */
    bool loadPreciseProducts(const std::string& orbit_file, const std::string& clock_file);
    
    /**
     * @brief Check convergence status
     */
    bool hasConverged() const { return converged_; }
    
    /**
     * @brief Get convergence time
     */
    double getConvergenceTime() const { return convergence_time_; }

private:
    PPPConfig ppp_config_;
    SPPProcessor spp_processor_;  ///< Fallback SPP processor
    PreciseProducts precise_products_;
    
    // Kalman filter state
    struct PPPState {
        VectorXd state;           ///< State vector [position, velocity, clock, troposphere, ambiguities]
        MatrixXd covariance;      ///< State covariance matrix
        
        // State indices
        int pos_index = 0;        ///< Position state start index
        int vel_index = 3;        ///< Velocity state start index
        int clock_index = 6;      ///< Clock state start index
        int trop_index = 7;       ///< Troposphere state start index
        int amb_index = 8;        ///< Ambiguity state start index
        
        std::map<SatelliteId, std::map<SignalType, int>> ambiguity_indices;
        int total_states = 8;     ///< Total number of states
    };
    
    PPPState filter_state_;
    bool filter_initialized_ = false;
    
    // Convergence monitoring
    bool converged_ = false;
    double convergence_time_ = 0.0;
    GNSSTime convergence_start_time_;
    std::vector<Vector3d> recent_positions_;
    
    // Ambiguity tracking
    struct PPPAmbiguityInfo {
        double float_value = 0.0;
        double fixed_value = 0.0;
        bool is_fixed = false;
        int lock_count = 0;
        double last_phase = 0.0;
        GNSSTime last_time;
        double quality_indicator = 0.0;
    };
    
    std::map<SatelliteId, std::map<SignalType, PPPAmbiguityInfo>> ambiguity_states_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    size_t total_epochs_processed_ = 0;
    size_t converged_solutions_ = 0;
    double total_convergence_time_ = 0.0;
    
    /**
     * @brief Initialize Kalman filter
     */
    bool initializeFilter(const ObservationData& obs, const NavigationData& nav);
    
    /**
     * @brief Predict filter state
     */
    void predictState(double dt);
    
    /**
     * @brief Update filter with measurements
     */
    bool updateFilter(const ObservationData& obs, const NavigationData& nav);
    
    /**
     * @brief Form ionosphere-free combinations
     */
    struct IonosphereFreeObs {
        SatelliteId satellite;
        double pseudorange_if = 0.0;
        double carrier_phase_if = 0.0;
        double variance_pr = 0.0;
        double variance_cp = 0.0;
        bool valid = false;
    };
    
    std::vector<IonosphereFreeObs> formIonosphereFree(const ObservationData& obs);
    
    /**
     * @brief Apply precise corrections
     */
    void applyPreciseCorrections(std::vector<IonosphereFreeObs>& observations,
                               const NavigationData& nav,
                               const GNSSTime& time);
    
    /**
     * @brief Detect cycle slips
     */
    void detectCycleSlips(const ObservationData& obs);
    
    /**
     * @brief Resolve PPP ambiguities
     */
    bool resolveAmbiguities();
    
    /**
     * @brief Calculate tropospheric delay
     */
    double calculateTroposphericDelay(const Vector3d& receiver_pos,
                                    const Vector3d& satellite_pos,
                                    double zenith_delay) const;
    
    /**
     * @brief Calculate tropospheric mapping function
     */
    double calculateMappingFunction(double elevation) const;
    
    /**
     * @brief Apply geophysical corrections
     */
    Vector3d applyGeophysicalCorrections(const Vector3d& position,
                                       const GNSSTime& time) const;
    
    /**
     * @brief Calculate solid Earth tides
     */
    Vector3d calculateSolidEarthTides(const Vector3d& position,
                                    const GNSSTime& time) const;
    
    /**
     * @brief Calculate ocean loading
     */
    Vector3d calculateOceanLoading(const Vector3d& position,
                                 const GNSSTime& time) const;
    
    /**
     * @brief Form measurement equations
     */
    struct MeasurementEquation {
        MatrixXd design_matrix;
        VectorXd observations;
        VectorXd predicted;
        MatrixXd weight_matrix;
        VectorXd residuals;
    };
    
    MeasurementEquation formMeasurementEquations(
        const std::vector<IonosphereFreeObs>& observations,
        const NavigationData& nav,
        const GNSSTime& time);
    
    /**
     * @brief Check convergence
     */
    void checkConvergence(const GNSSTime& current_time);
    
    /**
     * @brief Generate PPP solution
     */
    PositionSolution generateSolution(const GNSSTime& time,
                                    const std::vector<IonosphereFreeObs>& observations);
    
    /**
     * @brief Calculate position accuracy
     */
    Vector3d calculatePositionAccuracy() const;
    
    /**
     * @brief Update ambiguity states
     */
    void updateAmbiguityStates(const ObservationData& obs);
    
    /**
     * @brief Reset ambiguity for satellite
     */
    void resetAmbiguity(const SatelliteId& satellite, SignalType signal);
    
    /**
     * @brief Calculate measurement residuals
     */
    VectorXd calculateResiduals(const std::vector<IonosphereFreeObs>& observations,
                              const NavigationData& nav,
                              const GNSSTime& time) const;
    
    /**
     * @brief Update processing statistics
     */
    void updateStatistics(bool converged) const;
};

/**
 * @brief PPP utility functions
 */
namespace ppp_utils {
    
    /**
     * @brief Calculate ionosphere-free combination coefficients
     */
    std::pair<double, double> getIonosphereFreeCoefficients(double f1, double f2);
    
    /**
     * @brief Calculate Melbourne-Wubbena combination
     */
    double calculateMelbourneWubbena(double l1_phase, double l2_phase,
                                   double p1_range, double p2_range,
                                   double f1, double f2);
    
    /**
     * @brief Calculate geometry-free combination
     */
    double calculateGeometryFree(double l1_phase, double l2_phase);
    
    /**
     * @brief Interpolate precise orbits
     */
    bool interpolatePreciseOrbit(const std::vector<PreciseOrbitClock>& orbit_data,
                               const GNSSTime& time,
                               Vector3d& position,
                               Vector3d& velocity);
    
    /**
     * @brief Interpolate precise clocks
     */
    bool interpolatePreciseClock(const std::vector<PreciseOrbitClock>& clock_data,
                               const GNSSTime& time,
                               double& clock_bias,
                               double& clock_drift);
    
    /**
     * @brief Calculate satellite antenna phase center offset
     */
    Vector3d calculateSatelliteAntennaPCO(const SatelliteId& satellite,
                                        const Vector3d& satellite_pos,
                                        const Vector3d& sun_pos);
    
    /**
     * @brief Calculate receiver antenna phase center offset
     */
    Vector3d calculateReceiverAntennaPCO(const Vector3d& receiver_pos,
                                       const Vector3d& satellite_pos,
                                       const std::string& antenna_type);
    
    /**
     * @brief Calculate phase windup correction
     */
    double calculatePhaseWindup(const Vector3d& receiver_pos,
                              const Vector3d& satellite_pos,
                              double previous_windup);
}

} // namespace libgnss
