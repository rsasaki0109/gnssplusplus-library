#pragma once

#include "../core/processor.hpp"
#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"
#include <Eigen/Dense>

namespace libgnss {

/**
 * @brief Single Point Positioning (SPP) processor
 * 
 * Implements standard GNSS single point positioning using pseudorange
 * measurements from multiple satellites and constellations.
 */
class SPPProcessor : public ProcessorBase {
public:
    /**
     * @brief SPP-specific configuration
     */
    struct SPPConfig {
        double position_convergence_threshold = 1e-4;  ///< Position convergence threshold in meters
        int max_iterations = 10;                       ///< Maximum iterations for position solution
        double pseudorange_sigma = 3.0;               ///< Pseudorange measurement noise (1-sigma) in meters
        bool use_weighted_least_squares = true;       ///< Use elevation-dependent weighting
        bool estimate_receiver_clock = true;          ///< Estimate receiver clock bias
        bool apply_atmospheric_corrections = true;    ///< Apply ionosphere/troposphere corrections
        bool use_multi_constellation = true;          ///< Use multiple GNSS constellations
        
        // Outlier detection
        bool enable_outlier_detection = true;
        double outlier_threshold_sigma = 3.0;         ///< Outlier detection threshold
        
        // Clock modeling
        bool model_intersystem_bias = true;           ///< Model inter-system clock biases
    };
    
    SPPProcessor();
    explicit SPPProcessor(const SPPConfig& spp_config);
    ~SPPProcessor() override = default;
    
    // ProcessorBase interface
    bool initialize(const ProcessorConfig& config) override;
    PositionSolution processEpoch(const ObservationData& obs, const NavigationData& nav) override;
    ProcessorStats getStats() const override;
    void reset() override;
    
    /**
     * @brief Set SPP-specific configuration
     */
    void setSPPConfig(const SPPConfig& config) { spp_config_ = config; }
    
    /**
     * @brief Get SPP configuration
     */
    const SPPConfig& getSPPConfig() const { return spp_config_; }

private:
    SPPConfig spp_config_;
    
    // State variables
    Vector3d estimated_position_;           ///< Current position estimate (ECEF)
    double receiver_clock_bias_;            ///< Receiver clock bias in seconds
    std::map<GNSSSystem, double> system_biases_; ///< Inter-system clock biases
    
    // Statistics
    mutable std::mutex stats_mutex_;
    size_t total_epochs_processed_ = 0;
    size_t successful_solutions_ = 0;
    double total_processing_time_ms_ = 0.0;
    
    /**
     * @brief Solve position using weighted least squares
     */
    PositionSolution solvePosition(const std::vector<Observation>& valid_obs,
                                 const NavigationData& nav,
                                 const GNSSTime& time);
    
    /**
     * @brief Calculate predicted pseudorange
     */
    double calculatePredictedRange(const Vector3d& receiver_pos,
                                 const Vector3d& satellite_pos,
                                 double receiver_clock_bias,
                                 double satellite_clock_bias) const;
    
    /**
     * @brief Calculate geometry matrix (design matrix)
     */
    MatrixXd calculateGeometryMatrix(const Vector3d& receiver_pos,
                                   const std::vector<Vector3d>& satellite_positions) const;
    
    /**
     * @brief Calculate weight matrix based on elevation angles
     */
    MatrixXd calculateWeightMatrix(const std::vector<double>& elevations,
                                 const std::vector<double>& snr_values) const;
    
    /**
     * @brief Apply atmospheric corrections to observations
     */
    void applyAtmosphericCorrections(std::vector<Observation>& observations,
                                   const NavigationData& nav,
                                   const Vector3d& receiver_pos,
                                   const GNSSTime& time) const;
    
    /**
     * @brief Detect and remove outlier observations
     */
    std::vector<Observation> detectOutliers(const std::vector<Observation>& observations,
                                          const VectorXd& residuals,
                                          double threshold_sigma) const;
    
    /**
     * @brief Calculate dilution of precision values
     */
    void calculateDOP(const MatrixXd& geometry_matrix, PositionSolution& solution) const;
    
    /**
     * @brief Validate observations for SPP processing
     */
    std::vector<Observation> validateObservations(const ObservationData& obs,
                                                const NavigationData& nav,
                                                const GNSSTime& time) const;
    
    /**
     * @brief Initialize position estimate
     */
    bool initializePosition(const std::vector<Observation>& observations,
                          const NavigationData& nav,
                          const GNSSTime& time);
    
    /**
     * @brief Calculate satellite positions and clock corrections
     */
    struct SatelliteState {
        Vector3d position;
        Vector3d velocity;
        double clock_bias;
        double clock_drift;
        bool valid;
    };
    
    std::map<SatelliteId, SatelliteState> calculateSatelliteStates(
        const std::vector<Observation>& observations,
        const NavigationData& nav,
        const GNSSTime& time) const;
    
    /**
     * @brief Apply relativistic corrections
     */
    double calculateRelativisticCorrection(const Vector3d& satellite_pos,
                                         const Vector3d& satellite_vel) const;
    
    /**
     * @brief Calculate Earth rotation correction
     */
    Vector3d applyEarthRotationCorrection(const Vector3d& satellite_pos,
                                        double signal_travel_time) const;
    
    /**
     * @brief Estimate inter-system clock biases
     */
    void estimateInterSystemBiases(const std::vector<Observation>& observations,
                                 const std::map<SatelliteId, SatelliteState>& sat_states,
                                 const Vector3d& receiver_pos);
    
    /**
     * @brief Calculate covariance matrix
     */
    Matrix3d calculatePositionCovariance(const MatrixXd& geometry_matrix,
                                       const MatrixXd& weight_matrix,
                                       double measurement_variance) const;
    
    /**
     * @brief Update processing statistics
     */
    void updateStatistics(double processing_time_ms, bool success) const;
};

/**
 * @brief SPP utility functions
 */
namespace spp_utils {
    
    /**
     * @brief Calculate elevation angle
     */
    double calculateElevation(const Vector3d& receiver_pos, const Vector3d& satellite_pos);
    
    /**
     * @brief Calculate azimuth angle
     */
    double calculateAzimuth(const Vector3d& receiver_pos, const Vector3d& satellite_pos);
    
    /**
     * @brief Convert ECEF to geodetic coordinates
     */
    GeodeticCoord ecefToGeodetic(const Vector3d& ecef_pos);
    
    /**
     * @brief Convert geodetic to ECEF coordinates
     */
    Vector3d geodeticToEcef(const GeodeticCoord& geodetic_pos);
    
    /**
     * @brief Calculate geometric distance
     */
    double calculateGeometricDistance(const Vector3d& receiver_pos, const Vector3d& satellite_pos);
    
    /**
     * @brief Apply elevation-dependent weighting
     */
    double calculateElevationWeight(double elevation_rad, double min_elevation_rad = 0.0);
    
    /**
     * @brief Apply SNR-dependent weighting
     */
    double calculateSNRWeight(double snr_db, double min_snr_db = 35.0);
    
    /**
     * @brief Check satellite visibility
     */
    bool isSatelliteVisible(const Vector3d& receiver_pos, 
                          const Vector3d& satellite_pos,
                          double min_elevation_rad);
}

} // namespace libgnss
