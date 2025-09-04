#pragma once

#include "../core/processor.hpp"
#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"
#include "spp.hpp"
#include <Eigen/Dense>
#include <mutex>

namespace libgnss {

/**
 * @brief Real-Time Kinematic (RTK) processor
 * 
 * Implements RTK positioning using carrier phase measurements
 * with ambiguity resolution for centimeter-level accuracy.
 */
class RTKProcessor : public ProcessorBase {
public:
    /**
     * @brief RTK-specific configuration
     */
    struct RTKConfig {
        // Baseline constraints
        double max_baseline_length = 50000.0;      ///< Maximum baseline length in meters
        double min_baseline_length = 0.1;          ///< Minimum baseline length in meters
        
        // Ambiguity resolution
        enum class AmbiguityResolutionMode {
            OFF = 0,
            CONTINUOUS = 1,
            INSTANTANEOUS = 2,
            PARTIAL = 3
        };
        AmbiguityResolutionMode ar_mode = AmbiguityResolutionMode::CONTINUOUS;
        
        double ratio_threshold = 3.0;              ///< Ratio test threshold for ambiguity validation
        double ambiguity_ratio_threshold = 3.0;    ///< LAMBDA method ratio threshold
        double validation_threshold = 0.15;       ///< Chi-square test threshold
        int min_satellites_for_ar = 5;            ///< Minimum satellites for ambiguity resolution
        int min_lock_count = 5;                   ///< Minimum lock count for ambiguity candidate
        
        // Kalman filter parameters
        double process_noise_position = 0.01;     ///< Position process noise (m²/s)
        double process_noise_velocity = 0.1;      ///< Velocity process noise (m²/s³)
        double process_noise_ambiguity = 1e-6;    ///< Ambiguity process noise (cycles²/s)
        
        // Measurement noise
        double pseudorange_sigma = 3.0;           ///< Pseudorange measurement noise (m)
        double carrier_phase_sigma = 0.003;       ///< Carrier phase measurement noise (m)
        
        // Quality control
        bool enable_cycle_slip_detection = true;
        double cycle_slip_threshold = 0.05;       ///< Cycle slip detection threshold (cycles)
        bool enable_outlier_detection = true;
        double outlier_threshold = 3.0;           ///< Outlier detection threshold (sigma)
        
        // Processing options
        bool use_ionosphere_free_combination = true;
        bool estimate_troposphere = true;
        bool use_precise_orbits = false;
    };
    
    RTKProcessor();
    explicit RTKProcessor(const RTKConfig& rtk_config);
    ~RTKProcessor() override = default;
    
    // ProcessorBase interface
    bool initialize(const ProcessorConfig& config) override;
    PositionSolution processEpoch(const ObservationData& rover_obs, 
                                const NavigationData& nav) override;
    ProcessorStats getStats() const override;
    void reset() override;
    
    /**
     * @brief Process RTK epoch with base station data
     */
    PositionSolution processRTKEpoch(const ObservationData& rover_obs,
                                   const ObservationData& base_obs,
                                   const NavigationData& nav);
    
    /**
     * @brief Set base station position
     */
    void setBasePosition(const Vector3d& base_position) {
        base_position_ = base_position;
        base_position_known_ = true;
    }
    
    /**
     * @brief Set RTK configuration
     */
    void setRTKConfig(const RTKConfig& config) { rtk_config_ = config; }
    
    /**
     * @brief Get RTK configuration
     */
    const RTKConfig& getRTKConfig() const { return rtk_config_; }

private:
    RTKConfig rtk_config_;
    SPPProcessor spp_processor_;  ///< Fallback SPP processor
    
    // Base station information
    Vector3d base_position_;
    bool base_position_known_ = false;
    
    // Kalman filter state
    struct RTKState {
        VectorXd state;           ///< State vector [position, velocity, ambiguities]
        MatrixXd covariance;      ///< State covariance matrix
        std::map<SatelliteId, int> ambiguity_indices;  ///< Ambiguity parameter indices
        int num_position_states = 3;
        int num_velocity_states = 3;
        int num_ambiguity_states = 0;
    };
    
    RTKState filter_state_;
    bool filter_initialized_ = false;
    
    // Ambiguity tracking
    struct AmbiguityInfo {
        double float_value = 0.0;
        double fixed_value = 0.0;
        int lock_count = 0;
        bool is_fixed = false;
        double last_phase = 0.0;
        GNSSTime last_time;
    };
    
    std::map<SatelliteId, std::map<SignalType, AmbiguityInfo>> ambiguity_states_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    size_t total_epochs_processed_ = 0;
    size_t fixed_solutions_ = 0;
    size_t float_solutions_ = 0;
    
    /**
     * @brief Initialize Kalman filter
     */
    bool initializeFilter(const ObservationData& rover_obs,
                        const ObservationData& base_obs,
                        const NavigationData& nav);
    
    /**
     * @brief Predict filter state
     */
    void predictState(double dt);
    
    /**
     * @brief Update filter with measurements
     */
    bool updateFilter(const ObservationData& rover_obs,
                    const ObservationData& base_obs,
                    const NavigationData& nav);
    
    /**
     * @brief Form double differences
     */
    struct DoubleDifference {
        SatelliteId reference_satellite;
        SatelliteId satellite;
        SignalType signal;
        double pseudorange_dd = 0.0;
        double carrier_phase_dd = 0.0;
        double variance = 0.0;
        bool valid = false;
    };
    
    std::vector<DoubleDifference> formDoubleDifferences(
        const ObservationData& rover_obs,
        const ObservationData& base_obs,
        const NavigationData& nav);
    
    /**
     * @brief Detect cycle slips
     */
    void detectCycleSlips(const ObservationData& rover_obs,
                        const ObservationData& base_obs);
    
    /**
     * @brief Resolve integer ambiguities using LAMBDA method
     */
    bool resolveAmbiguities();
    
    /**
     * @brief LAMBDA method for integer ambiguity resolution
     * @param float_ambiguities Float ambiguity estimates
     * @param covariance_matrix Covariance matrix of float ambiguities
     * @param fixed_ambiguities Output fixed integer ambiguities
     * @param success_rate Success rate of ambiguity resolution
     * @return true if ambiguity resolution successful
     */
    bool lambdaMethod(const VectorXd& float_ambiguities,
                     const MatrixXd& covariance_matrix,
                     VectorXd& fixed_ambiguities,
                     double& success_rate);
    
    /**
     * @brief Z-transformation for decorrelation
     * @param Q Covariance matrix
     * @param Z Output Z-transformation matrix
     * @param L Output lower triangular matrix
     * @param D Output diagonal matrix
     */
    void zTransformation(const MatrixXd& Q, MatrixXd& Z, MatrixXd& L, VectorXd& D);
    
    /**
     * @brief Integer least squares search
     * @param a Float ambiguities in decorrelated space
     * @param L Lower triangular matrix
     * @param D Diagonal matrix
     * @param fixed_a Output fixed ambiguities
     * @param chi2 Chi-square test statistic
     * @return true if search successful
     */
    bool integerLeastSquares(const VectorXd& a, const MatrixXd& L, const VectorXd& D,
                           VectorXd& fixed_a, double& chi2);
    
    /**
     * @brief Validate ambiguity resolution using ratio test
     * @param chi2_1 Chi-square of best candidate
     * @param chi2_2 Chi-square of second best candidate
     * @return true if validation passed
     */
    bool validateAmbiguityResolution(double chi2_1, double chi2_2);
    
    /**
     * @brief LAMBDA method for ambiguity resolution
     */
    struct LAMBDAResult {
        VectorXd fixed_ambiguities;
        double ratio = 0.0;
        bool success = false;
    };
    
    LAMBDAResult solveLAMBDA(const VectorXd& float_ambiguities,
                           const MatrixXd& ambiguity_covariance);
    
    /**
     * @brief Validate ambiguity resolution
     */
    bool validateAmbiguityResolution(const VectorXd& fixed_ambiguities,
                                   const VectorXd& float_ambiguities,
                                   const MatrixXd& covariance,
                                   double ratio);
    
    /**
     * @brief Update ambiguity states
     */
    void updateAmbiguityStates(const std::vector<DoubleDifference>& double_diffs);
    
    /**
     * @brief Calculate baseline vector
     */
    Vector3d calculateBaseline() const;
    
    /**
     * @brief Generate RTK solution
     */
    PositionSolution generateSolution(const GNSSTime& time,
                                    SolutionStatus status,
                                    const std::vector<DoubleDifference>& measurements);
    
    /**
     * @brief Calculate measurement residuals
     */
    VectorXd calculateResiduals(const std::vector<DoubleDifference>& measurements,
                              const Vector3d& baseline) const;
    
    /**
     * @brief Form measurement matrix
     */
    MatrixXd formMeasurementMatrix(const std::vector<DoubleDifference>& measurements,
                                 const NavigationData& nav,
                                 const GNSSTime& time) const;
    
    /**
     * @brief Calculate measurement weights
     */
    MatrixXd calculateMeasurementWeights(const std::vector<DoubleDifference>& measurements) const;
    
    /**
     * @brief Check for sufficient satellites
     */
    bool hasSufficientSatellites(const std::vector<DoubleDifference>& measurements) const;
    
    /**
     * @brief Reset ambiguity for satellite
     */
    void resetAmbiguity(const SatelliteId& satellite, SignalType signal);
    
    /**
     * @brief Update processing statistics
     */
    void updateStatistics(SolutionStatus status) const;
};

/**
 * @brief RTK utility functions
 */
namespace rtk_utils {
    
    /**
     * @brief Calculate ionosphere-free combination
     */
    double calculateIonosphereFree(double l1_measurement, double l2_measurement,
                                 double f1, double f2);
    
    /**
     * @brief Calculate wide-lane combination
     */
    double calculateWideLane(double l1_phase, double l2_phase,
                           double l1_range, double l2_range,
                           double f1, double f2);
    
    /**
     * @brief Calculate narrow-lane combination
     */
    double calculateNarrowLane(double l1_phase, double l2_phase, double f1, double f2);
    
    /**
     * @brief Decorrelate ambiguity covariance matrix
     */
    MatrixXd decorrelateMatrix(const MatrixXd& covariance);
    
    /**
     * @brief Integer least squares search
     */
    VectorXd integerLeastSquares(const VectorXd& float_solution,
                               const MatrixXd& covariance_matrix);
    
    /**
     * @brief Calculate baseline length
     */
    double calculateBaselineLength(const Vector3d& baseline);
    
    /**
     * @brief Check baseline constraints
     */
    bool checkBaselineConstraints(const Vector3d& baseline,
                                double min_length, double max_length);
}

} // namespace libgnss
