#pragma once

#include "../core/processor.hpp"
#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"
#include <Eigen/Dense>
#include <array>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

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
        bool enable_raim_fde = true;                  ///< Enable leave-one-out RAIM/FDE
        double raim_fde_min_rms_improvement_ratio = 0.25; ///< Minimum fractional RMS improvement
        double raim_fde_min_rms_improvement_m = 1.0;  ///< Minimum absolute RMS improvement [m]
        double max_gdop = 50.0;                       ///< Reject solutions above this GDOP (<=0 disables)
        double max_residual_rms = 0.0;                ///< Reject solutions above this RMS [m] (<=0 disables)
        double max_chi_square_per_dof = 0.0;          ///< Reject solutions above reduced chi-square (<=0 disables)
        bool use_variance_model = true;               ///< Use explicit code/elevation/SNR/atmosphere variance
        double snr_reference_dbhz = 45.0;             ///< CN0 where no SNR penalty is applied
        double ionosphere_sigma_scale = 0.5;          ///< Residual ionosphere sigma as fraction of correction
        double troposphere_sigma_scale = 0.1;         ///< Residual troposphere sigma as fraction of correction
        bool enable_robust_weighting = false;         ///< Downweight large code residuals inside WLS iterations
        double robust_weight_threshold_sigma = 3.0;   ///< Huber threshold for residual downweighting
        double robust_weight_min_factor = 0.05;       ///< Minimum multiplicative robust weight factor
        bool enable_adaptive_robust_weighting = false; ///< Enable Huber weights only for residual-tail epochs
        double adaptive_robust_activation_threshold_sigma = 3.0; ///< Tail threshold to activate robust weights
        int adaptive_robust_min_tail_measurements = 2; ///< Minimum tail residual count for activation
        double adaptive_robust_min_tail_fraction = 0.08; ///< Minimum tail residual fraction for activation
        bool enable_position_jump_gate = false;       ///< Reject kinematically implausible SPP position jumps
        double max_position_jump_rate_mps = 0.0;      ///< Max accepted position step rate [m/s] (<=0 disables)
        double max_position_jump_min_m = 0.0;         ///< Minimum allowed position step [m]
        bool use_ionosphere_free_combination = false; ///< Use dual-frequency code IFLC when available
        bool use_ionex_corrections = true;            ///< Prefer loaded IONEX TEC maps over broadcast ionosphere
        bool use_dcb_corrections = true;              ///< Apply loaded OSB/DCB code-bias products when available
        bool use_precise_products = true;             ///< Use loaded SP3/CLK products for satellite orbit/clock
        bool use_ssr_corrections = true;              ///< Apply loaded SSR orbit/clock/code-bias corrections
        
        // Clock modeling
        bool model_intersystem_bias = true;           ///< Model inter-system clock biases
        bool enable_beidou = true;                    ///< Enable BeiDou SPP support
        bool enable_glonass = true;                   ///< Enable GLONASS SPP support
    };

    /**
     * @brief Corrected measurement for external solvers (e.g. particle filter)
     */
    struct CorrectedMeasurement {
        std::array<double, 3> satellite_ecef;    ///< Sagnac-corrected satellite ECEF [m]
        double corrected_pseudorange;            ///< Fully corrected pseudorange [m]
        double weight;                           ///< Inverse measurement variance [1/m^2]
        double variance;                         ///< Measurement variance [m^2]
        double elevation;                        ///< Elevation angle [rad]
        int system_id;                           ///< 0=GPS, 1=GLONASS, 2=Galileo, 3=BeiDou, 4=QZSS
        bool ionosphere_free = false;            ///< True when built from a dual-frequency IFLC
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
     * @brief Preprocess epoch: apply all corrections and return corrected measurements
     *
     * Returns satellite positions (Sagnac-corrected), corrected pseudoranges,
     * and elevation-based weights for external use (e.g. particle filter input).
     * Also returns the SPP position solution for reference.
     */
    std::pair<PositionSolution, std::vector<CorrectedMeasurement>>
    preprocessEpoch(const ObservationData& obs, const NavigationData& nav);

    /**
     * @brief Set SPP-specific configuration
     */
    void setSPPConfig(const SPPConfig& config) { spp_config_ = config; }
    
    /**
     * @brief Get SPP configuration
     */
    const SPPConfig& getSPPConfig() const { return spp_config_; }

    /**
     * @brief Load IONEX ionosphere products for SPP ionosphere corrections
     */
    bool loadIONEXProducts(const std::string& ionex_file);

    /**
     * @brief Load SP3 orbit and CLK clock products for SPP satellite states
     */
    bool loadPreciseProducts(const std::string& orbit_file, const std::string& clock_file);

    /**
     * @brief Load DCB / Bias-SINEX products for SPP code-bias corrections
     */
    bool loadDCBProducts(const std::string& dcb_file);

    /**
     * @brief Load CSV SSR orbit/clock/code-bias products for SPP corrections
     */
    bool loadSSRProducts(const std::string& ssr_file);

    bool hasLoadedPreciseProducts() const { return precise_products_loaded_; }
    bool hasLoadedIONEXProducts() const { return ionex_products_loaded_; }
    bool hasLoadedDCBProducts() const { return dcb_products_loaded_; }
    bool hasLoadedSSRProducts() const { return ssr_products_loaded_; }
    size_t getLoadedPreciseSatelliteCount() const { return precise_products_.orbit_clock_data.size(); }
    size_t getLoadedIONEXMapCount() const { return ionex_products_.tec_maps.size(); }
    size_t getLoadedDCBEntryCount() const { return dcb_products_.entries.size(); }
    size_t getLoadedSSRSatelliteCount() const { return ssr_products_.orbit_clock_corrections.size(); }
    int getLastAppliedPreciseOrbitClockMeasurements() const {
        return last_applied_precise_orbit_clock_measurements_;
    }
    int getLastAppliedSSROrbitClockCorrections() const {
        return last_applied_ssr_orbit_clock_corrections_;
    }
    int getLastAppliedSSRCodeBiasCorrections() const {
        return last_applied_ssr_code_bias_corrections_;
    }
    int getLastAppliedIonexCorrections() const { return last_applied_ionex_corrections_; }
    int getLastAppliedDcbCorrections() const { return last_applied_dcb_corrections_; }
    double getLastAppliedSSROrbitMeters() const { return last_applied_ssr_orbit_m_; }
    double getLastAppliedSSRClockMeters() const { return last_applied_ssr_clock_m_; }
    double getLastAppliedSSRCodeBiasMeters() const { return last_applied_ssr_code_bias_m_; }
    double getLastAppliedIonexMeters() const { return last_applied_ionex_m_; }
    double getLastAppliedDcbMeters() const { return last_applied_dcb_m_; }

private:
    SPPConfig spp_config_;
    PreciseProducts precise_products_;
    SSRProducts ssr_products_;
    IONEXProducts ionex_products_;
    DCBProducts dcb_products_;
    bool precise_products_loaded_ = false;
    bool ssr_products_loaded_ = false;
    bool ionex_products_loaded_ = false;
    bool dcb_products_loaded_ = false;
    
    // State variables
    Vector3d estimated_position_;           ///< Current position estimate (ECEF)
    double receiver_clock_bias_;            ///< Receiver clock bias in seconds
    std::map<GNSSSystem, double> system_biases_; ///< Inter-system clock biases
    bool has_last_valid_position_ = false;
    Vector3d last_valid_position_;          ///< Last accepted SPP position for jump gating
    GNSSTime last_valid_time_;
    double last_valid_clock_bias_ = 0.0;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    mutable size_t total_epochs_processed_ = 0;
    mutable size_t successful_solutions_ = 0;
    mutable double total_processing_time_ms_ = 0.0;
    int last_applied_precise_orbit_clock_measurements_ = 0;
    int last_applied_ssr_orbit_clock_corrections_ = 0;
    int last_applied_ssr_code_bias_corrections_ = 0;
    int last_applied_ionex_corrections_ = 0;
    int last_applied_dcb_corrections_ = 0;
    double last_applied_ssr_orbit_m_ = 0.0;
    double last_applied_ssr_clock_m_ = 0.0;
    double last_applied_ssr_code_bias_m_ = 0.0;
    double last_applied_ionex_m_ = 0.0;
    double last_applied_dcb_m_ = 0.0;

    struct SPPObservation {
        Observation observation;
        bool ionosphere_free = false;
        SignalType primary_signal = SignalType::SIGNAL_TYPE_COUNT;
        SignalType secondary_signal = SignalType::SIGNAL_TYPE_COUNT;
        double primary_coeff = 1.0;
        double secondary_coeff = 0.0;
        double variance_scale = 1.0;
    };
    
    /**
     * @brief Solve position using weighted least squares
     */
    PositionSolution solvePosition(const std::vector<SPPObservation>& valid_obs,
                                 const NavigationData& nav,
                                 const GNSSTime& time);

    /**
     * @brief Native weighted least-squares solver with atmospheric corrections
     */
    PositionSolution solvePositionLS(const std::vector<SPPObservation>& valid_obs,
                                    const NavigationData& nav,
                                    const GNSSTime& time,
                                    bool allow_qc_rejection = true);

    /**
     * @brief Calculate predicted pseudorange
     */
    double calculatePredictedRange(const Vector3d& receiver_pos,
                                 const Vector3d& satellite_pos,
                                 double receiver_clock_bias_m,
                                 double satellite_clock_bias_s) const;
    
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
    std::vector<SPPObservation> detectOutliers(const std::vector<SPPObservation>& observations,
                                               const VectorXd& residuals,
                                               double threshold_sigma) const;
    
    /**
     * @brief Calculate dilution of precision values
     */
    void calculateDOP(const MatrixXd& geometry_matrix, PositionSolution& solution) const;
    
    /**
     * @brief Validate observations for SPP processing
     */
    std::vector<SPPObservation> validateObservations(const ObservationData& obs,
                                                     const NavigationData& nav,
                                                     const GNSSTime& time) const;
    
    /**
     * @brief Initialize position estimate
     */
    bool initializePosition(const std::vector<SPPObservation>& observations,
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
        bool precise_orbit_clock = false;
        bool ssr_orbit_clock = false;
    };
    
    std::map<SatelliteId, SatelliteState> calculateSatelliteStates(
        const std::vector<SPPObservation>& observations,
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

    struct MeasurementVarianceInputs {
        double elevation_rad = 0.0;
        double snr_dbhz = 0.0;
        double pseudorange_sigma_m = 3.0;
        double ionosphere_delay_m = 0.0;
        double troposphere_delay_m = 0.0;
        bool ionosphere_corrected = false;
        bool troposphere_corrected = false;
        double snr_reference_dbhz = 45.0;
        double ionosphere_sigma_scale = 0.5;
        double troposphere_sigma_scale = 0.1;
    };

    /**
     * @brief Calculate SPP pseudorange variance from code, elevation, SNR and atmosphere terms
     */
    double calculatePseudorangeVariance(const MeasurementVarianceInputs& inputs);

    /**
     * @brief Calculate first-order ionosphere-free code combination coefficients
     */
    std::pair<double, double> ionosphereFreeCoefficients(double f1_hz, double f2_hz);

    /**
     * @brief Calculate first-order ionosphere-free pseudorange
     */
    double calculateIonosphereFreePseudorange(double p1_m,
                                              double p2_m,
                                              double f1_hz,
                                              double f2_hz);

    /**
     * @brief Calculate a robust residual inlier mask for redundant SPP measurements
     */
    std::vector<bool> calculateResidualInlierMask(const std::vector<double>& residuals,
                                                  double threshold_sigma,
                                                  double sigma_floor = 1.0);

    /**
     * @brief Select the best leave-one-out RAIM/FDE candidate by residual RMS.
     *
     * @return Index into candidate_rms_values, or -1 when no candidate clears
     * the configured improvement thresholds.
     */
    int selectRaimFdeCandidate(double baseline_rms,
                               const std::vector<double>& candidate_rms_values,
                               double min_improvement_ratio,
                               double min_improvement_m);
    
    /**
     * @brief Check satellite visibility
     */
    bool isSatelliteVisible(const Vector3d& receiver_pos, 
                          const Vector3d& satellite_pos,
                          double min_elevation_rad);
}

} // namespace libgnss
