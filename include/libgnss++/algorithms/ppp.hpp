#pragma once

#include "../core/processor.hpp"
#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"
#include "lambda.hpp"
#include "ppp_ar.hpp"
#include "ppp_shared.hpp"
#include "ppp_osr_types.hpp"
#include "spp.hpp"
#include <Eigen/Dense>
#include <array>
#include <mutex>

namespace libgnss {

/**
 * @brief Precise Point Positioning (PPP) processor
 * 
 * Implements PPP using precise orbits and clocks for decimeter to
 * centimeter level positioning without base stations.
 */
class PPPProcessor : public ProcessorBase {
public:
    using PPPConfig = ppp_shared::PPPConfig;
    using PPPState = ppp_shared::PPPState;
    using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;
    
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
     * @brief Load SSR orbit/clock corrections.
     */
    bool loadSSRProducts(const std::string& ssr_file);

    /**
     * @brief Load IONEX ionosphere products for future PPP hooks.
     */
    bool loadIONEXProducts(const std::string& ionex_file);

    /**
     * @brief Load DCB / Bias-SINEX products for future PPP hooks.
     */
    bool loadDCBProducts(const std::string& dcb_file);

    /**
     * @brief Load RTCM SSR orbit/clock corrections and convert them to sampled PPP corrections.
     */
    bool loadRTCMSSRProducts(const std::string& rtcm_file,
                             const NavigationData& nav,
                             double sample_step_seconds = 1.0);

    /**
     * @brief Interpolate a loaded SSR correction for inspection/debugging.
     */
    bool interpolateLoadedSSRCorrection(const SatelliteId& sat,
                                        const GNSSTime& time,
                                        Vector3d& orbit_correction_ecef,
                                        double& clock_correction_m,
                                        double* ura_sigma_m = nullptr,
                                        std::map<uint8_t, double>* code_bias_m = nullptr,
                                        std::map<uint8_t, double>* phase_bias_m = nullptr,
                                        std::map<std::string, std::string>* atmos_tokens = nullptr) const;

    /**
     * @brief Get the number of tropospheric atmospheric corrections applied in the last epoch.
     */
    int getLastAppliedAtmosphericTroposphereCorrections() const {
        return last_applied_atmos_trop_corrections_;
    }

    /**
     * @brief Get the number of ionospheric atmospheric corrections applied in the last epoch.
     */
    int getLastAppliedAtmosphericIonosphereCorrections() const {
        return last_applied_atmos_iono_corrections_;
    }

    /**
     * @brief Get the summed absolute atmospheric troposphere correction applied in the last epoch.
     */
    double getLastAppliedAtmosphericTroposphereMeters() const {
        return last_applied_atmos_trop_m_;
    }

    /**
     * @brief Get the summed absolute atmospheric ionosphere correction applied in the last epoch.
     */
    double getLastAppliedAtmosphericIonosphereMeters() const {
        return last_applied_atmos_iono_m_;
    }

    int getLastAppliedIonexCorrections() const {
        return last_applied_ionex_corrections_;
    }

    int getLastAppliedDcbCorrections() const {
        return last_applied_dcb_corrections_;
    }

    double getLastAppliedIonexMeters() const {
        return last_applied_ionex_m_;
    }

    double getLastAppliedDcbMeters() const {
        return last_applied_dcb_m_;
    }
    
    /**
     * @brief Check convergence status
     */
    bool hasConverged() const { return converged_; }

    /**
     * @brief Check whether any SSR products are loaded.
     */
    bool hasLoadedSSRProducts() const { return ssr_products_loaded_; }
    bool hasLoadedIONEXProducts() const { return ionex_products_loaded_; }
    bool hasLoadedDCBProducts() const { return dcb_products_loaded_; }
    size_t getLoadedIONEXMapCount() const { return ionex_products_.tec_maps.size(); }
    size_t getLoadedDCBEntryCount() const { return dcb_products_.entries.size(); }
    
    /**
     * @brief Get convergence time
     */
    double getConvergenceTime() const { return convergence_time_; }

private:
    PPPConfig ppp_config_;
    SPPProcessor spp_processor_;  ///< Fallback SPP processor
    PreciseProducts precise_products_;
    SSRProducts ssr_products_;
    IONEXProducts ionex_products_;
    DCBProducts dcb_products_;
    
    PPPState filter_state_;
    bool filter_initialized_ = false;
    
    // Convergence monitoring
    bool converged_ = false;
    double convergence_time_ = 0.0;
    GNSSTime convergence_start_time_;
    std::vector<Vector3d> recent_positions_;
    GNSSTime last_processed_time_;
    bool has_last_processed_time_ = false;
    bool precise_products_loaded_ = false;
    bool ssr_products_loaded_ = false;
    bool ionex_products_loaded_ = false;
    bool dcb_products_loaded_ = false;
    struct OceanLoadingCoefficients {
        std::array<double, 11> up_amplitudes_m{};
        std::array<double, 11> west_amplitudes_m{};
        std::array<double, 11> south_amplitudes_m{};
        std::array<double, 11> up_phases_deg{};
        std::array<double, 11> west_phases_deg{};
        std::array<double, 11> south_phases_deg{};
    };
    OceanLoadingCoefficients ocean_loading_coefficients_{};
    bool ocean_loading_loaded_ = false;
    std::map<std::string, std::map<SignalType, Vector3d>> receiver_antex_offsets_;
    bool receiver_antex_loaded_ = false;
    Vector3d static_anchor_position_ = Vector3d::Zero();
    bool has_static_anchor_position_ = false;
    double last_ar_ratio_ = 0.0;
    int last_fixed_ambiguities_ = 0;
    int last_applied_atmos_trop_corrections_ = 0;
    int last_applied_atmos_iono_corrections_ = 0;
    double last_applied_atmos_trop_m_ = 0.0;
    double last_applied_atmos_iono_m_ = 0.0;
    int last_applied_ionex_corrections_ = 0;
    int last_applied_dcb_corrections_ = 0;
    double last_applied_ionex_m_ = 0.0;
    double last_applied_dcb_m_ = 0.0;
    
public:
    using ARState = PPPState;
    using ARAmbiguityInfo = PPPAmbiguityInfo;

private:
    std::map<SatelliteId, PPPAmbiguityInfo> ambiguity_states_;
    std::map<SatelliteId, CLASDispersionCompensationInfo> clas_dispersion_compensation_;
    std::map<SatelliteId, CLASSisContinuityInfo> clas_sis_continuity_;

    // Pre-anchor covariance saved for DD-AR position correction
    Eigen::MatrixXd pre_anchor_covariance_;
    bool had_fixed_last_epoch_ = false;  ///< AR succeeded in previous epoch
    std::map<SatelliteId, double> windup_cache_;  ///< Phase wind-up cache for OSR
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> clas_phase_bias_repair_;

    // Statistics
    mutable std::mutex stats_mutex_;
    size_t total_epochs_processed_ = 0;
    size_t converged_solutions_ = 0;
    double total_convergence_time_ = 0.0;
    
    /**
     * @brief Initialize Kalman filter
     */
    bool initializeFilter(const ObservationData& obs,
                          const NavigationData& nav,
                          const PositionSolution* seed_solution = nullptr);
    
    /**
     * @brief Predict filter state
     */
    void predictState(double dt, const PositionSolution* seed_solution = nullptr);
    
    /**
     * @brief Update filter with measurements
     */
    bool updateFilter(const ObservationData& obs, const NavigationData& nav);
    
    /**
     * @brief Form ionosphere-free combinations
     */
    struct IonosphereFreeObs {
        SatelliteId satellite;
        SignalType primary_signal = SignalType::SIGNAL_TYPE_COUNT;
        SignalType secondary_signal = SignalType::SIGNAL_TYPE_COUNT;
        double primary_code_bias_coeff = 1.0;
        double secondary_code_bias_coeff = 0.0;
        double pseudorange_if = 0.0;
        double carrier_phase_if = 0.0;
        double pseudorange_code_bias_m = 0.0;
        double carrier_phase_bias_m = 0.0;
        double variance_pr = 0.0;
        double variance_cp = 0.0;
        Vector3d receiver_position = Vector3d::Zero();
        Vector3d satellite_position = Vector3d::Zero();
        Vector3d satellite_velocity = Vector3d::Zero();
        double satellite_clock_bias = 0.0;
        double satellite_clock_drift = 0.0;
        double elevation = 0.0;
        double azimuth = 0.0;
        double trop_mapping = 0.0;
        double modeled_trop_delay_m = 0.0;
        double antenna_pco_m = 0.0;
        double ambiguity_scale_m = 0.0;
        double atmospheric_trop_correction_m = 0.0;
        double atmospheric_iono_correction_m = 0.0;
        std::map<std::string, std::string> ssr_atmos_tokens;
        bool has_carrier_phase = false;
        bool valid = false;
    };
    
    std::vector<IonosphereFreeObs> formIonosphereFree(const ObservationData& obs,
                                                    const NavigationData& nav);
    
    /**
     * @brief Apply precise corrections
     */
    void applyPreciseCorrections(std::vector<IonosphereFreeObs>& observations,
                               const NavigationData& nav,
                               const GNSSTime& time);

    Vector3d calculateReceiverAntennaOffsetEcef(const Vector3d& receiver_marker_position,
                                                const IonosphereFreeObs& observation) const;
    
    /**
     * @brief Detect cycle slips
     */
    void detectCycleSlips(const ObservationData& obs);

    /**
     * @brief Ensure ambiguity states exist for valid carrier-phase observations.
     */
    void ensureAmbiguityStates(const std::vector<IonosphereFreeObs>& observations);

    /**
     * @brief Get ambiguity state index for a satellite, or -1 if absent.
     */
    int ambiguityStateIndex(const SatelliteId& satellite) const;

    /**
     * @brief Get or create ambiguity state for a satellite observation.
     */
    int getOrCreateAmbiguityState(const IonosphereFreeObs& observation);

    /**
     * @brief Initialize or reinitialize a phase-bias ambiguity state.
     */
    void initializeAmbiguityState(const IonosphereFreeObs& observation, int state_index);
    
    /**
     * @brief Resolve PPP ambiguities
     */
    bool resolveAmbiguities(const ObservationData& obs, const NavigationData& nav);
    bool resolveAmbiguitiesWLNL(const ObservationData& obs, const NavigationData& nav);
    std::map<SatelliteId, OSRCorrection> computeWlnlOsrCorrections(
        const ObservationData& obs,
        const NavigationData& nav,
        const Vector3d& receiver_position,
        double clock_bias_m,
        double trop_zenith) const;
    bool buildWlnlNlInfoForSatellite(
        const ObservationData& obs,
        const NavigationData& nav,
        const Vector3d& receiver_position,
        double clock_bias_m,
        double trop_zenith,
        const std::map<SatelliteId, OSRCorrection>& osr_by_sat,
        const SatelliteId& satellite,
        ppp_ar::WlnlNlInfo& info) const;
    bool buildFixedNlObservationForSatellite(
        const ObservationData& obs,
        const NavigationData& nav,
        const std::map<SatelliteId, OSRCorrection>& osr_by_sat,
        const SatelliteId& satellite,
        const PPPAmbiguityInfo& ambiguity,
        ppp_ar::FixedNlObservation& fixed_observation) const;
    bool buildFixedCarrierObservation(
        const IonosphereFreeObs& observation,
        ppp_ar::FixedCarrierObservation& fixed_observation) const;

    /// CLAS-PPP mode: process epoch using OSR-corrected observations
    PositionSolution processEpochCLAS(const ObservationData& obs, const NavigationData& nav);
    bool solveFixedPosition(const ObservationData& obs, const NavigationData& nav,
                            Vector3d& fixed_position);
    
    /**
     * @brief Calculate tropospheric delay
     */
    double calculateTroposphericDelay(const Vector3d& receiver_pos,
                                      const Vector3d& satellite_pos,
                                      const GNSSTime& time,
                                      double zenith_delay) const;
    
    /**
     * @brief Calculate tropospheric mapping function
     */
    double calculateMappingFunction(const Vector3d& receiver_pos,
                                    double elevation,
                                    const GNSSTime& time) const;
    
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
        std::vector<SatelliteId> row_satellites;  ///< Satellite for each row
        std::vector<bool> row_is_phase;           ///< true=carrier phase, false=code
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

    void reinitializeVectorState(int start_index, const Vector3d& value, double variance);
    void reinitializeScalarState(int index, double value, double variance);
    bool useLowDynamicsBroadcastSeedAssist() const;
    void recoverLowDynamicsBroadcastState(const ObservationData& obs,
                                          const PositionSolution* seed_solution);
    int receiverClockStateIndex(const SatelliteId& satellite) const;
    double receiverClockBiasMeters(const SatelliteId& satellite) const;
    double measurementVariance(const IonosphereFreeObs& observation, bool carrier_phase) const;
    
    /**
     * @brief Calculate measurement residuals
     */
        VectorXd calculateResiduals(const std::vector<IonosphereFreeObs>& observations,
                                  const NavigationData& nav,
                                  const GNSSTime& time) const;

    /**
     * @brief Clamp the unused velocity states when PPP runs in static or low-dynamics mode.
     */
    void constrainStaticVelocityStates();

    /**
     * @brief Keep static or low-dynamics PPP anchored to its initial SPP seed.
     */
    void constrainStaticAnchorPosition();

    bool solveFixedCarrierPhasePosition(const std::vector<IonosphereFreeObs>& observations,
                                        Vector3d& fixed_position) const;

    
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
