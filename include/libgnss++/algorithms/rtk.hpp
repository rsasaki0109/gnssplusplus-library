#pragma once

#include "../core/processor.hpp"
#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"
#include "rtk_selection.hpp"
#include "rtk_validation.hpp"
#include "spp.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <set>

namespace libgnss {

/**
 * @brief Real-Time Kinematic (RTK) processor
 *
 * Implements RTK positioning using carrier phase measurements
 * with ambiguity resolution for centimeter-level accuracy.
 *
 * State vector: [baseline(3), N1_SD_sat1, N1_SD_sat2, ..., N2_SD_sat1, ...]
 * SD (single-difference) ambiguities per satellite.
 * DD formed only in the observation model (H matrix maps SD states).
 * RTKLIB-compatible approach.
 */
class RTKProcessor : public ProcessorBase {
public:
    struct RTKConfig {
        double max_baseline_length = 50000.0;
        double min_baseline_length = 0.1;

        enum class AmbiguityResolutionMode {
            OFF = 0,
            CONTINUOUS = 1,
            INSTANTANEOUS = 2,
            PARTIAL = 3
        };
        enum class GlonassARMode {
            OFF = 0,
            ON = 1,
            AUTOCAL = 2
        };
        AmbiguityResolutionMode ar_mode = AmbiguityResolutionMode::CONTINUOUS;
        GlonassARMode glonass_ar_mode = GlonassARMode::OFF;

        double ratio_threshold = 3.0;
        double ambiguity_ratio_threshold = 3.0;
        double validation_threshold = 0.15;
        int min_satellites_for_ar = 5;
        int min_lock_count = 5;
        int min_hold_count = 5;   // RTKLIB minfix: consecutive fixes before holdamb

        // Ionosphere option
        enum class IonoOpt {
            OFF = 0,       // no ionosphere correction (L1/L2 separate)
            IFLC = 1,      // ionosphere-free linear combination
            EST = 2        // estimate DD ionosphere states in the KF
        };
        IonoOpt ionoopt = IonoOpt::OFF;

        // Position mode
        enum class PositionMode {
            STATIC = 0,    // position constant (add process noise)
            KINEMATIC = 1  // position reset each epoch (RTKLIB kinematic)
        };
        PositionMode position_mode = PositionMode::KINEMATIC;

        // Kalman filter parameters
        double process_noise_position = 1e-4;       // m^2/s for static baseline
        double process_noise_ambiguity = 1e-8;    // cycles^2/s (very small - ambiguities are constant)
        double process_noise_iono = 1e-4;         // m^2/s for DD ionosphere states
        int kf_iterations = 4;                       // more iterations for position convergence

        // Measurement noise
        double pseudorange_sigma = 0.3;           // m (RTKLIB eratio=100: 100*carrier_phase_sigma)
        double carrier_phase_sigma = 0.003;       // m (instrument noise)

        // Quality control
        bool enable_cycle_slip_detection = true;
        double cycle_slip_threshold = 0.05;
        bool enable_outlier_detection = true;
        double outlier_threshold = 3.0;

        // Elevation mask
        double elevation_mask = 15.0 * M_PI / 180.0;  // 15 degrees (RTKLIB default)

        // Incremental multi-GNSS rollout switches.
        bool enable_glonass = true;
        bool enable_beidou = true;
        double glonass_icb_l1_m_per_mhz = 0.0;
        double glonass_icb_l2_m_per_mhz = 0.0;
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

    PositionSolution processRTKEpoch(const ObservationData& rover_obs,
                                   const ObservationData& base_obs,
                                   const NavigationData& nav);

    void setBasePosition(const Vector3d& base_position) {
        base_position_ = base_position;
        base_position_known_ = true;
    }

    void setRTKConfig(const RTKConfig& config);
    const RTKConfig& getRTKConfig() const { return rtk_config_; }

public:
    bool lambdaMethod(const VectorXd& float_ambiguities,
                     const MatrixXd& covariance_matrix,
                     VectorXd& fixed_ambiguities,
                     double& success_rate);

    static int ambiguityStateIndex(const SatelliteId& sat, int freq) { return IB(sat, freq); }
    static int ionoStateIndex(const SatelliteId& sat) { return II(sat); }

private:
    RTKConfig rtk_config_;
    SPPProcessor spp_processor_;

    Vector3d base_position_;
    bool base_position_known_ = false;

    // Fixed-size state: [pos(3), glo_hw_bias(2), iono(MAXSAT), N1(MAXSAT), N2(MAXSAT)]
    // Satellite slots are system-aware so G01/E01/Q01 do not collide.
    static constexpr int BASE_STATES = 3;
    static constexpr int GLO_HWBIAS_STATES = 2;
    static constexpr int SATS_PER_SYSTEM = 64;
    static constexpr int SUPPORTED_SYSTEM_BLOCKS = 6;
    static constexpr int MAXSAT = SATS_PER_SYSTEM * SUPPORTED_SYSTEM_BLOCKS;
    static constexpr int REAL_STATES = BASE_STATES + GLO_HWBIAS_STATES;
    static constexpr int IONO_STATES = MAXSAT;
    static constexpr int NX = REAL_STATES + IONO_STATES + MAXSAT * 2;  // total state size

    static int systemSlotBase(GNSSSystem system) {
        switch (system) {
            case GNSSSystem::GPS: return 0 * SATS_PER_SYSTEM;
            case GNSSSystem::GLONASS: return 1 * SATS_PER_SYSTEM;
            case GNSSSystem::Galileo: return 2 * SATS_PER_SYSTEM;
            case GNSSSystem::BeiDou: return 3 * SATS_PER_SYSTEM;
            case GNSSSystem::QZSS: return 4 * SATS_PER_SYSTEM;
            case GNSSSystem::NavIC: return 5 * SATS_PER_SYSTEM;
            default: return 0;
        }
    }

    static int satelliteSlot(const SatelliteId& sat) {
        int prn = sat.prn;
        if (prn < 1) prn = 1;
        if (prn > SATS_PER_SYSTEM) prn = SATS_PER_SYSTEM;
        return systemSlotBase(sat.system) + (prn - 1);
    }

    static int IL(int freq) {
        return BASE_STATES + freq;
    }

    static int II(const SatelliteId& sat) {
        return REAL_STATES + satelliteSlot(sat);
    }

    static int IB(const SatelliteId& sat, int freq) {
        return REAL_STATES + IONO_STATES + freq * MAXSAT + satelliteSlot(sat);
    }

    struct RTKState {
        VectorXd state;
        MatrixXd covariance;
        // Legacy index maps (now use IB() directly)
        std::map<SatelliteId, int> iono_indices;
        std::map<SatelliteId, int> n1_indices;
        std::map<SatelliteId, int> n2_indices;
        int next_state_idx = BASE_STATES;
    };

    RTKState filter_state_;
    bool filter_initialized_ = false;

    // Lock count per satellite per frequency (for AR eligibility)
    std::map<SatelliteId, int> lock_count_l1_;
    std::map<SatelliteId, int> lock_count_l2_;

    // Reference satellite tracking
    SatelliteId current_ref_satellite_;
    bool has_ref_satellite_ = false;

    // Fixed solution baseline (from LAMBDA)
    Vector3d fixed_baseline_;
    bool has_fixed_solution_ = false;

    // Last validated fixed position (for position reset in next epoch)
    Vector3d last_fixed_position_ = Vector3d::Zero();
    bool has_last_fixed_position_ = false;
    Vector3d last_solution_position_ = Vector3d::Zero();
    bool has_last_solution_position_ = false;
    Vector3d last_trusted_position_ = Vector3d::Zero();
    bool has_last_trusted_position_ = false;

    // Consecutive fix tracking for holdamb
    int consecutive_fix_count_ = 0;

    // Last fix data for holdamb
    struct DDPair {
        SatelliteId ref_sat;
        int ref_idx;
        int sat_idx;
        SatelliteId sat;
        int freq;
    };
    struct HoldStateSnapshot {
        Vector3d last_fixed_position = Vector3d::Zero();
        bool has_last_fixed_position = false;
        std::vector<DDPair> dd_pairs;
        std::vector<int> best_subset;
        VectorXd dd_fixed;
        double ar_ratio = 0.0;
        int num_fixed_ambiguities = 0;

        bool hasHeldIntegers() const { return dd_fixed.size() > 0; }
    };
    std::vector<DDPair> last_dd_pairs_;
    std::vector<int> last_best_subset_;
    VectorXd last_dd_fixed_;
    double last_ar_ratio_ = 0.0;
    int last_num_fixed_ambiguities_ = 0;

    // Epoch tracking
    GNSSTime last_epoch_time_;
    bool has_last_epoch_ = false;
    GNSSTime last_trusted_time_;
    bool has_last_trusted_time_ = false;

    // Statistics
    mutable std::mutex stats_mutex_;
    size_t total_epochs_processed_ = 0;
    size_t fixed_solutions_ = 0;
    size_t float_solutions_ = 0;

    // Satellite data for current epoch
    struct SatelliteData {
        SatelliteId satellite;
        SignalType l1_signal = SignalType::GPS_L1CA;
        SignalType l2_signal = SignalType::GPS_L2C;
        double l1_wavelength = 0.0;
        double l2_wavelength = 0.0;
        double l1_frequency_hz = 0.0;
        double l2_frequency_hz = 0.0;
        // L1
        double rover_l1_phase = 0.0;  // cycles
        double rover_l1_code = 0.0;   // meters
        double base_l1_phase = 0.0;
        double base_l1_code = 0.0;
        bool has_l1 = false;
        int l1_lli = 0;
        // L2
        double rover_l2_phase = 0.0;
        double rover_l2_code = 0.0;
        double base_l2_phase = 0.0;
        double base_l2_code = 0.0;
        bool has_l2 = false;
        int l2_lli = 0;
        // Geometry
        Vector3d sat_pos;          // satellite position for rover (RTKLIB satposs from rover PR)
        Vector3d sat_pos_base;     // satellite position for base (RTKLIB satposs from base PR)
        double elevation = 0.0;       // elevation from rover position
        double base_elevation = 0.0;   // elevation from base position
        bool has_ephemeris = false;
    };

    // Current epoch satellite data (cached for use across functions)
    std::map<SatelliteId, SatelliteData> current_sat_data_;
    std::map<SatelliteId, double> gf_l1l2_history_;

    /**
     * Collect all satellite data for an epoch (L1+L2 for rover and base)
     */
    std::map<SatelliteId, SatelliteData> collectSatelliteData(
        const ObservationData& rover_obs,
        const ObservationData& base_obs,
        const NavigationData& nav);

    /**
     * Select reference satellite (highest elevation with L1+L2)
     */
    SatelliteId selectReferenceSatellite(
        const std::map<SatelliteId, SatelliteData>& sat_data);

    /**
     * Handle reference satellite change: no-op for SD parameterization
     */
    void handleReferenceSatelliteChange(const SatelliteId& new_ref,
                                        const std::map<SatelliteId, SatelliteData>& sat_data);

    /**
     * Initialize filter
     */
    bool initializeFilter(const ObservationData& rover_obs,
                        const ObservationData& base_obs,
                        const NavigationData& nav);

    /**
     * Update SD biases (RTKLIB udbias): initialize new, reset slipped
     */
    void updateBias(const std::map<SatelliteId, SatelliteData>& sat_data);

    /**
     * Predict state (kinematic: reset position variance)
     */
    void predictState(double dt);

    /**
     * Kinematic position reset: SPP position with large variance (RTKLIB udpos)
     */
    void resetPositionToSPP(const ObservationData& rover_obs, const NavigationData& nav);

    /**
     * Full KF update with DD observation model mapping to SD states
     */
    bool updateFilter(const std::map<SatelliteId, SatelliteData>& sat_data);

    // Keep old signature for compatibility
    bool updateFilter(const ObservationData& rover_obs,
                    const ObservationData& base_obs,
                    const NavigationData& nav);

    /**
     * Resolve ambiguities: SD->DD transform + LAMBDA (RTKLIB resamb_LAMBDA)
     */
    bool resolveAmbiguities();
    bool resolveAmbiguities(std::vector<DDPair> dd_pairs);

    PositionSolution generateSolution(const GNSSTime& time,
                                    SolutionStatus status,
                                    int num_satellites);
    void rememberSolution(const PositionSolution& solution);

    void updateStatistics(SolutionStatus status) const;

    /**
     * Validate fixed solution with post-fit residual check (RTKLIB valpos)
     */
    bool validateFixedSolution(const std::map<SatelliteId, SatelliteData>& sat_data);

    /**
     * Hold ambiguities after consecutive fixes (RTKLIB holdamb)
     */
    void applyHoldAmbiguity();

    /**
     * Try to produce a fixed solution using held DD integers when LAMBDA fails
     */
    bool tryHoldFix(const std::map<SatelliteId, SatelliteData>& sat_data,
                    const GNSSTime& time, int n_sats, PositionSolution& solution);
    HoldStateSnapshot captureHoldState() const;
    void restoreHoldState(const HoldStateSnapshot& snapshot);

    /**
     * Increment lock counts after KF update (RTKLIB: lock++ after filter)
     */
    void incrementLockCounts(const std::map<SatelliteId, SatelliteData>& sat_data);

    /**
     * Get or create SD ambiguity state index for a satellite
     */
    int getOrCreateN1Index(const SatelliteId& sat, double initial_value);
    int getOrCreateN2Index(const SatelliteId& sat, double initial_value);
    int getOrCreateIonoIndex(const SatelliteId& sat, double initial_value);

    bool selectSystemReferenceSatellite(const std::map<SatelliteId, SatelliteData>& sat_data,
                                        GNSSSystem system,
                                        int min_lock_count,
                                        SatelliteId& ref_sat) const;
    std::vector<rtk_selection::SatelliteSelectionData> buildSelectionSnapshot(
        const std::map<SatelliteId, SatelliteData>& sat_data) const;
    std::vector<DDPair> buildDoubleDifferencePairs(
        const std::map<SatelliteId, SatelliteData>& sat_data,
        int min_lock_count) const;

    /**
     * Expand state vector to accommodate new states
     */
    void expandState(int new_size);

    /**
     * Remove satellite from state
     */
    void removeSatelliteFromState(const SatelliteId& sat);
    void syncSPPConfig();
    void updateGlonassHardwareBias(double dt);

    /**
     * RTKLIB varerr: SD measurement error variance
     * var = 2.0 * (a^2 + b^2/sin^2(el))
     */
    double varerr(double elevation, bool is_phase) const;

    // Legacy stubs kept for interface compatibility
    struct DoubleDifference {
        SatelliteId reference_satellite;
        SatelliteId satellite;
        SignalType signal;
        double pseudorange_dd = 0.0;
        double carrier_phase_dd = 0.0;
        double geometric_range = 0.0;
        Vector3d unit_vector = Vector3d::Zero();
        double elevation = 0.0;
        double variance = 0.0;
        bool valid = false;
    };
    struct AmbiguityInfo {
        double float_value = 0.0;
        double fixed_value = 0.0;
        int lock_count = 0;
        bool is_fixed = false;
        double last_phase = 0.0;
        GNSSTime last_time;
    };
    std::map<SatelliteId, std::map<SignalType, AmbiguityInfo>> ambiguity_states_;
    std::vector<DoubleDifference> formDoubleDifferences(
        const ObservationData& rover_obs, const ObservationData& base_obs,
        const NavigationData& nav);
    void detectCycleSlips(const ObservationData& rover_obs,
                        const ObservationData& base_obs);
    bool resolveAmbiguities(int dummy);
    struct LAMBDAResult {
        VectorXd fixed_ambiguities;
        double ratio = 0.0;
        bool success = false;
    };
    LAMBDAResult solveLAMBDA(const VectorXd& float_ambiguities,
                           const MatrixXd& ambiguity_covariance);
    bool validateAmbiguityResolution(const VectorXd& fixed_ambiguities,
                                   const VectorXd& float_ambiguities,
                                   const MatrixXd& covariance,
                                   double ratio);
    void updateAmbiguityStates(const std::vector<DoubleDifference>& double_diffs);
    Vector3d calculateBaseline() const;
    VectorXd calculateResiduals(const std::vector<DoubleDifference>& measurements,
                              const Vector3d& baseline) const;
    MatrixXd formMeasurementMatrix(const std::vector<DoubleDifference>& measurements,
                                 const NavigationData& nav,
                                 const GNSSTime& time) const;
    MatrixXd calculateMeasurementWeights(const std::vector<DoubleDifference>& measurements) const;
    bool hasSufficientSatellites(const std::vector<DoubleDifference>& measurements) const;
    void resetAmbiguity(const SatelliteId& satellite, SignalType signal);
    bool applyFixedAmbiguities(const VectorXd& fixed_n1,
                               const VectorXd& fixed_n2,
                               const std::map<SatelliteId, SatelliteData>& sat_data);
    void solvePositionWithAmbiguities(const std::map<SatelliteId, SatelliteData>& sat_data);
    bool trySingleEpochAR(const std::map<SatelliteId, SatelliteData>& sat_data);
    double elevationWeight(double elevation) const;
};

namespace rtk_utils {
    double calculateIonosphereFree(double l1_measurement, double l2_measurement,
                                 double f1, double f2);
    double calculateWideLane(double l1_phase, double l2_phase,
                           double l1_range, double l2_range,
                           double f1, double f2);
    double calculateNarrowLane(double l1_phase, double l2_phase, double f1, double f2);
    MatrixXd decorrelateMatrix(const MatrixXd& covariance);
    VectorXd integerLeastSquares(const VectorXd& float_solution,
                               const MatrixXd& covariance_matrix);
    double calculateBaselineLength(const Vector3d& baseline);
    bool checkBaselineConstraints(const Vector3d& baseline,
                                double min_length, double max_length);
}

} // namespace libgnss
