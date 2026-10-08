#pragma once

#include "../core/types.hpp"

#include <Eigen/Dense>
#include <cstddef>
#include <deque>

namespace libgnss {
namespace barometer {

/// ICAO standard-atmosphere sea-level pressure [hPa].
constexpr double kStandardSeaLevelPressureHpa = 1013.25;

/**
 * @brief Pressure to height with the ICAO standard atmosphere (troposphere).
 *
 * h = 44330.77 * (1 - (p / 1013.25)^0.190263) [m].  The result is a *relative*
 * height scale only: weather, temperature and indoor HVAC leave an unknown,
 * drifting offset that BarometerHeightFilter estimates.  Returns NaN for
 * non-finite or non-positive pressure.
 */
double pressureToStandardAtmosphereHeightM(double pressure_hpa);

/// Inverse of pressureToStandardAtmosphereHeightM (NaN outside the model).
double standardAtmosphereHeightToPressureHpa(double height_m);

/**
 * @brief Barometer-aided SPP height configuration (default OFF).
 *
 * The filter state is x = [h, b]: the true ellipsoidal height and a barometer
 * bias, with baro measurement  hb = h + b + noise,  where hb is the standard
 * atmosphere height of the pressure sample.  GNSS height measures h only, so
 * b becomes observable through GNSS and is modelled as a random walk.
 */
struct BaroHeightConfig {
    bool enabled = false;
    double baro_sigma_m = 1.0;            ///< 1-sigma of the (windowed) pressure height [m]
    double height_walk_m_per_sqrt_s = 1.0; ///< process noise of true height [m/sqrt(s)]
    double bias_walk_m_per_sqrt_s = 0.05; ///< process noise of the baro bias [m/sqrt(s)]
    double gnss_sigma_scale = 1.0;        ///< inflation of the SPP vertical sigma
    double gnss_sigma_floor_m = 5.0;      ///< minimum GNSS height sigma [m]
    double init_max_gdop = 6.0;           ///< geometry gate for init and GNSS height update
    int init_min_satellites = 6;
    double innovation_gate_sigma = 4.0;   ///< gate for baro and GNSS height innovations
    int rebase_after_rejections = 10;     ///< consecutive baro rejections before re-basing the bias
    double rebase_bias_sigma_m = 3.0;     ///< extra bias uncertainty injected on re-base [m]
    double prior_sigma_floor_m = 0.5;     ///< smallest sigma of the LS height constraint [m]
    double max_prior_sigma_m = 8.0;       ///< no constraint when the prior is weaker than this [m]
    double sample_window_s = 2.0;         ///< causal averaging window of pressure samples [s]
    double max_sample_age_s = 3.0;        ///< newest sample must be this recent [s]
};

/**
 * @brief Causal pressure buffer; returns the mean pressure height over the
 * window (t - window, t] and never looks at samples after t.
 */
class BarometerSampleBuffer {
public:
    void add(const GNSSTime& time, double pressure_hpa);
    /// Mean standard-atmosphere height [m] over (t - window, t]; false if the
    /// newest sample is older than max_age_s or the window is empty.
    bool heightAt(const GNSSTime& time, double window_s, double max_age_s,
                  double& height_m, int* sample_count = nullptr) const;
    std::size_t size() const { return samples_.size(); }
    void clear() { samples_.clear(); }

private:
    struct Sample {
        GNSSTime time;
        double height_m;
    };
    std::deque<Sample> samples_;
};

/**
 * @brief Two-state Kalman filter for height h and barometer bias b.
 */
class BarometerHeightFilter {
public:
    explicit BarometerHeightFilter(const BaroHeightConfig& config = BaroHeightConfig());

    void setConfig(const BaroHeightConfig& config) { config_ = config; }
    void reset();
    bool initialized() const { return initialized_; }

    /// Start the filter from a GNSS height h with sigma and a baro height hb.
    void initialize(const GNSSTime& time, double gnss_height_m, double gnss_sigma_m,
                    double baro_height_m);
    /// Propagate to `time` (random walks on h and b). No-op when not initialized.
    void predict(const GNSSTime& time);
    /**
     * @brief Baro measurement update  hb = h + b + v.
     * @return false when the innovation gate rejects the sample. After
     * `rebase_after_rejections` consecutive rejections the bias is re-based
     * to the new pressure level (step in pressure, e.g. door/HVAC) and the
     * sample is accepted.
     */
    bool updateBaro(double baro_height_m);
    /**
     * @brief GNSS height update  hg = h + v. Gated like updateBaro; the gate
     * is skipped when not initialized.
     */
    bool updateGnss(double gnss_height_m, double gnss_sigma_m);

    double height() const { return x_(0); }
    double bias() const { return x_(1); }
    double heightSigma() const;
    double biasSigma() const;
    const Eigen::Matrix2d& covariance() const { return p_; }
    int consecutiveBaroRejections() const { return baro_rejections_; }
    int rebaseCount() const { return rebase_count_; }

private:
    bool scalarUpdate(const Eigen::Vector2d& h_row, double innovation, double sigma_m,
                      bool gate);

    BaroHeightConfig config_;
    bool initialized_ = false;
    GNSSTime time_;
    Eigen::Vector2d x_ = Eigen::Vector2d::Zero();
    Eigen::Matrix2d p_ = Eigen::Matrix2d::Zero();
    int baro_rejections_ = 0;
    int rebase_count_ = 0;
};

}  // namespace barometer
}  // namespace libgnss
