#include <libgnss++/algorithms/barometer_height.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace libgnss {
namespace barometer {

namespace {
constexpr double kScaleHeightM = 44330.77;
constexpr double kExponent = 0.190263;
constexpr double kMaxBufferedSeconds = 120.0;
}  // namespace

double pressureToStandardAtmosphereHeightM(double pressure_hpa) {
    if (!std::isfinite(pressure_hpa) || pressure_hpa <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return kScaleHeightM *
           (1.0 - std::pow(pressure_hpa / kStandardSeaLevelPressureHpa, kExponent));
}

double standardAtmosphereHeightToPressureHpa(double height_m) {
    if (!std::isfinite(height_m)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double ratio = 1.0 - height_m / kScaleHeightM;
    if (ratio <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return kStandardSeaLevelPressureHpa * std::pow(ratio, 1.0 / kExponent);
}

void BarometerSampleBuffer::add(const GNSSTime& time, double pressure_hpa) {
    const double height = pressureToStandardAtmosphereHeightM(pressure_hpa);
    if (!std::isfinite(height)) {
        return;
    }
    // Samples must arrive in time order; an out-of-order sample would break
    // causality of the windowed mean, so it is dropped.
    if (!samples_.empty() && (time - samples_.back().time) < 0.0) {
        return;
    }
    samples_.push_back({time, height});
    while (!samples_.empty() && (time - samples_.front().time) > kMaxBufferedSeconds) {
        samples_.pop_front();
    }
}

bool BarometerSampleBuffer::heightAt(const GNSSTime& time, double window_s, double max_age_s,
                                     double& height_m, int* sample_count) const {
    double sum = 0.0;
    int count = 0;
    double newest_age = std::numeric_limits<double>::infinity();
    for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
        const double age = time - it->time;
        if (age < 0.0) {
            continue;  // sample from the future: never used
        }
        newest_age = std::min(newest_age, age);
        if (age >= window_s) {
            break;
        }
        sum += it->height_m;
        ++count;
    }
    if (sample_count != nullptr) {
        *sample_count = count;
    }
    if (count == 0 || newest_age > max_age_s) {
        return false;
    }
    height_m = sum / static_cast<double>(count);
    return true;
}

BarometerHeightFilter::BarometerHeightFilter(const BaroHeightConfig& config)
    : config_(config) {}

void BarometerHeightFilter::reset() {
    initialized_ = false;
    time_ = GNSSTime();
    x_.setZero();
    p_.setZero();
    baro_rejections_ = 0;
    rebase_count_ = 0;
}

void BarometerHeightFilter::initialize(const GNSSTime& time, double gnss_height_m,
                                       double gnss_sigma_m, double baro_height_m) {
    const double sg2 = gnss_sigma_m * gnss_sigma_m;
    const double sb2 = config_.baro_sigma_m * config_.baro_sigma_m;
    x_(0) = gnss_height_m;
    x_(1) = baro_height_m - gnss_height_m;
    // b = hb - h  =>  Var(b) = sg2 + sb2, Cov(h, b) = -sg2.
    p_ << sg2, -sg2, -sg2, sg2 + sb2;
    time_ = time;
    initialized_ = true;
    baro_rejections_ = 0;
}

void BarometerHeightFilter::predict(const GNSSTime& time) {
    if (!initialized_) {
        return;
    }
    const double dt = time - time_;
    if (!(dt > 0.0)) {
        return;
    }
    p_(0, 0) += config_.height_walk_m_per_sqrt_s * config_.height_walk_m_per_sqrt_s * dt;
    p_(1, 1) += config_.bias_walk_m_per_sqrt_s * config_.bias_walk_m_per_sqrt_s * dt;
    time_ = time;
}

bool BarometerHeightFilter::scalarUpdate(const Eigen::Vector2d& h_row, double innovation,
                                         double sigma_m, bool gate) {
    const double r = sigma_m * sigma_m;
    const Eigen::Vector2d ph = p_ * h_row;
    const double s = h_row.dot(ph) + r;
    if (!(s > 0.0) || !std::isfinite(innovation)) {
        return false;
    }
    if (gate && config_.innovation_gate_sigma > 0.0 &&
        innovation * innovation > config_.innovation_gate_sigma * config_.innovation_gate_sigma * s) {
        return false;
    }
    const Eigen::Vector2d k = ph / s;
    x_ += k * innovation;
    // Joseph form keeps P symmetric positive semi-definite.
    const Eigen::Matrix2d ikh = Eigen::Matrix2d::Identity() - k * h_row.transpose();
    p_ = ikh * p_ * ikh.transpose() + k * r * k.transpose();
    p_ = 0.5 * (p_ + p_.transpose());
    return true;
}

bool BarometerHeightFilter::updateBaro(double baro_height_m) {
    if (!initialized_ || !std::isfinite(baro_height_m)) {
        return false;
    }
    const Eigen::Vector2d h_row(1.0, 1.0);
    const double innovation = baro_height_m - (x_(0) + x_(1));
    if (scalarUpdate(h_row, innovation, config_.baro_sigma_m, true)) {
        baro_rejections_ = 0;
        return true;
    }
    ++baro_rejections_;
    if (config_.rebase_after_rejections > 0 &&
        baro_rejections_ >= config_.rebase_after_rejections) {
        // Persistent pressure step (door, HVAC, train): keep the height, move
        // the bias to the new pressure level, and widen its uncertainty.
        const double ph = p_(0, 0);
        const double sb2 = config_.baro_sigma_m * config_.baro_sigma_m;
        const double step2 = config_.rebase_bias_sigma_m * config_.rebase_bias_sigma_m;
        x_(1) = baro_height_m - x_(0);
        p_ << ph, -ph, -ph, ph + sb2 + step2;
        baro_rejections_ = 0;
        ++rebase_count_;
        return true;
    }
    return false;
}

bool BarometerHeightFilter::updateGnss(double gnss_height_m, double gnss_sigma_m) {
    if (!initialized_ || !std::isfinite(gnss_height_m) || !(gnss_sigma_m > 0.0)) {
        return false;
    }
    const Eigen::Vector2d h_row(1.0, 0.0);
    return scalarUpdate(h_row, gnss_height_m - x_(0), gnss_sigma_m, true);
}

double BarometerHeightFilter::heightSigma() const {
    return std::sqrt(std::max(0.0, p_(0, 0)));
}

double BarometerHeightFilter::biasSigma() const {
    return std::sqrt(std::max(0.0, p_(1, 1)));
}

}  // namespace barometer
}  // namespace libgnss
