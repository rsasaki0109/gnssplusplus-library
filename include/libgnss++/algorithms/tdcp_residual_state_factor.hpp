#pragma once

// GTSAM research factor: preserve a scalar TDCP factor's original geometry,
// clock units, Jacobians and noise model; append -alpha * residual_slant_change.
// Not enabled by any production selector. Pair identity and exactly one prior
// per shared state must be enforced by the graph builder, not this wrapper.
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace libgnss::tdcp_frequency {

class ResidualStateFactor final : public gtsam::NoiseModelFactor {
    std::shared_ptr<const gtsam::NoiseModelFactor> original_;
    gtsam::Key residual_key_;
    double alpha_;

    static gtsam::KeyVector extendedKeys(
        const std::shared_ptr<const gtsam::NoiseModelFactor>& original,
        gtsam::Key residual_key, double alpha) {
        if (!original || !original->noiseModel() || original->dim() != 1 ||
            !std::isfinite(alpha) || alpha <= 0.0) {
            throw std::invalid_argument("Residual TDCP state requires a scalar factor and positive coefficient");
        }
        auto keys = original->keys();
        if (std::find(keys.begin(), keys.end(), residual_key) != keys.end()) {
            throw std::invalid_argument("Residual TDCP state key collides with original factor");
        }
        keys.push_back(residual_key);
        return keys;
    }

public:
    ResidualStateFactor(std::shared_ptr<const gtsam::NoiseModelFactor> original,
                        gtsam::Key residual_key, double alpha)
        : gtsam::NoiseModelFactor(original ? original->noiseModel() : nullptr,
                                 extendedKeys(original, residual_key, alpha)),
          original_(std::move(original)), residual_key_(residual_key), alpha_(alpha) {}

    bool active(const gtsam::Values& values) const override {
        return original_->active(values);
    }

    bool equals(const gtsam::NonlinearFactor& other, double tolerance = 1e-9) const override {
        const auto* factor = dynamic_cast<const ResidualStateFactor*>(&other);
        return factor && residual_key_ == factor->residual_key_ &&
               std::abs(alpha_ - factor->alpha_) <= tolerance &&
               original_->equals(*factor->original_, tolerance) &&
               gtsam::NoiseModelFactor::equals(other, tolerance);
    }

    using gtsam::NoiseModelFactor::unwhitenedError;
    gtsam::Vector unwhitenedError(const gtsam::Values& values,
                                  gtsam::OptionalMatrixVecType jacobians = nullptr) const override {
        std::vector<gtsam::Matrix> original_jacobians(original_->size());
        auto residual = original_->unwhitenedError(
            values, jacobians ? &original_jacobians : nullptr);
        const double state = values.at<double>(residual_key_);
        if (residual.size() != 1 || !residual.allFinite() || !std::isfinite(state)) {
            throw std::invalid_argument("Nonfinite or nonscalar residual TDCP state factor");
        }
        residual(0) -= alpha_ * state;
        if (!residual.allFinite()) throw std::invalid_argument("Residual TDCP state overflow");
        if (jacobians) {
            *jacobians = std::move(original_jacobians);
            jacobians->push_back(gtsam::Matrix::Constant(1, 1, -alpha_));
        }
        return residual;
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::make_shared<ResidualStateFactor>(*this);
    }
};
}  // namespace libgnss::tdcp_frequency
