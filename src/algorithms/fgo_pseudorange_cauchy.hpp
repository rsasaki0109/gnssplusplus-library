#pragma once

// Experimental undifferenced-P noise only. Not selected by any solver yet.
#include <gtsam/linear/NoiseModel.h>
#include <cmath>
#include <stdexcept>

namespace libgnss::fgo_gtsam_internal {
inline gtsam::SharedNoiseModel makePseudorangeCauchyNoise(double sigma_m,
                                                        double scale_sigma) {
    if (!std::isfinite(sigma_m) || sigma_m<=0 ||
        !std::isfinite(scale_sigma) || scale_sigma<=0)
        throw std::invalid_argument("Cauchy pseudorange noise requires finite positive sigma and scale");
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(scale_sigma),
        gtsam::noiseModel::Isotropic::Sigma(1,sigma_m));
}
}
