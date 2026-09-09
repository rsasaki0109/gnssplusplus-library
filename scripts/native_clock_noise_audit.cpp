// Inspect the actual linked GTSAM model; no measurements or solver input.
#include <gtsam/linear/NoiseModel.h>
#include <iostream>
#include <cmath>
int main() {
    gtsam::Vector sigmas = gtsam::Vector::Zero(7);
    sigmas(0) = 1.;
    const auto model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    const auto residual = gtsam::Vector::Ones(7).eval();
    const auto whitened = model->whiten(residual);
    std::cout << "constrained=" << model->isConstrained()
              << " tail_whitened_norm=" << whitened.tail(6).norm()
              << " squared_distance=" << model->squaredMahalanobisDistance(residual)
              << '\n';
    if (!model->isConstrained() || whitened.tail(6).norm() <= 0. ||
        !std::isfinite(model->squaredMahalanobisDistance(residual))) return 1;
    gtsam::Vector tail_only = residual;
    tail_only(0) = 0.;
    if (!(model->squaredMahalanobisDistance(tail_only) > 0.)) return 1;
}
