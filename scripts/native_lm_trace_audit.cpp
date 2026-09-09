#include "../src/algorithms/fgo_gtsam_internal.hpp"
#include <iostream>
class InvalidDerivativeProbe : public gtsam::NoiseModelFactorN<gtsam::Vector> {
public:
    InvalidDerivativeProbe() : gtsam::NoiseModelFactorN<gtsam::Vector>(
        gtsam::noiseModel::Isotropic::Sigma(1, 1.), 0) {}
    gtsam::Vector evaluateError(const gtsam::Vector& x,
        gtsam::OptionalMatrixType H) const override {
        if (H) {
            *H = gtsam::Matrix::Zero(1, 2);
            // Deliberately invalid derivative to exercise the caught failure
            // diagnostic. This is not a physical GNSS model or rank test.
            (*H)(0, 0) = std::numeric_limits<double>::quiet_NaN();
        }
        return gtsam::Vector::Constant(1, x(0)-1.);
    }
};
int main() {
    using Optimizer = libgnss::fgo_gtsam_internal::GtsamLmActiveSolveOptimizer;
    const std::string trace = "0 inf 0 1e-05 0 0.01\n1 nan 0 1e-04 1 0.01\n";
    const auto attempts = Optimizer::parseAttemptsCountForTesting(trace);
    const auto failures = Optimizer::parseLinearFailuresForTesting(trace);
    std::cout << "attempts=" << attempts << " linear_failures=" << failures << '\n';
    if (attempts != 2 || failures != 1) return 1;
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<InvalidDerivativeProbe>();
    gtsam::Values initial;
    initial.insert(0, gtsam::Vector::Zero(2).eval());
    gtsam::LevenbergMarquardtParams params;
    params.setLinearSolverType("MULTIFRONTAL_QR");
    // Deliberately invalid-derivative control. Zero upper bound terminates
    // the failed trial immediately; this is never a production setting.
    params.lambdaInitial = 0.;
    params.lambdaUpperBound = 0.;
    params.maxIterations = 3;
    params.setDiagonalDamping(false);
    Optimizer optimizer(graph, initial, params, true, true);
    const auto result = optimizer.optimize();
    const auto& telemetry = optimizer.telemetry();
    std::cout << "trylambda_failures=" << telemetry.indeterminate_linear_solve_count
              << " final_cost=" << telemetry.final_cost << '\n';
    return telemetry.indeterminate_linear_solve_count > 0 &&
           std::isfinite(result.at<gtsam::Vector>(0)(0)) ? 0 : 2;
}
