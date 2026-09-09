#pragma once
#include <Eigen/Core>
#include <cmath>
#include <optional>
#include <vector>

namespace libgnss::tdcp_endpoint_covariance {
inline std::optional<double> pairSigma(double previous,double current) {
    if(!std::isfinite(previous) || !std::isfinite(current) || previous<=0. || current<=0.)
        return std::nullopt;
    const double sigma=std::hypot(previous,current);
    if(!std::isfinite(sigma) || sigma<=0.) return std::nullopt;
    return sigma;
}
// One continuous, same-satellite/signal arc. Independent endpoint errors
// with supplied standard deviations in metres are an explicit hypothesis,
// not implied by Android uncertainty metadata or post-fit correlations.
// For d_i = L_{i+1}-L_i, C = D diag(sigma^2) D'. No fitted correlation.
// Caller must split gaps/slips and preserve distinct endpoint identities.
inline std::optional<Eigen::MatrixXd> make(const std::vector<double>& sigma_m) {
    if(sigma_m.size()<2) return std::nullopt;
    std::vector<double> variance;
    variance.reserve(sigma_m.size());
    for(double sigma:sigma_m) {
        const double value=sigma*sigma;
        if(!std::isfinite(sigma) || sigma<=0. || !std::isfinite(value) || value<=0.)
            return std::nullopt;
        variance.push_back(value);
    }
    const auto n=static_cast<Eigen::Index>(variance.size()-1);
    Eigen::MatrixXd covariance=Eigen::MatrixXd::Zero(n,n);
    for(Eigen::Index i=0;i<n;++i) {
        covariance(i,i)=variance[i]+variance[i+1];
        if(i+1<n) covariance(i,i+1)=covariance(i+1,i)=-variance[i+1];
    }
    if(!covariance.allFinite()) return std::nullopt;
    return covariance;
}
}
