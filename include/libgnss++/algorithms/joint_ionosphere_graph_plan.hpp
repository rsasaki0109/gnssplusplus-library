#pragma once
#include <libgnss++/algorithms/code_ionosphere_state_factor.hpp>
#include <libgnss++/algorithms/tdcp_ionosphere_endpoints_factor.hpp>
#include <libgnss++/algorithms/ionosphere_temporal_plan.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <set>

namespace libgnss::code_ionosphere {
struct CodeBinding { std::size_t factor, epoch; double coefficient; };
struct TdcpBinding { std::size_t factor, previous, current; double a_previous, a_current; };
struct GraphPlan {
    std::vector<std::pair<std::size_t,gtsam::NonlinearFactor::shared_ptr>> replacements;
    gtsam::NonlinearFactorGraph priors;
    gtsam::Values states;
};
// Validate and stage everything WITHOUT mutating the caller graph/Values.
// Caller replaces indexed factors (never appends originals plus wrappers).
inline GraphPlan planGraph(const gtsam::NonlinearFactorGraph& graph,
                           const gtsam::Values& initial,
                           const std::vector<GNSSTime>& times,
                           const std::vector<bool>& resets,
                           const std::vector<gtsam::Key>& keys,
                           const std::vector<CodeBinding>& codes,
                           const std::vector<TdcpBinding>& carriers,
                           double anchor_sigma,double density,double max_gap) {
    if(keys.size()!=times.size() || !std::isfinite(anchor_sigma) || anchor_sigma<=0)
        throw std::invalid_argument("Invalid joint ionosphere state configuration");
    const auto temporal=residual_ionosphere::temporalPlan(times,resets,max_gap,density);
    std::set<gtsam::Key> unique(keys.begin(),keys.end());
    if(unique.size()!=keys.size()) throw std::invalid_argument("Duplicate ionosphere keys");
    for(auto key:keys) if(initial.exists(key))
        throw std::invalid_argument("Ionosphere initial key collision");
    for(const auto& f:graph) if(f) for(auto key:f->keys()) if(unique.count(key))
        throw std::invalid_argument("Ionosphere graph key collision");
    GraphPlan plan;
    std::set<std::size_t> replaced;
    const auto original=[&](std::size_t index) {
        if(index>=graph.size() || !replaced.insert(index).second)
            throw std::invalid_argument("Invalid or duplicate ionosphere factor binding");
        auto factor=std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[index]);
        if(!factor) throw std::invalid_argument("Non-noise ionosphere factor binding");
        return factor;
    };
    for(const auto& b:codes) {
        if(b.epoch>=keys.size()) throw std::invalid_argument("Invalid code epoch binding");
        plan.replacements.emplace_back(b.factor,std::make_shared<ResidualStateFactor>(
            original(b.factor),keys[b.epoch],b.coefficient));
    }
    for(const auto& b:carriers) {
        if(b.previous>=b.current || b.current>=keys.size())
            throw std::invalid_argument("Invalid carrier epoch binding");
        plan.replacements.emplace_back(b.factor,std::make_shared<TdcpEndpointsFactor>(
            original(b.factor),keys[b.previous],keys[b.current],b.a_previous,b.a_current));
    }
    for(auto key:keys) plan.states.insert(key,0.);
    for(auto epoch:temporal.anchors)
        plan.priors.addPrior(keys[epoch],0.,gtsam::noiseModel::Isotropic::Sigma(1,anchor_sigma));
    for(const auto& edge:temporal.edges)
        plan.priors.emplace_shared<gtsam::BetweenFactor<double>>(
            keys[edge.previous],keys[edge.current],0.,
            gtsam::noiseModel::Isotropic::Sigma(1,edge.sigma_m));
    return plan;
}
}
