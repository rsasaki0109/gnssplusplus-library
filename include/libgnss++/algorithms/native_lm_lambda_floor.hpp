#pragma once
#include <algorithm>
#include <stdexcept>

namespace libgnss {
// Opt-in numerical experiment, not a measurement or convergence rule.
template <class Params>
void applyNativeLmLambdaFloor(Params& params, bool enabled, bool native_scope) {
    if (!enabled) return;
    if (!native_scope) throw std::invalid_argument("Native LM floor requires native raw C7 scope");
    constexpr double floor = 1e-8;
    if (!(params.lambdaInitial >= floor) || !(params.lambdaUpperBound >= floor))
        throw std::invalid_argument("Native LM floor exceeds initial/upper lambda");
    params.lambdaLowerBound = std::max(params.lambdaLowerBound, floor);
}
}
