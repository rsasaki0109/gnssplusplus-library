#pragma once

#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/observation.hpp>

#include <functional>
#include <map>

namespace libgnss::ppp_claslib_zdres {

using TropMappingFunction = ppp_clas::TropMappingFunction;
using AmbiguityResetFunction = ppp_clas::AmbiguityResetFunction;

ppp_clas::MeasurementBuildResult buildEpochMeasurementsPortedZdres(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function = {},
    bool debug_enabled = false);

}  // namespace libgnss::ppp_claslib_zdres
