#pragma once

#include <libgnss++/algorithms/ppp_shared.hpp>
#include "../core/navigation.hpp"
#include "../core/observation.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace libgnss {
namespace ppp_atmosphere {

struct ClasGridReference {
    double dlat_deg = 0.0;
    double dlon_deg = 0.0;
    size_t residual_index = 0;
    int network_id = 0;
    int grid_no = 0;
};

bool parseAtmosTokenDouble(const std::map<std::string, std::string>& atmos_tokens,
                           const std::string& key,
                           double& value);

bool parseAtmosTokenInt(const std::map<std::string, std::string>& atmos_tokens,
                        const std::string& key,
                        int& value);

bool parseAtmosListMeanValue(const std::map<std::string, std::string>& atmos_tokens,
                             const std::string& key,
                             double& value);

bool parseAtmosListValueAtIndex(const std::map<std::string, std::string>& atmos_tokens,
                                const std::string& key,
                                size_t index,
                                double& value);

bool resolveClasGridReference(const std::map<std::string, std::string>& atmos_tokens,
                              const Vector3d& receiver_position,
                              ClasGridReference& reference);

double atmosphericTroposphereCorrectionMeters(
    const std::map<std::string, std::string>& atmos_tokens,
    const Vector3d& receiver_position,
    const GNSSTime& time,
    double elevation,
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy value_policy =
        ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::FULL_COMPOSED,
    ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy subtype12_value_policy =
        ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy::FULL,
    ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy residual_sampling_policy =
        ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy::INDEXED_OR_MEAN);

double atmosphericStecTecu(const std::map<std::string, std::string>& atmos_tokens,
                           const SatelliteId& satellite,
                           const Vector3d& receiver_position,
                           ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy value_policy =
                               ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::FULL_COMPOSED,
                           ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy subtype12_value_policy =
                               ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy::FULL,
                           ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy residual_sampling_policy =
                               ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy::INDEXED_OR_MEAN);

double ionosphereDelayMetersFromTecu(SignalType signal,
                                     const Ephemeris* eph,
                                     double stec_tecu);

double observationIonosphereDelayMeters(const Ephemeris* eph,
                                        SignalType primary_signal,
                                        SignalType secondary_signal,
                                        bool use_ionosphere_free,
                                        double stec_tecu,
                                        double coeff_primary,
                                        double coeff_secondary);

}  // namespace ppp_atmosphere
}  // namespace libgnss
