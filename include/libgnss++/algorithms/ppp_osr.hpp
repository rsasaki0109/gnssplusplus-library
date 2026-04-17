#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <map>
#include <string>
#include <vector>

namespace libgnss {

int preferredClasNetworkId(const std::map<std::string, std::string>& atmos_tokens);

const char* clasPhaseContinuityPolicyName(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy);

const char* clasPhaseBiasValuePolicyName(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy);

const char* clasPhaseBiasReferenceTimePolicyName(
    ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy policy);

const char* clasSsrTimingPolicyName(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy);

const char* clasExpandedValueConstructionPolicyName(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy);

bool usesClasPhaseBiasTerms(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy);

bool usesClasRawPhaseBiasValues(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy);

bool usesClasPhaseCompensationValues(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy);

GNSSTime selectClasPhaseBiasReferenceTime(
    ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy policy,
    const GNSSTime& phase_bias_reference_time,
    const GNSSTime& clock_reference_time,
    const GNSSTime& observation_time);

bool usesClasSisContinuity(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy);

bool usesClasPhaseBiasRepair(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy);

bool usesClasClockBoundPhaseBias(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy);

bool usesClasClockBoundAtmos(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy);

bool usesClasExpandedPolynomialTerms(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy);

bool usesClasExpandedResidualTerms(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy);

std::map<std::string, std::string> selectClasEpochAtmosTokens(
    const SSRProducts& ssr_products,
    const std::vector<SatelliteId>& satellites,
    const GNSSTime& time,
    const Vector3d& receiver_position,
    const ppp_shared::PPPConfig& config);

CLASEpochContext prepareClasEpochContext(
    const ObservationData& obs,
    const NavigationData& nav,
    const SSRProducts& ssr,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, double>& prev_windup,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASSisContinuityInfo>& sis_continuity,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair);

/// Compute OSR corrections for all visible satellites.
/// This is the C++ equivalent of CLASLIB's cssr2osr.c processing pipeline.
///
/// @param obs          Raw observations for this epoch
/// @param nav          Navigation data (broadcast ephemeris)
/// @param ssr          SSR products (orbit/clock/bias corrections)
/// @param atmos_tokens Per-epoch atmosphere tokens from CLAS L6
/// @param receiver_pos Approximate receiver position (ECEF)
/// @param receiver_clk Approximate receiver clock bias (meters)
/// @param trop_zenith  Zenith tropospheric delay (meters)
/// @param prev_windup  Previous epoch's wind-up values per satellite
/// @return Vector of OSR corrections, one per satellite
std::vector<OSRCorrection> computeOSR(
    const ObservationData& obs,
    const NavigationData& nav,
    const SSRProducts& ssr,
    const std::map<std::string, std::string>& epoch_atmos_tokens,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, double>& prev_windup,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASSisContinuityInfo>& sis_continuity,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    const Vector3d& receiver_tide_displacement = Vector3d::Zero());

}  // namespace libgnss
