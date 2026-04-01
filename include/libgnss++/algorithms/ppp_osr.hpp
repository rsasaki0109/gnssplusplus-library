#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <map>
#include <string>
#include <vector>

namespace libgnss {

int preferredClasNetworkId(const std::map<std::string, std::string>& atmos_tokens);

std::map<std::string, std::string> selectClasEpochAtmosTokens(
    const SSRProducts& ssr_products,
    const std::vector<SatelliteId>& satellites,
    const GNSSTime& time,
    const Vector3d& receiver_position);

CLASEpochContext prepareClasEpochContext(
    const ObservationData& obs,
    const NavigationData& nav,
    const SSRProducts& ssr,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
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
    const std::map<std::string, std::string>& atmos_tokens,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
    std::map<SatelliteId, double>& prev_windup,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASSisContinuityInfo>& sis_continuity,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair);

}  // namespace libgnss
