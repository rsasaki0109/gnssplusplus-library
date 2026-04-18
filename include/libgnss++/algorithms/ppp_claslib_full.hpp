#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>

#include <map>
#include <set>

namespace libgnss::ppp_claslib_full {

constexpr int kClasMaxSat = 75;
constexpr int kClasNfreq = 3;
constexpr int kClasNp = 3;
constexpr int kClasNi = kClasMaxSat;
constexpr int kClasAmbStart = kClasNp + kClasNi;
constexpr int kClasNx = kClasAmbStart + kClasMaxSat * kClasNfreq;

struct ClaslibRtkState {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    std::map<SatelliteId, int> ionosphere_indices;
    std::map<SatelliteId, int> ambiguity_indices;
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion_compensation;
    std::map<SatelliteId, CLASSisContinuityInfo> sis_continuity;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> phase_bias_repair;
    std::map<SatelliteId, double> windup_cache;
    GNSSTime last_time;
    bool has_last_time = false;
    bool initialized = false;
    int epoch_count = 0;
    double last_ar_ratio = 0.0;
    int last_fixed_ambiguities = 0;
};

struct EpochResult {
    PositionSolution solution;
    bool updated = false;
    bool fixed = false;
    int measurement_count = 0;
    int active_state_count = 0;
};

void reset(ClaslibRtkState& state);

EpochResult runEpoch(const ObservationData& obs,
                     const NavigationData& nav,
                     const SSRProducts& ssr,
                     ClaslibRtkState& rtk,
                     const ppp_shared::PPPConfig& config);

}  // namespace libgnss::ppp_claslib_full
