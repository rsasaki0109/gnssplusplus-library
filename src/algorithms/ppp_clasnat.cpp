#include <libgnss++/algorithms/ppp_clasnat.hpp>

#include <libgnss++/algorithms/ppp_claslib_full.hpp>

namespace libgnss::ppp_clasnat {

struct RtkState::Impl {
    ppp_claslib_full::ClaslibRtkState full;
};

namespace {

void mirrorFromImpl(RtkState& state) {
    if (!state.impl_) {
        return;
    }
    const auto& full = state.impl_->full;
    state.x = full.x;
    state.P = full.P;
    state.xa = full.xa;
    state.Pa = full.Pa;
    state.last_time = full.last_time;
    state.has_last_time = full.has_last_time;
    state.initialized = full.initialized;
    state.last_ar_ratio = full.last_ar_ratio;
    state.last_fixed_ambiguities = full.last_fixed_ambiguities;
}

ppp_shared::PPPConfig clasnatConfig(const ppp_shared::PPPConfig& base) {
    ppp_shared::PPPConfig config = base;
    config.use_ported_clasnat = true;
    config.use_ported_full = true;
    config.use_clas_osr_filter = true;
    config.wlnl_strict_claslib_parity = true;
    config.use_ionosphere_free = false;
    config.estimate_ionosphere = true;
    config.estimate_troposphere = false;
    config.ar_method = ppp_shared::PPPConfig::ARMethod::DD_PER_FREQ;
    config.clas_epoch_policy = ppp_shared::PPPConfig::ClasEpochPolicy::STRICT_OSR;
    config.clas_correction_application_policy =
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
    config.clas_atmos_selection_policy =
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_FIRST;
    config.clas_decouple_clock_position = false;
    config.initial_ionosphere_variance = 1e-4;
    config.process_noise_ionosphere = 1e-6;
    config.apply_tides_to_receiver_position = true;
    config.apply_tide_as_osr = false;
    return config;
}

}  // namespace

RtkState::RtkState() : impl_(std::make_unique<Impl>()) {}

RtkState::~RtkState() = default;

RtkState::RtkState(RtkState&&) noexcept = default;

RtkState& RtkState::operator=(RtkState&&) noexcept = default;

void reset(RtkState& state) {
    state = RtkState{};
}

EpochResult runEpoch(const ObservationData& obs,
                     const libgnss::NavigationData& nav,
                     const SSRProducts& ssr,
                     RtkState& rtk,
                     const ppp_shared::PPPConfig& config) {
    if (!rtk.impl_) {
        rtk.impl_ = std::make_unique<RtkState::Impl>();
    }

    const ppp_shared::PPPConfig native_config = clasnatConfig(config);
    const auto full_result = ppp_claslib_full::runEpoch(
        obs, nav, ssr, rtk.impl_->full, native_config);
    mirrorFromImpl(rtk);

    EpochResult result;
    result.solution = full_result.solution;
    result.updated = full_result.updated;
    result.fixed = full_result.fixed;
    result.measurement_count = full_result.measurement_count;
    result.active_state_count = full_result.active_state_count;
    return result;
}

}  // namespace libgnss::ppp_clasnat
