#pragma once

#include <libgnss++/algorithms/native_imu_refinement_rebuild.hpp>

namespace libgnss::native_gnss_initial {

struct Input {
    native_imu_refinement::Handoff states;
    std::vector<raw_p_seed::RawPNoDopplerSeed> seeds;
    double interval_s = 0.0;
};

// The source Gpos.gradient uses a scalar observation interval, even where
// individual UTC differences vary. Build it from the same-run native position
// seeds, never from archived posbl coordinates or an optimized/fake IMU result.
inline Input fromNativeSeeds(const std::vector<ObservationData>& raw,
                             const FGOProcessor::FGOProblem& source) {
    const auto n=raw.size();
    if (n<2 || source.epochs.size()!=n || source.native_raw_p_no_doppler_seeds.size()!=n)
        throw std::invalid_argument("Cold GNSS rebuild requires complete same-run native seeds");
    Input out; out.states.observations=raw; out.seeds=source.native_raw_p_no_doppler_seeds;
    std::vector<double> intervals;
    for (std::size_t i=0;i<n;++i) {
        const auto& r=raw[i]; const auto& e=source.epochs[i]; const auto& seed=out.seeds[i];
        if (r.raw_source_index!=i || e.raw_source_index!=i || seed.raw_source_index!=i ||
            seed.epoch_index!=i || r.raw_utc_time_millis<=0 ||
            e.raw_utc_time_millis!=r.raw_utc_time_millis || seed.raw_utc_time_millis!=r.raw_utc_time_millis ||
            (e.time-r.time)!=0.0 || (seed.time-r.time)!=0.0 ||
            (i && (r.raw_utc_time_millis<=raw[i-1].raw_utc_time_millis || !(r.time-raw[i-1].time>0.0))))
            throw std::invalid_argument("Cold GNSS rebuild seed epoch identity mismatch");
        if (seed.status!=raw_p_seed::SeedAdapterStatus::Accepted ||
            !seed.has_position || !seed.has_clock || !seed.has_clock_rate ||
            !seed.c7_clock_mapping_supported || seed.reference_clock_group!=GNSSSystem::GPS ||
            !seed.clock_bias_component_available[0] || !std::isfinite(seed.clock_bias_m) ||
            seed.clock_bias_components_m[0]!=seed.clock_bias_m || !std::isfinite(seed.clock_rate_mps) ||
            !seed.position_ecef.allFinite() || seed.position_ecef.norm()<1e6 || seed.position_ecef.norm()>1e8 ||
            e.position_ecef!=seed.position_ecef || r.receiver_position!=seed.position_ecef ||
            r.receiver_clock_bias!=seed.clock_bias_m/constants::SPEED_OF_LIGHT ||
            r.receiver_clock_drift_mps!=seed.clock_rate_mps)
            throw std::invalid_argument("Cold GNSS rebuild needs native position and clock provenance");
        if (i) intervals.push_back((r.raw_utc_time_millis-raw[i-1].raw_utc_time_millis)/1000.0);
    }
    std::sort(intervals.begin(),intervals.end());
    const auto m=intervals.size()/2;
    out.interval_s=std::round((intervals.size()%2 ? intervals[m] : (intervals[m-1]+intervals[m])*0.5)*100.0)/100.0;
    if (!std::isfinite(out.interval_s) || out.interval_s<=0)
        throw std::invalid_argument("Cold GNSS nominal observation interval is invalid");
    out.states.velocity_ecef_mps.reserve(n);
    for (std::size_t i=0;i<n;++i) {
        const auto left=i ? i-1 : 0;
        const auto right=i+1<n ? i+1 : n-1;
        const Vector3d velocity=(out.seeds[right].position_ecef-out.seeds[left].position_ecef)/
            ((right-left)*out.interval_s);
        if (!velocity.allFinite()) throw std::invalid_argument("Cold GNSS position gradient is nonfinite");
        out.states.velocity_ecef_mps.push_back(velocity);
        out.seeds[i].velocity_ecef_mps=velocity;
        out.seeds[i].has_velocity=true;
    }
    return out;
}

struct Rebuild {
    FGOProcessor::FGOProblem problem;
    native_imu_refinement::DopplerMaskReport doppler_mask;
    double interval_s = 0.0;
};

inline Rebuild rebuild(const std::vector<ObservationData>& raw,
                       const NavigationData& nav,
                       const FGOProcessor::FGOProblem& source,
                       FGOProcessor::FGOConfig config) {
    if (config.use_imu || config.use_spp_seed ||
        !config.use_native_raw_p_ecef_doppler_gnss_first ||
        !config.use_native_source_clock_c0d_factor || !config.use_upstream_observable_quality)
        throw std::invalid_argument("Cold GNSS rebuild requires native GNSS-only ECEF-D/source-clock graph");
    auto input=fromNativeSeeds(raw,source);
    config.native_imu_observation_phase=NativeImuObservationPhase::Initialization;
    config.use_quality_anchor_initialization=false;
    config.retain_sparse_epochs_for_imu=true;
    config.use_undifferenced_doppler_factors=true;
    config.use_corrected_undifferenced_doppler_factors=true;
    config.upstream_absolute_doppler_residual_threshold_mps=20.0;
    config.retain_native_pseudorange_remasking_pool=false;
    auto geometry_config=config;
    geometry_config.native_imu_observation_phase=NativeImuObservationPhase::Legacy;
    geometry_config.use_pseudorange_factors=false;
    geometry_config.use_tdcp_factors=false;
    geometry_config.min_satellites_per_epoch=0;
    auto geometry=FGOProcessor(geometry_config).buildPseudorangeProblem(
        input.states.observations,nav,input.states.velocity_ecef_mps);
    Rebuild out; out.interval_s=input.interval_s;
    out.doppler_mask=native_imu_refinement::maskDoppler(input.states,geometry,input.interval_s,20.0);
    out.problem=FGOProcessor(config).buildPseudorangeProblem(
        input.states.observations,nav,input.states.velocity_ecef_mps);
    if (out.problem.epochs.size()!=raw.size())
        throw std::invalid_argument("Cold GNSS rebuild changed raw epoch coverage");
    for (std::size_t i=0;i<raw.size();++i) {
        const auto& epoch=out.problem.epochs[i];
        if (epoch.raw_source_index!=i || epoch.raw_utc_time_millis!=raw[i].raw_utc_time_millis ||
            (epoch.time-raw[i].time)!=0.0 || epoch.position_ecef!=input.seeds[i].position_ecef)
            throw std::invalid_argument("Cold GNSS rebuilt epoch differs from native seed");
    }
    out.problem.native_raw_p_no_doppler_seeds=std::move(input.seeds);
    out.problem.clock_jumps=source.clock_jumps;
    return out;  // New P rows are still uncorrected; caller applies base once.
}
}  // namespace libgnss::native_gnss_initial
