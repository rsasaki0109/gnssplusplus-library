#pragma once
#include <libgnss++/algorithms/native_imu_refinement_handoff.hpp>
#include <libgnss++/algorithms/native_gnss_imu_initial_handoff.hpp>
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
#include <map>
#include <set>
#include <tuple>

namespace libgnss::native_imu_refinement {
struct DopplerMaskReport { std::size_t candidates = 0, retained = 0, rejected = 0; };

// Input D rows are corrected raw measurements, evaluated at the warm position;
// they are not optimized velocities. Apply the source residual threshold BEFORE
// rebuilding adjacent P/D and L/D masks. This also preserves NaNs for misses.
inline DopplerMaskReport maskDoppler(Handoff& handoff,
                                    const FGOProcessor::FGOProblem& geometry,
                                    double interval_s, double threshold_mps = 3.0) {
    namespace upstream = observable_upstream;
    const auto n = handoff.observations.size();
    if (!n || geometry.epochs.size() != n || handoff.velocity_ecef_mps.size() != n ||
        !std::isfinite(interval_s) || interval_s <= 0.0 ||
        !std::isfinite(threshold_mps) || threshold_mps <= 0.0)
        throw std::invalid_argument("Invalid refinement Doppler geometry coverage or interval");
    for (std::size_t i=0; i<n; ++i) {
        const auto& a=handoff.observations[i]; const auto& b=geometry.epochs[i];
        if (a.raw_source_index!=i || b.raw_source_index!=i ||
            a.raw_utc_time_millis!=b.raw_utc_time_millis || (a.time-b.time)!=0.0 ||
            !handoff.velocity_ecef_mps[i].allFinite())
            throw std::invalid_argument("Refinement Doppler epoch identity mismatch");
    }
    using Key = std::tuple<std::size_t,SatelliteId,SignalType>;
    std::set<Key> raw_keys;
    for (std::size_t i=0;i<n;++i)
        for (const auto& obs:handoff.observations[i].observations)
            if (!raw_keys.emplace(i,obs.satellite,obs.signal).second)
                throw std::invalid_argument("Duplicate raw observation identity in refinement");
    std::map<Key,bool> accepted;
    for (const auto& row : geometry.undifferenced_doppler_factors) {
        if (row.epoch_index>=n || !row.los.allFinite() ||
            std::abs(row.los.norm()-1.0)>1e-6 || !std::isfinite(row.residual_mps))
            throw std::invalid_argument("Invalid refinement raw Doppler row");
        const auto i=row.epoch_index;
        if (!raw_keys.count(Key{i,row.satellite,row.signal}))
            throw std::invalid_argument("Refinement Doppler row has no raw observation");
        const double residual=row.residual_mps-row.los.dot(handoff.velocity_ecef_mps[i]);
        const bool keep=upstream::acceptsAbsoluteDopplerResidual(
            residual,handoff.observations[i].receiver_clock_drift_mps,interval_s,threshold_mps);
        if (!accepted.emplace(Key{i,row.satellite,row.signal},keep).second)
            throw std::invalid_argument("Duplicate refinement Doppler identity");
    }
    DopplerMaskReport report;
    for (std::size_t i=0; i<n; ++i) {
        for (auto& obs : handoff.observations[i].observations) {
            if (!std::isfinite(obs.doppler)) continue;
            ++report.candidates;
            const auto found=accepted.find(Key{i,obs.satellite,obs.signal});
            if (found!=accepted.end() && found->second) ++report.retained;
            else { obs.doppler=std::numeric_limits<double>::quiet_NaN(); ++report.rejected; }
        }
    }
    return report;
}

struct Rebuild {
    FGOProcessor::FGOProblem problem;
    DopplerMaskReport doppler_mask;
};

// Reconstruct receiver-dependent orbit geometry, atmosphere, SNR weights and
// residual/adjacent masks from raw observations. Base corrections are deliberately
// not applied here: the caller applies its existing same-run base model exactly
// once to the NEW rows before solving.
inline Rebuild rebuildHandoff(const std::vector<ObservationData>& raw,
                       const NavigationData& nav,
                       const FGOProcessor::FGOProblem& source,
                       Handoff handoff,
                       const std::vector<Vector3d>& velocity_nav,
                       const std::vector<double>& clock_drift_mps,
                       FGOProcessor::FGOConfig config) {
    if (!config.use_native_phase171_raw_p_no_doppler_imu_main ||
        !config.use_upstream_observable_quality || !config.use_imu ||
        !config.pose3_lever_arm_body_m.allFinite() || config.pose3_lever_arm_body_m.norm()!=0.0)
        throw std::invalid_argument("Refinement rebuild requires Phase171 native IMU quality graph with zero lever arm");
    if (raw.empty() || velocity_nav.size()!=raw.size() || clock_drift_mps.size()!=raw.size() ||
        !std::all_of(velocity_nav.begin(),velocity_nav.end(),[](const auto& v){return v.allFinite();}) ||
        !std::all_of(clock_drift_mps.begin(),clock_drift_mps.end(),[](double v){return std::isfinite(v);}))
        throw std::invalid_argument("IMU rebuild requires complete finite velocity and drift states");
    config.use_spp_seed=false;
    config.use_quality_anchor_initialization=false;
    config.retain_sparse_epochs_for_imu=true;
    config.use_undifferenced_doppler_factors=true;
    config.use_corrected_undifferenced_doppler_factors=true;
    config.upstream_absolute_doppler_residual_threshold_mps=
        nativeImuDopplerResidualThreshold(config.native_imu_observation_phase);
    auto geometry_config=config;
    geometry_config.native_imu_observation_phase=NativeImuObservationPhase::Legacy;
    geometry_config.use_upstream_observable_quality=true;
    geometry_config.use_upstream_absolute_doppler_residual_screen=false;
    geometry_config.retain_native_pseudorange_remasking_pool=false;
    geometry_config.use_pseudorange_factors=false;
    geometry_config.use_tdcp_factors=false;
    geometry_config.min_satellites_per_epoch=0;
    geometry_config.min_snr_dbhz=config.upstream_min_snr_dbhz;
    geometry_config.min_elevation_deg=config.upstream_min_elevation_deg;
    auto geometry=FGOProcessor(geometry_config).buildPseudorangeProblem(
        handoff.observations,nav,handoff.velocity_ecef_mps);
    std::vector<double> intervals;
    for (std::size_t i=1;i<raw.size();++i)
        intervals.push_back((raw[i].raw_utc_time_millis-raw[i-1].raw_utc_time_millis)/1000.0);
    if (intervals.empty()) throw std::invalid_argument("Refinement needs at least two epochs");
    std::sort(intervals.begin(),intervals.end());
    const auto m=intervals.size()/2;
    const double median=intervals.size()%2 ? intervals[m] : (intervals[m-1]+intervals[m])*0.5;
    Rebuild out;
    out.doppler_mask=maskDoppler(handoff,geometry,std::round(median*100.0)/100.0,
                               config.upstream_absolute_doppler_residual_threshold_mps);
    out.problem=FGOProcessor(config).buildPseudorangeProblem(
        handoff.observations,nav,handoff.velocity_ecef_mps);
    if (out.problem.epochs.size()!=source.epochs.size())
        throw std::invalid_argument("Refinement rebuild changed raw epoch coverage");
    for (std::size_t i=0;i<source.epochs.size();++i) {
        const auto& a=source.epochs[i]; const auto& b=out.problem.epochs[i];
        if (a.raw_source_index!=b.raw_source_index || a.raw_utc_time_millis!=b.raw_utc_time_millis ||
            (a.time-b.time)!=0.0)
            throw std::invalid_argument("Refinement rebuild changed epoch identity");
    }
    out.problem.imu=source.imu;
    out.problem.imu.refinement_velocity_seeds_nav=velocity_nav;
    out.problem.imu.stop_velocity_seeds_nav=velocity_nav;
    out.problem.imu.init_velocity_nav=velocity_nav.front();
    out.problem.imu.epoch_heading_attitudes_body_to_nav=handoff.attitude_body_to_nav;
    out.problem.imu.epoch_heading_attitude_times.clear();
    for (const auto& epoch:source.epochs) out.problem.imu.epoch_heading_attitude_times.push_back(epoch.time);
    out.problem.imu.init_attitude_body_to_nav=handoff.attitude_body_to_nav.front();
    out.problem.imu.init_accel_bias.setZero();
    out.problem.imu.init_gyro_bias.setZero();
    out.problem.native_source_clock_c0d_gnss_first_c_handoff_m=handoff.clock_components_m;
    out.problem.native_source_clock_c0d_gnss_first_d_handoff_mps=clock_drift_mps;
    out.problem.clock_jumps=source.clock_jumps;
    if (config.use_native_phase213_main_doppler)
        out.problem.native_phase213_main_doppler_rows=std::move(out.problem.undifferenced_doppler_factors);
    out.problem.undifferenced_doppler_factors.clear();
    return out;
}

inline Rebuild rebuild(const std::vector<ObservationData>& raw,
                       const NavigationData& nav,
                       const FGOProcessor::FGOProblem& source,
                       const FGOProcessor::FGOResult& result,
                       FGOProcessor::FGOConfig config) {
    return rebuildHandoff(raw,nav,source,fromResult(raw,source,result),
                          result.epoch_velocity_nav_mps,result.epoch_clock_drift_mps,config);
}

inline Rebuild rebuildInitial(const std::vector<ObservationData>& raw,
                              const NavigationData& nav,
                              const FGOProcessor::FGOProblem& source,
                              const FGOProcessor::FGOResult& gnss_result,
                              FGOProcessor::FGOConfig config) {
    if (config.native_imu_observation_phase!=NativeImuObservationPhase::Initialization)
        throw std::invalid_argument("Initial IMU rebuild requires the initialization observation phase");
    return rebuildHandoff(raw,nav,source,fromGnssResult(raw,source,gnss_result),
                          source.imu.stop_velocity_seeds_nav,gnss_result.epoch_clock_drift_mps,config);
}
} // namespace libgnss::native_imu_refinement
