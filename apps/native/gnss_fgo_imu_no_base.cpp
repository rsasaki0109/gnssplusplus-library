#include <libgnss++/algorithms/stationary_gyro_initializer.hpp>
#include <libgnss++/algorithms/adjacent_residual_moments.hpp>
#include <libgnss++/algorithms/code_edge_candidates.hpp>
#include <libgnss++/algorithms/pseudorange_position_information.hpp>
// Opt-in, no-base GNSS+IMU entry point for the smartphone research lane.
//
// This intentionally has a small surface: the existing FGO problem builder
// supplies no-base undifferenced pseudorange factors, while the GTSAM backend
// supplies the real Pose3/velocity/bias chain and CombinedImuFactor.  The
// production gnss_fgo defaults are not changed.  Android raw axes are never
// silently treated as body axes: the frozen taroz mounting rotation is
// applied explicitly after the raw adapter has converted only timestamps and
// stream alignment.

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/pseudorange_remasking.hpp>
#include <libgnss++/algorithms/raw_p_seed.hpp>
#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/phase126_raw_base_compound.hpp>
#include <libgnss++/algorithms/source_pseudorange_miss_mask.hpp>
#include <libgnss++/algorithms/carrier_code_leveling.hpp>
#include <libgnss++/algorithms/cn0_doppler_calibration.hpp>
#include <libgnss++/algorithms/pdc_state_bridge.hpp>
#include <libgnss++/algorithms/source_clock_c0d_initializer.hpp>
#include <libgnss++/algorithms/native_raw_p_ecef_doppler_staging.hpp>
#include <libgnss++/algorithms/native_utc_fallback_imu_noise.hpp>
#include <libgnss++/algorithms/native_utc_fallback_imu_offset.hpp>
#include <libgnss++/algorithms/upstream_position_offset.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>

#include <Eigen/Geometry>

#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <typeinfo>
#include <vector>
#include <unistd.h>

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kGpsEpochUnixSeconds = 315964800.0;
constexpr double kGpsUtcLeapSeconds = 18.0;
constexpr double kGravity = 9.80665;
constexpr int kDefaultEpochLimit = 30;
constexpr int kMinEpochLimit = 10;
constexpr int kMaxEpochLimit = 30;
constexpr std::size_t kStationarySamples = 250;
constexpr std::size_t kHeadingWindowEpochs = 25;
constexpr int kRequiredHeadingWindows = 3;
constexpr double kHeadingSpeedMinMps = 2.0;
constexpr double kHeadingSpeedMaxMps = 50.0;
constexpr double kHeadingVerticalSpeedMaxMps = 3.0;
constexpr double kHeadingConsistencyRad = 20.0 * kPi / 180.0;
constexpr double kGravityNormMin = 0.70 * kGravity;
constexpr double kGravityNormMax = 1.30 * kGravity;
constexpr double kGravityNormStdMax = 1.50;
constexpr double kUpstreamImuSyncCoefficient = 0.5;
constexpr double kNativeTdcpMaxGapS = 2.0;
constexpr double kNativeTdcpSigmaM = 0.03;
constexpr double kNativeTdcpCodePhaseJumpThresholdM = 10.0;

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string imu_path;
    std::string android_imu_path;
    std::string android_gnss_path;
    std::string native_base_rinex_path;
    std::string native_base_rinex_sha256;
    std::string out_path;
    std::string summary_path;
    std::string dataset_id = "native-fgo-v2-imu-no-base";
    int max_epochs = kDefaultEpochLimit;
    int skip_epochs = 0;
    bool all_epochs = false;
    bool android_raw_utc_key_contract = false;
    bool android_include_first_native_epoch = false;
    bool android_raw_clock_only = false;
    bool android_utc_wall_clock_fallback = false;
    bool fgo_imu_sparse_recovery = false;
    bool native_pdc_state_bridge = false;
    bool native_pdc_imu_tdcp = false;
    bool native_pdc_imu_tdcp_no_bridge = false;
    bool native_signal_bias_states = false;
    bool native_residual_ionosphere = false;
    bool native_upstream_quality = false;
    // Phase80 opt-in: consume the source P+D observable-quality contract
    // directly in the receiver-only graph.  This is intentionally distinct
    // from native_upstream_quality, whose legacy path first builds a PDC
    // initializer and bridges its states into the quality-enabled graph.
    bool native_source_direct_observable_quality = false;
    // Phase84 opt-in: source-exact ClockFactor_CCDD active C0/D row.  The
    // backend consumes the exact dataset phone identity from dataset_id;
    // route/path text is never used for this gate.
    bool native_source_clock_c0d_factor = false;
    // Phase92 opt-in: store C_i and global ISB_i in metres inside every
    // GTSAM batch consumer, with one explicit seconds conversion at export.
    bool native_source_clock_c0d_meter_state_parity = false;
    // Phase88 opt-in: report active GTSAM LM termination and C0/D
    // conditioning telemetry without changing the solve.
    bool native_source_clock_c0d_active_solve_diagnostic = false;
    // Phase91 opt-in: preserve the standard same-run GNSS-first in-memory
    // position/clock/velocity handoff and seed only main-graph D_i from the
    // exact retained raw Android EpochSeed receiver-clock drift.
    bool native_source_clock_c0d_gnss_first_raw_drift_d_initializer = false;
    // Phase93 opt-in: run the source-meter C0/D graph in GNSS-first and hand
    // optimized C/D states to the existing main meter-state graph.
    bool native_source_clock_c0d_gnss_first_meter_state_handoff = false;
    // Phase101 opt-in: use the official seven-component epoch-local C vector
    // in both GNSS-first and main graphs, with no legacy global ISB states.
    bool native_source_clock_c0d_epoch_vector_parity = false;
    // Phase94 opt-in: capture stage-local C0/D admission and failure
    // telemetry only.  This selector never publishes solution rows and does
    // not alter the graph, observation filters, solver settings, or fallback
    // policy of the Phase93 recipe.
    bool native_source_clock_c0d_phase94_stage_diagnostics = false;
    // Phase96 opt-in: expose main-graph factor-family, linearization, and
    // pinned LM trial diagnostics only.  No solution/accuracy lane is opened.
    bool native_source_clock_c0d_phase96_main_diagnostics = false;
    // Phase97 opt-in: capture singular-system graph/key/rank diagnostics only;
    // solution and accuracy output remain fail-closed.
    bool native_source_clock_c0d_phase97_singular_system_diagnostics = false;
    // Phase98 opt-in: capture compact solver-boundary metadata and typed
    // indeterminate-system nearby-key records only; no solution is exposed.
    bool native_source_clock_c0d_phase98_solver_rank_diagnostic = false;
    // Phase99 opt-in: use multifrontal QR only in the Phase93 source-meter
    // C0/D Pose3+IMU main graph.  GNSS-first and legacy/default remain on
    // multifrontal Cholesky; graph and LM settings are unchanged.
    bool native_source_clock_c0d_phase99_main_multifrontal_qr_solver = false;
    // Phase114 opt-in: seed the Phase99 QR+C0/D main graph directly from the
    // same-run raw Doppler WLS and raw EpochSeed clock/position values.  This
    // is mutually exclusive with the GNSS-first stage and never reads a
    // persisted/precomputed coordinate.
    bool native_direct_wls_ephemeral_c7d_main_seed = false;
    // Phase116 opt-in: collect ordinary TDCP incidence and residual-cost
    // telemetry only.  No carrier/DD/ambiguity state or solver path is
    // changed, and the solution remains withheld by the execution wrapper.
    bool native_phase116_carrier_tdcp_incidence_diagnostic = false;
    // Phase117 opt-in: replace only the sigma on existing ordinary TDCP
    // pairs with the official SNR/type-dependent model.  The library maps
    // source carrier cycles to native metres and fails closed on bad metadata.
    bool native_phase117_tdcp_snr_type_sigma = false;
    // Phase118 opt-in: preserve the Phase112 fixed TDCP sigma and select only
    // the official setting.Type-dependent Huber threshold.  The library owns
    // the final mapping; this option supplies the exact route Type.
    bool native_phase118_official_tdcp_huber_k = false;
    // Phase120 opt-in: ordinary TDCP uses the official resL measurement
    // normalization (raw carrier metres plus satellite-clock metres only).
    // The factor, pair/gate, noise, solver, and non-ordinary carrier paths
    // remain unchanged; legacy/default is false.
    bool native_phase120_official_tdcp_resl_atmosphere_cancellation = false;
    // Phase126 opt-in: admit the all-or-nothing source-complete raw-base
    // correction operator.  No partial source/legacy hybrid is permitted.
    bool native_phase126_raw_base_source_complete = false;
    bool native_paired_epoch_states = false;
    bool native_rover_epoch_states = false;
    bool native_dense_base_smoothing = false;
    bool native_base_mask_only_ablation = false;
    bool native_base_gps_values_only_ablation = false;
    bool native_base_gps_center_ablation = false;
    bool native_main_p_cauchy = false;
    bool native_stationary_gyro_initializer = false;
    bool native_tdcp_no_code_jump_gate = false;
    bool native_sparse_p_staging = false;
    bool native_tdcp_frequency_residual_states = false;
    bool native_lm_lambda_floor = false;
    bool native_nhc_monitor = false;
    bool native_batch_nhc = false;
    bool native_main_code_uncertainty_floor = false;
    bool native_main_code_edge_readmission = false;
    bool native_joint_ionosphere = false;
    bool native_doppler_rotation_rate = false;
    bool native_tdcp_adr_endpoint_sigma = false;
    double joint_ionosphere_anchor = 0., joint_ionosphere_density = 0., joint_ionosphere_gap = 0.;
    // Phase127 opt-in: require exact RINEX-header/broadcast-geph GLONASS
    // channel provenance inside the Phase126 raw-base operator.  The default
    // remains false and no independent/partial selector is supported.
    bool native_phase127_glonass_channel_provenance = false;
    // Phase128 opt-in: retain canonical GLONASS geph data[0..14] positions,
    // distinguish header absence/valid-empty/malformed, and admit each
    // record against the native GPST broadcast query.  This composes with
    // Phase127 only; default and solver/factor recipe remain unchanged.
    bool native_phase128_glonass_provenance_parser_admission = false;
    // Phase129 opt-in: turn an uncertified GLONASS observation into an
    // explicit local miss in both the raw-base stream and the shared FGO
    // factor admission ledger.  It composes with Phase126/127/128 only;
    // certified GLONASS and all non-GLONASS rows remain unchanged.
    bool native_phase129_glonass_local_miss_mask = false;
    // Phase131 opt-in: canonicalize only the raw-base correction join by
    // source-defined physical frequency family.  The estimator's typed
    // signal/factor topology remains unchanged and the default is off.
    bool native_phase131_canonical_correction_band_key = false;
    // Phase135 opt-in: use the complete official fixed-initial-geometry
    // affine P/D/ordinary-TDCP family as one transactional graph adapter.
    bool native_phase135_official_affine_measurement_family = false;
    // Phase138 opt-in: move the source-consistent fixed initial endpoint
    // range difference into the ordinary Phase135 affine TDCP measurement.
    // Default-off and dependent on the complete Phase135 selector.
    bool native_phase138_affine_tdcp_anchor_range_constant = false;
    // Phase141 opt-in: expose one native-authoritative, scalar-only schema
    // envelope for the existing diagnostics.  This never changes a graph,
    // factor, solver, or output row and is deliberately absent by default.
    bool native_phase141_telemetry_schema = false;
    // Phase143 opt-in: use the official 1000-iteration LM budget only for
    // the Phase142/99 meter-state Pose3+IMU main solve.  GNSS-first already
    // uses 1000; all other optimizer parameters and the legacy default stay
    // unchanged.
    bool native_phase143_official_main_lm_termination_budget = false;
    // Phase144 opt-in: select the repaired, deterministic scalar telemetry
    // serializer.  This changes neither the graph nor any optimizer input;
    // it only makes the summary JSON duplicate-free and source-explicit.
    bool native_phase144_telemetry_schema = false;
    // Phase104 opt-in: copy the already-computed GNSS-first trajectory after
    // the existing in-memory handoff and emit main displacement scalars.  The
    // copies are evaluation-only; they never re-enter a graph or initializer.
    bool native_phase104_stage_main_accuracy_attribution = false;
    std::string phase104_stage_ecef_path;
    std::string phase104_main_displacement_stats_path;
    bool native_upstream_absolute_doppler_screen = false;
    bool native_carrier_code_leveling = false;
    bool native_carrier_code_innovation_reset = false;
    bool native_carrier_code_primary_l1_e1 = false;
    bool native_carrier_code_gal_e1_e5a = false;
    bool native_upstream_stop_constraints = false;
    bool native_upstream_position_offset = false;
    bool native_signal_specific_galileo_tgd = false;
    bool native_quality_anchor = false;
    // Phase51 opt-in: floor each adopted raw Android pseudorange sigma with
    // the source ReceivedSvTimeUncertaintyNanos converted to metres.
    bool native_android_sv_time_uncertainty_sigma_floor = false;
    // Phase58 opt-in: floor each adopted raw Android undifferenced Doppler
    // sigma with the fixed C/N0 closure-residual model.  This is deliberately
    // separate from the existing p85/12 upstream quality path.
    bool native_cn0_doppler_calibration = false;
    // Phase65 opt-in: subtract source-compatible, smoothed base-station
    // pseudorange residuals from adopted undifferenced pseudorange factors.
    bool native_base_pseudorange_compensation = false;
    // Phase71 opt-in: preserve one selected base observation per supported
    // frequency band before building the Phase65 correction model.  This is
    // deliberately scoped to the base reader and does not alter rover/nav
    // observation selection or any factor population.
    bool native_base_pseudorange_preserve_additional_frequency_bands = false;
    // Phase73 opt-in: match the published graph's finite-pc miss mask by
    // retaining only adopted pseudorange factors with an in-domain correction.
    bool native_base_pseudorange_source_miss_mask = false;
    // Phase43 candidate: when the normal quality-anchor reconnaissance has no
    // eligible raw/nav solution, retry that same SPP reconnaissance with an
    // explicit -90 degree elevation gate and replay the selected anchor via
    // the ordinary factor-builder path.
    bool native_fallback_seed_quality_anchor_recovery = false;
    // Phase39 candidate: use the GNSS-first pass only for its independently
    // optimized Doppler velocity/heading seeds.  Position and receiver-clock
    // states remain the original raw SPP seeds.
    bool native_gnss_first_velocity_only_handoff = false;
    // Phase40 candidate: consume the existing raw-observable Doppler WLS
    // estimates directly.  No GNSS-first optimizer is run in this mode.
    bool native_direct_doppler_wls_handoff = false;
    // Phase149 opt-in: run the native raw-P same-run seed preflight and emit
    // only typed structural metadata. This path never enters FGO and keeps
    // all existing H/Doppler admission guards unchanged.
    bool native_phase149_raw_p_seed_stage = false;
    // Phase157 opt-in: obtain a same-run native raw-P cold-start position
    // before the ordinary elevation mask is applied by Phase149.
    bool native_phase157_raw_p_bootstrap = false;
    // Phase159 opt-in: collect every raw-P diagnostic epoch independently;
    // this remains a prep-only lane and never creates a graph seed set.
    bool native_phase159_collect_all_epochs = false;
    // Phase163 opt-in: adapt the same-run raw-P result to typed position /
    // velocity / clock handoff records. This lane is prep-only: it never
    // admits or launches an FGO graph, and never reads saved seed files.
    bool native_phase163_raw_p_no_doppler_seed_stage = false;
    // Phase165 opt-in: consume the same-run typed raw-P handoff in the
    // dedicated GNSS-only no-Doppler C7 graph.  This lane returns before any
    // IMU/main output path and never reads a persisted seed.
    bool native_phase165_raw_p_no_doppler_graph = false;
    // Phase167 opt-in: apply the source-backed 1000-iteration LM budget and
    // native Phase143 termination sidecar only to the Phase165 graph.
    bool native_phase167_raw_p_no_doppler_lm_termination_budget = false;
    // Phase171 opt-in: consume the same-run Phase165/167 GNSS-first result
    // directly in the existing Pose3+IMU main graph.  This is a dedicated
    // empty-generic-D handoff lane; normal-D and legacy paths remain intact.
    bool native_phase171_raw_p_no_doppler_imu_main = false;
    // Phase191 opt-in: retain the corrected raw-D rows in the Phase171
    // GNSS-first Point3/ECEF staging graph.  The Pose3+IMU main graph remains
    // on the Phase171 empty-generic-D contract.
    bool native_phase171_raw_p_ecef_doppler_gnss_first = false;
    // Phase184 opt-in: source Type-based ordinary-TDCP Huber-k composition
    // for the dedicated Phase171 no-Doppler lane only.
    bool native_phase184_source_tdcp_huber_k = false;
    // Phase180 opt-in: read Android GNSS clock fields and the raw IMU sample
    // fields needed for timestamp pairing through the existing explicit
    // UTC/GPS wall-clock fallback, then exit before GNSS observation
    // conversion, navigation loading, or any optimizer.
    bool native_phase180_android_clock_preflight = false;
    // Phase194 opt-in: when the actual Android UTC/GPS wall-clock fallback is
    // applied in the dedicated Phase171 ECEF-Doppler lane, use the source
    // coefficient-1.0 IMU white-noise densities.  Loader alignment remains
    // on its existing coefficient 0.5; bias random walk and integration noise
    // are intentionally untouched.
    bool native_phase194_source_utc_fallback_imu_noise = false;
    // Phase197 opt-in: apply the source -20 ms UTC offset only after the
    // validated raw UTC/GPS wall-clock fallback is actually selected.  Raw
    // UTC keys, pairing clocks, and elapsed-anchor paths remain unchanged.
    bool native_phase197_source_utc_fallback_imu_offset = false;
    // Phase201 opt-in: use the cached source-inclusive-forward IMU schedule
    // in the Pixel5 Phase171 Pose3/IMU main graph.  The legacy preceding-delta
    // plus boundary-tail schedule remains the default.
    bool native_phase201_source_inclusive_forward_imu_schedule = false;
    bool native_phase205_source_count_bias_density = false;
    bool native_phase209_source_separate_imu_factors = false;
    bool native_phase213_main_doppler = false;
    bool native_phase217_main_pose3_motion = false;
    bool native_source_tdcp_meter_sigma = false;
    bool native_source_tdcp_resl_observable = false;
    bool native_tdcp_only_affine_geometry = false;
    bool native_epoch_heading_attitude_seeds = false;
    bool native_omit_first_imu_bias_prior = false;
    bool native_omit_first_imu_velocity_prior = false;
    bool native_relative_height_pairs = false;
    bool native_pseudorange_remasking = false;
};

const char* carrierSignalName(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA: return "GPS_L1CA";
        case libgnss::SignalType::GAL_E1: return "GAL_E1";
        case libgnss::SignalType::GAL_E5A: return "GAL_E5A";
        default: return "UNSUPPORTED";
    }
}

const char* baseTelemetrySignalName(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA: return "GPS_L1CA";
        case libgnss::SignalType::GPS_L1P: return "GPS_L1P";
        case libgnss::SignalType::GPS_L2P: return "GPS_L2P";
        case libgnss::SignalType::GPS_L2C: return "GPS_L2C";
        case libgnss::SignalType::GPS_L5: return "GPS_L5";
        case libgnss::SignalType::GLO_L1CA: return "GLO_L1CA";
        case libgnss::SignalType::GLO_L1P: return "GLO_L1P";
        case libgnss::SignalType::GLO_L2CA: return "GLO_L2CA";
        case libgnss::SignalType::GLO_L2P: return "GLO_L2P";
        case libgnss::SignalType::GAL_E1: return "GAL_E1";
        case libgnss::SignalType::GAL_E5A: return "GAL_E5A";
        case libgnss::SignalType::GAL_E5B: return "GAL_E5B";
        case libgnss::SignalType::GAL_E6: return "GAL_E6";
        case libgnss::SignalType::BDS_B1I: return "BDS_B1I";
        case libgnss::SignalType::BDS_B2I: return "BDS_B2I";
        case libgnss::SignalType::BDS_B3I: return "BDS_B3I";
        case libgnss::SignalType::BDS_B1C: return "BDS_B1C";
        case libgnss::SignalType::BDS_B2A: return "BDS_B2A";
        case libgnss::SignalType::QZS_L1CA: return "QZS_L1CA";
        case libgnss::SignalType::QZS_L2C: return "QZS_L2C";
        case libgnss::SignalType::QZS_L5: return "QZS_L5";
        default: return "UNSUPPORTED";
    }
}

const char* baseTelemetryFrequencyBand(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
        case libgnss::SignalType::GPS_L1P:
        case libgnss::SignalType::QZS_L1CA:
            return "L1";
        case libgnss::SignalType::GLO_L1CA:
        case libgnss::SignalType::GLO_L1P:
            return "G1";
        case libgnss::SignalType::GAL_E1:
            return "E1";
        case libgnss::SignalType::BDS_B1I:
        case libgnss::SignalType::BDS_B1C:
            return "B1";
        case libgnss::SignalType::GPS_L2P:
        case libgnss::SignalType::GPS_L2C:
        case libgnss::SignalType::QZS_L2C:
            return "L2";
        case libgnss::SignalType::GLO_L2CA:
        case libgnss::SignalType::GLO_L2P:
            return "G2";
        case libgnss::SignalType::BDS_B2I:
            return "B2";
        case libgnss::SignalType::GPS_L5:
        case libgnss::SignalType::QZS_L5:
            return "L5";
        case libgnss::SignalType::GAL_E5A:
            return "E5A";
        case libgnss::SignalType::GAL_E5B:
            return "E5B";
        case libgnss::SignalType::BDS_B2A:
            return "B2A";
        case libgnss::SignalType::GAL_E6:
        case libgnss::SignalType::BDS_B3I:
            return signal == libgnss::SignalType::GAL_E6 ? "E6" : "B3";
        default:
            return "unknown";
    }
}

void usage(const char* program) {
    std::cout << "Usage: " << program
              << " (--obs <rover.obs> --imu <imu.csv>"
                 " | --android-gnss <device_gnss.csv> --android-imu <device_imu.csv>)"
                 " --nav <brdc.nav>"
                 " --out <submission.csv> --summary-json <summary.json>"
                 " [--dataset-id <id>] [--skip-epochs <n>]"
                 " [--max-epochs 10..30 | --all-epochs]"
                 " [--android-raw-utc-keys] [--android-include-first-native-epoch] [--android-raw-clock-only]"
                 " [--fgo-imu-sparse-recovery]"
                 " [--android-utc-wall-clock-fallback]"
                 " [--native-pdc-state-bridge] [--native-pdc-imu-tdcp]"
                 " [--native-pdc-imu-tdcp-no-bridge]"
                 " [--native-signal-bias-states] [--native-residual-ionosphere]"
                 " [--native-upstream-quality] [--native-carrier-code-leveling]"
                 " [--native-source-direct-observable-quality]"
                 " [--native-source-clock-c0d-factor]"
                 " [--native-source-clock-c0d-meter-state-parity]"
                 " [--native-source-clock-c0d-active-solve-diagnostic]"
                 " [--native-source-clock-c0d-gnss-first-raw-drift-d-initializer]"
                 " [--native-source-clock-c0d-gnss-first-meter-state-handoff]"
                 " [--native-source-clock-c0d-epoch-vector-parity]"
                 " [--native-source-clock-c0d-phase94-stage-diagnostics]"
                 " [--native-source-clock-c0d-phase96-main-diagnostics]"
                 " [--native-source-clock-c0d-phase97-singular-system-diagnostics]"
                 " [--native-source-clock-c0d-phase98-solver-rank-diagnostic]"
                 " [--native-source-clock-c0d-phase99-main-multifrontal-qr-solver]"
                 " [--native-direct-wls-ephemeral-c7d-main-seed]"
                 " [--native-phase116-carrier-tdcp-incidence-diagnostic]"
                 " [--native-phase117-tdcp-snr-type-sigma]"
                 " [--native-phase118-official-tdcp-huber-k]"
                 " [--native-phase120-official-tdcp-resl-atmosphere-cancellation]"
                 " [--native-phase126-raw-base-source-complete]"
                 " [--native-paired-epoch-states]"
                 " [--native-rover-epoch-states]"
                 " [--native-dense-base-smoothing]"
                 " [--native-base-mask-only-ablation]"
                 " [--native-base-gps-values-only-ablation]"
                 " [--native-base-gps-center-ablation]"
                 " [--native-main-p-cauchy]"
                 " [--native-stationary-gyro-initializer]"
                 " [--native-tdcp-no-code-jump-gate]"
                 " [--native-sparse-p-staging]"
                 " [--native-tdcp-frequency-residual-states]"
                 " [--native-phase127-glonass-channel-provenance]"
                 " [--native-phase128-glonass-provenance-parser-admission]"
                 " [--native-phase129-glonass-local-miss-mask]"
                 " [--native-phase131-canonical-correction-band-key]"
                 " [--native-phase135-official-affine-measurement-family]"
                 " [--native-phase138-affine-tdcp-anchor-range-constant]"
                 " [--native-phase141-telemetry-schema]"
                 " [--native-phase143-official-main-lm-termination-budget]"
                 " [--native-phase144-telemetry-schema]"
                 " [--native-phase104-stage-main-attribution"
                 " --phase104-stage-ecef <stage.csv>"
                 " --phase104-main-displacement-stats <stats.json>]"
                 " [--native-upstream-absolute-doppler-screen]"
                 " [--native-carrier-code-innovation-reset]"
                 " [--native-carrier-code-primary-l1-e1]"
                 " [--native-carrier-code-gal-e1-e5a]"
                 " [--native-upstream-stop-constraints]"
                 " [--native-upstream-position-offset]"
                 " [--native-signal-specific-galileo-tgd]"
                 " [--native-quality-anchor]"
                 " [--native-fallback-seed-quality-anchor-recovery]"
                 " [--native-android-sv-time-uncertainty-sigma-floor]"
                 " [--native-cn0-doppler-calibration]"
                 " [--native-base-pseudorange-compensation --native-base-rinex <base.obs>"
                 " --native-base-rinex-sha256 <sha256>]"
                 " [--native-base-pseudorange-preserve-additional-frequency-bands]"
                 " [--native-base-pseudorange-source-miss-mask]"
                 " [--native-gnss-first-velocity-only-handoff]"
                 " [--native-direct-doppler-wls-handoff]"
                 " [--native-phase149-raw-p-seed-stage]"
                 " [--native-phase157-raw-p-bootstrap]"
                 " [--native-phase159-collect-all-epochs]"
                 " [--native-phase163-raw-p-no-doppler-seed-stage]"
                 " [--native-phase165-raw-p-no-doppler-graph]"
                 " [--native-phase167-raw-p-no-doppler-lm-termination-budget]"
                 " [--native-phase171-raw-p-no-doppler-imu-main]"
                 " [--native-phase171-raw-p-ecef-doppler-gnss-first]"
                 " [--native-phase184-source-tdcp-huber-k]"
                 " [--native-phase180-android-clock-preflight]"
                 " [--native-phase194-source-utc-fallback-imu-noise]"
                 " [--native-phase197-source-utc-fallback-imu-offset]"
                 " [--native-phase201-source-inclusive-forward-imu-schedule]"
                 " [--native-phase205-source-count-bias-density]"
                 " [--native-phase209-source-separate-imu-factors]"
                 " [--native-phase213-main-doppler]"
                 " [--native-phase217-main-pose3-motion]"
                 " [--native-source-tdcp-meter-sigma]"
                 " [--native-lm-lambda-floor]"
                 " [--native-nhc-monitor]"
                 " [--native-batch-nhc]"
                 " [--native-main-code-uncertainty-floor]"
                 " [--native-main-code-edge-readmission]"
                 " [--native-joint-ionosphere ANCHOR_SIGMA_M DENSITY_M_SQRT_S MAX_GAP_S]"
                 " [--native-doppler-rotation-rate]"
                 " [--native-tdcp-adr-endpoint-sigma]"
                 " [--native-source-tdcp-resl-observable]"
                 " [--native-tdcp-only-affine-geometry]"
                 " [--native-epoch-heading-attitude-seeds]"
                 " [--native-omit-first-imu-bias-prior]"
                 " [--native-omit-first-imu-velocity-prior]"
                 " [--native-relative-height-pairs] [--native-pseudorange-remasking]\n";
    std::cout << "Phase149/163 prep and Phase165 graph modes accept --obs/--nav or"
                 " --android-gnss/--nav "
                 "with --summary-json only; it omits --out and IMU.\n";
}

bool requireValue(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc) {
        return false;
    }
    value = argv[++index];
    return !value.empty();
}

std::string phoneFromDatasetId(const std::string& dataset_id) {
    const std::size_t slash = dataset_id.find_last_of("/\\");
    return slash == std::string::npos ? dataset_id : dataset_id.substr(slash + 1U);
}

// Phase112 is an output-boundary candidate composed only with the exact
// Phase101 C7/D/QR handoff.  The diagnostics that withhold or replace the
// main output are deliberately excluded so GNSS-first/stage/handoff state is
// never corrected by this selector.
bool phase112MainOutputPositionOffsetSelectors(const Options& options) {
    const bool exact_main_seed =
        options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
        options.native_direct_wls_ephemeral_c7d_main_seed;
    return exact_main_seed &&
           options.native_source_clock_c0d_epoch_vector_parity &&
           options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver &&
           !options.native_source_clock_c0d_phase94_stage_diagnostics &&
           !options.native_source_clock_c0d_phase96_main_diagnostics &&
           !options.native_source_clock_c0d_phase97_singular_system_diagnostics &&
           !options.native_source_clock_c0d_phase98_solver_rank_diagnostic &&
           !options.native_phase104_stage_main_accuracy_attribution;
}

struct DirectObservableQualitySettings {
    bool valid = false;
    std::string environment;
    double pseudorange_huber_threshold_sigma = 0.0;
    double doppler_huber_threshold_sigma = 0.0;
    double official_tdcp_huber_threshold_sigma = 0.0;
};

// Phase80's P+D robust thresholds are a fixed Type mapping over the four
// frozen route IDs.  Dataset identity is the only selector; no route score,
// truth, or precomputed result is consulted.
DirectObservableQualitySettings directObservableQualitySettingsForDataset(
    const std::string& dataset_id) {
    struct Entry {
        const char* route_id;
        const char* environment;
        double pseudorange_huber_threshold_sigma;
        double doppler_huber_threshold_sigma;
    };
    static constexpr Entry kEntries[] = {
        {"2021-03-16-18-59-us-ca-mtv-a/pixel5", "Highway", 0.2, 0.8},
        {"2021-08-24-20-32-us-ca-mtv-h/pixel5", "Street", 0.1, 0.4},
        {"2022-04-01-18-22-us-ca-lax-t/pixel5", "Highway", 0.2, 0.8},
        {"2023-03-08-21-34-us-ca-mtv-u/pixel5", "Street", 0.1, 0.4},
    };
    for (const Entry& entry : kEntries) {
        if (dataset_id != entry.route_id) {
            continue;
        }
        DirectObservableQualitySettings settings;
        settings.valid = true;
        settings.environment = entry.environment;
        settings.pseudorange_huber_threshold_sigma =
            entry.pseudorange_huber_threshold_sigma;
        settings.doppler_huber_threshold_sigma =
            entry.doppler_huber_threshold_sigma;
        if (!libgnss::fgo::resolveOfficialTdcpHuberThresholdSigmaForType(
                settings.environment, settings.official_tdcp_huber_threshold_sigma)) {
            return {};
        }
        return settings;
    }
    return {};
}

bool hasMatExtension(const std::string& path) {
    const std::size_t slash = path.find_last_of("/\\");
    const std::size_t dot = path.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
        return false;
    }
    std::string extension = path.substr(dot);
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return extension == ".mat";
}

bool parseArguments(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return false;
        }
        if (arg == "--obs") {
            if (!requireValue(argc, argv, i, options.obs_path)) return false;
        } else if (arg == "--nav") {
            if (!requireValue(argc, argv, i, options.nav_path)) return false;
        } else if (arg == "--imu") {
            if (!requireValue(argc, argv, i, options.imu_path)) return false;
        } else if (arg == "--android-imu") {
            if (!requireValue(argc, argv, i, options.android_imu_path)) return false;
        } else if (arg == "--android-gnss") {
            if (!requireValue(argc, argv, i, options.android_gnss_path)) return false;
        } else if (arg == "--native-base-rinex") {
            if (!requireValue(argc, argv, i, options.native_base_rinex_path)) return false;
        } else if (arg == "--native-base-rinex-sha256") {
            if (!requireValue(argc, argv, i, options.native_base_rinex_sha256)) return false;
        } else if (arg == "--out") {
            if (!requireValue(argc, argv, i, options.out_path)) return false;
        } else if (arg == "--summary-json") {
            if (!requireValue(argc, argv, i, options.summary_path)) return false;
        } else if (arg == "--dataset-id") {
            if (!requireValue(argc, argv, i, options.dataset_id)) return false;
        } else if (arg == "--max-epochs") {
            std::string value;
            if (!requireValue(argc, argv, i, value)) return false;
            try {
                std::size_t consumed = 0;
                options.max_epochs = std::stoi(value, &consumed);
                if (consumed != value.size()) return false;
            } catch (...) {
                return false;
            }
            if (options.max_epochs < kMinEpochLimit || options.max_epochs > kMaxEpochLimit) {
                std::cerr << "--max-epochs must be between " << kMinEpochLimit << " and "
                          << kMaxEpochLimit << " for the frozen short-window contract\n";
                return false;
            }
        } else if (arg == "--skip-epochs") {
            std::string value;
            if (!requireValue(argc, argv, i, value)) return false;
            try {
                std::size_t consumed = 0;
                options.skip_epochs = std::stoi(value, &consumed);
                if (consumed != value.size() || options.skip_epochs < 0) return false;
            } catch (...) {
                return false;
            }
        } else if (arg == "--all-epochs") {
            options.all_epochs = true;
        } else if (arg == "--android-raw-utc-keys") {
            options.android_raw_utc_key_contract = true;
        } else if (arg == "--android-include-first-native-epoch") {
            options.android_include_first_native_epoch = true;
        } else if (arg == "--android-raw-clock-only") {
            options.android_raw_clock_only = true;
        } else if (arg == "--android-utc-wall-clock-fallback") {
            options.android_utc_wall_clock_fallback = true;
        } else if (arg == "--fgo-imu-sparse-recovery") {
            options.fgo_imu_sparse_recovery = true;
        } else if (arg == "--native-pdc-state-bridge") {
            options.native_pdc_state_bridge = true;
        } else if (arg == "--native-pdc-imu-tdcp") {
            options.native_pdc_imu_tdcp = true;
        } else if (arg == "--native-pdc-imu-tdcp-no-bridge") {
            options.native_pdc_imu_tdcp_no_bridge = true;
        } else if (arg == "--native-signal-bias-states") {
            options.native_signal_bias_states = true;
        } else if (arg == "--native-residual-ionosphere") {
            options.native_residual_ionosphere = true;
        } else if (arg == "--native-upstream-quality") {
            options.native_upstream_quality = true;
        } else if (arg == "--native-source-direct-observable-quality") {
            options.native_source_direct_observable_quality = true;
        } else if (arg == "--native-source-clock-c0d-factor") {
            options.native_source_clock_c0d_factor = true;
        } else if (arg == "--native-source-clock-c0d-meter-state-parity") {
            options.native_source_clock_c0d_meter_state_parity = true;
        } else if (arg == "--native-source-clock-c0d-active-solve-diagnostic") {
            options.native_source_clock_c0d_active_solve_diagnostic = true;
        } else if (arg == "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer") {
            options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer = true;
        } else if (arg == "--native-source-clock-c0d-gnss-first-meter-state-handoff") {
            options.native_source_clock_c0d_gnss_first_meter_state_handoff = true;
        } else if (arg == "--native-source-clock-c0d-epoch-vector-parity") {
            options.native_source_clock_c0d_epoch_vector_parity = true;
        } else if (arg == "--native-source-clock-c0d-phase94-stage-diagnostics") {
            options.native_source_clock_c0d_phase94_stage_diagnostics = true;
        } else if (arg == "--native-source-clock-c0d-phase96-main-diagnostics") {
            options.native_source_clock_c0d_phase96_main_diagnostics = true;
        } else if (arg == "--native-source-clock-c0d-phase97-singular-system-diagnostics") {
            options.native_source_clock_c0d_phase97_singular_system_diagnostics = true;
        } else if (arg == "--native-source-clock-c0d-phase98-solver-rank-diagnostic") {
            options.native_source_clock_c0d_phase98_solver_rank_diagnostic = true;
        } else if (arg == "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver") {
            options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver = true;
        } else if (arg == "--native-direct-wls-ephemeral-c7d-main-seed") {
            options.native_direct_wls_ephemeral_c7d_main_seed = true;
        } else if (arg == "--native-phase116-carrier-tdcp-incidence-diagnostic") {
            options.native_phase116_carrier_tdcp_incidence_diagnostic = true;
        } else if (arg == "--native-phase117-tdcp-snr-type-sigma") {
            options.native_phase117_tdcp_snr_type_sigma = true;
        } else if (arg == "--native-phase118-official-tdcp-huber-k") {
            options.native_phase118_official_tdcp_huber_k = true;
        } else if (arg ==
                   "--native-phase120-official-tdcp-resl-atmosphere-cancellation") {
            options.native_phase120_official_tdcp_resl_atmosphere_cancellation = true;
        } else if (arg == "--native-paired-epoch-states") {
            options.native_paired_epoch_states = true;
        } else if (arg == "--native-rover-epoch-states") {
            options.native_rover_epoch_states = true;
        } else if (arg == "--native-dense-base-smoothing") {
            options.native_dense_base_smoothing = true;
        } else if (arg == "--native-base-mask-only-ablation") {
            options.native_base_mask_only_ablation = true;
        } else if (arg == "--native-base-gps-values-only-ablation") {
            options.native_base_gps_values_only_ablation = true;
        } else if (arg == "--native-base-gps-center-ablation") {
            options.native_base_gps_center_ablation = true;
        } else if (arg == "--native-sparse-p-staging") {
            options.native_sparse_p_staging = true;
        } else if (arg == "--native-tdcp-frequency-residual-states") {
            options.native_tdcp_frequency_residual_states = true;
        } else if (arg == "--native-tdcp-no-code-jump-gate") {
            options.native_tdcp_no_code_jump_gate = true;
        } else if (arg == "--native-stationary-gyro-initializer") {
            options.native_stationary_gyro_initializer = true;
        } else if (arg == "--native-main-p-cauchy") {
            options.native_main_p_cauchy = true;
        } else if (arg == "--native-phase126-raw-base-source-complete") {
            options.native_phase126_raw_base_source_complete = true;
        } else if (arg == "--native-phase127-glonass-channel-provenance") {
            options.native_phase127_glonass_channel_provenance = true;
        } else if (arg ==
                   "--native-phase128-glonass-provenance-parser-admission") {
            options.native_phase128_glonass_provenance_parser_admission = true;
        } else if (arg == "--native-phase129-glonass-local-miss-mask") {
            options.native_phase129_glonass_local_miss_mask = true;
        } else if (arg ==
                   "--native-phase131-canonical-correction-band-key") {
            options.native_phase131_canonical_correction_band_key = true;
        } else if (arg ==
                   "--native-phase135-official-affine-measurement-family") {
            options.native_phase135_official_affine_measurement_family = true;
        } else if (arg ==
                   "--native-phase138-affine-tdcp-anchor-range-constant") {
            options.native_phase138_affine_tdcp_anchor_range_constant = true;
        } else if (arg == "--native-phase141-telemetry-schema") {
            options.native_phase141_telemetry_schema = true;
        } else if (arg ==
                   "--native-phase143-official-main-lm-termination-budget") {
            options.native_phase143_official_main_lm_termination_budget = true;
        } else if (arg == "--native-phase144-telemetry-schema") {
            options.native_phase144_telemetry_schema = true;
        } else if (arg == "--native-phase104-stage-main-attribution") {
            options.native_phase104_stage_main_accuracy_attribution = true;
        } else if (arg == "--phase104-stage-ecef") {
            if (!requireValue(argc, argv, i, options.phase104_stage_ecef_path)) return false;
        } else if (arg == "--phase104-main-displacement-stats") {
            if (!requireValue(argc, argv, i,
                              options.phase104_main_displacement_stats_path)) {
                return false;
            }
        } else if (arg == "--native-upstream-absolute-doppler-screen") {
            options.native_upstream_absolute_doppler_screen = true;
        } else if (arg == "--native-carrier-code-leveling") {
            options.native_carrier_code_leveling = true;
        } else if (arg == "--native-carrier-code-innovation-reset") {
            options.native_carrier_code_innovation_reset = true;
        } else if (arg == "--native-carrier-code-primary-l1-e1") {
            options.native_carrier_code_primary_l1_e1 = true;
        } else if (arg == "--native-carrier-code-gal-e1-e5a") {
            options.native_carrier_code_gal_e1_e5a = true;
        } else if (arg == "--native-upstream-stop-constraints") {
            options.native_upstream_stop_constraints = true;
        } else if (arg == "--native-upstream-position-offset") {
            options.native_upstream_position_offset = true;
        } else if (arg == "--native-signal-specific-galileo-tgd") {
            options.native_signal_specific_galileo_tgd = true;
        } else if (arg == "--native-quality-anchor") {
            options.native_quality_anchor = true;
        } else if (arg == "--native-fallback-seed-quality-anchor-recovery") {
            options.native_fallback_seed_quality_anchor_recovery = true;
        } else if (arg == "--native-android-sv-time-uncertainty-sigma-floor") {
            options.native_android_sv_time_uncertainty_sigma_floor = true;
        } else if (arg == "--native-cn0-doppler-calibration") {
            options.native_cn0_doppler_calibration = true;
        } else if (arg == "--native-base-pseudorange-compensation") {
            options.native_base_pseudorange_compensation = true;
        } else if (arg == "--native-base-pseudorange-preserve-additional-frequency-bands") {
            options.native_base_pseudorange_preserve_additional_frequency_bands = true;
        } else if (arg == "--native-base-pseudorange-source-miss-mask") {
            options.native_base_pseudorange_source_miss_mask = true;
        } else if (arg == "--native-gnss-first-velocity-only-handoff") {
            options.native_gnss_first_velocity_only_handoff = true;
        } else if (arg == "--native-direct-doppler-wls-handoff") {
            options.native_direct_doppler_wls_handoff = true;
        } else if (arg == "--native-phase149-raw-p-seed-stage") {
            options.native_phase149_raw_p_seed_stage = true;
        } else if (arg == "--native-phase157-raw-p-bootstrap") {
            options.native_phase157_raw_p_bootstrap = true;
        } else if (arg == "--native-phase159-collect-all-epochs") {
            options.native_phase159_collect_all_epochs = true;
        } else if (arg == "--native-phase163-raw-p-no-doppler-seed-stage") {
            options.native_phase163_raw_p_no_doppler_seed_stage = true;
        } else if (arg == "--native-phase165-raw-p-no-doppler-graph") {
            options.native_phase165_raw_p_no_doppler_graph = true;
        } else if (arg ==
                   "--native-phase167-raw-p-no-doppler-lm-termination-budget") {
            options.native_phase167_raw_p_no_doppler_lm_termination_budget = true;
        } else if (arg == "--native-phase171-raw-p-no-doppler-imu-main") {
            options.native_phase171_raw_p_no_doppler_imu_main = true;
        } else if (arg ==
                   "--native-phase171-raw-p-ecef-doppler-gnss-first") {
            options.native_phase171_raw_p_ecef_doppler_gnss_first = true;
        } else if (arg == "--native-phase184-source-tdcp-huber-k") {
            options.native_phase184_source_tdcp_huber_k = true;
        } else if (arg == "--native-phase180-android-clock-preflight") {
            options.native_phase180_android_clock_preflight = true;
        } else if (arg ==
                   "--native-phase194-source-utc-fallback-imu-noise") {
            options.native_phase194_source_utc_fallback_imu_noise = true;
        } else if (arg ==
                   "--native-phase197-source-utc-fallback-imu-offset") {
            options.native_phase197_source_utc_fallback_imu_offset = true;
        } else if (arg ==
                   "--native-phase201-source-inclusive-forward-imu-schedule") {
            options.native_phase201_source_inclusive_forward_imu_schedule = true;
        } else if (arg == "--native-phase205-source-count-bias-density") {
            options.native_phase205_source_count_bias_density = true;
        } else if (arg == "--native-phase209-source-separate-imu-factors") {
            options.native_phase209_source_separate_imu_factors = true;
        } else if (arg == "--native-phase213-main-doppler") {
            options.native_phase213_main_doppler = true;
        } else if (arg == "--native-phase217-main-pose3-motion") {
            options.native_phase217_main_pose3_motion = true;
        } else if (arg == "--native-lm-lambda-floor") {
            options.native_lm_lambda_floor = true;
        } else if (arg == "--native-nhc-monitor") {
            options.native_nhc_monitor = true;
        } else if (arg == "--native-batch-nhc") {
            options.native_batch_nhc = true;
        } else if (arg == "--native-main-code-uncertainty-floor") {
            options.native_main_code_uncertainty_floor = true;
        } else if (arg == "--native-main-code-edge-readmission") {
            options.native_main_code_edge_readmission = true;
        } else if (arg == "--native-tdcp-adr-endpoint-sigma") {
            options.native_tdcp_adr_endpoint_sigma = true;
        } else if (arg == "--native-doppler-rotation-rate") {
            options.native_doppler_rotation_rate = true;
        } else if (arg == "--native-joint-ionosphere") {
            if (options.native_joint_ionosphere) return false;
            for (double* value : {&options.joint_ionosphere_anchor,
                                  &options.joint_ionosphere_density,&options.joint_ionosphere_gap}) {
                std::string token;
                if (!requireValue(argc,argv,i,token)) return false;
                std::istringstream stream(token);
                if (!(stream >> *value) || !stream.eof() || !std::isfinite(*value) || *value <= 0)
                    return false;
            }
            options.native_joint_ionosphere = true;
        } else if (arg == "--native-source-tdcp-meter-sigma") {
            options.native_source_tdcp_meter_sigma = true;
        } else if (arg == "--native-source-tdcp-resl-observable") {
            options.native_source_tdcp_resl_observable = true;
        } else if (arg == "--native-tdcp-only-affine-geometry") {
            options.native_tdcp_only_affine_geometry = true;
        } else if (arg == "--native-epoch-heading-attitude-seeds") {
            options.native_epoch_heading_attitude_seeds = true;
        } else if (arg == "--native-omit-first-imu-bias-prior") {
            options.native_omit_first_imu_bias_prior = true;
        } else if (arg == "--native-pseudorange-remasking") {
            options.native_pseudorange_remasking = true;
        } else if (arg == "--native-relative-height-pairs") {
            options.native_relative_height_pairs = true;
        } else if (arg == "--native-omit-first-imu-velocity-prior") {
            options.native_omit_first_imu_velocity_prior = true;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }
    const bool phase149_seed_stage = options.native_phase149_raw_p_seed_stage;
    const bool phase163_seed_stage =
        options.native_phase163_raw_p_no_doppler_seed_stage;
    const bool phase165_graph_stage =
        options.native_phase165_raw_p_no_doppler_graph;
    const bool phase167_budget =
        options.native_phase167_raw_p_no_doppler_lm_termination_budget;
    const bool phase171_imu_main =
        options.native_phase171_raw_p_no_doppler_imu_main;
    const bool phase171_ecef_doppler =
        options.native_phase171_raw_p_ecef_doppler_gnss_first;
    const bool phase184_source_tdcp_huber_k =
        options.native_phase184_source_tdcp_huber_k;
    const bool phase180_clock_preflight =
        options.native_phase180_android_clock_preflight;
    const bool phase194_source_utc_fallback_imu_noise =
        options.native_phase194_source_utc_fallback_imu_noise;
    const bool phase197_source_utc_fallback_imu_offset =
        options.native_phase197_source_utc_fallback_imu_offset;
    const bool phase201_source_inclusive_forward_imu_schedule =
        options.native_phase201_source_inclusive_forward_imu_schedule;
    const bool seed_stage = phase149_seed_stage || phase163_seed_stage;
    const bool raw_only_stage =
        seed_stage || (phase165_graph_stage && !phase171_imu_main) ||
        phase180_clock_preflight;
    if ((phase149_seed_stage && phase163_seed_stage) ||
        (phase165_graph_stage && seed_stage)) {
        std::cerr << "Phase149, Phase163, and Phase165 raw-P selectors are mutually exclusive\n";
        return false;
    }
    if (phase165_graph_stage && options.native_phase159_collect_all_epochs) {
        std::cerr << "Phase159 collect-all is prep-only and cannot be combined "
                     "with the Phase165 graph selector\n";
        return false;
    }
    if (phase167_budget && !phase165_graph_stage) {
        std::cerr << "--native-phase167-raw-p-no-doppler-lm-termination-budget "
                     "requires the Phase165 graph selector\n";
        return false;
    }
    if (phase171_imu_main && !phase165_graph_stage) {
        std::cerr << "--native-phase171-raw-p-no-doppler-imu-main requires "
                     "the Phase165 graph selector\n";
        return false;
    }
    if (phase171_imu_main && !phase167_budget) {
        std::cerr << "--native-phase171-raw-p-no-doppler-imu-main requires "
                     "the Phase167 1000-iteration GNSS-first selector\n";
        return false;
    }
    if (phase171_ecef_doppler && !phase171_imu_main) {
        std::cerr << "--native-phase171-raw-p-ecef-doppler-gnss-first "
                     "requires the Phase171 raw-P IMU-main selector\n";
        return false;
    }
    if (phase194_source_utc_fallback_imu_noise &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback)) {
        std::cerr << "--native-phase194-source-utc-fallback-imu-noise requires "
                     "the Phase171 ECEF-Doppler lane and explicit UTC wall-clock "
                     "fallback\n";
        return false;
    }
    if (phase194_source_utc_fallback_imu_noise &&
        phoneFromDatasetId(options.dataset_id) != "pixel5") {
        std::cerr << "--native-phase194-source-utc-fallback-imu-noise is "
                     "pinned to the Pixel5 source-noise preset\n";
        return false;
    }
    if (phase197_source_utc_fallback_imu_offset &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback)) {
        std::cerr << "--native-phase197-source-utc-fallback-imu-offset requires "
                     "the Phase171 ECEF-Doppler lane and explicit UTC wall-clock "
                     "fallback\n";
        return false;
    }
    if (phase197_source_utc_fallback_imu_offset &&
        phoneFromDatasetId(options.dataset_id) != "pixel5") {
        std::cerr << "--native-phase197-source-utc-fallback-imu-offset is "
                     "pinned to the Pixel5 source timing preset\n";
        return false;
    }
    if (options.android_include_first_native_epoch &&
        (!options.android_raw_utc_key_contract || !phase171_imu_main ||
         !phase171_ecef_doppler || !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "Including first native epoch requires Phase171 ECEF-D, raw UTC keys and all epochs without skipping\n";
        return false;
    }
    if (options.native_epoch_heading_attitude_seeds &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         options.native_phase135_official_affine_measurement_family ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Epoch heading seeds require Pixel5 Phase171 ECEF-D, UTC fallback and no Phase135\n";
        return false;
    }
    if (options.native_pseudorange_remasking &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         options.native_base_pseudorange_compensation ||
         options.native_phase126_raw_base_source_complete ||
         options.native_phase135_official_affine_measurement_family ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Pseudorange re-masking requires Pixel5 Phase171 ECEF-D, UTC fallback and no base/Phase135\n";
        return false;
    }
    if (options.native_relative_height_pairs &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.native_upstream_stop_constraints ||
         !options.android_utc_wall_clock_fallback ||
         options.native_phase135_official_affine_measurement_family ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Relative height requires Pixel5 Phase171 ECEF-D, stop constraints, UTC fallback and no Phase135\n";
        return false;
    }
    if (options.native_omit_first_imu_velocity_prior &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.native_phase213_main_doppler ||
         !options.android_utc_wall_clock_fallback ||
         options.native_phase135_official_affine_measurement_family ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Omitting first velocity prior requires Pixel5 Phase171 ECEF-D, Phase213, UTC fallback and no Phase135\n";
        return false;
    }
    if (options.native_omit_first_imu_bias_prior &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         options.native_phase135_official_affine_measurement_family ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Omitting first bias prior requires Pixel5 Phase171 ECEF-D, UTC fallback and no Phase135\n";
        return false;
    }
    if (options.native_tdcp_only_affine_geometry &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         options.native_phase135_official_affine_measurement_family ||
         options.native_phase138_affine_tdcp_anchor_range_constant ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "TDCP-only affine geometry requires Pixel5 Phase171 ECEF-D, UTC fallback and no Phase135/138\n";
        return false;
    }
    if (options.native_source_tdcp_resl_observable &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         options.native_phase117_tdcp_snr_type_sigma ||
         options.native_phase118_official_tdcp_huber_k ||
         options.native_phase120_official_tdcp_resl_atmosphere_cancellation ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Source TDCP resL requires Pixel5 Phase171 ECEF-D, UTC fallback and no Phase117/118/120\n";
        return false;
    }
    if (options.native_source_tdcp_meter_sigma &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         options.native_phase117_tdcp_snr_type_sigma ||
         options.native_phase118_official_tdcp_huber_k ||
         options.native_phase120_official_tdcp_resl_atmosphere_cancellation ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Source TDCP metre sigma requires Pixel5 Phase171 ECEF-D, UTC fallback and no Phase117/118/120\n";
        return false;
    }
    if (options.native_phase217_main_pose3_motion &&
        (!phase171_imu_main || !phase171_ecef_doppler || !options.android_utc_wall_clock_fallback ||
         phase201_source_inclusive_forward_imu_schedule || options.native_phase205_source_count_bias_density ||
         options.native_phase209_source_separate_imu_factors ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Phase217 requires Pixel5 Phase171 ECEF-D, UTC fallback and legacy IMU options\n";
        return false;
    }
    if (options.native_phase213_main_doppler &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         phase201_source_inclusive_forward_imu_schedule ||
         options.native_phase205_source_count_bias_density ||
         options.native_phase209_source_separate_imu_factors ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Phase213 requires Pixel5 Phase171 ECEF-D with UTC fallback "
                     "and legacy IMU options\n";
        return false;
    }
    if (options.native_phase209_source_separate_imu_factors &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         phase201_source_inclusive_forward_imu_schedule ||
         options.native_phase205_source_count_bias_density ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Phase209 requires Pixel5 Phase171 ECEF-D with UTC fallback, "
                     "legacy IMU integration and Phase205 off\n";
        return false;
    }
    if (options.native_phase205_source_count_bias_density &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback ||
         phase201_source_inclusive_forward_imu_schedule ||
         phoneFromDatasetId(options.dataset_id) != "pixel5")) {
        std::cerr << "Phase205 requires Pixel5 Phase171 ECEF-D with UTC fallback "
                     "and legacy IMU integration\n";
        return false;
    }
    if (phase201_source_inclusive_forward_imu_schedule &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         !options.android_utc_wall_clock_fallback)) {
        std::cerr << "--native-phase201-source-inclusive-forward-imu-schedule "
                     "requires the Phase171 ECEF-Doppler lane and explicit "
                     "UTC wall-clock fallback\n";
        return false;
    }
    if (phase201_source_inclusive_forward_imu_schedule &&
        phoneFromDatasetId(options.dataset_id) != "pixel5") {
        std::cerr << "--native-phase201-source-inclusive-forward-imu-schedule "
                     "is pinned to the Pixel5 source timing preset\n";
        return false;
    }
    if (phase184_source_tdcp_huber_k && !phase171_imu_main) {
        std::cerr << "--native-phase184-source-tdcp-huber-k requires the "
                     "Phase171 raw-P no-Doppler IMU-main selector\n";
        return false;
    }
    if (phase184_source_tdcp_huber_k &&
        options.native_phase118_official_tdcp_huber_k) {
        std::cerr << "Phase184 source TDCP Huber-k cannot be combined with "
                     "the separate Phase118 selector\n";
        return false;
    }
    if (phase180_clock_preflight &&
        (phase149_seed_stage || phase163_seed_stage || phase165_graph_stage ||
         phase167_budget || phase171_imu_main)) {
        std::cerr << "Phase180 clock preflight is standalone and cannot be "
                     "combined with a raw-P/FGO selector\n";
        return false;
    }
    if (phase180_clock_preflight &&
        (options.android_gnss_path.empty() || options.android_imu_path.empty() ||
         !options.android_raw_clock_only ||
         !options.android_raw_utc_key_contract || !options.all_epochs ||
         options.skip_epochs != 0 || !options.android_utc_wall_clock_fallback)) {
        std::cerr << "Phase180 clock preflight requires Android raw GNSS+IMU, "
                     "raw-clock-only UTC keys, all epochs, and the explicit "
                     "UTC wall-clock fallback\n";
        return false;
    }
    if (phase171_imu_main &&
        (phase149_seed_stage || phase163_seed_stage ||
         options.native_phase159_collect_all_epochs ||
         options.native_phase143_official_main_lm_termination_budget ||
         options.native_phase135_official_affine_measurement_family ||
         options.native_phase138_affine_tdcp_anchor_range_constant ||
         options.native_direct_wls_ephemeral_c7d_main_seed ||
         options.native_direct_doppler_wls_handoff ||
         options.native_gnss_first_velocity_only_handoff ||
         options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer)) {
        std::cerr << "Phase171 requires the dedicated raw-P GNSS-first handoff "
                     "without prep-only, affine, direct-WLS, velocity-only, "
                     "or raw-drift selectors\n";
        return false;
    }
    if (phase167_budget && options.native_phase143_official_main_lm_termination_budget) {
        std::cerr << "Phase167 dedicated budget cannot be combined with the "
                     "Phase143 main-graph budget\n";
        return false;
    }
    const bool android_raw =
        !options.android_imu_path.empty() ||
        (raw_only_stage && !options.android_gnss_path.empty());
    if (phase171_imu_main) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase171-raw-p-no-doppler-imu-main requires a GTSAM build\n";
        return false;
#endif
        if (!android_raw || options.android_gnss_path.empty() ||
            options.android_imu_path.empty() || !options.android_raw_clock_only ||
            !options.android_raw_utc_key_contract || !options.all_epochs ||
            options.skip_epochs != 0 || !options.native_phase157_raw_p_bootstrap ||
            !options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic ||
            !options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            !options.native_source_clock_c0d_epoch_vector_parity ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
            std::cerr << "Phase171 requires the pinned raw-clock-only Android "
                         "GNSS+IMU, bootstrap, direct-quality/no-bridge, and "
                         "source C0/D epoch-vector handoff recipe\n";
            return false;
        }
    }
    const bool has_observation_file = !options.obs_path.empty();
    const bool ordinary_input_shape =
        !options.nav_path.empty() &&
        (options.imu_path.empty() != options.android_imu_path.empty()) &&
        (options.android_imu_path.empty() == options.android_gnss_path.empty()) &&
        (android_raw != has_observation_file) &&
        !options.out_path.empty() && !options.summary_path.empty();
    const bool raw_only_stage_input_shape =
        !options.nav_path.empty() &&
        (android_raw != has_observation_file) &&
        options.imu_path.empty() && options.android_imu_path.empty() &&
        options.out_path.empty() && !options.summary_path.empty();
    const bool phase180_input_shape =
        options.nav_path.empty() && options.obs_path.empty() &&
        options.imu_path.empty() && !options.android_gnss_path.empty() &&
        !options.android_imu_path.empty() && options.out_path.empty() &&
        !options.summary_path.empty();
    if (phase180_clock_preflight
            ? !phase180_input_shape
            : (raw_only_stage ? !raw_only_stage_input_shape
                               : !ordinary_input_shape)) {
        usage(argv[0]);
        return false;
    }
    const std::array<const std::string*, 7> file_paths = {
        &options.obs_path, &options.nav_path, &options.imu_path,
        &options.android_imu_path, &options.android_gnss_path, &options.out_path,
        &options.native_base_rinex_path};
    for (const std::string* path : file_paths) {
        if (path != nullptr && !path->empty() && hasMatExtension(*path)) {
            std::cerr << "MATLAB .mat paths are forbidden by the native/raw contract\n";
            return false;
        }
    }
    if (hasMatExtension(options.summary_path)) {
        std::cerr << "MATLAB .mat paths are forbidden by the native/raw contract\n";
        return false;
    }
    if (raw_only_stage &&
        (!options.phase104_stage_ecef_path.empty() ||
         !options.phase104_main_displacement_stats_path.empty() ||
         options.native_base_pseudorange_compensation ||
         options.native_base_pseudorange_preserve_additional_frequency_bands ||
         options.native_base_pseudorange_source_miss_mask ||
         !options.native_base_rinex_path.empty() ||
         !options.native_base_rinex_sha256.empty() ||
         options.native_carrier_code_leveling ||
         options.native_carrier_code_innovation_reset ||
         options.native_carrier_code_primary_l1_e1 ||
         options.native_carrier_code_gal_e1_e5a ||
         options.native_pdc_state_bridge || options.native_pdc_imu_tdcp ||
         options.native_pdc_imu_tdcp_no_bridge || options.native_signal_bias_states ||
         options.native_residual_ionosphere || options.native_upstream_quality ||
         options.native_source_direct_observable_quality ||
         options.native_source_clock_c0d_factor ||
         options.native_source_clock_c0d_meter_state_parity ||
         options.native_source_clock_c0d_active_solve_diagnostic ||
         options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
         options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
         options.native_source_clock_c0d_epoch_vector_parity ||
         options.native_source_clock_c0d_phase94_stage_diagnostics ||
         options.native_source_clock_c0d_phase96_main_diagnostics ||
         options.native_source_clock_c0d_phase97_singular_system_diagnostics ||
         options.native_source_clock_c0d_phase98_solver_rank_diagnostic ||
         options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver ||
         options.native_direct_wls_ephemeral_c7d_main_seed ||
         options.native_phase116_carrier_tdcp_incidence_diagnostic ||
         options.native_phase117_tdcp_snr_type_sigma ||
         options.native_phase118_official_tdcp_huber_k ||
         options.native_phase120_official_tdcp_resl_atmosphere_cancellation ||
         options.native_phase126_raw_base_source_complete ||
         options.native_phase127_glonass_channel_provenance ||
         options.native_phase128_glonass_provenance_parser_admission ||
         options.native_phase129_glonass_local_miss_mask ||
         options.native_phase131_canonical_correction_band_key ||
         options.native_phase135_official_affine_measurement_family ||
         options.native_phase138_affine_tdcp_anchor_range_constant ||
         options.native_phase141_telemetry_schema ||
         options.native_phase143_official_main_lm_termination_budget ||
         options.native_phase144_telemetry_schema ||
         options.native_upstream_absolute_doppler_screen ||
         options.native_upstream_stop_constraints || options.native_upstream_position_offset ||
         options.native_signal_specific_galileo_tgd || options.native_quality_anchor ||
         options.native_fallback_seed_quality_anchor_recovery ||
         options.native_android_sv_time_uncertainty_sigma_floor ||
         options.native_cn0_doppler_calibration ||
         options.native_gnss_first_velocity_only_handoff ||
         options.native_direct_doppler_wls_handoff)) {
        std::cerr << "raw-P prep stages accept only raw P input "
                     "and the stage controls\n";
        return false;
    }
    if (options.native_phase157_raw_p_bootstrap &&
        !raw_only_stage && !phase171_imu_main) {
        std::cerr << "--native-phase157-raw-p-bootstrap requires "
                     "a raw-P prep selector\n";
        return false;
    }
    if (options.native_phase159_collect_all_epochs && !raw_only_stage) {
        std::cerr << "--native-phase159-collect-all-epochs requires "
                     "a raw-P prep selector\n";
        return false;
    }
    if (options.native_pdc_imu_tdcp_no_bridge) {
        // This explicit split keeps the existing --native-pdc-imu-tdcp
        // behavior byte-compatible while exposing the same in-process
        // TDCP/IMU/FGO graph without its failed PDC state initializer.
        options.native_pdc_imu_tdcp = true;
    } else if (options.native_pdc_imu_tdcp) {
        // Preserve the historical TDCP recipe: it includes the in-process
        // PDC state bridge unless the explicit no-bridge spelling above was
        // requested.
        options.native_pdc_state_bridge = true;
    }
    if (options.android_raw_utc_key_contract && !android_raw) {
        std::cerr << "--android-raw-utc-keys requires the Android raw GNSS/IMU path\n";
        return false;
    }
    if (options.android_raw_clock_only && !android_raw) {
        std::cerr << "--android-raw-clock-only requires Android raw GNSS/IMU input\n";
        return false;
    }
    if (options.native_android_sv_time_uncertainty_sigma_floor && !android_raw) {
        std::cerr << "--native-android-sv-time-uncertainty-sigma-floor requires "
                     "Android raw GNSS/IMU input\n";
        return false;
    }
    if (options.native_cn0_doppler_calibration && !android_raw) {
        std::cerr << "--native-cn0-doppler-calibration requires Android raw "
                     "GNSS/IMU input\n";
        return false;
    }
    if (options.native_doppler_rotation_rate &&
        (!android_raw || !phase171_imu_main || !phase171_ecef_doppler ||
         !options.native_phase213_main_doppler || options.native_joint_ionosphere)) {
        std::cerr << "Rotation-rate Doppler requires raw Phase171 ECEF-D and Phase213 main, joint ionosphere off\n";
        return false;
    }
    if (options.native_tdcp_adr_endpoint_sigma &&
        (!android_raw || !phase171_imu_main || !phase171_ecef_doppler ||
         !options.native_source_tdcp_meter_sigma || !options.native_phase213_main_doppler ||
         options.native_joint_ionosphere || options.native_tdcp_frequency_residual_states ||
         options.native_doppler_rotation_rate)) {
        std::cerr << "ADR endpoint sigma requires dedicated raw Phase171/213 and source metre sigma, other noise experiments off\n";
        return false;
    }
    if (options.native_joint_ionosphere &&
        (!android_raw || !phase171_imu_main || !phase171_ecef_doppler ||
         !options.all_epochs || options.skip_epochs != 0 || options.native_batch_nhc ||
         options.native_main_code_edge_readmission || options.native_main_code_uncertainty_floor ||
         options.native_android_sv_time_uncertainty_sigma_floor ||
         options.native_tdcp_frequency_residual_states)) {
        std::cerr << "Joint ionosphere requires dedicated raw Phase171 ECEF-D main scope\n";
        return false;
    }
    if (options.native_main_code_edge_readmission &&
        (!android_raw || !phase171_imu_main || !phase171_ecef_doppler ||
         !options.all_epochs || options.skip_epochs != 0 ||
         options.native_main_code_uncertainty_floor ||
         options.native_android_sv_time_uncertainty_sigma_floor ||
         options.native_batch_nhc || options.native_tdcp_frequency_residual_states)) {
        std::cerr << "Main code edge readmission requires raw all-epoch Phase171 ECEF-D without code floors, NHC or frequency states\n";
        return false;
    }
    if (options.native_main_code_uncertainty_floor &&
        (!android_raw || !phase171_imu_main || !phase171_ecef_doppler ||
         options.native_android_sv_time_uncertainty_sigma_floor ||
         options.native_batch_nhc || options.native_tdcp_frequency_residual_states)) {
        std::cerr << "Main code uncertainty floor requires raw Phase171 ECEF-D without builder floor, NHC or frequency states\n";
        return false;
    }
    if ((options.native_nhc_monitor || options.native_batch_nhc) &&
        (!android_raw || !phase171_imu_main || !phase171_ecef_doppler)) {
        std::cerr << "Native NHC requires Android raw Phase171 ECEF-D\n";
        return false;
    }
    if (options.native_lm_lambda_floor &&
        (!phase171_imu_main || !phase171_ecef_doppler ||
         options.native_tdcp_frequency_residual_states)) {
        std::cerr << "--native-lm-lambda-floor requires Phase171 ECEF-D without frequency-state experiment\n";
        return false;
    }
    if (options.native_tdcp_frequency_residual_states &&
        (!android_raw || !options.android_raw_clock_only ||
         !options.android_raw_utc_key_contract || !options.all_epochs ||
         options.skip_epochs != 0 || !phase171_imu_main || !phase171_ecef_doppler ||
         !options.native_source_tdcp_meter_sigma || !options.native_phase213_main_doppler ||
         !options.native_phase217_main_pose3_motion ||
         !options.native_phase197_source_utc_fallback_imu_offset ||
         options.native_tdcp_no_code_jump_gate || options.native_base_pseudorange_compensation ||
         options.native_main_p_cauchy || options.native_stationary_gyro_initializer ||
         options.native_epoch_heading_attitude_seeds ||
         options.native_phase120_official_tdcp_resl_atmosphere_cancellation ||
         (options.dataset_id != "2021-08-24-20-32-us-ca-mtv-h/pixel5" &&
          options.dataset_id != "2022-04-01-18-22-us-ca-lax-t/pixel5"))) {
        std::cerr << "--native-tdcp-frequency-residual-states requires raw H/LAX-T baseline frequency experiment recipe\n";
        return false;
    }
    if (options.native_sparse_p_staging &&
        (!android_raw || !options.android_raw_clock_only ||
         !options.android_raw_utc_key_contract || !options.all_epochs ||
         options.skip_epochs != 0 || !phase171_imu_main || !phase171_ecef_doppler ||
         options.native_base_pseudorange_compensation || options.native_main_p_cauchy ||
         options.native_stationary_gyro_initializer || options.native_epoch_heading_attitude_seeds ||
         options.dataset_id != "2022-04-01-18-22-us-ca-lax-t/pixel5")) {
        std::cerr << "--native-sparse-p-staging requires raw LAX-T all-epoch Phase171 base-off recipe\n";
        return false;
    }
    if (options.native_tdcp_no_code_jump_gate &&
        (!android_raw || !options.android_raw_clock_only ||
         !options.android_raw_utc_key_contract || !options.all_epochs ||
         options.skip_epochs != 0 || !phase171_imu_main || !phase171_ecef_doppler ||
         options.native_base_pseudorange_compensation || options.native_main_p_cauchy ||
         options.native_stationary_gyro_initializer || options.native_epoch_heading_attitude_seeds ||
         (options.dataset_id != "2021-08-24-20-32-us-ca-mtv-h/pixel5" &&
          options.dataset_id != "2023-03-08-21-34-us-ca-mtv-u/pixel5" &&
          options.dataset_id != "2022-04-01-18-22-us-ca-lax-t/pixel5"))) {
        std::cerr << "--native-tdcp-no-code-jump-gate requires raw H/U/LAX-T all-epoch Phase171 base-off recipe\n";
        return false;
    }
    if (options.native_stationary_gyro_initializer &&
        (!android_raw || !options.android_raw_clock_only ||
         !options.android_raw_utc_key_contract || !options.all_epochs ||
         options.skip_epochs != 0 || !phase171_imu_main || !phase171_ecef_doppler ||
         !options.native_phase197_source_utc_fallback_imu_offset ||
         options.native_base_pseudorange_compensation || options.native_main_p_cauchy ||
         options.native_epoch_heading_attitude_seeds ||
         options.dataset_id != "2021-08-24-20-32-us-ca-mtv-h/pixel5")) {
        std::cerr << "--native-stationary-gyro-initializer requires raw H all-epoch Phase171 base-off Phase197 recipe\n";
        return false;
    }
    if (options.native_main_p_cauchy &&
        (!android_raw || !options.android_raw_clock_only ||
         !options.android_raw_utc_key_contract || !options.all_epochs ||
         options.skip_epochs!=0 || !phase171_imu_main || !phase171_ecef_doppler ||
         options.native_base_pseudorange_compensation || options.native_paired_epoch_states ||
         options.dataset_id!="2021-08-24-20-32-us-ca-mtv-h/pixel5")) {
        std::cerr << "--native-main-p-cauchy requires raw H all-epoch Phase171 base-off recipe\n";
        return false;
    }
    if (options.native_base_gps_center_ablation && !options.native_base_gps_values_only_ablation) {
        std::cerr << "--native-base-gps-center-ablation requires --native-base-gps-values-only-ablation\n";
        return false;
    }
    if (options.native_base_gps_values_only_ablation &&
        (!options.native_paired_epoch_states || !options.native_dense_base_smoothing ||
         options.native_base_mask_only_ablation)) {
        std::cerr << "--native-base-gps-values-only-ablation requires paired/dense and rejects mask-only\n";
        return false;
    }
    if (options.native_base_mask_only_ablation &&
        (!options.native_paired_epoch_states || !options.native_dense_base_smoothing)) {
        std::cerr << "--native-base-mask-only-ablation requires paired epoch states and dense smoothing\n";
        return false;
    }
    if (options.native_dense_base_smoothing && !options.native_paired_epoch_states) {
        std::cerr << "--native-dense-base-smoothing requires --native-paired-epoch-states\n";
        return false;
    }
    if (options.native_rover_epoch_states &&
        (!android_raw || !options.android_raw_clock_only ||
         !options.android_raw_utc_key_contract || !options.all_epochs ||
         options.skip_epochs != 0 || !phase171_imu_main || !phase171_ecef_doppler ||
         options.native_base_pseudorange_compensation || options.native_paired_epoch_states ||
         options.native_phase126_raw_base_source_complete ||
         options.dataset_id != "2021-08-24-20-32-us-ca-mtv-h/pixel5")) {
        std::cerr << "--native-rover-epoch-states requires raw H Phase171 all-epoch "
                     "UTC recipe with base/paired/Phase126 off\n";
        return false;
    }
    if (options.native_paired_epoch_states) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-paired-epoch-states requires GTSAM\n";
        return false;
#endif
        if (!android_raw || !options.android_raw_clock_only ||
            !options.android_raw_utc_key_contract || !options.all_epochs ||
            options.skip_epochs != 0 ||
            !options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            !options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_phase126_raw_base_source_complete ||
            options.native_phase127_glonass_channel_provenance ||
            options.native_phase128_glonass_provenance_parser_admission ||
            options.native_phase129_glonass_local_miss_mask ||
            options.native_phase131_canonical_correction_band_key ||
            options.native_signal_specific_galileo_tgd ||
            options.dataset_id != "2021-08-24-20-32-us-ca-mtv-h/pixel5") {
            std::cerr << "--native-paired-epoch-states requires raw H all-epoch "
                         "UTC/meter-handoff/base/miss-mask recipe without legacy "
                         "Phase126-131, extra-band, or signal-specific TGD selectors\n";
            return false;
        }
    }
    if (options.native_base_pseudorange_compensation && !android_raw) {
        std::cerr << "--native-base-pseudorange-compensation requires Android raw "
                     "GNSS/IMU input\n";
        return false;
    }
    if (options.native_base_pseudorange_source_miss_mask && !android_raw) {
        std::cerr << "--native-base-pseudorange-source-miss-mask requires Android "
                     "raw GNSS/IMU input\n";
        return false;
    }
    if (options.native_base_pseudorange_source_miss_mask &&
        !options.native_base_pseudorange_compensation) {
        std::cerr << "--native-base-pseudorange-source-miss-mask requires "
                     "--native-base-pseudorange-compensation\n";
        return false;
    }
    if (options.native_base_pseudorange_preserve_additional_frequency_bands &&
        !options.native_base_pseudorange_compensation) {
        std::cerr << "--native-base-pseudorange-preserve-additional-frequency-bands "
                     "requires --native-base-pseudorange-compensation\n";
        return false;
    }
    if (options.native_base_pseudorange_source_miss_mask &&
        options.native_base_pseudorange_preserve_additional_frequency_bands &&
        !options.native_source_clock_c0d_gnss_first_meter_state_handoff) {
        // "--native-base-pseudorange-source-miss-mask cannot be combined"
        // with the additional-band selector outside the sealed handoff
        // admission.
        std::cerr << "--native-base-pseudorange-source-miss-mask plus "
                     "--native-base-pseudorange-preserve-additional-frequency-bands "
                     "requires the Phase101 meter-state handoff\n";
        return false;
    }
    if (options.native_base_pseudorange_compensation &&
        options.native_base_rinex_path.empty()) {
        std::cerr << "--native-base-pseudorange-compensation requires "
                     "--native-base-rinex\n";
        return false;
    }
    if (options.native_base_pseudorange_compensation &&
        options.native_base_rinex_sha256.size() != 64U) {
        std::cerr << "--native-base-pseudorange-compensation requires the "
                     "64-character manifest-verified base SHA-256\n";
        return false;
    }
    if (options.native_base_pseudorange_compensation &&
        !std::all_of(options.native_base_rinex_sha256.begin(),
                     options.native_base_rinex_sha256.end(),
                     [](unsigned char ch) { return std::isxdigit(ch) != 0; })) {
        std::cerr << "--native-base-rinex-sha256 must contain only hexadecimal digits\n";
        return false;
    }
    if (!options.native_base_pseudorange_compensation &&
        !options.native_base_rinex_path.empty()) {
        std::cerr << "--native-base-rinex requires "
                     "--native-base-pseudorange-compensation\n";
        return false;
    }
    if (!options.native_base_pseudorange_compensation &&
        !options.native_base_rinex_sha256.empty()) {
        std::cerr << "--native-base-rinex-sha256 requires "
                     "--native-base-pseudorange-compensation\n";
        return false;
    }
    if (options.android_raw_utc_key_contract &&
        (!options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--android-raw-utc-keys requires --all-epochs without --skip-epochs\n";
        return false;
    }
    if (options.android_utc_wall_clock_fallback &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--android-utc-wall-clock-fallback requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.fgo_imu_sparse_recovery &&
        (!android_raw || !options.android_raw_utc_key_contract)) {
        std::cerr << "--fgo-imu-sparse-recovery requires Android raw input and "
                     "--android-raw-utc-keys\n";
        return false;
    }
    if (options.native_pdc_state_bridge &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-pdc-state-bridge requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_pdc_imu_tdcp &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-pdc-imu-tdcp requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_pdc_imu_tdcp_no_bridge && options.native_pdc_state_bridge) {
        std::cerr << "--native-pdc-imu-tdcp-no-bridge conflicts with "
                     "--native-pdc-state-bridge\n";
        return false;
    }
    if (options.native_pdc_imu_tdcp_no_bridge && options.native_upstream_quality) {
        std::cerr << "--native-pdc-imu-tdcp-no-bridge is incompatible with "
                     "--native-upstream-quality, which requires the PDC bridge\n";
        return false;
    }
    if (options.native_source_direct_observable_quality &&
        options.native_upstream_quality) {
        std::cerr << "--native-source-direct-observable-quality conflicts with "
                     "--native-upstream-quality; choose direct/no-PDC or the "
                     "legacy PDC-bridge quality path\n";
        return false;
    }
    if (options.native_source_direct_observable_quality &&
        !options.native_pdc_imu_tdcp_no_bridge) {
        std::cerr << "--native-source-direct-observable-quality requires "
                     "--native-pdc-imu-tdcp-no-bridge\n";
        return false;
    }
    if (options.native_source_direct_observable_quality &&
        options.native_pdc_state_bridge) {
        std::cerr << "--native-source-direct-observable-quality conflicts with "
                     "the PDC state bridge\n";
        return false;
    }
    if (options.native_source_direct_observable_quality &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-source-direct-observable-quality requires Android "
                     "raw input, --android-raw-utc-keys, --all-epochs, and no "
                     "skipped epochs\n";
        return false;
    }
    if (options.native_source_direct_observable_quality) {
        const DirectObservableQualitySettings direct_settings =
            directObservableQualitySettingsForDataset(options.dataset_id);
        if (!direct_settings.valid) {
            std::cerr << "--native-source-direct-observable-quality requires one "
                         "of the four pinned Phase80 dataset IDs (exact match): "
                         "MTV-a, MTV-h, LAX-t, or MTV-u\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_factor) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-factor requires a GTSAM build; "
                     "the Eigen backend is not source-exact for this candidate\n";
        return false;
#endif
        if (!options.native_source_direct_observable_quality) {
            std::cerr << "--native-source-clock-c0d-factor requires "
                         "--native-source-direct-observable-quality\n";
            return false;
        }
        if (!options.native_pdc_imu_tdcp_no_bridge) {
            std::cerr << "--native-source-clock-c0d-factor requires "
                         "--native-pdc-imu-tdcp-no-bridge\n";
            return false;
        }
        if (options.native_pdc_state_bridge || options.native_upstream_quality) {
            std::cerr << "--native-source-clock-c0d-factor conflicts with the "
                         "PDC bridge and legacy --native-upstream-quality\n";
            return false;
        }
        const std::string phone = phoneFromDatasetId(options.dataset_id);
        if (phone.empty()) {
            std::cerr << "--native-source-clock-c0d-factor requires an exact "
                         "dataset phone identity\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_meter_state_parity) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-meter-state-parity requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_source_clock_c0d_factor ||
            !options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_active_solve_diagnostic ||
            (!options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer &&
             !options.native_source_clock_c0d_gnss_first_meter_state_handoff &&
             !options.native_direct_wls_ephemeral_c7d_main_seed)) {
            std::cerr << "--native-source-clock-c0d-meter-state-parity requires "
                         "the direct-quality, no-bridge, active-diagnostic, "
            "and a frozen raw-drift, Phase93, or Phase114 C0/D recipe\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_active_solve_diagnostic) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-active-solve-diagnostic "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_source_clock_c0d_factor) {
            std::cerr << "--native-source-clock-c0d-active-solve-diagnostic "
                         "requires --native-source-clock-c0d-factor\n";
            return false;
        }
        if (!options.native_gnss_first_velocity_only_handoff &&
            !options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer &&
            !options.native_source_clock_c0d_gnss_first_meter_state_handoff &&
            !options.native_direct_wls_ephemeral_c7d_main_seed) {
            std::cerr << "--native-source-clock-c0d-active-solve-diagnostic "
                         "requires --native-gnss-first-velocity-only-handoff, "
                         "the Phase91 raw-drift D initializer, or Phase93 "
                         "meter-state handoff, or Phase114 direct-WLS seed\n";
            return false;
        }
        if (options.native_pdc_state_bridge ||
            options.native_direct_doppler_wls_handoff) {
            std::cerr << "--native-source-clock-c0d-active-solve-diagnostic "
                         "requires no PDC bridge and no alternate velocity "
                         "handoff\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.all_epochs || options.skip_epochs != 0) {
            std::cerr << "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer "
                         "requires Android raw input, --android-raw-utc-keys, "
                         "--all-epochs, and no skipped epochs\n";
            return false;
        }
        if (!options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_active_solve_diagnostic) {
            std::cerr << "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer "
                         "requires the direct-quality, no-bridge C0/D recipe and "
                         "active-solve diagnostic\n";
            return false;
        }
        if (options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality) {
            std::cerr << "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer "
                         "forbids PDC, direct-WLS, velocity-only, and legacy "
                         "upstream handoffs\n";
            return false;
        }
        if (options.native_base_pseudorange_compensation ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_pseudorange_source_miss_mask ||
            !options.native_base_rinex_path.empty() ||
            !options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer "
                         "forbids base/external coordinate inputs\n";
            return false;
        }
        if (options.fgo_imu_sparse_recovery || options.native_signal_bias_states ||
            options.native_residual_ionosphere ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            options.native_upstream_stop_constraints ||
            options.native_upstream_position_offset ||
            options.native_signal_specific_galileo_tgd ||
            options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer "
                         "requires the frozen standard GNSS-first/raw C0/D recipe "
                         "without other optional candidate switches\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-gnss-first-meter-state-handoff "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.all_epochs || options.skip_epochs != 0) {
            std::cerr << "--native-source-clock-c0d-gnss-first-meter-state-handoff "
                         "requires Android raw input, --android-raw-utc-keys, "
                         "--all-epochs, and no skipped epochs\n";
            return false;
        }
        if (!options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic) {
            std::cerr << "--native-source-clock-c0d-gnss-first-meter-state-handoff "
                         "requires the direct-quality, no-bridge, C0/D, "
                         "meter-state, and active-diagnostic recipe\n";
            return false;
        }
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality) {
            std::cerr << "--native-source-clock-c0d-gnss-first-meter-state-handoff "
                         "is mutually exclusive with raw-D-only, PDC, direct-WLS, "
                         "velocity-only, and legacy upstream handoffs\n";
            return false;
        }
        // Phase107 and Phase109 are the sole admissions for the already
        // implemented native raw-base correction path alongside the Phase101
        // handoff.  The correction is applied once to the shared problem
        // before the GNSS-first copy and main handoff; preserve the existing
        // equations, factors, and all solver/configuration behavior.  Phase109
        // only admits the existing additional-band reader under this exact
        // recipe; it does not introduce a new signal map or fallback.
        const bool has_any_base_input =
            options.native_base_pseudorange_compensation ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_pseudorange_source_miss_mask ||
            !options.native_base_rinex_path.empty() ||
            !options.native_base_rinex_sha256.empty();
        const bool phase101_exact_selector_recipe =
            options.native_source_clock_c0d_gnss_first_meter_state_handoff &&
            options.native_source_clock_c0d_epoch_vector_parity &&
            options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver;
        const bool phase107_raw_base_source_parity_admission =
            phase101_exact_selector_recipe &&
            options.native_base_pseudorange_compensation &&
            options.native_base_pseudorange_source_miss_mask &&
            !options.native_base_pseudorange_preserve_additional_frequency_bands &&
            !options.native_base_rinex_path.empty() &&
            !options.native_base_rinex_sha256.empty();
        const bool phase109_raw_base_frequency_parity_admission =
            phase101_exact_selector_recipe &&
            options.native_base_pseudorange_compensation &&
            options.native_base_pseudorange_source_miss_mask &&
            options.native_base_pseudorange_preserve_additional_frequency_bands &&
            !options.native_base_rinex_path.empty() &&
            !options.native_base_rinex_sha256.empty();
        const bool phase101_raw_base_source_parity_admission =
            phase107_raw_base_source_parity_admission ||
            phase109_raw_base_frequency_parity_admission;
        if (has_any_base_input && !phase101_raw_base_source_parity_admission) {
            std::cerr << "--native-source-clock-c0d-gnss-first-meter-state-handoff "
                         "permits only the exact Phase101 selectors with "
                         "Phase107 raw-base RINEX or Phase109 additional-frequency-band "
                         "admission\n";
            return false;
        }
        if (options.native_upstream_position_offset &&
            (!phase112MainOutputPositionOffsetSelectors(options) ||
             phoneFromDatasetId(options.dataset_id) != "pixel5")) {
            std::cerr << "--native-upstream-position-offset with the Phase101 "
                         "meter-state handoff requires the exact Phase112 "
                         "C7/D/QR main recipe and dataset phone pixel5\n";
            return false;
        }
        if (options.fgo_imu_sparse_recovery || options.native_signal_bias_states ||
            options.native_residual_ionosphere ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            (options.native_upstream_stop_constraints && !phase171_imu_main) ||
            options.native_signal_specific_galileo_tgd ||
            options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-source-clock-c0d-gnss-first-meter-state-handoff "
                         "requires the frozen Phase93 recipe without other "
                         "optional candidate switches\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_epoch_vector_parity) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-epoch-vector-parity requires a GTSAM build\n";
        return false;
#endif
        if ((!options.native_source_clock_c0d_gnss_first_meter_state_handoff &&
             !options.native_direct_wls_ephemeral_c7d_main_seed) ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
            std::cerr << "--native-source-clock-c0d-epoch-vector-parity requires "
                         "the Phase93/114 handoff and Phase99 QR main recipe\n";
            return false;
        }
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality ||
            options.native_signal_bias_states || options.native_residual_ionosphere) {
            std::cerr << "--native-source-clock-c0d-epoch-vector-parity forbids "
                         "legacy/PDC/alternate state families\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_phase94_stage_diagnostics) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-phase94-stage-diagnostics "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_source_clock_c0d_gnss_first_meter_state_handoff) {
            std::cerr << "--native-source-clock-c0d-phase94-stage-diagnostics "
                         "requires --native-source-clock-c0d-gnss-first-meter-state-handoff\n";
            return false;
        }
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.all_epochs || options.skip_epochs != 0) {
            std::cerr << "--native-source-clock-c0d-phase94-stage-diagnostics "
                         "requires Android raw input, --android-raw-utc-keys, "
                         "--all-epochs, and no skipped epochs\n";
            return false;
        }
        if (!options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic) {
            std::cerr << "--native-source-clock-c0d-phase94-stage-diagnostics "
                         "requires the frozen Phase93 direct-quality, no-bridge, "
                         "C0/D, meter-state, and active-diagnostic recipe\n";
            return false;
        }
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality) {
            std::cerr << "--native-source-clock-c0d-phase94-stage-diagnostics "
                         "forbids raw-D-only, PDC, direct-WLS, velocity-only, "
                         "and legacy upstream handoffs\n";
            return false;
        }
        if (options.native_base_pseudorange_compensation ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_pseudorange_source_miss_mask ||
            !options.native_base_rinex_path.empty() ||
            !options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-source-clock-c0d-phase94-stage-diagnostics "
                         "forbids base/external coordinate inputs\n";
            return false;
        }
        if (options.fgo_imu_sparse_recovery || options.native_signal_bias_states ||
            options.native_residual_ionosphere ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            options.native_upstream_stop_constraints ||
            options.native_upstream_position_offset ||
            options.native_signal_specific_galileo_tgd ||
            options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-source-clock-c0d-phase94-stage-diagnostics "
                         "requires the frozen Phase93 recipe without other "
                         "optional candidate switches\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_phase96_main_diagnostics) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-phase96-main-diagnostics "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_source_clock_c0d_gnss_first_meter_state_handoff) {
            std::cerr << "--native-source-clock-c0d-phase96-main-diagnostics "
                         "requires the Phase93 GNSS-first meter-state handoff\n";
            return false;
        }
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.all_epochs || options.skip_epochs != 0) {
            std::cerr << "--native-source-clock-c0d-phase96-main-diagnostics "
                         "requires Android raw input, --android-raw-utc-keys, "
                         "--all-epochs, and no skipped epochs\n";
            return false;
        }
        if (!options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic) {
            std::cerr << "--native-source-clock-c0d-phase96-main-diagnostics "
                         "requires the frozen Phase93 direct-quality, "
                         "no-bridge, C0/D, meter-state, and active-diagnostic recipe\n";
            return false;
        }
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality ||
            options.native_base_pseudorange_compensation ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_pseudorange_source_miss_mask ||
            !options.native_base_rinex_path.empty() ||
            !options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-source-clock-c0d-phase96-main-diagnostics "
                         "forbids raw-D-only, alternate/PDC, or base/external "
                         "inputs\n";
            return false;
        }
        if (options.fgo_imu_sparse_recovery || options.native_signal_bias_states ||
            options.native_residual_ionosphere ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            options.native_upstream_stop_constraints ||
            options.native_upstream_position_offset ||
            options.native_signal_specific_galileo_tgd ||
            options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-source-clock-c0d-phase96-main-diagnostics "
                         "requires the frozen Phase93 recipe without other "
                         "optional candidate switches\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_phase97_singular_system_diagnostics) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-phase97-singular-system-diagnostics "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_source_clock_c0d_gnss_first_meter_state_handoff) {
            std::cerr << "--native-source-clock-c0d-phase97-singular-system-diagnostics "
                         "requires the Phase93 GNSS-first meter-state handoff\n";
            return false;
        }
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.all_epochs || options.skip_epochs != 0) {
            std::cerr << "--native-source-clock-c0d-phase97-singular-system-diagnostics "
                         "requires Android raw input, --android-raw-utc-keys, "
                         "--all-epochs, and no skipped epochs\n";
            return false;
        }
        if (!options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic) {
            std::cerr << "--native-source-clock-c0d-phase97-singular-system-diagnostics "
                         "requires the frozen Phase93 direct-quality, "
                         "no-bridge, C0/D, meter-state, and active-diagnostic recipe\n";
            return false;
        }
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality ||
            options.native_base_pseudorange_compensation ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_pseudorange_source_miss_mask ||
            !options.native_base_rinex_path.empty() ||
            !options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-source-clock-c0d-phase97-singular-system-diagnostics "
                         "forbids raw-D-only, alternate/PDC, or base/external inputs\n";
            return false;
        }
        if (options.fgo_imu_sparse_recovery || options.native_signal_bias_states ||
            options.native_residual_ionosphere ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            options.native_upstream_stop_constraints ||
            options.native_upstream_position_offset ||
            options.native_signal_specific_galileo_tgd ||
            options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-source-clock-c0d-phase97-singular-system-diagnostics "
                         "requires the frozen Phase93 recipe without other optional switches\n";
            return false;
        }
    }
    if (options.native_source_clock_c0d_phase98_solver_rank_diagnostic) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-phase98-solver-rank-diagnostic "
                     "requires a GTSAM build\n";
        return false;
#endif
    }
    if (options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_source_clock_c0d_gnss_first_meter_state_handoff &&
            !options.native_direct_wls_ephemeral_c7d_main_seed) {
            std::cerr << "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver "
                         "requires the Phase93 GNSS-first or Phase114 direct-WLS "
                         "meter-state handoff\n";
            return false;
        }
        // The preceding Phase93 handoff validation owns the complete
        // direct-quality/no-bridge/C0/D/meter-state recipe and rejects the
        // alternate raw-D, PDC, base, and tuning switches.  This selector is
        // deliberately subordinate to that exact contract.
    }
    if (options.native_phase116_carrier_tdcp_incidence_diagnostic) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase116-carrier-tdcp-incidence-diagnostic "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.android_raw_clock_only ||
            !options.android_utc_wall_clock_fallback || !options.all_epochs ||
            options.skip_epochs != 0) {
            std::cerr << "--native-phase116-carrier-tdcp-incidence-diagnostic "
                         "requires the exact raw UTC/all-epoch recipe\n";
            return false;
        }
        if (!options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_direct_observable_quality ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic ||
            !options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            !options.native_source_clock_c0d_epoch_vector_parity ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
            std::cerr << "--native-phase116-carrier-tdcp-incidence-diagnostic "
                         "requires the frozen Phase101 C7/D/QR ordinary-TDCP "
                         "recipe\n";
            return false;
        }
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_direct_wls_ephemeral_c7d_main_seed ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality ||
            options.native_source_clock_c0d_phase94_stage_diagnostics ||
            options.native_source_clock_c0d_phase96_main_diagnostics ||
            options.native_source_clock_c0d_phase97_singular_system_diagnostics ||
            options.native_source_clock_c0d_phase98_solver_rank_diagnostic ||
            options.native_phase104_stage_main_accuracy_attribution) {
            std::cerr << "--native-phase116-carrier-tdcp-incidence-diagnostic "
                         "forbids alternate handoffs, PDC bridge, and other "
                         "diagnostic/output selectors\n";
            return false;
        }
        if (!options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            !options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-phase116-carrier-tdcp-incidence-diagnostic "
                         "requires the existing Phase109 raw-base admission\n";
            return false;
        }
        const bool supported_route =
            options.dataset_id == "2021-03-16-18-59-us-ca-mtv-a/pixel5" ||
            options.dataset_id == "2022-04-01-18-22-us-ca-lax-t/pixel5";
        if (!supported_route) {
            std::cerr << "--native-phase116-carrier-tdcp-incidence-diagnostic "
                         "is restricted to the frozen MTV-A and LAX-T routes\n";
            return false;
        }
        if (!options.native_upstream_position_offset ||
            phoneFromDatasetId(options.dataset_id) != "pixel5" ||
            !phase112MainOutputPositionOffsetSelectors(options)) {
            std::cerr << "--native-phase116-carrier-tdcp-incidence-diagnostic "
                         "requires the exact Pixel5 Phase112 output boundary\n";
            return false;
        }
        if (options.fgo_imu_sparse_recovery || options.native_signal_bias_states ||
            options.native_residual_ionosphere ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            options.native_upstream_stop_constraints ||
            options.native_signal_specific_galileo_tgd || options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-phase116-carrier-tdcp-incidence-diagnostic "
                         "requires the frozen raw/base recipe without other "
                         "candidate switches\n";
            return false;
        }
    }
    if (options.native_phase117_tdcp_snr_type_sigma &&
        !options.native_phase116_carrier_tdcp_incidence_diagnostic) {
        std::cerr << "--native-phase117-tdcp-snr-type-sigma requires the "
                     "frozen Phase116 ordinary-TDCP diagnostic recipe\n";
        return false;
    }
    if (options.native_phase118_official_tdcp_huber_k) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase118-official-tdcp-huber-k requires a GTSAM build\n";
        return false;
#endif
        // Phase118 is intentionally admitted only as a composition on the
        // sealed Phase112 champion recipe.  This keeps the Huber-k change
        // isolated from alternate seed, graph, base, output, and diagnostic
        // lanes while preserving the existing Phase112 fixed 0.03 m sigma.
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.android_raw_clock_only ||
            !options.android_utc_wall_clock_fallback || !options.all_epochs ||
            options.skip_epochs != 0) {
            std::cerr << "--native-phase118-official-tdcp-huber-k requires the "
                         "exact Phase112 raw UTC/all-epoch recipe\n";
            return false;
        }
        if (!options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_direct_observable_quality ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic ||
            !options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            options.native_direct_wls_ephemeral_c7d_main_seed ||
            !options.native_source_clock_c0d_epoch_vector_parity ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
            std::cerr << "--native-phase118-official-tdcp-huber-k requires the "
                         "Phase112 GNSS-first C7/D/C0D/meter/QR recipe\n";
            return false;
        }
        if (options.native_phase117_tdcp_snr_type_sigma) {
            std::cerr << "--native-phase118-official-tdcp-huber-k cannot be "
                         "combined with Phase117 dynamic TDCP sigma\n";
            return false;
        }
        // Phase118 historically composes with the Phase109 additional-band
        // admission.  Phase126 is the sole source-complete exception and
        // deliberately excludes that selector, while retaining every other
        // Phase118 setting.
        const bool phase126_source_complete_base =
            options.native_phase126_raw_base_source_complete;
        const bool phase135_phase107_raw_base =
            options.native_phase135_official_affine_measurement_family &&
            !options.native_phase126_raw_base_source_complete &&
            !options.native_phase127_glonass_channel_provenance &&
            !options.native_phase128_glonass_provenance_parser_admission &&
            !options.native_phase129_glonass_local_miss_mask &&
            !options.native_phase131_canonical_correction_band_key &&
            options.native_base_pseudorange_compensation &&
            options.native_base_pseudorange_source_miss_mask &&
            !options.native_base_pseudorange_preserve_additional_frequency_bands &&
            !options.native_base_rinex_path.empty() &&
            !options.native_base_rinex_sha256.empty();
        const bool phase118_base_recipe =
            phase126_source_complete_base
                ? !options.native_base_pseudorange_preserve_additional_frequency_bands
                : (phase135_phase107_raw_base
                       ? !options.native_base_pseudorange_preserve_additional_frequency_bands
                       : options.native_base_pseudorange_preserve_additional_frequency_bands);
        if (!options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            !phase118_base_recipe || options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-phase118-official-tdcp-huber-k requires the "
                         "existing Phase112 raw-base correction recipe (or the "
                         "Phase126 source-complete raw-base admission)\n";
            return false;
        }
        if (!options.native_upstream_position_offset ||
            phoneFromDatasetId(options.dataset_id) != "pixel5" ||
            !phase112MainOutputPositionOffsetSelectors(options)) {
            std::cerr << "--native-phase118-official-tdcp-huber-k requires the "
                         "exact Phase112 Pixel5 final-output boundary\n";
            return false;
        }
        const DirectObservableQualitySettings direct_settings =
            directObservableQualitySettingsForDataset(options.dataset_id);
        if (!direct_settings.valid ||
            !std::isfinite(direct_settings.official_tdcp_huber_threshold_sigma) ||
            !(direct_settings.official_tdcp_huber_threshold_sigma > 0.0)) {
            std::cerr << "--native-phase118-official-tdcp-huber-k requires an "
                         "exact route with a recognised source setting.Type\n";
            return false;
        }
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality ||
            options.fgo_imu_sparse_recovery || options.native_signal_bias_states ||
            options.native_residual_ionosphere ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            options.native_upstream_stop_constraints ||
            options.native_signal_specific_galileo_tgd || options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-phase118-official-tdcp-huber-k requires the "
                         "Phase112 recipe without alternate or tuning switches\n";
            return false;
        }
    }
    if (options.native_phase120_official_tdcp_resl_atmosphere_cancellation) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase120-official-tdcp-resl-atmosphere-cancellation "
                     "requires a GTSAM build\n";
        return false;
#endif
        // Phase120 is deliberately a composition on the sealed Phase118
        // recipe: this keeps fixed sigma=0.03 m, the route-Type k mapping,
        // C7/D/C0D, QR, raw-base, Pixel5 offset, and all other gates explicit.
        if (!options.native_phase118_official_tdcp_huber_k) {
            std::cerr << "--native-phase120-official-tdcp-resl-atmosphere-cancellation "
                         "requires --native-phase118-official-tdcp-huber-k\n";
            return false;
        }
        if (!options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_pdc_imu_tdcp || !android_raw ||
            !options.android_raw_utc_key_contract ||
            !options.android_raw_clock_only ||
            !options.android_utc_wall_clock_fallback || !options.all_epochs ||
            options.skip_epochs != 0) {
            std::cerr << "--native-phase120-official-tdcp-resl-atmosphere-cancellation "
                         "requires the exact Phase118 raw/no-bridge all-epoch TDCP recipe\n";
            return false;
        }
        if (options.native_phase117_tdcp_snr_type_sigma) {
            std::cerr << "--native-phase120-official-tdcp-resl-atmosphere-cancellation "
                         "cannot be combined with Phase117 dynamic TDCP sigma\n";
            return false;
        }
        const DirectObservableQualitySettings direct_settings =
            directObservableQualitySettingsForDataset(options.dataset_id);
        if (!direct_settings.valid || direct_settings.environment.empty() ||
            !std::isfinite(direct_settings.official_tdcp_huber_threshold_sigma) ||
            !(direct_settings.official_tdcp_huber_threshold_sigma > 0.0)) {
            std::cerr << "--native-phase120-official-tdcp-resl-atmosphere-cancellation "
                         "requires one exact route with a recognised Type/k metadata row\n";
            return false;
        }
    }
    if (options.native_phase126_raw_base_source_complete) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase126-raw-base-source-complete requires a GTSAM build\n";
        return false;
#endif
        // Phase126 is one compound admission on the already sealed
        // Phase118 recipe.  The selector is intentionally not decomposed
        // into independently runnable state/geometry/atmosphere knobs.
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.android_raw_clock_only ||
            !options.android_utc_wall_clock_fallback || !options.all_epochs ||
            options.skip_epochs != 0) {
            std::cerr << "--native-phase126-raw-base-source-complete requires "
                         "the exact Phase118 raw UTC/all-epoch recipe\n";
            return false;
        }
        if (!options.native_phase118_official_tdcp_huber_k ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_direct_observable_quality ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic ||
            !options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            !options.native_source_clock_c0d_epoch_vector_parity ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
            std::cerr << "--native-phase126-raw-base-source-complete requires "
                         "the sealed Phase118 C7/D/C0D/QR recipe\n";
            return false;
        }
        if (options.native_phase117_tdcp_snr_type_sigma ||
            options.native_phase120_official_tdcp_resl_atmosphere_cancellation ||
            options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_direct_wls_ephemeral_c7d_main_seed ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff || options.native_pdc_state_bridge ||
            options.native_upstream_quality ||
            options.native_signal_specific_galileo_tgd ||
            options.native_source_clock_c0d_phase94_stage_diagnostics ||
            options.native_source_clock_c0d_phase96_main_diagnostics ||
            options.native_source_clock_c0d_phase97_singular_system_diagnostics ||
            options.native_source_clock_c0d_phase98_solver_rank_diagnostic ||
            options.native_phase104_stage_main_accuracy_attribution ||
            options.native_phase116_carrier_tdcp_incidence_diagnostic) {
            std::cerr << "--native-phase126-raw-base-source-complete forbids "
                         "partial/alternate candidate selectors\n";
            return false;
        }
        if (!options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-phase126-raw-base-source-complete requires "
                         "the hash-bound Phase118 raw-base source/miss contract "
                         "without the excluded additional-band selector\n";
            return false;
        }
        const bool supported_route =
            options.dataset_id == "2021-03-16-18-59-us-ca-mtv-a/pixel5" ||
            options.dataset_id == "2022-04-01-18-22-us-ca-lax-t/pixel5";
        if (!supported_route) {
            std::cerr << "--native-phase126-raw-base-source-complete is restricted "
                         "to the frozen MTV-A and LAX-T routes\n";
            return false;
        }
        if (!options.native_upstream_position_offset ||
            phoneFromDatasetId(options.dataset_id) != "pixel5" ||
            !phase112MainOutputPositionOffsetSelectors(options)) {
            std::cerr << "--native-phase126-raw-base-source-complete requires "
                         "the exact Pixel5 Phase112 final-output boundary\n";
            return false;
        }
    }
    if (options.native_phase127_glonass_channel_provenance) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase127-glonass-channel-provenance requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_phase126_raw_base_source_complete) {
            std::cerr << "--native-phase127-glonass-channel-provenance requires "
                         "--native-phase126-raw-base-source-complete\n";
            return false;
        }
        // Phase126 validation above owns the Android/raw, route, base-RINEX,
        // all-epoch, C7/D/C0D/QR, and Pixel5 boundary.  Repeat only the
        // provenance-specific composition here so a future validation edit
        // cannot accidentally make Phase127 independently runnable.
        if (!options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-phase127-glonass-channel-provenance requires "
                         "the exact Phase126 raw-base source/miss contract\n";
            return false;
        }
    }
    if (options.native_phase128_glonass_provenance_parser_admission) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase128-glonass-provenance-parser-admission "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_phase127_glonass_channel_provenance) {
            std::cerr << "--native-phase128-glonass-provenance-parser-admission "
                         "requires --native-phase127-glonass-channel-provenance\n";
            return false;
        }
        // Phase127 validation owns the route, raw/base, and frozen recipe
        // boundary.  Phase128 only adds parser/admission semantics and may
        // never become an independently runnable or partial selector.
        if (!options.native_phase126_raw_base_source_complete ||
            !options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-phase128-glonass-provenance-parser-admission "
                         "requires the sealed Phase126/127 raw-base contract\n";
            return false;
        }
    }
    if (options.native_phase129_glonass_local_miss_mask) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase129-glonass-local-miss-mask requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_phase126_raw_base_source_complete ||
            !options.native_phase127_glonass_channel_provenance ||
            !options.native_phase128_glonass_provenance_parser_admission) {
            std::cerr << "--native-phase129-glonass-local-miss-mask requires "
                         "the composed Phase126/127/128 selectors\n";
            return false;
        }
        // Phase126 owns the raw Android, route, base-RINEX, no-additional-
        // band, C7/D/C0D/QR, Pixel5, and source-complete contract.  Phase129
        // only changes local admission after those checks have succeeded.
        if (!options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-phase129-glonass-local-miss-mask requires "
                         "the sealed Phase126 raw-base source/miss contract\n";
            return false;
        }
    }
    if (options.native_phase131_canonical_correction_band_key) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase131-canonical-correction-band-key requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_phase126_raw_base_source_complete ||
            !options.native_phase127_glonass_channel_provenance ||
            !options.native_phase128_glonass_provenance_parser_admission ||
            !options.native_phase129_glonass_local_miss_mask) {
            std::cerr << "--native-phase131-canonical-correction-band-key requires "
                         "the composed Phase126/127/128/129 selectors\n";
            return false;
        }
        if (!options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-phase131-canonical-correction-band-key requires "
                         "the exact Phase126 raw-base source/miss contract\n";
            return false;
        }
        if (!options.native_phase118_official_tdcp_huber_k ||
            options.native_phase117_tdcp_snr_type_sigma ||
            options.native_phase120_official_tdcp_resl_atmosphere_cancellation ||
            options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_direct_wls_ephemeral_c7d_main_seed ||
            options.native_phase116_carrier_tdcp_incidence_diagnostic ||
            options.native_source_clock_c0d_phase94_stage_diagnostics ||
            options.native_source_clock_c0d_phase96_main_diagnostics ||
            options.native_source_clock_c0d_phase97_singular_system_diagnostics ||
            options.native_source_clock_c0d_phase98_solver_rank_diagnostic ||
            options.native_phase104_stage_main_accuracy_attribution) {
            std::cerr << "--native-phase131-canonical-correction-band-key forbids "
                         "alternate/diagnostic candidate selectors\n";
            return false;
        }
        const bool supported_route =
            options.dataset_id == "2021-03-16-18-59-us-ca-mtv-a/pixel5" ||
            options.dataset_id == "2022-04-01-18-22-us-ca-lax-t/pixel5";
        if (!supported_route) {
            std::cerr << "--native-phase131-canonical-correction-band-key is restricted "
                         "to the frozen MTV-A and LAX-T routes\n";
            return false;
        }
    }
    if (options.native_phase135_official_affine_measurement_family) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase135-official-affine-measurement-family "
                     "requires a GTSAM build\n";
        return false;
#endif
        // Phase135 is a single transactional composition on the complete
        // Phase118/126-131 recipe.  Requiring the GNSS-first meter handoff
        // here is important: the same affine family must be built in the
        // Point3+velocity stage and in the main Pose3+IMU graph; otherwise
        // the latter could silently consume a legacy/non-affine seed.
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.android_raw_clock_only ||
            !options.android_utc_wall_clock_fallback || !options.all_epochs ||
            options.skip_epochs != 0) {
            std::cerr << "--native-phase135-official-affine-measurement-family "
                         "requires the exact Phase118 raw UTC/all-epoch recipe\n";
            return false;
        }
        const bool phase126_131_source_complete =
            options.native_phase126_raw_base_source_complete &&
            options.native_phase127_glonass_channel_provenance &&
            options.native_phase128_glonass_provenance_parser_admission &&
            options.native_phase129_glonass_local_miss_mask &&
            options.native_phase131_canonical_correction_band_key;
        const bool phase107_raw_base_compatibility =
            !options.native_phase126_raw_base_source_complete &&
            !options.native_phase127_glonass_channel_provenance &&
            !options.native_phase128_glonass_provenance_parser_admission &&
            !options.native_phase129_glonass_local_miss_mask &&
            !options.native_phase131_canonical_correction_band_key &&
            options.native_base_pseudorange_compensation &&
            options.native_base_pseudorange_source_miss_mask &&
            !options.native_base_pseudorange_preserve_additional_frequency_bands &&
            !options.native_base_rinex_path.empty() &&
            !options.native_base_rinex_sha256.empty();
        if (!options.native_phase118_official_tdcp_huber_k ||
            (!phase126_131_source_complete && !phase107_raw_base_compatibility) ||
            !options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_epoch_vector_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver ||
            !options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge) {
            std::cerr << "--native-phase135-official-affine-measurement-family "
                         "requires the complete Phase118/126-131 C7/D/QR "
                         "and GNSS-first handoff recipe\n";
            return false;
        }
        if (options.native_phase117_tdcp_snr_type_sigma ||
            options.native_phase120_official_tdcp_resl_atmosphere_cancellation ||
            options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_direct_wls_ephemeral_c7d_main_seed ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_pdc_state_bridge || options.native_upstream_quality ||
            options.native_signal_bias_states || options.native_residual_ionosphere ||
            options.native_phase116_carrier_tdcp_incidence_diagnostic ||
            options.native_source_clock_c0d_phase94_stage_diagnostics ||
            options.native_source_clock_c0d_phase96_main_diagnostics ||
            options.native_source_clock_c0d_phase97_singular_system_diagnostics ||
            options.native_source_clock_c0d_phase98_solver_rank_diagnostic ||
            options.native_phase104_stage_main_accuracy_attribution ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            options.native_upstream_stop_constraints ||
            options.native_signal_specific_galileo_tgd ||
            options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-phase135-official-affine-measurement-family "
                         "forbids partial, alternate, diagnostic, or tuning "
                         "selectors\n";
            return false;
        }
        if (!options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty()) {
            std::cerr << "--native-phase135-official-affine-measurement-family "
                         "requires the exact Phase107 raw-base or Phase126 "
                         "raw-base correction ledger\n";
            return false;
        }
        const bool supported_route =
            options.dataset_id == "2021-03-16-18-59-us-ca-mtv-a/pixel5" ||
            options.dataset_id == "2022-04-01-18-22-us-ca-lax-t/pixel5" ||
            options.dataset_id == "2021-08-24-20-32-us-ca-mtv-h/pixel5" ||
            options.dataset_id == "2023-03-08-21-34-us-ca-mtv-u/pixel5";
        if (!supported_route) {
            std::cerr << "--native-phase135-official-affine-measurement-family "
                         "is restricted to the frozen MTV-A, MTV-H, LAX-T, and "
                         "MTV-U Pixel5 routes\n";
            return false;
        }
        if (!options.native_upstream_position_offset ||
            phoneFromDatasetId(options.dataset_id) != "pixel5" ||
            !phase112MainOutputPositionOffsetSelectors(options)) {
            std::cerr << "--native-phase135-official-affine-measurement-family "
                         "requires the exact Phase112 Pixel5 final-output "
                         "offset boundary\n";
            return false;
        }
    }
    if (options.native_phase138_affine_tdcp_anchor_range_constant) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase138-affine-tdcp-anchor-range-constant "
                     "requires a GTSAM build\n";
        return false;
#endif
        if (!options.native_phase135_official_affine_measurement_family) {
            std::cerr << "--native-phase138-affine-tdcp-anchor-range-constant "
                         "requires --native-phase135-official-affine-"
                         "measurement-family\n";
            return false;
        }
        if (options.native_phase117_tdcp_snr_type_sigma ||
            options.native_phase120_official_tdcp_resl_atmosphere_cancellation) {
            std::cerr << "--native-phase138-affine-tdcp-anchor-range-constant "
                         "requires the frozen Phase118 TDCP measurement\n";
            return false;
        }
    }
    if (options.native_phase141_telemetry_schema) {
        // Phase141 is a summary-boundary diagnostic only.  Requiring the
        // complete source-affine recipe keeps the schema from being mistaken
        // for a partial legacy report while leaving all optimizer settings
        // untouched.
        if (!options.native_phase135_official_affine_measurement_family ||
            !options.native_phase138_affine_tdcp_anchor_range_constant) {
            std::cerr << "--native-phase141-telemetry-schema requires the "
                         "complete Phase135+Phase138 affine recipe\n";
            return false;
        }
    }
    if (options.native_phase143_official_main_lm_termination_budget) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase143-official-main-lm-termination-budget "
                     "requires a GTSAM build\n";
        return false;
#endif
        // Phase143 is an optimizer-boundary candidate, not an alternate
        // recipe.  Keep the complete Phase142 affine/clock/QR/raw input
        // contract as the admission boundary so a bare legacy Pose3 solve
        // cannot accidentally acquire the official iteration budget.
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.android_raw_clock_only ||
            !options.android_utc_wall_clock_fallback || !options.all_epochs ||
            options.skip_epochs != 0 ||
            !options.native_phase135_official_affine_measurement_family ||
            !options.native_phase138_affine_tdcp_anchor_range_constant ||
            !options.native_phase118_official_tdcp_huber_k ||
            !options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_epoch_vector_parity ||
            !options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            !options.native_source_clock_c0d_active_solve_diagnostic ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver ||
            !options.native_upstream_position_offset ||
            phoneFromDatasetId(options.dataset_id) != "pixel5" ||
            !phase112MainOutputPositionOffsetSelectors(options)) {
            std::cerr << "--native-phase143-official-main-lm-termination-budget "
                         "requires the complete Phase142 affine/C7/D/QR "
                         "Pixel5 main recipe\n";
            return false;
        }
        const bool supported_route =
            options.dataset_id == "2021-03-16-18-59-us-ca-mtv-a/pixel5" ||
            options.dataset_id == "2022-04-01-18-22-us-ca-lax-t/pixel5" ||
            options.dataset_id == "2021-08-24-20-32-us-ca-mtv-h/pixel5" ||
            options.dataset_id == "2023-03-08-21-34-us-ca-mtv-u/pixel5";
        if (!supported_route) {
            std::cerr << "--native-phase143-official-main-lm-termination-budget "
                         "is restricted to the frozen MTV-A, MTV-H, LAX-T, and "
                         "MTV-U Pixel5 routes\n";
            return false;
        }
        if (options.native_phase117_tdcp_snr_type_sigma ||
            options.native_phase120_official_tdcp_resl_atmosphere_cancellation ||
            options.native_direct_wls_ephemeral_c7d_main_seed ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff ||
            options.native_phase104_stage_main_accuracy_attribution ||
            options.native_phase116_carrier_tdcp_incidence_diagnostic ||
            options.native_source_clock_c0d_phase94_stage_diagnostics ||
            options.native_source_clock_c0d_phase96_main_diagnostics ||
            options.native_source_clock_c0d_phase97_singular_system_diagnostics ||
            options.native_source_clock_c0d_phase98_solver_rank_diagnostic ||
            options.native_upstream_quality || options.native_signal_bias_states ||
            options.native_residual_ionosphere || options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-phase143-official-main-lm-termination-budget "
                         "forbids alternate or diagnostic optimizer selectors\n";
            return false;
        }
        if (!options.native_base_pseudorange_compensation ||
            !options.native_base_pseudorange_source_miss_mask ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_rinex_path.empty() ||
            options.native_base_rinex_sha256.empty() ||
            options.native_phase126_raw_base_source_complete ||
            options.native_phase127_glonass_channel_provenance ||
            options.native_phase128_glonass_provenance_parser_admission ||
            options.native_phase129_glonass_local_miss_mask ||
            options.native_phase131_canonical_correction_band_key) {
            std::cerr << "--native-phase143-official-main-lm-termination-budget "
                         "requires the frozen Phase107 raw-base recipe\n";
            return false;
        }
    }
    if (options.native_direct_wls_ephemeral_c7d_main_seed) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-direct-wls-ephemeral-c7d-main-seed requires a GTSAM build\n";
        return false;
#endif
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.all_epochs || options.skip_epochs != 0) {
            std::cerr << "--native-direct-wls-ephemeral-c7d-main-seed requires Android raw "
                         "input, --android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
            return false;
        }
        if (!options.native_source_direct_observable_quality ||
            !options.native_pdc_imu_tdcp_no_bridge ||
            !options.native_source_clock_c0d_factor ||
            !options.native_source_clock_c0d_meter_state_parity ||
            !options.native_source_clock_c0d_epoch_vector_parity ||
            !options.native_source_clock_c0d_active_solve_diagnostic ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
            std::cerr << "--native-direct-wls-ephemeral-c7d-main-seed requires the "
                         "direct-quality, no-bridge, C0/D, meter-state, active-diagnostic, "
                         "C7, and Phase99 QR recipe\n";
            return false;
        }
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            options.native_gnss_first_velocity_only_handoff ||
            options.native_direct_doppler_wls_handoff || options.native_pdc_state_bridge ||
            options.native_upstream_quality) {
            std::cerr << "--native-direct-wls-ephemeral-c7d-main-seed forbids GNSS-first, "
                         "raw-D-only, velocity-only, legacy direct-WLS, PDC, and legacy "
                         "quality handoffs\n";
            return false;
        }
        if (options.native_source_clock_c0d_phase94_stage_diagnostics ||
            options.native_source_clock_c0d_phase96_main_diagnostics ||
            options.native_source_clock_c0d_phase97_singular_system_diagnostics ||
            options.native_source_clock_c0d_phase98_solver_rank_diagnostic ||
            options.native_phase104_stage_main_accuracy_attribution) {
            std::cerr << "--native-direct-wls-ephemeral-c7d-main-seed does not compose "
                         "with stage/diagnostic/attribution selectors\n";
            return false;
        }
        const bool has_any_base_input =
            options.native_base_pseudorange_compensation ||
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_base_pseudorange_source_miss_mask ||
            !options.native_base_rinex_path.empty() ||
            !options.native_base_rinex_sha256.empty();
        const bool direct_phase107_base_admission =
            options.native_base_pseudorange_compensation &&
            options.native_base_pseudorange_source_miss_mask &&
            !options.native_base_pseudorange_preserve_additional_frequency_bands &&
            !options.native_base_rinex_path.empty() &&
            !options.native_base_rinex_sha256.empty();
        const bool direct_phase109_base_admission =
            options.native_base_pseudorange_compensation &&
            options.native_base_pseudorange_source_miss_mask &&
            options.native_base_pseudorange_preserve_additional_frequency_bands &&
            !options.native_base_rinex_path.empty() &&
            !options.native_base_rinex_sha256.empty();
        if (has_any_base_input && !direct_phase107_base_admission &&
            !direct_phase109_base_admission) {
            std::cerr << "--native-direct-wls-ephemeral-c7d-main-seed permits only "
                         "the existing Phase107/109 raw-base admission\n";
            return false;
        }
        if (options.native_upstream_position_offset &&
            (!phase112MainOutputPositionOffsetSelectors(options) ||
             phoneFromDatasetId(options.dataset_id) != "pixel5")) {
            std::cerr << "--native-upstream-position-offset with the Phase114 direct-WLS "
                         "seed requires the exact Phase112 C7/D/QR main recipe and "
                         "dataset phone pixel5\n";
            return false;
        }
        if (options.fgo_imu_sparse_recovery || options.native_signal_bias_states ||
            options.native_residual_ionosphere ||
            options.native_upstream_absolute_doppler_screen ||
            options.native_carrier_code_leveling ||
            options.native_carrier_code_innovation_reset ||
            options.native_carrier_code_primary_l1_e1 ||
            options.native_carrier_code_gal_e1_e5a ||
            options.native_upstream_stop_constraints ||
            options.native_signal_specific_galileo_tgd || options.native_quality_anchor ||
            options.native_fallback_seed_quality_anchor_recovery ||
            options.native_android_sv_time_uncertainty_sigma_floor ||
            options.native_cn0_doppler_calibration) {
            std::cerr << "--native-direct-wls-ephemeral-c7d-main-seed requires the "
                         "frozen raw-only recipe without other candidate switches\n";
            return false;
        }
    }
    const auto hasForbiddenPhase104PathTerm = [](const std::string& path) {
        std::string lower = path;
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char ch) {
                           return static_cast<char>(std::tolower(ch));
                       });
        static constexpr const char* kForbidden[] = {
            ".mat", "truth", "ground_truth", "base", "pdc",
            "precomputed", "coordinate", "kaggle", "token"};
        for (const char* term : kForbidden) {
            if (lower.find(term) != std::string::npos) return true;
        }
        return false;
    };
    if (options.native_phase104_stage_main_accuracy_attribution) {
#ifndef GNSSPP_HAS_GTSAM
        std::cerr << "--native-phase104-stage-main-attribution requires a GTSAM build\n";
        return false;
#endif
        if (!android_raw || !options.android_raw_utc_key_contract ||
            !options.all_epochs || options.skip_epochs != 0) {
            std::cerr << "--native-phase104-stage-main-attribution requires Android raw "
                         "input, --android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
            return false;
        }
        if (!options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
            !options.native_source_clock_c0d_epoch_vector_parity ||
            !options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
            std::cerr << "--native-phase104-stage-main-attribution requires the frozen "
                         "Phase101 C7/D handoff and Phase99 QR selectors\n";
            return false;
        }
        if (options.phase104_stage_ecef_path.empty() ||
            options.phase104_main_displacement_stats_path.empty()) {
            std::cerr << "--native-phase104-stage-main-attribution requires both "
                         "--phase104-stage-ecef and --phase104-main-displacement-stats\n";
            return false;
        }
        if (hasMatExtension(options.phase104_stage_ecef_path) ||
            hasMatExtension(options.phase104_main_displacement_stats_path) ||
            hasForbiddenPhase104PathTerm(options.phase104_stage_ecef_path) ||
            hasForbiddenPhase104PathTerm(options.phase104_main_displacement_stats_path)) {
            std::cerr << "Phase104 evaluation-only output paths contain a forbidden "
                         "MAT/truth/base/PDC/coordinate token\n";
            return false;
        }
        if (options.phase104_stage_ecef_path == options.out_path ||
            options.phase104_stage_ecef_path == options.summary_path ||
            options.phase104_main_displacement_stats_path == options.out_path ||
            options.phase104_main_displacement_stats_path == options.summary_path ||
            options.phase104_stage_ecef_path ==
                options.phase104_main_displacement_stats_path) {
            std::cerr << "Phase104 evaluation-only output paths must be distinct from "
                         "native solution/summary paths and each other\n";
            return false;
        }
    } else if (!options.phase104_stage_ecef_path.empty() ||
               !options.phase104_main_displacement_stats_path.empty()) {
        std::cerr << "Phase104 output paths require --native-phase104-stage-main-attribution\n";
        return false;
    }
    if (options.native_fallback_seed_quality_anchor_recovery &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-fallback-seed-quality-anchor-recovery requires "
                     "Android raw input, --android-raw-utc-keys, --all-epochs, "
                     "and no skipped epochs\n";
        return false;
    }
    if (options.native_fallback_seed_quality_anchor_recovery &&
        options.native_pdc_state_bridge) {
        std::cerr << "--native-fallback-seed-quality-anchor-recovery is "
                     "incompatible with --native-pdc-state-bridge; the raw/nav "
                     "anchor must remain the initial position/clock seed\n";
        return false;
    }
    if (options.native_signal_bias_states && !android_raw) {
        std::cerr << "--native-signal-bias-states requires Android raw GNSS/IMU input\n";
        return false;
    }
    if (options.native_residual_ionosphere &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-residual-ionosphere requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_residual_ionosphere &&
        (!options.native_signal_bias_states || !options.native_pdc_imu_tdcp)) {
        std::cerr << "--native-residual-ionosphere requires the frozen "
                     "--native-signal-bias-states and --native-pdc-imu-tdcp base recipe\n";
        return false;
    }
    if (options.native_upstream_quality &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0 ||
         !options.native_residual_ionosphere ||
         !options.native_signal_bias_states || !options.native_pdc_imu_tdcp)) {
        std::cerr << "--native-upstream-quality requires the frozen Android raw "
                     "Phase12 base recipe and all epochs\n";
        return false;
    }
    if (options.native_upstream_absolute_doppler_screen &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0 ||
         !options.native_residual_ionosphere ||
         !options.native_signal_bias_states || !options.native_pdc_imu_tdcp ||
         options.native_upstream_quality)) {
        std::cerr << "--native-upstream-absolute-doppler-screen requires the "
                     "frozen Android raw Phase12 base recipe, all epochs, and "
                     "Phase13 upstream quality off\n";
        return false;
    }
    if (options.native_carrier_code_leveling &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0 ||
         !options.native_residual_ionosphere ||
         !options.native_signal_bias_states || !options.native_pdc_imu_tdcp ||
         options.native_upstream_quality)) {
        std::cerr << "--native-carrier-code-leveling requires the frozen Android "
                     "Phase12 base recipe, all epochs, and Phase13 off\n";
        return false;
    }
    if (options.native_carrier_code_innovation_reset &&
        !options.native_carrier_code_leveling) {
        std::cerr << "--native-carrier-code-innovation-reset requires "
                     "--native-carrier-code-leveling\n";
        return false;
    }
    if (options.native_carrier_code_primary_l1_e1 &&
        (!options.native_carrier_code_leveling ||
         !options.native_carrier_code_innovation_reset)) {
        std::cerr << "--native-carrier-code-primary-l1-e1 requires "
                     "--native-carrier-code-leveling and "
                     "--native-carrier-code-innovation-reset\n";
        return false;
    }
    if (options.native_carrier_code_gal_e1_e5a &&
        (!options.native_carrier_code_leveling ||
         !options.native_carrier_code_innovation_reset)) {
        std::cerr << "--native-carrier-code-gal-e1-e5a requires "
                     "--native-carrier-code-leveling and "
                     "--native-carrier-code-innovation-reset\n";
        return false;
    }
    if (options.native_upstream_stop_constraints &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-upstream-stop-constraints requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_upstream_position_offset &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-upstream-position-offset requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_signal_specific_galileo_tgd && !android_raw) {
        std::cerr << "--native-signal-specific-galileo-tgd requires Android raw "
                     "GNSS/IMU input\n";
        return false;
    }
    if (options.native_carrier_code_primary_l1_e1 &&
        options.native_carrier_code_gal_e1_e5a) {
        std::cerr << "--native-carrier-code-primary-l1-e1 and "
                     "--native-carrier-code-gal-e1-e5a are mutually exclusive; "
                     "Phase 19 excludes GPS L1\n";
        return false;
    }
    if (options.native_gnss_first_velocity_only_handoff &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-gnss-first-velocity-only-handoff requires Android "
                     "raw input, --android-raw-utc-keys, --all-epochs, and no "
                     "skipped epochs\n";
        return false;
    }
    if (options.native_gnss_first_velocity_only_handoff &&
        options.native_pdc_state_bridge) {
        std::cerr << "--native-gnss-first-velocity-only-handoff is incompatible "
                     "with --native-pdc-state-bridge; position/clock seeds must "
                     "remain the original raw SPP seeds\n";
        return false;
    }
    if (options.native_direct_doppler_wls_handoff &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-direct-doppler-wls-handoff requires Android "
                     "raw input, --android-raw-utc-keys, --all-epochs, and no "
                     "skipped epochs\n";
        return false;
    }
    if (options.native_direct_doppler_wls_handoff &&
        options.native_pdc_state_bridge) {
        std::cerr << "--native-direct-doppler-wls-handoff is incompatible "
                     "with --native-pdc-state-bridge; position/clock seeds "
                     "must remain the original raw SPP seeds\n";
        return false;
    }
    if (options.native_direct_doppler_wls_handoff &&
        options.native_gnss_first_velocity_only_handoff) {
        std::cerr << "--native-direct-doppler-wls-handoff and "
                     "--native-gnss-first-velocity-only-handoff are mutually "
                     "exclusive\n";
        return false;
    }
    return true;
}

bool atomicWrite(const std::string& path, const std::string& contents) {
    const std::filesystem::path destination(path);
    if (!destination.parent_path().empty()) {
        std::error_code error;
        std::filesystem::create_directories(destination.parent_path(), error);
        if (error) return false;
    }
    const std::string temporary = path + ".tmp." + std::to_string(static_cast<long long>(::getpid()));
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) return false;
        output << contents;
        output.flush();
        if (!output.good()) return false;
    }
    if (std::rename(temporary.c_str(), path.c_str()) != 0) {
        std::remove(temporary.c_str());
        return false;
    }
    return true;
}

double unixMillis(const libgnss::GNSSTime& time) {
    const double seconds = kGpsEpochUnixSeconds +
                           static_cast<double>(time.week) * 604800.0 + time.tow -
                           kGpsUtcLeapSeconds;
    return std::round(seconds * 1000.0);
}

Eigen::Matrix3d tarozMountingRotation() {
    // Exact frozen translation of gtsam.Rot3.RzRyRx(deg2rad([-85 178 -94]')):
    // Rz(-94 deg) * Ry(178 deg) * Rx(-85 deg).  Applying this matrix to the
    // Android sensor vector is equivalent to using the upstream body_P_sensor
    // rotation while keeping the native backend's body-FLU input contract.
    const double rx = -85.0 / kRadToDeg;
    const double ry = 178.0 / kRadToDeg;
    const double rz = -94.0 / kRadToDeg;
    return (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

Eigen::Matrix3d rpyToBodyToNav(const Eigen::Vector3d& rpy_rad) {
    return (Eigen::AngleAxisd(rpy_rad.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy_rad.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy_rad.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

bool deriveGnssFirstVelocities(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& gnss_first_result,
    std::vector<libgnss::Vector3d>& velocities_enu,
    std::string& error) {
    const auto& solutions = gnss_first_result.solution.solutions;
    const auto& optimized_velocities = gnss_first_result.epoch_velocities_ecef_mps;
    if (problem.epochs.size() < 2 || solutions.size() != problem.epochs.size() ||
        optimized_velocities.size() != problem.epochs.size()) {
        error = "GNSS-first optimized velocity state count does not match observation epochs";
        return false;
    }
    const libgnss::Vector3d origin = solutions.front().position_ecef;
    if (!origin.allFinite() || origin.norm() < 1.0e6) {
        error = "GNSS-first solution has invalid ENU origin";
        return false;
    }
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    libgnss::ecef2geodetic(origin, lat, lon, height);
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(height)) {
        error = "GNSS-first solution has invalid geodetic origin";
        return false;
    }
    velocities_enu.resize(solutions.size());
    for (std::size_t i = 0; i < solutions.size(); ++i) {
        if (!solutions[i].position_ecef.allFinite()) {
            error = "GNSS-first solution contains non-finite position";
            return false;
        }
        // The upstream GNSS pass optimizes a velocity state from P+D.  Use
        // that state directly; a position-difference proxy would hide a
        // missing Doppler graph and is therefore not accepted by this mode.
        // Convert the ECEF velocity with the same linear ENU basis as the
        // position frame.
        const libgnss::Vector3d velocity_enu = libgnss::ecef2enu(
            optimized_velocities[i], lat, lon);
        if (!optimized_velocities[i].allFinite() || !velocity_enu.allFinite()) {
            error = "GNSS-first optimized velocity state is non-finite";
            return false;
        }
        velocities_enu[i] = velocity_enu;
    }
    return true;
}

struct ImuBuildReport {
    bool stationary_gyro_initializer_applied = false;
    std::size_t stationary_gyro_blocks = 0;
    double stationary_gyro_scatter_radps = 0.0;
    bool ok = false;
    bool heading_latched = false;
    std::string heading_initialization_mode;
    std::size_t velocity_heading_low_speed_count = 0;
    std::size_t velocity_heading_linear_fill_count = 0;
    std::size_t velocity_heading_nearest_fill_count = 0;
    bool gnss_first_attempted = false;
    bool gnss_first_converged = false;
    bool pseudorange_remasking_applied = false;
    std::size_t pseudorange_remasking_pool = 0, pseudorange_remasking_old = 0;
    std::size_t pseudorange_remasking_new = 0, pseudorange_remasking_recovered = 0;
    std::size_t pseudorange_remasking_removed = 0, pseudorange_remasking_unchanged = 0;
    std::size_t gnss_first_epochs = 0;
    std::size_t gnss_first_doppler_factors = 0;
    bool gnss_first_ecef_doppler_stage_enabled = false;
    std::size_t gnss_first_doppler_factors_inserted = 0;
    std::size_t gnss_first_tdcp_only_affine_factors_inserted = 0;
    std::size_t gnss_first_velocity_states = 0;
    std::size_t gnss_first_position_invalid_count = 0;
    std::size_t gnss_first_position_nonfinite_count = 0;
    std::size_t gnss_first_position_out_of_earth_count = 0;
    std::size_t gnss_first_clock_invalid_count = 0;
    std::size_t gnss_first_velocity_nonfinite_count = 0;
    std::size_t gnss_first_velocity_over_bound_count = 0;
    std::size_t gnss_first_velocity_valid_count = 0;
    std::size_t gnss_first_velocity_initializer_direct_count = 0;
    std::size_t gnss_first_velocity_initializer_propagated_count = 0;
    std::size_t gnss_first_velocity_initializer_edge_hold_count = 0;
    double gnss_first_velocity_initializer_edge_hold_max_s = 0.0;
    double gnss_first_max_velocity_norm_mps = 0.0;
    double gnss_first_first_invalid_position_norm_m =
        std::numeric_limits<double>::quiet_NaN();
    std::size_t original_raw_seed_position_count = 0;
    std::size_t original_raw_seed_position_invalid_count = 0;
    std::size_t gnss_first_positions_clocks_copied = 0;
    bool gnss_first_velocity_only_handoff = false;
    bool direct_doppler_wls_handoff = false;
    // Phase91 exact same-run GNSS-first/raw identity audit. The backend's
    // finite D_i coverage is reported in FGOResultDiagnostics; these fields
    // cover the cross-stream handoff before the main graph is built.
    bool phase91_raw_drift_d_initializer_enabled = false;
    bool phase91_raw_drift_d_initializer_alignment_valid = false;
    std::size_t phase91_main_epoch_count = 0;
    std::size_t phase91_gnss_first_epoch_count = 0;
    std::size_t phase91_solution_epoch_count = 0;
    std::size_t phase91_raw_epoch_count = 0;
    std::size_t phase91_raw_utc_key_count = 0;
    std::size_t phase91_aligned_epoch_count = 0;
    std::size_t phase91_epoch_count_mismatch_count = 0;
    std::size_t phase91_nonfinite_time_count = 0;
    std::size_t phase91_gnss_first_time_mismatch_count = 0;
    std::size_t phase91_raw_time_mismatch_count = 0;
    std::size_t phase91_raw_utc_key_order_mismatch_count = 0;
    std::size_t phase91_duplicate_raw_utc_key_count = 0;
    std::size_t phase91_raw_utc_key_mismatch_count = 0;
    std::size_t phase91_retained_source_index_mismatch_count = 0;
    std::size_t phase91_retained_source_order_mismatch_count = 0;
    std::size_t phase91_duplicate_retained_source_index_count = 0;
    std::size_t phase91_retained_raw_key_mismatch_count = 0;
    std::size_t phase91_raw_drift_count_mismatch_count = 0;
    std::size_t phase91_raw_drift_nonfinite_count = 0;
    std::size_t phase91_nonfinite_solution_count = 0;
    std::string phase91_raw_drift_d_initializer_failure;
    // Phase93 optimized GNSS-first D export/handoff gate.  The Phase91 key
    // audit above proves that vector index i is the same retained source epoch
    // across main, GNSS-first, solution, and raw tables; these fields verify
    // the new optimized D vector itself before it reaches the main graph.
    bool phase93_meter_state_handoff_enabled = false;
    bool phase93_optimized_d_export_valid = false;
    std::size_t phase93_optimized_d_epoch_count = 0;
    std::size_t phase93_optimized_d_finite_count = 0;
    std::size_t phase93_optimized_d_nonfinite_count = 0;
    std::string phase93_optimized_d_handoff_failure;
    // Phase101 complete source-parity C_i export/handoff witness.
    bool phase101_epoch_vector_handoff_enabled = false;
    bool phase101_optimized_c_export_valid = false;
    std::size_t phase101_optimized_c_epoch_count = 0;
    std::size_t phase101_optimized_c_finite_count = 0;
    std::size_t phase101_optimized_c_nonfinite_count = 0;
    std::string phase101_optimized_c_handoff_failure;
    std::size_t direct_doppler_wls_epochs = 0;
    std::size_t direct_doppler_wls_direct_valid_count = 0;
    std::size_t direct_doppler_wls_propagated_valid_count = 0;
    std::size_t direct_doppler_wls_rejected_count = 0;
    std::size_t direct_doppler_wls_nonfinite_count = 0;
    std::size_t direct_doppler_wls_over_bound_count = 0;
    std::size_t direct_doppler_wls_clock_rate_over_bound_count = 0;
    std::size_t direct_doppler_wls_edge_hold_count = 0;
    double direct_doppler_wls_edge_hold_max_s = 1.0;
    double direct_doppler_wls_max_velocity_norm_mps = 0.0;
    double direct_doppler_wls_max_clock_rate_abs_mps = 0.0;
    std::size_t direct_doppler_wls_first_solved_rows = 0;
    double direct_doppler_wls_first_solved_velocity_norm_mps = 0.0;
    double direct_doppler_wls_first_solved_clock_rate_abs_mps = 0.0;
    std::string direct_doppler_wls_first_solved_reason;
    std::size_t direct_doppler_wls_original_raw_seed_position_count = 0;
    std::size_t direct_doppler_wls_original_raw_seed_position_invalid_count = 0;
    std::size_t direct_doppler_wls_positions_clocks_copied = 0;
    // Phase114 direct-WLS -> main C7/D ephemeral-seed admission witness.
    bool direct_wls_ephemeral_c7d_main_seed_enabled = false;
    bool direct_wls_ephemeral_c7d_main_seed_valid = false;
    std::size_t direct_wls_ephemeral_raw_epoch_count = 0;
    std::size_t direct_wls_ephemeral_retained_epoch_count = 0;
    std::size_t direct_wls_ephemeral_exact_key_count = 0;
    std::size_t direct_wls_ephemeral_finite_position_count = 0;
    std::size_t direct_wls_ephemeral_finite_clock_count = 0;
    std::size_t direct_wls_ephemeral_finite_drift_count = 0;
    std::size_t direct_wls_ephemeral_finite_velocity_count = 0;
    std::size_t direct_wls_ephemeral_raw_key_mismatch_count = 0;
    std::size_t direct_wls_ephemeral_raw_key_order_mismatch_count = 0;
    std::size_t direct_wls_ephemeral_raw_drift_mismatch_count = 0;
    bool direct_wls_ephemeral_full_raw_coverage = false;
    std::string direct_wls_ephemeral_failure;
    int gnss_first_iterations = 0;
    double gnss_first_initial_cost = 0.0;
    double gnss_first_final_cost = 0.0;
    std::size_t gnss_first_c0d_factor_count = 0;
    std::size_t gnss_first_c0d_accepted_outer_iterations = 0;
    bool gnss_first_c0d_active_solve_finite_costs = false;
    // Phase143 GNSS-first stage report is copied directly from the staged
    // FGOResult.  It remains separate from the main report because the
    // official-budget selector changes only the later main LM call.
    libgnss::FGOProcessor::FGOPhase143TerminationDiagnostics
        gnss_first_phase143_termination;
    std::string gnss_first_failure;
    std::string failure;
    std::size_t loaded_samples = 0;
    std::size_t stationary_samples = 0;
    double gravity_mean_norm = 0.0;
    double gravity_norm_std = 0.0;
    bool phase194_source_utc_fallback_imu_noise_requested = false;
    bool phase194_source_utc_fallback_imu_noise_applied = false;
    double imu_accel_noise_sigma =
        libgnss::native_utc_fallback_imu_noise::kDefaultAccelNoiseSigma;
    double imu_gyro_noise_sigma =
        libgnss::native_utc_fallback_imu_noise::kDefaultGyroNoiseSigma;
    double imu_measurement_noise_sync_coefficient = 0.5;
    std::string imu_measurement_noise_source =
        "native-pixel5-coefficient-0.5";
    bool phase197_source_utc_fallback_imu_offset_requested = false;
    bool phase197_source_utc_fallback_imu_offset_applied = false;
    std::int64_t phase197_source_utc_fallback_imu_offset_configured_ms = 0;
    std::int64_t phase197_source_utc_fallback_imu_offset_effective_ms = 0;
    std::string imu_utc_fallback_offset_source =
        "native-no-utc-fallback-offset";
    int heading_windows = 0;
    bool android_raw = false;
    libgnss::io::AndroidRawGnssDiagnostics android_gnss_diagnostics;
    libgnss::AndroidImuCsvLoadResult android_load;
    libgnss::AndroidGnssTimeAnchorLoadResult android_gnss_anchor_load;
    libgnss::AndroidGnssUtcGpsMappingLoadResult android_gnss_utc_mapping_load;
};

bool validatePhase91GnssFirstHandoff(
    const libgnss::FGOProcessor::FGOProblem& main_problem,
    const libgnss::FGOProcessor::FGOProblem& gnss_first_problem,
    const libgnss::FGOProcessor::FGOResult& gnss_first_result,
    const std::vector<libgnss::GNSSTime>& raw_epoch_times,
    const std::vector<std::int64_t>& raw_utc_keys,
    const std::vector<double>& raw_drift_mps,
    ImuBuildReport& report,
    std::string& error,
    bool require_raw_drift_coverage = true) {
    report.phase91_raw_drift_d_initializer_enabled = require_raw_drift_coverage;

    std::vector<libgnss::source_clock_c0d::RetainedRawEpochKey> main_keys;
    std::vector<libgnss::source_clock_c0d::RetainedRawEpochKey>
        gnss_first_keys;
    std::vector<libgnss::GNSSTime> solution_times;
    main_keys.reserve(main_problem.epochs.size());
    gnss_first_keys.reserve(gnss_first_problem.epochs.size());
    solution_times.reserve(gnss_first_result.solution.solutions.size());
    for (const auto& epoch : main_problem.epochs) {
        main_keys.push_back({epoch.raw_source_index,
                             epoch.raw_utc_time_millis, epoch.time});
    }
    for (const auto& epoch : gnss_first_problem.epochs) {
        gnss_first_keys.push_back({epoch.raw_source_index,
                                   epoch.raw_utc_time_millis, epoch.time});
    }
    for (const auto& solution : gnss_first_result.solution.solutions) {
        solution_times.push_back(solution.time);
    }

    const auto alignment = [&]() {
        libgnss::source_clock_c0d::ExactEpochAlignmentReport value;
        libgnss::source_clock_c0d::validateRetainedRawDAlignment(
            main_keys, gnss_first_keys, solution_times, raw_epoch_times,
            raw_utc_keys, raw_drift_mps, value, require_raw_drift_coverage);
        return value;
    }();
    report.phase91_main_epoch_count = alignment.main_epoch_count;
    report.phase91_gnss_first_epoch_count = alignment.gnss_first_epoch_count;
    report.phase91_solution_epoch_count = alignment.solution_epoch_count;
    report.phase91_raw_epoch_count = alignment.raw_epoch_count;
    report.phase91_raw_utc_key_count = alignment.raw_utc_key_count;
    report.phase91_aligned_epoch_count = alignment.aligned_epoch_count;
    report.phase91_epoch_count_mismatch_count =
        alignment.epoch_count_mismatch_count;
    report.phase91_nonfinite_time_count = alignment.nonfinite_time_count;
    report.phase91_gnss_first_time_mismatch_count =
        alignment.gnss_first_time_mismatch_count;
    report.phase91_raw_time_mismatch_count = alignment.raw_time_mismatch_count;
    report.phase91_raw_utc_key_order_mismatch_count =
        alignment.raw_utc_key_order_mismatch_count;
    report.phase91_duplicate_raw_utc_key_count =
        alignment.duplicate_raw_utc_key_count;
    report.phase91_raw_utc_key_mismatch_count =
        alignment.raw_utc_key_mismatch_count;
    report.phase91_retained_source_index_mismatch_count =
        alignment.retained_source_index_mismatch_count;
    report.phase91_retained_source_order_mismatch_count =
        alignment.retained_source_order_mismatch_count;
    report.phase91_duplicate_retained_source_index_count =
        alignment.duplicate_retained_source_index_count;
    report.phase91_retained_raw_key_mismatch_count =
        alignment.retained_raw_key_mismatch_count;
    report.phase91_raw_drift_count_mismatch_count =
        alignment.raw_drift_count_mismatch_count;
    report.phase91_raw_drift_nonfinite_count =
        alignment.raw_drift_nonfinite_count;
    if (!alignment.valid) {
        error = alignment.failure;
        report.phase91_raw_drift_d_initializer_failure = error;
        return false;
    }

    if (gnss_first_result.epoch_velocities_ecef_mps.size() !=
        main_problem.epochs.size()) {
        error = "GNSS-first velocity state coverage is not exactly epoch-aligned";
        report.phase91_raw_drift_d_initializer_failure = error;
        return false;
    }
    for (std::size_t i = 0; i < gnss_first_result.solution.solutions.size(); ++i) {
        const auto& solution = gnss_first_result.solution.solutions[i];
        if (!solution.position_ecef.allFinite() ||
            !std::isfinite(solution.receiver_clock_bias) ||
            !gnss_first_result.epoch_velocities_ecef_mps[i].allFinite()) {
            ++report.phase91_nonfinite_solution_count;
        }
    }
    if (report.phase91_nonfinite_solution_count != 0U) {
        error = "GNSS-first handoff contains non-finite position, clock, or velocity";
        report.phase91_raw_drift_d_initializer_failure = error;
        return false;
    }
    report.phase91_raw_drift_d_initializer_alignment_valid = true;
    return true;
}

bool validatePhase93OptimizedDExport(
    const libgnss::FGOProcessor::FGOProblem& main_problem,
    const libgnss::FGOProcessor::FGOResult& gnss_first_result,
    ImuBuildReport& report,
    std::string& error) {
    report.phase93_meter_state_handoff_enabled = true;
    const auto& optimized_d = gnss_first_result.epoch_clock_drift_mps;
    report.phase93_optimized_d_epoch_count = optimized_d.size();
    if (optimized_d.size() != main_problem.epochs.size()) {
        error = "GNSS-first optimized D export is not exactly retained-epoch aligned";
        report.phase93_optimized_d_handoff_failure = error;
        return false;
    }
    for (const double value : optimized_d) {
        if (std::isfinite(value)) {
            ++report.phase93_optimized_d_finite_count;
        } else {
            ++report.phase93_optimized_d_nonfinite_count;
        }
    }
    if (report.phase93_optimized_d_nonfinite_count != 0U) {
        error = "GNSS-first optimized D export contains non-finite state";
        report.phase93_optimized_d_handoff_failure = error;
        return false;
    }
    report.phase93_optimized_d_export_valid = true;
    return true;
}

bool validatePhase101OptimizedCExport(
    const libgnss::FGOProcessor::FGOProblem& main_problem,
    const libgnss::FGOProcessor::FGOProblem& gnss_first_problem,
    const libgnss::FGOProcessor::FGOResult& gnss_first_result,
    ImuBuildReport& report,
    std::string& error) {
    report.phase101_epoch_vector_handoff_enabled = true;
    const auto& optimized_c = gnss_first_result.epoch_clock_bias_components_m;
    report.phase101_optimized_c_epoch_count = optimized_c.size();
    if (optimized_c.size() != main_problem.epochs.size() ||
        gnss_first_problem.epochs.size() != main_problem.epochs.size()) {
        error = "GNSS-first optimized C export is not exactly retained-epoch aligned";
        report.phase101_optimized_c_handoff_failure = error;
        return false;
    }
    for (std::size_t i = 0; i < optimized_c.size(); ++i) {
        const auto& main_epoch = main_problem.epochs[i];
        const auto& first_epoch = gnss_first_problem.epochs[i];
        const double dt = first_epoch.time - main_epoch.time;
        if (first_epoch.raw_source_index != main_epoch.raw_source_index ||
            first_epoch.raw_utc_time_millis != main_epoch.raw_utc_time_millis ||
            !std::isfinite(dt) || std::abs(dt) > 1e-9) {
            error = "GNSS-first optimized C export key/order does not match main retained epochs";
            report.phase101_optimized_c_handoff_failure = error;
            return false;
        }
        bool finite_epoch = true;
        for (const double value : optimized_c[i]) {
            if (std::isfinite(value)) {
                ++report.phase101_optimized_c_finite_count;
            } else {
                ++report.phase101_optimized_c_nonfinite_count;
                finite_epoch = false;
            }
        }
        if (!finite_epoch) {
            error = "GNSS-first optimized C export contains non-finite state";
            report.phase101_optimized_c_handoff_failure = error;
            return false;
        }
    }
    report.phase101_optimized_c_export_valid = true;
    return true;
}

bool earthValidEcef(const libgnss::Vector3d& position) {
    if (!position.allFinite()) return false;
    const double norm = position.norm();
    return std::isfinite(norm) && norm >= 6.0e6 && norm <= 7.0e6;
}

// Phase94 is deliberately an app-local observer.  These records expose the
// already-frozen stage boundaries without adding a solver/configuration field
// or changing the backend's C0/D admission.  In particular, the edge count
// below mirrors the existing source-clock edge predicate so a GNSS-first
// backend exception can be explained before an FGOResult exists.
struct Phase94C0DPreflight {
    bool c0d_factor_enabled = false;
    bool meter_state_parity_enabled = false;
    bool raw_d_initializer_enabled = false;
    bool gnss_first_handoff_enabled = false;
    bool direct_quality_enabled = false;
    bool undifferenced_doppler_enabled = false;
    bool motion_factors_enabled = false;
    bool clock_motion_factors_enabled = false;
    bool pdc_bridge_disabled = false;
    bool fixed_lag_disabled = false;
    bool phone_identity_present = false;
    bool backend_configuration_allowed = false;
    bool source_clock_c0d_problem_path = false;
    bool retained_epoch_count_ge_two = false;
    std::size_t retained_epoch_count = 0;
    std::size_t retained_undifferenced_doppler_factor_count = 0;
    std::size_t eligible_c0d_pair_count = 0;
    std::size_t eligible_c0d_factor_count = 0;
    std::size_t invalid_dt_skip_count = 0;
    std::size_t gap_skip_count = 0;
    std::size_t phone_exclusion_skip_count = 0;
    std::size_t clock_jump_skip_count = 0;
    double dt_min_s = std::numeric_limits<double>::quiet_NaN();
    double dt_max_s = std::numeric_limits<double>::quiet_NaN();
    std::size_t d_initializer_epoch_count = 0;
    std::size_t d_initializer_finite_count = 0;
    std::size_t d_initializer_nonfinite_count = 0;
    bool d_initializer_coverage_valid = false;
    bool d_initializer_all_finite = false;
    bool guard_rejected = false;
    std::string guard_predicate;
    std::vector<std::string> guard_failed_predicates;
};

enum class Phase94C0DSkipReason {
    Eligible,
    InvalidDt,
    Gap,
    PhoneExcluded,
    ClockJump,
};

Phase94C0DSkipReason phase94C0DEdgeReason(double dt_s,
                                          bool clock_jump,
                                          const std::string& phone) {
    // Keep the order identical to nativeSourceClockC0DEdgeDecision in the
    // GTSAM backend: dt, source gap, phone exclusion, then clock jump.
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        return Phase94C0DSkipReason::InvalidDt;
    }
    if (dt_s >= 1.5) {
        return Phase94C0DSkipReason::Gap;
    }
    if (phone.empty() || phone == "sm-a205u" || phone == "sm-a505u" ||
        phone == "samsunga325g") {
        return Phase94C0DSkipReason::PhoneExcluded;
    }
    if (clock_jump) {
        return Phase94C0DSkipReason::ClockJump;
    }
    return Phase94C0DSkipReason::Eligible;
}

Phase94C0DPreflight makePhase94C0DPreflight(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOConfig& config) {
    Phase94C0DPreflight report;
    report.c0d_factor_enabled = config.use_native_source_clock_c0d_factor;
    report.meter_state_parity_enabled =
        config.use_native_source_clock_c0d_meter_state_parity;
    report.raw_d_initializer_enabled =
        config.use_native_source_clock_c0d_raw_drift_d_initializer;
    report.gnss_first_handoff_enabled =
        config.use_native_source_clock_c0d_gnss_first_meter_state_handoff;
    report.direct_quality_enabled = config.use_upstream_observable_quality;
    report.undifferenced_doppler_enabled =
        config.use_undifferenced_doppler_factors;
    report.motion_factors_enabled = config.use_motion_factors;
    report.clock_motion_factors_enabled = config.use_clock_motion_factors;
    report.pdc_bridge_disabled = !config.use_native_pdc_state_bridge;
    report.fixed_lag_disabled = !config.use_fixed_lag_smoother;
    report.phone_identity_present =
        !config.native_source_clock_c0d_phone.empty();
    report.retained_epoch_count = problem.epochs.size();
    report.retained_epoch_count_ge_two = report.retained_epoch_count >= 2U;
    report.retained_undifferenced_doppler_factor_count =
        problem.undifferenced_doppler_factors.size();

    const bool use_imu = config.use_pose3_state && config.use_imu &&
                         problem.imu.valid && report.retained_epoch_count >= 2U;
    const bool use_gnss_velocity_states =
        !use_imu && !config.use_pose3_state && config.use_velocity_states &&
        !problem.undifferenced_doppler_factors.empty();
    const bool gnss_first_point3_velocity_path =
        report.gnss_first_handoff_enabled && !config.use_pose3_state &&
        !config.use_imu && config.use_velocity_states &&
        report.direct_quality_enabled && report.undifferenced_doppler_enabled &&
        report.motion_factors_enabled && report.clock_motion_factors_enabled &&
        report.pdc_bridge_disabled && report.fixed_lag_disabled &&
        report.phone_identity_present;
    const bool pose3_imu_path =
        config.use_pose3_state && config.use_imu &&
        report.direct_quality_enabled && report.undifferenced_doppler_enabled &&
        report.motion_factors_enabled && report.clock_motion_factors_enabled &&
        report.pdc_bridge_disabled && report.fixed_lag_disabled &&
        report.phone_identity_present;
    report.backend_configuration_allowed =
        !report.c0d_factor_enabled || pose3_imu_path ||
        gnss_first_point3_velocity_path;
    report.source_clock_c0d_problem_path =
        use_imu || (report.gnss_first_handoff_enabled &&
                    use_gnss_velocity_states);

    for (const auto& epoch : problem.epochs) {
        if (!std::isfinite(epoch.receiver_clock_drift_mps)) {
            ++report.d_initializer_nonfinite_count;
        } else {
            ++report.d_initializer_finite_count;
        }
    }
    report.d_initializer_epoch_count = problem.epochs.size();
    report.d_initializer_all_finite =
        report.d_initializer_epoch_count > 0U &&
        report.d_initializer_nonfinite_count == 0U &&
        report.d_initializer_finite_count == report.d_initializer_epoch_count;
    report.d_initializer_coverage_valid = report.d_initializer_all_finite;

    const std::string& phone = config.native_source_clock_c0d_phone;
    bool have_dt = false;
    for (std::size_t i = 1; i < problem.epochs.size(); ++i) {
        const double dt_s = problem.epochs[i].time - problem.epochs[i - 1].time;
        if (std::isfinite(dt_s) && dt_s > 0.0) {
            if (!have_dt) {
                report.dt_min_s = dt_s;
                report.dt_max_s = dt_s;
                have_dt = true;
            } else {
                report.dt_min_s = std::min(report.dt_min_s, dt_s);
                report.dt_max_s = std::max(report.dt_max_s, dt_s);
            }
        }
        const bool clock_jump = i < problem.clock_jumps.size() &&
                                problem.clock_jumps[i];
        switch (phase94C0DEdgeReason(dt_s, clock_jump, phone)) {
            case Phase94C0DSkipReason::Eligible:
                ++report.eligible_c0d_pair_count;
                ++report.eligible_c0d_factor_count;
                break;
            case Phase94C0DSkipReason::InvalidDt:
                ++report.invalid_dt_skip_count;
                break;
            case Phase94C0DSkipReason::Gap:
                ++report.gap_skip_count;
                break;
            case Phase94C0DSkipReason::PhoneExcluded:
                ++report.phone_exclusion_skip_count;
                break;
            case Phase94C0DSkipReason::ClockJump:
                ++report.clock_jump_skip_count;
                break;
        }
    }

    report.guard_predicate =
        "use_native_source_clock_c0d_factor && "
        "(!backend_configuration_allowed || "
        "!source_clock_c0d_problem_path || retained_epoch_count < 2)";
    report.guard_rejected =
        report.c0d_factor_enabled &&
        (!report.backend_configuration_allowed ||
         !report.source_clock_c0d_problem_path ||
         !report.retained_epoch_count_ge_two);
    if (!report.backend_configuration_allowed) {
        report.guard_failed_predicates.push_back(
            "backend_configuration_not_allowed");
    }
    if (!report.source_clock_c0d_problem_path) {
        report.guard_failed_predicates.push_back(
            "source_clock_c0d_problem_path_unavailable");
        if (problem.undifferenced_doppler_factors.empty()) {
            report.guard_failed_predicates.push_back(
                "gnss_first_problem.undifferenced_doppler_factors.empty()");
        }
    }
    if (!report.retained_epoch_count_ge_two) {
        report.guard_failed_predicates.push_back(
            "gnss_first_problem.epochs.size() < 2");
    }
    return report;
}

struct Phase94GnssFirstTelemetry {
    bool attempted = false;
    bool result_returned = false;
    bool converged = false;
    std::size_t epochs = 0;
    std::size_t undifferenced_doppler_factors = 0;
    std::size_t velocity_states = 0;
    int iterations = 0;
    double initial_cost = std::numeric_limits<double>::quiet_NaN();
    double final_cost = std::numeric_limits<double>::quiet_NaN();
    bool costs_finite = false;
    std::size_t c0d_factor_count = 0;
    std::size_t c0d_accepted_outer_iterations = 0;
    std::size_t c0d_inner_lambda_attempts = 0;
    bool c0d_active_solve_attempted = false;
    bool c0d_active_solve_finite_costs = false;
    bool strict_cost_progress = false;
    std::size_t optimized_d_epoch_count = 0;
    std::size_t optimized_d_finite_count = 0;
    std::size_t optimized_d_nonfinite_count = 0;
    bool optimized_d_coverage = false;
    bool exact_retained_key_alignment = false;
    bool unknown_guard_reason = false;
    std::string terminal_branch;
    std::string failure;
};

Phase94GnssFirstTelemetry makePhase94GnssFirstTelemetry(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& result,
    const ImuBuildReport& imu_report) {
    Phase94GnssFirstTelemetry report;
    report.attempted = true;
    report.result_returned = true;
    report.converged = result.diagnostics.converged;
    report.epochs = result.solution.solutions.size();
    report.undifferenced_doppler_factors =
        problem.undifferenced_doppler_factors.size();
    report.velocity_states = result.epoch_velocities_ecef_mps.size();
    report.iterations = result.diagnostics.iterations;
    report.initial_cost = result.diagnostics.initial_cost;
    report.final_cost = result.diagnostics.final_cost;
    report.costs_finite = std::isfinite(report.initial_cost) &&
                          std::isfinite(report.final_cost);
    report.c0d_factor_count =
        result.diagnostics.native_source_clock_c0d_factor_count;
    report.c0d_accepted_outer_iterations =
        result.diagnostics.native_source_clock_c0d_accepted_outer_iterations;
    report.c0d_inner_lambda_attempts =
        result.diagnostics.native_source_clock_c0d_total_inner_lambda_attempts;
    report.c0d_active_solve_attempted =
        result.diagnostics.native_source_clock_c0d_active_solve_attempted;
    report.c0d_active_solve_finite_costs =
        result.diagnostics.native_source_clock_c0d_active_solve_finite_costs;
    report.strict_cost_progress =
        result.diagnostics.native_source_clock_c0d_factor_enabled &&
        result.diagnostics.native_source_clock_c0d_meter_state_parity_enabled &&
        report.c0d_factor_count > 0U && report.c0d_active_solve_attempted &&
        report.c0d_accepted_outer_iterations > 0U &&
        report.c0d_active_solve_finite_costs && report.costs_finite &&
        report.final_cost < report.initial_cost;
    report.optimized_d_epoch_count = result.epoch_clock_drift_mps.size();
    for (const double value : result.epoch_clock_drift_mps) {
        if (std::isfinite(value)) {
            ++report.optimized_d_finite_count;
        } else {
            ++report.optimized_d_nonfinite_count;
        }
    }
    report.optimized_d_coverage =
        report.optimized_d_epoch_count == problem.epochs.size() &&
        report.optimized_d_nonfinite_count == 0U;
    report.exact_retained_key_alignment =
        imu_report.phase91_raw_drift_d_initializer_alignment_valid;
    report.terminal_branch =
        result.diagnostics.native_source_clock_c0d_termination_branch_reason;
    report.failure = imu_report.gnss_first_failure;
    return report;
}

struct Phase94MainTelemetry {
    bool attempted = false;
    bool result_returned = false;
    bool converged = false;
    bool solution_nonempty = false;
    std::size_t problem_epoch_count = 0;
    std::size_t position_solution_size = 0;
    bool position_size_matches_problem_epochs = false;
    std::size_t receiver_clock_solution_size = 0;
    bool receiver_clock_size_matches_problem_epochs = false;
    std::size_t earth_valid_position_count = 0;
    std::size_t position_nonfinite_count = 0;
    std::size_t position_out_of_earth_count = 0;
    bool all_positions_earth_valid = false;
    std::size_t receiver_clock_finite_count = 0;
    std::size_t receiver_clock_nonfinite_count = 0;
    bool all_receiver_clocks_finite = false;
    std::size_t optimized_d_epoch_count = 0;
    bool optimized_d_size_matches_problem_epochs = false;
    std::size_t optimized_d_finite_count = 0;
    std::size_t optimized_d_nonfinite_count = 0;
    bool optimized_d_all_finite = false;
    std::size_t velocity_epoch_count = 0;
    bool velocity_size_matches_problem_epochs = false;
    std::size_t velocity_finite_count = 0;
    std::size_t velocity_nonfinite_count = 0;
    bool all_velocities_finite = false;
    bool exact_retained_key_alignment = false;
    bool gnss_first_progress = false;
    bool c0d_factor_enabled = false;
    bool meter_state_parity_enabled = false;
    std::size_t c0d_factor_count = 0;
    bool active_solve_attempted = false;
    std::size_t accepted_outer_iterations = 0;
    std::size_t inner_lambda_attempts = 0;
    double initial_cost = std::numeric_limits<double>::quiet_NaN();
    double final_cost = std::numeric_limits<double>::quiet_NaN();
    bool active_solve_costs_finite = false;
    bool final_cost_strictly_less_than_initial = false;
    double initial_lambda = std::numeric_limits<double>::quiet_NaN();
    double maximum_lambda = std::numeric_limits<double>::quiet_NaN();
    double final_lambda = std::numeric_limits<double>::quiet_NaN();
    double conditioning_proxy = std::numeric_limits<double>::quiet_NaN();
    bool termination_trace_complete = false;
    std::string terminal_branch;
    bool contract_passed = false;
};

Phase94MainTelemetry makePhase94MainTelemetry(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& result,
    const ImuBuildReport& imu_report,
    const Phase94GnssFirstTelemetry& gnss_first) {
    Phase94MainTelemetry report;
    report.attempted = true;
    report.result_returned = true;
    report.converged = result.diagnostics.converged;
    report.solution_nonempty = !result.solution.isEmpty();
    report.problem_epoch_count = problem.epochs.size();
    report.position_solution_size = result.solution.solutions.size();
    report.position_size_matches_problem_epochs =
        report.position_solution_size == report.problem_epoch_count;
    report.receiver_clock_solution_size = result.solution.solutions.size();
    report.receiver_clock_size_matches_problem_epochs =
        report.receiver_clock_solution_size == report.problem_epoch_count;
    for (const auto& solution : result.solution.solutions) {
        if (!solution.position_ecef.allFinite()) {
            ++report.position_nonfinite_count;
        } else if (earthValidEcef(solution.position_ecef)) {
            ++report.earth_valid_position_count;
        } else {
            ++report.position_out_of_earth_count;
        }
        if (std::isfinite(solution.receiver_clock_bias)) {
            ++report.receiver_clock_finite_count;
        } else {
            ++report.receiver_clock_nonfinite_count;
        }
    }
    report.all_positions_earth_valid =
        report.position_size_matches_problem_epochs &&
        report.earth_valid_position_count == report.problem_epoch_count;
    report.all_receiver_clocks_finite =
        report.receiver_clock_size_matches_problem_epochs &&
        report.receiver_clock_finite_count == report.problem_epoch_count;

    report.optimized_d_epoch_count = result.epoch_clock_drift_mps.size();
    report.optimized_d_size_matches_problem_epochs =
        report.optimized_d_epoch_count == report.problem_epoch_count;
    for (const double value : result.epoch_clock_drift_mps) {
        if (std::isfinite(value)) {
            ++report.optimized_d_finite_count;
        } else {
            ++report.optimized_d_nonfinite_count;
        }
    }
    report.optimized_d_all_finite =
        report.optimized_d_size_matches_problem_epochs &&
        report.optimized_d_finite_count == report.problem_epoch_count;

    report.velocity_epoch_count = result.epoch_velocity_nav_mps.size();
    report.velocity_size_matches_problem_epochs =
        report.velocity_epoch_count == report.problem_epoch_count;
    for (const auto& velocity : result.epoch_velocity_nav_mps) {
        if (velocity.allFinite()) {
            ++report.velocity_finite_count;
        } else {
            ++report.velocity_nonfinite_count;
        }
    }
    report.all_velocities_finite =
        report.velocity_size_matches_problem_epochs &&
        report.velocity_finite_count == report.problem_epoch_count;
    report.exact_retained_key_alignment =
        imu_report.phase91_raw_drift_d_initializer_alignment_valid;
    report.gnss_first_progress = gnss_first.strict_cost_progress;

    const auto& diagnostics = result.diagnostics;
    report.c0d_factor_enabled = diagnostics.native_source_clock_c0d_factor_enabled;
    report.meter_state_parity_enabled =
        diagnostics.native_source_clock_c0d_meter_state_parity_enabled;
    report.c0d_factor_count = diagnostics.native_source_clock_c0d_factor_count;
    report.active_solve_attempted =
        diagnostics.native_source_clock_c0d_active_solve_attempted;
    report.accepted_outer_iterations =
        diagnostics.native_source_clock_c0d_accepted_outer_iterations;
    report.inner_lambda_attempts =
        diagnostics.native_source_clock_c0d_total_inner_lambda_attempts;
    report.initial_cost = diagnostics.initial_cost;
    report.final_cost = diagnostics.final_cost;
    report.active_solve_costs_finite =
        diagnostics.native_source_clock_c0d_active_solve_finite_costs &&
        std::isfinite(report.initial_cost) && std::isfinite(report.final_cost);
    report.final_cost_strictly_less_than_initial =
        report.active_solve_costs_finite &&
        report.final_cost < report.initial_cost;
    report.initial_lambda = diagnostics.native_source_clock_c0d_initial_lambda;
    report.maximum_lambda = diagnostics.native_source_clock_c0d_maximum_lambda;
    report.final_lambda = diagnostics.native_source_clock_c0d_final_lambda;
    report.conditioning_proxy =
        diagnostics.native_source_clock_c0d_conditioning_proxy;
    report.termination_trace_complete =
        diagnostics.native_source_clock_c0d_termination_trace_complete;
    report.terminal_branch =
        diagnostics.native_source_clock_c0d_termination_branch_reason;
    report.contract_passed =
        report.exact_retained_key_alignment &&
        report.optimized_d_size_matches_problem_epochs &&
        report.optimized_d_all_finite &&
        report.position_size_matches_problem_epochs &&
        report.receiver_clock_size_matches_problem_epochs &&
        report.all_positions_earth_valid && report.all_receiver_clocks_finite &&
        report.velocity_size_matches_problem_epochs &&
        report.all_velocities_finite && report.c0d_factor_enabled &&
        report.meter_state_parity_enabled && report.c0d_factor_count > 0U &&
        report.active_solve_attempted && report.accepted_outer_iterations > 0U &&
        report.active_solve_costs_finite &&
        report.final_cost_strictly_less_than_initial;
    return report;
}

struct Phase94StageDiagnostics {
    bool enabled = false;
    bool phase96_enabled = false;
    bool phase97_enabled = false;
    bool phase98_enabled = false;
    std::string dataset_id;
    std::string status = "not-started";
    std::string failure_stage = "not-reached";
    std::string failure_reason;
    std::string exception_type;
    std::string exception_message;
    bool solution_output_published = false;
    bool accuracy_output_published = false;
    bool truth_used = false;
    bool mat_used = false;
    bool kaggle_or_token_accessed = false;
    Phase94C0DPreflight gnss_first_preflight;
    Phase94C0DPreflight main_preflight;
    Phase94GnssFirstTelemetry gnss_first;
    Phase94MainTelemetry main;
    libgnss::FGOProcessor::FGOPhase96MainDiagnostics phase96_main;
    libgnss::FGOProcessor::FGOPhase97MainDiagnostics phase97_main;
    libgnss::FGOProcessor::FGOPhase98SolverDiagnostics phase98_solver;
};

bool validateGnssFirstVelocityOnlyHandoff(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& gnss_first_result,
    std::vector<libgnss::Vector3d>& velocities_enu,
    ImuBuildReport& report,
    std::string& error) {
    report.gnss_first_velocity_only_handoff = true;

    report.original_raw_seed_position_count = problem.epochs.size();
    for (const auto& epoch : problem.epochs) {
        if (!earthValidEcef(epoch.position_ecef)) {
            ++report.original_raw_seed_position_invalid_count;
        }
    }
    if (report.original_raw_seed_position_invalid_count != 0U) {
        error = "original raw SPP seed positions are not all Earth-valid";
        return false;
    }

    const auto& solutions = gnss_first_result.solution.solutions;
    const auto& velocities = gnss_first_result.epoch_velocities_ecef_mps;
    for (const auto& solution : solutions) {
        const bool finite_position = solution.position_ecef.allFinite();
        const bool earth_position = earthValidEcef(solution.position_ecef);
        if (!finite_position) ++report.gnss_first_position_nonfinite_count;
        if (finite_position && !earth_position) {
            ++report.gnss_first_position_out_of_earth_count;
        }
        if (!earth_position) {
            ++report.gnss_first_position_invalid_count;
            if (!std::isfinite(report.gnss_first_first_invalid_position_norm_m)) {
                report.gnss_first_first_invalid_position_norm_m =
                    finite_position ? solution.position_ecef.norm()
                                    : std::numeric_limits<double>::quiet_NaN();
            }
        }
        if (!std::isfinite(solution.receiver_clock_bias)) {
            ++report.gnss_first_clock_invalid_count;
        }
    }

    if (solutions.size() != problem.epochs.size() ||
        velocities.size() != problem.epochs.size()) {
        error = "GNSS-first velocity count does not match observation epochs";
        return false;
    }
    for (const auto& velocity : velocities) {
        if (!velocity.allFinite()) {
            ++report.gnss_first_velocity_nonfinite_count;
            continue;
        }
        const double norm = velocity.norm();
        if (!std::isfinite(norm)) {
            ++report.gnss_first_velocity_nonfinite_count;
            continue;
        }
        report.gnss_first_max_velocity_norm_mps =
            std::max(report.gnss_first_max_velocity_norm_mps, norm);
        if (norm > 70.0) {
            ++report.gnss_first_velocity_over_bound_count;
        } else {
            ++report.gnss_first_velocity_valid_count;
        }
    }
    // Convert the independently optimized ECEF velocity states at the
    // original raw SPP origin.  Do not call deriveGnssFirstVelocities here:
    // that historical helper takes its ENU origin from the GNSS-first
    // position result, which is precisely the state this candidate diagnoses
    // and deliberately refuses to hand off.
    const libgnss::Vector3d raw_origin = problem.epochs.front().position_ecef;
    double raw_lat = 0.0;
    double raw_lon = 0.0;
    double raw_height = 0.0;
    libgnss::ecef2geodetic(raw_origin, raw_lat, raw_lon, raw_height);
    if (!earthValidEcef(raw_origin) || !std::isfinite(raw_lat) ||
        !std::isfinite(raw_lon) || !std::isfinite(raw_height)) {
        error = "original raw SPP seed has invalid ENU origin";
        return false;
    }
    velocities_enu.resize(velocities.size());
    for (std::size_t index = 0; index < velocities.size(); ++index) {
        velocities_enu[index] = libgnss::ecef2enu(
            velocities[index], raw_lat, raw_lon);
        if (!velocities_enu[index].allFinite() ||
            !std::isfinite(velocities_enu[index].norm())) {
            error = "GNSS-first ENU velocity sequence is non-finite";
            return false;
        }
    }
    if (report.gnss_first_velocity_nonfinite_count != 0U ||
        report.gnss_first_velocity_over_bound_count != 0U ||
        report.gnss_first_velocity_valid_count != problem.epochs.size()) {
        std::ostringstream detail;
        detail << "GNSS-first optimized Doppler velocity gate failed: count="
               << velocities.size() << "/" << problem.epochs.size()
               << " nonfinite=" << report.gnss_first_velocity_nonfinite_count
               << " over_70_mps=" << report.gnss_first_velocity_over_bound_count
               << " max_mps=" << report.gnss_first_max_velocity_norm_mps;
        error = detail.str();
        return false;
    }
    // This helper is deliberately the only candidate handoff boundary.  The
    // caller does not copy GNSS-first position or clock values in this mode.
    report.gnss_first_positions_clocks_copied = 0U;
    return true;
}

bool validateDirectDopplerWlsHandoff(
    const libgnss::FGOProcessor::FGOProblem& problem,
    std::vector<libgnss::Vector3d>& velocities_enu,
    ImuBuildReport& report,
    std::string& error) {
    report.direct_doppler_wls_handoff = true;
    report.direct_doppler_wls_epochs = problem.epochs.size();
    report.direct_doppler_wls_edge_hold_max_s = 1.0;
    report.direct_doppler_wls_original_raw_seed_position_count =
        problem.epochs.size();
    for (const auto& epoch : problem.epochs) {
        if (!earthValidEcef(epoch.position_ecef)) {
            ++report.direct_doppler_wls_original_raw_seed_position_invalid_count;
        }
    }
    if (report.direct_doppler_wls_original_raw_seed_position_invalid_count != 0U) {
        error = "original raw SPP seed positions are not all Earth-valid";
        return false;
    }
    if (problem.doppler_velocity_wls_estimates.size() != problem.epochs.size()) {
        error = "direct Doppler WLS estimate count does not match observation epochs";
        return false;
    }

    velocities_enu.clear();
    velocities_enu.resize(problem.epochs.size());
    for (std::size_t index = 0; index < problem.epochs.size(); ++index) {
        const auto& estimate = problem.doppler_velocity_wls_estimates[index];
        // A rejected solve still carries the state produced immediately
        // before its final gate.  Preserve the first such diagnostic so a
        // fail-closed run explains whether rows reached a physical gate
        // without treating that state as a handoff candidate.
        if (estimate.rows >= 4 &&
            estimate.reason != "insufficient-rows" &&
            estimate.reason != "nonfinite-or-invalid-row" &&
            estimate.reason != "clock-discontinuity-reset" &&
            report.direct_doppler_wls_first_solved_reason.empty()) {
            report.direct_doppler_wls_first_solved_rows =
                static_cast<std::size_t>(estimate.rows);
            report.direct_doppler_wls_first_solved_reason = estimate.reason;
            if (estimate.velocity_ecef_mps.allFinite() &&
                std::isfinite(estimate.velocity_ecef_mps.norm())) {
                report.direct_doppler_wls_first_solved_velocity_norm_mps =
                    estimate.velocity_ecef_mps.norm();
            }
            if (std::isfinite(estimate.clock_rate_mps)) {
                report.direct_doppler_wls_first_solved_clock_rate_abs_mps =
                    std::abs(estimate.clock_rate_mps);
            }
        }
        if (!estimate.valid) {
            ++report.direct_doppler_wls_rejected_count;
            const bool finite_velocity = estimate.velocity_ecef_mps.allFinite() &&
                                         std::isfinite(estimate.velocity_ecef_mps.norm());
            const bool finite_clock = std::isfinite(estimate.clock_rate_mps);
            if (!finite_velocity || !finite_clock) {
                ++report.direct_doppler_wls_nonfinite_count;
            } else {
                const double velocity_norm = estimate.velocity_ecef_mps.norm();
                report.direct_doppler_wls_max_velocity_norm_mps = std::max(
                    report.direct_doppler_wls_max_velocity_norm_mps, velocity_norm);
                report.direct_doppler_wls_max_clock_rate_abs_mps = std::max(
                    report.direct_doppler_wls_max_clock_rate_abs_mps,
                    std::abs(estimate.clock_rate_mps));
                if (velocity_norm > 70.0) {
                    ++report.direct_doppler_wls_over_bound_count;
                }
                if (std::abs(estimate.clock_rate_mps) > 2000.0) {
                    ++report.direct_doppler_wls_clock_rate_over_bound_count;
                }
            }
            continue;
        }
        const bool finite_velocity = estimate.velocity_ecef_mps.allFinite() &&
                                     std::isfinite(estimate.velocity_ecef_mps.norm());
        const bool finite_clock = std::isfinite(estimate.clock_rate_mps);
        const double velocity_norm = finite_velocity
                                         ? estimate.velocity_ecef_mps.norm()
                                         : std::numeric_limits<double>::quiet_NaN();
        const bool velocity_in_bound = finite_velocity && velocity_norm <= 70.0;
        const bool clock_in_bound = finite_clock &&
                                    std::abs(estimate.clock_rate_mps) <= 2000.0;
        if (!finite_velocity || !finite_clock || !velocity_in_bound ||
            !clock_in_bound) {
            ++report.direct_doppler_wls_rejected_count;
            if (!finite_velocity || !finite_clock) {
                ++report.direct_doppler_wls_nonfinite_count;
            }
            if (!velocity_in_bound && finite_velocity) {
                ++report.direct_doppler_wls_over_bound_count;
            }
            if (!clock_in_bound && finite_clock) {
                ++report.direct_doppler_wls_clock_rate_over_bound_count;
            }
            continue;
        }
        report.direct_doppler_wls_max_velocity_norm_mps = std::max(
            report.direct_doppler_wls_max_velocity_norm_mps, velocity_norm);
        report.direct_doppler_wls_max_clock_rate_abs_mps = std::max(
            report.direct_doppler_wls_max_clock_rate_abs_mps,
            std::abs(estimate.clock_rate_mps));
        if (estimate.propagated) {
            ++report.direct_doppler_wls_propagated_valid_count;
            if (estimate.reason == "bounded-edge-hold") {
                ++report.direct_doppler_wls_edge_hold_count;
            }
        } else {
            ++report.direct_doppler_wls_direct_valid_count;
        }
    }

    const libgnss::Vector3d raw_origin = problem.epochs.front().position_ecef;
    double raw_lat = 0.0;
    double raw_lon = 0.0;
    double raw_height = 0.0;
    libgnss::ecef2geodetic(raw_origin, raw_lat, raw_lon, raw_height);
    if (!earthValidEcef(raw_origin) || !std::isfinite(raw_lat) ||
        !std::isfinite(raw_lon) || !std::isfinite(raw_height)) {
        error = "original raw SPP seed has invalid ENU origin";
        return false;
    }
    for (std::size_t index = 0; index < problem.epochs.size(); ++index) {
        const auto& estimate = problem.doppler_velocity_wls_estimates[index];
        if (!estimate.valid || !estimate.velocity_ecef_mps.allFinite() ||
            !std::isfinite(estimate.velocity_ecef_mps.norm()) ||
            estimate.velocity_ecef_mps.norm() > 70.0 ||
            !std::isfinite(estimate.clock_rate_mps) ||
            std::abs(estimate.clock_rate_mps) > 2000.0) {
            continue;
        }
        velocities_enu[index] = libgnss::ecef2enu(
            estimate.velocity_ecef_mps, raw_lat, raw_lon);
        if (!velocities_enu[index].allFinite() ||
            !std::isfinite(velocities_enu[index].norm()) ||
            velocities_enu[index].norm() > 70.0) {
            ++report.direct_doppler_wls_rejected_count;
            ++report.direct_doppler_wls_nonfinite_count;
        }
    }

    const std::size_t covered = report.direct_doppler_wls_direct_valid_count +
                                report.direct_doppler_wls_propagated_valid_count;
    if (report.direct_doppler_wls_rejected_count != 0U ||
        covered != problem.epochs.size()) {
        std::ostringstream detail;
        detail << std::setprecision(17);
        detail << "direct Doppler WLS coverage gate failed: count=" << covered
               << "/" << problem.epochs.size()
               << " direct=" << report.direct_doppler_wls_direct_valid_count
               << " propagated=" << report.direct_doppler_wls_propagated_valid_count
               << " rejected=" << report.direct_doppler_wls_rejected_count
               << " nonfinite=" << report.direct_doppler_wls_nonfinite_count
               << " over_70_mps=" << report.direct_doppler_wls_over_bound_count
               << " clock_over_2000_mps="
               << report.direct_doppler_wls_clock_rate_over_bound_count
               << " max_mps=" << report.direct_doppler_wls_max_velocity_norm_mps
               << " max_clock_rate_abs_mps="
               << report.direct_doppler_wls_max_clock_rate_abs_mps
               << " first_solved_rows="
               << report.direct_doppler_wls_first_solved_rows
               << " first_solved_reason="
               << (report.direct_doppler_wls_first_solved_reason.empty()
                       ? "none"
                       : report.direct_doppler_wls_first_solved_reason)
               << " first_solved_velocity_norm_mps="
               << report.direct_doppler_wls_first_solved_velocity_norm_mps
               << " first_solved_clock_rate_abs_mps="
               << report.direct_doppler_wls_first_solved_clock_rate_abs_mps;
        error = detail.str();
        return false;
    }
    report.direct_doppler_wls_positions_clocks_copied = 0U;
    return true;
}

bool validateDirectWlsEphemeralMainSeed(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const std::vector<libgnss::GNSSTime>& raw_epoch_times,
    const std::vector<std::int64_t>& raw_utc_keys,
    const std::vector<double>& raw_drift_mps,
    ImuBuildReport& report,
    std::vector<libgnss::Vector3d>& velocity_ecef_mps,
    std::vector<double>& d_handoff_mps,
    std::vector<libgnss::FGOProcessor::EpochClockBiasComponentsM>&
        c_handoff_m,
    std::string& error) {
    report.direct_wls_ephemeral_c7d_main_seed_enabled = true;
    report.direct_wls_ephemeral_raw_epoch_count = raw_epoch_times.size();
    report.direct_wls_ephemeral_retained_epoch_count = problem.epochs.size();
    velocity_ecef_mps.assign(
        problem.epochs.size(),
        libgnss::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
    std::vector<double> retained_clock_bias_m;
    std::vector<double> retained_drift;
    std::vector<libgnss::source_clock_c0d::RetainedRawEpochKey> retained_keys;
    std::vector<libgnss::Vector3d> retained_positions;
    retained_clock_bias_m.reserve(problem.epochs.size());
    retained_drift.reserve(problem.epochs.size());
    retained_keys.reserve(problem.epochs.size());
    retained_positions.reserve(problem.epochs.size());

    if (problem.doppler_velocity_wls_estimates.size() !=
        problem.epochs.size()) {
        error = "direct WLS estimate count does not match retained epochs";
        report.direct_wls_ephemeral_failure = error;
        return false;
    }
    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        const auto& epoch = problem.epochs[i];
        if (!epoch.receiver_clock_bias_is_meters) {
            error = "direct WLS seed clock is not explicitly metre-valued";
            report.direct_wls_ephemeral_failure = error;
            return false;
        }
        retained_keys.push_back({epoch.raw_source_index,
                                 epoch.raw_utc_time_millis, epoch.time});
        retained_positions.push_back(epoch.position_ecef);
        retained_clock_bias_m.push_back(epoch.receiver_clock_bias_m);
        retained_drift.push_back(epoch.receiver_clock_drift_mps);
        const auto& estimate = problem.doppler_velocity_wls_estimates[i];
        if (estimate.valid && estimate.velocity_ecef_mps.allFinite() &&
            std::isfinite(estimate.velocity_ecef_mps.norm()) &&
            estimate.velocity_ecef_mps.norm() <= 70.0 &&
            std::isfinite(estimate.clock_rate_mps) &&
            std::abs(estimate.clock_rate_mps) <= 2000.0) {
            velocity_ecef_mps[i] = estimate.velocity_ecef_mps;
        }
        if (!earthValidEcef(epoch.position_ecef)) {
            error = "direct WLS seed position is not Earth-valid";
            report.direct_wls_ephemeral_failure = error;
            return false;
        }
    }

    libgnss::source_clock_c0d::DirectWlsEphemeralC7DSeedReport seed_report;
    const bool valid =
        libgnss::source_clock_c0d::validateAndCopyDirectWlsEphemeralC7DSeed(
            retained_keys, retained_positions, retained_clock_bias_m,
            retained_drift, velocity_ecef_mps, raw_epoch_times, raw_utc_keys,
            raw_drift_mps, c_handoff_m, d_handoff_mps, seed_report);
    report.direct_wls_ephemeral_raw_epoch_count = seed_report.raw_epoch_count;
    report.direct_wls_ephemeral_retained_epoch_count =
        seed_report.retained_epoch_count;
    report.direct_wls_ephemeral_exact_key_count = seed_report.exact_key_count;
    report.direct_wls_ephemeral_finite_position_count =
        seed_report.finite_position_count;
    report.direct_wls_ephemeral_finite_clock_count =
        seed_report.finite_clock_count;
    report.direct_wls_ephemeral_finite_drift_count =
        seed_report.finite_drift_count;
    report.direct_wls_ephemeral_finite_velocity_count =
        seed_report.finite_velocity_count;
    report.direct_wls_ephemeral_raw_key_mismatch_count =
        seed_report.raw_key_mismatch_count;
    report.direct_wls_ephemeral_raw_key_order_mismatch_count =
        seed_report.raw_key_order_mismatch_count;
    report.direct_wls_ephemeral_raw_drift_mismatch_count =
        seed_report.raw_drift_mismatch_count;
    report.direct_wls_ephemeral_full_raw_coverage =
        seed_report.full_raw_coverage;
    report.direct_wls_ephemeral_c7d_main_seed_valid = valid;
    if (!valid) {
        error = seed_report.failure.empty()
                    ? "direct WLS ephemeral C7/D seed failed closed"
                    : seed_report.failure;
        report.direct_wls_ephemeral_failure = error;
        return false;
    }
    return true;
}

struct RawUtcOutputReport {
    bool enabled = false;
    bool warmup_epoch_excluded = false;
    std::size_t raw_epoch_keys = 0;
    std::size_t target_epochs = 0;
    std::size_t exact_solution_epochs = 0;
    std::size_t interpolated_epochs = 0;
    std::size_t edge_hold_epochs = 0;
    std::size_t unresolved_epochs = 0;
    double solution_time_tolerance_ms = 0.0;
    double max_interpolation_gap_ms = 0.0;
    double max_edge_hold_gap_ms = 0.0;
};

struct NativePdcBridgeReport {
    bool enabled = false;
    bool ok = false;
    std::size_t epochs = 0;
    std::size_t pseudorange_rows = 0;
    std::size_t doppler_rows = 0;
    std::size_t motion_intervals = 0;
    std::size_t valid_position_states = 0;
    std::size_t rejected_position_states = 0;
    std::size_t valid_velocity_states = 0;
    std::size_t state_seeds = 0;
    int iterations = 0;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double max_velocity_norm_mps = 0.0;
    double max_clock_rate_abs_mps = 0.0;
    double max_normalized_rms = 0.0;
    std::size_t first_epoch_doppler_rows = 0;
    std::size_t later_epoch_doppler_rows = 0;
    std::size_t epochs_with_doppler = 0;
    std::size_t finite_raw_clock_drift_epochs = 0;
    // Keep the disabled diagnostic JSON standards-compliant.  solve() fills
    // the actual minimum when the bridge is enabled; zero means "not run".
    double min_epoch_dt_s = 0.0;
    double max_epoch_dt_s = 0.0;
    std::size_t invalid_or_large_epoch_intervals = 0;
    double doppler_residual_rms_mps = 0.0;
    double doppler_abs_p50_mps = 0.0;
    double doppler_abs_max_mps = 0.0;
    // As above, a disabled bridge must not serialize IEEE infinity as JSON.
    double doppler_sigma_min_mps = 0.0;
    double doppler_sigma_max_mps = 0.0;
    double initial_pseudorange_rms_m = 0.0;
    double initial_pseudorange_max_m = 0.0;
    double seed_position_step_max_mps = 0.0;
    double all_state_max_velocity_norm_mps = 0.0;
    double all_state_max_clock_rate_abs_mps = 0.0;
    std::size_t all_state_velocity_over_bound = 0;
    std::size_t all_state_clock_rate_over_bound = 0;
    std::size_t wls_valid_epochs = 0;
    std::size_t wls_rejected_epochs = 0;
    std::size_t wls_propagated_epochs = 0;
    double wls_max_velocity_norm_mps = 0.0;
    double wls_max_clock_rate_abs_mps = 0.0;
    double wls_max_normalized_rms = 0.0;
    double wls_first_velocity_norm_mps = 0.0;
    double wls_first_clock_rate_mps = 0.0;
    std::string wls_first_reason;
    std::size_t integrated_seed_anchor_index = 0;
    std::size_t integrated_seed_epochs = 0;
    std::size_t integrated_seed_held_velocity_epochs = 0;
    std::size_t integrated_seed_spp_fallback_epochs = 0;
    std::size_t integrated_seed_reset_intervals = 0;
    double integrated_seed_max_step_speed_mps = 0.0;
    double integrated_seed_max_displacement_from_spp_m = 0.0;
    std::size_t fgo_seed_displacement_count = 0;
    double fgo_seed_displacement_p50_m = 0.0;
    double fgo_seed_displacement_max_m = 0.0;
    std::string failure;
};

struct TdcpRuntimeReport {
    struct SignalAggregate {
        std::size_t count = 0;
        std::size_t tail_count = 0;
        double residual_sum_m = 0.0;
        double residual_squared_sum_m2 = 0.0;
        double huber_cost_sum = 0.0;
        double max_abs_residual_m = 0.0;
    };
    // Enum-valued system/signal keys, no satellite IDs or epoch coordinates.
    std::map<std::pair<int, int>, SignalAggregate> signal_aggregates;
    std::size_t robust_tail_count = 0;
    std::size_t robust_cost_nonfinite_count = 0;
    double robust_cost_sum = 0.0;
    double quadratic_cost_sum = 0.0;
    bool enabled = false;
    std::size_t factors_built = 0;
    std::size_t factors_inserted = 0;
    std::size_t candidate_pairs = 0;
    std::size_t rejected_gap = 0;
    std::size_t rejected_clock_discontinuity = 0;
    std::size_t rejected_missing_previous = 0;
    std::size_t rejected_loss_of_lock = 0;
    std::size_t rejected_invalid_measurement = 0;
    std::size_t rejected_code_phase_jump = 0;
    std::size_t rejected_invalid_weight = 0;
    std::size_t finite_residuals = 0;
    std::size_t nonfinite_residuals = 0;
    std::size_t arc_count = 0;
    std::size_t min_arc_length_epochs = 0;
    std::size_t max_arc_length_epochs = 0;
    double median_arc_length_epochs = 0.0;
    double sigma_m = kNativeTdcpSigmaM;
    double max_gap_s = kNativeTdcpMaxGapS;
    double code_phase_jump_threshold_m = kNativeTdcpCodePhaseJumpThresholdM;
    double residual_rms_m = 0.0;
    double normalized_residual_rms = 0.0;
    double max_abs_residual_m = 0.0;
};

// Phase116 is deliberately a read-only incidence/cost report for the
// existing ordinary (same-satellite, same-signal) TDCP factors.  It contains
// no carrier ambiguity/DD state and never feeds a value back into the graph.
struct Phase116CarrierTdcpSignalReport {
    std::string signal;
    std::string frequency_band;
    std::size_t carrier_rows_seen = 0U;
    std::size_t carrier_phase_rows = 0U;
    std::size_t retained_carrier_rows = 0U;
    std::size_t missing_wavelength = 0U;
    std::size_t nonfinite_measurements = 0U;
    std::size_t candidate_pairs = 0U;
    std::size_t accepted_pairs = 0U;
    std::size_t rejected_gap = 0U;
    std::size_t rejected_clock_discontinuity = 0U;
    std::size_t rejected_missing_previous = 0U;
    std::size_t rejected_loss_of_lock = 0U;
    std::size_t rejected_nonfinite = 0U;
    std::size_t rejected_code_phase_jump = 0U;
    std::size_t rejected_invalid_weight = 0U;
    double sigma_m = kNativeTdcpSigmaM;
    std::size_t initial_residual_count = 0U;
    std::size_t final_residual_count = 0U;
    std::size_t initial_nonfinite_residual_count = 0U;
    std::size_t final_nonfinite_residual_count = 0U;
    std::size_t initial_robust_outlier_count = 0U;
    std::size_t final_robust_outlier_count = 0U;
    double initial_robust_cost = std::numeric_limits<double>::quiet_NaN();
    double final_robust_cost = std::numeric_limits<double>::quiet_NaN();
    double initial_unwhitened_cost = std::numeric_limits<double>::quiet_NaN();
    double final_unwhitened_cost = std::numeric_limits<double>::quiet_NaN();
    double initial_whitened_cost = std::numeric_limits<double>::quiet_NaN();
    double final_whitened_cost = std::numeric_limits<double>::quiet_NaN();
    std::size_t connected_epoch_count = 0U;
    std::set<std::string> connected_keys;
};

struct Phase116CarrierTdcpReport {
    bool enabled = false;
    // These are explicit invariants of this diagnostic lane.  They are
    // serialized so a result cannot be mistaken for a DD/ambiguity run.
    bool read_only = true;
    bool ordinary_tdcp_only = true;
    bool standalone_carrier_phase_factors = false;
    bool double_difference_factors = false;
    bool ambiguity_states = false;
    bool pdc_state_bridge = false;
    std::size_t factors_built = 0U;
    std::size_t factors_inserted = 0U;
    bool factors_inserted_exact = false;
    double sigma_m = kNativeTdcpSigmaM;
    double graph_initial_cost = std::numeric_limits<double>::quiet_NaN();
    double graph_final_cost = std::numeric_limits<double>::quiet_NaN();
    std::map<libgnss::SignalType, Phase116CarrierTdcpSignalReport> signals;
};

struct OutputPosition {
    std::int64_t utc_time_millis = 0;
    libgnss::Vector3d position_ecef = libgnss::Vector3d::Zero();
};

struct Phase104StageExportReport {
    bool enabled = false;
    bool attempted = false;
    bool written = false;
    bool exact_retained_key_alignment = false;
    bool all_positions_finite = false;
    bool all_positions_earth_valid = false;
    std::string path;
    std::size_t source_epoch_count = 0U;
    std::size_t target_epoch_count = 0U;
    std::size_t exact_epoch_count = 0U;
    std::size_t interpolated_epoch_count = 0U;
    std::size_t edge_hold_epoch_count = 0U;
    std::size_t unresolved_epoch_count = 0U;
    std::size_t finite_position_count = 0U;
    std::size_t nonfinite_position_count = 0U;
    std::size_t out_of_earth_position_count = 0U;
    std::string failure;
};

struct Phase104MainDisplacementReport {
    bool enabled = false;
    bool attempted = false;
    bool written = false;
    bool all_transitions_finite = false;
    std::string path;
    std::size_t solution_epoch_count = 0U;
    std::size_t transition_count = 0U;
    std::size_t finite_transition_count = 0U;
    std::size_t nonfinite_transition_count = 0U;
    double p50_displacement_m = std::numeric_limits<double>::quiet_NaN();
    double p95_displacement_m = std::numeric_limits<double>::quiet_NaN();
    double max_displacement_m = std::numeric_limits<double>::quiet_NaN();
    double max_speed_mps = std::numeric_limits<double>::quiet_NaN();
    std::size_t over_70_mps_count = 0U;
    std::string failure;
};

double phase104LinearPercentile(std::vector<double> values, double quantile) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                  values.end());
    if (values.empty() || !std::isfinite(quantile) || quantile < 0.0 ||
        quantile > 1.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    if (values.size() == 1U) return values.front();
    const double rank = quantile * static_cast<double>(values.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(rank));
    const std::size_t upper = std::min(lower + 1U, values.size() - 1U);
    return values[lower] + (rank - static_cast<double>(lower)) *
                             (values[upper] - values[lower]);
}

bool exportPhase104StageEcef(const Options& options,
                             const libgnss::FGOProcessor::FGOProblem& problem,
                             const libgnss::FGOProcessor::FGOResult& stage,
                             const std::vector<std::int64_t>& raw_utc_keys,
                             Phase104StageExportReport& report,
                             std::string& error) {
    report.enabled = true;
    report.attempted = true;
    report.path = options.phase104_stage_ecef_path;
    report.source_epoch_count = stage.solution.solutions.size();
    if (stage.solution.solutions.size() != problem.epochs.size() ||
        raw_utc_keys.size() != problem.epochs.size() ||
        raw_utc_keys.size() < 2U) {
        error = "Phase104 stage export source/retained epoch count mismatch";
        report.failure = error;
        return false;
    }

    std::ostringstream csv;
    csv << "phone,UnixTimeMillis,PositionEcefX_m,PositionEcefY_m,PositionEcefZ_m\n";
    csv << std::setprecision(17);
    report.target_epoch_count = raw_utc_keys.size() - 1U;
    report.interpolated_epoch_count = 0U;
    report.edge_hold_epoch_count = 0U;
    report.unresolved_epoch_count = 0U;
    for (std::size_t index = 0U; index < raw_utc_keys.size(); ++index) {
        const auto& epoch = problem.epochs[index];
        const auto& solution = stage.solution.solutions[index];
        if (epoch.raw_utc_time_millis != raw_utc_keys[index] ||
            unixMillis(solution.time) !=
                static_cast<double>(raw_utc_keys[index])) {
            error = "Phase104 stage export timestamp/source key mismatch";
            report.failure = error;
            return false;
        }
        if (index == 0U) continue;
        ++report.exact_epoch_count;
        if (!solution.position_ecef.allFinite()) {
            ++report.nonfinite_position_count;
            continue;
        }
        if (!earthValidEcef(solution.position_ecef)) {
            ++report.out_of_earth_position_count;
            continue;
        }
        ++report.finite_position_count;
        csv << options.dataset_id << ',' << raw_utc_keys[index] << ','
            << solution.position_ecef.x() << ',' << solution.position_ecef.y()
            << ',' << solution.position_ecef.z() << '\n';
    }
    report.exact_retained_key_alignment =
        report.exact_epoch_count == report.target_epoch_count;
    report.all_positions_finite =
        report.finite_position_count == report.target_epoch_count &&
        report.nonfinite_position_count == 0U;
    report.all_positions_earth_valid =
        report.finite_position_count == report.target_epoch_count &&
        report.out_of_earth_position_count == 0U;
    if (!report.exact_retained_key_alignment ||
        !report.all_positions_finite || !report.all_positions_earth_valid) {
        error = "Phase104 stage export failed exact-key/finite/Earth-valid contract";
        report.failure = error;
        return false;
    }
    if (!atomicWrite(options.phase104_stage_ecef_path, csv.str())) {
        error = "failed to atomically write Phase104 stage ECEF sidecar";
        report.failure = error;
        return false;
    }
    report.written = true;
    return true;
}

bool writePhase104MainDisplacementStats(
    const Options& options,
    const libgnss::FGOProcessor::FGOResult& result,
    Phase104MainDisplacementReport& report,
    std::string& error) {
    report.enabled = true;
    report.attempted = true;
    report.path = options.phase104_main_displacement_stats_path;
    report.solution_epoch_count = result.solution.solutions.size();
    if (report.solution_epoch_count < 2U) {
        error = "Phase104 main displacement stats require at least two solutions";
        report.failure = error;
        return false;
    }
    std::vector<double> displacements;
    displacements.reserve(report.solution_epoch_count - 1U);
    report.transition_count = report.solution_epoch_count - 1U;
    double max_speed = 0.0;
    for (std::size_t index = 1U; index < report.solution_epoch_count; ++index) {
        const auto& previous = result.solution.solutions[index - 1U];
        const auto& current = result.solution.solutions[index];
        const double dt_s = (unixMillis(current.time) -
                             unixMillis(previous.time)) /
                            1000.0;
        const double displacement_m =
            (current.position_ecef - previous.position_ecef).norm();
        const double speed_mps = displacement_m / dt_s;
        if (!std::isfinite(dt_s) || dt_s <= 0.0 ||
            !current.position_ecef.allFinite() ||
            !previous.position_ecef.allFinite() ||
            !std::isfinite(displacement_m) || !std::isfinite(speed_mps)) {
            ++report.nonfinite_transition_count;
            continue;
        }
        ++report.finite_transition_count;
        displacements.push_back(displacement_m);
        max_speed = std::max(max_speed, speed_mps);
        if (speed_mps > 70.0) ++report.over_70_mps_count;
    }
    report.all_transitions_finite =
        report.finite_transition_count == report.transition_count &&
        report.nonfinite_transition_count == 0U;
    if (!report.all_transitions_finite || displacements.empty()) {
        error = "Phase104 main displacement stats contain nonfinite/invalid transition";
        report.failure = error;
        return false;
    }
    report.p50_displacement_m = phase104LinearPercentile(displacements, 0.50);
    report.p95_displacement_m = phase104LinearPercentile(displacements, 0.95);
    report.max_displacement_m = *std::max_element(displacements.begin(),
                                                  displacements.end());
    report.max_speed_mps = max_speed;
    if (!std::isfinite(report.p50_displacement_m) ||
        !std::isfinite(report.p95_displacement_m) ||
        !std::isfinite(report.max_displacement_m) ||
        !std::isfinite(report.max_speed_mps)) {
        error = "Phase104 main displacement stats are nonfinite";
        report.failure = error;
        return false;
    }
    std::ostringstream json;
    json << std::setprecision(17)
         << "{\n"
         << "  \"schema_version\": \"smartphone-r5-phase104-main-displacement-stats.v1\",\n"
         << "  \"diagnostic_only\": true,\n"
         << "  \"coordinate_rows_exported\": false,\n"
         << "  \"solution_epoch_count\": " << report.solution_epoch_count << ",\n"
         << "  \"transition_count\": " << report.transition_count << ",\n"
         << "  \"finite_transition_count\": " << report.finite_transition_count << ",\n"
         << "  \"nonfinite_transition_count\": " << report.nonfinite_transition_count << ",\n"
         << "  \"p50_displacement_m\": " << report.p50_displacement_m << ",\n"
         << "  \"p95_displacement_m\": " << report.p95_displacement_m << ",\n"
         << "  \"max_displacement_m\": " << report.max_displacement_m << ",\n"
         << "  \"max_speed_mps\": " << report.max_speed_mps << ",\n"
         << "  \"over_70_mps_count\": " << report.over_70_mps_count << "\n"
         << "}\n";
    if (!atomicWrite(options.phase104_main_displacement_stats_path,
                     json.str())) {
        error = "failed to atomically write Phase104 main displacement stats";
        report.failure = error;
        return false;
    }
    report.written = true;
    return true;
}

struct UpstreamPositionOffsetReport {
    bool enabled = false;
    bool applied = false;
    // Exactly one successful post-solve application pass is permitted.  This
    // is populated at the sole mutation boundary and is not reconstructed by
    // a wrapper from corrected row counts.
    std::size_t application_passes = 0U;
    std::string phone;
    std::size_t corrected_epochs = 0U;
    double offset_rl_m = 0.0;
    double offset_ud_m = 0.0;
    double max_offset_enu_m = 0.0;
    std::string failure;
};

// This ledger belongs to the final-output boundary, not to the GNSS-first
// stage or its in-memory handoff.  It makes a second post-solve application a
// fail-closed event before any trajectory row can be mutated.
struct UpstreamPositionOffsetApplicationGuard {
    bool claimed = false;

    bool claim() noexcept {
        if (claimed) return false;
        claimed = true;
        return true;
    }
};

struct BasePseudorangeCompensationReport {
    bool enabled = false;
    bool source_complete = false;
    bool phase127_enabled = false;
    bool phase128_enabled = false;
    bool phase129_enabled = false;
    bool phase129_configuration_valid = true;
    std::string phase128_header_status = "absent";
    std::size_t phase128_canonical_records = 0U;
    std::size_t phase128_canonical_rejected_records = 0U;
    std::size_t phase127_glonass_rows = 0U;
    std::size_t phase127_accepted_rows = 0U;
    std::size_t phase127_header_primary_rows = 0U;
    std::size_t phase127_ephemeris_fallback_rows = 0U;
    std::size_t phase127_header_entries_seen = 0U;
    std::size_t phase127_header_duplicate_entries = 0U;
    std::size_t phase127_header_conflict_entries = 0U;
    std::size_t phase127_header_malformed_entries = 0U;
    std::size_t phase127_ephemeris_candidates = 0U;
    std::size_t phase127_ephemeris_ties = 0U;
    std::size_t phase127_ephemeris_duplicate_entries = 0U;
    std::size_t phase127_ephemeris_conflict_entries = 0U;
    std::size_t phase127_query_time_coverage_gaps = 0U;
    std::size_t phase127_invalid_channels = 0U;
    std::map<std::string, std::size_t> phase127_failure_counts;
    std::size_t phase129_glonass_local_miss_rows = 0U;
    std::map<std::string, std::size_t> phase129_glonass_local_miss_counts;
    std::size_t phase129_glonass_local_miss_streams = 0U;
    bool phase129_glonass_row_count_consistent = true;
    std::string phase129_configuration_failure;
    bool phase131_enabled = false;
    bool phase131_configuration_valid = true;
    std::string phase131_configuration_failure;
    std::size_t phase131_canonical_rows = 0U;
    std::size_t phase131_canonical_rejected_rows = 0U;
    std::size_t phase131_unknown_band_rows = 0U;
    std::size_t phase131_canonical_key_conflicts = 0U;
    std::size_t phase131_canonical_duplicate_rows = 0U;
    std::size_t phase131_canonical_streams = 0U;
    std::size_t phase131_canonical_selected_streams = 0U;
    std::size_t phase131_canonical_merged_streams = 0U;
    std::map<std::string, std::size_t> phase131_failure_counts;
    // Phase126's A/B/C compound transaction markers.  They are monotonic:
    // C is set only after the corrected factor vector has been committed;
    // any failed step returns before a candidate graph can be solved.
    bool phase126_atomic_step_a_raw_ingress_verified = false;
    bool phase126_atomic_step_b_source_stream_verified = false;
    bool phase126_atomic_step_c_application_committed = false;
    bool phase126_compound_admitted = false;
    bool station_reference_verified = false;
    bool antenna_reference_is_approx_position = false;
    bool antenna_delta_present = false;
    bool antenna_delta_applied = false;
    bool official_no_explicit_tgd_bgd = false;
    std::size_t sagnac_evaluations = 0U;
    std::size_t source_complete_signal_rows = 0U;
    bool built = false;
    bool applied = false;
    bool correction_applied_exactly_once = false;
    bool source_miss_conservation = false;
    bool no_raw_or_zero_fallback = false;
    bool duplicate_correction_rejected = false;
    bool preserve_additional_frequency_bands = false;
    std::string base_rinex_path;
    std::string base_rinex_sha256;
    std::string failure;
    double header_version = std::numeric_limits<double>::quiet_NaN();
    double header_interval_s = std::numeric_limits<double>::quiet_NaN();
    double base_interval_s = std::numeric_limits<double>::quiet_NaN();
    double expected_interval_s = std::numeric_limits<double>::quiet_NaN();
    libgnss::Vector3d base_position_ecef = libgnss::Vector3d::Zero();
    std::uintmax_t base_rinex_bytes = 0U;
    std::size_t moving_mean_samples = 0U;
    std::size_t base_epochs = 0U;
    std::size_t base_observation_rows = 0U;
    std::size_t matching_streams = 0U;
    std::size_t matched_base_rows = 0U;
    std::size_t finite_base_residual_rows = 0U;
    std::size_t smoothed_rows = 0U;
    std::size_t interpolated_rows = 0U;
    std::size_t interpolation_misses = 0U;
    std::size_t adopted_pseudorange_rows = 0U;
    std::size_t adopted_rows_corrected = 0U;
    // Phase71 exact denominator accounting.  A matched row has an exact
    // (satellite,SignalType) stream in the finite base model; finite rows are
    // the in-domain, finite correction subset of those matched rows.
    std::size_t selected_band_observation_rows = 0U;
    std::size_t selected_band_streams = 0U;
    std::map<std::string, std::size_t>
        selected_band_observation_rows_by_signal;
    std::map<std::string, std::size_t> selected_band_streams_by_signal;
    std::size_t matched_factor_rows = 0U;
    std::size_t finite_correction_rows_among_matched = 0U;
    std::size_t source_model_build_count = 0U;
    std::size_t correction_application_pass_count = 0U;
    std::size_t corrected_rows = 0U;
    std::map<libgnss::SignalType,
             libgnss::source_pseudorange_miss_mask::SignalCounts>
        source_miss_taxonomy_by_signal;
    bool signal_count_consistent = false;
    // Phase73 source-exact graph miss-mask accounting.  These fields are
    // populated only when the new opt-in is enabled; legacy base-compensation
    // summaries retain their historical semantics and bytes.
    bool source_miss_mask_enabled = false;
    bool source_miss_mask_canonical_key_mode = false;
    std::string source_miss_mask_matching_key = "(satellite,signal)";
    std::size_t original_adopted_pseudorange_rows = 0U;
    std::size_t retained_finite_pc_pseudorange_rows = 0U;
    std::size_t dropped_missing_exact_stream_rows = 0U;
    std::size_t dropped_out_of_domain_rows = 0U;
    std::size_t dropped_nonfinite_correction_rows = 0U;
    double retained_finite_pc_fraction = 0.0;
    double retained_over_original_fraction = 0.0;
    bool pseudorange_factor_count_consistent = false;
    double correction_abs_p50_m = std::numeric_limits<double>::quiet_NaN();
    double correction_abs_p95_m = std::numeric_limits<double>::quiet_NaN();
    double correction_abs_max_m = std::numeric_limits<double>::quiet_NaN();
};

// Build the one summary-boundary snapshot consumed by
// FGOProblemDiagnostics::synchronizePhase131Diagnostics().  The source
// report is authoritative for both canonicalization and source-exact miss
// accounting; this function only copies/derives telemetry and never touches
// the factor vector or any optimizer input.
libgnss::FGOProcessor::FGOProblemDiagnostics::Phase131DiagnosticsSnapshot
phase131DiagnosticsSnapshotFromBaseReport(
    const BasePseudorangeCompensationReport& report) {
    using Snapshot =
        libgnss::FGOProcessor::FGOProblemDiagnostics::Phase131DiagnosticsSnapshot;
    Snapshot snapshot;
    snapshot.enabled = report.phase131_enabled;
    snapshot.configuration_valid = report.phase131_configuration_valid;
    snapshot.configuration_failure = report.phase131_configuration_failure;
    snapshot.canonical_rows = report.phase131_canonical_rows;
    snapshot.canonical_rejected_rows = report.phase131_canonical_rejected_rows;
    snapshot.unknown_band_rows = report.phase131_unknown_band_rows;
    snapshot.canonical_key_conflicts = report.phase131_canonical_key_conflicts;
    snapshot.canonical_duplicate_rows = report.phase131_canonical_duplicate_rows;
    snapshot.canonical_streams = report.phase131_canonical_streams;
    snapshot.canonical_selected_streams =
        report.phase131_canonical_selected_streams;
    snapshot.canonical_merged_streams = report.phase131_canonical_merged_streams;
    snapshot.failure_counts = report.phase131_failure_counts;
    snapshot.canonicalization_attempt_rows =
        report.phase131_canonical_rows + report.phase131_canonical_rejected_rows;
    snapshot.resolver_call_count = snapshot.canonicalization_attempt_rows;
    snapshot.source_miss_mask_enabled = report.source_miss_mask_enabled;
    snapshot.source_miss_mask_canonical_key_mode =
        report.source_miss_mask_canonical_key_mode;
    snapshot.source_miss_mask_matching_key =
        report.source_miss_mask_matching_key;
    snapshot.original_adopted_pseudorange_rows =
        report.original_adopted_pseudorange_rows;
    snapshot.retained_finite_pc_pseudorange_rows =
        report.retained_finite_pc_pseudorange_rows;
    snapshot.dropped_missing_exact_stream_rows =
        report.dropped_missing_exact_stream_rows;
    snapshot.dropped_out_of_domain_rows = report.dropped_out_of_domain_rows;
    snapshot.dropped_nonfinite_correction_rows =
        report.dropped_nonfinite_correction_rows;
    snapshot.matched_factor_rows = report.matched_factor_rows;
    snapshot.finite_correction_rows_among_matched =
        report.finite_correction_rows_among_matched;
    snapshot.source_model_build_count = report.source_model_build_count;
    snapshot.correction_application_pass_count =
        report.correction_application_pass_count;
    snapshot.corrected_rows = report.corrected_rows;
    snapshot.pseudorange_factor_count_consistent =
        report.pseudorange_factor_count_consistent;
    snapshot.signal_count_consistent = report.signal_count_consistent;
    snapshot.applied = report.applied;
    snapshot.correction_applied_exactly_once =
        report.correction_applied_exactly_once;
    snapshot.duplicate_correction_rejected =
        report.duplicate_correction_rejected;
    return snapshot;
}

double finiteMedian(std::vector<double> values) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                  values.end());
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return values.size() % 2U == 0U
               ? 0.5 * (values[middle - 1U] + values[middle])
               : values[middle];
}

double finitePercentile(std::vector<double> values, double percentile) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                  values.end());
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    if (values.size() == 1U) return values.front();
    const double rank = 0.5 + percentile / 100.0 *
                                      static_cast<double>(values.size());
    if (rank <= 1.0) return values.front();
    if (rank >= static_cast<double>(values.size())) return values.back();
    const double lower_rank = std::floor(rank);
    const std::size_t lower = static_cast<std::size_t>(lower_rank - 1.0);
    const std::size_t upper = lower + 1U;
    return values[lower] + (rank - lower_rank) *
                             (values[upper] - values[lower]);
}

bool selectBaseSampling(const libgnss::ObservationSeries& base_series,
                        double& interval_s,
                        std::size_t& moving_mean_samples,
                        std::string& failure) {
    std::vector<double> intervals;
    intervals.reserve(base_series.epochs.size());
    for (std::size_t index = 1U; index < base_series.epochs.size(); ++index) {
        const double dt = base_series.epochs[index].time -
                          base_series.epochs[index - 1U].time;
        if (!std::isfinite(dt) || !(dt > 0.0)) {
            failure = "base epoch times are not strictly increasing";
            return false;
        }
        intervals.push_back(dt);
    }
    interval_s = finiteMedian(intervals);
    if (!std::isfinite(interval_s)) {
        failure = "base observation series has no observed interval";
        return false;
    }
    if (std::abs(interval_s - 1.0) <= 1.0e-6) {
        interval_s = 1.0;
        moving_mean_samples = 151U;
        return true;
    }
    if (std::abs(interval_s - 15.0) <= 1.0e-6) {
        interval_s = 15.0;
        moving_mean_samples = 11U;
        return true;
    }
    failure = "observed base interval is neither frozen 1 s nor 15 s";
    return false;
}

double phase116HuberCost(double normalized_residual,
                         const libgnss::FGOProcessor::FGOConfig& config,
                         bool& outlier);

TdcpRuntimeReport evaluateTdcpRuntime(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& result,
    const libgnss::FGOProcessor::FGOConfig& config,
    bool enabled) {
    TdcpRuntimeReport report;
    report.enabled = enabled;
    if (!enabled) return report;
    report.factors_built = problem.tdcp_factors.size();
    if (config.use_native_tdcp_frequency_residual_states) {
        if (result.tdcp_frequency_corrections.size()!=problem.tdcp_factors.size()) {
            throw std::invalid_argument("TDCP frequency correction handoff size mismatch");
        }
    } else if (!result.tdcp_frequency_corrections.empty()) {
        throw std::invalid_argument("Unexpected TDCP frequency correction handoff");
    }
    report.factors_inserted = result.diagnostics.tdcp_factors_inserted;
    report.candidate_pairs = problem.diagnostics.tdcp_candidate_pairs;
    report.rejected_gap = problem.diagnostics.tdcp_rejected_gap;
    report.rejected_clock_discontinuity =
        problem.diagnostics.tdcp_rejected_clock_discontinuity;
    report.rejected_missing_previous =
        problem.diagnostics.tdcp_rejected_missing_previous;
    report.rejected_loss_of_lock = problem.diagnostics.tdcp_rejected_loss_of_lock;
    report.rejected_invalid_measurement =
        problem.diagnostics.tdcp_rejected_invalid_measurement;
    report.rejected_code_phase_jump =
        problem.diagnostics.tdcp_rejected_code_phase_jump;
    report.rejected_invalid_weight =
        problem.diagnostics.tdcp_rejected_invalid_weight;

    using CarrierKey = std::pair<libgnss::SatelliteId, libgnss::SignalType>;
    struct ArcState {
        std::size_t last_current_epoch = 0;
        std::size_t length_epochs = 0;
    };
    std::map<CarrierKey, ArcState> active_arcs;
    libgnss::AdjacentResidualMoments<CarrierKey> adjacent_residuals;
    std::vector<std::size_t> arc_lengths;
    arc_lengths.reserve(problem.tdcp_factors.size());
    double sum_squared = 0.0;
    double sum_normalized_squared = 0.0;
    for (const auto& factor : problem.tdcp_factors) {
        const CarrierKey key{factor.satellite, factor.signal};
        auto arc_it = active_arcs.find(key);
        if (arc_it == active_arcs.end() ||
            factor.previous_epoch_index != arc_it->second.last_current_epoch) {
            if (arc_it != active_arcs.end()) {
                arc_lengths.push_back(arc_it->second.length_epochs);
            }
            active_arcs[key] = {factor.current_epoch_index, 2U};
        } else {
            arc_it->second.last_current_epoch = factor.current_epoch_index;
            ++arc_it->second.length_epochs;
        }

        const bool indices_valid =
            factor.previous_epoch_index < result.solution.solutions.size() &&
            factor.current_epoch_index < result.solution.solutions.size();
        if (!indices_valid) {
            adjacent_residuals.breakStream(key);
            ++report.nonfinite_residuals;
            continue;
        }
        const auto& previous =
            result.solution.solutions[factor.previous_epoch_index];
        const auto& current = result.solution.solutions[factor.current_epoch_index];
        const double previous_range =
            (factor.previous_satellite_position_ecef - previous.position_ecef).norm();
        const double current_range =
            (factor.current_satellite_position_ecef - current.position_ecef).norm();
        double residual =
            current_range + libgnss::constants::SPEED_OF_LIGHT * current.receiver_clock_bias -
            previous_range - libgnss::constants::SPEED_OF_LIGHT * previous.receiver_clock_bias -
            factor.delta_carrier_m;
        if (config.use_native_tdcp_frequency_residual_states) {
            const auto index=static_cast<std::size_t>(&factor-problem.tdcp_factors.data());
            const auto& correction=result.tdcp_frequency_corrections[index];
            if (correction.previous_epoch_index!=factor.previous_epoch_index ||
                correction.current_epoch_index!=factor.current_epoch_index ||
                !(correction.satellite==factor.satellite) || correction.signal!=factor.signal ||
                !std::isfinite(correction.alpha_slant_change_m)) {
                throw std::invalid_argument("TDCP frequency correction identity or value mismatch");
            }
            residual-=correction.alpha_slant_change_m;
        }
        if (!std::isfinite(residual) || !(factor.sigma_m > 0.0) ||
            !std::isfinite(factor.sigma_m)) {
            adjacent_residuals.breakStream(key);
            ++report.nonfinite_residuals;
            continue;
        }
        ++report.finite_residuals;
        adjacent_residuals.add(key,factor.previous_epoch_index,factor.current_epoch_index,residual);
        sum_squared += residual * residual;
        const double normalized = residual / factor.sigma_m;
        sum_normalized_squared += normalized * normalized;
        // Read-only reconstruction using the existing ordinary-TDCP Huber
        // resolver. No graph state, residual, or noise model is modified.
        bool outlier = false;
        const double robust_cost = phase116HuberCost(normalized, config, outlier);
        if (std::isfinite(robust_cost) &&
            std::isfinite(0.5 * normalized * normalized)) {
            report.robust_tail_count += outlier ? 1U : 0U;
            report.robust_cost_sum += robust_cost;
            report.quadratic_cost_sum += 0.5 * normalized * normalized;
            auto& signal = report.signal_aggregates[{
                static_cast<int>(factor.satellite.system),
                static_cast<int>(factor.signal)}];
            ++signal.count;
            signal.tail_count += outlier ? 1U : 0U;
            signal.residual_sum_m += residual;
            signal.residual_squared_sum_m2 += residual * residual;
            signal.huber_cost_sum += robust_cost;
            signal.max_abs_residual_m = std::max(
                signal.max_abs_residual_m, std::abs(residual));
        } else {
            ++report.robust_cost_nonfinite_count;
        }
        report.max_abs_residual_m =
            std::max(report.max_abs_residual_m, std::abs(residual));
        if (report.finite_residuals == 1U) report.sigma_m = factor.sigma_m;
    }
    for (const auto& [key, arc] : active_arcs) {
        (void)key;
        arc_lengths.push_back(arc.length_epochs);
    }
    if (!arc_lengths.empty()) {
        std::sort(arc_lengths.begin(), arc_lengths.end());
        report.arc_count = arc_lengths.size();
        report.min_arc_length_epochs = arc_lengths.front();
        report.max_arc_length_epochs = arc_lengths.back();
        const std::size_t middle = arc_lengths.size() / 2U;
        report.median_arc_length_epochs =
            arc_lengths.size() % 2U == 0U
                ? 0.5 * static_cast<double>(arc_lengths[middle - 1U] +
                                             arc_lengths[middle])
                : static_cast<double>(arc_lengths[middle]);
    }
    // The reconstruction above omits joint-ionosphere terms; do not report
    // this monitor as graph residual statistics in that experimental lane.
    if (!config.use_native_joint_ionosphere) {
        const auto correlation=adjacent_residuals.correlation();
        std::cerr << "[native-tdcp-adjacent-residual] pairs=" << adjacent_residuals.pairs()
                  << " correlation=";
        if(correlation) std::cerr << *correlation; else std::cerr << "unavailable";
        std::cerr << " scope=pooled-postfit-metres factors_changed=0\n";
    }
    if (report.finite_residuals > 0U) {
        report.residual_rms_m =
            std::sqrt(sum_squared / static_cast<double>(report.finite_residuals));
        report.normalized_residual_rms = std::sqrt(
            sum_normalized_squared / static_cast<double>(report.finite_residuals));
    }
    return report;
}

double phase116HuberCost(double normalized_residual,
                         const libgnss::FGOProcessor::FGOConfig& config,
                         bool& outlier) {
    outlier = false;
    if (!std::isfinite(normalized_residual)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double threshold = config.tdcp_huber_threshold_sigma;
    if (!libgnss::fgo::resolveOrdinaryTdcpHuberThresholdSigma(
            config, threshold)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (!config.use_robust_loss || !(threshold > 0.0) ||
        std::abs(normalized_residual) <= threshold) {
        return 0.5 * normalized_residual * normalized_residual;
    }
    outlier = true;
    return threshold *
           (std::abs(normalized_residual) - 0.5 * threshold);
}

Phase116CarrierTdcpReport evaluatePhase116CarrierTdcp(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& result,
    const libgnss::FGOProcessor::FGOConfig& config,
    const TdcpRuntimeReport& tdcp_report) {
    Phase116CarrierTdcpReport report;
    report.enabled = config.use_carrier_tdcp_incidence_diagnostic;
    if (!report.enabled) return report;

    report.factors_built = problem.tdcp_factors.size();
    report.factors_inserted = result.diagnostics.tdcp_factors_inserted;
    report.factors_inserted_exact =
        report.factors_inserted == report.factors_built;
    report.sigma_m = tdcp_report.sigma_m;
    report.graph_initial_cost = result.diagnostics.initial_cost;
    report.graph_final_cost = result.diagnostics.final_cost;
    report.standalone_carrier_phase_factors =
        !problem.carrier_phase_factors.empty();
    report.double_difference_factors =
        !problem.double_difference_pseudorange_factors.empty() ||
        !problem.double_difference_carrier_factors.empty();
    report.ambiguity_states = !problem.ambiguity_states.empty();
    report.pdc_state_bridge = !problem.native_pdc_state_seeds.empty();
    report.ordinary_tdcp_only =
        !report.standalone_carrier_phase_factors &&
        !report.double_difference_factors && !report.ambiguity_states &&
        !report.pdc_state_bridge;

    // Start with every signal observed by the builder, including signals
    // whose carrier rows did not produce a pair.  The factor loop below also
    // creates a record defensively if a backend ever returns a signal that
    // was not present in the builder counter map.
    for (const auto& [signal, diagnostics] :
         problem.diagnostics.tdcp_signal_diagnostics) {
        auto& destination = report.signals[signal];
        destination.signal = baseTelemetrySignalName(signal);
        destination.frequency_band = baseTelemetryFrequencyBand(signal);
        destination.carrier_rows_seen = diagnostics.carrier_rows_seen;
        destination.carrier_phase_rows = diagnostics.carrier_phase_rows;
        destination.retained_carrier_rows = diagnostics.retained_carrier_rows;
        destination.missing_wavelength = diagnostics.missing_wavelength;
        destination.nonfinite_measurements = diagnostics.nonfinite_measurements;
        destination.candidate_pairs = diagnostics.candidate_pairs;
        destination.accepted_pairs = diagnostics.accepted_pairs;
        destination.rejected_gap = diagnostics.rejected_gap;
        destination.rejected_clock_discontinuity =
            diagnostics.rejected_clock_discontinuity;
        destination.rejected_missing_previous =
            diagnostics.rejected_missing_previous;
        destination.rejected_loss_of_lock = diagnostics.rejected_loss_of_lock;
        destination.rejected_nonfinite = diagnostics.rejected_nonfinite;
        destination.rejected_code_phase_jump =
            diagnostics.rejected_code_phase_jump;
        destination.rejected_invalid_weight =
            diagnostics.rejected_invalid_weight;
        destination.initial_robust_cost = 0.0;
        destination.final_robust_cost = 0.0;
        destination.initial_unwhitened_cost = 0.0;
        destination.final_unwhitened_cost = 0.0;
        destination.initial_whitened_cost = 0.0;
        destination.final_whitened_cost = 0.0;
    }

    const bool have_initial_epoch_c0 =
        config.use_native_source_clock_c0d_epoch_vector_parity &&
        problem.native_source_clock_c0d_gnss_first_c_handoff_m.size() ==
            problem.epochs.size();
    const bool have_final_epoch_c0 =
        config.use_native_source_clock_c0d_epoch_vector_parity &&
        result.epoch_clock_bias_components_m.size() ==
            result.solution.solutions.size();
    const auto initialClockMeters = [&](std::size_t index) {
        if (have_initial_epoch_c0 && index <
                problem.native_source_clock_c0d_gnss_first_c_handoff_m.size()) {
            return problem.native_source_clock_c0d_gnss_first_c_handoff_m[index][0];
        }
        if (index >= problem.epochs.size()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const auto& epoch = problem.epochs[index];
        return epoch.receiver_clock_bias_is_meters
                   ? epoch.receiver_clock_bias_m
                   : epoch.receiver_clock_bias_m *
                         libgnss::constants::SPEED_OF_LIGHT;
    };
    const auto finalClockMeters = [&](std::size_t index) {
        if (have_final_epoch_c0 && index <
                result.epoch_clock_bias_components_m.size()) {
            return result.epoch_clock_bias_components_m[index][0];
        }
        if (index >= result.solution.solutions.size()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double clock_seconds =
            result.solution.solutions[index].receiver_clock_bias;
        return std::isfinite(clock_seconds)
                   ? clock_seconds * libgnss::constants::SPEED_OF_LIGHT
                   : std::numeric_limits<double>::quiet_NaN();
    };
    const auto residualAt = [&](const libgnss::FGOProcessor::TimeDifferencedCarrierFactor& factor,
                                bool initial) {
        const std::size_t previous = factor.previous_epoch_index;
        const std::size_t current = factor.current_epoch_index;
        libgnss::Vector3d previous_position = libgnss::Vector3d::Zero();
        libgnss::Vector3d current_position = libgnss::Vector3d::Zero();
        double previous_clock = std::numeric_limits<double>::quiet_NaN();
        double current_clock = std::numeric_limits<double>::quiet_NaN();
        if (initial) {
            if (previous >= problem.epochs.size() ||
                current >= problem.epochs.size()) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            previous_position = problem.epochs[previous].position_ecef;
            current_position = problem.epochs[current].position_ecef;
            previous_clock = initialClockMeters(previous);
            current_clock = initialClockMeters(current);
        } else {
            if (previous >= result.solution.solutions.size() ||
                current >= result.solution.solutions.size()) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            previous_position =
                result.solution.solutions[previous].position_ecef;
            current_position = result.solution.solutions[current].position_ecef;
            previous_clock = finalClockMeters(previous);
            current_clock = finalClockMeters(current);
        }
        if (!previous_position.allFinite() || !current_position.allFinite() ||
            !factor.previous_satellite_position_ecef.allFinite() ||
            !factor.current_satellite_position_ecef.allFinite() ||
            !std::isfinite(previous_clock) || !std::isfinite(current_clock) ||
            !std::isfinite(factor.delta_carrier_m)) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double previous_range =
            (factor.previous_satellite_position_ecef - previous_position).norm();
        const double current_range =
            (factor.current_satellite_position_ecef - current_position).norm();
        if (!(previous_range > 0.0) || !(current_range > 0.0) ||
            !std::isfinite(previous_range) || !std::isfinite(current_range)) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return current_range + current_clock - previous_range - previous_clock -
               factor.delta_carrier_m;
    };

    const auto addCost = [&](Phase116CarrierTdcpSignalReport& destination,
                             double residual, bool initial) {
        std::size_t& count = initial ? destination.initial_residual_count
                                     : destination.final_residual_count;
        std::size_t& nonfinite =
            initial ? destination.initial_nonfinite_residual_count
                    : destination.final_nonfinite_residual_count;
        double& robust_cost = initial ? destination.initial_robust_cost
                                      : destination.final_robust_cost;
        double& unwhitened_cost = initial ? destination.initial_unwhitened_cost
                                          : destination.final_unwhitened_cost;
        double& whitened_cost = initial ? destination.initial_whitened_cost
                                        : destination.final_whitened_cost;
        std::size_t& outlier_count =
            initial ? destination.initial_robust_outlier_count
                    : destination.final_robust_outlier_count;
        if (!std::isfinite(residual) || !(destination.sigma_m > 0.0) ||
            !std::isfinite(destination.sigma_m)) {
            ++nonfinite;
            return;
        }
        const double normalized = residual / destination.sigma_m;
        bool outlier = false;
        const double robust = phase116HuberCost(normalized, config, outlier);
        if (!std::isfinite(normalized) || !std::isfinite(robust)) {
            ++nonfinite;
            return;
        }
        ++count;
        if (outlier) ++outlier_count;
        unwhitened_cost += residual * residual;
        whitened_cost += normalized * normalized;
        robust_cost += robust;
    };

    std::map<libgnss::SignalType, std::set<std::size_t>> connected_epochs;
    for (const auto& factor : problem.tdcp_factors) {
        auto& destination = report.signals[factor.signal];
        if (destination.signal.empty()) {
            destination.signal = baseTelemetrySignalName(factor.signal);
            destination.frequency_band = baseTelemetryFrequencyBand(factor.signal);
            destination.initial_robust_cost = 0.0;
            destination.final_robust_cost = 0.0;
            destination.initial_unwhitened_cost = 0.0;
            destination.final_unwhitened_cost = 0.0;
            destination.initial_whitened_cost = 0.0;
            destination.final_whitened_cost = 0.0;
        }
        if (destination.initial_residual_count == 0U &&
            std::isfinite(factor.sigma_m) && factor.sigma_m > 0.0) {
            destination.sigma_m = factor.sigma_m;
        }
        connected_epochs[factor.signal].insert(factor.previous_epoch_index);
        connected_epochs[factor.signal].insert(factor.current_epoch_index);
        destination.connected_keys.insert(
            "x" + std::to_string(factor.previous_epoch_index));
        destination.connected_keys.insert(
            "c" + std::to_string(factor.previous_epoch_index));
        destination.connected_keys.insert(
            "x" + std::to_string(factor.current_epoch_index));
        destination.connected_keys.insert(
            "c" + std::to_string(factor.current_epoch_index));
        addCost(destination, residualAt(factor, true), true);
        addCost(destination, residualAt(factor, false), false);
    }
    for (auto& [signal, destination] : report.signals) {
        destination.connected_epoch_count = connected_epochs[signal].size();
        if (destination.initial_residual_count == 0U) {
            destination.initial_robust_cost =
                std::numeric_limits<double>::quiet_NaN();
            destination.initial_unwhitened_cost =
                std::numeric_limits<double>::quiet_NaN();
            destination.initial_whitened_cost =
                std::numeric_limits<double>::quiet_NaN();
        }
        if (destination.final_residual_count == 0U) {
            destination.final_robust_cost =
                std::numeric_limits<double>::quiet_NaN();
            destination.final_unwhitened_cost =
                std::numeric_limits<double>::quiet_NaN();
            destination.final_whitened_cost =
                std::numeric_limits<double>::quiet_NaN();
        }
    }
    return report;
}

bool populateNativePdcStateBridge(
    libgnss::FGOProcessor::FGOProblem& problem,
    NativePdcBridgeReport& report) {
    report.enabled = true;
    report.epochs = problem.epochs.size();
    report.pseudorange_rows = problem.pseudorange_factors.size();
    report.doppler_rows = problem.undifferenced_doppler_factors.size();
    if (problem.epochs.empty() || problem.pseudorange_factors.empty()) {
        report.failure = "native PDC bridge has no pseudorange rows";
        return false;
    }

    // buildPseudorangeProblem historically carries a fresh SPP clock in the
    // receiver_clock_bias_m field as seconds, while all FGO state equations
    // use metres.  The marker makes this candidate-only normalization
    // explicit and prevents magnitude-based unit guesses.
    for (auto& epoch : problem.epochs) {
        if (!epoch.receiver_clock_bias_is_meters) {
            if (!std::isfinite(epoch.receiver_clock_bias_m)) {
                report.failure = "non-finite SPP clock seed";
                return false;
            }
            epoch.receiver_clock_bias_m *= libgnss::constants::SPEED_OF_LIGHT;
            epoch.receiver_clock_bias_is_meters = true;
        }
    }

    std::vector<libgnss::pdc_state_bridge::EpochInput> epochs;
    std::vector<libgnss::pdc_state_bridge::PositionSeedVelocity>
        seed_velocities;
    epochs.reserve(problem.epochs.size());
    seed_velocities.reserve(problem.epochs.size());
    for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
         ++epoch_index) {
        const auto& source = problem.epochs[epoch_index];
        if (!source.position_ecef.allFinite() ||
            !std::isfinite(source.receiver_clock_bias_m)) {
            report.failure = "non-finite PDC epoch seed";
            return false;
        }
        libgnss::Vector3d seed_velocity_ecef_mps =
            libgnss::Vector3d::Zero();
        double seed_clock_rate_mps = 0.0;
        bool has_seed_velocity = false;
        bool has_seed_clock_rate = false;
        if (epoch_index < problem.doppler_velocity_wls_estimates.size()) {
            const auto& wls = problem.doppler_velocity_wls_estimates[epoch_index];
            if (wls.valid && wls.velocity_ecef_mps.allFinite() &&
                std::isfinite(wls.clock_rate_mps)) {
                seed_velocity_ecef_mps = wls.velocity_ecef_mps;
                seed_clock_rate_mps = wls.clock_rate_mps;
                has_seed_velocity = true;
                has_seed_clock_rate = true;
            }
        }
        epochs.push_back({source.time,
                          source.position_ecef,
                          source.receiver_clock_bias_m,
                          epoch_index < problem.clock_jumps.size()
                              ? problem.clock_jumps[epoch_index]
                              : false,
                          seed_velocity_ecef_mps,
                          seed_clock_rate_mps,
                          has_seed_velocity,
                          has_seed_clock_rate});
        seed_velocities.push_back({has_seed_velocity, seed_velocity_ecef_mps});
    }

    const auto integrated_seeds = libgnss::pdc_state_bridge::integratePositionSeeds(
        epochs, seed_velocities, problem.clock_jumps, 1.5);
    if (!integrated_seeds.valid ||
        integrated_seeds.positions.size() != epochs.size()) {
        report.failure = "invalid integrated raw WLS position seed";
        return false;
    }
    report.integrated_seed_anchor_index = integrated_seeds.anchor_index;
    report.integrated_seed_epochs = integrated_seeds.integrated_epochs;
    report.integrated_seed_held_velocity_epochs =
        integrated_seeds.held_velocity_epochs;
    report.integrated_seed_spp_fallback_epochs =
        integrated_seeds.per_epoch_spp_fallback_epochs;
    report.integrated_seed_reset_intervals = integrated_seeds.reset_intervals;
    report.integrated_seed_max_step_speed_mps =
        integrated_seeds.max_integrated_step_speed_mps;
    report.integrated_seed_max_displacement_from_spp_m =
        integrated_seeds.max_displacement_from_spp_m;
    std::vector<libgnss::pdc_state_bridge::EpochInput> bridge_epochs = epochs;
    for (std::size_t epoch_index = 0; epoch_index < bridge_epochs.size();
         ++epoch_index) {
        bridge_epochs[epoch_index].seed_position_ecef =
            integrated_seeds.positions[epoch_index];
    }

    std::vector<libgnss::pdc_state_bridge::PseudorangeRow> pseudorange_rows;
    pseudorange_rows.reserve(problem.pseudorange_factors.size());
    for (const auto& factor : problem.pseudorange_factors) {
        if (factor.epoch_index >= epochs.size()) {
            report.failure = "PDC row epoch index out of range";
            return false;
        }
        pseudorange_rows.push_back({factor.epoch_index,
                                    factor.satellite,
                                    factor.clock_group,
                                    factor.satellite_position_ecef,
                                    factor.corrected_pseudorange_m,
                                    factor.sigma_m});
    }
    std::vector<libgnss::pdc_state_bridge::DopplerRow> doppler_rows;
    doppler_rows.reserve(problem.undifferenced_doppler_factors.size());
    for (const auto& factor : problem.undifferenced_doppler_factors) {
        if (factor.epoch_index >= epochs.size()) {
            report.failure = "PDC Doppler row epoch index out of range";
            return false;
        }
        doppler_rows.push_back({factor.epoch_index,
                                factor.los,
                                factor.residual_mps,
                                factor.sigma_mps});
    }

    // Capture the bridge input contract before solving.  These are diagnostics
    // only: they do not gate, reweight, or select a state.  In particular, the
    // first corrected-Doppler epoch is intentionally empty because receiver
    // clock drift is defined by an adjacent epoch difference in the native
    // contract.  Keeping that distinction visible prevents a harmless first
    // row count of zero from being mistaken for a global Doppler unit failure.
    std::vector<double> absolute_doppler_residuals;
    absolute_doppler_residuals.reserve(doppler_rows.size());
    double doppler_residual_square_sum = 0.0;
    std::size_t doppler_residual_count = 0;
    for (std::size_t epoch_index = 0; epoch_index < epochs.size(); ++epoch_index) {
        if (epoch_index > 0U) {
            const double dt = epochs[epoch_index].time -
                              epochs[epoch_index - 1U].time;
            if (std::isfinite(dt) && dt > 0.0) {
                report.min_epoch_dt_s = std::min(report.min_epoch_dt_s, dt);
                report.max_epoch_dt_s = std::max(report.max_epoch_dt_s, dt);
            } else {
                ++report.invalid_or_large_epoch_intervals;
            }
            const double seed_speed =
                (epochs[epoch_index].seed_position_ecef -
                 epochs[epoch_index - 1U].seed_position_ecef).norm() /
                (std::isfinite(dt) && dt > 0.0 ? dt : 1.0);
            if (std::isfinite(seed_speed)) {
                report.seed_position_step_max_mps =
                    std::max(report.seed_position_step_max_mps, seed_speed);
            }
        }
        if (std::isfinite(epochs[epoch_index].seed_clock_bias_m)) {
            report.initial_pseudorange_max_m = std::max(
                report.initial_pseudorange_max_m,
                std::abs(epochs[epoch_index].seed_clock_bias_m));
        }
        if (std::isfinite(problem.epochs[epoch_index].receiver_clock_drift_mps)) {
            ++report.finite_raw_clock_drift_epochs;
        }
    }
    if (!std::isfinite(report.min_epoch_dt_s)) {
        report.min_epoch_dt_s = 0.0;
    }
    std::vector<double> initial_pseudorange_residuals;
    initial_pseudorange_residuals.reserve(pseudorange_rows.size());
    for (const auto& row : pseudorange_rows) {
        if (row.epoch_index >= epochs.size()) continue;
        const double residual =
            (row.satellite_position_ecef - epochs[row.epoch_index].seed_position_ecef).norm() +
            epochs[row.epoch_index].seed_clock_bias_m - row.corrected_pseudorange_m;
        if (std::isfinite(residual)) {
            initial_pseudorange_residuals.push_back(std::abs(residual));
        }
    }
    if (!initial_pseudorange_residuals.empty()) {
        std::sort(initial_pseudorange_residuals.begin(),
                  initial_pseudorange_residuals.end());
        double sum = 0.0;
        for (const double value : initial_pseudorange_residuals) {
            sum += value * value;
        }
        report.initial_pseudorange_rms_m =
            std::sqrt(sum / static_cast<double>(initial_pseudorange_residuals.size()));
        report.initial_pseudorange_max_m = initial_pseudorange_residuals.back();
    }
    for (const auto& row : doppler_rows) {
        if (row.epoch_index == 0U) {
            ++report.first_epoch_doppler_rows;
        } else {
            ++report.later_epoch_doppler_rows;
        }
        if (row.epoch_index < epochs.size()) {
            // The row's LOS and residual are already the shared corrected
            // contract used by the standalone Doppler WLS and FGO factor.
            if (std::isfinite(row.residual_mps) &&
                std::isfinite(row.sigma_mps) && row.sigma_mps > 0.0) {
                const double absolute = std::abs(row.residual_mps);
                absolute_doppler_residuals.push_back(absolute);
                doppler_residual_square_sum += row.residual_mps * row.residual_mps;
                ++doppler_residual_count;
                report.doppler_sigma_min_mps =
                    std::min(report.doppler_sigma_min_mps, row.sigma_mps);
                report.doppler_sigma_max_mps =
                    std::max(report.doppler_sigma_max_mps, row.sigma_mps);
            }
        }
    }
    report.epochs_with_doppler = 0U;
    for (std::size_t epoch_index = 0; epoch_index < epochs.size(); ++epoch_index) {
        if (std::any_of(doppler_rows.begin(), doppler_rows.end(),
                        [epoch_index](const auto& row) {
                            return row.epoch_index == epoch_index;
                        })) {
            ++report.epochs_with_doppler;
        }
    }
    if (doppler_residual_count > 0U) {
        report.doppler_residual_rms_mps = std::sqrt(
            doppler_residual_square_sum /
            static_cast<double>(doppler_residual_count));
        std::sort(absolute_doppler_residuals.begin(),
                  absolute_doppler_residuals.end());
        const std::size_t middle = absolute_doppler_residuals.size() / 2U;
        report.doppler_abs_p50_mps =
            absolute_doppler_residuals.size() % 2U == 0U
                ? 0.5 * (absolute_doppler_residuals[middle - 1U] +
                         absolute_doppler_residuals[middle])
                : absolute_doppler_residuals[middle];
        report.doppler_abs_max_mps = absolute_doppler_residuals.back();
    }

    libgnss::pdc_state_bridge::Options bridge_options;
    // Match the dedicated native PDC recipe. These values are physical/config
    // defaults fixed before truth; this bridge does not learn from the route.
    bridge_options.max_iterations = 1000;
    bridge_options.min_pseudorange_rows = 4;
    bridge_options.min_doppler_rows = 4;
    bridge_options.pseudorange_huber_threshold_sigma = 1.234;
    bridge_options.doppler_huber_threshold_sigma = 1.234;
    bridge_options.position_prior_sigma_m = 1000.0;
    bridge_options.clock_prior_sigma_m = 1.0e6;
    bridge_options.velocity_prior_sigma_mps = 1000.0;
    bridge_options.clock_rate_prior_sigma_mps = 1000.0;
    bridge_options.motion_sigma_m = 0.1;
    bridge_options.clock_motion_sigma_m = 0.1;
    bridge_options.clock_jump_sigma_m = 1.0e6;
    bridge_options.inter_system_clock_motion_sigma_m = 1.0e-6;
    bridge_options.clock_rate_between_sigma_mps = 0.1;
    bridge_options.max_velocity_mps = 70.0;
    bridge_options.max_clock_rate_mps = 2000.0;
    bridge_options.max_normalized_rms = 25.0;
    bridge_options.max_position_norm_m = 1.0e7;

    // The bridge is intentionally diagnosed against the already-built
    // raw-observable Doppler WLS initializer.  This does not reweight or
    // select measurements; it records the exact finite seed that is handed
    // to the bridge when available, so a later quality failure remains
    // distinguishable from a Doppler-unit failure.
    for (std::size_t epoch_index = 0;
         epoch_index < problem.doppler_velocity_wls_estimates.size();
         ++epoch_index) {
        const auto& estimate = problem.doppler_velocity_wls_estimates[epoch_index];
        if (!estimate.valid) {
            ++report.wls_rejected_epochs;
            if (epoch_index == 0U) report.wls_first_reason = estimate.reason;
            continue;
        }
        ++report.wls_valid_epochs;
        if (estimate.propagated) ++report.wls_propagated_epochs;
        if (estimate.velocity_ecef_mps.allFinite()) {
            report.wls_max_velocity_norm_mps = std::max(
                report.wls_max_velocity_norm_mps,
                estimate.velocity_ecef_mps.norm());
        }
        if (std::isfinite(estimate.clock_rate_mps)) {
            report.wls_max_clock_rate_abs_mps = std::max(
                report.wls_max_clock_rate_abs_mps,
                std::abs(estimate.clock_rate_mps));
        }
        if (std::isfinite(estimate.normalized_rms)) {
            report.wls_max_normalized_rms = std::max(
                report.wls_max_normalized_rms, estimate.normalized_rms);
        }
        if (epoch_index == 0U) {
            report.wls_first_velocity_norm_mps =
                estimate.velocity_ecef_mps.norm();
            report.wls_first_clock_rate_mps = estimate.clock_rate_mps;
            report.wls_first_reason = estimate.reason;
        }
    }

    const auto solve = libgnss::pdc_state_bridge::solve(
        bridge_epochs, pseudorange_rows, doppler_rows, bridge_options);
    report.motion_intervals = solve.motion_intervals;
    report.iterations = solve.iterations;
    report.initial_cost = solve.initial_cost;
    report.final_cost = solve.final_cost;
    report.max_velocity_norm_mps = solve.max_velocity_norm_mps;
    report.max_clock_rate_abs_mps = solve.max_clock_rate_abs_mps;
    for (const auto& estimate : solve.epochs) {
        if (estimate.state.velocity_ecef_mps.allFinite()) {
            const double velocity_norm = estimate.state.velocity_ecef_mps.norm();
            if (std::isfinite(velocity_norm)) {
                report.all_state_max_velocity_norm_mps = std::max(
                    report.all_state_max_velocity_norm_mps, velocity_norm);
                if (velocity_norm > bridge_options.max_velocity_mps) {
                    ++report.all_state_velocity_over_bound;
                }
            }
        }
        if (std::isfinite(estimate.state.clock_rate_mps)) {
            const double clock_rate_abs = std::abs(estimate.state.clock_rate_mps);
            report.all_state_max_clock_rate_abs_mps = std::max(
                report.all_state_max_clock_rate_abs_mps, clock_rate_abs);
            if (clock_rate_abs > bridge_options.max_clock_rate_mps) {
                ++report.all_state_clock_rate_over_bound;
            }
        }
        if (estimate.valid) {
            ++report.valid_position_states;
            ++report.valid_velocity_states;
        } else {
            ++report.rejected_position_states;
        }
        if (std::isfinite(estimate.normalized_rms)) {
            report.max_normalized_rms =
                std::max(report.max_normalized_rms, estimate.normalized_rms);
        }
    }
    problem.native_pdc_state_seeds.clear();
    problem.native_pdc_state_seeds.reserve(solve.epochs.size());
    for (std::size_t epoch_index = 0; epoch_index < solve.epochs.size();
         ++epoch_index) {
        const auto& estimate = solve.epochs[epoch_index];
        if (!estimate.valid) continue;
        libgnss::FGOProcessor::NativePdcStateSeed seed;
        seed.epoch_index = epoch_index;
        seed.position_ecef = estimate.state.position_ecef;
        seed.velocity_ecef_mps = estimate.state.velocity_ecef_mps;
        seed.clock_bias_m = estimate.state.clock_bias_m;
        seed.clock_rate_mps = estimate.state.clock_rate_mps;
        seed.pseudorange_rows = estimate.pseudorange_rows;
        seed.doppler_rows = estimate.doppler_rows;
        seed.normalized_pseudorange_rms = estimate.normalized_rms;
        seed.has_position = seed.position_ecef.allFinite();
        seed.has_velocity = seed.velocity_ecef_mps.allFinite();
        seed.has_clock = std::isfinite(seed.clock_bias_m[0]);
        seed.has_clock_rate = std::isfinite(seed.clock_rate_mps);
        if (seed.has_position || seed.has_velocity) {
            problem.native_pdc_state_seeds.push_back(seed);
        }
    }
    report.state_seeds = problem.native_pdc_state_seeds.size();
    report.ok = solve.valid && report.state_seeds > 0;
    if (!report.ok) {
        std::ostringstream failure;
        failure << (solve.reason.empty() ? "native PDC state solve failed"
                                         : solve.reason)
                << "; converged=" << (solve.converged ? "true" : "false")
                << "; iterations=" << solve.iterations
                << "; valid_epochs=" << solve.valid_epochs
                << "; final_cost=" << solve.final_cost
                << "; max_velocity_mps=" << solve.max_velocity_norm_mps
                << "; max_clock_rate_mps=" << solve.max_clock_rate_abs_mps
                << "; max_normalized_rms=" << report.max_normalized_rms
                << "; total_d_rows=" << report.doppler_rows
                << "; first_d_rows=" << report.first_epoch_doppler_rows
                << "; later_d_rows=" << report.later_epoch_doppler_rows
                << "; epochs_with_d=" << report.epochs_with_doppler
                << "; raw_clock_drift_epochs="
                << report.finite_raw_clock_drift_epochs
                << "; dt_s=" << report.min_epoch_dt_s << ".."
                << report.max_epoch_dt_s
                << "; d_abs_p50_mps=" << report.doppler_abs_p50_mps
                << "; d_abs_max_mps=" << report.doppler_abs_max_mps
                << "; d_sigma_mps=" << report.doppler_sigma_min_mps << ".."
                << report.doppler_sigma_max_mps
                << "; p_seed_rms_m=" << report.initial_pseudorange_rms_m
                << "; p_seed_max_m=" << report.initial_pseudorange_max_m
                << "; seed_step_max_mps=" << report.seed_position_step_max_mps
                << "; state_vel_max_mps="
                << report.all_state_max_velocity_norm_mps
                << "; state_clock_rate_max_mps="
                << report.all_state_max_clock_rate_abs_mps
                << "; state_vel_over_bound="
                << report.all_state_velocity_over_bound
                << "; state_clock_over_bound="
                << report.all_state_clock_rate_over_bound
                << "; wls_valid_epochs=" << report.wls_valid_epochs
                << "; wls_rejected_epochs=" << report.wls_rejected_epochs
                << "; wls_propagated_epochs=" << report.wls_propagated_epochs
                << "; wls_max_velocity_mps="
                << report.wls_max_velocity_norm_mps
                << "; wls_max_clock_rate_mps="
                << report.wls_max_clock_rate_abs_mps
                << "; wls_max_normalized_rms="
                << report.wls_max_normalized_rms
                << "; wls_first_velocity_mps="
                << report.wls_first_velocity_norm_mps
                << "; wls_first_clock_rate_mps="
                << report.wls_first_clock_rate_mps
                << "; wls_first_reason=" << report.wls_first_reason
                << "; integrated_seed_anchor="
                << report.integrated_seed_anchor_index
                << "; integrated_seed_epochs="
                << report.integrated_seed_epochs
                << "; integrated_seed_holds="
                << report.integrated_seed_held_velocity_epochs
                << "; integrated_seed_spp_fallbacks="
                << report.integrated_seed_spp_fallback_epochs
                << "; integrated_seed_resets="
                << report.integrated_seed_reset_intervals
                << "; integrated_seed_step_max_mps="
                << report.integrated_seed_max_step_speed_mps
                << "; integrated_seed_displacement_max_m="
                << report.integrated_seed_max_displacement_from_spp_m;
        if (!solve.epochs.empty()) {
            const auto& first = solve.epochs.front();
            double initial_sum_squared = 0.0;
            double initial_max_abs = 0.0;
            int initial_count = 0;
            for (const auto& row : pseudorange_rows) {
                if (row.epoch_index != 0) continue;
                const double residual =
                    (row.satellite_position_ecef - epochs.front().seed_position_ecef).norm() +
                    epochs.front().seed_clock_bias_m - row.corrected_pseudorange_m;
                if (std::isfinite(residual)) {
                    initial_sum_squared += residual * residual;
                    initial_max_abs = std::max(initial_max_abs, std::abs(residual));
                    ++initial_count;
                }
            }
            failure << "; first_reason=" << first.reason
                    << "; first_p_rows=" << first.pseudorange_rows
                    << "; first_d_rows=" << first.doppler_rows
                    << "; first_pos_norm=" << first.state.position_ecef.norm()
                    << "; first_pos_seed_norm="
                    << bridge_epochs.front().seed_position_ecef.norm()
                    << "; first_pos_spp_seed_norm="
                    << epochs.front().seed_position_ecef.norm()
                    << "; first_clock_m=" << first.state.clock_bias_m[0]
                    << "; first_vel_norm="
                    << first.state.velocity_ecef_mps.norm()
                    << "; seed_clock_m=" << epochs.front().seed_clock_bias_m
                    << "; first_initial_p_rms_m="
                    << (initial_count > 0
                            ? std::sqrt(initial_sum_squared /
                                       static_cast<double>(initial_count))
                            : std::numeric_limits<double>::quiet_NaN())
                    << "; first_initial_p_max_m=" << initial_max_abs;
        }
        report.failure = failure.str();
    }
    return report.ok;
}

bool buildImuInput(const std::string& path,
                   libgnss::FGOProcessor::FGOProblem& problem,
                   ImuBuildReport& report,
                   bool android_raw = false,
                   const std::vector<libgnss::AndroidGnssTimeAnchor>& gnss_time_anchors = {},
                   const std::vector<libgnss::Vector3d>* gnss_first_velocities_enu = nullptr,
                   const libgnss::AndroidGnssUtcGpsMapping* utc_gps_mapping = nullptr,
                   bool direct_wls_ephemeral_seed = false,
                   bool phase194_source_utc_fallback_imu_noise = false,
                   bool phase197_source_utc_fallback_imu_offset = false,
                   bool epoch_heading_attitude_seeds = false,
                   bool stationary_gyro_initializer = false) {
    if (problem.epochs.size() < 2) {
        report.failure = "fewer than two GNSS epochs";
        return false;
    }

    libgnss::ImuSeries series;
    report.android_raw = android_raw;
    if (android_raw) {
        libgnss::AndroidImuCsvConfig android_config;
        android_config.require_gnss_elapsed_anchor = true;
        android_config.allow_utc_wall_clock_fallback = utc_gps_mapping != nullptr;
        android_config.apply_utc_wall_clock_fallback_offset =
            phase197_source_utc_fallback_imu_offset;
        android_config.utc_wall_clock_fallback_offset_ms =
            phase197_source_utc_fallback_imu_offset
                ? libgnss::native_utc_fallback_imu_offset::kSourceUtcWallClockOffsetMs
                : 0;
        android_config.imu_sync_coefficient = kUpstreamImuSyncCoefficient;
        report.android_load = libgnss::loadAndroidImuCsv(
            path, series, android_config, gnss_time_anchors, utc_gps_mapping);
        const auto offset_selection =
            libgnss::native_utc_fallback_imu_offset::select(
                phase197_source_utc_fallback_imu_offset,
                report.android_load.utc_wall_clock_fallback_applied);
        report.phase197_source_utc_fallback_imu_offset_requested =
            offset_selection.requested;
        report.phase197_source_utc_fallback_imu_offset_applied =
            report.android_load.utc_wall_clock_fallback_offset_applied;
        report.phase197_source_utc_fallback_imu_offset_configured_ms =
            report.android_load.utc_wall_clock_fallback_offset_ms;
        report.phase197_source_utc_fallback_imu_offset_effective_ms =
            report.android_load.utc_wall_clock_fallback_effective_offset_ms;
        report.imu_utc_fallback_offset_source = offset_selection.source;
        if (!report.android_load.ok || series.isEmpty()) {
            report.failure = report.android_load.error.empty()
                                 ? "empty Android IMU series"
                                 : report.android_load.error;
            return false;
        }
    } else {
        const libgnss::ImuCsvLoadResult processed_load =
            libgnss::loadImuCsv(path, series);
        if (!processed_load.ok || series.isEmpty()) {
            report.failure = processed_load.error.empty() ? "empty IMU series"
                                                           : processed_load.error;
            return false;
        }
    }
    series.sortByTime();
    const Eigen::Matrix3d mounting = tarozMountingRotation();
    std::vector<libgnss::ImuSample> samples;
    samples.reserve(series.samples.size());
    for (auto sample : series.samples) {
        const Eigen::Vector3d accel = mounting * sample.accel_raw;
        const Eigen::Vector3d gyro = mounting * sample.gyro_raw_radps;
        if (!accel.allFinite() || !gyro.allFinite()) {
            report.failure = "non-finite IMU sample";
            return false;
        }
        sample.accel_raw = accel;
        sample.gyro_raw_radps = gyro;
        samples.push_back(sample);
    }
    if (samples.size() < kStationarySamples) {
        report.failure = "IMU stream shorter than frozen leveling window";
        return false;
    }
    report.loaded_samples = samples.size();

    const std::size_t stationary_count = std::min(kStationarySamples, samples.size());
    std::vector<libgnss::ImuSample> stationary(samples.begin(),
                                                samples.begin() + stationary_count);
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    std::vector<double> norms;
    norms.reserve(stationary.size());
    for (const auto& sample : stationary) {
        accel_sum += sample.accel_raw;
        norms.push_back(sample.accel_raw.norm());
    }
    const double n = static_cast<double>(stationary.size());
    const Eigen::Vector3d accel_mean = accel_sum / n;
    const double mean_norm = accel_mean.norm();
    double variance = 0.0;
    for (double norm : norms) variance += (norm - mean_norm) * (norm - mean_norm);
    const double norm_std = std::sqrt(variance / n);
    report.stationary_samples = stationary.size();
    report.gravity_mean_norm = mean_norm;
    report.gravity_norm_std = norm_std;
    if (!std::isfinite(mean_norm) || !std::isfinite(norm_std) ||
        mean_norm < kGravityNormMin || mean_norm > kGravityNormMax ||
        norm_std > kGravityNormStdMax) {
        report.failure = "leveling window is not stationary/low-dynamics under frozen gravity gate";
        return false;
    }

    const libgnss::Vector3d origin_ecef = problem.epochs.front().position_ecef;
    double lat = 0.0, lon = 0.0, height = 0.0;
    libgnss::ecef2geodetic(origin_ecef, lat, lon, height);
    if (!std::isfinite(lat) || !std::isfinite(lon) || !origin_ecef.allFinite()) {
        report.failure = "invalid GNSS nav origin";
        return false;
    }

    const libgnss::NominalState aligned = libgnss::fusion_initialization::alignStatic(
        stationary, libgnss::Vector3d::Zero(), kGravity);
    libgnss::FusionState state;
    state.nominal = aligned;
    state.covariance.setIdentity();
    libgnss::Vector3d initial_velocity = libgnss::Vector3d::Zero();
    if (android_raw) {
        // The Android pass obtains attitude from the same-run velocity
        // sequence (`vel2rpy.m`) before constructing the IMU graph.  Phase114
        // supplies the direct-WLS sequence here; it is only an attitude seed
        // and is not used to replace the exact ECEF velocity values retained
        // in FGOProblem for the main graph.
        if (gnss_first_velocities_enu == nullptr ||
            gnss_first_velocities_enu->size() != problem.epochs.size()) {
            report.failure = direct_wls_ephemeral_seed
                                 ? "Android direct-WLS mode requires the exact velocity sequence"
                                 : "Android upstream mode requires GNSS-first velocity sequence";
            return false;
        }
        libgnss::fusion_initialization::VelocityHeadingConfig heading_config;
        heading_config.nearest_fill_interior = epoch_heading_attitude_seeds;
        const auto velocity_heading = libgnss::fusion_initialization::velocityToRpy(
            *gnss_first_velocities_enu, heading_config);
        if (!velocity_heading.ok || velocity_heading.rpy_rad.empty() ||
            velocity_heading.smoothed_velocity_enu.size() != problem.epochs.size()) {
            report.failure = velocity_heading.error.empty()
                                 ? "GNSS-first velocity heading initialization failed"
                                 : velocity_heading.error;
            return false;
        }
        report.heading_initialization_mode = direct_wls_ephemeral_seed
                                                 ? "direct-wls-vel2rpy"
                                                 : "upstream-vel2rpy";
        report.velocity_heading_low_speed_count = velocity_heading.low_speed_count;
        report.velocity_heading_linear_fill_count = velocity_heading.linear_fill_count;
        report.velocity_heading_nearest_fill_count = velocity_heading.nearest_fill_count;
        state.nominal.attitude_body_to_enu = Eigen::Quaterniond(
            rpyToBodyToNav(velocity_heading.rpy_rad.front()));
        initial_velocity = velocity_heading.smoothed_velocity_enu.front();
        if (epoch_heading_attitude_seeds) {
            problem.imu.epoch_heading_attitudes_body_to_nav.clear();
            problem.imu.epoch_heading_attitude_times.clear();
            for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
                problem.imu.epoch_heading_attitudes_body_to_nav.push_back(
                    rpyToBodyToNav(velocity_heading.rpy_rad.at(i)));
                problem.imu.epoch_heading_attitude_times.push_back(problem.epochs[i].time);
            }
        }
    } else {
        libgnss::Vector3d previous_velocity = libgnss::Vector3d::Zero();
        libgnss::Vector3d velocity_sum = libgnss::Vector3d::Zero();
        int consistent = 0;
        int windows = 0;
        for (std::size_t i = 0; i + kHeadingWindowEpochs < problem.epochs.size(); ++i) {
            const std::size_t j = i + kHeadingWindowEpochs;
            const double dt = problem.epochs[j].time - problem.epochs[i].time;
            if (dt <= 1e-3) continue;
            const Eigen::Vector3d p0 = libgnss::ecef2enu(
                problem.epochs[i].position_ecef - origin_ecef, lat, lon);
            const Eigen::Vector3d p1 = libgnss::ecef2enu(
                problem.epochs[j].position_ecef - origin_ecef, lat, lon);
            const libgnss::Vector3d velocity = (p1 - p0) / dt;
            const double speed = std::hypot(velocity.x(), velocity.y());
            if (!std::isfinite(speed) || speed < kHeadingSpeedMinMps ||
                speed > kHeadingSpeedMaxMps || std::abs(velocity.z()) > kHeadingVerticalSpeedMaxMps) {
                consistent = 0;
                velocity_sum.setZero();
                continue;
            }
            ++windows;
            bool direction_consistent = true;
            if (consistent > 0) {
                const double previous_speed = std::hypot(previous_velocity.x(), previous_velocity.y());
                const double cosine = velocity.x() * previous_velocity.x() +
                                      velocity.y() * previous_velocity.y();
                direction_consistent = cosine /
                    std::max(1e-9, speed * previous_speed) >= std::cos(kHeadingConsistencyRad);
            }
            if (!direction_consistent) {
                consistent = 0;
                velocity_sum.setZero();
            }
            previous_velocity = velocity;
            velocity_sum += velocity;
            ++consistent;
            if (consistent >= kRequiredHeadingWindows) {
                const libgnss::Vector3d course = velocity_sum / static_cast<double>(consistent);
                if (libgnss::fusion_initialization::tryAlignHeading(state, course, 1.0, 5.0)) {
                    initial_velocity = course;
                    report.heading_latched = true;
                    break;
                }
            }
        }
        report.heading_windows = windows;
        report.heading_initialization_mode = "legacy-consistency-latch";
        if (!report.heading_latched) {
            report.failure = "GNSS course did not make heading observable under frozen gate";
            return false;
        }
    }

    auto& imu = problem.imu;
    imu.valid = true;
    imu.nav_origin_ecef = origin_ecef;
    imu.nav_origin_lat_rad = lat;
    imu.nav_origin_lon_rad = lon;
    imu.samples_body_flu = std::move(samples);
    imu.init_attitude_body_to_nav = state.nominal.attitude_body_to_enu.toRotationMatrix();
    imu.init_velocity_nav = initial_velocity;
    if (gnss_first_velocities_enu != nullptr &&
        gnss_first_velocities_enu->size() == problem.epochs.size()) {
        imu.stop_velocity_seeds_nav = *gnss_first_velocities_enu;
    }
    imu.init_accel_bias = aligned.accel_bias;
    imu.init_gyro_bias = aligned.gyro_bias;
    if (stationary_gyro_initializer) {
        const auto estimate = libgnss::stationary_gyro::estimate(imu.samples_body_flu);
        imu.init_gyro_bias = estimate.bias_radps;
        report.stationary_gyro_initializer_applied = true;
        report.stationary_gyro_blocks = estimate.blocks;
        report.stationary_gyro_scatter_radps = estimate.block_scatter_rms_radps;
    }
    // The values are the public taroz pixel preset after the 0.5 synchronization
    // coefficient, with no truth-driven tuning.
    imu.noise.gravity_mps2 = kGravity;
    const auto noise_selection =
        libgnss::native_utc_fallback_imu_noise::apply(
            imu.noise, phase194_source_utc_fallback_imu_noise,
            android_raw && report.android_load.utc_wall_clock_fallback_applied);
    report.phase194_source_utc_fallback_imu_noise_requested =
        noise_selection.requested;
    report.phase194_source_utc_fallback_imu_noise_applied =
        noise_selection.applied;
    report.imu_accel_noise_sigma = noise_selection.accel_noise_sigma;
    report.imu_gyro_noise_sigma = noise_selection.gyro_noise_sigma;
    report.imu_measurement_noise_sync_coefficient =
        noise_selection.measurement_sync_coefficient;
    report.imu_measurement_noise_source = noise_selection.source;
    imu.noise.accel_bias_rw_sigma = 0.00025;
    imu.noise.gyro_bias_rw_sigma = 0.0000005;
    imu.noise.integration_sigma = 0.05;
    imu.init_attitude_sigma_roll_pitch_rad = 0.05;
    imu.init_attitude_sigma_yaw_rad = 5.0 / kRadToDeg;
    imu.init_velocity_sigma_mps = 0.5;
    imu.init_accel_bias_sigma = 0.1;
    imu.init_gyro_bias_sigma = 0.01;
    report.ok = true;
    return true;
}

void writeJsonString(std::ostringstream& out, const std::string& value) {
    out << '"';
    for (const char ch : value) {
        switch (ch) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << ch; break;
        }
    }
    out << '"';
}

std::string makePhase149RawPSeedJson(
    const std::string& dataset_id,
    const libgnss::raw_p_seed::Result& result) {
    std::ostringstream out;
    const auto writeFinite = [&out](double value) {
        if (std::isfinite(value)) {
            out << value;
        } else {
            out << "null";
        }
    };
    out << std::setprecision(17)
        << "{\n  \"schema_version\": "
        << "\"smartphone-r5-phase149-raw-p-seed-stage.v1\",\n"
        << "  \"phase\": 149,\n  \"dataset_id\": ";
    writeJsonString(out, dataset_id);
    out << ",\n  \"status\": ";
    writeJsonString(out, result.ok ? "accepted" : "failed-closed");
    out << ",\n  \"failure_status\": ";
    writeJsonString(out,
                    libgnss::raw_p_seed::epochStatusName(result.failure_status));
    out << ",\n  \"failure_reason\": ";
    writeJsonString(out, result.failure_reason);
    out << ",\n  \"collect_all_epochs_for_diagnostics\": "
        << (result.collect_all_epochs_for_diagnostics ? "true" : "false")
        << ",\n  \"velocity_diagnostics_disabled\": "
        << (result.velocity_diagnostics_disabled ? "true" : "false")
        << ",\n  \"input_epoch_count\": "
        << result.input_epoch_count
        << ",\n  \"evaluated_epoch_count\": "
        << result.evaluated_epoch_count
        << ",\n  \"accepted_epoch_count\": "
        << result.accepted_epoch_count
        << ",\n  \"rejected_epoch_count\": "
        << result.rejected_epoch_count;
    out << ",\n  \"velocity_endpoint_policy\": ";
    writeJsonString(
        out,
        libgnss::raw_p_seed::velocityEndpointPolicyName(result.endpoint_policy));
    out << ",\n  \"raw_pseudorange_only\": true,\n"
        << "  \"doppler_consumed\": false,\n"
        << "  \"imported_seed_files\": false,\n"
        << "  \"fgo_entered\": false,\n"
        << "  \"truth_used\": false,\n"
        << "  \"mat_used\": false,\n"
        << "  \"epochs\": [\n";
    for (std::size_t i = 0; i < result.epochs.size(); ++i) {
        const auto& epoch = result.epochs[i];
        out << "    {\n"
            << "      \"input_epoch_index\": " << epoch.input_epoch_index << ",\n"
            << "      \"time_week\": " << epoch.time.week << ",\n"
            << "      \"time_tow\": ";
        writeFinite(epoch.time.tow);
        out << ",\n      \"raw_source_index\": " << epoch.raw_source_index << ",\n"
            << "      \"raw_utc_time_millis\": " << epoch.raw_utc_time_millis << ",\n"
            << "      \"status\": ";
        writeJsonString(out, libgnss::raw_p_seed::epochStatusName(epoch.status));
        out << ",\n      \"reason\": ";
        writeJsonString(out, epoch.reason);
        out << ",\n      \"native_spp_status_available\": "
            << (epoch.native_spp_status_available ? "true" : "false")
            << ",\n      \"native_spp_status\": ";
        writeJsonString(out,
                        libgnss::raw_p_seed::nativeSppStatusName(
                            epoch.native_spp_status));
        out << ",\n      \"raw_pseudorange_rows\": "
            << epoch.raw_pseudorange_rows << ",\n"
            << "      \"raw_pseudorange_satellites\": "
            << epoch.raw_pseudorange_satellites << ",\n"
            << "      \"raw_clock_groups\": " << epoch.raw_clock_groups << ",\n"
            << "      \"corrected_pseudorange_rows\": "
            << epoch.corrected_pseudorange_rows << ",\n"
            << "      \"native_used_pseudorange_rows\": "
            << epoch.native_used_pseudorange_rows << ",\n"
            << "      \"corrected_clock_groups\": "
            << epoch.corrected_clock_groups << ",\n"
            << "      \"reference_clock_group\": ";
        writeJsonString(
            out,
            libgnss::raw_p_seed::clockGroupName(epoch.reference_clock_group));
        out << ",\n      \"clock_group_biases\": [\n";
        for (std::size_t group_index = 0;
             group_index < epoch.clock_group_biases.size(); ++group_index) {
            const auto& group = epoch.clock_group_biases[group_index];
            out << "        {\n          \"group\": ";
            writeJsonString(out, libgnss::raw_p_seed::clockGroupName(group.group));
            out << ",\n          \"observed\": "
                << (group.observed ? "true" : "false")
                << ",\n          \"is_reference\": "
                << (group.is_reference ? "true" : "false")
                << ",\n          \"estimate_available\": "
                << (group.estimate_available ? "true" : "false")
                << ",\n          \"bias_m\": ";
            writeFinite(group.bias_m);
            out << ",\n          \"corrected_rows\": "
                << group.corrected_rows << "\n        }"
                << (group_index + 1U == epoch.clock_group_biases.size()
                        ? "\n"
                        : ",\n");
        }
        out << "      ],\n"
            << "      \"preprocessing_diagnostics_available\": "
            << (epoch.preprocessing_diagnostics_available ? "true" : "false")
            << ",\n"
            << "      \"preprocessing_input_rows\": "
            << epoch.preprocessing_input_rows << ",\n"
            << "      \"preprocessing_accepted_rows\": "
            << epoch.preprocessing_accepted_rows << ",\n"
            << "      \"preprocessing_rejected_rows\": "
            << epoch.preprocessing_rejected_rows << ",\n"
            << "      \"preprocessing_reason_counts\": [\n";
        for (std::size_t reason_index = 0;
             reason_index < epoch.preprocessing_reason_counts.size();
             ++reason_index) {
            const auto& reason = epoch.preprocessing_reason_counts[reason_index];
            out << "        {\"reason\": ";
            writeJsonString(out, reason.reason);
            out << ", \"count\": " << reason.count << "}"
                << (reason_index + 1U == epoch.preprocessing_reason_counts.size()
                        ? "\n"
                        : ",\n");
        }
        out << "      ],\n"
            << "      \"preprocessing_rows\": [\n";
        for (std::size_t row_index = 0;
             row_index < epoch.preprocessing_rows.size(); ++row_index) {
            const auto& row = epoch.preprocessing_rows[row_index];
            out << "        {\"input_row_index\": " << row.input_row_index
                << ", \"system\": ";
            writeJsonString(out, libgnss::raw_p_seed::clockGroupName(row.system));
            out << ", \"accepted\": " << (row.accepted ? "true" : "false")
                << ", \"reason\": ";
            writeJsonString(out, row.reason);
            out << "}"
                << (row_index + 1U == epoch.preprocessing_rows.size()
                        ? "\n"
                        : ",\n");
        }
        out << "      ],\n"
            << "      \"satellites_used\": " << epoch.satellites_used << ",\n"
            << "      \"iterations\": " << epoch.iterations << ",\n"
            << "      \"degrees_of_freedom\": " << epoch.degrees_of_freedom << ",\n"
            << "      \"converged\": " << (epoch.converged ? "true" : "false") << ",\n"
            << "      \"gdop\": ";
        writeFinite(epoch.gdop);
        out << ",\n      \"pdop\": ";
        writeFinite(epoch.pdop);
        out << ",\n      \"residual_rms_m\": ";
        writeFinite(epoch.residual_rms_m);
        out << ",\n      \"max_abs_residual_m\": ";
        writeFinite(epoch.max_abs_residual_m);
        out << ",\n      \"position_finite\": "
            << (epoch.position_ecef.allFinite() ? "true" : "false") << ",\n"
            << "      \"clock_finite\": "
            << (std::isfinite(epoch.receiver_clock_bias_m) ? "true" : "false")
            << ",\n"
            << "      \"velocity_finite\": "
            << (epoch.has_velocity && epoch.velocity_ecef_mps.allFinite()
                    ? "true"
                    : "false")
            << ",\n"
            << "      \"geometry_rank_status\": ";
        writeJsonString(
            out,
            libgnss::raw_p_seed::rankStatusName(epoch.geometry_rank.status));
        out << ",\n      \"geometry_rank\": " << epoch.geometry_rank.rank << ",\n"
            << "      \"geometry_required_rank\": "
            << epoch.geometry_rank.required_rank << ",\n"
            << "      \"geometry_rank_rows\": " << epoch.geometry_rank.rows << "\n"
            << "    }" << (i + 1U == result.epochs.size() ? "\n" : ",\n");
    }
    out << "  ]\n}\n";
    return out.str();
}

std::string makePhase163RawPNoDopplerSeedJson(
    const std::string& dataset_id,
    const libgnss::raw_p_seed::Result& raw_result,
    const libgnss::raw_p_seed::RawPNoDopplerSeedAdapterResult& adapter) {
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\n  \"schema_version\": "
        << "\"smartphone-r5-phase163-raw-p-no-doppler-seed-stage.v1\",\n"
        << "  \"phase\": 163,\n  \"dataset_id\": ";
    writeJsonString(out, dataset_id);
    out << ",\n  \"status\": ";
    writeJsonString(out, adapter.ok ? "accepted" : "failed-closed");
    out << ",\n  \"adapter_status\": ";
    writeJsonString(
        out, libgnss::raw_p_seed::seedAdapterStatusName(adapter.status));
    out << ",\n  \"failure_reason\": ";
    writeJsonString(out, adapter.failure_reason);
    out << ",\n  \"graph_compatible\": "
        << (adapter.graph_compatible ? "true" : "false")
        << ",\n  \"graph_disabled_reason\": ";
    writeJsonString(out, adapter.graph_disabled_reason);
    out << ",\n  \"input_epoch_count\": " << adapter.input_epoch_count
        << ",\n  \"accepted_epoch_count\": "
        << adapter.accepted_epoch_count
        << ",\n  \"rejected_epoch_count\": "
        << adapter.rejected_epoch_count
        << ",\n  \"raw_p_seed_ok\": "
        << (raw_result.ok ? "true" : "false")
        << ",\n  \"same_run_raw_p_only\": true,\n"
        << "  \"velocity_source\": "
        << "\"raw-p-same-run-position-gradient\",\n"
        << "  \"velocity_endpoint_policy\": ";
    writeJsonString(
        out,
        libgnss::raw_p_seed::velocityEndpointPolicyName(
            raw_result.endpoint_policy));
    out << ",\n  \"clock_rate_source\": "
        << "\"exact-original-epoch-receiver_clock_drift_mps\",\n"
        << "  \"c7_reference_policy\": "
        << "\"GPS-reference-only-C0;unproven-frequency-ISBs-unavailable\",\n"
        << "  \"imported_seed_files\": false,\n"
        << "  \"fgo_entered\": false,\n"
        << "  \"truth_used\": false,\n"
        << "  \"mat_used\": false,\n"
        << "  \"seeds\": [\n";
    for (std::size_t i = 0; i < adapter.seeds.size(); ++i) {
        const auto& seed = adapter.seeds[i];
        const auto& raw_seed = raw_result.epochs[i];
        out << "    {\n"
            << "      \"epoch_index\": " << seed.epoch_index << ",\n"
            << "      \"raw_source_index\": " << seed.raw_source_index
            << ",\n      \"raw_utc_time_millis\": "
            << seed.raw_utc_time_millis
            << ",\n      \"raw_p_status\": ";
        writeJsonString(out,
                        libgnss::raw_p_seed::epochStatusName(raw_seed.status));
        out << ",\n      \"adapter_status\": ";
        writeJsonString(
            out, libgnss::raw_p_seed::seedAdapterStatusName(seed.status));
        out << ",\n      \"reason\": ";
        writeJsonString(out, seed.reason);
        out << ",\n      \"reference_clock_group\": ";
        writeJsonString(
            out, libgnss::raw_p_seed::clockGroupName(seed.reference_clock_group));
        out << ",\n      \"position_finite\": "
            << (seed.has_position && seed.position_ecef.allFinite() ? "true"
                                                                      : "false")
            << ",\n      \"velocity_finite\": "
            << (seed.has_velocity && seed.velocity_ecef_mps.allFinite() ? "true"
                                                                          : "false")
            << ",\n      \"clock_bias_finite\": "
            << (seed.has_clock && std::isfinite(seed.clock_bias_m) ? "true"
                                                                    : "false")
            << ",\n      \"clock_rate_finite\": "
            << (seed.has_clock_rate && std::isfinite(seed.clock_rate_mps)
                    ? "true"
                    : "false")
            << ",\n      \"c7_clock_mapping_supported\": "
            << (seed.c7_clock_mapping_supported ? "true" : "false")
            << ",\n      \"c7_component_available\": [";
        for (std::size_t component = 0; component < seed.clock_bias_component_available.size();
             ++component) {
            if (component != 0U) out << ", ";
            out << (seed.clock_bias_component_available[component] ? "true"
                                                                    : "false");
        }
        out << "]\n    }"
            << (i + 1U == adapter.seeds.size() ? "\n" : ",\n");
    }
    out << "  ]\n}\n";
    return out.str();
}

void writePhase143TerminationDiagnostics(
    std::ostringstream& out,
    const libgnss::FGOProcessor::FGOPhase143TerminationDiagnostics& report);

std::string makePhase165RawPNoDopplerGraphJson(
    const std::string& dataset_id,
    const libgnss::raw_p_seed::RawPNoDopplerSeedAdapterResult& adapter,
    const libgnss::raw_p_seed::Result* raw_result,
    const libgnss::FGOProcessor::FGOProblem* problem,
    const libgnss::FGOProcessor::FGOResult* result,
    const std::string& failure,
    bool fgo_attempted,
    bool fgo_returned,
    bool phase167_budget) {
    std::size_t supported_rows = 0U;
    std::size_t unsupported_rows = 0U;
    for (const auto& seed : adapter.seeds) {
        supported_rows += seed.c7_supported_pseudorange_rows;
        unsupported_rows += seed.c7_unsupported_pseudorange_rows;
    }
    std::ostringstream out;
    const auto writeFinite = [&out](double value) {
        if (std::isfinite(value)) {
            out << value;
        } else {
            out << "null";
        }
    };
    out << std::setprecision(17)
        << "{\n  \"schema_version\": "
        << (phase167_budget
                ? "\"smartphone-r5-phase167-raw-p-no-doppler-graph.v1\",\n"
                : "\"smartphone-r5-phase165-raw-p-no-doppler-graph.v1\",\n")
        << "  \"phase\": " << (phase167_budget ? 167 : 165)
        << ",\n  \"dataset_id\": ";
    writeJsonString(out, dataset_id);
    out << ",\n  \"status\": ";
    writeJsonString(
        out, phase167_budget
                 ? (result != nullptr && fgo_returned &&
                            result->diagnostics.converged
                        ? "completed"
                        : "failed-closed")
                 : (result != nullptr && result->diagnostics.converged
                        ? "converged"
                        : "failed-closed"));
    out << ",\n  \"failure_reason\": ";
    writeJsonString(out, failure);
    out << ",\n  \"same_run_raw_p_only\": true,\n"
        << "  \"imported_seed_files\": false,\n"
        << "  \"truth_used\": false,\n  \"mat_used\": false,\n"
        << "  \"imu_or_main_graph_entered\": false,\n"
        << "  \"adapter_ok\": " << (adapter.ok ? "true" : "false")
        << ",\n  \"adapter_graph_compatible\": "
        << (adapter.graph_compatible ? "true" : "false")
        << ",\n  \"adapter_input_epochs\": " << adapter.input_epoch_count
        << ",\n  \"adapter_accepted_epochs\": "
        << adapter.accepted_epoch_count
        << ",\n  \"adapter_rejected_epochs\": "
        << adapter.rejected_epoch_count
        << ",\n  \"raw_supported_c7_rows\": " << supported_rows
        << ",\n  \"raw_unsupported_c7_rows_explicitly_excluded\": "
        << unsupported_rows;
    out << ",\n  \"raw_result_present\": "
        << (raw_result != nullptr ? "true" : "false")
        << ",\n  \"phase167_budget_selector\": "
        << (phase167_budget ? "true" : "false")
        << ",\n  \"fgo_attempted\": "
        << (fgo_attempted ? "true" : "false")
        << ",\n  \"fgo_returned\": "
        << (fgo_returned ? "true" : "false");
    if (raw_result != nullptr) {
        out << ",\n  \"raw_result_ok\": "
            << (raw_result->ok ? "true" : "false")
            << ",\n  \"raw_failure_status\": ";
        writeJsonString(
            out, libgnss::raw_p_seed::epochStatusName(raw_result->failure_status));
        out << ",\n  \"raw_failure_reason\": ";
        writeJsonString(out, raw_result->failure_reason);
        out << ",\n  \"raw_input_epoch_count\": "
            << raw_result->input_epoch_count
            << ",\n  \"raw_evaluated_epoch_count\": "
            << raw_result->evaluated_epoch_count
            << ",\n  \"raw_accepted_epoch_count\": "
            << raw_result->accepted_epoch_count
            << ",\n  \"raw_rejected_epoch_count\": "
            << raw_result->rejected_epoch_count
            << ",\n  \"raw_unassessed_epoch_count\": "
            << (raw_result->input_epoch_count >= raw_result->evaluated_epoch_count
                    ? raw_result->input_epoch_count - raw_result->evaluated_epoch_count
                    : 0U);
        const auto failed_epoch = std::find_if(
            raw_result->epochs.begin(), raw_result->epochs.end(),
            [](const auto& epoch) {
                return epoch.status != libgnss::raw_p_seed::EpochStatus::Accepted;
            });
        out << ",\n  \"raw_failed_epoch_present\": "
            << (failed_epoch != raw_result->epochs.end() ? "true" : "false");
        if (failed_epoch != raw_result->epochs.end()) {
            out << ",\n  \"raw_failed_epoch_index\": "
                << failed_epoch->input_epoch_index
                << ",\n  \"raw_failed_epoch_status\": ";
            writeJsonString(
                out, libgnss::raw_p_seed::epochStatusName(failed_epoch->status));
            out << ",\n  \"raw_failed_epoch_reason\": ";
            writeJsonString(out, failed_epoch->reason);
        }
    }
    if (problem != nullptr) {
        out << ",\n  \"retained_epochs\": " << problem->epochs.size()
            << ",\n  \"retained_pseudorange_factors\": "
            << problem->pseudorange_factors.size()
            << ",\n  \"retained_tdcp_factors\": "
            << problem->tdcp_factors.size()
            << ",\n  \"retained_undifferenced_doppler_factors\": "
            << problem->undifferenced_doppler_factors.size();
    }
    if (result != nullptr) {
        std::size_t finite_positions = 0U;
        std::size_t earth_valid_positions = 0U;
        std::size_t finite_clocks = 0U;
        for (const auto& solution : result->solution.solutions) {
            if (solution.position_ecef.allFinite()) ++finite_positions;
            if (earthValidEcef(solution.position_ecef)) ++earth_valid_positions;
            if (std::isfinite(solution.receiver_clock_bias)) ++finite_clocks;
        }
        std::size_t finite_velocities = 0U;
        for (const auto& velocity : result->epoch_velocities_ecef_mps) {
            if (velocity.allFinite()) ++finite_velocities;
        }
        std::size_t finite_c7 = 0U;
        for (const auto& components : result->epoch_clock_bias_components_m) {
            if (std::all_of(components.begin(), components.end(),
                            [](double value) { return std::isfinite(value); })) {
                ++finite_c7;
            }
        }
        std::size_t finite_clock_drifts = 0U;
        for (const double drift : result->epoch_clock_drift_mps) {
            if (std::isfinite(drift)) ++finite_clock_drifts;
        }
        out << ",\n  \"converged\": "
            << (result->diagnostics.converged ? "true" : "false")
            << ",\n  \"iterations\": " << result->diagnostics.iterations
            << ",\n  \"solution_output_cardinality\": "
            << result->solution.solutions.size()
            << ",\n  \"finite_position_count\": " << finite_positions
            << ",\n  \"earth_valid_position_count\": "
            << earth_valid_positions
            << ",\n  \"finite_clock_count\": " << finite_clocks
            << ",\n  \"velocity_output_cardinality\": "
            << result->epoch_velocities_ecef_mps.size()
            << ",\n  \"finite_velocity_count\": " << finite_velocities
            << ",\n  \"c7_output_cardinality\": "
            << result->epoch_clock_bias_components_m.size()
            << ",\n  \"finite_c7_count\": " << finite_c7
            << ",\n  \"clock_drift_output_cardinality\": "
            << result->epoch_clock_drift_mps.size()
            << ",\n  \"finite_clock_drift_count\": "
            << finite_clock_drifts
            << ",\n  \"initial_cost\": ";
        writeFinite(result->diagnostics.initial_cost);
        out << ",\n  \"final_cost\": ";
        writeFinite(result->diagnostics.final_cost);
        out << ",\n  \"costs_finite\": "
            << (std::isfinite(result->diagnostics.initial_cost) &&
                        std::isfinite(result->diagnostics.final_cost)
                    ? "true"
                    : "false")
            << ",\n  \"graph_factors\": "
            << result->diagnostics.graph_factors
            << ",\n  \"graph_values\": "
            << result->diagnostics.graph_values
            << ",\n  \"pseudorange_factors\": "
            << result->diagnostics.pseudorange_factors
            << ",\n  \"tdcp_factors\": "
            << result->diagnostics.tdcp_factors
            << ",\n  \"undifferenced_doppler_factors\": "
            << result->diagnostics.undifferenced_doppler_factors
            << ",\n  \"motion_factors\": "
            << result->diagnostics.motion_factors
            << ",\n  \"c0d_factors\": "
            << result->diagnostics.native_source_clock_c0d_factor_count
            << ",\n  \"unobserved_clock_gauge_components\": "
            << result->diagnostics
                   .native_raw_p_no_doppler_unobserved_clock_gauge_components;
    }
    out << ",\n  \"phase167_termination\": ";
    if (phase167_budget && result != nullptr) {
        const auto& termination = result->diagnostics.native_phase143_termination;
        out << "{\n"
            << "    \"schema_version\": "
            << "\"smartphone-r5-phase167-termination.v1\",\n"
            << "    \"selector\": "
            << "\"--native-phase167-raw-p-no-doppler-lm-termination-budget\",\n"
            << "    \"authority\": "
            << "\"FGOResult.diagnostics.native_phase143_termination\",\n"
            << "    \"solver_reported_converged\": "
            << (result->diagnostics.converged ? "true" : "false") << ",\n"
            << "    \"termination_class\": ";
        writeJsonString(out, termination.termination_branch);
        out << ",\n    \"tolerance_proven\": "
            << (termination.termination_branch == "outer_convergence_tolerance"
                    ? "true"
                    : "false")
            << ",\n    \"effective_cap_reached\": "
            << (termination.termination_branch == "maximum_outer_iterations"
                    ? "true"
                    : "false")
            << ",\n    \"report\": ";
        writePhase143TerminationDiagnostics(
            out, result->diagnostics.native_phase143_termination);
        out << "\n  }";
    } else {
        out << "null";
    }
    out << "\n}\n";
    return out.str();
}

// Phase144 keeps the implementation's compact RHS diagnostic for internal
// use, but emits one canonical semantic representation at the JSON boundary.
// The fixed order is intentional: launch-free validators can compare the
// representation without parsing or inferring an equation from prose.
constexpr const char kPhase138EquationSemanticId[] =
    "phase138-affine-tdcp-anchor-range-constant-v1";
constexpr const char kPhase138EquationDisplay[] =
    "tdcp_phase138 = tdcp_native - (rho_current_initial - rho_previous_initial)";

void writePhase138EquationMetadata(std::ostringstream& out,
                                  const std::string& indent) {
    out << "{\n"
        << indent << "  \"semantic_id\": \""
        << kPhase138EquationSemanticId << "\",\n"
        << indent << "  \"representation\": \"full-assignment\",\n"
        << indent << "  \"display_expression\": \""
        << kPhase138EquationDisplay << "\",\n"
        << indent << "  \"ast\": {\n"
        << indent << "    \"kind\": \"assign\",\n"
        << indent << "    \"lhs\": \"tdcp_phase138\",\n"
        << indent << "    \"rhs\": {\n"
        << indent << "      \"kind\": \"sub\",\n"
        << indent << "      \"left\": \"tdcp_native\",\n"
        << indent << "      \"right\": {\n"
        << indent << "        \"kind\": \"sub\",\n"
        << indent << "        \"left\": \"rho_current_initial\",\n"
        << indent << "        \"right\": \"rho_previous_initial\"\n"
        << indent << "      }\n"
        << indent << "    }\n"
        << indent << "  },\n"
        << indent << "  \"token_tuple\": [\"ASSIGN\", \"tdcp_phase138\", "
           "\"SUB\", \"GROUP_OPEN\", \"tdcp_native\", \"SUB\", "
           "\"GROUP_OPEN\", \"rho_current_initial\", \"SUB\", "
           "\"rho_previous_initial\", \"GROUP_CLOSE\", \"GROUP_CLOSE\"],\n"
        << indent << "  \"rhs_only_token_tuple\": [\"tdcp_native\", "
           "\"SUB\", \"GROUP_OPEN\", \"rho_current_initial\", "
           "\"SUB\", \"rho_previous_initial\", \"GROUP_CLOSE\"],\n"
        << indent << "  \"whitespace_normalization_only\": true\n"
        << indent << "}";
}

void writePhase94Bool(std::ostringstream& out, bool value) {
    out << (value ? "true" : "false");
}

void writePhase94Double(std::ostringstream& out, double value) {
    // JSON has no NaN/Inf values.  Keep unavailable/nonfinite telemetry
    // explicit as null; the companion finite/count fields retain the exact
    // failure evidence without ever publishing a solution value.
    if (std::isfinite(value)) {
        out << value;
    } else {
        out << "null";
    }
}

void writePhase104StageReport(std::ostringstream& out,
                              const Phase104StageExportReport& report) {
    out << "{\n      \"enabled\": ";
    writePhase94Bool(out, report.enabled);
    out << ",\n      \"attempted\": ";
    writePhase94Bool(out, report.attempted);
    out << ",\n      \"written\": ";
    writePhase94Bool(out, report.written);
    out << ",\n      \"path\": ";
    writeJsonString(out, report.path);
    out << ",\n      \"source_epoch_count\": " << report.source_epoch_count
        << ",\n      \"target_epoch_count\": " << report.target_epoch_count
        << ",\n      \"exact_epoch_count\": " << report.exact_epoch_count
        << ",\n      \"interpolated_epoch_count\": "
        << report.interpolated_epoch_count
        << ",\n      \"edge_hold_epoch_count\": "
        << report.edge_hold_epoch_count
        << ",\n      \"unresolved_epoch_count\": "
        << report.unresolved_epoch_count
        << ",\n      \"finite_position_count\": "
        << report.finite_position_count
        << ",\n      \"nonfinite_position_count\": "
        << report.nonfinite_position_count
        << ",\n      \"out_of_earth_position_count\": "
        << report.out_of_earth_position_count
        << ",\n      \"exact_retained_key_alignment\": ";
    writePhase94Bool(out, report.exact_retained_key_alignment);
    out << ",\n      \"all_positions_finite\": ";
    writePhase94Bool(out, report.all_positions_finite);
    out << ",\n      \"all_positions_earth_valid\": ";
    writePhase94Bool(out, report.all_positions_earth_valid);
    out << ",\n      \"failure\": ";
    writeJsonString(out, report.failure);
    out << "\n    }";
}

void writePhase104DisplacementReport(
    std::ostringstream& out,
    const Phase104MainDisplacementReport& report) {
    out << "{\n      \"enabled\": ";
    writePhase94Bool(out, report.enabled);
    out << ",\n      \"attempted\": ";
    writePhase94Bool(out, report.attempted);
    out << ",\n      \"written\": ";
    writePhase94Bool(out, report.written);
    out << ",\n      \"path\": ";
    writeJsonString(out, report.path);
    out << ",\n      \"solution_epoch_count\": "
        << report.solution_epoch_count
        << ",\n      \"transition_count\": " << report.transition_count
        << ",\n      \"finite_transition_count\": "
        << report.finite_transition_count
        << ",\n      \"nonfinite_transition_count\": "
        << report.nonfinite_transition_count
        << ",\n      \"p50_displacement_m\": ";
    writePhase94Double(out, report.p50_displacement_m);
    out << ",\n      \"p95_displacement_m\": ";
    writePhase94Double(out, report.p95_displacement_m);
    out << ",\n      \"max_displacement_m\": ";
    writePhase94Double(out, report.max_displacement_m);
    out << ",\n      \"max_speed_mps\": ";
    writePhase94Double(out, report.max_speed_mps);
    out << ",\n      \"over_70_mps_count\": "
        << report.over_70_mps_count
        << ",\n      \"all_transitions_finite\": ";
    writePhase94Bool(out, report.all_transitions_finite);
    out << ",\n      \"failure\": ";
    writeJsonString(out, report.failure);
    out << "\n    }";
}

void writePhase94StringArray(std::ostringstream& out,
                             const std::vector<std::string>& values) {
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0U) out << ", ";
        writeJsonString(out, values[i]);
    }
    out << "]";
}

void writePhase94Preflight(std::ostringstream& out,
                           const Phase94C0DPreflight& report) {
    out << "{\n"
        << "      \"c0d_factor_enabled\": ";
    writePhase94Bool(out, report.c0d_factor_enabled);
    out << ",\n      \"meter_state_parity_enabled\": ";
    writePhase94Bool(out, report.meter_state_parity_enabled);
    out << ",\n      \"raw_d_initializer_enabled\": ";
    writePhase94Bool(out, report.raw_d_initializer_enabled);
    out << ",\n      \"gnss_first_handoff_enabled\": ";
    writePhase94Bool(out, report.gnss_first_handoff_enabled);
    out << ",\n      \"direct_quality_enabled\": ";
    writePhase94Bool(out, report.direct_quality_enabled);
    out << ",\n      \"undifferenced_doppler_enabled\": ";
    writePhase94Bool(out, report.undifferenced_doppler_enabled);
    out << ",\n      \"motion_factors_enabled\": ";
    writePhase94Bool(out, report.motion_factors_enabled);
    out << ",\n      \"clock_motion_factors_enabled\": ";
    writePhase94Bool(out, report.clock_motion_factors_enabled);
    out << ",\n      \"pdc_bridge_disabled\": ";
    writePhase94Bool(out, report.pdc_bridge_disabled);
    out << ",\n      \"fixed_lag_disabled\": ";
    writePhase94Bool(out, report.fixed_lag_disabled);
    out << ",\n      \"phone_identity_present\": ";
    writePhase94Bool(out, report.phone_identity_present);
    out << ",\n      \"backend_configuration_allowed\": ";
    writePhase94Bool(out, report.backend_configuration_allowed);
    out << ",\n      \"source_clock_c0d_problem_path\": ";
    writePhase94Bool(out, report.source_clock_c0d_problem_path);
    out << ",\n      \"retained_epoch_count_ge_two\": ";
    writePhase94Bool(out, report.retained_epoch_count_ge_two);
    out << ",\n      \"retained_epoch_count\": "
        << report.retained_epoch_count
        << ",\n      \"retained_undifferenced_doppler_factor_count\": "
        << report.retained_undifferenced_doppler_factor_count
        << ",\n      \"eligible_c0d_pair_count\": "
        << report.eligible_c0d_pair_count
        << ",\n      \"eligible_c0d_factor_count\": "
        << report.eligible_c0d_factor_count
        << ",\n      \"invalid_dt_skip_count\": "
        << report.invalid_dt_skip_count
        << ",\n      \"gap_skip_count\": " << report.gap_skip_count
        << ",\n      \"phone_exclusion_skip_count\": "
        << report.phone_exclusion_skip_count
        << ",\n      \"clock_jump_skip_count\": "
        << report.clock_jump_skip_count
        << ",\n      \"dt_min_s\": ";
    writePhase94Double(out, report.dt_min_s);
    out << ",\n      \"dt_max_s\": ";
    writePhase94Double(out, report.dt_max_s);
    out << ",\n      \"d_initializer_epoch_count\": "
        << report.d_initializer_epoch_count
        << ",\n      \"d_initializer_finite_count\": "
        << report.d_initializer_finite_count
        << ",\n      \"d_initializer_nonfinite_count\": "
        << report.d_initializer_nonfinite_count
        << ",\n      \"d_initializer_coverage_valid\": ";
    writePhase94Bool(out, report.d_initializer_coverage_valid);
    out << ",\n      \"d_initializer_all_finite\": ";
    writePhase94Bool(out, report.d_initializer_all_finite);
    out << ",\n      \"guard_rejected\": ";
    writePhase94Bool(out, report.guard_rejected);
    out << ",\n      \"guard_predicate\": ";
    writeJsonString(out, report.guard_predicate);
    out << ",\n      \"guard_failed_predicates\": ";
    writePhase94StringArray(out, report.guard_failed_predicates);
    out << "\n    }";
}

void writePhase94GnssFirst(std::ostringstream& out,
                           const Phase94GnssFirstTelemetry& report) {
    out << "{\n      \"attempted\": ";
    writePhase94Bool(out, report.attempted);
    out << ",\n      \"result_returned\": ";
    writePhase94Bool(out, report.result_returned);
    out << ",\n      \"converged\": ";
    writePhase94Bool(out, report.converged);
    out << ",\n      \"epochs\": " << report.epochs
        << ",\n      \"undifferenced_doppler_factors\": "
        << report.undifferenced_doppler_factors
        << ",\n      \"velocity_states\": " << report.velocity_states
        << ",\n      \"iterations\": " << report.iterations
        << ",\n      \"initial_cost\": ";
    writePhase94Double(out, report.initial_cost);
    out << ",\n      \"final_cost\": ";
    writePhase94Double(out, report.final_cost);
    out << ",\n      \"costs_finite\": ";
    writePhase94Bool(out, report.costs_finite);
    out << ",\n      \"c0d_factor_count\": " << report.c0d_factor_count
        << ",\n      \"c0d_accepted_outer_iterations\": "
        << report.c0d_accepted_outer_iterations
        << ",\n      \"c0d_inner_lambda_attempts\": "
        << report.c0d_inner_lambda_attempts
        << ",\n      \"c0d_active_solve_attempted\": ";
    writePhase94Bool(out, report.c0d_active_solve_attempted);
    out << ",\n      \"c0d_active_solve_finite_costs\": ";
    writePhase94Bool(out, report.c0d_active_solve_finite_costs);
    out << ",\n      \"strict_cost_progress\": ";
    writePhase94Bool(out, report.strict_cost_progress);
    out << ",\n      \"optimized_d_epoch_count\": "
        << report.optimized_d_epoch_count
        << ",\n      \"optimized_d_finite_count\": "
        << report.optimized_d_finite_count
        << ",\n      \"optimized_d_nonfinite_count\": "
        << report.optimized_d_nonfinite_count
        << ",\n      \"optimized_d_coverage\": ";
    writePhase94Bool(out, report.optimized_d_coverage);
    out << ",\n      \"exact_retained_key_alignment\": ";
    writePhase94Bool(out, report.exact_retained_key_alignment);
    out << ",\n      \"unknown_guard_reason\": ";
    writePhase94Bool(out, report.unknown_guard_reason);
    out << ",\n      \"terminal_branch\": ";
    writeJsonString(out, report.terminal_branch);
    out << ",\n      \"failure\": ";
    writeJsonString(out, report.failure);
    out << "\n    }";
}

void writePhase94Main(std::ostringstream& out,
                      const Phase94MainTelemetry& report) {
    out << "{\n      \"attempted\": ";
    writePhase94Bool(out, report.attempted);
    out << ",\n      \"result_returned\": ";
    writePhase94Bool(out, report.result_returned);
    out << ",\n      \"converged\": ";
    writePhase94Bool(out, report.converged);
    out << ",\n      \"solution_nonempty\": ";
    writePhase94Bool(out, report.solution_nonempty);
    out << ",\n      \"problem_epoch_count\": " << report.problem_epoch_count
        << ",\n      \"position_solution_size\": "
        << report.position_solution_size
        << ",\n      \"position_size_matches_problem_epochs\": ";
    writePhase94Bool(out, report.position_size_matches_problem_epochs);
    out << ",\n      \"receiver_clock_solution_size\": "
        << report.receiver_clock_solution_size
        << ",\n      \"receiver_clock_size_matches_problem_epochs\": ";
    writePhase94Bool(out, report.receiver_clock_size_matches_problem_epochs);
    out << ",\n      \"earth_valid_position_count\": "
        << report.earth_valid_position_count
        << ",\n      \"position_nonfinite_count\": "
        << report.position_nonfinite_count
        << ",\n      \"position_out_of_earth_count\": "
        << report.position_out_of_earth_count
        << ",\n      \"all_positions_earth_valid\": ";
    writePhase94Bool(out, report.all_positions_earth_valid);
    out << ",\n      \"receiver_clock_finite_count\": "
        << report.receiver_clock_finite_count
        << ",\n      \"receiver_clock_nonfinite_count\": "
        << report.receiver_clock_nonfinite_count
        << ",\n      \"all_receiver_clocks_finite\": ";
    writePhase94Bool(out, report.all_receiver_clocks_finite);
    out << ",\n      \"optimized_d_epoch_count\": "
        << report.optimized_d_epoch_count
        << ",\n      \"optimized_d_size_matches_problem_epochs\": ";
    writePhase94Bool(out, report.optimized_d_size_matches_problem_epochs);
    out << ",\n      \"optimized_d_finite_count\": "
        << report.optimized_d_finite_count
        << ",\n      \"optimized_d_nonfinite_count\": "
        << report.optimized_d_nonfinite_count
        << ",\n      \"optimized_d_all_finite\": ";
    writePhase94Bool(out, report.optimized_d_all_finite);
    out << ",\n      \"velocity_epoch_count\": " << report.velocity_epoch_count
        << ",\n      \"velocity_size_matches_problem_epochs\": ";
    writePhase94Bool(out, report.velocity_size_matches_problem_epochs);
    out << ",\n      \"velocity_finite_count\": " << report.velocity_finite_count
        << ",\n      \"velocity_nonfinite_count\": "
        << report.velocity_nonfinite_count
        << ",\n      \"all_velocities_finite\": ";
    writePhase94Bool(out, report.all_velocities_finite);
    out << ",\n      \"exact_retained_key_alignment\": ";
    writePhase94Bool(out, report.exact_retained_key_alignment);
    out << ",\n      \"gnss_first_progress\": ";
    writePhase94Bool(out, report.gnss_first_progress);
    out << ",\n      \"c0d_factor_enabled\": ";
    writePhase94Bool(out, report.c0d_factor_enabled);
    out << ",\n      \"meter_state_parity_enabled\": ";
    writePhase94Bool(out, report.meter_state_parity_enabled);
    out << ",\n      \"c0d_factor_count\": " << report.c0d_factor_count
        << ",\n      \"active_solve_attempted\": ";
    writePhase94Bool(out, report.active_solve_attempted);
    out << ",\n      \"accepted_outer_iterations\": "
        << report.accepted_outer_iterations
        << ",\n      \"inner_lambda_attempts\": "
        << report.inner_lambda_attempts
        << ",\n      \"initial_cost\": ";
    writePhase94Double(out, report.initial_cost);
    out << ",\n      \"final_cost\": ";
    writePhase94Double(out, report.final_cost);
    out << ",\n      \"active_solve_costs_finite\": ";
    writePhase94Bool(out, report.active_solve_costs_finite);
    out << ",\n      \"final_cost_strictly_less_than_initial\": ";
    writePhase94Bool(out, report.final_cost_strictly_less_than_initial);
    out << ",\n      \"initial_lambda\": ";
    writePhase94Double(out, report.initial_lambda);
    out << ",\n      \"maximum_lambda\": ";
    writePhase94Double(out, report.maximum_lambda);
    out << ",\n      \"final_lambda\": ";
    writePhase94Double(out, report.final_lambda);
    out << ",\n      \"conditioning_proxy\": ";
    writePhase94Double(out, report.conditioning_proxy);
    out << ",\n      \"termination_trace_complete\": ";
    writePhase94Bool(out, report.termination_trace_complete);
    out << ",\n      \"terminal_branch\": ";
    writeJsonString(out, report.terminal_branch);
    out << ",\n      \"contract_passed\": ";
    writePhase94Bool(out, report.contract_passed);
    out << "\n    }";
}

void writePhase96MainDiagnostics(
    std::ostringstream& out,
    const libgnss::FGOProcessor::FGOPhase96MainDiagnostics& report) {
    out << "{\n      \"enabled\": ";
    writePhase94Bool(out, report.enabled);
    out << ",\n      \"attempted\": ";
    writePhase94Bool(out, report.attempted);
    out << ",\n      \"graph_observed\": ";
    writePhase94Bool(out, report.graph_observed);
    out << ",\n      \"initial_linearization_observed\": ";
    writePhase94Bool(out, report.initial_linearization_observed);
    out << ",\n      \"trial_trace_complete\": ";
    writePhase94Bool(out, report.trial_trace_complete);
    out << ",\n      \"trial_limit\": " << report.trial_limit
        << ",\n      \"graph_factor_count\": " << report.graph_factor_count
        << ",\n      \"graph_value_count\": " << report.graph_value_count
        << ",\n      \"graph_initial_cost\": ";
    writePhase94Double(out, report.graph_initial_cost);
    out << ",\n      \"initial_cost\": ";
    writePhase94Double(out, report.initial_cost);
    out << ",\n      \"final_cost\": ";
    writePhase94Double(out, report.final_cost);
    out << ",\n      \"accepted_outer_iterations\": "
        << report.accepted_outer_iterations
        << ",\n      \"terminal_branch\": ";
    writeJsonString(out, report.terminal_branch);
    out << ",\n      \"factor_family_cost_sum\": ";
    writePhase94Double(out, report.factor_family_cost_sum);
    out << ",\n      \"factor_families\": [";
    for (std::size_t i = 0; i < report.factor_families.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& family = report.factor_families[i];
        out << "{\"family\": ";
        writeJsonString(out, family.family);
        out << ", \"factor_count\": " << family.factor_count
            << ", \"finite_factor_count\": "
            << family.finite_factor_count
            << ", \"nonfinite_factor_count\": "
            << family.nonfinite_factor_count
            << ", \"initial_cost\": ";
        writePhase94Double(out, family.initial_cost);
        out << "}";
    }
    out << "],\n      \"variable_family_norms\": [";
    for (std::size_t i = 0; i < report.variable_family_norms.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& norm = report.variable_family_norms[i];
        out << "{\"family\": ";
        writeJsonString(out, norm.family);
        out << ", \"variable_bucket\": ";
        writeJsonString(out, norm.variable_bucket);
        out << ", \"contribution_count\": " << norm.contribution_count
            << ", \"finite_contribution_count\": "
            << norm.finite_contribution_count
            << ", \"nonfinite_contribution_count\": "
            << norm.nonfinite_contribution_count
            << ", \"gradient_l2_norm\": ";
        writePhase94Double(out, norm.gradient_l2_norm);
        out << ", \"normal_diagonal_l2_norm\": ";
        writePhase94Double(out, norm.normal_diagonal_l2_norm);
        out << ", \"normal_diagonal_min\": ";
        writePhase94Double(out, norm.normal_diagonal_min);
        out << ", \"normal_diagonal_max\": ";
        writePhase94Double(out, norm.normal_diagonal_max);
        out << "}";
    }
    out << "],\n      \"lm_trials\": [";
    for (std::size_t i = 0; i < report.lm_trials.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& trial = report.lm_trials[i];
        out << "{\"trial_index\": " << trial.trial_index
            << ", \"outer_iteration\": " << trial.outer_iteration
            << ", \"lambda\": ";
        writePhase94Double(out, trial.lambda);
        out << ", \"old_linearized_cost\": ";
        writePhase94Double(out, trial.old_linearized_cost);
        out << ", \"new_linearized_cost\": ";
        writePhase94Double(out, trial.new_linearized_cost);
        out << ", \"predicted_reduction\": ";
        writePhase94Double(out, trial.predicted_reduction);
        out << ", \"candidate_nonlinear_cost\": ";
        writePhase94Double(out, trial.candidate_nonlinear_cost);
        out << ", \"actual_reduction\": ";
        writePhase94Double(out, trial.actual_reduction);
        out << ", \"model_fidelity\": ";
        writePhase94Double(out, trial.model_fidelity);
        out << ", \"candidate_finite\": ";
        writePhase94Bool(out, trial.candidate_finite);
        out << ", \"linear_system_solved\": ";
        writePhase94Bool(out, trial.linear_system_solved);
        out << ", \"linear_system_status\": ";
        writeJsonString(out, trial.linear_system_status);
        out << ", \"rejection_reason\": ";
        writeJsonString(out, trial.rejection_reason);
        out << "}";
    }
    out << "],\n      \"exceptions\": [";
    for (std::size_t i = 0; i < report.exceptions.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& exception = report.exceptions[i];
        out << "{\"stage\": ";
        writeJsonString(out, exception.stage);
        out << ", \"classification\": ";
        writeJsonString(out, exception.classification);
        out << ", \"type\": ";
        writeJsonString(out, exception.type);
        out << ", \"message\": ";
        writeJsonString(out, exception.message);
        out << ", \"count\": " << exception.count << "}";
    }
    out << "]\n    }";
}

void writePhase97KeyReference(
    std::ostringstream& out,
    const libgnss::FGOProcessor::FGOPhase97KeyReference& key) {
    out << "{\"numeric_key\": " << key.numeric_key
        << ", \"symbol_character\": ";
    writeJsonString(out, std::string(1, key.symbol_character));
    out << ", \"symbol_index\": " << key.symbol_index << "}";
}

void writePhase97DoubleArray(std::ostringstream& out,
                             const std::vector<double>& values) {
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0U) out << ", ";
        writePhase94Double(out, values[i]);
    }
    out << "]";
}

void writePhase97MainDiagnostics(
    std::ostringstream& out,
    const libgnss::FGOProcessor::FGOPhase97MainDiagnostics& report) {
    out << "{\n      \"enabled\": ";
    writePhase94Bool(out, report.enabled);
    out << ",\n      \"attempted\": ";
    writePhase94Bool(out, report.attempted);
    out << ",\n      \"graph_observed\": ";
    writePhase94Bool(out, report.graph_observed);
    out << ",\n      \"initial_linearization_observed\": ";
    writePhase94Bool(out, report.initial_linearization_observed);
    out << ",\n      \"diagnostic_complete\": ";
    writePhase94Bool(out, report.diagnostic_complete);
    out << ",\n      \"graph_factor_count\": " << report.graph_factor_count
        << ",\n      \"graph_value_count\": " << report.graph_value_count
        << ",\n      \"graph_value_dimension\": " << report.graph_value_dimension
        << ",\n      \"graph_initial_cost\": ";
    writePhase94Double(out, report.graph_initial_cost);
    out << ",\n      \"missing_factor_key_count\": "
        << report.missing_factor_key_count
        << ",\n      \"duplicate_key_reference_count\": "
        << report.duplicate_key_reference_count
        << ",\n      \"value_type_mismatch_count\": "
        << report.value_type_mismatch_count
        << ",\n      \"empty_factor_count\": " << report.empty_factor_count
        << ",\n      \"isolated_value_key_count\": "
        << report.isolated_value_key_count
        << ",\n      \"connected_component_count\": "
        << report.connected_component_count
        << ",\n      \"exact_zero_column_count\": "
        << report.exact_zero_column_count
        << ",\n      \"near_zero_column_count\": "
        << report.near_zero_column_count
        << ",\n      \"near_zero_threshold\": "
        << std::setprecision(17) << report.near_zero_threshold
        << ",\n      \"initial_cost\": ";
    writePhase94Double(out, report.initial_cost);
    out << ",\n      \"final_cost\": ";
    writePhase94Double(out, report.final_cost);
    out << ",\n      \"accepted_outer_iterations\": "
        << report.accepted_outer_iterations
        << ",\n      \"trial_limit\": " << report.trial_limit
        << ",\n      \"trial_trace_complete\": ";
    writePhase94Bool(out, report.trial_trace_complete);
    out << ",\n      \"terminal_branch\": ";
    writeJsonString(out, report.terminal_branch);
    out << ",\n      \"ordering_context\": ";
    writeJsonString(out, report.ordering_context);
    out << ",\n      \"missing_factor_keys\": [";
    for (std::size_t i = 0; i < report.missing_factor_keys.size(); ++i) {
        if (i != 0U) out << ", ";
        writePhase97KeyReference(out, report.missing_factor_keys[i]);
    }
    out << "],\n      \"duplicate_key_reference_keys\": [";
    for (std::size_t i = 0; i < report.duplicate_key_reference_keys.size(); ++i) {
        if (i != 0U) out << ", ";
        writePhase97KeyReference(out, report.duplicate_key_reference_keys[i]);
    }
    out << "],\n      \"value_type_mismatch_keys\": [";
    for (std::size_t i = 0; i < report.value_type_mismatch_keys.size(); ++i) {
        if (i != 0U) out << ", ";
        writePhase97KeyReference(out, report.value_type_mismatch_keys[i]);
    }
    out << "],\n      \"nearby_variable_capture_status\": ";
    writeJsonString(out, report.nearby_variable_capture_status);
    out << ",\n      \"nearby_variables\": [";
    for (std::size_t i = 0; i < report.nearby_variables.size(); ++i) {
        if (i != 0U) out << ", ";
        writePhase97KeyReference(out, report.nearby_variables[i]);
    }
    out << "],\n      \"keys\": [";
    for (std::size_t i = 0; i < report.keys.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& key = report.keys[i];
        out << "{\"key\": ";
        writePhase97KeyReference(out, key.key);
        out << ", \"variable_bucket\": ";
        writeJsonString(out, key.variable_bucket);
        out << ", \"value_type\": ";
        writeJsonString(out, key.value_type);
        out << ", \"value_dimension\": " << key.value_dimension
            << ", \"value_present\": ";
        writePhase94Bool(out, key.value_present);
        out << ", \"factor_degree\": " << key.factor_degree
            << ", \"prior_factor_degree\": " << key.prior_factor_degree
            << ", \"component_id\": " << key.component_id
            << ", \"linearized_contribution_count\": "
            << key.linearized_contribution_count
            << ", \"finite_linearized_contribution_count\": "
            << key.finite_linearized_contribution_count
            << ", \"nonfinite_linearized_contribution_count\": "
            << key.nonfinite_linearized_contribution_count
            << ", \"gradient_l2_norm\": ";
        writePhase94Double(out, key.gradient_l2_norm);
        out << ", \"normal_diagonal_l2_norm\": ";
        writePhase94Double(out, key.normal_diagonal_l2_norm);
        out << ", \"normal_diagonal_min\": ";
        writePhase94Double(out, key.normal_diagonal_min);
        out << ", \"normal_diagonal_max\": ";
        writePhase94Double(out, key.normal_diagonal_max);
        out << ", \"exact_zero_normal_diagonal_count\": "
            << key.exact_zero_normal_diagonal_count
            << ", \"near_zero_normal_diagonal_count\": "
            << key.near_zero_normal_diagonal_count
            << ", \"exact_zero_column\": ";
        writePhase94Bool(out, key.exact_zero_column);
        out << ", \"near_zero_column\": ";
        writePhase94Bool(out, key.near_zero_column);
        out << ", \"gradient\": ";
        writePhase97DoubleArray(out, key.gradient);
        out << ", \"normal_diagonal\": ";
        writePhase97DoubleArray(out, key.normal_diagonal);
        out << ", \"family_degrees\": [";
        for (std::size_t j = 0; j < key.family_degrees.size(); ++j) {
            if (j != 0U) out << ", ";
            out << "{\"family\": ";
            writeJsonString(out, key.family_degrees[j].family);
            out << ", \"factor_degree\": "
                << key.family_degrees[j].factor_degree << "}";
        }
        out << "]}";
    }
    out << "],\n      \"factors\": [";
    for (std::size_t i = 0; i < report.factors.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& factor = report.factors[i];
        out << "{\"graph_index\": " << factor.graph_index
            << ", \"concrete_factor_type\": ";
        writeJsonString(out, factor.concrete_factor_type);
        out << ", \"family\": ";
        writeJsonString(out, factor.family);
        out << ", \"factor_dimension\": " << factor.factor_dimension
            << ", \"finite_error\": ";
        writePhase94Bool(out, factor.finite_error);
        out << ", \"is_prior_or_anchor\": ";
        writePhase94Bool(out, factor.is_prior_or_anchor);
        out << ", \"keys\": [";
        for (std::size_t j = 0; j < factor.keys.size(); ++j) {
            if (j != 0U) out << ", ";
            writePhase97KeyReference(out, factor.keys[j]);
        }
        out << "]}";
    }
    out << "],\n      \"components\": [";
    for (std::size_t i = 0; i < report.components.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& component = report.components[i];
        out << "{\"component_id\": " << component.component_id
            << ", \"key_count\": " << component.key_count
            << ", \"factor_count\": " << component.factor_count
            << ", \"anchored\": ";
        writePhase94Bool(out, component.anchored);
        out << ", \"keys\": [";
        for (std::size_t j = 0; j < component.keys.size(); ++j) {
            if (j != 0U) out << ", ";
            writePhase97KeyReference(out, component.keys[j]);
        }
        out << "]}";
    }
    out << "],\n      \"rank_decompositions\": [";
    for (std::size_t i = 0; i < report.rank_decompositions.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& rank = report.rank_decompositions[i];
        out << "{\"attempted\": ";
        writePhase94Bool(out, rank.attempted);
        out << ", \"rank_known\": ";
        writePhase94Bool(out, rank.rank_known);
        out << ", \"nullity_known\": ";
        writePhase94Bool(out, rank.nullity_known);
        out << ", \"finite\": ";
        writePhase94Bool(out, rank.finite);
        out << ", \"row_count\": " << rank.row_count
            << ", \"column_count\": " << rank.column_count
            << ", \"rank\": " << rank.rank
            << ", \"nullity\": " << rank.nullity
            << ", \"method\": ";
        writeJsonString(out, rank.method);
        out << ", \"status\": ";
        writeJsonString(out, rank.status);
        out << ", \"threshold\": ";
        writePhase94Double(out, rank.threshold);
        out << ", \"nullspace_attribution\": [";
        for (std::size_t j = 0; j < rank.nullspace_attribution.size(); ++j) {
            if (j != 0U) out << ", ";
            writePhase97KeyReference(out, rank.nullspace_attribution[j]);
        }
        out << "]}";
    }
    out << "],\n      \"lm_trials\": [";
    for (std::size_t i = 0; i < report.lm_trials.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& trial = report.lm_trials[i];
        out << "{\"trial_index\": " << trial.trial_index
            << ", \"outer_iteration\": " << trial.outer_iteration
            << ", \"lambda\": ";
        writePhase94Double(out, trial.lambda);
        out << ", \"old_linearized_cost\": ";
        writePhase94Double(out, trial.old_linearized_cost);
        out << ", \"new_linearized_cost\": ";
        writePhase94Double(out, trial.new_linearized_cost);
        out << ", \"predicted_reduction\": ";
        writePhase94Double(out, trial.predicted_reduction);
        out << ", \"candidate_nonlinear_cost\": ";
        writePhase94Double(out, trial.candidate_nonlinear_cost);
        out << ", \"actual_reduction\": ";
        writePhase94Double(out, trial.actual_reduction);
        out << ", \"model_fidelity\": ";
        writePhase94Double(out, trial.model_fidelity);
        out << ", \"candidate_finite\": ";
        writePhase94Bool(out, trial.candidate_finite);
        out << ", \"linear_system_solved\": ";
        writePhase94Bool(out, trial.linear_system_solved);
        out << ", \"linear_system_status\": ";
        writeJsonString(out, trial.linear_system_status);
        out << ", \"rejection_reason\": ";
        writeJsonString(out, trial.rejection_reason);
        out << ", \"nearby_variable_available\": ";
        writePhase94Bool(out, trial.nearby_variable_available);
        out << ", \"nearby_variable\": ";
        if (trial.nearby_variable_available) {
            writePhase97KeyReference(out, trial.nearby_variable);
        } else {
            out << "null";
        }
        out << ", \"nearby_variable_status\": ";
        writeJsonString(out, trial.nearby_variable_status);
        out << "}";
    }
    out << "],\n      \"exceptions\": [";
    for (std::size_t i = 0; i < report.exceptions.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& exception = report.exceptions[i];
        out << "{\"stage\": ";
        writeJsonString(out, exception.stage);
        out << ", \"classification\": ";
        writeJsonString(out, exception.classification);
        out << ", \"type\": ";
        writeJsonString(out, exception.type);
        out << ", \"message\": ";
        writeJsonString(out, exception.message);
        out << ", \"count\": " << exception.count << "}";
    }
    out << "]\n    }";
}

void writePhase98SolverDiagnostics(
    std::ostringstream& out,
    const libgnss::FGOProcessor::FGOPhase98SolverDiagnostics& report) {
    out << "{\n      \"enabled\": ";
    writePhase94Bool(out, report.enabled);
    out << ",\n      \"attempted\": ";
    writePhase94Bool(out, report.attempted);
    out << ",\n      \"exception_captured\": ";
    writePhase94Bool(out, report.exception_captured);
    out << ",\n      \"solver_type\": ";
    writeJsonString(out, report.solver_type);
    out << ",\n      \"solver_branch\": ";
    writeJsonString(out, report.solver_branch);
    out << ",\n      \"elimination_function\": ";
    writeJsonString(out, report.elimination_function);
    out << ",\n      \"ordering_type\": ";
    writeJsonString(out, report.ordering_type);
    out << ",\n      \"explicit_ordering_present\": ";
    writePhase94Bool(out, report.explicit_ordering_present);
    out << ",\n      \"ordering_size\": " << report.ordering_size
        << ",\n      \"ordering_digest\": ";
    writeJsonString(out, report.ordering_digest);
    out << ",\n      \"diagonal_damping\": ";
    writePhase94Bool(out, report.diagonal_damping);
    out << ",\n      \"existing_lm_trial_limit\": "
        << report.existing_lm_trial_limit
        << ",\n      \"indeterminate_exceptions\": [";
    for (std::size_t i = 0; i < report.indeterminate_exceptions.size(); ++i) {
        if (i != 0U) out << ", ";
        const auto& exception = report.indeterminate_exceptions[i];
        out << "{\"stage\": ";
        writeJsonString(out, exception.stage);
        out << ", \"lambda\": ";
        writePhase94Double(out, exception.lambda);
        out << ", \"solver_type\": ";
        writeJsonString(out, exception.solver_type);
        out << ", \"solver_branch\": ";
        writeJsonString(out, exception.solver_branch);
        out << ", \"elimination_function\": ";
        writeJsonString(out, exception.elimination_function);
        out << ", \"ordering_type\": ";
        writeJsonString(out, exception.ordering_type);
        out << ", \"explicit_ordering_present\": ";
        writePhase94Bool(out, exception.explicit_ordering_present);
        out << ", \"ordering_size\": " << exception.ordering_size
            << ", \"ordering_digest\": ";
        writeJsonString(out, exception.ordering_digest);
        out << ", \"diagonal_damping\": ";
        writePhase94Bool(out, exception.diagonal_damping);
        out << ", \"nearby_variable_available\": ";
        writePhase94Bool(out, exception.nearby_variable_available);
        out << ", \"nearby_variable\": ";
        if (exception.nearby_variable_available) {
            writePhase97KeyReference(out, exception.nearby_variable);
        } else {
            out << "null";
        }
        out << ", \"nearby_variable_status\": ";
        writeJsonString(out, exception.nearby_variable_status);
        out << ", \"exception_type\": ";
        writeJsonString(out, exception.exception_type);
        out << ", \"exception_message\": ";
        writeJsonString(out, exception.exception_message);
        out << "}";
    }
    out << "]\n    }";
}

std::string makePhase94StageDiagnosticsJson(
    const Phase94StageDiagnostics& diagnostics) {
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\n  \"schema_version\": "
        << (diagnostics.phase98_enabled
                ? "\"smartphone-r5-phase98-solver-rank-diagnostic-telemetry.v1\",\n"
                : (diagnostics.phase97_enabled
                ? "\"smartphone-r5-phase97-singular-system-diagnostic-telemetry.v1\",\n"
                : (diagnostics.phase96_enabled
                       ? "\"smartphone-r5-phase96-main-c0d-diagnostic-telemetry.v1\",\n"
                       : "\"smartphone-r5-phase94-source-clock-c0d-stage-diagnostics.v1\",\n")))
        << "  \"phase\": "
        << (diagnostics.phase98_enabled
                ? 98
                : (diagnostics.phase97_enabled
                       ? 97
                       : (diagnostics.phase96_enabled ? 96 : 94)))
        << ",\n  \"candidate\": "
        << (diagnostics.phase98_enabled
                ? "\"phase98_solver_rank_boundary_diagnostic_telemetry\",\n"
                : (diagnostics.phase97_enabled
                       ? "\"phase97_singular_system_graph_key_rank_diagnostic_telemetry\",\n"
                       : (diagnostics.phase96_enabled
                              ? "\"phase96_main_c0d_factor_family_lm_trial_diagnostic_telemetry\",\n"
                              : "\"phase94_source_clock_c0d_stage_admission_and_failure_telemetry\",\n")))
        << "  \"dataset_id\": ";
    writeJsonString(out, diagnostics.dataset_id);
    out << ",\n  \"status\": ";
    writeJsonString(out, diagnostics.status);
    out << ",\n  \"failure_stage\": ";
    writeJsonString(out, diagnostics.failure_stage);
    out << ",\n  \"failure_reason\": ";
    writeJsonString(out, diagnostics.failure_reason);
    out << ",\n  \"exception_type\": ";
    writeJsonString(out, diagnostics.exception_type);
    out << ",\n  \"exception_message\": ";
    writeJsonString(out, diagnostics.exception_message);
    out << ",\n  \"solution_output_published\": ";
    writePhase94Bool(out, diagnostics.solution_output_published);
    out << ",\n  \"accuracy_output_published\": ";
    writePhase94Bool(out, diagnostics.accuracy_output_published);
    out << ",\n  \"truth_used\": ";
    writePhase94Bool(out, diagnostics.truth_used);
    out << ",\n  \"mat_used\": ";
    writePhase94Bool(out, diagnostics.mat_used);
    out << ",\n  \"kaggle_or_token_accessed\": ";
    writePhase94Bool(out, diagnostics.kaggle_or_token_accessed);
    out << ",\n  \"raw_truth_mat_kaggle_forbidden\": true,\n"
        << "  \"gnss_first_preflight\": ";
    writePhase94Preflight(out, diagnostics.gnss_first_preflight);
    out << ",\n  \"main_preflight\": ";
    writePhase94Preflight(out, diagnostics.main_preflight);
    out << ",\n  \"gnss_first\": ";
    writePhase94GnssFirst(out, diagnostics.gnss_first);
    out << ",\n  \"main\": ";
    writePhase94Main(out, diagnostics.main);
    if (diagnostics.phase96_enabled) {
        out << ",\n  \"phase96_main\": ";
        writePhase96MainDiagnostics(out, diagnostics.phase96_main);
    }
    if (diagnostics.phase97_enabled) {
        out << ",\n  \"phase97_main\": ";
        writePhase97MainDiagnostics(out, diagnostics.phase97_main);
    }
    if (diagnostics.phase98_enabled) {
        out << ",\n  \"phase98_solver\": ";
        writePhase98SolverDiagnostics(out, diagnostics.phase98_solver);
    }
    out << "\n}\n";
    return out.str();
}

bool writePhase94StageDiagnostics(const Options& options,
                                  const Phase94StageDiagnostics& diagnostics) {
    return atomicWrite(options.summary_path,
                       makePhase94StageDiagnosticsJson(diagnostics));
}

void phase94RecordFailure(Phase94StageDiagnostics& diagnostics,
                          const std::string& stage,
                          const std::string& reason,
                          const std::string& exception_type = {},
                          const std::string& exception_message = {}) {
    diagnostics.status = "failed-closed";
    diagnostics.failure_stage = stage;
    diagnostics.failure_reason = reason;
    diagnostics.exception_type = exception_type;
    diagnostics.exception_message = exception_message;
}

std::string makePhase180AndroidClockPreflightJson(
    const Options& options,
    const libgnss::AndroidGnssTimeAnchorLoadResult& anchor_result,
    const libgnss::AndroidGnssUtcGpsMappingLoadResult& mapping_result,
    const libgnss::AndroidImuCsvLoadResult& imu_result,
    bool mapping_attempted,
    std::size_t loaded_imu_samples) {
    const auto& mapping = mapping_result.mapping;
    const auto write_double = [](std::ostringstream& out, double value) {
        if (std::isfinite(value)) {
            out << std::setprecision(17) << value;
        } else {
            out << "null";
        }
    };
    std::ostringstream out;
    out << "{\n"
        << "  \"schema_version\": "
        << "\"smartphone-r5-phase180-android-clock-preflight.v1\",\n"
        << "  \"phase\": 180,\n"
        << "  \"dataset_id\": ";
    writeJsonString(out, options.dataset_id);
    out << ",\n"
        << "  \"status\": "
        << ((mapping_attempted ? mapping_result.ok : anchor_result.ok) &&
                    imu_result.ok && loaded_imu_samples > 0U
                ? "\"ok\""
                : "\"failed-closed\"")
        << ",\n"
        << "  \"selector\": \"native-phase180-android-clock-preflight\",\n"
        << "  \"solver_entered\": false,\n"
        << "  \"raw_observation_conversion\": false,\n"
        << "  \"navigation_read\": false,\n"
        << "  \"truth_mat_base_kaggle_reads\": 0,\n"
        << "  \"clock_contract\": {\n"
        << "    \"explicit_wall_clock_fallback\": "
        << (options.android_utc_wall_clock_fallback ? "true" : "false")
        << ",\n"
        << "    \"elapsed_anchor_ok\": "
        << (anchor_result.ok ? "true" : "false") << ",\n"
        << "    \"elapsed_anchor_raw_rows\": " << anchor_result.raw_rows << ",\n"
        << "    \"elapsed_anchor_unique_rows\": " << anchor_result.unique_anchors
        << ",\n"
        << "    \"elapsed_anchor_error\": ";
    writeJsonString(out, anchor_result.error);
    out << ",\n"
        << "    \"utc_gps_mapping_attempted\": "
        << (mapping_attempted ? "true" : "false") << ",\n"
        << "    \"utc_gps_mapping_ok\": "
        << (mapping_result.ok ? "true" : "false") << ",\n"
        << "    \"utc_gps_mapping_input_rows\": " << mapping_result.input_rows
        << ",\n"
        << "    \"utc_gps_mapping_raw_rows\": " << mapping_result.raw_rows << ",\n"
        << "    \"utc_gps_mapping_unique_anchors\": " << mapping.unique_anchors
        << ",\n"
        << "    \"utc_gps_mapping_hardware_clock_count_present\": "
        << (mapping.hardware_clock_count_field_present ? "true" : "false")
        << ",\n"
        << "    \"utc_gps_mapping_hardware_clock_count_constant\": "
        << (mapping.hardware_clock_count_constant ? "true" : "false") << ",\n"
        << "    \"utc_gps_mapping_drift_ppm\": ";
    write_double(out, mapping.drift_ppm);
    out << ",\n"
        << "    \"utc_gps_mapping_slope_ns_per_ms\": ";
    write_double(out, mapping.slope_nanos_per_ms);
    out << ",\n"
        << "    \"utc_gps_mapping_max_fit_residual_ms\": ";
    write_double(out, mapping.maximum_fit_residual_ms);
    out << ",\n"
        << "    \"utc_gps_mapping_max_anchor_gap_ms\": ";
    write_double(out, mapping.maximum_anchor_gap_ms);
    out << ",\n"
        << "    \"utc_gps_mapping_error\": ";
    writeJsonString(out, mapping_result.error);
    out << "\n  },\n"
        << "  \"imu_timestamp_load\": {\n"
        << "    \"ok\": " << (imu_result.ok ? "true" : "false") << ",\n"
        << "    \"total_rows\": " << imu_result.total_rows << ",\n"
        << "    \"accel_rows\": " << imu_result.accel_rows << ",\n"
        << "    \"gyro_rows\": " << imu_result.gyro_rows << ",\n"
        << "    \"paired_rows\": " << imu_result.paired_rows << ",\n"
        << "    \"omitted_rows\": " << imu_result.omitted_rows << ",\n"
        << "    \"elapsed_clock_preserved\": "
        << (imu_result.elapsed_clock_preserved ? "true" : "false") << ",\n"
        << "    \"gnss_elapsed_anchor_applied\": "
        << (imu_result.gnss_elapsed_anchor_applied ? "true" : "false") << ",\n"
        << "    \"utc_wall_clock_fallback_applied\": "
        << (imu_result.utc_wall_clock_fallback_applied ? "true" : "false")
        << ",\n"
        << "    \"utc_mapping_anchors\": " << imu_result.utc_mapping_anchors << ",\n"
        << "    \"utc_mapping_drift_ppm\": ";
    write_double(out, imu_result.utc_mapping_drift_ppm);
    out << ",\n"
        << "    \"utc_mapping_max_fit_residual_ms\": ";
    write_double(out, imu_result.utc_mapping_max_fit_residual_ms);
    out << ",\n"
        << "    \"error\": ";
    writeJsonString(out, imu_result.error);
    out << "\n  },\n"
        << "  \"loaded_imu_samples\": " << loaded_imu_samples << ",\n"
        << "  \"coordinate_values_published\": false\n"
        << "}\n";
    return out.str();
}

int runPhase180AndroidClockPreflight(const Options& options) {
    std::vector<libgnss::AndroidGnssTimeAnchor> anchors;
    const auto anchor_result =
        libgnss::loadAndroidGnssTimeAnchors(options.android_gnss_path, anchors);
    libgnss::AndroidGnssUtcGpsMapping mapping;
    libgnss::AndroidGnssUtcGpsMappingLoadResult mapping_result;
    const bool mapping_attempted = !anchor_result.ok;
    if (mapping_attempted) {
        mapping_result = libgnss::loadAndroidGnssUtcGpsMapping(
            options.android_gnss_path, mapping);
    } else {
        mapping_result.ok = true;
        mapping_result.mapping.valid = false;
    }

    libgnss::ImuSeries series;
    libgnss::AndroidImuCsvConfig imu_config;
    imu_config.require_gnss_elapsed_anchor = true;
    imu_config.allow_utc_wall_clock_fallback = mapping_attempted;
    const auto imu_result = libgnss::loadAndroidImuCsv(
        options.android_imu_path, series, imu_config, anchors,
        mapping_attempted ? &mapping : nullptr);
    if (!atomicWrite(
            options.summary_path,
            makePhase180AndroidClockPreflightJson(
                options, anchor_result, mapping_result, imu_result,
                mapping_attempted, series.samples.size()))) {
        std::cerr << "failed to write Phase180 clock preflight summary\n";
        return 1;
    }
    const bool ok = (mapping_attempted ? mapping_result.ok : anchor_result.ok) &&
                    imu_result.ok && !series.samples.empty();
    if (!ok) {
        std::cerr << "Phase180 Android clock preflight failed closed: "
                  << (!mapping_result.error.empty()
                          ? mapping_result.error
                          : (!imu_result.error.empty() ? imu_result.error
                                                        : anchor_result.error))
                  << "\n";
        return 1;
    }
    return 0;
}

void writeJsonSizeMap(std::ostringstream& out,
                      const std::map<std::string, std::size_t>& values) {
    out << "{";
    bool first = true;
    for (const auto& [key, value] : values) {
        if (!first) out << ", ";
        first = false;
        writeJsonString(out, key);
        out << ": " << value;
    }
    out << "}";
}

void writeJsonSourceMissTaxonomy(
    std::ostringstream& out,
    const std::map<libgnss::SignalType,
                   libgnss::source_pseudorange_miss_mask::SignalCounts>& values) {
    out << "{";
    bool first = true;
    for (const auto& [signal, counts] : values) {
        if (!first) out << ", ";
        first = false;
        const std::string signal_name = baseTelemetrySignalName(signal);
        writeJsonString(out, signal_name);
        out << ": {\"signal\": ";
        writeJsonString(out, signal_name);
        out << ", \"frequency_band\": ";
        writeJsonString(out, baseTelemetryFrequencyBand(signal));
        out << ", \"original_adopted_rows\": "
            << counts.original_adopted_rows
            << ", \"retained_finite_pc_rows\": "
            << counts.retained_finite_pc_rows
            << ", \"corrected_rows\": " << counts.corrected_rows
            << ", \"matched_exact_stream_rows\": "
            << counts.matched_exact_stream_rows
            << ", \"finite_correction_rows_among_matched\": "
            << counts.finite_correction_rows_among_matched
            << ", \"dropped_missing_exact_stream_rows\": "
            << counts.dropped_missing_exact_stream_rows
            << ", \"dropped_out_of_domain_rows\": "
            << counts.dropped_out_of_domain_rows
            << ", \"dropped_nonfinite_correction_rows\": "
            << counts.dropped_nonfinite_correction_rows
            << ", \"factor_count_consistent\": "
            << (counts.factor_count_consistent ? "true" : "false")
            << "}";
    }
    out << "}";
}

void writePhase116CarrierTdcpReport(
    std::ostringstream& out, const Phase116CarrierTdcpReport& report) {
    out << "{\n    \"enabled\": ";
    writePhase94Bool(out, report.enabled);
    out << ",\n    \"read_only\": ";
    writePhase94Bool(out, report.read_only);
    out << ",\n    \"ordinary_tdcp_only\": ";
    writePhase94Bool(out, report.ordinary_tdcp_only);
    out << ",\n    \"standalone_carrier_phase_factors\": ";
    writePhase94Bool(out, report.standalone_carrier_phase_factors);
    out << ",\n    \"double_difference_factors\": ";
    writePhase94Bool(out, report.double_difference_factors);
    out << ",\n    \"ambiguity_states\": ";
    writePhase94Bool(out, report.ambiguity_states);
    out << ",\n    \"pdc_state_bridge\": ";
    writePhase94Bool(out, report.pdc_state_bridge);
    out << ",\n    \"factors_built\": " << report.factors_built
        << ",\n    \"factors_inserted\": " << report.factors_inserted
        << ",\n    \"factors_inserted_exact\": ";
    writePhase94Bool(out, report.factors_inserted_exact);
    out << ",\n    \"sigma_m\": ";
    writePhase94Double(out, report.sigma_m);
    out << ",\n    \"graph_initial_cost\": ";
    writePhase94Double(out, report.graph_initial_cost);
    out << ",\n    \"graph_final_cost\": ";
    writePhase94Double(out, report.graph_final_cost);
    out << ",\n    \"cost_definition\": {\n"
        << "      \"unwhitened\": \"sum residual_m^2\",\n"
        << "      \"whitened\": \"sum (residual_m/sigma_m)^2\",\n"
        << "      \"robust\": \"GTSAM Huber loss on whitened residual\"\n"
        << "    },\n    \"signals\": {";
    bool first_signal = true;
    for (const auto& [signal, value] : report.signals) {
        if (!first_signal) out << ",";
        first_signal = false;
        writeJsonString(out, baseTelemetrySignalName(signal));
        out << ": {\n        \"signal\": ";
        writeJsonString(out, value.signal);
        out << ",\n        \"frequency_band\": ";
        writeJsonString(out, value.frequency_band);
        out << ",\n        \"carrier_rows_seen\": " << value.carrier_rows_seen
            << ",\n        \"carrier_phase_rows\": " << value.carrier_phase_rows
            << ",\n        \"retained_carrier_rows\": "
            << value.retained_carrier_rows
            << ",\n        \"missing_wavelength\": "
            << value.missing_wavelength
            << ",\n        \"nonfinite_measurements\": "
            << value.nonfinite_measurements
            << ",\n        \"candidate_pairs\": " << value.candidate_pairs
            << ",\n        \"accepted_pairs\": " << value.accepted_pairs
            << ",\n        \"rejected_gap\": " << value.rejected_gap
            << ",\n        \"rejected_clock_discontinuity\": "
            << value.rejected_clock_discontinuity
            << ",\n        \"rejected_missing_previous\": "
            << value.rejected_missing_previous
            << ",\n        \"rejected_loss_of_lock\": "
            << value.rejected_loss_of_lock
            << ",\n        \"rejected_nonfinite\": "
            << value.rejected_nonfinite
            << ",\n        \"rejected_code_phase_jump\": "
            << value.rejected_code_phase_jump
            << ",\n        \"rejected_invalid_weight\": "
            << value.rejected_invalid_weight
            << ",\n        \"sigma_m\": ";
        writePhase94Double(out, value.sigma_m);
        out << ",\n        \"initial_residual_count\": "
            << value.initial_residual_count
            << ",\n        \"final_residual_count\": "
            << value.final_residual_count
            << ",\n        \"initial_nonfinite_residual_count\": "
            << value.initial_nonfinite_residual_count
            << ",\n        \"final_nonfinite_residual_count\": "
            << value.final_nonfinite_residual_count
            << ",\n        \"initial_robust_outlier_count\": "
            << value.initial_robust_outlier_count
            << ",\n        \"final_robust_outlier_count\": "
            << value.final_robust_outlier_count
            << ",\n        \"initial_robust_cost\": ";
        writePhase94Double(out, value.initial_robust_cost);
        out << ",\n        \"final_robust_cost\": ";
        writePhase94Double(out, value.final_robust_cost);
        out << ",\n        \"initial_unwhitened_cost\": ";
        writePhase94Double(out, value.initial_unwhitened_cost);
        out << ",\n        \"final_unwhitened_cost\": ";
        writePhase94Double(out, value.final_unwhitened_cost);
        out << ",\n        \"initial_whitened_cost\": ";
        writePhase94Double(out, value.initial_whitened_cost);
        out << ",\n        \"final_whitened_cost\": ";
        writePhase94Double(out, value.final_whitened_cost);
        out << ",\n        \"connected_epoch_count\": "
            << value.connected_epoch_count
            << ",\n        \"connected_key_count\": "
            << value.connected_keys.size()
            << ",\n        \"connected_keys\": [";
        bool first_key = true;
        for (const auto& key : value.connected_keys) {
            if (!first_key) out << ", ";
            first_key = false;
            writeJsonString(out, key);
        }
        out << "]\n      }";
    }
    out << "}\n  }";
}

// Emit the Phase141 schema from native reports at the summary boundary.  This
// function is deliberately scalar-only: it never serializes Values, solution
// rows, coordinates, or measurements.  Every field is copied from the
// already-authoritative native report passed to this function; the structural
// wrapper validates this object and is not allowed to reconstruct it.
void writePhase141Telemetry(
    std::ostringstream& out, const Options& options,
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& result,
    const ImuBuildReport& imu_report, bool fallback,
    const RawUtcOutputReport& raw_utc_report,
    const UpstreamPositionOffsetReport& position_offset_report,
    const BasePseudorangeCompensationReport& base_report) {
    const auto& diagnostics = result.diagnostics;
    auto writeBool = [&out](bool value) {
        out << (value ? "true" : "false");
    };
    auto writeFamily = [&](const char* admitted_name,
                           std::size_t admitted_rows,
                           std::size_t affine_rows, bool key_order_exact,
                           bool finite_values, bool same_geometry) {
        out << "{\n        \"admitted_rows\": " << admitted_rows
            << ",\n        \"affine_factors_inserted\": " << affine_rows
            << ",\n        \"key_order_exact\": ";
        writeBool(key_order_exact);
        out << ",\n        \"finite_values\": ";
        writeBool(finite_values);
        out << ",\n        \"source_geometry_same_path\": ";
        writeBool(same_geometry);
        out << ",\n        \"source_report\": ";
        writeJsonString(out, admitted_name);
        out << "\n      }";
    };
    auto writeStage = [&](const char* stage, bool attempted,
                          std::size_t accepted_iterations, double initial_cost,
                          double final_cost, bool costs_finite,
                          bool strict_cost_decrease, const std::string& terminal,
                          bool no_fallback) {
        out << "\"" << stage << "\": {\n        \"attempted\": ";
        writeBool(attempted);
        out << ",\n        \"accepted_iterations\": "
            << accepted_iterations << ",\n        \"initial_cost\": ";
        writePhase94Double(out, initial_cost);
        out << ",\n        \"final_cost\": ";
        writePhase94Double(out, final_cost);
        out << ",\n        \"costs_finite\": ";
        writeBool(costs_finite);
        out << ",\n        \"strict_cost_decrease\": ";
        writeBool(strict_cost_decrease);
        out << ",\n        \"terminal_branch\": ";
        writeJsonString(out, terminal);
        out << ",\n        \"no_fallback\": ";
        writeBool(no_fallback);
        out << "\n      }";
    };

    std::size_t finite_positions = 0U;
    std::size_t earth_valid_positions = 0U;
    for (const auto& solution : result.solution.solutions) {
        if (solution.position_ecef.allFinite()) {
            ++finite_positions;
            if (earthValidEcef(solution.position_ecef)) {
                ++earth_valid_positions;
            }
        }
    }
    const std::size_t position_epochs = result.solution.solutions.size();
    const bool output_finite = finite_positions == position_epochs;
    const bool output_earth_valid =
        output_finite && earth_valid_positions == problem.epochs.size() &&
        position_epochs == problem.epochs.size();
    const bool output_expected_coverage =
        position_epochs == problem.epochs.size() &&
        (!raw_utc_report.enabled ||
         (raw_utc_report.unresolved_epochs == 0U &&
          raw_utc_report.target_epochs == raw_utc_report.exact_solution_epochs));
    const bool gnss_costs_finite =
        std::isfinite(imu_report.gnss_first_initial_cost) &&
        std::isfinite(imu_report.gnss_first_final_cost) &&
        imu_report.gnss_first_c0d_active_solve_finite_costs;
    const bool gnss_strict_cost_decrease =
        gnss_costs_finite &&
        imu_report.gnss_first_final_cost < imu_report.gnss_first_initial_cost;
    const bool main_costs_finite = std::isfinite(diagnostics.initial_cost) &&
                                   std::isfinite(diagnostics.final_cost) &&
                                   diagnostics
                                       .native_source_clock_c0d_active_solve_finite_costs;
    const bool main_strict_cost_decrease =
        main_costs_finite && diagnostics.final_cost < diagnostics.initial_cost;
    const bool phase118_recipe_unchanged =
        diagnostics.official_tdcp_huber_k_enabled &&
        !options.native_phase117_tdcp_snr_type_sigma &&
        !options.native_phase120_official_tdcp_resl_atmosphere_cancellation;
    const std::size_t c_epoch_count = result.epoch_clock_bias_components_m.size();
    const std::size_t d_epoch_count = result.epoch_clock_drift_mps.size();
    std::size_t c_finite_count = 0U;
    for (const auto& value : result.epoch_clock_bias_components_m) {
        if (std::all_of(value.begin(), value.end(),
                        [](double component) { return std::isfinite(component); })) {
            ++c_finite_count;
        }
    }
    std::size_t d_finite_count = 0U;
    for (const double value : result.epoch_clock_drift_mps) {
        if (std::isfinite(value)) ++d_finite_count;
    }

    out << "{\n"
        << "    \"schema_version\": \""
        << (options.native_phase144_telemetry_schema
                ? "smartphone-r5-native-fgo-phase144-telemetry.v1"
                : "smartphone-r5-native-fgo-phase141-telemetry.v1")
        << "\",\n"
        << "    \"enabled\": true,\n"
        << "    \"sync_count\": 1,\n"
        << "    \"authority\": {\n"
        << "      \"gnss_first\": \"ImuBuildReport+GNSS-first-FGOResult.diagnostics\",\n"
        << "      \"main\": \"FGOResult.diagnostics+FGOProblem\",\n"
        << "      \"raw_base\": \"BasePseudorangeCompensationReport\",\n"
        << "      \"offset\": \"UpstreamPositionOffsetReport\",\n"
        << "      \"output\": \"native-output-boundary\",\n"
        << "      \"wrapper_read_accounting\": \"independent-wrapper-observation\"\n"
        << "    },\n"
        << "    \"selectors\": {\n"
        << "      \"phase135_official_affine_measurement_family\": ";
    writeBool(options.native_phase135_official_affine_measurement_family);
    out << ",\n      \"phase138_affine_tdcp_anchor_range_constant\": ";
    writeBool(options.native_phase138_affine_tdcp_anchor_range_constant);
    out << ",\n      \"phase118_official_tdcp_huber_k\": ";
    writeBool(options.native_phase118_official_tdcp_huber_k);
    out << ",\n      \"phase184_source_tdcp_huber_k\": ";
    writeBool(options.native_phase184_source_tdcp_huber_k);
    out << ",\n      \"phase117_dynamic_tdcp_sigma\": ";
    writeBool(options.native_phase117_tdcp_snr_type_sigma);
    out << ",\n      \"phase120_official_tdcp_resl_atmosphere_cancellation\": ";
    writeBool(options.native_phase120_official_tdcp_resl_atmosphere_cancellation);
    out << ",\n      \"phase126_raw_base_source_complete\": ";
    writeBool(options.native_phase126_raw_base_source_complete);
    out << ",\n      \"native_paired_epoch_states\": ";
    writeBool(options.native_paired_epoch_states);
    out << ",\n      \"native_rover_epoch_states\": ";
    writeBool(options.native_rover_epoch_states);
    out << ",\n      \"native_dense_base_smoothing\": ";
    writeBool(options.native_dense_base_smoothing);
    out << ",\n      \"native_base_mask_only_ablation\": ";
    writeBool(options.native_base_mask_only_ablation);
    out << ",\n      \"native_base_gps_values_only_ablation\": ";
    writeBool(options.native_base_gps_values_only_ablation);
    out << ",\n      \"native_base_gps_center_ablation\": ";
    writeBool(options.native_base_gps_center_ablation);
    out << ",\n      \"native_main_p_cauchy\": ";
    writeBool(options.native_main_p_cauchy);
    out << ",\n      \"phase127_glonass_channel_provenance\": ";
    writeBool(options.native_phase127_glonass_channel_provenance);
    out << ",\n      \"phase128_glonass_provenance_parser_admission\": ";
    writeBool(options.native_phase128_glonass_provenance_parser_admission);
    out << ",\n      \"phase129_glonass_local_miss_mask\": ";
    writeBool(options.native_phase129_glonass_local_miss_mask);
    out << ",\n      \"phase130_shared_ledger_key_local_support\": false"
        << ",\n      \"phase131_canonical_correction_band_key\": ";
    writeBool(options.native_phase131_canonical_correction_band_key);
    out << ",\n      \"phase107_raw_base_compensation\": ";
    writeBool(options.native_base_pseudorange_compensation);
    out << ",\n      \"phase107_raw_base_source_miss_mask\": ";
    writeBool(options.native_base_pseudorange_source_miss_mask);
    out << ",\n      \"phase107_preserve_additional_frequency_bands\": ";
    writeBool(options.native_base_pseudorange_preserve_additional_frequency_bands);
    out << ",\n      \"phase143_official_main_lm_termination_budget\": ";
    writeBool(options.native_phase143_official_main_lm_termination_budget);
    out << ",\n      \""
        << (options.native_phase144_telemetry_schema
                ? "phase144_telemetry_schema"
                : "phase141_telemetry_schema")
        << "\": true\n"
        << "    },\n"
        << "    \"equation\": {\n"
        << "      \"semantic_id\": \"phase138-affine-tdcp-anchor-range-constant-v1\",\n"
        << "      \"representation\": "
           << (options.native_phase144_telemetry_schema
                   ? "\"full-assignment\""
                   : "\"rhs-only-native-diagnostic\"")
           << ",\n"
        << "      \"display_expression\": ";
    writeJsonString(out, options.native_phase144_telemetry_schema
                            ? kPhase138EquationDisplay
                            : diagnostics.phase138_measurement_equation);
    out << ",\n      \"ast\": {\n"
        << "        \"kind\": \"assign\",\n"
        << "        \"lhs\": \"tdcp_phase138\",\n"
        << "        \"rhs\": {\"kind\": \"sub\", \"left\": \"tdcp_native\", \"right\": {\"kind\": \"sub\", \"left\": \"rho_current_initial\", \"right\": \"rho_previous_initial\"}}\n"
        << "      },\n"
        << "      \"token_tuple\": [\"ASSIGN\", \"tdcp_phase138\", \"SUB\", \"GROUP_OPEN\", \"tdcp_native\", \"SUB\", \"GROUP_OPEN\", \"rho_current_initial\", \"SUB\", \"rho_previous_initial\", \"GROUP_CLOSE\", \"GROUP_CLOSE\"],\n"
        << "      \"rhs_only_token_tuple\": [\"tdcp_native\", \"SUB\", \"GROUP_OPEN\", \"rho_current_initial\", \"SUB\", \"rho_previous_initial\", \"GROUP_CLOSE\"],\n"
        << "      \"whitespace_normalization_only\": true\n"
        << "    },\n"
        << "    \"phase135\": {\n"
        << "      \"enabled\": ";
    writeBool(diagnostics.phase135_official_affine_measurement_family_enabled);
    out << ",\n      \"configuration_valid\": ";
    writeBool(diagnostics.phase135_configuration_valid);
    out << ",\n      \"transactional\": ";
    writeBool(diagnostics.phase135_transactional);
    out << ",\n      \"fixed_initial_geometry\": ";
    writeBool(diagnostics.phase135_fixed_initial_geometry);
    out << ",\n      \"finite_jacobians\": ";
    writeBool(diagnostics.phase135_finite_jacobians);
    out << ",\n      \"single_sagnac_representation\": ";
    writeBool(diagnostics.phase135_single_sagnac_representation);
    out << ",\n      \"los_convention\": \"-e=(receiver-satellite)/range\",\n"
        << "      \"geometry_rows\": "
        << diagnostics.phase135_geometry_rows_validated
        << ",\n      \"sagnac_evaluations\": "
        << diagnostics.phase135_geometry_rows_validated
        << ",\n      \"pseudorange\": ";
    writeFamily("FGOProblem.pseudorange_factors",
                problem.pseudorange_factors.size(),
                diagnostics.phase135_pseudorange_factors_inserted,
                diagnostics.phase135_pseudorange_key_order_exact,
                diagnostics.phase135_pseudorange_finite_values,
                diagnostics.phase135_pseudorange_source_geometry_same_path);
    out << ",\n      \"doppler\": ";
    writeFamily("FGOProblem.undifferenced_doppler_factors",
                problem.undifferenced_doppler_factors.size(),
                diagnostics.phase135_doppler_factors_inserted,
                diagnostics.phase135_doppler_key_order_exact,
                diagnostics.phase135_doppler_finite_values,
                diagnostics.phase135_doppler_source_geometry_same_path);
    out << ",\n      \"ordinary_tdcp\": ";
    writeFamily("FGOProblem.tdcp_factors", problem.tdcp_factors.size(),
                diagnostics.phase135_tdcp_factors_inserted,
                diagnostics.phase135_tdcp_key_order_exact,
                diagnostics.phase135_tdcp_finite_values,
                diagnostics.phase135_tdcp_source_geometry_same_path);
    out << ",\n      \"legacy_factor_counts\": {\n"
        << "        \"pseudorange\": "
        << diagnostics.phase135_legacy_pseudorange_factor_count << ",\n"
        << "        \"doppler\": "
        << diagnostics.phase135_legacy_doppler_factor_count << ",\n"
        << "        \"ordinary_tdcp\": "
        << diagnostics.phase135_legacy_tdcp_factor_count << "\n"
        << "      },\n"
        << "      \"pose3_x_bridge\": {\n"
        << "        \"count\": " << diagnostics.phase135_pose3_x_bridge_factors
        << ",\n        \"keys_exact\": ";
    writeBool(diagnostics.phase135_pose3_x_bridge_keys_exact);
    out << "\n      }\n    },\n"
        << "    \"phase138\": {\n"
        << "      \"enabled\": ";
    writeBool(diagnostics.phase138_affine_tdcp_anchor_range_constant_enabled);
    out << ",\n      \"phase135_dependency_satisfied\": ";
    writeBool(diagnostics.phase135_official_affine_measurement_family_enabled);
    out << ",\n      \"configuration_valid\": ";
    writeBool(diagnostics.phase138_configuration_valid);
    out << ",\n      \"transactional\": ";
    writeBool(diagnostics.phase138_transactional);
    out << ",\n      \"adjusted_exactly_once\": ";
    writeBool(diagnostics.phase138_adjusted_exactly_once);
    out << ",\n      \"factor_count_unchanged\": ";
    writeBool(diagnostics.phase138_factor_count_unchanged);
    out << ",\n      \"same_endpoint_epoch_and_satellite_state\": ";
    writeBool(diagnostics.phase138_same_endpoint_epoch_and_satellite_state);
    out << ",\n      \"same_satellite_state\": ";
    writeBool(diagnostics.phase138_same_satellite_state);
    out << ",\n      \"finite_adjusted_measurements\": ";
    writeBool(diagnostics.phase138_finite_adjusted_measurements);
    out << ",\n      \"no_raw_or_zero_fallback\": ";
    writeBool(diagnostics.phase138_no_raw_or_zero_fallback);
    out << ",\n      \"phase118_atmosphere_sigma_huber_unchanged\": ";
    writeBool(phase118_recipe_unchanged);
    out << ",\n      \"single_sagnac_representation\": ";
    writeBool(diagnostics.phase135_single_sagnac_representation);
    out << ",\n      \"measurement_equation\": ";
    writeJsonString(out, options.native_phase144_telemetry_schema
                            ? kPhase138EquationDisplay
                            : diagnostics.phase138_measurement_equation);
    out << ",\n      \"equation\": {\n"
        << "        \"semantic_id\": \"phase138-affine-tdcp-anchor-range-constant-v1\",\n"
        << "        \"representation\": "
           << (options.native_phase144_telemetry_schema
                   ? "\"full-assignment\""
                   : "\"rhs-only-native-diagnostic\"")
           << ",\n"
        << "        \"display_expression\": ";
    writeJsonString(out, options.native_phase144_telemetry_schema
                            ? kPhase138EquationDisplay
                            : diagnostics.phase138_measurement_equation);
    out << ",\n        \"ast\": {\n"
        << "          \"kind\": \"assign\",\n"
        << "          \"lhs\": \"tdcp_phase138\",\n"
        << "          \"rhs\": {\"kind\": \"sub\", \"left\": \"tdcp_native\", \"right\": {\"kind\": \"sub\", \"left\": \"rho_current_initial\", \"right\": \"rho_previous_initial\"}}\n"
        << "        },\n"
        << "        \"token_tuple\": [\"ASSIGN\", \"tdcp_phase138\", \"SUB\", \"GROUP_OPEN\", \"tdcp_native\", \"SUB\", \"GROUP_OPEN\", \"rho_current_initial\", \"SUB\", \"rho_previous_initial\", \"GROUP_CLOSE\", \"GROUP_CLOSE\"],\n"
        << "        \"rhs_only_token_tuple\": [\"tdcp_native\", \"SUB\", \"GROUP_OPEN\", \"rho_current_initial\", \"SUB\", \"rho_previous_initial\", \"GROUP_CLOSE\"],\n"
        << "        \"whitespace_normalization_only\": true\n"
        << "      },\n"
        << "      \"geometry_representation\": ";
    writeJsonString(out, diagnostics.phase138_geometry_representation);
    out << ",\n      \"range_constants_validated\": "
        << diagnostics.phase138_tdcp_range_constants_validated
        << ",\n      \"tdcp_measurements_adjusted\": "
        << diagnostics.phase138_tdcp_measurements_adjusted
        << ",\n      \"affine_tdcp_factor_count\": "
        << diagnostics.phase135_tdcp_factors_inserted
        << ",\n      \"adjustment_application_passes\": "
        << diagnostics.phase138_adjustment_application_passes
        << ",\n      \"legacy_tdcp_factor_count\": "
        << diagnostics.phase138_legacy_tdcp_factor_count << "\n"
        << "    },\n"
        << "    \"raw_base\": {\n"
        << "      \"phase107_recipe\": ";
    writeBool(options.native_base_pseudorange_compensation &&
              options.native_base_pseudorange_source_miss_mask &&
              !options.native_base_pseudorange_preserve_additional_frequency_bands &&
              !options.native_phase126_raw_base_source_complete &&
              !options.native_phase127_glonass_channel_provenance &&
              !options.native_phase128_glonass_provenance_parser_admission &&
              !options.native_phase129_glonass_local_miss_mask &&
              !options.native_phase131_canonical_correction_band_key);
    out << ",\n      \"applied_exactly_once\": ";
    writeBool(base_report.correction_applied_exactly_once);
    out << ",\n      \"source_miss_conservation\": ";
    writeBool(base_report.source_miss_conservation);
    out << ",\n      \"no_raw_or_zero_fallback\": ";
    writeBool(base_report.no_raw_or_zero_fallback);
    out << ",\n      \"correction_application_pass_count\": "
        << base_report.correction_application_pass_count
        << ",\n      \"original_adopted_pseudorange_rows\": "
        << base_report.original_adopted_pseudorange_rows
        << ",\n      \"retained_finite_pc_pseudorange_rows\": "
        << base_report.retained_finite_pc_pseudorange_rows
        << ",\n      \"corrected_rows\": " << base_report.corrected_rows
        << "\n    },\n"
        << "    \"clock\": {\n"
        << "      \"c_units\": \"metres\",\n"
        << "      \"d_units\": \"metres/second\",\n"
        << "      \"c7_mapping_exact\": ";
    writeBool(diagnostics.native_source_clock_c0d_epoch_vector_parity_enabled);
    out << ",\n      \"d_full_finite_exact_alignment\": ";
    writeBool(imu_report.phase91_raw_drift_d_initializer_alignment_valid &&
              d_epoch_count == problem.epochs.size() &&
              d_finite_count == d_epoch_count);
    out << ",\n      \"c_epoch_count\": " << c_epoch_count
        << ",\n      \"c_finite_count\": " << c_finite_count
        << ",\n      \"d_epoch_count\": " << d_epoch_count
        << ",\n      \"d_finite_count\": " << d_finite_count
        << ",\n      \"c7_dimension\": "
        << diagnostics.native_source_clock_c0d_epoch_vector_dimension
        << ",\n      \"c7_state_count\": "
        << diagnostics.native_source_clock_c0d_epoch_vector_state_count
        << ",\n      \"c7_handoff_count\": "
        << diagnostics.native_source_clock_c0d_epoch_vector_handoff_count
        << "\n    },\n"
        << "    \"solver\": {\n      \"linear_solver\": ";
    writeJsonString(out, diagnostics.selected_linear_solver_type);
    out << ",\n      \"elimination\": ";
    writeJsonString(out, diagnostics.selected_elimination_function);
    out << ",\n      ";
    writeStage("gnss_first", imu_report.gnss_first_attempted,
                imu_report.gnss_first_c0d_accepted_outer_iterations,
                imu_report.gnss_first_initial_cost,
                imu_report.gnss_first_final_cost, gnss_costs_finite,
                gnss_strict_cost_decrease, "native-gnss-first",
                !fallback);
    out << ",\n      ";
    writeStage("main", diagnostics.native_source_clock_c0d_active_solve_attempted,
                diagnostics.native_source_clock_c0d_accepted_outer_iterations,
                diagnostics.initial_cost, diagnostics.final_cost,
                main_costs_finite, main_strict_cost_decrease,
                diagnostics.native_source_clock_c0d_termination_branch_reason,
                !fallback);
    out << "\n    },\n"
        << "    \"factor_counts\": {\n"
        << "      \"pseudorange\": " << problem.pseudorange_factors.size()
        << ",\n      \"doppler\": "
        << problem.undifferenced_doppler_factors.size()
        << ",\n      \"ordinary_tdcp\": " << problem.tdcp_factors.size()
        << ",\n      \"legacy_pseudorange\": "
        << diagnostics.phase135_legacy_pseudorange_factor_count
        << ",\n      \"legacy_doppler\": "
        << diagnostics.phase135_legacy_doppler_factor_count
        << ",\n      \"legacy_tdcp\": "
        << diagnostics.phase135_legacy_tdcp_factor_count << "\n"
        << "    },\n"
        << "    \"bridge\": {\n"
        << "      \"pose3_x_count\": "
        << diagnostics.phase135_pose3_x_bridge_factors
        << ",\n      \"pose3_x_keys_exact\": ";
    writeBool(diagnostics.phase135_pose3_x_bridge_keys_exact);
    out << ",\n      \"phase131_sync_count\": "
        << problem.diagnostics.phase131_diagnostics_bridge_sync_count
        << "\n    },\n"
        << "    \"offset\": {\n"
        << "      \"enabled\": ";
    writeBool(position_offset_report.enabled);
    out << ",\n      \"applied\": ";
    writeBool(position_offset_report.applied);
    out << ",\n      \"application_passes\": "
        << position_offset_report.application_passes
        << ",\n      \"corrected_epochs\": "
        << position_offset_report.corrected_epochs << "\n    },\n"
        << "    \"output\": {\n"
        << "      \"finite\": ";
    writeBool(output_finite);
    out << ",\n      \"earth_valid\": ";
    writeBool(output_earth_valid);
    out << ",\n      \"expected_epoch_coverage\": ";
    writeBool(output_expected_coverage);
    out << ",\n      \"opaque_solution_seal\": true,\n"
        << "      \"coordinate_rows_interpreted\": false,\n"
        << "      \"pixel5_offset_applications\": "
        << position_offset_report.application_passes << "\n    },\n"
        << "    \"policy\": {\n"
        << "      \"truth_used\": false,\n"
        << "      \"coordinate_rows_interpreted\": false,\n"
        << "      \"solution_publication_authorized\": false,\n"
        << "      \"fallback\": ";
    writeBool(fallback);
    out << ",\n      \"rerun\": false\n    }\n"
        << "  }";
}

// Emit the Phase143 native-authoritative LM termination sidecar.  The source
// values are copied from FGOResult diagnostics for each optimizer stage; no
// generic result iteration field, return code, Values object, or coordinate
// row is used to reconstruct this report.
void writePhase143TerminationDiagnostics(
    std::ostringstream& out,
    const libgnss::FGOProcessor::FGOPhase143TerminationDiagnostics& report) {
    auto writeBool = [&out](bool value) {
        out << (value ? "true" : "false");
    };
    out << "{\n"
        << "      \"selector_enabled\": ";
    writeBool(report.selector_enabled);
    out << ",\n      \"stage\": ";
    writeJsonString(out, report.stage);
    out << ",\n      \"configured_max_iterations\": "
        << report.configured_max_iterations
        << ",\n      \"effective_max_iterations\": "
        << report.effective_max_iterations
        << ",\n      \"attempted\": ";
    writeBool(report.attempted);
    out << ",\n      \"attempted_outer_iterations\": "
        << report.attempted_outer_iterations
        << ",\n      \"accepted_outer_iterations\": "
        << report.accepted_outer_iterations
        << ",\n      \"rejected_outer_iterations\": "
        << report.rejected_outer_iterations
        << ",\n      \"total_inner_lambda_attempts\": "
        << report.total_inner_lambda_attempts
        << ",\n      \"parsed_trial_count\": "
        << report.parsed_trial_count
        << ",\n      \"native_inner_iterations\": "
        << report.native_inner_iterations
        << ",\n      \"expected_trial_count\": "
        << report.expected_trial_count
        << ",\n      \"initial_cost\": ";
    writePhase94Double(out, report.initial_cost);
    out << ",\n      \"final_cost\": ";
    writePhase94Double(out, report.final_cost);
    out << ",\n      \"costs_finite\": ";
    writeBool(report.costs_finite);
    out << ",\n      \"strict_cost_decrease\": ";
    writeBool(report.strict_cost_decrease);
    out << ",\n      \"termination_branch\": ";
    writeJsonString(out, report.termination_branch);
    out << ",\n      \"relative_error_tolerance\": ";
    writePhase94Double(out, report.relative_error_tolerance);
    out << ",\n      \"absolute_error_tolerance\": ";
    writePhase94Double(out, report.absolute_error_tolerance);
    out << ",\n      \"error_tolerance\": ";
    writePhase94Double(out, report.error_tolerance);
    out << ",\n      \"initial_lambda\": ";
    writePhase94Double(out, report.initial_lambda);
    out << ",\n      \"final_lambda\": ";
    writePhase94Double(out, report.final_lambda);
    out << ",\n      \"maximum_lambda\": ";
    writePhase94Double(out, report.maximum_lambda);
    out << ",\n      \"lambda_factor\": ";
    writePhase94Double(out, report.lambda_factor);
    out << ",\n      \"lambda_lower_bound\": ";
    writePhase94Double(out, report.lambda_lower_bound);
    out << ",\n      \"lambda_upper_bound\": ";
    writePhase94Double(out, report.lambda_upper_bound);
    out << ",\n      \"min_model_fidelity\": ";
    writePhase94Double(out, report.min_model_fidelity);
    out << ",\n      \"diagonal_damping\": ";
    writeBool(report.diagonal_damping);
    out << ",\n      \"use_fixed_lambda_factor\": ";
    writeBool(report.use_fixed_lambda_factor);
    out << ",\n      \"linear_solver\": ";
    writeJsonString(out, report.linear_solver);
    out << ",\n      \"elimination\": ";
    writeJsonString(out, report.elimination);
    out << ",\n      \"ordering_type\": ";
    writeJsonString(out, report.ordering_type);
    out << ",\n      \"explicit_ordering_present\": ";
    writeBool(report.explicit_ordering_present);
    out << ",\n      \"no_fallback\": ";
    writeBool(report.no_fallback);
    out << ",\n      \"termination_trace_complete\": ";
    writeBool(report.termination_trace_complete);
    out << ",\n      \"configuration_valid\": ";
    writeBool(report.configuration_valid);
    out << ",\n      \"configuration_failure\": ";
    writeJsonString(out, report.configuration_failure);
    out << "\n    }";
}

std::string makeSummary(const Options& options,
                        const libgnss::FGOProcessor::FGOProblem& problem,
                        const libgnss::FGOProcessor::FGOResult& result,
                        const ImuBuildReport& imu_report,
                        bool fallback,
                        const NativePdcBridgeReport& pdc_bridge_report =
                            NativePdcBridgeReport{},
                        const RawUtcOutputReport& raw_utc_report = RawUtcOutputReport{},
                        const TdcpRuntimeReport& tdcp_report = TdcpRuntimeReport{},
                        const libgnss::carrier_code_leveling::Diagnostics&
                            carrier_code_leveling_report =
                                libgnss::carrier_code_leveling::Diagnostics{},
                        const UpstreamPositionOffsetReport& position_offset_report =
                            UpstreamPositionOffsetReport{},
                        const BasePseudorangeCompensationReport& base_report =
                            BasePseudorangeCompensationReport{},
                        const Phase104StageExportReport& phase104_stage_report =
                            Phase104StageExportReport{},
                        const Phase104MainDisplacementReport& phase104_displacement_report =
                            Phase104MainDisplacementReport{},
                        const Phase116CarrierTdcpReport& phase116_tdcp_report =
                            Phase116CarrierTdcpReport{}) {
    const bool observable_quality_enabled =
        options.native_upstream_quality ||
        options.native_source_direct_observable_quality;
    const bool meter_state_parity =
        options.native_source_clock_c0d_meter_state_parity;
    const DirectObservableQualitySettings direct_quality_settings =
        directObservableQualitySettingsForDataset(options.dataset_id);
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n"
        << "  \"schema_version\": \"smartphone-r5-native-fgo-android-imu-no-base-run.v3\",\n"
        << "  \"dataset_id\": ";
    writeJsonString(out, options.dataset_id);
    out << ",\n  \"status\": "
        << (fallback ? "\"fallback-native-fgo-v1\"" : "\"imu-combined-factor\"") << ",\n"
        << "  \"truth_used\": false,\n"
        << "  \"base_factors\": false,\n"
        << "  \"no_base_contract\": true,\n"
        << "  \"production_default_changed\": false,\n"
        << "  \"fgo_imu_sparse_recovery\": "
        << (options.fgo_imu_sparse_recovery ? "true" : "false") << ",\n"
        << "  \"native_pdc_state_bridge\": "
        << (options.native_pdc_state_bridge ? "true" : "false") << ",\n"
        << "  \"native_pdc_imu_tdcp\": "
        << (options.native_pdc_imu_tdcp ? "true" : "false") << ",\n"
        << "  \"native_pdc_imu_tdcp_no_bridge\": "
        << (options.native_pdc_imu_tdcp_no_bridge ? "true" : "false") << ",\n"
        << "  \"native_phase117_tdcp_snr_type_sigma\": "
        << (options.native_phase117_tdcp_snr_type_sigma ? "true" : "false")
        << ",\n"
        << "  \"native_phase118_official_tdcp_huber_k\": "
        << (options.native_phase118_official_tdcp_huber_k ? "true" : "false")
        << ",\n"
        << "  \"native_phase184_source_tdcp_huber_k\": "
        << (options.native_phase184_source_tdcp_huber_k ? "true" : "false")
        << ",\n"
        << "  \"native_phase120_official_tdcp_resl_atmosphere_cancellation\": "
        << (options.native_phase120_official_tdcp_resl_atmosphere_cancellation
                ? "true"
                : "false")
        << ",\n"
        << "  \"native_phase126_raw_base_source_complete\": "
        << (options.native_phase126_raw_base_source_complete ? "true" : "false")
        << ",\n  \"native_paired_epoch_states\": "
        << (options.native_paired_epoch_states ? "true" : "false")
        << ",\n  \"native_rover_epoch_states\": "
        << (options.native_rover_epoch_states ? "true" : "false")
        << ",\n  \"native_dense_base_smoothing\": "
        << (options.native_dense_base_smoothing ? "true" : "false")
        << ",\n  \"native_base_mask_only_ablation\": "
        << (options.native_base_mask_only_ablation ? "true" : "false")
        << ",\n  \"native_base_gps_values_only_ablation\": "
        << (options.native_base_gps_values_only_ablation ? "true" : "false")
        << ",\n  \"native_base_gps_center_ablation\": "
        << (options.native_base_gps_center_ablation ? "true" : "false")
        << ",\n  \"native_main_p_cauchy\": "
        << (options.native_main_p_cauchy ? "true" : "false")
        << ",\n  \"native_base_nonzero_correction_enabled\": "
        << (options.native_base_pseudorange_compensation &&
                    !options.native_base_mask_only_ablation ? "true" : "false")
        << ",\n"
        << "  \"native_phase127_glonass_channel_provenance\": "
        << (options.native_phase127_glonass_channel_provenance ? "true" : "false")
        << ",\n"
        << "  \"native_phase128_glonass_provenance_parser_admission\": "
        << (options.native_phase128_glonass_provenance_parser_admission
                ? "true"
                : "false")
        << ",\n"
        << "  \"native_phase129_glonass_local_miss_mask\": "
        << (options.native_phase129_glonass_local_miss_mask ? "true" : "false")
        << ",\n"
        << "  \"native_phase131_canonical_correction_band_key\": "
        << (options.native_phase131_canonical_correction_band_key ? "true" : "false")
        << ",\n"
        << "  \"native_phase138_affine_tdcp_anchor_range_constant\": "
        << (options.native_phase138_affine_tdcp_anchor_range_constant
                ? "true"
                : "false")
        << ",\n"
        << "  \"native_signal_bias_states\": "
        << (options.native_signal_bias_states ? "true" : "false") << ",\n"
        << "  \"native_residual_ionosphere\": "
        << (options.native_residual_ionosphere ? "true" : "false") << ",\n"
        << "  \"native_upstream_quality\": "
        << (options.native_upstream_quality ? "true" : "false") << ",\n"
        << "  \"native_source_direct_observable_quality_enabled\": "
        << (options.native_source_direct_observable_quality ? "true" : "false")
        << ",\n"
        << "  \"native_source_clock_c0d_factor_enabled\": "
        << (options.native_source_clock_c0d_factor ? "true" : "false")
        << ",\n"
        << "  \"native_source_clock_c0d_meter_state_parity_enabled\": "
        << (meter_state_parity ? "true" : "false") << ",\n"
        << "  \"native_source_clock_c0d_active_solve_diagnostic_enabled\": "
        << (options.native_source_clock_c0d_active_solve_diagnostic
                ? "true"
                : "false")
        << ",\n"
        << "  \"native_source_clock_c0d_gnss_first_raw_drift_d_initializer_enabled\": "
        << (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer
                ? "true"
                : "false")
        << ",\n"
        << "  \"native_source_clock_c0d_gnss_first_meter_state_handoff_enabled\": "
        << (options.native_source_clock_c0d_gnss_first_meter_state_handoff
                ? "true"
                : "false")
        << ",\n"
        << "  \"native_source_clock_c0d_epoch_vector_parity_enabled\": "
        << (options.native_source_clock_c0d_epoch_vector_parity ? "true" : "false")
        << ",\n"
        << "  \"native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled\": "
        << (options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver
                ? "true"
                : "false")
        << ",\n"
        << "  \"native_phase171_raw_p_no_doppler_imu_main\": "
        << (options.native_phase171_raw_p_no_doppler_imu_main ? "true" : "false")
        << ",\n"
        << "  \"native_phase171_raw_p_ecef_doppler_gnss_first\": "
        << (options.native_phase171_raw_p_ecef_doppler_gnss_first ? "true"
                                                                  : "false")
        << ",\n"
        << "  \"native_phase171_gnss_first_doppler_frame\": ";
    writeJsonString(out, options.native_phase171_raw_p_ecef_doppler_gnss_first
                             ? "ECEF"
                             : "none");
    out << ",\n"
        << "  \"native_phase171_gnss_first_doppler_factors\": "
        << imu_report.gnss_first_doppler_factors << ",\n"
        << "  \"native_phase171_gnss_first_ecef_doppler_stage_enabled\": "
        << (imu_report.gnss_first_ecef_doppler_stage_enabled ? "true"
                                                             : "false")
        << ",\n"
        << "  \"native_phase171_gnss_first_doppler_factors_inserted\": "
        << imu_report.gnss_first_doppler_factors_inserted << ",\n"
        << "  \"native_phase171_main_generic_doppler_factors\": "
        << result.diagnostics.undifferenced_doppler_factors_inserted << ",\n"
        << "  \"native_phase171_main_configured_max_iterations\": "
        << (options.native_phase171_raw_p_no_doppler_imu_main ? 12 : 0)
        << ",\n"
        << "  \"native_phase171_main_effective_max_iterations\": "
        << (options.native_phase171_raw_p_no_doppler_imu_main ? 1000 : 0)
        << ",\n"
        << "  \"native_phase171_unobserved_clock_gauge_components\": "
        << result.diagnostics.native_phase171_unobserved_clock_gauge_components
        << ",\n"
        << "  \"native_phase171_unobserved_clock_gauge_sigma_m\": ";
    writePhase94Double(
        out, result.diagnostics.native_phase171_unobserved_clock_gauge_sigma_m);
    out << ",\n"
        << "  \"native_phase201_source_inclusive_forward_imu_schedule\": "
        << (options.native_phase201_source_inclusive_forward_imu_schedule
                ? "true"
                : "false")
        << ",\n"
        << "  \"native_direct_wls_ephemeral_c7d_main_seed_enabled\": "
        << (options.native_direct_wls_ephemeral_c7d_main_seed ? "true" : "false")
        << ",\n"
        << "  \"native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected\": "
        << (result.diagnostics
                    .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected
                ? "true"
                : "false")
        << ",\n"
        << "  \"selected_linear_solver_type\": ";
    writeJsonString(out, result.diagnostics.selected_linear_solver_type);
    out << ",\n"
        << "  \"selected_solver_branch\": ";
    writeJsonString(out, result.diagnostics.selected_solver_branch);
    out << ",\n"
        << "  \"selected_elimination_function\": ";
    writeJsonString(out, result.diagnostics.selected_elimination_function);
    out << ",\n"
        << "  \"phase135_official_affine_measurement_family\": {\n"
        << "    \"enabled\": "
        << (result.diagnostics
                    .phase135_official_affine_measurement_family_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"configuration_valid\": "
        << (result.diagnostics.phase135_configuration_valid ? "true" : "false")
        << ",\n"
        << "    \"configuration_failure\": ";
    writeJsonString(out, result.diagnostics.phase135_configuration_failure);
    out << ",\n"
        << "    \"pseudorange_factors_inserted\": "
        << result.diagnostics.phase135_pseudorange_factors_inserted << ",\n"
        << "    \"doppler_factors_inserted\": "
        << result.diagnostics.phase135_doppler_factors_inserted << ",\n"
        << "    \"tdcp_factors_inserted\": "
        << result.diagnostics.phase135_tdcp_factors_inserted << ",\n"
        << "    \"pose3_x_bridge_factors\": "
        << result.diagnostics.phase135_pose3_x_bridge_factors << ",\n"
        << "    \"geometry_rows_validated\": "
        << result.diagnostics.phase135_geometry_rows_validated << ",\n"
        << "    \"geometry_representation\": ";
    writeJsonString(out, result.diagnostics.phase135_geometry_representation);
    out << ",\n"
        << "    \"factor_key_order\": {\n"
        << "      \"pseudorange\": [\"X_i\", \"C_i\"],\n"
        << "      \"doppler\": [\"V_i\", \"D_i\"],\n"
        << "      \"ordinary_tdcp\": [\"X_i\", \"X_i+1\", \"C_i\", \"C_i+1\"]\n"
        << "    },\n"
        << "    \"single_sagnac_representation\": true,\n"
        << "    \"doppler_factor_los_convention\": \"-e=(receiver-satellite)/range\",\n"
        << "    \"doppler_residual_source\": \"official-Gsat-Gobs-range-rate-with-receiver-velocity\",\n"
        << "    \"doppler_residual_provenance_required\": true,\n"
        << "    \"phase107_raw_base_compatibility\": "
        << (options.native_phase135_official_affine_measurement_family &&
                    options.native_base_pseudorange_compensation &&
                    options.native_base_pseudorange_source_miss_mask &&
                    !options.native_base_pseudorange_preserve_additional_frequency_bands &&
                    !options.native_phase126_raw_base_source_complete &&
                    !options.native_phase127_glonass_channel_provenance &&
                    !options.native_phase128_glonass_provenance_parser_admission &&
                    !options.native_phase129_glonass_local_miss_mask &&
                    !options.native_phase131_canonical_correction_band_key
                ? "true"
                : "false")
        << ",\n"
        << "    \"mixed_or_partial_family_allowed\": false,\n"
        << "    \"solution_publication_authorized\": false\n"
        << "  },\n"
        << "  \"phase138_affine_tdcp_anchor_range_constant\": {\n"
        << "    \"enabled\": "
        << (result.diagnostics
                    .phase138_affine_tdcp_anchor_range_constant_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"phase135_dependency_satisfied\": "
        << (options.native_phase135_official_affine_measurement_family
                ? "true"
                : "false")
        << ",\n"
        << "    \"configuration_valid\": "
        << (result.diagnostics.phase138_configuration_valid ? "true" : "false")
        << ",\n"
        << "    \"configuration_failure\": ";
    writeJsonString(out, result.diagnostics.phase138_configuration_failure);
    out << ",\n"
        << "    \"measurement_equation\": ";
    writeJsonString(out, options.native_phase144_telemetry_schema
                            ? kPhase138EquationDisplay
                            : result.diagnostics.phase138_measurement_equation);
    out << ",\n";
    if (options.native_phase144_telemetry_schema) {
        out << "    \"equation\": ";
        writePhase138EquationMetadata(out, "      ");
        out << ",\n";
    }
    out << "    \"geometry_representation\": ";
    writeJsonString(out, result.diagnostics.phase138_geometry_representation);
    out << ",\n"
        << "    \"range_constants_validated\": "
        << result.diagnostics.phase138_tdcp_range_constants_validated << ",\n"
        << "    \"tdcp_measurements_adjusted\": "
        << result.diagnostics.phase138_tdcp_measurements_adjusted << ",\n"
        << "    \"adjustment_application_passes\": "
        << result.diagnostics.phase138_adjustment_application_passes << ",\n"
        << "    \"adjusted_exactly_once\": "
        << (result.diagnostics.phase138_adjusted_exactly_once ? "true" : "false")
        << ",\n"
        << "    \"factor_count_unchanged\": "
        << (result.diagnostics.phase138_factor_count_unchanged ? "true" : "false")
        << ",\n"
        << "    \"phase118_atmosphere_sigma_huber_unchanged\": true,\n"
        << "    \"single_sagnac_representation\": true,\n"
        << "    \"mixed_or_partial_family_allowed\": false,\n"
        << "    \"solution_publication_authorized\": false\n"
        << "  },\n"
        << "  \"native_upstream_absolute_doppler_screen\": "
        << (options.native_upstream_absolute_doppler_screen ? "true" : "false")
        << ",\n"
        << "  \"native_carrier_code_leveling\": "
        << (options.native_carrier_code_leveling ? "true" : "false") << ",\n"
        << "  \"native_carrier_code_innovation_reset\": "
        << (options.native_carrier_code_innovation_reset ? "true" : "false")
        << ",\n"
        << "  \"native_upstream_stop_constraints\": "
        << (options.native_upstream_stop_constraints ? "true" : "false")
        << ",\n"
        << "  \"native_upstream_position_offset\": "
        << (options.native_upstream_position_offset ? "true" : "false")
        << ",\n"
        << "  \"native_signal_specific_galileo_tgd\": "
        << (options.native_signal_specific_galileo_tgd ? "true" : "false")
        << ",\n"
        << "  \"native_quality_anchor\": "
        << (options.native_quality_anchor ? "true" : "false") << ",\n"
        ;
    if (options.native_phase143_official_main_lm_termination_budget ||
        options.native_phase171_raw_p_no_doppler_imu_main) {
        out << "  \"native_phase143_official_main_lm_termination_budget\": true,\n";
    }
    if (options.native_phase144_telemetry_schema) {
        out << "  \"native_phase144_telemetry_schema\": true,\n";
    }
    if (options.native_phase141_telemetry_schema) {
        // Keep the legacy/default summary byte-compatible: the candidate's
        // selector witness is present only in the opt-in schema envelope.
        // The preceding field already emitted a comma/newline terminator.
        out << "  \"native_phase141_telemetry_schema\": true,\n";
    }
    if (options.native_android_sv_time_uncertainty_sigma_floor) {
        const auto& uncertainty = problem.diagnostics;
        out << "  \"native_android_sv_time_uncertainty_sigma_floor\": {\n"
            << "    \"enabled\": true,\n"
            << "    \"source_field\": \"ReceivedSvTimeUncertaintyNanos\",\n"
            << "    \"sigma_formula\": \"max(existing_sigma_m, c*uncertainty_nanos*1e-9)\",\n"
            << "    \"speed_of_light_mps\": "
            << libgnss::constants::SPEED_OF_LIGHT << ",\n"
            << "    \"coefficient\": 1.0,\n"
            << "    \"upper_clip\": false,\n"
            << "    \"spp_applied\": false,\n"
            << "    \"scope\": \"adopted raw Android FGO pseudorange factors\",\n"
            << "    \"rows_applied\": "
            << uncertainty.native_android_sv_time_uncertainty_rows_applied << ",\n"
            << "    \"rows_fallback_existing_sigma\": "
            << uncertainty.native_android_sv_time_uncertainty_rows_fallback << ",\n"
            << "    \"factors_affected\": "
            << uncertainty.native_android_sv_time_uncertainty_factors_affected << ",\n"
            << "    \"floor_min_m\": "
            << uncertainty.native_android_sv_time_uncertainty_floor_min_m << ",\n"
            << "    \"floor_median_m\": "
            << uncertainty.native_android_sv_time_uncertainty_floor_median_m << ",\n"
            << "    \"floor_p95_m\": "
            << uncertainty.native_android_sv_time_uncertainty_floor_p95_m << ",\n"
            << "    \"floor_max_m\": "
            << uncertainty.native_android_sv_time_uncertainty_floor_max_m << "\n"
            << "  },\n";
    }
    if (options.native_cn0_doppler_calibration) {
        const auto& calibration = problem.diagnostics;
        out << "  \"native_cn0_doppler_calibration\": {\n"
            << "    \"enabled\": true,\n"
            << "    \"source_field\": \"Cn0DbHz via Observation::snr\",\n"
            << "    \"reference_cn0_dbhz\": "
            << libgnss::cn0_doppler_calibration::kReferenceCn0DbHz << ",\n"
            << "    \"alpha_mps_at_reference\": "
            << libgnss::cn0_doppler_calibration::kAlphaMpsAtReference << ",\n"
            << "    \"shape_formula\": \"alpha*10^(-(Cn0DbHz-reference)/20)\",\n"
            << "    \"sigma_formula\": \"max(existing_doppler_sigma_mps,model_sigma_mps)\",\n"
            << "    \"coefficient\": 1.0,\n"
            << "    \"upper_clip\": false,\n"
            << "    \"scope\": \"adopted Android raw undifferenced FGO Doppler factors only\",\n"
            << "    \"spp_applied\": false,\n"
            << "    \"tdcp_applied\": false,\n"
            << "    \"single_difference_doppler_applied\": false,\n"
            << "    \"existing_p85_over_12_path_untouched\": true,\n"
            << "    \"candidate_rows\": "
            << calibration.native_cn0_doppler_calibration_candidate_rows << ",\n"
            << "    \"finite_cn0_rows\": "
            << calibration.native_cn0_doppler_calibration_finite_cn0_rows << ",\n"
            << "    \"fallback_rows_existing_sigma\": "
            << calibration.native_cn0_doppler_calibration_fallback_rows << ",\n"
            << "    \"factors_affected\": "
            << calibration.native_cn0_doppler_calibration_factors_affected << ",\n"
            << "    \"model_sigma_min_mps\": "
            << calibration.native_cn0_doppler_calibration_model_sigma_min_mps << ",\n"
            << "    \"model_sigma_median_mps\": "
            << calibration.native_cn0_doppler_calibration_model_sigma_median_mps << ",\n"
            << "    \"model_sigma_p95_mps\": "
            << calibration.native_cn0_doppler_calibration_model_sigma_p95_mps << ",\n"
            << "    \"model_sigma_max_mps\": "
            << calibration.native_cn0_doppler_calibration_model_sigma_max_mps << "\n"
            << "  },\n";
    }
    if (options.native_base_pseudorange_compensation) {
        out << "  \"native_base_pseudorange_compensation\": {\n"
            << "    \"enabled\": true,\n"
            << "    \"phase126_source_complete\": "
            << (base_report.source_complete ? "true" : "false") << ",\n"
            << "    \"phase127_glonass_channel_provenance\": "
            << (base_report.phase127_enabled ? "true" : "false") << ",\n"
            << "    \"phase128_glonass_provenance_parser_admission\": "
            << (base_report.phase128_enabled ? "true" : "false") << ",\n"
            << "    \"phase128_header_status\": ";
        writeJsonString(out, base_report.phase128_header_status);
        out << ",\n"
            << "    \"phase128_canonical_records\": "
            << base_report.phase128_canonical_records << ",\n"
            << "    \"phase128_canonical_rejected_records\": "
            << base_report.phase128_canonical_rejected_records << ",\n"
            << "    \"phase129_glonass_local_miss_mask\": "
            << (base_report.phase129_enabled ? "true" : "false") << ",\n"
            << "    \"phase129_configuration_valid\": "
            << (base_report.phase129_configuration_valid ? "true" : "false")
            << ",\n"
            << "    \"phase129_configuration_failure\": ";
        writeJsonString(out, base_report.phase129_configuration_failure);
        out << ",\n"
            << "    \"phase129_glonass_local_miss_rows\": "
            << base_report.phase129_glonass_local_miss_rows << ",\n"
            << "    \"phase129_glonass_local_miss_streams\": "
            << base_report.phase129_glonass_local_miss_streams << ",\n"
            << "    \"phase129_glonass_local_miss_counts\": ";
        writeJsonSizeMap(out, base_report.phase129_glonass_local_miss_counts);
        out << ",\n"
            << "    \"phase129_glonass_row_count_consistent\": "
            << (base_report.phase129_glonass_row_count_consistent ? "true" : "false")
            << ",\n"
            << "    \"phase131_canonical_correction_band_key\": "
            << (base_report.phase131_enabled ? "true" : "false") << ",\n"
            << "    \"phase131_configuration_valid\": "
            << (base_report.phase131_configuration_valid ? "true" : "false")
            << ",\n"
            << "    \"phase131_configuration_failure\": ";
        writeJsonString(out, base_report.phase131_configuration_failure);
        out << ",\n"
            << "    \"phase131_canonical_rows\": "
            << base_report.phase131_canonical_rows << ",\n"
            << "    \"phase131_canonical_rejected_rows\": "
            << base_report.phase131_canonical_rejected_rows << ",\n"
            << "    \"phase131_unknown_band_rows\": "
            << base_report.phase131_unknown_band_rows << ",\n"
            << "    \"phase131_canonical_key_conflicts\": "
            << base_report.phase131_canonical_key_conflicts << ",\n"
            << "    \"phase131_canonical_duplicate_rows\": "
            << base_report.phase131_canonical_duplicate_rows << ",\n"
            << "    \"phase131_canonical_streams\": "
            << base_report.phase131_canonical_streams << ",\n"
            << "    \"phase131_canonical_selected_streams\": "
            << base_report.phase131_canonical_selected_streams << ",\n"
            << "    \"phase131_canonical_merged_streams\": "
            << base_report.phase131_canonical_merged_streams << ",\n"
            << "    \"phase131_failure_counts\": ";
        writeJsonSizeMap(out, base_report.phase131_failure_counts);
        out << ",\n"
            << "    \"source_miss_mask_canonical_key_mode\": "
            << (base_report.source_miss_mask_canonical_key_mode ? "true" : "false")
            << ",\n"
            << "    \"source_miss_mask_matching_key\": ";
        writeJsonString(out, base_report.source_miss_mask_matching_key);
        out << ",\n"
            << "    \"phase127_glonass_rows\": "
            << base_report.phase127_glonass_rows << ",\n"
            << "    \"phase127_accepted_rows\": "
            << base_report.phase127_accepted_rows << ",\n"
            << "    \"phase127_header_primary_rows\": "
            << base_report.phase127_header_primary_rows << ",\n"
            << "    \"phase127_ephemeris_fallback_rows\": "
            << base_report.phase127_ephemeris_fallback_rows << ",\n"
            << "    \"phase127_header_entries_seen\": "
            << base_report.phase127_header_entries_seen << ",\n"
            << "    \"phase127_header_duplicate_entries\": "
            << base_report.phase127_header_duplicate_entries << ",\n"
            << "    \"phase127_header_conflict_entries\": "
            << base_report.phase127_header_conflict_entries << ",\n"
            << "    \"phase127_header_malformed_entries\": "
            << base_report.phase127_header_malformed_entries << ",\n"
            << "    \"phase127_ephemeris_candidates\": "
            << base_report.phase127_ephemeris_candidates << ",\n"
            << "    \"phase127_ephemeris_ties\": "
            << base_report.phase127_ephemeris_ties << ",\n"
            << "    \"phase127_ephemeris_duplicate_entries\": "
            << base_report.phase127_ephemeris_duplicate_entries << ",\n"
            << "    \"phase127_ephemeris_conflict_entries\": "
            << base_report.phase127_ephemeris_conflict_entries << ",\n"
            << "    \"phase127_query_time_coverage_gaps\": "
            << base_report.phase127_query_time_coverage_gaps << ",\n"
            << "    \"phase127_invalid_channels\": "
            << base_report.phase127_invalid_channels << ",\n"
            << "    \"phase127_failure_counts\": ";
        writeJsonSizeMap(out, base_report.phase127_failure_counts);
        out << ",\n"
            << "    \"phase126_atomic_step_a_raw_ingress_verified\": "
            << (base_report.phase126_atomic_step_a_raw_ingress_verified
                    ? "true"
                    : "false")
            << ",\n"
            << "    \"phase126_atomic_step_b_source_stream_verified\": "
            << (base_report.phase126_atomic_step_b_source_stream_verified
                    ? "true"
                    : "false")
            << ",\n"
            << "    \"phase126_atomic_step_c_application_committed\": "
            << (base_report.phase126_atomic_step_c_application_committed
                    ? "true"
                    : "false")
            << ",\n"
            << "    \"phase126_compound_admitted\": "
            << (base_report.phase126_compound_admitted ? "true" : "false")
            << ",\n"
            << "    \"station_reference_verified\": "
            << (base_report.station_reference_verified ? "true" : "false")
            << ",\n"
            << "    \"antenna_reference_is_approx_position\": "
            << (base_report.antenna_reference_is_approx_position ? "true" : "false")
            << ",\n"
            << "    \"antenna_delta_present\": "
            << (base_report.antenna_delta_present ? "true" : "false") << ",\n"
            << "    \"antenna_delta_applied\": "
            << (base_report.antenna_delta_applied ? "true" : "false") << ",\n"
            << "    \"official_no_explicit_tgd_bgd\": "
            << (base_report.official_no_explicit_tgd_bgd ? "true" : "false")
            << ",\n"
            << "    \"sagnac_evaluations\": "
            << base_report.sagnac_evaluations << ",\n"
            << "    \"source_complete_signal_rows\": "
            << base_report.source_complete_signal_rows << ",\n"
            << "    \"built\": " << (base_report.built ? "true" : "false")
            << ",\n"
            << "    \"applied\": " << (base_report.applied ? "true" : "false")
            << ",\n"
            << "    \"source_model_build_count\": "
            << base_report.source_model_build_count << ",\n"
            << "    \"correction_application_pass_count\": "
            << base_report.correction_application_pass_count << ",\n"
            << "    \"correction_applied_exactly_once\": "
            << (base_report.correction_applied_exactly_once ? "true" : "false")
            << ",\n"
            << "    \"duplicate_correction_rejected\": "
            << (base_report.duplicate_correction_rejected ? "true" : "false")
            << ",\n"
            << "    \"preserve_additional_frequency_bands\": "
            << (base_report.preserve_additional_frequency_bands ? "true" : "false")
            << ",\n"
            << "    \"source_repository\": \"https://github.com/taroz/gsdc2023\",\n"
            << "    \"source_commit\": \"29923f9f370f09ebc00f96d8cca375007a18e7d5\",\n"
            << "    \"source_function\": \"functions/correct_pseudorange.m\",\n"
            << "    \"base_rinex\": ";
        writeJsonString(out, base_report.base_rinex_path);
        out << ",\n"
            << "    \"base_rinex_bytes\": " << base_report.base_rinex_bytes << ",\n"
            << "    \"base_rinex_sha256\": ";
        writeJsonString(out, base_report.base_rinex_sha256);
        out << ",\n"
            << "    \"base_member_sha256\": ";
        writeJsonString(out, base_report.base_rinex_sha256);
        out << ",\n"
            << "    \"sha256_verification\": \"structural runner hashes the file before launch and asserts this declared digest\",\n"
            << "    \"base_rinex_read_count\": 1,\n"
            << "    \"base_coordinate_provenance\": \"RINEX header APPROX POSITION XYZ\",\n"
            << "    \"base_coordinate_xyz_m\": ["
            << base_report.base_position_ecef.x() << ", "
            << base_report.base_position_ecef.y() << ", "
            << base_report.base_position_ecef.z() << "],\n"
            << "    \"header_version\": ";
        if (std::isfinite(base_report.header_version)) {
            out << base_report.header_version;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"header_interval_s\": ";
        if (std::isfinite(base_report.header_interval_s)) {
            out << base_report.header_interval_s;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"observed_interval_s\": ";
        if (std::isfinite(base_report.base_interval_s)) {
            out << base_report.base_interval_s;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"expected_interval_s\": ";
        if (std::isfinite(base_report.expected_interval_s)) {
            out << base_report.expected_interval_s;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"moving_mean_samples\": " << base_report.moving_mean_samples << ",\n"
            << "    \"moving_mean_edge_policy\": \"centered finite window shrinks at edges\",\n"
            << "    \"matching_key\": ";
        writeJsonString(out, base_report.source_miss_mask_matching_key);
        out << ",\n"
            << "    \"same_satellite_signal_only\": "
            << (base_report.source_miss_mask_canonical_key_mode ? "false" : "true")
            << ",\n"
            << "    \"literal_tracking_code_in_join\": "
            << (base_report.source_miss_mask_canonical_key_mode ? "false" : "true")
            << ",\n"
            << "    \"base_epochs\": " << base_report.base_epochs << ",\n"
            << "    \"base_observation_rows\": " << base_report.base_observation_rows << ",\n"
            << "    \"selected_band_observation_rows\": "
            << base_report.selected_band_observation_rows << ",\n"
            << "    \"selected_band_streams\": "
            << base_report.selected_band_streams << ",\n"
            << "    \"selected_band_observation_rows_by_signal\": ";
        writeJsonSizeMap(out, base_report.selected_band_observation_rows_by_signal);
        out << ",\n"
            << "    \"selected_band_streams_by_signal\": ";
        writeJsonSizeMap(out, base_report.selected_band_streams_by_signal);
        out << ",\n"
            << "    \"matching_streams\": " << base_report.matching_streams << ",\n"
            << "    \"matched_base_rows\": " << base_report.matched_base_rows << ",\n"
            << "    \"finite_base_residual_rows\": "
            << base_report.finite_base_residual_rows << ",\n"
            << "    \"smoothed_rows\": " << base_report.smoothed_rows << ",\n"
            << "    \"in_domain_rows\": " << base_report.interpolated_rows << ",\n"
            << "    \"interpolation_misses\": " << base_report.interpolation_misses << ",\n"
            << "    \"adopted_pseudorange_rows\": "
            << base_report.adopted_pseudorange_rows << ",\n"
            << "    \"adopted_rows_corrected\": "
            << base_report.adopted_rows_corrected << ",\n"
            << "    \"matched_factor_rows\": "
            << base_report.matched_factor_rows << ",\n"
            << "    \"finite_correction_rows_among_matched\": "
            << base_report.finite_correction_rows_among_matched << ",\n"
            << "    \"matched_factor_fraction\": ";
        if (base_report.adopted_pseudorange_rows > 0U) {
            out << static_cast<double>(base_report.matched_factor_rows) /
                         static_cast<double>(base_report.adopted_pseudorange_rows);
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"finite_correction_fraction_among_matched\": ";
        if (base_report.matched_factor_rows > 0U) {
            out << static_cast<double>(base_report.finite_correction_rows_among_matched) /
                         static_cast<double>(base_report.matched_factor_rows);
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"finite_correction_fraction\": ";
        if (base_report.adopted_pseudorange_rows > 0U) {
            out << static_cast<double>(base_report.adopted_rows_corrected) /
                         static_cast<double>(base_report.adopted_pseudorange_rows);
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"correction_abs_p50_m\": ";
        if (std::isfinite(base_report.correction_abs_p50_m)) {
            out << base_report.correction_abs_p50_m;
        } else {
            out << "null";
        }
        out << ",\n    \"correction_abs_p95_m\": ";
        if (std::isfinite(base_report.correction_abs_p95_m)) {
            out << base_report.correction_abs_p95_m;
        } else {
            out << "null";
        }
        out << ",\n    \"correction_abs_max_m\": ";
        if (std::isfinite(base_report.correction_abs_max_m)) {
            out << base_report.correction_abs_max_m;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"source_miss_taxonomy_by_signal\": ";
        writeJsonSourceMissTaxonomy(out, base_report.source_miss_taxonomy_by_signal);
        out << ",\n"
            << "    \"formula\": \"P_rover_corrected_m=P_rover_raw_m-pc_m\",\n"
            << "    \"base_residual_formula\": \"Phase126: pc_raw=P_base+satellite_clock_m-geometric_range_m-ionosphere_m-troposphere_m (official no explicit TGD/BGD); legacy: pc_raw=P_base+satellite_clock_m-ionosphere_m-troposphere_m-group_delay_m-geometric_range_m; pc=movmean(pc_raw)\",\n"
            << "    \"interpolation\": \"linear in-domain at rover epoch; no extrapolation, endpoint hold, or nearest fill\",\n"
            << "    \"moving_mean_selection\": \"observed median interval 1 s=>151 samples, 15 s=>11 samples\",\n"
            << "    \"scope\": \"adopted undifferenced FGO pseudorange factors only\",\n"
            << "    \"spp_applied\": false,\n"
            << "    \"tdcp_applied\": false,\n"
            << "    \"doppler_applied\": false,\n"
            << "    \"no_extrapolation_or_endpoint_hold\": true,\n"
            << "    \"failure\": ";
        writeJsonString(out, base_report.failure);
        out << "\n  },\n";
    }
    if (options.native_base_pseudorange_source_miss_mask) {
        out << "  \"native_base_pseudorange_source_miss_mask\": {\n"
            << "    \"enabled\": true,\n"
            << "    \"canonical_key_mode\": "
            << (base_report.source_miss_mask_canonical_key_mode ? "true" : "false")
            << ",\n"
            << "    \"matching_key\": ";
        writeJsonString(out, base_report.source_miss_mask_matching_key);
        out << ",\n"
            << "    \"source_contract\": \"finite in-domain pc only; missing/out-of-domain pc is a pseudorange-factor miss\",\n"
            << "    \"scope\": \"adopted undifferenced FGO pseudorange factors only\",\n"
            << "    \"original_adopted_pseudorange_rows\": "
            << base_report.original_adopted_pseudorange_rows << ",\n"
            << "    \"retained_finite_pc_pseudorange_rows\": "
            << base_report.retained_finite_pc_pseudorange_rows << ",\n"
            << "    \"dropped_missing_exact_stream_rows\": "
            << base_report.dropped_missing_exact_stream_rows << ",\n"
            << "    \"dropped_out_of_domain_rows\": "
            << base_report.dropped_out_of_domain_rows << ",\n"
            << "    \"dropped_nonfinite_correction_rows\": "
            << base_report.dropped_nonfinite_correction_rows << ",\n"
            << "    \"corrected_rows\": "
            << base_report.corrected_rows << ",\n"
            << "    \"signal_count_consistent\": "
            << (base_report.signal_count_consistent ? "true" : "false")
            << ",\n"
            << "    \"retained_finite_pc_fraction\": "
            << base_report.retained_finite_pc_fraction << ",\n"
            << "    \"retained_over_original_fraction\": "
            << base_report.retained_over_original_fraction << ",\n"
            << "    \"pseudorange_factors_inserted\": "
            << problem.pseudorange_factors.size() << ",\n"
            << "    \"pseudorange_factor_count_consistent\": "
            << (base_report.pseudorange_factor_count_consistent ? "true" : "false")
            << ",\n"
            << "    \"correction_application_pass_count\": "
            << base_report.correction_application_pass_count << ",\n"
            << "    \"correction_applied_exactly_once\": "
            << (base_report.correction_applied_exactly_once ? "true" : "false")
            << ",\n"
            << "    \"duplicate_correction_rejected\": "
            << (base_report.duplicate_correction_rejected ? "true" : "false")
            << ",\n"
            << "    \"signal_taxonomy_by_signal\": ";
        writeJsonSourceMissTaxonomy(out, base_report.source_miss_taxonomy_by_signal);
        out << ",\n"
            << "    \"correction_abs_p50_m\": "
            << base_report.correction_abs_p50_m << ",\n"
            << "    \"correction_abs_p95_m\": "
            << base_report.correction_abs_p95_m << ",\n"
            << "    \"correction_abs_max_m\": "
            << base_report.correction_abs_max_m << ",\n"
            << "    \"retained_factor_epoch_indices_unchanged\": true,\n"
            << "    \"tdcp_doppler_imu_spp_unchanged\": true,\n"
            << "    \"no_extrapolation_or_endpoint_hold\": true,\n"
            << "    \"sign\": \"P_rover_corrected_m=P_rover_raw_m-pc_m\",\n"
            << "    \"failure\": ";
        writeJsonString(out, base_report.failure);
        out << "\n  },\n";
    }
    if (options.native_fallback_seed_quality_anchor_recovery) {
        out << "  \"native_fallback_seed_quality_anchor_recovery\": true,\n";
    }
    if (options.native_direct_doppler_wls_handoff) {
        out << "  \"native_direct_doppler_wls_handoff\": true,\n";
    }
    if (options.native_carrier_code_primary_l1_e1) {
        out << "  \"native_carrier_code_primary_l1_e1\": true,\n";
    }
    if (options.native_carrier_code_gal_e1_e5a) {
        out << "  \"native_carrier_code_gal_e1_e5a\": true,\n";
    }
    out << "  \"android_utc_wall_clock_fallback\": "
        << (options.android_utc_wall_clock_fallback ? "true" : "false") << ",\n"
        << "  \"android_utc_wall_clock_fallback_applied\": "
        << (imu_report.android_load.utc_wall_clock_fallback_applied
                ? "true"
        : "false") << ",\n"
        << "  \"native_phase194_source_utc_fallback_imu_noise\": "
        << (options.native_phase194_source_utc_fallback_imu_noise
                ? "true"
                : "false") << ",\n"
        << "  \"imu_measurement_noise\": {\n"
        << "    \"selector_requested\": "
        << (imu_report.phase194_source_utc_fallback_imu_noise_requested
                ? "true"
                : "false") << ",\n"
        << "    \"fallback_applied\": "
        << (imu_report.android_load.utc_wall_clock_fallback_applied
                ? "true"
                : "false") << ",\n"
        << "    \"source_utc_fallback_applied\": "
        << (imu_report.phase194_source_utc_fallback_imu_noise_applied
                ? "true"
                : "false") << ",\n"
        << "    \"source_branch\": ";
    writeJsonString(out, imu_report.imu_measurement_noise_source);
    out << ",\n"
        << "    \"measurement_sync_coefficient\": "
        << imu_report.imu_measurement_noise_sync_coefficient << ",\n"
        << "    \"accel_noise_sigma\": "
        << imu_report.imu_accel_noise_sigma << ",\n"
        << "    \"gyro_noise_sigma\": "
        << imu_report.imu_gyro_noise_sigma << ",\n"
        << "    \"accel_covariance_diagonal\": "
        << imu_report.imu_accel_noise_sigma * imu_report.imu_accel_noise_sigma
        << ",\n"
        << "    \"gyro_covariance_diagonal\": "
        << imu_report.imu_gyro_noise_sigma * imu_report.imu_gyro_noise_sigma
        << ",\n"
        << "    \"alignment_sync_coefficient\": "
        << kUpstreamImuSyncCoefficient << ",\n"
        << "    \"bias_random_walk_unchanged\": true,\n"
        << "    \"integration_noise_unchanged\": true\n"
        << "  },\n"
        << "  \"native_phase197_source_utc_fallback_imu_offset\": "
        << (options.native_phase197_source_utc_fallback_imu_offset
                ? "true"
                : "false") << ",\n"
        << "  \"imu_utc_fallback_offset\": {\n"
        << "    \"selector_requested\": "
        << (imu_report.phase197_source_utc_fallback_imu_offset_requested
                ? "true"
                : "false") << ",\n"
        << "    \"fallback_applied\": "
        << (imu_report.android_load.utc_wall_clock_fallback_applied
                ? "true"
                : "false") << ",\n"
        << "    \"offset_applied\": "
        << (imu_report.phase197_source_utc_fallback_imu_offset_applied
                ? "true"
                : "false") << ",\n"
        << "    \"configured_offset_ms\": "
        << imu_report.phase197_source_utc_fallback_imu_offset_configured_ms
        << ",\n"
        << "    \"effective_offset_ms\": "
        << imu_report.phase197_source_utc_fallback_imu_offset_effective_ms
        << ",\n"
        << "    \"source_branch\": ";
    writeJsonString(out, imu_report.imu_utc_fallback_offset_source);
    out << ",\n"
        << "    \"pairing_clock_unchanged\": true,\n"
        << "    \"raw_utc_keys_unchanged\": true,\n"
        << "    \"elapsed_clock_preserved\": "
        << (imu_report.android_load.elapsed_clock_preserved
                ? "true"
                : "false") << "\n"
        << "  },\n"
        << "  \"native_pdc_state_bridge_report\": {\n"
        << "    \"enabled\": "
        << (pdc_bridge_report.enabled ? "true" : "false") << ",\n"
        << "    \"ok\": " << (pdc_bridge_report.ok ? "true" : "false") << ",\n"
        << "    \"epochs\": " << pdc_bridge_report.epochs << ",\n"
        << "    \"pseudorange_rows\": " << pdc_bridge_report.pseudorange_rows << ",\n"
        << "    \"doppler_rows\": " << pdc_bridge_report.doppler_rows << ",\n"
        << "    \"motion_intervals\": " << pdc_bridge_report.motion_intervals << ",\n"
        << "    \"valid_position_states\": "
        << pdc_bridge_report.valid_position_states << ",\n"
        << "    \"rejected_position_states\": "
        << pdc_bridge_report.rejected_position_states << ",\n"
        << "    \"valid_velocity_states\": "
        << pdc_bridge_report.valid_velocity_states << ",\n"
        << "    \"state_seeds\": " << pdc_bridge_report.state_seeds << ",\n"
        << "    \"iterations\": " << pdc_bridge_report.iterations << ",\n"
        << "    \"initial_cost\": " << pdc_bridge_report.initial_cost << ",\n"
        << "    \"final_cost\": " << pdc_bridge_report.final_cost << ",\n"
        << "    \"max_velocity_norm_mps\": "
        << pdc_bridge_report.max_velocity_norm_mps << ",\n"
        << "    \"max_clock_rate_abs_mps\": "
        << pdc_bridge_report.max_clock_rate_abs_mps << ",\n"
        << "    \"max_normalized_rms\": "
        << pdc_bridge_report.max_normalized_rms << ",\n"
        << "    \"first_epoch_doppler_rows\": "
        << pdc_bridge_report.first_epoch_doppler_rows << ",\n"
        << "    \"later_epoch_doppler_rows\": "
        << pdc_bridge_report.later_epoch_doppler_rows << ",\n"
        << "    \"epochs_with_doppler\": "
        << pdc_bridge_report.epochs_with_doppler << ",\n"
        << "    \"finite_raw_clock_drift_epochs\": "
        << pdc_bridge_report.finite_raw_clock_drift_epochs << ",\n"
        << "    \"min_epoch_dt_s\": "
        << pdc_bridge_report.min_epoch_dt_s << ",\n"
        << "    \"max_epoch_dt_s\": "
        << pdc_bridge_report.max_epoch_dt_s << ",\n"
        << "    \"invalid_or_large_epoch_intervals\": "
        << pdc_bridge_report.invalid_or_large_epoch_intervals << ",\n"
        << "    \"doppler_residual_rms_mps\": "
        << pdc_bridge_report.doppler_residual_rms_mps << ",\n"
        << "    \"doppler_abs_p50_mps\": "
        << pdc_bridge_report.doppler_abs_p50_mps << ",\n"
        << "    \"doppler_abs_max_mps\": "
        << pdc_bridge_report.doppler_abs_max_mps << ",\n"
        << "    \"doppler_sigma_min_mps\": "
        << pdc_bridge_report.doppler_sigma_min_mps << ",\n"
        << "    \"doppler_sigma_max_mps\": "
        << pdc_bridge_report.doppler_sigma_max_mps << ",\n"
        << "    \"initial_pseudorange_rms_m\": "
        << pdc_bridge_report.initial_pseudorange_rms_m << ",\n"
        << "    \"initial_pseudorange_max_m\": "
        << pdc_bridge_report.initial_pseudorange_max_m << ",\n"
        << "    \"seed_position_step_max_mps\": "
        << pdc_bridge_report.seed_position_step_max_mps << ",\n"
        << "    \"all_state_max_velocity_norm_mps\": "
        << pdc_bridge_report.all_state_max_velocity_norm_mps << ",\n"
        << "    \"all_state_max_clock_rate_abs_mps\": "
        << pdc_bridge_report.all_state_max_clock_rate_abs_mps << ",\n"
        << "    \"all_state_velocity_over_bound\": "
        << pdc_bridge_report.all_state_velocity_over_bound << ",\n"
        << "    \"all_state_clock_rate_over_bound\": "
        << pdc_bridge_report.all_state_clock_rate_over_bound << ",\n"
        << "    \"wls_valid_epochs\": "
        << pdc_bridge_report.wls_valid_epochs << ",\n"
        << "    \"wls_rejected_epochs\": "
        << pdc_bridge_report.wls_rejected_epochs << ",\n"
        << "    \"wls_propagated_epochs\": "
        << pdc_bridge_report.wls_propagated_epochs << ",\n"
        << "    \"wls_max_velocity_norm_mps\": "
        << pdc_bridge_report.wls_max_velocity_norm_mps << ",\n"
        << "    \"wls_max_clock_rate_abs_mps\": "
        << pdc_bridge_report.wls_max_clock_rate_abs_mps << ",\n"
        << "    \"wls_max_normalized_rms\": "
        << pdc_bridge_report.wls_max_normalized_rms << ",\n"
        << "    \"wls_first_velocity_norm_mps\": "
        << pdc_bridge_report.wls_first_velocity_norm_mps << ",\n"
        << "    \"wls_first_clock_rate_mps\": "
        << pdc_bridge_report.wls_first_clock_rate_mps << ",\n"
        << "    \"wls_first_reason\": ";
    writeJsonString(out, pdc_bridge_report.wls_first_reason);
    out << ",\n"
        << "    \"integrated_seed_anchor_index\": "
        << pdc_bridge_report.integrated_seed_anchor_index << ",\n"
        << "    \"integrated_seed_epochs\": "
        << pdc_bridge_report.integrated_seed_epochs << ",\n"
        << "    \"integrated_seed_held_velocity_epochs\": "
        << pdc_bridge_report.integrated_seed_held_velocity_epochs << ",\n"
        << "    \"integrated_seed_spp_fallback_epochs\": "
        << pdc_bridge_report.integrated_seed_spp_fallback_epochs << ",\n"
        << "    \"integrated_seed_reset_intervals\": "
        << pdc_bridge_report.integrated_seed_reset_intervals << ",\n"
        << "    \"integrated_seed_max_step_speed_mps\": "
        << pdc_bridge_report.integrated_seed_max_step_speed_mps << ",\n"
        << "    \"integrated_seed_max_displacement_from_spp_m\": "
        << pdc_bridge_report.integrated_seed_max_displacement_from_spp_m << ",\n"
        << "    \"fgo_seed_displacement_count\": "
        << pdc_bridge_report.fgo_seed_displacement_count << ",\n"
        << "    \"fgo_seed_displacement_p50_m\": "
        << pdc_bridge_report.fgo_seed_displacement_p50_m << ",\n"
        << "    \"fgo_seed_displacement_max_m\": "
        << pdc_bridge_report.fgo_seed_displacement_max_m << ",\n"
        << "    \"failure\": ";
    writeJsonString(out, pdc_bridge_report.failure);
    out << "\n  },\n"
        << "  \"skip_epochs\": " << options.skip_epochs << ",\n"
        << "  \"all_epochs\": " << (options.all_epochs ? "true" : "false") << ",\n"
        << "  \"inputs\": {\n"
        << "    \"observation\": ";
    if (options.obs_path.empty()) {
        out << "null";
    } else {
        writeJsonString(out, options.obs_path);
    }
    out << ",\n    \"navigation\": ";
    writeJsonString(out, options.nav_path);
    out << ",\n    \"imu\": ";
    writeJsonString(out, options.android_imu_path.empty()
                            ? options.imu_path
                            : options.android_imu_path);
    out << ",\n    \"android_gnss\": ";
    if (options.android_gnss_path.empty()) {
        out << "null";
    } else {
        writeJsonString(out, options.android_gnss_path);
    }
    out << "\n  },\n";
    if (imu_report.android_raw) {
        const auto& gnss = imu_report.android_gnss_diagnostics;
        out << "  \"android_gnss_diagnostics\": {\n"
            << "    \"input_rows\": " << gnss.input_rows << ",\n"
            << "    \"raw_rows\": " << gnss.raw_rows << ",\n"
            << "    \"selected_rows\": " << gnss.selected_rows << ",\n"
            << "    \"selected_epochs\": " << gnss.selected_epochs << ",\n"
            << "    \"carrier_rows\": " << gnss.carrier_rows << ",\n"
            << "    \"doppler_rows\": " << gnss.doppler_rows << ",\n"
            << "    \"clock_discontinuities\": " << gnss.clock_discontinuities << ",\n"
            << "    \"enriched_pseudorange_checks\": "
            << gnss.enriched_pseudorange_checks << ",\n"
            << "    \"enriched_pseudorange_mismatches\": "
            << gnss.enriched_pseudorange_mismatches << ",\n"
            << "    \"enriched_pseudorange_ignored_rows\": "
            << gnss.enriched_pseudorange_ignored_rows << ",\n"
            << "    \"enriched_pseudorange_input_ignored\": "
            << (gnss.enriched_pseudorange_input_ignored ? "true" : "false")
            << ",\n"
            << "    \"pseudorange_source\": \"raw Android clock timing only\",\n"
            << "    \"timing_formula\": ";
        writeJsonString(out, gnss.timing_formula);
        out << ",\n    \"no_device_wls_seed\": true\n"
            << "  },\n";
    }
    out
        << "  \"epochs\": {\n"
        << "    \"problem\": " << problem.epochs.size() << ",\n"
        << "    \"output\": " << result.solution.solutions.size() << ",\n"
        << "    \"sparse_epochs_retained\": "
        << result.diagnostics.sparse_epochs_retained << ",\n"
        << "    \"sparse_empty_epochs_retained\": "
        << result.diagnostics.sparse_empty_epochs_retained << ",\n"
        << "    \"pseudorange_factors\": " << problem.pseudorange_factors.size() << ",\n"
        << "    \"receiver_signal_bias_factors\": "
        << result.diagnostics.receiver_signal_bias_factors << ",\n"
        << "    \"receiver_signal_bias_states\": "
        << result.diagnostics.receiver_signal_bias_states << ",\n"
        << "    \"residual_ionosphere_factors\": "
        << result.diagnostics.residual_ionosphere_factors << ",\n"
        << "    \"residual_ionosphere_states\": "
        << result.diagnostics.residual_ionosphere_states << ",\n"
        << "    \"residual_ionosphere_resets\": "
        << result.diagnostics.residual_ionosphere_resets << ",\n"
        << "    \"tdcp_factors_built\": " << problem.tdcp_factors.size() << ",\n"
        << "    \"double_difference_pseudorange_factors\": "
        << problem.double_difference_pseudorange_factors.size() << ",\n"
        << "    \"double_difference_carrier_factors\": "
        << problem.double_difference_carrier_factors.size() << "\n"
        << "  },\n"
        << "  \"galileo_e1_group_delay\": {\n"
        << "    \"enabled\": "
        << (options.native_signal_specific_galileo_tgd ? "true" : "false")
        << ",\n"
        << "    \"fnav_rows\": "
        << problem.diagnostics.galileo_e1_fnav_group_delay_rows << ",\n"
        << "    \"inav_rows\": "
        << problem.diagnostics.galileo_e1_inav_group_delay_rows << ",\n"
        << "    \"source_fallback_rows\": "
        << problem.diagnostics.galileo_e1_group_delay_source_fallback_rows
        << ",\n"
        << "    \"invalid_rows\": "
        << problem.diagnostics.galileo_e1_group_delay_invalid_rows << ",\n"
        << "    \"source_bits\": {\"fnav_clock\": 256, \"inav_clock\": 512},\n"
        << "    \"correction_contract\": \"F/NAV=tgd(BGD E1/E5a), I/NAV=tgd_secondary(BGD E1/E5b), ambiguous=tgd\"\n"
        << "  },\n"
        << "  \"upstream_observable_quality\": {\n"
        << "    \"enabled\": "
        << (observable_quality_enabled ? "true" : "false") << ",\n"
        << "    \"snr_percentile\": 85.0,\n"
        << "    \"snr_denominator_db\": 20.0,\n"
        << "    \"min_snr_dbhz\": 20.0,\n"
        << "    \"min_elevation_deg\": 5.0,\n"
        << "    \"max_adjacent_gap_s\": 1.5,\n"
        << "    \"tdcp_sigma_m_unchanged\": "
        << ((options.native_source_tdcp_meter_sigma ||
             options.native_phase117_tdcp_snr_type_sigma) ? "null" : "0.03")
        << ",\n"
        << "    \"snr_l1_dbhz\": ";
    if (observable_quality_enabled &&
        std::isfinite(problem.diagnostics.upstream_snr_l1_dbhz)) {
        out << problem.diagnostics.upstream_snr_l1_dbhz;
    } else {
        out << "null";
    }
    out << ",\n    \"snr_l5_dbhz\": ";
    if (observable_quality_enabled &&
        std::isfinite(problem.diagnostics.upstream_snr_l5_dbhz)) {
        out << problem.diagnostics.upstream_snr_l5_dbhz;
    } else {
        out << "null";
    }
    out << ",\n"
        << "    \"pseudorange_candidates\": "
        << problem.diagnostics.upstream_pseudorange_candidates << ",\n"
        << "    \"pseudorange_factors\": "
        << problem.diagnostics.upstream_pseudorange_factors << ",\n"
        << "    \"doppler_candidates\": "
        << problem.diagnostics.upstream_doppler_candidates << ",\n"
        << "    \"doppler_factors\": "
        << problem.diagnostics.upstream_doppler_factors << ",\n"
        << "    \"doppler_graph_factors\": "
        << result.diagnostics.undifferenced_doppler_factors_inserted << ",\n"
        << "    \"pd_pair_rejections\": "
        << problem.diagnostics.upstream_pd_pair_rejections << ",\n"
        << "    \"ld_pair_rejections\": "
        << problem.diagnostics.upstream_ld_pair_rejections << ",\n"
        << "    \"doppler_residual_rejections\": "
        << problem.diagnostics.upstream_doppler_residual_rejections << ",\n"
        << "    \"pseudorange_residual_rejections\": "
        << problem.diagnostics.upstream_pseudorange_residual_rejections << ",\n"
        << "    \"pseudorange_postfit_rms_m\": "
        << result.diagnostics.residual_rms_m << ",\n"
        << "    \"pseudorange_postfit_normalized_rms\": "
        << result.diagnostics.upstream_pseudorange_normalized_rms << ",\n"
        << "    \"doppler_postfit_rms_mps\": "
        << result.diagnostics.undifferenced_doppler_residual_rms_mps << ",\n"
        << "    \"doppler_postfit_normalized_rms\": "
        << result.diagnostics.upstream_doppler_normalized_rms << ",\n"
        << "    \"pseudorange_sigma_contract\": \"snr_scale*signal_type_factor\",\n"
        << "    \"doppler_sigma_contract\": \"snr_scale/12\",\n"
        << "    \"carrier_tdcp_contract\": "
        << (options.native_source_tdcp_meter_sigma
                ? "\"Source SNR/type sigma in metres; no wavelength multiplication\""
                : options.native_phase117_tdcp_snr_type_sigma
                ? "\"Phase117 official SNR/type sigma; source L cycles converted by retained wavelength\""
                : "\"Phase12 frozen sigma 0.03; upstream L weighting not enabled\"")
        << "\n"
        << "  },\n";
    if (options.native_doppler_rotation_rate) {
        out << "  \"native_doppler_rotation_rate\": {\"enabled\": true, \"gnss_first_and_main\": true},\n";
    }
    if (options.native_tdcp_adr_endpoint_sigma) {
        out << "  \"native_tdcp_adr_endpoint_sigma\": {\"enabled\": true, \"diagonal_only\": true, \"gnss_first_and_main\": true},\n";
    }
    if (options.native_joint_ionosphere) {
        out << "  \"native_joint_ionosphere\": {\"enabled\": true, \"main_only\": true, "
            << "\"anchor_sigma_m\": " << options.joint_ionosphere_anchor
            << ", \"density_m_sqrt_s\": " << options.joint_ionosphere_density
            << ", \"max_gap_s\": " << options.joint_ionosphere_gap << "},\n";
    }
    if (options.native_source_direct_observable_quality) {
        out << "  \"native_source_direct_observable_quality\": {\n"
            << "    \"enabled\": true,\n"
            << "    \"direct_no_pdc\": true,\n"
            << "    \"pdc_bridge\": false,\n"
            << "    \"native_pdc_state_bridge\": false,\n"
            << "    \"environment\": ";
        writeJsonString(out, direct_quality_settings.environment);
        out << ",\n"
            << "    \"p_and_d_huber\": {\n"
            << "      \"pseudorange_sigma\": "
            << direct_quality_settings.pseudorange_huber_threshold_sigma << ",\n"
            << "      \"doppler_sigma\": "
            << direct_quality_settings.doppler_huber_threshold_sigma << "\n"
            << "    },\n"
            << "    \"config\": {\n"
            << "      \"use_upstream_observable_quality\": true,\n"
            << "      \"upstream_snr_percentile\": 85.0,\n"
            << "      \"upstream_min_snr_dbhz\": 20.0,\n"
            << "      \"upstream_min_elevation_deg\": 5.0,\n"
            << "      \"upstream_max_adjacent_gap_s\": 1.5,\n"
            << "      \"pseudorange_huber_threshold_sigma\": "
            << direct_quality_settings.pseudorange_huber_threshold_sigma << ",\n"
            << "      \"undifferenced_doppler_huber_threshold_sigma\": "
            << direct_quality_settings.doppler_huber_threshold_sigma << ",\n"
            << "      \"tdcp_sigma_m_unchanged\": 0.03,\n"
            << "      \"pseudorange_sigma_contract\": \"snr_scale*signal_type_factor\",\n"
            << "      \"doppler_sigma_contract\": \"snr_scale/12\",\n"
            << "      \"adjacent_mask_contract\": \"applyAdjacentMasks Pmask_dDP/Lmask_dDL unchanged\",\n"
            << "      \"pseudorange_residual_screen\": \"Pmask_res L1=20m/L5=15m\",\n"
            << "      \"doppler_residual_screen\": \"Dmask_res 3m/s; non-initializer\"\n"
            << "    },\n"
            << "    \"counts\": {\n"
            << "      \"pseudorange_candidates\": "
            << problem.diagnostics.upstream_pseudorange_candidates << ",\n"
            << "      \"pseudorange_factors\": "
            << problem.diagnostics.upstream_pseudorange_factors << ",\n"
            << "      \"doppler_candidates\": "
            << problem.diagnostics.upstream_doppler_candidates << ",\n"
            << "      \"doppler_factors\": "
            << problem.diagnostics.upstream_doppler_factors << ",\n"
            << "      \"doppler_graph_factors\": "
            << result.diagnostics.undifferenced_doppler_factors_inserted << ",\n"
            << "      \"pseudorange_residual_rejections\": "
            << problem.diagnostics.upstream_pseudorange_residual_rejections << ",\n"
            << "      \"doppler_residual_rejections\": "
            << problem.diagnostics.upstream_doppler_residual_rejections << "\n"
            << "    }\n"
            << "  },\n";
    }
    out << "  \"native_source_clock_c0d_factor\": {\n"
        << "    \"clock_c0d_enabled\": "
        << (options.native_source_clock_c0d_factor ? "true" : "false")
        << ",\n"
        << "    \"gnss_first_meter_state_handoff_enabled\": "
        << (options.native_source_clock_c0d_gnss_first_meter_state_handoff
                ? "true"
                : "false")
        << ",\n"
        << "    \"meter_state_parity_enabled\": "
        << (meter_state_parity ? "true" : "false") << ",\n"
        << "    \"epoch_vector_parity_enabled\": "
        << (options.native_source_clock_c0d_epoch_vector_parity ? "true" : "false")
        << ",\n"
        << "    \"epoch_vector_dimension\": "
        << result.diagnostics.native_source_clock_c0d_epoch_vector_dimension << ",\n"
        << "    \"epoch_vector_state_count\": "
        << result.diagnostics.native_source_clock_c0d_epoch_vector_state_count << ",\n"
        << "    \"epoch_vector_handoff_count\": "
        << result.diagnostics.native_source_clock_c0d_epoch_vector_handoff_count << ",\n"
        << "    \"global_isb_state_count\": "
        << result.diagnostics.native_source_clock_c0d_global_isb_state_count << ",\n"
        << "    \"internal_clock_state_unit\": "
        << (meter_state_parity ? "\"metres\"" : "\"seconds\"") << ",\n"
        << "    \"internal_isb_state_unit\": "
        << (meter_state_parity ? "\"metres\"" : "\"seconds\"") << ",\n"
        << "    \"internal_drift_state_unit\": \"metres_per_second\",\n"
        << "    \"clock_c0d_factor_count\": "
        << result.diagnostics.native_source_clock_c0d_factor_count << ",\n"
        << "    \"raw_drift_d_initializer\": {\n"
        << "      \"enabled\": "
        << (result.diagnostics
                    .native_source_clock_c0d_raw_drift_d_initializer_enabled
                ? "true"
                : "false")
        << ",\n"
        << "      \"attempted\": "
        << (result.diagnostics
                    .native_source_clock_c0d_raw_drift_d_initializer_attempted
                ? "true"
                : "false")
        << ",\n"
        << "      \"coverage_valid\": "
        << (result.diagnostics
                    .native_source_clock_c0d_raw_drift_d_initializer_coverage_valid
                ? "true"
                : "false")
        << ",\n"
        << "      \"epoch_count\": "
        << result.diagnostics
               .native_source_clock_c0d_raw_drift_d_initializer_epoch_count
        << ",\n"
        << "      \"finite_count\": "
        << result.diagnostics
               .native_source_clock_c0d_raw_drift_d_initializer_finite_count
        << ",\n"
        << "      \"nonfinite_count\": "
        << result.diagnostics
               .native_source_clock_c0d_raw_drift_d_initializer_nonfinite_count
        << ",\n"
        << "      \"min_mps\": ";
    if (std::isfinite(result.diagnostics
                          .native_source_clock_c0d_raw_drift_d_initializer_min_mps)) {
        out << result.diagnostics
                   .native_source_clock_c0d_raw_drift_d_initializer_min_mps;
    } else {
        out << "null";
    }
    out << ",\n"
        << "      \"max_mps\": ";
    if (std::isfinite(result.diagnostics
                          .native_source_clock_c0d_raw_drift_d_initializer_max_mps)) {
        out << result.diagnostics
                   .native_source_clock_c0d_raw_drift_d_initializer_max_mps;
    } else {
        out << "null";
    }
    out << ",\n"
        << "      \"source_field\": \"EpochSeed.receiver_clock_drift_mps\",\n"
        << "      \"units\": \"metres_per_second\",\n"
        << "      \"fallback\": \"none\",\n"
        << "      \"failure\": ";
    writeJsonString(
        out, result.diagnostics.native_source_clock_c0d_raw_drift_d_initializer_failure);
    out << "\n    },\n"
        << "    \"active_solve_diagnostic_enabled\": "
        << (result.diagnostics
                    .native_source_clock_c0d_active_solve_diagnostic_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"active_solve_attempted\": "
        << (result.diagnostics.native_source_clock_c0d_active_solve_attempted
                ? "true"
                : "false")
        << ",\n"
        << "    \"active_solve_initial_cost\": "
        << result.diagnostics.initial_cost << ",\n"
        << "    \"active_solve_final_cost\": "
        << result.diagnostics.final_cost << ",\n"
        << "    \"accepted_outer_iterations\": "
        << result.diagnostics.native_source_clock_c0d_accepted_outer_iterations
        << ",\n"
        << "    \"total_inner_lambda_attempts\": "
        << result.diagnostics.native_source_clock_c0d_total_inner_lambda_attempts
        << ",\n"
        << "    \"initial_lambda\": "
        << result.diagnostics.native_source_clock_c0d_initial_lambda << ",\n"
        << "    \"maximum_lambda\": "
        << result.diagnostics.native_source_clock_c0d_maximum_lambda << ",\n"
        << "    \"final_lambda\": "
        << result.diagnostics.native_source_clock_c0d_final_lambda << ",\n"
        << "    \"indeterminate_linear_solve_count\": "
        << result.diagnostics
               .native_source_clock_c0d_indeterminate_linear_solve_count
        << ",\n"
        << "    \"unsuccessful_model_step_count\": "
        << result.diagnostics
               .native_source_clock_c0d_unsuccessful_model_step_count
        << ",\n"
        << "    \"small_cost_change_stop_count\": "
        << result.diagnostics
               .native_source_clock_c0d_small_cost_change_stop_count
        << ",\n"
        << "    \"maximum_lambda_stop_count\": "
        << result.diagnostics
               .native_source_clock_c0d_maximum_lambda_stop_count
        << ",\n"
        << "    \"active_solve_finite_costs\": "
        << (result.diagnostics
                    .native_source_clock_c0d_active_solve_finite_costs
                ? "true"
                : "false")
        << ",\n"
        << "    \"termination_trace_complete\": "
        << (result.diagnostics
                    .native_source_clock_c0d_termination_trace_complete
                ? "true"
                : "false")
        << ",\n"
        << "    \"termination_branch_reason\": ";
    writeJsonString(
        out, result.diagnostics.native_source_clock_c0d_termination_branch_reason);
    out << ",\n"
        << "    \"max_whitened_clock_column_norm\": "
        << result.diagnostics
               .native_source_clock_c0d_max_whitened_clock_column_norm
        << ",\n"
        << "    \"max_whitened_drift_column_norm\": "
        << result.diagnostics
               .native_source_clock_c0d_max_whitened_drift_column_norm
        << ",\n"
        << "    \"conditioning_proxy\": "
        << result.diagnostics.native_source_clock_c0d_conditioning_proxy << ",\n"
        << "    \"clock_c0d_clock_jump_skips\": "
        << result.diagnostics.native_source_clock_c0d_clock_jump_skips << ",\n"
        << "    \"clock_c0d_gap_skips\": "
        << result.diagnostics.native_source_clock_c0d_gap_skips << ",\n"
        << "    \"clock_c0d_invalid_dt_skips\": "
        << result.diagnostics.native_source_clock_c0d_invalid_dt_skips << ",\n"
        << "    \"clock_c0d_phone_exclusion_skips\": "
        << result.diagnostics.native_source_clock_c0d_phone_exclusion_skips
        << ",\n"
        << "    \"clock_c0d_dt_min_s\": "
        << result.diagnostics.native_source_clock_c0d_dt_min_s << ",\n"
        << "    \"clock_c0d_dt_max_s\": "
        << result.diagnostics.native_source_clock_c0d_dt_max_s << ",\n"
        << "    \"clock_c0d_equation\": "
        << (meter_state_parity
                ? "\"(C2-C1)-((D1+D2)*dt/2)\""
                : "\"(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))\"")
        << ",\n"
        << "    \"clock_c0d_jacobian_order\": [\"c1\", \"c2\", \"d1\", \"d2\"],\n"
        << "    \"clock_c0d_jacobian\": "
        << (meter_state_parity
                ? "\"[-1,+1,-dt/2,-dt/2]\""
                : "\"[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]\"")
        << ",\n"
        << "    \"clock_c0d_units\": {\"clock\": "
        << (meter_state_parity ? "\"metres\"" : "\"seconds\"")
        << ", \"isb\": "
        << (meter_state_parity ? "\"metres\"" : "\"seconds\"")
        << ", \"drift\": \"metres_per_second\", \"dt\": \"seconds\", "
           "\"residual\": "
        << (meter_state_parity ? "\"metres\"" : "\"seconds\"")
        << ", \"sigma\": "
        << (meter_state_parity ? "\"metres\"" : "\"seconds\"")
        << "},\n"
        << "    \"speed_of_light_mps\": "
        << libgnss::constants::SPEED_OF_LIGHT << ",\n"
        << "    \"clock_c0d_sigma_seconds\": "
        << (0.1 / libgnss::constants::SPEED_OF_LIGHT) << ",\n"
        << "    \"clock_c0d_sigma_m\": 0.1,\n"
        << "    \"public_clock_output_unit\": \"seconds\",\n"
        << "    \"public_clock_output_conversion\": "
           "\"meter_state ? C_i/C_LIGHT : C_i\",\n"
        << "    \"clock_jump_noise\": \"Inf (active C0 factor omitted)\",\n"
        << "    \"legacy_scalar_clock_between_factor_count\": "
        << result.diagnostics.native_source_clock_c0d_legacy_between_factor_count
        << ",\n"
        << "    \"parity_scope\": \"C0/D active-row parity; not full seven-vector\"\n"
        << "  },\n";
    out << "  \"upstream_absolute_doppler_screen\": {\n"
        << "    \"enabled\": "
        << (options.native_upstream_absolute_doppler_screen ? "true" : "false")
        << ",\n"
        << "    \"threshold_mps\": 3.0,\n"
        << "    \"clock_source\": \"raw DriftNanosPerSecond*c/1e9\",\n"
        << "    \"residual_contract\": \"abs(residual_mps-dclk_mps/observation_interval_s)\",\n"
        << "    \"candidates\": "
        << problem.diagnostics.upstream_absolute_doppler_candidates << ",\n"
        << "    \"factors\": "
        << problem.diagnostics.upstream_absolute_doppler_factors << ",\n"
        << "    \"rejections\": "
        << problem.diagnostics.upstream_absolute_doppler_rejections << ",\n"
        << "    \"missing_clock\": "
        << problem.diagnostics.upstream_absolute_doppler_missing_clock << ",\n"
        << "    \"max_abs_corrected_residual_mps\": "
        << problem.diagnostics.upstream_absolute_doppler_max_abs_corrected_residual
        << "\n"
        << "  },\n"
        << "  \"receiver_signal_bias_estimates_m\": {";
    bool first_signal_bias = true;
    for (const auto& [key, value] : result.receiver_signal_bias_estimates_m) {
        if (!first_signal_bias) out << ",";
        first_signal_bias = false;
        out << "\"" << static_cast<int>(static_cast<unsigned char>(key.first))
            << ":" << static_cast<int>(static_cast<unsigned char>(key.second))
            << "\": " << value;
    }
    out << "},\n"
        << "  \"residual_ionosphere_contract\": {\n"
        << "    \"enabled\": "
        << (options.native_residual_ionosphere ? "true" : "false") << ",\n"
        << "    \"prior_sigma_m\": 10.0,\n"
        << "    \"random_walk_sigma_m_per_sqrt_s\": 1.0,\n"
        << "    \"max_abs_state_limit_m\": 30.0,\n"
        << "    \"max_gap_s\": 2.0,\n"
        << "    \"mapping\": \"thin-shell earth-radius 6371000 m, shell-height 350000 m\",\n"
        << "    \"state_units\": \"vertical L1 residual ionosphere metres\",\n"
        << "    \"sign\": \"positive coefficient*state increases predicted pseudorange\",\n"
        << "    \"factors\": " << result.diagnostics.residual_ionosphere_factors << ",\n"
        << "    \"states\": " << result.diagnostics.residual_ionosphere_states << ",\n"
        << "    \"resets\": " << result.diagnostics.residual_ionosphere_resets << ",\n"
        << "    \"invalid_coefficients\": "
        << result.diagnostics.residual_ionosphere_invalid_coefficients << ",\n"
        << "    \"max_abs_state_m\": "
        << result.diagnostics.residual_ionosphere_max_abs_m << ",\n"
        << "    \"rms_state_m\": "
        << result.diagnostics.residual_ionosphere_rms_m << ",\n"
        << "    \"min_coefficient\": "
        << result.diagnostics.residual_ionosphere_min_coefficient << ",\n"
        << "    \"max_coefficient\": "
        << result.diagnostics.residual_ionosphere_max_coefficient << "\n"
        << "  },\n"
        << "  \"tdcp_contract\": {\n"
        << "    \"enabled\": " << (tdcp_report.enabled ? "true" : "false") << ",\n"
        << "    \"official_snr_type_sigma_enabled\": "
        << (options.native_phase117_tdcp_snr_type_sigma ? "true" : "false")
        << ",\n"
        << "    \"official_huber_k_enabled\": "
        << (result.diagnostics.official_tdcp_huber_k_enabled ? "true" : "false")
        << ",\n"
        << "    \"phase184_source_tdcp_huber_k_enabled\": "
        << (result.diagnostics.native_phase184_source_tdcp_huber_k_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"phase184_source_tdcp_setting_type\": ";
    if (result.diagnostics.native_phase184_source_tdcp_huber_k_enabled) {
        writeJsonString(out,
                        result.diagnostics.native_phase184_tdcp_setting_type);
    } else {
        out << "null";
    }
    out << ",\n"
        << "    \"official_resl_atmosphere_cancellation_enabled\": "
        << (result.diagnostics
                    .official_tdcp_resl_atmosphere_cancellation_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"ordinary_measurement_m\": \"carrier_m + satellite_clock_m "
           "(no ionosphere/troposphere) when enabled; legacy corrected carrier "
           "otherwise\",\n"
        << "    \"official_huber_k\": ";
    writePhase94Double(out,
                       result.diagnostics.official_tdcp_huber_threshold_sigma);
    out << ",\n    \"official_setting_type\": ";
    if (result.diagnostics.official_tdcp_huber_k_enabled) {
        writeJsonString(out, result.diagnostics.official_tdcp_setting_type);
    } else {
        out << "null";
    }
    out << ",\n"
        << "    \"official_huber_k_mapping\": "
           "\"Street/Mix=0.2; Highway=0.5; unknown=fail-closed\",\n"
        << "    \"tdcp_only_affine_geometry_requested\": "
        << (options.native_tdcp_only_affine_geometry ? "true" : "false") << ",\n"
        << "    \"epoch_heading_attitude_seeds_requested\": "
        << (options.native_epoch_heading_attitude_seeds ? "true" : "false") << ",\n"
        << "    \"epoch_heading_attitude_seeds_inserted\": "
        << result.diagnostics.epoch_heading_attitude_seeds_inserted << ",\n"
        << "    \"omit_first_imu_bias_prior_requested\": "
        << (options.native_omit_first_imu_bias_prior ? "true" : "false") << ",\n"
        << "    \"first_imu_bias_priors_inserted\": "
        << result.diagnostics.first_imu_bias_priors_inserted << ",\n"
        << "    \"first_imu_bias_priors_omitted\": "
        << result.diagnostics.first_imu_bias_priors_omitted << ",\n"
        << "    \"omit_first_imu_velocity_prior_requested\": "
        << (options.native_omit_first_imu_velocity_prior ? "true" : "false") << ",\n"
        << "    \"pseudorange_remasking_requested\": "
        << (options.native_pseudorange_remasking ? "true" : "false") << ",\n"
        << "    \"pseudorange_remasking_applied\": "
        << (imu_report.pseudorange_remasking_applied ? "true" : "false") << ",\n"
        << "    \"pseudorange_remasking_pool\": " << imu_report.pseudorange_remasking_pool << ",\n"
        << "    \"pseudorange_remasking_old\": " << imu_report.pseudorange_remasking_old << ",\n"
        << "    \"pseudorange_remasking_new\": " << imu_report.pseudorange_remasking_new << ",\n"
        << "    \"pseudorange_remasking_recovered\": " << imu_report.pseudorange_remasking_recovered << ",\n"
        << "    \"pseudorange_remasking_removed\": " << imu_report.pseudorange_remasking_removed << ",\n"
        << "    \"pseudorange_remasking_unchanged\": " << imu_report.pseudorange_remasking_unchanged << ",\n"
        << "    \"pseudorange_remasking_convention\": \"same-run-range-clock-residual-only; fixed-measurements-sigma\",\n"
        << "    \"relative_height_pairs_requested\": "
        << (options.native_relative_height_pairs ? "true" : "false") << ",\n"
        << "    \"relative_height_pairs_selected\": "
        << result.diagnostics.relative_height_pairs_selected << ",\n"
        << "    \"relative_height_factors_inserted\": "
        << result.diagnostics.relative_height_factors_inserted << ",\n"
        << "    \"relative_height_pair_convention\": \"source-cumulative-speed-samples\",\n"
        << "    \"relative_height_sigma_m\": 0.1,\n"
        << "    \"relative_height_huber_k\": 0.5,\n"
        << "    \"relative_height_proximity_m\": 15,\n"
        << "    \"relative_height_speed_sample_sum_threshold\": 100,\n"
        << "    \"relative_height_no_external_reference\": true,\n"
        << "    \"first_imu_velocity_priors_inserted\": "
        << result.diagnostics.first_imu_velocity_priors_inserted << ",\n"
        << "    \"first_imu_velocity_priors_omitted\": "
        << result.diagnostics.first_imu_velocity_priors_omitted << ",\n"
        << "    \"tdcp_only_affine_factors_inserted\": "
        << result.diagnostics.tdcp_only_affine_factors_inserted << ",\n"
        << "    \"gnss_first_tdcp_only_affine_factors_inserted\": "
        << imu_report.gnss_first_tdcp_only_affine_factors_inserted << ",\n"
        << "    \"source_tdcp_resl_observable_requested\": "
        << (options.native_source_tdcp_resl_observable ? "true" : "false") << ",\n"
        << "    \"source_tdcp_meter_sigma_requested\": "
        << (options.native_source_tdcp_meter_sigma ? "true" : "false") << ",\n"
        << "    \"source_tdcp_sigma_units\": \"metres; no wavelength multiplication\",\n"
        << "    \"fixed_sigma_m\": "
        << (options.native_source_tdcp_meter_sigma ? "null" : "0.03") << ",\n"
        << "    \"official_snr_percentile\": "
        << libgnss::observable_upstream::kOfficialSnrPercentile << ",\n"
        << "    \"official_carrier_sigma_units\": "
        << (options.native_source_tdcp_meter_sigma
            ? "\"source resL sigma in metres; no wavelength conversion\",\n"
            : "\"historical Phase117 wavelength-scaled sigma; inactive unless selected\",\n")
        << "    \"official_invalid_weight_fail_closed\": true,\n"
        << "    \"factors_built\": " << tdcp_report.factors_built << ",\n"
        << "    \"factors_inserted\": " << tdcp_report.factors_inserted << ",\n"
        << "    \"candidate_pairs\": " << tdcp_report.candidate_pairs << ",\n"
        << "    \"rejected_gap\": " << tdcp_report.rejected_gap << ",\n"
        << "    \"rejected_clock_discontinuity\": "
        << tdcp_report.rejected_clock_discontinuity << ",\n"
        << "    \"rejected_missing_previous\": "
        << tdcp_report.rejected_missing_previous << ",\n"
        << "    \"rejected_loss_of_lock\": "
        << tdcp_report.rejected_loss_of_lock << ",\n"
        << "    \"rejected_invalid_measurement\": "
        << tdcp_report.rejected_invalid_measurement << ",\n"
        << "    \"rejected_code_phase_jump\": "
        << tdcp_report.rejected_code_phase_jump << ",\n"
        << "    \"rejected_invalid_weight\": "
        << tdcp_report.rejected_invalid_weight << ",\n"
        << "    \"finite_residuals\": " << tdcp_report.finite_residuals << ",\n"
        << "    \"nonfinite_residuals\": " << tdcp_report.nonfinite_residuals << ",\n"
        << "    \"arc_count\": " << tdcp_report.arc_count << ",\n"
        << "    \"min_arc_length_epochs\": "
        << tdcp_report.min_arc_length_epochs << ",\n"
        << "    \"median_arc_length_epochs\": "
        << tdcp_report.median_arc_length_epochs << ",\n"
        << "    \"max_arc_length_epochs\": "
        << tdcp_report.max_arc_length_epochs << ",\n"
        << "    \"sigma_m\": " << tdcp_report.sigma_m << ",\n"
        << "    \"max_gap_s\": " << tdcp_report.max_gap_s << ",\n"
        << "    \"code_phase_jump_threshold_m\": "
        << tdcp_report.code_phase_jump_threshold_m << ",\n"
        << "    \"code_phase_jump_gate_disabled_requested\": "
        << (options.native_tdcp_no_code_jump_gate ? "true" : "false") << ",\n"
        << "    \"sparse_p_staging_requested\": "
        << (options.native_sparse_p_staging ? "true" : "false") << ",\n"
        << "    \"residual_rms_m\": " << tdcp_report.residual_rms_m << ",\n"
        << "    \"normalized_residual_rms\": "
        << tdcp_report.normalized_residual_rms << ",\n"
        << "    \"reconstructed_huber_tail_count\": "
        << tdcp_report.robust_tail_count << ",\n"
        << "    \"reconstructed_huber_cost_sum\": "
        << tdcp_report.robust_cost_sum << ",\n"
        << "    \"reconstructed_quadratic_cost_sum\": "
        << tdcp_report.quadratic_cost_sum << ",\n"
        << "    \"reconstructed_cost_nonfinite_count\": "
        << tdcp_report.robust_cost_nonfinite_count << ",\n"
        << "    \"optimized_imu_bias_count\": "
        << result.diagnostics.optimized_imu_bias_count << ",\n"
        << "    \"nominal_p_information_epochs\": "
        << result.diagnostics.nominal_p_information_epochs << ",\n"
        << "    \"nominal_p_information_rank_deficient_epochs\": "
        << result.diagnostics.nominal_p_information_rank_deficient_epochs << ",\n"
        << "    \"nominal_p_information_min_eigenvalue_per_m2\": "
        << result.diagnostics.nominal_p_information_min_eigenvalue_per_m2 << ",\n"
        << "    \"nominal_p_information_scope\": \"single-epoch nominal P; clocks projected; no robust or temporal weights\",\n"
        << "    \"optimized_accel_bias_max_norm_mps2\": "
        << result.diagnostics.optimized_accel_bias_max_norm_mps2 << ",\n"
        << "    \"optimized_gyro_bias_max_norm_radps\": "
        << result.diagnostics.optimized_gyro_bias_max_norm_radps << ",\n"
        << "    \"frequency_residual_state_count\": "
        << result.diagnostics.tdcp_frequency_residual_states << ",\n"
        << "    \"frequency_residual_factor_count\": "
        << result.diagnostics.tdcp_frequency_residual_factors << ",\n"
        << "    \"frequency_residual_prior_count\": "
        << result.diagnostics.tdcp_frequency_residual_priors << ",\n"
        << "    \"backend_residual_rms_m\": "
        << result.diagnostics.tdcp_residual_rms_m << ",\n"
        << "    \"reconstructed_signal_aggregates\": [";
    bool first_tdcp_signal = true;
    for (const auto& [key, value] : tdcp_report.signal_aggregates) {
        if (!first_tdcp_signal) out << ",";
        first_tdcp_signal = false;
        out << "{\"system_enum\":" << key.first
            << ",\"signal_enum\":" << key.second
            << ",\"count\":" << value.count
            << ",\"tail_count\":" << value.tail_count
            << ",\"residual_sum_m\":" << value.residual_sum_m
            << ",\"residual_squared_sum_m2\":" << value.residual_squared_sum_m2
            << ",\"huber_cost_sum\":" << value.huber_cost_sum
            << ",\"max_abs_residual_m\":" << value.max_abs_residual_m << "}";
    }
    out << "],\n"
        << "    \"max_abs_residual_m\": " << tdcp_report.max_abs_residual_m << ",\n"
        << "    \"pair_key\": \"(satellite,signal)\",\n"
        << "    \"adr_state_slip_fail_closed\": true,\n"
        << "    \"standalone_carrier_ambiguity_factors\": false,\n"
        << "    \"base_or_double_difference_factors\": false\n"
        << "  },\n"
        << "  \"upstream_stop_constraints\": {\n"
        << "    \"enabled\": "
        << (options.native_upstream_stop_constraints ? "true" : "false") << ",\n"
        << "    \"window_samples\": 500,\n"
        << "    \"acceleration_std_offset_mps2\": 0.08,\n"
        << "    \"gyro_std_offset_radps\": 0.005,\n"
        << "    \"gyro_norm_max_radps\": 0.05,\n"
        << "    \"velocity_threshold_mps\": 0.5,\n"
        << "    \"velocity_sigma_mps\": 0.01,\n"
        << "    \"velocity_huber_k_sigma\": 0.5,\n"
        << "    \"pose_rotation_sigma_rad\": 0.0017453292519943296,\n"
        << "    \"pose_translation_sigma_m\": 0.02,\n"
        << "    \"pose_huber_k_sigma\": 0.5,\n"
        << "    \"detection_source\": \"aligned raw IMU acceleration/gyro only\",\n"
        << "    \"epoch_mapping\": \"nearest with endpoint hold\",\n"
        << "    \"no_height_or_truth_factor\": true,\n"
        << "    \"detected_epochs\": "
        << result.diagnostics.upstream_stop_epochs << ",\n"
        << "    \"velocity_factors\": "
        << result.diagnostics.upstream_stop_velocity_factors << ",\n"
        << "    \"pose_factors\": "
        << result.diagnostics.upstream_stop_pose_factors << ",\n"
        << "    \"velocity_key_missing_epochs\": "
        << result.diagnostics.upstream_stop_velocity_key_missing_epochs
        << ",\n"
        << "    \"seed_unavailable_epochs\": "
        << result.diagnostics.upstream_stop_seed_unavailable_epochs
        << ",\n"
        << "    \"seed_nonfinite_epochs\": "
        << result.diagnostics.upstream_stop_seed_nonfinite_epochs
        << ",\n"
        << "    \"graph_velocity_fallback_epochs\": "
        << result.diagnostics.upstream_stop_graph_velocity_fallback_epochs
        << ",\n"
        << "    \"graph_velocity_nonfinite_epochs\": "
        << result.diagnostics.upstream_stop_graph_velocity_nonfinite_epochs
        << ",\n"
        << "    \"speed_nonfinite_epochs\": "
        << result.diagnostics.upstream_stop_speed_nonfinite_epochs
        << ",\n"
        << "    \"speed_evaluated_epochs\": "
        << result.diagnostics.upstream_stop_speed_evaluated_epochs
        << ",\n"
        << "    \"speed_gate_accepted_epochs\": "
        << result.diagnostics.upstream_stop_speed_gate_accepted_epochs
        << ",\n"
        << "    \"speed_gate_rejected_epochs\": "
        << result.diagnostics.upstream_stop_speed_gate_rejected_epochs
        << ",\n"
        << "    \"speed_min_mps\": ";
    if (result.diagnostics.upstream_stop_speed_evaluated_epochs == 0U) {
        out << "null";
    } else {
        out << result.diagnostics.upstream_stop_speed_min_mps;
    }
    out << ",\n"
        << "    \"speed_max_mps\": ";
    if (result.diagnostics.upstream_stop_speed_evaluated_epochs == 0U) {
        out << "null";
    } else {
        out << result.diagnostics.upstream_stop_speed_max_mps;
    }
    out << ",\n"
        << "    \"imu_samples\": "
        << result.diagnostics.upstream_stop_imu_samples << ",\n"
        << "    \"acceleration_std_threshold_mps2\": "
        << result.diagnostics.upstream_stop_acceleration_std_threshold_mps2 << ",\n"
        << "    \"gyro_std_threshold_radps\": "
        << result.diagnostics.upstream_stop_gyro_std_threshold_radps << "\n"
        << "  },\n"
        << "  \"upstream_position_offset\": {\n"
        << "    \"enabled\": "
        << (position_offset_report.enabled ? "true" : "false") << ",\n"
        << "    \"applied\": "
        << (position_offset_report.applied ? "true" : "false") << ",\n"
        << "    \"phone\": ";
    writeJsonString(out, position_offset_report.phone);
    out << ",\n"
        << "    \"corrected_epochs\": "
        << position_offset_report.corrected_epochs << ",\n"
        << "    \"application_passes\": "
        << position_offset_report.application_passes << ",\n"
        << "    \"offset_rl_m\": " << position_offset_report.offset_rl_m << ",\n"
        << "    \"offset_ud_m\": " << position_offset_report.offset_ud_m << ",\n"
        << "    \"max_offset_enu_m\": "
        << position_offset_report.max_offset_enu_m << ",\n"
        << "    \"rotation_contract\": \"Rx*Ry*Rz(rpy-[0,0,pi])\",\n"
        << "    \"source\": \"in-memory optimized GTSAM Rot3::rpy; raw-only\",\n"
        << "    \"failure\": ";
    writeJsonString(out, position_offset_report.failure);
    out << "\n  },\n"
        << "  \"phase127_glonass_channel_provenance\": {\n"
        << "    \"enabled\": "
        << (problem.diagnostics.phase127_glonass_channel_provenance_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"glonass_rows\": "
        << problem.diagnostics.phase127_glonass_rows << ",\n"
        << "    \"accepted_rows\": "
        << problem.diagnostics.phase127_accepted_rows << ",\n"
        << "    \"header_primary_rows\": "
        << problem.diagnostics.phase127_header_primary_rows << ",\n"
        << "    \"ephemeris_fallback_rows\": "
        << problem.diagnostics.phase127_ephemeris_fallback_rows << ",\n"
        << "    \"header_entries_seen\": "
        << problem.diagnostics.phase127_header_entries_seen << ",\n"
        << "    \"header_duplicate_entries\": "
        << problem.diagnostics.phase127_header_duplicate_entries << ",\n"
        << "    \"header_conflict_entries\": "
        << problem.diagnostics.phase127_header_conflict_entries << ",\n"
        << "    \"header_malformed_entries\": "
        << problem.diagnostics.phase127_header_malformed_entries << ",\n"
        << "    \"ephemeris_candidates\": "
        << problem.diagnostics.phase127_ephemeris_candidates << ",\n"
        << "    \"ephemeris_ties\": "
        << problem.diagnostics.phase127_ephemeris_ties << ",\n"
        << "    \"ephemeris_duplicate_entries\": "
        << problem.diagnostics.phase127_ephemeris_duplicate_entries << ",\n"
        << "    \"ephemeris_conflict_entries\": "
        << problem.diagnostics.phase127_ephemeris_conflict_entries << ",\n"
        << "    \"query_time_coverage_gaps\": "
        << problem.diagnostics.phase127_query_time_coverage_gaps << ",\n"
        << "    \"invalid_channels\": "
        << problem.diagnostics.phase127_invalid_channels << ",\n"
        << "    \"failure_counts\": ";
    writeJsonSizeMap(out, problem.diagnostics.phase127_failure_counts);
    out << ",\n    \"failure\": ";
    writeJsonString(out, problem.diagnostics.phase127_failure);
    out << ",\n    \"source_order\": [\"rinex-header\", \"broadcast-geph-frq\"],\n"
           "    \"query_time_window_s\": 1800.0,\n"
           "    \"fcn_domain\": [-7, 6],\n"
           "    \"no_frequency_inference_or_fixed_fallback\": true,\n"
           "    \"no_new_factor_or_state\": true\n"
           "  },\n"
        << "  \"phase128_glonass_provenance_parser_admission\": {\n"
        << "    \"enabled\": "
        << (problem.diagnostics.phase128_glonass_provenance_parser_admission_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"header_status\": ";
    writeJsonString(out, problem.diagnostics.phase128_header_status);
    out << ",\n"
        << "    \"canonical_records\": "
        << problem.diagnostics.phase128_canonical_records << ",\n"
        << "    \"canonical_rejected_records\": "
        << problem.diagnostics.phase128_canonical_rejected_records << ",\n"
        << "    \"typed_satellite_key\": true,\n"
        << "    \"native_gpst_query_and_toe\": true,\n"
        << "    \"header_absent_does_not_short_circuit_nav\": true,\n"
        << "    \"phone_carrier_frequency_is_not_fcn_source\": true,\n"
        << "    \"fixed_fcn_or_external_table\": false,\n"
        << "    \"factor_or_solver_change\": false\n"
        << "  },\n"
        << "  \"phase129_glonass_local_miss_mask\": {\n"
        << "    \"enabled\": "
        << (problem.diagnostics.phase129_glonass_local_miss_mask_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"configuration_valid\": "
        << (problem.diagnostics.phase129_configuration_valid ? "true" : "false")
        << ",\n"
        << "    \"configuration_failure\": ";
    writeJsonString(out, problem.diagnostics.phase129_configuration_failure);
    out << ",\n"
        << "    \"glonass_local_miss_rows\": "
        << problem.diagnostics.phase129_glonass_local_miss_rows << ",\n"
        << "    \"glonass_factor_rows_dropped\": "
        << problem.diagnostics.phase129_glonass_factor_rows_dropped << ",\n"
        << "    \"glonass_factor_rows_retained\": "
        << problem.diagnostics.phase129_glonass_factor_rows_retained << ",\n"
        << "    \"factor_count_consistent\": "
        << (problem.diagnostics.phase129_glonass_factor_count_consistent
                ? "true"
                : "false")
        << ",\n"
        << "    \"glonass_local_miss_counts\": ";
    writeJsonSizeMap(out, problem.diagnostics.phase129_glonass_local_miss_counts);
    out << ",\n"
        << "    \"row_count_consistent\": "
        << (problem.diagnostics.phase129_glonass_row_count_consistent
                ? "true"
                : "false")
        << ",\n"
        << "    \"raw_uncorrected_or_zero_fallback\": false,\n"
        << "    \"no_partial_application\": true,\n"
        << "    \"shared_reason_ledger\": true,\n"
        << "    \"factor_topology_changed\": false\n"
        << "  },\n"
        << "  \"phase131_canonical_correction_band_key\": {\n"
        << "    \"enabled\": "
        << (problem.diagnostics.phase131_canonical_correction_band_key_enabled
                ? "true"
                : "false")
        << ",\n"
        << "    \"configuration_valid\": "
        << (problem.diagnostics.phase131_configuration_valid ? "true" : "false")
        << ",\n"
        << "    \"configuration_failure\": ";
    writeJsonString(out, problem.diagnostics.phase131_configuration_failure);
    out << ",\n"
        << "    \"canonical_rows\": "
        << problem.diagnostics.phase131_canonical_rows << ",\n"
        << "    \"canonical_rejected_rows\": "
        << problem.diagnostics.phase131_canonical_rejected_rows << ",\n"
        << "    \"unknown_band_rows\": "
        << problem.diagnostics.phase131_unknown_band_rows << ",\n"
        << "    \"canonical_key_conflicts\": "
        << problem.diagnostics.phase131_canonical_key_conflicts << ",\n"
        << "    \"canonical_duplicate_rows\": "
        << problem.diagnostics.phase131_canonical_duplicate_rows << ",\n"
        << "    \"canonical_streams\": "
        << problem.diagnostics.phase131_canonical_streams << ",\n"
        << "    \"canonical_selected_streams\": "
        << problem.diagnostics.phase131_canonical_selected_streams << ",\n"
        << "    \"canonical_merged_streams\": "
        << problem.diagnostics.phase131_canonical_merged_streams << ",\n"
        << "    \"failure_counts\": ";
    writeJsonSizeMap(out, problem.diagnostics.phase131_failure_counts);
    out << ",\n"
        << "    \"join_key\": \"GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN]\",\n"
        << "    \"literal_tracking_code_in_join\": false,\n"
        << "    \"factor_topology_changed\": false,\n"
        << "    \"raw_or_zero_correction_fallback\": false";
    // The bridge fields are emitted only for the selector-on path.  Keeping
    // them conditional preserves the legacy/default summary shape while
    // making the authoritative native counters and conservation ledger
    // available to the opt-in runner.
    if (problem.diagnostics.phase131_diagnostics_bridge_sync_count != 0U) {
        out << ",\n"
            << "    \"canonicalization_attempt_rows\": "
            << problem.diagnostics.phase131_canonicalization_attempt_rows
            << ",\n"
            << "    \"resolver_call_count\": "
            << problem.diagnostics.phase131_resolver_call_count
            << ",\n"
            << "    \"correction_conservation\": {\n"
            << "      \"source_miss_mask_enabled\": "
            << (problem.diagnostics.phase131_source_miss_mask_enabled
                    ? "true"
                    : "false")
            << ",\n"
            << "      \"canonical_key_mode\": "
            << (problem.diagnostics.phase131_source_miss_mask_canonical_key_mode
                    ? "true"
                    : "false")
            << ",\n"
            << "      \"matching_key\": ";
        writeJsonString(
            out, problem.diagnostics.phase131_source_miss_mask_matching_key);
        out << ",\n"
            << "      \"original_adopted_pseudorange_rows\": "
            << problem.diagnostics.phase131_original_adopted_pseudorange_rows
            << ",\n"
            << "      \"retained_finite_pc_pseudorange_rows\": "
            << problem.diagnostics.phase131_retained_finite_pc_pseudorange_rows
            << ",\n"
            << "      \"dropped_missing_exact_stream_rows\": "
            << problem.diagnostics.phase131_dropped_missing_exact_stream_rows
            << ",\n"
            << "      \"dropped_out_of_domain_rows\": "
            << problem.diagnostics.phase131_dropped_out_of_domain_rows
            << ",\n"
            << "      \"dropped_nonfinite_correction_rows\": "
            << problem.diagnostics.phase131_dropped_nonfinite_correction_rows
            << ",\n"
            << "      \"matched_factor_rows\": "
            << problem.diagnostics.phase131_matched_factor_rows << ",\n"
            << "      \"finite_correction_rows_among_matched\": "
            << problem.diagnostics.phase131_finite_correction_rows_among_matched
            << ",\n"
            << "      \"source_model_build_count\": "
            << problem.diagnostics.phase131_source_model_build_count << ",\n"
            << "      \"correction_application_pass_count\": "
            << problem.diagnostics.phase131_correction_application_pass_count
            << ",\n"
            << "      \"corrected_rows\": "
            << problem.diagnostics.phase131_corrected_rows << ",\n"
            << "      \"pseudorange_factor_count_consistent\": "
            << (problem.diagnostics.phase131_pseudorange_factor_count_consistent
                    ? "true"
                    : "false")
            << ",\n"
            << "      \"signal_count_consistent\": "
            << (problem.diagnostics.phase131_signal_count_consistent ? "true"
                                                                      : "false")
            << ",\n"
            << "      \"applied\": "
            << (problem.diagnostics.phase131_applied ? "true" : "false")
            << ",\n"
            << "      \"correction_applied_exactly_once\": "
            << (problem.diagnostics.phase131_correction_applied_exactly_once
                    ? "true"
                    : "false")
            << ",\n"
            << "      \"duplicate_correction_rejected\": "
            << (problem.diagnostics.phase131_duplicate_correction_rejected
                    ? "true"
                    : "false")
            << "\n"
            << "    },\n"
            << "    \"diagnostics_bridge\": {\n"
            << "      \"source\": \"BasePseudorangeCompensationReport\",\n"
            << "      \"synchronization_count\": "
            << problem.diagnostics.phase131_diagnostics_bridge_sync_count
            << ",\n"
            << "      \"exactly_once\": "
            << (problem.diagnostics.phase131_diagnostics_bridge_sync_count == 1U
                    ? "true"
                    : "false")
            << "\n"
            << "    }";
    }
    out << "\n  },\n"
        << "  \"quality_anchor_initialization\": {\n"
        << "    \"enabled\": "
        << (problem.diagnostics.quality_anchor_initialization_enabled
                ? "true"
                : "false") << ",\n"
        << "    \"selected\": "
        << (problem.diagnostics.quality_anchor_selected ? "true" : "false")
        << ",\n"
        << "    \"anchor_index\": ";
    if (problem.diagnostics.quality_anchor_selected) {
        out << problem.diagnostics.quality_anchor_index;
    } else {
        out << "null";
    }
    out << ",\n"
        << "    \"eligible_candidates\": "
        << problem.diagnostics.quality_anchor_candidates << ",\n"
        << "    \"forward_valid_epochs\": "
        << problem.diagnostics.quality_anchor_forward_valid_epochs << ",\n"
        << "    \"backward_valid_epochs\": "
        << problem.diagnostics.quality_anchor_backward_valid_epochs << ",\n"
        << "    \"fallback_epochs\": "
        << problem.diagnostics.quality_anchor_fallback_epochs << ",\n"
        << "    \"anchor_satellites\": "
        << problem.diagnostics.quality_anchor_satellites << ",\n"
        << "    \"anchor_gdop\": ";
    if (std::isfinite(problem.diagnostics.quality_anchor_gdop)) {
        out << problem.diagnostics.quality_anchor_gdop;
    } else {
        out << "null";
    }
    out << ",\n"
        << "    \"anchor_normalized_residual_rms\": ";
    if (std::isfinite(problem.diagnostics.quality_anchor_normalized_residual_rms)) {
        out << problem.diagnostics.quality_anchor_normalized_residual_rms;
    } else {
        out << "null";
    }
    out << ",\n";
    if (options.native_fallback_seed_quality_anchor_recovery) {
        out << "    \"recovery_enabled\": "
            << (problem.diagnostics.quality_anchor_recovery_enabled
                    ? "true"
                    : "false") << ",\n"
            << "    \"normal_quality_anchor_candidates\": "
            << problem.diagnostics.quality_anchor_normal_candidates << ",\n"
            << "    \"recovery_quality_anchor_candidates\": "
            << problem.diagnostics.quality_anchor_recovery_candidates << ",\n"
            << "    \"recovery_trigger\": "
            << (problem.diagnostics.quality_anchor_recovery_triggered
                    ? "true"
                    : "false") << ",\n"
            << "    \"recovery_triggered\": "
            << (problem.diagnostics.quality_anchor_recovery_triggered
                    ? "true"
                    : "false") << ",\n"
            << "    \"recovery_anchor_selected\": "
            << (problem.diagnostics.quality_anchor_recovery_selected
                    ? "true"
                    : "false") << ",\n"
            << "    \"recovery_anchor_index\": ";
        if (problem.diagnostics.quality_anchor_recovery_selected) {
            out << problem.diagnostics.quality_anchor_recovery_anchor_index;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"recovery_anchor_satellites\": "
            << problem.diagnostics.quality_anchor_recovery_anchor_satellites
            << ",\n"
            << "    \"recovery_anchor_gdop\": ";
        if (std::isfinite(problem.diagnostics.quality_anchor_recovery_anchor_gdop)) {
            out << problem.diagnostics.quality_anchor_recovery_anchor_gdop;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"recovery_anchor_normalized_residual_rms\": ";
        if (std::isfinite(
                problem.diagnostics
                    .quality_anchor_recovery_anchor_normalized_residual_rms)) {
            out << problem.diagnostics
                       .quality_anchor_recovery_anchor_normalized_residual_rms;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"recovery_replay_valid_epochs\": "
            << problem.diagnostics.quality_anchor_recovery_replay_valid_epochs
            << ",\n"
            << "    \"recovery_replay_invalid_epochs\": "
            << problem.diagnostics.quality_anchor_recovery_replay_invalid_epochs
            << ",\n"
            << "    \"sentinel_factor_bypass\": "
            << (problem.diagnostics.sentinel_factor_bypass ? "true" : "false")
            << ",\n";
    }
    out << "    \"ranking\": [\"satellites_desc\", \"gdop_asc\", "
           "\"normalized_residual_rms_asc\", \"input_index_asc\"],\n"
        << "    \"truth_free\": true,\n"
        << "    \"graph_model_changed\": false\n"
        << "  },\n"
        << "  \"graph\": {\n"
        << "    \"factors\": " << result.diagnostics.graph_factors << ",\n"
        << "    \"values\": " << result.diagnostics.graph_values << ",\n"
        << "    \"imu_intervals\": " << result.diagnostics.imu_intervals << ",\n"
        << "    \"phase217_main_motion\": {"
        << "\"requested\": " << (options.native_phase217_main_pose3_motion ? "true" : "false")
        << ", \"enabled\": " << (result.diagnostics.native_phase217_main_motion_enabled ? "true" : "false")
        << ", \"factors\": " << result.diagnostics.native_phase217_main_motion_factors
        << ", \"gap_skips\": " << result.diagnostics.native_phase217_main_motion_gap_skips
        << ", \"sigma_m\": 0.05, \"gap_threshold_s\": 1.5},\n"
        << "    \"phase213_main_doppler\": {"
        << "\"requested\": " << (options.native_phase213_main_doppler ? "true" : "false")
        << ", \"enabled\": " << (result.diagnostics.native_phase213_main_doppler_enabled ? "true" : "false")
        << ", \"factors\": " << result.diagnostics.native_phase213_main_doppler_factors
        << "},\n"
        << "    \"phase209_separate_imu\": {"
        << "\"requested\": " << (options.native_phase209_source_separate_imu_factors ? "true" : "false")
        << ", \"enabled\": " << (result.diagnostics.native_phase209_separate_imu_enabled ? "true" : "false")
        << ", \"motion_factors\": " << result.diagnostics.native_phase209_motion_factors
        << ", \"bias_factors\": " << result.diagnostics.native_phase209_bias_factors
        << ", \"inclusive_samples\": " << result.diagnostics.native_phase209_inclusive_samples
        << "},\n"
        << "    \"phase205_bias_density\": {"
        << "\"requested\": " << (options.native_phase205_source_count_bias_density ? "true" : "false")
        << ", \"enabled\": " << (result.diagnostics.native_phase205_bias_density_enabled ? "true" : "false")
        << ", \"intervals\": " << result.diagnostics.native_phase205_bias_density_intervals
        << ", \"inclusive_samples\": " << result.diagnostics.native_phase205_bias_density_samples
        << ", \"scale_min\": " << result.diagnostics.native_phase205_bias_density_scale_min
        << ", \"scale_max\": " << result.diagnostics.native_phase205_bias_density_scale_max
        << "},\n"
        << "    \"phase201_source_inclusive_forward_imu_schedule\": {\n"
        << "      \"requested\": "
        << (options.native_phase201_source_inclusive_forward_imu_schedule
                ? "true"
                : "false")
        << ",\n"
        << "      \"selected\": "
        << (result.diagnostics
                    .native_phase201_source_inclusive_forward_imu_schedule_enabled
                ? "true"
                : "false")
        << ",\n"
        << "      \"attempted\": "
        << (result.diagnostics
                    .native_phase201_source_inclusive_forward_imu_schedule_attempted
                ? "true"
                : "false")
        << ",\n"
        << "      \"configuration_valid\": "
        << (result.diagnostics
                    .native_phase201_source_inclusive_forward_imu_schedule_configuration_valid
                ? "true"
                : "false")
        << ",\n"
        << "      \"intervals\": "
        << result.diagnostics.native_phase201_intervals << ",\n"
        << "      \"intervals_with_samples\": "
        << result.diagnostics.native_phase201_intervals_with_samples << ",\n"
        << "      \"inserted_samples\": "
        << result.diagnostics.native_phase201_inserted_samples << ",\n"
        << "      \"invalid_sample_count\": "
        << result.diagnostics.native_phase201_invalid_sample_count << ",\n"
        << "      \"nonfinite_dt_count\": "
        << result.diagnostics.native_phase201_nonfinite_dt_count << ",\n"
        << "      \"nonpositive_dt_count\": "
        << result.diagnostics.native_phase201_nonpositive_dt_count << ",\n"
        << "      \"empty_interval_count\": "
        << result.diagnostics.native_phase201_empty_interval_count << ",\n"
        << "      \"integrated_duration_min_s\": ";
    writePhase94Double(
        out, result.diagnostics.native_phase201_integrated_duration_min_s);
    out << ",\n"
        << "      \"integrated_duration_max_s\": ";
    writePhase94Double(
        out, result.diagnostics.native_phase201_integrated_duration_max_s);
    out << ",\n"
        << "      \"gnss_interval_duration_min_s\": ";
    writePhase94Double(
        out, result.diagnostics.native_phase201_gnss_interval_duration_min_s);
    out << ",\n"
        << "      \"gnss_interval_duration_max_s\": ";
    writePhase94Double(
        out, result.diagnostics.native_phase201_gnss_interval_duration_max_s);
    out << ",\n"
        << "      \"duration_error_min_s\": ";
    writePhase94Double(
        out, result.diagnostics.native_phase201_duration_error_min_s);
    out << ",\n"
        << "      \"duration_error_max_s\": ";
    writePhase94Double(
        out, result.diagnostics.native_phase201_duration_error_max_s);
    out << ",\n"
        << "      \"duration_error_max_abs_s\": ";
    writePhase94Double(
        out, result.diagnostics.native_phase201_duration_error_max_abs_s);
    out << ",\n"
        << "      \"failure\": ";
    writeJsonString(
        out, result.diagnostics.native_phase201_configuration_failure);
    out << "\n    },\n"
        << "    \"upstream_stop_epochs\": "
        << result.diagnostics.upstream_stop_epochs << ",\n"
        << "    \"upstream_stop_velocity_factors\": "
        << result.diagnostics.upstream_stop_velocity_factors << ",\n"
        << "    \"upstream_stop_pose_factors\": "
        << result.diagnostics.upstream_stop_pose_factors << ",\n"
        << "    \"upstream_stop_velocity_key_missing_epochs\": "
        << result.diagnostics.upstream_stop_velocity_key_missing_epochs
        << ",\n"
        << "    \"upstream_stop_seed_unavailable_epochs\": "
        << result.diagnostics.upstream_stop_seed_unavailable_epochs
        << ",\n"
        << "    \"upstream_stop_seed_nonfinite_epochs\": "
        << result.diagnostics.upstream_stop_seed_nonfinite_epochs
        << ",\n"
        << "    \"upstream_stop_graph_velocity_fallback_epochs\": "
        << result.diagnostics.upstream_stop_graph_velocity_fallback_epochs
        << ",\n"
        << "    \"upstream_stop_graph_velocity_nonfinite_epochs\": "
        << result.diagnostics.upstream_stop_graph_velocity_nonfinite_epochs
        << ",\n"
        << "    \"upstream_stop_speed_nonfinite_epochs\": "
        << result.diagnostics.upstream_stop_speed_nonfinite_epochs
        << ",\n"
        << "    \"upstream_stop_speed_evaluated_epochs\": "
        << result.diagnostics.upstream_stop_speed_evaluated_epochs
        << ",\n"
        << "    \"upstream_stop_speed_gate_accepted_epochs\": "
        << result.diagnostics.upstream_stop_speed_gate_accepted_epochs
        << ",\n"
        << "    \"upstream_stop_speed_gate_rejected_epochs\": "
        << result.diagnostics.upstream_stop_speed_gate_rejected_epochs
        << ",\n"
        << "    \"upstream_stop_speed_min_mps\": ";
    if (result.diagnostics.upstream_stop_speed_evaluated_epochs == 0U) {
        out << "null";
    } else {
        out << result.diagnostics.upstream_stop_speed_min_mps;
    }
    out << ",\n"
        << "    \"upstream_stop_speed_max_mps\": ";
    if (result.diagnostics.upstream_stop_speed_evaluated_epochs == 0U) {
        out << "null";
    } else {
        out << result.diagnostics.upstream_stop_speed_max_mps;
    }
    out << ",\n"
        << "    \"upstream_stop_imu_samples\": "
        << result.diagnostics.upstream_stop_imu_samples << ",\n"
        << "    \"upstream_stop_acceleration_std_threshold_mps2\": "
        << result.diagnostics.upstream_stop_acceleration_std_threshold_mps2 << ",\n"
        << "    \"upstream_stop_gyro_std_threshold_radps\": "
        << result.diagnostics.upstream_stop_gyro_std_threshold_radps << ",\n"
        << "    \"iterations\": " << result.diagnostics.iterations << ",\n"
        << "    \"converged\": " << (result.diagnostics.converged ? "true" : "false") << ",\n"
        << "    \"initial_cost\": " << result.diagnostics.initial_cost << ",\n"
        << "    \"final_cost\": " << result.diagnostics.final_cost << "\n"
        << "  },\n"
        << "  \"carrier_code_leveling\": {\n"
        << "    \"enabled\": "
        << (carrier_code_leveling_report.enabled ? "true" : "false") << ",\n"
        << "    \"signal\": ";
    writeJsonString(out, options.native_carrier_code_gal_e1_e5a
                             ? "GAL_E1+GAL_E5A"
                             : options.native_carrier_code_primary_l1_e1
                                   ? "GPS_L1CA+GAL_E1"
                                   : "Galileo E1");
    out << ",\n"
        << "    \"window_samples\": 30,\n"
        << "    \"max_gap_s\": 1.5,\n"
        << "    \"target_rows\": "
        << carrier_code_leveling_report.target_rows << ",\n"
        << "    \"eligible_rows\": "
        << carrier_code_leveling_report.eligible_rows << ",\n"
        << "    \"smoothed_rows\": "
        << carrier_code_leveling_report.smoothed_rows << ",\n"
        << "    \"arcs_started\": "
        << carrier_code_leveling_report.arcs_started << ",\n"
        << "    \"updates\": " << carrier_code_leveling_report.updates << ",\n"
        << "    \"reset_invalid_code\": "
        << carrier_code_leveling_report.reset_invalid_code << ",\n"
        << "    \"reset_invalid_adr\": "
        << carrier_code_leveling_report.reset_invalid_adr << ",\n"
        << "    \"reset_adr\": "
        << carrier_code_leveling_report.reset_adr << ",\n"
        << "    \"reset_cycle_slip\": "
        << carrier_code_leveling_report.reset_cycle_slip << ",\n"
        << "    \"reset_missing_or_nonfinite_adr\": "
        << carrier_code_leveling_report.reset_missing_or_nonfinite_adr << ",\n"
        << "    \"reset_gap\": "
        << carrier_code_leveling_report.reset_gap << ",\n"
        << "    \"reset_clock_discontinuity\": "
        << carrier_code_leveling_report.reset_clock_discontinuity << ",\n"
        << "    \"innovation_reset_enabled\": "
        << (options.native_carrier_code_innovation_reset ? "true" : "false")
        << ",\n"
        << "    \"innovation_reset_threshold_m\": "
        << carrier_code_leveling_report.innovation_reset_threshold_m << ",\n"
        << "    \"reset_innovation\": "
        << carrier_code_leveling_report.reset_innovation << ",\n"
        << "    \"max_abs_innovation_accepted_m\": "
        << carrier_code_leveling_report.max_abs_innovation_accepted_m << ",\n"
        << "    \"max_abs_innovation_rejected_m\": "
        << carrier_code_leveling_report.max_abs_innovation_rejected_m << ",\n"
        << "    \"max_abs_level_adjustment_m\": "
        << carrier_code_leveling_report.max_abs_level_adjustment_m << ",\n"
        << "    \"max_abs_phase_increment_m\": "
        << carrier_code_leveling_report.max_abs_phase_increment_m << ",\n"
        ;
    if (options.native_carrier_code_primary_l1_e1 ||
        options.native_carrier_code_gal_e1_e5a) {
        out << "    \"signals\": [";
        if (options.native_carrier_code_gal_e1_e5a) {
            out << "\"GAL_E1\", \"GAL_E5A\"";
        } else {
            out << "\"GPS_L1CA\", \"GAL_E1\"";
        }
        out << "],\n"
            << "    \"per_signal\": [";
        for (std::size_t index = 0U;
             index < carrier_code_leveling_report.per_signal.size(); ++index) {
            if (index != 0U) out << ",";
            const auto& signal_diagnostics =
                carrier_code_leveling_report.per_signal[index];
            out << "\n      {\n        \"signal\": ";
            writeJsonString(out, carrierSignalName(signal_diagnostics.signal));
            out << ",\n"
                << "        \"target_rows\": "
                << signal_diagnostics.target_rows << ",\n"
                << "        \"eligible_rows\": "
                << signal_diagnostics.eligible_rows << ",\n"
                << "        \"smoothed_rows\": "
                << signal_diagnostics.smoothed_rows << ",\n"
                << "        \"arcs_started\": "
                << signal_diagnostics.arcs_started << ",\n"
                << "        \"updates\": " << signal_diagnostics.updates << ",\n"
                << "        \"reset_invalid_code\": "
                << signal_diagnostics.reset_invalid_code << ",\n"
                << "        \"reset_invalid_adr\": "
                << signal_diagnostics.reset_invalid_adr << ",\n"
                << "        \"reset_adr\": "
                << signal_diagnostics.reset_adr << ",\n"
                << "        \"reset_cycle_slip\": "
                << signal_diagnostics.reset_cycle_slip << ",\n"
                << "        \"reset_missing_or_nonfinite_adr\": "
                << signal_diagnostics.reset_missing_or_nonfinite_adr << ",\n"
                << "        \"reset_gap\": " << signal_diagnostics.reset_gap << ",\n"
                << "        \"reset_clock_discontinuity\": "
                << signal_diagnostics.reset_clock_discontinuity << ",\n"
                << "        \"reset_innovation\": "
                << signal_diagnostics.reset_innovation << ",\n"
                << "        \"innovation_reset_threshold_m\": "
                << signal_diagnostics.innovation_reset_threshold_m << ",\n"
                << "        \"max_abs_innovation_accepted_m\": "
                << signal_diagnostics.max_abs_innovation_accepted_m << ",\n"
                << "        \"max_abs_innovation_rejected_m\": "
                << signal_diagnostics.max_abs_innovation_rejected_m << "\n"
                << "      }";
        }
        out << "\n    ],\n";
    }
    out << "    \"no_new_graph_state\": true,\n"
        << "    \"truth_free\": true\n"
        << "  },\n"
        << "  \"gnss_first\": {\n"
        << "    \"attempted\": " << (imu_report.gnss_first_attempted ? "true" : "false") << ",\n"
        << "    \"converged\": " << (imu_report.gnss_first_converged ? "true" : "false") << ",\n"
        << "    \"epochs\": " << imu_report.gnss_first_epochs << ",\n"
        << "    \"undifferenced_doppler_factors\": "
        << imu_report.gnss_first_doppler_factors << ",\n"
        << "    \"ecef_doppler_stage_enabled\": "
        << (imu_report.gnss_first_ecef_doppler_stage_enabled ? "true"
                                                             : "false")
        << ",\n"
        << "    \"doppler_frame\": ";
    writeJsonString(out, options.native_phase171_raw_p_ecef_doppler_gnss_first
                             ? "ECEF"
                             : "none");
    out << ",\n"
        << "    \"main_generic_doppler_factors\": "
        << result.diagnostics.undifferenced_doppler_factors_inserted << ",\n"
        << "    \"doppler_factors_inserted\": "
        << imu_report.gnss_first_doppler_factors_inserted << ",\n"
        << "    \"velocity_states_exported\": "
        << imu_report.gnss_first_velocity_states << ",\n"
        << "    \"iterations\": " << imu_report.gnss_first_iterations << ",\n"
        << "    \"initial_cost\": " << imu_report.gnss_first_initial_cost << ",\n"
        << "    \"final_cost\": " << imu_report.gnss_first_final_cost << ",\n"
        << "    \"c0d_factor_count\": "
        << imu_report.gnss_first_c0d_factor_count << ",\n"
        << "    \"c0d_accepted_outer_iterations\": "
        << imu_report.gnss_first_c0d_accepted_outer_iterations << ",\n"
        << "    \"c0d_active_solve_finite_costs\": "
        << (imu_report.gnss_first_c0d_active_solve_finite_costs ? "true" : "false")
        << ",\n"
        << "    \"failure\": ";
    writeJsonString(out, imu_report.gnss_first_failure);
    if (options.native_gnss_first_velocity_only_handoff) {
        out << ",\n"
            << "    \"handoff_mode\": \"velocity-only\",\n"
            << "    \"position_invalid_count\": "
            << imu_report.gnss_first_position_invalid_count << ",\n"
            << "    \"position_nonfinite_count\": "
            << imu_report.gnss_first_position_nonfinite_count << ",\n"
            << "    \"position_out_of_earth_count\": "
            << imu_report.gnss_first_position_out_of_earth_count << ",\n"
            << "    \"first_invalid_position_norm_m\": ";
        if (std::isfinite(imu_report.gnss_first_first_invalid_position_norm_m)) {
            out << imu_report.gnss_first_first_invalid_position_norm_m;
        } else {
            out << "null";
        }
        out << ",\n"
            << "    \"clock_invalid_count\": "
            << imu_report.gnss_first_clock_invalid_count << ",\n"
            << "    \"velocity_valid_count\": "
            << imu_report.gnss_first_velocity_valid_count << ",\n"
            << "    \"velocity_nonfinite_count\": "
            << imu_report.gnss_first_velocity_nonfinite_count << ",\n"
            << "    \"velocity_over_70_mps_count\": "
            << imu_report.gnss_first_velocity_over_bound_count << ",\n"
            << "    \"max_velocity_norm_mps\": "
            << imu_report.gnss_first_max_velocity_norm_mps << ",\n"
            << "    \"velocity_handoff_source\": \"gnss-first-optimizer-result\",\n"
            << "    \"velocity_initializer\": \"raw-doppler-wls\",\n"
            << "    \"velocity_initializer_direct_count\": "
            << imu_report.gnss_first_velocity_initializer_direct_count << ",\n"
            << "    \"velocity_initializer_propagated_count\": "
            << imu_report.gnss_first_velocity_initializer_propagated_count << ",\n"
            << "    \"velocity_initializer_edge_hold_count\": "
            << imu_report.gnss_first_velocity_initializer_edge_hold_count << ",\n"
            << "    \"velocity_initializer_edge_hold_max_s\": "
            << imu_report.gnss_first_velocity_initializer_edge_hold_max_s << ",\n"
            << "    \"original_raw_seed_position_count\": "
            << imu_report.original_raw_seed_position_count << ",\n"
            << "    \"original_raw_seed_position_invalid_count\": "
            << imu_report.original_raw_seed_position_invalid_count << ",\n"
            << "    \"positions_clocks_copied\": "
            << imu_report.gnss_first_positions_clocks_copied << "\n";
    } else if (options.native_direct_wls_ephemeral_c7d_main_seed) {
        out << ",\n"
            << "    \"handoff_mode\": \"main-direct-wls-ephemeral-c7d-seed\",\n"
            << "    \"velocity_handoff_source\": \"same-run-raw-doppler-wls-ecef\",\n"
            << "    \"position_clock_handoff_source\": \"same-run-raw-spp-epoch-seed\",\n"
            << "    \"clock_drift_handoff_source\": \"same-run-retained-raw-epoch-seed\",\n"
            << "    \"clock_drift_unit\": \"metres_per_second\",\n"
            << "    \"c_vector_unit\": \"metres\",\n"
            << "    \"c_vector_dimension\": 7,\n"
            << "    \"c_vector_component_order\": [\"base_gps_l1\",\"glo_l1\",\"gal_l1\",\"bds_l1\",\"gps_l5\",\"gal_l5\",\"bds_l5\"],\n"
            << "    \"direct_seed_valid\": "
            << (imu_report.direct_wls_ephemeral_c7d_main_seed_valid ? "true" : "false")
            << ",\n"
            << "    \"raw_epoch_count\": "
            << imu_report.direct_wls_ephemeral_raw_epoch_count << ",\n"
            << "    \"retained_epoch_count\": "
            << imu_report.direct_wls_ephemeral_retained_epoch_count << ",\n"
            << "    \"exact_key_count\": "
            << imu_report.direct_wls_ephemeral_exact_key_count << ",\n"
            << "    \"finite_position_count\": "
            << imu_report.direct_wls_ephemeral_finite_position_count << ",\n"
            << "    \"finite_clock_count\": "
            << imu_report.direct_wls_ephemeral_finite_clock_count << ",\n"
            << "    \"finite_drift_count\": "
            << imu_report.direct_wls_ephemeral_finite_drift_count << ",\n"
            << "    \"finite_velocity_count\": "
            << imu_report.direct_wls_ephemeral_finite_velocity_count << ",\n"
            << "    \"raw_key_mismatch_count\": "
            << imu_report.direct_wls_ephemeral_raw_key_mismatch_count << ",\n"
            << "    \"raw_key_order_mismatch_count\": "
            << imu_report.direct_wls_ephemeral_raw_key_order_mismatch_count << ",\n"
            << "    \"raw_drift_mismatch_count\": "
            << imu_report.direct_wls_ephemeral_raw_drift_mismatch_count << ",\n"
            << "    \"full_raw_coverage\": "
            << (imu_report.direct_wls_ephemeral_full_raw_coverage ? "true" : "false")
            << ",\n"
            << "    \"positions_clocks_copied\": 0,\n"
            << "    \"gnss_first_stage_run\": false,\n"
            << "    \"forbidden_coordinate_sources\": [\"GNSS-first-result\",\"PDC\",\"external\",\"precomputed\"],\n"
            << "    \""
            << (options.native_phase144_telemetry_schema ? "handoff_failure"
                                                           : "failure")
            << "\": ";
        writeJsonString(out, imu_report.direct_wls_ephemeral_failure);
    } else if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {
        out << ",\n"
            << "    \"handoff_mode\": "
               "\"gnss-first-in-memory-meter-clock-state\",\n"
            << "    \"velocity_handoff_source\": "
               "\"same-run-gnss-first-optimizer-result\",\n"
            << "    \"position_clock_handoff_source\": "
               "\"same-run-gnss-first-optimizer-result\",\n"
            << "    \"clock_drift_handoff_source\": "
               "\"same-run-gnss-first-optimizer-result\",\n"
            << "    \"clock_drift_unit\": \"metres_per_second\",\n"
            << "    \"optimized_d_export_valid\": "
            << (imu_report.phase93_optimized_d_export_valid ? "true" : "false")
            << ",\n"
            << "    \"optimized_d_epoch_count\": "
            << imu_report.phase93_optimized_d_epoch_count << ",\n"
            << "    \"optimized_d_finite_count\": "
            << imu_report.phase93_optimized_d_finite_count << ",\n"
            << "    \"optimized_d_nonfinite_count\": "
            << imu_report.phase93_optimized_d_nonfinite_count << ",\n"
            << "    \"optimized_c_vector_parity_enabled\": "
            << (options.native_source_clock_c0d_epoch_vector_parity ? "true" : "false")
            << ",\n"
            << "    \"optimized_c_export_valid\": "
            << (imu_report.phase101_optimized_c_export_valid ? "true" : "false")
            << ",\n"
            << "    \"optimized_c_epoch_count\": "
            << imu_report.phase101_optimized_c_epoch_count << ",\n"
            << "    \"optimized_c_finite_component_count\": "
            << imu_report.phase101_optimized_c_finite_count << ",\n"
            << "    \"optimized_c_nonfinite_component_count\": "
            << imu_report.phase101_optimized_c_nonfinite_count << ",\n"
            << "    \"optimized_c_dimension\": 7,\n"
            << "    \"optimized_c_component_order\": "
               "[\"base_gps_l1\",\"glo_l1\",\"gal_l1\",\"bds_l1\","
               "\"gps_l5\",\"gal_l5\",\"bds_l5\"],\n"
            << "    \"epoch_identity_alignment_valid\": "
            << (imu_report.phase91_raw_drift_d_initializer_alignment_valid
                    ? "true"
                    : "false")
            << ",\n"
            << "    \"main_epoch_count\": "
            << imu_report.phase91_main_epoch_count << ",\n"
            << "    \"gnss_first_epoch_count\": "
            << imu_report.phase91_gnss_first_epoch_count << ",\n"
            << "    \"solution_epoch_count\": "
            << imu_report.phase91_solution_epoch_count << ",\n"
            << "    \"raw_epoch_count\": "
            << imu_report.phase91_raw_epoch_count << ",\n"
            << "    \"raw_utc_key_count\": "
            << imu_report.phase91_raw_utc_key_count << ",\n"
            << "    \"aligned_epoch_count\": "
            << imu_report.phase91_aligned_epoch_count << ",\n"
            << "    \"positions_clocks_copied\": "
            << imu_report.gnss_first_positions_clocks_copied << ",\n"
            << "    \"coordinates_source\": \"in-memory GNSS-first result only\",\n"
            << "    \"forbidden_coordinate_sources\": [\"PDC\", \"direct-WLS\", "
               "\"external\", \"precomputed\"],\n"
            << "    \""
            << (options.native_phase144_telemetry_schema ? "handoff_failure"
                                                           : "failure")
            << "\": ";
        writeJsonString(out, options.native_source_clock_c0d_epoch_vector_parity &&
                                !imu_report.phase101_optimized_c_handoff_failure.empty()
                            ? imu_report.phase101_optimized_c_handoff_failure
                            : imu_report.phase93_optimized_d_handoff_failure);
    } else if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer) {
        out << ",\n"
            << "    \"handoff_mode\": "
               "\"gnss-first-in-memory-position-clock-velocity\",\n"
            << "    \"velocity_handoff_source\": "
               "\"same-run-gnss-first-optimizer-result\",\n"
            << "    \"position_clock_handoff_source\": "
               "\"same-run-gnss-first-optimizer-result\",\n"
            << "    \"raw_drift_d_initializer\": "
               "\"main-graph-only\",\n"
            << "    \"epoch_identity_alignment_valid\": "
            << (imu_report.phase91_raw_drift_d_initializer_alignment_valid
                    ? "true"
                    : "false")
            << ",\n"
            << "    \"main_epoch_count\": "
            << imu_report.phase91_main_epoch_count << ",\n"
            << "    \"gnss_first_epoch_count\": "
            << imu_report.phase91_gnss_first_epoch_count << ",\n"
            << "    \"solution_epoch_count\": "
            << imu_report.phase91_solution_epoch_count << ",\n"
            << "    \"raw_epoch_count\": "
            << imu_report.phase91_raw_epoch_count << ",\n"
            << "    \"raw_utc_key_count\": "
            << imu_report.phase91_raw_utc_key_count << ",\n"
            << "    \"aligned_epoch_count\": "
            << imu_report.phase91_aligned_epoch_count << ",\n"
            << "    \"epoch_count_mismatch_count\": "
            << imu_report.phase91_epoch_count_mismatch_count << ",\n"
            << "    \"nonfinite_time_count\": "
            << imu_report.phase91_nonfinite_time_count << ",\n"
            << "    \"gnss_first_time_mismatch_count\": "
            << imu_report.phase91_gnss_first_time_mismatch_count << ",\n"
            << "    \"raw_time_mismatch_count\": "
            << imu_report.phase91_raw_time_mismatch_count << ",\n"
            << "    \"raw_utc_key_order_mismatch_count\": "
            << imu_report.phase91_raw_utc_key_order_mismatch_count << ",\n"
            << "    \"duplicate_raw_utc_key_count\": "
            << imu_report.phase91_duplicate_raw_utc_key_count << ",\n"
            << "    \"raw_utc_key_mismatch_count\": "
            << imu_report.phase91_raw_utc_key_mismatch_count << ",\n"
            << "    \"raw_drift_count_mismatch_count\": "
            << imu_report.phase91_raw_drift_count_mismatch_count << ",\n"
            << "    \"raw_drift_nonfinite_count\": "
            << imu_report.phase91_raw_drift_nonfinite_count << ",\n"
            << "    \"nonfinite_solution_count\": "
            << imu_report.phase91_nonfinite_solution_count << ",\n"
            << "    \"positions_clocks_copied\": "
            << imu_report.gnss_first_positions_clocks_copied << ",\n"
            << "    \"coordinates_source\": \"in-memory GNSS-first result only\",\n"
            << "    \"forbidden_coordinate_sources\": [\"PDC\", \"direct-WLS\", "
               "\"external\", \"precomputed\"],\n"
            << "    \""
            << (options.native_phase144_telemetry_schema ? "handoff_failure"
                                                           : "failure")
            << "\": ";
        writeJsonString(out, imu_report.phase91_raw_drift_d_initializer_failure);
    } else if (options.native_direct_doppler_wls_handoff) {
        const std::size_t direct_valid =
            imu_report.direct_doppler_wls_direct_valid_count;
        const std::size_t propagated_valid =
            imu_report.direct_doppler_wls_propagated_valid_count;
        const std::size_t valid = direct_valid + propagated_valid;
        out << ",\n"
            << "    \"handoff_mode\": \"direct-doppler-wls\",\n"
            << "    \"velocity_handoff_source\": \"raw-doppler-wls-estimates\",\n"
            << "    \"velocity_initializer\": \"raw-doppler-wls\",\n"
            << "    \"direct_valid_count\": " << direct_valid << ",\n"
            << "    \"propagated_valid_count\": " << propagated_valid << ",\n"
            << "    \"rejected_count\": "
            << imu_report.direct_doppler_wls_rejected_count << ",\n"
            << "    \"valid_count\": " << valid << ",\n"
            << "    \"nonfinite_count\": "
            << imu_report.direct_doppler_wls_nonfinite_count << ",\n"
            << "    \"over_70_mps_count\": "
            << imu_report.direct_doppler_wls_over_bound_count << ",\n"
            << "    \"clock_rate_over_2000_mps_count\": "
            << imu_report.direct_doppler_wls_clock_rate_over_bound_count << ",\n"
            << "    \"max_velocity_norm_mps\": "
            << imu_report.direct_doppler_wls_max_velocity_norm_mps << ",\n"
            << "    \"max_clock_rate_abs_mps\": "
            << imu_report.direct_doppler_wls_max_clock_rate_abs_mps << ",\n"
            << "    \"first_solved_rows\": "
            << imu_report.direct_doppler_wls_first_solved_rows << ",\n"
            << "    \"first_solved_velocity_norm_mps\": "
            << imu_report.direct_doppler_wls_first_solved_velocity_norm_mps
            << ",\n"
            << "    \"first_solved_clock_rate_abs_mps\": "
            << imu_report.direct_doppler_wls_first_solved_clock_rate_abs_mps
            << ",\n"
            << "    \"first_solved_reason\": ";
        writeJsonString(out, imu_report.direct_doppler_wls_first_solved_reason);
        out << ",\n"
            << "    \"edge_hold_count\": "
            << imu_report.direct_doppler_wls_edge_hold_count << ",\n"
            << "    \"edge_hold_max_s\": "
            << imu_report.direct_doppler_wls_edge_hold_max_s << ",\n"
            << "    \"coverage_epochs\": "
            << imu_report.direct_doppler_wls_epochs << ",\n"
            << "    \"coverage_all_epochs\": "
            << ((valid == imu_report.direct_doppler_wls_epochs &&
                 imu_report.direct_doppler_wls_rejected_count == 0U)
                    ? "true"
                    : "false") << ",\n"
            << "    \"original_raw_seed_position_count\": "
            << imu_report.direct_doppler_wls_original_raw_seed_position_count
            << ",\n"
            << "    \"original_raw_seed_position_invalid_count\": "
            << imu_report.direct_doppler_wls_original_raw_seed_position_invalid_count
            << ",\n"
            << "    \"positions_clocks_copied\": 0\n";
    }
    out << "\n  },\n"
        << "  \"imu_initialization\": {\n"
        << "    \"input_format\": ";
    writeJsonString(out, imu_report.android_raw ? "android-device_imu.csv"
                                                : "gpst-metric-imu.csv");
    out << ",\n"
        << "    \"loaded_samples\": " << imu_report.loaded_samples << ",\n"
        << "    \"stationary_samples\": " << imu_report.stationary_samples << ",\n"
        << "    \"stationary_gyro_initializer_requested\": " << (options.native_stationary_gyro_initializer ? "true" : "false") << ",\n"
        << "    \"stationary_gyro_initializer_applied\": " << (imu_report.stationary_gyro_initializer_applied ? "true" : "false") << ",\n"
        << "    \"stationary_gyro_blocks\": " << imu_report.stationary_gyro_blocks << ",\n"
        << "    \"stationary_gyro_scatter_radps\": " << imu_report.stationary_gyro_scatter_radps << ",\n"
        << "    \"gravity_mean_norm_mps2\": " << imu_report.gravity_mean_norm << ",\n"
        << "    \"gravity_norm_std_mps2\": " << imu_report.gravity_norm_std << ",\n"
        << "    \"heading_windows\": " << imu_report.heading_windows << ",\n"
        << "    \"heading_latched\": " << (imu_report.heading_latched ? "true" : "false") << ",\n"
        << "    \"heading_initialization_mode\": ";
    writeJsonString(out, imu_report.heading_initialization_mode);
    out << ",\n"
        << "    \"velocity_heading_low_speed_count\": "
        << imu_report.velocity_heading_low_speed_count << ",\n"
        << "    \"velocity_heading_linear_fill_count\": "
        << imu_report.velocity_heading_linear_fill_count << ",\n"
        << "    \"velocity_heading_nearest_fill_count\": "
        << imu_report.velocity_heading_nearest_fill_count << ",\n"
        << "    \"axis_contract\": \"identity raw-axis selection + explicit RzRyRx mounting\",\n"
        << "    \"mounting_rpy_deg_xyz\": [-85.0, 178.0, -94.0],\n"
        << "    \"timestamp_contract\": ";
    writeJsonString(out, imu_report.android_raw
                           ? (imu_report.android_load.utc_wall_clock_fallback_applied
                                  ? "Raw GNSS TimeNanos-FullBiasNanos-BiasNanos -> UTC milliseconds affine map; UTC wall-clock pairing; raw UTC -> GPST; elapsedRealtimeNanos absent and not fabricated"
                                  : "GNSS ChipsetElapsedRealtimeNanos -> UTC milliseconds by linear interpolation/extrapolation; gyro anchors; sync coefficient 0.5; UTC + 18 s -> GPST")
                           : "GPST week/TOW CSV; no runtime offset");
    out << ",\n"
        << "    \"android_alignment\": ";
    if (!imu_report.android_raw) {
        out << "null,\n";
    } else {
        const auto& alignment = imu_report.android_load;
        out << "{\n"
            << "      \"total_rows\": " << alignment.total_rows << ",\n"
            << "      \"accel_rows\": " << alignment.accel_rows << ",\n"
            << "      \"gyro_rows\": " << alignment.gyro_rows << ",\n"
            << "      \"unsupported_rows\": " << alignment.unsupported_rows << ",\n"
            << "      \"duplicate_accel_timestamps\": "
            << alignment.duplicate_accel_timestamps << ",\n"
            << "      \"duplicate_gyro_timestamps\": "
            << alignment.duplicate_gyro_timestamps << ",\n"
            << "      \"paired_rows\": " << alignment.paired_rows << ",\n"
            << "      \"exact_elapsed_matches\": "
            << alignment.exact_elapsed_matches << ",\n"
            << "      \"interpolated_rows\": " << alignment.interpolated_rows << ",\n"
            << "      \"endpoint_nearest_rows\": "
            << alignment.endpoint_nearest_rows << ",\n"
            << "      \"omitted_rows\": " << alignment.omitted_rows << ",\n"
            << "      \"median_abs_pair_offset_ms\": "
            << alignment.median_abs_pair_offset_ms << ",\n"
            << "      \"maximum_abs_pair_offset_ms\": "
            << alignment.maximum_abs_pair_offset_ms << ",\n"
            << "      \"gnss_anchor_points\": "
            << alignment.gnss_anchor_points << ",\n"
            << "      \"gnss_anchor_exact_rows\": "
            << alignment.gnss_anchor_exact_rows << ",\n"
            << "      \"gnss_anchor_interpolated_rows\": "
            << alignment.gnss_anchor_interpolated_rows << ",\n"
            << "      \"gnss_anchor_extrapolated_rows\": "
            << alignment.gnss_anchor_extrapolated_rows << ",\n"
            << "      \"first_mapped_utc_time_ms\": "
            << alignment.first_mapped_utc_time_ms << ",\n"
            << "      \"last_mapped_utc_time_ms\": "
            << alignment.last_mapped_utc_time_ms << ",\n"
            << "      \"imu_sync_coefficient\": "
            << alignment.imu_sync_coefficient << ",\n"
            << "      \"first_dt_s\": " << alignment.first_dt_s << ",\n"
            << "      \"last_dt_s\": " << alignment.last_dt_s << ",\n"
            << "      \"dt_tail_repeated\": "
            << (alignment.dt_tail_repeated ? "true" : "false") << ",\n"
            << "      \"anchor_input_rows\": "
            << imu_report.android_gnss_anchor_load.input_rows << ",\n"
            << "      \"anchor_raw_rows\": "
            << imu_report.android_gnss_anchor_load.raw_rows << ",\n"
            << "      \"anchor_duplicate_utc_timestamps\": "
            << imu_report.android_gnss_anchor_load.duplicate_utc_timestamps << ",\n"
            << "      \"utc_mapping_input_rows\": "
            << imu_report.android_gnss_utc_mapping_load.input_rows << ",\n"
            << "      \"utc_mapping_raw_rows\": "
            << imu_report.android_gnss_utc_mapping_load.raw_rows << ",\n"
            << "      \"utc_mapping_unsupported_rows\": "
            << imu_report.android_gnss_utc_mapping_load.unsupported_rows << ",\n"
            << "      \"utc_mapping_duplicate_utc_timestamps\": "
            << imu_report.android_gnss_utc_mapping_load.duplicate_utc_timestamps << ",\n"
            << "      \"utc_mapping_hardware_clock_count_field_present\": "
            << (imu_report.android_gnss_utc_mapping_load.mapping
                        .hardware_clock_count_field_present
                    ? "true"
                    : "false") << ",\n"
            << "      \"utc_mapping_hardware_clock_count_constant\": "
            << (imu_report.android_gnss_utc_mapping_load.mapping
                        .hardware_clock_count_constant
                    ? "true"
                    : "false") << ",\n"
            << "      \"first_gyro_elapsed_ns\": "
            << alignment.first_gyro_elapsed_ns << ",\n"
            << "      \"last_gyro_elapsed_ns\": "
            << alignment.last_gyro_elapsed_ns << ",\n"
            << "      \"elapsed_clock_preserved\": "
            << (alignment.elapsed_clock_preserved ? "true" : "false") << ",\n"
            << "      \"gnss_elapsed_anchor_applied\": "
            << (alignment.gnss_elapsed_anchor_applied ? "true" : "false") << ",\n"
            << "      \"utc_wall_clock_fallback_applied\": "
            << (alignment.utc_wall_clock_fallback_applied ? "true" : "false") << ",\n"
            << "      \"utc_wall_clock_fallback_offset_requested\": "
            << (alignment.utc_wall_clock_fallback_offset_requested
                    ? "true"
                    : "false") << ",\n"
            << "      \"utc_wall_clock_fallback_offset_applied\": "
            << (alignment.utc_wall_clock_fallback_offset_applied
                    ? "true"
                    : "false") << ",\n"
            << "      \"utc_wall_clock_fallback_offset_ms\": "
            << alignment.utc_wall_clock_fallback_offset_ms << ",\n"
            << "      \"utc_wall_clock_fallback_effective_offset_ms\": "
            << alignment.utc_wall_clock_fallback_effective_offset_ms << ",\n"
            << "      \"utc_mapping_anchors\": "
            << alignment.utc_mapping_anchors << ",\n"
            << "      \"utc_mapping_slope_ns_per_ms\": "
            << alignment.utc_mapping_slope_ns_per_ms << ",\n"
            << "      \"utc_mapping_drift_ppm\": "
            << alignment.utc_mapping_drift_ppm << ",\n"
            << "      \"utc_mapping_max_fit_residual_ms\": "
            << alignment.utc_mapping_max_fit_residual_ms << ",\n"
            << "      \"utc_mapping_max_anchor_gap_ms\": "
            << alignment.utc_mapping_max_anchor_gap_ms << "\n"
            << "    },\n";
    }
    out << "    \"failure\": ";
    writeJsonString(out, imu_report.failure);
    out << "\n  },\n";
    if (raw_utc_report.enabled) {
        out << "  \"raw_utc_key_contract\": {\n"
            << "    \"warmup_epoch_excluded\": "
            << (raw_utc_report.warmup_epoch_excluded ? "true" : "false") << ",\n"
            << "    \"raw_epoch_keys\": " << raw_utc_report.raw_epoch_keys << ",\n"
            << "    \"target_epochs\": " << raw_utc_report.target_epochs << ",\n"
            << "    \"exact_solution_epochs\": "
            << raw_utc_report.exact_solution_epochs << ",\n"
            << "    \"interpolated_epochs\": "
            << raw_utc_report.interpolated_epochs << ",\n"
            << "    \"edge_hold_epochs\": " << raw_utc_report.edge_hold_epochs << ",\n"
            << "    \"unresolved_epochs\": " << raw_utc_report.unresolved_epochs << ",\n"
            << "    \"solution_time_tolerance_ms\": "
            << raw_utc_report.solution_time_tolerance_ms << ",\n"
            << "    \"max_interpolation_gap_ms\": "
            << raw_utc_report.max_interpolation_gap_ms << ",\n"
            << "    \"max_edge_hold_gap_ms\": "
            << raw_utc_report.max_edge_hold_gap_ms << ",\n"
            << "    \"coordinate_interpolation\": \"ECEF linear then geodetic\",\n"
            << "    \"device_wls_coordinates_used\": false\n"
            << "  },\n";
    }
    out << "  \"output_contract\": {\n"
        << "    \"header\": \"phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\",\n"
        << "    \"finite_coordinates\": true,\n"
        << "    \"unix_time_leap_seconds\": 18,\n"
        << "    \"atomic_publish\": true\n"
        << "  }";
    if (phase104_stage_report.enabled) {
        out << ",\n  \"phase104_stage_export\": ";
        writePhase104StageReport(out, phase104_stage_report);
        out << ",\n  \"phase104_main_displacement\": ";
        writePhase104DisplacementReport(out, phase104_displacement_report);
    }
    if (phase116_tdcp_report.enabled) {
        out << ",\n  \"phase116_carrier_tdcp_incidence\": ";
        writePhase116CarrierTdcpReport(out, phase116_tdcp_report);
    }
    if (options.native_phase143_official_main_lm_termination_budget ||
        options.native_phase171_raw_p_no_doppler_imu_main) {
        out << ",\n  \"phase143_termination\": {\n"
            << "    \"schema_version\": \"smartphone-r5-native-fgo-phase143-termination.v1\",\n"
            << "    \"authority\": \"FGOResult.diagnostics.native_phase143_termination\",\n"
            << "    \"main\": ";
        writePhase143TerminationDiagnostics(
            out, result.diagnostics.native_phase143_termination);
        out << ",\n    \"gnss_first\": ";
        writePhase143TerminationDiagnostics(
            out, imu_report.gnss_first_phase143_termination);
        out << ",\n    \"no_solution_or_accuracy_fields\": true\n  }";
    }
    if (options.native_phase141_telemetry_schema ||
        options.native_phase144_telemetry_schema) {
        out << ",\n  \""
            << (options.native_phase144_telemetry_schema
                    ? "phase144_telemetry"
                    : "phase141_telemetry")
            << "\": ";
        writePhase141Telemetry(out, options, problem, result, imu_report, fallback,
                               raw_utc_report, position_offset_report,
                               base_report);
    }
    out << "\n}\n";
    return out.str();
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArguments(argc, argv, options)) return 2;
    if (options.native_phase180_android_clock_preflight) {
        return runPhase180AndroidClockPreflight(options);
    }

    Phase94StageDiagnostics phase94_diagnostics;
    phase94_diagnostics.enabled =
        options.native_source_clock_c0d_phase94_stage_diagnostics ||
        options.native_source_clock_c0d_phase96_main_diagnostics ||
        options.native_source_clock_c0d_phase97_singular_system_diagnostics ||
        options.native_source_clock_c0d_phase98_solver_rank_diagnostic;
    phase94_diagnostics.phase96_enabled =
        options.native_source_clock_c0d_phase96_main_diagnostics;
    phase94_diagnostics.phase97_enabled =
        options.native_source_clock_c0d_phase97_singular_system_diagnostics;
    phase94_diagnostics.phase98_enabled =
        options.native_source_clock_c0d_phase98_solver_rank_diagnostic;
    phase94_diagnostics.phase96_main.enabled =
        options.native_source_clock_c0d_phase96_main_diagnostics;
    phase94_diagnostics.phase97_main.enabled =
        options.native_source_clock_c0d_phase97_singular_system_diagnostics;
    phase94_diagnostics.phase98_solver.enabled =
        options.native_source_clock_c0d_phase98_solver_rank_diagnostic;
    phase94_diagnostics.dataset_id = options.dataset_id;
    if (phase94_diagnostics.enabled) {
        phase94_diagnostics.status = "running";
    }
    Phase104StageExportReport phase104_stage_report;
    Phase104MainDisplacementReport phase104_displacement_report;
    Phase116CarrierTdcpReport phase116_tdcp_report;

    const bool android_raw =
        !options.android_imu_path.empty() ||
        ((options.native_phase149_raw_p_seed_stage ||
          options.native_phase163_raw_p_no_doppler_seed_stage ||
          options.native_phase165_raw_p_no_doppler_graph) &&
         !options.android_gnss_path.empty());
    libgnss::io::RINEXReader obs_reader;
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    libgnss::io::AndroidRawGnssResult android_gnss;
    std::vector<libgnss::GNSSTime> android_raw_epoch_times;
    if (android_raw) {
        libgnss::io::AndroidRawGnssConfig android_gnss_config;
        android_gnss_config.require_frequency_pair_timing =
            options.native_tdcp_frequency_residual_states;
        android_gnss_config.verify_enriched_pseudorange =
            !options.android_raw_clock_only;
        std::string conversion_error;
        if (!libgnss::io::loadAndroidRawGnssCsv(
                options.android_gnss_path, android_gnss_config, android_gnss,
                conversion_error)) {
            std::cerr << "failed to convert raw Android GNSS: "
                      << conversion_error << "\n";
            return 1;
        }
        if (android_gnss.observations.isEmpty()) {
            std::cerr << "raw Android GNSS has no usable observation epochs\n";
            return 1;
        }
        android_raw_epoch_times.reserve(android_gnss.observations.epochs.size());
        for (const auto& raw_epoch : android_gnss.observations.epochs) {
            android_raw_epoch_times.push_back(raw_epoch.time);
        }
        // Raw mode consumes the Android observation series directly.  It does
        // not synthesize a RINEX intermediate and does not use the optional
        // device-provided WLS position retained by the adapter.
        obs_header.interval = 1.0;
    } else {
        if (!obs_reader.open(options.obs_path)) {
            std::cerr << "failed to open observation file\n";
            return 1;
        }
        if (!obs_reader.readHeader(obs_header)) {
            std::cerr << "failed to read observation header\n";
            return 1;
        }
    }
    libgnss::io::RINEXReader nav_reader;
    if (!nav_reader.open(options.nav_path)) {
        std::cerr << "failed to open navigation file\n";
        return 1;
    }
    libgnss::NavigationData nav;
    if (!nav_reader.readNavigationData(nav)) {
        std::cerr << "failed to read navigation file\n";
        return 1;
    }

    libgnss::base_pseudorange_compensation::Model base_pseudorange_model;
    BasePseudorangeCompensationReport base_pseudorange_report;
    base_pseudorange_report.enabled =
        options.native_base_pseudorange_compensation;
    base_pseudorange_report.source_complete =
        options.native_phase126_raw_base_source_complete || options.native_paired_epoch_states;
    base_pseudorange_report.phase127_enabled =
        options.native_phase127_glonass_channel_provenance;
    base_pseudorange_report.phase128_enabled =
        options.native_phase128_glonass_provenance_parser_admission;
    base_pseudorange_report.phase129_enabled =
        options.native_phase129_glonass_local_miss_mask;
    base_pseudorange_report.phase131_enabled =
        options.native_phase131_canonical_correction_band_key;
    base_pseudorange_report.official_no_explicit_tgd_bgd =
        options.native_phase126_raw_base_source_complete || options.native_paired_epoch_states;
    base_pseudorange_report.base_rinex_path = options.native_base_rinex_path;
    base_pseudorange_report.base_rinex_sha256 =
        options.native_base_rinex_sha256;
    base_pseudorange_report.preserve_additional_frequency_bands =
        options.native_base_pseudorange_preserve_additional_frequency_bands ||
        options.native_paired_epoch_states;
    if (options.native_base_pseudorange_compensation) {
        libgnss::io::RINEXReader base_reader;
        libgnss::io::RINEXReader::RINEXHeader base_header;
        base_reader.setPreserveAdditionalFrequencyBands(
            options.native_base_pseudorange_preserve_additional_frequency_bands ||
            options.native_paired_epoch_states);
        base_reader.setSourceHeaderTrackingFilter(options.native_paired_epoch_states);
        if (!base_reader.open(options.native_base_rinex_path) ||
            !base_reader.readHeader(base_header)) {
            std::cerr << "failed to read frozen base RINEX header\n";
            return 1;
        }
        if (options.native_paired_epoch_states &&
            (base_header.version < 3 || base_header.version >= 4 ||
             !base_header.has_antenna_delta || !base_header.antenna_delta.isZero())) {
            std::cerr << "paired epoch states require RINEX3 with explicit zero antenna delta\n";
            return 1;
        }
        if (options.native_phase126_raw_base_source_complete || options.native_paired_epoch_states) {
            // RINEX APPROX POSITION XYZ is admitted as the antenna reference
            // for this frozen source contract.  The optional H/E/N delta is
            // retained as provenance and is not added a second time; an
            // unproven marker-to-antenna interpretation fails closed in the
            // library rather than silently changing the station coordinate.
            base_pseudorange_report.station_reference_verified =
                base_header.has_approximate_position;
            base_pseudorange_report.antenna_reference_is_approx_position = true;
            base_pseudorange_report.antenna_delta_present =
                base_header.has_antenna_delta;
            base_pseudorange_report.antenna_delta_applied = false;
            if (!base_header.has_approximate_position ||
                !base_header.approximate_position.allFinite()) {
                std::cerr << "Phase126 requires finite RINEX APPROX POSITION XYZ\n";
                return 1;
            }
            if (base_header.has_antenna_delta &&
                !base_header.antenna_delta.allFinite()) {
                std::cerr << "Phase126 requires finite RINEX antenna delta\n";
                return 1;
            }
        }
        libgnss::ObservationSeries base_series;
        if (!base_reader.readAllObservations(base_series)) {
            std::cerr << "failed to read frozen base RINEX observations\n";
            return 1;
        }
        std::set<std::string> selected_band_stream_keys;
        for (const auto& epoch : base_series.epochs) {
            for (const auto& observation : epoch.observations) {
                ++base_pseudorange_report.selected_band_observation_rows;
                const std::string signal_name =
                    baseTelemetrySignalName(observation.signal);
                ++base_pseudorange_report
                          .selected_band_observation_rows_by_signal[signal_name];
                const std::string stream_key =
                    std::to_string(static_cast<int>(observation.satellite.system)) +
                    ":" + std::to_string(static_cast<int>(observation.satellite.prn)) +
                    ":" + signal_name;
                if (selected_band_stream_keys.insert(stream_key).second) {
                    ++base_pseudorange_report.selected_band_streams;
                    ++base_pseudorange_report
                              .selected_band_streams_by_signal[signal_name];
                }
            }
        }
        base_reader.close();
        base_pseudorange_report.header_version = base_header.version;
        base_pseudorange_report.header_interval_s = base_header.interval;
        base_pseudorange_report.base_position_ecef =
            base_header.approximate_position;
        std::error_code file_size_error;
        base_pseudorange_report.base_rinex_bytes =
            std::filesystem::file_size(options.native_base_rinex_path,
                                        file_size_error);
        if (file_size_error) {
            std::cerr << "failed to stat frozen base RINEX\n";
            return 1;
        }
        std::size_t moving_mean_samples = 0U;
        std::string sampling_error;
        if (!selectBaseSampling(base_series,
                                base_pseudorange_report.expected_interval_s,
                                moving_mean_samples, sampling_error)) {
            base_pseudorange_report.failure = sampling_error;
            std::cerr << "base RINEX sampling contract failed closed: "
                      << sampling_error << "\n";
            return 1;
        }
        if (options.native_phase126_raw_base_source_complete) {
            // Atomic step A ends only after the header, complete raw stream,
            // and frozen interval/window contract have all been admitted.
            base_pseudorange_report
                .phase126_atomic_step_a_raw_ingress_verified = true;
        }
        base_pseudorange_report.moving_mean_samples = moving_mean_samples;
        libgnss::base_pseudorange_compensation::Config base_config;
        base_config.base_position_ecef = base_header.approximate_position;
        base_config.source_complete =
            options.native_phase126_raw_base_source_complete || options.native_paired_epoch_states;
        base_config.use_source_epoch_states = options.native_paired_epoch_states;
        base_config.use_source_fgo_frequency_slots = options.native_paired_epoch_states;
        base_config.use_dense_epoch_smoothing = options.native_dense_base_smoothing;
        base_config.approximate_position_present =
            base_header.has_approximate_position;
        base_config.antenna_delta_present = base_header.has_antenna_delta;
        base_config.station_reference_verified =
            base_pseudorange_report.station_reference_verified;
        base_config.antenna_reference_is_approx_position =
            base_pseudorange_report.antenna_reference_is_approx_position;
        base_config.antenna_delta_enu = base_header.has_antenna_delta
                                            ? base_header.antenna_delta
                                            : libgnss::Vector3d::Zero();
        base_config.expected_interval_s =
            base_pseudorange_report.expected_interval_s;
        base_config.moving_mean_samples = moving_mean_samples;
        // The Phase43/native recipe leaves the ordinary broadcast atmosphere
        // model enabled.  Keep these explicit in the base model rather than
        // inheriting mutable global/default state.
        base_config.use_ionosphere_model = true;
        base_config.use_troposphere_model = true;
        base_config.use_signal_specific_galileo_group_delay =
            options.native_signal_specific_galileo_tgd;
        base_config.use_phase127_glonass_channel_provenance =
            options.native_phase127_glonass_channel_provenance;
        base_config.use_phase128_glonass_provenance_parser_admission =
            options.native_phase128_glonass_provenance_parser_admission;
        base_config.use_phase129_glonass_local_miss_mask =
            options.native_phase129_glonass_local_miss_mask;
        base_config.use_phase131_canonical_correction_band_key =
            options.native_phase131_canonical_correction_band_key;
        base_config.glonass_frequency_channel_header_status =
            base_header.glonass_frequency_channel_header_status;
        base_config.glonass_frequency_channel_entries =
            base_header.glonass_frequency_channel_entries;
        base_config.glonass_frequency_channel_malformed_entries =
            base_header.glonass_frequency_channel_malformed_entries;
        ++base_pseudorange_report.source_model_build_count;
        if (!base_pseudorange_model.build(base_series, nav, base_config)) {
            base_pseudorange_report.failure =
                base_pseudorange_model.diagnostics().failure;
            std::cerr << "base pseudorange model failed closed: "
                      << base_pseudorange_report.failure << "\n";
            return 1;
        }
        if (options.native_phase126_raw_base_source_complete) {
            base_pseudorange_report
                .phase126_atomic_step_b_source_stream_verified = true;
        }
        const auto& diagnostics = base_pseudorange_model.diagnostics();
        base_pseudorange_report.built = diagnostics.built;
        base_pseudorange_report.base_interval_s = diagnostics.base_interval_s;
        base_pseudorange_report.base_epochs = diagnostics.base_epochs;
        base_pseudorange_report.base_observation_rows =
            diagnostics.base_observation_rows;
        base_pseudorange_report.matching_streams = diagnostics.matching_streams;
        base_pseudorange_report.matched_base_rows = diagnostics.matched_base_rows;
        base_pseudorange_report.finite_base_residual_rows =
            diagnostics.finite_base_residual_rows;
        base_pseudorange_report.smoothed_rows = diagnostics.smoothed_rows;
        base_pseudorange_report.correction_abs_p50_m =
            diagnostics.correction_abs_p50_m;
        base_pseudorange_report.correction_abs_p95_m =
            diagnostics.correction_abs_p95_m;
        base_pseudorange_report.correction_abs_max_m =
            diagnostics.correction_abs_max_m;
        base_pseudorange_report.sagnac_evaluations =
            diagnostics.sagnac_evaluations;
        base_pseudorange_report.source_complete_signal_rows =
            diagnostics.source_complete_signal_rows;
        base_pseudorange_report.phase127_glonass_rows =
            diagnostics.phase127_glonass_rows;
        base_pseudorange_report.phase127_accepted_rows =
            diagnostics.phase127_accepted_rows;
        base_pseudorange_report.phase127_header_primary_rows =
            diagnostics.phase127_header_primary_rows;
        base_pseudorange_report.phase127_ephemeris_fallback_rows =
            diagnostics.phase127_ephemeris_fallback_rows;
        base_pseudorange_report.phase127_header_entries_seen =
            diagnostics.phase127_header_entries_seen;
        base_pseudorange_report.phase127_header_duplicate_entries =
            diagnostics.phase127_header_duplicate_entries;
        base_pseudorange_report.phase127_header_conflict_entries =
            diagnostics.phase127_header_conflict_entries;
        base_pseudorange_report.phase127_header_malformed_entries =
            diagnostics.phase127_header_malformed_entries;
        base_pseudorange_report.phase127_ephemeris_candidates =
            diagnostics.phase127_ephemeris_candidates;
        base_pseudorange_report.phase127_ephemeris_ties =
            diagnostics.phase127_ephemeris_ties;
        base_pseudorange_report.phase127_ephemeris_duplicate_entries =
            diagnostics.phase127_ephemeris_duplicate_entries;
        base_pseudorange_report.phase127_ephemeris_conflict_entries =
            diagnostics.phase127_ephemeris_conflict_entries;
        base_pseudorange_report.phase127_query_time_coverage_gaps =
            diagnostics.phase127_query_time_coverage_gaps;
        base_pseudorange_report.phase127_invalid_channels =
            diagnostics.phase127_invalid_channels;
        base_pseudorange_report.phase127_failure_counts =
            diagnostics.phase127_failure_counts;
        base_pseudorange_report.phase128_enabled = diagnostics.phase128_enabled;
        base_pseudorange_report.phase128_header_status =
            diagnostics.phase128_header_status;
        base_pseudorange_report.phase128_canonical_records =
            diagnostics.phase128_canonical_records;
        base_pseudorange_report.phase128_canonical_rejected_records =
            diagnostics.phase128_canonical_rejected_records;
        base_pseudorange_report.phase129_enabled = diagnostics.phase129_enabled;
        base_pseudorange_report.phase129_configuration_valid =
            diagnostics.phase129_configuration_valid;
        base_pseudorange_report.phase129_configuration_failure =
            diagnostics.phase129_configuration_failure;
        base_pseudorange_report.phase129_glonass_local_miss_rows =
            diagnostics.phase129_glonass_local_miss_rows;
        base_pseudorange_report.phase129_glonass_local_miss_counts =
            diagnostics.phase129_glonass_local_miss_counts;
        base_pseudorange_report.phase129_glonass_local_miss_streams =
            diagnostics.phase129_glonass_local_miss_streams;
        base_pseudorange_report.phase129_glonass_row_count_consistent =
            diagnostics.phase129_glonass_row_count_consistent;
        base_pseudorange_report.phase131_enabled = diagnostics.phase131_enabled;
        base_pseudorange_report.phase131_configuration_valid =
            diagnostics.phase131_configuration_valid;
        base_pseudorange_report.phase131_configuration_failure =
            diagnostics.phase131_configuration_failure;
        base_pseudorange_report.phase131_canonical_rows =
            diagnostics.phase131_canonical_rows;
        base_pseudorange_report.phase131_canonical_rejected_rows =
            diagnostics.phase131_canonical_rejected_rows;
        base_pseudorange_report.phase131_unknown_band_rows =
            diagnostics.phase131_unknown_band_rows;
        base_pseudorange_report.phase131_canonical_key_conflicts =
            diagnostics.phase131_canonical_key_conflicts;
        base_pseudorange_report.phase131_canonical_duplicate_rows =
            diagnostics.phase131_canonical_duplicate_rows;
        base_pseudorange_report.phase131_canonical_streams =
            diagnostics.phase131_canonical_streams;
        base_pseudorange_report.phase131_canonical_selected_streams =
            diagnostics.phase131_canonical_selected_streams;
        base_pseudorange_report.phase131_canonical_merged_streams =
            diagnostics.phase131_canonical_merged_streams;
        base_pseudorange_report.phase131_failure_counts =
            diagnostics.phase131_failure_counts;
    }

    if (android_raw && options.native_phase171_raw_p_no_doppler_imu_main &&
        options.all_epochs && options.skip_epochs == 0) {
        // Compute before moving/filtering epochs. Identity is exact within
        // this raw vector; carry only a diagnostic label on the same row.
        const auto candidates = libgnss::codeEdgeCandidates(
            android_gnss.observations.epochs,
            android_gnss.epoch_hardware_clock_discontinuity_count);
        for (const auto& [index, satellite, signal] : candidates) {
            for (auto& row : android_gnss.observations.epochs.at(index).observations) {
                if (row.satellite == satellite && row.signal == signal)
                    row.native_code_edge_diagnostic_candidate = true;
            }
        }
        std::cerr << "[native-code-edge-candidates] raw_candidates="
                  << candidates.size() << " factors_added=0\n";
    }
    std::vector<libgnss::ObservationData> epochs;
    libgnss::ObservationData epoch;
    int observation_index = 0;
    if (android_raw) {
        for (auto& raw_epoch : android_gnss.observations.epochs) {
            if (observation_index++ < options.skip_epochs) continue;
            if (!options.all_epochs &&
                epochs.size() >= static_cast<std::size_t>(options.max_epochs)) {
                break;
            }
            // SPPProcessor uses the epoch position only as a numerical initial
            // frame when no receiver seed is supplied.  This fixed Earth
            // surface point is deliberately route/device independent; it is
            // not emitted and cannot carry a device-WLS/result coordinate.
            raw_epoch.receiver_position = libgnss::Vector3d(6378137.0, 0.0, 0.0);
            epochs.push_back(std::move(raw_epoch));
        }
    } else {
        while ((options.all_epochs ||
                epochs.size() < static_cast<std::size_t>(options.max_epochs)) &&
               obs_reader.readObservationEpoch(epoch)) {
            if (observation_index++ < options.skip_epochs) continue;
            if (obs_header.approximate_position.norm() > 1.0e6) {
                epoch.receiver_position = obs_header.approximate_position;
            }
            epochs.push_back(epoch);
        }
    }
    if (epochs.size() < 2) {
        std::cerr << "fewer than two observation epochs\n";
        return 1;
    }

    libgnss::raw_p_seed::RawPNoDopplerSeedAdapterResult phase165_adapter;
    libgnss::raw_p_seed::Result phase165_raw_result;
    bool phase165_raw_result_available = false;
    if (options.native_phase165_raw_p_no_doppler_graph) {
        // The graph selector owns one same-invocation raw-P solve.  Its typed
        // positions/clocks are copied into the in-memory builder input before
        // any corrected factor geometry is selected; no serialized seed or
        // previous result can enter this lane.
        libgnss::raw_p_seed::Config seed_config;
        seed_config.processor_config.elevation_mask = 0.0;
        seed_config.processor_config.snr_mask = 0.0;
        seed_config.bootstrap_position_before_elevation =
            options.native_phase157_raw_p_bootstrap;
        phase165_raw_result =
            libgnss::raw_p_seed::solve(epochs, nav, seed_config);
        phase165_raw_result_available = true;
        phase165_adapter =
            libgnss::raw_p_seed::adaptSameRunNoDopplerSeeds(epochs,
                                                            phase165_raw_result);
        if (!phase165_adapter.ok || !phase165_adapter.graph_compatible ||
            phase165_adapter.seeds.size() != epochs.size()) {
            const std::string failure = phase165_adapter.failure_reason.empty()
                                             ? phase165_adapter.graph_disabled_reason
                                             : phase165_adapter.failure_reason;
            if (!atomicWrite(
                    options.summary_path,
                    makePhase165RawPNoDopplerGraphJson(
                        options.dataset_id, phase165_adapter,
                        phase165_raw_result_available ? &phase165_raw_result : nullptr,
                        nullptr, nullptr,
                        failure.empty() ? "raw-P adapter rejected graph handoff"
                                        : failure,
                        false, false,
                        options.native_phase167_raw_p_no_doppler_lm_termination_budget))) {
                std::cerr << "failed to write Phase165 raw-P graph summary\n";
            }
            std::cerr << "Phase165 raw-P graph handoff failed closed: "
                      << (failure.empty() ? "adapter rejected" : failure) << "\n";
            return 1;
        }
        for (std::size_t i = 0; i < epochs.size(); ++i) {
            const auto& seed = phase165_adapter.seeds[i];
            if (seed.epoch_index != i || !seed.has_position ||
                !seed.position_ecef.allFinite() || !seed.has_clock ||
                !std::isfinite(seed.clock_bias_m)) {
                const std::string failure = "raw-P seed identity/value handoff failed";
                atomicWrite(options.summary_path,
                            makePhase165RawPNoDopplerGraphJson(
                                options.dataset_id, phase165_adapter,
                                phase165_raw_result_available ? &phase165_raw_result
                                                              : nullptr,
                                nullptr, nullptr, failure, false, false,
                                options.native_phase167_raw_p_no_doppler_lm_termination_budget));
                std::cerr << failure << "\n";
                return 1;
            }
            epochs[i].receiver_position = seed.position_ecef;
            epochs[i].receiver_clock_bias =
                seed.clock_bias_m / libgnss::constants::SPEED_OF_LIGHT;
        }
        // Existing FGO admission is the authority for retained rows.  Remove
        // only raw measurement rows with no source C7 slot before factor
        // construction, while retaining every supported primary/secondary
        // P, TDCP, and other observation with its original epoch identity.
        for (auto& epoch : epochs) {
            epoch.observations.erase(
                std::remove_if(
                    epoch.observations.begin(), epoch.observations.end(),
                    [](const libgnss::Observation& observation) {
                        const bool measured =
                            observation.has_pseudorange ||
                            observation.has_carrier_phase ||
                            observation.has_doppler;
                        return measured &&
                               observation.valid &&
                               libgnss::raw_p_seed::c7ClockComponentFor(
                                   observation.satellite.system,
                                   observation.signal) < 0;
                    }),
                epoch.observations.end());
        }
    }

    if (options.native_phase149_raw_p_seed_stage) {
        // Phase149 is deliberately a preparatory lane: it runs the existing
        // native SPP position/clock solve on a private Doppler-cleared copy,
        // derives only same-run position-gradient velocities, emits structural
        // metadata, and exits before any FGO/IMU path is entered.
        libgnss::raw_p_seed::Config seed_config;
        seed_config.processor_config.elevation_mask = 0.0;
        seed_config.processor_config.snr_mask = 0.0;
        seed_config.bootstrap_position_before_elevation =
            options.native_phase157_raw_p_bootstrap;
        seed_config.collect_all_epochs_for_diagnostics =
            options.native_phase159_collect_all_epochs;
        const auto seed_result =
            libgnss::raw_p_seed::solve(epochs, nav, seed_config);
        if (!atomicWrite(options.summary_path,
                         makePhase149RawPSeedJson(options.dataset_id,
                                                  seed_result))) {
            std::cerr << "failed to write Phase149 raw-P seed summary\n";
            return 1;
        }
        if (!seed_result.ok) {
            std::cerr << "Phase149 raw-P seed stage failed closed: "
                      << seed_result.failure_reason << "\n";
            return 1;
        }
        return 0;
    }

    if (options.native_phase163_raw_p_no_doppler_seed_stage) {
        // Phase163 is a same-invocation handoff adapter only.  raw_p_seed
        // clears Doppler on its private copy, while this original `epochs`
        // vector remains the sole source of an exact raw receiver clock rate.
        // No serialized seed, coordinate, or graph path is consulted here.
        libgnss::raw_p_seed::Config seed_config;
        seed_config.processor_config.elevation_mask = 0.0;
        seed_config.processor_config.snr_mask = 0.0;
        seed_config.bootstrap_position_before_elevation =
            options.native_phase157_raw_p_bootstrap;
        seed_config.collect_all_epochs_for_diagnostics =
            options.native_phase159_collect_all_epochs;
        const auto raw_result =
            libgnss::raw_p_seed::solve(epochs, nav, seed_config);
        const auto adapter =
            libgnss::raw_p_seed::adaptSameRunNoDopplerSeeds(epochs, raw_result);
        if (!atomicWrite(options.summary_path,
                         makePhase163RawPNoDopplerSeedJson(
                             options.dataset_id, raw_result, adapter))) {
            std::cerr << "failed to write Phase163 raw-P seed adapter summary\n";
            return 1;
        }
        if (!adapter.ok) {
            std::cerr << "Phase163 raw-P seed adapter failed closed: "
                      << adapter.failure_reason << "\n";
            return 1;
        }
        // A valid adapter is still prep-only.  Graph admission is reported in
        // the summary and remains disabled when native C7 mapping is not
        // source-proven; no FGO invocation is allowed from this branch.
        return 0;
    }

    libgnss::carrier_code_leveling::Diagnostics carrier_code_leveling_report;
    if (options.native_carrier_code_leveling) {
        // Keep the leveling transform in the same process and before the FGO
        // problem builder.  Only raw P is changed for the explicitly selected
        // primary signals; carrier, Doppler, timestamps, nav, and every other
        // signal remain untouched.
        libgnss::ObservationSeries raw_series;
        raw_series.epochs = epochs;
        libgnss::carrier_code_leveling::Config leveling_config;
        leveling_config.enable_innovation_reset =
            options.native_carrier_code_innovation_reset;
        if (options.native_carrier_code_primary_l1_e1) {
            leveling_config.signals = {libgnss::SignalType::GPS_L1CA,
                                       libgnss::SignalType::GAL_E1};
        }
        leveling_config.include_gal_e5a = options.native_carrier_code_gal_e1_e5a;
        const auto leveling = libgnss::carrier_code_leveling::apply(
            raw_series, android_gnss.epoch_utc_time_millis,
            android_gnss.epoch_hardware_clock_discontinuity_count,
            leveling_config);
        if (!leveling.ok) {
            std::cerr << "carrier-code leveling failed closed: "
                      << leveling.error << "\n";
            return 1;
        }
        epochs = leveling.observations.epochs;
        carrier_code_leveling_report = leveling.diagnostics;
    }

    libgnss::FGOProcessor::FGOConfig config;
    config.use_native_tdcp_frequency_residual_states =
        options.native_tdcp_frequency_residual_states;
    config.use_native_lm_lambda_floor = options.native_lm_lambda_floor;
    config.monitor_motion_constraints = options.native_nhc_monitor;
    config.use_native_batch_nhc = options.native_batch_nhc;
    config.use_native_joint_ionosphere = options.native_joint_ionosphere;
    config.use_native_doppler_rotation_rate = options.native_doppler_rotation_rate;
    config.use_native_tdcp_adr_endpoint_sigma = options.native_tdcp_adr_endpoint_sigma;
    config.native_joint_ionosphere_anchor_sigma_m = options.joint_ionosphere_anchor;
    config.native_joint_ionosphere_density_m_sqrt_s = options.joint_ionosphere_density;
    config.native_joint_ionosphere_max_gap_s = options.joint_ionosphere_gap;
    // Phase412: fixed prospective hypothesis, not a calibrated ionosphere prior.
    config.native_tdcp_frequency_residual_prior_sigma_m =
        options.native_tdcp_frequency_residual_states ? 0.01 : 0.0;
    config.backend = libgnss::FGOBackend::GTSAM;
    config.max_iterations = 12;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_double_difference_factors = false;
    config.use_pseudorange_factors = true;
    config.use_carrier_phase_factors = false;
    // Ordinary TDCP remains disabled for the historical raw lane.  The
    // Phase10 candidate enables the already audited same-satellite/same-signal
    // ADR path only through its explicit opt-in, with no standalone ambiguity
    // or base/double-difference factors.
        config.use_tdcp_factors = options.native_pdc_imu_tdcp;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    if (android_raw) {
        // Upstream's first GNSS pass is P+D with an explicit velocity state.
        // The corrected Android Doppler contract is already validated by the
        // raw adapter; keep it opt-in to this research entry point only.
        config.use_undifferenced_doppler_factors = true;
        config.use_corrected_undifferenced_doppler_factors = true;
    }
    if (options.fgo_imu_sparse_recovery) {
        // One fixed, raw-only recovery candidate: retain epochs below the
        // normal four-satellite floor for the IMU/temporal chain and admit
        // all supported secondary raw frequencies to the same P/D graph.
        // This is opt-in; the production/default graph remains unchanged.
        config.retain_sparse_epochs_for_imu = true;
        config.use_multi_frequency_double_difference = true;
    }
    if (options.native_pdc_state_bridge) {
        // One frozen bridge recipe: solve the full native P+D+temporal state
        // from the same in-memory raw rows, then use finite states only as
        // initial values for the CombinedImuFactor graph. No duplicate P/D
        // priors are added because the graph owns those measurements.
        // Secondary-frequency DD is intentionally not enabled here; this is a
        // PDC-state bridge, not the previously rejected sparse multi-frequency
        // coverage candidate.
        config.retain_sparse_epochs_for_imu = true;
        config.use_doppler_velocity_wls_initialization = true;
        // Match the already-used raw native SPP seed contract: a receiver-only
        // first pass avoids rejecting low-count mixed-system epochs before the
        // in-process P+D state solve sees their finite rows.
        config.spp_model_intersystem_bias = false;
        config.use_native_pdc_state_bridge = true;
    }
    if (options.native_direct_doppler_wls_handoff) {
        // Phase40 consumes the existing raw-observable WLS sequence directly
        // as the IMU velocity/heading seed.  Keep the bounded temporal
        // completion enabled at its frozen 1.0 s edge limit; the candidate
        // validates whether every completed estimate is physically usable
        // before constructing the IMU graph.
        config.use_doppler_velocity_wls_initialization = true;
        config.doppler_velocity_wls_edge_hold_max_s = 1.0;
    }
    if (options.native_pdc_imu_tdcp) {
        // Freeze the ordinary TDCP contract explicitly instead of inheriting
        // mutable library defaults: 30 ms ADR-difference noise, a 2 s
        // adjacent-epoch limit, and fail-closed loss-of-lock/code-phase gates.
        config.tdcp_sigma_m = kNativeTdcpSigmaM;
        config.max_tdcp_gap_s = kNativeTdcpMaxGapS;
        config.reject_tdcp_loss_of_lock = true;
        config.reject_tdcp_code_phase_jump = true;
        config.tdcp_code_phase_jump_threshold_m =
            kNativeTdcpCodePhaseJumpThresholdM;
    }
    if (options.native_signal_bias_states) {
        // Phase11 candidate: attach static meter-valued receiver secondary
        // signal-bias states to raw undifferenced code factors.  The prior is
        // frozen by the phase record and only regularizes the global gauge;
        // all Phase10 P/D/TDCP/IMU settings remain unchanged.  The common
        // FGO eligibility gate admits secondary raw observations only when
        // multi-frequency mode is enabled; enabling that gate here is what
        // makes the declared GPS L5/Galileo E5a bias states observable.  It
        // does not form a DD/IFLC combination or alter the primary factors.
        config.use_multi_frequency_double_difference = true;
        config.use_receiver_signal_bias_states = true;
        config.receiver_signal_bias_prior_sigma_m = 1000.0;
    }
    if (options.native_residual_ionosphere) {
        // Phase12 candidate: retain the Phase11 static receiver IFB states
        // and add one raw-geometry-mapped residual L1 ionosphere state per
        // epoch.  These are fixed physical defaults (thin-shell mapping in
        // the contract header, weak zero gauge, and random walk); no value is
        // selected from truth or a trajectory.
        config.use_residual_ionosphere_states = true;
        config.residual_ionosphere_prior_sigma_m = 10.0;
        config.residual_ionosphere_random_walk_sigma_m_per_sqrt_s = 1.0;
        config.residual_ionosphere_max_abs_m = 30.0;
        config.residual_ionosphere_max_gap_s = 2.0;
    }
    if (options.native_upstream_quality) {
        // Phase13 fixed raw-observable contract.  This is deliberately kept
        // separate from the frozen Phase12 TDCP sigma: only code and
        // receiver-only Doppler factors receive the upstream SNR model.
        config.use_upstream_observable_quality = true;
        config.upstream_snr_percentile = 85.0;
        config.upstream_min_snr_dbhz = 20.0;
        config.upstream_min_elevation_deg = 5.0;
        config.upstream_max_adjacent_gap_s = 1.5;
        // Pixel devices in the declared development recipe do not use the
        // legacy carrier sign-offset list.  An empty model is intentional:
        // it prevents device identity from selecting a quality mask.
        config.upstream_device_model.clear();
    }
    if (options.native_source_direct_observable_quality) {
        // Phase80 uses the exact source P+D quality contract in the main
        // receiver-only graph.  This block deliberately does not set any
        // PDC bridge option; the bridge branch below remains guarded only by
        // the legacy --native-upstream-quality flag.
        const DirectObservableQualitySettings direct_settings =
            directObservableQualitySettingsForDataset(options.dataset_id);
        if (!direct_settings.valid) {
            std::cerr << "direct Phase80 observable-quality dataset mapping is invalid\n";
            return 1;
        }
        config.use_upstream_observable_quality = true;
        config.upstream_snr_percentile = 85.0;
        config.upstream_min_snr_dbhz = 20.0;
        config.upstream_min_elevation_deg = 5.0;
        config.upstream_max_adjacent_gap_s = 1.5;
        config.upstream_device_model.clear();
        config.pseudorange_huber_threshold_sigma =
            direct_settings.pseudorange_huber_threshold_sigma;
        config.undifferenced_doppler_huber_threshold_sigma =
            direct_settings.doppler_huber_threshold_sigma;
    }
    if (options.native_source_clock_c0d_factor) {
        // Phase84 consumes only the exact adjacent-epoch C0/D row.  Pass the
        // phone component separately so the backend can apply the source
        // exclusion list without inspecting route/path names.
        config.use_native_source_clock_c0d_factor = true;
        config.native_source_clock_c0d_phone =
            phoneFromDatasetId(options.dataset_id);
    }
    if (options.native_source_clock_c0d_meter_state_parity) {
        config.use_native_source_clock_c0d_meter_state_parity = true;
    }
    if (options.native_source_clock_c0d_active_solve_diagnostic) {
        config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    }
    if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer) {
        config.use_native_source_clock_c0d_raw_drift_d_initializer = true;
    }
    if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {
        config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    }
    if (options.native_source_clock_c0d_epoch_vector_parity) {
        config.use_native_source_clock_c0d_epoch_vector_parity = true;
        // Phase101 owns every inter-system component inside the epoch-local
        // C vector.  Do not let the legacy global `i` state be inserted by
        // either staged or main graph construction.
        config.use_inter_system_biases = false;
    }
    if (options.native_source_clock_c0d_phase96_main_diagnostics) {
        config.use_native_source_clock_c0d_phase96_main_diagnostics = true;
    }
    if (options.native_source_clock_c0d_phase97_singular_system_diagnostics) {
        config.use_native_source_clock_c0d_phase97_singular_system_diagnostics =
            true;
    }
    if (options.native_source_clock_c0d_phase98_solver_rank_diagnostic) {
        config.use_native_source_clock_c0d_phase98_solver_rank_diagnostic = true;
    }
    if (options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver) {
        config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
            true;
    }
    if (options.native_direct_wls_ephemeral_c7d_main_seed) {
        // Phase114 keeps every existing graph/noise/LM setting and only
        // switches the source of the main initial state.  The WLS sequence is
        // built by the same FGO problem builder from this raw run; no staging
        // optimizer is constructed for this selector.
        config.use_native_direct_wls_ephemeral_c7d_main_seed = true;
        config.use_doppler_velocity_wls_initialization = true;
        config.doppler_velocity_wls_edge_hold_max_s = 1.0;
    }
    if (options.native_phase116_carrier_tdcp_incidence_diagnostic) {
        // Phase116 is a post-build/read-only report over the ordinary TDCP
        // population.  This flag is intentionally the only config mutation
        // introduced by the candidate and has no effect on graph construction.
        config.use_carrier_tdcp_incidence_diagnostic = true;
    }
    if (options.native_phase117_tdcp_snr_type_sigma) {
        // Phase117 changes only the scalar noise attached to already
        // accepted ordinary TDCP pairs.  The source p85/type table and
        // cycles-to-metres conversion live in the library builder; all
        // residual, admission, robust-loss, and solver settings remain fixed.
        config.use_official_tdcp_snr_type_sigma = true;
    }
    if (options.native_phase118_official_tdcp_huber_k) {
        // Phase118 changes only the ordinary TDCP robust threshold.  Keep the
        // Phase112 0.03 m factor sigma explicit and pass the exact route
        // setting.Type to the library resolver; no P/D or dynamic-sigma
        // setting is changed here.
        const DirectObservableQualitySettings direct_settings =
            directObservableQualitySettingsForDataset(options.dataset_id);
        if (!direct_settings.valid || direct_settings.environment.empty()) {
            throw std::invalid_argument(
                "Phase118 requires a recognised source setting.Type");
        }
        config.use_official_tdcp_huber_k = true;
        config.official_tdcp_setting_type = direct_settings.environment;
        config.tdcp_sigma_m = kNativeTdcpSigmaM;
        config.use_official_tdcp_snr_type_sigma = false;
    }
    if (options.native_phase120_official_tdcp_resl_atmosphere_cancellation) {
        // Phase120 changes only the ordinary TDCP measurement preparation.
        // Keep the Phase118 fixed sigma/Type-k and every graph/factor setting
        // untouched; the library owns the source-parity formula.
        config.use_official_tdcp_resl_atmosphere_cancellation = true;
    }
    if (options.native_rover_epoch_states) {
        // Independent ablation: no base mask/correction or code-bias change.
        config.use_source_rover_epoch_states = true;
    }
    if (options.native_paired_epoch_states) {
        config.use_source_rover_epoch_states = true;
        config.use_native_phase126_raw_base_source_complete = true;
    }
    if (options.native_phase126_raw_base_source_complete) {
        // The Phase126 compound operator owns the symmetric official
        // no-explicit-TGD/BGD policy at both the SPP seed and FGO factor
        // preparation boundaries.  No graph family, factor, unit, or solver
        // setting is changed here.
        config.use_native_phase126_raw_base_source_complete = true;
    }
    if (options.native_phase127_glonass_channel_provenance) {
        // Phase127 is a provenance/admission overlay only.  The existing
        // Phase126 base model receives the RINEX header ledger; the FGO
        // observation builders use the same exact query-time broadcast FCN
        // through their local observation copies.
        config.use_native_phase127_glonass_channel_provenance = true;
    }
    if (options.native_phase128_glonass_provenance_parser_admission) {
        // Phase128 is a parser/admission-only overlay.  It consumes the
        // native GPST/geph metadata and exact typed keys, without changing
        // any observation equation, factor, or solver setting.
        config.use_native_phase128_glonass_provenance_parser_admission = true;
    }
    if (options.native_phase129_glonass_local_miss_mask) {
        // Phase129 changes only the local admission outcome for an
        // uncertified GLO row.  The same Phase127/128 resolver and reason
        // ledger are used by the base stream and rover factor builder.
        config.use_native_phase129_glonass_local_miss_mask = true;
    }
    if (options.native_phase131_canonical_correction_band_key) {
        // Phase131 changes only the source/base correction lookup key.  The
        // original typed SignalType remains on every factor, while certified
        // GLONASS FCN provenance is carried separately for the canonical
        // stream lookup; no graph/factor/equation setting is changed.
        config.use_native_phase131_canonical_correction_band_key = true;
    }
    if (options.native_phase135_official_affine_measurement_family) {
        // Phase135 is a transactional graph-family selector.  All of its
        // prerequisites were validated above; this single assignment is the
        // only library configuration mutation owned by the CLI option.
        config.use_native_phase135_official_affine_measurement_family = true;
        // The structural Phase135 recipe may intentionally retain the
        // already-frozen Phase107 raw-base/miss-mask path while leaving the
        // later Phase126-134 compound selectors off.  The app has already
        // validated the hash-bound RINEX path and exact selector combination;
        // carry that admission boundary into the library without changing
        // any correction equation or graph factor.
        config.use_native_phase135_phase107_raw_base_recipe =
            options.native_base_pseudorange_compensation &&
            options.native_base_pseudorange_source_miss_mask &&
            !options.native_base_pseudorange_preserve_additional_frequency_bands &&
            !options.native_phase126_raw_base_source_complete &&
            !options.native_phase127_glonass_channel_provenance &&
            !options.native_phase128_glonass_provenance_parser_admission &&
            !options.native_phase129_glonass_local_miss_mask &&
            !options.native_phase131_canonical_correction_band_key;
    }
    if (options.native_phase138_affine_tdcp_anchor_range_constant) {
        // Phase138 changes only the ordinary Phase135 TDCP measurement
        // constant.  The app validation above establishes the dependency and
        // frozen Phase118 composition; no other graph/configuration setting is
        // altered by this assignment.
        config.use_native_phase138_affine_tdcp_anchor_range_constant = true;
    }
    if (options.native_phase143_official_main_lm_termination_budget) {
        // Phase143 changes only the maxIterations value at the existing
        // GTSAM main-solve boundary.  The backend applies the 1000 cap only
        // after identifying the Phase99 meter-state main graph; the copied
        // GNSS-first configuration remains at its existing 1000 cap.
        config.use_native_phase143_official_main_lm_termination_budget = true;
    }
    if (options.native_upstream_absolute_doppler_screen) {
        // Phase23 isolated source-level port: use the raw Android receiver
        // clock drift in exobs_residuals.m's absolute D residual gate while
        // leaving Phase12 weights, factors, and Huber settings untouched.
        config.use_upstream_absolute_doppler_residual_screen = true;
        config.upstream_absolute_doppler_residual_threshold_mps = 3.0;
    }
    if (options.native_upstream_stop_constraints) {
        // These values are the pinned upstream parameters; they are not
        // selected from a truth file or from a route score.  Keep them in the
        // config object so the backend and its structural diagnostics expose
        // the complete candidate contract.
        config.use_upstream_stop_constraints = true;
        config.upstream_stop_window_samples = 500;
        config.upstream_stop_acceleration_std_offset_mps2 = 0.08;
        config.upstream_stop_gyro_std_offset_radps = 0.005;
        config.upstream_stop_gyro_norm_max_radps = 0.05;
        config.upstream_stop_velocity_threshold_mps = 0.5;
        config.upstream_stop_velocity_sigma_mps = 0.01;
        config.upstream_stop_velocity_huber_k_sigma = 0.5;
        config.upstream_stop_pose_rotation_sigma_rad =
            0.1 * kPi / 180.0;
        config.upstream_stop_pose_translation_sigma_m = 0.02;
        config.upstream_stop_pose_huber_k_sigma = 0.5;
    }
    config.pose3_lever_arm_body_m = libgnss::Vector3d::Zero();
    config.use_signal_specific_galileo_group_delay =
        options.native_signal_specific_galileo_tgd;
    config.use_quality_anchor_initialization =
        options.native_quality_anchor ||
        options.native_fallback_seed_quality_anchor_recovery;
    config.use_fallback_seed_quality_anchor_recovery =
        options.native_fallback_seed_quality_anchor_recovery;
    config.use_native_android_sv_time_uncertainty_sigma_floor =
        options.native_android_sv_time_uncertainty_sigma_floor;
    config.use_native_cn0_doppler_calibration =
        options.native_cn0_doppler_calibration;
    if (android_raw) {
        // The raw path starts SPP from the route-independent Earth-surface
        // initializer above.  Applying a nonzero elevation mask at that
        // deliberately distant point can discard the whole first epoch
        // before SPP has a chance to converge.  The native SPP seed still
        // applies its finite geometry/health checks; this only makes raw
        // startup independent of a device-provided coordinate.
        config.min_elevation_deg = 0.0;
        config.min_snr_dbhz = 0.0;
    }
    if (options.native_phase165_raw_p_no_doppler_graph) {
        // Dedicated Phase165 GNSS-only graph.  The raw-P adapter has already
        // populated the in-memory receiver seed before this builder call;
        // disabling SPP here prevents a second chronological/held fallback
        // from replacing that exact handoff.
        config.backend = libgnss::FGOBackend::GTSAM;
        config.use_pose3_state = false;
        config.use_imu = false;
        config.use_spp_seed = false;
        config.use_multi_constellation = true;
        config.use_multi_frequency_double_difference = true;
        config.use_velocity_states = true;
        config.use_motion_factors = true;
        config.use_position_motion_factors = false;
        config.use_velocity_motion_factors = true;
        config.use_clock_motion_factors = true;
        config.use_native_source_clock_c0d_factor = true;
        config.native_source_clock_c0d_phone =
            phoneFromDatasetId(options.dataset_id);
        config.use_native_source_clock_c0d_meter_state_parity = true;
        config.use_native_source_clock_c0d_epoch_vector_parity = true;
        config.use_native_raw_p_no_doppler_graph = true;
        config.use_inter_system_biases = false;
        config.use_receiver_signal_bias_states = false;
        config.use_residual_ionosphere_states = false;
        config.use_undifferenced_doppler_factors = false;
        config.use_single_difference_doppler_factors = false;
        config.use_single_difference_tdcp_factors = false;
        config.use_carrier_phase_factors = false;
        config.use_double_difference_factors = false;
        config.use_tdcp_factors = true;
        config.use_doppler_velocity_wls_initialization = false;
        if (options.native_phase167_raw_p_no_doppler_lm_termination_budget) {
            // Phase167 changes only the dedicated graph's LM budget and
            // enables the already-audited native Phase143 termination trace.
            // Factors, Values, initialization, tolerances, and all graph
            // admission settings above remain exactly the Phase165 recipe.
            config.max_iterations = 1000;
            config.use_native_phase167_raw_p_no_doppler_lm_termination_budget =
                true;
        }
    }
    if (options.native_phase171_raw_p_no_doppler_imu_main) {
        // Phase171 keeps the normal Pose3+IMU main graph as the config owned
        // by `processor`; only its in-memory GNSS-first staging copy below
        // receives the Phase165/167 Point3/V/C7/D recipe.  The main graph
        // consumes the resulting C/D/velocity handoff and retains P/TDCP,
        // while its generic Doppler family is intentionally empty.
        config.backend = libgnss::FGOBackend::GTSAM;
        config.max_iterations = 12;
        config.use_pose3_state = true;
        config.use_imu = true;
        config.use_spp_seed = false;
        config.use_undifferenced_doppler_factors = false;
        config.use_corrected_undifferenced_doppler_factors = false;
        config.use_single_difference_doppler_factors = false;
        config.use_single_difference_tdcp_factors = false;
        config.use_carrier_phase_factors = false;
        config.use_double_difference_factors = false;
        config.use_tdcp_factors = true;
        config.use_native_source_clock_c0d_factor = true;
        config.native_source_clock_c0d_phone =
            phoneFromDatasetId(options.dataset_id);
        config.use_native_source_clock_c0d_meter_state_parity = true;
        config.use_native_source_clock_c0d_epoch_vector_parity = true;
        config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
        config.use_native_source_clock_c0d_raw_drift_d_initializer = false;
        config.use_native_source_clock_c0d_active_solve_diagnostic = true;
        config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
            true;
        if (options.native_phase184_source_tdcp_huber_k) {
            // Phase184 is the raw no-Doppler port of the source GNSS/IMU
            // recipe.  The cached source parameters use L_robust_prm=0.2 for
            // Street/Mix (and 0.5 for Highway), whereas the native legacy
            // default is 4.0.  Carry the dedicated Type mapping into both the
            // GNSS-first staging copy and the main graph below, while keeping
            // the source fixed TDCP sigma and every legacy/default path
            // unchanged.
            const DirectObservableQualitySettings phase184_tdcp_settings =
                directObservableQualitySettingsForDataset(options.dataset_id);
            if (!phase184_tdcp_settings.valid ||
                !std::isfinite(
                    phase184_tdcp_settings.official_tdcp_huber_threshold_sigma) ||
                !(phase184_tdcp_settings.official_tdcp_huber_threshold_sigma > 0.0)) {
                throw std::invalid_argument(
                    "Phase184 requires a recognised source setting.Type for "
                    "TDCP Huber-k parity");
            }
            config.use_native_phase184_source_tdcp_huber_k = true;
            config.native_phase184_tdcp_setting_type =
                phase184_tdcp_settings.environment;
            config.tdcp_sigma_m = kNativeTdcpSigmaM;
            config.use_official_tdcp_huber_k = false;
            config.official_tdcp_setting_type.clear();
        }
        config.use_official_tdcp_snr_type_sigma = false;
        // Reuse the already source-pinned Phase143 boundary for the main
        // graph: configured max_iterations remains 12, while the backend's
        // dedicated selector makes the effective LM cap 1000 and records the
        // native termination sidecar.  The Phase167 staging copy below keeps
        // its own 1000-iteration selector.
        config.use_native_phase143_official_main_lm_termination_budget = true;
        config.use_native_phase171_raw_p_no_doppler_imu_main = true;
        config.use_native_phase201_source_inclusive_forward_imu_schedule =
            options.native_phase201_source_inclusive_forward_imu_schedule;
        config.use_native_phase205_source_count_bias_density =
            options.native_phase205_source_count_bias_density;
        config.use_native_phase209_source_separate_imu_factors =
            options.native_phase209_source_separate_imu_factors;
        config.use_native_phase213_main_doppler = options.native_phase213_main_doppler;
        config.use_native_phase217_main_pose3_motion = options.native_phase217_main_pose3_motion;
        config.use_source_tdcp_meter_sigma = options.native_source_tdcp_meter_sigma;
        config.use_source_tdcp_resl_observable = options.native_source_tdcp_resl_observable;
        config.use_native_tdcp_only_affine_geometry = options.native_tdcp_only_affine_geometry;
        config.use_native_epoch_heading_attitude_seeds = options.native_epoch_heading_attitude_seeds;
        config.omit_native_first_imu_bias_prior = options.native_omit_first_imu_bias_prior;
        config.omit_native_first_imu_velocity_prior = options.native_omit_first_imu_velocity_prior;
        config.use_native_relative_height_pairs = options.native_relative_height_pairs;
        config.retain_native_pseudorange_remasking_pool = options.native_pseudorange_remasking;
        config.use_native_raw_p_no_doppler_graph = false;
        // The dedicated ECEF-D selector belongs only to the GNSS-first
        // staging copy below.  The Pose3+IMU main graph keeps its existing
        // empty generic-D contract and performs one ECEF->ENU handoff.
        config.use_native_raw_p_ecef_doppler_gnss_first = false;
        config.use_native_phase167_raw_p_no_doppler_lm_termination_budget = false;
        config.use_inter_system_biases = false;
        config.use_receiver_signal_bias_states = false;
        config.use_residual_ionosphere_states = false;
        config.use_native_pdc_state_bridge = false;
        config.use_doppler_velocity_wls_initialization = false;
    }
    config.use_main_pseudorange_cauchy_loss=options.native_main_p_cauchy;
    // Applied to both same-run GNSS-first and main problem construction.
    if (options.native_tdcp_no_code_jump_gate) {
        if (!config.use_upstream_observable_quality || !config.reject_tdcp_loss_of_lock)
            throw std::invalid_argument("TDCP code-gate ablation requires upstream masks and slip rejection");
        config.reject_tdcp_code_phase_jump = false;
    }
    const libgnss::FGOProcessor processor(config);
    libgnss::FGOProcessor::FGOProblem problem;
    NativePdcBridgeReport pdc_bridge_report;
    bool pdc_bridge_prepopulated = false;
    if (options.native_upstream_quality) {
        // Keep the frozen Phase12 PDC initializer independent of the new
        // residual/SNR masks.  The initializer is not a second output lane:
        // its finite state is passed in-memory into the quality-enabled graph,
        // while every final P/D factor is built from the quality problem below.
        // This prevents the upstream SNR sigma (which is intentionally much
        // smaller than the PDC bridge's broad physical gate) from making the
        // unchanged initializer fail before the candidate graph is tested.
        libgnss::FGOProcessor::FGOConfig bridge_config = config;
        bridge_config.use_upstream_observable_quality = false;
        const libgnss::FGOProcessor bridge_processor(bridge_config);
        auto bridge_problem =
            bridge_processor.buildPseudorangeProblem(epochs, nav);
        if (options.native_phase127_glonass_channel_provenance &&
            !bridge_problem.diagnostics.phase127_failure.empty() &&
            (!options.native_phase129_glonass_local_miss_mask ||
             !bridge_problem.diagnostics.phase129_configuration_valid)) {
            std::cerr << "Phase127 GLONASS provenance failed closed in the "
                         "initializer: "
                      << bridge_problem.diagnostics.phase127_failure << "\n";
            return 1;
        }
        if (bridge_problem.epochs.size() != epochs.size() ||
            !populateNativePdcStateBridge(bridge_problem, pdc_bridge_report)) {
            if (pdc_bridge_report.failure.empty()) {
                pdc_bridge_report.failure =
                    "unfiltered Phase12 PDC initializer epoch alignment failed";
            }
            std::cerr << "native PDC state bridge failed closed: "
                      << pdc_bridge_report.failure << "\n";
            return 1;
        }
        problem = processor.buildPseudorangeProblem(epochs, nav);
        if (problem.epochs.size() != bridge_problem.epochs.size()) {
            std::cerr << "quality candidate changed epoch indexing relative to the "
                         "frozen PDC initializer\n";
            return 1;
        }
        problem.native_pdc_state_seeds =
            std::move(bridge_problem.native_pdc_state_seeds);
        pdc_bridge_prepopulated = true;
    } else {
        problem = processor.buildPseudorangeProblem(epochs, nav);
    }
    if (options.native_phase127_glonass_channel_provenance &&
        !problem.diagnostics.phase127_failure.empty() &&
        (!options.native_phase129_glonass_local_miss_mask ||
         !problem.diagnostics.phase129_configuration_valid)) {
        std::cerr << "Phase127 GLONASS provenance failed closed before factor "
                     "construction: "
                  << problem.diagnostics.phase127_failure << "\n";
        return 1;
    }
    if (options.native_phase165_raw_p_no_doppler_graph) {
        // Re-key the typed adapter by the immutable source identity retained
        // by the builder.  Vector position is not a valid join key because
        // ordinary factor admission may remove an epoch.
        std::vector<libgnss::raw_p_seed::RawPNoDopplerSeed> retained_seeds;
        retained_seeds.reserve(problem.epochs.size());
        bool identity_ok = true;
        std::string identity_failure;
        for (std::size_t retained_index = 0;
             retained_index < problem.epochs.size(); ++retained_index) {
            const auto& epoch_seed = problem.epochs[retained_index];
            const auto match = std::find_if(
                phase165_adapter.seeds.begin(), phase165_adapter.seeds.end(),
                [&](const auto& seed) {
                    return seed.time == epoch_seed.time &&
                           seed.raw_source_index == epoch_seed.raw_source_index &&
                           seed.raw_utc_time_millis ==
                               epoch_seed.raw_utc_time_millis;
                });
            if (match == phase165_adapter.seeds.end()) {
                identity_ok = false;
                identity_failure = "retained-epoch-source-identity-not-found";
                break;
            }
            auto retained_seed = *match;
            retained_seed.epoch_index = retained_index;
            retained_seeds.push_back(std::move(retained_seed));
        }
        if (!identity_ok || retained_seeds.size() != problem.epochs.size() ||
            problem.epochs.size() < 2U || problem.pseudorange_factors.empty()) {
            const std::string failure = identity_failure.empty()
                                             ? "raw-P graph retained problem is empty-or-short"
                                             : identity_failure;
            atomicWrite(options.summary_path,
                        makePhase165RawPNoDopplerGraphJson(
                            options.dataset_id, phase165_adapter,
                            phase165_raw_result_available ? &phase165_raw_result
                                                          : nullptr,
                            &problem, nullptr, failure, false, false,
                            options.native_phase167_raw_p_no_doppler_lm_termination_budget));
            std::cerr << "Phase165 raw-P graph handoff failed closed: "
                      << failure << "\n";
            return 1;
        }
        problem.native_raw_p_no_doppler_seeds = std::move(retained_seeds);
        if (options.native_phase171_raw_p_no_doppler_imu_main) {
            // Phase171 consumes this exact retained-key seed vector in the
            // following GNSS-first staging call.  It must not launch the
            // Phase165 terminal GNSS-only graph or duplicate that solve.
        } else {
        libgnss::FGOProcessor::FGOResult phase165_result;
        std::string phase165_failure;
        bool phase165_fgo_returned = false;
        try {
            phase165_result = processor.optimizeProblem(problem);
            phase165_fgo_returned = true;
            if (!phase165_result.diagnostics.converged ||
                phase165_result.solution.isEmpty()) {
                phase165_failure = "dedicated-no-doppler-graph-did-not-converge";
            }
        } catch (const std::exception& error) {
            phase165_failure = error.what();
        } catch (...) {
            phase165_failure = "dedicated-no-doppler-graph-unknown-exception";
        }
        if (!atomicWrite(options.summary_path,
                         makePhase165RawPNoDopplerGraphJson(
                             options.dataset_id, phase165_adapter,
                             phase165_raw_result_available ? &phase165_raw_result
                                                           : nullptr,
                             &problem, &phase165_result, phase165_failure,
                             true, phase165_fgo_returned,
                             options.native_phase167_raw_p_no_doppler_lm_termination_budget))) {
            std::cerr << "failed to write Phase165 raw-P graph summary\n";
            return 1;
        }
        if (!phase165_failure.empty()) {
            std::cerr << "Phase165 raw-P graph failed closed: "
                      << phase165_failure << "\n";
            return 1;
        }
        // This selector is a GNSS-only structural/graph lane.  It must never
        // fall through to the IMU/main output serializer.
        return 0;
        }
    }
    if (options.native_base_pseudorange_compensation) {
        base_pseudorange_report.adopted_pseudorange_rows =
            problem.pseudorange_factors.size();
        if (options.native_base_pseudorange_source_miss_mask) {
            base_pseudorange_report.source_miss_mask_enabled = true;
            base_pseudorange_report.source_miss_mask_canonical_key_mode =
                options.native_phase131_canonical_correction_band_key;
            base_pseudorange_report.source_miss_mask_matching_key =
                options.native_phase131_canonical_correction_band_key
                    ? "(GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN])"
                    : "(satellite,signal)";
            libgnss::source_pseudorange_miss_mask::Report miss_mask_report;
            bool mask_ok = false;
            if (options.native_phase131_canonical_correction_band_key) {
                mask_ok =
                    libgnss::source_pseudorange_miss_mask::applyCanonical(
                        problem.pseudorange_factors, problem.epochs,
                        [&base_pseudorange_model](
                            const libgnss::SatelliteId& satellite,
                            libgnss::SignalType signal,
                            bool has_glonass_frequency_channel,
                            int glonass_frequency_channel) {
                            return base_pseudorange_model.hasCanonicalStream(
                                satellite, signal,
                                has_glonass_frequency_channel,
                                glonass_frequency_channel);
                        },
                        [&base_pseudorange_model](
                            const libgnss::GNSSTime& time,
                            const libgnss::SatelliteId& satellite,
                            libgnss::SignalType signal,
                            bool has_glonass_frequency_channel,
                            int glonass_frequency_channel,
                            double& correction_m) {
                            return base_pseudorange_model.correctionAtCanonical(
                                time, satellite, signal,
                                has_glonass_frequency_channel,
                                glonass_frequency_channel, correction_m);
                        },
                        miss_mask_report);
            } else {
                libgnss::source_pseudorange_miss_mask::CorrectionAt correction_at =
                    [&base_pseudorange_model](const libgnss::GNSSTime& time,
                                              const libgnss::SatelliteId& satellite,
                                              libgnss::SignalType signal,
                                              double& correction_m) {
                        return base_pseudorange_model.correctionAt(
                            time, satellite, signal, correction_m);
                    };
                std::vector<libgnss::FGOProcessor::PseudorangeFactor> mask_only_reference;
                if (options.native_base_mask_only_ablation || options.native_base_gps_values_only_ablation) {
                    // Verify support against actual subtraction in a private
                    // same-process copy, never a saved graph/position input.
                    mask_only_reference = problem.pseudorange_factors;
                    libgnss::source_pseudorange_miss_mask::Report reference_report;
                    if (!libgnss::source_pseudorange_miss_mask::apply(
                            mask_only_reference, problem.epochs,
                            [&base_pseudorange_model](const libgnss::SatelliteId& sat,
                                                      libgnss::SignalType signal) {
                                return base_pseudorange_model.hasStream(sat, signal);
                            }, correction_at, reference_report)) {
                        std::cerr << "mask-only reference support check failed\n";
                        return 1;
                    }
                    if (options.native_base_gps_center_ablation) {
                        using CenterKey=std::pair<libgnss::SatelliteId,libgnss::SignalType>;
                        correction_at=libgnss::source_pseudorange_miss_mask::gpsCenteredValuesAblation(
                            correction_at,
                            [&base_pseudorange_model,cache=std::map<CenterKey,double>{}]
                            (const libgnss::SatelliteId& sat,libgnss::SignalType signal,double& value) mutable {
                                const CenterKey key{sat,signal};
                                const auto found=cache.find(key);
                                if (found!=cache.end()) {value=found->second;return true;}
                                if (!base_pseudorange_model.streamMedian(sat,signal,value)) return false;
                                cache.emplace(key,value);
                                return true;
                            });
                    } else correction_at = options.native_base_gps_values_only_ablation
                        ? libgnss::source_pseudorange_miss_mask::gpsValuesOnlyAblation(correction_at)
                        : libgnss::source_pseudorange_miss_mask::finiteMaskOnlyAblation(correction_at);
                }
                mask_ok = libgnss::source_pseudorange_miss_mask::apply(
                    problem.pseudorange_factors, problem.epochs,
                    [&base_pseudorange_model](const libgnss::SatelliteId& satellite,
                                              libgnss::SignalType signal) {
                        return base_pseudorange_model.hasStream(satellite, signal);
                    },
                    correction_at,
                    miss_mask_report);
                if ((options.native_base_mask_only_ablation || options.native_base_gps_values_only_ablation) &&
                    (mask_only_reference.size() != problem.pseudorange_factors.size() ||
                     !std::equal(mask_only_reference.begin(), mask_only_reference.end(),
                                 problem.pseudorange_factors.begin(),
                                 [](const auto& expected, const auto& actual) {
                                     return expected.epoch_index == actual.epoch_index &&
                                            expected.satellite == actual.satellite &&
                                            expected.signal == actual.signal;
                                 }))) {
                    std::cerr << "mask-only retained factor identities differ\n";
                    return 1;
                }
            }
            base_pseudorange_report.original_adopted_pseudorange_rows =
                miss_mask_report.original_adopted_rows;
            base_pseudorange_report.retained_finite_pc_pseudorange_rows =
                miss_mask_report.retained_finite_pc_rows;
            base_pseudorange_report.dropped_missing_exact_stream_rows =
                miss_mask_report.dropped_missing_exact_stream_rows;
            base_pseudorange_report.dropped_out_of_domain_rows =
                miss_mask_report.dropped_out_of_domain_rows;
            base_pseudorange_report.dropped_nonfinite_correction_rows =
                miss_mask_report.dropped_nonfinite_correction_rows;
            base_pseudorange_report.retained_finite_pc_fraction =
                miss_mask_report.retained_finite_pc_fraction;
            base_pseudorange_report.retained_over_original_fraction =
                miss_mask_report.retained_over_original_fraction;
            base_pseudorange_report.pseudorange_factor_count_consistent =
                miss_mask_report.factor_count_consistent;
            base_pseudorange_report.adopted_rows_corrected =
                miss_mask_report.retained_finite_pc_rows;
            base_pseudorange_report.interpolated_rows =
                miss_mask_report.retained_finite_pc_rows;
            base_pseudorange_report.interpolation_misses =
                miss_mask_report.dropped_missing_exact_stream_rows +
                miss_mask_report.dropped_out_of_domain_rows +
                miss_mask_report.dropped_nonfinite_correction_rows;
            base_pseudorange_report.matched_factor_rows =
                miss_mask_report.matched_exact_stream_rows;
            base_pseudorange_report.finite_correction_rows_among_matched =
                miss_mask_report.finite_correction_rows_among_matched;
            base_pseudorange_report.correction_abs_p50_m =
                miss_mask_report.correction_abs_p50_m;
            base_pseudorange_report.correction_abs_p95_m =
                miss_mask_report.correction_abs_p95_m;
            base_pseudorange_report.correction_abs_max_m =
                miss_mask_report.correction_abs_max_m;
            base_pseudorange_report.corrected_rows =
                miss_mask_report.corrected_rows;
            base_pseudorange_report.source_miss_taxonomy_by_signal =
                miss_mask_report.signal_counts;
            base_pseudorange_report.signal_count_consistent =
                miss_mask_report.signal_count_consistent;
            base_pseudorange_report.correction_application_pass_count =
                miss_mask_report.correction_application_passes;
            base_pseudorange_report.source_miss_mask_canonical_key_mode =
                miss_mask_report.canonical_key_mode;
            base_pseudorange_report.duplicate_correction_rejected =
                miss_mask_report.correction_already_applied;
            base_pseudorange_report.source_miss_conservation =
                miss_mask_report.signal_count_consistent &&
                miss_mask_report.factor_count_consistent;
            base_pseudorange_report.applied =
                mask_ok && miss_mask_report.factor_count_consistent &&
                miss_mask_report.retained_finite_pc_rows > 0U;
            base_pseudorange_report.correction_applied_exactly_once =
                base_pseudorange_report.applied &&
                base_pseudorange_report.source_model_build_count == 1U &&
                base_pseudorange_report.correction_application_pass_count == 1U &&
                !base_pseudorange_report.duplicate_correction_rejected;
            base_pseudorange_report.no_raw_or_zero_fallback =
                base_pseudorange_report.correction_applied_exactly_once;
            if (!mask_ok || !base_pseudorange_report.applied) {
                base_pseudorange_report.failure = miss_mask_report.failure.empty()
                    ? "source-exact finite-pc miss mask retained no usable factor"
                    : miss_mask_report.failure;
                std::cerr << "source-exact pseudorange miss mask failed closed: "
                          << base_pseudorange_report.failure << "\n";
                return 1;
            }
            if (options.native_phase126_raw_base_source_complete) {
                // Atomic step C is the single point at which the retained
                // corrected factor vector becomes visible to GNSS-first and
                // main graph construction.
                base_pseudorange_report
                    .phase126_atomic_step_c_application_committed =
                    base_pseudorange_report.correction_applied_exactly_once &&
                    base_pseudorange_report.signal_count_consistent &&
                    base_pseudorange_report.pseudorange_factor_count_consistent;
                base_pseudorange_report.phase126_compound_admitted =
                    base_pseudorange_report
                        .phase126_atomic_step_a_raw_ingress_verified &&
                    base_pseudorange_report
                        .phase126_atomic_step_b_source_stream_verified &&
                    base_pseudorange_report
                        .phase126_atomic_step_c_application_committed;
                if (!base_pseudorange_report.phase126_atomic_step_c_application_committed) {
                    base_pseudorange_report.failure =
                        "Phase126 atomic application invariants failed closed";
                    std::cerr << base_pseudorange_report.failure << "\n";
                    return 1;
                }
            }
        } else {
            if (std::any_of(
                    problem.pseudorange_factors.begin(),
                    problem.pseudorange_factors.end(),
                    [](const libgnss::FGOProcessor::PseudorangeFactor& factor) {
                        return factor.native_base_pseudorange_correction_applied;
                    })) {
                base_pseudorange_report.duplicate_correction_rejected = true;
                base_pseudorange_report.failure =
                    "base pseudorange correction was already applied to a factor";
                std::cerr << "base pseudorange compensation failed closed: "
                          << base_pseudorange_report.failure << "\n";
                return 1;
            }
            if (!problem.pseudorange_factors.empty()) {
                base_pseudorange_report.correction_application_pass_count = 1U;
            }
            std::vector<double> absolute_corrections;
            absolute_corrections.reserve(problem.pseudorange_factors.size());
            for (auto& factor : problem.pseudorange_factors) {
                const bool exact_stream_matched = base_pseudorange_model.hasStream(
                    factor.satellite, factor.signal);
                if (exact_stream_matched) {
                    ++base_pseudorange_report.matched_factor_rows;
                }
                if (factor.epoch_index >= problem.epochs.size()) {
                    ++base_pseudorange_report.interpolation_misses;
                    continue;
                }
                double correction_m = std::numeric_limits<double>::quiet_NaN();
                if (!base_pseudorange_model.correctionAt(
                        problem.epochs[factor.epoch_index].time, factor.satellite,
                        factor.signal, correction_m)) {
                    ++base_pseudorange_report.interpolation_misses;
                    continue;
                }
                const double corrected =
                    libgnss::base_pseudorange_compensation::subtractCorrection(
                        factor.corrected_pseudorange_m, correction_m);
                if (!std::isfinite(corrected)) {
                    ++base_pseudorange_report.interpolation_misses;
                    continue;
                }
                if (exact_stream_matched) {
                    ++base_pseudorange_report.finite_correction_rows_among_matched;
                }
                factor.corrected_pseudorange_m = corrected;
                factor.native_base_pseudorange_correction_applied = true;
                ++base_pseudorange_report.adopted_rows_corrected;
                ++base_pseudorange_report.interpolated_rows;
                absolute_corrections.push_back(std::abs(correction_m));
            }
            base_pseudorange_report.correction_abs_p50_m =
                finiteMedian(absolute_corrections);
            base_pseudorange_report.correction_abs_p95_m =
                finitePercentile(absolute_corrections, 95.0);
            base_pseudorange_report.correction_abs_max_m =
                absolute_corrections.empty()
                    ? std::numeric_limits<double>::quiet_NaN()
                    : *std::max_element(absolute_corrections.begin(),
                                        absolute_corrections.end());
            base_pseudorange_report.corrected_rows =
                base_pseudorange_report.adopted_rows_corrected;
            base_pseudorange_report.applied =
                base_pseudorange_report.adopted_pseudorange_rows > 0U &&
                base_pseudorange_report.adopted_rows_corrected > 0U;
            if (!base_pseudorange_report.applied) {
                base_pseudorange_report.failure =
                    "no adopted pseudorange factor had an in-domain base correction";
                std::cerr << "base pseudorange compensation failed closed: "
                          << base_pseudorange_report.failure << "\n";
                return 1;
            }
            base_pseudorange_report.correction_applied_exactly_once =
                base_pseudorange_report.source_model_build_count == 1U &&
                base_pseudorange_report.correction_application_pass_count == 1U &&
                !base_pseudorange_report.duplicate_correction_rejected;
            base_pseudorange_report.source_miss_conservation =
                base_pseudorange_report.applied &&
                base_pseudorange_report.adopted_rows_corrected <=
                    base_pseudorange_report.adopted_pseudorange_rows;
            base_pseudorange_report.no_raw_or_zero_fallback =
                base_pseudorange_report.correction_applied_exactly_once;
        }
    }
    if (options.native_phase131_canonical_correction_band_key &&
        base_pseudorange_report.phase131_enabled) {
        // This is the sole Phase131 report-to-problem synchronization site.
        // It is deliberately after the existing source-exact correction
        // accounting and before any summary is serialized; it has no access
        // to, and cannot mutate, the graph factor/state containers.
        const auto snapshot = phase131DiagnosticsSnapshotFromBaseReport(
            base_pseudorange_report);
        if (!problem.diagnostics.synchronizePhase131Diagnostics(snapshot)) {
            std::cerr << "Phase131 diagnostics bridge attempted more than once\n";
            return 1;
        }
    }
    if (options.native_fallback_seed_quality_anchor_recovery &&
        problem.diagnostics.quality_anchor_recovery_triggered &&
        !problem.diagnostics.quality_anchor_recovery_selected) {
        std::cerr << "fallback-seed quality-anchor recovery found no eligible "
                     "raw/nav anchor; candidate failed closed\n";
        return 1;
    }
    if (options.native_fallback_seed_quality_anchor_recovery &&
        problem.diagnostics.sentinel_factor_bypass) {
        std::cerr << "fallback-seed quality-anchor recovery forbids sentinel "
                     "factor-level bypass\n";
        return 1;
    }
    if (problem.epochs.size() < 2 || problem.pseudorange_factors.empty()) {
        std::cerr << "no-base problem has insufficient seeded pseudorange factors\n";
        return 1;
    }

    if (options.native_pdc_state_bridge &&
        !pdc_bridge_prepopulated &&
        !populateNativePdcStateBridge(problem, pdc_bridge_report)) {
        std::cerr << "native PDC state bridge failed closed: "
                  << pdc_bridge_report.failure << "\n";
        return 1;
    }

    ImuBuildReport imu_report;
    if (android_raw) {
        imu_report.android_gnss_diagnostics = android_gnss.diagnostics;
    }
    std::vector<libgnss::Vector3d> gnss_first_velocities_enu;
    std::vector<double> gnss_first_clock_drift_mps;
    std::vector<libgnss::FGOProcessor::EpochClockBiasComponentsM>
        gnss_first_clock_bias_components_m;
    std::vector<libgnss::Vector3d> direct_doppler_wls_velocities_enu;
    std::vector<libgnss::Vector3d>
        direct_wls_ephemeral_velocity_ecef_mps;
    std::vector<libgnss::Vector3d>
        direct_wls_ephemeral_velocity_enu;
    std::vector<double> direct_wls_ephemeral_d_handoff_mps;
    std::vector<libgnss::FGOProcessor::EpochClockBiasComponentsM>
        direct_wls_ephemeral_c_handoff_m;
    bool gnss_first_ok = !android_raw;
    bool direct_doppler_wls_ok = !android_raw;
    bool direct_wls_ephemeral_ok = !android_raw;
    if (android_raw && options.native_direct_wls_ephemeral_c7d_main_seed) {
        // Phase114 bypasses the GNSS-first optimizer entirely.  Validate the
        // existing raw-observable WLS estimates first for the same physical
        // bounds used by the legacy direct-WLS contract, then copy only its
        // finite ECEF velocity together with the raw EpochSeed C0/D values
        // through the strict source-keyed adapter.  No position difference,
        // zero/held D, or persisted coordinate can enter this path.
        std::vector<libgnss::Vector3d> ignored_velocity_enu;
        std::string direct_wls_error;
        direct_wls_ephemeral_ok = validateDirectDopplerWlsHandoff(
            problem, ignored_velocity_enu, imu_report, direct_wls_error);
        if (direct_wls_ephemeral_ok) {
            std::vector<double> raw_drift_mps;
            raw_drift_mps.reserve(android_gnss.observations.epochs.size());
            for (const auto& raw_epoch : android_gnss.observations.epochs) {
                raw_drift_mps.push_back(raw_epoch.receiver_clock_drift_mps);
            }
            direct_wls_ephemeral_ok = validateDirectWlsEphemeralMainSeed(
                problem, android_raw_epoch_times,
                android_gnss.epoch_utc_time_millis, raw_drift_mps, imu_report,
                direct_wls_ephemeral_velocity_ecef_mps,
                direct_wls_ephemeral_d_handoff_mps,
                direct_wls_ephemeral_c_handoff_m, direct_wls_error);
            if (direct_wls_ephemeral_ok) {
                double raw_lat = 0.0;
                double raw_lon = 0.0;
                double raw_height = 0.0;
                libgnss::ecef2geodetic(problem.epochs.front().position_ecef,
                                       raw_lat, raw_lon, raw_height);
                direct_wls_ephemeral_velocity_enu.resize(
                    direct_wls_ephemeral_velocity_ecef_mps.size());
                for (std::size_t i = 0;
                     i < direct_wls_ephemeral_velocity_ecef_mps.size(); ++i) {
                    direct_wls_ephemeral_velocity_enu[i] = libgnss::ecef2enu(
                        direct_wls_ephemeral_velocity_ecef_mps[i], raw_lat,
                        raw_lon);
                    if (!direct_wls_ephemeral_velocity_enu[i].allFinite()) {
                        direct_wls_error =
                            "direct-WLS ECEF-to-ENU velocity conversion failed";
                        direct_wls_ephemeral_ok = false;
                        break;
                    }
                }
            }
        }
        imu_report.gnss_first_positions_clocks_copied = 0U;
        if (!direct_wls_ephemeral_ok) {
            imu_report.failure = direct_wls_error.empty()
                                     ? "direct-WLS ephemeral C7/D seed failed"
                                     : direct_wls_error;
            std::cerr << "direct-WLS ephemeral C7/D seed failed closed: "
                      << imu_report.failure << "\n";
            return 1;
        }
        problem.native_direct_wls_ephemeral_velocity_ecef_mps =
            direct_wls_ephemeral_velocity_ecef_mps;
        problem.native_direct_wls_ephemeral_d_handoff_mps =
            direct_wls_ephemeral_d_handoff_mps;
        problem.native_direct_wls_ephemeral_c_handoff_m =
            direct_wls_ephemeral_c_handoff_m;
    } else if (android_raw && options.native_direct_doppler_wls_handoff) {
        // Phase40 deliberately bypasses the GNSS-first optimizer.  The
        // problem builder has already solved the raw-Doppler WLS estimates
        // from the same satellite state and raw SPP seed used by the graph.
        std::string direct_wls_error;
        direct_doppler_wls_ok = validateDirectDopplerWlsHandoff(
            problem, direct_doppler_wls_velocities_enu, imu_report,
            direct_wls_error);
        imu_report.gnss_first_positions_clocks_copied = 0U;
        if (!direct_doppler_wls_ok) {
            imu_report.failure = direct_wls_error.empty()
                                     ? "direct Doppler WLS handoff failed"
                                     : direct_wls_error;
            std::cerr << "direct Doppler WLS handoff failed closed: "
                      << imu_report.failure << "\n";
            return 1;
        }
    } else if (android_raw) {
        // Upstream run_fgo.m performs a GNSS-only pass before the IMU pass.
        // Keep that handoff in memory: no device-WLS, result file, or truth
        // position can enter the Android initialization path.
        imu_report.gnss_first_attempted = true;
        libgnss::FGOProcessor::FGOConfig gnss_first_config = config;
        // Keep the GNSS-first initializer identical to the baseline.
        gnss_first_config.use_native_tdcp_frequency_residual_states = false;
        gnss_first_config.native_tdcp_frequency_residual_prior_sigma_m = 0.0;
        gnss_first_config.use_main_pseudorange_cauchy_loss=false;
        gnss_first_config.use_native_epoch_heading_attitude_seeds = false;
        gnss_first_config.omit_native_first_imu_bias_prior = false;
        gnss_first_config.omit_native_first_imu_velocity_prior = false;
        gnss_first_config.use_native_relative_height_pairs = false;
        gnss_first_config.use_native_batch_nhc = false;
        gnss_first_config.use_native_joint_ionosphere = false;
        gnss_first_config.retain_native_pseudorange_remasking_pool = false;
        gnss_first_config.backend = libgnss::FGOBackend::GTSAM;
        gnss_first_config.use_imu = false;
        gnss_first_config.use_pose3_state = false;
        // Phase96 is scoped to the main C0/D graph.  GNSS-first remains the
        // existing staging solve; its aggregate Phase93 telemetry is copied
        // separately.  When Phase143 is selected, its own native termination
        // sidecar may request the existing SUMMARY verbosity surface for this
        // stage, without changing any optimizer parameter.
        gnss_first_config.use_native_source_clock_c0d_phase96_main_diagnostics =
            false;
        // Phase98 is scoped to the existing main solve; do not instrument the
        // GNSS-first staging optimizer or change its diagnostic surface.
        gnss_first_config.use_native_source_clock_c0d_phase98_solver_rank_diagnostic =
            false;
        // Phase99 is scoped exclusively to the Pose3+IMU main solve.  Keep
        // the GNSS-first Point3/velocity staging optimizer on its historical
        // multifrontal Cholesky branch even when the caller selected QR for
        // the later main graph.
        gnss_first_config
            .use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
            false;
        const bool phase93_meter_state_handoff =
            options.native_source_clock_c0d_gnss_first_meter_state_handoff;
        const bool phase171_imu_main =
            options.native_phase171_raw_p_no_doppler_imu_main;
        const bool phase171_ecef_doppler =
            options.native_phase171_raw_p_ecef_doppler_gnss_first;
        if (phase93_meter_state_handoff && !phase171_imu_main) {
            // Phase93 deliberately carries the source-meter C0/D graph into
            // this Point3+velocity initializer.  Its D_i values are seeded
            // from the retained EpochSeed raw drift and its optimized C_i/D_i
            // states are exported for the same-run main handoff below.
            gnss_first_config.use_native_source_clock_c0d_factor = true;
            gnss_first_config.native_source_clock_c0d_phone =
                phoneFromDatasetId(options.dataset_id);
            gnss_first_config.use_native_source_clock_c0d_meter_state_parity = true;
            gnss_first_config.use_native_source_clock_c0d_active_solve_diagnostic =
                true;
            gnss_first_config.use_native_source_clock_c0d_raw_drift_d_initializer =
                true;
            gnss_first_config.use_native_source_clock_c0d_epoch_vector_parity =
                options.native_source_clock_c0d_epoch_vector_parity;
            if (options.native_source_clock_c0d_epoch_vector_parity) {
                gnss_first_config.use_inter_system_biases = false;
            }
        } else if (phase171_imu_main) {
            // Phase171 runs the existing same-run raw-P GNSS-first graph in
            // this in-memory staging copy.  Its result is handed directly to
            // the unchanged Pose3+IMU path below; no persisted seed and no
            // second SPP/FGO solve is permitted.
            gnss_first_config.backend = libgnss::FGOBackend::GTSAM;
            gnss_first_config.use_imu = false;
            gnss_first_config.use_pose3_state = false;
            gnss_first_config.use_spp_seed = false;
            gnss_first_config.use_multi_constellation = true;
            gnss_first_config.use_multi_frequency_double_difference = true;
            gnss_first_config.use_velocity_states = true;
            gnss_first_config.use_motion_factors = true;
            gnss_first_config.use_position_motion_factors = false;
            gnss_first_config.use_velocity_motion_factors = true;
            gnss_first_config.use_clock_motion_factors = true;
            gnss_first_config.use_native_source_clock_c0d_factor = true;
            gnss_first_config.native_source_clock_c0d_phone =
                phoneFromDatasetId(options.dataset_id);
            gnss_first_config.use_native_source_clock_c0d_meter_state_parity =
                true;
            gnss_first_config.use_native_source_clock_c0d_active_solve_diagnostic =
                true;
            gnss_first_config.use_native_source_clock_c0d_raw_drift_d_initializer =
                false;
            // The staging graph is the dedicated raw-P Point3/V/C7/D path;
            // its own C/D export does not use the legacy Phase93 handoff
            // admission flag (which, for a no-IMU graph, requires the raw-D
            // initializer).  The main Pose3+IMU config retains that flag
            // below and consumes the validated in-memory C/D handoff.
            gnss_first_config.use_native_source_clock_c0d_gnss_first_meter_state_handoff =
                false;
            gnss_first_config.use_native_source_clock_c0d_epoch_vector_parity =
                true;
            gnss_first_config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
                false;
            gnss_first_config.use_native_raw_p_no_doppler_graph =
                !phase171_ecef_doppler;
            gnss_first_config.use_native_raw_p_ecef_doppler_gnss_first =
                phase171_ecef_doppler;
            gnss_first_config.use_native_phase167_raw_p_no_doppler_lm_termination_budget =
                true;
            gnss_first_config.use_native_phase143_official_main_lm_termination_budget =
                false;
            gnss_first_config.use_native_phase171_raw_p_no_doppler_imu_main =
                false;
            gnss_first_config.use_native_phase205_source_count_bias_density = false;
            gnss_first_config.use_native_phase209_source_separate_imu_factors = false;
            gnss_first_config.use_native_phase213_main_doppler = false;
            gnss_first_config.use_native_phase217_main_pose3_motion = false;
            // The source-inclusive schedule is a main Pose3/IMU experiment;
            // the preceding Point3/V/C7/D staging graph keeps its legacy
            // schedule and never receives this selector.
            gnss_first_config
                .use_native_phase201_source_inclusive_forward_imu_schedule =
                false;
            gnss_first_config.use_inter_system_biases = false;
            gnss_first_config.use_receiver_signal_bias_states = false;
            gnss_first_config.use_residual_ionosphere_states = false;
            gnss_first_config.use_undifferenced_doppler_factors =
                phase171_ecef_doppler;
            gnss_first_config.use_corrected_undifferenced_doppler_factors =
                phase171_ecef_doppler;
            if (phase171_ecef_doppler) {
                // The ECEF-D lane requires direct source-quality admission;
                // selector-off inherits the already-frozen Phase171 setting.
                gnss_first_config.use_upstream_observable_quality = true;
            }
            gnss_first_config.use_single_difference_doppler_factors = false;
            gnss_first_config.use_single_difference_tdcp_factors = false;
            gnss_first_config.use_carrier_phase_factors = false;
            gnss_first_config.use_double_difference_factors = false;
            gnss_first_config.use_tdcp_factors = true;
            gnss_first_config.use_doppler_velocity_wls_initialization = false;
            gnss_first_config.max_iterations = 1000;
        } else {
            // The historical GNSS-first handoff is a separate
            // Point3/velocity initializer, not the frozen Pose3+IMU C0/D
            // candidate graph.  Keep the legacy selector-off behavior exact.
            gnss_first_config.use_native_source_clock_c0d_factor = false;
            gnss_first_config.native_source_clock_c0d_phone.clear();
            gnss_first_config.use_native_source_clock_c0d_meter_state_parity = false;
            gnss_first_config.use_native_source_clock_c0d_active_solve_diagnostic =
                false;
            gnss_first_config.use_native_source_clock_c0d_raw_drift_d_initializer =
                false;
            gnss_first_config.use_native_source_clock_c0d_epoch_vector_parity =
                false;
        }
        gnss_first_config.use_velocity_states = true;
        // The dedicated Phase171 staging graph is the complete Phase164
        // Point3/V/C7/D recipe and therefore must retain its velocity-motion
        // factors through the common post-branch defaults.  Legacy GNSS-first
        // paths keep their historical false setting.
        gnss_first_config.use_velocity_motion_factors = phase171_imu_main;
        gnss_first_config.use_doppler_velocity_wls_initialization = false;
        if (options.native_gnss_first_velocity_only_handoff) {
            // The candidate's GNSS-first velocity states are initialized from
            // the same raw receiver-only Doppler rows with the existing
            // truth-free bounded WLS physical gate.  Disable edge holds: any
            // permitted bounded interpolation remains diagnostics-only, and
            // the handed-off values are still the final optimizer states.
            gnss_first_config.use_doppler_velocity_wls_initialization = true;
            gnss_first_config.doppler_velocity_wls_edge_hold_max_s = 0.0;
        }
        // Upstream fgo_gnss.m permits up to 1000 LM iterations.  The raw
        // GNSS-first pass is a separate initializer, so use that published
        // bound without changing the frozen 12-iteration IMU pass.
        gnss_first_config.max_iterations = 1000;
        gnss_first_config.allow_native_raw_p_sparse_epochs = options.native_sparse_p_staging;
        const libgnss::FGOProcessor gnss_first_processor(gnss_first_config);
        libgnss::FGOProcessor::FGOProblem gnss_first_problem = problem;
        if (phase171_ecef_doppler) {
            // The main processor intentionally built an empty-generic-D
            // problem.  Build a temporary staging view with the same mutated
            // raw epochs/nav and SPP disabled so corrected raw-D rows are
            // selected by the native quality/mask path.  Copy only those D
            // rows into the existing problem: P, TDCP, epoch seeds, and all
            // their existing retained identities remain authoritative.
            auto staged_problem =
                gnss_first_processor.buildPseudorangeProblem(epochs, nav);
            if (staged_problem.epochs.size() < 2U ||
                staged_problem.undifferenced_doppler_factors.empty()) {
                imu_report.failure =
                    "Phase171 ECEF raw-P+D staging retained no usable D rows";
                std::cerr << imu_report.failure << "\n";
                return 1;
            }
            std::string staging_identity_failure;
            std::vector<libgnss::FGOProcessor::UndifferencedDopplerFactor>
                staged_doppler;
            if (!libgnss::raw_p_ecef_doppler::remapDopplerFactors(
                    gnss_first_problem.epochs, staged_problem.epochs,
                    staged_problem.undifferenced_doppler_factors,
                    staged_doppler, staging_identity_failure)) {
                imu_report.failure =
                    "Phase171 ECEF raw-P+D staging transfer failed: " +
                    staging_identity_failure;
                std::cerr << imu_report.failure << "\n";
                return 1;
            }
            gnss_first_problem.undifferenced_doppler_factors =
                std::move(staged_doppler);
            if (options.native_phase213_main_doppler) {
                std::string main_transfer_failure;
                if (!libgnss::raw_p_ecef_doppler::remapDopplerFactors(
                        problem.epochs, gnss_first_problem.epochs,
                        gnss_first_problem.undifferenced_doppler_factors,
                        problem.native_phase213_main_doppler_rows, main_transfer_failure)) {
                    std::cerr << "Phase213 main Doppler transfer failed: "
                              << main_transfer_failure << "\n";
                    return 1;
                }
            }
        } else if (options.native_gnss_first_velocity_only_handoff) {
            gnss_first_problem =
                gnss_first_processor.buildPseudorangeProblem(epochs, nav);
            imu_report.gnss_first_velocity_initializer_edge_hold_max_s =
                gnss_first_config.doppler_velocity_wls_edge_hold_max_s;
            for (const auto& estimate : gnss_first_problem.doppler_velocity_wls_estimates) {
                if (estimate.valid) {
                    if (estimate.propagated) {
                        ++imu_report.gnss_first_velocity_initializer_propagated_count;
                        if (estimate.reason == "bounded-edge-hold") {
                            ++imu_report.gnss_first_velocity_initializer_edge_hold_count;
                        }
                    } else {
                        ++imu_report.gnss_first_velocity_initializer_direct_count;
                    }
                }
            }
        }
        if (options.native_phase127_glonass_channel_provenance &&
            !gnss_first_problem.diagnostics.phase127_failure.empty() &&
            (!options.native_phase129_glonass_local_miss_mask ||
             !gnss_first_problem.diagnostics.phase129_configuration_valid)) {
            std::cerr << "Phase127 GLONASS provenance failed closed in the "
                         "GNSS-first stage: "
                      << gnss_first_problem.diagnostics.phase127_failure << "\n";
            return 1;
        }
        if (phase94_diagnostics.enabled) {
            phase94_diagnostics.gnss_first_preflight =
                makePhase94C0DPreflight(gnss_first_problem,
                                         gnss_first_config);
        }
        libgnss::FGOProcessor::FGOResult gnss_first_result;
        if (phase94_diagnostics.enabled) {
            phase94_diagnostics.gnss_first.attempted = true;
            try {
                gnss_first_result =
                    gnss_first_processor.optimizeProblem(gnss_first_problem);
                phase94_diagnostics.gnss_first.result_returned = true;
            } catch (const std::invalid_argument& error) {
                const std::string message = error.what();
                const std::string known_guard =
                    "native source ClockFactor_CCDD C0/D requires direct observable "
                    "quality, no PDC bridge, and an approved GTSAM batch path";
                const bool known = message == known_guard &&
                                   !phase94_diagnostics
                                        .gnss_first_preflight
                                        .guard_failed_predicates.empty();
                phase94_diagnostics.gnss_first.unknown_guard_reason = !known;
                phase94_diagnostics.gnss_first.failure = message;
                phase94_diagnostics.gnss_first.terminal_branch =
                    known ? "source-c0d-backend-admission-guard"
                          : "unknown-backend-exception";
                phase94_diagnostics.gnss_first_preflight.guard_rejected = true;
                phase94RecordFailure(
                    phase94_diagnostics, "gnss-first-optimize",
                    known ? "native C0/D backend admission guard rejected"
                          : "unknown GNSS-first backend exception; fail closed",
                    "std::invalid_argument", message);
                if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                    std::cerr << "failed to publish Phase94 diagnostics\n";
                }
                std::cerr << "Phase94 GNSS-first backend exception: " << message
                          << "\n";
                return 1;
            } catch (const std::exception& error) {
                phase94_diagnostics.gnss_first.unknown_guard_reason = true;
                phase94_diagnostics.gnss_first.failure = error.what();
                phase94_diagnostics.gnss_first.terminal_branch =
                    "unknown-backend-exception";
                phase94RecordFailure(
                    phase94_diagnostics, "gnss-first-optimize",
                    "unknown GNSS-first backend exception; fail closed",
                    "std::exception", error.what());
                if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                    std::cerr << "failed to publish Phase94 diagnostics\n";
                }
                std::cerr << "Phase94 GNSS-first backend exception: "
                          << error.what() << "\n";
                return 1;
            } catch (...) {
                phase94_diagnostics.gnss_first.unknown_guard_reason = true;
                phase94_diagnostics.gnss_first.failure =
                    "non-standard exception";
                phase94_diagnostics.gnss_first.terminal_branch =
                    "unknown-backend-exception";
                phase94RecordFailure(
                    phase94_diagnostics, "gnss-first-optimize",
                    "unknown GNSS-first backend exception; fail closed",
                    "unknown", "non-standard exception");
                if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                    std::cerr << "failed to publish Phase94 diagnostics\n";
                }
                std::cerr << "Phase94 GNSS-first backend exception: unknown\n";
                return 1;
            }
        } else {
            gnss_first_result =
                gnss_first_processor.optimizeProblem(gnss_first_problem);
        }
        imu_report.gnss_first_converged = gnss_first_result.diagnostics.converged;
        imu_report.gnss_first_epochs = gnss_first_result.solution.solutions.size();
        imu_report.gnss_first_doppler_factors =
            gnss_first_problem.undifferenced_doppler_factors.size();
        imu_report.gnss_first_ecef_doppler_stage_enabled =
            gnss_first_result.diagnostics
                .native_raw_p_ecef_doppler_gnss_first_enabled;
        imu_report.gnss_first_doppler_factors_inserted =
            gnss_first_result.diagnostics.undifferenced_doppler_factors_inserted;
        imu_report.gnss_first_tdcp_only_affine_factors_inserted =
            gnss_first_result.diagnostics.tdcp_only_affine_factors_inserted;
        imu_report.gnss_first_velocity_states =
            gnss_first_result.epoch_velocities_ecef_mps.size();
        imu_report.gnss_first_iterations = gnss_first_result.diagnostics.iterations;
        imu_report.gnss_first_initial_cost = gnss_first_result.diagnostics.initial_cost;
        imu_report.gnss_first_final_cost = gnss_first_result.diagnostics.final_cost;
        imu_report.gnss_first_c0d_factor_count =
            gnss_first_result.diagnostics.native_source_clock_c0d_factor_count;
        imu_report.gnss_first_c0d_accepted_outer_iterations =
            gnss_first_result.diagnostics
                .native_source_clock_c0d_accepted_outer_iterations;
        imu_report.gnss_first_c0d_active_solve_finite_costs =
            gnss_first_result.diagnostics
                .native_source_clock_c0d_active_solve_finite_costs;
        if (options.native_phase143_official_main_lm_termination_budget ||
            options.native_phase171_raw_p_no_doppler_imu_main) {
            // Copy the native stage report once, without deriving it from the
            // generic iterations/converged fields or from output rows.
            imu_report.gnss_first_phase143_termination =
                gnss_first_result.diagnostics.native_phase143_termination;
        }
        std::string gnss_first_error;
        bool phase91_handoff_ok = true;
        if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
            phase93_meter_state_handoff) {
            std::vector<double> raw_drift_mps;
            // Keep the complete adapter source sequence.  `epochs` may be a
            // retained/limited problem vector (and may have rows removed by
            // an upstream filter); indexing its drift values by position
            // would silently mis-seed D_i after the first omission.  The
            // retained EpochSeed source index/UTC/GPST key is validated below
            // against this immutable raw table before the main graph starts.
            raw_drift_mps.reserve(android_gnss.observations.epochs.size());
            for (const auto& raw_epoch : android_gnss.observations.epochs) {
                raw_drift_mps.push_back(raw_epoch.receiver_clock_drift_mps);
            }
            phase91_handoff_ok = validatePhase91GnssFirstHandoff(
                problem, gnss_first_problem, gnss_first_result,
                android_raw_epoch_times, android_gnss.epoch_utc_time_millis,
                raw_drift_mps, imu_report, gnss_first_error,
                !phase171_imu_main);
            if (phase91_handoff_ok && phase93_meter_state_handoff) {
                const auto& c0d_diagnostics =
                    gnss_first_result.diagnostics;
                if (!c0d_diagnostics.native_source_clock_c0d_factor_enabled ||
                    !c0d_diagnostics
                         .native_source_clock_c0d_meter_state_parity_enabled ||
                    c0d_diagnostics.native_source_clock_c0d_factor_count == 0U ||
                    !c0d_diagnostics.native_source_clock_c0d_active_solve_finite_costs ||
                    c0d_diagnostics.native_source_clock_c0d_accepted_outer_iterations ==
                        0U ||
                    !std::isfinite(c0d_diagnostics.initial_cost) ||
                    !std::isfinite(c0d_diagnostics.final_cost) ||
                    c0d_diagnostics.final_cost >= c0d_diagnostics.initial_cost) {
                    gnss_first_error =
                        "Phase93 GNSS-first C0/D solve did not make a finite strict-cost-progress step";
                    phase91_handoff_ok = false;
                }
            }
            if (phase91_handoff_ok && phase93_meter_state_handoff) {
                phase91_handoff_ok = validatePhase93OptimizedDExport(
                    problem, gnss_first_result, imu_report, gnss_first_error);
                if (phase91_handoff_ok) {
                    gnss_first_clock_drift_mps =
                        gnss_first_result.epoch_clock_drift_mps;
                }
            }
            if (phase91_handoff_ok && phase93_meter_state_handoff &&
                options.native_source_clock_c0d_epoch_vector_parity) {
                phase91_handoff_ok = validatePhase101OptimizedCExport(
                    problem, gnss_first_problem, gnss_first_result,
                    imu_report, gnss_first_error);
                if (phase91_handoff_ok) {
                    gnss_first_clock_bias_components_m =
                        gnss_first_result.epoch_clock_bias_components_m;
                }
            }
        }
        if (phase94_diagnostics.enabled) {
            phase94_diagnostics.gnss_first = makePhase94GnssFirstTelemetry(
                gnss_first_problem, gnss_first_result, imu_report);
            // The call above returned normally, so preserve the attempted
            // marker even when later exact-key/progress validation fails.
            phase94_diagnostics.gnss_first.attempted = true;
            phase94_diagnostics.gnss_first.result_returned = true;
        }
        gnss_first_ok = !gnss_first_result.solution.isEmpty() &&
                        gnss_first_result.diagnostics.converged &&
                        phase91_handoff_ok &&
                        (options.native_gnss_first_velocity_only_handoff
                             ? validateGnssFirstVelocityOnlyHandoff(
                                   problem, gnss_first_result,
                                   gnss_first_velocities_enu, imu_report,
                                   gnss_first_error)
                             : deriveGnssFirstVelocities(
                                   problem, gnss_first_result,
                                   gnss_first_velocities_enu, gnss_first_error));
        if (!gnss_first_ok) {
            imu_report.gnss_first_failure = gnss_first_error.empty()
                                                 ? "GNSS-first optimizer did not converge"
                                                 : gnss_first_error;
            std::cerr << "GNSS-first initialization unavailable: "
                      << imu_report.gnss_first_failure
                      << "; main_epochs=" << problem.epochs.size()
                      << "; gnss_first_epochs=" << imu_report.phase91_gnss_first_epoch_count
                      << "; solution_epochs=" << imu_report.phase91_solution_epoch_count
                      << "; raw_epochs=" << imu_report.phase91_raw_epoch_count
                      << "; gnss_first_iterations=" << imu_report.gnss_first_iterations
                      << "; gnss_first_c0d_factors="
                      << gnss_first_result.diagnostics.native_source_clock_c0d_factor_count
                      << "; gnss_first_termination_attempted="
                      << gnss_first_result.diagnostics.native_phase143_termination.attempted
                      << "\n";
            if (phase94_diagnostics.enabled) {
                phase94RecordFailure(phase94_diagnostics, "exact-key-handoff",
                                     imu_report.gnss_first_failure);
                phase94_diagnostics.gnss_first.failure =
                    imu_report.gnss_first_failure;
                if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                    std::cerr << "failed to publish Phase94 diagnostics\n";
                }
                std::cerr << "Phase94 GNSS-first candidate failed closed\n";
                return 1;
            }
            if (options.native_gnss_first_velocity_only_handoff ||
                options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
                phase93_meter_state_handoff) {
                std::cerr << "native GNSS-first candidate failed closed\n";
                return 1;
            }
        } else {
            if (options.native_gnss_first_velocity_only_handoff) {
                // Position and receiver-clock states intentionally remain the
                // original raw SPP seeds.  Only the independently optimized
                // Doppler velocity sequence reaches buildImuInput below.
                imu_report.gnss_first_positions_clocks_copied = 0U;
            } else {
                // Seed the following in-memory problem with the GNSS-only
                // result, including its receiver clock in the internal metre
                // convention. This is the historical handoff and is retained
                // byte-for-byte when the Phase39 flag is off.
                const auto& gnss_solutions = gnss_first_result.solution.solutions;
                if (options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
                    phase93_meter_state_handoff) {
                    imu_report.gnss_first_positions_clocks_copied =
                        gnss_solutions.size();
                }
                for (std::size_t i = 0; i < gnss_solutions.size(); ++i) {
                    problem.epochs[i].position_ecef = gnss_solutions[i].position_ecef;
                    if (std::isfinite(gnss_solutions[i].receiver_clock_bias)) {
                        problem.epochs[i].receiver_clock_bias_m =
                            gnss_solutions[i].receiver_clock_bias *
                            libgnss::constants::SPEED_OF_LIGHT;
                        problem.epochs[i].receiver_clock_bias_is_meters = true;
                    }
                }
                if (phase93_meter_state_handoff) {
                    // Exact retained-key alignment was validated above, so
                    // vector index i is the same source epoch in both staged
                    // and main problems.  The main backend rejects an empty,
                    // short, or nonfinite vector rather than selecting a
                    // forbidden fallback.
                    problem.native_source_clock_c0d_gnss_first_d_handoff_mps =
                        gnss_first_clock_drift_mps;
                    if (options.native_source_clock_c0d_epoch_vector_parity) {
                        problem.native_source_clock_c0d_gnss_first_c_handoff_m =
                            gnss_first_clock_bias_components_m;
                    }
                }
            }
            if (options.native_phase104_stage_main_accuracy_attribution) {
                std::string stage_export_error;
                if (!exportPhase104StageEcef(
                        options, problem, gnss_first_result,
                        android_gnss.epoch_utc_time_millis,
                        phase104_stage_report, stage_export_error)) {
                    std::cerr << "Phase104 stage export failed closed: "
                              << stage_export_error << "\n";
                    return 1;
                }
            }
        }
    }
    const std::string imu_path = android_raw ? options.android_imu_path : options.imu_path;
    std::vector<libgnss::AndroidGnssTimeAnchor> gnss_time_anchors;
    libgnss::AndroidGnssUtcGpsMapping android_utc_gps_mapping;
    if (android_raw) {
        if (options.android_utc_wall_clock_fallback) {
            // Prefer the authoritative monotonic Android anchor whenever it
            // is present.  The opt-in wall-clock mapping is a portability
            // fallback for the mi8-style all-blank elapsed column, never a
            // second competing clock selected for a route that already has
            // valid elapsed anchors.
            imu_report.android_gnss_anchor_load =
                libgnss::loadAndroidGnssTimeAnchors(options.android_gnss_path,
                                                    gnss_time_anchors);
            if (!imu_report.android_gnss_anchor_load.ok) {
                imu_report.android_gnss_utc_mapping_load =
                    libgnss::loadAndroidGnssUtcGpsMapping(
                        options.android_gnss_path, android_utc_gps_mapping);
                if (!imu_report.android_gnss_utc_mapping_load.ok) {
                    std::cerr << "failed to load raw GNSS UTC/GPS mapping: "
                              << imu_report.android_gnss_utc_mapping_load.error << "\n";
                    if (phase94_diagnostics.enabled) {
                        phase94RecordFailure(
                            phase94_diagnostics, "gnss-utc-mapping-load",
                            imu_report.android_gnss_utc_mapping_load.error);
                        if (!writePhase94StageDiagnostics(
                                options, phase94_diagnostics)) {
                            std::cerr << "failed to publish Phase94 diagnostics\n";
                        }
                    }
                    return 1;
                }
            }
        } else {
            imu_report.android_gnss_anchor_load =
                libgnss::loadAndroidGnssTimeAnchors(options.android_gnss_path,
                                                    gnss_time_anchors);
            if (!imu_report.android_gnss_anchor_load.ok) {
                std::cerr << "failed to load GNSS elapsed-time anchors: "
                          << imu_report.android_gnss_anchor_load.error << "\n";
                if (phase94_diagnostics.enabled) {
                    phase94RecordFailure(
                        phase94_diagnostics, "gnss-elapsed-anchor-load",
                        imu_report.android_gnss_anchor_load.error);
                    if (!writePhase94StageDiagnostics(
                            options, phase94_diagnostics)) {
                        std::cerr << "failed to publish Phase94 diagnostics\n";
                    }
                }
                return 1;
            }
        }
    }
    if (options.native_pseudorange_remasking) {
        if (!gnss_first_ok ||
            !imu_report.phase91_raw_drift_d_initializer_alignment_valid ||
            imu_report.gnss_first_positions_clocks_copied != problem.epochs.size()) {
            std::cerr << "Pseudorange re-masking requires validated same-run position/clock handoff\n";
            return 1;
        }
        try {
            const auto selected = libgnss::pseudorange_remasking::select(problem);
            if (selected.pool_indices.empty())
                throw std::invalid_argument("re-masking retained no P rows");
            std::vector<libgnss::FGOProcessor::PseudorangeFactor> rows;
            rows.reserve(selected.pool_indices.size());
            for (const auto i : selected.pool_indices)
                rows.push_back(problem.native_pseudorange_remasking_pool.at(i));
            imu_report.pseudorange_remasking_pool = problem.native_pseudorange_remasking_pool.size();
            imu_report.pseudorange_remasking_old = problem.pseudorange_factors.size();
            imu_report.pseudorange_remasking_new = rows.size();
            imu_report.pseudorange_remasking_recovered = selected.recovered;
            imu_report.pseudorange_remasking_removed = selected.removed;
            imu_report.pseudorange_remasking_unchanged = selected.unchanged;
            problem.pseudorange_factors.swap(rows);
            imu_report.pseudorange_remasking_applied = true;
        } catch (const std::exception& error) {
            std::cerr << "Pseudorange re-masking failed closed: " << error.what() << '\n';
            return 1;
        }
    }
    const std::vector<libgnss::Vector3d>* velocity_handoff = nullptr;
    if (android_raw) {
        if (options.native_direct_wls_ephemeral_c7d_main_seed) {
            velocity_handoff = direct_wls_ephemeral_ok
                                   ? &direct_wls_ephemeral_velocity_enu
                                   : nullptr;
        } else if (options.native_direct_doppler_wls_handoff) {
            velocity_handoff = direct_doppler_wls_ok
                                   ? &direct_doppler_wls_velocities_enu
                                   : nullptr;
        } else if (gnss_first_ok) {
            velocity_handoff = &gnss_first_velocities_enu;
        }
    }
    bool use_imu = buildImuInput(imu_path, problem, imu_report, android_raw,
                                 gnss_time_anchors, velocity_handoff,
                                 options.android_utc_wall_clock_fallback
                                     ? &android_utc_gps_mapping
                                     : nullptr,
                                 options.native_direct_wls_ephemeral_c7d_main_seed,
                                 options.native_phase194_source_utc_fallback_imu_noise,
                                 options.native_phase197_source_utc_fallback_imu_offset,
                                 options.native_epoch_heading_attitude_seeds,
                                 options.native_stationary_gyro_initializer);
    if (options.native_main_code_edge_readmission) {
        std::set<std::tuple<std::size_t, libgnss::SatelliteId, libgnss::SignalType>> keys;
        for (const auto& factor : problem.pseudorange_factors)
            keys.emplace(factor.epoch_index, factor.satellite, factor.signal);
        for (const auto& factor : problem.native_code_edge_readmission_pool) {
            if (factor.epoch_index >= problem.epochs.size() ||
                !factor.satellite_position_ecef.allFinite() ||
                !std::isfinite(factor.corrected_pseudorange_m) ||
                !std::isfinite(factor.sigma_m) || factor.sigma_m <= 0.0 ||
                !keys.emplace(factor.epoch_index, factor.satellite, factor.signal).second) {
                std::cerr << "Main code readmission invalid or duplicate factor\n";
                return 1;
            }
        }
        problem.pseudorange_factors.insert(problem.pseudorange_factors.end(),
            problem.native_code_edge_readmission_pool.begin(),
            problem.native_code_edge_readmission_pool.end());
        std::cerr << "[native-main-code-edge-readmission] added="
                  << problem.native_code_edge_readmission_pool.size()
                  << " gnss_first_changed=0 median_changed=0\n";
    }
    if (options.native_main_code_uncertainty_floor) {
        // GNSS-first and all main row selection have already completed.
        // Only main P sigmas change; code values, keys and seeds stay fixed.
        std::size_t changed = 0;
        for (auto& factor : problem.pseudorange_factors) {
            const double floor = factor.android_sv_time_uncertainty_floor_m;
            if (!std::isfinite(factor.sigma_m) || factor.sigma_m <= 0.0) {
                std::cerr << "Main code floor encountered invalid sigma\n";
                return 1;
            }
            if (factor.android_sv_time_uncertainty_available &&
                std::isfinite(floor) && floor > factor.sigma_m) {
                factor.sigma_m = floor;
                factor.android_sv_time_uncertainty_floor_applied = true;
                ++changed;
            }
        }
        std::cerr << "[native-main-code-uncertainty-floor] retained="
                  << problem.pseudorange_factors.size() << " changed=" << changed
                  << " gnss_first_changed=0 selection_changed=0\n";
    }
    if (phase94_diagnostics.enabled) {
        phase94_diagnostics.main_preflight =
            makePhase94C0DPreflight(problem, config);
        phase94_diagnostics.main.problem_epoch_count = problem.epochs.size();
        phase94_diagnostics.main.exact_retained_key_alignment =
            imu_report.phase91_raw_drift_d_initializer_alignment_valid;
    }
    bool fallback = false;
    libgnss::FGOProcessor::FGOResult result;
    if (options.native_phase171_raw_p_no_doppler_imu_main &&
        !config.use_native_android_sv_time_uncertainty_sigma_floor &&
        !options.native_main_code_uncertainty_floor) {
        // Diagnostic only, after final main-stage row selection. No sigma
        // changes; count the literal source uncertainty floor's reach.
        std::size_t available = 0, would_increase = 0, invalid_sigma = 0;
        double max_ratio = 0.0;
        for (const auto& factor : problem.pseudorange_factors) {
            if (!std::isfinite(factor.sigma_m) || factor.sigma_m <= 0.0) {
                ++invalid_sigma;
                continue;
            }
            const double floor = factor.android_sv_time_uncertainty_floor_m;
            if (!factor.android_sv_time_uncertainty_available ||
                !std::isfinite(floor) || floor <= 0.0) continue;
            ++available;
            if (floor > factor.sigma_m) ++would_increase;
            max_ratio = std::max(max_ratio, floor / factor.sigma_m);
        }
        std::cerr << "[native-code-uncertainty-monitor] retained="
                  << problem.pseudorange_factors.size()
                  << " available=" << available
                  << " would_increase=" << would_increase
                  << " invalid_sigma=" << invalid_sigma
                  << " max_floor_to_sigma_ratio=" << max_ratio
                  << " weights_changed=0\n";
    }
    if (options.native_phase171_raw_p_no_doppler_imu_main) {
        // Check endpoint metadata directly on retained TDCP, including edges
        // whose code rows were rejected. No nearest-time or code-row fallback.
        std::set<std::tuple<std::size_t, libgnss::SatelliteId, libgnss::SignalType>> code_keys;
        for (const auto& f : problem.pseudorange_factors)
            code_keys.emplace(f.epoch_index, f.satellite, f.signal);
        std::size_t endpoint_valid = 0, endpoint_invalid = 0, missing_code_edges = 0;
        double endpoint_min = std::numeric_limits<double>::infinity(), endpoint_max = 0.;
        for (const auto& f : problem.tdcp_factors) {
            const double a = f.previous_residual_ionosphere_coefficient;
            const double b = f.current_residual_ionosphere_coefficient;
            if (!std::isfinite(a) || !std::isfinite(b) || a <= 0 || b <= 0) {
                ++endpoint_invalid; continue;
            }
            ++endpoint_valid;
            endpoint_min = std::min(endpoint_min, std::min(a,b));
            endpoint_max = std::max(endpoint_max, std::max(a,b));
            missing_code_edges += !code_keys.count({f.previous_epoch_index,f.satellite,f.signal}) ||
                                  !code_keys.count({f.current_epoch_index,f.satellite,f.signal});
        }
        std::cerr << "[native-tdcp-ionosphere-endpoints] valid=" << endpoint_valid
                  << " invalid=" << endpoint_invalid << " missing_code_edges=" << missing_code_edges
                  << " min=" << (endpoint_valid ? endpoint_min : 0.) << " max=" << endpoint_max
                  << " factors_changed=0\n";
        // Read-only local Gaussian information. Stored SPP-elevation ionosphere
        // coefficients, but position Jacobians at the same-run main handoff.
        // No IMU/clock priors or robust reweighting enter this diagnostic.
        std::vector<std::vector<const libgnss::FGOProcessor::PseudorangeFactor*>>
            rows(problem.epochs.size());
        std::size_t invalid = 0, measured = 0;
        for (const auto& factor : problem.pseudorange_factors) {
            if (factor.epoch_index >= rows.size()) { ++invalid; continue; }
            rows[factor.epoch_index].push_back(&factor);
        }
        std::vector<double> fractions, information;
        for (std::size_t epoch = 0; epoch < rows.size(); ++epoch) {
            const auto& factors = rows[epoch];
            if (factors.empty()) continue;
            Eigen::MatrixXd nuisance = Eigen::MatrixXd::Zero(factors.size(), 10);
            Eigen::VectorXd coefficient(factors.size()), sigma(factors.size());
            bool valid = true;
            for (std::size_t i = 0; i < factors.size(); ++i) {
                const auto& f = *factors[i];
                const int component = libgnss::raw_p_seed::c7ClockComponentFor(
                    f.satellite.system, f.signal);
                const libgnss::Vector3d delta = problem.epochs[epoch].position_ecef -
                                               f.satellite_position_ecef;
                const double range = delta.norm();
                if (component < 0 || component >= 7 || !std::isfinite(range) || range <= 0 ||
                    !std::isfinite(f.sigma_m) || f.sigma_m <= 0 ||
                    !std::isfinite(f.residual_ionosphere_coefficient) ||
                    f.residual_ionosphere_coefficient <= 0) { valid = false; break; }
                nuisance.block<1,3>(i,0) = (delta / range).transpose();
                nuisance(i,3) = 1.;
                nuisance(i,3 + component) = 1.;
                coefficient[i] = f.residual_ionosphere_coefficient;
                sigma[i] = f.sigma_m;
            }
            if (!valid) { ++invalid; continue; }
            const double total = (coefficient.array() / sigma.array()).matrix().squaredNorm();
            const double projected = libgnss::pseudorange_information::scalarProjectedInformation(
                coefficient, nuisance, sigma);
            if (!std::isfinite(total) || total <= 0) { ++invalid; continue; }
            ++measured;
            fractions.push_back(projected / total);
            information.push_back(projected);
        }
        const auto median = [](std::vector<double> values) {
            if (values.empty()) return 0.;
            std::sort(values.begin(), values.end());
            const auto n = values.size();
            return n % 2 ? values[n/2] : .5 * (values[n/2-1] + values[n/2]);
        };
        std::cerr << "[native-code-ionosphere-information] epochs=" << measured
                  << " invalid=" << invalid << " median_fraction=" << median(fractions)
                  << " median_information_per_m2=" << median(information)
                  << " factors_changed=0 weights_changed=0\n";
    }
    if (!use_imu &&
        (options.native_direct_wls_ephemeral_c7d_main_seed ||
         options.native_direct_doppler_wls_handoff ||
         options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
         options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
         options.native_phase135_official_affine_measurement_family ||
         options.native_phase171_raw_p_no_doppler_imu_main)) {
        std::cerr << "candidate IMU initialization failed closed; fallback is "
                     "forbidden\n";
        if (phase94_diagnostics.enabled) {
            phase94RecordFailure(phase94_diagnostics, "imu-build",
                                 imu_report.failure.empty()
                                     ? "IMU initialization failed"
                                     : imu_report.failure);
            if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                std::cerr << "failed to publish Phase94 diagnostics\n";
            }
            std::cerr << "Phase94 IMU initialization failed closed\n";
        }
        return 1;
    }
    if (use_imu) {
        try {
            result = processor.optimizeProblem(problem);
        } catch (const std::exception& error) {
            if (phase94_diagnostics.enabled) {
                if (phase94_diagnostics.phase96_enabled) {
                    phase94_diagnostics.phase96_main.attempted = true;
                    phase94_diagnostics.phase96_main.exceptions.push_back(
                        {"main-optimize", "exception", typeid(error).name(),
                         error.what(), 1});
                }
                if (phase94_diagnostics.phase97_enabled) {
                    phase94_diagnostics.phase97_main.attempted = true;
                    phase94_diagnostics.phase97_main.exceptions.push_back(
                        {"main-optimize", "exception", typeid(error).name(),
                         error.what(), 1});
                }
                phase94RecordFailure(phase94_diagnostics, "main-optimize",
                                     "main optimizer exception", typeid(error).name(),
                                     error.what());
                if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                    std::cerr << "failed to publish Phase96/94 diagnostics\n";
                }
            }
            std::cerr << "main optimizer exception: " << error.what() << "\n";
            return 1;
        } catch (...) {
            if (phase94_diagnostics.enabled) {
                if (phase94_diagnostics.phase96_enabled) {
                    phase94_diagnostics.phase96_main.attempted = true;
                    phase94_diagnostics.phase96_main.exceptions.push_back(
                        {"main-optimize", "exception", "unknown",
                         "unknown exception", 1});
                }
                if (phase94_diagnostics.phase97_enabled) {
                    phase94_diagnostics.phase97_main.attempted = true;
                    phase94_diagnostics.phase97_main.exceptions.push_back(
                        {"main-optimize", "exception", "unknown",
                         "unknown exception", 1});
                }
                phase94RecordFailure(phase94_diagnostics, "main-optimize",
                                     "main optimizer unknown exception", "unknown",
                                     "unknown exception");
                if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                    std::cerr << "failed to publish Phase96/94 diagnostics\n";
                }
            }
            std::cerr << "main optimizer unknown exception\n";
            return 1;
        }
        if (phase94_diagnostics.enabled) {
            phase94_diagnostics.main = makePhase94MainTelemetry(
                problem, result, imu_report, phase94_diagnostics.gnss_first);
            phase94_diagnostics.phase96_main =
                result.diagnostics.native_source_clock_c0d_phase96_main;
            phase94_diagnostics.phase97_main =
                result.diagnostics.native_source_clock_c0d_phase97_singular_system;
            phase94_diagnostics.phase98_solver =
                result.diagnostics.native_source_clock_c0d_phase98_solver_rank;
        }
        if (result.solution.isEmpty() || !result.diagnostics.converged ||
            result.diagnostics.imu_intervals == 0) {
            if (phase94_diagnostics.enabled) {
                phase94RecordFailure(
                    phase94_diagnostics, "main-optimize",
                    result.solution.isEmpty()
                        ? "main result is empty"
                        : (!result.diagnostics.converged
                               ? "main result is not converged"
                               : "main result has zero IMU intervals"));
                if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                    std::cerr << "failed to publish Phase94 diagnostics\n";
                }
                std::cerr << "Phase94 main candidate failed closed\n";
                return 1;
            }
            if (options.native_upstream_stop_constraints ||
                options.native_direct_wls_ephemeral_c7d_main_seed ||
                options.native_direct_doppler_wls_handoff ||
                options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer ||
                options.native_source_clock_c0d_gnss_first_meter_state_handoff ||
                options.native_phase135_official_affine_measurement_family ||
                options.native_phase171_raw_p_no_doppler_imu_main) {
                std::cerr << "upstream stop-constraint candidate failed closed; "
                             "fallback is forbidden for candidate evaluation\n";
                return 1;
            }
            use_imu = false;
            fallback = true;
        }
    }
    if (!use_imu) {
        fallback = true;
        problem.imu.valid = false;
        libgnss::FGOProcessor::FGOConfig fallback_config = config;
        fallback_config.use_imu = false;
        fallback_config.use_pose3_state = false;
        const libgnss::FGOProcessor fallback_processor(fallback_config);
        result = fallback_processor.optimizeProblem(problem);
    }
    if (result.solution.isEmpty() || !result.diagnostics.converged) {
        std::cerr << "FGO produced no finite converged output\n";
        if (phase94_diagnostics.enabled) {
            phase94RecordFailure(phase94_diagnostics, "main-optimize",
                                 "main result is empty or not converged");
            if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                std::cerr << "failed to publish Phase94 diagnostics\n";
            }
        }
        return 1;
    }
    if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {
        const bool optimized_d_coverage =
            result.epoch_clock_drift_mps.size() == problem.epochs.size() &&
            std::all_of(result.epoch_clock_drift_mps.begin(),
                        result.epoch_clock_drift_mps.end(),
                        [](double value) { return std::isfinite(value); });
        const bool optimized_position_clock_coverage =
            result.solution.solutions.size() == problem.epochs.size() &&
            std::all_of(
                result.solution.solutions.begin(), result.solution.solutions.end(),
                [](const auto& solution) {
                    return earthValidEcef(solution.position_ecef) &&
                           std::isfinite(solution.receiver_clock_bias);
                });
        const bool optimized_velocity_coverage =
            result.epoch_velocity_nav_mps.size() == problem.epochs.size() &&
            std::all_of(result.epoch_velocity_nav_mps.begin(),
                        result.epoch_velocity_nav_mps.end(),
                        [](const auto& velocity) { return velocity.allFinite(); });
        const auto& c0d_diagnostics = result.diagnostics;
        const bool strict_progress =
            c0d_diagnostics.native_source_clock_c0d_factor_enabled &&
            c0d_diagnostics.native_source_clock_c0d_meter_state_parity_enabled &&
            c0d_diagnostics.native_source_clock_c0d_factor_count > 0U &&
            c0d_diagnostics.native_source_clock_c0d_active_solve_attempted &&
            c0d_diagnostics.native_source_clock_c0d_accepted_outer_iterations > 0U &&
            c0d_diagnostics.native_source_clock_c0d_active_solve_finite_costs &&
            std::isfinite(c0d_diagnostics.initial_cost) &&
            std::isfinite(c0d_diagnostics.final_cost) &&
            c0d_diagnostics.final_cost < c0d_diagnostics.initial_cost;
        if (!optimized_d_coverage || !optimized_position_clock_coverage ||
            !optimized_velocity_coverage || !strict_progress) {
            if (phase94_diagnostics.enabled) {
                phase94_diagnostics.main.contract_passed = false;
                phase94RecordFailure(
                    phase94_diagnostics, "coverage-contract",
                    "Phase93 optimized-D/progress contract failed closed");
                if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                    std::cerr << "failed to publish Phase94 diagnostics\n";
                }
            }
            std::cerr << "Phase93 optimized-D/progress contract failed closed: "
                         "coverage="
                      << (optimized_d_coverage ? "true" : "false")
                      << " position_clock="
                      << (optimized_position_clock_coverage ? "true" : "false")
                      << " velocity="
                      << (optimized_velocity_coverage ? "true" : "false")
                      << " c0d_factor_count="
                      << c0d_diagnostics.native_source_clock_c0d_factor_count
                      << " accepted_outer_iterations="
                      << c0d_diagnostics.native_source_clock_c0d_accepted_outer_iterations
                      << " initial_cost=" << c0d_diagnostics.initial_cost
                      << " final_cost=" << c0d_diagnostics.final_cost << "\n";
            return 1;
        }
        if (phase94_diagnostics.enabled) {
            // Phase94 is diagnostic-only even when every structural predicate
            // happens to pass.  Keep the result in memory for telemetry, but
            // never construct or publish the submission CSV/accuracy lane.
            phase94_diagnostics.main.contract_passed = true;
            phase94_diagnostics.status =
                "captured-solution-output-withheld";
            phase94_diagnostics.failure_stage = "not-reached";
            if (!writePhase94StageDiagnostics(options, phase94_diagnostics)) {
                std::cerr << "failed to publish Phase94 diagnostics\n";
            }
            std::cerr << "Phase94 stage diagnostics captured; "
                         "solution/accuracy output withheld\n";
            return 1;
        }
    }
    if (options.native_direct_wls_ephemeral_c7d_main_seed) {
        const bool optimized_d_coverage =
            result.epoch_clock_drift_mps.size() == problem.epochs.size() &&
            std::all_of(result.epoch_clock_drift_mps.begin(),
                        result.epoch_clock_drift_mps.end(),
                        [](double value) { return std::isfinite(value); });
        const bool optimized_c_coverage =
            result.epoch_clock_bias_components_m.size() == problem.epochs.size() &&
            std::all_of(
                result.epoch_clock_bias_components_m.begin(),
                result.epoch_clock_bias_components_m.end(),
                [](const auto& value) {
                    return std::all_of(value.begin(), value.end(),
                                       [](double component) {
                                           return std::isfinite(component);
                                       });
                });
        const bool optimized_position_clock_coverage =
            result.solution.solutions.size() == problem.epochs.size() &&
            std::all_of(
                result.solution.solutions.begin(), result.solution.solutions.end(),
                [](const auto& solution) {
                    return earthValidEcef(solution.position_ecef) &&
                           std::isfinite(solution.receiver_clock_bias);
                });
        const bool optimized_velocity_coverage =
            result.epoch_velocity_nav_mps.size() == problem.epochs.size() &&
            std::all_of(result.epoch_velocity_nav_mps.begin(),
                        result.epoch_velocity_nav_mps.end(),
                        [](const auto& velocity) { return velocity.allFinite(); });
        const auto& diagnostics = result.diagnostics;
        const bool strict_progress =
            diagnostics.native_source_clock_c0d_factor_enabled &&
            diagnostics.native_source_clock_c0d_meter_state_parity_enabled &&
            diagnostics.native_source_clock_c0d_epoch_vector_parity_enabled &&
            diagnostics.native_source_clock_c0d_factor_count > 0U &&
            diagnostics.native_source_clock_c0d_active_solve_attempted &&
            diagnostics.native_source_clock_c0d_accepted_outer_iterations > 0U &&
            diagnostics.native_source_clock_c0d_active_solve_finite_costs &&
            diagnostics.native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected &&
            std::isfinite(diagnostics.initial_cost) &&
            std::isfinite(diagnostics.final_cost) &&
            diagnostics.final_cost < diagnostics.initial_cost;
        if (!imu_report.direct_wls_ephemeral_c7d_main_seed_valid ||
            !optimized_d_coverage || !optimized_c_coverage ||
            !optimized_position_clock_coverage || !optimized_velocity_coverage ||
            !strict_progress || fallback) {
            std::cerr << "Phase114 direct-WLS C7/D/progress contract failed closed: "
                      << "seed="
                      << (imu_report.direct_wls_ephemeral_c7d_main_seed_valid
                              ? "true"
                              : "false")
                      << " D=" << (optimized_d_coverage ? "true" : "false")
                      << " C=" << (optimized_c_coverage ? "true" : "false")
                      << " position_clock="
                      << (optimized_position_clock_coverage ? "true" : "false")
                      << " velocity="
                      << (optimized_velocity_coverage ? "true" : "false")
                      << " qr="
                      << (diagnostics.native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected
                              ? "true"
                              : "false")
                      << " accepted_outer_iterations="
                      << diagnostics.native_source_clock_c0d_accepted_outer_iterations
                      << " initial_cost=" << diagnostics.initial_cost
                      << " final_cost=" << diagnostics.final_cost << "\n";
            return 1;
        }
    }
    if (options.native_phase104_stage_main_accuracy_attribution) {
        std::string displacement_error;
        if (!writePhase104MainDisplacementStats(
                options, result, phase104_displacement_report,
                displacement_error)) {
            std::cerr << "Phase104 main displacement stats failed closed: "
                      << displacement_error << "\n";
            return 1;
        }
    }
    if (options.native_residual_ionosphere) {
        const auto& residual_diagnostics = result.diagnostics;
        const bool finite_states =
            result.residual_ionosphere_estimates_m.size() == problem.epochs.size() &&
            std::all_of(result.residual_ionosphere_estimates_m.begin(),
                        result.residual_ionosphere_estimates_m.end(),
                        [](double value) { return std::isfinite(value); });
        const bool structural_ok =
            !fallback &&
            problem.diagnostics.residual_ionosphere_invalid_coefficients == 0U &&
            residual_diagnostics.residual_ionosphere_invalid_coefficients == 0U &&
            residual_diagnostics.residual_ionosphere_factors ==
                problem.pseudorange_factors.size() &&
            residual_diagnostics.residual_ionosphere_states == problem.epochs.size() &&
            finite_states &&
            residual_diagnostics.residual_ionosphere_max_abs_m <=
                config.residual_ionosphere_max_abs_m &&
            residual_diagnostics.residual_ionosphere_factors > 0U &&
            std::isfinite(residual_diagnostics.initial_cost) &&
            std::isfinite(residual_diagnostics.final_cost) &&
            residual_diagnostics.final_cost <= residual_diagnostics.initial_cost;
        if (!structural_ok) {
            std::cerr << "residual-ionosphere structural contract failed closed: "
                      << "fallback=" << (fallback ? "true" : "false")
                      << " factors=" << residual_diagnostics.residual_ionosphere_factors
                      << "/" << problem.pseudorange_factors.size()
                      << " states=" << residual_diagnostics.residual_ionosphere_states
                      << "/" << problem.epochs.size()
                      << " invalid="
                      << residual_diagnostics.residual_ionosphere_invalid_coefficients
                      << " max_abs_m="
                      << residual_diagnostics.residual_ionosphere_max_abs_m << "\n";
            return 1;
        }
    }
    if (!problem.double_difference_pseudorange_factors.empty() ||
        !problem.double_difference_carrier_factors.empty()) {
        std::cerr << "no-base contract violated by problem builder\n";
        return 1;
    }

    const TdcpRuntimeReport tdcp_report = evaluateTdcpRuntime(
        problem, result, config, options.native_pdc_imu_tdcp);
    if (options.native_tdcp_frequency_residual_states &&
        (result.diagnostics.tdcp_frequency_residual_priors !=
             result.diagnostics.tdcp_frequency_residual_states ||
         !std::isfinite(result.diagnostics.tdcp_residual_rms_m) ||
         !std::isfinite(tdcp_report.residual_rms_m) ||
         std::abs(result.diagnostics.tdcp_residual_rms_m -
                  tdcp_report.residual_rms_m) > 1e-7)) {
        std::cerr << "native TDCP frequency prior/RMS diagnostic contract failed\n";
        return 1;
    }
    if (options.native_pdc_imu_tdcp &&
        (tdcp_report.factors_built == 0U ||
         tdcp_report.factors_inserted != tdcp_report.factors_built ||
         tdcp_report.nonfinite_residuals != 0U)) {
        std::cerr << "native PDC+IMU TDCP contract failed: built="
                  << tdcp_report.factors_built
                  << " inserted=" << tdcp_report.factors_inserted
                  << " nonfinite_residuals=" << tdcp_report.nonfinite_residuals
                  << "\n";
        return 1;
    }
    if (options.native_phase116_carrier_tdcp_incidence_diagnostic) {
        phase116_tdcp_report = evaluatePhase116CarrierTdcp(
            problem, result, config, tdcp_report);
    }

    // Keep the bridge's basin effect observable without reading truth: compare
    // each final FGO position with the corresponding finite PDC initializer.
    // This is a diagnostic only; it never changes a state or selects a lane.
    if (options.native_pdc_state_bridge &&
        !problem.native_pdc_state_seeds.empty()) {
        std::vector<double> displacements;
        displacements.reserve(problem.native_pdc_state_seeds.size());
        for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
             ++epoch_index) {
            const auto seed_it = std::find_if(
                problem.native_pdc_state_seeds.begin(),
                problem.native_pdc_state_seeds.end(),
                [epoch_index](const auto& seed) {
                    return seed.epoch_index == epoch_index;
                });
            if (seed_it == problem.native_pdc_state_seeds.end() ||
                epoch_index >= result.solution.solutions.size()) {
                continue;
            }
            const auto& solution = result.solution.solutions[epoch_index];
            const double displacement =
                (solution.position_ecef - seed_it->position_ecef).norm();
            if (std::isfinite(displacement)) displacements.push_back(displacement);
        }
        if (!displacements.empty()) {
            std::sort(displacements.begin(), displacements.end());
            const std::size_t middle = displacements.size() / 2U;
            pdc_bridge_report.fgo_seed_displacement_count = displacements.size();
            pdc_bridge_report.fgo_seed_displacement_p50_m =
                (displacements.size() % 2U == 0U)
                    ? 0.5 * (displacements[middle - 1U] + displacements[middle])
                    : displacements[middle];
            pdc_bridge_report.fgo_seed_displacement_max_m = displacements.back();
        }
    }

    UpstreamPositionOffsetReport position_offset_report;
    UpstreamPositionOffsetApplicationGuard position_offset_application;
    position_offset_report.enabled = options.native_upstream_position_offset;
    if (options.native_upstream_position_offset) {
        position_offset_report.phone = phoneFromDatasetId(options.dataset_id);
        libgnss::upstream_position_offset::PhoneOffset phone_offset;
        const bool phase112_recipe =
            phase112MainOutputPositionOffsetSelectors(options);
        if (phase112_recipe && position_offset_report.phone != "pixel5") {
            position_offset_report.failure =
                "Phase112 output candidate requires exact dataset phone pixel5";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        const bool known_phone = libgnss::upstream_position_offset::phoneOffset(
            position_offset_report.phone, phone_offset);
        if (!known_phone) {
            position_offset_report.failure = "unknown phone family";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        position_offset_report.offset_rl_m = phone_offset.offset_rl_m;
        position_offset_report.offset_ud_m = phone_offset.offset_ud_m;
        if (fallback || !problem.imu.valid ||
            result.epoch_attitude_rpy_rad.size() !=
                result.solution.solutions.size()) {
            position_offset_report.failure =
                "candidate requires a finite native IMU Pose3 rpy for every output epoch";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        const double origin_lat = problem.imu.nav_origin_lat_rad;
        const double origin_lon = problem.imu.nav_origin_lon_rad;
        if (!std::isfinite(origin_lat) || !std::isfinite(origin_lon)) {
            position_offset_report.failure = "native ENU origin is non-finite";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        if (position_offset_report.applied ||
            !position_offset_application.claim()) {
            position_offset_report.failure =
                "position offset application attempted more than once";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        // This is the sole application site: after same-run main optimization
        // and before raw UTC-key alignment.  GNSS-first/stage/handoff objects
        // are never passed through this post-solve correction.
        for (std::size_t index = 0; index < result.solution.solutions.size();
             ++index) {
            auto& solution = result.solution.solutions[index];
            const auto offset = libgnss::upstream_position_offset::offsetFromRpy(
                position_offset_report.phone,
                result.epoch_attitude_rpy_rad[index]);
            if (!offset.ok || !solution.position_ecef.allFinite()) {
                position_offset_report.failure =
                    "non-finite native attitude or position";
                std::cerr << "upstream position offset failed closed: "
                          << position_offset_report.failure << "\n";
                return 1;
            }
            const double offset_norm = offset.offset_enu_m.norm();
            const libgnss::Vector3d corrected =
                solution.position_ecef +
                libgnss::enu2ecef(offset.offset_enu_m, origin_lat, origin_lon);
            if (!corrected.allFinite() || corrected.norm() < 6.0e6 ||
                corrected.norm() > 7.0e6 || !std::isfinite(offset_norm)) {
                position_offset_report.failure =
                    "corrected position or offset is non-finite/out-of-Earth";
                std::cerr << "upstream position offset failed closed: "
                          << position_offset_report.failure << "\n";
                return 1;
            }
            solution.position_ecef = corrected;
            double lat = 0.0;
            double lon = 0.0;
            double height = 0.0;
            libgnss::ecef2geodetic(corrected, lat, lon, height);
            if (!std::isfinite(lat) || !std::isfinite(lon) ||
                !std::isfinite(height)) {
                position_offset_report.failure =
                    "corrected geodetic position is non-finite";
                std::cerr << "upstream position offset failed closed: "
                          << position_offset_report.failure << "\n";
                return 1;
            }
            solution.position_geodetic = libgnss::GeodeticCoord(lat, lon, height);
            position_offset_report.max_offset_enu_m =
                std::max(position_offset_report.max_offset_enu_m, offset_norm);
            ++position_offset_report.corrected_epochs;
        }
        position_offset_report.applied =
            position_offset_report.corrected_epochs ==
            result.solution.solutions.size();
        if (!position_offset_report.applied) {
            position_offset_report.failure = "not every native solution was corrected";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        position_offset_report.application_passes = 1U;
    }

    RawUtcOutputReport raw_utc_report;
    std::vector<OutputPosition> output_positions;
    if (options.android_raw_utc_key_contract) {
        constexpr double kSolutionTimeToleranceMs = 2.0;
        std::vector<libgnss::io::AndroidRawGnssSolutionPoint> solution_points;
        solution_points.reserve(result.solution.solutions.size());
        for (const auto& solution : result.solution.solutions) {
            solution_points.push_back({solution.time, solution.position_ecef});
        }
        libgnss::io::AndroidRawGnssEpochAlignment alignment;
        std::string alignment_error;
        if (!libgnss::io::alignAndroidRawGnssSolutionsToUtcKeys(
                android_raw_epoch_times, android_gnss.epoch_utc_time_millis,
                solution_points, kSolutionTimeToleranceMs, alignment,
                alignment_error, options.android_include_first_native_epoch)) {
            std::cerr << "failed to align output to raw UTC keys: "
                      << alignment_error << "\n";
            return 1;
        }
        raw_utc_report.enabled = true;
        raw_utc_report.warmup_epoch_excluded = !options.android_include_first_native_epoch;
        raw_utc_report.raw_epoch_keys = android_gnss.epoch_utc_time_millis.size();
        raw_utc_report.target_epochs = alignment.target_epochs;
        raw_utc_report.exact_solution_epochs = alignment.exact_solution_epochs;
        raw_utc_report.interpolated_epochs = alignment.interpolated_epochs;
        raw_utc_report.edge_hold_epochs = alignment.edge_hold_epochs;
        raw_utc_report.unresolved_epochs = alignment.unresolved_epochs;
        raw_utc_report.solution_time_tolerance_ms = kSolutionTimeToleranceMs;
        raw_utc_report.max_interpolation_gap_ms = alignment.max_interpolation_gap_ms;
        raw_utc_report.max_edge_hold_gap_ms = alignment.max_edge_hold_gap_ms;
        output_positions.reserve(alignment.epochs.size());
        for (const auto& aligned : alignment.epochs) {
            output_positions.push_back({aligned.utc_time_millis,
                                        aligned.position_ecef});
        }
    } else {
        output_positions.reserve(result.solution.solutions.size());
        for (const auto& solution : result.solution.solutions) {
            const double timestamp = unixMillis(solution.time);
            if (!std::isfinite(timestamp)) {
                std::cerr << "non-finite output timestamp\n";
                return 1;
            }
            output_positions.push_back({static_cast<std::int64_t>(timestamp),
                                        solution.position_ecef});
        }
    }

    std::ostringstream csv;
    csv << "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n";
    std::size_t finite_rows = 0;
    for (const auto& output_position : output_positions) {
        if (!output_position.position_ecef.allFinite() ||
            output_position.position_ecef.norm() < 6.0e6 ||
            output_position.position_ecef.norm() > 7.0e6) {
            std::cerr << "non-finite or out-of-Earth output row\n";
            return 1;
        }
        double lat = 0.0;
        double lon = 0.0;
        double output_height = 0.0;
        libgnss::ecef2geodetic(output_position.position_ecef, lat, lon,
                               output_height);
        if (!std::isfinite(lat) || !std::isfinite(lon) ||
            std::abs(lat) > kPi / 2.0 || std::abs(lon) > kPi) {
            std::cerr << "non-finite or out-of-range output row\n";
            return 1;
        }
        csv << options.dataset_id << ',' << output_position.utc_time_millis << ','
            << std::fixed << std::setprecision(10) << lat * kRadToDeg << ','
            << lon * kRadToDeg << '\n';
        ++finite_rows;
    }
    if (finite_rows == 0 || !atomicWrite(options.out_path, csv.str())) {
        std::cerr << "failed to atomically publish output\n";
        return 1;
    }
    const std::string summary = makeSummary(options, problem, result, imu_report,
                                            fallback, pdc_bridge_report,
                                            raw_utc_report, tdcp_report,
                                            carrier_code_leveling_report,
                                            position_offset_report,
                                            base_pseudorange_report,
                                            phase104_stage_report,
                                            phase104_displacement_report,
                                            phase116_tdcp_report);
    if (!atomicWrite(options.summary_path, summary)) {
        std::cerr << "failed to atomically publish summary\n";
        return 1;
    }
    std::cout << "native no-base FGO: epochs=" << problem.epochs.size()
              << " output=" << finite_rows
              << " imu_intervals=" << result.diagnostics.imu_intervals
              << " graph_factors=" << result.diagnostics.graph_factors
              << " graph_values=" << result.diagnostics.graph_values
              << " fallback=" << (fallback ? "yes" : "no") << '\n';
    return 0;
}
