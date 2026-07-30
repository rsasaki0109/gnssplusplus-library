#include <algorithm>
#include <chrono>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/external/madocalib_bridge.hpp>
#include <libgnss++/io/madoca_l6.hpp>
#include <libgnss++/io/rinex.hpp>

namespace {

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string sp3_path;
    std::string clk_path;
    std::string ssr_path;
    std::string ssr_rtcm_path;
    std::vector<std::string> madoca_l6_paths;
    std::vector<std::string> madoca_l6d_paths;
    std::vector<std::string> madoca_l6d_shadow_paths;
    std::string madoca_materialization_dump_path;
    bool madoca_materialization_dump_only = false;
    std::string ionex_path;
    std::string dcb_path;
    std::string antex_path;
    std::string receiver_antenna_type;
    std::string blq_path;
    std::string ocean_loading_station_name;
    std::string out_path;
    std::string summary_json_path;
    std::string kml_path;
    int max_epochs = 0;
    int convergence_min_epochs = 20;
    std::string convergence_policy = "legacy-3d";
    double convergence_threshold_horizontal = 0.1;
    double convergence_threshold_vertical = 0.2;
    double ssr_step_seconds = 1.0;
    bool estimate_troposphere = true;
    bool estimate_ionosphere = false;
    bool use_ionosphere_free = true;
    bool use_clas_osr_filter = false;
    std::string clas_epoch_policy = "strict-osr";
    std::string clas_osr_application = "full-osr";
    std::string clas_phase_continuity = "full-repair";
    std::string clas_phase_bias_values = "full";
    std::string clas_phase_bias_reference_time = "phase-bias-reference";
    std::string clas_ssr_timing = "lag-tolerant";
    std::string clas_expanded_values = "full-composed";
    std::string clas_subtype12_values = "full";
    std::string clas_residual_sampling = "indexed-or-mean";
    std::string clas_atmos_selection = "grid-first";
    double clas_atmos_stale_after_seconds = 15.0;
    bool madocalib_bridge = false;
    std::string madocalib_config_path;
    std::string madocalib_profile = "ppp";
    std::vector<std::string> madocalib_l6_paths;
    std::vector<std::string> madocalib_mdciono_paths;
    std::string madocalib_start_time;
    std::string madocalib_end_time;
    double madocalib_time_interval_seconds = 0.0;
    int madocalib_trace_level = 0;
    bool madocalib_trace_ar = false;
    bool kinematic_mode = false;
    bool low_dynamics_mode = false;
    bool use_dynamics_model = false;
    bool emit_epoch_time = false;
    bool apply_static_anchor_blend = false;
    bool enable_ar = false;
    double ar_ratio_threshold = 3.0;
    std::string ar_method = "iflc";
    double process_noise_iono = 0.0;
    bool process_noise_iono_set = false;
    bool use_iers_solid_tide = true;
    bool use_iers_ocean_loading = false;
    std::string eop_c04_file;
    bool use_iers_pole_tide = true;
    bool use_iers_sub_daily_eop = true;
    std::string atm_tidal_loading_file;
    bool use_iers_atm_tidal_loading = false;
    bool quiet = false;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --obs <rover.obs> [options]\n"
        << "Options:\n"
        << "  --nav <nav.rnx>           Optional broadcast navigation file\n"
        << "  --sp3 <orbit.sp3>        Precise orbit file\n"
        << "  --clk <clock.clk>        Precise clock file\n"
        << "  --ssr <corrections.csv>  Simple SSR orbit/clock corrections CSV\n"
        << "  --ssr-rtcm <file|ntrip://...|serial://...|tcp://...>\n"
        << "                          RTCM SSR source converted/read for PPP use\n"
        << "  --madoca-l6 <file>       Native MADOCA L6E SSR channel (repeatable,\n"
        << "                          e.g. PRN 204 and 206); requires --nav\n"
        << "  --madoca-l6d <file>      Native MADOCA L6D STEC input (repeatable);\n"
        << "                          opt-in application for per-frequency PPP;\n"
        << "                          accepts hourly sequences for multi-hour runs\n"
        << "  --madoca-l6d-shadow <file>\n"
        << "                          Native L6D ionosphere shadow input (repeatable);\n"
        << "                          reports lookup diagnostics without changing PPP\n"
        << "  --madoca-materialization-dump <csv>\n"
        << "                          Dump native MADOCA L6E SSRProducts materialization\n"
        << "                          rows before PPP processing\n"
        << "  --madoca-materialization-dump-only\n"
        << "                          Exit after writing --madoca-materialization-dump;\n"
        << "                          useful for correction-materialization sign-off\n"
        << "  --ionex <maps.ionex>     Optional IONEX TEC map product\n"
        << "  --dcb <bias.bsx>         Optional DCB / Bias-SINEX product\n"
        << "  --antex <antennas.atx>   Optional ANTEX file for receiver antenna PCO\n"
        << "  --receiver-antenna-type <type>\n"
        << "                          Override the RINEX receiver antenna type used with --antex\n"
        << "  --blq <station.blq>      Optional BLQ ocean loading coefficient file\n"
        << "  --ocean-loading-station <name>\n"
        << "                          Station name to select from the BLQ file\n"
        << "  --ssr-step-seconds <seconds>\n"
        << "                          Sampling step for RTCM SSR conversion (default: 1.0)\n"
        << "  --out <solution.pos>     Output position file (required)\n"
        << "  --summary-json <summary.json>\n"
        << "                          Optional machine-readable run summary\n"
        << "  --kml <solution.kml>     Optional KML output\n"
        << "  --max-epochs <count>     Limit processed epochs (default: all)\n"
        << "  --convergence-min-epochs <count>\n"
        << "                          Minimum epochs before PPP convergence/AR checks (default: 20)\n"
        << "  --convergence-policy <legacy-3d|local-enu>\n"
        << "                          Convergence metric policy (default: legacy-3d)\n"
        << "  --convergence-threshold-horizontal <m>\n"
        << "                          Horizontal threshold for local-enu policy (default: 0.1)\n"
        << "  --convergence-threshold-vertical <m>\n"
        << "                          Absolute Up threshold for local-enu policy (default: 0.2)\n"
        << "  --no-estimate-troposphere\n"
        << "                          Disable zenith troposphere estimation\n"
        << "  --estimate-troposphere  Enable zenith troposphere estimation (default)\n"
        << "  --clas-osr              Use the CLAS OSR-based PPP-RTK filter path\n"
        << "  --clas-epoch-policy <strict-osr|hybrid-standard-ppp>\n"
        << "                          CLAS epoch boundary policy (default: strict-osr)\n"
        << "  --clas-osr-application <full-osr|orbit-clock-bias|orbit-clock-only>\n"
        << "                          CLAS accepted-update correction semantics (default: full-osr)\n"
        << "  --clas-phase-continuity <full-repair|sis-continuity-only|repair-only|raw-phase-bias|no-phase-bias>\n"
        << "                          CLAS phase-bias / continuity semantics (default: full-repair)\n"
        << "  --clas-phase-bias-values <full|phase-bias-only|compensation-only>\n"
        << "                          CLAS phase-bias value-construction policy before SIS/repair injection (default: full)\n"
        << "  --clas-phase-bias-reference-time <phase-bias-reference|clock-reference|observation-epoch>\n"
        << "                          CLAS phase-bias reference-time semantics before SIS/repair injection (default: phase-bias-reference)\n"
        << "  --clas-ssr-timing <lag-tolerant|clock-bound-phase-bias|clock-bound-atmos-and-phase-bias>\n"
        << "                          CLAS expanded-SSR source timing policy (default: lag-tolerant)\n"
        << "  --clas-expanded-values <full-composed|residual-only|polynomial-only>\n"
        << "                          CLAS expanded atmosphere-value construction policy (default: full-composed)\n"
        << "  --clas-subtype12-values <full|planar|offset-only>\n"
        << "                          CLAS subtype-12 surface-term policy before residual formation (default: full)\n"
        << "  --clas-residual-sampling <indexed-or-mean|indexed-only|mean-only>\n"
        << "                          CLAS residual-list sampling policy inside expanded atmosphere rows (default: indexed-or-mean)\n"
        << "  --clas-atmos-selection <grid-first|grid-guarded|balanced|freshness-first>\n"
        << "                          CLAS atmosphere token selection policy (default: grid-first)\n"
        << "  --clas-atmos-stale-after-seconds <seconds>\n"
        << "                          Balanced policy stale threshold (default: 15.0)\n"
        << "  --madocalib-bridge      Delegate this run to linked MADOCALIB postpos()\n"
        << "                          (requires CMake -DMADOCALIB_PARITY_LINK=ON)\n"
        << "  --madocalib-l6 <file>   Extra MADOCA L6 input file; repeat for two-channel L6E\n"
        << "  --madocalib-mdciono <file>\n"
        << "                          Extra MADOCA L6D ionosphere file (repeatable);\n"
        << "                          hourly sequences are condensed by stream\n"
        << "  --madocalib-config <file>\n"
        << "                          MADOCALIB rnx2rtkp config (default: sample.conf)\n"
        << "  --madocalib-profile <ppp|pppar|pppar-ion>\n"
        << "                          Select a bundled MADOCALIB sample config (default: ppp)\n"
        << "  --madocalib-start <time>\n"
        << "                          MADOCALIB start time, e.g. '2025/04/01 00:00:00'\n"
        << "  --madocalib-end <time>  MADOCALIB end time, e.g. '2025/04/01 00:59:30'\n"
        << "  --madocalib-ti <seconds>\n"
        << "                          MADOCALIB output interval passed to postpos()\n"
        << "  --madocalib-trace <level>\n"
        << "                          MADOCALIB trace level (default: 0)\n"
        << "  --madocalib-trace-ar   Include MADOCALIB ambiguity means, covariance,\n"
        << "                          LAMBDA candidates, and partial-AR decisions in trace\n"
        << "  --static                Use a static PPP motion model (default)\n"
        << "  --kinematic             Use a kinematic PPP motion model\n"
        << "  --use-dynamics-model    Continuous pos/vel dynamics (MRTKLIB accel model)\n"
        << "  --emit-epoch-time       Write GPS week/TOW and geodetic columns in .pos output\n"
        << "  --low-dynamics          Keep kinematic PPP anchored for quasi-static motion\n"
        << "  --no-low-dynamics       Disable quasi-static anchoring (default)\n"
        << "  --apply-static-anchor-blend\n"
        << "                          Blend static/low-dynamics PPP toward the initial SPP seed\n"
        << "  --no-static-anchor-blend\n"
        << "                          Disable static-anchor blending (default)\n"
        << "  --enable-ar             Enable PPP ambiguity fixing when supported\n"
        << "  --disable-ar            Disable PPP ambiguity fixing (default)\n"
        << "  --ar-ratio-threshold <value>\n"
        << "                          Ratio threshold for PPP ambiguity fixing (default: 3.0)\n"
        << "  --ar-method <name>      AR method: iflc, wlnl, per-freq (default: iflc);\n"
        << "                          per-freq enables uncombined estimated-STEC states\n"
        << "  --process-noise-iono <v>\n"
        << "                          KF process noise for per-satellite ionosphere states,\n"
        << "                          m^2/s (default: 1e-4 for MADOCA per-frequency mode,\n"
        << "                          1e-3 otherwise)\n"
        << "  --use-iers-solid-tide   Use the IERS Conventions 2010 (Dehant) Step-1+Step-2\n"
        << "                          solid-earth-tide model via libgnss::iers (default).\n"
        << "                          See docs/iers-integration-plan.md\n"
        << "  --no-iers-solid-tide    Revert to the built-in Step-1-only Love-number\n"
        << "                          approximation (legacy path)\n"
        << "  --use-iers-ocean-loading\n"
        << "                          Use the IERS Conventions 2010 (HARDISP) ocean-loading\n"
        << "                          model from libgnss::iers (spline-interpolated 342-harmonic\n"
        << "                          admittance) instead of the legacy 11-constituent direct\n"
        << "                          cosine sum. Requires --blq. Opt-in pending truth-bench\n"
        << "                          validation. See docs/iers-integration-plan.md\n"
        << "  --no-iers-ocean-loading Use the legacy 11-constituent direct cosine sum (default)\n"
        << "  --eop-c04 <eop-file>     IERS Earth Orientation Parameter file (daily samples).\n"
        << "                          Format auto-detected: IERS 20 C04 (final values, ~1-week\n"
        << "                          publication lag) or IERS Bulletin A finals2000A.daily\n"
        << "                          (combined observed + predicted, extends ~12 months ahead).\n"
        << "                          Loaded into PPPProcessor for downstream IERS Phase D\n"
        << "                          consumers (pole tide, sub-daily EOP).\n"
        << "                          Sources:\n"
        << "                            C04         https://hpiers.obspm.fr/iers/eop/eopc04/eopc04.1962-now\n"
        << "                            Bulletin A  https://maia.usno.navy.mil/ser7/finals2000A.daily\n"
        << "  --eop-bulletin-a <file>  Alias for --eop-c04 (the auto-detector accepts both formats\n"
        << "                          via either flag; this alias clarifies intent at the call site).\n"
        << "  --use-iers-pole-tide     Apply the IERS Conventions 2010 §7.1.4 pole-tide station\n"
        << "                          displacement (default). Requires --eop-c04 to be set;\n"
        << "                          otherwise the displacement is silently skipped because\n"
        << "                          xp/yp are unknown. Multi-site bench (PR #69) shows median\n"
        << "                          0.4 mm at mid-latitudes, well within IERS sub-cm envelope.\n"
        << "  --no-iers-pole-tide      Skip the pole-tide model (escape hatch).\n"
        << "  --use-iers-sub-daily-eop Apply the IERS Conventions 2010 §5.5.1.1 + §8.2 sub-daily\n"
        << "                          EOP corrections (default): libration of CIP (Tables 5.1a/b)\n"
        << "                          and ocean-tide EOP corrections (Table 8.2, Eanes-Ray model).\n"
        << "                          Requires --eop-c04; deterministic harmonic series with\n"
        << "                          peak ~0.5 mas xp/yp and ~30 µs UT1, RMS 1.5 µm at receiver.\n"
        << "  --no-iers-sub-daily-eop  Skip the sub-daily EOP corrections (escape hatch).\n"
        << "  --atm-tidal-loading <file>\n"
        << "                          Per-site S1/S2 atmospheric tidal-loading coefficient file\n"
        << "                          (Phase D-3). Custom format with $$ comment lines + S1 / S2\n"
        << "                          rows of 6 numbers each (radial/west/south amplitudes in m,\n"
        << "                          radial/west/south phases in deg). Required for the\n"
        << "                          --use-iers-atm-tidal-loading flag to do anything.\n"
        << "  --use-iers-atm-tidal-loading\n"
        << "                          Apply the IERS Conventions 2010 §7.1.5 atmospheric tidal\n"
        << "                          loading station displacement (S1 + S2). Peak ~1 mm radial.\n"
        << "  --no-iers-atm-tidal-loading\n"
        << "                          Skip atmospheric tidal loading (default).\n"
        << "  --quiet                  Suppress per-run summary output\n"
        << "  -h, --help               Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}

Options parseArguments(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--obs" && i + 1 < argc) {
            options.obs_path = argv[++i];
        } else if (arg == "--nav" && i + 1 < argc) {
            options.nav_path = argv[++i];
        } else if (arg == "--sp3" && i + 1 < argc) {
            options.sp3_path = argv[++i];
        } else if (arg == "--clk" && i + 1 < argc) {
            options.clk_path = argv[++i];
        } else if (arg == "--ssr" && i + 1 < argc) {
            options.ssr_path = argv[++i];
        } else if (arg == "--ssr-rtcm" && i + 1 < argc) {
            options.ssr_rtcm_path = argv[++i];
        } else if (arg == "--madoca-l6" && i + 1 < argc) {
            options.madoca_l6_paths.push_back(argv[++i]);
        } else if (arg == "--madoca-l6d" && i + 1 < argc) {
            options.madoca_l6d_paths.push_back(argv[++i]);
        } else if (arg == "--madoca-l6d-shadow" && i + 1 < argc) {
            options.madoca_l6d_shadow_paths.push_back(argv[++i]);
        } else if (arg == "--madoca-materialization-dump" && i + 1 < argc) {
            options.madoca_materialization_dump_path = argv[++i];
        } else if (arg == "--madoca-materialization-dump-only") {
            options.madoca_materialization_dump_only = true;
        } else if (arg == "--ionex" && i + 1 < argc) {
            options.ionex_path = argv[++i];
        } else if (arg == "--dcb" && i + 1 < argc) {
            options.dcb_path = argv[++i];
        } else if (arg == "--antex" && i + 1 < argc) {
            options.antex_path = argv[++i];
        } else if (arg == "--receiver-antenna-type" && i + 1 < argc) {
            options.receiver_antenna_type = argv[++i];
        } else if (arg == "--blq" && i + 1 < argc) {
            options.blq_path = argv[++i];
        } else if (arg == "--ocean-loading-station" && i + 1 < argc) {
            options.ocean_loading_station_name = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            options.out_path = argv[++i];
        } else if (arg == "--summary-json" && i + 1 < argc) {
            options.summary_json_path = argv[++i];
        } else if (arg == "--kml" && i + 1 < argc) {
            options.kml_path = argv[++i];
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            options.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--convergence-min-epochs" && i + 1 < argc) {
            options.convergence_min_epochs = std::stoi(argv[++i]);
        } else if (arg == "--convergence-policy" && i + 1 < argc) {
            options.convergence_policy = argv[++i];
        } else if (arg == "--convergence-threshold-horizontal" && i + 1 < argc) {
            options.convergence_threshold_horizontal = std::stod(argv[++i]);
        } else if (arg == "--convergence-threshold-vertical" && i + 1 < argc) {
            options.convergence_threshold_vertical = std::stod(argv[++i]);
        } else if (arg == "--ssr-step-seconds" && i + 1 < argc) {
            options.ssr_step_seconds = std::stod(argv[++i]);
        } else if (arg == "--no-estimate-troposphere") {
            options.estimate_troposphere = false;
        } else if (arg == "--estimate-troposphere") {
            options.estimate_troposphere = true;
        } else if (arg == "--clas-osr") {
            options.use_clas_osr_filter = true;
        } else if (arg == "--clas-epoch-policy" && i + 1 < argc) {
            options.clas_epoch_policy = argv[++i];
        } else if (arg == "--clas-osr-application" && i + 1 < argc) {
            options.clas_osr_application = argv[++i];
        } else if (arg == "--clas-phase-continuity" && i + 1 < argc) {
            options.clas_phase_continuity = argv[++i];
        } else if (arg == "--clas-phase-bias-values" && i + 1 < argc) {
            options.clas_phase_bias_values = argv[++i];
        } else if (arg == "--clas-phase-bias-reference-time" && i + 1 < argc) {
            options.clas_phase_bias_reference_time = argv[++i];
        } else if (arg == "--clas-ssr-timing" && i + 1 < argc) {
            options.clas_ssr_timing = argv[++i];
        } else if (arg == "--clas-expanded-values" && i + 1 < argc) {
            options.clas_expanded_values = argv[++i];
        } else if (arg == "--clas-subtype12-values" && i + 1 < argc) {
            options.clas_subtype12_values = argv[++i];
        } else if (arg == "--clas-residual-sampling" && i + 1 < argc) {
            options.clas_residual_sampling = argv[++i];
        } else if (arg == "--clas-atmos-selection" && i + 1 < argc) {
            options.clas_atmos_selection = argv[++i];
        } else if (arg == "--clas-atmos-stale-after-seconds" && i + 1 < argc) {
            options.clas_atmos_stale_after_seconds = std::stod(argv[++i]);
        } else if (arg == "--madocalib-bridge") {
            options.madocalib_bridge = true;
        } else if (arg == "--madocalib-l6" && i + 1 < argc) {
            options.madocalib_l6_paths.push_back(argv[++i]);
        } else if (arg == "--madocalib-mdciono" && i + 1 < argc) {
            options.madocalib_mdciono_paths.push_back(argv[++i]);
        } else if (arg == "--madocalib-config" && i + 1 < argc) {
            options.madocalib_config_path = argv[++i];
        } else if (arg == "--madocalib-profile" && i + 1 < argc) {
            options.madocalib_profile = argv[++i];
        } else if (arg == "--madocalib-start" && i + 1 < argc) {
            options.madocalib_start_time = argv[++i];
        } else if (arg == "--madocalib-end" && i + 1 < argc) {
            options.madocalib_end_time = argv[++i];
        } else if (arg == "--madocalib-ti" && i + 1 < argc) {
            options.madocalib_time_interval_seconds = std::stod(argv[++i]);
        } else if (arg == "--madocalib-trace" && i + 1 < argc) {
            options.madocalib_trace_level = std::stoi(argv[++i]);
        } else if (arg == "--madocalib-trace-ar") {
            options.madocalib_trace_ar = true;
        } else if (arg == "--no-ionosphere-free") {
            options.use_ionosphere_free = false;
        } else if (arg == "--ionosphere-free") {
            options.use_ionosphere_free = true;
        } else if (arg == "--estimate-ionosphere") {
            options.estimate_ionosphere = true;
        } else if (arg == "--no-estimate-ionosphere") {
            options.estimate_ionosphere = false;
        } else if (arg == "--static") {
            options.kinematic_mode = false;
        } else if (arg == "--kinematic") {
            options.kinematic_mode = true;
        } else if (arg == "--use-dynamics-model") {
            options.use_dynamics_model = true;
        } else if (arg == "--emit-epoch-time") {
            options.emit_epoch_time = true;
        } else if (arg == "--low-dynamics") {
            options.low_dynamics_mode = true;
        } else if (arg == "--no-low-dynamics") {
            options.low_dynamics_mode = false;
        } else if (arg == "--apply-static-anchor-blend") {
            options.apply_static_anchor_blend = true;
        } else if (arg == "--no-static-anchor-blend") {
            options.apply_static_anchor_blend = false;
        } else if (arg == "--enable-ar") {
            options.enable_ar = true;
        } else if (arg == "--disable-ar") {
            options.enable_ar = false;
        } else if (arg == "--ar-ratio-threshold" && i + 1 < argc) {
            options.ar_ratio_threshold = std::stod(argv[++i]);
        } else if (arg == "--ar-method" && i + 1 < argc) {
            options.ar_method = argv[++i];
        } else if (arg == "--process-noise-iono" && i + 1 < argc) {
            options.process_noise_iono = std::stod(argv[++i]);
            options.process_noise_iono_set = true;
        } else if (arg == "--use-iers-solid-tide") {
            options.use_iers_solid_tide = true;
        } else if (arg == "--no-iers-solid-tide") {
            options.use_iers_solid_tide = false;
        } else if (arg == "--use-iers-ocean-loading") {
            options.use_iers_ocean_loading = true;
        } else if (arg == "--no-iers-ocean-loading") {
            options.use_iers_ocean_loading = false;
        } else if ((arg == "--eop-c04" || arg == "--eop-bulletin-a")
                   && i + 1 < argc) {
            options.eop_c04_file = argv[++i];
        } else if (arg == "--use-iers-pole-tide") {
            options.use_iers_pole_tide = true;
        } else if (arg == "--no-iers-pole-tide") {
            options.use_iers_pole_tide = false;
        } else if (arg == "--use-iers-sub-daily-eop") {
            options.use_iers_sub_daily_eop = true;
        } else if (arg == "--no-iers-sub-daily-eop") {
            options.use_iers_sub_daily_eop = false;
        } else if (arg == "--atm-tidal-loading" && i + 1 < argc) {
            options.atm_tidal_loading_file = argv[++i];
        } else if (arg == "--use-iers-atm-tidal-loading") {
            options.use_iers_atm_tidal_loading = true;
        } else if (arg == "--no-iers-atm-tidal-loading") {
            options.use_iers_atm_tidal_loading = false;
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    if (options.obs_path.empty()) {
        argumentError("--obs is required", argv[0]);
    }
    if (options.out_path.empty() && !options.madoca_materialization_dump_only) {
        argumentError("--out is required", argv[0]);
    }
    if (options.madoca_materialization_dump_only &&
        options.madoca_materialization_dump_path.empty()) {
        argumentError(
            "--madoca-materialization-dump-only requires --madoca-materialization-dump",
            argv[0]);
    }
    if (options.nav_path.empty() && options.sp3_path.empty()) {
        argumentError("provide at least one of --nav or --sp3", argv[0]);
    }
    if (!options.ssr_rtcm_path.empty() && options.nav_path.empty()) {
        argumentError("--ssr-rtcm requires --nav", argv[0]);
    }
    if (!options.madoca_l6_paths.empty() && options.nav_path.empty()) {
        argumentError("--madoca-l6 requires --nav (broadcast ephemeris)", argv[0]);
    }
    if (!options.madoca_l6d_paths.empty() &&
        !options.madoca_l6d_shadow_paths.empty()) {
        argumentError(
            "--madoca-l6d cannot be combined with --madoca-l6d-shadow",
            argv[0]);
    }
    if (!options.madoca_l6d_paths.empty() &&
        options.madoca_l6_paths.empty()) {
        argumentError(
            "--madoca-l6d requires native --madoca-l6 corrections",
            argv[0]);
    }
    if (!options.madoca_l6d_paths.empty() &&
        options.ar_method != "per-freq") {
        argumentError(
            "--madoca-l6d requires --ar-method per-freq",
            argv[0]);
    }
    if (options.max_epochs < 0) {
        argumentError("--max-epochs must be non-negative", argv[0]);
    }
    if (options.convergence_min_epochs <= 0) {
        argumentError("--convergence-min-epochs must be positive", argv[0]);
    }
    if (options.convergence_policy != "legacy-3d" &&
        options.convergence_policy != "local-enu") {
        argumentError("--convergence-policy must be one of: legacy-3d, local-enu", argv[0]);
    }
    if (options.convergence_threshold_horizontal <= 0.0) {
        argumentError("--convergence-threshold-horizontal must be positive", argv[0]);
    }
    if (options.convergence_threshold_vertical <= 0.0) {
        argumentError("--convergence-threshold-vertical must be positive", argv[0]);
    }
    if (options.ar_ratio_threshold <= 0.0) {
        argumentError("--ar-ratio-threshold must be positive", argv[0]);
    }
    if (options.clas_atmos_stale_after_seconds <= 0.0) {
        argumentError("--clas-atmos-stale-after-seconds must be positive", argv[0]);
    }
    if (options.ssr_step_seconds <= 0.0) {
        argumentError("--ssr-step-seconds must be positive", argv[0]);
    }
    if (options.madocalib_time_interval_seconds < 0.0) {
        argumentError("--madocalib-ti must be non-negative", argv[0]);
    }
    if (options.madocalib_trace_level < 0) {
        argumentError("--madocalib-trace must be non-negative", argv[0]);
    }
    if (options.madocalib_profile != "ppp" &&
        options.madocalib_profile != "pppar" &&
        options.madocalib_profile != "pppar-ion") {
        argumentError("--madocalib-profile must be one of: ppp, pppar, pppar-ion", argv[0]);
    }
    if (!options.madocalib_config_path.empty() && options.madocalib_profile != "ppp") {
        argumentError("--madocalib-config cannot be combined with a non-default --madocalib-profile", argv[0]);
    }
    if (options.madocalib_profile == "pppar-ion" && options.madocalib_mdciono_paths.empty()) {
        argumentError("--madocalib-profile pppar-ion requires --madocalib-mdciono", argv[0]);
    }
    if (options.clas_epoch_policy != "strict-osr" &&
        options.clas_epoch_policy != "hybrid-standard-ppp") {
        argumentError(
            "--clas-epoch-policy must be one of: strict-osr, hybrid-standard-ppp",
            argv[0]);
    }
    if (options.clas_osr_application != "full-osr" &&
        options.clas_osr_application != "orbit-clock-bias" &&
        options.clas_osr_application != "orbit-clock-only") {
        argumentError(
            "--clas-osr-application must be one of: full-osr, orbit-clock-bias, orbit-clock-only",
            argv[0]);
    }
    if (options.clas_phase_continuity != "full-repair" &&
        options.clas_phase_continuity != "sis-continuity-only" &&
        options.clas_phase_continuity != "repair-only" &&
        options.clas_phase_continuity != "raw-phase-bias" &&
        options.clas_phase_continuity != "no-phase-bias") {
        argumentError(
            "--clas-phase-continuity must be one of: full-repair, sis-continuity-only, repair-only, raw-phase-bias, no-phase-bias",
            argv[0]);
    }
    if (options.clas_phase_bias_values != "full" &&
        options.clas_phase_bias_values != "phase-bias-only" &&
        options.clas_phase_bias_values != "compensation-only") {
        argumentError(
            "--clas-phase-bias-values must be one of: full, phase-bias-only, compensation-only",
            argv[0]);
    }
    if (options.clas_phase_bias_reference_time != "phase-bias-reference" &&
        options.clas_phase_bias_reference_time != "clock-reference" &&
        options.clas_phase_bias_reference_time != "observation-epoch") {
        argumentError(
            "--clas-phase-bias-reference-time must be one of: phase-bias-reference, clock-reference, observation-epoch",
            argv[0]);
    }
    if (options.clas_ssr_timing != "lag-tolerant" &&
        options.clas_ssr_timing != "clock-bound-phase-bias" &&
        options.clas_ssr_timing != "clock-bound-atmos-and-phase-bias") {
        argumentError(
            "--clas-ssr-timing must be one of: lag-tolerant, clock-bound-phase-bias, clock-bound-atmos-and-phase-bias",
            argv[0]);
    }
    if (options.clas_expanded_values != "full-composed" &&
        options.clas_expanded_values != "residual-only" &&
        options.clas_expanded_values != "polynomial-only") {
        argumentError(
            "--clas-expanded-values must be one of: full-composed, residual-only, polynomial-only",
            argv[0]);
    }
    if (options.clas_subtype12_values != "full" &&
        options.clas_subtype12_values != "planar" &&
        options.clas_subtype12_values != "offset-only") {
        argumentError(
            "--clas-subtype12-values must be one of: full, planar, offset-only",
            argv[0]);
    }
    if (options.clas_residual_sampling != "indexed-or-mean" &&
        options.clas_residual_sampling != "indexed-only" &&
        options.clas_residual_sampling != "mean-only") {
        argumentError(
            "--clas-residual-sampling must be one of: indexed-or-mean, indexed-only, mean-only",
            argv[0]);
    }
    if (options.clas_atmos_selection != "grid-first" &&
        options.clas_atmos_selection != "grid-guarded" &&
        options.clas_atmos_selection != "balanced" &&
        options.clas_atmos_selection != "freshness-first") {
        argumentError(
            "--clas-atmos-selection must be one of: grid-first, grid-guarded, balanced, freshness-first",
            argv[0]);
    }
    return options;
}

std::string jsonEscape(const std::string& value) {
    std::ostringstream escaped;
    for (const char ch : value) {
        switch (ch) {
        case '\\':
            escaped << "\\\\";
            break;
        case '"':
            escaped << "\\\"";
            break;
        case '\n':
            escaped << "\\n";
            break;
        case '\r':
            escaped << "\\r";
            break;
        case '\t':
            escaped << "\\t";
            break;
        default:
            escaped << ch;
            break;
        }
    }
    return escaped.str();
}

}  // namespace

namespace {

libgnss::PPPProcessor::PPPConfig::ClasEpochPolicy parseClasEpochPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasEpochPolicy;
    if (value == "hybrid-standard-ppp") {
        return Policy::HYBRID_STANDARD_PPP_FALLBACK;
    }
    return Policy::STRICT_OSR;
}

libgnss::PPPProcessor::PPPConfig::ClasAtmosSelectionPolicy parseClasAtmosSelectionPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasAtmosSelectionPolicy;
    if (value == "grid-first") {
        return Policy::GRID_FIRST;
    }
    if (value == "grid-guarded") {
        return Policy::GRID_GUARDED;
    }
    if (value == "balanced") {
        return Policy::BALANCED;
    }
    return Policy::FRESHNESS_FIRST;
}

libgnss::PPPProcessor::PPPConfig::ClasCorrectionApplicationPolicy parseClasCorrectionApplicationPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasCorrectionApplicationPolicy;
    if (value == "orbit-clock-bias") {
        return Policy::ORBIT_CLOCK_BIAS;
    }
    if (value == "orbit-clock-only") {
        return Policy::ORBIT_CLOCK_ONLY;
    }
    return Policy::FULL_OSR;
}

libgnss::PPPProcessor::PPPConfig::ClasPhaseContinuityPolicy parseClasPhaseContinuityPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasPhaseContinuityPolicy;
    if (value == "sis-continuity-only") {
        return Policy::SIS_CONTINUITY_ONLY;
    }
    if (value == "repair-only") {
        return Policy::REPAIR_ONLY;
    }
    if (value == "raw-phase-bias") {
        return Policy::RAW_PHASE_BIAS;
    }
    if (value == "no-phase-bias") {
        return Policy::NO_PHASE_BIAS;
    }
    return Policy::FULL_REPAIR;
}

libgnss::PPPProcessor::PPPConfig::ClasPhaseBiasValuePolicy parseClasPhaseBiasValuePolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasPhaseBiasValuePolicy;
    if (value == "phase-bias-only") {
        return Policy::PHASE_BIAS_ONLY;
    }
    if (value == "compensation-only") {
        return Policy::COMPENSATION_ONLY;
    }
    return Policy::FULL;
}

libgnss::PPPProcessor::PPPConfig::ClasPhaseBiasReferenceTimePolicy
parseClasPhaseBiasReferenceTimePolicy(const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasPhaseBiasReferenceTimePolicy;
    if (value == "clock-reference") {
        return Policy::CLOCK_REFERENCE;
    }
    if (value == "observation-epoch") {
        return Policy::OBSERVATION_EPOCH;
    }
    return Policy::PHASE_BIAS_REFERENCE;
}

libgnss::PPPProcessor::PPPConfig::ClasSsrTimingPolicy parseClasSsrTimingPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasSsrTimingPolicy;
    if (value == "clock-bound-phase-bias") {
        return Policy::CLOCK_BOUND_PHASE_BIAS;
    }
    if (value == "clock-bound-atmos-and-phase-bias") {
        return Policy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS;
    }
    return Policy::LAG_TOLERANT;
}

libgnss::PPPProcessor::PPPConfig::ClasExpandedValueConstructionPolicy parseClasExpandedValueConstructionPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasExpandedValueConstructionPolicy;
    if (value == "residual-only") {
        return Policy::RESIDUAL_ONLY;
    }
    if (value == "polynomial-only") {
        return Policy::POLYNOMIAL_ONLY;
    }
    return Policy::FULL_COMPOSED;
}

libgnss::PPPProcessor::PPPConfig::ClasExpandedResidualSamplingPolicy parseClasExpandedResidualSamplingPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasExpandedResidualSamplingPolicy;
    if (value == "indexed-only") {
        return Policy::INDEXED_ONLY;
    }
    if (value == "mean-only") {
        return Policy::MEAN_ONLY;
    }
    return Policy::INDEXED_OR_MEAN;
}

libgnss::PPPProcessor::PPPConfig::ClasSubtype12ValueConstructionPolicy
parseClasSubtype12ValueConstructionPolicy(const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasSubtype12ValueConstructionPolicy;
    if (value == "planar") {
        return Policy::PLANAR;
    }
    if (value == "offset-only") {
        return Policy::OFFSET_ONLY;
    }
    return Policy::FULL;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parseArguments(argc, argv);

        if (options.madocalib_bridge) {
            namespace madocalib = libgnss::external::madocalib;
            if (!madocalib::isAvailable()) {
                std::cerr << "Error: --madocalib-bridge requested, but this binary was not "
                             "built with -DMADOCALIB_PARITY_LINK=ON\n";
                return 1;
            }

            madocalib::PostposOptions bridge_options;
            bridge_options.obs_path = options.obs_path;
            bridge_options.nav_path = options.nav_path;
            bridge_options.out_path = options.out_path;
            bridge_options.config_path = options.madocalib_config_path;
            if (bridge_options.config_path.empty() && options.madocalib_profile != "ppp") {
                const std::string filename =
                    options.madocalib_profile == "pppar"
                        ? "sample_pppar.conf"
                        : "sample_pppar_iono.conf";
                bridge_options.config_path =
                    (std::filesystem::path(madocalib::defaultRootDir()) /
                     "app/consapp/rnx2rtkp/gcc_mingw" / filename).string();
            }
            bridge_options.antenna_path = options.antex_path;
            bridge_options.start_time = options.madocalib_start_time;
            bridge_options.end_time = options.madocalib_end_time;
            bridge_options.time_interval_seconds =
                options.madocalib_time_interval_seconds;
            bridge_options.trace_level = options.madocalib_trace_level;
            bridge_options.trace_ar = options.madocalib_trace_ar;
            if (!options.sp3_path.empty()) {
                bridge_options.auxiliary_input_paths.push_back(options.sp3_path);
            }
            if (!options.clk_path.empty()) {
                bridge_options.auxiliary_input_paths.push_back(options.clk_path);
            }
            if (!options.ssr_path.empty()) {
                bridge_options.auxiliary_input_paths.push_back(options.ssr_path);
            }
            for (const std::string& l6_path : options.madocalib_l6_paths) {
                bridge_options.auxiliary_input_paths.push_back(l6_path);
            }
            bridge_options.mdciono_paths = options.madocalib_mdciono_paths;

            const std::filesystem::path output_path(options.out_path);
            if (output_path.has_parent_path()) {
                std::filesystem::create_directories(output_path.parent_path());
            }

            std::string bridge_error;
            const int bridge_status =
                madocalib::runPostpos(bridge_options, &bridge_error);
            if (bridge_status != 0) {
                std::cerr << "Error: " << bridge_error << "\n";
                return 1;
            }
            if (!std::filesystem::is_regular_file(options.out_path)) {
                std::cerr << "Error: MADOCALIB bridge completed without output file: "
                          << options.out_path << "\n";
                return 1;
            }

            if (!options.summary_json_path.empty()) {
                const std::filesystem::path summary_path(options.summary_json_path);
                if (summary_path.has_parent_path()) {
                    std::filesystem::create_directories(summary_path.parent_path());
                }
                std::ofstream summary(summary_path);
                if (!summary.is_open()) {
                    std::cerr << "Error: failed to write summary JSON: "
                              << options.summary_json_path << "\n";
                    return 1;
                }
                summary << "{\n"
                        << "  \"obs\": \"" << jsonEscape(options.obs_path) << "\",\n"
                        << "  \"nav\": "
                        << (options.nav_path.empty() ?
                                "null" :
                                ("\"" + jsonEscape(options.nav_path) + "\""))
                        << ",\n"
                        << "  \"sp3\": "
                        << (options.sp3_path.empty() ?
                                "null" :
                                ("\"" + jsonEscape(options.sp3_path) + "\""))
                        << ",\n"
                        << "  \"clk\": "
                        << (options.clk_path.empty() ?
                                "null" :
                                ("\"" + jsonEscape(options.clk_path) + "\""))
                        << ",\n"
                        << "  \"ssr\": "
                        << (options.ssr_path.empty() ?
                                "null" :
                                ("\"" + jsonEscape(options.ssr_path) + "\""))
                        << ",\n"
                        << "  \"out\": \"" << jsonEscape(options.out_path) << "\",\n"
                        << "  \"madocalib_bridge\": true,\n"
                        << "  \"madocalib_bridge_status\": " << bridge_status << ",\n"
                        << "  \"madocalib_profile\": \""
                        << jsonEscape(options.madocalib_profile) << "\",\n"
                        << "  \"madocalib_config\": "
                        << (options.madocalib_config_path.empty() ?
                                "null" :
                                ("\"" + jsonEscape(options.madocalib_config_path) + "\""))
                        << ",\n"
                        << "  \"madocalib_start\": "
                        << (options.madocalib_start_time.empty() ?
                                "null" :
                                ("\"" + jsonEscape(options.madocalib_start_time) + "\""))
                        << ",\n"
                        << "  \"madocalib_end\": "
                        << (options.madocalib_end_time.empty() ?
                                "null" :
                                ("\"" + jsonEscape(options.madocalib_end_time) + "\""))
                        << ",\n"
                        << "  \"madocalib_time_interval_seconds\": "
                        << options.madocalib_time_interval_seconds << ",\n"
                        << "  \"madocalib_trace_ar\": "
                        << (options.madocalib_trace_ar ? "true" : "false") << ",\n"
                        << "  \"madocalib_l6_inputs\": [";
                bool first_l6 = true;
                for (const std::string& l6_path : options.madocalib_l6_paths) {
                    if (!first_l6) {
                        summary << ", ";
                    }
                    summary << "\"" << jsonEscape(l6_path) << "\"";
                    first_l6 = false;
                }
                summary << "]\n"
                        << "}\n";
            }

            if (!options.quiet) {
                std::cout << "MADOCALIB bridge: delegated run to linked MADOCALIB postpos()\n";
                std::cout << "  output: " << options.out_path << "\n";
            }
            return 0;
        }

        libgnss::io::RINEXReader obs_reader;
        const auto& ppp_env_overrides = libgnss::pppEnvOverrides();
        const bool clas_mrtklib_profile =
            options.use_clas_osr_filter && options.kinematic_mode &&
            options.use_dynamics_model && !options.low_dynamics_mode &&
            ppp_env_overrides.clas_mrtklib_float_parity;
        const bool clas_zd_parity_profile =
            options.use_clas_osr_filter && ppp_env_overrides.clas_dd_filter &&
            ppp_env_overrides.clas_code_row_bias_identity;
        if (options.ar_method == "per-freq" || clas_mrtklib_profile ||
            clas_zd_parity_profile) {
            obs_reader.setPreserveAdditionalFrequencyBands(true);
        }
        if (!options.madoca_l6_paths.empty() &&
            !ppp_env_overrides.qzss_prefer_l1l_present) {
            obs_reader.setQzssL1Preference(true);
        }
        if (!options.madoca_l6_paths.empty() &&
            ppp_env_overrides.madoca_qzss_l5) {
            obs_reader.setQzssSecondaryL5Preference(true);
        }
        if (options.use_clas_osr_filter && options.kinematic_mode &&
            options.use_dynamics_model && !options.low_dynamics_mode) {
            // MRTKLIB nf=3 assigns QZSS C5Q to pntpos()'s second code slot.
            obs_reader.setQzssSecondaryL5Preference(true);
        }
        if (!obs_reader.open(options.obs_path)) {
            std::cerr << "Error: failed to open observation file: " << options.obs_path << "\n";
            return 1;
        }

        libgnss::io::RINEXReader::RINEXHeader obs_header;
        if (!obs_reader.readHeader(obs_header)) {
            std::cerr << "Error: failed to read observation header: " << options.obs_path << "\n";
            return 1;
        }
        if (!options.blq_path.empty() &&
            options.ocean_loading_station_name.empty() &&
            obs_header.marker_name.find_first_not_of(' ') == std::string::npos) {
            std::cerr << "Error: --blq requires a marker name in the observation header or "
                         "--ocean-loading-station\n";
            return 1;
        }

        libgnss::NavigationData nav_data;
        if (!options.nav_path.empty()) {
            libgnss::io::RINEXReader nav_reader;
            if (!nav_reader.open(options.nav_path)) {
                std::cerr << "Error: failed to open navigation file: " << options.nav_path << "\n";
                return 1;
            }
            if (!nav_reader.readNavigationData(nav_data)) {
                std::cerr << "Error: failed to read navigation data: " << options.nav_path << "\n";
                return 1;
            }
        }

        libgnss::PPPProcessor::PPPConfig ppp_config;
        ppp_config.orbit_file_path = options.sp3_path;
        ppp_config.clock_file_path = options.clk_path;
        ppp_config.use_precise_orbits = !options.sp3_path.empty();
        ppp_config.use_precise_clocks = !options.clk_path.empty();
        ppp_config.ssr_file_path = options.ssr_path;
        ppp_config.use_ssr_corrections =
            !options.ssr_path.empty() || !options.ssr_rtcm_path.empty() ||
            !options.madoca_l6_paths.empty();
        ppp_config.ionex_file_path = options.ionex_path;
        ppp_config.dcb_file_path = options.dcb_path;
        ppp_config.antex_file_path = options.antex_path;
        ppp_config.ocean_loading_file_path = options.blq_path;
        ppp_config.estimate_troposphere = options.estimate_troposphere;
        const bool per_frequency_ar = options.ar_method == "per-freq";
        ppp_config.estimate_ionosphere = options.estimate_ionosphere || per_frequency_ar;
        ppp_config.use_ionosphere_free = options.use_ionosphere_free && !per_frequency_ar;
        ppp_config.apply_madoca_l6d_ionosphere =
            !options.madoca_l6d_paths.empty();
        ppp_config.use_clas_osr_filter = options.use_clas_osr_filter;
        ppp_config.clas_epoch_policy =
            parseClasEpochPolicy(options.clas_epoch_policy);
        ppp_config.clas_correction_application_policy =
            parseClasCorrectionApplicationPolicy(options.clas_osr_application);
        ppp_config.clas_phase_continuity_policy =
            parseClasPhaseContinuityPolicy(options.clas_phase_continuity);
        ppp_config.clas_phase_bias_value_policy =
            parseClasPhaseBiasValuePolicy(options.clas_phase_bias_values);
        ppp_config.clas_phase_bias_reference_time_policy =
            parseClasPhaseBiasReferenceTimePolicy(options.clas_phase_bias_reference_time);
        ppp_config.clas_ssr_timing_policy =
            parseClasSsrTimingPolicy(options.clas_ssr_timing);
        ppp_config.clas_expanded_value_construction_policy =
            parseClasExpandedValueConstructionPolicy(options.clas_expanded_values);
        ppp_config.clas_subtype12_value_construction_policy =
            parseClasSubtype12ValueConstructionPolicy(options.clas_subtype12_values);
        ppp_config.clas_expanded_residual_sampling_policy =
            parseClasExpandedResidualSamplingPolicy(options.clas_residual_sampling);
        ppp_config.clas_atmos_selection_policy =
            parseClasAtmosSelectionPolicy(options.clas_atmos_selection);
        ppp_config.clas_atmos_stale_after_seconds =
            options.clas_atmos_stale_after_seconds;
        ppp_config.kinematic_mode = options.kinematic_mode;
        ppp_config.low_dynamics_mode = options.low_dynamics_mode;
        if (options.use_dynamics_model) {
            ppp_config.use_dynamics_model = true;
            ppp_config.reset_kinematic_position_to_spp_each_epoch = false;
            ppp_config.reset_clock_to_spp_each_epoch = false;
            ppp_config.process_noise_position = 0.04;
            ppp_config.process_noise_velocity = 0.01;
            // Velocity is unobserved by H (only reached through the F
            // pos+=vel*dt coupling), so a loose prior here inflates
            // position uncertainty every predict step via F*P*F' long
            // before velocity itself converges. The previous default
            // (100 m^2/s^2, i.e. 10 m/s std) made the startup transient
            // (before ambiguities/troposphere converge) swing tens to
            // hundreds of meters in height before settling; 4.0 (2 m/s
            // std) is still loose enough to capture urban-drive dynamics
            // but keeps that transient much smaller.
            ppp_config.initial_velocity_variance = 4.0;
            // MRTKLIB literal-port track: the dynamics-model kinematic CLAS
            // path is the MRTKLIB-equivalence path, so it uses MRTKLIB's
            // varerr measurement variance model. The typed environment
            // override retains the exact-0 kill switch for A/B comparisons.
            ppp_config.clas_mrtklib_float_parity =
                ppp_env_overrides.clas_mrtklib_float_parity;
        }
        ppp_config.emit_solution_epoch_time = options.emit_epoch_time;
        ppp_config.apply_static_anchor_blend = options.apply_static_anchor_blend;
        if (!options.madoca_l6_paths.empty() &&
            ppp_env_overrides.madoca_early_window) {
            ppp_config.code_phase_error_ratio_l1 = 300.0;
            ppp_config.code_phase_error_ratio_l2 = 300.0;
        }
        if (!options.madoca_l6_paths.empty() && per_frequency_ar) {
            // MADOCALIB initializes each per-frequency ambiguity immediately
            // before ppp_res() and admits that epoch's carrier-phase row.  The
            // shared native default waits for one completed lifecycle update,
            // making the first usable MADOCA epoch code-only and shifting the
            // float-state trajectory by one epoch.
            ppp_config.phase_measurement_min_lock_count = 0;
        }
        ppp_config.enable_ambiguity_resolution = options.enable_ar;
        ppp_config.convergence_min_epochs = options.convergence_min_epochs;
        ppp_config.convergence_policy =
            options.convergence_policy == "local-enu"
                ? libgnss::ppp_shared::ConvergencePolicy::LOCAL_ENU_COMPONENTS
                : libgnss::ppp_shared::ConvergencePolicy::LEGACY_ECEF_3D;
        ppp_config.convergence_threshold_horizontal =
            options.convergence_threshold_horizontal;
        ppp_config.convergence_threshold_vertical =
            options.convergence_threshold_vertical;
        ppp_config.ar_ratio_threshold = options.ar_ratio_threshold;
        using ARMethod = libgnss::PPPProcessor::PPPConfig::ARMethod;
        if (options.ar_method == "iflc") {
            ppp_config.ar_method = ARMethod::DD_IFLC;
        } else if (options.ar_method == "wlnl") {
            ppp_config.ar_method = ARMethod::DD_WLNL;
        } else if (options.ar_method == "per-freq") {
            ppp_config.ar_method = ARMethod::DD_PER_FREQ;
        } else {
            argumentError("--ar-method must be one of: iflc, wlnl, per-freq", argv[0]);
        }
        // The MRTKLIB literal CLAS path resolves the uncombined L1/L2 state-DD
        // system directly (ddmat/resamb_LAMBDA).  In the native processor that
        // implementation is hosted by DD_WLNL even though it deliberately has
        // no WL-fix prerequisite.  Do not leave parity runs on the CLI default
        // DD_IFLC path when the wrapper omits --ar-method.
        if (ppp_config.clas_mrtklib_float_parity &&
            ppp_config.kinematic_mode && !ppp_config.low_dynamics_mode &&
            ppp_config.use_clas_osr_filter && ppp_config.use_dynamics_model) {
            ppp_config.ar_method = ARMethod::DD_WLNL;
        }
        ppp_config.use_iers_solid_tide = options.use_iers_solid_tide;
        ppp_config.use_iers_ocean_loading = options.use_iers_ocean_loading;
        ppp_config.eop_path = options.eop_c04_file;
        ppp_config.use_iers_pole_tide = options.use_iers_pole_tide;
        ppp_config.use_iers_sub_daily_eop = options.use_iers_sub_daily_eop;
        ppp_config.atm_tidal_loading_path = options.atm_tidal_loading_file;
        ppp_config.use_iers_atm_tidal_loading = options.use_iers_atm_tidal_loading;
        // Per-satellite ionosphere process noise. MADOCA uncombined (per-frequency,
        // est-stec) PPP has no external STEC constraint, so the per-satellite
        // ionosphere states are pinned only by the dual-frequency code-phase
        // combination. The shared 1e-3 m^2/s random walk is too loose for that
        // regime: the free iono states absorb measurement systematics that then
        // leak into the height estimate. Tightening to 1e-4 roughly halves the
        // 3D error on MIZU/ALIC. CLAS (use_clas_osr_filter) brings its own STEC
        // grid, so it keeps the shared default. An explicit --process-noise-iono
        // always wins.
        const bool madoca_per_freq_iono =
            ppp_config.estimate_ionosphere && !ppp_config.use_ionosphere_free &&
            !ppp_config.use_clas_osr_filter;
        if (options.process_noise_iono_set) {
            ppp_config.process_noise_ionosphere = options.process_noise_iono;
        } else if (madoca_per_freq_iono) {
            ppp_config.process_noise_ionosphere = 1e-4;
        }
        if (options.low_dynamics_mode) {
            ppp_config.reset_clock_to_spp_each_epoch = false;
            ppp_config.reset_kinematic_position_to_spp_each_epoch = false;
            ppp_config.use_dynamics_model = false;
            ppp_config.process_noise_position = 0.0;
            ppp_config.process_noise_velocity = 1e-8;
        }
        // GPS week for L6 binary decode (only needed when SSR is L6 format)
        if (obs_header.first_obs.week > 0) {
            ppp_config.l6_gps_week = obs_header.first_obs.week;
        }
        if (!options.madoca_materialization_dump_path.empty()) {
            if (options.madoca_l6_paths.empty()) {
                std::cerr << "Error: --madoca-materialization-dump requires --madoca-l6\n";
                return 1;
            }
            const std::filesystem::path dump_path(options.madoca_materialization_dump_path);
            if (!dump_path.parent_path().empty()) {
                std::filesystem::create_directories(dump_path.parent_path());
            }
            const int rows = libgnss::io::writeMadocaL6eMaterializationCsv(
                options.madoca_l6_paths,
                ppp_config.l6_gps_week,
                options.madoca_materialization_dump_path,
                ppp_env_overrides.madoca_ssr_replay);
            if (rows < 0) {
                std::cerr << "Error: failed to write MADOCA materialization dump: "
                          << options.madoca_materialization_dump_path << "\n";
                return 1;
            }
            if (rows == 0) {
                std::cerr << "Error: MADOCA materialization dump produced no rows\n";
                return 1;
            }
            if (options.madoca_materialization_dump_only) {
                if (!options.summary_json_path.empty()) {
                    const std::filesystem::path summary_path(options.summary_json_path);
                    if (!summary_path.parent_path().empty()) {
                        std::filesystem::create_directories(summary_path.parent_path());
                    }
                    std::ofstream summary(summary_path);
                    if (!summary.is_open()) {
                        std::cerr << "Error: failed to write summary JSON: "
                                  << options.summary_json_path << "\n";
                        return 1;
                    }
                    summary << "{\n"
                            << "  \"obs\": \"" << jsonEscape(options.obs_path) << "\",\n"
                            << "  \"nav\": \"" << jsonEscape(options.nav_path) << "\",\n"
                            << "  \"madoca_materialization_dump\": \""
                            << jsonEscape(options.madoca_materialization_dump_path) << "\",\n"
                            << "  \"madoca_materialization_dump_only\": true,\n"
                            << "  \"madoca_materialization_rows\": " << rows << ",\n"
                            << "  \"madoca_l6_inputs\": [";
                    bool first_l6 = true;
                    for (const std::string& l6_path : options.madoca_l6_paths) {
                        if (!first_l6) {
                            summary << ", ";
                        }
                        summary << "\"" << jsonEscape(l6_path) << "\"";
                        first_l6 = false;
                    }
                    summary << "]\n"
                            << "}\n";
                }
                if (!options.quiet) {
                    std::cout << "MADOCA materialization dump written.\n";
                    std::cout << "  rows: " << rows << "\n";
                    std::cout << "  output: " << options.madoca_materialization_dump_path << "\n";
                }
                return 0;
            }
        }
        ppp_config.approximate_position = obs_header.approximate_position;
        ppp_config.receiver_antenna_type =
            options.receiver_antenna_type.empty() ?
                obs_header.antenna_type :
                options.receiver_antenna_type;
        ppp_config.receiver_antenna_delta_enu = obs_header.antenna_delta;
        ppp_config.ocean_loading_station_name =
            options.ocean_loading_station_name.empty() ?
                obs_header.marker_name :
                options.ocean_loading_station_name;
        ppp_config.apply_ocean_loading = !options.blq_path.empty();

        libgnss::ProcessorConfig processor_config;
        processor_config.mode = libgnss::PositioningMode::PPP;
        processor_config.use_precise_orbits = ppp_config.use_precise_orbits;
        processor_config.use_precise_clocks = ppp_config.use_precise_clocks;
        processor_config.orbit_file_path = options.sp3_path;
        processor_config.clock_file_path = options.clk_path;
        if (!options.madoca_l6_paths.empty() && ppp_env_overrides.madoca_low_elev) {
            processor_config.elevation_mask = 10.0;
        }

        libgnss::PPPProcessor processor(ppp_config);
        if (!processor.initialize(processor_config)) {
            std::cerr << "Error: failed to initialize PPP processor\n";
            return 1;
        }
        if (!options.ssr_rtcm_path.empty() &&
            !processor.loadRTCMSSRProducts(
                options.ssr_rtcm_path, nav_data, options.ssr_step_seconds)) {
            std::cerr << "Error: failed to load RTCM SSR corrections: "
                      << options.ssr_rtcm_path << "\n";
            return 1;
        }
        if (!options.madoca_l6_paths.empty() &&
            !processor.loadMadocaL6Products(options.madoca_l6_paths)) {
            std::cerr << "Error: failed to load MADOCA L6E corrections\n";
            return 1;
        }
        const auto& madoca_l6d_input_paths =
            !options.madoca_l6d_paths.empty()
                ? options.madoca_l6d_paths
                : options.madoca_l6d_shadow_paths;
        if (!madoca_l6d_input_paths.empty()) {
            if (obs_header.first_obs.week <= 0 ||
                obs_header.approximate_position.norm() <= 1e3) {
                std::cerr << "Error: MADOCA L6D input requires RINEX first-observation "
                             "time and approximate receiver position\n";
                return 1;
            }
            const auto reference_tp = obs_header.first_obs.toSystemTime();
            const std::time_t reference_time =
                std::chrono::system_clock::to_time_t(reference_tp);
            std::tm reference_tm{};
#if defined(_WIN32)
            gmtime_s(&reference_tm, &reference_time);
#else
            gmtime_r(&reference_time, &reference_tm);
#endif
            const double reference_epoch[6] = {
                static_cast<double>(reference_tm.tm_year + 1900),
                static_cast<double>(reference_tm.tm_mon + 1),
                static_cast<double>(reference_tm.tm_mday),
                static_cast<double>(reference_tm.tm_hour),
                static_cast<double>(reference_tm.tm_min),
                static_cast<double>(reference_tm.tm_sec),
            };
            const double receiver_ecef[3] = {
                obs_header.approximate_position.x(),
                obs_header.approximate_position.y(),
                obs_header.approximate_position.z(),
            };
            if (!processor.loadMadocaL6dProducts(
                    madoca_l6d_input_paths, reference_epoch, receiver_ecef)) {
                std::cerr << "Error: failed to load MADOCA L6D corrections\n";
                return 1;
            }
        }

        libgnss::Solution solutions;
        libgnss::ObservationData observation_data;
        int processed_epochs = 0;
        int valid_solutions = 0;
        int ppp_float_solutions = 0;
        int ppp_fixed_solutions = 0;
        int fallback_solutions = 0;
        int atmospheric_trop_corrections = 0;
        int atmospheric_iono_corrections = 0;
        int ionex_corrections = 0;
        int dcb_corrections = 0;
        int clas_hybrid_fallback_epochs = 0;
        int madoca_l6d_shadow_snapshot_epochs = 0;
        int madoca_l6d_shadow_stale_epochs = 0;
        int madoca_l6d_shadow_matched_satellites = 0;
        double madoca_l6d_shadow_max_age_s = 0.0;
        int madoca_l6d_shadow_last_region = -1;
        int madoca_l6d_shadow_last_area = 0;
        int madoca_l6d_constraint_epochs = 0;
        int madoca_l6d_constraint_rows = 0;
        int madoca_l6d_constraint_position_gate_epochs = 0;
        std::map<std::string, int> clas_hybrid_fallback_reasons;
        double atmospheric_trop_meters = 0.0;
        double atmospheric_iono_meters = 0.0;
        double ionex_meters = 0.0;
        double dcb_meters = 0.0;
        // TEMP-DEBUG (remove before merge): skip obs epochs before a GPS TOW.
        const char* skip_until_env = std::getenv("GNSS_PPP_SKIP_UNTIL_TOW");
        const double skip_until_tow =
            skip_until_env != nullptr ? std::atof(skip_until_env) : -1.0;
        while ((options.max_epochs == 0 || processed_epochs < options.max_epochs) &&
               obs_reader.readObservationEpoch(observation_data)) {
            if (skip_until_tow >= 0.0 &&
                observation_data.time.tow < skip_until_tow) {
                continue;
            }
            if (obs_header.approximate_position.norm() > 0.0) {
                observation_data.receiver_position = obs_header.approximate_position;
            }

            const auto solution = processor.processEpoch(observation_data, nav_data);
            atmospheric_trop_corrections +=
                processor.getLastAppliedAtmosphericTroposphereCorrections();
            atmospheric_iono_corrections +=
                processor.getLastAppliedAtmosphericIonosphereCorrections();
            ionex_corrections += processor.getLastAppliedIonexCorrections();
            dcb_corrections += processor.getLastAppliedDcbCorrections();
            atmospheric_trop_meters +=
                processor.getLastAppliedAtmosphericTroposphereMeters();
            atmospheric_iono_meters +=
                processor.getLastAppliedAtmosphericIonosphereMeters();
            ionex_meters += processor.getLastAppliedIonexMeters();
            dcb_meters += processor.getLastAppliedDcbMeters();
            const auto& l6d_shadow = processor.getLastMadocaL6dShadowStatus();
            if (l6d_shadow.snapshot_available) {
                ++madoca_l6d_shadow_snapshot_epochs;
                madoca_l6d_shadow_matched_satellites += l6d_shadow.matched_satellites;
                madoca_l6d_shadow_max_age_s =
                    std::max(madoca_l6d_shadow_max_age_s, l6d_shadow.age_s);
                madoca_l6d_shadow_last_region = l6d_shadow.region_id;
                madoca_l6d_shadow_last_area = l6d_shadow.area_number;
            } else if (l6d_shadow.stale) {
                ++madoca_l6d_shadow_stale_epochs;
            }
            if (l6d_shadow.constraint_rows > 0) {
                ++madoca_l6d_constraint_epochs;
                madoca_l6d_constraint_rows +=
                    l6d_shadow.constraint_rows;
            }
            if (l6d_shadow.constraint_skipped_position_covariance) {
                ++madoca_l6d_constraint_position_gate_epochs;
            }
            if (processor.getLastClasHybridFallbackUsed()) {
                ++clas_hybrid_fallback_epochs;
                ++clas_hybrid_fallback_reasons[processor.getLastClasHybridFallbackReason()];
            }
            processed_epochs++;
            if (solution.isValid()) {
                solutions.addSolution(solution);
                valid_solutions++;
                if (solution.status == libgnss::SolutionStatus::PPP_FLOAT) {
                    ppp_float_solutions++;
                } else if (solution.status == libgnss::SolutionStatus::PPP_FIXED) {
                    ppp_fixed_solutions++;
                } else {
                    fallback_solutions++;
                }
            }
        }

        if (solutions.isEmpty()) {
            std::cerr << "Error: PPP processing produced no valid solutions\n";
            return 1;
        }

        if (!solutions.writeToFile(options.out_path)) {
            std::cerr << "Error: failed to write solution file: " << options.out_path << "\n";
            return 1;
        }
        if (!options.kml_path.empty() && !solutions.writeKML(options.kml_path)) {
            std::cerr << "Error: failed to write KML file: " << options.kml_path << "\n";
            return 1;
        }

        const auto stats = processor.getStats();
        const auto& convergence = processor.getConvergenceTelemetry();
        const auto& ar_stage = processor.getARStageTelemetry();
        const double ppp_solution_rate =
            valid_solutions > 0 ?
                100.0 * static_cast<double>(ppp_float_solutions + ppp_fixed_solutions) /
                    static_cast<double>(valid_solutions) :
                0.0;
        if (!options.summary_json_path.empty()) {
            const std::filesystem::path summary_path(options.summary_json_path);
            if (!summary_path.parent_path().empty()) {
                std::filesystem::create_directories(summary_path.parent_path());
            }
            std::ofstream summary(summary_path);
            if (!summary.is_open()) {
                std::cerr << "Error: failed to write summary JSON: "
                          << options.summary_json_path << "\n";
                return 1;
            }
            summary << "{\n"
                    << "  \"obs\": \"" << jsonEscape(options.obs_path) << "\",\n"
                    << "  \"nav\": "
                    << (options.nav_path.empty() ? "null" : ("\"" + jsonEscape(options.nav_path) + "\""))
                    << ",\n"
                    << "  \"sp3\": "
                    << (options.sp3_path.empty() ? "null" : ("\"" + jsonEscape(options.sp3_path) + "\""))
                    << ",\n"
                    << "  \"clk\": "
                    << (options.clk_path.empty() ? "null" : ("\"" + jsonEscape(options.clk_path) + "\""))
                    << ",\n"
                    << "  \"ionex\": "
                    << (options.ionex_path.empty() ? "null" : ("\"" + jsonEscape(options.ionex_path) + "\""))
                    << ",\n"
                    << "  \"dcb\": "
                    << (options.dcb_path.empty() ? "null" : ("\"" + jsonEscape(options.dcb_path) + "\""))
                    << ",\n"
                    << "  \"antex\": "
                    << (options.antex_path.empty() ? "null" : ("\"" + jsonEscape(options.antex_path) + "\""))
                    << ",\n"
                    << "  \"receiver_antenna_type\": "
                    << (ppp_config.receiver_antenna_type.empty() ?
                            "null" :
                            ("\"" + jsonEscape(ppp_config.receiver_antenna_type) + "\""))
                    << ",\n"
                    << "  \"receiver_antenna_type_source\": \""
                    << (options.receiver_antenna_type.empty() ? "rinex-header" : "cli")
                    << "\",\n"
                    << "  \"madoca_materialization_dump\": "
                    << (options.madoca_materialization_dump_path.empty() ?
                            "null" :
                            ("\"" + jsonEscape(options.madoca_materialization_dump_path) + "\""))
                    << ",\n"
                    << "  \"out\": \"" << jsonEscape(options.out_path) << "\",\n"
                    << "  \"mode\": \"" << (options.kinematic_mode ? "kinematic" : "static") << "\",\n"
                    << "  \"low_dynamics\": " << (options.low_dynamics_mode ? "true" : "false") << ",\n"
                    << "  \"static_anchor_blend\": " << (options.apply_static_anchor_blend ? "true" : "false") << ",\n"
                    << "  \"clas_epoch_policy\": \"" << jsonEscape(options.clas_epoch_policy) << "\",\n"
                    << "  \"clas_osr_application\": \"" << jsonEscape(options.clas_osr_application) << "\",\n"
                    << "  \"clas_phase_continuity\": \"" << jsonEscape(options.clas_phase_continuity) << "\",\n"
                    << "  \"clas_phase_bias_values\": \"" << jsonEscape(options.clas_phase_bias_values) << "\",\n"
                    << "  \"clas_phase_bias_reference_time\": \"" << jsonEscape(options.clas_phase_bias_reference_time) << "\",\n"
                    << "  \"clas_ssr_timing\": \"" << jsonEscape(options.clas_ssr_timing) << "\",\n"
                    << "  \"clas_expanded_values\": \"" << jsonEscape(options.clas_expanded_values) << "\",\n"
                    << "  \"clas_subtype12_values\": \"" << jsonEscape(options.clas_subtype12_values) << "\",\n"
                    << "  \"clas_residual_sampling\": \"" << jsonEscape(options.clas_residual_sampling) << "\",\n"
                    << "  \"clas_atmos_selection\": \"" << jsonEscape(options.clas_atmos_selection) << "\",\n"
                    << "  \"clas_atmos_stale_after_seconds\": " << options.clas_atmos_stale_after_seconds << ",\n"
                    << "  \"ambiguity_resolution_enabled\": " << (options.enable_ar ? "true" : "false") << ",\n"
                    << "  \"ar_method\": \"" << jsonEscape(options.ar_method) << "\",\n"
                    << "  \"estimate_ionosphere\": "
                    << (ppp_config.estimate_ionosphere ? "true" : "false") << ",\n"
                    << "  \"use_ionosphere_free\": "
                    << (ppp_config.use_ionosphere_free ? "true" : "false") << ",\n"
                    << "  \"phase_measurement_min_lock_count\": "
                    << ppp_config.phase_measurement_min_lock_count << ",\n"
                    << "  \"ar_ratio_threshold\": " << options.ar_ratio_threshold << ",\n"
                    << "  \"processed_epochs\": " << processed_epochs << ",\n"
                    << "  \"valid_solutions\": " << valid_solutions << ",\n"
                    << "  \"ppp_float_solutions\": " << ppp_float_solutions << ",\n"
                    << "  \"ppp_fixed_solutions\": " << ppp_fixed_solutions << ",\n"
                    << "  \"fallback_solutions\": " << fallback_solutions << ",\n"
                    << "  \"clas_hybrid_fallback_epochs\": " << clas_hybrid_fallback_epochs << ",\n"
                    << "  \"ppp_solution_rate_pct\": " << ppp_solution_rate << ",\n"
                    << "  \"ssr_corrections_enabled\": "
                    << ((!options.ssr_path.empty() || !options.ssr_rtcm_path.empty() ||
                         !options.madoca_l6_paths.empty()) ? "true" : "false") << ",\n"
                    << "  \"atmospheric_trop_corrections\": " << atmospheric_trop_corrections << ",\n"
                    << "  \"atmospheric_trop_meters\": " << atmospheric_trop_meters << ",\n"
                    << "  \"atmospheric_iono_corrections\": " << atmospheric_iono_corrections << ",\n"
                    << "  \"atmospheric_iono_meters\": " << atmospheric_iono_meters << ",\n"
                    << "  \"ionex_loaded\": " << (processor.hasLoadedIONEXProducts() ? "true" : "false") << ",\n"
                    << "  \"ionex_maps\": " << processor.getLoadedIONEXMapCount() << ",\n"
                    << "  \"ionex_corrections\": " << ionex_corrections << ",\n"
                    << "  \"ionex_meters\": " << ionex_meters << ",\n"
                    << "  \"dcb_loaded\": " << (processor.hasLoadedDCBProducts() ? "true" : "false") << ",\n"
                    << "  \"dcb_entries\": " << processor.getLoadedDCBEntryCount() << ",\n"
                    << "  \"dcb_corrections\": " << dcb_corrections << ",\n"
                    << "  \"dcb_meters\": " << dcb_meters << ",\n"
                    << "  \"madoca_l6d_shadow_loaded\": "
                    << (processor.hasLoadedMadocaL6dProducts() ? "true" : "false") << ",\n"
                    << "  \"madoca_l6d_shadow_snapshot_epochs\": "
                    << madoca_l6d_shadow_snapshot_epochs << ",\n"
                    << "  \"madoca_l6d_shadow_stale_epochs\": "
                    << madoca_l6d_shadow_stale_epochs << ",\n"
                    << "  \"madoca_l6d_shadow_matched_satellites\": "
                    << madoca_l6d_shadow_matched_satellites << ",\n"
                    << "  \"madoca_l6d_shadow_max_age_s\": "
                    << madoca_l6d_shadow_max_age_s << ",\n"
                    << "  \"madoca_l6d_shadow_last_region\": "
                    << madoca_l6d_shadow_last_region << ",\n"
                    << "  \"madoca_l6d_shadow_last_area\": "
                    << madoca_l6d_shadow_last_area << ",\n"
                    << "  \"madoca_l6d_apply_enabled\": "
                    << (ppp_config.apply_madoca_l6d_ionosphere ? "true" : "false")
                    << ",\n"
                    << "  \"madoca_l6d_constraint_epochs\": "
                    << madoca_l6d_constraint_epochs << ",\n"
                    << "  \"madoca_l6d_constraint_rows\": "
                    << madoca_l6d_constraint_rows << ",\n"
                    << "  \"madoca_l6d_constraint_position_gate_epochs\": "
                    << madoca_l6d_constraint_position_gate_epochs << ",\n"
                    << "  \"clas_hybrid_fallback_reasons\": {";
            bool first_reason = true;
            for (const auto& [reason, count] : clas_hybrid_fallback_reasons) {
                if (!first_reason) {
                    summary << ", ";
                }
                summary << "\"" << jsonEscape(reason) << "\": " << count;
                first_reason = false;
            }
            summary << "},\n"
                    << "  \"converged\": " << (processor.hasConverged() ? "true" : "false") << ",\n"
                    << "  \"convergence_time_s\": " << processor.getConvergenceTime() << ",\n"
                    << "  \"convergence_gate_reason\": \"" << jsonEscape(convergence.gate_reason) << "\",\n"
                    << "  \"convergence_evaluated_epochs\": " << convergence.evaluated_epochs << ",\n"
                    << "  \"convergence_insufficient_history_epochs\": "
                    << convergence.insufficient_history_epochs << ",\n"
                    << "  \"convergence_unstable_position_epochs\": "
                    << convergence.unstable_position_epochs << ",\n"
                    << "  \"convergence_unstable_horizontal_epochs\": "
                    << convergence.unstable_horizontal_epochs << ",\n"
                    << "  \"convergence_unstable_vertical_epochs\": "
                    << convergence.unstable_vertical_epochs << ",\n"
                    << "  \"convergence_window_epochs\": " << convergence.window_epochs << ",\n"
                    << "  \"convergence_required_window_epochs\": "
                    << convergence.required_window_epochs << ",\n"
                    << "  \"convergence_max_position_deviation_m\": "
                    << convergence.max_position_deviation_m << ",\n"
                    << "  \"convergence_max_horizontal_position_deviation_m\": "
                    << convergence.max_horizontal_position_deviation_m << ",\n"
                    << "  \"convergence_max_vertical_position_deviation_m\": "
                    << convergence.max_vertical_position_deviation_m << ",\n"
                    << "  \"convergence_position_deviation_threshold_m\": "
                    << convergence.position_deviation_threshold_m << ",\n"
                    << "  \"convergence_horizontal_position_deviation_threshold_m\": "
                    << convergence.horizontal_position_deviation_threshold_m << ",\n"
                    << "  \"convergence_vertical_position_deviation_threshold_m\": "
                    << convergence.vertical_position_deviation_threshold_m << ",\n"
                    << "  \"convergence_policy\": \"" << convergence.policy << "\",\n"
                    << "  \"ar_stage_last\": \"" << jsonEscape(ar_stage.last_stage) << "\",\n"
                    << "  \"ar_per_frequency_attempts\": "
                    << ar_stage.per_frequency_attempts << ",\n"
                    << "  \"ar_insufficient_satellite_epochs\": "
                    << ar_stage.insufficient_satellite_epochs << ",\n"
                    << "  \"ar_no_wide_lane_epochs\": "
                    << ar_stage.no_wide_lane_epochs << ",\n"
                    << "  \"ar_wide_lane_only_epochs\": "
                    << ar_stage.wide_lane_only_epochs << ",\n"
                    << "  \"ar_n1_lambda_failure_epochs\": "
                    << ar_stage.n1_lambda_failure_epochs << ",\n"
                    << "  \"ar_n1_ratio_rejection_epochs\": "
                    << ar_stage.n1_ratio_rejection_epochs << ",\n"
                    << "  \"ar_n1_fixed_epochs\": " << ar_stage.n1_fixed_epochs << ",\n"
                    << "  \"ar_last_satellite_candidates\": "
                    << ar_stage.last_satellite_candidates << ",\n"
                    << "  \"ar_last_wide_lane_pairs\": "
                    << ar_stage.last_wide_lane_pairs << ",\n"
                    << "  \"ar_last_n1_candidates\": "
                    << ar_stage.last_n1_candidates << ",\n"
                    << "  \"ar_last_n1_ratio\": " << ar_stage.last_n1_ratio << ",\n"
                    << "  \"average_processing_time_ms\": " << stats.average_processing_time_ms << "\n"
                    << "}\n";
        }

        if (!options.quiet) {
            std::cout << "PPP summary:\n";
            std::cout << "  processed epochs: " << processed_epochs << "\n";
            std::cout << "  valid solutions: " << valid_solutions << "\n";
            std::cout << "  PPP float solutions: " << ppp_float_solutions << "\n";
            std::cout << "  PPP fixed solutions: " << ppp_fixed_solutions << "\n";
            std::cout << "  fallback solutions: " << fallback_solutions << "\n";
            std::cout << "  CLAS epoch policy: " << options.clas_epoch_policy << "\n";
            std::cout << "  CLAS OSR application: " << options.clas_osr_application << "\n";
            std::cout << "  CLAS phase continuity: " << options.clas_phase_continuity << "\n";
            std::cout << "  CLAS phase-bias values: " << options.clas_phase_bias_values << "\n";
            std::cout << "  CLAS phase-bias reference time: "
                      << options.clas_phase_bias_reference_time << "\n";
            std::cout << "  CLAS SSR timing: " << options.clas_ssr_timing << "\n";
            std::cout << "  CLAS expanded values: " << options.clas_expanded_values << "\n";
            std::cout << "  CLAS subtype-12 values: " << options.clas_subtype12_values << "\n";
            std::cout << "  CLAS residual sampling: " << options.clas_residual_sampling << "\n";
            std::cout << "  mode: " << (options.kinematic_mode ? "kinematic" : "static") << "\n";
            std::cout << "  low dynamics: " << (options.low_dynamics_mode ? "on" : "off") << "\n";
            std::cout << "  static anchor blend: "
                      << (options.apply_static_anchor_blend ? "on" : "off") << "\n";
            std::cout << "  CLAS atmosphere selection: " << options.clas_atmos_selection << "\n";
            std::cout << "  CLAS stale-after (s): " << options.clas_atmos_stale_after_seconds << "\n";
            if (clas_hybrid_fallback_epochs > 0) {
                std::cout << "  CLAS hybrid fallback epochs: "
                          << clas_hybrid_fallback_epochs << "\n";
            }
            std::cout << "  ambiguity resolution: " << (options.enable_ar ? "on" : "off") << "\n";
            std::cout << "  SSR corrections: "
                      << ((options.ssr_path.empty() && options.ssr_rtcm_path.empty() &&
                           options.madoca_l6_paths.empty()) ? "off" : "on")
                      << "\n";
            if (!options.madoca_materialization_dump_path.empty()) {
                std::cout << "  MADOCA materialization dump: "
                          << options.madoca_materialization_dump_path << "\n";
            }
            if (processor.hasLoadedMadocaL6dProducts()) {
                std::cout << "  MADOCA L6D snapshot epochs: "
                          << madoca_l6d_shadow_snapshot_epochs << "\n";
                std::cout << "  MADOCA L6D stale epochs: "
                          << madoca_l6d_shadow_stale_epochs << "\n";
                std::cout << "  MADOCA L6D matched satellites: "
                          << madoca_l6d_shadow_matched_satellites << "\n";
                if (ppp_config.apply_madoca_l6d_ionosphere) {
                    std::cout << "  MADOCA L6D constraint epochs: "
                              << madoca_l6d_constraint_epochs << "\n";
                    std::cout << "  MADOCA L6D constraint rows: "
                              << madoca_l6d_constraint_rows << "\n";
                }
            }
            if (atmospheric_trop_corrections > 0 || atmospheric_iono_corrections > 0) {
                std::cout << "  atmospheric trop corrections: "
                          << atmospheric_trop_corrections << "\n";
                std::cout << "  atmospheric trop meters: "
                          << atmospheric_trop_meters << "\n";
                std::cout << "  atmospheric ionosphere corrections: "
                          << atmospheric_iono_corrections << "\n";
                std::cout << "  atmospheric ionosphere meters: "
                          << atmospheric_iono_meters << "\n";
            }
            if (processor.hasLoadedIONEXProducts()) {
                std::cout << "  ionex maps: "
                          << processor.getLoadedIONEXMapCount() << "\n";
                std::cout << "  ionex corrections: "
                          << ionex_corrections << "\n";
                std::cout << "  ionex meters: "
                          << ionex_meters << "\n";
            }
            if (processor.hasLoadedDCBProducts()) {
                std::cout << "  dcb entries: "
                          << processor.getLoadedDCBEntryCount() << "\n";
                std::cout << "  dcb corrections: "
                          << dcb_corrections << "\n";
                std::cout << "  dcb meters: "
                          << dcb_meters << "\n";
            }
            if (options.enable_ar) {
                std::cout << "  AR ratio threshold: " << options.ar_ratio_threshold << "\n";
            }
            if (valid_solutions > 0) {
                std::cout << "  PPP solution rate (%): " << ppp_solution_rate << "\n";
            }
            std::cout << "  converged: " << (processor.hasConverged() ? "yes" : "no") << "\n";
            if (processor.hasConverged()) {
                std::cout << "  convergence time (s): " << processor.getConvergenceTime() << "\n";
            }
            std::cout << "  average processing time (ms): " << stats.average_processing_time_ms << "\n";
            std::cout << "  output: " << options.out_path << "\n";
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
