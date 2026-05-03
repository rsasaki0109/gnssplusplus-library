#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <libgnss++/algorithms/madoca_core.hpp>
#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/external/madocalib_bridge.hpp>
#include <libgnss++/io/rinex.hpp>

namespace {

struct Options {
    std::string obs_path;
    std::vector<std::string> nav_paths;
    std::string sp3_path;
    std::string clk_path;
    std::string ssr_path;
    std::string ssr_rtcm_path;
    std::string ionex_path;
    std::string dcb_path;
    std::string antex_path;
    std::string blq_path;
    std::string ocean_loading_station_name;
    std::string out_path;
    std::string summary_json_path;
    std::string ppp_correction_log_path;
    std::string ppp_filter_log_path;
    std::string ppp_residual_log_path;
    std::string kml_path;
    int max_epochs = 0;
    double start_tow = -1.0;  // negative disables the gate
    int convergence_min_epochs = 20;
    int phase_measurement_min_lock_count = 1;
    bool enable_initial_phase_admission_warm_start = false;
    bool enable_all_frequency_initial_phase_admission_warm_start = false;
    int initial_phase_admission_warm_start_navsys_mask = 0;
    std::string initial_phase_admission_warm_start_satellites_arg;
    std::string initial_phase_admission_warm_start_frequency_indexes_arg;
    std::string initial_phase_admission_warm_start_satellite_frequency_pairs_arg;
    std::string phase_admission_excluded_satellite_frequency_pairs_arg;
    std::string phase_admission_excluded_before_satellite_frequency_pairs_arg;
    std::string phase_admission_residual_floor_satellite_frequency_pairs_arg;
    std::string exclude_known_bad_madoca_sats_preset;
    bool reset_phase_ambiguity_on_before_exclusion = false;
    double elevation_mask_deg = 15.0;
    double ssr_step_seconds = 1.0;
    bool enforce_ssr_orbit_iode = false;
    bool enforce_ssr_orbit_iode_admission_only = false;
    int ssr_orbit_iode_admission_gate_warmup_epochs = 0;
    std::string ar_method_arg;  // "dd-iflc" (default), "dd-wlnl", "dd-per-freq"
    bool estimate_troposphere = true;
    bool estimate_ionosphere = false;
    bool use_ionosphere_free = true;
    bool enable_per_frequency_phase_bias_states = false;
    bool enable_extra_band_observations = false;
    bool initialize_phase_ambiguity_with_ionosphere_state = false;
    bool enable_ppp_outlier_detection = true;
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
    bool clas_kinematic_position_reseed = false;
    bool clas_kinematic_position_reseed_set = false;
    double clas_kinematic_position_reseed_variance = -1.0;
    bool madocalib_bridge = false;
    std::string madocalib_config_path;
    std::vector<std::string> madoca_l6e_paths;
    std::vector<std::string> madoca_l6d_paths;
    int madoca_navsys_mask = 0;
    std::vector<std::string> madocalib_l6_paths;
    std::vector<std::string> madocalib_mdciono_paths;
    std::string madocalib_start_time;
    std::string madocalib_end_time;
    double madocalib_time_interval_seconds = 0.0;
    int madocalib_trace_level = 0;
    bool kinematic_mode = false;
    bool low_dynamics_mode = false;
    bool enable_ar = false;
    double ar_ratio_threshold = 3.0;
    double kinematic_preconvergence_phase_residual_floor_m = 200.0;
    int filter_iterations = 0;  // 0 keeps PPPConfig default
    double initial_ionosphere_variance = -1.0;  // negative keeps default
    double initial_troposphere_variance = -1.0;
    double code_phase_error_ratio_l1 = -1.0;
    double code_phase_error_ratio_l2 = -1.0;
    bool enable_ppp_holdamb = false;
    double ppp_holdamb_innovation_gate_m = 0.0;
    bool quiet = false;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --obs <rover.obs> [options]\n"
        << "Options:\n"
        << "  --nav <nav.rnx>           Optional broadcast navigation file; repeat to merge multiple files\n"
        << "  --sp3 <orbit.sp3>        Precise orbit file\n"
        << "  --clk <clock.clk>        Precise clock file\n"
        << "  --ssr <corrections.csv>  Simple SSR orbit/clock corrections CSV\n"
        << "  --ssr-rtcm <file|ntrip://...|serial://...|tcp://...>\n"
        << "                          RTCM SSR source converted/read for PPP use\n"
        << "  --madoca-l6e <file>    Native MADOCA L6E correction file; repeat for multiple channels\n"
        << "  --madoca-l6d <file>    Native MADOCA L6D ionosphere file; repeat for multiple channels\n"
        << "                          Requires --madoca-l6e and an approximate receiver position in RINEX header\n"
        << "  --madoca-navsys <mask>\n"
        << "                          Native MADOCA RTKLIB navsys mask (0=all, 29 excludes BDS, 61 includes BDS)\n"
        << "  --ionex <maps.ionex>     Optional IONEX TEC map product\n"
        << "  --dcb <bias.bsx>         Optional DCB / Bias-SINEX product\n"
        << "  --antex <antennas.atx>   Optional ANTEX file for receiver PCO/PCV and satellite PCO\n"
        << "  --blq <station.blq>      Optional BLQ ocean loading coefficient file\n"
        << "  --ocean-loading-station <name>\n"
        << "                          Station name to select from the BLQ file\n"
        << "  --ssr-step-seconds <seconds>\n"
        << "                          Sampling step for RTCM SSR conversion (default: 1.0)\n"
        << "  --enforce-ssr-orbit-iode\n"
        << "                          Require SSR orbit IODE to match a broadcast ephemeris\n"
        << "  --enforce-ssr-orbit-iode-admission-only\n"
        << "                          Gate admission on SSR orbit IODE match without\n"
        << "                          swapping broadcast-state selection for range modeling\n"
        << "  --ssr-orbit-iode-admission-gate-warmup-epochs <N>\n"
        << "                          Skip the admission-only IODE gate for the first N\n"
        << "                          epochs so stale-CSSR-IODE observations can seed the\n"
        << "                          filter before the gate activates (default: 0)\n"
        << "  --ar-method <dd-iflc|dd-wlnl|dd-per-freq|dd-madoca-cascaded>\n"
        << "                          PPP ambiguity resolution method (default: dd-iflc).\n"
        << "                          dd-madoca-cascaded adds MADOCALIB-style Kalman\n"
        << "                          WL state-constraint injection before N1 LAMBDA\n"
        << "  --out <solution.pos>     Output position file (required)\n"
        << "  --summary-json <summary.json>\n"
        << "                          Optional machine-readable run summary\n"
        << "  --ppp-correction-log <log.csv>\n"
        << "                          Optional per-epoch PPP correction diagnostics CSV\n"
        << "  --ppp-filter-log <log.csv>\n"
        << "                          Optional per-iteration PPP filter diagnostics CSV\n"
        << "  --ppp-residual-log <log.csv>\n"
        << "                          Optional per-row PPP residual diagnostics CSV\n"
        << "  --kml <solution.kml>     Optional KML output\n"
        << "  --max-epochs <count>     Limit processed epochs (default: all)\n"
        << "  --start-tow <seconds>    Drop observation epochs with TOW < this value\n"
        << "  --convergence-min-epochs <count>\n"
        << "                          Minimum epochs before PPP convergence/AR checks (default: 20)\n"
        << "  --phase-measurement-min-lock-count <count>\n"
        << "                          Minimum lock count before SSR/MADOCA phase rows are admitted (default: 1; 0 admits immediately)\n"
        << "  --enable-initial-phase-admission-warm-start\n"
        << "                          Admit low-residual initial primary SSR/MADOCA phase rows before the lock gate\n"
        << "  --disable-initial-phase-admission-warm-start\n"
        << "                          Disable initial phase admission warm start (default)\n"
        << "  --enable-all-frequency-initial-phase-admission-warm-start\n"
        << "                          Admit low-residual initial SSR/MADOCA phase rows on all frequencies before the lock gate\n"
        << "  --disable-all-frequency-initial-phase-admission-warm-start\n"
        << "                          Disable all-frequency initial phase admission warm start (default)\n"
        << "  --initial-phase-admission-warm-start-navsys <mask>\n"
        << "                          Limit initial phase warm-start rows to a RTKLIB navsys mask (0=all; default)\n"
        << "  --initial-phase-admission-warm-start-sats <csv>\n"
        << "                          Limit initial phase warm-start rows to satellite IDs (e.g. R09,C30; default all)\n"
        << "  --initial-phase-admission-warm-start-frequency-indexes <csv>\n"
        << "                          Limit initial phase warm-start rows to frequency indexes (e.g. 0,1; default all)\n"
        << "  --initial-phase-admission-warm-start-sat-frequency-pairs <csv>\n"
        << "                          Limit initial phase warm-start rows to satellite:frequency pairs (e.g. R09:0,C30:1; default all)\n"
        << "  --phase-admission-exclude-sat-frequency-pairs <csv>\n"
        << "                          Exclude diagnostic carrier-phase rows by satellite:frequency pairs (e.g. E31:0,C29:1; default none)\n"
        << "  --phase-admission-exclude-sat-frequency-pairs-before <csv>\n"
        << "                          Exclude diagnostic carrier-phase rows before GPS week/TOW (e.g. E31:0:2360:173610; default none)\n"
        << "  --phase-admission-residual-floor-sat-frequency-pairs <csv>\n"
        << "                          Override phase residual floor by satellite:frequency:meters (e.g. C30:1:60; default none)\n"
        << "  --exclude-known-bad-madoca-sats <preset>\n"
        << "                          Expand a named preset to --phase-admission-exclude-sat-frequency-pairs.\n"
        << "                          Presets target satellites whose MADOCA SSR phase bias does not capture\n"
        << "                          the per-sat hardware bias (see docs). Supported: mizu-2025-04.\n"
        << "  --reset-phase-ambiguity-on-before-exclusion\n"
        << "                          Reset ambiguity state while before-time phase exclusion is active (default: keep state)\n"
        << "  --elevation-mask <deg>  Satellite elevation mask in degrees (default: 15)\n"
        << "  --no-estimate-troposphere\n"
        << "                          Disable zenith troposphere estimation\n"
        << "  --estimate-troposphere  Enable zenith troposphere estimation (default)\n"
        << "  --no-ionosphere-free\n"
        << "                          Use per-frequency observations instead of the ionosphere-free combination\n"
        << "  --ionosphere-free       Use the ionosphere-free dual-frequency combination (default)\n"
        << "  --estimate-ionosphere   Estimate per-satellite ionosphere states\n"
        << "  --no-estimate-ionosphere\n"
        << "                          Disable per-satellite ionosphere state estimation (default)\n"
        << "  --enable-extra-band-observations\n"
        << "                          Emit RINEX observations for bands beyond the\n"
        << "                          primary/secondary pair (enables 3rd-freq states).\n"
        << "                          Experimental; SSR bias path for extra bands may\n"
        << "                          not be fully wired for all product types.\n"
        << "  --enable-per-frequency-phase-bias-states\n"
        << "                          Add guarded non-IFLC carrier-phase ambiguity states per frequency\n"
        << "  --disable-per-frequency-phase-bias-states\n"
        << "                          Disable per-frequency phase-bias states (default)\n"
        << "  --enable-ionosphere-aware-phase-ambiguity-init\n"
        << "                          Seed phase ambiguities with the current estimated ionosphere state\n"
        << "  --disable-ionosphere-aware-phase-ambiguity-init\n"
        << "                          Disable ionosphere-aware phase ambiguity seeding (default)\n"
        << "  --disable-ppp-outlier-detection\n"
        << "                          Disable PPP residual outlier rejection for parity experiments\n"
        << "  --enable-ppp-outlier-detection\n"
        << "                          Enable PPP residual outlier rejection (default)\n"
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
        << "  --clas-kinematic-reseed-position\n"
        << "                          CLASLIB-faithful: re-init position state from SPP every kinematic epoch (default on)\n"
        << "  --no-clas-kinematic-reseed-position\n"
        << "                          Disable CLASLIB-faithful kinematic position re-seed\n"
        << "  --clas-kinematic-reseed-position-variance <m^2>\n"
        << "                          Variance reset on kinematic re-seed (default: 10000.0 = CLASLIB VAR_POS)\n"
        << "  --madocalib-bridge      Delegate this run to linked MADOCALIB postpos()\n"
        << "                          (requires CMake -DMADOCALIB_PARITY_LINK=ON)\n"
        << "  --madocalib-l6 <file>   Extra MADOCA L6 input file; repeat for two-channel L6E\n"
        << "  --madocalib-mdciono <file>\n"
        << "                          Extra MADOCA L6D ionosphere file; repeat up to three\n"
        << "  --madocalib-config <file>\n"
        << "                          MADOCALIB rnx2rtkp config (default: sample.conf)\n"
        << "  --madocalib-start <time>\n"
        << "                          MADOCALIB start time, e.g. '2025/04/01 00:00:00'\n"
        << "  --madocalib-end <time>  MADOCALIB end time, e.g. '2025/04/01 00:59:30'\n"
        << "  --madocalib-ti <seconds>\n"
        << "                          MADOCALIB output interval passed to postpos()\n"
        << "  --madocalib-trace <level>\n"
        << "                          MADOCALIB trace level (default: 0)\n"
        << "  --static                Use a static PPP motion model (default)\n"
        << "  --kinematic             Use a kinematic PPP motion model\n"
        << "  --kinematic-preconvergence-phase-residual-floor <meters>\n"
        << "                          Kinematic carrier-phase outlier floor before convergence (default: 200.0)\n"
        << "  --low-dynamics          Keep kinematic PPP anchored for quasi-static motion\n"
        << "  --no-low-dynamics       Disable quasi-static anchoring (default)\n"
        << "  --enable-ar             Enable PPP ambiguity fixing when supported\n"
        << "  --disable-ar            Disable PPP ambiguity fixing (default)\n"
        << "  --ar-ratio-threshold <value>\n"
        << "                          Ratio threshold for PPP ambiguity fixing (default: 3.0)\n"
        << "  --enable-ppp-holdamb     Apply tight Kalman pseudo-obs update for fixed DD ambiguities\n"
        << "                          on DD_IFLC / DD_PER_FREQ / DD_MADOCA_CASCADED (default: off)\n"
        << "  --no-ppp-holdamb         Disable PPP holdamb (default)\n"
        << "  --ppp-holdamb-innovation-gate <m>\n"
        << "                          Per-pair innovation gate in meters before constraint is dropped\n"
        << "                          (0 = no gate, default: 0)\n"
        << "  --quiet                  Suppress per-run summary output\n"
        << "  -h, --help               Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}


std::string trimCopy(const std::string& value) {
    const std::string whitespace = " \t\n\r";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return "";
    }
    const std::size_t last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1);
}

bool equalsIgnoreCase(const std::string& lhs, const std::string& rhs) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (std::toupper(static_cast<unsigned char>(lhs[i])) !=
            std::toupper(static_cast<unsigned char>(rhs[i]))) {
            return false;
        }
    }
    return true;
}

std::vector<std::string> splitCsvList(const std::string& value) {
    std::vector<std::string> tokens;
    std::stringstream stream(value);
    std::string token;
    while (std::getline(stream, token, ',')) {
        const std::string trimmed = trimCopy(token);
        if (!trimmed.empty()) {
            tokens.push_back(trimmed);
        }
    }
    return tokens;
}

libgnss::SatelliteId parseSatelliteIdToken(const std::string& token) {
    const std::string trimmed = trimCopy(token);
    if (trimmed.size() < 2) {
        throw std::invalid_argument("satellite ID must look like G04, R09, C30, or J03");
    }

    libgnss::GNSSSystem system = libgnss::GNSSSystem::UNKNOWN;
    switch (std::toupper(static_cast<unsigned char>(trimmed[0]))) {
    case 'G':
        system = libgnss::GNSSSystem::GPS;
        break;
    case 'R':
        system = libgnss::GNSSSystem::GLONASS;
        break;
    case 'E':
        system = libgnss::GNSSSystem::Galileo;
        break;
    case 'C':
        system = libgnss::GNSSSystem::BeiDou;
        break;
    case 'J':
        system = libgnss::GNSSSystem::QZSS;
        break;
    case 'S':
        system = libgnss::GNSSSystem::SBAS;
        break;
    case 'I':
        system = libgnss::GNSSSystem::NavIC;
        break;
    default:
        throw std::invalid_argument("unsupported satellite system in ID: " + token);
    }

    const std::string prn_text = trimmed.substr(1);
    if (prn_text.empty() ||
        std::any_of(prn_text.begin(), prn_text.end(), [](unsigned char ch) {
            return !std::isdigit(ch);
        })) {
        throw std::invalid_argument("satellite PRN must be numeric in ID: " + token);
    }
    const int prn = std::stoi(prn_text);
    if (prn <= 0 || prn > 255) {
        throw std::invalid_argument("satellite PRN must be in 1..255 in ID: " + token);
    }
    return libgnss::SatelliteId(system, static_cast<uint8_t>(prn));
}

std::set<libgnss::SatelliteId> parseSatelliteIdSet(const std::string& value) {
    if (trimCopy(value).empty() || equalsIgnoreCase(trimCopy(value), "all")) {
        return {};
    }
    std::set<libgnss::SatelliteId> satellites;
    for (const std::string& token : splitCsvList(value)) {
        satellites.insert(parseSatelliteIdToken(token));
    }
    return satellites;
}

int parseFrequencyIndexToken(const std::string& token) {
    if (token.empty() ||
        std::any_of(token.begin(), token.end(), [](unsigned char ch) {
            return !std::isdigit(ch);
        })) {
        throw std::invalid_argument("frequency index list must contain non-negative integers");
    }
    const int index = std::stoi(token);
    if (index < 0 || index > 16) {
        throw std::invalid_argument("frequency indexes must be in 0..16");
    }
    return index;
}

std::set<int> parseFrequencyIndexSet(const std::string& value) {
    if (trimCopy(value).empty() || equalsIgnoreCase(trimCopy(value), "all")) {
        return {};
    }
    std::set<int> indexes;
    for (const std::string& token : splitCsvList(value)) {
        indexes.insert(parseFrequencyIndexToken(token));
    }
    return indexes;
}

std::pair<libgnss::SatelliteId, int> parseSatelliteFrequencyPairToken(
    const std::string& token) {
    const std::string trimmed = trimCopy(token);
    const std::size_t separator = trimmed.find(':');
    if (separator == std::string::npos || separator == 0 || separator + 1 >= trimmed.size()) {
        throw std::invalid_argument(
            "satellite/frequency pair must look like G04:0, R09:0, or C30:1");
    }
    return {
        parseSatelliteIdToken(trimmed.substr(0, separator)),
        parseFrequencyIndexToken(trimmed.substr(separator + 1)),
    };
}

std::set<std::pair<libgnss::SatelliteId, int>> parseSatelliteFrequencyPairSet(
    const std::string& value) {
    if (trimCopy(value).empty() || equalsIgnoreCase(trimCopy(value), "all")) {
        return {};
    }
    std::set<std::pair<libgnss::SatelliteId, int>> pairs;
    for (const std::string& token : splitCsvList(value)) {
        pairs.insert(parseSatelliteFrequencyPairToken(token));
    }
    return pairs;
}

std::pair<std::pair<libgnss::SatelliteId, int>, libgnss::GNSSTime>
parseSatelliteFrequencyPairBeforeTimeToken(const std::string& token) {
    const std::string trimmed = trimCopy(token);
    std::vector<std::string> parts;
    std::stringstream stream(trimmed);
    std::string part;
    while (std::getline(stream, part, ':')) {
        parts.push_back(trimCopy(part));
    }
    if (parts.size() != 4) {
        throw std::invalid_argument(
            "timed satellite/frequency pair must look like E31:0:2360:173610");
    }
    const auto pair = std::make_pair(
        parseSatelliteIdToken(parts[0]), parseFrequencyIndexToken(parts[1]));
    if (parts[2].empty() ||
        std::any_of(parts[2].begin(), parts[2].end(), [](unsigned char ch) {
            return !std::isdigit(ch);
        })) {
        throw std::invalid_argument("GPS week must be a non-negative integer");
    }
    const int week = std::stoi(parts[2]);
    const double tow = std::stod(parts[3]);
    if (week < 0 || !std::isfinite(tow) || tow < 0.0 || tow >= 604800.0) {
        throw std::invalid_argument("GPS TOW must be in [0, 604800) seconds");
    }
    return {pair, libgnss::GNSSTime(week, tow)};
}

std::map<std::pair<libgnss::SatelliteId, int>, libgnss::GNSSTime>
parseSatelliteFrequencyPairBeforeTimeMap(const std::string& value) {
    if (trimCopy(value).empty() || equalsIgnoreCase(trimCopy(value), "none")) {
        return {};
    }
    std::map<std::pair<libgnss::SatelliteId, int>, libgnss::GNSSTime> pairs;
    for (const std::string& token : splitCsvList(value)) {
        const auto parsed = parseSatelliteFrequencyPairBeforeTimeToken(token);
        pairs[parsed.first] = parsed.second;
    }
    return pairs;
}

std::pair<std::pair<libgnss::SatelliteId, int>, double>
parseSatelliteFrequencyPairResidualFloorToken(const std::string& token) {
    const std::string trimmed = trimCopy(token);
    std::vector<std::string> parts;
    std::stringstream stream(trimmed);
    std::string part;
    while (std::getline(stream, part, ':')) {
        parts.push_back(trimCopy(part));
    }
    if (parts.size() != 3) {
        throw std::invalid_argument(
            "phase residual floor pair must look like C30:1:60");
    }
    const auto pair = std::make_pair(
        parseSatelliteIdToken(parts[0]), parseFrequencyIndexToken(parts[1]));
    const double floor_m = std::stod(parts[2]);
    if (!std::isfinite(floor_m) || floor_m <= 0.0) {
        throw std::invalid_argument("phase residual floor must be positive");
    }
    return {pair, floor_m};
}

// Empirically derived "known bad" MADOCA satellite/frequency pairs: satellites
// whose tail-window mean phase residual stays above ~15 mm on MIZU benchmarks,
// where MADOCA SSR phase bias does not capture the per-sat hardware bias.
// Excluding these pairs halved tail-60 3D RMS (0.304 m → 0.156 m on MIZU
// 2025-04 day 091). The list is dataset/month specific; re-audit as the
// constellation evolves (retires, replacements, new IIIA/IIR-M launches).
std::map<std::string, std::string> knownBadMadocaSatPresets() {
    return {
        {"mizu-2025-04",
         "J02:0,J02:1,J03:0,G26:0,G26:1,G16:0,G16:1"},
    };
}

std::string lookupKnownBadMadocaSatPreset(const std::string& name) {
    const std::string trimmed = trimCopy(name);
    if (trimmed.empty()) {
        return {};
    }
    const auto presets = knownBadMadocaSatPresets();
    const auto it = presets.find(trimmed);
    if (it == presets.end()) {
        std::string available;
        for (const auto& entry : presets) {
            if (!available.empty()) {
                available += ", ";
            }
            available += entry.first;
        }
        throw std::invalid_argument(
            "unknown --exclude-known-bad-madoca-sats preset '" + trimmed +
            "' (available: " + available + ")");
    }
    return it->second;
}

std::string mergeCsvPairLists(const std::string& left, const std::string& right) {
    std::string merged = trimCopy(left);
    const std::string rhs = trimCopy(right);
    if (rhs.empty()) {
        return merged;
    }
    if (merged.empty()) {
        return rhs;
    }
    merged += ",";
    merged += rhs;
    return merged;
}

std::map<std::pair<libgnss::SatelliteId, int>, double>
parseSatelliteFrequencyPairResidualFloorMap(const std::string& value) {
    if (trimCopy(value).empty() || equalsIgnoreCase(trimCopy(value), "none")) {
        return {};
    }
    std::map<std::pair<libgnss::SatelliteId, int>, double> floors;
    for (const std::string& token : splitCsvList(value)) {
        const auto parsed = parseSatelliteFrequencyPairResidualFloorToken(token);
        floors[parsed.first] = parsed.second;
    }
    return floors;
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
            options.nav_paths.push_back(argv[++i]);
        } else if (arg == "--sp3" && i + 1 < argc) {
            options.sp3_path = argv[++i];
        } else if (arg == "--clk" && i + 1 < argc) {
            options.clk_path = argv[++i];
        } else if (arg == "--ssr" && i + 1 < argc) {
            options.ssr_path = argv[++i];
        } else if (arg == "--ssr-rtcm" && i + 1 < argc) {
            options.ssr_rtcm_path = argv[++i];
        } else if (arg == "--madoca-l6e" && i + 1 < argc) {
            options.madoca_l6e_paths.push_back(argv[++i]);
        } else if (arg == "--madoca-l6d" && i + 1 < argc) {
            options.madoca_l6d_paths.push_back(argv[++i]);
        } else if (arg == "--madoca-navsys" && i + 1 < argc) {
            options.madoca_navsys_mask = std::stoi(argv[++i]);
        } else if (arg == "--ionex" && i + 1 < argc) {
            options.ionex_path = argv[++i];
        } else if (arg == "--dcb" && i + 1 < argc) {
            options.dcb_path = argv[++i];
        } else if (arg == "--antex" && i + 1 < argc) {
            options.antex_path = argv[++i];
        } else if (arg == "--blq" && i + 1 < argc) {
            options.blq_path = argv[++i];
        } else if (arg == "--ocean-loading-station" && i + 1 < argc) {
            options.ocean_loading_station_name = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            options.out_path = argv[++i];
        } else if (arg == "--summary-json" && i + 1 < argc) {
            options.summary_json_path = argv[++i];
        } else if (arg == "--ppp-correction-log" && i + 1 < argc) {
            options.ppp_correction_log_path = argv[++i];
        } else if (arg == "--ppp-filter-log" && i + 1 < argc) {
            options.ppp_filter_log_path = argv[++i];
        } else if (arg == "--ppp-residual-log" && i + 1 < argc) {
            options.ppp_residual_log_path = argv[++i];
        } else if (arg == "--kml" && i + 1 < argc) {
            options.kml_path = argv[++i];
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            options.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--start-tow" && i + 1 < argc) {
            options.start_tow = std::stod(argv[++i]);
        } else if (arg == "--convergence-min-epochs" && i + 1 < argc) {
            options.convergence_min_epochs = std::stoi(argv[++i]);
        } else if (arg == "--phase-measurement-min-lock-count" && i + 1 < argc) {
            options.phase_measurement_min_lock_count = std::stoi(argv[++i]);
        } else if (arg == "--enable-initial-phase-admission-warm-start") {
            options.enable_initial_phase_admission_warm_start = true;
        } else if (arg == "--disable-initial-phase-admission-warm-start") {
            options.enable_initial_phase_admission_warm_start = false;
        } else if (arg == "--enable-all-frequency-initial-phase-admission-warm-start") {
            options.enable_all_frequency_initial_phase_admission_warm_start = true;
        } else if (arg == "--disable-all-frequency-initial-phase-admission-warm-start") {
            options.enable_all_frequency_initial_phase_admission_warm_start = false;
        } else if (arg == "--initial-phase-admission-warm-start-navsys" && i + 1 < argc) {
            options.initial_phase_admission_warm_start_navsys_mask = std::stoi(argv[++i]);
        } else if (arg == "--initial-phase-admission-warm-start-sats" && i + 1 < argc) {
            options.initial_phase_admission_warm_start_satellites_arg = argv[++i];
        } else if (arg == "--initial-phase-admission-warm-start-frequency-indexes" && i + 1 < argc) {
            options.initial_phase_admission_warm_start_frequency_indexes_arg = argv[++i];
        } else if (arg == "--initial-phase-admission-warm-start-sat-frequency-pairs" && i + 1 < argc) {
            options.initial_phase_admission_warm_start_satellite_frequency_pairs_arg = argv[++i];
        } else if (arg == "--phase-admission-exclude-sat-frequency-pairs" && i + 1 < argc) {
            options.phase_admission_excluded_satellite_frequency_pairs_arg = argv[++i];
        } else if (arg == "--phase-admission-exclude-sat-frequency-pairs-before" && i + 1 < argc) {
            options.phase_admission_excluded_before_satellite_frequency_pairs_arg = argv[++i];
        } else if (arg == "--phase-admission-residual-floor-sat-frequency-pairs" && i + 1 < argc) {
            options.phase_admission_residual_floor_satellite_frequency_pairs_arg = argv[++i];
        } else if (arg == "--exclude-known-bad-madoca-sats" && i + 1 < argc) {
            options.exclude_known_bad_madoca_sats_preset = argv[++i];
        } else if (arg == "--reset-phase-ambiguity-on-before-exclusion") {
            options.reset_phase_ambiguity_on_before_exclusion = true;
        } else if (arg == "--elevation-mask" && i + 1 < argc) {
            options.elevation_mask_deg = std::stod(argv[++i]);
        } else if (arg == "--ssr-step-seconds" && i + 1 < argc) {
            options.ssr_step_seconds = std::stod(argv[++i]);
        } else if (arg == "--enforce-ssr-orbit-iode") {
            options.enforce_ssr_orbit_iode = true;
        } else if (arg == "--no-enforce-ssr-orbit-iode") {
            options.enforce_ssr_orbit_iode = false;
        } else if (arg == "--enforce-ssr-orbit-iode-admission-only") {
            options.enforce_ssr_orbit_iode_admission_only = true;
        } else if (arg == "--no-enforce-ssr-orbit-iode-admission-only") {
            options.enforce_ssr_orbit_iode_admission_only = false;
        } else if (arg == "--ssr-orbit-iode-admission-gate-warmup-epochs" && i + 1 < argc) {
            options.ssr_orbit_iode_admission_gate_warmup_epochs = std::stoi(argv[++i]);
        } else if (arg == "--ar-method" && i + 1 < argc) {
            options.ar_method_arg = argv[++i];
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
        } else if (arg == "--clas-kinematic-reseed-position") {
            options.clas_kinematic_position_reseed = true;
            options.clas_kinematic_position_reseed_set = true;
        } else if (arg == "--no-clas-kinematic-reseed-position") {
            options.clas_kinematic_position_reseed = false;
            options.clas_kinematic_position_reseed_set = true;
        } else if (arg == "--clas-kinematic-reseed-position-variance" && i + 1 < argc) {
            options.clas_kinematic_position_reseed_variance = std::stod(argv[++i]);
        } else if (arg == "--madocalib-bridge") {
            options.madocalib_bridge = true;
        } else if (arg == "--madocalib-l6" && i + 1 < argc) {
            options.madocalib_l6_paths.push_back(argv[++i]);
        } else if (arg == "--madocalib-mdciono" && i + 1 < argc) {
            options.madocalib_mdciono_paths.push_back(argv[++i]);
        } else if (arg == "--madocalib-config" && i + 1 < argc) {
            options.madocalib_config_path = argv[++i];
        } else if (arg == "--madocalib-start" && i + 1 < argc) {
            options.madocalib_start_time = argv[++i];
        } else if (arg == "--madocalib-end" && i + 1 < argc) {
            options.madocalib_end_time = argv[++i];
        } else if (arg == "--madocalib-ti" && i + 1 < argc) {
            options.madocalib_time_interval_seconds = std::stod(argv[++i]);
        } else if (arg == "--madocalib-trace" && i + 1 < argc) {
            options.madocalib_trace_level = std::stoi(argv[++i]);
        } else if (arg == "--no-ionosphere-free") {
            options.use_ionosphere_free = false;
        } else if (arg == "--ionosphere-free") {
            options.use_ionosphere_free = true;
        } else if (arg == "--estimate-ionosphere") {
            options.estimate_ionosphere = true;
        } else if (arg == "--no-estimate-ionosphere") {
            options.estimate_ionosphere = false;
        } else if (arg == "--enable-per-frequency-phase-bias-states") {
            options.enable_per_frequency_phase_bias_states = true;
        } else if (arg == "--disable-per-frequency-phase-bias-states") {
            options.enable_per_frequency_phase_bias_states = false;
        } else if (arg == "--enable-extra-band-observations") {
            options.enable_extra_band_observations = true;
        } else if (arg == "--disable-extra-band-observations") {
            options.enable_extra_band_observations = false;
        } else if (arg == "--enable-ionosphere-aware-phase-ambiguity-init") {
            options.initialize_phase_ambiguity_with_ionosphere_state = true;
        } else if (arg == "--disable-ionosphere-aware-phase-ambiguity-init") {
            options.initialize_phase_ambiguity_with_ionosphere_state = false;
        } else if (arg == "--disable-ppp-outlier-detection") {
            options.enable_ppp_outlier_detection = false;
        } else if (arg == "--enable-ppp-outlier-detection") {
            options.enable_ppp_outlier_detection = true;
        } else if (arg == "--static") {
            options.kinematic_mode = false;
        } else if (arg == "--kinematic") {
            options.kinematic_mode = true;
        } else if (arg == "--kinematic-preconvergence-phase-residual-floor" && i + 1 < argc) {
            options.kinematic_preconvergence_phase_residual_floor_m = std::stod(argv[++i]);
        } else if (arg == "--low-dynamics") {
            options.low_dynamics_mode = true;
        } else if (arg == "--no-low-dynamics") {
            options.low_dynamics_mode = false;
        } else if (arg == "--enable-ar") {
            options.enable_ar = true;
        } else if (arg == "--disable-ar") {
            options.enable_ar = false;
        } else if (arg == "--ar-ratio-threshold" && i + 1 < argc) {
            options.ar_ratio_threshold = std::stod(argv[++i]);
        } else if (arg == "--filter-iterations" && i + 1 < argc) {
            options.filter_iterations = std::stoi(argv[++i]);
        } else if (arg == "--initial-ionosphere-variance" && i + 1 < argc) {
            options.initial_ionosphere_variance = std::stod(argv[++i]);
        } else if (arg == "--initial-troposphere-variance" && i + 1 < argc) {
            options.initial_troposphere_variance = std::stod(argv[++i]);
        } else if (arg == "--code-phase-error-ratio-l1" && i + 1 < argc) {
            options.code_phase_error_ratio_l1 = std::stod(argv[++i]);
        } else if (arg == "--code-phase-error-ratio-l2" && i + 1 < argc) {
            options.code_phase_error_ratio_l2 = std::stod(argv[++i]);
        } else if (arg == "--enable-ppp-holdamb") {
            options.enable_ppp_holdamb = true;
        } else if (arg == "--no-ppp-holdamb") {
            options.enable_ppp_holdamb = false;
        } else if (arg == "--ppp-holdamb-innovation-gate" && i + 1 < argc) {
            options.ppp_holdamb_innovation_gate_m = std::stod(argv[++i]);
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    if (options.obs_path.empty()) {
        argumentError("--obs is required", argv[0]);
    }
    if (options.out_path.empty()) {
        argumentError("--out is required", argv[0]);
    }
    if (options.nav_paths.empty() && options.sp3_path.empty()) {
        argumentError("provide at least one of --nav or --sp3", argv[0]);
    }
    if (!options.ssr_rtcm_path.empty() && options.nav_paths.empty()) {
        argumentError("--ssr-rtcm requires --nav", argv[0]);
    }
    if (options.madoca_l6d_paths.empty() == false && options.madoca_l6e_paths.empty()) {
        argumentError("--madoca-l6d requires --madoca-l6e", argv[0]);
    }
    if (options.madoca_l6e_paths.empty() == false &&
        (options.ssr_path.empty() == false || options.ssr_rtcm_path.empty() == false)) {
        argumentError("--madoca-l6e/--madoca-l6d cannot be combined with --ssr or --ssr-rtcm", argv[0]);
    }
    if (options.madocalib_bridge && !options.ppp_correction_log_path.empty()) {
        argumentError("--ppp-correction-log is only supported by the native PPP path", argv[0]);
    }
    if (options.madocalib_bridge && !options.ppp_filter_log_path.empty()) {
        argumentError("--ppp-filter-log is only supported by the native PPP path", argv[0]);
    }
    if (options.madocalib_bridge && !options.ppp_residual_log_path.empty()) {
        argumentError("--ppp-residual-log is only supported by the native PPP path", argv[0]);
    }
    if (options.madocalib_bridge && options.madoca_l6e_paths.empty() == false) {
        argumentError(
            "--madoca-l6e is native; use --madocalib-l6 with --madocalib-bridge",
            argv[0]);
    }
    if (options.madocalib_bridge && options.madoca_l6d_paths.empty() == false) {
        argumentError(
            "--madoca-l6d is native; use --madocalib-mdciono with --madocalib-bridge",
            argv[0]);
    }
    if (options.madoca_navsys_mask != 0 && options.madoca_l6e_paths.empty()) {
        argumentError("--madoca-navsys requires --madoca-l6e", argv[0]);
    }
    if (options.max_epochs < 0) {
        argumentError("--max-epochs must be non-negative", argv[0]);
    }
    if (options.convergence_min_epochs <= 0) {
        argumentError("--convergence-min-epochs must be positive", argv[0]);
    }
    if (options.phase_measurement_min_lock_count < 0) {
        argumentError("--phase-measurement-min-lock-count must be non-negative", argv[0]);
    }
    if (options.initial_phase_admission_warm_start_navsys_mask < 0 ||
        options.initial_phase_admission_warm_start_navsys_mask > 127) {
        argumentError("--initial-phase-admission-warm-start-navsys must be a RTKLIB navsys mask from 0 to 127", argv[0]);
    }
    try {
        (void)parseSatelliteIdSet(options.initial_phase_admission_warm_start_satellites_arg);
        (void)parseFrequencyIndexSet(options.initial_phase_admission_warm_start_frequency_indexes_arg);
        (void)parseSatelliteFrequencyPairSet(
            options.initial_phase_admission_warm_start_satellite_frequency_pairs_arg);
        if (!options.exclude_known_bad_madoca_sats_preset.empty()) {
            const std::string preset_csv = lookupKnownBadMadocaSatPreset(
                options.exclude_known_bad_madoca_sats_preset);
            options.phase_admission_excluded_satellite_frequency_pairs_arg =
                mergeCsvPairLists(
                    options.phase_admission_excluded_satellite_frequency_pairs_arg,
                    preset_csv);
        }
        (void)parseSatelliteFrequencyPairSet(
            options.phase_admission_excluded_satellite_frequency_pairs_arg);
        (void)parseSatelliteFrequencyPairBeforeTimeMap(
            options.phase_admission_excluded_before_satellite_frequency_pairs_arg);
        (void)parseSatelliteFrequencyPairResidualFloorMap(
            options.phase_admission_residual_floor_satellite_frequency_pairs_arg);
    } catch (const std::invalid_argument& error) {
        argumentError(error.what(), argv[0]);
    }
    if (options.ar_ratio_threshold <= 0.0) {
        argumentError("--ar-ratio-threshold must be positive", argv[0]);
    }
    if (!std::isfinite(options.kinematic_preconvergence_phase_residual_floor_m) ||
        options.kinematic_preconvergence_phase_residual_floor_m <= 0.0) {
        argumentError("--kinematic-preconvergence-phase-residual-floor must be positive", argv[0]);
    }
    if (options.filter_iterations < 0) {
        argumentError("--filter-iterations must be non-negative", argv[0]);
    }
    if (options.initial_ionosphere_variance >= 0.0 &&
        options.initial_ionosphere_variance == 0.0) {
        argumentError("--initial-ionosphere-variance must be positive", argv[0]);
    }
    if (options.initial_troposphere_variance >= 0.0 &&
        options.initial_troposphere_variance == 0.0) {
        argumentError("--initial-troposphere-variance must be positive", argv[0]);
    }
    if (options.code_phase_error_ratio_l1 >= 0.0 &&
        options.code_phase_error_ratio_l1 <= 0.0) {
        argumentError("--code-phase-error-ratio-l1 must be positive", argv[0]);
    }
    if (options.code_phase_error_ratio_l2 >= 0.0 &&
        options.code_phase_error_ratio_l2 <= 0.0) {
        argumentError("--code-phase-error-ratio-l2 must be positive", argv[0]);
    }
    if (options.elevation_mask_deg < 0.0 || options.elevation_mask_deg >= 90.0) {
        argumentError("--elevation-mask must be in [0, 90) degrees", argv[0]);
    }
    if (options.clas_atmos_stale_after_seconds <= 0.0) {
        argumentError("--clas-atmos-stale-after-seconds must be positive", argv[0]);
    }
    if (options.clas_kinematic_position_reseed_variance <= 0.0 &&
        options.clas_kinematic_position_reseed_variance != -1.0) {
        argumentError("--clas-kinematic-reseed-position-variance must be positive", argv[0]);
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
    if (options.madoca_navsys_mask < 0 || options.madoca_navsys_mask > 127) {
        argumentError("--madoca-navsys must be a RTKLIB navsys mask from 0 to 127", argv[0]);
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


std::string primaryNavPath(const Options& options) {
    return options.nav_paths.empty() ? std::string() : options.nav_paths.front();
}

void writeJsonStringArray(std::ostream& output, const std::vector<std::string>& values) {
    output << "[";
    bool first_value = true;
    for (const std::string& value : values) {
        if (!first_value) {
            output << ", ";
        }
        output << '"' << jsonEscape(value) << '"';
        first_value = false;
    }
    output << "]";
}

void writeJsonSatelliteArray(std::ostream& output, const std::set<libgnss::SatelliteId>& values) {
    output << "[";
    bool first_value = true;
    for (const auto& satellite : values) {
        if (!first_value) {
            output << ", ";
        }
        output << '"' << satellite.toString() << '"';
        first_value = false;
    }
    output << "]";
}

void writeJsonIntegerArray(std::ostream& output, const std::set<int>& values) {
    output << "[";
    bool first_value = true;
    for (const int value : values) {
        if (!first_value) {
            output << ", ";
        }
        output << value;
        first_value = false;
    }
    output << "]";
}

void writeJsonSatelliteFrequencyPairArray(
    std::ostream& output,
    const std::set<std::pair<libgnss::SatelliteId, int>>& values) {
    output << "[";
    bool first_value = true;
    for (const auto& [satellite, frequency_index] : values) {
        if (!first_value) {
            output << ", ";
        }
        output << '"' << satellite.toString() << ':' << frequency_index << '"';
        first_value = false;
    }
    output << "]";
}

void writeJsonSatelliteFrequencyPairBeforeTimeArray(
    std::ostream& output,
    const std::map<std::pair<libgnss::SatelliteId, int>, libgnss::GNSSTime>& values) {
    output << "[";
    bool first_value = true;
    for (const auto& [pair, time] : values) {
        if (!first_value) {
            output << ", ";
        }
        output << '"' << pair.first.toString() << ':' << pair.second << ':'
               << time.week << ':' << time.tow << '"';
        first_value = false;
    }
    output << "]";
}

void writeJsonSatelliteFrequencyPairResidualFloorArray(
    std::ostream& output,
    const std::map<std::pair<libgnss::SatelliteId, int>, double>& values) {
    output << "[";
    bool first_value = true;
    for (const auto& [pair, floor_m] : values) {
        if (!first_value) {
            output << ", ";
        }
        output << '"' << pair.first.toString() << ':' << pair.second << ':'
               << floor_m << '"';
        first_value = false;
    }
    output << "]";
}

void mergeNavigationData(const libgnss::NavigationData& source,
                         libgnss::NavigationData& destination) {
    for (const auto& [satellite, ephemerides] : source.ephemeris_data) {
        (void)satellite;
        for (const libgnss::Ephemeris& eph : ephemerides) {
            destination.addEphemeris(eph);
        }
    }
    if (source.ionosphere_model.valid) {
        destination.ionosphere_model = source.ionosphere_model;
    }
}

std::set<libgnss::GNSSSystem> systemsFromRtklibNavsysMask(int mask) {
    using libgnss::GNSSSystem;

    std::set<GNSSSystem> systems;
    if ((mask & 1) != 0) {
        systems.insert(GNSSSystem::GPS);
    }
    if ((mask & 2) != 0) {
        systems.insert(GNSSSystem::SBAS);
    }
    if ((mask & 4) != 0) {
        systems.insert(GNSSSystem::GLONASS);
    }
    if ((mask & 8) != 0) {
        systems.insert(GNSSSystem::Galileo);
    }
    if ((mask & 16) != 0) {
        systems.insert(GNSSSystem::QZSS);
    }
    if ((mask & 32) != 0) {
        systems.insert(GNSSSystem::BeiDou);
    }
    if ((mask & 64) != 0) {
        systems.insert(GNSSSystem::NavIC);
    }
    return systems;
}

std::string gnssSystemName(libgnss::GNSSSystem system) {
    switch (system) {
    case libgnss::GNSSSystem::GPS:
        return "GPS";
    case libgnss::GNSSSystem::GLONASS:
        return "GLONASS";
    case libgnss::GNSSSystem::Galileo:
        return "Galileo";
    case libgnss::GNSSSystem::BeiDou:
        return "BeiDou";
    case libgnss::GNSSSystem::QZSS:
        return "QZSS";
    case libgnss::GNSSSystem::SBAS:
        return "SBAS";
    case libgnss::GNSSSystem::NavIC:
        return "NavIC";
    default:
        return "UNKNOWN";
    }
}

const char* csvBool(bool value) {
    return value ? "1" : "0";
}

void writeCorrectionLogHeader(std::ostream& output) {
    output
        << "week,tow,sat,primary_signal,secondary_signal,primary_observation_code,"
           "secondary_observation_code,frequency_index,ionosphere_coefficient,"
           "has_carrier_phase,primary_code_bias_coeff,"
           "secondary_code_bias_coeff,ssr_available,ssr_orbit_iode,broadcast_iode,"
           "orbit_clock_applied,orbit_clock_skip_reason,orbit_dx_m,"
           "orbit_dy_m,orbit_dz_m,clock_m,ura_sigma_m,code_bias_m,phase_bias_m,"
           "trop_m,stec_tecu,iono_m,ionosphere_estimation_constraint,dcb_applied,"
           "dcb_bias_m,ionex_applied,ionex_iono_m,atmos_token_count,"
           "preferred_network_id,elevation_deg,variance_pr,variance_cp,"
           "valid_after_corrections,solution_status,receiver_x_m,receiver_y_m,"
           "receiver_z_m,satellite_x_m,satellite_y_m,satellite_z_m,"
           "satellite_clock_bias_m,geometric_range_m,los_x,los_y,los_z\n";
}

void writeCorrectionLogRow(
    std::ostream& output,
    const libgnss::GNSSTime& time,
    const libgnss::PPPProcessor::SSRApplicationDiagnostic& diagnostic,
    libgnss::SolutionStatus solution_status) {
    output << time.week << ','
           << time.tow << ','
           << diagnostic.satellite.toString() << ','
           << static_cast<int>(diagnostic.primary_signal) << ','
           << static_cast<int>(diagnostic.secondary_signal) << ','
           << diagnostic.primary_observation_code << ','
           << diagnostic.secondary_observation_code << ','
           << diagnostic.frequency_index << ','
           << diagnostic.ionosphere_coefficient << ','
           << csvBool(diagnostic.has_carrier_phase) << ','
           << diagnostic.primary_code_bias_coeff << ','
           << diagnostic.secondary_code_bias_coeff << ','
           << csvBool(diagnostic.ssr_available) << ','
           << diagnostic.ssr_orbit_iode << ','
           << diagnostic.broadcast_iode << ','
           << csvBool(diagnostic.orbit_clock_applied) << ','
           << diagnostic.orbit_clock_skip_reason << ','
           << diagnostic.orbit_correction_x_m << ','
           << diagnostic.orbit_correction_y_m << ','
           << diagnostic.orbit_correction_z_m << ','
           << diagnostic.clock_correction_m << ','
           << diagnostic.ura_sigma_m << ','
           << diagnostic.code_bias_m << ','
           << diagnostic.phase_bias_m << ','
           << diagnostic.trop_correction_m << ','
           << diagnostic.stec_tecu << ','
           << diagnostic.iono_correction_m << ','
           << csvBool(diagnostic.ionosphere_estimation_constraint) << ','
           << csvBool(diagnostic.dcb_applied) << ','
           << diagnostic.dcb_bias_m << ','
           << csvBool(diagnostic.ionex_applied) << ','
           << diagnostic.ionex_iono_m << ','
           << diagnostic.atmos_token_count << ','
           << diagnostic.preferred_network_id << ','
           << diagnostic.elevation_deg << ','
           << diagnostic.variance_pr << ','
           << diagnostic.variance_cp << ','
           << csvBool(diagnostic.valid_after_corrections) << ','
           << static_cast<int>(solution_status) << ','
           << diagnostic.receiver_position_x_m << ','
           << diagnostic.receiver_position_y_m << ','
           << diagnostic.receiver_position_z_m << ','
           << diagnostic.satellite_position_x_m << ','
           << diagnostic.satellite_position_y_m << ','
           << diagnostic.satellite_position_z_m << ','
           << diagnostic.satellite_clock_bias_m << ','
           << diagnostic.geometric_range_m << ','
           << diagnostic.line_of_sight_x << ','
           << diagnostic.line_of_sight_y << ','
           << diagnostic.line_of_sight_z << '\n';
}

void writePPPFilterLogHeader(std::ostream& output) {
    output
        << "week,tow,iteration,rows,code_rows,phase_rows,ionosphere_constraint_rows,"
           "pos_delta_m,clock_delta_m,trop_delta_m,iono_delta_rms_m,"
           "iono_delta_max_abs_m,clock_state_m,"
           "trop_state_m,iono_state_rms_m,iono_state_max_abs_m,code_residual_rms_m,"
           "code_residual_max_abs_m,code_residual_max_sat,phase_residual_rms_m,"
           "phase_residual_max_abs_m,phase_residual_max_sat,solution_status,"
           "pos_delta_x_m,pos_delta_y_m,pos_delta_z_m,position_x_m,position_y_m,"
           "position_z_m,glo_clock_delta_m,gal_clock_delta_m,qzs_clock_delta_m,"
           "bds_clock_delta_m,bds2_clock_delta_m,bds3_clock_delta_m,"
           "glo_clock_state_m,gal_clock_state_m,qzs_clock_state_m,bds_clock_state_m,"
           "bds2_clock_state_m,bds3_clock_state_m\n";
}

void writePPPFilterLogRow(
    std::ostream& output,
    const libgnss::GNSSTime& time,
    const libgnss::PPPProcessor::PPPFilterIterationDiagnostic& diagnostic,
    libgnss::SolutionStatus solution_status) {
    output << time.week << ','
           << time.tow << ','
           << diagnostic.iteration << ','
           << diagnostic.rows << ','
           << diagnostic.code_rows << ','
           << diagnostic.phase_rows << ','
           << diagnostic.ionosphere_constraint_rows << ','
           << diagnostic.pos_delta_m << ','
           << diagnostic.clock_delta_m << ','
           << diagnostic.trop_delta_m << ','
           << diagnostic.iono_delta_rms_m << ','
           << diagnostic.iono_delta_max_abs_m << ','
           << diagnostic.clock_state_m << ','
           << diagnostic.trop_state_m << ','
           << diagnostic.iono_state_rms_m << ','
           << diagnostic.iono_state_max_abs_m << ','
           << diagnostic.code_residual_rms_m << ','
           << diagnostic.code_residual_max_abs_m << ','
           << diagnostic.code_residual_max_sat.toString() << ','
           << diagnostic.phase_residual_rms_m << ','
           << diagnostic.phase_residual_max_abs_m << ','
           << diagnostic.phase_residual_max_sat.toString() << ','
           << static_cast<int>(solution_status) << ','
           << diagnostic.pos_delta_x_m << ','
           << diagnostic.pos_delta_y_m << ','
           << diagnostic.pos_delta_z_m << ','
           << diagnostic.position_x_m << ','
           << diagnostic.position_y_m << ','
           << diagnostic.position_z_m << ','
           << diagnostic.glo_clock_delta_m << ','
           << diagnostic.gal_clock_delta_m << ','
           << diagnostic.qzs_clock_delta_m << ','
           << diagnostic.bds_clock_delta_m << ','
           << diagnostic.bds2_clock_delta_m << ','
           << diagnostic.bds3_clock_delta_m << ','
           << diagnostic.glo_clock_state_m << ','
           << diagnostic.gal_clock_state_m << ','
           << diagnostic.qzs_clock_state_m << ','
           << diagnostic.bds_clock_state_m << ','
           << diagnostic.bds2_clock_state_m << ','
           << diagnostic.bds3_clock_state_m << '\n';
}

void writePPPResidualLogHeader(std::ostream& output) {
    output
        << "week,tow,iteration,row_index,sat,row_type,observation_m,predicted_m,"
           "residual_m,variance_m2,elevation_deg,iono_state_m,solution_status,"
           "primary_signal,secondary_signal,primary_observation_code,"
           "secondary_observation_code,frequency_index,ionosphere_coefficient,"
           "phase_candidate,phase_accepted,phase_ready,phase_skip_reason,"
           "innovation_variance_m2,innovation_inverse_diagonal_1_per_m2,"
           "innovation_covariance_code_coupling_abs_m2,"
           "innovation_covariance_phase_coupling_abs_m2,"
           "innovation_covariance_ionosphere_constraint_coupling_abs_m2,"
           "innovation_inverse_code_coupling_abs_1_per_m2,"
           "innovation_inverse_phase_coupling_abs_1_per_m2,"
           "innovation_inverse_ionosphere_constraint_coupling_abs_1_per_m2,"
           "position_x_kalman_gain,position_y_kalman_gain,"
           "position_z_kalman_gain,position_update_contribution_x_m,"
           "position_update_contribution_y_m,position_update_contribution_z_m,"
           "position_update_contribution_3d_m,receiver_clock_state_index,"
           "receiver_clock_design_coeff,"
           "receiver_clock_kalman_gain,"
           "receiver_clock_update_contribution_m,ionosphere_state_index,"
           "ionosphere_design_coeff,ionosphere_kalman_gain,"
           "ionosphere_update_contribution_m,ambiguity_state_index,"
           "ambiguity_design_coeff,ambiguity_kalman_gain,"
           "ambiguity_update_contribution_m,ambiguity_lock_count,"
           "required_lock_count,phase_limit_m\n";
}

void writePPPResidualLogRow(
    std::ostream& output,
    const libgnss::GNSSTime& time,
    const libgnss::PPPProcessor::PPPResidualDiagnostic& diagnostic,
    libgnss::SolutionStatus solution_status) {
    output << time.week << ','
           << time.tow << ','
           << diagnostic.iteration << ','
           << diagnostic.row_index << ','
           << diagnostic.satellite.toString() << ','
           << (diagnostic.ionosphere_constraint
                   ? "iono_constraint"
                   : (diagnostic.phase_candidate
                          ? "phase_candidate"
                          : (diagnostic.carrier_phase ? "phase" : "code"))) << ','
           << diagnostic.observation_m << ','
           << diagnostic.predicted_m << ','
           << diagnostic.residual_m << ','
           << diagnostic.variance_m2 << ','
           << diagnostic.elevation_deg << ','
           << diagnostic.iono_state_m << ','
           << static_cast<int>(solution_status) << ','
           << static_cast<int>(diagnostic.primary_signal) << ','
           << static_cast<int>(diagnostic.secondary_signal) << ','
           << diagnostic.primary_observation_code << ','
           << diagnostic.secondary_observation_code << ','
           << diagnostic.frequency_index << ','
           << diagnostic.ionosphere_coefficient << ','
           << csvBool(diagnostic.phase_candidate) << ','
           << csvBool(diagnostic.phase_accepted) << ','
           << csvBool(diagnostic.phase_ready) << ','
           << diagnostic.phase_skip_reason << ','
           << diagnostic.innovation_variance_m2 << ','
           << diagnostic.innovation_inverse_diagonal_1_per_m2 << ','
           << diagnostic.innovation_covariance_code_coupling_abs_m2 << ','
           << diagnostic.innovation_covariance_phase_coupling_abs_m2 << ','
           << diagnostic.innovation_covariance_ionosphere_constraint_coupling_abs_m2 << ','
           << diagnostic.innovation_inverse_code_coupling_abs_1_per_m2 << ','
           << diagnostic.innovation_inverse_phase_coupling_abs_1_per_m2 << ','
           << diagnostic.innovation_inverse_ionosphere_constraint_coupling_abs_1_per_m2 << ','
           << diagnostic.position_x_kalman_gain << ','
           << diagnostic.position_y_kalman_gain << ','
           << diagnostic.position_z_kalman_gain << ','
           << diagnostic.position_update_contribution_x_m << ','
           << diagnostic.position_update_contribution_y_m << ','
           << diagnostic.position_update_contribution_z_m << ','
           << diagnostic.position_update_contribution_3d_m << ','
           << diagnostic.receiver_clock_state_index << ','
           << diagnostic.receiver_clock_design_coeff << ','
           << diagnostic.receiver_clock_kalman_gain << ','
           << diagnostic.receiver_clock_update_contribution_m << ','
           << diagnostic.ionosphere_state_index << ','
           << diagnostic.ionosphere_design_coeff << ','
           << diagnostic.ionosphere_kalman_gain << ','
           << diagnostic.ionosphere_update_contribution_m << ','
           << diagnostic.ambiguity_state_index << ','
           << diagnostic.ambiguity_design_coeff << ','
           << diagnostic.ambiguity_kalman_gain << ','
           << diagnostic.ambiguity_update_contribution_m << ','
           << diagnostic.ambiguity_lock_count << ','
           << diagnostic.required_lock_count << ','
           << diagnostic.phase_limit_m << '\n';
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
            bridge_options.nav_path = primaryNavPath(options);
            bridge_options.out_path = options.out_path;
            bridge_options.config_path = options.madocalib_config_path;
            bridge_options.antenna_path = options.antex_path;
            bridge_options.start_time = options.madocalib_start_time;
            bridge_options.end_time = options.madocalib_end_time;
            bridge_options.time_interval_seconds =
                options.madocalib_time_interval_seconds;
            bridge_options.trace_level = options.madocalib_trace_level;
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
                        << (options.nav_paths.empty() ?
                                "null" :
                                ("\"" + jsonEscape(primaryNavPath(options)) + "\""))
                        << ",\n";
                summary << "  \"nav_files\": ";
                writeJsonStringArray(summary, options.nav_paths);
                summary << ",\n"
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
        if (options.enable_extra_band_observations) {
            obs_reader.setEmitExtraBandObservations(true);
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

        const bool use_madoca_native_l6e = options.madoca_l6e_paths.empty() == false;
        const bool use_madoca_native_l6d = options.madoca_l6d_paths.empty() == false;
        const std::set<libgnss::GNSSSystem> madoca_navsys_systems =
            systemsFromRtklibNavsysMask(options.madoca_navsys_mask);
        const std::set<libgnss::GNSSSystem> initial_phase_warm_start_systems =
            systemsFromRtklibNavsysMask(options.initial_phase_admission_warm_start_navsys_mask);
        const std::set<libgnss::SatelliteId> initial_phase_warm_start_satellites =
            parseSatelliteIdSet(options.initial_phase_admission_warm_start_satellites_arg);
        const std::set<int> initial_phase_warm_start_frequency_indexes =
            parseFrequencyIndexSet(options.initial_phase_admission_warm_start_frequency_indexes_arg);
        const std::set<std::pair<libgnss::SatelliteId, int>>
            initial_phase_warm_start_satellite_frequency_pairs =
                parseSatelliteFrequencyPairSet(
                    options.initial_phase_admission_warm_start_satellite_frequency_pairs_arg);
        const std::set<std::pair<libgnss::SatelliteId, int>>
            phase_admission_excluded_satellite_frequency_pairs =
                parseSatelliteFrequencyPairSet(
                    options.phase_admission_excluded_satellite_frequency_pairs_arg);
        const std::map<std::pair<libgnss::SatelliteId, int>, libgnss::GNSSTime>
            phase_admission_excluded_before_satellite_frequency_pairs =
                parseSatelliteFrequencyPairBeforeTimeMap(
                    options.phase_admission_excluded_before_satellite_frequency_pairs_arg);
        const std::map<std::pair<libgnss::SatelliteId, int>, double>
            phase_admission_residual_floor_satellite_frequency_pairs =
                parseSatelliteFrequencyPairResidualFloorMap(
                    options.phase_admission_residual_floor_satellite_frequency_pairs_arg);
        libgnss::ObservationData preloaded_observation_data;
        bool has_preloaded_observation = false;
        if (use_madoca_native_l6e && obs_header.first_obs.week <= 0) {
            if (!obs_reader.readObservationEpoch(preloaded_observation_data)) {
                std::cerr << "Error: failed to read first observation epoch for --madoca-l6e\n";
                return 1;
            }
            has_preloaded_observation = true;
            obs_header.first_obs = preloaded_observation_data.time;
            if (obs_header.first_obs.week <= 0) {
                std::cerr << "Error: failed to determine GPS week for --madoca-l6e\n";
                return 1;
            }
        }

        libgnss::NavigationData nav_data;
        for (const std::string& nav_path : options.nav_paths) {
            libgnss::io::RINEXReader nav_reader;
            if (!nav_reader.open(nav_path)) {
                std::cerr << "Error: failed to open navigation file: " << nav_path << "\n";
                return 1;
            }
            libgnss::NavigationData nav_file_data;
            if (!nav_reader.readNavigationData(nav_file_data)) {
                std::cerr << "Error: failed to read navigation data: " << nav_path << "\n";
                return 1;
            }
            mergeNavigationData(nav_file_data, nav_data);
        }

        libgnss::PPPProcessor::PPPConfig ppp_config;
        ppp_config.orbit_file_path = options.sp3_path;
        ppp_config.clock_file_path = options.clk_path;
        ppp_config.use_precise_orbits = !options.sp3_path.empty();
        ppp_config.use_precise_clocks = !options.clk_path.empty();
        ppp_config.ssr_file_path = options.ssr_path;
        ppp_config.enforce_ssr_orbit_iode = options.enforce_ssr_orbit_iode;
        ppp_config.enforce_ssr_orbit_iode_admission_only =
            options.enforce_ssr_orbit_iode_admission_only;
        ppp_config.ssr_orbit_iode_admission_gate_warmup_epochs =
            options.ssr_orbit_iode_admission_gate_warmup_epochs;
        if (!options.ar_method_arg.empty()) {
            if (options.ar_method_arg == "dd-iflc") {
                ppp_config.ar_method = libgnss::PPPProcessor::PPPConfig::ARMethod::DD_IFLC;
            } else if (options.ar_method_arg == "dd-wlnl") {
                ppp_config.ar_method = libgnss::PPPProcessor::PPPConfig::ARMethod::DD_WLNL;
            } else if (options.ar_method_arg == "dd-per-freq") {
                ppp_config.ar_method = libgnss::PPPProcessor::PPPConfig::ARMethod::DD_PER_FREQ;
            } else if (options.ar_method_arg == "dd-madoca-cascaded") {
                ppp_config.ar_method =
                    libgnss::PPPProcessor::PPPConfig::ARMethod::DD_MADOCA_CASCADED;
            } else {
                argumentError(
                    "--ar-method must be dd-iflc|dd-wlnl|dd-per-freq|dd-madoca-cascaded",
                    argv[0]);
            }
        }
        // MADOCA-PPP SSR corrections are delivered at the satellite antenna
        // phase center (pos1-sateph=brdc+ssrapc in MADOCALIB). Suppress the
        // satellite PCO application to avoid double-correcting the orbit.
        if (use_madoca_native_l6e) {
            ppp_config.ssr_orbit_reference_is_apc = true;
        }
        ppp_config.use_ssr_corrections =
            options.ssr_path.empty() == false ||
            options.ssr_rtcm_path.empty() == false ||
            use_madoca_native_l6e;
        ppp_config.prefer_receiver_position_seed =
            use_madoca_native_l6e && obs_header.approximate_position.norm() > 1000.0;
        ppp_config.ionex_file_path = options.ionex_path;
        ppp_config.dcb_file_path = options.dcb_path;
        ppp_config.antex_file_path = options.antex_path;
        if (use_madoca_native_l6e) {
            // Keep the receiver clock white-noise-like via SPP seeding, but
            // do not replace the kinematic PPP position state every epoch.
            ppp_config.reset_kinematic_position_to_spp_each_epoch = false;
            ppp_config.require_ssr_orbit_clock = true;
            ppp_config.use_rtklib_broadcast_selection = true;
            ppp_config.initial_troposphere_variance = 0.12 * 0.12;
            ppp_config.allowed_systems = madoca_navsys_systems;
        }
        ppp_config.ocean_loading_file_path = options.blq_path;
        ppp_config.estimate_troposphere = options.estimate_troposphere;
        ppp_config.estimate_ionosphere = options.estimate_ionosphere;
        ppp_config.use_ionosphere_free = options.use_ionosphere_free;
        ppp_config.enable_per_frequency_phase_bias_states =
            options.enable_per_frequency_phase_bias_states;
        ppp_config.initialize_phase_ambiguity_with_ionosphere_state =
            options.initialize_phase_ambiguity_with_ionosphere_state;
        ppp_config.enable_outlier_detection = options.enable_ppp_outlier_detection;
        if (options.filter_iterations > 0) {
            ppp_config.filter_iterations = options.filter_iterations;
        }
        if (options.initial_ionosphere_variance > 0.0) {
            ppp_config.initial_ionosphere_variance = options.initial_ionosphere_variance;
        }
        if (options.initial_troposphere_variance > 0.0) {
            ppp_config.initial_troposphere_variance = options.initial_troposphere_variance;
        }
        if (options.code_phase_error_ratio_l1 > 0.0) {
            ppp_config.code_phase_error_ratio_l1 = options.code_phase_error_ratio_l1;
        }
        if (options.code_phase_error_ratio_l2 > 0.0) {
            ppp_config.code_phase_error_ratio_l2 = options.code_phase_error_ratio_l2;
        }
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
        if (options.clas_kinematic_position_reseed_set) {
            ppp_config.clas_kinematic_position_reseed =
                options.clas_kinematic_position_reseed;
        }
        if (options.clas_kinematic_position_reseed_variance > 0.0) {
            ppp_config.clas_kinematic_position_reseed_variance =
                options.clas_kinematic_position_reseed_variance;
        }
        ppp_config.kinematic_mode = options.kinematic_mode;
        ppp_config.kinematic_preconvergence_phase_residual_floor_m =
            options.kinematic_preconvergence_phase_residual_floor_m;
        ppp_config.low_dynamics_mode = options.low_dynamics_mode;
        ppp_config.enable_ambiguity_resolution = options.enable_ar;
        ppp_config.enable_ppp_holdamb = options.enable_ppp_holdamb;
        ppp_config.ppp_holdamb_innovation_gate_m =
            options.ppp_holdamb_innovation_gate_m;
        ppp_config.convergence_min_epochs = options.convergence_min_epochs;
        ppp_config.phase_measurement_min_lock_count =
            options.phase_measurement_min_lock_count;
        ppp_config.enable_initial_phase_admission_warm_start =
            options.enable_initial_phase_admission_warm_start;
        ppp_config.enable_all_frequency_initial_phase_admission_warm_start =
            options.enable_all_frequency_initial_phase_admission_warm_start;
        ppp_config.initial_phase_admission_warm_start_systems =
            initial_phase_warm_start_systems;
        ppp_config.initial_phase_admission_warm_start_satellites =
            initial_phase_warm_start_satellites;
        ppp_config.initial_phase_admission_warm_start_frequency_indexes =
            initial_phase_warm_start_frequency_indexes;
        ppp_config.initial_phase_admission_warm_start_satellite_frequency_pairs =
            initial_phase_warm_start_satellite_frequency_pairs;
        ppp_config.phase_admission_excluded_satellite_frequency_pairs =
            phase_admission_excluded_satellite_frequency_pairs;
        ppp_config.phase_admission_excluded_before_by_satellite_frequency_pair =
            phase_admission_excluded_before_satellite_frequency_pairs;
        ppp_config.phase_admission_residual_floor_by_satellite_frequency_pair =
            phase_admission_residual_floor_satellite_frequency_pairs;
        ppp_config.reset_phase_ambiguity_on_before_exclusion =
            options.reset_phase_ambiguity_on_before_exclusion;
        ppp_config.ar_ratio_threshold = options.ar_ratio_threshold;
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
        ppp_config.approximate_position = obs_header.approximate_position;
        ppp_config.receiver_antenna_type = obs_header.antenna_type;
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
        processor_config.elevation_mask = options.elevation_mask_deg;

        libgnss::PPPProcessor processor(ppp_config);
        if (!processor.initialize(processor_config)) {
            std::cerr << "Error: failed to initialize PPP processor\n";
            return 1;
        }
        bool madoca_native_l6e_loaded = false;
        bool madoca_native_l6d_loaded = false;
        std::size_t madoca_native_l6e_epochs = 0;
        std::size_t madoca_native_l6d_results = 0;
        int madoca_native_l6e_first_week = 0;
        double madoca_native_l6e_first_tow = 0.0;
        int madoca_native_l6e_last_week = 0;
        double madoca_native_l6e_last_tow = 0.0;
        libgnss::algorithms::madoca_core::CorrectionSummary madoca_native_summary;
        if (use_madoca_native_l6e) {
            const int gps_week = ppp_config.l6_gps_week;
            if (gps_week <= 0) {
                std::cerr << "Error: failed to determine GPS week for --madoca-l6e\n";
                return 1;
            }
            const auto madoca_native_epochs =
                libgnss::algorithms::madoca_core::decodeL6EFiles(
                    options.madoca_l6e_paths, gps_week);
            madoca_native_l6e_epochs = madoca_native_epochs.size();
            if (madoca_native_epochs.empty()) {
                std::cerr << "Error: no native MADOCA L6E corrections decoded\n";
                return 1;
            }
            madoca_native_l6e_first_week = madoca_native_epochs.front().week;
            madoca_native_l6e_first_tow = madoca_native_epochs.front().tow;
            madoca_native_l6e_last_week = madoca_native_epochs.front().week;
            madoca_native_l6e_last_tow = madoca_native_epochs.front().tow;
            for (const auto& epoch : madoca_native_epochs) {
                if (epoch.week < madoca_native_l6e_first_week ||
                    (epoch.week == madoca_native_l6e_first_week &&
                     epoch.tow < madoca_native_l6e_first_tow)) {
                    madoca_native_l6e_first_week = epoch.week;
                    madoca_native_l6e_first_tow = epoch.tow;
                }
                if (epoch.week > madoca_native_l6e_last_week ||
                    (epoch.week == madoca_native_l6e_last_week &&
                     epoch.tow > madoca_native_l6e_last_tow)) {
                    madoca_native_l6e_last_week = epoch.week;
                    madoca_native_l6e_last_tow = epoch.tow;
                }
            }

            libgnss::algorithms::madoca_core::NativeCorrectionStore madoca_native_store;
            madoca_native_store.addL6ECorrections(madoca_native_epochs);
            if (use_madoca_native_l6d) {
                if (obs_header.approximate_position.norm() <= 1000.0) {
                    std::cerr << "Error: --madoca-l6d requires an approximate receiver "
                                 "position in the observation RINEX header\n";
                    return 1;
                }
                const auto madoca_native_l6d_corrections =
                    libgnss::algorithms::madoca_core::decodeL6DFiles(
                        options.madoca_l6d_paths,
                        gps_week,
                        obs_header.approximate_position);
                madoca_native_l6d_results = madoca_native_l6d_corrections.size();
                if (madoca_native_l6d_corrections.empty()) {
                    std::cerr << "Error: no native MADOCA L6D ionosphere corrections decoded\n";
                    return 1;
                }
                madoca_native_store.addL6DCorrections(madoca_native_l6d_corrections);
                madoca_native_l6d_loaded = true;
            }
            madoca_native_summary = madoca_native_store.summary();
            if (libgnss::algorithms::madoca_core::loadCorrections(
                    processor, madoca_native_store) == false) {
                std::cerr << "Error: failed to load native MADOCA corrections\n";
                return 1;
            }
            madoca_native_l6e_loaded = true;
        }

        if (!options.ssr_rtcm_path.empty() &&
            !processor.loadRTCMSSRProducts(
                options.ssr_rtcm_path, nav_data, options.ssr_step_seconds)) {
            std::cerr << "Error: failed to load RTCM SSR corrections: "
                      << options.ssr_rtcm_path << "\n";
            return 1;
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
        std::map<std::string, int> clas_hybrid_fallback_reasons;
        double atmospheric_trop_meters = 0.0;
        double atmospheric_iono_meters = 0.0;
        double ionex_meters = 0.0;
        double dcb_meters = 0.0;
        int ppp_correction_log_rows = 0;
        int ppp_filter_log_rows = 0;
        int ppp_residual_log_rows = 0;
        std::ofstream ppp_correction_log;
        std::ofstream ppp_filter_log;
        std::ofstream ppp_residual_log;
        if (!options.ppp_correction_log_path.empty()) {
            const std::filesystem::path correction_log_path(options.ppp_correction_log_path);
            if (!correction_log_path.parent_path().empty()) {
                std::filesystem::create_directories(correction_log_path.parent_path());
            }
            ppp_correction_log.open(correction_log_path);
            if (!ppp_correction_log.is_open()) {
                std::cerr << "Error: failed to write PPP correction log: "
                          << options.ppp_correction_log_path << "\n";
                return 1;
            }
            ppp_correction_log << std::setprecision(15);
            writeCorrectionLogHeader(ppp_correction_log);
        }
        if (!options.ppp_filter_log_path.empty()) {
            const std::filesystem::path filter_log_path(options.ppp_filter_log_path);
            if (!filter_log_path.parent_path().empty()) {
                std::filesystem::create_directories(filter_log_path.parent_path());
            }
            ppp_filter_log.open(filter_log_path);
            if (!ppp_filter_log.is_open()) {
                std::cerr << "Error: failed to write PPP filter log: "
                          << options.ppp_filter_log_path << "\n";
                return 1;
            }
            ppp_filter_log << std::setprecision(15);
            writePPPFilterLogHeader(ppp_filter_log);
        }
        if (!options.ppp_residual_log_path.empty()) {
            const std::filesystem::path residual_log_path(options.ppp_residual_log_path);
            if (!residual_log_path.parent_path().empty()) {
                std::filesystem::create_directories(residual_log_path.parent_path());
            }
            ppp_residual_log.open(residual_log_path);
            if (!ppp_residual_log.is_open()) {
                std::cerr << "Error: failed to write PPP residual log: "
                          << options.ppp_residual_log_path << "\n";
                return 1;
            }
            ppp_residual_log << std::setprecision(15);
            writePPPResidualLogHeader(ppp_residual_log);
        }
        while (options.max_epochs == 0 || processed_epochs < options.max_epochs) {
            if (has_preloaded_observation) {
                observation_data = preloaded_observation_data;
                has_preloaded_observation = false;
            } else if (!obs_reader.readObservationEpoch(observation_data)) {
                break;
            }

            if (options.start_tow >= 0.0 &&
                observation_data.time.tow < options.start_tow) {
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
            if (processor.getLastClasHybridFallbackUsed()) {
                ++clas_hybrid_fallback_epochs;
                ++clas_hybrid_fallback_reasons[processor.getLastClasHybridFallbackReason()];
            }
            if (ppp_correction_log.is_open()) {
                for (const auto& diagnostic : processor.getLastSSRApplicationDiagnostics()) {
                    writeCorrectionLogRow(
                        ppp_correction_log,
                        observation_data.time,
                        diagnostic,
                        solution.status);
                    ++ppp_correction_log_rows;
                }
            }
            if (ppp_filter_log.is_open()) {
                for (const auto& diagnostic : processor.getLastPPPFilterIterationDiagnostics()) {
                    writePPPFilterLogRow(
                        ppp_filter_log,
                        observation_data.time,
                        diagnostic,
                        solution.status);
                    ++ppp_filter_log_rows;
                }
            }
            if (ppp_residual_log.is_open()) {
                for (const auto& diagnostic : processor.getLastPPPResidualDiagnostics()) {
                    writePPPResidualLogRow(
                        ppp_residual_log,
                        observation_data.time,
                        diagnostic,
                        solution.status);
                    ++ppp_residual_log_rows;
                }
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
                    << (options.nav_paths.empty() ? "null" : ("\"" + jsonEscape(primaryNavPath(options)) + "\""))
                    << ",\n";
            summary << "  \"nav_files\": ";
            writeJsonStringArray(summary, options.nav_paths);
            summary << ",\n"
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
                    << "  \"out\": \"" << jsonEscape(options.out_path) << "\",\n"
                    << "  \"ppp_correction_log\": "
                    << (options.ppp_correction_log_path.empty() ?
                            "null" :
                            ("\"" + jsonEscape(options.ppp_correction_log_path) + "\""))
                    << ",\n"
                    << "  \"ppp_correction_log_rows\": " << ppp_correction_log_rows << ",\n"
                    << "  \"ppp_filter_log\": "
                    << (options.ppp_filter_log_path.empty() ?
                            "null" :
                            ("\"" + jsonEscape(options.ppp_filter_log_path) + "\""))
                    << ",\n"
                    << "  \"ppp_filter_log_rows\": " << ppp_filter_log_rows << ",\n"
                    << "  \"ppp_residual_log\": "
                    << (options.ppp_residual_log_path.empty() ?
                            "null" :
                            ("\"" + jsonEscape(options.ppp_residual_log_path) + "\""))
                    << ",\n"
                    << "  \"ppp_residual_log_rows\": " << ppp_residual_log_rows << ",\n"
                    << "  \"mode\": \"" << (options.kinematic_mode ? "kinematic" : "static") << "\",\n"
                    << "  \"use_clas_osr_filter\": "
                    << (ppp_config.use_clas_osr_filter ? "true" : "false") << ",\n"
                    << "  \"use_ionosphere_free\": "
                    << (ppp_config.use_ionosphere_free ? "true" : "false") << ",\n"
                    << "  \"estimate_ionosphere\": "
                    << (ppp_config.estimate_ionosphere ? "true" : "false") << ",\n"
                    << "  \"kinematic_preconvergence_phase_residual_floor_m\": "
                    << ppp_config.kinematic_preconvergence_phase_residual_floor_m << ",\n"
                    << "  \"phase_measurement_min_lock_count\": "
                    << ppp_config.phase_measurement_min_lock_count << ",\n"
                    << "  \"initial_phase_admission_warm_start\": "
                    << (ppp_config.enable_initial_phase_admission_warm_start ? "true" : "false") << ",\n"
                    << "  \"all_frequency_initial_phase_admission_warm_start\": "
                    << (ppp_config.enable_all_frequency_initial_phase_admission_warm_start ? "true" : "false") << ",\n"
                    << "  \"initial_phase_admission_warm_start_navsys_mask\": "
                    << options.initial_phase_admission_warm_start_navsys_mask << ",\n"
                    << "  \"initial_phase_admission_warm_start_unrestricted\": "
                    << (options.initial_phase_admission_warm_start_navsys_mask == 0 ? "true" : "false") << ",\n"
                    << "  \"initial_phase_admission_warm_start_systems\": [";
            bool first_initial_phase_warm_start_system = true;
            for (const auto system : initial_phase_warm_start_systems) {
                if (first_initial_phase_warm_start_system == false) {
                    summary << ", ";
                }
                summary << '"' << gnssSystemName(system) << '"';
                first_initial_phase_warm_start_system = false;
            }
            summary << "],\n"
                    << "  \"initial_phase_admission_warm_start_satellites_unrestricted\": "
                    << (initial_phase_warm_start_satellites.empty() ? "true" : "false") << ",\n"
                    << "  \"initial_phase_admission_warm_start_satellites\": ";
            writeJsonSatelliteArray(summary, initial_phase_warm_start_satellites);
            summary << ",\n"
                    << "  \"initial_phase_admission_warm_start_frequency_indexes_unrestricted\": "
                    << (initial_phase_warm_start_frequency_indexes.empty() ? "true" : "false") << ",\n"
                    << "  \"initial_phase_admission_warm_start_frequency_indexes\": ";
            writeJsonIntegerArray(summary, initial_phase_warm_start_frequency_indexes);
            summary << ",\n"
                    << "  \"initial_phase_admission_warm_start_sat_frequency_pairs_unrestricted\": "
                    << (initial_phase_warm_start_satellite_frequency_pairs.empty() ?
                            "true" : "false") << ",\n"
                    << "  \"initial_phase_admission_warm_start_sat_frequency_pairs\": ";
            writeJsonSatelliteFrequencyPairArray(
                summary,
                initial_phase_warm_start_satellite_frequency_pairs);
            summary << ",\n"
                    << "  \"phase_admission_excluded_sat_frequency_pairs_unrestricted\": "
                    << (phase_admission_excluded_satellite_frequency_pairs.empty() ?
                            "true" : "false") << ",\n"
                    << "  \"phase_admission_excluded_sat_frequency_pairs\": ";
            writeJsonSatelliteFrequencyPairArray(
                summary,
                phase_admission_excluded_satellite_frequency_pairs);
            summary << ",\n"
                    << "  \"phase_admission_excluded_before_sat_frequency_pairs_unrestricted\": "
                    << (phase_admission_excluded_before_satellite_frequency_pairs.empty() ?
                            "true" : "false") << ",\n"
                    << "  \"phase_admission_excluded_before_sat_frequency_pairs\": ";
            writeJsonSatelliteFrequencyPairBeforeTimeArray(
                summary,
                phase_admission_excluded_before_satellite_frequency_pairs);
            summary << ",\n"
                    << "  \"phase_admission_residual_floor_sat_frequency_pairs_unrestricted\": "
                    << (phase_admission_residual_floor_satellite_frequency_pairs.empty() ?
                            "true" : "false") << ",\n"
                    << "  \"phase_admission_residual_floor_sat_frequency_pairs\": ";
            writeJsonSatelliteFrequencyPairResidualFloorArray(
                summary,
                phase_admission_residual_floor_satellite_frequency_pairs);
            summary << ",\n"
                    << "  \"reset_phase_ambiguity_on_before_exclusion\": "
                    << (options.reset_phase_ambiguity_on_before_exclusion ? "true" : "false")
                    << ",\n"
                    << "  \"enforce_ssr_orbit_iode\": "
                    << (ppp_config.enforce_ssr_orbit_iode ? "true" : "false") << ",\n"
                    << "  \"enforce_ssr_orbit_iode_admission_only\": "
                    << (ppp_config.enforce_ssr_orbit_iode_admission_only ? "true" : "false") << ",\n"
                    << "  \"ssr_orbit_iode_admission_gate_warmup_epochs\": "
                    << ppp_config.ssr_orbit_iode_admission_gate_warmup_epochs << ",\n"
                    << "  \"low_dynamics\": " << (options.low_dynamics_mode ? "true" : "false") << ",\n"
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
                    << "  \"per_frequency_phase_bias_states\": "
                    << (options.enable_per_frequency_phase_bias_states ? "true" : "false") << ",\n"
                    << "  \"ionosphere_aware_phase_ambiguity_init\": "
                    << (options.initialize_phase_ambiguity_with_ionosphere_state ? "true" : "false") << ",\n"
                    << "  \"ppp_outlier_detection_enabled\": "
                    << (options.enable_ppp_outlier_detection ? "true" : "false") << ",\n"
                    << "  \"ar_ratio_threshold\": " << options.ar_ratio_threshold << ",\n"
                    << "  \"processed_epochs\": " << processed_epochs << ",\n"
                    << "  \"valid_solutions\": " << valid_solutions << ",\n"
                    << "  \"ppp_float_solutions\": " << ppp_float_solutions << ",\n"
                    << "  \"ppp_fixed_solutions\": " << ppp_fixed_solutions << ",\n"
                    << "  \"fallback_solutions\": " << fallback_solutions << ",\n"
                    << "  \"clas_hybrid_fallback_epochs\": " << clas_hybrid_fallback_epochs << ",\n"
                    << "  \"ppp_solution_rate_pct\": " << ppp_solution_rate << ",\n"
                    << "  \"ssr_corrections_enabled\": "
                    << ((options.ssr_path.empty() == false ||
                         options.ssr_rtcm_path.empty() == false ||
                         madoca_native_l6e_loaded) ? "true" : "false") << ",\n"
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
                    << "  \"madoca_native_navsys_mask\": "
                    << options.madoca_navsys_mask << ",\n"
                    << "  \"madoca_native_navsys_all_observed\": "
                    << (options.madoca_navsys_mask == 0 ? "true" : "false") << ",\n"
                    << "  \"madoca_native_allowed_systems\": [";
            bool first_madoca_allowed_system = true;
            for (const auto system : madoca_navsys_systems) {
                if (first_madoca_allowed_system == false) {
                    summary << ", ";
                }
                summary << '"' << gnssSystemName(system) << '"';
                first_madoca_allowed_system = false;
            }
            summary << "],\n"
                    << "  \"madoca_native_l6e_loaded\": "
                    << (madoca_native_l6e_loaded ? "true" : "false") << ",\n"
                    << "  \"madoca_native_l6e_files\": [";
            bool first_madoca_native_l6e_file = true;
            for (const std::string& l6e_path : options.madoca_l6e_paths) {
                if (first_madoca_native_l6e_file == false) {
                    summary << ", ";
                }
                summary << '"' << jsonEscape(l6e_path) << '"';
                first_madoca_native_l6e_file = false;
            }
            summary << "],\n"
                    << "  \"madoca_native_l6e_epochs\": " << madoca_native_l6e_epochs << ",\n"
                    << "  \"madoca_native_l6d_loaded\": "
                    << (madoca_native_l6d_loaded ? "true" : "false") << ",\n"
                    << "  \"madoca_native_l6d_files\": [";
            bool first_madoca_native_l6d_file = true;
            for (const std::string& l6d_path : options.madoca_l6d_paths) {
                if (first_madoca_native_l6d_file == false) {
                    summary << ", ";
                }
                summary << '"' << jsonEscape(l6d_path) << '"';
                first_madoca_native_l6d_file = false;
            }
            summary << "],\n"
                    << "  \"madoca_native_l6d_results\": "
                    << madoca_native_l6d_results << ",\n"
                    << "  \"madoca_native_l6e_first_week\": "
                    << madoca_native_l6e_first_week << ",\n"
                    << "  \"madoca_native_l6e_first_tow\": "
                    << madoca_native_l6e_first_tow << ",\n"
                    << "  \"madoca_native_l6e_last_week\": "
                    << madoca_native_l6e_last_week << ",\n"
                    << "  \"madoca_native_l6e_last_tow\": "
                    << madoca_native_l6e_last_tow << ",\n"
                    << "  \"madoca_native_corrections_total\": "
                    << madoca_native_summary.total << ",\n"
                    << "  \"madoca_native_corrections_satellites\": "
                    << madoca_native_summary.satellites << ",\n"
                    << "  \"madoca_native_corrections_orbit\": "
                    << madoca_native_summary.orbit << ",\n"
                    << "  \"madoca_native_corrections_clock\": "
                    << madoca_native_summary.clock << ",\n"
                    << "  \"madoca_native_corrections_code_bias\": "
                    << madoca_native_summary.code_bias << ",\n"
                    << "  \"madoca_native_corrections_phase_bias\": "
                    << madoca_native_summary.phase_bias << ",\n"
                    << "  \"madoca_native_corrections_atmosphere\": "
                    << madoca_native_summary.atmosphere << ",\n"
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
            std::cout << "  CLAS OSR filter: "
                      << (ppp_config.use_clas_osr_filter ? "on" : "off") << "\n";
            std::cout << "  ionosphere-free combination: "
                      << (ppp_config.use_ionosphere_free ? "on" : "off") << "\n";
            std::cout << "  estimate ionosphere: "
                      << (ppp_config.estimate_ionosphere ? "on" : "off") << "\n";
            std::cout << "  kinematic pre-convergence phase residual floor (m): "
                      << ppp_config.kinematic_preconvergence_phase_residual_floor_m << "\n";
            std::cout << "  low dynamics: " << (options.low_dynamics_mode ? "on" : "off") << "\n";
            std::cout << "  CLAS atmosphere selection: " << options.clas_atmos_selection << "\n";
            std::cout << "  CLAS stale-after (s): " << options.clas_atmos_stale_after_seconds << "\n";
            if (clas_hybrid_fallback_epochs > 0) {
                std::cout << "  CLAS hybrid fallback epochs: "
                          << clas_hybrid_fallback_epochs << "\n";
            }
            std::cout << "  ambiguity resolution: " << (options.enable_ar ? "on" : "off") << "\n";
            std::cout << "  per-frequency phase-bias states: "
                      << (options.enable_per_frequency_phase_bias_states ? "on" : "off") << "\n";
            std::cout << "  ionosphere-aware phase ambiguity init: "
                      << (options.initialize_phase_ambiguity_with_ionosphere_state ? "on" : "off") << "\n";
            std::cout << "  phase measurement min lock count: "
                      << ppp_config.phase_measurement_min_lock_count << "\n";
            std::cout << "  initial phase admission warm start: "
                      << (ppp_config.enable_initial_phase_admission_warm_start ? "on" : "off") << "\n";
            std::cout << "  all-frequency initial phase admission warm start: "
                      << (ppp_config.enable_all_frequency_initial_phase_admission_warm_start ? "on" : "off") << "\n";
            if (options.initial_phase_admission_warm_start_navsys_mask == 0) {
                std::cout << "  initial phase admission warm-start navsys: all systems\n";
            } else {
                std::cout << "  initial phase admission warm-start navsys mask: "
                          << options.initial_phase_admission_warm_start_navsys_mask << "\n";
                std::cout << "  initial phase admission warm-start systems:";
                for (const auto system : initial_phase_warm_start_systems) {
                    std::cout << " " << gnssSystemName(system);
                }
                std::cout << "\n";
            }
            std::cout << "  initial phase admission warm-start satellites:";
            if (initial_phase_warm_start_satellites.empty()) {
                std::cout << " all";
            } else {
                for (const auto& satellite : initial_phase_warm_start_satellites) {
                    std::cout << " " << satellite.toString();
                }
            }
            std::cout << "\n";
            std::cout << "  initial phase admission warm-start frequency indexes:";
            if (initial_phase_warm_start_frequency_indexes.empty()) {
                std::cout << " all";
            } else {
                for (const int index : initial_phase_warm_start_frequency_indexes) {
                    std::cout << " " << index;
                }
            }
            std::cout << "\n";
            std::cout << "  initial phase admission warm-start satellite/frequency pairs:";
            if (initial_phase_warm_start_satellite_frequency_pairs.empty()) {
                std::cout << " all";
            } else {
                for (const auto& [satellite, frequency_index] :
                     initial_phase_warm_start_satellite_frequency_pairs) {
                    std::cout << " " << satellite.toString() << ':' << frequency_index;
                }
            }
            std::cout << "\n";
            std::cout << "  phase admission excluded satellite/frequency pairs:";
            if (phase_admission_excluded_satellite_frequency_pairs.empty()) {
                std::cout << " none";
            } else {
                for (const auto& [satellite, frequency_index] :
                     phase_admission_excluded_satellite_frequency_pairs) {
                    std::cout << " " << satellite.toString() << ':' << frequency_index;
                }
            }
            std::cout << "\n";
            std::cout << "  phase admission excluded-before satellite/frequency pairs:";
            if (phase_admission_excluded_before_satellite_frequency_pairs.empty()) {
                std::cout << " none";
            } else {
                for (const auto& [pair, time] :
                     phase_admission_excluded_before_satellite_frequency_pairs) {
                    std::cout << " " << pair.first.toString() << ':' << pair.second
                              << ':' << time.week << ':' << time.tow;
                }
            }
            std::cout << "\n";
            std::cout << "  phase admission residual floors:";
            if (phase_admission_residual_floor_satellite_frequency_pairs.empty()) {
                std::cout << " none";
            } else {
                for (const auto& [pair, floor_m] :
                     phase_admission_residual_floor_satellite_frequency_pairs) {
                    std::cout << " " << pair.first.toString() << ':' << pair.second
                              << ':' << floor_m;
                }
            }
            std::cout << "\n";
            std::cout << "  reset phase ambiguity on before-exclusion: "
                      << (options.reset_phase_ambiguity_on_before_exclusion ? "yes" : "no")
                      << "\n";
            std::cout << "  PPP outlier detection: "
                      << (options.enable_ppp_outlier_detection ? "on" : "off") << "\n";
            std::cout << "  SSR corrections: "
                      << (ppp_config.use_ssr_corrections ? "on" : "off") << "\n";
            if (ppp_config.use_ssr_corrections) {
                std::cout << "  SSR orbit IODE enforcement: "
                          << (ppp_config.enforce_ssr_orbit_iode ? "on" : "off") << "\n";
                std::cout << "  SSR orbit IODE admission-only gate: "
                          << (ppp_config.enforce_ssr_orbit_iode_admission_only ? "on" : "off")
                          << "\n";
                if (ppp_config.enforce_ssr_orbit_iode_admission_only &&
                    ppp_config.ssr_orbit_iode_admission_gate_warmup_epochs > 0) {
                    std::cout << "  SSR orbit IODE gate warmup epochs: "
                              << ppp_config.ssr_orbit_iode_admission_gate_warmup_epochs
                              << "\n";
                }
            }
            if (madoca_native_l6e_loaded) {
                std::cout << "  MADOCA native L6E files: "
                          << options.madoca_l6e_paths.size() << "\n";
                if (options.madoca_navsys_mask == 0) {
                    std::cout << "  MADOCA native navsys: all observed\n";
                } else {
                    std::cout << "  MADOCA native navsys mask: "
                              << options.madoca_navsys_mask << "\n";
                    std::cout << "  MADOCA native navsys systems:";
                    for (const auto system : madoca_navsys_systems) {
                        std::cout << " " << gnssSystemName(system);
                    }
                    std::cout << "\n";
                }
                std::cout << "  MADOCA native L6E epochs: "
                          << madoca_native_l6e_epochs << "\n";
                if (madoca_native_l6d_loaded) {
                    std::cout << "  MADOCA native L6D files: "
                              << options.madoca_l6d_paths.size() << "\n";
                    std::cout << "  MADOCA native L6D correction epochs: "
                              << madoca_native_l6d_results << "\n";
                }
                std::cout << "  MADOCA native corrections: "
                          << madoca_native_summary.total
                          << " total, " << madoca_native_summary.satellites
                          << " satellites\n";
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
