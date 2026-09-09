#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>
#include "../src/algorithms/fgo_internal.hpp"
#include <libgnss++/algorithms/source_epoch_states.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/io/rinex.hpp>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>

// Raw admission scope, NOT final factor admission or an accuracy score.
// Uses the default per-row transmission-time path and code-bias helper of FGO.
// Satellite states stay in memory; no receiver position or trajectory is used.
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    using namespace libgnss;
    io::RINEXReader reader;
    NavigationData nav;
    if (!reader.open(argv[1]) || !reader.readNavigationData(nav)) return 3;
    io::AndroidRawGnssConfig config;
    config.verify_enriched_pseudorange = false;
    config.device_model = "pixel5";
    io::AndroidRawGnssResult raw;
    std::string error;
    if (!io::loadAndroidRawGnssCsv(argv[2], config, raw, error)) return 4;
    struct Stats {
        size_t n = 0, alternate_changed = 0;
        double sum = 0, lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity(), alternate_max = 0;
        void add(double x, double alternate) {
            ++n; sum += x; lo = std::min(lo, x); hi = std::max(hi, x);
            alternate_changed += alternate != x;
            alternate_max = std::max(alternate_max, std::abs(alternate - x));
        }
    };
    std::map<std::pair<int, int>, Stats> stats;
    size_t missing = 0, rejected = 0;
    try {
        for (const auto& epoch : raw.observations.epochs) {
            for (const auto& obs : epoch.observations) {
                if (!obs.valid || !obs.has_pseudorange || !std::isfinite(obs.pseudorange) ||
                    obs.pseudorange <= 0 || !fgo_internal::isEligibleFgoSignal(
                        obs.satellite, obs.signal, true, true)) continue;
                auto transmit = epoch.time - obs.pseudorange / constants::SPEED_OF_LIGHT;
                Vector3d position, velocity;
                double clock = 0, drift = 0;
                if (!nav.calculateSatelliteState(obs.satellite, transmit, position, velocity, clock, drift)) {
                    ++missing; continue;
                }
                transmit = transmit - clock;
                if (!nav.calculateSatelliteState(obs.satellite, transmit, position, velocity, clock, drift)) {
                    ++missing; continue;
                }
                const auto* eph = nav.getEphemeris(obs.satellite, transmit);
                if (!eph || !fgo_internal::isHealthyForPositioning(obs, *eph)) {
                    ++rejected; continue;
                }
                const double x = fgo_internal::groupDelayCorrectionMeters(obs, *eph);
                const double alternate = fgo_internal::groupDelayCorrectionMeters(obs, *eph, true);
                if (!std::isfinite(x) || !std::isfinite(alternate)) return 5;
                stats[{static_cast<int>(obs.satellite.system), static_cast<int>(obs.signal)}].add(x, alternate);
            }
        }
    } catch (...) { std::cerr << "group-delay audit failed\n"; return 6; }
    std::cout << std::setprecision(12);
    std::cout << "epochs=" << raw.observations.epochs.size() << " missing_state_rows=" << missing
              << " missing_or_unhealthy_rows=" << rejected << '\n';
    std::cout << "system_enum,signal_enum,rows,min_m,mean_m,max_m,galileo_alternate_changed,galileo_alternate_max_delta_m\n";
    for (const auto& [key, s] : stats)
        std::cout << key.first << ',' << key.second << ',' << s.n << ',' << s.lo << ','
                  << s.sum / s.n << ',' << s.hi << ',' << s.alternate_changed << ',' << s.alternate_max << '\n';
    return stats.empty() ? 7 : 0;
}
