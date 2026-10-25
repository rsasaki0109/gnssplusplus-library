// Analysis-only exporter at every rover epoch: rover and base carrier phase,
// the base aligned with libgnss_apps::interpolateBaseEpoch(model_range_at_target)
// (or the exact base epoch), the geometric ranges from the reference rover
// position (analysis only, never a solver input) and the surveyed base, and the
// rover-to-satellite unit vector.
#include "rtk_base_epoch_align.hpp"
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/rinex.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace libgnss;

static bool range(const NavigationData& nav, const SatelliteId& sat, const GNSSTime& t,
                  const Vector3d& rx, double pseudorange, double& rho, double& el, Vector3d& los) {
    Vector3d pos, vel;
    double clk, drift;
    const double travel = pseudorange / constants::SPEED_OF_LIGHT;
    if (!nav.calculateSatelliteState(sat, t - travel, pos, vel, clk, drift)) return false;
    if (!nav.calculateSatelliteState(sat, t - travel - clk, pos, vel, clk, drift)) return false;
    el = nav.calculateGeometry(rx, pos).elevation;
    rho = geodist(pos, rx) + models::tropDelaySaastamoinen(rx, el);
    los = (pos - rx).normalized();
    return true;
}

int main(int argc, char** argv) {
    if (argc != 9) {
        std::cerr << "usage: export_truth_dd nav rover base reference.csv bx by bz out.csv\n";
        return 2;
    }
    const Vector3d base(std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7]));
    std::map<long long, Vector3d> gt;
    {
        std::ifstream in(argv[4]);
        std::string line;
        std::getline(in, line);
        while (std::getline(in, line)) {
            std::stringstream s(line);
            std::vector<double> row;
            std::string c;
            while (std::getline(s, c, ',')) row.push_back(std::stod(c));
            if (row.size() >= 8) gt[std::llround(row[0] * 1000)] = Vector3d(row[5], row[6], row[7]);
        }
    }
    io::RINEXReader nr, rr, br;
    io::RINEXReader::RINEXHeader nh, rh, bh;
    NavigationData nav;
    if (!nr.open(argv[1]) || !nr.readHeader(nh) || !nr.readNavigationData(nav)) throw std::runtime_error("nav");
    if (!rr.open(argv[2]) || !rr.readHeader(rh)) throw std::runtime_error("rover");
    if (!br.open(argv[3]) || !br.readHeader(bh)) throw std::runtime_error("base");
    std::ofstream out(argv[8]);
    out << std::setprecision(12)
        << "tow,sat,obs_type,lambda,rov_L,base_L,rov_P,base_P,rov_lli,base_lli,rov_snr,el_deg,rho_rov,rho_base,ux,uy,uz,base_dt\n";
    ObservationData ro, lower, upper;
    bool have_lower = false;
    bool have_upper = br.readObservationEpoch(upper);
    std::size_t rows = 0, interpolated = 0, exact = 0;
    while (rr.readObservationEpoch(ro)) {
        while (have_upper && upper.time - ro.time < -1e-6) {
            lower = upper;
            have_lower = true;
            have_upper = br.readObservationEpoch(upper);
        }
        ObservationData bo;
        double base_dt = 0.0;
        if (have_upper && std::abs(upper.time - ro.time) <= 1e-6) {
            bo = upper;
            ++exact;
        } else if (have_lower && have_upper &&
                   libgnss_apps::interpolateBaseEpoch(lower, upper, ro.time, base, nav, bo, true)) {
            base_dt = ro.time - lower.time;
            ++interpolated;
        } else {
            continue;
        }
        const auto it = gt.find(std::llround(ro.time.tow * 1000));
        if (it == gt.end()) continue;
        for (const auto& o : ro.observations) {
            if (!o.has_carrier_phase || !o.has_pseudorange) continue;
            if (o.satellite.system == GNSSSystem::GLONASS) continue;
            const Observation* b = bo.getObservation(o.satellite, o.signal);
            if (!b || !b->has_carrier_phase || !b->has_pseudorange) continue;
            const double lam = signalWavelengthMeters(o);
            if (!(lam > 0)) continue;
            double rr_rho, rb_rho, el, el_b;
            Vector3d los, los_b;
            if (!range(nav, o.satellite, ro.time, it->second, o.pseudorange, rr_rho, el, los)) continue;
            if (!range(nav, o.satellite, ro.time, base, b->pseudorange, rb_rho, el_b, los_b)) continue;
            out << ro.time.tow << ',' << o.satellite.toString() << ',' << o.carrier_phase_observation_type
                << ',' << lam << ',' << o.carrier_phase << ',' << b->carrier_phase << ',' << o.pseudorange << ','
                << b->pseudorange << ',' << int(o.lli) << ','
                << int(b->lli) << ',' << o.snr << ',' << el * 180 / M_PI << ',' << rr_rho << ',' << rb_rho
                << ',' << los.x() << ',' << los.y() << ',' << los.z() << ',' << base_dt << '\n';
            ++rows;
        }
    }
    std::cout << "rows " << rows << " exact " << exact << " interpolated " << interpolated << '\n';
    return 0;
}
