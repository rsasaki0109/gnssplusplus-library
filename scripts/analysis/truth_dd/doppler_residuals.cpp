// Analysis-only Doppler check at every rover epoch with a reference sample:
// per-observation range-rate residuals against the reference position AND
// velocity (never a solver input), and the velocity of the library solve
// spp_velocity::solveVelocityFromObservations() at the reference position next
// to a robust variant that repeatedly drops the worst residual beyond 4 sigma
// (sigma = 0.5 m/s / sin(el), the solver's own weighting).
//
// usage: doppler_residuals nav rover reference.csv out_obs.csv out_epoch.csv
// The reference is the PPC-Dataset reference.csv layout (TOW, week, lat, lon,
// h, ECEF xyz, roll, pitch, heading, ENU velocity).
#include <libgnss++/algorithms/spp_velocity.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/rinex.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

constexpr double kSigmaMps = 0.5;
constexpr double kRejectSigma = 4.0;

struct Truth {
    Eigen::Vector3d pos, vel;
};

std::map<long long, Truth> readReference(const char* path) {
    std::map<long long, Truth> out;
    std::ifstream in(path);
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string tok;
        std::vector<double> v;
        while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
        if (v.size() < 14) continue;
        const double lat = v[2] * M_PI / 180.0, lon = v[3] * M_PI / 180.0;
        Eigen::Matrix3d enu_to_ecef;
        enu_to_ecef << -std::sin(lon), -std::sin(lat) * std::cos(lon), std::cos(lat) * std::cos(lon),
                        std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat) * std::sin(lon),
                        0.0, std::cos(lat), std::sin(lat);
        out[std::llround(v[0] * 1000.0)] = {Eigen::Vector3d(v[5], v[6], v[7]),
                                            enu_to_ecef * Eigen::Vector3d(v[11], v[12], v[13])};
    }
    return out;
}

double sigmaFor(double elevation_rad) {
    return kSigmaMps / std::max(0.1, std::sin(elevation_rad));
}

// Range-rate residual of one observation for a receiver velocity and clock drift (m/s).
double residual(const spp_velocity::DopplerObservation& d, const Eigen::Vector3d& rx,
                const Eigen::Vector3d& velocity, double clock_drift_mps) {
    const Eigen::Vector3d los = (d.satellite_position_ecef - rx).normalized();
    const double measured = -d.doppler_hz * constants::SPEED_OF_LIGHT / d.frequency_hz;
    return measured - ((d.satellite_velocity_ecef - velocity).dot(los) -
                       constants::SPEED_OF_LIGHT * d.satellite_clock_drift + clock_drift_mps);
}

void writeVector(std::ostream& out, bool ok, const Eigen::Vector3d& v) {
    for (int i = 0; i < 3; ++i) out << ',' << (ok ? v(i) : NAN);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "usage: doppler_residuals nav rover reference.csv out_obs.csv out_epoch.csv\n";
        return 2;
    }
    io::RINEXReader nav_reader, rover_reader;
    io::RINEXReader::RINEXHeader nav_header, rover_header;
    NavigationData nav;
    if (!nav_reader.open(argv[1]) || !nav_reader.readHeader(nav_header) ||
        !nav_reader.readNavigationData(nav) || !rover_reader.open(argv[2]) ||
        !rover_reader.readHeader(rover_header)) {
        std::cerr << "cannot read inputs\n";
        return 2;
    }
    const auto truth = readReference(argv[3]);
    std::ofstream obs_out(argv[4]), epoch_out(argv[5]);
    obs_out << std::setprecision(10) << "tow,sat,sys,signal,el_deg,snr,lli,resid_mps\n";
    epoch_out << std::setprecision(10)
              << "tow,n,lib_ok,lib_vx,lib_vy,lib_vz,rob_ok,rob_vx,rob_vy,rob_vz,rob_dropped,"
                 "ref_x,ref_y,ref_z,ref_vx,ref_vy,ref_vz\n";

    const double c = constants::SPEED_OF_LIGHT;
    ObservationData rover;
    while (rover_reader.readObservationEpoch(rover)) {
        const auto it = truth.find(std::llround(rover.time.tow * 1000.0));
        if (it == truth.end()) continue;
        const Truth& t = it->second;

        // Same satellite-state steps as solveVelocityFromObservations().
        std::vector<const Observation*> sources;
        std::vector<spp_velocity::DopplerObservation> rows;
        for (const auto& o : rover.observations) {
            if (!o.valid || !o.has_doppler || o.doppler == 0.0) continue;
            const double travel = o.has_pseudorange && o.pseudorange > 0.0 ? o.pseudorange / c : 0.075;
            GNSSTime tx = rover.time - travel;
            Eigen::Vector3d sat_pos, sat_vel;
            double clock = 0.0, drift = 0.0;
            if (!nav.calculateSatelliteState(o.satellite, tx, sat_pos, sat_vel, clock, drift)) continue;
            tx = tx - clock;
            if (!nav.calculateSatelliteState(o.satellite, tx, sat_pos, sat_vel, clock, drift)) continue;
            const double angle = constants::OMEGA_E * (sat_pos - t.pos).norm() / c;
            Eigen::Matrix3d earth_rotation;
            earth_rotation << std::cos(angle), std::sin(angle), 0.0, -std::sin(angle), std::cos(angle), 0.0,
                              0.0, 0.0, 1.0;
            const double frequency = signalFrequencyHz(o.signal, nav.getEphemeris(o.satellite, tx));
            if (!(frequency > 0.0)) continue;
            spp_velocity::DopplerObservation d;
            d.satellite_position_ecef = earth_rotation * sat_pos;
            d.satellite_velocity_ecef = earth_rotation * sat_vel;
            d.satellite_clock_drift = drift;
            d.doppler_hz = o.doppler;
            d.frequency_hz = frequency;
            d.elevation_rad = nav.calculateGeometry(t.pos, d.satellite_position_ecef).elevation;
            rows.push_back(d);
            sources.push_back(&o);
        }
        if (rows.size() < 5) continue;

        // Residual at the reference velocity; the receiver clock drift is the
        // per-epoch median, so one bad observation cannot shift the others.
        std::vector<double> res;
        for (const auto& d : rows) res.push_back(residual(d, t.pos, t.vel, 0.0));
        std::vector<double> sorted = res;
        std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
        const double median = sorted[sorted.size() / 2];
        for (size_t i = 0; i < rows.size(); ++i) {
            obs_out << rover.time.tow << ',' << sources[i]->satellite.toString() << ','
                    << static_cast<int>(sources[i]->satellite.system) << ','
                    << static_cast<int>(sources[i]->signal) << ',' << rows[i].elevation_rad * 180.0 / M_PI << ','
                    << sources[i]->snr << ',' << sources[i]->loss_of_lock << ',' << res[i] - median << '\n';
        }

        const auto library = spp_velocity::solveVelocityFromObservations(rover, nav, t.pos, kSigmaMps);
        std::vector<spp_velocity::DopplerObservation> kept = rows;
        spp_velocity::DopplerVelocityResult robust;
        int dropped = 0;
        for (;;) {
            robust = spp_velocity::solveVelocity(kept, t.pos, kSigmaMps, 4);
            if (!robust.ok || kept.size() <= 5) break;
            double worst = 0.0;
            size_t worst_index = 0;
            for (size_t i = 0; i < kept.size(); ++i) {
                const double z = std::abs(residual(kept[i], t.pos, robust.velocity_ecef,
                                                   robust.receiver_clock_drift * c)) /
                                 sigmaFor(kept[i].elevation_rad);
                if (z > worst) {
                    worst = z;
                    worst_index = i;
                }
            }
            if (worst <= kRejectSigma) break;
            kept.erase(kept.begin() + static_cast<long>(worst_index));
            ++dropped;
        }

        epoch_out << rover.time.tow << ',' << rows.size() << ',' << library.ok;
        writeVector(epoch_out, library.ok, library.velocity_ecef);
        epoch_out << ',' << robust.ok;
        writeVector(epoch_out, robust.ok, robust.velocity_ecef);
        epoch_out << ',' << dropped;
        writeVector(epoch_out, true, t.pos);
        writeVector(epoch_out, true, t.vel);
        epoch_out << '\n';
    }
    return 0;
}
