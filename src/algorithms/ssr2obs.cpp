#include <libgnss++/algorithms/ssr2obs.hpp>

#include <libgnss++/algorithms/ppp_clas.hpp>

#include <cmath>
#include <map>

namespace libgnss::ssr2obs {

namespace {

int signalFreqIndex(const OSRCorrection& osr, SignalType sig) {
    for (int f = 0; f < osr.num_frequencies; ++f) {
        if (osr.signals[f] == sig) {
            return f;
        }
    }
    return -1;
}

}  // namespace

ObservationData buildSsrCorrectedObservations(
    const ObservationData& raw,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config) {
    ObservationData out = raw;
    std::map<SatelliteId, const OSRCorrection*> by_sat;
    for (const auto& row : osr_corrections) {
        if (row.valid) {
            by_sat[row.satellite] = &row;
        }
    }
    const auto policy = config.clas_correction_application_policy;
    for (auto& obs : out.observations) {
        const auto it = by_sat.find(obs.satellite);
        if (it == by_sat.end() || !it->second) {
            continue;
        }
        const OSRCorrection& osr = *it->second;
        const int fi = signalFreqIndex(osr, obs.signal);
        if (fi < 0) {
            continue;
        }
        const auto applied =
            ppp_clas::selectAppliedOsrCorrections(osr, fi, policy);
        if (obs.has_pseudorange && std::isfinite(obs.pseudorange)) {
            obs.pseudorange -= applied.pseudorange_correction_m;
        }
        if (obs.has_carrier_phase && std::isfinite(obs.carrier_phase) &&
            osr.wavelengths[fi] > 0.0) {
            obs.carrier_phase -=
                applied.carrier_phase_correction_m / osr.wavelengths[fi];
        }
    }
    return out;
}

}  // namespace libgnss::ssr2obs
