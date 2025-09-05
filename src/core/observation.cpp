#include <libgnss++/core/observation.hpp>
#include <algorithm>
#include <set>

namespace libgnss {

std::vector<Observation> ObservationData::getObservations(const SatelliteId& sat) const {
    std::vector<Observation> result;
    for (const auto& obs : observations) {
        if (obs.satellite == sat) {
            result.push_back(obs);
        }
    }
    return result;
}

std::vector<Observation> ObservationData::getObservations(GNSSSystem system) const {
    std::vector<Observation> result;
    for (const auto& obs : observations) {
        if (obs.satellite.system == system) {
            result.push_back(obs);
        }
    }
    return result;
}

std::vector<Observation> ObservationData::getObservations(SignalType signal) const {
    std::vector<Observation> result;
    for (const auto& obs : observations) {
        if (obs.signal == signal) {
            result.push_back(obs);
        }
    }
    return result;
}

std::vector<SatelliteId> ObservationData::getSatellites() const {
    std::set<SatelliteId> unique_sats;
    for (const auto& obs : observations) {
        unique_sats.insert(obs.satellite);
    }
    return std::vector<SatelliteId>(unique_sats.begin(), unique_sats.end());
}

size_t ObservationData::getNumSatellites() const {
    return getSatellites().size();
}

bool ObservationData::hasObservation(const SatelliteId& sat, SignalType signal) const {
    for (const auto& obs : observations) {
        if (obs.satellite == sat && obs.signal == signal) {
            return true;
        }
    }
    return false;
}

const Observation* ObservationData::getObservation(const SatelliteId& sat, SignalType signal) const {
    for (const auto& obs : observations) {
        if (obs.satellite == sat && obs.signal == signal) {
            return &obs;
        }
    }
    return nullptr;
}

std::vector<Observation> ObservationData::filterByElevation(double min_elevation, 
                                                          const Vector3d& receiver_pos) const {
    std::vector<Observation> result;
    // Simplified implementation - would need satellite positions to calculate elevation
    for (const auto& obs : observations) {
        if (obs.valid) {
            result.push_back(obs);
        }
    }
    return result;
}

std::vector<Observation> ObservationData::filterBySNR(double min_snr) const {
    std::vector<Observation> result;
    for (const auto& obs : observations) {
        if (obs.snr >= min_snr && obs.valid) {
            result.push_back(obs);
        }
    }
    return result;
}

void ObservationData::applyQualityControl(double min_elevation, 
                                         double min_snr,
                                         const Vector3d& receiver_pos) {
    observations.erase(
        std::remove_if(observations.begin(), observations.end(),
                      [min_snr](const Observation& obs) {
                          return obs.snr < min_snr || !obs.valid;
                      }),
        observations.end());
}

MatrixXd ObservationData::calculateGeometryMatrix(const Vector3d& receiver_pos,
                                                 const std::vector<Vector3d>& sat_positions) const {
    size_t n_sats = sat_positions.size();
    MatrixXd H = MatrixXd::Zero(n_sats, 4);
    
    for (size_t i = 0; i < n_sats; ++i) {
        Vector3d los = sat_positions[i] - receiver_pos;
        double range = los.norm();
        
        if (range > 0) {
            H(i, 0) = -los(0) / range;  // dx
            H(i, 1) = -los(1) / range;  // dy
            H(i, 2) = -los(2) / range;  // dz
            H(i, 3) = 1.0;              // dt
        }
    }
    
    return H;
}

ObservationData::DOPValues ObservationData::calculateDOP(const Vector3d& receiver_pos,
                                                        const std::vector<Vector3d>& sat_positions) const {
    DOPValues dop;
    
    if (sat_positions.size() < 4) {
        return dop; // Return default high values
    }
    
    MatrixXd H = calculateGeometryMatrix(receiver_pos, sat_positions);
    MatrixXd Q = (H.transpose() * H).inverse();
    
    dop.gdop = std::sqrt(Q.trace());
    dop.pdop = std::sqrt(Q(0,0) + Q(1,1) + Q(2,2));
    dop.hdop = std::sqrt(Q(0,0) + Q(1,1));
    dop.vdop = std::sqrt(Q(2,2));
    dop.tdop = std::sqrt(Q(3,3));
    
    return dop;
}

ObservationData::EpochStats ObservationData::getStats(const Vector3d& receiver_pos) const {
    EpochStats stats;
    stats.total_observations = observations.size();
    
    double snr_sum = 0.0;
    std::set<SatelliteId> unique_sats;
    std::set<GNSSSystem> unique_systems;
    
    for (const auto& obs : observations) {
        if (obs.valid) {
            stats.valid_observations++;
            snr_sum += obs.snr;
        }
        unique_sats.insert(obs.satellite);
        unique_systems.insert(obs.satellite.system);
    }
    
    stats.num_satellites = unique_sats.size();
    stats.num_systems = unique_systems.size();
    
    if (stats.valid_observations > 0) {
        stats.average_snr = snr_sum / stats.valid_observations;
    }
    
    return stats;
}

// ObservationSeries methods
const ObservationData* ObservationSeries::getEpoch(const GNSSTime& time) const {
    for (const auto& epoch : epochs) {
        if (epoch.time == time) {
            return &epoch;
        }
    }
    return nullptr;
}

std::vector<ObservationData> ObservationSeries::getEpochs(const GNSSTime& start, 
                                                         const GNSSTime& end) const {
    std::vector<ObservationData> result;
    for (const auto& epoch : epochs) {
        if (epoch.time <= end && start <= epoch.time) {
            result.push_back(epoch);
        }
    }
    return result;
}

void ObservationSeries::sortByTime() {
    std::sort(epochs.begin(), epochs.end(),
              [](const ObservationData& a, const ObservationData& b) {
                  return a.time < b.time;
              });
}

std::pair<GNSSTime, GNSSTime> ObservationSeries::getTimeSpan() const {
    if (epochs.empty()) {
        return {GNSSTime(), GNSSTime()};
    }
    
    auto minmax = std::minmax_element(epochs.begin(), epochs.end(),
                                     [](const ObservationData& a, const ObservationData& b) {
                                         return a.time < b.time;
                                     });
    
    return {minmax.first->time, minmax.second->time};
}

} // namespace libgnss
