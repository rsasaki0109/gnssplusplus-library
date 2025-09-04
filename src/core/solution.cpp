#include <libgnss++/core/solution.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace libgnss {

double PositionSolution::getHorizontalAccuracy() const {
    // 95% confidence (2-sigma)
    double sigma_h = std::sqrt(position_covariance(0,0) + position_covariance(1,1));
    return 2.0 * sigma_h;
}

double PositionSolution::getVerticalAccuracy() const {
    // 95% confidence (2-sigma)
    double sigma_v = std::sqrt(position_covariance(2,2));
    return 2.0 * sigma_v;
}

double PositionSolution::get3DAccuracy() const {
    // 95% confidence
    double sigma_3d = std::sqrt(position_covariance(0,0) + position_covariance(1,1) + position_covariance(2,2));
    return 2.0 * sigma_3d;
}

std::string PositionSolution::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "Time: " << time.week << " " << time.tow << "\n";
    oss << "Position (ECEF): " << position_ecef.transpose() << "\n";
    oss << "Position (Geodetic): " << position_geodetic.latitude * 180.0/M_PI 
        << " " << position_geodetic.longitude * 180.0/M_PI 
        << " " << position_geodetic.height << "\n";
    oss << "Status: " << static_cast<int>(status) << "\n";
    oss << "Satellites: " << num_satellites << "\n";
    oss << "PDOP: " << pdop << "\n";
    return oss.str();
}

std::string PositionSolution::toNMEA() const {
    // Simple GGA message implementation
    std::ostringstream oss;
    oss << "$GPGGA,";
    // Time would be formatted here
    oss << "000000.00,";
    // Latitude/longitude formatting would be here
    oss << "0000.0000,N,00000.0000,E,";
    oss << (isValid() ? "1" : "0") << ",";
    oss << std::setfill('0') << std::setw(2) << num_satellites << ",";
    oss << std::fixed << std::setprecision(1) << hdop << ",";
    oss << std::fixed << std::setprecision(1) << position_geodetic.height << ",M,0.0,M,,";
    oss << "*00\r\n"; // Checksum would be calculated
    return oss.str();
}

Solution::SolutionStatistics Solution::calculateStatistics(const Vector3d& reference_position) const {
    SolutionStatistics stats;
    
    if (solutions.empty()) {
        return stats;
    }
    
    stats.total_epochs = solutions.size();
    
    double sum_hdop = 0.0, sum_vdop = 0.0, sum_pdop = 0.0;
    double sum_processing_time = 0.0, sum_satellites = 0.0;
    
    for (const auto& sol : solutions) {
        if (sol.isValid()) {
            stats.valid_solutions++;
            if (sol.isFixed()) {
                stats.fixed_solutions++;
            } else if (sol.status == SolutionStatus::FLOAT) {
                stats.float_solutions++;
            }
            
            sum_hdop += sol.hdop;
            sum_vdop += sol.vdop;
            sum_pdop += sol.pdop;
            sum_processing_time += sol.processing_time_ms;
            sum_satellites += sol.num_satellites;
        }
    }
    
    if (stats.valid_solutions > 0) {
        stats.availability_rate = static_cast<double>(stats.valid_solutions) / stats.total_epochs;
        stats.fix_rate = static_cast<double>(stats.fixed_solutions) / stats.valid_solutions;
        
        stats.mean_hdop = sum_hdop / stats.valid_solutions;
        stats.mean_vdop = sum_vdop / stats.valid_solutions;
        stats.mean_pdop = sum_pdop / stats.valid_solutions;
        stats.mean_processing_time = sum_processing_time / stats.valid_solutions;
        stats.mean_satellites = sum_satellites / stats.valid_solutions;
        
        // Calculate RMS errors if reference position is provided
        if (reference_position.norm() > 0) {
            double sum_h_error_sq = 0.0, sum_v_error_sq = 0.0, sum_3d_error_sq = 0.0;
            
            for (const auto& sol : solutions) {
                if (sol.isValid()) {
                    Vector3d error = sol.position_ecef - reference_position;
                    sum_h_error_sq += error(0)*error(0) + error(1)*error(1);
                    sum_v_error_sq += error(2)*error(2);
                    sum_3d_error_sq += error.squaredNorm();
                }
            }
            
            stats.rms_horizontal = std::sqrt(sum_h_error_sq / stats.valid_solutions);
            stats.rms_vertical = std::sqrt(sum_v_error_sq / stats.valid_solutions);
            stats.rms_3d = std::sqrt(sum_3d_error_sq / stats.valid_solutions);
        }
    }
    
    return stats;
}

bool Solution::writeToFile(const std::string& filename, const std::string& format) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    file << "% LibGNSS++ Position Solution\n";
    file << "% Format: " << format << "\n";
    file << "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP\n";
    
    for (const auto& sol : solutions) {
        file << std::fixed << std::setprecision(6);
        file << sol.time.week << " " << sol.time.tow << " ";
        file << sol.position_ecef(0) << " " << sol.position_ecef(1) << " " << sol.position_ecef(2) << " ";
        file << sol.position_geodetic.latitude * 180.0/M_PI << " ";
        file << sol.position_geodetic.longitude * 180.0/M_PI << " ";
        file << sol.position_geodetic.height << " ";
        file << static_cast<int>(sol.status) << " ";
        file << sol.num_satellites << " ";
        file << sol.pdop << "\n";
    }
    
    return true;
}

bool Solution::writeKML(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    file << "<Document>\n";
    file << "<name>LibGNSS++ Solution</name>\n";
    file << "<Placemark>\n";
    file << "<name>GNSS Track</name>\n";
    file << "<LineString>\n";
    file << "<coordinates>\n";
    
    for (const auto& sol : solutions) {
        if (sol.isValid()) {
            file << std::fixed << std::setprecision(8);
            file << sol.position_geodetic.longitude * 180.0/M_PI << ",";
            file << sol.position_geodetic.latitude * 180.0/M_PI << ",";
            file << sol.position_geodetic.height << "\n";
        }
    }
    
    file << "</coordinates>\n";
    file << "</LineString>\n";
    file << "</Placemark>\n";
    file << "</Document>\n";
    file << "</kml>\n";
    
    return true;
}

bool Solution::writeNMEA(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    for (const auto& sol : solutions) {
        file << sol.toNMEA();
    }
    
    return true;
}

const PositionSolution* Solution::getSolution(const GNSSTime& time) const {
    for (const auto& sol : solutions) {
        if (sol.time == time) {
            return &sol;
        }
    }
    return nullptr;
}

void Solution::sortByTime() {
    std::sort(solutions.begin(), solutions.end(), 
              [](const PositionSolution& a, const PositionSolution& b) {
                  return a.time < b.time;
              });
}

std::pair<GNSSTime, GNSSTime> Solution::getTimeSpan() const {
    if (solutions.empty()) {
        return {GNSSTime(), GNSSTime()};
    }
    
    auto minmax = std::minmax_element(solutions.begin(), solutions.end(),
                                     [](const PositionSolution& a, const PositionSolution& b) {
                                         return a.time < b.time;
                                     });
    
    return {minmax.first->time, minmax.second->time};
}

} // namespace libgnss
