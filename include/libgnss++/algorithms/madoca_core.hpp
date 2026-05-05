#pragma once

#include <libgnss++/algorithms/madoca_parity.hpp>
#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/types.hpp>
#include <libgnss++/io/qzss_l6.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace libgnss::algorithms::madoca_core {

inline constexpr double kDefaultCorrectionMaxAgeSeconds = 120.0;

enum class CorrectionSource {
    Unknown,
    L6E,
    L6D,
    ExternalFixture,
};

struct NativeCoreConfig {
    int preferred_network_id = 0;
    bool use_l6e_orbit_clock_bias = true;
    bool use_l6d_ionosphere = true;
    bool enable_ambiguity_resolution = false;
    double correction_max_age_seconds = kDefaultCorrectionMaxAgeSeconds;
    std::set<GNSSSystem> allowed_systems;
};

struct CorrectionSummary {
    std::size_t total = 0;
    std::size_t satellites = 0;
    std::size_t orbit = 0;
    std::size_t clock = 0;
    std::size_t code_bias = 0;
    std::size_t phase_bias = 0;
    std::size_t atmosphere = 0;
};

SatelliteId satelliteIdFromMadocalibSat(int sat);
GNSSTime gnssTimeFromMadocalibTime(madoca_parity::GTime time);
double l1StecDelayFactorMeters(const SatelliteId& satellite);
bool mionoDelayMetersToStecTecu(const SatelliteId& satellite,
                                double delay_m,
                                double& stec_tecu);
std::vector<SSROrbitClockCorrection> materializeMionoCorrections(
    const madoca_parity::MionoCorrResult& result,
    int network_id = 0);
std::vector<SSROrbitClockCorrection> materializeL6DCorrections(
    const std::vector<madoca_parity::MionoCorrResult>& results,
    int network_id = 0);
std::vector<SSROrbitClockCorrection> materializeL6ECorrections(
    const std::vector<qzss_l6::CssrEpoch>& epochs,
    int preferred_network_id = 0);
std::vector<madoca_parity::MionoCorrResult> decodeL6DFile(
    const std::string& l6d_file,
    int gps_week,
    const Vector3d& receiver_position_ecef);
std::vector<madoca_parity::MionoCorrResult> decodeL6DFiles(
    const std::vector<std::string>& l6d_files,
    int gps_week,
    const Vector3d& receiver_position_ecef);
std::vector<qzss_l6::CssrEpoch> decodeL6EFile(const std::string& l6e_file,
                                              int gps_week);
std::vector<qzss_l6::CssrEpoch> decodeL6EFiles(
    const std::vector<std::string>& l6e_files, int gps_week);
ppp_shared::PPPConfig makePPPConfig(
    const NativeCoreConfig& core_config,
    const ppp_shared::PPPConfig& base_config = ppp_shared::PPPConfig{});

class NativeCorrectionStore {
public:
    void clear();

    void addCorrection(const SSROrbitClockCorrection& correction,
                       CorrectionSource source = CorrectionSource::Unknown);
    void addCorrections(const std::vector<SSROrbitClockCorrection>& corrections,
                        CorrectionSource source = CorrectionSource::Unknown);
    void addMionoCorrections(const madoca_parity::MionoCorrResult& result,
                             int network_id = 0);
    void addL6DCorrections(const std::vector<madoca_parity::MionoCorrResult>& results,
                           int network_id = 0);
    void addL6ECorrections(const std::vector<qzss_l6::CssrEpoch>& epochs,
                           int preferred_network_id = 0);

    bool empty() const { return summary_.total == 0; }
    const CorrectionSummary& summary() const { return summary_; }
    const std::map<CorrectionSource, std::size_t>& sourceCounts() const {
        return source_counts_;
    }

    const SSRProducts& products() const { return products_; }
    SSRProducts exportProducts() const { return products_; }

    bool interpolateCorrection(const SatelliteId& sat,
                               const GNSSTime& time,
                               Vector3d& orbit_correction_ecef,
                               double& clock_correction_m,
                               double* ura_sigma_m = nullptr,
                               std::map<uint8_t, double>* code_bias_m = nullptr,
                               std::map<uint8_t, double>* phase_bias_m = nullptr,
                               std::map<std::string, std::string>* atmos_tokens = nullptr,
                               int preferred_network_id = 0) const;

private:
    SSRProducts products_;
    CorrectionSummary summary_;
    std::set<SatelliteId> satellites_;
    std::map<CorrectionSource, std::size_t> source_counts_;
};

bool loadCorrections(PPPProcessor& processor, const NativeCorrectionStore& store);
bool loadL6DCorrections(PPPProcessor& processor,
                        const std::vector<madoca_parity::MionoCorrResult>& results,
                        int network_id = 0);
bool loadL6DFile(PPPProcessor& processor,
                 const std::string& l6d_file,
                 int gps_week,
                 const Vector3d& receiver_position_ecef,
                 int network_id = 0);
bool loadL6DFiles(PPPProcessor& processor,
                  const std::vector<std::string>& l6d_files,
                  int gps_week,
                  const Vector3d& receiver_position_ecef,
                  int network_id = 0);
bool loadL6ECorrections(PPPProcessor& processor,
                        const std::vector<qzss_l6::CssrEpoch>& epochs,
                        int preferred_network_id = 0);
bool loadL6EFile(PPPProcessor& processor,
                 const std::string& l6e_file,
                 int gps_week,
                 int preferred_network_id = 0);
bool loadL6EFiles(PPPProcessor& processor,
                  const std::vector<std::string>& l6e_files,
                  int gps_week,
                  int preferred_network_id = 0);

}  // namespace libgnss::algorithms::madoca_core
