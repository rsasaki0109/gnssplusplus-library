#pragma once

#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>

#include <Eigen/Dense>

#include <array>
#include <map>
#include <memory>
#include <vector>

namespace libgnss::ppp_clasnat {

constexpr int kMaxSat = 75;
constexpr int kNfreq = 3;
constexpr int kNp = 3;
constexpr int kNi = kMaxSat;
constexpr int kAmbStart = kNp + kNi;
constexpr int kNx = kAmbStart + kMaxSat * kNfreq;

struct ObsRow {
    GNSSTime time;
    SatelliteId satellite;
    std::array<double, kNfreq> carrier_phase_cycles{};
    std::array<double, kNfreq> pseudorange_m{};
    std::array<double, kNfreq> doppler_hz{};
    std::array<double, kNfreq> snr_dbhz{};
    std::array<SignalType, kNfreq> signals{};
};

struct NavData {
    std::map<SatelliteId, std::vector<Ephemeris>> ephemerides;
};

struct Solution {
    GNSSTime time;
    Vector3d rr = Vector3d::Zero();
    Eigen::Matrix3d Qr = Eigen::Matrix3d::Zero();
    SolutionStatus status = SolutionStatus::NONE;
    int ns = 0;
    double ratio = 0.0;
};

struct SatState {
    std::array<int, kNfreq> lock{};
    std::array<int, kNfreq> outc{};
    std::array<int, kNfreq> slip{};
    std::array<int, kNfreq> fix{};
    std::array<double, kNfreq> resp{};
    std::array<double, kNfreq> resc{};
    double phw = 0.0;
};

struct RtkState {
    RtkState();
    ~RtkState();
    RtkState(RtkState&&) noexcept;
    RtkState& operator=(RtkState&&) noexcept;

    RtkState(const RtkState&) = delete;
    RtkState& operator=(const RtkState&) = delete;

    Eigen::VectorXd x = Eigen::VectorXd::Zero(kNx);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(kNx, kNx);
    Eigen::VectorXd xa = Eigen::VectorXd::Zero(kNx);
    Eigen::MatrixXd Pa = Eigen::MatrixXd::Zero(kNx, kNx);
    std::array<SatState, kMaxSat> ssat{};
    Solution sol;
    GNSSTime last_time;
    bool has_last_time = false;
    bool initialized = false;
    double last_ar_ratio = 0.0;
    int last_fixed_ambiguities = 0;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct EpochResult {
    PositionSolution solution;
    bool updated = false;
    bool fixed = false;
    int measurement_count = 0;
    int active_state_count = 0;
};

void reset(RtkState& state);

EpochResult runEpoch(const ObservationData& obs,
                     const libgnss::NavigationData& nav,
                     const SSRProducts& ssr,
                     RtkState& rtk,
                     const ppp_shared::PPPConfig& config);

}  // namespace libgnss::ppp_clasnat
