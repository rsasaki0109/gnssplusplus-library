// GTSAM backend for FGOProcessor::optimizeProblem (Phase 1, RTK/DD-focused).
//
// Consumes the exact same FGOProcessor::FGOProblem the native Eigen backend
// (fgo.cpp) consumes and produces an equivalent FGOProcessor::FGOResult.
// Everything in this file is compiled only when GNSSPP_HAS_GTSAM is defined
// (see CMakeLists.txt: only added to gnss_lib_noopt when find_package(GTSAM)
// succeeds), so no other translation unit needs to guard against a missing
// GTSAM install.
//
// --- Observation-convention note (see docs/gtsam_backend_design.md risk #2) ---
// gtsam::gnss::DoubleDifferenceData::observed() computes
//     (rovRef - baseRef) - (rovTarget - baseTarget)
// while libgnss's FGOProcessor::DoubleDifferencePseudorangeFactor stores
//     observed_dd_pseudorange_m = (satellite_* - base_satellite_*)
//                                - (reference_* - base_reference_*)
// i.e. (target - reference), the mirror image of GTSAM's (ref - target).
// Feeding libgnss's "satellite" (target) into GTSAM's "Ref" slot and
// libgnss's "reference_satellite" into GTSAM's "Target" slot makes the two
// conventions coincide exactly (verified by the runtime assertion below,
// and by the DD carrier equivalent). The same swap applies to the ambiguity
// keys of DoubleDifferenceCarrierPhaseFactor: GTSAM forms
// lam*(ambRef - ambTarget); libgnss tracks a single lumped DD ambiguity
// equal to lam*(N_target - N_reference). So ambRef <- the real ambiguity
// node (seeded with libgnss's lumped value, in cycles) and ambTarget <- a
// single shared dummy node pinned to 0 with a tight prior.

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GnssCommon.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PseudorangeFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace libgnss {
namespace {

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Symbol;
using SharedNoise = gtsam::SharedNoiseModel;

// MF-AR step 1: restrict a per-epoch LAMBDA candidate set to independent
// satellites -- keep at most one ambiguity per satellite (primary band
// preferred, else the longest-wavelength secondary; ties broken by index for
// determinism). Multiple frequency bands of one satellite share the same
// line-of-sight, so they contribute little to the DD geometry but do inflate
// the all-or-nothing integer ratio test and lower the ratio. The dropped
// bands' DD factors remain in the graph and still constrain the float; they
// are simply not forced into the integer search. No-op for single-frequency
// DD (each satellite already has a single band). Input indices are assumed
// unique (one DD carrier factor per (sat,signal) per epoch).
std::vector<std::size_t> selectOneBandPerSatellite(
    const std::vector<std::size_t>& indices,
    const FGOProcessor::FGOProblem& problem) {
    std::map<SatelliteId, std::size_t> best_by_satellite;
    for (std::size_t idx : indices) {
        if (idx >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto& amb = problem.ambiguity_states[idx];
        const auto it = best_by_satellite.find(amb.satellite);
        if (it == best_by_satellite.end()) {
            best_by_satellite.emplace(amb.satellite, idx);
            continue;
        }
        const auto& cur = problem.ambiguity_states[it->second];
        const bool amb_primary =
            signal_policy::isPrimarySignal(amb.satellite.system, amb.signal);
        const bool cur_primary =
            signal_policy::isPrimarySignal(cur.satellite.system, cur.signal);
        bool replace = false;
        if (amb_primary != cur_primary) {
            replace = amb_primary;  // prefer the primary band
        } else if (amb.wavelength_m != cur.wavelength_m) {
            replace = amb.wavelength_m > cur.wavelength_m;  // longer wavelength: easier to fix
        } else {
            replace = idx < it->second;  // deterministic
        }
        if (replace) {
            it->second = idx;
        }
    }
    std::vector<std::size_t> selected;
    selected.reserve(best_by_satellite.size());
    for (const auto& [sat, idx] : best_by_satellite) {
        (void)sat;
        selected.push_back(idx);
    }
    return selected;
}

// Mirror of fgo.cpp's clockBiasGroup(): which receiver-clock group a system
// belongs to. GPS+QZSS share one clock; every other constellation gets its own
// inter-system-bias clock. This matches the per-constellation bias columns the
// native Eigen backend forms when config.use_inter_system_biases is set.
GNSSSystem clockBiasGroup(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return GNSSSystem::GPS;
        case GNSSSystem::Galileo:
        case GNSSSystem::BeiDou:
        case GNSSSystem::GLONASS:
        case GNSSSystem::NavIC:
            return system;
        default:
            return GNSSSystem::UNKNOWN;
    }
}

// Key spaces: 'x' rover position per epoch, 'c' receiver base-clock bias [s]
// per epoch (the GPS/QZSS group), 'i' a GLOBAL (time-constant) inter-system
// bias [s] per non-GPS constellation shared across every epoch -- matching the
// native backend, which carries one bias column per constellation, not a fresh
// clock each epoch, 'a' per-ambiguity node (cycles for DD, meters for
// undifferenced), 'z' a single shared dummy ambiguity node pinned at 0.
inline gtsam::Key positionKey(std::size_t epoch) { return Symbol('x', epoch); }
inline gtsam::Key clockKey(std::size_t epoch) { return Symbol('c', epoch); }
inline gtsam::Key isbKey(int group_ordinal) { return Symbol('i', group_ordinal); }
inline gtsam::Key ambiguityKey(std::size_t index) { return Symbol('a', index); }
inline gtsam::Key dummyAmbiguityKey() { return Symbol('z', 0); }
// Milestone 2b IMU states: 'v' body velocity in nav (ENU), 'b' IMU bias, per epoch.
inline gtsam::Key velocityKey(std::size_t epoch) { return Symbol('v', epoch); }
inline gtsam::Key biasKey(std::size_t epoch) { return Symbol('b', epoch); }

// ENU-from-ECEF rotation whose columns are the East/North/Up basis vectors
// expressed in ECEF, i.e. ecef_vec = R_ecef_enu * enu_vec. Matches
// core/coordinates.hpp enu2ecef(). Used to build the ecef_T_nav Pose3 that
// the DD '...FactorArm' factors and the IMU factor share (nav = local ENU).
inline gtsam::Rot3 ecefFromEnuRotation(double lat, double lon) {
    const double sinlat = std::sin(lat), coslat = std::cos(lat);
    const double sinlon = std::sin(lon), coslon = std::cos(lon);
    gtsam::Matrix3 R;
    R << -sinlon, -sinlat * coslon, coslat * coslon,
          coslon, -sinlat * sinlon, coslat * sinlon,
          0.0,     coslat,          sinlat;
    return gtsam::Rot3(R);
}

// Stable small ordinal per clock group. GPS/QZSS are group 0 (the base clock,
// no ISB node); every other constellation gets its own global ISB node. UNKNOWN
// groups (SBAS etc.) fold into GPS(0). When inter-system biases are disabled
// everything is group 0 (single base clock, no ISB), matching the native
// backend with use_inter_system_biases=false.
int clockGroupOrdinal(GNSSSystem group, bool use_inter_system_biases) {
    if (!use_inter_system_biases) {
        return 0;
    }
    switch (group) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::Galileo: return 1;
        case GNSSSystem::BeiDou: return 2;
        case GNSSSystem::GLONASS: return 3;
        case GNSSSystem::NavIC: return 4;
        default: return 0;
    }
}

// Plain-Euclidean geometric range and its 1x3 Jacobian w.r.t. the receiver.
//
// NOTE ON SAGNAC: the native backend earth-rotation-corrects each satellite
// position at build time (earthRotationCorrected, using the seed position) and
// then forms the pseudorange residual with a *plain* Euclidean norm. GTSAM's
// gtsam::gnss::geodist would apply its OWN first-order Sagnac correction on top
// of those already-corrected positions -- a double correction that biases the
// undifferenced-pseudorange position solution by ~1 m on multi-GNSS data (it
// mostly cancels in the double-difference, which is why the stock DD factors
// still reach cm parity). So these undifferenced factors use a plain norm to
// match the native backend exactly.
inline double plainRange(const Point3& satellite, const Point3& receiver,
                         gtsam::Matrix13* H_receiver) {
    const Point3 delta = receiver - satellite;
    const double range = delta.norm();
    if (H_receiver) {
        if (range > 0.0) {
            *H_receiver = (delta / range).transpose();
        } else {
            H_receiver->setZero();
        }
    }
    return range;
}

// Undifferenced pseudorange factor with a global inter-system bias node.
//   error = ||rcv - sat|| + c*(baseClock + isb) - measuredPseudorange
// Base clock is per epoch; isb is one global (time-constant) node per non-GPS
// constellation -- mirroring the native backend's per-epoch clock + shared
// per-constellation bias column.
class PseudorangeFactorISB : public gtsam::NoiseModelFactorN<Point3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISB(gtsam::Key position, gtsam::Key base_clock, gtsam::Key isb,
                         double measured_pseudorange, const Point3& satellite_position,
                         const gtsam::SharedNoiseModel& model)
        : Base(model, position, base_clock, isb),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position) {}

    gtsam::Vector evaluateError(const Point3& position, const double& base_clock,
                                const double& isb, gtsam::OptionalMatrixType H_position,
                                gtsam::OptionalMatrixType H_base_clock,
                                gtsam::OptionalMatrixType H_isb) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, position, H_position ? &H_range : nullptr);
        const double error =
            range + gtsam::gnss::C_LIGHT * (base_clock + isb) - measurement_;
        if (H_position) {
            *H_position = H_range;
        }
        if (H_base_clock) {
            *H_base_clock = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        if (H_isb) {
            *H_isb = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// GPS/base-group undifferenced pseudorange factor (no ISB node).
//   error = ||rcv - sat|| + c*baseClock - measuredPseudorange
class PseudorangeFactorPlain : public gtsam::NoiseModelFactorN<Point3, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double>;
    using Base::evaluateError;
    PseudorangeFactorPlain(gtsam::Key position, gtsam::Key base_clock,
                           double measured_pseudorange, const Point3& satellite_position,
                           const gtsam::SharedNoiseModel& model)
        : Base(model, position, base_clock),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position) {}

    gtsam::Vector evaluateError(const Point3& position, const double& base_clock,
                                gtsam::OptionalMatrixType H_position,
                                gtsam::OptionalMatrixType H_base_clock) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + gtsam::gnss::C_LIGHT * base_clock - measurement_;
        if (H_position) {
            *H_position = H_range;
        }
        if (H_base_clock) {
            *H_base_clock = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// --- Milestone 2a: Pose3 + lever-arm plumbing (docs/gtsam_backend_design.md) ---
//
// Undifferenced plain-norm pseudorange factors are Pose3 analogs of
// PseudorangeFactorPlain/ISB above: same "no double Sagnac correction"
// reasoning (the satellite position is already earth-rotation-corrected by
// the native builder), just evaluated at the ANTENNA position derived from
// a body Pose3 + lever arm via gtsam::gnss::LeverArm (antenna_ecef =
// pose.translation() + pose.rotation() * leverArm, exactly the convention
// used by the stock '...FactorArm' factors and by this project's own
// Stage-1 ESKF, see fusion_measurement.cpp). GTSAM's navigation module has
// no stock plain-norm Arm factor (only geodist-based ones), hence these.

class PseudorangeFactorPlainArm : public gtsam::NoiseModelFactorN<Pose3, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainArm(gtsam::Key pose, gtsam::Key base_clock,
                              double measured_pseudorange, const Point3& satellite_position,
                              const gtsam::gnss::LeverArm& arm,
                              const gtsam::SharedNoiseModel& model)
        : Base(model, pose, base_clock),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm) {}

    gtsam::Vector evaluateError(const Pose3& pose, const double& base_clock,
                                gtsam::OptionalMatrixType H_pose,
                                gtsam::OptionalMatrixType H_base_clock) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range + gtsam::gnss::C_LIGHT * base_clock - measurement_;
        if (H_pose) {
            *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        }
        if (H_base_clock) {
            *H_base_clock = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorISBArm : public gtsam::NoiseModelFactorN<Pose3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBArm(gtsam::Key pose, gtsam::Key base_clock, gtsam::Key isb,
                            double measured_pseudorange, const Point3& satellite_position,
                            const gtsam::gnss::LeverArm& arm,
                            const gtsam::SharedNoiseModel& model)
        : Base(model, pose, base_clock, isb),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm) {}

    gtsam::Vector evaluateError(const Pose3& pose, const double& base_clock, const double& isb,
                                gtsam::OptionalMatrixType H_pose,
                                gtsam::OptionalMatrixType H_base_clock,
                                gtsam::OptionalMatrixType H_isb) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error =
            range + gtsam::gnss::C_LIGHT * (base_clock + isb) - measurement_;
        if (H_pose) {
            *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        }
        if (H_base_clock) {
            *H_base_clock = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        if (H_isb) {
            *H_isb = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// Rotation-only gauge-fixing prior for a Pose3 state. Needed because the
// DD/undifferenced '...Arm' factors only observe the ANTENNA position
// (translation + R*leverArm): for a lever arm != 0 that is a rank<=3
// function of the full 6-dim pose tangent, so every pose has an exact
// (generic) 3-dim rotational null space that leaves gtsam::Marginals'
// Cholesky factorization singular. Since attitude is genuinely unobservable
// here (no IMU until milestone 2b), pinning it at its identity seed simply
// resolves the gauge freedom -- it is not a modeling approximation on top of
// real information, there is none to lose. Uses Pose3::rotation(H), whose
// Jacobian w.r.t. the pose tangent is structurally [I3 | 0] (rotation never
// depends on the translation tangent component, in any Pose3 chart), so the
// translation block of this factor's Jacobian is exactly zero and it cannot
// bias the estimated antenna position.
class Pose3RotationPrior : public gtsam::NoiseModelFactorN<Pose3> {
    Rot3 prior_;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3>;
    using Base::evaluateError;
    Pose3RotationPrior(gtsam::Key pose, const Rot3& prior_rotation,
                       const gtsam::SharedNoiseModel& model)
        : Base(model, pose), prior_(prior_rotation) {}

    gtsam::Vector evaluateError(const Pose3& pose,
                                gtsam::OptionalMatrixType H) const override {
        gtsam::Matrix36 H_rot;
        const Rot3 rot = pose.rotation(H_rot);
        if (!H) {
            return prior_.localCoordinates(rot);
        }
        gtsam::Matrix3 H_local;
        const gtsam::Vector3 error = prior_.localCoordinates(rot, {}, H_local);
        *H = H_local * H_rot;
        return error;
    }
};

// --- Milestone 2d: Non-Holonomic Constraint factor -------------------------
// Mirrors inuex35 buildfactor/nhc.py: a ground vehicle's body-frame lateral
// (Left, y) and vertical (Up, z) velocity is ~0. Binary factor on
// (Pose3 body-in-nav, Vector3 velocity-in-nav) with the 2-D residual
//   err = [ (R^T v_nav).y , (R^T v_nav).z ]      (R = nav_R_body = pose.rotation)
// Exact Jacobians come from GTSAM's own Rot3::unrotate / Pose3::rotation, so
// the linearization is correct in GTSAM's retract convention (no hand-derived
// skew term as in the Python CustomFactor). Caller gates it to moving,
// non-turning epochs. The NHC lever arm (evaluate at the rear axle) is left at
// zero -- the antenna lever is separate and the omega x lever term is a small
// correction inuex35 also defaults off (nhc_lever = 0).
class NonHolonomicFactor : public gtsam::NoiseModelFactorN<Pose3, gtsam::Vector3> {
 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, gtsam::Vector3>;
    using Base::evaluateError;
    NonHolonomicFactor(gtsam::Key pose, gtsam::Key velocity,
                       const gtsam::SharedNoiseModel& model)
        : Base(model, pose, velocity) {}

    gtsam::Vector evaluateError(const Pose3& pose, const gtsam::Vector3& v_nav,
                                gtsam::OptionalMatrixType H_pose,
                                gtsam::OptionalMatrixType H_vel) const override {
        gtsam::Matrix36 H_rot_pose;  // d(rot tangent)/d(pose tangent) = [I3|0]
        const Rot3 R = pose.rotation(H_pose ? &H_rot_pose : nullptr);
        gtsam::Matrix3 H_unrot_rot, H_unrot_v;
        const gtsam::Point3 v_body =
            R.unrotate(gtsam::Point3(v_nav), H_pose ? &H_unrot_rot : nullptr,
                       H_vel ? &H_unrot_v : nullptr);
        if (H_pose) {
            const gtsam::Matrix36 dvb_dpose = H_unrot_rot * H_rot_pose;  // 3x6
            gtsam::Matrix H = gtsam::Matrix::Zero(2, 6);
            H.row(0) = dvb_dpose.row(1);  // d(v_body.y)/d(pose)
            H.row(1) = dvb_dpose.row(2);  // d(v_body.z)/d(pose)
            *H_pose = H;
        }
        if (H_vel) {
            gtsam::Matrix H = gtsam::Matrix::Zero(2, 3);
            H.row(0) = H_unrot_v.row(1);
            H.row(1) = H_unrot_v.row(2);
            *H_vel = H;
        }
        return (gtsam::Vector(2) << v_body.y(), v_body.z()).finished();
    }
};

// Stationarity stats over an IMU sample sub-range [begin, end), mirroring the
// Stage-1 ESKF LooseCouplingProcessor::detectStationary(): std of the norm of
// (accel - accel_bias) and (gyro - gyro_bias), plus the median of the gyro
// deviation norm. Bias-referenced against the Stage-1 init bias (not a running
// estimate), exactly as inuex35's compute_zupt_stats recommends.
struct ImuWindowStats {
    int n = 0;
    double accel_std = 0.0;
    double gyro_std = 0.0;
    double gyro_median = 0.0;
    double yaw_rate_abs = 0.0;  ///< |mean(gyro_z - bias_z)|, for the NHC turn gate
};

inline ImuWindowStats imuWindowStats(const std::vector<ImuSample>& samples, std::size_t begin,
                                     std::size_t end, const Vector3d& accel_bias,
                                     const Vector3d& gyro_bias) {
    ImuWindowStats s;
    const std::size_t n = end > begin ? end - begin : 0;
    s.n = static_cast<int>(n);
    if (n == 0) {
        return s;
    }
    std::vector<double> accel_devs, gyro_devs;
    accel_devs.reserve(n);
    gyro_devs.reserve(n);
    double gyro_z_sum = 0.0;
    for (std::size_t k = begin; k < end; ++k) {
        accel_devs.push_back((samples[k].accel_raw - accel_bias).norm());
        gyro_devs.push_back((samples[k].gyro_raw_radps - gyro_bias).norm());
        gyro_z_sum += samples[k].gyro_raw_radps.z() - gyro_bias.z();
    }
    auto mean = [](const std::vector<double>& v) {
        double m = 0.0;
        for (double x : v) m += x;
        return m / static_cast<double>(v.size());
    };
    auto stddev = [&](const std::vector<double>& v) {
        const double m = mean(v);
        double s2 = 0.0;
        for (double x : v) {
            const double d = x - m;
            s2 += d * d;
        }
        return std::sqrt(s2 / static_cast<double>(v.size()));
    };
    std::vector<double> g = gyro_devs;
    std::sort(g.begin(), g.end());
    s.accel_std = stddev(accel_devs);
    s.gyro_std = stddev(gyro_devs);
    s.gyro_median = g[g.size() / 2];
    s.yaw_rate_abs = std::abs(gyro_z_sum / static_cast<double>(n));
    return s;
}

SharedNoise makeNoise(double sigma_m, bool robust, double huber_threshold_sigma) {
    const double safe_sigma = std::max(1e-9, sigma_m);
    SharedNoise base = gtsam::noiseModel::Isotropic::Sigma(1, safe_sigma);
    if (robust && huber_threshold_sigma > 0.0) {
        return gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(huber_threshold_sigma),
            base);
    }
    return base;
}


// Result of the mini DDPR-only LS anchor solve (FGOConfig::use_ddpr_anchor;
// port of the inuex35 reference's utils/ls_solvers.py ddpr_only_position).
// `pose` is a body-in-nav-ENU Pose3, the SAME convention as the main graph's
// positionKey(i) -- i.e. translation() is the body position in nav-ENU, NOT
// the antenna position (apply the caller's antennaOf()/lever-arm convention
// to get the antenna ECEF, exactly like the main loop does for pose_i).
struct DdprAnchorResult {
    bool ok = false;
    Pose3 pose;
    int n_active = 0;
    double res_rms = std::numeric_limits<double>::infinity();
};

// Cross-checks that gtsam::gnss::DoubleDifferenceData::observed() reproduces
// libgnss's precomputed observed DD value for the mapping used below. Cheap
// (a handful of subtractions) so it is left active in all builds rather than
// only in debug; a mismatch indicates the obs-convention mapping regressed.
void checkObservedDdMatches(double gtsam_observed,
                            double libgnss_observed,
                            const char* what) {
    const double diff = std::abs(gtsam_observed - libgnss_observed);
    if (diff > 1e-6) {
        std::fprintf(stderr,
                     "[fgo_gtsam_backend] WARNING: %s observed-DD mismatch: "
                     "gtsam=%.9f libgnss=%.9f diff=%.9e\n",
                     what, gtsam_observed, libgnss_observed, diff);
        assert(false && "GTSAM/libgnss DD observation convention mismatch");
    }
}

}  // namespace

// ===========================================================================
// Milestone 2c: IncrementalFixedLagSmoother path (docs/gtsam_backend_design.md)
// ===========================================================================
//
// Streams the IMU-coupled DD-RTK graph through gtsam::IncrementalFixedLagSmoother
// instead of a single batch LM solve, so memory and per-epoch marginals stay
// bounded at full-dataset scale (the batch gtsam::Marginals over ~25k vars hit
// bad_alloc). Only used when config.use_fixed_lag_smoother && the IMU path is
// active; otherwise optimizeProblemWithGtsam runs its batch LM path unchanged.
//
// Key-timestamp scheme (nav = ENU): every key is timestamped with the epoch
// time in seconds relative to epoch 0. Pose/Vel/Bias X(i)/V(i)/B(i) are stamped
// with epoch i, so they marginalize out fixed_lag_smoother_lag_s after epoch i.
// Ambiguity nodes and the shared dummy node are RE-STAMPED to the current epoch
// every time they are observed, so an ambiguity persists in the window while
// its arc is active and marginalizes lag-seconds after its last observation
// (arc end) -- i.e. ambiguities spanning the lag are simply kept in-window by
// re-stamping (no fix-and-hold; per-epoch LAMBDA is a read-out off the windowed
// marginals, ambiguities stay float in the graph, matching Phase-1/2b).
static FGOProcessor::FGOResult optimizeProblemFixedLag(
    const FGOProcessor::FGOProblem& problem,
    const FGOProcessor::FGOConfig& config,
    FGOProcessor::FGOResult result) {
    const auto start_time = std::chrono::high_resolution_clock::now();

    const std::size_t num_epochs = problem.epochs.size();
    const Point3 lever_arm_body(config.pose3_lever_arm_body_m.x(),
                                config.pose3_lever_arm_body_m.y(),
                                config.pose3_lever_arm_body_m.z());
    const Rot3 R_ecef_nav = ecefFromEnuRotation(problem.imu.nav_origin_lat_rad,
                                                problem.imu.nav_origin_lon_rad);
    const Pose3 ecef_T_nav(R_ecef_nav, Point3(problem.imu.nav_origin_ecef));
    const gtsam::gnss::LeverArm gnss_lever_arm(lever_arm_body, ecef_T_nav);
    const Rot3 init_attitude_nav(problem.imu.init_attitude_body_to_nav);
    const gtsam::imuBias::ConstantBias init_bias(problem.imu.init_accel_bias,
                                                 problem.imu.init_gyro_bias);

    auto antennaOf = [&](const Pose3& pose) -> Point3 {
        return gnss_lever_arm.antennaPosition(pose);
    };
    auto navAntenna = [&](const Vector3d& ecef) -> Vector3d {
        return ecef2enu(ecef - problem.imu.nav_origin_ecef, problem.imu.nav_origin_lat_rad,
                        problem.imu.nav_origin_lon_rad);
    };
    auto stampOf = [&](std::size_t epoch) -> double {
        return problem.epochs[epoch].time - problem.epochs[0].time;
    };

    // Group DD factors by epoch for O(1) streaming access.
    std::vector<std::vector<const FGOProcessor::DoubleDifferencePseudorangeFactor*>> pr_by_epoch(
        num_epochs);
    std::vector<std::vector<const FGOProcessor::DoubleDifferenceCarrierFactor*>> cp_by_epoch(
        num_epochs);
    for (const auto& f : problem.double_difference_pseudorange_factors) {
        if (f.epoch_index < num_epochs) pr_by_epoch[f.epoch_index].push_back(&f);
    }
    for (const auto& f : problem.double_difference_carrier_factors) {
        if (f.epoch_index < num_epochs && f.ambiguity_index < problem.ambiguity_states.size()) {
            cp_by_epoch[f.epoch_index].push_back(&f);
        }
    }

    // --- DDPR-LS anchor solve (FGOConfig::use_ddpr_anchor; port of the
    // reference's ddpr_only_position / _ddpr_build_specs / _ddpr_solve_with_fde).
    // Reuses the SAME DoubleDifferencePseudorangeFactorArm construction the
    // main loop uses (see the "DD pseudorange factors at epoch i" block
    // below) against a dedicated, throwaway Pose3 key ('y', 0) so the mini
    // solve never touches the smoother's key space. Plain (non-robust) noise
    // throughout, matching the reference's huber_pr=0 default -- the same
    // noise model is reused for both the LM solve and the FDE residual
    // evaluation (the reference rebuilds a second "non-robust" graph for
    // eval only because ITS default solve pass may be robust; ours already
    // isn't, so one graph suffices for both).
    auto solveDdprAnchor = [&](std::size_t epoch_idx, const Pose3& pose_init) -> DdprAnchorResult {
        DdprAnchorResult out;
        std::vector<const FGOProcessor::DoubleDifferencePseudorangeFactor*> active(
            pr_by_epoch[epoch_idx].begin(), pr_by_epoch[epoch_idx].end());
        if (active.size() < static_cast<std::size_t>(std::max(1, config.ddpr_anchor_min_factors))) {
            return out;
        }
        const gtsam::Key anchor_key = Symbol('y', 0);
        gtsam::Vector6 prior_sigmas;
        prior_sigmas << 0.05, 0.05, 0.1, 50.0, 50.0, 50.0;
        const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);

        gtsam::Values est;
        double res_rms = std::numeric_limits<double>::infinity();
        bool have_est = false;

        for (int fde_iter = 0; fde_iter < 3; ++fde_iter) {
            gtsam::NonlinearFactorGraph g;
            gtsam::Values v;
            v.insert(anchor_key, pose_init);
            g.addPrior(anchor_key, pose_init, prior_noise);
            for (const auto* fp : active) {
                const gtsam::gnss::DoubleDifferenceData dd{
                    fp->rover_satellite_model.corrected_pseudorange_m,
                    fp->base_satellite_model.corrected_pseudorange_m,
                    fp->rover_reference_model.corrected_pseudorange_m,
                    fp->base_reference_model.corrected_pseudorange_m,
                    Point3(fp->rover_satellite_position_ecef),
                    Point3(fp->rover_reference_position_ecef),
                    Point3(fp->base_satellite_position_ecef),
                    Point3(fp->base_reference_position_ecef),
                    Point3(fp->base_position_ecef)};
                g.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                    anchor_key, dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget, dd.satRefRov,
                    dd.satTargetRov, dd.satRefBase, dd.satTargetBase, dd.basePos, lever_arm_body,
                    ecef_T_nav, makeNoise(fp->sigma_m, /*robust=*/false, 0.0));
            }
            gtsam::Values cur_est;
            try {
                gtsam::LevenbergMarquardtParams lm_params;
                lm_params.setMaxIterations(10);
                cur_est = gtsam::LevenbergMarquardtOptimizer(g, v, lm_params).optimize();
            } catch (const std::exception&) {
                return out;  // solve failed -> not ok
            }
            est = cur_est;
            have_est = true;

            std::vector<const FGOProcessor::DoubleDifferencePseudorangeFactor*> kept;
            kept.reserve(active.size());
            double sq_sum = 0.0;
            std::size_t dropped = 0;
            for (std::size_t k = 0; k < active.size(); ++k) {
                const double err = g.at(1 + k)->error(est);  // factor 0 is the pose prior
                const double res_m = std::sqrt(std::max(0.0, err) * 2.0) * active[k]->sigma_m;
                if (res_m > config.ddpr_anchor_fde_threshold_m &&
                    (active.size() - dropped) > static_cast<std::size_t>(std::max(1, config.ddpr_anchor_min_factors))) {
                    ++dropped;
                    continue;
                }
                kept.push_back(active[k]);
                sq_sum += res_m * res_m;
            }
            res_rms = kept.empty() ? 0.0 : std::sqrt(sq_sum / static_cast<double>(kept.size()));
            active = kept;
            if (dropped == 0) break;
        }
        if (!have_est || active.size() < static_cast<std::size_t>(std::max(1, config.ddpr_anchor_min_factors))) {
            return out;
        }
        out.ok = true;
        out.pose = est.at<Pose3>(anchor_key);
        out.n_active = static_cast<int>(active.size());
        out.res_rms = res_rms;
        return out;
    };

    // IMU preintegration params (ENU / Z-up; gravity -Z).
    auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(problem.imu.noise.gravity_mps2);
    const auto sq = [](double s) { return s * s; };
    imu_params->setAccelerometerCovariance(gtsam::I_3x3 * sq(problem.imu.noise.accel_noise_sigma));
    imu_params->setGyroscopeCovariance(gtsam::I_3x3 * sq(problem.imu.noise.gyro_noise_sigma));
    // Reference parity (FGOConfig::imu_integration_covariance, fgo.hpp):
    // imu_integration_covariance IS the covariance directly (reference
    // imu_integ_cov value semantics -- no squaring), replacing the
    // sq(integration_sigma) computation for this fixed-lag path. Default
    // 1e-6 reproduces the harness's hardcoded integration_sigma=1e-3 squared,
    // so this is bit-identical unless the new knob is overridden.
    imu_params->setIntegrationCovariance(gtsam::I_3x3 * config.imu_integration_covariance);
    imu_params->setBiasAccCovariance(gtsam::I_3x3 * sq(problem.imu.noise.accel_bias_rw_sigma));
    imu_params->setBiasOmegaCovariance(gtsam::I_3x3 * sq(problem.imu.noise.gyro_bias_rw_sigma));
    const auto& imu_samples = problem.imu.samples_body_flu;

    // Smoother. findUnusedFactorSlots is REQUIRED for fixed-lag marginalization;
    // relinearize aggressively (skip=1, low threshold) since the IMU factors are
    // strongly nonlinear and we take one update per epoch.
    gtsam::ISAM2Params isam_params;
    isam_params.findUnusedFactorSlots = true;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    isam_params.factorization = gtsam::ISAM2Params::CHOLESKY;
    gtsam::IncrementalFixedLagSmoother smoother(config.fixed_lag_smoother_lag_s, isam_params);

    // Per-epoch outputs.
    std::vector<Point3> epoch_float_position(num_epochs, Point3(0, 0, 0));
    std::vector<Point3> epoch_fixed_position(num_epochs, Point3(0, 0, 0));
    std::vector<bool> epoch_has_fixed(num_epochs, false);
    std::vector<bool> epoch_fixed(num_epochs, false);
    std::vector<int> epoch_fixed_count(num_epochs, 0);
    std::vector<double> epoch_ratio(num_epochs, 0.0);
    std::vector<Vector3d> epoch_rpy_deg(num_epochs, Vector3d::Zero());
    std::vector<Vector3d> epoch_vel_nav(num_epochs, Vector3d::Zero());
    std::vector<bool> epoch_solved(num_epochs, false);
    std::map<std::size_t, double> amb_float_cycles;
    std::map<std::size_t, int> amb_fixed_cycles;
    std::map<std::size_t, double> amb_fixed_residual;

    // 2e fix-and-hold: arcs whose DD ambiguity has been validated-fixed and
    // pinned in the graph at its integer (reset automatically when the arc ends
    // and the builder issues a fresh ambiguity index).
    std::set<std::size_t> pinned_ambiguities;
    std::size_t held_epoch_count = 0;

    std::set<std::size_t> ambiguity_created;
    bool dummy_created = false;
    constexpr double kDummyPinSigmaCycles = 1e-3;

    // Previous refined nav state (seed source for the next epoch's prediction).
    gtsam::NavState prev_nav;
    gtsam::imuBias::ConstantBias prev_bias = init_bias;
    std::size_t sample_cursor = 0;
    std::size_t total_fixed_ambiguities = 0;
    std::size_t marginals_failures = 0;
    std::size_t lambda_attempts = 0;
    double best_ratio = 0.0;
    const int min_candidates = std::max(6, std::max(1, config.min_fixed_ambiguities + 1));

    const bool robust = config.use_robust_loss;

    // Quality gates: satellites whose post-fit DDPR residual exceeded
    // gate_per_sat_res_max_m LAST epoch (reference: prefit
    // apply_per_sat_residual_gate keeps them out of the LAMBDA tree).
    std::set<SatelliteId> gate_bad_sats;

    // --- Sat-badness EWMA down-weighting state (FGOConfig::
    // use_sat_badness_downweight; port of the inuex35 reference's
    // preprocess/sat_quality.py SatQualityState). See use_sat_badness_
    // downweight's comment in fgo.hpp for the full behavioural summary and
    // deviations (cppr substitution, satellite identity, el/snr wiring,
    // pair-update gating). All maps are no-ops (never populated, never
    // read) whenever the master switch is off.
    //
    // last_ddpr_per_sat: snapshot of this epoch's (pre-FDE) per_sat_res,
    // taken once per epoch below and read by the NEXT epoch's DD factor
    // build for the res_s term -- reference tc._last_main_ddpr_per_sat /
    // _mres_signals.per_sat, which is likewise set from main_ddpr_residuals'
    // per-sat map and consumed one epoch later at factor-build time.
    std::map<SatelliteId, double> sb_last_ddpr_per_sat;
    std::map<SatelliteId, double> sb_obsq_ewma;
    std::map<SatelliteId, int> sb_obsq_bad_streak;
    std::map<SatelliteId, double> sb_recent_worst;
    std::map<SatelliteId, double> sb_recent_cppr;
    std::map<SatelliteId, double> sb_recent_ref_bad;
    std::map<std::tuple<SatelliteId, SatelliteId, SignalType>, double> sb_recent_pair_bad;
    std::map<SatelliteId, double> sb_latest_el_deg;
    std::map<SatelliteId, double> sb_latest_snr_dbhz;
    // Deviation (see fgo.hpp): substitute for the reference's unported
    // CP-vs-PR innovation-consistency reject counter (rejc_cp_pr). Persistent
    // per-(satellite, signal) count of THIS backend's own FDE carrier
    // rejections.
    //
    // CLAMPED-variant deviation (fgo.hpp sat_badness_cppr_decay): unlike the
    // faithful port (which never reset this and let it grow forever), this
    // is now a decayed float -- cppr[s,sig] = decay*prev + this_epoch_rejects
    // -- applied once per epoch by runFde() below (decay=1.0 reproduces the
    // old ever-growing-counter behaviour exactly). Structurally empty/0
    // whenever use_fde is off.
    std::map<std::pair<SatelliteId, SignalType>, double> sb_fde_cp_reject_count;

    // CLAMPED-variant helper (fgo.hpp sat_badness_residual_clamp_m): caps a
    // per-satellite post-fit DDPR residual before it feeds any badness term.
    // Applied only at badness's OWN reads/snapshots of per_sat_res (see the
    // per-epoch state-update block below) -- never to the shared per_sat_res
    // map itself, which use_epoch_quality_gates/use_cp_hold_recovery also
    // consume raw and which must stay numerically untouched by this knob.
    // 0 = no clamp = faithful (returns r unchanged).
    auto sbClampResidual = [&](double r) -> double {
        return config.sat_badness_residual_clamp_m > 0.0
                   ? std::min(r, config.sat_badness_residual_clamp_m)
                   : r;
    };

    // Continuous per-(reference, target, signal) badness score (0 when the
    // master switch is off). `ref_sat` non-null adds the directional-pair
    // term (only when sat_badness_alpha_recent_pair > 0 -- see fgo.hpp
    // deviation 5). Mirrors SatQualityState.sat_badness() term-for-term, then
    // (CLAMPED-variant deviation, fgo.hpp sat_badness_score_cap) caps the
    // total before returning it -- 0 = no cap = faithful.
    auto satBadness = [&](SatelliteId sat_id, SignalType freq,
                          const SatelliteId* ref_sat) -> double {
        if (!config.use_sat_badness_downweight) return 0.0;
        const double ddpr_thr = std::max(1e-6, config.sat_badness_ddpr_threshold_m);
        const int cppr_thr = std::max(1, config.sat_badness_cppr_threshold);
        const double alpha_ddpr = std::max(0.0, config.sat_badness_alpha_ddpr);
        const double alpha_cppr = std::max(0.0, config.sat_badness_alpha_cppr);
        const double alpha_recent_cppr = std::max(0.0, config.sat_badness_alpha_recent_cppr);
        const double alpha_recent_worst = std::max(0.0, config.sat_badness_alpha_recent_worst);
        const double alpha_recent_ref = std::max(0.0, config.sat_badness_alpha_recent_ref);
        const double alpha_recent_pair = std::max(0.0, config.sat_badness_alpha_recent_pair);
        const double alpha_obsq_ewma = std::max(0.0, config.sat_badness_alpha_obsq_ewma);
        const double alpha_obsq_streak = std::max(0.0, config.sat_badness_alpha_obsq_streak);
        const double alpha_el = std::max(0.0, config.sat_badness_alpha_el);
        const double alpha_snr = std::max(0.0, config.sat_badness_alpha_snr);
        const double obsq_thr = std::max(1e-6, config.sat_badness_obsq_res_threshold_m);
        const int obsq_streak_cap = std::max(1, config.sat_badness_obsq_bad_streak_cap);
        const double el_ref_deg = std::max(1.0, config.sat_badness_el_ref_deg);
        const double snr_ref_dbhz = config.sat_badness_snr_ref_dbhz;
        const double snr_span_db = std::max(1.0, config.sat_badness_snr_span_db);

        double score = 0.0;
        {
            const auto it = sb_last_ddpr_per_sat.find(sat_id);
            const double res_s = it != sb_last_ddpr_per_sat.end() ? it->second : 0.0;
            if (res_s > 0.0) score += alpha_ddpr * (res_s / ddpr_thr);
        }
        {
            const auto it = sb_fde_cp_reject_count.find(std::make_pair(sat_id, freq));
            const double cppr = it != sb_fde_cp_reject_count.end() ? it->second : 0.0;
            if (cppr > 0.0) score += alpha_cppr * (cppr / cppr_thr);
        }
        {
            const auto it = sb_recent_cppr.find(sat_id);
            const double q = it != sb_recent_cppr.end() ? it->second : 0.0;
            if (q > 0.0) score += alpha_recent_cppr * (q / cppr_thr);
        }
        {
            const auto it = sb_recent_worst.find(sat_id);
            const double q = it != sb_recent_worst.end() ? it->second : 0.0;
            if (q > 0.0) score += alpha_recent_worst * q;
        }
        {
            const auto it = sb_recent_ref_bad.find(sat_id);
            const double q = it != sb_recent_ref_bad.end() ? it->second : 0.0;
            if (q > 0.0) score += alpha_recent_ref * q;
        }
        if (ref_sat != nullptr && alpha_recent_pair > 0.0) {
            const auto it = sb_recent_pair_bad.find(std::make_tuple(*ref_sat, sat_id, freq));
            const double q = it != sb_recent_pair_bad.end() ? it->second : 0.0;
            if (q > 0.0) score += alpha_recent_pair * q;
        }
        {
            const auto it = sb_obsq_ewma.find(sat_id);
            const double q = it != sb_obsq_ewma.end() ? it->second : 0.0;
            if (q > 0.0) score += alpha_obsq_ewma * (q / obsq_thr);
        }
        {
            const auto it = sb_obsq_bad_streak.find(sat_id);
            const int streak = std::min(obsq_streak_cap, it != sb_obsq_bad_streak.end() ? it->second : 0);
            if (streak > 0) {
                score += alpha_obsq_streak * (static_cast<double>(streak) / obsq_streak_cap);
            }
        }
        if (alpha_el > 0.0) {
            const auto it = sb_latest_el_deg.find(sat_id);
            const double el_deg = it != sb_latest_el_deg.end() ? it->second : 90.0;
            const double penalty = std::max(0.0, std::min(1.0, (el_ref_deg - el_deg) / el_ref_deg));
            score += alpha_el * penalty;
        }
        if (alpha_snr > 0.0) {
            const auto it = sb_latest_snr_dbhz.find(sat_id);
            const double snr = it != sb_latest_snr_dbhz.end() ? it->second : snr_ref_dbhz;
            const double penalty = std::max(0.0, std::min(1.0, (snr_ref_dbhz - snr) / snr_span_db));
            score += alpha_snr * penalty;
        }
        score = std::max(0.0, score);
        if (config.sat_badness_score_cap > 0.0) {
            score = std::min(score, config.sat_badness_score_cap);
        }
        return score;
    };

    // --- CP-hold / sanity FSM state (use_cp_hold_recovery) ---
    //
    // Arc-regeneration overlay: our front-end (fgo.cpp) statically assigns one
    // ambiguity_index per continuous carrier arc for the WHOLE dataset before
    // this backend ever runs, so it cannot react to a backend-only decision
    // (bad post-fit residuals) to invalidate an arc mid-stream. `amb_generation`
    // adds that reactive layer: bumping an index's generation makes
    // ambSymbolId() mint a brand-new backend-local graph symbol the next time
    // that ambiguity_index is observed, which the existing "ambiguity_created"
    // fresh-arc bookkeeping below (seed value + prior, no held integer) then
    // treats exactly like a genuinely new arc. Generation-0 resolves to the
    // identity (symbol id == ambiguity_index), so this whole mechanism is a
    // no-op -- and the backend bit-identical to pre-port -- whenever
    // use_cp_hold_recovery is false.
    std::map<std::size_t, int> amb_generation;
    std::map<std::pair<std::size_t, int>, std::size_t> amb_symbol_id;
    std::size_t next_free_amb_symbol_id = problem.ambiguity_states.size();
    // Ambiguity indices with a symbol currently live in the graph (added under
    // their CURRENT generation); mass reset removes exactly these factors and
    // then clears the set (see reset_ambiguities_with_cp_hold below).
    std::set<std::size_t> live_ambiguity_indices;
    // Reverse of ambSymbolId(): backend graph symbol id -> the caller-facing
    // ambiguity_index it currently resolves for. Populated on every
    // ambSymbolId() call (cheap; at most one entry per (index, generation)
    // ever observed). Used by FDE's iterative mode, which discovers rejected
    // carrier factors by scanning the live graph rather than iterating
    // problem.ambiguity_states, so it needs to map a factor's ambiguityKey
    // symbol back to the ambiguity_index whose hold/generation it must
    // update.
    std::map<std::size_t, std::size_t> sym_to_ambiguity_index;
    auto ambSymbolId = [&](std::size_t idx) -> std::size_t {
        std::size_t sid = idx;
        if (config.use_cp_hold_recovery) {
            const auto git = amb_generation.find(idx);
            const int gen = (git == amb_generation.end()) ? 0 : git->second;
            if (gen != 0) {
                const auto key = std::make_pair(idx, gen);
                const auto it = amb_symbol_id.find(key);
                if (it != amb_symbol_id.end()) {
                    sid = it->second;
                } else {
                    sid = next_free_amb_symbol_id++;
                    amb_symbol_id.emplace(key, sid);
                }
            }
        }
        sym_to_ambiguity_index[sid] = idx;
        return sid;
    };

    int cp_hold_counter = 0;         ///< remaining epochs with carrier suppressed (reference _recov_cp_hold)
    int cp_hold_release_streak = 0;  ///< consecutive clean epochs while held (reference _recov_cp_release_streak)
    int ddpr_bad_count = 0;          ///< consecutive bad epochs (reference _ddpr_bad_count)
    double last_ddpr_rms = 0.0;      ///< previous epoch's post-fit DDPR RMS (reference _last_main_ddpr_res)
    // Epoch index at which last_ddpr_rms was last written (reference
    // _last_main_ddpr_epoch / _mres_signals.epoch); sentinel far in the past
    // so the very first epochs are always treated as stale (reference inits
    // to -10**9). Feeds the imu_integration_covariance_inflation staleness
    // test below.
    long long last_ddpr_rms_epoch = -1000000000LL;
    bool pim_discontinuity = false;  ///< one-shot: break the IMU chain at the NEXT epoch (reference _pim_discontinuity)
    // DDPR-anchor bootstrap re-seed countdown (use_ddpr_anchor; reference
    // tc._tc_bootstrap_ddpr_epochs). While > 0, every epoch gets a
    // translation-only anchor prior (see the bootstrap block in the main
    // loop) and CP-hold is forced to 0 via effectiveCpHoldEpochs() below
    // (reference state.effective_cp_hold_epochs: bootstrap wins).
    int ddpr_bootstrap_epochs_remaining = 0;

    // Reference state.effective_cp_hold_epochs(): the configured CP-hold
    // length, suppressed to 0 while the DDPR-anchor bootstrap countdown is
    // active. A no-op (always returns config.cp_hold_epochs) whenever
    // use_ddpr_anchor is false, so use_cp_hold_recovery's behaviour is
    // unaffected unless the anchor is also enabled.
    auto effectiveCpHoldEpochs = [&]() -> int {
        if (config.use_ddpr_anchor && ddpr_bootstrap_epochs_remaining > 0) return 0;
        return config.cp_hold_epochs;
    };

    // Mass reset shared by the persist path and the catastrophic fast path
    // (reference _apply_sanity_reset / reset_ambiguities_with_cp_hold): collect
    // every live ambiguity's CURRENT-generation factor indices out of the live
    // ISAM2 graph, remove them via a smoother update, bump every generation
    // (forcing fresh arcs), clear the pinned/live bookkeeping, and (re)engage
    // CP-hold at full strength. Returns the number of factors removed.
    auto resetAmbiguitiesWithCpHold = [&]() -> std::size_t {
        gtsam::FactorIndices remove_indices;
        if (!live_ambiguity_indices.empty()) {
            std::set<gtsam::Key> live_keys;
            for (std::size_t idx : live_ambiguity_indices) {
                live_keys.insert(ambiguityKey(ambSymbolId(idx)));
            }
            const auto& factors = smoother.getISAM2().getFactorsUnsafe();
            for (std::size_t fi = 0; fi < factors.size(); ++fi) {
                const auto& f = factors[fi];
                if (!f) continue;
                for (gtsam::Key k : f->keys()) {
                    if (live_keys.count(k)) {
                        remove_indices.push_back(fi);
                        break;
                    }
                }
            }
        }
        for (std::size_t idx : live_ambiguity_indices) {
            const std::size_t old_sid = ambSymbolId(idx);  // resolve BEFORE bumping
            ++amb_generation[idx];
            ++result.diagnostics.ambiguity_generation_bumps;
            pinned_ambiguities.erase(old_sid);
        }
        live_ambiguity_indices.clear();
        if (!remove_indices.empty()) {
            try {
                smoother.update(gtsam::NonlinearFactorGraph(), gtsam::Values(),
                                gtsam::FixedLagSmoother::KeyTimestampMap(), remove_indices);
                ++result.diagnostics.smoother_updates;
            } catch (const std::exception& e) {
                std::fprintf(stderr,
                             "[fgo_gtsam_backend] cp-hold factor removal threw: %s\n", e.what());
            }
        }
        // Our adaptation of the reference's bootstrap arming (see
        // FGOConfig::use_ddpr_anchor's deviation note): opt-in re-seed after
        // a mass/fast ambiguity reset, not just after a full warm reset.
        // Armed BEFORE the hold length is computed below, so that -- per the
        // reference's state.effective_cp_hold_epochs (bootstrap and global
        // CP-hold are mutually exclusive; bootstrap wins) -- the reset that
        // arms the bootstrap engages NO carrier hold: the bootstrap needs
        // the fresh carrier arcs + anchor translation priors flowing
        // immediately to pull the float back.
        if (config.use_ddpr_anchor && config.cp_hold_bootstrap_after_mass_reset) {
            ddpr_bootstrap_epochs_remaining = config.ddpr_anchor_bootstrap_epochs;
        }
        cp_hold_counter = effectiveCpHoldEpochs();
        cp_hold_release_streak = 0;
        ddpr_bad_count = 0;
        if (config.cp_hold_break_imu_chain) pim_discontinuity = true;
        ++result.diagnostics.cp_hold_triggers;
        return remove_indices.size();
    };

    // --- FDE (GICI-style Fault Detection and Exclusion; FGOConfig::use_fde).
    // See use_fde's comment in fgo.hpp for the full design rationale
    // (ordering vs. the sanity FSM / LAMBDA, residual reconstruction
    // arithmetic, single-pass vs. iterative). Called from the per-epoch
    // loop AFTER the shared post-fit DDPR diagnostics pass (so the sanity
    // FSM below still sees PRE-FDE residuals) but BEFORE per-epoch LAMBDA.
    //
    // `local_indices_for_epoch` / `new_indices_for_epoch` identify THIS
    // epoch's own DD PR/CP factors precisely (local index within the
    // graph just passed to smoother.update(), and that update's
    // ISAM2Result::newFactorsIndices to resolve them to live graph
    // indices) -- used only in single-pass mode. Iterative mode ignores
    // them and scans the whole live graph by factor type instead.
    //
    // On any rejected CP factor, the caller-visible ambiguity_index is
    // written into `*rejected_ambiguity_indices` so the caller can drop it
    // from this epoch's still-pending LAMBDA candidate list (the factor
    // backing that candidate no longer exists in the graph).
    //
    // Returns the number of factors actually removed (0 = no-op /
    // safeguarded / evaluation or removal failed).
    auto runFde = [&](std::size_t epoch_idx,
                      const std::vector<std::size_t>& local_indices_for_epoch,
                      const gtsam::FactorIndices& new_indices_for_epoch,
                      std::set<std::size_t>* rejected_ambiguity_indices) -> std::size_t {
        if (!config.use_fde) return 0;
        const int max_iter = std::max(1, config.fde_max_iterations);
        const bool iterative = max_iter > 1;
        std::size_t total_rejected = 0;

        // CLAMPED-variant deviation (fgo.hpp sat_badness_cppr_decay): this
        // call's rejects, by (satellite, signal), accumulated across every
        // FDE round below and folded into sb_fde_cp_reject_count's decay
        // exactly once (at applyCpprDecay()'s call sites, one per return
        // path) before this lambda returns.
        std::map<std::pair<SatelliteId, SignalType>, int> sb_cppr_delta_this_call;
        auto applyCpprDecay = [&]() {
            const double decay = std::min(1.0, std::max(0.0, config.sat_badness_cppr_decay));
            std::set<std::pair<SatelliteId, SignalType>> keys;
            for (const auto& kv : sb_fde_cp_reject_count) keys.insert(kv.first);
            for (const auto& kv : sb_cppr_delta_this_call) keys.insert(kv.first);
            for (const auto& k : keys) {
                const double prev = sb_fde_cp_reject_count.count(k) ? sb_fde_cp_reject_count[k] : 0.0;
                const double add = sb_cppr_delta_this_call.count(k)
                                        ? static_cast<double>(sb_cppr_delta_this_call.at(k))
                                        : 0.0;
                sb_fde_cp_reject_count[k] = decay * prev + add;
            }
        };

        struct FdeEntry {
            std::size_t graph_idx;
            bool is_carrier;
            double res_m;
        };

        for (int fde_iter = 0; fde_iter < max_iter; ++fde_iter) {
            const auto& factors = smoother.getISAM2().getFactorsUnsafe();
            std::vector<std::size_t> candidate_graph_indices;
            if (iterative) {
                // Whole live graph, re-scanned fresh every round so a prior
                // round's removal never leaves a stale index dangling.
                candidate_graph_indices.reserve(factors.size());
                for (std::size_t fi = 0; fi < factors.size(); ++fi) {
                    if (factors[fi]) candidate_graph_indices.push_back(fi);
                }
            } else {
                candidate_graph_indices.reserve(local_indices_for_epoch.size());
                for (std::size_t li : local_indices_for_epoch) {
                    if (li < new_indices_for_epoch.size()) {
                        candidate_graph_indices.push_back(new_indices_for_epoch[li]);
                    }
                }
            }
            if (candidate_graph_indices.empty()) break;

            gtsam::Values estimate;
            try {
                estimate = smoother.calculateEstimate();
            } catch (const std::exception&) {
                break;  // cannot evaluate residuals -- abandon FDE, keep current estimate
            }

            // Residual in meters: evaluateError() directly, NOT factor->error()
            // (see the design-note above runFde -- error() runs the noise
            // model's loss(), which for a Robust/Huber-wrapped factor
            // returns the DOWN-WEIGHTED loss, not the raw chi-squared
            // distance, silently weakening outlier detection exactly for
            // the large residuals FDE exists to catch). evaluateError()
            // bypasses the noise model entirely and returns the 1-D raw
            // (unwhitened) residual directly in meters, so no sigma
            // reconstruction is needed at all -- correct whether or not
            // config.use_robust_loss is set.
            std::vector<FdeEntry> pr_entries, cp_entries;
            for (std::size_t gi : candidate_graph_indices) {
                if (gi >= factors.size() || !factors[gi]) continue;
                const auto& f = factors[gi];
                const auto* pr_f =
                    dynamic_cast<const gtsam::DoubleDifferencePseudorangeFactorArm*>(f.get());
                const auto* cp_f =
                    pr_f ? nullptr
                         : dynamic_cast<const gtsam::DoubleDifferenceCarrierPhaseFactorArm*>(
                               f.get());
                if (!pr_f && !cp_f) continue;
                try {
                    if (pr_f) {
                        const Pose3 pose = estimate.at<Pose3>(f->keys()[0]);
                        const double res_m = std::abs(pr_f->evaluateError(pose)(0));
                        pr_entries.push_back({gi, false, res_m});
                    } else {
                        const Pose3 pose = estimate.at<Pose3>(f->keys()[0]);
                        const double amb_ref = estimate.at<double>(f->keys()[1]);
                        const double amb_target = estimate.at<double>(f->keys()[2]);
                        const double res_m =
                            std::abs(cp_f->evaluateError(pose, amb_ref, amb_target)(0));
                        cp_entries.push_back({gi, true, res_m});
                    }
                } catch (const std::exception&) {
                    continue;
                }
            }

            double pr_median = 0.0, cp_median = 0.0;
            if (config.fde_median_subtraction) {
                auto medianOf = [](const std::vector<FdeEntry>& v) {
                    if (v.empty()) return 0.0;
                    std::vector<double> r;
                    r.reserve(v.size());
                    for (const auto& e : v) r.push_back(e.res_m);
                    std::sort(r.begin(), r.end());
                    return r[r.size() / 2];
                };
                pr_median = medianOf(pr_entries);
                cp_median = medianOf(cp_entries);
            }

            std::vector<FdeEntry> reject_entries;
            if (iterative) {
                // Single worst |res - median| exceeder across PR and CP
                // combined (reference _fde_pick_rejects_iterative); ties
                // favor whichever group is scanned first (PR), matching the
                // reference's strict '>' override in the CP loop.
                double best_d = 0.0;
                int best_pr = -1, best_cp = -1;
                for (std::size_t k = 0; k < pr_entries.size(); ++k) {
                    const double d = std::abs(pr_entries[k].res_m - pr_median);
                    if (d > config.fde_pseudorange_threshold_m && d > best_d) {
                        best_d = d;
                        best_pr = static_cast<int>(k);
                        best_cp = -1;
                    }
                }
                for (std::size_t k = 0; k < cp_entries.size(); ++k) {
                    const double d = std::abs(cp_entries[k].res_m - cp_median);
                    if (d > config.fde_carrier_threshold_m && d > best_d) {
                        best_d = d;
                        best_cp = static_cast<int>(k);
                        best_pr = -1;
                    }
                }
                if (best_pr < 0 && best_cp < 0) break;  // no outlier this round -- done
                reject_entries.push_back(best_pr >= 0 ? pr_entries[static_cast<std::size_t>(best_pr)]
                                                       : cp_entries[static_cast<std::size_t>(best_cp)]);
            } else {
                for (const auto& e : pr_entries) {
                    if (std::abs(e.res_m - pr_median) > config.fde_pseudorange_threshold_m) {
                        reject_entries.push_back(e);
                    }
                }
                for (const auto& e : cp_entries) {
                    if (std::abs(e.res_m - cp_median) > config.fde_carrier_threshold_m) {
                        reject_entries.push_back(e);
                    }
                }
                if (reject_entries.empty()) break;  // no-op: nothing exceeded threshold
                // Safeguard (single-pass only, matching the reference): a
                // runaway reject fraction likely means the FLOAT itself is
                // wrong (not the measurements), so excluding that many
                // factors would just poison the graph further -- abandon
                // FDE for the epoch and hand off to CP-hold instead.
                const std::size_t nv = pr_entries.size() + cp_entries.size();
                const double frac_limit = config.fde_max_rejected_fraction *
                                          static_cast<double>(std::max<std::size_t>(1, nv));
                if (static_cast<double>(reject_entries.size()) > frac_limit) {
                    ++result.diagnostics.fde_safeguard_skips;
                    // Reference trigger_cp_hold(..., skip_if_active=True):
                    // engage the hold at full strength unless already
                    // active. With use_cp_hold_recovery off there is no
                    // hold to engage -- FDE simply skips this epoch.
                    if (config.use_cp_hold_recovery && cp_hold_counter <= 0) {
                        cp_hold_counter = effectiveCpHoldEpochs();
                        cp_hold_release_streak = 0;
                        ++result.diagnostics.cp_hold_triggers;
                    }
                    applyCpprDecay();
                    return total_rejected;  // 0: FDE skipped entirely this epoch
                }
            }

            // Cycle-slip bookkeeping for rejected CP factors (reference
            // _fde_reset_rejected_amb): release the fix-and-hold pin and
            // bump the generation. Applied BEFORE the removal update below
            // is attempted, mirroring the reference's own ordering (which
            // does this even though the isam2 removal could still fail).
            for (const auto& e : reject_entries) {
                if (!e.is_carrier) continue;
                const auto& f = factors[e.graph_idx];
                if (!f || f->keys().size() < 2) continue;
                // Key 1 is ambRef = ambiguityKey(sym_idx) by construction
                // (see the DoubleDifferenceCarrierPhaseFactorArm emplace
                // site above: position, ambRef, ambTarget=dummy).
                const gtsam::Symbol sym(f->keys()[1]);
                const std::size_t sym_idx = sym.index();
                const auto it = sym_to_ambiguity_index.find(sym_idx);
                if (it == sym_to_ambiguity_index.end()) continue;
                const std::size_t amb_idx = it->second;
                pinned_ambiguities.erase(sym_idx);
                ++amb_generation[amb_idx];
                ++result.diagnostics.ambiguity_generation_bumps;
                live_ambiguity_indices.erase(amb_idx);
                if (rejected_ambiguity_indices) rejected_ambiguity_indices->insert(amb_idx);
                // Sat-badness cppr substitute (see fgo.hpp deviation 1): count
                // this FDE carrier reject against the arc's physical
                // (satellite, signal) identity. Cheap and harmless when the
                // master switch is off; only actually consumed by satBadness()
                // when config.use_sat_badness_downweight is true.
                if (amb_idx < problem.ambiguity_states.size()) {
                    const auto& amb = problem.ambiguity_states[amb_idx];
                    ++sb_cppr_delta_this_call[std::make_pair(amb.satellite, amb.signal)];
                }
            }

            gtsam::FactorIndices remove_indices;
            remove_indices.reserve(reject_entries.size());
            std::size_t pr_rejected = 0, cp_rejected = 0;
            for (const auto& e : reject_entries) {
                remove_indices.push_back(e.graph_idx);
                if (e.is_carrier) {
                    ++cp_rejected;
                } else {
                    ++pr_rejected;
                }
            }
            try {
                smoother.update(gtsam::NonlinearFactorGraph(), gtsam::Values(),
                                gtsam::FixedLagSmoother::KeyTimestampMap(), remove_indices);
                ++result.diagnostics.smoother_updates;
            } catch (const std::exception& e) {
                std::fprintf(stderr,
                             "[fgo_gtsam_backend] FDE factor removal epoch %zu threw: %s\n",
                             epoch_idx, e.what());
                break;  // abandon FDE; keep the estimate as it stood before this attempt
            }
            result.diagnostics.fde_pseudorange_rejections += pr_rejected;
            result.diagnostics.fde_carrier_rejections += cp_rejected;
            total_rejected += reject_entries.size();

            if (!iterative) {
                applyCpprDecay();
                return total_rejected;  // single removal batch, done
            }
        }
        applyCpprDecay();
        return total_rejected;
    };

    // --- Exception recovery (use_solve_exception_recovery): full warm reset
    // (reference recovery.warm_reset_phase2), the fallback for REPEATED
    // smoother.update() failures. Destroys and recreates the smoother from
    // scratch and re-anchors epoch `epoch_idx` at (seed_pose, zero velocity,
    // seed_bias) with loose-but-finite priors, matching the reference's
    // rot/pos/vel/bias sigmas. Unlike the Python reference (which restarts
    // its own relative epoch counter at 0), this keeps the SAME epoch index
    // i / key space -- there is nothing special about our key numbering to
    // reset, so re-anchoring in place is the direct equivalent. Also bumps
    // every live ambiguity's generation (the fresh smoother has none of
    // their factors any more) and engages CP-hold. Throws on failure (the
    // caller decides whether to give up on the epoch).
    auto performFullWarmReset = [&](std::size_t epoch_idx, const Pose3& seed_pose,
                                    const gtsam::imuBias::ConstantBias& seed_bias) {
        smoother = gtsam::IncrementalFixedLagSmoother(config.fixed_lag_smoother_lag_s, isam_params);
        dummy_created = false;
        ambiguity_created.clear();
        pinned_ambiguities.clear();
        for (std::size_t idx : live_ambiguity_indices) {
            ++amb_generation[idx];
            ++result.diagnostics.ambiguity_generation_bumps;
        }
        live_ambiguity_indices.clear();

        constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
        const double rot_sigma = config.cp_hold_warm_reset_rotation_sigma_deg * kDegToRad;
        gtsam::Vector6 pose_sigmas;
        pose_sigmas << rot_sigma, rot_sigma, rot_sigma, 2.0, 2.0, 3.0;
        const gtsam::Vector3 zero_vel = gtsam::Vector3::Zero();

        gtsam::NonlinearFactorGraph reset_factors;
        gtsam::Values reset_values;
        gtsam::FixedLagSmoother::KeyTimestampMap reset_ts;
        reset_values.insert(positionKey(epoch_idx), seed_pose);
        reset_values.insert(velocityKey(epoch_idx), zero_vel);
        reset_values.insert(biasKey(epoch_idx), seed_bias);
        reset_factors.addPrior(positionKey(epoch_idx), seed_pose,
                               gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
        reset_factors.addPrior<gtsam::Vector3>(
            velocityKey(epoch_idx), zero_vel, gtsam::noiseModel::Isotropic::Sigma(3, 3.0));
        reset_factors.addPrior(biasKey(epoch_idx), seed_bias,
                               gtsam::noiseModel::Isotropic::Sigma(6, 0.01));
        const double te_reset = stampOf(epoch_idx);
        reset_ts[positionKey(epoch_idx)] = te_reset;
        reset_ts[velocityKey(epoch_idx)] = te_reset;
        reset_ts[biasKey(epoch_idx)] = te_reset;
        smoother.update(reset_factors, reset_values, reset_ts);
        ++result.diagnostics.smoother_updates;

        prev_nav = gtsam::NavState(seed_pose, zero_vel);
        prev_bias = seed_bias;
        // Bootstrap re-seed: reference arms this once at Phase-2 init; this
        // port arms it after EVERY full warm reset instead (see
        // FGOConfig::use_ddpr_anchor's deviation note) -- unconditional
        // (unlike the mass/fast-reset arming in resetAmbiguitiesWithCpHold,
        // which is opt-in via cp_hold_bootstrap_after_mass_reset) because a
        // full warm reset always throws away the smoother's linearization
        // entirely. Armed BEFORE the hold computation below so the
        // reference's bootstrap-suppresses-CP-hold rule
        // (state.effective_cp_hold_epochs) applies to this very reset.
        if (config.use_ddpr_anchor) {
            ddpr_bootstrap_epochs_remaining = config.ddpr_anchor_bootstrap_epochs;
        }
        if (config.use_cp_hold_recovery) {
            cp_hold_counter = effectiveCpHoldEpochs();
            cp_hold_release_streak = 0;
        }
    };

    for (std::size_t i = 0; i < num_epochs; ++i) {
        gtsam::NonlinearFactorGraph new_factors;
        gtsam::Values new_values;
        gtsam::FixedLagSmoother::KeyTimestampMap ts;
        const double te = stampOf(i);

        // --- Pose / velocity / bias seed for epoch i ---
        Pose3 pose_seed;
        gtsam::Vector3 vel_seed;
        std::size_t win_begin = sample_cursor, win_end = sample_cursor;  // IMU window for ZUPT/NHC
        if (i == 0) {
            const Vector3d antenna_nav = navAntenna(problem.epochs[0].position_ecef);
            pose_seed = Pose3(init_attitude_nav,
                              Point3(antenna_nav) - init_attitude_nav * lever_arm_body);
            vel_seed = gtsam::Vector3(problem.imu.init_velocity_nav);
            // ZUPT/NHC window for epoch 0: the first interval [epoch0, epoch1).
            const GNSSTime a = problem.epochs[0].time;
            const GNSSTime b = problem.epochs[1].time;
            std::size_t k = 0;
            while (k < imu_samples.size() && imu_samples[k].time < a) ++k;
            win_begin = k;
            while (k < imu_samples.size() && imu_samples[k].time < b) ++k;
            win_end = k;
        } else {
            // Preintegrate [epoch i-1, epoch i) and IMU-predict the seed.
            const GNSSTime t0 = problem.epochs[i - 1].time;
            const GNSSTime t1 = problem.epochs[i].time;
            while (sample_cursor < imu_samples.size() && imu_samples[sample_cursor].time < t0) {
                ++sample_cursor;
            }
            win_begin = sample_cursor;
            // Reference parity: residual-driven per-epoch integration-
            // covariance inflation (FGOConfig::
            // use_imu_integration_covariance_inflation; port of
            // imu_preintegration.py's _apply_mres_integ_cov_override). Mutate
            // the SHARED imu_params in place immediately before this epoch's
            // PIM is constructed -- integrateMeasurement() below bakes the
            // covariance in at integration time, so this affects only the
            // PIM about to be built, never any already-linearized factor.
            if (config.use_imu_integration_covariance_inflation) {
                const int stale_max = config.imu_integration_covariance_stale_epochs;
                const bool is_stale = stale_max > 0 &&
                    (static_cast<long long>(i) - last_ddpr_rms_epoch) > stale_max;
                const double default_cov = config.imu_integration_covariance;
                double integ_eff = default_cov;
                if (!is_stale) {
                    const double dt_epoch = std::max(t1 - t0, 1e-3);
                    integ_eff = std::max(default_cov, sq(last_ddpr_rms) / dt_epoch);
                    const double cap = config.imu_integration_covariance_max;
                    if (cap > 0.0) integ_eff = std::min(integ_eff, cap);
                }
                imu_params->setIntegrationCovariance(gtsam::I_3x3 * integ_eff);
            }
            gtsam::PreintegratedCombinedMeasurements pim(imu_params, prev_bias);
            std::size_t j = sample_cursor;
            GNSSTime prev_time = t0;
            std::size_t integrated = 0;
            while (j < imu_samples.size() && imu_samples[j].time < t1) {
                const double dt = imu_samples[j].time - prev_time;
                if (dt > 1e-9) {
                    pim.integrateMeasurement(gtsam::Vector3(imu_samples[j].accel_raw),
                                             gtsam::Vector3(imu_samples[j].gyro_raw_radps), dt);
                    ++integrated;
                }
                prev_time = imu_samples[j].time;
                ++j;
            }
            win_end = j;
            const double dt_tail = t1 - prev_time;
            if (integrated > 0 && dt_tail > 1e-9 && j > 0) {
                pim.integrateMeasurement(gtsam::Vector3(imu_samples[j - 1].accel_raw),
                                         gtsam::Vector3(imu_samples[j - 1].gyro_raw_radps), dt_tail);
            }
            if (integrated > 0) {
                const gtsam::NavState pred = pim.predict(prev_nav, prev_bias);
                pose_seed = pred.pose();
                vel_seed = pred.velocity();
                // CP-hold / sanity FSM: break the IMU chain the epoch after a
                // mass/fast reset (reference add_imu_chain's discontinuity
                // path). The seed is still the IMU prediction (dead-reckoned
                // from the pre-reset state, same as normal), but NO
                // CombinedImuFactor links epoch i back to epoch i-1 -- instead
                // a loose PriorPose3/PriorVector anchors epoch i at the seed so
                // a wrong pre-reset pose cannot drag the post-reset solution
                // back through the IMU factor.
                if (config.use_cp_hold_recovery && pim_discontinuity) {
                    gtsam::Vector6 break_sigmas;
                    const double trans_sig = config.cp_hold_imu_break_translation_sigma_m;
                    break_sigmas << 0.1, 0.1, 0.3, trans_sig, trans_sig, trans_sig;
                    new_factors.addPrior(positionKey(i), pose_seed,
                                         gtsam::noiseModel::Diagonal::Sigmas(break_sigmas));
                    new_factors.addPrior<gtsam::Vector3>(
                        velocityKey(i), vel_seed, gtsam::noiseModel::Isotropic::Sigma(3, 2.0));
                    new_factors.addPrior(biasKey(i), prev_bias,
                                         gtsam::noiseModel::Isotropic::Sigma(6, 0.01));
                    pim_discontinuity = false;
                } else {
                    new_factors.emplace_shared<gtsam::CombinedImuFactor>(
                        positionKey(i - 1), velocityKey(i - 1), positionKey(i), velocityKey(i),
                        biasKey(i - 1), biasKey(i), pim);
                }
            } else {
                // IMU dropout: hold pose at the DD antenna seed, loose continuity.
                const Vector3d antenna_nav = navAntenna(problem.epochs[i].position_ecef);
                pose_seed = Pose3(prev_nav.attitude(),
                                  Point3(antenna_nav) - prev_nav.attitude() * lever_arm_body);
                vel_seed = prev_nav.velocity();
                new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
                    velocityKey(i - 1), velocityKey(i), gtsam::Vector3::Zero(),
                    gtsam::noiseModel::Isotropic::Sigma(3, 10.0));
                new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
                    biasKey(i - 1), biasKey(i), gtsam::imuBias::ConstantBias(),
                    gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
            }
        }
        new_values.insert(positionKey(i), pose_seed);
        new_values.insert(velocityKey(i), vel_seed);
        new_values.insert(biasKey(i), prev_bias);
        ts[positionKey(i)] = te;
        ts[velocityKey(i)] = te;
        ts[biasKey(i)] = te;

        if (i == 0) {
            gtsam::Vector6 pose_prior_sigmas;
            pose_prior_sigmas << problem.imu.init_attitude_sigma_roll_pitch_rad,
                problem.imu.init_attitude_sigma_roll_pitch_rad,
                problem.imu.init_attitude_sigma_yaw_rad, 1e6, 1e6, 1e6;
            new_factors.addPrior(positionKey(0), pose_seed,
                                 gtsam::noiseModel::Diagonal::Sigmas(pose_prior_sigmas));
            new_factors.addPrior(velocityKey(0), gtsam::Vector3(problem.imu.init_velocity_nav),
                                 gtsam::noiseModel::Isotropic::Sigma(
                                     3, problem.imu.init_velocity_sigma_mps));
            gtsam::Vector6 bias_prior_sigmas;
            bias_prior_sigmas << problem.imu.init_accel_bias_sigma,
                problem.imu.init_accel_bias_sigma, problem.imu.init_accel_bias_sigma,
                problem.imu.init_gyro_bias_sigma, problem.imu.init_gyro_bias_sigma,
                problem.imu.init_gyro_bias_sigma;
            new_factors.addPrior(biasKey(0), init_bias,
                                 gtsam::noiseModel::Diagonal::Sigmas(bias_prior_sigmas));
        }

        // --- DD pseudorange factors at epoch i ---
        // fde_local_indices: local index (within new_factors) of every DD
        // PR/CP factor added THIS epoch, only tracked when FGOConfig::use_fde
        // is set (see runFde()'s single-pass mode, which resolves these to
        // live graph indices via this epoch's ISAM2Result::newFactorsIndices
        // right after the smoother update below).
        std::vector<std::size_t> fde_local_indices;
        for (const auto* fp : pr_by_epoch[i]) {
            const auto& factor = *fp;
            const gtsam::gnss::DoubleDifferenceData dd{
                factor.rover_satellite_model.corrected_pseudorange_m,
                factor.base_satellite_model.corrected_pseudorange_m,
                factor.rover_reference_model.corrected_pseudorange_m,
                factor.base_reference_model.corrected_pseudorange_m,
                Point3(factor.rover_satellite_position_ecef),
                Point3(factor.rover_reference_position_ecef),
                Point3(factor.base_satellite_position_ecef),
                Point3(factor.base_reference_position_ecef),
                Point3(factor.base_position_ecef)};
            // Sat-badness sigma inflation (FGOConfig::use_sat_badness_downweight;
            // port of factors.py's _add_ddpr_factor): bad_pair = max badness of
            // the pair's two satellites, computed from the PREVIOUS epoch's
            // quality state (see satBadness()/sb_last_ddpr_per_sat above), then
            // inflates the base sigma BEFORE robust-loss wrapping -- mirrors the
            // reference's base noise -> scale -> robust-wrap order exactly.
            double pr_sigma = factor.sigma_m;
            if (config.use_sat_badness_downweight) {
                const double bad_pair = std::max(
                    satBadness(factor.reference_satellite, factor.signal, nullptr),
                    satBadness(factor.satellite, factor.signal, &factor.reference_satellite));
                result.diagnostics.sat_badness_max_score_seen =
                    std::max(result.diagnostics.sat_badness_max_score_seen, bad_pair);
                if (config.sat_badness_pseudorange_sigma_scale > 0.0 && bad_pair > 0.0) {
                    pr_sigma *= (1.0 + config.sat_badness_pseudorange_sigma_scale * bad_pair);
                    ++result.diagnostics.sat_badness_downweighted_factors;
                }
            }
            const auto noise = makeNoise(pr_sigma, robust,
                                         config.pseudorange_huber_threshold_sigma);
            const std::size_t local_idx = new_factors.size();
            new_factors.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                positionKey(i), dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget, dd.satRefRov,
                dd.satTargetRov, dd.satRefBase, dd.satTargetBase, dd.basePos, lever_arm_body,
                ecef_T_nav, noise);
            if (config.use_fde) fde_local_indices.push_back(local_idx);
        }

        // --- CP-hold / sanity FSM: release hysteresis + carrier suppression
        // decision for epoch i (reference preprocess/gate.py lines ~113-128).
        // Decided BEFORE building this epoch's DD carrier factors so the
        // suppression takes effect immediately. last_ddpr_rms is the PREVIOUS
        // epoch's post-fit DDPR RMS; this epoch's own RMS is computed after
        // its solve (below) and feeds the FSM trigger for the epoch AFTER.
        bool skip_cp_now = false;
        if (config.use_cp_hold_recovery) {
            skip_cp_now = cp_hold_counter > 0;
            if (skip_cp_now) {
                --cp_hold_counter;
                ++result.diagnostics.cp_hold_epochs_held;
                const double release_thr = config.cp_hold_release_threshold_m;
                if (release_thr > 0.0) {
                    if (last_ddpr_rms > 0.0 && last_ddpr_rms <= release_thr) {
                        ++cp_hold_release_streak;
                    } else {
                        cp_hold_release_streak = 0;
                    }
                    // Release hysteresis: don't let the hold counter reach 0
                    // until residuals have proven clean for
                    // cp_hold_release_count consecutive epochs -- extend by one
                    // more epoch at a time otherwise (reference: tc._recov_cp_hold = 1).
                    if (cp_hold_counter <= 0 && cp_hold_release_streak < config.cp_hold_release_count) {
                        cp_hold_counter = 1;
                    }
                }
            }
        }

        // --- DD carrier factors + ambiguity nodes at epoch i ---
        std::vector<std::size_t> epoch_amb_indices;
        for (const auto* fp : cp_by_epoch[i]) {
            const auto& factor = *fp;
            if (config.use_cp_hold_recovery && skip_cp_now) {
                // Carrier suppressed for the duration of the hold: DD
                // pseudorange keeps flowing (added above, unaffected) but no
                // carrier factor/symbol is added here, and (mirroring
                // _carry_prev_amb) the arc's generation bumps every held
                // epoch so there is no carrier continuity across the hold
                // once it releases.
                const std::size_t idx = factor.ambiguity_index;
                const std::size_t old_sid = ambSymbolId(idx);
                ++amb_generation[idx];
                ++result.diagnostics.ambiguity_generation_bumps;
                pinned_ambiguities.erase(old_sid);
                live_ambiguity_indices.erase(idx);
                continue;
            }
            const auto& ambiguity = problem.ambiguity_states[factor.ambiguity_index];
            const std::size_t sym_idx = ambSymbolId(factor.ambiguity_index);
            if (!dummy_created) {
                new_values.insert(dummyAmbiguityKey(), 0.0);
                new_factors.addPrior(dummyAmbiguityKey(), 0.0,
                                     gtsam::noiseModel::Isotropic::Sigma(1, kDummyPinSigmaCycles));
                dummy_created = true;
            }
            if (ambiguity_created.insert(sym_idx).second) {
                const double seed_cycles = ambiguity.wavelength_m > 0.0
                                               ? ambiguity.initial_ambiguity_m / ambiguity.wavelength_m
                                               : ambiguity.initial_ambiguity_m;
                new_values.insert(ambiguityKey(sym_idx), seed_cycles);
                if (config.use_ambiguity_priors && config.ambiguity_prior_sigma_m > 0.0 &&
                    ambiguity.wavelength_m > 0.0) {
                    new_factors.addPrior(
                        ambiguityKey(sym_idx), seed_cycles,
                        gtsam::noiseModel::Isotropic::Sigma(
                            1, config.ambiguity_prior_sigma_m / ambiguity.wavelength_m));
                }
            }
            const gtsam::gnss::DoubleDifferenceData dd{
                factor.rover_satellite_model.corrected_carrier_m,
                factor.base_satellite_model.corrected_carrier_m,
                factor.rover_reference_model.corrected_carrier_m,
                factor.base_reference_model.corrected_carrier_m,
                Point3(factor.rover_satellite_position_ecef),
                Point3(factor.rover_reference_position_ecef),
                Point3(factor.base_satellite_position_ecef),
                Point3(factor.base_reference_position_ecef),
                Point3(factor.base_position_ecef)};
            // Sat-badness sigma inflation (port of factors.py's
            // _compute_cp_sigma; the ddcp_res_weight_* block further down in
            // the reference is a separate, unported mechanism -- out of
            // scope). Same bad_pair formula as the DD PR site above.
            double cp_sigma = factor.sigma_m;
            if (config.use_sat_badness_downweight) {
                const double bad_pair = std::max(
                    satBadness(factor.reference_satellite, factor.signal, nullptr),
                    satBadness(factor.satellite, factor.signal, &factor.reference_satellite));
                result.diagnostics.sat_badness_max_score_seen =
                    std::max(result.diagnostics.sat_badness_max_score_seen, bad_pair);
                if (config.sat_badness_carrier_sigma_scale > 0.0 && bad_pair > 0.0) {
                    cp_sigma *= (1.0 + config.sat_badness_carrier_sigma_scale * bad_pair);
                    ++result.diagnostics.sat_badness_downweighted_factors;
                }
            }
            const auto noise = makeNoise(cp_sigma, robust,
                                         config.carrier_phase_huber_threshold_sigma);
            const std::size_t cp_local_idx = new_factors.size();
            new_factors.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
                positionKey(i), ambiguityKey(sym_idx), dummyAmbiguityKey(),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget, dd.satRefRov, dd.satTargetRov,
                dd.satRefBase, dd.satTargetBase, dd.basePos, ambiguity.wavelength_m, lever_arm_body,
                ecef_T_nav, noise);
            if (config.use_fde) fde_local_indices.push_back(cp_local_idx);
            ts[ambiguityKey(sym_idx)] = te;  // re-stamp: keep in-window while active
            epoch_amb_indices.push_back(factor.ambiguity_index);
            if (config.use_cp_hold_recovery) live_ambiguity_indices.insert(factor.ambiguity_index);
        }
        if (dummy_created) ts[dummyAmbiguityKey()] = te;  // dummy persists (re-stamped every epoch)

        // --- DDPR-anchor bootstrap re-seed (use_ddpr_anchor; port of
        // optimize/stage.py's BOOT_DDPR_EPOCHS translation-only re-seed).
        // While the countdown (armed after a warm/mass reset -- see
        // resetAmbiguitiesWithCpHold / performFullWarmReset) is running,
        // every epoch gets an anchor-only translation prior alongside the
        // normal graph: rotation unconstrained (sigma 1e6, taken from the
        // IMU-predicted pose so it does not fight attitude), translation
        // pulled toward THIS epoch's independent DDPR-LS position (sigma
        // ddpr_anchor_bootstrap_sigma_m). This is the primary lever against
        // the CP-hold FSM's known FLOAT-degradation cost: a pull-back
        // channel to truth that does not depend on carrier/AR recovering
        // first. The countdown decrements every armed epoch regardless of
        // whether this epoch's anchor solve succeeds (reference: stage.py
        // decrements outside the try/except).
        if (config.use_ddpr_anchor && ddpr_bootstrap_epochs_remaining > 0) {
            ++result.diagnostics.ddpr_anchor_solves;
            const DdprAnchorResult boot_anchor = solveDdprAnchor(i, pose_seed);
            if (boot_anchor.ok && boot_anchor.n_active >= config.ddpr_anchor_min_factors) {
                // "successes" counts TRUSTED solves (res gate included) for
                // diagnostic consistency with the other two call sites; the
                // prior itself is added on the reference's weaker condition
                // (solve ok, n >= 4 -- stage.py has no residual gate here).
                if (boot_anchor.res_rms <= config.ddpr_anchor_max_residual_m) {
                    ++result.diagnostics.ddpr_anchor_successes;
                }
                gtsam::Vector6 boot_sigmas;
                const double bs = config.ddpr_anchor_bootstrap_sigma_m;
                boot_sigmas << 1e6, 1e6, 1e6, bs, bs, bs;
                const Pose3 boot_pose(pose_seed.rotation(), boot_anchor.pose.translation());
                new_factors.addPrior(positionKey(i), boot_pose,
                                     gtsam::noiseModel::Diagonal::Sigmas(boot_sigmas));
                ++result.diagnostics.ddpr_anchor_bootstrap_prior_epochs;
            }
            --ddpr_bootstrap_epochs_remaining;
        }

        // MF-AR step 1: the DD carrier factors for every band stay in the graph
        // (added above) and constrain the float, but the per-epoch LAMBDA
        // integer ratio test below is restricted to one band per satellite so
        // correlated multi-frequency bands don't weaken the all-or-nothing fix.
        if (config.double_difference_lambda_one_band_per_satellite &&
            epoch_amb_indices.size() > 1) {
            epoch_amb_indices = selectOneBandPerSatellite(epoch_amb_indices, problem);
        }
        // MF-AR step 2 (gated): keep Galileo arcs out of the integer search /
        // fix-and-hold entirely -- their DD factors still shape the float.
        if (config.exclude_galileo_ambiguity_fixing) {
            epoch_amb_indices.erase(
                std::remove_if(epoch_amb_indices.begin(), epoch_amb_indices.end(),
                               [&](std::size_t idx) {
                                   return problem.ambiguity_states[idx].satellite.system ==
                                          GNSSSystem::Galileo;
                               }),
                epoch_amb_indices.end());
        }

        // --- Milestone 2d: NHC + ZUPT pseudo-measurements (gated per epoch) ---
        const ImuWindowStats wstats =
            (config.use_nhc || config.use_zupt)
                ? imuWindowStats(imu_samples, win_begin, win_end, problem.imu.init_accel_bias,
                                 problem.imu.init_gyro_bias)
                : ImuWindowStats{};
        const double seed_speed = vel_seed.norm();
        const bool stationary =
            config.use_zupt && wstats.n >= config.zupt_min_samples &&
            wstats.accel_std <= config.zupt_max_accel_std &&
            wstats.gyro_std <= config.zupt_max_gyro_std &&
            wstats.gyro_median <= config.zupt_max_gyro_median &&
            // Velocity gate: reject false ZUPT during constant-velocity motion
            // (quiet accelerometer but non-zero speed).
            (config.zupt_max_speed_mps <= 0.0 || seed_speed <= config.zupt_max_speed_mps);
        if (stationary && config.zupt_sigma_mps > 0.0) {
            // ZUPT: pin Vel(i) ~ 0 (mirrors inuex35 _add_zero_velocity_prior).
            const gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
            new_factors.addPrior<gtsam::Vector3>(
                velocityKey(i), zero_velocity,
                gtsam::noiseModel::Isotropic::Sigma(3, config.zupt_sigma_mps));
            ++result.diagnostics.zupt_epochs;
        }
        if (config.use_nhc && !stationary) {
            // NHC: apply only to moving, non-turning epochs so it never fights
            // legitimate lateral motion (mirrors nhc.py's speed gate + a
            // yaw-rate gate for turns). Speed comes from the seed velocity.
            const double speed = std::hypot(vel_seed.x(), vel_seed.y());
            if (speed >= config.nhc_min_speed_mps &&
                wstats.yaw_rate_abs <= config.nhc_max_yaw_rate_radps) {
                gtsam::Vector2 nhc_sigmas(config.nhc_sigma_lateral_mps,
                                          config.nhc_sigma_vertical_mps);
                new_factors.emplace_shared<NonHolonomicFactor>(
                    positionKey(i), velocityKey(i),
                    gtsam::noiseModel::Diagonal::Sigmas(nhc_sigmas));
                ++result.diagnostics.nhc_epochs;
            }
        }

        // --- Preemptive regularization ---
        // Weak-geometry urban epochs (few DD sats + near-stationary IMU) can
        // leave a DOF (typically yaw, or velocity/bias) unobservable, so the
        // incremental Cholesky throws IndeterminantLinearSystemException. That
        // is unrecoverable here: iSAM2 partially commits the new values before
        // throwing, and skipping the epoch orphans X(i)/V(i)/B(i) so every later
        // IMU factor references a missing key ("invalid map key") and the whole
        // tail cascades. A very loose per-epoch prior on pose/velocity/bias
        // (anchored to the seed) regularizes the Hessian so it is always
        // positive-definite. Sigmas are far looser than the real measurements
        // (rot 10 rad, trans 1000 m, vel 100 m/s, bias 10), so the effect on
        // the DD-cm solution is negligible -- they only fill rank in the null
        // space that would otherwise be unobservable.
        {
            // Sigmas loose vs the real measurements (DD ~cm, IMU) but tight
            // enough to keep the Hessian well-conditioned even when an epoch's
            // measurement info on some DOF is ~0 (few DD sats / IMU dropout):
            // rot 1 rad, trans 100 m, vel 10 m/s, bias 1.
            gtsam::Vector6 reg_pose;
            reg_pose << 1.0, 1.0, 1.0, 100.0, 100.0, 100.0;
            new_factors.addPrior(positionKey(i), pose_seed,
                                 gtsam::noiseModel::Diagonal::Sigmas(reg_pose));
            const gtsam::Vector3 v_reg = vel_seed;
            new_factors.addPrior<gtsam::Vector3>(velocityKey(i), v_reg,
                                                 gtsam::noiseModel::Isotropic::Sigma(3, 10.0));
            new_factors.addPrior(biasKey(i), prev_bias,
                                 gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
        }

        // --- Smoother update ---
        bool update_ok = true;
        // FDE (use_fde): resolved only on the PRIMARY (non-exception) path,
        // since exception recovery below (loose-prior retry / warm reset)
        // never actually adds this epoch's own DD PR/CP factors to the
        // graph -- there would be nothing for FDE to evaluate this epoch in
        // that case (see runFde's call site further down for the gating).
        gtsam::FactorIndices fde_new_indices;
        bool fde_indices_valid = false;
        try {
            smoother.update(new_factors, new_values, ts);
            if (config.use_fde) {
                fde_new_indices = smoother.getISAM2Result().newFactorsIndices;
                fde_indices_valid = true;
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[fgo_gtsam_backend] smoother.update() epoch %zu threw: %s\n", i,
                         e.what());
            update_ok = false;
            ++result.diagnostics.smoother_recovery_epochs;
            // --- Exception recovery (use_solve_exception_recovery; reference
            // recovery.handle_solve_exception). Targets the known
            // IndeterminantLinearSystemException tail failure (tokyo run2,
            // ~epoch 6390): without this, the epoch is skipped (`continue`
            // below) and the run keeps going in a degraded state, but a
            // SECOND failure while the graph is still poisoned would repeat
            // forever with no recovery. ---
            if (config.use_solve_exception_recovery) {
                // Stage 0 (use_ddpr_anchor only; reference
                // handle_solve_exception's ACTUAL first choice --
                // try_ddpr_reset before the loose-prior fallback): warm-reset
                // the smoother seeded at (this epoch's DDPR-LS anchor
                // translation, IMU-predicted rotation, zero velocity) when
                // the anchor is trusted. Falls through to stage 1 otherwise.
                bool anchor_recovered = false;
                if (config.use_ddpr_anchor) {
                    ++result.diagnostics.ddpr_anchor_solves;
                    const DdprAnchorResult anchor = solveDdprAnchor(i, pose_seed);
                    if (anchor.ok && anchor.n_active >= config.ddpr_anchor_min_factors &&
                        anchor.res_rms <= config.ddpr_anchor_max_residual_m) {
                        ++result.diagnostics.ddpr_anchor_successes;
                        try {
                            const Pose3 anchor_seed_pose(pose_seed.rotation(),
                                                         anchor.pose.translation());
                            performFullWarmReset(i, anchor_seed_pose, prev_bias);
                            update_ok = true;
                            anchor_recovered = true;
                            ++result.diagnostics.ddpr_anchored_warm_resets;
                        } catch (const std::exception& e_anchor) {
                            std::fprintf(stderr,
                                         "[fgo_gtsam_backend] ddpr-anchored warm reset epoch %zu "
                                         "threw: %s -- falling back\n",
                                         i, e_anchor.what());
                        }
                    }
                }
                if (!anchor_recovered) {
                // Stage 1: retry with ONLY loose re-seed priors at the
                // IMU-predicted state, discarding this epoch's GNSS/IMU
                // factors entirely.
                gtsam::NonlinearFactorGraph retry_factors;
                gtsam::Values retry_values;
                gtsam::FixedLagSmoother::KeyTimestampMap retry_ts;
                retry_values.insert(positionKey(i), pose_seed);
                retry_values.insert(velocityKey(i), vel_seed);
                retry_values.insert(biasKey(i), prev_bias);
                retry_factors.addPrior(
                    positionKey(i), pose_seed,
                    gtsam::noiseModel::Isotropic::Sigma(6, config.solve_exception_pose_sigma_m));
                retry_factors.addPrior<gtsam::Vector3>(
                    velocityKey(i), vel_seed,
                    gtsam::noiseModel::Isotropic::Sigma(3, config.solve_exception_velocity_sigma_mps));
                retry_factors.addPrior(
                    biasKey(i), prev_bias,
                    gtsam::noiseModel::Isotropic::Sigma(6, config.solve_exception_bias_sigma));
                retry_ts[positionKey(i)] = te;
                retry_ts[velocityKey(i)] = te;
                retry_ts[biasKey(i)] = te;
                bool retry_ok = true;
                try {
                    smoother.update(retry_factors, retry_values, retry_ts);
                    ++result.diagnostics.smoother_updates;
                } catch (const std::exception& e2) {
                    std::fprintf(stderr,
                                 "[fgo_gtsam_backend] loose-prior retry epoch %zu ALSO threw: %s "
                                 "-- performing full warm reset\n",
                                 i, e2.what());
                    retry_ok = false;
                }
                if (retry_ok) {
                    update_ok = true;
                    ++result.diagnostics.solve_exception_recoveries;
                } else {
                    // Stage 2: repeated failure -- full warm reset (reference
                    // warm_reset_phase2), seeded at the IMU prediction (the
                    // DDPR anchor was untrusted or use_ddpr_anchor is off).
                    try {
                        performFullWarmReset(i, pose_seed, prev_bias);
                        update_ok = true;
                        ++result.diagnostics.solve_exception_warm_resets;
                    } catch (const std::exception& e3) {
                        std::fprintf(stderr,
                                     "[fgo_gtsam_backend] full warm reset epoch %zu threw: %s -- "
                                     "epoch skipped\n",
                                     i, e3.what());
                        update_ok = false;
                    }
                }
                }  // !anchor_recovered
            }
        }
        ++result.diagnostics.smoother_updates;
        if (!update_ok) {
            continue;
        }
        epoch_solved[i] = true;
        result.diagnostics.smoother_max_window_vars = std::max(
            result.diagnostics.smoother_max_window_vars, smoother.getLinearizationPoint().size());

        // Refined state for epoch i (seed source for next epoch).
        Pose3 pose_i = smoother.calculateEstimate<Pose3>(positionKey(i));
        gtsam::Vector3 vel_i = smoother.calculateEstimate<gtsam::Vector3>(velocityKey(i));
        prev_bias = smoother.calculateEstimate<gtsam::imuBias::ConstantBias>(biasKey(i));
        prev_nav = gtsam::NavState(pose_i, vel_i);

        // Re-read every still-in-window pose so each epoch's recorded estimate
        // is its most-smoothed value at the moment it exits the window.
        const gtsam::Values& lin = smoother.getLinearizationPoint();
        for (std::size_t j = i + 1; j-- > 0;) {
            if (!lin.exists(positionKey(j))) break;  // older keys already marginalized
            const Pose3 pj = smoother.calculateEstimate<Pose3>(positionKey(j));
            const gtsam::Vector3 vj = smoother.calculateEstimate<gtsam::Vector3>(velocityKey(j));
            epoch_float_position[j] = antennaOf(pj);
            const gtsam::Matrix3 R = pj.rotation().matrix();
            const Eigen::Vector3d fwd = R.col(0);
            const Eigen::Vector3d left = R.col(1);
            constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
            double heading = std::atan2(fwd.x(), fwd.y()) * kRadToDeg;
            if (heading < 0.0) heading += 360.0;
            const double pitch = std::asin(std::max(-1.0, std::min(1.0, fwd.z()))) * kRadToDeg;
            const double roll = std::atan2(left.z(), R.col(2).z()) * kRadToDeg;
            epoch_rpy_deg[j] = Vector3d(roll, pitch, heading);
            epoch_vel_nav[j] = Vector3d(vj);
        }

        // --- Shared post-fit DDPR residual / GDOP computation (reference:
        // postfit.main_ddpr_residuals -- epoch RMS + per-sat max, charged to
        // both the target and the reference satellite -- and gate.py's
        // GDOP/nsat gate). Computed ONCE at the smoothed antenna position
        // AFTER the update and reused by BOTH the per-epoch quality gates
        // (use_epoch_quality_gates) and the CP-hold/sanity FSM
        // (use_cp_hold_recovery) below -- the two features are independent
        // knobs but share this one (possibly expensive) residual pass. ---
        const bool need_ddpr_residuals =
            config.use_epoch_quality_gates || config.use_cp_hold_recovery ||
            config.use_sat_badness_downweight ||
            config.use_imu_integration_covariance_inflation;
        int nsat = 0;
        double gdop = std::numeric_limits<double>::infinity();
        double ddpr_rms = 0.0;
        std::map<SatelliteId, double> per_sat_res;
        // Per-(ref, target, signal) residual rows, only collected when the
        // sat-badness pair term is actually active (fgo.hpp deviation 5) --
        // feeds update_pair_quality below.
        const bool collect_pair_rows =
            config.use_sat_badness_downweight && config.sat_badness_alpha_recent_pair > 0.0;
        std::vector<std::tuple<SatelliteId, SatelliteId, SignalType, double>> sb_pair_rows;
        if (need_ddpr_residuals) {
            const Point3 ant_p = antennaOf(pose_i);
            const Vector3d ant(ant_p.x(), ant_p.y(), ant_p.z());

            // (a) GDOP / nsat over this epoch's DD satellites.
            std::map<SatelliteId, Vector3d> sat_positions;
            for (const auto* fp : pr_by_epoch[i]) {
                sat_positions.emplace(fp->satellite, fp->rover_satellite_position_ecef);
                sat_positions.emplace(fp->reference_satellite,
                                      fp->rover_reference_position_ecef);
            }
            nsat = static_cast<int>(sat_positions.size());
            if (nsat >= 4) {
                Eigen::MatrixXd H(nsat, 4);
                int row = 0;
                for (const auto& [sid, sp] : sat_positions) {
                    (void)sid;
                    const Vector3d d = sp - ant;
                    const double rng = d.norm();
                    if (rng > 0.0) {
                        H.block<1, 3>(row, 0) = (-d / rng).transpose();
                    } else {
                        H.block<1, 3>(row, 0).setZero();
                    }
                    H(row, 3) = 1.0;
                    ++row;
                }
                const Eigen::Matrix4d Ninv = (H.transpose() * H).inverse();
                if (Ninv.allFinite()) {
                    gdop = std::sqrt(std::max(0.0, Ninv.trace()));
                }
            }

            // (b) post-fit DD-pseudorange residuals at the smoothed antenna.
            double res_sq_sum = 0.0;
            std::size_t res_n = 0;
            for (const auto* fp : pr_by_epoch[i]) {
                const double geom =
                    ((fp->rover_satellite_position_ecef - ant).norm() -
                     (fp->base_satellite_position_ecef - fp->base_position_ecef).norm()) -
                    ((fp->rover_reference_position_ecef - ant).norm() -
                     (fp->base_reference_position_ecef - fp->base_position_ecef).norm());
                const double res = std::abs(fp->observed_dd_pseudorange_m - geom);
                res_sq_sum += res * res;
                ++res_n;
                double& worst = per_sat_res[fp->satellite];
                worst = std::max(worst, res);
                double& worst_ref = per_sat_res[fp->reference_satellite];
                worst_ref = std::max(worst_ref, res);
                if (collect_pair_rows) {
                    sb_pair_rows.emplace_back(fp->reference_satellite, fp->satellite,
                                              fp->signal, res);
                }
            }
            ddpr_rms = res_n > 0 ? std::sqrt(res_sq_sum / static_cast<double>(res_n)) : 0.0;
        }

        // --- Sat-badness EWMA down-weighting: per-epoch state update
        // (FGOConfig::use_sat_badness_downweight; port of the reference's
        // update_reference_quality / update_observation_quality /
        // update_pair_quality, called once per epoch directly off THIS
        // epoch's pre-FDE per_sat_res -- same ordering as the reference's
        // stage.py _compute_postfit_diagnostics, which calls these BEFORE
        // apply_fde). ---
        if (config.use_sat_badness_downweight) {
            constexpr double kRadToDegSb = 180.0 / 3.14159265358979323846;

            // (a) update_reference_quality: decay/update recent_ref_bad for
            // every DISTINCT reference satellite backing this epoch's DD
            // pseudorange factors (the reference selects one ref per
            // (system, signal) group; only the set of currently-active refs
            // matters here, not the group key itself).
            {
                const double decay = std::min(1.0, std::max(0.0, config.sat_badness_recent_ref_decay));
                const double thr = std::max(1e-6, config.sat_badness_obsq_res_threshold_m);
                std::set<SatelliteId> active_refs;
                for (const auto* fp : pr_by_epoch[i]) active_refs.insert(fp->reference_satellite);
                for (SatelliteId r : active_refs) {
                    const double prev = sb_recent_ref_bad.count(r) ? sb_recent_ref_bad[r] : 0.0;
                    const auto it = per_sat_res.find(r);
                    // CLAMPED-variant deviation (fgo.hpp sat_badness_residual_
                    // clamp_m): clamp upstream of the reference's own existing
                    // min(2,.) increment limit below (left unchanged).
                    const double res = it != per_sat_res.end() ? sbClampResidual(it->second) : 0.0;
                    const double incr = res > 0.0 ? std::min(2.0, res / thr) : 0.0;
                    sb_recent_ref_bad[r] = decay * prev + incr;
                }
                for (auto& [sid, v] : sb_recent_ref_bad) {
                    if (!active_refs.count(sid)) v = decay * v;
                }
            }

            // (b) update_observation_quality: per-sat EWMA/streak (hard reset
            // -- not decay -- for sats not seen this epoch), recent_worst /
            // recent_cppr decay, and latest el/snr snapshot.
            std::optional<SatelliteId> worst_sat;
            {
                const double alpha = std::min(1.0, std::max(0.0, config.sat_badness_obsq_ewma_alpha));
                const double thr_obsq = config.sat_badness_obsq_bad_streak_threshold_m;
                const double worst_decay = std::min(1.0, std::max(0.0, config.sat_badness_recent_worst_decay));
                const double cppr_decay = std::min(1.0, std::max(0.0, config.sat_badness_recent_cppr_decay));

                std::set<SatelliteId> seen;
                double worst_val = -1.0;
                for (const auto& [sid, rmax] : per_sat_res) {
                    seen.insert(sid);
                    // CLAMPED-variant deviation (fgo.hpp sat_badness_residual_
                    // clamp_m): clamp the residual before it feeds the EWMA
                    // input or the bad-streak comparison -- both direct
                    // badness inputs. The "worst satellite" argmax just below
                    // deliberately keeps the RAW rmax: clamping is monotonic
                    // (order-preserving), so it cannot change which satellite
                    // is identified as worst, and per_sat_res itself (read by
                    // use_epoch_quality_gates/use_cp_hold_recovery elsewhere)
                    // is never touched.
                    const double rmax_c = sbClampResidual(rmax);
                    const double prev = sb_obsq_ewma.count(sid) ? sb_obsq_ewma[sid] : 0.0;
                    sb_obsq_ewma[sid] = prev <= 0.0 ? rmax_c : ((1.0 - alpha) * prev + alpha * rmax_c);
                    if (rmax_c > thr_obsq) {
                        sb_obsq_bad_streak[sid] = (sb_obsq_bad_streak.count(sid) ? sb_obsq_bad_streak[sid] : 0) + 1;
                    } else {
                        sb_obsq_bad_streak[sid] = 0;
                    }
                    if (rmax > worst_val) {
                        worst_val = rmax;
                        worst_sat = sid;
                    }
                }
                for (auto& [sid, v] : sb_obsq_ewma) {
                    if (!seen.count(sid)) v = 0.0;
                }
                for (auto& [sid, v] : sb_obsq_bad_streak) {
                    if (!seen.count(sid)) v = 0;
                }

                // recent_cppr's "this epoch" input: our FDE-substitute reject
                // count as of THIS point in the pipeline (i.e. as of the end
                // of the LAST epoch FDE actually ran -- FDE for THIS epoch
                // runs later, below), maxed across signal for each satellite
                // (mirrors gate.py's sat_cppr_sat: max over freq of
                // rejc_cp_pr). See fgo.hpp deviation 1.
                std::map<SatelliteId, double> cppr_this_epoch;
                for (const auto& [key, cnt] : sb_fde_cp_reject_count) {
                    double& v = cppr_this_epoch[key.first];
                    v = std::max(v, cnt);
                }

                std::set<SatelliteId> active_sats(seen);
                if (worst_sat) active_sats.insert(*worst_sat);
                for (const auto& [sid, v] : cppr_this_epoch) {
                    (void)v;
                    active_sats.insert(sid);
                }

                for (SatelliteId sid : active_sats) {
                    const double prev_w = sb_recent_worst.count(sid) ? sb_recent_worst[sid] : 0.0;
                    const double is_worst = (worst_sat && *worst_sat == sid) ? 1.0 : 0.0;
                    sb_recent_worst[sid] = worst_decay * prev_w + is_worst;

                    const double prev_c = sb_recent_cppr.count(sid) ? sb_recent_cppr[sid] : 0.0;
                    const double cppr_cur = cppr_this_epoch.count(sid) ? cppr_this_epoch[sid] : 0.0;
                    sb_recent_cppr[sid] = cppr_decay * prev_c + cppr_cur;
                }
                for (auto& [sid, v] : sb_recent_worst) {
                    if (!active_sats.count(sid)) v = worst_decay * v;
                }
                for (auto& [sid, v] : sb_recent_cppr) {
                    if (!active_sats.count(sid)) v = cppr_decay * v;
                }

                // Latest elevation/SNR: only entries observed THIS epoch are
                // updated (reference: latest_el_deg/latest_snr_dbhz retain
                // their last-seen value for sats absent this epoch).
                for (const auto* fp : pr_by_epoch[i]) {
                    sb_latest_el_deg[fp->satellite] = fp->elevation_rad * kRadToDegSb;
                    sb_latest_el_deg[fp->reference_satellite] =
                        fp->rover_reference_model.elevation_rad * kRadToDegSb;
                    sb_latest_snr_dbhz[fp->satellite] = fp->rover_satellite_model.snr_dbhz;
                    sb_latest_snr_dbhz[fp->reference_satellite] =
                        fp->rover_reference_model.snr_dbhz;
                }
            }

            // (c) update_pair_quality: gated on the alpha itself (fgo.hpp
            // deviation 5) -- the reference profile ships alpha_recent_pair
            // at 0.0, contributing nothing.
            if (collect_pair_rows) {
                const double decay = std::min(1.0, std::max(0.0, config.sat_badness_recent_pair_decay));
                const double thr = std::max(1e-6, config.sat_badness_obsq_res_threshold_m);
                std::set<std::tuple<SatelliteId, SatelliteId, SignalType>> seen_pairs;
                for (const auto& [ref, sat, freq, res] : sb_pair_rows) {
                    const auto key = std::make_tuple(ref, sat, freq);
                    seen_pairs.insert(key);
                    const double prev = sb_recent_pair_bad.count(key) ? sb_recent_pair_bad[key] : 0.0;
                    // CLAMPED-variant deviation (fgo.hpp sat_badness_residual_
                    // clamp_m): same upstream clamp as recent_ref_bad above,
                    // ahead of the reference's own existing min(2,.) limit.
                    const double incr = res > 0.0 ? std::min(2.0, sbClampResidual(res) / thr) : 0.0;
                    sb_recent_pair_bad[key] = decay * prev + incr;
                }
                for (auto& [key, v] : sb_recent_pair_bad) {
                    if (!seen_pairs.count(key)) v = decay * v;
                }
            }

            // (d) Snapshot THIS epoch's (pre-FDE) per_sat_res as "last epoch"
            // for the NEXT epoch's res_s term (reference: tc._mres_signals.
            // per_sat, set from this same pre-FDE per_sat_res pass).
            // CLAMPED-variant deviation (fgo.hpp sat_badness_residual_clamp_m):
            // the snapshot itself is clamped -- this is the direct res_s term
            // satBadness() reads next epoch -- while per_sat_res (the shared
            // map other features also read) stays raw.
            sb_last_ddpr_per_sat.clear();
            for (const auto& [sid, r] : per_sat_res) {
                sb_last_ddpr_per_sat[sid] = sbClampResidual(r);
            }
        }

        // --- Per-epoch quality gates (port of the reference's gate.py /
        // postfit.py fixing policy; see FGOConfig::use_epoch_quality_gates).
        // Gated epochs keep their factors but attempt no LAMBDA, add no
        // holds, and are never labelled FIXED via held integers. ---
        bool fix_allowed = true;
        if (config.use_epoch_quality_gates) {
            if (nsat < config.gate_min_satellites ||
                gdop > config.gate_gdop_max ||
                (config.gate_ddpr_res_max_m > 0.0 &&
                 ddpr_rms > config.gate_ddpr_res_max_m)) {
                fix_allowed = false;
                ++result.diagnostics.quality_gated_epochs;
            }

            // (c) satellites flagged by LAST epoch's post-fit residuals stay
            // out of this epoch's LAMBDA candidate set (their factors and
            // held pins are untouched).
            if (!gate_bad_sats.empty() && !epoch_amb_indices.empty()) {
                epoch_amb_indices.erase(
                    std::remove_if(
                        epoch_amb_indices.begin(), epoch_amb_indices.end(),
                        [&](std::size_t idx) {
                            const auto& amb = problem.ambiguity_states[idx];
                            return gate_bad_sats.count(amb.satellite) > 0 ||
                                   gate_bad_sats.count(amb.reference_satellite) > 0;
                        }),
                    epoch_amb_indices.end());
            }
            gate_bad_sats.clear();
            if (config.gate_per_sat_res_max_m > 0.0) {
                for (const auto& [sid, r] : per_sat_res) {
                    if (r > config.gate_per_sat_res_max_m) {
                        gate_bad_sats.insert(sid);
                    }
                }
            }
        }

        // --- FDE (use_fde): runs AFTER the shared post-fit DDPR diagnostics
        // pass above (nsat/gdop/ddpr_rms/per_sat_res, already computed at
        // the post-solve pre-FDE estimate -- the CP-hold/sanity FSM further
        // below reuses THOSE pre-FDE numbers unchanged, exactly mirroring
        // the reference's stage.py ordering: main_ddpr_residuals runs
        // before apply_fde, so the sanity trigger sees pre-FDE residuals)
        // but BEFORE per-epoch LAMBDA/fix-and-hold, so ambiguity resolution
        // sees the float already cleaned of gross outliers. Gated on the
        // PRIMARY smoother.update having succeeded without exception (see
        // fde_indices_valid's declaration above).
        if (config.use_fde && fde_indices_valid) {
            std::set<std::size_t> fde_rejected_amb;
            const std::size_t fde_removed =
                runFde(i, fde_local_indices, fde_new_indices, &fde_rejected_amb);
            if (fde_removed > 0) {
                ++result.diagnostics.fde_epochs;
                // A rejected CP factor's ambiguity_index may still be
                // sitting in this epoch's still-pending LAMBDA candidate
                // list (built before FDE ran) -- its factor no longer
                // exists, so drop it (mirrors the cp-hold suppression
                // branch above, which never adds a suppressed arc to
                // epoch_amb_indices in the first place).
                if (!fde_rejected_amb.empty()) {
                    epoch_amb_indices.erase(
                        std::remove_if(epoch_amb_indices.begin(), epoch_amb_indices.end(),
                                       [&](std::size_t idx) {
                                           return fde_rejected_amb.count(idx) > 0;
                                       }),
                        epoch_amb_indices.end());
                }
                // Refresh the post-FDE estimate: downstream LAMBDA/fix-and-
                // hold and the REPORTED pose for this epoch must see the
                // cleaned float (reference: the pose snapshot in stage.py's
                // _compute_postfit_diagnostics happens AFTER apply_fde).
                pose_i = smoother.calculateEstimate<Pose3>(positionKey(i));
                vel_i = smoother.calculateEstimate<gtsam::Vector3>(velocityKey(i));
                prev_bias = smoother.calculateEstimate<gtsam::imuBias::ConstantBias>(biasKey(i));
                prev_nav = gtsam::NavState(pose_i, vel_i);
                epoch_float_position[i] = antennaOf(pose_i);
                const gtsam::Matrix3 R_fde = pose_i.rotation().matrix();
                const Eigen::Vector3d fwd_fde = R_fde.col(0);
                const Eigen::Vector3d left_fde = R_fde.col(1);
                constexpr double kRadToDegFde = 180.0 / 3.14159265358979323846;
                double heading_fde = std::atan2(fwd_fde.x(), fwd_fde.y()) * kRadToDegFde;
                if (heading_fde < 0.0) heading_fde += 360.0;
                const double pitch_fde =
                    std::asin(std::max(-1.0, std::min(1.0, fwd_fde.z()))) * kRadToDegFde;
                const double roll_fde = std::atan2(left_fde.z(), R_fde.col(2).z()) * kRadToDegFde;
                epoch_rpy_deg[i] = Vector3d(roll_fde, pitch_fde, heading_fde);
                epoch_vel_nav[i] = Vector3d(vel_i);
            }
        }

        // --- Per-epoch LAMBDA off the bounded windowed marginals ---
        if (fix_allowed && config.use_lambda_ambiguity_fix && !epoch_amb_indices.empty()) {
            std::sort(epoch_amb_indices.begin(), epoch_amb_indices.end(),
                      [&](std::size_t a, std::size_t b) {
                          const auto& sa = problem.ambiguity_states[a];
                          const auto& sb = problem.ambiguity_states[b];
                          return std::tie(sa.satellite, sa.reference_satellite, sa.signal, a) <
                                 std::tie(sb.satellite, sb.reference_satellite, sb.signal, b);
                      });
            const int n = static_cast<int>(epoch_amb_indices.size());
            if (n >= min_candidates) {
                gtsam::KeyVector keys;
                keys.reserve(n + 1);
                keys.push_back(positionKey(i));
                for (std::size_t idx : epoch_amb_indices) keys.push_back(ambiguityKey(ambSymbolId(idx)));

                Eigen::VectorXd float_amb(n);
                Eigen::MatrixXd q_amb(n, n);
                Eigen::MatrixXd pos_amb(3, n);
                bool ok = true;
                try {
                    const gtsam::JointMarginal joint =
                        smoother.getISAM2().jointMarginalCovariance(keys);
                    // Cov(antenna, amb) via the 3x6 antenna Jacobian at pose_i.
                    gtsam::Matrix36 H_antenna_pose;
                    {
                        gtsam::gnss::LeverArm::PoseFrame frame;
                        gnss_lever_arm.antennaPosition(pose_i, &frame);
                        for (int r3 = 0; r3 < 3; ++r3) {
                            gtsam::Matrix13 unit = gtsam::Matrix13::Zero();
                            unit(0, r3) = 1.0;
                            H_antenna_pose.row(r3) = gnss_lever_arm.antennaPoseJacobian(unit, frame);
                        }
                    }
                    for (int r = 0; r < n && ok; ++r) {
                        const gtsam::Key rk = ambiguityKey(ambSymbolId(epoch_amb_indices[r]));
                        float_amb(r) = smoother.calculateEstimate<double>(rk);
                        const gtsam::Matrix pr = joint(positionKey(i), rk);  // 6x1
                        const Eigen::Vector3d pr3 = H_antenna_pose * pr;
                        pos_amb(0, r) = pr3(0);
                        pos_amb(1, r) = pr3(1);
                        pos_amb(2, r) = pr3(2);
                        for (int c = 0; c < n; ++c) {
                            q_amb(r, c) = joint(rk, ambiguityKey(ambSymbolId(epoch_amb_indices[c])))(0, 0);
                        }
                    }
                } catch (const std::exception&) {
                    ok = false;
                }
                if (ok && float_amb.allFinite() && q_amb.allFinite()) {
                    q_amb = 0.5 * (q_amb + q_amb.transpose());
                    for (int k = 0; k < n; ++k) {
                        q_amb(k, k) += std::max(1e-12, std::abs(q_amb(k, k)) * 1e-9);
                    }

                    // --- MF-AR step 2: partial AR (port of the Eigen path's
                    // use_partial_lambda_ambiguity_fix). Candidate order:
                    // identity (= the deterministic sat/ref/signal sort above)
                    // for the legacy all-or-nothing path; best-first by
                    // (fractional cycles, variance) with a
                    // max_lambda_ambiguities cap when partial AR is enabled.
                    // The loop below runs exactly ONCE at full size when
                    // partial AR is off -- bit-identical legacy behaviour. ---
                    std::vector<int> order(n);
                    std::iota(order.begin(), order.end(), 0);
                    int attempt_n = n;
                    int min_subset = n;  // all-or-nothing: single attempt
                    if (config.use_fixed_lag_partial_lambda) {
                        std::stable_sort(
                            order.begin(), order.end(), [&](int a, int b) {
                                const double fa = std::abs(
                                    float_amb(a) - std::round(float_amb(a)));
                                const double fb = std::abs(
                                    float_amb(b) - std::round(float_amb(b)));
                                if (fa == fb) {
                                    return q_amb(a, a) < q_amb(b, b);
                                }
                                return fa < fb;
                            });
                        attempt_n = config.max_lambda_ambiguities > 0
                                        ? std::min(n, config.max_lambda_ambiguities)
                                        : n;
                        min_subset = std::max(1, config.min_fixed_ambiguities);
                        // Subset floor: drop a few outliers, never the
                        // majority (see fixed_lag_partial_lambda_min_fraction).
                        const double fraction = std::min(
                            1.0, std::max(0.0,
                                          config.fixed_lag_partial_lambda_min_fraction));
                        min_subset = std::max(
                            min_subset,
                            static_cast<int>(std::ceil(fraction * attempt_n)));
                    }

                    bool float_cycles_recorded = false;
                    for (int subset = attempt_n; subset >= min_subset; --subset) {
                        Eigen::VectorXd sub_float(subset);
                        Eigen::MatrixXd sub_q(subset, subset);
                        Eigen::MatrixXd sub_pos(3, subset);
                        for (int r = 0; r < subset; ++r) {
                            sub_float(r) = float_amb(order[r]);
                            sub_pos.col(r) = pos_amb.col(order[r]);
                            for (int c = 0; c < subset; ++c) {
                                sub_q(r, c) = q_amb(order[r], order[c]);
                            }
                        }

                        Eigen::VectorXd fixed_amb;
                        double ratio = 0.0;
                        ++lambda_attempts;
                        ++result.diagnostics.lambda_ambiguity_attempts;
                        result.diagnostics.lambda_ambiguity_candidates +=
                            static_cast<std::size_t>(subset);
                        if (!lambdaSearch(sub_float, sub_q, fixed_amb, ratio)) {
                            if (!config.use_fixed_lag_partial_lambda) break;
                            continue;
                        }
                        result.diagnostics.lambda_ambiguity_fix_solved = true;
                        best_ratio = std::max(best_ratio, ratio);
                        epoch_ratio[i] = ratio;
                        const bool fixed_epoch =
                            std::isfinite(ratio) &&
                            (config.lambda_ratio_threshold <= 0.0 ||
                             ratio > config.lambda_ratio_threshold) &&
                            fixed_amb.size() == subset;
                        // Record float ambiguity values for the result mapping
                        // (full candidate set, once).
                        if (!float_cycles_recorded) {
                            for (int r = 0; r < n; ++r) {
                                amb_float_cycles[epoch_amb_indices[r]] = float_amb(r);
                            }
                            float_cycles_recorded = true;
                        }
                        if (!fixed_epoch) {
                            if (!config.use_fixed_lag_partial_lambda) break;
                            continue;
                        }

                        epoch_fixed[i] = true;
                        epoch_fixed_count[i] = subset;
                        total_fixed_ambiguities += static_cast<std::size_t>(subset);
                        result.diagnostics.lambda_ambiguity_fix_used = true;
                        result.diagnostics.lambda_ambiguity_used_candidates +=
                            static_cast<std::size_t>(subset);
                        if (subset < attempt_n) {
                            result.diagnostics.partial_lambda_ambiguity_fix_used = true;
                        }
                        for (int r = 0; r < subset; ++r) {
                            const std::size_t idx = epoch_amb_indices[order[r]];
                            const int fi = static_cast<int>(std::lround(fixed_amb(r)));
                            amb_fixed_cycles[idx] = fi;
                            amb_fixed_residual[idx] =
                                sub_float(r) - static_cast<double>(fi);
                        }
                        if (config.use_epoch_lambda_fixed_output) {
                            const Eigen::VectorXd delta = sub_float - fixed_amb;
                            const Eigen::LDLT<Eigen::MatrixXd> ldlt(sub_q);
                            if (ldlt.info() == Eigen::Success) {
                                const Eigen::VectorXd corr = ldlt.solve(delta);
                                if (corr.allFinite()) {
                                    const Eigen::Vector3d pd = sub_pos * corr;
                                    if (pd.allFinite()) {
                                        epoch_fixed_position[i] = antennaOf(pose_i) - Point3(pd);
                                        epoch_has_fixed[i] = true;
                                    }
                                }
                            }
                        }

                        // --- 2e fix-and-hold: pin newly-validated arcs at
                        // their integer so they persist for the rest of the
                        // arc. Stricter ratio gate than "mark FIXED". With
                        // partial AR, only a FULL-set validation may hold:
                        // a shrunken subset that squeaked past the ratio test
                        // is enough to label this epoch FIXED, but pinning its
                        // integers would poison the arc for its whole
                        // remaining lifetime if any are wrong (measured on
                        // tokyo1 full-run deep urban: FIXED rms 6.2 m). ---
                        if (config.use_ambiguity_hold &&
                            subset == attempt_n &&
                            ratio > config.ambiguity_hold_ratio_threshold &&
                            subset >= config.ambiguity_hold_min_fixed) {
                            gtsam::NonlinearFactorGraph hold_factors;
                            const auto hold_noise = gtsam::noiseModel::Isotropic::Sigma(
                                1, config.ambiguity_hold_sigma_cycles);
                            for (int r = 0; r < subset; ++r) {
                                const std::size_t idx = epoch_amb_indices[order[r]];
                                const std::size_t sym_idx = ambSymbolId(idx);
                                if (pinned_ambiguities.count(sym_idx)) continue;
                                const int fi = static_cast<int>(std::lround(fixed_amb(r)));
                                hold_factors.addPrior(ambiguityKey(sym_idx),
                                                      static_cast<double>(fi), hold_noise);
                                pinned_ambiguities.insert(sym_idx);
                            }
                            if (hold_factors.size() > 0) {
                                try {
                                    smoother.update(hold_factors, gtsam::Values(),
                                                    gtsam::FixedLagSmoother::KeyTimestampMap());
                                    ++result.diagnostics.smoother_updates;
                                    // Re-read epoch i now that holds pin it.
                                    pose_i = smoother.calculateEstimate<Pose3>(positionKey(i));
                                    vel_i = smoother.calculateEstimate<gtsam::Vector3>(
                                        velocityKey(i));
                                    prev_nav = gtsam::NavState(pose_i, vel_i);
                                    epoch_float_position[i] = antennaOf(pose_i);
                                } catch (const std::exception& e) {
                                    std::fprintf(stderr,
                                                 "[fgo_gtsam_backend] hold update epoch %zu "
                                                 "threw: %s\n",
                                                 i, e.what());
                                }
                            }
                        }
                        break;  // validated subset found
                    }
                } else {
                    ++marginals_failures;
                }
            }
        }

        // --- 2e fix-and-hold: an epoch whose arcs are (mostly) already held is
        // FIXED regardless of the fresh per-epoch LAMBDA -- the integers are
        // known and the smoother position (with the held priors active) is the
        // fixed solution. This is the main fix-rate lever. Quality-gated
        // epochs are never labelled FIXED this way (reference: no fixing on a
        // corrupt epoch). ---
        if (fix_allowed && config.use_ambiguity_hold && !epoch_fixed[i]) {
            int held_here = 0;
            for (std::size_t idx : epoch_amb_indices) {
                if (pinned_ambiguities.count(ambSymbolId(idx))) ++held_here;
            }
            if (held_here >= config.ambiguity_hold_min_fixed) {
                epoch_fixed[i] = true;
                epoch_fixed_count[i] = held_here;
                epoch_has_fixed[i] = true;
                epoch_fixed_position[i] = antennaOf(pose_i);
                ++held_epoch_count;
                for (std::size_t idx : epoch_amb_indices) {
                    const std::size_t sym_idx = ambSymbolId(idx);
                    if (pinned_ambiguities.count(sym_idx) && !amb_fixed_cycles.count(idx)) {
                        // Record the held integer (rounded from the current estimate).
                        const double v = smoother.calculateEstimate<double>(ambiguityKey(sym_idx));
                        amb_fixed_cycles[idx] = static_cast<int>(std::lround(v));
                    }
                }
            }
        }

        // --- CP-hold / sanity FSM (use_cp_hold_recovery). Runs LAST, after
        // this epoch's LAMBDA/fix-and-hold, because the catastrophic fast
        // path needs "no fixed solution this epoch" (nb == 0) exactly like
        // the reference's postfit.run_ddpr_sanity(..., nb=...), which the
        // runner calls from validation/postprocess.py AFTER AR for the
        // epoch (see validation/postfit.py's trigger -> multipath-skip ->
        // fast-path -> persist -> gdop-skip -> anchor stages -> apply-reset
        // pipeline; the DDPR-LS anchor stages are ported inside the persist
        // path below, gated by FGOConfig::use_ddpr_anchor). ---
        if (config.use_cp_hold_recovery) {
            const int nb = epoch_fixed[i] ? epoch_fixed_count[i] : 0;
            bool did_reset = false;
            double pred_res = 0.0;
            if (ddpr_rms > config.cp_hold_main_residual_threshold_m) {
                // Stage: multipath-dominated skip -- one dominant multipath
                // satellite is not a wrong basin (reference
                // _ddpr_multipath_dominated). Bad-count/CP-hold are frozen
                // (not reset, not incremented) on this path, matching the
                // reference: the skip returns before _ddpr_sanity_persist.
                bool multipath_dominated = false;
                if (config.cp_hold_multipath_median_ratio > 0.0 &&
                    static_cast<int>(per_sat_res.size()) >= config.cp_hold_multipath_min_satellites) {
                    std::vector<double> vals;
                    vals.reserve(per_sat_res.size());
                    for (const auto& [sid, r] : per_sat_res) {
                        (void)sid;
                        vals.push_back(r);
                    }
                    std::sort(vals.begin(), vals.end());
                    const double median = vals[vals.size() / 2];
                    const double max_v = vals.back();
                    if (median > 1e-3 && (max_v / median) > config.cp_hold_multipath_median_ratio) {
                        multipath_dominated = true;
                        ++result.diagnostics.sanity_multipath_skips;
                    }
                }
                if (!multipath_dominated) {
                    // DDPR residual evaluated at the IMU-predicted pose
                    // (pose_seed, this epoch's pre-solve seed) -- reference
                    // _compute_res_at_pred. Feeds the fast path AND the pose
                    // replacement decision below.
                    {
                        const Point3 pred_ant_p = antennaOf(pose_seed);
                        const Vector3d pred_ant(pred_ant_p.x(), pred_ant_p.y(), pred_ant_p.z());
                        double sq_sum = 0.0;
                        std::size_t n = 0;
                        for (const auto* fp : pr_by_epoch[i]) {
                            const double geom =
                                ((fp->rover_satellite_position_ecef - pred_ant).norm() -
                                 (fp->base_satellite_position_ecef - fp->base_position_ecef).norm()) -
                                ((fp->rover_reference_position_ecef - pred_ant).norm() -
                                 (fp->base_reference_position_ecef - fp->base_position_ecef).norm());
                            const double res = std::abs(fp->observed_dd_pseudorange_m - geom);
                            sq_sum += res * res;
                            ++n;
                        }
                        pred_res = n > 0 ? std::sqrt(sq_sum / static_cast<double>(n)) : 0.0;
                    }

                    // Stage: catastrophic fast path. Fires immediately
                    // (bypassing persist) when residuals are catastrophic,
                    // no fixed solution this epoch, and the worst
                    // per-satellite residual clears the fast-path floor
                    // (reference _ddpr_sanity_fast_path).
                    double worst_sat_res = 0.0;
                    for (const auto& [sid, r] : per_sat_res) {
                        (void)sid;
                        worst_sat_res = std::max(worst_sat_res, r);
                    }
                    const bool fast_eligible =
                        ddpr_rms > config.cp_hold_catastrophic_threshold_m && nb == 0 &&
                        worst_sat_res >= config.cp_hold_fast_worst_satellite_min_m;
                    if (fast_eligible) {
                        resetAmbiguitiesWithCpHold();
                        ++result.diagnostics.sanity_fast_resets;
                        did_reset = true;
                    } else {
                        // Stage: persist. Every bad epoch (re)engages
                        // CP-hold at full strength; the mass reset itself
                        // only fires once cp_hold_persist_epochs consecutive
                        // bad epochs have accumulated (reference
                        // _ddpr_sanity_persist / trigger_cp_hold).
                        ++ddpr_bad_count;
                        cp_hold_counter = std::max(cp_hold_counter, effectiveCpHoldEpochs());
                        cp_hold_release_streak = 0;
                        ++result.diagnostics.cp_hold_triggers;
                        if (ddpr_bad_count >= config.cp_hold_persist_epochs) {
                            // Stage: GDOP gate -- abort the reset (CP-hold
                            // stays engaged) when geometry is too weak to
                            // trust the residual signal (reference
                            // _ddpr_sanity_gdop_ok).
                            if (config.cp_hold_max_gdop <= 0.0 || gdop <= config.cp_hold_max_gdop) {
                                // Stage: DDPR-LS anchor fetch + anchor-vs-IMU
                                // gap (use_ddpr_anchor; reference
                                // _ddpr_sanity_fetch_anchor / _anchor_vs_imu).
                                // DIAGNOSTIC ONLY here: the reference's own
                                // control flow converges on the exact same
                                // _apply_sanity_reset() call whether the
                                // anchor is untrusted, disagrees with the IMU
                                // prediction, or agrees -- see
                                // FGOConfig::use_ddpr_anchor's comment. So
                                // this records whether the gate WOULD have
                                // skipped/allowed the reset without actually
                                // gating it, faithfully matching the
                                // reference.
                                if (config.use_ddpr_anchor) {
                                    ++result.diagnostics.ddpr_anchor_solves;
                                    const DdprAnchorResult anchor = solveDdprAnchor(i, pose_seed);
                                    const bool anchor_trusted =
                                        anchor.ok && anchor.n_active >= config.ddpr_anchor_min_factors &&
                                        anchor.res_rms <= config.ddpr_anchor_max_residual_m;
                                    if (anchor_trusted) {
                                        ++result.diagnostics.ddpr_anchor_successes;
                                        const Point3 anchor_ant = antennaOf(anchor.pose);
                                        const Point3 pred_ant = antennaOf(pose_seed);
                                        const double gap = (anchor_ant - pred_ant).norm();
                                        const bool clean_anchor =
                                            anchor.res_rms < config.ddpr_anchor_clean_residual_m &&
                                            ddpr_rms > config.ddpr_anchor_clean_main_residual_m;
                                        const bool catastrophic_res =
                                            ddpr_rms > config.cp_hold_catastrophic_threshold_m;
                                        const bool persistent_bad =
                                            ddpr_bad_count >= config.ddpr_anchor_persist_override &&
                                            anchor.res_rms < config.ddpr_anchor_clean_residual_m;
                                        const bool hard_reject =
                                            gap > config.ddpr_anchor_imu_hard_max_m && !clean_anchor;
                                        const bool soft_reject =
                                            gap > config.ddpr_anchor_imu_max_gap_m &&
                                            !catastrophic_res && !persistent_bad;
                                        if (hard_reject || soft_reject) {
                                            ++result.diagnostics.ddpr_anchor_gated_resets_skipped;
                                        } else {
                                            ++result.diagnostics.ddpr_anchor_gated_resets_allowed;
                                        }
                                    } else {
                                        ++result.diagnostics.ddpr_anchor_gated_resets_skipped;
                                    }
                                }
                                resetAmbiguitiesWithCpHold();
                                ++result.diagnostics.sanity_mass_resets;
                                did_reset = true;
                            } else {
                                ++result.diagnostics.sanity_gdop_skips;
                            }
                        }
                    }

                    // Stage: pose replacement for the REPORTED solution only
                    // (reference _sanity_report_translation /
                    // _apply_sanity_reset always reports 'FLT' after a
                    // reset). The graph values (pose_i, prev_nav, the
                    // smoother's linearization point) are untouched.
                    if (did_reset) {
                        epoch_fixed[i] = false;
                        const double thr = config.cp_hold_pose_replace_threshold_m;
                        const Point3 graph_ant_pose = antennaOf(pose_i);
                        Point3 report_ant = graph_ant_pose;
                        if (thr > 0.0 && pred_res <= thr) {
                            const Point3 pred_ant_pose = antennaOf(pose_seed);
                            const double gap = (graph_ant_pose - pred_ant_pose).norm();
                            if (gap > thr) {
                                report_ant = pred_ant_pose;
                                ++result.diagnostics.sanity_pose_replacements;
                            }
                        }
                        epoch_float_position[i] = report_ant;
                    }
                }
            } else {
                ddpr_bad_count = 0;
            }
            last_ddpr_rms = ddpr_rms;
            last_ddpr_rms_epoch = static_cast<long long>(i);
        }
    }
    result.diagnostics.partial_lambda_ambiguity_fix_used = held_epoch_count > 0;
    result.diagnostics.ambiguity_hold_epochs = held_epoch_count;
    result.diagnostics.ambiguity_hold_arcs = pinned_ambiguities.size();

    // --- Diagnostics ---
    result.diagnostics.converged = true;
    result.diagnostics.iterations = static_cast<int>(result.diagnostics.smoother_updates);
    result.diagnostics.imu_intervals = num_epochs > 0 ? num_epochs - 1 : 0;
    result.diagnostics.lambda_ambiguity_ratio = best_ratio;
    result.diagnostics.fixed_solution = total_fixed_ambiguities > 0;
    result.diagnostics.fixed_ambiguities = total_fixed_ambiguities;
    if (marginals_failures > 0) {
        std::fprintf(stderr, "[fgo_gtsam_backend] fixed-lag: %zu epoch(s) LAMBDA marginal failure\n",
                     marginals_failures);
    }

    // --- Ambiguity estimates (captured during streaming; window-marginalized
    // nodes cannot be read from the final smoother) ---
    result.ambiguity_estimates.reserve(problem.ambiguity_states.size());
    for (std::size_t idx = 0; idx < problem.ambiguity_states.size(); ++idx) {
        const auto& ambiguity = problem.ambiguity_states[idx];
        FGOProcessor::AmbiguityEstimate est;
        est.satellite = ambiguity.satellite;
        est.signal = ambiguity.signal;
        est.segment_index = ambiguity.segment_index;
        est.wavelength_m = ambiguity.wavelength_m;
        const auto fit = amb_float_cycles.find(idx);
        if (fit != amb_float_cycles.end()) {
            est.ambiguity_cycles = fit->second;
            est.ambiguity_m = fit->second * ambiguity.wavelength_m;
        }
        const auto xit = amb_fixed_cycles.find(idx);
        if (xit != amb_fixed_cycles.end()) {
            est.is_fixed = true;
            est.fixed_by_lambda = true;
            est.fixed_cycles = xit->second;
            est.fixed_ambiguity_m = xit->second * ambiguity.wavelength_m;
            est.fix_residual_cycles = amb_fixed_residual.at(idx);
        }
        result.ambiguity_estimates.push_back(est);
    }

    // --- Per-epoch solutions ---
    const bool have_amb = !problem.ambiguity_states.empty();
    result.epoch_attitude_rpy_deg.resize(num_epochs);
    result.epoch_velocity_nav_mps.resize(num_epochs);
    for (std::size_t i = 0; i < num_epochs; ++i) {
        PositionSolution solution;
        solution.time = problem.epochs[i].time;
        const bool fixed = epoch_fixed[i] && epoch_has_fixed[i];
        if (fixed) {
            solution.status = SolutionStatus::FIXED;
            solution.position_ecef = epoch_fixed_position[i];
        } else {
            solution.status = have_amb ? SolutionStatus::FLOAT : SolutionStatus::SPP;
            solution.position_ecef = epoch_float_position[i];
        }
        if (!epoch_solved[i]) {
            solution.status = SolutionStatus::NONE;
        }
        solution.num_frequencies = 1;
        solution.ratio = epoch_ratio[i];
        solution.num_fixed_ambiguities = epoch_fixed_count[i];
        double lat = 0.0, lon = 0.0, h = 0.0;
        ecef2geodetic(solution.position_ecef, lat, lon, h);
        solution.position_geodetic = GeodeticCoord(lat, lon, h);
        result.solution.addSolution(solution);
        result.epoch_attitude_rpy_deg[i] = epoch_rpy_deg[i];
        result.epoch_velocity_nav_mps[i] = epoch_vel_nav[i];
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    result.diagnostics.processing_time_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time)
            .count();
    result.diagnostics.total_processing_time_ms = result.diagnostics.processing_time_ms;
    return result;
}

FGOProcessor::FGOResult optimizeProblemWithGtsam(
    const FGOProcessor::FGOProblem& problem,
    const FGOProcessor::FGOConfig& config,
    FGOProcessor::FGOResult result) {
    // Milestone 2c: dispatch to the incremental fixed-lag smoother when
    // requested and the IMU-coupled Pose3 path is fully specified. Otherwise
    // fall through to the batch LM path below (Phase-1 / 2a / 2b), unchanged.
    if (config.use_fixed_lag_smoother && config.use_pose3_state && config.use_imu &&
        problem.imu.valid && problem.epochs.size() >= 2) {
        return optimizeProblemFixedLag(problem, config, std::move(result));
    }
    const auto start_time = std::chrono::high_resolution_clock::now();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // --- Milestone 2a/2b: Pose3 + lever-arm state (docs/gtsam_backend_design.md) ---
    // use_pose3 selects whether positionKey(epoch) holds a gtsam::Pose3 (body
    // pose, antenna offset by pose3_lever_arm_body_m) or the Phase-1
    // gtsam::Point3 (bare antenna position).
    //
    // use_imu (2b) additionally: interprets the Pose3 as body-in-nav (local
    // ENU) via a real ecef_T_nav, adds Vector3 velocity + imuBias states per
    // epoch, and links consecutive epochs with CombinedImuFactors. In 2a
    // (use_pose3 without IMU) the pose is expressed directly in ECEF (no
    // ecef_T_nav) and only the rotation gauge-pin constrains attitude.
    const std::size_t num_epochs = problem.epochs.size();
    const bool use_pose3 = config.use_pose3_state;
    const bool use_imu =
        use_pose3 && config.use_imu && problem.imu.valid && num_epochs >= 2;
    const Point3 lever_arm_body(config.pose3_lever_arm_body_m.x(),
                                config.pose3_lever_arm_body_m.y(),
                                config.pose3_lever_arm_body_m.z());

    // ecef_T_nav: identity/unused in 2a (pose in ECEF); the ENU-from-ECEF
    // transform at the nav origin in 2b (pose in nav). The DD '...FactorArm'
    // factors and the IMU factor share this same Pose3 through it.
    gtsam::gnss::LeverArm gnss_lever_arm(lever_arm_body);
    Pose3 ecef_T_nav;  // identity unless use_imu
    if (use_imu) {
        const Rot3 R_ecef_nav = ecefFromEnuRotation(problem.imu.nav_origin_lat_rad,
                                                    problem.imu.nav_origin_lon_rad);
        ecef_T_nav = Pose3(R_ecef_nav, Point3(problem.imu.nav_origin_ecef));
        gnss_lever_arm = gtsam::gnss::LeverArm(lever_arm_body, ecef_T_nav);
    }

    // Initial body->nav (ENU) attitude from Stage-1 alignment; used as the
    // dead-reckoning start point for the 2b pose attitude seeds below.
    const Rot3 init_attitude_nav(problem.imu.init_attitude_body_to_nav);

    // Antenna ECEF position for the optimized state at `epoch`, uniform across
    // all three representations (Point3, 2a Pose3-in-ECEF, 2b Pose3-in-nav):
    // gnss::LeverArm::antennaPosition applies ecef_T_nav internally when set.
    // Used everywhere downstream that needs "the position": LAMBDA covariance
    // propagation, residual RMS diagnostics, final per-epoch solution mapping.
    auto antennaPositionOf = [&](const gtsam::Values& values,
                                 std::size_t epoch) -> Point3 {
        if (use_pose3) {
            return gnss_lever_arm.antennaPosition(values.at<Pose3>(positionKey(epoch)));
        }
        return values.at<Point3>(positionKey(epoch));
    };

    // Pose seeds (Point3 and 2a Pose3-in-ECEF; the 2b IMU poses are seeded
    // below, after preintegration, so their attitudes can be dead-reckoned).
    for (std::size_t i = 0; i < num_epochs; ++i) {
        if (use_imu) {
            continue;
        }
        if (use_pose3) {
            // 2a: pose in ECEF, identity attitude (unobservable without IMU);
            // translation = antenna seed minus lever arm.
            initial.insert(positionKey(i),
                           Pose3(Rot3(), Point3(problem.epochs[i].position_ecef) - lever_arm_body));
        } else {
            initial.insert(positionKey(i), Point3(problem.epochs[i].position_ecef));
        }
    }

    if (use_imu) {
        const gtsam::imuBias::ConstantBias init_bias(problem.imu.init_accel_bias,
                                                     problem.imu.init_gyro_bias);

        // --- IMU preintegration params (ENU / Z-up; gravity along -Z) ---
        auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(
            problem.imu.noise.gravity_mps2);
        const auto sq = [](double s) { return s * s; };
        imu_params->setAccelerometerCovariance(gtsam::I_3x3 * sq(problem.imu.noise.accel_noise_sigma));
        imu_params->setGyroscopeCovariance(gtsam::I_3x3 * sq(problem.imu.noise.gyro_noise_sigma));
        imu_params->setIntegrationCovariance(gtsam::I_3x3 * sq(problem.imu.noise.integration_sigma));
        imu_params->setBiasAccCovariance(gtsam::I_3x3 * sq(problem.imu.noise.accel_bias_rw_sigma));
        imu_params->setBiasOmegaCovariance(gtsam::I_3x3 * sq(problem.imu.noise.gyro_bias_rw_sigma));

        // --- Preintegrate each [epoch i, epoch i+1) interval up front ---
        // We need the preintegrated rotations to dead-reckon attitude seeds
        // BEFORE inserting the pose values: seeding every pose at the same
        // initial attitude makes each IMU factor's rotation residual the full
        // accumulated turn between epochs, which gives a huge initial cost and
        // a divergent first LM step (observed: 1e19 cost, no convergence). By
        // forward-propagating attitude through deltaRij() the rotation residual
        // starts ~0 and the DD-accurate translation keeps position anchored, so
        // the graph is well conditioned from iteration 0.
        const auto& imu_samples = problem.imu.samples_body_flu;
        std::vector<gtsam::PreintegratedCombinedMeasurements> pims;
        pims.reserve(num_epochs > 0 ? num_epochs - 1 : 0);
        std::vector<bool> pim_valid(num_epochs > 0 ? num_epochs - 1 : 0, false);
        std::size_t sample_cursor = 0;
        std::size_t imu_intervals = 0;
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            const GNSSTime t0 = problem.epochs[i].time;
            const GNSSTime t1 = problem.epochs[i + 1].time;
            while (sample_cursor < imu_samples.size() && imu_samples[sample_cursor].time < t0) {
                ++sample_cursor;
            }
            gtsam::PreintegratedCombinedMeasurements pim(imu_params, init_bias);
            std::size_t j = sample_cursor;
            GNSSTime prev_time = t0;
            std::size_t integrated = 0;
            while (j < imu_samples.size() && imu_samples[j].time < t1) {
                const double dt = imu_samples[j].time - prev_time;
                if (dt > 1e-9) {
                    pim.integrateMeasurement(gtsam::Vector3(imu_samples[j].accel_raw),
                                             gtsam::Vector3(imu_samples[j].gyro_raw_radps), dt);
                    ++integrated;
                }
                prev_time = imu_samples[j].time;
                ++j;
            }
            const double dt_tail = t1 - prev_time;
            if (integrated > 0 && dt_tail > 1e-9 && j > 0) {
                pim.integrateMeasurement(gtsam::Vector3(imu_samples[j - 1].accel_raw),
                                         gtsam::Vector3(imu_samples[j - 1].gyro_raw_radps), dt_tail);
            }
            pims.push_back(pim);
            pim_valid[i] = integrated > 0;
            if (integrated > 0) ++imu_intervals;
        }

        // --- Dead-reckon per-epoch attitude seeds from the aligned initial
        // attitude (rotation residual ~0 at the seed). ---
        std::vector<Rot3> attitude_seed(num_epochs, init_attitude_nav);
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            attitude_seed[i + 1] =
                pim_valid[i] ? attitude_seed[i] * pims[i].deltaRij() : attitude_seed[i];
        }

        // GNSS antenna positions in nav, for translation + velocity seeds.
        std::vector<Point3> antenna_nav(num_epochs);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            antenna_nav[i] = Point3(ecef2enu(
                problem.epochs[i].position_ecef - problem.imu.nav_origin_ecef,
                problem.imu.nav_origin_lat_rad, problem.imu.nav_origin_lon_rad));
        }

        // --- Insert pose / velocity / bias seeds ---
        for (std::size_t i = 0; i < num_epochs; ++i) {
            // body-in-nav translation: antenna at the DD position, offset back
            // through the (dead-reckoned) attitude's lever arm.
            const Point3 body_nav = antenna_nav[i] - attitude_seed[i] * lever_arm_body;
            initial.insert(positionKey(i), Pose3(attitude_seed[i], body_nav));

            Vector3d vel = problem.imu.init_velocity_nav;
            if (num_epochs >= 2) {
                const std::size_t a = (i + 1 < num_epochs) ? i : i - 1;
                const double dt = problem.epochs[a + 1].time - problem.epochs[a].time;
                if (dt > 1e-3) {
                    vel = (antenna_nav[a + 1] - antenna_nav[a]) / dt;
                }
            }
            initial.insert(velocityKey(i), gtsam::Vector3(vel));
            initial.insert(biasKey(i), init_bias);
        }

        // --- First-state priors (replace the 2a per-epoch rotation pin): they
        // anchor the IMU integration chain and resolve the gauge. Attitude is
        // now observable (gravity fixes roll/pitch; motion fixes yaw), so no
        // per-pose rotation pin is needed. Position stays fully DD-driven. ---
        gtsam::Vector6 pose_prior_sigmas;
        pose_prior_sigmas << problem.imu.init_attitude_sigma_roll_pitch_rad,
            problem.imu.init_attitude_sigma_roll_pitch_rad,
            problem.imu.init_attitude_sigma_yaw_rad,
            1e6, 1e6, 1e6;  // translation left free (DD constrains it)
        graph.addPrior(positionKey(0), initial.at<Pose3>(positionKey(0)),
                       gtsam::noiseModel::Diagonal::Sigmas(pose_prior_sigmas));
        graph.addPrior(velocityKey(0), gtsam::Vector3(problem.imu.init_velocity_nav),
                       gtsam::noiseModel::Isotropic::Sigma(3, problem.imu.init_velocity_sigma_mps));
        gtsam::Vector6 bias_prior_sigmas;
        bias_prior_sigmas << problem.imu.init_accel_bias_sigma, problem.imu.init_accel_bias_sigma,
            problem.imu.init_accel_bias_sigma, problem.imu.init_gyro_bias_sigma,
            problem.imu.init_gyro_bias_sigma, problem.imu.init_gyro_bias_sigma;
        graph.addPrior(biasKey(0), init_bias,
                       gtsam::noiseModel::Diagonal::Sigmas(bias_prior_sigmas));

        // --- Add the CombinedImuFactors (or a loose velocity/bias continuity
        // across an IMU dropout) ---
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            if (pim_valid[i]) {
                graph.emplace_shared<gtsam::CombinedImuFactor>(
                    positionKey(i), velocityKey(i), positionKey(i + 1), velocityKey(i + 1),
                    biasKey(i), biasKey(i + 1), pims[i]);
            } else {
                graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
                    velocityKey(i), velocityKey(i + 1), gtsam::Vector3::Zero(),
                    gtsam::noiseModel::Isotropic::Sigma(3, 10.0));
                graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
                    biasKey(i), biasKey(i + 1), gtsam::imuBias::ConstantBias(),
                    gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
            }
        }
        result.diagnostics.imu_intervals = imu_intervals;
    } else if (use_pose3) {
        // 2a rotation-only gauge pin (no IMU): resolves the exact rotational
        // null space left by lever-arm-only position observations, without
        // perturbing the estimated antenna position beyond kRotationPinSigmaRad
        // * |lever_arm| (~0.06 mm at 1e-4 rad and the tokyo ~0.62 m lever arm).
        constexpr double kRotationPinSigmaRad = 1e-4;
        const auto rotation_noise = gtsam::noiseModel::Isotropic::Sigma(3, kRotationPinSigmaRad);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            graph.emplace_shared<Pose3RotationPrior>(positionKey(i), Rot3(), rotation_noise);
        }
    }

    // Undifferenced factors need a per-epoch base receiver clock [s] plus, for
    // every non-GPS constellation, ONE global (time-constant) inter-system bias
    // node shared across all epochs -- matching the native backend's per-epoch
    // clock + per-constellation bias columns.
    const bool need_clock_states =
        !problem.pseudorange_factors.empty() || !problem.carrier_phase_factors.empty();
    std::set<gtsam::Key> inserted_clock_keys;
    std::set<int> inserted_isb_ordinals;
    auto ensureBaseClock = [&](std::size_t epoch) -> gtsam::Key {
        const gtsam::Key key = clockKey(epoch);
        if (inserted_clock_keys.insert(key).second) {
            initial.insert(key, epoch < num_epochs
                                    ? problem.epochs[epoch].receiver_clock_bias_m /
                                          constants::SPEED_OF_LIGHT
                                    : 0.0);
        }
        return key;
    };
    // Returns the ISB ordinal for a system: 0 means GPS/base (no ISB node),
    // >0 means a global ISB node was ensured for that constellation.
    auto ensureIsb = [&](GNSSSystem system) -> int {
        const int ordinal =
            clockGroupOrdinal(clockBiasGroup(system), config.use_inter_system_biases);
        if (ordinal != 0 && inserted_isb_ordinals.insert(ordinal).second) {
            initial.insert(isbKey(ordinal), 0.0);
        }
        return ordinal;
    };

    // Ambiguity nodes: one per AmbiguityState, in the units that state's
    // consuming factor expects (cycles for DD, meters for undifferenced).
    for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
        const auto& ambiguity = problem.ambiguity_states[i];
        const double value =
            ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0
                ? ambiguity.initial_ambiguity_m / ambiguity.wavelength_m
                : ambiguity.initial_ambiguity_m;
        initial.insert(ambiguityKey(i), value);
    }
    const bool need_dummy_ambiguity =
        std::any_of(problem.ambiguity_states.begin(), problem.ambiguity_states.end(),
                    [](const FGOProcessor::AmbiguityState& a) { return a.is_double_difference; });
    if (need_dummy_ambiguity) {
        initial.insert(dummyAmbiguityKey(), 0.0);
        // Pin the shared dummy ambiguity at 0 so the lumped libgnss DD ambiguity
        // lives entirely in ambRef (the estimate is then simply ambRef). The
        // dummy is shared across every DD carrier factor, which introduces a
        // common-mode null space (shift dummy + every ambRef by the same delta
        // leaves all carrier residuals unchanged); only this prior constrains
        // it. A near-zero sigma (e.g. 1e-6) pins it but makes the prior's
        // Hessian block ~1e12 against position blocks ~1e2, i.e. condition
        // number ~1e10 -> Cholesky returns a garbage step, every LM trial
        // diverges to inf error, and LM gives up at iteration 0. 1e-3 cycles
        // (~0.2 mm of range) is still "fixed" physically but keeps the system
        // well conditioned (~1e4).
        constexpr double kDummyAmbiguityPinSigmaCycles = 1e-3;
        graph.addPrior(dummyAmbiguityKey(), 0.0,
                       gtsam::noiseModel::Isotropic::Sigma(1, kDummyAmbiguityPinSigmaCycles));
    }

    // --- Undifferenced pseudorange / carrier-phase factors ---
    for (const auto& factor : problem.pseudorange_factors) {
        const gtsam::Key base_clock = ensureBaseClock(factor.epoch_index);
        const int ordinal = ensureIsb(factor.satellite.system);
        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.pseudorange_huber_threshold_sigma);
        if (use_pose3) {
            if (ordinal == 0) {
                graph.emplace_shared<PseudorangeFactorPlainArm>(
                    positionKey(factor.epoch_index), base_clock,
                    factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                    gnss_lever_arm, noise);
            } else {
                graph.emplace_shared<PseudorangeFactorISBArm>(
                    positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                    factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                    gnss_lever_arm, noise);
            }
        } else if (ordinal == 0) {
            graph.emplace_shared<PseudorangeFactorPlain>(
                positionKey(factor.epoch_index), base_clock,
                factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                noise);
        } else {
            graph.emplace_shared<PseudorangeFactorISB>(
                positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                noise);
        }
    }
    // Undifferenced carrier phase (rare here: use_carrier_phase_factors is off
    // in the DD RTK config) uses only the base clock; the ISB affects code and
    // phase identically and cancels in the DD path, so this is adequate for the
    // SPP-seed carrier path and can gain its own ISB term if PPP-RTK needs it.
    // Not yet ported to Pose3 (milestone 2a targets the pure-DD RTK path,
    // where use_carrier_phase_factors is off); skip with a warning rather
    // than silently mixing a Point3 factor into a Pose3 graph.
    for (const auto& factor : problem.carrier_phase_factors) {
        if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        if (use_pose3) {
            std::fprintf(stderr,
                         "[fgo_gtsam_backend] WARNING: undifferenced carrier_phase_factors are "
                         "not yet supported with use_pose3_state; skipping %zu factor(s).\n",
                         problem.carrier_phase_factors.size());
            break;
        }
        const gtsam::Key base_clock = ensureBaseClock(factor.epoch_index);
        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.carrier_phase_huber_threshold_sigma);
        graph.emplace_shared<gtsam::CarrierPhaseFactor>(
            positionKey(factor.epoch_index), base_clock,
            ambiguityKey(factor.ambiguity_index), factor.corrected_carrier_m,
            Point3(factor.satellite_position_ecef), 0.0, noise);
    }

    // --- Double-difference pseudorange factors ---
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        const gtsam::gnss::DoubleDifferenceData dd{
            factor.rover_satellite_model.corrected_pseudorange_m,
            factor.base_satellite_model.corrected_pseudorange_m,
            factor.rover_reference_model.corrected_pseudorange_m,
            factor.base_reference_model.corrected_pseudorange_m,
            Point3(factor.rover_satellite_position_ecef),
            Point3(factor.rover_reference_position_ecef),
            Point3(factor.base_satellite_position_ecef),
            Point3(factor.base_reference_position_ecef),
            Point3(factor.base_position_ecef),
        };
        checkObservedDdMatches(dd.observed(), factor.observed_dd_pseudorange_m,
                              "DD pseudorange");

        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.pseudorange_huber_threshold_sigma);
        if (use_imu) {
            // 2b: pose is body-in-nav (ENU) -> feed ecef_T_nav so the factor
            // reconstructs the antenna ECEF from the nav-frame pose.
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, lever_arm_body, ecef_T_nav, noise);
        } else if (use_pose3) {
            // 2a: pose expressed directly in ECEF (no ecef_T_nav).
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, lever_arm_body, noise);
        } else {
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactor>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, noise);
        }
    }

    // --- Double-difference carrier-phase factors ---
    for (const auto& factor : problem.double_difference_carrier_factors) {
        if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto& ambiguity = problem.ambiguity_states[factor.ambiguity_index];
        const gtsam::gnss::DoubleDifferenceData dd{
            factor.rover_satellite_model.corrected_carrier_m,
            factor.base_satellite_model.corrected_carrier_m,
            factor.rover_reference_model.corrected_carrier_m,
            factor.base_reference_model.corrected_carrier_m,
            Point3(factor.rover_satellite_position_ecef),
            Point3(factor.rover_reference_position_ecef),
            Point3(factor.base_satellite_position_ecef),
            Point3(factor.base_reference_position_ecef),
            Point3(factor.base_position_ecef),
        };
        checkObservedDdMatches(dd.observed(), factor.observed_dd_carrier_m, "DD carrier");

        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.carrier_phase_huber_threshold_sigma);
        if (use_imu) {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, lever_arm_body, ecef_T_nav, noise);
        } else if (use_pose3) {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, lever_arm_body, noise);
        } else {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactor>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, noise);
        }
    }

    // --- Ambiguity priors ---
    if (config.use_ambiguity_priors && config.ambiguity_prior_sigma_m > 0.0) {
        for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
            const auto& ambiguity = problem.ambiguity_states[i];
            if (ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0) {
                const double mean = ambiguity.initial_ambiguity_m / ambiguity.wavelength_m;
                const double sigma = config.ambiguity_prior_sigma_m / ambiguity.wavelength_m;
                graph.addPrior(ambiguityKey(i), mean, gtsam::noiseModel::Isotropic::Sigma(1, sigma));
            } else {
                graph.addPrior(ambiguityKey(i), ambiguity.initial_ambiguity_m,
                               gtsam::noiseModel::Isotropic::Sigma(1, config.ambiguity_prior_sigma_m));
            }
        }
    }

    // --- Ambiguity continuity (BetweenFactor<double>) ---
    for (const auto& factor : problem.ambiguity_between_factors) {
        if (factor.previous_ambiguity_index >= problem.ambiguity_states.size() ||
            factor.current_ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto& ambiguity = problem.ambiguity_states[factor.current_ambiguity_index];
        const double wavelength = ambiguity.wavelength_m;
        const double sigma_cycles = wavelength > 0.0 ? factor.sigma_m / wavelength : factor.sigma_m;
        graph.emplace_shared<gtsam::BetweenFactor<double>>(
            ambiguityKey(factor.previous_ambiguity_index),
            ambiguityKey(factor.current_ambiguity_index), 0.0,
            gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-9, sigma_cycles)));
    }

    // --- Position motion (random-walk) factors between consecutive epochs ---
    if (config.use_motion_factors && config.use_position_motion_factors &&
        config.motion_sigma_m > 0.0) {
        if (use_pose3) {
            // Pose3 analog of the Point3 zero-motion BetweenFactor below: same
            // loose translation sigma (motion_sigma_m, e.g. 100 m -- a
            // near-no-op regularizer at DD noise levels), and an even looser
            // rotation sigma so it never competes with the dedicated
            // Pose3RotationPrior gauge pin above.
            constexpr double kLooseRotationSigmaRad = 10.0;
            gtsam::Vector6 sigmas;
            sigmas << kLooseRotationSigmaRad, kLooseRotationSigmaRad, kLooseRotationSigmaRad,
                config.motion_sigma_m, config.motion_sigma_m, config.motion_sigma_m;
            const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
            for (std::size_t i = 1; i < num_epochs; ++i) {
                graph.emplace_shared<gtsam::BetweenFactor<Pose3>>(
                    positionKey(i - 1), positionKey(i), Pose3(Rot3(), Point3(0.0, 0.0, 0.0)),
                    noise);
            }
        } else {
            const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, config.motion_sigma_m);
            for (std::size_t i = 1; i < num_epochs; ++i) {
                graph.emplace_shared<gtsam::BetweenFactor<Point3>>(
                    positionKey(i - 1), positionKey(i), Point3(0.0, 0.0, 0.0), noise);
            }
        }
    }

    // --- Optional absolute priors (disabled by default: sigma <= 0) ---
    if (config.position_prior_sigma_m > 0.0) {
        if (use_pose3) {
            constexpr double kLooseRotationSigmaRad = 1e6;
            gtsam::Vector6 sigmas;
            sigmas << kLooseRotationSigmaRad, kLooseRotationSigmaRad, kLooseRotationSigmaRad,
                config.position_prior_sigma_m, config.position_prior_sigma_m,
                config.position_prior_sigma_m;
            const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
            for (std::size_t i = 0; i < num_epochs; ++i) {
                graph.addPrior(positionKey(i),
                               Pose3(Rot3(), Point3(problem.epochs[i].position_ecef) - lever_arm_body),
                               noise);
            }
        } else {
            const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, config.position_prior_sigma_m);
            for (std::size_t i = 0; i < num_epochs; ++i) {
                graph.addPrior(positionKey(i), Point3(problem.epochs[i].position_ecef), noise);
            }
        }
    }
    if (need_clock_states && config.clock_prior_sigma_m > 0.0) {
        const auto noise = gtsam::noiseModel::Isotropic::Sigma(
            1, config.clock_prior_sigma_m / constants::SPEED_OF_LIGHT);
        for (const gtsam::Key key : inserted_clock_keys) {
            graph.addPrior(key, initial.at<double>(key), noise);
        }
    }

    result.diagnostics.graph_factors = graph.size();
    result.diagnostics.graph_values = initial.size();

    // --- Solve ---
    gtsam::LevenbergMarquardtParams params;
    if (std::getenv("GNSSPP_GTSAM_LM_VERBOSE") != nullptr) {
        params.setVerbosityLM("SUMMARY");
    }
    params.setMaxIterations(std::max(1, config.max_iterations));
    params.setAbsoluteErrorTol(config.absolute_cost_convergence_threshold > 0.0
                                   ? config.absolute_cost_convergence_threshold
                                   : 1e-10);
    params.setRelativeErrorTol(config.relative_cost_convergence_threshold > 0.0
                                   ? config.relative_cost_convergence_threshold
                                   : 1e-8);

    const double initial_cost = graph.error(initial);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    gtsam::Values optimized;
    bool solved = true;
    try {
        optimized = optimizer.optimize();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[fgo_gtsam_backend] LM optimize() threw: %s\n", e.what());
        solved = false;
        optimized = initial;
    }
    const double final_cost = graph.error(optimized);

    result.diagnostics.iterations = static_cast<int>(optimizer.iterations());
    result.diagnostics.converged = solved;
    result.diagnostics.initial_cost = initial_cost;
    result.diagnostics.final_cost = final_cost;

    // --- Per-epoch / partial LAMBDA integer fixing ---
    // Mirrors the native backend's per-epoch LAMBDA path (fgo.cpp
    // process_epoch_lambda_work): group each epoch's DD ambiguities, take their
    // joint marginal covariance (here via gtsam::Marginals over the whole
    // graph), run the SAME libgnss::lambdaSearch with the same ratio test, and
    // -- when use_epoch_lambda_fixed_output is set -- snap that epoch's position
    // to the fixed solution via the position/ambiguity cross-covariance. GTSAM
    // ambiguity nodes are already in cycles, so no wavelength scaling is needed.
    std::map<std::size_t, int> fixed_cycles_by_index;      // amb index -> integer cycles
    std::map<std::size_t, double> fixed_residual_by_index;  // amb index -> float-int residual
    std::vector<bool> epoch_fixed(num_epochs, false);
    std::vector<int> epoch_fixed_count(num_epochs, 0);
    std::vector<double> epoch_ratio(num_epochs, 0.0);
    std::vector<Point3> epoch_output_position(num_epochs, Point3(0, 0, 0));
    for (std::size_t i = 0; i < num_epochs; ++i) {
        epoch_output_position[i] = antennaPositionOf(optimized, i);
    }
    std::size_t total_fixed_ambiguities = 0;
    std::size_t covariance_failures = 0;
    double best_ratio = 0.0;

    const bool do_fix =
        solved && config.use_lambda_ambiguity_fix &&
        !problem.double_difference_carrier_factors.empty();
    if (do_fix) {
        // DD ambiguity indices present at each epoch.
        std::map<std::size_t, std::set<std::size_t>> ambiguity_indices_by_epoch;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.epoch_index < num_epochs &&
                factor.ambiguity_index < problem.ambiguity_states.size()) {
                ambiguity_indices_by_epoch[factor.epoch_index].insert(factor.ambiguity_index);
            }
        }
        // Same minimum-candidate gate as the native per-epoch path.
        const int min_candidates =
            std::max(6, std::max(1, config.min_fixed_ambiguities + 1));

        gtsam::Marginals marginals;
        bool have_marginals = false;
        try {
            marginals = gtsam::Marginals(graph, optimized, gtsam::Marginals::CHOLESKY);
            have_marginals = true;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[fgo_gtsam_backend] Marginals failed, no fixing: %s\n",
                         e.what());
        }

        for (auto it = ambiguity_indices_by_epoch.begin();
             have_marginals && it != ambiguity_indices_by_epoch.end(); ++it) {
            const std::size_t epoch_index = it->first;
            if (static_cast<int>(it->second.size()) < min_candidates) {
                continue;
            }
            // Deterministic candidate order (satellite/reference), matching the
            // native path's sort.
            std::vector<std::size_t> candidates(it->second.begin(), it->second.end());
            // MF-AR step 1: one band per satellite for the integer ratio test.
            if (config.double_difference_lambda_one_band_per_satellite &&
                candidates.size() > 1) {
                candidates = selectOneBandPerSatellite(candidates, problem);
            }
            std::sort(candidates.begin(), candidates.end(),
                      [&](std::size_t a, std::size_t b) {
                          const auto& sa = problem.ambiguity_states[a];
                          const auto& sb = problem.ambiguity_states[b];
                          return std::tie(sa.satellite, sa.reference_satellite, sa.signal, a) <
                                 std::tie(sb.satellite, sb.reference_satellite, sb.signal, b);
                      });
            const int n = static_cast<int>(candidates.size());

            gtsam::KeyVector keys;
            keys.reserve(candidates.size() + 1);
            keys.push_back(positionKey(epoch_index));
            for (std::size_t idx : candidates) {
                keys.push_back(ambiguityKey(idx));
            }

            Eigen::VectorXd float_amb(n);
            Eigen::MatrixXd q_amb(n, n);
            Eigen::MatrixXd pos_amb(3, n);  // Cov(antenna_position, ambiguity_row)
            bool ok = true;
            try {
                const gtsam::JointMarginal joint = marginals.jointMarginalCovariance(keys);
                const gtsam::Key pos_key = positionKey(epoch_index);
                // Pose3 mode: the joint marginal gives Cov(pose_tangent, amb),
                // a 6x1 block; propagate it through the antenna Jacobian
                // (3x6, constant for this epoch/optimized-value) to get
                // Cov(antenna_position, amb), matching the Point3 case's
                // native 3x1 block below.
                gtsam::Matrix36 H_antenna_pose;
                if (use_pose3) {
                    gtsam::gnss::LeverArm::PoseFrame frame;
                    gnss_lever_arm.antennaPosition(optimized.at<Pose3>(pos_key), &frame);
                    for (int r3 = 0; r3 < 3; ++r3) {
                        gtsam::Matrix13 unit = gtsam::Matrix13::Zero();
                        unit(0, r3) = 1.0;
                        H_antenna_pose.row(r3) = gnss_lever_arm.antennaPoseJacobian(unit, frame);
                    }
                }
                for (int r = 0; r < n && ok; ++r) {
                    const gtsam::Key rk = ambiguityKey(candidates[r]);
                    float_amb(r) = optimized.at<double>(rk);
                    const gtsam::Matrix pr = joint(pos_key, rk);  // 6x1 (Pose3) or 3x1 (Point3)
                    const Eigen::Vector3d pr3 = use_pose3 ? Eigen::Vector3d(H_antenna_pose * pr)
                                                          : Eigen::Vector3d(pr.col(0));
                    pos_amb(0, r) = pr3(0);
                    pos_amb(1, r) = pr3(1);
                    pos_amb(2, r) = pr3(2);
                    for (int c = 0; c < n; ++c) {
                        q_amb(r, c) = joint(rk, ambiguityKey(candidates[c]))(0, 0);
                    }
                }
            } catch (const std::exception&) {
                ok = false;
            }
            if (!ok || !float_amb.allFinite() || !q_amb.allFinite()) {
                ++covariance_failures;
                continue;
            }

            q_amb = 0.5 * (q_amb + q_amb.transpose());
            for (int i = 0; i < n; ++i) {
                q_amb(i, i) += std::max(1e-12, std::abs(q_amb(i, i)) * 1e-9);
            }

            Eigen::VectorXd fixed_amb;
            double ratio = 0.0;
            ++result.diagnostics.lambda_ambiguity_attempts;
            result.diagnostics.lambda_ambiguity_candidates += static_cast<std::size_t>(n);
            const bool solved_lambda = lambdaSearch(float_amb, q_amb, fixed_amb, ratio);
            if (!solved_lambda) {
                continue;
            }
            result.diagnostics.lambda_ambiguity_fix_solved = true;
            best_ratio = std::max(best_ratio, ratio);
            epoch_ratio[epoch_index] = ratio;
            const bool fixed_epoch =
                std::isfinite(ratio) &&
                (config.lambda_ratio_threshold <= 0.0 || ratio > config.lambda_ratio_threshold) &&
                fixed_amb.size() == n;
            if (!fixed_epoch) {
                continue;
            }

            epoch_fixed[epoch_index] = true;
            epoch_fixed_count[epoch_index] = n;
            total_fixed_ambiguities += static_cast<std::size_t>(n);
            result.diagnostics.lambda_ambiguity_fix_used = true;
            result.diagnostics.lambda_ambiguity_used_candidates += static_cast<std::size_t>(n);
            for (int r = 0; r < n; ++r) {
                const int fixed_int = static_cast<int>(std::lround(fixed_amb(r)));
                fixed_cycles_by_index[candidates[r]] = fixed_int;
                fixed_residual_by_index[candidates[r]] = float_amb(r) - static_cast<double>(fixed_int);
            }

            // Position snap to the fixed ambiguities (only when requested):
            //   x_fixed = x_float - Cov(x,N) Cov(N,N)^-1 (N_float - N_fixed)
            if (config.use_epoch_lambda_fixed_output) {
                const Eigen::VectorXd delta = float_amb - fixed_amb;
                const Eigen::LDLT<Eigen::MatrixXd> ldlt(q_amb);
                if (ldlt.info() == Eigen::Success) {
                    const Eigen::VectorXd correction = ldlt.solve(delta);
                    if (correction.allFinite()) {
                        const Eigen::Vector3d pos_delta = pos_amb * correction;
                        if (pos_delta.allFinite()) {
                            epoch_output_position[epoch_index] =
                                antennaPositionOf(optimized, epoch_index) - Point3(pos_delta);
                        }
                    }
                }
            }
        }
    }

    const bool any_fixed = total_fixed_ambiguities > 0;
    result.diagnostics.lambda_ambiguity_ratio = best_ratio;
    result.diagnostics.fixed_solution = any_fixed;
    result.diagnostics.fixed_ambiguities = total_fixed_ambiguities;
    if (covariance_failures > 0) {
        std::fprintf(stderr,
                     "[fgo_gtsam_backend] %zu epoch(s) skipped: non-PSD / marginal failure\n",
                     covariance_failures);
    }

    // --- Map ambiguity estimates ---
    result.ambiguity_estimates.reserve(problem.ambiguity_states.size());
    for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
        const auto& ambiguity = problem.ambiguity_states[i];
        FGOProcessor::AmbiguityEstimate estimate;
        estimate.satellite = ambiguity.satellite;
        estimate.signal = ambiguity.signal;
        estimate.segment_index = ambiguity.segment_index;
        estimate.wavelength_m = ambiguity.wavelength_m;
        const double value = optimized.at<double>(ambiguityKey(i));
        if (ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0) {
            estimate.ambiguity_cycles = value;
            estimate.ambiguity_m = value * ambiguity.wavelength_m;
        } else {
            estimate.ambiguity_m = value;
            estimate.ambiguity_cycles =
                ambiguity.wavelength_m > 0.0 ? value / ambiguity.wavelength_m : 0.0;
        }
        const auto fixed_it = fixed_cycles_by_index.find(i);
        if (fixed_it != fixed_cycles_by_index.end()) {
            estimate.is_fixed = true;
            estimate.fixed_by_lambda = true;
            estimate.fixed_cycles = fixed_it->second;
            estimate.fixed_ambiguity_m = fixed_it->second * ambiguity.wavelength_m;
            estimate.fix_residual_cycles = fixed_residual_by_index.at(i);
        }
        result.ambiguity_estimates.push_back(estimate);
    }

    // --- Map per-epoch solutions ---
    const bool have_ambiguities = !problem.ambiguity_states.empty();
    for (std::size_t i = 0; i < num_epochs; ++i) {
        PositionSolution solution;
        solution.time = problem.epochs[i].time;
        if (epoch_fixed[i]) {
            solution.status = SolutionStatus::FIXED;
        } else if (have_ambiguities) {
            solution.status = SolutionStatus::FLOAT;
        } else {
            solution.status = SolutionStatus::SPP;
        }
        // Fixed position snap only takes effect when use_epoch_lambda_fixed_output
        // is set; otherwise epoch_output_position holds the float position.
        solution.position_ecef = epoch_output_position[i];
        if (need_clock_states && optimized.exists(clockKey(i))) {
            solution.receiver_clock_bias = optimized.at<double>(clockKey(i));
        }
        solution.num_frequencies = 1;
        solution.ratio = epoch_ratio[i];
        solution.num_fixed_ambiguities = epoch_fixed_count[i];
        solution.iterations = result.diagnostics.iterations;

        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        ecef2geodetic(solution.position_ecef, lat, lon, height);
        solution.position_geodetic = GeodeticCoord(lat, lon, height);

        result.solution.addSolution(solution);
    }

    // --- Milestone 2b: export estimated attitude (roll/pitch/heading, deg)
    // and ENU velocity so callers can confirm attitude is now observable. ---
    if (use_imu) {
        result.epoch_attitude_rpy_deg.resize(num_epochs);
        result.epoch_velocity_nav_mps.resize(num_epochs);
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const Rot3 R_body_to_nav = optimized.at<Pose3>(positionKey(i)).rotation();
            const gtsam::Matrix3 R = R_body_to_nav.matrix();
            const Eigen::Vector3d fwd = R.col(0);   // body forward in ENU
            const Eigen::Vector3d left = R.col(1);  // body left in ENU
            // heading: clockwise from North (atan2(E, N)); pitch: nose-up from
            // the forward axis Up component; roll: from the left axis Up
            // component. Matches the reference.csv Roll/Pitch/Heading convention.
            double heading = std::atan2(fwd.x(), fwd.y()) * kRadToDeg;
            if (heading < 0.0) heading += 360.0;
            const double pitch =
                std::asin(std::max(-1.0, std::min(1.0, fwd.z()))) * kRadToDeg;
            const double roll = std::atan2(left.z(), R.col(2).z()) * kRadToDeg;
            result.epoch_attitude_rpy_deg[i] = Vector3d(roll, pitch, heading);
            result.epoch_velocity_nav_mps[i] =
                Vector3d(optimized.at<gtsam::Vector3>(velocityKey(i)));
        }
    }

    // --- Residual RMS diagnostics (recomputed from the optimized state so
    // they are directly comparable with the native backend's fields) ---
    auto accumulate_rms = [](double sum, std::size_t count) {
        return count > 0 ? std::sqrt(sum / static_cast<double>(count)) : 0.0;
    };

    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.double_difference_pseudorange_factors) {
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const gtsam::gnss::DoubleDifferenceData dd{
                factor.rover_satellite_model.corrected_pseudorange_m,
                factor.base_satellite_model.corrected_pseudorange_m,
                factor.rover_reference_model.corrected_pseudorange_m,
                factor.base_reference_model.corrected_pseudorange_m,
                Point3(factor.rover_satellite_position_ecef),
                Point3(factor.rover_reference_position_ecef),
                Point3(factor.base_satellite_position_ecef),
                Point3(factor.base_reference_position_ecef),
                Point3(factor.base_position_ecef),
            };
            const double residual = dd.observed() - dd.model(position);
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.double_difference_pseudorange_residual_rms_m =
            accumulate_rms(sum, count);
    }
    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
                continue;
            }
            const auto& ambiguity = problem.ambiguity_states[factor.ambiguity_index];
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const double amb_cycles = optimized.at<double>(ambiguityKey(factor.ambiguity_index));
            const gtsam::gnss::DoubleDifferenceData dd{
                factor.rover_satellite_model.corrected_carrier_m,
                factor.base_satellite_model.corrected_carrier_m,
                factor.rover_reference_model.corrected_carrier_m,
                factor.base_reference_model.corrected_carrier_m,
                Point3(factor.rover_satellite_position_ecef),
                Point3(factor.rover_reference_position_ecef),
                Point3(factor.base_satellite_position_ecef),
                Point3(factor.base_reference_position_ecef),
                Point3(factor.base_position_ecef),
            };
            const double residual =
                dd.observed() - (dd.model(position) + ambiguity.wavelength_m * amb_cycles);
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.double_difference_carrier_residual_rms_m = accumulate_rms(sum, count);
    }
    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.pseudorange_factors) {
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const int ordinal = clockGroupOrdinal(
                clockBiasGroup(factor.satellite.system), config.use_inter_system_biases);
            double clock_s = optimized.at<double>(clockKey(factor.epoch_index));
            if (ordinal != 0 && optimized.exists(isbKey(ordinal))) {
                clock_s += optimized.at<double>(isbKey(ordinal));
            }
            // Plain Euclidean range to match the native backend and the factor
            // model above (satellite positions are already earth-rotation
            // corrected; no additional Sagnac term).
            const double range =
                (Point3(factor.satellite_position_ecef) - position).norm();
            const double predicted = range + constants::SPEED_OF_LIGHT * clock_s;
            const double residual = factor.corrected_pseudorange_m - predicted;
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.residual_rms_m = accumulate_rms(sum, count);
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    result.diagnostics.processing_time_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time -
                                                                               start_time)
            .count();
    result.diagnostics.total_processing_time_ms = result.diagnostics.processing_time_ms;

    return result;
}

}  // namespace libgnss
