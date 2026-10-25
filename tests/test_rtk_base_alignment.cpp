#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_base_alignment.hpp>
#include <libgnss++/core/constants.hpp>

#include "../apps/native/rtk_base_epoch_align.hpp"

#include <cmath>
#include <limits>
#include <map>

using namespace libgnss;
namespace align = libgnss::rtk_base_alignment;

namespace {
const Vector3d kBase(-3959000.0, 3352000.0, 3697000.0);  // mid-latitude, static
const GNSSTime kBaseTime(2300, 100000.0);
constexpr double kCodeResidual = 1234.5;    // base clock + atmosphere, held
constexpr double kPhaseResidual = 987.654;  // phase ambiguity + clock, held
constexpr double kLambda = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;

Ephemeris makeEphemeris(uint8_t prn) {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GPS, prn);
    eph.valid = true;
    eph.week = 2300;
    eph.toe = GNSSTime(eph.week, 100000.0);
    eph.toc = eph.toe;
    eph.tof = eph.toe;
    eph.toes = eph.toe.tow;
    eph.sqrt_a = std::sqrt(26560000.0);
    eph.e = 0.004 + 0.0002 * prn;
    eph.i0 = 0.94 + 0.01 * (prn % 3);
    eph.omega0 = 0.35 * prn;
    eph.omega = 0.17 * prn;
    eph.m0 = 0.61 * prn;
    eph.delta_n = 1e-9 * prn;
    eph.omega_dot = -8.0e-9;
    eph.health = 0;
    return eph;
}

NavigationData makeNavigation() {
    NavigationData nav;
    for (uint8_t prn = 1; prn <= 32; ++prn) nav.addEphemeris(makeEphemeris(prn));
    return nav;
}

// The same orbit re-referenced to toe + dt with an along-track offset of about
// 0.5 m, standing in for the next broadcast set: nearest-toe selection prefers
// it from toe + dt / 2 on.
Ephemeris nextEphemeris(const Ephemeris& eph, double dt) {
    Ephemeris next = eph;
    next.toe = eph.toe + dt;
    next.toc = next.toe;
    next.tof = next.toe;
    next.toes = next.toe.tow;
    const double a = eph.sqrt_a * eph.sqrt_a;
    const double n = std::sqrt(3.986005e14 / (a * a * a)) + eph.delta_n;
    next.m0 = eph.m0 + n * dt + 2e-8;
    next.omega0 = eph.omega0 + eph.omega_dot * dt;
    return next;
}

// A base epoch built from the shared model (fixed travel time so it does not
// depend on the held pseudorange): only satellites visible at both times are kept.
ObservationData generate(const NavigationData& nav, const GNSSTime& time,
                         const GNSSTime& other_time, std::size_t* count = nullptr) {
    ObservationData epoch(time);
    for (uint8_t prn = 1; prn <= 32; ++prn) {
        const SatelliteId sat(GNSSSystem::GPS, prn);
        double here = 0.0, there = 0.0;
        if (!align::calculateModeledBaseRange(sat, time, 0.0, kBase, nav, here) ||
            !align::calculateModeledBaseRange(sat, other_time, 0.0, kBase, nav, there))
            continue;
        Observation obs(sat, SignalType::GPS_L1CA);
        obs.pseudorange = here + kCodeResidual + 0.1 * prn;
        obs.has_pseudorange = true;
        obs.carrier_phase = (here + kPhaseResidual + 0.01 * prn) / kLambda;
        obs.has_carrier_phase = true;
        obs.doppler = -1000.0 + 10.0 * prn;
        obs.has_doppler = true;
        obs.snr = 40.0 + prn;
        obs.code = 3;
        epoch.addObservation(obs);
    }
    if (count) *count = epoch.observations.size();
    return epoch;
}
}  // namespace

TEST(RtkBaseAlignmentTest, HoldReproducesObservationsGeneratedAtTargetTime) {
    const NavigationData nav = makeNavigation();
    for (const double age : {0.2, 0.6, 1.0, 1.8}) {
        const GNSSTime target = kBaseTime + age;
        std::size_t n = 0;
        const ObservationData base = generate(nav, kBaseTime, target, &n);
        ASSERT_GE(n, 4U);
        const ObservationData truth = generate(nav, target, kBaseTime);
        ObservationData held;
        ASSERT_TRUE(align::holdBaseEpoch(base, target, kBase, nav, 2.0, held));
        EXPECT_NEAR(held.time - target, 0.0, 1e-9);
        ASSERT_EQ(held.observations.size(), truth.observations.size());
        double worst_code = 0.0, worst_phase = 0.0;
        for (std::size_t i = 0; i < held.observations.size(); ++i) {
            const auto& h = held.observations[i];
            const auto& t = truth.observations[i];
            ASSERT_EQ(h.satellite.prn, t.satellite.prn);
            ASSERT_TRUE(h.has_pseudorange);
            ASSERT_TRUE(h.has_carrier_phase);
            worst_code = std::max(worst_code, std::abs(h.pseudorange - t.pseudorange));
            worst_phase = std::max(worst_phase, std::abs((h.carrier_phase - t.carrier_phase) * kLambda));
            EXPECT_DOUBLE_EQ(h.doppler, base.observations[i].doppler);
            EXPECT_DOUBLE_EQ(h.snr, base.observations[i].snr);
            EXPECT_EQ(h.code, base.observations[i].code);
            EXPECT_EQ(h.lli, base.observations[i].lli);
        }
        // Static base, constant residual: only the broadcast-model travel-time
        // approximation differs (sub-millimetre).
        EXPECT_LT(worst_code, 2e-3) << "age " << age;
        EXPECT_LT(worst_phase, 2e-3) << "age " << age;
        // The unhold value differs from the raw epoch by the satellite motion,
        // so the correction is not a no-op.
        double motion = 0.0;
        for (std::size_t i = 0; i < held.observations.size(); ++i)
            motion = std::max(motion,
                std::abs(held.observations[i].pseudorange - base.observations[i].pseudorange));
        EXPECT_GT(motion, 10.0 * age) << "age " << age;
    }
}

TEST(RtkBaseAlignmentTest, LossOfLockDropsCarrierButKeepsCode) {
    const NavigationData nav = makeNavigation();
    const GNSSTime target = kBaseTime + 0.4;
    ObservationData base = generate(nav, kBaseTime, target);
    ASSERT_GE(base.observations.size(), 4U);
    base.observations[0].lli = 0x01;
    base.observations[1].loss_of_lock = true;
    base.observations[2].lli = 0x02;  // bit 1 is not a cycle-slip flag
    ObservationData held;
    ASSERT_TRUE(align::holdBaseEpoch(base, target, kBase, nav, 2.0, held));
    ASSERT_EQ(held.observations.size(), base.observations.size());
    EXPECT_FALSE(held.observations[0].has_carrier_phase);
    EXPECT_TRUE(held.observations[0].has_pseudorange);
    EXPECT_EQ(held.observations[0].lli, 0x01);
    EXPECT_FALSE(held.observations[1].has_carrier_phase);
    EXPECT_TRUE(held.observations[1].has_pseudorange);
    EXPECT_TRUE(held.observations[2].has_carrier_phase);
    EXPECT_TRUE(held.observations[3].has_carrier_phase);
}

TEST(RtkBaseAlignmentTest, AgeLimitIsRespected) {
    const NavigationData nav = makeNavigation();
    const ObservationData base = generate(nav, kBaseTime, kBaseTime + 1.0);
    ObservationData held;
    EXPECT_TRUE(align::holdBaseEpoch(base, kBaseTime, kBase, nav, 2.0, held));       // age 0
    EXPECT_TRUE(align::holdBaseEpoch(base, kBaseTime + 2.0, kBase, nav, 2.0, held)); // age == limit
    EXPECT_FALSE(align::holdBaseEpoch(base, kBaseTime + 2.2, kBase, nav, 2.0, held));
    EXPECT_FALSE(align::holdBaseEpoch(base, kBaseTime + 1.0, kBase, nav, 0.5, held));
    EXPECT_FALSE(align::holdBaseEpoch(base, kBaseTime - 0.2, kBase, nav, 2.0, held)); // future target
    EXPECT_FALSE(align::holdBaseEpoch(base, kBaseTime + 0.2, kBase, nav, 0.0, held));
    EXPECT_FALSE(align::holdBaseEpoch(base, kBaseTime + 0.2, kBase, nav,
        std::numeric_limits<double>::quiet_NaN(), held));
}

TEST(RtkBaseAlignmentTest, UnmodelableSatellitesAreOmittedAndEmptyFails) {
    const NavigationData nav = makeNavigation();
    const GNSSTime target = kBaseTime + 0.4;
    ObservationData base = generate(nav, kBaseTime, target);
    const std::size_t n = base.observations.size();
    ASSERT_GE(n, 4U);
    Observation unknown(SatelliteId(GNSSSystem::GPS, 40), SignalType::GPS_L1CA);  // no ephemeris
    unknown.pseudorange = 2.2e7;
    unknown.has_pseudorange = true;
    base.addObservation(unknown);
    Observation no_code = base.observations[0];
    no_code.satellite = SatelliteId(GNSSSystem::GPS, 41);
    base.addObservation(no_code);
    base.observations[1].has_pseudorange = false;  // carrier-only rows are not held
    ObservationData held;
    ASSERT_TRUE(align::holdBaseEpoch(base, target, kBase, nav, 2.0, held));
    EXPECT_EQ(held.observations.size(), n - 1);
    for (const auto& obs : held.observations) {
        EXPECT_NE(obs.satellite.prn, 40);
        EXPECT_NE(obs.satellite.prn, 41);
    }
    EXPECT_FALSE(align::holdBaseEpoch(ObservationData(kBaseTime), target, kBase, nav, 2.0, held));
    EXPECT_FALSE(align::holdBaseEpoch(base, target, kBase, NavigationData{}, 2.0, held));
}

// gnss_solve / gnss_fuse align a 1 Hz base to a faster rover by interpolating
// between the two bracketing base epochs. The residual to the modeled range is
// interpolated linearly, so the modeled range itself has to be evaluated at
// the target time (opt-in `model_range_at_target`, gnss_solve
// --base-interp-curvature); interpolating it linearly as well reduces the
// result to a linear interpolation of the raw observable and drops the range
// curvature.
TEST(RtkBaseAlignmentTest, InterpolationReproducesObservationsGeneratedAtTargetTime) {
    const NavigationData nav = makeNavigation();
    const GNSSTime after_time = kBaseTime + 1.0;
    const ObservationData before = generate(nav, kBaseTime, after_time);
    const ObservationData after = generate(nav, after_time, kBaseTime);
    ASSERT_GE(before.observations.size(), 4U);
    ASSERT_EQ(before.observations.size(), after.observations.size());
    for (const double alpha : {0.25, 0.5, 0.75}) {
        const GNSSTime target = kBaseTime + alpha;
        std::map<int, Observation> truth;
        for (const auto& obs : generate(nav, target, after_time).observations)
            truth[obs.satellite.prn] = obs;
        ObservationData interpolated;
        ASSERT_TRUE(libgnss_apps::interpolateBaseEpoch(before, after, target, kBase, nav,
                                                       interpolated, true));
        ASSERT_EQ(interpolated.observations.size(), before.observations.size());
        double worst_code = 0.0, worst_phase = 0.0, curvature = 0.0;
        for (std::size_t i = 0; i < interpolated.observations.size(); ++i) {
            const auto& obs = interpolated.observations[i];
            ASSERT_EQ(obs.satellite.prn, before.observations[i].satellite.prn);
            const auto found = truth.find(obs.satellite.prn);
            ASSERT_NE(found, truth.end());
            ASSERT_TRUE(obs.has_pseudorange);
            ASSERT_TRUE(obs.has_carrier_phase);
            worst_code = std::max(worst_code, std::abs(obs.pseudorange - found->second.pseudorange));
            worst_phase = std::max(worst_phase,
                std::abs((obs.carrier_phase - found->second.carrier_phase) * kLambda));
            const double raw_linear = before.observations[i].carrier_phase +
                alpha * (after.observations[i].carrier_phase - before.observations[i].carrier_phase);
            curvature = std::max(curvature,
                std::abs((raw_linear - found->second.carrier_phase) * kLambda));
        }
        // The scene has range curvature that a raw linear interpolation misses.
        EXPECT_GT(curvature, 5e-3) << "alpha " << alpha;
        EXPECT_LT(worst_code, 1e-3) << "alpha " << alpha;
        EXPECT_LT(worst_phase, 1e-3) << "alpha " << alpha;
    }
}

// A broadcast ephemeris switch inside the base gap must not enter the
// interpolated observation: the curvature term has to come from one orbit.
TEST(RtkBaseAlignmentTest, InterpolationIsNotShiftedByAnEphemerisSwitchInsideTheGap) {
    const NavigationData truth_nav = makeNavigation();
    NavigationData nav = makeNavigation();
    for (uint8_t prn = 1; prn <= 32; ++prn) nav.addEphemeris(nextEphemeris(makeEphemeris(prn), 0.8));
    const GNSSTime after_time = kBaseTime + 1.0;
    const ObservationData before = generate(truth_nav, kBaseTime, after_time);
    const ObservationData after = generate(truth_nav, after_time, kBaseTime);
    ASSERT_GE(before.observations.size(), 4U);
    for (const double alpha : {0.25, 0.5, 0.75}) {
        const GNSSTime target = kBaseTime + alpha;
        std::map<int, Observation> truth;
        for (const auto& obs : generate(truth_nav, target, after_time).observations)
            truth[obs.satellite.prn] = obs;
        ObservationData interpolated;
        ASSERT_TRUE(libgnss_apps::interpolateBaseEpoch(before, after, target, kBase, nav,
                                                       interpolated, true));
        ASSERT_EQ(interpolated.observations.size(), before.observations.size());
        double worst_code = 0.0, worst_phase = 0.0;
        for (const auto& obs : interpolated.observations) {
            const auto found = truth.find(obs.satellite.prn);
            ASSERT_NE(found, truth.end());
            worst_code = std::max(worst_code, std::abs(obs.pseudorange - found->second.pseudorange));
            worst_phase = std::max(worst_phase,
                std::abs((obs.carrier_phase - found->second.carrier_phase) * kLambda));
        }
        EXPECT_LT(worst_code, 1e-3) << "alpha " << alpha;
        EXPECT_LT(worst_phase, 1e-3) << "alpha " << alpha;
    }
}

// The default keeps the legacy behaviour: the modeled range is interpolated
// linearly too, so the result equals a linear interpolation of the raw
// observable.
TEST(RtkBaseAlignmentTest, DefaultInterpolationIsLinearInTheRawObservable) {
    const NavigationData nav = makeNavigation();
    const GNSSTime after_time = kBaseTime + 1.0;
    const ObservationData before = generate(nav, kBaseTime, after_time);
    const ObservationData after = generate(nav, after_time, kBaseTime);
    ASSERT_GE(before.observations.size(), 4U);
    const double alpha = 0.5;
    ObservationData interpolated;
    ASSERT_TRUE(libgnss_apps::interpolateBaseEpoch(before, after, kBaseTime + alpha, kBase, nav,
                                                   interpolated));
    ASSERT_EQ(interpolated.observations.size(), before.observations.size());
    for (std::size_t i = 0; i < interpolated.observations.size(); ++i) {
        const auto& b = before.observations[i];
        const auto& a = after.observations[i];
        const auto& obs = interpolated.observations[i];
        EXPECT_NEAR(obs.pseudorange, b.pseudorange + alpha * (a.pseudorange - b.pseudorange), 1e-6);
        EXPECT_NEAR(obs.carrier_phase, b.carrier_phase + alpha * (a.carrier_phase - b.carrier_phase),
                    1e-6);
    }
}
