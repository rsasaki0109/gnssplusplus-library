#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_clas_dd.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <cmath>

using namespace libgnss;

namespace {

OSRCorrection makeCorrection() {
    OSRCorrection correction;
    correction.num_frequencies = 2;
    correction.trop_correction_m = 2.0;
    correction.relativity_correction_m = 0.3;
    correction.receiver_antenna_m[0] = 0.4;
    correction.receiver_antenna_m[1] = 0.6;
    correction.code_bias_m[0] = 1.1;
    correction.code_bias_m[1] = 1.3;
    correction.phase_bias_m[0] = 0.7;
    correction.phase_bias_m[1] = 0.9;
    correction.windup_m[0] = 0.2;
    correction.windup_m[1] = 0.25;
    correction.phase_compensation_m[0] = 0.05;
    correction.phase_compensation_m[1] = 0.08;
    correction.PRC[0] = 5.0;
    correction.PRC[1] = 7.0;
    correction.CPC[0] = 6.0;
    correction.CPC[1] = 8.0;
    return correction;
}

}  // namespace

TEST(PPPClasTest, FullOsrModeUsesDefaultFullCpcPhaseCorrections) {
    const auto correction = makeCorrection();
    const auto applied = ppp_clas::selectAppliedOsrCorrections(
        correction,
        0,
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR);

    // Trop remains inside the default full-CPC phase rows; the filter keeps
    // only the residual phase trop model.
    EXPECT_DOUBLE_EQ(applied.pseudorange_correction_m, 3.0);
    EXPECT_DOUBLE_EQ(applied.carrier_phase_correction_m, 6.0);
    EXPECT_TRUE(ppp_clas::usesClasTropospherePrior(
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR));
}

TEST(PPPClasTest, OrbitClockBiasModeSuppressesAtmosphereButKeepsBiasTerms) {
    const auto correction = makeCorrection();
    const auto applied = ppp_clas::selectAppliedOsrCorrections(
        correction,
        1,
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS);

    EXPECT_DOUBLE_EQ(applied.pseudorange_correction_m, 2.2);
    EXPECT_DOUBLE_EQ(applied.carrier_phase_correction_m, 2.13);
    EXPECT_FALSE(ppp_clas::usesClasTropospherePrior(
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS));
}

TEST(PPPClasTest, OrbitClockOnlyModeDropsBiasAndPhaseCompensationTerms) {
    const auto correction = makeCorrection();
    const auto applied = ppp_clas::selectAppliedOsrCorrections(
        correction,
        1,
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY);

    EXPECT_DOUBLE_EQ(applied.pseudorange_correction_m, 0.9);
    EXPECT_DOUBLE_EQ(applied.carrier_phase_correction_m, 1.15);
    EXPECT_FALSE(ppp_clas::usesClasTropospherePrior(
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY));
}

TEST(PPPClasDdTest, RowBuilderSelectsHighestElevationReferenceAndFormsResidual) {
    ObservationData obs(GNSSTime(2068, 230572.0));
    const Vector3d receiver(constants::WGS84_A, 0.0, 0.0);
    const double wavelength = constants::GPS_L1_WAVELENGTH;

    auto make_osr = [&](uint8_t prn,
                        const Vector3d& satellite_position,
                        double code_residual,
                        double phase_residual) {
        OSRCorrection osr;
        osr.satellite = SatelliteId(GNSSSystem::GPS, prn);
        osr.valid = true;
        osr.num_frequencies = 1;
        osr.signals[0] = SignalType::GPS_L1CA;
        osr.frequencies[0] = constants::GPS_L1_FREQ;
        osr.wavelengths[0] = wavelength;
        osr.satellite_position = satellite_position;
        osr.satellite_clock_bias_s = 0.0;
        osr.PRC[0] = 1.25 + prn;
        osr.CPC[0] = -0.75 + prn;

        double lat = 0.0;
        double lon = 0.0;
        double h = 0.0;
        ecef2geodetic(receiver, lat, lon, h);
        const Vector3d enu = ecef2enu(satellite_position - receiver, lat, lon);
        osr.elevation = std::atan2(enu.z(), std::hypot(enu.x(), enu.y()));

        const double geo = geodist(satellite_position, receiver);
        Observation raw(osr.satellite, SignalType::GPS_L1CA);
        raw.valid = true;
        raw.has_pseudorange = true;
        raw.has_carrier_phase = true;
        raw.pseudorange = geo + osr.PRC[0] + code_residual;
        raw.carrier_phase = (geo + osr.CPC[0] + phase_residual) / wavelength;
        obs.addObservation(raw);
        return osr;
    };

    const OSRCorrection high = make_osr(
        1,
        receiver + Vector3d(20200000.0, 0.0, 1000000.0),
        4.0,
        0.4);
    const OSRCorrection low = make_osr(
        2,
        receiver + Vector3d(15000000.0, 15000000.0, 0.0),
        1.5,
        -0.2);

    ppp_clas_dd::StateLayoutOptions options;
    options.frequencies = 1;
    options.ionosphere_mode = ppp_clas_dd::IonosphereMode::Off;
    options.troposphere_mode = ppp_clas_dd::TroposphereMode::Off;
    const ppp_clas_dd::StateLayout layout{options};
    VectorXd state = VectorXd::Zero(layout.nx());
    state.segment(0, 3) = receiver;

    ppp_shared::PPPConfig config;
    config.estimate_ionosphere = false;
    config.estimate_troposphere = false;
    config.use_ionosphere_free = false;

    const auto build = ppp_clas_dd::buildDdMeasurementSystem(
        obs,
        {low, high},
        layout,
        state,
        config,
        [](const Vector3d&, double, const GNSSTime&) { return 0.0; });

    ASSERT_EQ(build.rows.size(), 2u);
    ASSERT_EQ(build.phase_rows, 1);
    ASSERT_EQ(build.code_rows, 1);
    for (const auto& row : build.rows) {
        EXPECT_EQ(row.reference_satellite, high.satellite);
        EXPECT_EQ(row.target_satellite, low.satellite);
        if (row.is_phase) {
            EXPECT_NEAR(row.residual_m, 0.6, 1e-6);
            EXPECT_NEAR(
                row.state_coefficients.at(0).coefficient,
                wavelength,
                1e-12);
            EXPECT_NEAR(
                row.state_coefficients.at(1).coefficient,
                -wavelength,
                1e-12);
        } else {
            EXPECT_NEAR(row.residual_m, 2.5, 1e-6);
            EXPECT_TRUE(row.state_coefficients.empty());
        }
    }
}

TEST(PPPClasDdTest, LambdaConditioningFixesDdNativeAmbiguities) {
    VectorXd state(5);
    state << 10.0, -3.0, 5.02, 4.00, 7.01;

    MatrixXd covariance = MatrixXd::Zero(5, 5);
    covariance.diagonal() << 0.5, 0.4, 0.0004, 0.0004, 0.0004;
    covariance(0, 2) = covariance(2, 0) = 0.020;
    covariance(0, 3) = covariance(3, 0) = 0.006;
    covariance(0, 4) = covariance(4, 0) = -0.004;
    covariance(1, 2) = covariance(2, 1) = -0.010;
    covariance(1, 3) = covariance(3, 1) = 0.003;
    covariance(1, 4) = covariance(4, 1) = 0.002;

    const std::vector<rtk_measurement::AmbiguityDifference> differences = {
        {2, 3},
        {2, 4},
    };
    const auto transform =
        rtk_measurement::buildAmbiguityTransform(state, covariance, 2, differences);

    VectorXd fixed_ambiguities;
    double ratio = 0.0;
    ASSERT_TRUE(lambdaSearch(
        transform.dd_float,
        transform.ambiguity_covariance,
        fixed_ambiguities,
        ratio));
    ASSERT_GT(ratio, 2.0);
    ASSERT_EQ(fixed_ambiguities.size(), 2);
    EXPECT_NEAR(fixed_ambiguities(0), 1.0, 1e-12);
    EXPECT_NEAR(fixed_ambiguities(1), -2.0, 1e-12);

    Eigen::LDLT<MatrixXd> ldlt(transform.ambiguity_covariance);
    ASSERT_EQ(ldlt.info(), Eigen::Success);
    const VectorXd dd_residual = transform.dd_float - fixed_ambiguities;
    const VectorXd conditioned_head =
        transform.head_state -
        transform.head_ambiguity_covariance * ldlt.solve(dd_residual);
    const MatrixXd conditioned_covariance =
        covariance.topLeftCorner(2, 2) -
        transform.head_ambiguity_covariance *
            ldlt.solve(transform.head_ambiguity_covariance.transpose());

    EXPECT_LT((conditioned_head - transform.head_state).norm(), 1.0);
    EXPECT_TRUE(conditioned_covariance.isApprox(
        conditioned_covariance.transpose(), 1e-14));
    EXPECT_LT(conditioned_covariance(0, 0), covariance(0, 0));
    EXPECT_LT(conditioned_covariance(1, 1), covariance(1, 1));
}
