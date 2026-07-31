#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_clas_dd.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/observation.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <string>
#include <tuple>

using namespace libgnss;

TEST(PPPClasDdTest, VersionedMeasurementRowDumpPreservesCanonicalKey) {
    const std::string path = "/tmp/gnsspp_clas_dd_measurement_rows_test.csv";
    std::remove(path.c_str());

    ppp_clas_dd::DdMeasurementBuildResult build;
    ppp_clas_dd::DdRow row;
    row.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
    row.target_satellite = SatelliteId(GNSSSystem::GPS, 2);
    row.is_phase = true;
    row.frequency_index = 1;
    row.system_group = 0;
    row.raw_dd_m = 0.005;
    row.residual_m = 0.004;
    row.reference_variance_m2 = 1e-4;
    row.target_variance_m2 = 2e-4;
    row.reference_elevation_rad = 0.9;
    row.target_elevation_rad = 0.7;
    row.position_coefficients = Vector3d(0.1, 0.2, 0.3);
    build.rows.push_back(row);
    build.linearization_position_ecef = Vector3d(1.0, 2.0, 3.0);

    ppp_clas_dd::appendDdMeasurementRowsCsv(
        path, GNSSTime(2068, 230420.0), build);

    std::ifstream input(path);
    ASSERT_TRUE(input.good());
    std::string header;
    std::string data;
    ASSERT_TRUE(static_cast<bool>(std::getline(input, header)));
    ASSERT_TRUE(static_cast<bool>(std::getline(input, data)));
    EXPECT_NE(header.find("target_satellite,raw_dd_m,residual_m"), std::string::npos);
    EXPECT_NE(
        data.find("clas_dd_measurement.v3,2068,230420,prefit,0,1,phase,G01,G02,"),
        std::string::npos);
    std::remove(path.c_str());
}

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

TEST(PPPClasOsrTest, ReceiverAntennaCorrectionUsesRtkLibEnuConvention) {
    const Vector3d receiver_delta_enu(0.10, -0.20, 1.00);
    const Vector3d antenna_offset_neu(0.02, -0.03, 0.12);
    const double pcv_m = 0.004;
    const double azimuth_rad = 35.0 * M_PI / 180.0;
    const double elevation_rad = 40.0 * M_PI / 180.0;

    const double cosel = std::cos(elevation_rad);
    const Vector3d los_enu(
        std::sin(azimuth_rad) * cosel,
        std::cos(azimuth_rad) * cosel,
        std::sin(elevation_rad));
    const Vector3d expected_offset_enu(
        receiver_delta_enu.x() + antenna_offset_neu.y(),
        receiver_delta_enu.y() + antenna_offset_neu.x(),
        receiver_delta_enu.z() + antenna_offset_neu.z());
    const double expected = -expected_offset_enu.dot(los_enu) + pcv_m;

    EXPECT_NEAR(
        clasReceiverAntennaCorrectionMeters(
            receiver_delta_enu,
            antenna_offset_neu,
            pcv_m,
            azimuth_rad,
            elevation_rad),
        expected,
        1e-12);
}

TEST(PPPClasOsrTest, ReceiverAntennaMaterializationUpdatesAggregateCorrections) {
    OSRCorrection osr;
    osr.num_frequencies = 2;
    osr.receiver_antenna_m[1] = 0.20;
    osr.PRC[1] = 10.0;
    osr.CPC[1] = 20.0;

    setClasOsrReceiverAntennaCorrection(osr, 1, 0.45);

    EXPECT_DOUBLE_EQ(osr.receiver_antenna_m[1], 0.45);
    EXPECT_DOUBLE_EQ(osr.PRC[1], 10.25);
    EXPECT_DOUBLE_EQ(osr.CPC[1], 20.25);
}

TEST(PPPClasOsrTest, ReceiverAntennaLookupUsesL1SlotForExactGpsL2w) {
    OSRCorrection osr;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 14);
    osr.num_frequencies = 2;
    osr.signals[0] = SignalType::GPS_L2C;
    osr.pseudorange_rinex_codes[0] = "C2W";
    osr.carrier_rinex_codes[0] = "L2W";
    osr.bias_exact_identity[0] = true;
    osr.signals[1] = SignalType::GPS_L2C;
    osr.pseudorange_rinex_codes[1] = "C2X";
    osr.carrier_rinex_codes[1] = "L2X";
    osr.bias_exact_identity[1] = true;

    EXPECT_EQ(clasReceiverAntennaLookupSignal(osr, 0), SignalType::GPS_L1CA);
    EXPECT_EQ(clasReceiverAntennaLookupSignal(osr, 1), SignalType::GPS_L2C);
}

TEST(PPPClasOsrTest, ReceiverAntennaLookupUsesLegacyR01AndC02SlotsForQzss) {
    OSRCorrection osr;
    osr.satellite = SatelliteId(GNSSSystem::QZSS, 2);
    osr.num_frequencies = 2;
    osr.signals[0] = SignalType::QZS_L1CA;
    osr.signals[1] = SignalType::QZS_L2C;
    osr.claslib_qzss_receiver_antenna_slots = true;

    EXPECT_EQ(clasReceiverAntennaLookupSignal(osr, 0), SignalType::GLO_L1CA);
    EXPECT_EQ(clasReceiverAntennaLookupSignal(osr, 1), SignalType::BDS_B1I);

    osr.claslib_qzss_receiver_antenna_slots = false;
    EXPECT_EQ(clasReceiverAntennaLookupSignal(osr, 0), SignalType::QZS_L1CA);
    EXPECT_EQ(clasReceiverAntennaLookupSignal(osr, 1), SignalType::QZS_L2C);
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
    const Vector3d geometry_displacement(0.1, -0.2, 0.05);
    const auto displaced_build = ppp_clas_dd::buildDdMeasurementSystem(
        obs,
        {low, high},
        layout,
        state,
        config,
        [](const Vector3d&, double, const GNSSTime&) { return 0.0; },
        geometry_displacement);

    ASSERT_EQ(build.rows.size(), 2u);
    ASSERT_EQ(displaced_build.rows.size(), build.rows.size());
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
    for (size_t index = 0; index < build.rows.size(); ++index) {
        EXPECT_NEAR(
            displaced_build.rows[index].raw_dd_m - build.rows[index].raw_dd_m,
            -build.rows[index].position_coefficients.dot(geometry_displacement),
            1e-6);
    }

}

TEST(PPPClasDdTest, RowBuilderAdmitsQzssRowsWithDedicatedReferenceGroup) {
    ObservationData obs(GNSSTime(2068, 230602.0));
    const Vector3d receiver(constants::WGS84_A, 0.0, 0.0);
    const double l1_wavelength = constants::GPS_L1_WAVELENGTH;
    const double l2_wavelength = constants::GPS_L2_WAVELENGTH;

    auto make_qzss_osr = [&](uint8_t prn,
                             const Vector3d& satellite_position,
                             double code_l1_residual,
                             double phase_l1_residual,
                             double code_l2_residual,
                             double phase_l2_residual) {
        OSRCorrection osr;
        osr.satellite = SatelliteId(GNSSSystem::QZSS, prn);
        osr.valid = true;
        osr.num_frequencies = 2;
        osr.signals[0] = SignalType::QZS_L1CA;
        osr.signals[1] = SignalType::QZS_L2C;
        osr.frequencies[0] = constants::GPS_L1_FREQ;
        osr.frequencies[1] = constants::GPS_L2_FREQ;
        osr.wavelengths[0] = l1_wavelength;
        osr.wavelengths[1] = l2_wavelength;
        osr.satellite_position = satellite_position;
        osr.PRC[0] = 0.2 + 0.1 * prn;
        osr.PRC[1] = 0.4 + 0.1 * prn;
        osr.CPC[0] = -0.3 + 0.1 * prn;
        osr.CPC[1] = -0.5 + 0.1 * prn;

        double lat = 0.0;
        double lon = 0.0;
        double h = 0.0;
        ecef2geodetic(receiver, lat, lon, h);
        const Vector3d enu = ecef2enu(satellite_position - receiver, lat, lon);
        osr.elevation = std::atan2(enu.z(), std::hypot(enu.x(), enu.y()));

        const double geo = geodist(satellite_position, receiver);
        auto add_observation = [&](SignalType signal,
                                   int frequency_index,
                                   double wavelength,
                                   double code_residual,
                                   double phase_residual) {
            Observation raw(osr.satellite, signal);
            raw.valid = true;
            raw.has_pseudorange = true;
            raw.has_carrier_phase = true;
            raw.pseudorange = geo + osr.PRC[frequency_index] + code_residual;
            raw.carrier_phase =
                (geo + osr.CPC[frequency_index] + phase_residual) /
                wavelength;
            obs.addObservation(raw);
        };
        add_observation(
            SignalType::QZS_L1CA, 0, l1_wavelength, code_l1_residual,
            phase_l1_residual);
        add_observation(
            SignalType::QZS_L2C, 1, l2_wavelength, code_l2_residual,
            phase_l2_residual);
        return osr;
    };

    const OSRCorrection j01 = make_qzss_osr(
        1,
        receiver + Vector3d(16000000.0, 15000000.0, 0.0),
        1.0,
        0.10,
        1.5,
        0.15);
    const OSRCorrection j02 = make_qzss_osr(
        2,
        receiver + Vector3d(22000000.0, 1000000.0, 0.0),
        4.0,
        0.40,
        4.5,
        0.45);
    const OSRCorrection j03 = make_qzss_osr(
        3,
        receiver + Vector3d(17000000.0, -10000000.0, 1000000.0),
        2.0,
        0.20,
        2.5,
        0.25);

    ppp_clas_dd::StateLayoutOptions options;
    options.frequencies = 2;
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
        {j01, j03, j02},
        layout,
        state,
        config,
        [](const Vector3d&, double, const GNSSTime&) { return 0.0; });

    ASSERT_EQ(build.rows.size(), 8u);
    EXPECT_EQ(build.phase_rows, 4);
    EXPECT_EQ(build.code_rows, 4);
    EXPECT_EQ(build.reference_groups, 4);

    std::set<std::tuple<int, int, bool>> reference_keys;
    for (const auto& group : build.reference_groups_detail) {
        EXPECT_EQ(group.system_group, 4);
        EXPECT_EQ(group.reference_satellite, j02.satellite);
        reference_keys.insert(
            {group.system_group, group.frequency_index, group.is_phase});
    }
    EXPECT_EQ(reference_keys.size(), 4u);
    EXPECT_TRUE(reference_keys.count({4, 0, true}));
    EXPECT_TRUE(reference_keys.count({4, 0, false}));
    EXPECT_TRUE(reference_keys.count({4, 1, true}));
    EXPECT_TRUE(reference_keys.count({4, 1, false}));

    for (const auto& row : build.rows) {
        EXPECT_EQ(row.system_group, 4);
        EXPECT_EQ(row.reference_satellite, j02.satellite);
        EXPECT_NE(row.target_satellite, j02.satellite);
        EXPECT_TRUE(row.target_satellite == j01.satellite ||
                    row.target_satellite == j03.satellite);
    }

    auto parity_j01 = j01;
    auto parity_j02 = j02;
    auto parity_j03 = j03;
    for (int frequency = 0; frequency < 2; ++frequency) {
        parity_j01.phase_bias_present[frequency] = true;
        parity_j02.phase_bias_present[frequency] = true;
        parity_j03.phase_bias_present[frequency] = false;
    }
    config.clas_mrtklib_float_parity = true;
    const auto parity_build = ppp_clas_dd::buildDdMeasurementSystem(
        obs,
        {parity_j01, parity_j03, parity_j02},
        layout,
        state,
        config,
        [](const Vector3d&, double, const GNSSTime&) { return 0.0; });
    ASSERT_EQ(parity_build.rows.size(), 4u);
    EXPECT_TRUE(std::none_of(
        parity_build.rows.begin(), parity_build.rows.end(),
        [&](const ppp_clas_dd::DdRow& row) {
            return row.reference_satellite == parity_j03.satellite ||
                   row.target_satellite == parity_j03.satellite;
        }));
}

TEST(PPPClasDdTest, PostfitValidationRejectsLargePhaseRms) {
    ppp_clas_dd::StateLayoutOptions options;
    options.frequencies = 1;
    options.ionosphere_mode = ppp_clas_dd::IonosphereMode::Off;
    options.troposphere_mode = ppp_clas_dd::TroposphereMode::Off;
    const ppp_clas_dd::StateLayout layout{options};

    ppp_clas_dd::DdMeasurementBuildResult build;
    build.rows.resize(4);
    build.measurement_system.design_matrix = MatrixXd::Zero(4, layout.nx());
    build.measurement_system.residuals = VectorXd::Zero(4);
    build.measurement_system.covariance = MatrixXd::Identity(4, 4) * 0.01;
    for (int row = 0; row < 4; ++row) {
        build.rows[static_cast<size_t>(row)].is_phase = true;
        build.rows[static_cast<size_t>(row)].frequency_index = 0;
        build.rows[static_cast<size_t>(row)].residual_m = 1.0;
        build.measurement_system.residuals(row) = 1.0;
    }

    const auto validation = ppp_clas_dd::validateDdPostfitResiduals(
        build,
        layout,
        MatrixXd::Identity(layout.nx(), layout.nx()));

    EXPECT_FALSE(validation.accepted);
    EXPECT_EQ(validation.reject_reason, "postfit_rms");
    EXPECT_GT(validation.phase_residual_rms_m, 0.75);
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

TEST(PPPClasTest, DetectClasCycleSlipsFlagsGeometryFreeJump) {
    ObservationData obs;
    obs.time = GNSSTime(2324, 177000.0);

    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 7);
    osr.num_frequencies = 2;
    osr.frequencies[0] = 1575.42e6;
    osr.frequencies[1] = 1227.60e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
    osr.phase_bias_m[0] = 0.0;
    osr.phase_bias_m[1] = 0.0;
    osr.code_bias_m[0] = 0.0;
    osr.code_bias_m[1] = 0.0;

    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = true;
    l1.has_carrier_phase = true;
    l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;

    Observation l2 = l1;
    l2.signal = SignalType::GPS_L2C;
    l2.carrier_phase = 780.0;
    l2.pseudorange = 2.0e7;

    obs.observations.push_back(l1);
    obs.observations.push_back(l2);

    ppp_shared::PPPConfig config;
    config.kinematic_mode = true;
    config.enable_cycle_slip_detection = true;

    ppp_shared::PPPState filter_state;
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    auto& amb = ambiguity_states[osr.satellite];
    amb.has_last_geometry_free = true;
    amb.last_geometry_free_m = 0.0;
    amb.mw_count = 5;
    amb.mw_mean_cycles = 1.0;

    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    int reset_calls = 0;
    const auto reset_fn = [&](const SatelliteId&, SignalType) { ++reset_calls; };

    const auto stats = ppp_clas::detectClasCycleSlips(
        obs,
        {osr},
        config,
        0.2,
        filter_state,
        ambiguity_states,
        dispersion,
        repair,
        reset_fn,
        3600.0,
        false);

    EXPECT_GE(stats.gf_count, 1);
    EXPECT_GE(stats.total_resets, 1);
    EXPECT_GE(reset_calls, 2);
    EXPECT_TRUE(ambiguity_states[osr.satellite].has_last_slip_time);
    EXPECT_TRUE(dispersion[osr.satellite].slip[0]);
    EXPECT_TRUE(dispersion[osr.satellite].slip[1]);
}

TEST(PPPClasTest, DetectClasCycleSlipsIgnoresCssrPhaseBiasChanges) {
    ObservationData obs;
    obs.time = GNSSTime(2324, 177067.4);

    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 4);
    osr.num_frequencies = 2;
    osr.frequencies[0] = 1575.42e6;
    osr.frequencies[1] = 1227.60e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
    osr.signals[0] = SignalType::GPS_L1CA;
    osr.signals[1] = SignalType::GPS_L2C;
    // A correction-message update is not a receiver cycle slip.  The
    // differential bias is deliberately larger than the GF slip threshold.
    osr.phase_bias_m[0] = 1.0;
    osr.phase_bias_m[1] = 0.0;

    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = true;
    l1.has_carrier_phase = true;
    l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;
    Observation l2 = l1;
    l2.signal = SignalType::GPS_L2C;
    l2.carrier_phase = 780.0;
    obs.observations = {l1, l2};

    ppp_shared::PPPConfig config;
    config.kinematic_mode = true;
    config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = true;
    config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;

    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    auto& ambiguity = ambiguity_states[osr.satellite];
    ambiguity.has_last_geometry_free = true;
    ambiguity.last_geometry_free_m =
        l1.carrier_phase * osr.wavelengths[0] -
        l2.carrier_phase * osr.wavelengths[1];
    ambiguity.mw_count = config.wl_min_averaging_epochs;
    ambiguity.mw_mean_cycles = 1.0;

    ppp_shared::PPPState filter_state;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    int reset_calls = 0;
    const auto stats = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, filter_state, ambiguity_states, dispersion,
        repair,
        [&](const SatelliteId&, SignalType) { ++reset_calls; },
        3600.0, false);

    EXPECT_EQ(stats.gf_count, 0);
    EXPECT_EQ(stats.total_resets, 0);
    EXPECT_EQ(reset_calls, 0);
    EXPECT_FALSE(ambiguity_states[osr.satellite].has_last_slip_time);
}

TEST(PPPClasTest, MrtklibGpsGeometryFreeDoesNotFallbackFromL2wToL2l) {
    ObservationData obs;
    obs.time = GNSSTime(2324, 177067.4);

    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 4);
    osr.num_frequencies = 2;
    osr.frequencies[0] = 1575.42e6;
    osr.frequencies[1] = 1227.60e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
    osr.signals[0] = SignalType::GPS_L1CA;
    osr.signals[1] = SignalType::GPS_L2C;

    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = true;
    l1.has_carrier_phase = true;
    l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;
    l1.carrier_phase_observation_type = "L1C";
    Observation l2 = l1;
    l2.signal = SignalType::GPS_L2C;
    l2.carrier_phase = 780.0;
    l2.carrier_phase_observation_type = "L2L";
    obs.observations = {l1, l2};

    ppp_shared::PPPConfig config;
    config.kinematic_mode = true;
    config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = true;
    config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;

    const SatelliteId l2_ambiguity(GNSSSystem::GPS, 104);
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    auto& ambiguity = ambiguity_states[osr.satellite];
    ambiguity.has_last_geometry_free = true;
    ambiguity.last_geometry_free_m = 10.0;
    ambiguity_states[l2_ambiguity] = {};

    ppp_shared::PPPState filter_state;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    int reset_calls = 0;
    const auto stats = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, filter_state, ambiguity_states, dispersion,
        repair,
        [&](const SatelliteId&, SignalType) { ++reset_calls; },
        3600.0, false);

    EXPECT_EQ(stats.gf_count, 0);
    EXPECT_EQ(stats.total_resets, 0);
    EXPECT_EQ(reset_calls, 0);
    EXPECT_DOUBLE_EQ(
        ambiguity_states[osr.satellite].last_geometry_free_m, 10.0);
}

TEST(PPPClasTest, MrtklibUsableObservationsClearOutageCounterBeforeOverflow) {
    ObservationData obs;
    obs.time = GNSSTime(2324, 177000.0);

    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 11);
    osr.num_frequencies = 2;
    osr.frequencies[0] = 1575.42e6;
    osr.frequencies[1] = 1227.60e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];

    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = true;
    l1.has_carrier_phase = true;
    l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;
    Observation l2 = l1;
    l2.signal = SignalType::GPS_L2C;
    l2.carrier_phase = 780.0;
    obs.observations = {l1, l2};

    ppp_shared::PPPConfig config;
    config.kinematic_mode = true;
    config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = true;
    config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;

    const SatelliteId l2_ambiguity(
        GNSSSystem::GPS, static_cast<uint8_t>(osr.satellite.prn + 100));
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguity_states;
    ambiguity_states[osr.satellite].needs_reinitialization = false;
    ambiguity_states[l2_ambiguity].needs_reinitialization = false;
    ppp_shared::PPPState filter_state;
    filter_state.total_states = 1;
    filter_state.state = VectorXd::Constant(1, 3.0);
    filter_state.covariance = MatrixXd::Constant(1, 1, 2.0);
    filter_state.ionosphere_indices[osr.satellite] = 0;
    filter_state.adaptive_ionosphere_process_noise[osr.satellite] = 1.0;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    int reset_calls = 0;
    const auto reset_fn = [&](const SatelliteId&, SignalType) { ++reset_calls; };

    const auto first = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, filter_state, ambiguity_states, dispersion,
        repair, reset_fn, 3600.0, false);
    EXPECT_EQ(first.per_sat_outage_resets, 0);
    EXPECT_EQ(ambiguity_states[osr.satellite].outage_count, 0);
    EXPECT_EQ(ambiguity_states[l2_ambiguity].outage_count, 0);

    obs.time = GNSSTime(2324, 177000.2);
    const auto second = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, filter_state, ambiguity_states, dispersion,
        repair, reset_fn, 3600.0, false);
    EXPECT_EQ(second.per_sat_outage_resets, 0);
    EXPECT_EQ(reset_calls, 0);
    EXPECT_EQ(ambiguity_states[osr.satellite].lock_count, 0);
    EXPECT_EQ(ambiguity_states[l2_ambiguity].lock_count, 0);
    EXPECT_EQ(ambiguity_states[osr.satellite].outage_count, 0);
    EXPECT_EQ(ambiguity_states[l2_ambiguity].outage_count, 0);
    EXPECT_DOUBLE_EQ(filter_state.state(0), 3.0);
    EXPECT_DOUBLE_EQ(filter_state.covariance(0, 0), 2.0);
    EXPECT_DOUBLE_EQ(
        filter_state.adaptive_ionosphere_process_noise[osr.satellite], 1.0);
    EXPECT_FALSE(dispersion[osr.satellite].slip[0]);
    EXPECT_FALSE(dispersion[osr.satellite].slip[1]);

    ppp_clas::updateObservedAmbiguities(
        obs.time,
        {{osr.satellite, l1.signal, osr.wavelengths[0], l1.carrier_phase, 45.0},
         {l2_ambiguity, l2.signal, osr.wavelengths[1], l2.carrier_phase, 45.0}},
        filter_state,
        ambiguity_states,
        [](const SatelliteId&) { return -1; });
    EXPECT_EQ(ambiguity_states[osr.satellite].outage_count, 0);
    EXPECT_EQ(ambiguity_states[l2_ambiguity].outage_count, 0);
    EXPECT_EQ(ambiguity_states[osr.satellite].lock_count, 1);
    EXPECT_EQ(ambiguity_states[l2_ambiguity].lock_count, 1);
}

TEST(PPPClasTest, MrtklibOutageIgnoresFilterExcludedQzssL2AndGalileo) {
    ppp_shared::PPPConfig config;
    config.kinematic_mode = true;
    config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = true;
    config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;

    auto verify_excluded_outage = [&](GNSSSystem system,
                                      SignalType l1_signal,
                                      SignalType l2_signal) {
        ObservationData obs;
        obs.time = GNSSTime(2068, 230439.0);
        OSRCorrection osr;
        osr.valid = true;
        osr.satellite = SatelliteId(system, 2);
        osr.num_frequencies = 2;
        osr.frequencies[0] = 1575.42e6;
        osr.frequencies[1] = 1227.60e6;
        osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
        osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
        Observation l1;
        l1.satellite = osr.satellite;
        l1.signal = l1_signal;
        l1.valid = l1.has_carrier_phase = l1.has_pseudorange = true;
        l1.carrier_phase = 1000.0;
        l1.pseudorange = 2.0e7;
        Observation l2 = l1;
        l2.signal = l2_signal;
        l2.carrier_phase = 780.0;
        obs.observations = {l1, l2};

        const SatelliteId l2_ambiguity(
            system, static_cast<uint8_t>(osr.satellite.prn + 100));
        std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguities;
        ambiguities[osr.satellite].outage_count =
            system == GNSSSystem::Galileo ? 1 : -1;
        ambiguities[l2_ambiguity].outage_count = 1;
        ppp_shared::PPPState state;
        std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
        std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
        int resets = 0;
        const auto stats = ppp_clas::detectClasCycleSlips(
            obs, {osr}, config, 1.0, state, ambiguities, dispersion, repair,
            [&](const SatelliteId&, SignalType) { ++resets; }, 3600.0, false);

        EXPECT_EQ(stats.per_sat_outage_resets, 0);
        EXPECT_EQ(stats.total_resets, 0);
        EXPECT_EQ(resets, 0);
    };

    verify_excluded_outage(
        GNSSSystem::QZSS, SignalType::QZS_L1CA, SignalType::QZS_L2C);
    verify_excluded_outage(
        GNSSSystem::Galileo, SignalType::GAL_E1, SignalType::GAL_E5A);
}

TEST(PPPClasTest, MrtklibReturningL1OnlySatelliteClearsObservedOutageCounter) {
    ObservationData obs;
    obs.time = GNSSTime(2324, 177091.4);
    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 11);
    osr.num_frequencies = 1;
    osr.frequencies[0] = 1575.42e6;
    osr.frequencies[1] = 1227.60e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = l1.has_carrier_phase = l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;
    obs.observations = {l1};
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 0, "1C");
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 1, "2W");
    obs.addRinexTrackingObservation("1C", l1);

    ppp_shared::PPPConfig config;
    config.kinematic_mode = config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguities;
    ambiguities[osr.satellite].outage_count = 1;
    ppp_shared::PPPState state;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    int resets = 0;
    const auto stats = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, state, ambiguities, dispersion, repair,
        [&](const SatelliteId& sat, SignalType) {
            EXPECT_EQ(sat, osr.satellite);
            ++resets;
        },
        3600.0, false);
    EXPECT_EQ(stats.per_sat_outage_resets, 0);
    EXPECT_EQ(resets, 0);
    EXPECT_EQ(ambiguities[osr.satellite].outage_count, 0);
    EXPECT_EQ(ambiguities[osr.satellite].lock_count, 0);
}

TEST(PPPClasTest, MrtklibReturningCompletePairLliResetsLocksAfterOutageClear) {
    ObservationData obs;
    obs.time = GNSSTime(2324, 177090.8);
    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 9);
    osr.num_frequencies = 2;
    osr.frequencies[0] = 1575.42e6;
    osr.frequencies[1] = 1227.60e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];

    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = l1.has_carrier_phase = l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;
    Observation l2 = l1;
    l2.signal = SignalType::GPS_L2C;
    l2.carrier_phase = 780.0;
    l2.lli = 1;
    l2.loss_of_lock = true;
    l1.pseudorange_observation_type = l1.carrier_phase_observation_type = "1C";
    l2.pseudorange_observation_type = l2.carrier_phase_observation_type = "2W";
    obs.observations = {l1, l2};
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 0, "1C");
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 1, "2W");
    obs.addRinexTrackingObservation("1C", l1);
    obs.addRinexTrackingObservation("2W", l2);

    ppp_shared::PPPConfig config;
    config.kinematic_mode = config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;
    const SatelliteId l2_ambiguity(
        GNSSSystem::GPS, static_cast<uint8_t>(osr.satellite.prn + 100));
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguities;
    ambiguities[osr.satellite].outage_count = 0;
    ambiguities[l2_ambiguity].outage_count = 1;
    ppp_shared::PPPState state;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    int resets = 0;

    const auto stats = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, state, ambiguities, dispersion, repair,
        [&](const SatelliteId&, SignalType) { ++resets; }, 3600.0, false);

    EXPECT_EQ(stats.per_sat_outage_resets, 0);
    EXPECT_EQ(stats.lli_count, 1);
    EXPECT_EQ(resets, 2);
    EXPECT_EQ(ambiguities[osr.satellite].lock_count, -5);
    EXPECT_EQ(ambiguities[l2_ambiguity].lock_count, -5);
    EXPECT_EQ(ambiguities[osr.satellite].outage_count, 0);
    EXPECT_EQ(ambiguities[l2_ambiguity].outage_count, 0);
    EXPECT_TRUE(dispersion[osr.satellite].slip[0]);
    EXPECT_TRUE(dispersion[osr.satellite].slip[1]);
}

TEST(PPPClasTest, MrtklibOutageResetKillSwitchRestoresLegacyOverflow) {
    if (PPPEnvOverrides::fromEnvironment().clas_outage_reset_parity) {
        GTEST_SKIP() << "Covered by the dedicated kill-switch CTest process";
    }
    ASSERT_FALSE(pppEnvOverrides().clas_outage_reset_parity);

    ObservationData obs;
    obs.time = GNSSTime(2324, 177091.4);
    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 11);
    osr.num_frequencies = 1;
    osr.frequencies[0] = 1575.42e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = l1.has_carrier_phase = l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;
    obs.observations = {l1};
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 0, "1C");
    obs.addRinexTrackingObservation("1C", l1);

    ppp_shared::PPPConfig config;
    config.kinematic_mode = config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguities;
    ambiguities[osr.satellite].outage_count = 1;
    ppp_shared::PPPState state;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    int resets = 0;

    const auto stats = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, state, ambiguities, dispersion, repair,
        [&](const SatelliteId&, SignalType) { ++resets; }, 3600.0, false);

    EXPECT_EQ(stats.per_sat_outage_resets, 1);
    EXPECT_EQ(resets, 1);
    EXPECT_EQ(ambiguities[osr.satellite].outage_count, 2);
    EXPECT_EQ(ambiguities[osr.satellite].lock_count, -5);
}

TEST(PPPClasTest, MrtklibL1OnlyLliResetsBeforeOutageOverflow) {
    ObservationData obs;
    obs.time = GNSSTime(2324, 177084.4);
    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 4);
    osr.num_frequencies = 1;
    osr.frequencies[0] = 1575.42e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = l1.has_carrier_phase = l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;
    l1.lli = 1;
    l1.loss_of_lock = true;
    obs.observations = {l1};
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 0, "1C");
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 1, "2W");
    obs.addRinexTrackingObservation("1C", l1);

    ppp_shared::PPPConfig config;
    config.kinematic_mode = config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;
    const SatelliteId l2_ambiguity(GNSSSystem::GPS, 104);
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguities;
    ambiguities[osr.satellite].lock_count = 7;
    ambiguities[osr.satellite].outage_count = -1;
    ambiguities[l2_ambiguity].lock_count = 7;
    ambiguities[l2_ambiguity].outage_count = -1;
    ppp_shared::PPPState state;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    std::map<SatelliteId, SignalType> reset_signals;
    const auto stats = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, state, ambiguities, dispersion, repair,
        [&](const SatelliteId& sat, SignalType signal) {
            reset_signals[sat] = signal;
        },
        3600.0, false);

    EXPECT_EQ(stats.per_sat_outage_resets, 0);
    EXPECT_EQ(stats.lli_count, 1);
    ASSERT_EQ(reset_signals.size(), 1U);
    EXPECT_EQ(reset_signals[osr.satellite], SignalType::GPS_L1CA);
    EXPECT_EQ(ambiguities[osr.satellite].lock_count, -5);
    EXPECT_EQ(ambiguities[l2_ambiguity].lock_count, 7);
    EXPECT_TRUE(dispersion[osr.satellite].slip[0]);
    EXPECT_TRUE(dispersion[osr.satellite].slip[1]);
}

TEST(PPPClasTest, MrtklibUnavailableL2OutageDoesNotResetL1OnlyIonosphere) {
    ObservationData obs;
    obs.time = GNSSTime(2324, 177091.6);
    OSRCorrection osr;
    osr.valid = true;
    osr.satellite = SatelliteId(GNSSSystem::GPS, 11);
    osr.num_frequencies = 1;
    osr.frequencies[0] = 1575.42e6;
    osr.frequencies[1] = 1227.60e6;
    osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
    osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
    Observation l1;
    l1.satellite = osr.satellite;
    l1.signal = SignalType::GPS_L1CA;
    l1.valid = l1.has_carrier_phase = l1.has_pseudorange = true;
    l1.carrier_phase = 1000.0;
    l1.pseudorange = 2.0e7;
    obs.observations = {l1};
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 0, "1C");
    obs.setRinexFrequencySlot(GNSSSystem::GPS, 1, "2W");
    obs.addRinexTrackingObservation("1C", l1);

    ppp_shared::PPPConfig config;
    config.kinematic_mode = config.enable_cycle_slip_detection = true;
    config.clas_mrtklib_float_parity = config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;
    const SatelliteId l2_ambiguity(GNSSSystem::GPS, 111);
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo> ambiguities;
    ambiguities[osr.satellite].outage_count = -1;
    ambiguities[l2_ambiguity].outage_count = 1;
    ppp_shared::PPPState state;
    state.total_states = 1;
    state.state = VectorXd::Constant(1, 3.0);
    state.covariance = MatrixXd::Constant(1, 1, 2.0);
    state.ionosphere_indices[osr.satellite] = 0;
    state.adaptive_ionosphere_process_noise[osr.satellite] = 1.0;
    std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion;
    std::map<SatelliteId, CLASPhaseBiasRepairInfo> repair;
    const auto stats = ppp_clas::detectClasCycleSlips(
        obs, {osr}, config, 0.2, state, ambiguities, dispersion, repair,
        [](const SatelliteId&, SignalType) {}, 3600.0, false);
    EXPECT_EQ(stats.per_sat_outage_resets, 1);
    EXPECT_DOUBLE_EQ(state.state(0), 3.0);
    EXPECT_DOUBLE_EQ(state.covariance(0, 0), 2.0);
    EXPECT_DOUBLE_EQ(
        state.adaptive_ionosphere_process_noise[osr.satellite], 1.0);
}

TEST(PPPClasTest, MrtklibAdaptiveIonoNoiseClampsOnlyObservedSatellite) {
    ppp_shared::PPPConfig config;
    config.kinematic_mode = true;
    config.clas_mrtklib_float_parity = true;
    config.use_clas_osr_filter = true;
    config.use_dynamics_model = true;
    config.estimate_ionosphere = true;

    ppp_shared::PPPState state;
    state.total_states = 14;
    state.pos_index = 0;
    state.vel_index = 3;
    state.accel_index = 6;
    state.clock_index = 9;
    state.glo_clock_index = 10;
    state.trop_index = 11;
    state.state = VectorXd::Ones(state.total_states);
    state.covariance = MatrixXd::Identity(state.total_states, state.total_states);
    const SatelliteId observed(GNSSSystem::GPS, 6);
    const SatelliteId unobserved(GNSSSystem::GPS, 9);
    state.ionosphere_indices[observed] = 12;
    state.ionosphere_indices[unobserved] = 13;
    state.adaptive_ionosphere_process_noise[observed] = 1.0;
    state.adaptive_ionosphere_process_noise[unobserved] = 1.0;
    const std::set<SatelliteId> observed_satellites{observed};

    ppp_clas::predictFilterState(
        state, config, 0.2, Vector3d::Zero(), 0.0, false,
        &observed_satellites);

    EXPECT_NEAR(state.adaptive_ionosphere_process_noise[observed],
                0.05 * 0.05, 1e-15);
    EXPECT_DOUBLE_EQ(state.adaptive_ionosphere_process_noise[unobserved], 1.0);
    EXPECT_NEAR(state.covariance(12, 12), 1.0 + 0.05 * 0.05 * 0.2, 1e-15);
    EXPECT_DOUBLE_EQ(state.covariance(13, 13), 1.0);
}

TEST(PPPClasTest, MrtklibDdLayoutKeepsCanonicalDynamicsOff) {
    ppp_shared::PPPConfig config;
    config.kinematic_mode = true;
    config.use_dynamics_model = true;
    config.clas_mrtklib_float_parity = true;

    ppp_shared::PPPState native_state;
    native_state.pos_index = 0;
    native_state.total_states = 9;
    native_state.state = VectorXd::Zero(9);
    native_state.state.head<3>() << -3957235.0, 3310368.0, 3737530.0;
    native_state.covariance = MatrixXd::Identity(9, 9);

    PositionSolution native_solution;
    native_solution.position_ecef = native_state.state.head<3>();

    ppp_clas_dd::DdFilterScaffold scaffold;
    ObservationData obs(GNSSTime(2068, 230420.0));
    CLASEpochContext context;
    scaffold.processFloatUpdate(
        obs, context, native_state, native_solution, config,
        [](const Vector3d&, double, const GNSSTime&) { return 0.0; });

    ASSERT_TRUE(scaffold.hasSnapshot());
    EXPECT_FALSE(scaffold.snapshot().layout.options.dynamics);
    EXPECT_EQ(scaffold.snapshot().layout.np(), 3);
    EXPECT_EQ(scaffold.snapshot().layout.nt(), 0);
    EXPECT_EQ(scaffold.snapshot().layout.nr(), 3);
    EXPECT_DOUBLE_EQ(scaffold.snapshot().covariance(0, 0), 900.0);
}
