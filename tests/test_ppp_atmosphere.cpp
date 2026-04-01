#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <sstream>

using namespace libgnss;

namespace {

std::string makeResidualList(size_t size, size_t hot_index, double hot_value, double fill_value = 0.0) {
    std::ostringstream stream;
    for (size_t index = 0; index < size; ++index) {
        if (index != 0) {
            stream << ';';
        }
        stream << (index == hot_index ? hot_value : fill_value);
    }
    return stream.str();
}

}  // namespace

TEST(PPPAtmosphereTest, ResolveClasGridReferenceChoosesNearestGridPoint) {
    std::map<std::string, std::string> atmos_tokens;
    atmos_tokens["atmos_network_id"] = "3";
    atmos_tokens["atmos_grid_count"] = "17";

    const Vector3d receiver_position = geodetic2ecef(33.17 * M_PI / 180.0, 130.15 * M_PI / 180.0, 10.0);
    ppp_atmosphere::ClasGridReference reference;
    ASSERT_TRUE(ppp_atmosphere::resolveClasGridReference(atmos_tokens, receiver_position, reference));
    EXPECT_EQ(reference.network_id, 3);
    EXPECT_EQ(reference.grid_no, 16);
    EXPECT_EQ(reference.residual_index, 15U);
    EXPECT_NEAR(reference.dlat_deg, 0.01, 1e-6);
    EXPECT_NEAR(reference.dlon_deg, -0.01, 1e-6);
}

TEST(PPPAtmosphereTest, AppliesGridResidualsToTroposphereAndStecCorrections) {
    std::map<std::string, std::string> atmos_tokens;
    atmos_tokens["atmos_network_id"] = "3";
    atmos_tokens["atmos_grid_count"] = "17";
    atmos_tokens["atmos_trop_type"] = "0";
    atmos_tokens["atmos_trop_t00_m"] = "0.4";
    atmos_tokens["atmos_trop_residuals_m"] = makeResidualList(17, 15, 0.2);

    const std::string suffix = ":G01";
    atmos_tokens["atmos_stec_type" + suffix] = "3";
    atmos_tokens["atmos_stec_c00_tecu" + suffix] = "2.0";
    atmos_tokens["atmos_stec_c01_tecu_per_deg" + suffix] = "0.5";
    atmos_tokens["atmos_stec_c10_tecu_per_deg" + suffix] = "-0.2";
    atmos_tokens["atmos_stec_c11_tecu_per_deg2" + suffix] = "0.1";
    atmos_tokens["atmos_stec_c02_tecu_per_deg2" + suffix] = "0.05";
    atmos_tokens["atmos_stec_c20_tecu_per_deg2" + suffix] = "-0.07";
    atmos_tokens["atmos_stec_residuals_tecu" + suffix] = makeResidualList(17, 15, 0.3);

    const Vector3d receiver_position = geodetic2ecef(33.16 * M_PI / 180.0, 130.16 * M_PI / 180.0, 0.0);
    const GNSSTime time(2400, 345600.0);

    const double trop_correction_m =
        ppp_atmosphere::atmosphericTroposphereCorrectionMeters(atmos_tokens, receiver_position, time, 0.7);
    EXPECT_NEAR(trop_correction_m, 0.6, 1e-9);

    const SatelliteId satellite(GNSSSystem::GPS, 1);
    const double stec_tecu =
        ppp_atmosphere::atmosphericStecTecu(atmos_tokens, satellite, receiver_position);
    EXPECT_NEAR(stec_tecu, 2.3, 1e-9);
}

TEST(PPPAtmosphereTest, ResidualOnlyValueConstructionDropsPolynomialTerms) {
    using Policy = ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy;

    std::map<std::string, std::string> atmos_tokens;
    atmos_tokens["atmos_network_id"] = "3";
    atmos_tokens["atmos_grid_count"] = "17";
    atmos_tokens["atmos_trop_type"] = "0";
    atmos_tokens["atmos_trop_t00_m"] = "0.4";
    atmos_tokens["atmos_trop_offset_m"] = "0.1";
    atmos_tokens["atmos_trop_residuals_m"] = makeResidualList(17, 15, 0.2);

    const std::string suffix = ":G01";
    atmos_tokens["atmos_stec_type" + suffix] = "3";
    atmos_tokens["atmos_stec_c00_tecu" + suffix] = "2.0";
    atmos_tokens["atmos_stec_c01_tecu_per_deg" + suffix] = "0.5";
    atmos_tokens["atmos_stec_residuals_tecu" + suffix] = makeResidualList(17, 15, 0.3);

    const Vector3d receiver_position = geodetic2ecef(33.16 * M_PI / 180.0, 130.16 * M_PI / 180.0, 0.0);
    const GNSSTime time(2400, 345600.0);
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    const double trop_correction_m =
        ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
            atmos_tokens, receiver_position, time, 0.7, Policy::RESIDUAL_ONLY);
    EXPECT_NEAR(trop_correction_m, 0.3, 1e-9);

    const double stec_tecu =
        ppp_atmosphere::atmosphericStecTecu(
            atmos_tokens, satellite, receiver_position, Policy::RESIDUAL_ONLY);
    EXPECT_NEAR(stec_tecu, 0.3, 1e-9);
}

TEST(PPPAtmosphereTest, PolynomialOnlyValueConstructionDropsResidualTerms) {
    using Policy = ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy;

    std::map<std::string, std::string> atmos_tokens;
    atmos_tokens["atmos_network_id"] = "3";
    atmos_tokens["atmos_grid_count"] = "17";
    atmos_tokens["atmos_trop_type"] = "0";
    atmos_tokens["atmos_trop_t00_m"] = "0.4";
    atmos_tokens["atmos_trop_offset_m"] = "0.1";
    atmos_tokens["atmos_trop_residuals_m"] = makeResidualList(17, 15, 0.2);

    const std::string suffix = ":G01";
    atmos_tokens["atmos_stec_type" + suffix] = "3";
    atmos_tokens["atmos_stec_c00_tecu" + suffix] = "2.0";
    atmos_tokens["atmos_stec_c01_tecu_per_deg" + suffix] = "0.5";
    atmos_tokens["atmos_stec_residuals_tecu" + suffix] = makeResidualList(17, 15, 0.3);

    const Vector3d receiver_position = geodetic2ecef(33.16 * M_PI / 180.0, 130.16 * M_PI / 180.0, 0.0);
    const GNSSTime time(2400, 345600.0);
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    const double trop_correction_m =
        ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
            atmos_tokens, receiver_position, time, 0.7, Policy::POLYNOMIAL_ONLY);
    EXPECT_NEAR(trop_correction_m, 0.4, 1e-9);

    const double stec_tecu =
        ppp_atmosphere::atmosphericStecTecu(
            atmos_tokens, satellite, receiver_position, Policy::POLYNOMIAL_ONLY);
    EXPECT_NEAR(stec_tecu, 2.0, 1e-9);
}

TEST(PPPAtmosphereTest, IndexedOnlyResidualSamplingUsesGridResidualInsteadOfMean) {
    using ValuePolicy = ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy;
    using Subtype12Policy = ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy;
    using SamplingPolicy = ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy;

    std::map<std::string, std::string> atmos_tokens;
    atmos_tokens["atmos_network_id"] = "3";
    atmos_tokens["atmos_grid_count"] = "17";
    atmos_tokens["atmos_trop_type"] = "0";
    atmos_tokens["atmos_trop_residuals_m"] = makeResidualList(17, 15, 0.2, 1.0);

    const std::string suffix = ":G01";
    atmos_tokens["atmos_stec_type" + suffix] = "0";
    atmos_tokens["atmos_stec_residuals_tecu" + suffix] = makeResidualList(17, 15, 0.3, 1.5);

    const Vector3d receiver_position = geodetic2ecef(33.16 * M_PI / 180.0, 130.16 * M_PI / 180.0, 0.0);
    const GNSSTime time(2400, 345600.0);
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    const double trop_correction_m = ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
        atmos_tokens,
        receiver_position,
        time,
        0.7,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::FULL,
        SamplingPolicy::INDEXED_ONLY);
    EXPECT_NEAR(trop_correction_m, 0.2, 1e-9);

    const double stec_tecu = ppp_atmosphere::atmosphericStecTecu(
        atmos_tokens,
        satellite,
        receiver_position,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::FULL,
        SamplingPolicy::INDEXED_ONLY);
    EXPECT_NEAR(stec_tecu, 0.3, 1e-9);
}

TEST(PPPAtmosphereTest, MeanOnlyResidualSamplingIgnoresGridResidualIndex) {
    using ValuePolicy = ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy;
    using Subtype12Policy = ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy;
    using SamplingPolicy = ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy;

    std::map<std::string, std::string> atmos_tokens;
    atmos_tokens["atmos_network_id"] = "3";
    atmos_tokens["atmos_grid_count"] = "17";
    atmos_tokens["atmos_trop_type"] = "0";
    atmos_tokens["atmos_trop_residuals_m"] = makeResidualList(17, 15, 0.2, 1.0);

    const std::string suffix = ":G01";
    atmos_tokens["atmos_stec_type" + suffix] = "0";
    atmos_tokens["atmos_stec_residuals_tecu" + suffix] = makeResidualList(17, 15, 0.3, 1.5);

    const Vector3d receiver_position = geodetic2ecef(33.16 * M_PI / 180.0, 130.16 * M_PI / 180.0, 0.0);
    const GNSSTime time(2400, 345600.0);
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    const double expected_trop_mean = ((16.0 * 1.0) + 0.2) / 17.0;
    const double expected_stec_mean = ((16.0 * 1.5) + 0.3) / 17.0;

    const double trop_correction_m = ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
        atmos_tokens,
        receiver_position,
        time,
        0.7,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::FULL,
        SamplingPolicy::MEAN_ONLY);
    EXPECT_NEAR(trop_correction_m, expected_trop_mean, 1e-9);

    const double stec_tecu = ppp_atmosphere::atmosphericStecTecu(
        atmos_tokens,
        satellite,
        receiver_position,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::FULL,
        SamplingPolicy::MEAN_ONLY);
    EXPECT_NEAR(stec_tecu, expected_stec_mean, 1e-9);
}

TEST(PPPAtmosphereTest, Subtype12PlanarValueConstructionDropsQuadraticTermsButKeepsLinearTerms) {
    using ValuePolicy = ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy;
    using Subtype12Policy = ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy;
    using SamplingPolicy = ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy;

    std::map<std::string, std::string> atmos_tokens;
    atmos_tokens["atmos_network_id"] = "3";
    atmos_tokens["atmos_grid_count"] = "17";
    atmos_tokens["atmos_trop_source_subtype"] = "12";
    atmos_tokens["atmos_trop_type"] = "2";
    atmos_tokens["atmos_trop_t00_m"] = "0.5";
    atmos_tokens["atmos_trop_t01_m_per_deg"] = "2.0";
    atmos_tokens["atmos_trop_t10_m_per_deg"] = "-1.0";
    atmos_tokens["atmos_trop_t11_m_per_deg2"] = "10.0";
    atmos_tokens["atmos_trop_residuals_m"] = makeResidualList(17, 15, 0.2);

    const std::string suffix = ":G01";
    atmos_tokens["atmos_stec_source_subtype" + suffix] = "12";
    atmos_tokens["atmos_stec_type" + suffix] = "3";
    atmos_tokens["atmos_stec_c00_tecu" + suffix] = "1.0";
    atmos_tokens["atmos_stec_c01_tecu_per_deg" + suffix] = "3.0";
    atmos_tokens["atmos_stec_c10_tecu_per_deg" + suffix] = "1.0";
    atmos_tokens["atmos_stec_c11_tecu_per_deg2" + suffix] = "100.0";
    atmos_tokens["atmos_stec_c02_tecu_per_deg2" + suffix] = "200.0";
    atmos_tokens["atmos_stec_c20_tecu_per_deg2" + suffix] = "-150.0";
    atmos_tokens["atmos_stec_residuals_tecu" + suffix] = makeResidualList(17, 15, 0.3);

    const Vector3d receiver_position = geodetic2ecef(33.17 * M_PI / 180.0, 130.15 * M_PI / 180.0, 0.0);
    const GNSSTime time(2400, 345600.0);
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    const double trop_planar = ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
        atmos_tokens,
        receiver_position,
        time,
        0.7,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::PLANAR,
        SamplingPolicy::INDEXED_OR_MEAN);
    const double trop_full = ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
        atmos_tokens,
        receiver_position,
        time,
        0.7,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::FULL,
        SamplingPolicy::INDEXED_OR_MEAN);
    EXPECT_NEAR(trop_planar, 0.73, 1e-9);
    EXPECT_NEAR(trop_full, 0.729, 1e-9);

    const double stec_planar = ppp_atmosphere::atmosphericStecTecu(
        atmos_tokens,
        satellite,
        receiver_position,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::PLANAR,
        SamplingPolicy::INDEXED_OR_MEAN);
    const double stec_full = ppp_atmosphere::atmosphericStecTecu(
        atmos_tokens,
        satellite,
        receiver_position,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::FULL,
        SamplingPolicy::INDEXED_OR_MEAN);
    EXPECT_NEAR(stec_planar, 1.32, 1e-9);
    EXPECT_NEAR(stec_full, 1.315, 1e-9);
}

TEST(PPPAtmosphereTest, Subtype12OffsetOnlyValueConstructionDropsLinearAndQuadraticTerms) {
    using ValuePolicy = ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy;
    using Subtype12Policy = ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy;
    using SamplingPolicy = ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy;

    std::map<std::string, std::string> atmos_tokens;
    atmos_tokens["atmos_network_id"] = "3";
    atmos_tokens["atmos_grid_count"] = "17";
    atmos_tokens["atmos_trop_source_subtype"] = "12";
    atmos_tokens["atmos_trop_type"] = "2";
    atmos_tokens["atmos_trop_t00_m"] = "0.5";
    atmos_tokens["atmos_trop_t01_m_per_deg"] = "2.0";
    atmos_tokens["atmos_trop_t10_m_per_deg"] = "-1.0";
    atmos_tokens["atmos_trop_t11_m_per_deg2"] = "10.0";
    atmos_tokens["atmos_trop_residuals_m"] = makeResidualList(17, 15, 0.2);

    const std::string suffix = ":G01";
    atmos_tokens["atmos_stec_source_subtype" + suffix] = "12";
    atmos_tokens["atmos_stec_type" + suffix] = "3";
    atmos_tokens["atmos_stec_c00_tecu" + suffix] = "1.0";
    atmos_tokens["atmos_stec_c01_tecu_per_deg" + suffix] = "3.0";
    atmos_tokens["atmos_stec_c10_tecu_per_deg" + suffix] = "1.0";
    atmos_tokens["atmos_stec_c11_tecu_per_deg2" + suffix] = "100.0";
    atmos_tokens["atmos_stec_c02_tecu_per_deg2" + suffix] = "200.0";
    atmos_tokens["atmos_stec_c20_tecu_per_deg2" + suffix] = "-150.0";
    atmos_tokens["atmos_stec_residuals_tecu" + suffix] = makeResidualList(17, 15, 0.3);

    const Vector3d receiver_position = geodetic2ecef(33.16 * M_PI / 180.0, 130.16 * M_PI / 180.0, 0.0);
    const GNSSTime time(2400, 345600.0);
    const SatelliteId satellite(GNSSSystem::GPS, 1);

    const double trop_offset_only = ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
        atmos_tokens,
        receiver_position,
        time,
        0.7,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::OFFSET_ONLY,
        SamplingPolicy::INDEXED_OR_MEAN);
    EXPECT_NEAR(trop_offset_only, 0.7, 1e-9);

    const double stec_offset_only = ppp_atmosphere::atmosphericStecTecu(
        atmos_tokens,
        satellite,
        receiver_position,
        ValuePolicy::FULL_COMPOSED,
        Subtype12Policy::OFFSET_ONLY,
        SamplingPolicy::INDEXED_OR_MEAN);
    EXPECT_NEAR(stec_offset_only, 1.3, 1e-9);
}
