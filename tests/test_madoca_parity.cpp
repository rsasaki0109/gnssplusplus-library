#include <gtest/gtest.h>

#include <libgnss++/algorithms/madoca_parity.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/external/madocalib_oracle.hpp>

#include <cmath>
#include <string>
#include <vector>

#ifndef GNSSPP_HAS_MADOCALIB_ORACLE
#define GNSSPP_HAS_MADOCALIB_ORACLE 0
#endif

namespace {

namespace madoca = libgnss::algorithms::madoca_parity;

constexpr double kDegToRad = 3.141592653589793238462643383279502884 / 180.0;
constexpr double kParityTolerance = madoca::kOracleTolerance;

struct GeometrySample {
    double rr[3] = {};
    double pos[3] = {};
    double rs[3] = {};
    double azel[2] = {};
};

void copyVector3(const libgnss::Vector3d& value, double out[3]) {
    out[0] = value(0);
    out[1] = value(1);
    out[2] = value(2);
}

GeometrySample makeSample(int index) {
    GeometrySample sample;
    const double lat = (35.681236 + 0.25 * static_cast<double>(index % 3)) * kDegToRad;
    const double lon = (139.767125 + 0.40 * static_cast<double>(index % 4)) * kDegToRad;
    const double height = 45.0 + 35.0 * static_cast<double>(index % 5);
    sample.pos[0] = lat;
    sample.pos[1] = lon;
    sample.pos[2] = height;

    const libgnss::Vector3d rr = libgnss::geodetic2ecef(lat, lon, height);
    copyVector3(rr, sample.rr);

    const double az = std::fmod((25.0 + 53.0 * static_cast<double>(index)) * kDegToRad,
                                2.0 * 3.141592653589793238462643383279502884);
    const double el = (12.0 + 9.0 * static_cast<double>(index % 7)) * kDegToRad;
    sample.azel[0] = az;
    sample.azel[1] = el;

    const libgnss::Vector3d los_enu(std::sin(az) * std::cos(el),
                                    std::cos(az) * std::cos(el),
                                    std::sin(el));
    const libgnss::Vector3d los_ecef = libgnss::enu2ecef(los_enu, lat, lon).normalized();
    const double range_m = 21300000.0 + 475000.0 * static_cast<double>(index);
    const libgnss::Vector3d rs = rr + range_m * los_ecef;
    copyVector3(rs, sample.rs);
    return sample;
}

std::vector<GeometrySample> makeSamples() {
    std::vector<GeometrySample> samples;
    for (int i = 0; i < 10; ++i) {
        samples.push_back(makeSample(i));
    }
    return samples;
}

void expectNearArray(const std::string& label,
                     const double* native,
                     const double* oracle,
                     int size) {
    for (int i = 0; i < size; ++i) {
        EXPECT_NEAR(native[i], oracle[i], kParityTolerance)
            << label << " component=" << i
            << " native=" << native[i]
            << " oracle=" << oracle[i];
    }
}

TEST(MadocaParityConfig, OracleToleranceIsDefined) {
    EXPECT_DOUBLE_EQ(madoca::kOracleTolerance, 1e-6);
}

#if GNSSPP_HAS_MADOCALIB_ORACLE

class MadocaParity : public ::testing::Test {
protected:
    void SetUp() override {
        if (!libgnss::external::madocalib_oracle::available()) {
            GTEST_SKIP() << "MADOCALIB oracle unavailable; configure with "
                         << "-DMADOCALIB_PARITY_LINK=ON";
        }
    }
};

TEST_F(MadocaParity, SatnoSystemPrnMapping) {
    ASSERT_TRUE(madoca::satnoAvailable());
    struct SatnoCase {
        int sys;
        int prn;
    };
    const SatnoCase cases[] = {
        {madoca::kSysGps, 1},   {madoca::kSysGps, 32},  {madoca::kSysGps, 33},
        {madoca::kSysGlo, 1},   {madoca::kSysGlo, 27},  {madoca::kSysGal, 1},
        {madoca::kSysGal, 36},  {madoca::kSysQzs, 193}, {madoca::kSysQzs, 202},
        {madoca::kSysCmp, 1},   {madoca::kSysCmp, 63},  {madoca::kSysBd2, 18},
        {madoca::kSysBd2, 19},  {madoca::kSysIrn, 1},   {madoca::kSysIrn, 14},
        {madoca::kSysSbs, 120}, {madoca::kSysSbs, 158}, {madoca::kSysLeo, 1},
        {madoca::kSysNone, 1},  {madoca::kSysGps, 0},
    };

    for (const auto& sample : cases) {
        EXPECT_EQ(madoca::satno(sample.sys, sample.prn),
                  libgnss::external::madocalib_oracle::satno(sample.sys, sample.prn))
            << "sys=" << sample.sys << " prn=" << sample.prn;
    }
}

TEST_F(MadocaParity, SatsysSatelliteNumberMapping) {
    ASSERT_TRUE(madoca::satsysAvailable());
    const int sats[] = {
        0, 1, 32, 33, 59, 60, 95, 96, 105, 106, 168, 169, 182, 183, 221, 222,
    };

    for (int sat : sats) {
        int native_prn = -1;
        int oracle_prn = -2;
        const int native_sys = madoca::satsys(sat, &native_prn);
        const int oracle_sys =
            libgnss::external::madocalib_oracle::satsys(sat, &oracle_prn);
        EXPECT_EQ(native_sys, oracle_sys) << "sat=" << sat;
        EXPECT_EQ(native_prn, oracle_prn) << "sat=" << sat;
        EXPECT_EQ(madoca::satsys(sat, nullptr),
                  libgnss::external::madocalib_oracle::satsys(sat, nullptr))
            << "sat=" << sat << " nullptr";
    }
}

TEST_F(MadocaParity, IonmapfSingleLayer) {
    ASSERT_TRUE(madoca::ionmapfAvailable());
    const auto samples = makeSamples();
    for (size_t i = 0; i < samples.size(); ++i) {
        const double native = madoca::ionmapf(samples[i].pos, samples[i].azel);
        const double oracle =
            libgnss::external::madocalib_oracle::ionmapf(samples[i].pos, samples[i].azel);
        EXPECT_NEAR(native, oracle, kParityTolerance) << "sample=" << i;
    }

    const double high_pos[3] = {35.0 * kDegToRad, 139.0 * kDegToRad, 350000.0};
    const double azel[2] = {75.0 * kDegToRad, 40.0 * kDegToRad};
    EXPECT_NEAR(madoca::ionmapf(high_pos, azel),
                libgnss::external::madocalib_oracle::ionmapf(high_pos, azel),
                kParityTolerance);
}

TEST_F(MadocaParity, GeodistSagnacRangeAndLos) {
    ASSERT_TRUE(madoca::geodistAvailable());
    const auto samples = makeSamples();
    for (size_t i = 0; i < samples.size(); ++i) {
        double native_e[3] = {};
        double oracle_e[3] = {};
        const double native = madoca::geodist(samples[i].rs, samples[i].rr, native_e);
        const double oracle =
            libgnss::external::madocalib_oracle::geodist(samples[i].rs, samples[i].rr, oracle_e);
        EXPECT_NEAR(native, oracle, kParityTolerance)
            << "range sample=" << i;
        expectNearArray("los sample=" + std::to_string(i), native_e, oracle_e, 3);
    }
}

#else

TEST(MadocaParityDefault, OracleDisabledInDefaultBuild) {
    EXPECT_FALSE(libgnss::external::madocalib_oracle::available());
}

#endif

}  // namespace
