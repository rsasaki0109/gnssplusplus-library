#include <gtest/gtest.h>

#include <libgnss++/algorithms/madoca_parity.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/external/madocalib_oracle.hpp>

#include <cmath>
#include <iterator>
#include <string>
#include <vector>

#ifndef GNSSPP_HAS_MADOCALIB_ORACLE
#define GNSSPP_HAS_MADOCALIB_ORACLE 0
#endif

namespace {

namespace madoca = libgnss::algorithms::madoca_parity;

constexpr double kDegToRad = 3.141592653589793238462643383279502884 / 180.0;
constexpr double kParityTolerance = madoca::kOracleTolerance;
constexpr double kClockTolerance = 1e-12;

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

madoca::GTime makeTime(int year, int mon, int day, int hour, int min, double second) {
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    madoca::GTime time = {};
    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) {
        return time;
    }
    const int days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] +
                     day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
    const int sec = static_cast<int>(std::floor(second));
    time.time = static_cast<decltype(time.time)>(days) * 86400 + hour * 3600 +
                min * 60 + sec;
    time.sec = second - sec;
    return time;
}

madoca::GTime addSeconds(madoca::GTime time, double seconds) {
    time.sec += seconds;
    const double whole = std::floor(time.sec);
    time.time += static_cast<decltype(time.time)>(whole);
    time.sec -= whole;
    return time;
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

madoca::AntennaPcv makePcv() {
    madoca::AntennaPcv pcv;
    for (int i = 0; i < madoca::kMadocalibNFreqPcv; ++i) {
        for (int j = 0; j < 3; ++j) {
            pcv.off[i][j] =
                0.0125 * static_cast<double>(i + 1) +
                0.004 * static_cast<double>(j + 1) -
                0.0007 * static_cast<double>((i + j) % 3);
        }
        for (int j = 0; j < madoca::kMadocalibPcvAngles; ++j) {
            pcv.var[i][j] =
                0.0008 * static_cast<double>(i + 1) +
                0.00013 * static_cast<double>(j) +
                0.00001 * static_cast<double>((i * j) % 5);
        }
    }
    return pcv;
}

madoca::BroadcastEphemeris makeEph(int sys, int prn, int index) {
    madoca::BroadcastEphemeris eph;
    eph.sat = madoca::satno(sys, prn);
    eph.iode = 10 + index;
    eph.iodc = 20 + index;
    eph.sva = sys == madoca::kSysGal ? 65 + index : 2 + index;
    eph.svh = 0;
    eph.week = 2400;
    eph.code = sys == madoca::kSysGal ? (1 << 9) : 0;
    eph.flag = 0;
    eph.toe = makeTime(2026, 4, 19, 12, 0, 0.0 + 600.0 * index);
    eph.toc = addSeconds(eph.toe, -120.0 + 15.0 * index);
    eph.ttr = addSeconds(eph.toe, 30.0);
    eph.A = (sys == madoca::kSysCmp && prn <= 5) ? 42164000.0 : 26560000.0;
    if (sys == madoca::kSysGal) {
        eph.A = 29601000.0;
    } else if (sys == madoca::kSysCmp && prn > 5) {
        eph.A = 27906100.0;
    }
    eph.e = 0.008 + 0.001 * static_cast<double>(index);
    eph.i0 = (sys == madoca::kSysCmp && prn <= 5 ? 4.5 : 55.0 + index) * kDegToRad;
    eph.OMG0 = (45.0 + 11.0 * static_cast<double>(index)) * kDegToRad;
    eph.omg = (15.0 + 7.0 * static_cast<double>(index)) * kDegToRad;
    eph.M0 = (5.0 + 19.0 * static_cast<double>(index)) * kDegToRad;
    eph.deln = (3.5 + static_cast<double>(index)) * 1E-9;
    eph.OMGd = (-8.0 - 0.4 * static_cast<double>(index)) * 1E-9;
    eph.idot = (0.2 + 0.03 * static_cast<double>(index)) * 1E-10;
    eph.crc = 180.0 + 8.0 * static_cast<double>(index);
    eph.crs = -70.0 + 4.0 * static_cast<double>(index);
    eph.cuc = (0.9 + 0.1 * static_cast<double>(index)) * 1E-6;
    eph.cus = (1.7 + 0.08 * static_cast<double>(index)) * 1E-6;
    eph.cic = (-0.5 + 0.03 * static_cast<double>(index)) * 1E-6;
    eph.cis = (0.7 + 0.02 * static_cast<double>(index)) * 1E-6;
    eph.toes = 302400.0 + 600.0 * static_cast<double>(index);
    eph.fit = 4.0;
    eph.f0 = (2.0 + 0.3 * static_cast<double>(index)) * 1E-4;
    eph.f1 = (-4.0 + 0.2 * static_cast<double>(index)) * 1E-12;
    eph.f2 = (1.0 + 0.1 * static_cast<double>(index)) * 1E-18;
    return eph;
}

madoca::MionoAreaFixture makeMionoArea(int region_id,
                                       int area_number,
                                       int sid,
                                       double ref_lat_deg,
                                       double ref_lon_deg,
                                       double span0,
                                       double span1,
                                       int type) {
    madoca::MionoAreaFixture area;
    area.region_id = region_id;
    area.area_number = area_number;
    area.rvalid = 1;
    area.ralert = 0;
    area.avalid = 1;
    area.sid = sid;
    area.type = type;
    area.ref[0] = ref_lat_deg;
    area.ref[1] = ref_lon_deg;
    area.span[0] = span0;
    area.span[1] = span1;
    return area;
}

void setMionoSat(madoca::MionoAreaFixture& area,
                 int sat,
                 madoca::GTime time,
                 int sqi,
                 const double coef[6]) {
    area.sat[sat - 1].t0 = time;
    area.sat[sat - 1].sqi = sqi;
    for (int i = 0; i < 6; ++i) {
        area.sat[sat - 1].coef[i] = coef[i];
    }
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

void expectNearMionoSat(const std::string& label,
                        const madoca::MionoCorrResult& native,
                        const madoca::MionoCorrResult& oracle,
                        int sat) {
    const int idx = sat - 1;
    EXPECT_EQ(native.t0[idx].time, oracle.t0[idx].time) << label;
    EXPECT_NEAR(native.t0[idx].sec, oracle.t0[idx].sec, kClockTolerance) << label;
    EXPECT_NEAR(native.delay[idx], oracle.delay[idx], kParityTolerance)
        << label << " delay";
    EXPECT_NEAR(native.std[idx], oracle.std[idx], kParityTolerance)
        << label << " std";
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

TEST_F(MadocaParity, TropmodelSaastamoinenDelay) {
    ASSERT_TRUE(madoca::tropmodelAvailable());
    const auto samples = makeSamples();
    const madoca::GTime time = makeTime(2026, 4, 20, 6, 30, 15.25);
    for (size_t i = 0; i < samples.size(); ++i) {
        const double humi = 0.15 + 0.08 * static_cast<double>(i % 7);
        const double native = madoca::tropmodel(time, samples[i].pos, samples[i].azel, humi);
        const double oracle = libgnss::external::madocalib_oracle::tropmodel(
            time, samples[i].pos, samples[i].azel, humi);
        EXPECT_NEAR(native, oracle, kParityTolerance) << "sample=" << i;
    }

    const double low_pos[3] = {35.0 * kDegToRad, 139.0 * kDegToRad, -101.0};
    const double azel[2] = {30.0 * kDegToRad, 20.0 * kDegToRad};
    EXPECT_NEAR(madoca::tropmodel(time, low_pos, azel, 0.5),
                libgnss::external::madocalib_oracle::tropmodel(time, low_pos, azel, 0.5),
                kParityTolerance);
}

TEST_F(MadocaParity, TropmapfNiellMappingFunction) {
    ASSERT_TRUE(madoca::tropmapfAvailable());
    const auto samples = makeSamples();
    const madoca::GTime times[] = {
        makeTime(2026, 1, 15, 0, 0, 0.0),
        makeTime(2026, 4, 20, 12, 30, 0.0),
        makeTime(2026, 8, 1, 23, 45, 30.0),
    };
    for (size_t i = 0; i < samples.size(); ++i) {
        double native_mapfw = -1.0;
        double oracle_mapfw = -2.0;
        const madoca::GTime time = times[i % 3];
        const double native =
            madoca::tropmapf(time, samples[i].pos, samples[i].azel, &native_mapfw);
        const double oracle = libgnss::external::madocalib_oracle::tropmapf(
            time, samples[i].pos, samples[i].azel, &oracle_mapfw);
        EXPECT_NEAR(native, oracle, kParityTolerance) << "dry sample=" << i;
        EXPECT_NEAR(native_mapfw, oracle_mapfw, kParityTolerance)
            << "wet sample=" << i;
        EXPECT_NEAR(madoca::tropmapf(time, samples[i].pos, samples[i].azel, nullptr),
                    libgnss::external::madocalib_oracle::tropmapf(
                        time, samples[i].pos, samples[i].azel, nullptr),
                    kParityTolerance)
            << "nullptr sample=" << i;
    }
}

TEST_F(MadocaParity, AntmodelReceiverPcoPcv) {
    ASSERT_TRUE(madoca::antmodelAvailable());
    const auto samples = makeSamples();
    const madoca::AntennaPcv pcv = makePcv();
    const double del[3] = {0.012, -0.025, 0.081};

    for (size_t i = 0; i < samples.size(); ++i) {
        for (int opt : {0, 1}) {
            double native[madoca::kMadocalibNFreqPcv] = {};
            double oracle[madoca::kMadocalibNFreqPcv] = {};
            madoca::antmodel(&pcv, del, samples[i].azel, opt, native);
            libgnss::external::madocalib_oracle::antmodel(
                &pcv, del, samples[i].azel, opt, oracle);
            expectNearArray("antmodel sample=" + std::to_string(i) +
                                " opt=" + std::to_string(opt),
                            native,
                            oracle,
                            madoca::kMadocalibNFreqPcv);
        }
    }
}

TEST_F(MadocaParity, AntmodelSatellitePcv) {
    ASSERT_TRUE(madoca::antmodelSAvailable());
    const madoca::AntennaPcv pcv = makePcv();
    const double nadirs[] = {0.0, 1.0 * kDegToRad, 4.25 * kDegToRad, 9.1 * kDegToRad};
    for (double nadir : nadirs) {
        double native[madoca::kMadocalibNFreqPcv] = {};
        double oracle[madoca::kMadocalibNFreqPcv] = {};
        madoca::antmodel_s(&pcv, nadir, native);
        libgnss::external::madocalib_oracle::antmodel_s(&pcv, nadir, oracle);
        expectNearArray("antmodel_s nadir=" + std::to_string(nadir),
                        native,
                        oracle,
                        madoca::kMadocalibNFreqPcv);
    }
}

TEST_F(MadocaParity, Eph2clkBroadcastClock) {
    ASSERT_TRUE(madoca::eph2clkAvailable());
    const madoca::BroadcastEphemeris ephs[] = {
        makeEph(madoca::kSysGps, 3, 0),
        makeEph(madoca::kSysGal, 11, 1),
        makeEph(madoca::kSysCmp, 3, 2),
        makeEph(madoca::kSysCmp, 20, 3),
    };
    for (size_t i = 0; i < std::size(ephs); ++i) {
        for (double dt : {-360.0, 0.0, 540.0, 1800.0}) {
            const madoca::GTime time = addSeconds(ephs[i].toc, dt);
            const double native = madoca::eph2clk(time, &ephs[i]);
            const double oracle = libgnss::external::madocalib_oracle::eph2clk(time, &ephs[i]);
            EXPECT_NEAR(native, oracle, kClockTolerance)
                << "eph=" << i << " dt=" << dt;
        }
    }
}

TEST_F(MadocaParity, Eph2posBroadcastKeplerEcef) {
    ASSERT_TRUE(madoca::eph2posAvailable());
    const madoca::BroadcastEphemeris ephs[] = {
        makeEph(madoca::kSysGps, 3, 0),
        makeEph(madoca::kSysGal, 11, 1),
        makeEph(madoca::kSysCmp, 3, 2),
        makeEph(madoca::kSysCmp, 20, 3),
    };
    for (size_t i = 0; i < std::size(ephs); ++i) {
        for (double dt : {60.0, 900.0, 2400.0}) {
            const madoca::GTime time = addSeconds(ephs[i].toe, dt);
            double native_rs[3] = {};
            double oracle_rs[3] = {};
            double native_dts = 0.0;
            double oracle_dts = 0.0;
            double native_var = 0.0;
            double oracle_var = 0.0;
            madoca::eph2pos(time, &ephs[i], native_rs, &native_dts, &native_var);
            libgnss::external::madocalib_oracle::eph2pos(
                time, &ephs[i], oracle_rs, &oracle_dts, &oracle_var);
            expectNearArray("eph2pos rs eph=" + std::to_string(i) +
                                " dt=" + std::to_string(dt),
                            native_rs,
                            oracle_rs,
                            3);
            EXPECT_NEAR(native_dts, oracle_dts, kClockTolerance)
                << "dts eph=" << i << " dt=" << dt;
            EXPECT_NEAR(native_var, oracle_var, kParityTolerance)
                << "var eph=" << i << " dt=" << dt;
        }
    }
}

TEST_F(MadocaParity, McssrBiasCodeSelection) {
    ASSERT_TRUE(madoca::mcssrSelBiascodeAvailable());
    struct BiasCase {
        int sys;
        int code;
    };
    const BiasCase cases[] = {
        {madoca::kSysGps, madoca::kCodeL1C},
        {madoca::kSysGps, madoca::kCodeL1P},
        {madoca::kSysGps, madoca::kCodeL1L},
        {madoca::kSysGps, madoca::kCodeL2S},
        {madoca::kSysGps, madoca::kCodeL5Q},
        {madoca::kSysGlo, madoca::kCodeL1P},
        {madoca::kSysGlo, madoca::kCodeL2P},
        {madoca::kSysGal, madoca::kCodeL1B},
        {madoca::kSysGal, madoca::kCodeL6X},
        {madoca::kSysQzs, madoca::kCodeL1L},
        {madoca::kSysQzs, madoca::kCodeL5I},
        {madoca::kSysQzs, madoca::kCodeL1E},
        {madoca::kSysCmp, madoca::kCodeL2X},
        {madoca::kSysCmp, madoca::kCodeL6Q},
        {madoca::kSysCmp, madoca::kCodeL7D},
        {madoca::kSysBd2, madoca::kCodeL2I},
        {madoca::kSysGps, madoca::kCodeL7D},
    };

    for (const auto& sample : cases) {
        EXPECT_EQ(madoca::mcssr_sel_biascode(sample.sys, sample.code),
                  libgnss::external::madocalib_oracle::mcssr_sel_biascode(
                      sample.sys, sample.code))
            << "sys=" << sample.sys << " code=" << sample.code;
    }
}

TEST_F(MadocaParity, MionoRectangleCorrectionPolynomial) {
    ASSERT_TRUE(madoca::mionoGetCorrAvailable());
    const auto sample = makeSample(0);
    const madoca::GTime time = makeTime(2026, 4, 20, 7, 0, 15.0);
    std::vector<madoca::MionoAreaFixture> areas;
    areas.push_back(makeMionoArea(11, 4, 0, 35.65, 139.75, 0.20, 0.30, 3));

    const int sat = madoca::satno(madoca::kSysGps, 5);
    const double coef[6] = {12.5, 0.35, -0.22, 0.015, 0.006, 99.0};
    setMionoSat(areas[0], sat, time, 9, coef);

    madoca::MionoCorrResult native;
    madoca::MionoCorrResult oracle;
    const int native_status =
        madoca::miono_get_corr(time, sample.rr, areas.data(), areas.size(), &native);
    const int oracle_status = libgnss::external::madocalib_oracle::miono_get_corr(
        time, sample.rr, areas.data(), areas.size(), &oracle);

    ASSERT_EQ(native_status, oracle_status);
    ASSERT_EQ(native_status, 1);
    EXPECT_EQ(native.rid, oracle.rid);
    EXPECT_EQ(native.area_number, oracle.area_number);
    expectNearMionoSat("rectangle gps", native, oracle, sat);
}

TEST_F(MadocaParity, MionoCircleSelectsNearestArea) {
    ASSERT_TRUE(madoca::mionoGetCorrAvailable());
    const auto sample = makeSample(2);
    const double lat_deg = sample.pos[0] / kDegToRad;
    const double lon_deg = sample.pos[1] / kDegToRad;
    const madoca::GTime time = makeTime(2026, 4, 20, 7, 10, 30.0);

    std::vector<madoca::MionoAreaFixture> areas;
    areas.push_back(makeMionoArea(20, 1, 1, lat_deg + 1.0, lon_deg + 1.0, 1000.0, 0.0, 2));
    areas.push_back(makeMionoArea(21, 2, 1, lat_deg + 0.02, lon_deg - 0.01, 1000.0, 0.0, 2));

    const int sat = madoca::satno(madoca::kSysGal, 11);
    const double far_coef[6] = {30.0, 0.10, 0.20, -0.010, 0.0, 0.0};
    const double near_coef[6] = {8.0, -0.30, 0.40, 0.025, 0.0, 0.0};
    setMionoSat(areas[0], sat, time, 17, far_coef);
    setMionoSat(areas[1], sat, time, 17, near_coef);

    madoca::MionoCorrResult native;
    madoca::MionoCorrResult oracle;
    const int native_status =
        madoca::miono_get_corr(time, sample.rr, areas.data(), areas.size(), &native);
    const int oracle_status = libgnss::external::madocalib_oracle::miono_get_corr(
        time, sample.rr, areas.data(), areas.size(), &oracle);

    ASSERT_EQ(native_status, oracle_status);
    ASSERT_EQ(native_status, 1);
    EXPECT_EQ(native.rid, 21);
    EXPECT_EQ(native.area_number, 2);
    EXPECT_EQ(native.rid, oracle.rid);
    EXPECT_EQ(native.area_number, oracle.area_number);
    expectNearMionoSat("nearest circle galileo", native, oracle, sat);
}

TEST_F(MadocaParity, MionoQualityIndicatorStdBoundaries) {
    ASSERT_TRUE(madoca::mionoGetCorrAvailable());
    const auto sample = makeSample(1);
    const madoca::GTime time = makeTime(2026, 4, 20, 7, 20, 45.0);
    std::vector<madoca::MionoAreaFixture> areas;
    areas.push_back(makeMionoArea(33, 7, 0, 35.9, 140.1, 1.0, 1.0, 0));

    const int gps_sat = madoca::satno(madoca::kSysGps, 1);
    const int gal_sat = madoca::satno(madoca::kSysGal, 2);
    const int qzs_sat = madoca::satno(madoca::kSysQzs, 193);
    const double gps_coef[6] = {4.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const double gal_coef[6] = {5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const double qzs_coef[6] = {6.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    setMionoSat(areas[0], gps_sat, time, 0, gps_coef);
    setMionoSat(areas[0], gal_sat, time, 63, gal_coef);
    setMionoSat(areas[0], qzs_sat, time, 9, qzs_coef);

    madoca::MionoCorrResult native;
    madoca::MionoCorrResult oracle;
    const int native_status =
        madoca::miono_get_corr(time, sample.rr, areas.data(), areas.size(), &native);
    const int oracle_status = libgnss::external::madocalib_oracle::miono_get_corr(
        time, sample.rr, areas.data(), areas.size(), &oracle);

    ASSERT_EQ(native_status, oracle_status);
    ASSERT_EQ(native_status, 1);
    expectNearMionoSat("sqi gps undef", native, oracle, gps_sat);
    expectNearMionoSat("sqi gal max", native, oracle, gal_sat);
    expectNearMionoSat("sqi qzs", native, oracle, qzs_sat);
    EXPECT_NEAR(native.std[gps_sat - 1], 5.4665, kParityTolerance);
    EXPECT_NEAR(native.std[gal_sat - 1], 5.4665, kParityTolerance);
    EXPECT_NEAR(native.std[qzs_sat - 1], 0.00275, kParityTolerance);
}

TEST_F(MadocaParity, SatposBroadcastPositionVelocity) {
    ASSERT_TRUE(madoca::satposAvailable());
    const std::vector<madoca::BroadcastEphemeris> ephs = {
        makeEph(madoca::kSysGps, 3, 0),
        makeEph(madoca::kSysGal, 11, 1),
        makeEph(madoca::kSysCmp, 20, 3),
    };

    for (size_t i = 0; i < ephs.size(); ++i) {
        const madoca::GTime time = addSeconds(ephs[i].toe, 900.0);
        double native_rs[6] = {};
        double oracle_rs[6] = {};
        double native_dts[2] = {};
        double oracle_dts[2] = {};
        double native_var = 0.0;
        double oracle_var = 0.0;
        int native_svh = -2;
        int oracle_svh = -3;

        const int native_status = madoca::satpos(time,
                                                 time,
                                                 ephs[i].sat,
                                                 madoca::kEphOptBrdc,
                                                 ephs.data(),
                                                 ephs.size(),
                                                 native_rs,
                                                 native_dts,
                                                 &native_var,
                                                 &native_svh);
        const int oracle_status = libgnss::external::madocalib_oracle::satpos(
            time,
            time,
            ephs[i].sat,
            madoca::kEphOptBrdc,
            ephs.data(),
            ephs.size(),
            oracle_rs,
            oracle_dts,
            &oracle_var,
            &oracle_svh);

        ASSERT_EQ(native_status, oracle_status) << "eph=" << i;
        ASSERT_EQ(native_status, 1) << "eph=" << i;
        EXPECT_EQ(native_svh, oracle_svh) << "eph=" << i;
        expectNearArray("satpos rs eph=" + std::to_string(i), native_rs, oracle_rs, 6);
        EXPECT_NEAR(native_dts[0], oracle_dts[0], kClockTolerance)
            << "dts bias eph=" << i;
        EXPECT_NEAR(native_dts[1], oracle_dts[1], kClockTolerance)
            << "dts drift eph=" << i;
        EXPECT_NEAR(native_var, oracle_var, kParityTolerance) << "var eph=" << i;
    }
}

#else

TEST(MadocaParityDefault, OracleDisabledInDefaultBuild) {
    EXPECT_FALSE(libgnss::external::madocalib_oracle::available());
}

#endif

}  // namespace
