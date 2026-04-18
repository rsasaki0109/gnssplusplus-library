#include <gtest/gtest.h>

#include <libgnss++/algorithms/clasnat_parity.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/external/claslib_oracle.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace {

using libgnss::GNSSTime;
using libgnss::Vector3d;

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kArcsecToRad = kDegToRad / 3600.0;
constexpr double kParityTolerance = 1e-6;

struct GeometrySample {
    GNSSTime time;
    double rr[3] = {};
    double pos[3] = {};
    double rs[6] = {};
    double azel[2] = {};
};

Vector3d receiverEcef() {
    return libgnss::geodetic2ecef(36.1037748 * kDegToRad,
                                  140.0878550 * kDegToRad,
                                  67.0);
}

void copyVector3(const Vector3d& v, double out[3]) {
    out[0] = v(0);
    out[1] = v(1);
    out[2] = v(2);
}

GeometrySample makeSample(int epoch_index, int sat_index) {
    GeometrySample sample;
    sample.time = GNSSTime(2029, 230400.0 + epoch_index * 30.0);
    const double lat = 36.1037748 * kDegToRad;
    const double lon = 140.0878550 * kDegToRad;
    sample.pos[0] = lat;
    sample.pos[1] = lon;
    sample.pos[2] = 67.0;

    const Vector3d rr = receiverEcef();
    copyVector3(rr, sample.rr);

    const double az = std::fmod((35.0 + sat_index * 67.0 + epoch_index * 4.0) * kDegToRad,
                                2.0 * M_PI);
    const double el = (18.0 + sat_index * 14.0 + epoch_index * 1.5) * kDegToRad;
    sample.azel[0] = az;
    sample.azel[1] = el;

    const Vector3d los_enu(std::sin(az) * std::cos(el),
                           std::cos(az) * std::cos(el),
                           std::sin(el));
    const Vector3d los_ecef = libgnss::enu2ecef(los_enu, lat, lon).normalized();
    const double range_m = 23200000.0 + sat_index * 850000.0 + epoch_index * 25000.0;
    const Vector3d sat_pos = rr + range_m * los_ecef;
    sample.rs[0] = sat_pos(0);
    sample.rs[1] = sat_pos(1);
    sample.rs[2] = sat_pos(2);

    const Vector3d east = libgnss::enu2ecef(Vector3d(1.0, 0.0, 0.0), lat, lon).normalized();
    const Vector3d north = libgnss::enu2ecef(Vector3d(0.0, 1.0, 0.0), lat, lon).normalized();
    const Vector3d up = libgnss::enu2ecef(Vector3d(0.0, 0.0, 1.0), lat, lon).normalized();
    const Vector3d sat_vel =
        2800.0 * (std::cos(az) * east - std::sin(az) * north) +
        (35.0 + sat_index * 7.0) * up;
    sample.rs[3] = sat_vel(0);
    sample.rs[4] = sat_vel(1);
    sample.rs[5] = sat_vel(2);

    return sample;
}

std::vector<GeometrySample> makeSamples() {
    std::vector<GeometrySample> samples;
    for (int epoch = 0; epoch < 5; ++epoch) {
        for (int sat = 0; sat < 4; ++sat) {
            samples.push_back(makeSample(epoch, sat));
        }
    }
    return samples;
}

libgnss::clasnat_parity::ReceiverPcvModel makePcvModel() {
    libgnss::clasnat_parity::ReceiverPcvModel pcv;
    for (int f = 0; f < libgnss::clasnat_parity::kParityMaxFreq; ++f) {
        pcv.offsets_m[static_cast<size_t>(f)] = {
            0.0015 * (f + 1),
            -0.0020 * (f + 1),
            0.0450 + 0.0040 * f,
        };
        for (int i = 0; i < libgnss::clasnat_parity::kParityPcvGridSize; ++i) {
            pcv.variations_m[static_cast<size_t>(f)][static_cast<size_t>(i)] =
                0.00015 * (f + 1) + 0.000025 * i + 0.000003 * f * i;
        }
    }
    return pcv;
}

void expectNearArray(const std::string& label,
                     const double* native,
                     const double* oracle,
                     int n) {
    for (int i = 0; i < n; ++i) {
        EXPECT_NEAR(native[i], oracle[i], kParityTolerance)
            << label << " component=" << i
            << " native=" << native[i]
            << " oracle=" << oracle[i];
    }
}

class ClasnatParity : public ::testing::Test {
protected:
    void SetUp() override {
        if (!libgnss::external::claslib_oracle::available()) {
            GTEST_SKIP() << "CLASLIB oracle unavailable; configure with "
                         << "-DCLASLIB_PARITY_LINK=ON";
        }
    }
};

TEST_F(ClasnatParity, TidedispSolidOtlPoleEcef) {
    ASSERT_TRUE(libgnss::clasnat_parity::tidedispAvailable());
    const double erpv[5] = {
        0.073 * kArcsecToRad,
        0.358 * kArcsecToRad,
        -0.126,
        0.0,
        0.0,
    };
    const auto samples = makeSamples();
    for (size_t i = 0; i < samples.size(); i += 4) {
        double native[3] = {};
        double oracle[3] = {};
        libgnss::clasnat_parity::tidedisp(samples[i].time, samples[i].rr, erpv, native);
        libgnss::external::claslib_oracle::tidedisp(
            samples[i].time, samples[i].rr, erpv, oracle);
        expectNearArray("tidedisp sample=" + std::to_string(i / 4), native, oracle, 3);
    }
}

TEST_F(ClasnatParity, WindupcorrCycles) {
    ASSERT_TRUE(libgnss::clasnat_parity::windupcorrAvailable());
    const auto samples = makeSamples();
    double native_phw = 0.125;
    double oracle_phw = 0.125;
    for (size_t i = 0; i < samples.size(); ++i) {
        libgnss::clasnat_parity::windupcorr(
            samples[i].time, samples[i].rs, samples[i].rr, native_phw);
        libgnss::external::claslib_oracle::windupcorr(
            samples[i].time, samples[i].rs, samples[i].rr, oracle_phw);
        EXPECT_NEAR(native_phw, oracle_phw, kParityTolerance)
            << "windupcorr sample=" << i;
    }
}

TEST_F(ClasnatParity, AntmodelReceiverPcoPcv) {
    ASSERT_TRUE(libgnss::clasnat_parity::antmodelAvailable());
    const auto pcv = makePcvModel();
    const double del[3] = {0.0025, -0.0010, 0.0140};
    const auto samples = makeSamples();
    for (size_t i = 0; i < samples.size(); ++i) {
        double native[libgnss::clasnat_parity::kParityMaxFreq] = {};
        double oracle[libgnss::clasnat_parity::kParityMaxFreq] = {};
        libgnss::clasnat_parity::antmodel(pcv, del, samples[i].azel, 1, native);
        libgnss::external::claslib_oracle::antmodel(pcv, del, samples[i].azel, 1, oracle);
        expectNearArray("antmodel sample=" + std::to_string(i),
                        native,
                        oracle,
                        libgnss::clasnat_parity::kParityMaxFreq);
    }
}

TEST_F(ClasnatParity, IonmapfSingleLayer) {
    ASSERT_TRUE(libgnss::clasnat_parity::ionmapfAvailable());
    const auto samples = makeSamples();
    for (size_t i = 0; i < samples.size(); ++i) {
        const double native = libgnss::clasnat_parity::ionmapf(
            samples[i].pos, samples[i].azel);
        const double oracle = libgnss::external::claslib_oracle::ionmapf(
            samples[i].pos, samples[i].azel);
        EXPECT_NEAR(native, oracle, kParityTolerance) << "ionmapf sample=" << i;
    }
}

TEST_F(ClasnatParity, PrectropModeledSlant) {
    ASSERT_TRUE(libgnss::clasnat_parity::prectropAvailable());
    const auto samples = makeSamples();
    for (size_t i = 0; i < samples.size(); ++i) {
        const double zwd = 0.92 + 0.01 * static_cast<double>(i % 4);
        const double ztd = 1.04 + 0.005 * static_cast<double>(i / 4);
        const double native = libgnss::clasnat_parity::prectrop(
            samples[i].time, samples[i].pos, samples[i].azel, zwd, ztd);
        const double oracle = libgnss::external::claslib_oracle::prectrop(
            samples[i].time, samples[i].pos, samples[i].azel, zwd, ztd);
        EXPECT_NEAR(native, oracle, kParityTolerance) << "prectrop sample=" << i;
    }
}

TEST_F(ClasnatParity, SatposSsrNativeTargetMissing) {
    libgnss::clasnat_parity::SatposSsrOutput native;
    libgnss::clasnat_parity::SatposSsrOutput oracle;
    const bool native_ok = libgnss::clasnat_parity::satpos_ssr(
        GNSSTime(2029, 230400.0), GNSSTime(2029, 230400.0), 1, native);
    const bool oracle_ok = libgnss::external::claslib_oracle::satpos_ssr(
        GNSSTime(2029, 230400.0), GNSSTime(2029, 230400.0), 1, oracle);
    EXPECT_TRUE(native_ok)
        << "native satpos_ssr parity entry point is missing; iter41 port target";
    EXPECT_TRUE(oracle_ok)
        << "CLASLIB satpos_ssr oracle adapter is missing realistic nav/SSR fixture wiring";
}

TEST_F(ClasnatParity, CorrmeasNativeTargetMissing) {
    libgnss::clasnat_parity::CorrmeasOutput native;
    libgnss::clasnat_parity::CorrmeasOutput oracle;
    const bool native_ok = libgnss::clasnat_parity::corrmeas(native);
    const bool oracle_ok = libgnss::external::claslib_oracle::corrmeas(oracle);
    EXPECT_TRUE(native_ok)
        << "native corrmeas parity entry point is missing; iter41 port target";
    EXPECT_TRUE(oracle_ok)
        << "CLASLIB corrmeas oracle adapter is missing realistic CSSR fixture wiring";
}

}  // namespace
