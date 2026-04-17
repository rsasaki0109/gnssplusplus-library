#include <libgnss++/algorithms/ppp_claslib_pntpos.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

#ifdef LAPACK
extern "C" {
void dgemm_(char* transa,
            char* transb,
            int* m,
            int* n,
            int* k,
            double* alpha,
            double* a,
            int* lda,
            double* b,
            int* ldb,
            double* beta,
            double* c,
            int* ldc);
void dgetrf_(int* m, int* n, double* a, int* lda, int* ipiv, int* info);
void dgetri_(int* n,
             double* a,
             int* lda,
             int* ipiv,
             double* work,
             int* lwork,
             int* info);
}
#endif

namespace libgnss::ppp_claslib_pntpos {

namespace {

constexpr int kNx = 7;
constexpr int kMaxItr = 10;
constexpr double kErrIon = 5.0;
constexpr double kErrTrop = 3.0;
constexpr double kErrSaas = 0.3;
constexpr double kErrBrdcI = 0.5;
constexpr double kErrCodeBias = 0.3;
constexpr double kRelHumi = 0.7;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRtolKepler = 1e-13;

constexpr std::array<double, 100> kChiSquare001 = {
    10.8, 13.8, 16.3, 18.5, 20.5, 22.5, 24.3, 26.1, 27.9, 29.6,
    31.3, 32.9, 34.5, 36.1, 37.7, 39.3, 40.8, 42.3, 43.8, 45.3,
    46.8, 48.3, 49.7, 51.2, 52.6, 54.1, 55.5, 56.9, 58.3, 59.7,
    61.1, 62.5, 63.9, 65.2, 66.6, 68.0, 69.3, 70.7, 72.1, 73.4,
    74.7, 76.0, 77.3, 78.6, 80.0, 81.3, 82.6, 84.0, 85.4, 86.7,
    88.0, 89.3, 90.6, 91.9, 93.3, 94.7, 96.0, 97.4, 98.7, 100.0,
    101.0, 102.0, 103.0, 104.0, 105.0, 107.0, 108.0, 109.0, 110.0, 112.0,
    113.0, 114.0, 115.0, 116.0, 118.0, 119.0, 120.0, 122.0, 123.0, 125.0,
    126.0, 127.0, 128.0, 129.0, 131.0, 132.0, 133.0, 134.0, 135.0, 137.0,
    138.0, 139.0, 140.0, 142.0, 143.0, 144.0, 145.0, 147.0, 148.0, 149.0};

struct EffectiveOptions {
    bool iono_free = true;
    bool trop_saas = true;
};

struct PntObservation {
    SatelliteId satellite;
    int satno = 0;
    std::array<double, 3> P = {};
    std::array<double, 3> D = {};
    std::array<double, 3> SNR = {};
    std::array<SignalType, 3> signals = {
        SignalType::SIGNAL_TYPE_COUNT,
        SignalType::SIGNAL_TYPE_COUNT,
        SignalType::SIGNAL_TYPE_COUNT};
    std::array<const Observation*, 3> source = {nullptr, nullptr, nullptr};
    const Ephemeris* eph = nullptr;
    Vector3d rs = Vector3d::Zero();
    Vector3d vs = Vector3d::Zero();
    std::array<double, 2> dts = {};
    double vare = 0.0;
    int svh = -1;
    bool has_satpos = false;
};

struct EstimationState {
    PntposResult result;
    VectorXd weighted_residuals;
    MatrixXd covariance = MatrixXd::Zero(kNx, kNx);
    int nv = 0;
};

bool shouldDumpPntpos(const GNSSTime& time);

void claslibMatmul(const char* tr,
                   int n,
                   int k,
                   int m,
                   double alpha,
                   const double* A,
                   const double* B,
                   double beta,
                   double* C) {
#ifdef LAPACK
    int lda = tr[0] == 'T' ? m : n;
    int ldb = tr[1] == 'T' ? k : m;
    char transa = tr[0];
    char transb = tr[1];
    dgemm_(&transa,
           &transb,
           &n,
           &k,
           &m,
           &alpha,
           const_cast<double*>(A),
           &lda,
           const_cast<double*>(B),
           &ldb,
           &beta,
           C,
           &n);
#else
    const int f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2)
                               : (tr[1] == 'N' ? 3 : 4);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < k; ++j) {
            double d = 0.0;
            switch (f) {
                case 1:
                    for (int x = 0; x < m; ++x) {
                        if (A[i + x * n] == 0.0 || B[x + j * m] == 0.0) {
                            continue;
                        }
                        d += A[i + x * n] * B[x + j * m];
                    }
                    break;
                case 2:
                    for (int x = 0; x < m; ++x) {
                        if (A[i + x * n] == 0.0 || B[j + x * k] == 0.0) {
                            continue;
                        }
                        d += A[i + x * n] * B[j + x * k];
                    }
                    break;
                case 3:
                    for (int x = 0; x < m; ++x) {
                        if (A[x + i * m] == 0.0 || B[x + j * m] == 0.0) {
                            continue;
                        }
                        d += A[x + i * m] * B[x + j * m];
                    }
                    break;
                case 4:
                    for (int x = 0; x < m; ++x) {
                        if (A[x + i * m] == 0.0 || B[j + x * k] == 0.0) {
                            continue;
                        }
                        d += A[x + i * m] * B[j + x * k];
                    }
                    break;
            }
            C[i + j * n] =
                beta == 0.0 ? alpha * d : alpha * d + beta * C[i + j * n];
        }
    }
#endif
}

#ifndef LAPACK
int claslibLudcmp(double* A, int n, int* indx, double* d) {
    std::vector<double> vv(static_cast<size_t>(n), 0.0);
    int imax = 0;
    *d = 1.0;
    for (int i = 0; i < n; ++i) {
        double big = 0.0;
        for (int j = 0; j < n; ++j) {
            const double tmp = std::fabs(A[i + j * n]);
            if (tmp > big) {
                big = tmp;
            }
        }
        if (big > 0.0) {
            vv[static_cast<size_t>(i)] = 1.0 / big;
        } else {
            return -1;
        }
    }
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < j; ++i) {
            double s = A[i + j * n];
            for (int k = 0; k < i; ++k) {
                s -= A[i + k * n] * A[k + j * n];
            }
            A[i + j * n] = s;
        }
        double big = 0.0;
        for (int i = j; i < n; ++i) {
            double s = A[i + j * n];
            for (int k = 0; k < j; ++k) {
                s -= A[i + k * n] * A[k + j * n];
            }
            A[i + j * n] = s;
            const double tmp = vv[static_cast<size_t>(i)] * std::fabs(s);
            if (tmp >= big) {
                big = tmp;
                imax = i;
            }
        }
        if (j != imax) {
            for (int k = 0; k < n; ++k) {
                std::swap(A[imax + k * n], A[j + k * n]);
            }
            *d = -(*d);
            vv[static_cast<size_t>(imax)] = vv[static_cast<size_t>(j)];
        }
        indx[j] = imax;
        if (A[j + j * n] == 0.0) {
            return -1;
        }
        if (j != n - 1) {
            const double tmp = 1.0 / A[j + j * n];
            for (int i = j + 1; i < n; ++i) {
                A[i + j * n] *= tmp;
            }
        }
    }
    return 0;
}

void claslibLubksb(const double* A, int n, const int* indx, double* b) {
    int ii = -1;
    for (int i = 0; i < n; ++i) {
        const int ip = indx[i];
        double s = b[ip];
        b[ip] = b[i];
        if (ii >= 0) {
            for (int j = ii; j < i; ++j) {
                s -= A[i + j * n] * b[j];
            }
        } else if (s) {
            ii = i;
        }
        b[i] = s;
    }
    for (int i = n - 1; i >= 0; --i) {
        double s = b[i];
        for (int j = i + 1; j < n; ++j) {
            s -= A[i + j * n] * b[j];
        }
        b[i] = s / A[i + i * n];
    }
}
#endif

int claslibMatinv(double* A, int n) {
#ifdef LAPACK
    int info = 0;
    int lwork = n * 16;
    std::vector<int> ipiv(static_cast<size_t>(n), 0);
    std::vector<double> work(static_cast<size_t>(lwork), 0.0);
    dgetrf_(&n, &n, A, &n, ipiv.data(), &info);
    if (!info) {
        dgetri_(&n, A, &n, ipiv.data(), work.data(), &lwork, &info);
    }
    return info;
#else
    double d = 0.0;
    std::vector<int> indx(static_cast<size_t>(n), 0);
    std::vector<double> B(A, A + n * n);
    if (claslibLudcmp(B.data(), n, indx.data(), &d)) {
        return -1;
    }
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            A[i + j * n] = 0.0;
        }
        A[j + j * n] = 1.0;
        claslibLubksb(B.data(), n, indx.data(), A + j * n);
    }
    return 0;
#endif
}

int claslibLsq(const double* A,
               const double* y,
               int n,
               int m,
               double* x,
               double* Q) {
    if (m < n) {
        return -1;
    }
    std::vector<double> Ay(static_cast<size_t>(n), 0.0);
    claslibMatmul("NN", n, 1, m, 1.0, A, y, 0.0, Ay.data());
    claslibMatmul("NT", n, n, m, 1.0, A, A, 0.0, Q);
    const int info = claslibMatinv(Q, n);
    if (!info) {
        claslibMatmul("NN", n, 1, n, 1.0, Q, Ay.data(), 0.0, x);
    }
    return info;
}

int solveClaslibLeastSquares(const MatrixXd& weighted_H,
                             const VectorXd& weighted_v,
                             int nv,
                             VectorXd& dx,
                             MatrixXd& Q) {
    std::vector<double> A(static_cast<size_t>(kNx * nv), 0.0);
    std::vector<double> y(static_cast<size_t>(nv), 0.0);
    std::vector<double> x(static_cast<size_t>(kNx), 0.0);
    std::vector<double> q(static_cast<size_t>(kNx * kNx), 0.0);
    for (int j = 0; j < nv; ++j) {
        y[static_cast<size_t>(j)] = weighted_v(j);
        for (int k = 0; k < kNx; ++k) {
            A[static_cast<size_t>(k + j * kNx)] = weighted_H(j, k);
        }
    }
    const int info = claslibLsq(A.data(), y.data(), kNx, nv, x.data(), q.data());
    if (info) {
        return info;
    }
    for (int i = 0; i < kNx; ++i) {
        dx(i) = x[static_cast<size_t>(i)];
        for (int j = 0; j < kNx; ++j) {
            Q(i, j) = q[static_cast<size_t>(i + j * kNx)];
        }
    }
    return 0;
}

EffectiveOptions effectiveOptions(const PntposOptions& options) {
    EffectiveOptions effective;
    if (options.mode == PntposOptions::Mode::PppRtk) {
        effective.iono_free = true;
        effective.trop_saas = true;
    } else if (options.mode == PntposOptions::Mode::Precise) {
        effective.iono_free = false;
        effective.trop_saas = true;
    } else {
        effective.iono_free = false;
        effective.trop_saas = true;
    }
    return effective;
}

int clasSatNo(const SatelliteId& satellite) {
    switch (satellite.system) {
        case GNSSSystem::GPS:
            return satellite.prn >= 1 && satellite.prn <= 32 ? satellite.prn : 0;
        case GNSSSystem::GLONASS:
            return satellite.prn >= 1 && satellite.prn <= 24 ? 32 + satellite.prn : 0;
        case GNSSSystem::Galileo:
            return satellite.prn >= 1 && satellite.prn <= 36 ? 32 + 24 + satellite.prn : 0;
        case GNSSSystem::QZSS: {
            const int prn = satellite.prn >= 193 ? satellite.prn - 192 : satellite.prn;
            return prn >= 1 && prn <= 7 ? 32 + 24 + 36 + prn : 0;
        }
        case GNSSSystem::BeiDou:
            return satellite.prn >= 1 && satellite.prn <= 35 ? 32 + 24 + 36 + 7 + satellite.prn : 0;
        default:
            return 0;
    }
}

bool systemEnabled(GNSSSystem system, const PntposOptions& options) {
    switch (system) {
        case GNSSSystem::GPS:
            return options.enable_gps;
        case GNSSSystem::Galileo:
            return options.enable_galileo;
        case GNSSSystem::QZSS:
            return options.enable_qzss;
        case GNSSSystem::GLONASS:
            return options.enable_glonass;
        case GNSSSystem::BeiDou:
            return options.enable_beidou;
        default:
            return false;
    }
}

int frequencyIndex(const Observation& observation) {
    switch (observation.signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GPS_L1P:
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
        case SignalType::GAL_E1:
        case SignalType::BDS_B1I:
        case SignalType::BDS_B1C:
        case SignalType::QZS_L1CA:
            return 0;
        case SignalType::GPS_L2C:
        case SignalType::GPS_L2P:
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
        case SignalType::BDS_B2I:
        case SignalType::BDS_B2A:
        case SignalType::QZS_L2C:
            return 1;
        case SignalType::GPS_L5:
        case SignalType::GAL_E5A:
        case SignalType::QZS_L5:
            return 2;
        default:
            return -1;
    }
}

int secondaryFrequencyIndex(GNSSSystem system) {
    if (system == GNSSSystem::Galileo || system == GNSSSystem::SBAS) {
        return 2;
    }
    return 1;
}

double signalFrequencyForObservation(const PntObservation& obs, int index) {
    if (index < 0 || index >= 3 ||
        obs.signals[index] == SignalType::SIGNAL_TYPE_COUNT) {
        return 0.0;
    }
    return signalFrequencyHz(obs.signals[index], obs.eph);
}

double broadcastClockNoRelativity(const Ephemeris& eph, const GNSSTime& time) {
    double t = time - eph.toc;
    for (int i = 0; i < 2; ++i) {
        t -= eph.af0 + eph.af1 * t + eph.af2 * t * t;
    }
    return eph.af0 + eph.af1 * t + eph.af2 * t * t;
}

double broadcastVariance(const Ephemeris& eph) {
    static constexpr std::array<double, 15> kGpsUra = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0,
        96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};
    if (eph.satellite.system == GNSSSystem::Galileo) {
        const double sisa_cm = eph.sv_accuracy * 100.0;
        int sva = 255;
        if (0.0 <= sisa_cm && sisa_cm < 50.0) {
            sva = static_cast<int>(std::ceil(sisa_cm));
        } else if (50.0 <= sisa_cm && sisa_cm < 100.0) {
            sva = static_cast<int>(std::ceil((sisa_cm - 50.0) / 2.0)) + 50;
        } else if (100.0 <= sisa_cm && sisa_cm < 200.0) {
            sva = static_cast<int>(std::ceil((sisa_cm - 100.0) / 4.0)) + 75;
        } else if (200.0 <= sisa_cm && sisa_cm <= 600.0) {
            sva = static_cast<int>(std::ceil((sisa_cm - 200.0) / 16.0)) + 100;
        }
        double temp = 6144.0 * 100.0;
        if (0 <= sva && sva <= 49) {
            temp = static_cast<double>(sva);
        } else if (50 <= sva && sva <= 74) {
            temp = static_cast<double>((sva - 50) * 2 + 50);
        } else if (75 <= sva && sva <= 99) {
            temp = static_cast<double>((sva - 75) * 4 + 100);
        } else if (100 <= sva && sva <= 125) {
            temp = static_cast<double>((sva - 100) * 16 + 200);
        }
        const double sigma = temp / 100.0;
        return sigma * sigma;
    }
    int ura = static_cast<int>(eph.ura);
    if (ura == 0 && eph.sv_accuracy > 0.0) {
        ura = 14;
        for (int i = 0; i < static_cast<int>(kGpsUra.size()); ++i) {
            if (kGpsUra[static_cast<size_t>(i)] >= eph.sv_accuracy) {
                ura = i;
                break;
            }
        }
    }
    if (ura < 0 || ura > 14) {
        return 6144.0 * 6144.0;
    }
    return kGpsUra[static_cast<size_t>(ura)] * kGpsUra[static_cast<size_t>(ura)];
}

bool calculateBroadcastState(const Ephemeris& eph,
                             const GNSSTime& time,
                             Vector3d& position,
                             double& clock_s,
                             double* variance_m2 = nullptr) {
    if (!eph.valid || eph.sqrt_a <= 0.0) {
        position.setZero();
        clock_s = 0.0;
        if (variance_m2) *variance_m2 = 0.0;
        return false;
    }

    constexpr double kMuGps = 3.9860050e14;
    constexpr double kMuGal = 3.986004418e14;
    constexpr double kMuBds = 3.986004418e14;
    constexpr double kOmgeGps = 7.2921151467e-5;
    constexpr double kOmgeGal = 7.2921151467e-5;
    constexpr double kOmgeBds = 7.292115e-5;
    constexpr double kSinMinusFiveDeg = -0.0871557427476582;
    constexpr double kCosMinusFiveDeg = 0.9961946980917456;

    const double a = eph.sqrt_a * eph.sqrt_a;
    const double tk = time - eph.toe;
    double mu = kMuGps;
    double omge = kOmgeGps;
    if (eph.satellite.system == GNSSSystem::Galileo) {
        mu = kMuGal;
        omge = kOmgeGal;
    } else if (eph.satellite.system == GNSSSystem::BeiDou) {
        mu = kMuBds;
        omge = kOmgeBds;
    }

    const double mean_anomaly =
        eph.m0 + (std::sqrt(mu / (a * a * a)) + eph.delta_n) * tk;
    double eccentric_anomaly = mean_anomaly;
    double previous_eccentric_anomaly = 0.0;
    int n = 0;
    for (n = 0; n < 30; ++n) {
        previous_eccentric_anomaly = eccentric_anomaly;
        eccentric_anomaly -=
            (eccentric_anomaly - eph.e * std::sin(eccentric_anomaly) -
             mean_anomaly) /
            (1.0 - eph.e * std::cos(eccentric_anomaly));
        if (std::abs(eccentric_anomaly - previous_eccentric_anomaly) <= kRtolKepler) {
            break;
        }
    }
    if (n >= 30) {
        return false;
    }

    const double sin_e = std::sin(eccentric_anomaly);
    const double cos_e = std::cos(eccentric_anomaly);
    double u = std::atan2(std::sqrt(1.0 - eph.e * eph.e) * sin_e,
                          cos_e - eph.e) +
               eph.omega;
    double r = a * (1.0 - eph.e * cos_e);
    double inclination = eph.i0 + eph.idot * tk;
    const double sin_2u = std::sin(2.0 * u);
    const double cos_2u = std::cos(2.0 * u);
    u += eph.cus * sin_2u + eph.cuc * cos_2u;
    r += eph.crs * sin_2u + eph.crc * cos_2u;
    inclination += eph.cis * sin_2u + eph.cic * cos_2u;

    const double x = r * std::cos(u);
    const double y = r * std::sin(u);
    const double cos_i = std::cos(inclination);
    const double toes = eph.toes != 0.0 ? eph.toes : eph.toe.tow;

    if (eph.satellite.system == GNSSSystem::BeiDou && eph.satellite.prn <= 5) {
        const double omega = eph.omega0 + eph.omega_dot * tk - omge * toes;
        const double sin_omega = std::sin(omega);
        const double cos_omega = std::cos(omega);
        const double xg = x * cos_omega - y * cos_i * sin_omega;
        const double yg = x * sin_omega + y * cos_i * cos_omega;
        const double zg = y * std::sin(inclination);
        const double sin_rot = std::sin(omge * tk);
        const double cos_rot = std::cos(omge * tk);
        position.x() = xg * cos_rot + yg * sin_rot * kCosMinusFiveDeg +
                       zg * sin_rot * kSinMinusFiveDeg;
        position.y() = -xg * sin_rot + yg * cos_rot * kCosMinusFiveDeg +
                       zg * cos_rot * kSinMinusFiveDeg;
        position.z() = -yg * kSinMinusFiveDeg + zg * kCosMinusFiveDeg;
    } else {
        const double omega =
            eph.omega0 + (eph.omega_dot - omge) * tk - omge * toes;
        const double sin_omega = std::sin(omega);
        const double cos_omega = std::cos(omega);
        position.x() = x * cos_omega - y * cos_i * sin_omega;
        position.y() = x * sin_omega + y * cos_i * cos_omega;
        position.z() = y * std::sin(inclination);
    }

    const double tc = time - eph.toc;
    clock_s = eph.af0 + eph.af1 * tc + eph.af2 * tc * tc;
    clock_s -=
        2.0 * std::sqrt(mu * a) * eph.e * sin_e /
        (constants::SPEED_OF_LIGHT * constants::SPEED_OF_LIGHT);
    if (variance_m2) {
        *variance_m2 = broadcastVariance(eph);
    }
    return true;
}

bool satposs(PntObservation& obs, const NavigationData& nav, const GNSSTime& teph) {
    obs.rs.setZero();
    obs.vs.setZero();
    obs.dts = {};
    obs.vare = 0.0;
    obs.svh = -1;
    obs.has_satpos = false;
    obs.eph = nullptr;

    double pr = 0.0;
    for (int i = 0; i < 3; ++i) {
        if (obs.P[static_cast<size_t>(i)] != 0.0) {
            pr = obs.P[static_cast<size_t>(i)];
            break;
        }
    }
    if (pr == 0.0) {
        return false;
    }

    const Ephemeris* eph = nav.getEphemeris(obs.satellite, teph);
    if (eph == nullptr || !eph->valid) {
        return false;
    }
    obs.eph = eph;

    GNSSTime tx_time = teph - pr / constants::SPEED_OF_LIGHT;
    const double dt = broadcastClockNoRelativity(*eph, tx_time);
    tx_time = tx_time - dt;

    Vector3d rs0 = Vector3d::Zero();
    Vector3d rs1 = Vector3d::Zero();
    double dts0 = 0.0;
    double dts1 = 0.0;
    double vare = 0.0;
    if (!calculateBroadcastState(*eph, tx_time, rs0, dts0, &vare)) {
        return false;
    }
    if (!calculateBroadcastState(*eph, tx_time + 1e-3, rs1, dts1, nullptr)) {
        return false;
    }
    obs.rs = rs0;
    obs.vs = (rs1 - rs0) / 1e-3;
    obs.dts[0] = dts0;
    obs.dts[1] = (dts1 - dts0) / 1e-3;
    obs.vare = vare;
    obs.svh = static_cast<int>(eph->health);
    obs.has_satpos = true;
    return true;
}

std::vector<PntObservation> collectObservations(const ObservationData& obs,
                                                const PntposOptions& options) {
    std::map<SatelliteId, PntObservation> by_satellite;
    for (const Observation& raw : obs.observations) {
        if (!raw.valid || !raw.has_pseudorange || raw.pseudorange == 0.0) {
            continue;
        }
        if (!systemEnabled(raw.satellite.system, options)) {
            continue;
        }
        const int satno = clasSatNo(raw.satellite);
        if (satno <= 0) {
            continue;
        }
        const int freq = frequencyIndex(raw);
        if (freq < 0 || freq >= 3) {
            continue;
        }
        PntObservation& entry = by_satellite[raw.satellite];
        entry.satellite = raw.satellite;
        entry.satno = satno;
        const size_t idx = static_cast<size_t>(freq);
        if (entry.P[idx] == 0.0) {
            entry.P[idx] = raw.pseudorange;
            entry.D[idx] = raw.has_doppler ? raw.doppler : 0.0;
            entry.SNR[idx] = raw.snr;
            entry.signals[idx] = raw.signal;
            entry.source[idx] = &raw;
        }
    }
    std::vector<PntObservation> observations;
    observations.reserve(by_satellite.size());
    for (auto& [satellite, entry] : by_satellite) {
        (void)satellite;
        observations.push_back(entry);
    }
    std::sort(observations.begin(), observations.end(),
              [](const PntObservation& lhs, const PntObservation& rhs) {
                  return lhs.satno < rhs.satno;
              });
    return observations;
}

double snrThreshold(const PntposOptions& options, double el, int freq) {
    if (!options.snr_mask_rover || freq < 0 || freq >= 3) {
        return 0.0;
    }
    double a = (el * 180.0 / kPi + 5.0) / 10.0;
    int i = static_cast<int>(std::floor(a));
    a -= static_cast<double>(i);
    const auto& mask = options.snr_mask[static_cast<size_t>(freq)];
    if (i < 1) {
        return mask[0];
    }
    if (i > 8) {
        return mask[8];
    }
    return (1.0 - a) * mask[static_cast<size_t>(i - 1)] +
           a * mask[static_cast<size_t>(i)];
}

bool testsnr(const PntposOptions& options, int freq, double el, double snr) {
    if (!options.snr_mask_rover || freq < 0 || freq >= 3) {
        return false;
    }
    return snr < snrThreshold(options, el, freq);
}

double prange(const PntObservation& obs,
              const EffectiveOptions& effective,
              const PntposOptions& options,
              double elevation,
              int iter,
              double& var) {
    var = 0.0;
    const int i = 0;
    const int j = secondaryFrequencyIndex(obs.satellite.system);
    const double f1 = signalFrequencyForObservation(obs, i);
    const double f2 = signalFrequencyForObservation(obs, j);
    if (f1 == 0.0 || f2 == 0.0) {
        return 0.0;
    }
    if (iter > 0) {
        if (testsnr(options, i, elevation, obs.SNR[static_cast<size_t>(i)])) {
            return 0.0;
        }
        if (effective.iono_free &&
            testsnr(options, j, elevation, obs.SNR[static_cast<size_t>(j)])) {
            return 0.0;
        }
    }

    const double gamma = (f1 * f1) / (f2 * f2);
    double P1 = obs.P[static_cast<size_t>(i)];
    double P2 = obs.P[static_cast<size_t>(j)];
    double PC = 0.0;
    if (effective.iono_free) {
        if (P1 == 0.0 || P2 == 0.0) {
            return 0.0;
        }
        PC = (gamma * P1 - P2) / (gamma - 1.0);
    } else {
        if (P1 == 0.0) {
            return 0.0;
        }
        double P1_P2 = 0.0;
        if (obs.eph != nullptr &&
            (obs.satellite.system == GNSSSystem::GPS ||
             obs.satellite.system == GNSSSystem::Galileo ||
             obs.satellite.system == GNSSSystem::QZSS)) {
            P1_P2 = (1.0 - gamma) * constants::SPEED_OF_LIGHT * obs.eph->tgd;
        }
        PC = P1 - P1_P2 / (1.0 - gamma);
    }
    var = kErrCodeBias * kErrCodeBias;
    return PC;
}

double varerr(const PntposOptions& options,
              const EffectiveOptions& effective,
              double el,
              GNSSSystem system) {
    const double fact = system == GNSSSystem::GLONASS ? 1.5 :
                        system == GNSSSystem::SBAS ? 3.0 : 1.0;
    double var =
        options.err[0] * options.err[0] *
        (options.err[1] * options.err[1] +
         options.err[2] * options.err[2] / std::sin(el));
    if (effective.iono_free) {
        var *= 9.0;
    }
    return fact * fact * var;
}

double ionmodelClaslib(const NavigationData& nav,
                       const Vector3d& receiver_position,
                       double azimuth,
                       double elevation,
                       const GNSSTime& time) {
    static constexpr std::array<double, 4> kDefaultAlpha = {
        0.1118e-07, -0.7451e-08, -0.5961e-07, 0.1192e-06};
    static constexpr std::array<double, 4> kDefaultBeta = {
        0.1167e+06, -0.2294e+06, -0.1311e+06, 0.1049e+07};

    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver_position, lat, lon, height);
    if (height < -1000.0 || elevation <= 0.0) {
        return 0.0;
    }
    double ion_norm = 0.0;
    for (int i = 0; i < 4; ++i) {
        ion_norm += nav.ionosphere_model.alpha[i] * nav.ionosphere_model.alpha[i];
        ion_norm += nav.ionosphere_model.beta[i] * nav.ionosphere_model.beta[i];
    }
    const double* alpha =
        (nav.ionosphere_model.valid && ion_norm > 0.0)
            ? nav.ionosphere_model.alpha
            : kDefaultAlpha.data();
    const double* beta =
        (nav.ionosphere_model.valid && ion_norm > 0.0)
            ? nav.ionosphere_model.beta
            : kDefaultBeta.data();

    const double psi = 0.0137 / (elevation / kPi + 0.11) - 0.022;
    double phi = lat / kPi + psi * std::cos(azimuth);
    phi = std::clamp(phi, -0.416, 0.416);
    const double lam = lon / kPi +
                       psi * std::sin(azimuth) / std::cos(phi * kPi);
    phi += 0.064 * std::cos((lam - 1.617) * kPi);

    double tt = 43200.0 * lam + time.tow;
    tt -= std::floor(tt / 86400.0) * 86400.0;
    const double f = 1.0 + 16.0 * std::pow(0.53 - elevation / kPi, 3.0);
    double amp =
        alpha[0] + phi * (alpha[1] + phi * (alpha[2] + phi * alpha[3]));
    double per =
        beta[0] + phi * (beta[1] + phi * (beta[2] + phi * beta[3]));
    amp = std::max(amp, 0.0);
    per = std::max(per, 72000.0);
    const double x = 2.0 * kPi * (tt - 50400.0) / per;
    const double delay_s =
        f * (std::abs(x) < 1.57
                 ? 5e-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0))
                 : 5e-9);
    return constants::SPEED_OF_LIGHT * delay_s;
}

double tropcorrClaslib(const Vector3d& receiver_position,
                       double elevation,
                       double& variance) {
    (void)kRelHumi;
    variance =
        (kErrSaas / (std::sin(elevation) + 0.1)) *
        (kErrSaas / (std::sin(elevation) + 0.1));
    return models::tropDelaySaastamoinen(receiver_position, elevation);
}

double geometricDistanceAndLos(const Vector3d& rs,
                               const Vector3d& rr,
                               Vector3d& e) {
    double sat_norm_sq = 0.0;
    for (int i = 0; i < 3; ++i) {
        sat_norm_sq += rs(i) * rs(i);
    }
    if (std::sqrt(sat_norm_sq) < constants::WGS84_A) {
        e.setZero();
        return -1.0;
    }
    for (int i = 0; i < 3; ++i) {
        e(i) = rs(i) - rr(i);
    }
    double r_sq = 0.0;
    for (int i = 0; i < 3; ++i) {
        r_sq += e(i) * e(i);
    }
    const double r = std::sqrt(r_sq);
    if (r <= 0.0) {
        e.setZero();
        return 0.0;
    }
    for (int i = 0; i < 3; ++i) {
        e(i) /= r;
    }
    return r + constants::OMEGA_E *
        (rs.x() * rr.y() - rs.y() * rr.x()) / constants::SPEED_OF_LIGHT;
}

NavigationData::SatelliteGeometry geometryFromLos(const Vector3d& rr,
                                                  const Vector3d& e) {
    NavigationData::SatelliteGeometry geom{};
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(rr, lat, lon, height);
    if (height <= -constants::WGS84_A) {
        geom.azimuth = 0.0;
        geom.elevation = kPi / 2.0;
        geom.distance = 0.0;
        return geom;
    }
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);
    const Vector3d east(-sin_lon, cos_lon, 0.0);
    const Vector3d north(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    const Vector3d up(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
    const double enu_e = e.dot(east);
    const double enu_n = e.dot(north);
    const double enu_u = e.dot(up);
    geom.elevation = std::asin(std::clamp(enu_u, -1.0, 1.0));
    geom.azimuth = std::atan2(enu_e, enu_n);
    if (geom.azimuth < 0.0) {
        geom.azimuth += 2.0 * kPi;
    }
    geom.distance = 0.0;
    return geom;
}

bool satsigexclude(const PntObservation& obs, const PntposOptions& options) {
    if (obs.svh < 0) {
        return true;
    }
    if (!systemEnabled(obs.satellite.system, options)) {
        return true;
    }
    if (obs.svh != 0 && obs.satellite.system != GNSSSystem::QZSS) {
        return true;
    }
    return false;
}

int rescode(int iter,
            std::vector<PntObservation>& observations,
            const NavigationData& nav,
            const GNSSTime& time,
            const VectorXd& x,
            const EffectiveOptions& effective,
            const PntposOptions& options,
            const std::set<int>& outp,
            VectorXd& v,
            MatrixXd& H,
            VectorXd& var,
            std::vector<SatelliteStatus>& statuses,
            int& ns) {
    const Vector3d rr = x.segment<3>(0);
    const double dtr = x(3);
    int nv = 0;
    ns = 0;
    std::array<bool, 4> mask = {false, false, false, false};
    statuses.assign(observations.size(), SatelliteStatus{});

    for (size_t i = 0; i < observations.size(); ++i) {
        PntObservation& obs = observations[i];
        statuses[i].satellite = obs.satellite;
        if (outp.find(obs.satno) != outp.end()) {
            continue;
        }
        if (!obs.has_satpos) {
            continue;
        }
        Vector3d e = Vector3d::Zero();
        const double r = geometricDistanceAndLos(obs.rs, rr, e);
        if (r <= 0.0) {
            continue;
        }
        const auto geom = geometryFromLos(rr, e);
        statuses[i].azimuth_rad = geom.azimuth;
        statuses[i].elevation_rad = geom.elevation;
        if (geom.elevation < options.elmin_rad) {
            continue;
        }
        if (satsigexclude(obs, options)) {
            continue;
        }
        double vmeas = 0.0;
        const double P = prange(obs, effective, options, geom.elevation, iter, vmeas);
        if (P == 0.0) {
            continue;
        }

        double dion = 0.0;
        double vion = 0.0;
        if (iter == 0 || !effective.iono_free) {
            dion = ionmodelClaslib(nav, rr, geom.azimuth, geom.elevation, time);
            vion = (dion * kErrBrdcI) * (dion * kErrBrdcI);
            const double f1 = signalFrequencyForObservation(obs, 0);
            if (f1 > 0.0) {
                const double lam_l1 = constants::SPEED_OF_LIGHT / f1;
                const double gps_l1_lam =
                    constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;
                dion *= (lam_l1 / gps_l1_lam) * (lam_l1 / gps_l1_lam);
            }
        }

        double vtrp = 0.0;
        const double dtrp = tropcorrClaslib(rr, geom.elevation, vtrp);

        double residual =
            P - (r + dtr - constants::SPEED_OF_LIGHT * obs.dts[0] + dion + dtrp);

        H.row(nv).setZero();
        H(nv, 0) = -e.x();
        H(nv, 1) = -e.y();
        H(nv, 2) = -e.z();
        H(nv, 3) = 1.0;

        int bias_column = -1;
        if (obs.satellite.system == GNSSSystem::GLONASS) {
            residual -= x(4);
            bias_column = 4;
            mask[1] = true;
        } else if (obs.satellite.system == GNSSSystem::Galileo) {
            residual -= x(5);
            bias_column = 5;
            mask[2] = true;
        } else if (obs.satellite.system == GNSSSystem::BeiDou) {
            residual -= x(6);
            bias_column = 6;
            mask[3] = true;
        } else {
            mask[0] = true;
        }
        if (bias_column >= 0) {
            H(nv, bias_column) = 1.0;
        }

        v(nv) = residual;
        statuses[i].valid = true;
        statuses[i].pseudorange_residual_m = residual;
        ++ns;

        var(nv) =
            varerr(options, effective, geom.elevation, obs.satellite.system) +
            obs.vare + vmeas + vion + vtrp;
        ++nv;
    }

    for (int i = 0; i < 4; ++i) {
        if (mask[static_cast<size_t>(i)]) {
            continue;
        }
        v(nv) = 0.0;
        H.row(nv).setZero();
        H(nv, i + 3) = 1.0;
        var(nv) = 0.01;
        ++nv;
    }
    return nv;
}

void excludeLargeResiduals(const std::vector<PntObservation>& observations,
                           const std::vector<SatelliteStatus>& statuses,
                           const PntposOptions& options,
                           std::set<int>& outp) {
    const double threshold_sq = options.rejethres_m * options.rejethres_m;
    double sum_sq = 0.0;
    int count = 0;
    for (const SatelliteStatus& status : statuses) {
        if (!status.valid) {
            continue;
        }
        sum_sq += status.pseudorange_residual_m * status.pseudorange_residual_m;
        ++count;
    }
    if (count == 0 || sum_sq / static_cast<double>(count) > threshold_sq) {
        return;
    }
    for (size_t i = 0; i < observations.size() && i < statuses.size(); ++i) {
        if (!statuses[i].valid) {
            continue;
        }
        if (statuses[i].pseudorange_residual_m *
                statuses[i].pseudorange_residual_m >
            threshold_sq) {
            outp.insert(observations[i].satno);
        }
    }
}

bool computeDops(const std::vector<SatelliteStatus>& statuses,
                 const PntposOptions& options,
                 std::array<double, 4>& dop) {
    dop = {0.0, 0.0, 0.0, 0.0};
    std::vector<SatelliteStatus> used;
    for (const SatelliteStatus& status : statuses) {
        if (!status.valid || status.elevation_rad < options.elmin_rad ||
            status.elevation_rad <= 0.0) {
            continue;
        }
        used.push_back(status);
    }
    if (used.size() < 4) {
        return false;
    }
    std::vector<double> H(static_cast<size_t>(4 * used.size()), 0.0);
    for (int i = 0; i < static_cast<int>(used.size()); ++i) {
        const double cosel = std::cos(used[static_cast<size_t>(i)].elevation_rad);
        const double sinel = std::sin(used[static_cast<size_t>(i)].elevation_rad);
        const double az = used[static_cast<size_t>(i)].azimuth_rad;
        H[static_cast<size_t>(4 * i)] = cosel * std::sin(az);
        H[static_cast<size_t>(1 + 4 * i)] = cosel * std::cos(az);
        H[static_cast<size_t>(2 + 4 * i)] = sinel;
        H[static_cast<size_t>(3 + 4 * i)] = 1.0;
    }
    double Q[16] = {};
    claslibMatmul("NT",
                  4,
                  4,
                  static_cast<int>(used.size()),
                  1.0,
                  H.data(),
                  H.data(),
                  0.0,
                  Q);
    if (claslibMatinv(Q, 4)) {
        return false;
    }
    dop[0] = std::sqrt(std::max(0.0, Q[0] + Q[5] + Q[10] + Q[15]));
    dop[1] = std::sqrt(std::max(0.0, Q[0] + Q[5] + Q[10]));
    dop[2] = std::sqrt(std::max(0.0, Q[0] + Q[5]));
    dop[3] = std::sqrt(std::max(0.0, Q[10]));
    return true;
}

bool valsol(const std::vector<SatelliteStatus>& statuses,
            const PntposOptions& options,
            const VectorXd& weighted_residuals,
            int nv,
            std::array<double, 4>& dop,
            std::string& msg) {
    const double vv = weighted_residuals.head(nv).squaredNorm();
    if (nv > kNx) {
        const int dof_index = nv - kNx - 1;
        if (dof_index >= 0 &&
            dof_index < static_cast<int>(kChiSquare001.size()) &&
            vv > kChiSquare001[static_cast<size_t>(dof_index)]) {
            std::ostringstream ss;
            ss << "chi-square error nv=" << nv << " vv=" << std::fixed
               << std::setprecision(1) << vv
               << " cs=" << kChiSquare001[static_cast<size_t>(dof_index)];
            msg = ss.str();
            return false;
        }
    }
    if (!computeDops(statuses, options, dop) || dop[0] <= 0.0 ||
        dop[0] > options.maxgdop) {
        std::ostringstream ss;
        ss << "gdop error nv=" << nv << " gdop=" << std::fixed
           << std::setprecision(1) << dop[0];
        msg = ss.str();
        return false;
    }
    return true;
}

PositionSolution toPositionSolution(const GNSSTime& obs_time,
                                    const PntposResult& result) {
    PositionSolution solution;
    solution.time = obs_time - result.dtr[0];
    solution.status = result.valid ? SolutionStatus::SPP : SolutionStatus::NONE;
    solution.position_ecef =
        Vector3d(result.rr[0], result.rr[1], result.rr[2]);
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, lat, lon, height);
    solution.position_geodetic = GeodeticCoord(lat, lon, height);
    solution.receiver_clock_bias =
        result.dtr[0] * constants::SPEED_OF_LIGHT;
    solution.num_satellites = result.ns;
    solution.num_frequencies = 2;
    solution.iterations = result.iterations;
    solution.gdop = result.gdop;
    solution.pdop = result.pdop;
    solution.hdop = result.hdop;
    solution.vdop = result.vdop;
    solution.residual_rms = result.residual_rms_m;
    for (const SatelliteStatus& status : result.satellites) {
        if (!status.valid) {
            continue;
        }
        solution.satellites_used.push_back(status.satellite);
        solution.satellite_elevations.push_back(status.elevation_rad);
        solution.satellite_residuals.push_back(status.pseudorange_residual_m);
    }
    return solution;
}

bool estpos(std::vector<PntObservation> observations,
            const NavigationData& nav,
            const GNSSTime& time,
            const Vector3d& initial_rr,
            const EffectiveOptions& effective,
            const PntposOptions& options,
            EstimationState& estimation) {
    VectorXd x = VectorXd::Zero(kNx);
    x.head<3>() = initial_rr;
    VectorXd dx = VectorXd::Zero(kNx);
    MatrixXd Q = MatrixXd::Zero(kNx, kNx);
    VectorXd v = VectorXd::Zero(static_cast<int>(observations.size()) + 4);
    MatrixXd H = MatrixXd::Zero(static_cast<int>(observations.size()) + 4, kNx);
    VectorXd var = VectorXd::Zero(static_cast<int>(observations.size()) + 4);
    std::vector<SatelliteStatus> statuses;
    std::set<int> outp;
    int nv = 0;
    int ns = 0;
    int iter = 0;

    for (iter = 0; iter < kMaxItr; ++iter) {
        if (iter > 0 && nv > options.rejeminsat && options.rejethres_m > 0.0) {
            excludeLargeResiduals(observations, statuses, options, outp);
        }
        v.setZero();
        H.setZero();
        var.setZero();
        nv = rescode(iter, observations, nav, time, x, effective, options, outp,
                     v, H, var, statuses, ns);
        if (options.debug_dump && shouldDumpPntpos(time)) {
            std::cerr << std::setprecision(12)
                      << "[CLAS-SPP-ITER] source=lib port=ported_pntpos"
                      << " week=" << time.week
                      << " tow=" << time.tow
                      << " iter=" << iter
                      << " ns=" << ns
                      << " nv=" << nv
                      << " residuals=";
            bool first = true;
            for (size_t status_index = 0;
                 status_index < statuses.size() && status_index < observations.size();
                 ++status_index) {
                if (!statuses[status_index].valid) {
                    continue;
                }
                if (!first) {
                    std::cerr << ",";
                }
                first = false;
                std::cerr << observations[status_index].satellite.toString()
                          << ":" << statuses[status_index].pseudorange_residual_m;
            }
            std::cerr << "\n";
        }
        if (nv < kNx) {
            std::ostringstream ss;
            ss << "lack of valid sats ns=" << nv;
            estimation.result.message = ss.str();
            return false;
        }

        MatrixXd weighted_H = H.topRows(nv);
        VectorXd weighted_v = v.head(nv);
        for (int j = 0; j < nv; ++j) {
            const double sig = std::sqrt(var(j));
            if (!(sig > 0.0) || !std::isfinite(sig)) {
                estimation.result.message = "invalid variance";
                return false;
            }
            weighted_v(j) /= sig;
            for (int k = 0; k < kNx; ++k) {
                weighted_H(j, k) /= sig;
            }
        }

        const int info =
            solveClaslibLeastSquares(weighted_H, weighted_v, nv, dx, Q);
        if (info || !dx.allFinite()) {
            std::ostringstream ss;
            ss << "lsq error info=" << (info ? info : 1);
            estimation.result.message = ss.str();
            return false;
        }
        x += dx;
        estimation.weighted_residuals = weighted_v;
        estimation.covariance = Q;
        estimation.nv = nv;

        if (dx.norm() < 1e-4) {
            std::array<double, 4> dop = {0.0, 0.0, 0.0, 0.0};
            std::string msg;
            const bool valid = valsol(statuses, options, weighted_v, nv, dop, msg);

            PntposResult result;
            result.valid = valid;
            result.stat = valid ? 5 : 0;
            result.ns = ns;
            result.iterations = iter + 1;
            result.rr[0] = x(0);
            result.rr[1] = x(1);
            result.rr[2] = x(2);
            result.dtr[0] = x(3) / constants::SPEED_OF_LIGHT;
            result.dtr[1] = x(4) / constants::SPEED_OF_LIGHT;
            result.dtr[2] = x(5) / constants::SPEED_OF_LIGHT;
            result.dtr[3] = x(6) / constants::SPEED_OF_LIGHT;
            result.qr[0] = static_cast<float>(Q(0, 0));
            result.qr[1] = static_cast<float>(Q(1, 1));
            result.qr[2] = static_cast<float>(Q(2, 2));
            result.qr[3] = static_cast<float>(Q(0, 1));
            result.qr[4] = static_cast<float>(Q(1, 2));
            result.qr[5] = static_cast<float>(Q(0, 2));
            result.gdop = dop[0];
            result.pdop = dop[1];
            result.hdop = dop[2];
            result.vdop = dop[3];
            result.message = msg;
            result.satellites = statuses;
            double residual_sum_sq = 0.0;
            int residual_count = 0;
            for (const SatelliteStatus& status : statuses) {
                if (!status.valid) {
                    continue;
                }
                residual_sum_sq += status.pseudorange_residual_m *
                                   status.pseudorange_residual_m;
                ++residual_count;
            }
            result.residual_rms_m =
                residual_count > 0
                    ? std::sqrt(residual_sum_sq /
                                static_cast<double>(residual_count))
                    : 0.0;
            result.solution = toPositionSolution(time, result);
            estimation.result = result;
            return valid;
        }
    }
    std::ostringstream ss;
    ss << "iteration divergent i=" << iter;
    estimation.result.message = ss.str();
    return false;
}

double usedResidualRms(const PntposResult& result) {
    double sum_sq = 0.0;
    int count = 0;
    for (const SatelliteStatus& status : result.satellites) {
        if (!status.valid) {
            continue;
        }
        sum_sq += status.pseudorange_residual_m *
                  status.pseudorange_residual_m;
        ++count;
    }
    return count > 0 ? std::sqrt(sum_sq / static_cast<double>(count)) : 100.0;
}

bool raimFde(const std::vector<PntObservation>& observations,
             const NavigationData& nav,
             const GNSSTime& time,
             const Vector3d& initial_rr,
             const EffectiveOptions& effective,
             const PntposOptions& options,
             EstimationState& estimation) {
    double best_rms = 100.0;
    bool stat = false;
    EstimationState best;
    for (size_t i = 0; i < observations.size(); ++i) {
        std::vector<PntObservation> reduced;
        reduced.reserve(observations.size() - 1);
        for (size_t j = 0; j < observations.size(); ++j) {
            if (i == j) {
                continue;
            }
            reduced.push_back(observations[j]);
        }
        EstimationState candidate;
        if (!estpos(reduced, nav, time, initial_rr, effective, options, candidate)) {
            continue;
        }
        int nvsat = 0;
        double rms = 0.0;
        for (const SatelliteStatus& status : candidate.result.satellites) {
            if (!status.valid) {
                continue;
            }
            rms += status.pseudorange_residual_m *
                   status.pseudorange_residual_m;
            ++nvsat;
        }
        if (nvsat < 5) {
            continue;
        }
        rms = std::sqrt(rms / static_cast<double>(nvsat));
        if (rms > best_rms) {
            continue;
        }
        stat = true;
        best_rms = rms;
        best = candidate;
    }
    if (stat) {
        estimation = best;
    }
    return stat;
}

bool shouldDumpPntpos(const GNSSTime& time) {
    return time.week == 2068 && time.tow >= 230420.0 - 1e-6 &&
           time.tow <= 230425.0 + 1e-6;
}

const char* dumpPath() {
    const char* path = std::getenv("CLASLIB_PNTPOS_DUMP");
    if (path != nullptr && *path != '\0') {
        return path;
    }
    path = std::getenv("CLASLIB_RCV_POS_DUMP");
    if (path != nullptr && *path != '\0') {
        return path;
    }
    return nullptr;
}

std::string formatSeedDump(const GNSSTime& time, const PntposResult& result) {
    std::ostringstream ss;
    ss << std::setprecision(18)
       << "[CLAS-SPP-SEED] source=lib port=ported_pntpos"
       << " week=" << time.week
       << " tow=" << time.tow
       << " stat=" << result.stat
       << " ns=" << result.ns
       << " x=" << result.rr[0]
       << " y=" << result.rr[1]
       << " z=" << result.rr[2]
       << " dtr0=" << result.dtr[0]
       << " dtr1=" << result.dtr[1]
       << " dtr2=" << result.dtr[2]
       << " dtr3=" << result.dtr[3]
       << " dtr4=" << result.dtr[4]
       << " dtr5=" << result.dtr[5]
       << " clk_m=" << result.dtr[0] * constants::SPEED_OF_LIGHT
       << " rms=" << result.residual_rms_m
       << " gdop=" << result.gdop
       << " sats=";
    bool first = true;
    for (const SatelliteStatus& status : result.satellites) {
        if (!status.valid) {
            continue;
        }
        if (!first) {
            ss << ",";
        }
        first = false;
        ss << status.satellite.toString()
           << ":res=" << status.pseudorange_residual_m
           << ":el=" << status.elevation_rad * 180.0 / kPi;
    }
    if (!result.message.empty()) {
        ss << " msg=\"" << result.message << "\"";
    }
    return ss.str();
}

void dumpSeed(const GNSSTime& time,
              const PntposResult& result,
              const PntposOptions& options) {
    if (!options.debug_dump || !shouldDumpPntpos(time)) {
        return;
    }
    const std::string line = formatSeedDump(time, result);
    std::cerr << line << "\n";
    if (const char* path = dumpPath()) {
        std::ofstream out(path, std::ios::app);
        if (out.is_open()) {
            out << line << "\n";
        }
    }
}

}  // namespace

PntposResult pntpos(const ObservationData& obs,
                    const NavigationData& nav,
                    const Vector3d& initial_rr,
                    const PntposOptions& options) {
    PntposResult result;
    result.solution.time = obs.time;
    result.solution.status = SolutionStatus::NONE;

    std::vector<PntObservation> observations = collectObservations(obs, options);
    if (observations.empty()) {
        result.message = "no observation data";
        dumpSeed(obs.time, result, options);
        return result;
    }
    for (PntObservation& observation : observations) {
        satposs(observation, nav, obs.time);
    }

    const EffectiveOptions effective = effectiveOptions(options);
    EstimationState estimation;
    if (!estpos(observations, nav, obs.time, initial_rr, effective, options,
                estimation)) {
        if (options.raim_fde && observations.size() >= 6) {
            EstimationState raim_estimation;
            if (raimFde(observations, nav, obs.time, initial_rr, effective,
                        options, raim_estimation)) {
                estimation = raim_estimation;
            }
        }
    }
    if (estimation.result.valid) {
        result = estimation.result;
    } else {
        result = estimation.result;
        result.solution.time = obs.time;
        result.solution.status = SolutionStatus::NONE;
    }
    dumpSeed(obs.time, result, options);
    return result;
}

bool solvePntposSeed(const ObservationData& obs,
                     const NavigationData& nav,
                     const Vector3d& initial_rr,
                     PositionSolution& solution,
                     PntposResult* result,
                     const PntposOptions& options) {
    PntposResult local = pntpos(obs, nav, initial_rr, options);
    if (result != nullptr) {
        *result = local;
    }
    if (!local.valid) {
        return false;
    }
    solution = local.solution;
    return solution.isValid();
}

}  // namespace libgnss::ppp_claslib_pntpos
