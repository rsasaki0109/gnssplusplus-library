// Smoke tests for the vendored IAU SOFA library (third_party/sofa/).
//
// Reference values come directly from SOFA's own self-test program
// (third_party/sofa/src/t_sofa_c.c), which is the canonical source for
// expected outputs. If this file fails, the SOFA vendor copy is broken
// or mis-built — investigate before assuming a numerical issue.

#include <gtest/gtest.h>

extern "C" {
#include "sofa.h"
}

TEST(SofaVendor, Cal2jd) {
    double djm0 = 0.0;
    double djm  = 0.0;
    const int j = iauCal2jd(2003, 6, 1, &djm0, &djm);

    EXPECT_EQ(j, 0);
    EXPECT_DOUBLE_EQ(djm0, 2400000.5);
    EXPECT_DOUBLE_EQ(djm,  52791.0);
}

TEST(SofaVendor, Era00) {
    const double era = iauEra00(2400000.5, 54388.0);
    EXPECT_NEAR(era, 0.4022837240028158102, 1e-12);
}

TEST(SofaVendor, Gmst06) {
    const double theta = iauGmst06(2400000.5, 53736.0,
                                   2400000.5, 53736.0);
    EXPECT_NEAR(theta, 1.754174971870091203, 1e-12);
}

TEST(SofaVendor, Pom00) {
    const double xp =  2.55060238e-7;
    const double yp =  1.860359247e-6;
    const double sp = -0.1367174580728891460e-10;

    double rpom[3][3];
    iauPom00(xp, yp, sp, rpom);

    EXPECT_NEAR(rpom[0][0],  0.9999999999999674721,    1e-12);
    EXPECT_NEAR(rpom[0][1], -0.1367174580728846989e-10, 1e-16);
    EXPECT_NEAR(rpom[0][2],  0.2550602379999972345e-6,  1e-16);
}
