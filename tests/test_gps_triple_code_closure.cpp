#include <gtest/gtest.h>
#include <libgnss++/algorithms/gps_triple_code_closure.hpp>
#include <limits>

TEST(GpsTripleCodeClosure, CancelsCommonGeometryAndFirstOrderIonosphere) {
    using namespace libgnss;
    const double g2=std::pow(constants::GPS_L1_FREQ/constants::GPS_L2_FREQ,2);
    const double g5=std::pow(constants::GPS_L1_FREQ/constants::GPS_L5_FREQ,2);
    for (double common : {2e7, 2.5e7, 3e7}) for (double iono : {-10.,0.,10.,100.}) {
        const auto c=gps_triple_code::closure(common+iono,common+g2*iono,common+g5*iono);
        ASSERT_TRUE(c);
        EXPECT_NEAR(*c,0.,1e-8);
    }
}
TEST(GpsTripleCodeClosure, RetainsDifferentialBiasNotItsPhysicalAttribution) {
    using namespace libgnss::gps_triple_code;
    EXPECT_NEAR(*closure(2e7,2e7,2e7+3.),3.,1e-12);
    EXPECT_NEAR(*closure(2e7,2e7+3.,2e7),3.*l2Weight(),1e-12);
    EXPECT_NEAR(*closure(2e7+3.,2e7,2e7),-3.*(1.+l2Weight()),1e-12);
}
TEST(GpsTripleCodeClosure, RejectsInvalidObservations) {
    using libgnss::gps_triple_code::closure;
    EXPECT_FALSE(closure(0.,2e7,2e7));
    EXPECT_FALSE(closure(2e7,-1.,2e7));
    EXPECT_FALSE(closure(2e7,2e7,std::numeric_limits<double>::quiet_NaN()));
}
TEST(GpsTripleCodeClosure, TgdComponentMatchesPerSignalCodeDelays) {
    using namespace libgnss;
    const double g2=std::pow(constants::GPS_L1_FREQ/constants::GPS_L2_FREQ,2);
    for(double seconds:{-1e-8,0.,1e-8}) {
        const double t=constants::SPEED_OF_LIGHT*seconds;
        const auto component=gps_triple_code::tgdComponent(seconds);
        ASSERT_TRUE(component);
        const auto observed=gps_triple_code::closure(2e7+t,2e7+g2*t,2e7+t);
        ASSERT_TRUE(observed);
        EXPECT_NEAR(*observed,*component,1e-8);
    }
    EXPECT_FALSE(gps_triple_code::tgdComponent(std::numeric_limits<double>::infinity()));
}
