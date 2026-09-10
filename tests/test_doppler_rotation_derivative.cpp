#include <gtest/gtest.h>
#include <libgnss++/algorithms/doppler_contract.hpp>
#include <libgnss++/algorithms/doppler_rotation_rate.hpp>
#include <iostream>

TEST(DopplerRotationDerivative, RotationRateIterationMatchesClosedForm) {
    using namespace libgnss;
    // Isolate the Google reference's range-rate-dependent rotation term.
    // Satellite ephemeris velocity is supplied at transmit time; no additional
    // transmit-time derivative or receiver clock-tag denominator in this test.
    const Vector3d p(15600000.,20100000.,14000000.), r(6371000.,0.,0.);
    const Vector3d u=(p-r).normalized(), v(-1800.,2200.,1300.);
    for(double speed:{0.,-30.,30.}) {
        const Vector3d vr(0.,speed,0.);
        const double base=u.dot(v-vr);
        const double coefficient=constants::OMEGA_E/constants::SPEED_OF_LIGHT*
            u.dot(Vector3d(p.y(),-p.x(),0.));
        double rate=base;
        for(int i=0;i<5;++i) rate=base+coefficient*rate;
        EXPECT_NEAR(rate,base/(1-coefficient),1e-10);
        const auto model=doppler_rotation_rate::make(p,v,r);
        ASSERT_TRUE(model);
        const auto retained=doppler_rotation_rate::fromLos(u,p,v);
        ASSERT_TRUE(retained);
        EXPECT_DOUBLE_EQ(retained->satellite_mps,model->satellite_mps);
        EXPECT_LT((retained->receiver_velocity_jacobian-model->receiver_velocity_jacobian).norm(),1e-15);
        EXPECT_NEAR(model->satellite_mps+model->receiver_velocity_jacobian.dot(vr),rate,1e-10);
        for(int axis=0;axis<3;++axis) {
            Vector3d plus=vr,minus=vr; plus[axis]+=.01; minus[axis]-=.01;
            const double difference=(u.dot(v-plus)/(1-coefficient)-
                u.dot(v-minus)/(1-coefficient))/.02;
            EXPECT_NEAR(difference,model->receiver_velocity_jacobian[axis],1e-10);
        }
        EXPECT_LT(std::abs(coefficient),1e-4);
    }
    // Stationary case mirrors the reference wrapper's receiver assumption;
    // moving cases validate the algebraic extension, not Java implementation.
}

TEST(DopplerRotationDerivative, RotationRateModelRejectsInvalidInputs) {
    using namespace libgnss;
    EXPECT_FALSE(doppler_rotation_rate::fromLos(Vector3d::Zero(),Vector3d::Ones(),Vector3d::Ones()));
    EXPECT_FALSE(doppler_rotation_rate::make(Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero()));
    EXPECT_FALSE(doppler_rotation_rate::make(Vector3d(1.,2.,3.),
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),Vector3d::Zero()));
}

TEST(DopplerRotationDerivative, ImplicitTransmitTimeMatchesCentralDifference) {
    using namespace libgnss;
    const Vector3d s0(15600000.,20100000.,14000000.), r0(6371000.,0.,0.);
    double maximum_difference=0.;
    for(double speed:{-2500.,0.,2500.}) for(double car:{-30.,0.,30.}) {
        const Vector3d sv(speed,2100.,1300.), rv(0.,car,0.);
        const auto solve=[&](double t) -> Vector3d {
            double tau=.08;
            Vector3d p=Vector3d::Zero();
            for(int j=0;j<20;++j) {
                const double angle=constants::OMEGA_E*tau;
                Matrix3d rotation;
                rotation<<std::cos(angle),std::sin(angle),0.,
                         -std::sin(angle),std::cos(angle),0.,0.,0.,1.;
                p=rotation*(s0+sv*(t-tau));
                tau=(p-r0-rv*t).norm()/constants::SPEED_OF_LIGHT;
            }
            return p;
        };
        const Vector3d p=solve(0.), los=(p-r0).normalized();
        const double tau=(p-r0).norm()/constants::SPEED_OF_LIGHT;
        const double angle=constants::OMEGA_E*tau;
        Matrix3d rotation;
        rotation<<std::cos(angle),std::sin(angle),0.,
                 -std::sin(angle),std::cos(angle),0.,0.,0.,1.;
        const double satellite_rate=los.dot(rotation*sv);
        const double rotation_rate=constants::OMEGA_E*los.dot(Vector3d(p.y(),-p.x(),0.));
        const double approximate=satellite_rate-los.dot(rv);
        const double exact=approximate/(1.+(satellite_rate-rotation_rate)/constants::SPEED_OF_LIGHT);
        const double h=.1;
        const double numeric=((solve(h)-r0-rv*h).norm()-(solve(-h)-r0+rv*h).norm())/(2*h);
        EXPECT_NEAR(exact,numeric,2e-6);
        for(double receiver_drift:{-1e-6,0.,1e-6})
        for(double satellite_drift:{-1e-10,0.,1e-10}) {
            // Synthetic observable P=c*(receiver clock tag - satellite tag).
            // Satellite clock is evaluated at t-rho(t)/c, not receive time.
            const auto code=[&](double t) {
                const double rho=(solve(t)-r0-rv*t).norm();
                return rho+constants::SPEED_OF_LIGHT*(receiver_drift*t-
                    satellite_drift*(t-rho/constants::SPEED_OF_LIGHT));
            };
            const double receiver_tag_interval=2*h*(1+receiver_drift);
            const double observed=(code(h)-code(-h))/receiver_tag_interval;
            const double predicted=(exact+constants::SPEED_OF_LIGHT*receiver_drift-
                constants::SPEED_OF_LIGHT*satellite_drift*
                (1-exact/constants::SPEED_OF_LIGHT))/(1+receiver_drift);
            EXPECT_NEAR(observed,predicted,2e-6);
        }
        maximum_difference=std::max(maximum_difference,std::abs(exact-approximate));
    }
    std::cout<<"implicit_light_time_max_rate_difference_mps="<<maximum_difference<<'\n';
    // Linear synthetic satellite motion, no atmosphere or satellite clock.
    // This does not establish an Android observable convention or correction.
}

TEST(DopplerRotationDerivative, RotatedStateOmitsTravelTimeAngleDerivative) {
    using namespace libgnss;
    const Vector3d s(15600000.,20100000.,14000000.), v(-1800.,2200.,1300.);
    const Vector3d r(6371000.,0.,0.), rv(0.,20.,0.);
    const auto range=[&](double t) {
        Vector3d p,w;
        EXPECT_TRUE(doppler_contract::earthRotationCorrectedSatelliteState(
            s+t*v,v,r+t*rv,p,w));
        return (p-r-t*rv).norm();
    };
    Vector3d p,w,los; double known;
    ASSERT_TRUE(doppler_contract::earthRotationCorrectedSatelliteState(s,v,r,p,w));
    ASSERT_TRUE(doppler_contract::knownSatelliteRangeRate(s,v,r,true,los,known));
    const double modeled=known-los.dot(rv);
    const Vector3d raw_los=(s-r).normalized();
    const double angle_rate=constants::OMEGA_E/constants::SPEED_OF_LIGHT*raw_los.dot(v-rv);
    const double omitted=los.dot(Vector3d(p.y(),-p.x(),0.))*angle_rate;
    const double numeric=(range(.1)-range(-.1))/.2;
    EXPECT_NEAR(numeric,modeled+omitted,1e-6);
    EXPECT_GT(std::abs(omitted),1e-7);
    EXPECT_LT(std::abs(omitted),.01);
    std::cout<<"modeled_mps="<<modeled<<" angle_derivative_mps="<<omitted
             <<" finite_difference_error_mps="<<numeric-modeled-omitted<<'\n';
    // Synthetic helper consistency only. Not the derivative of a complete
    // implicit transmit-time ephemeris/clock model or a positioning diagnosis.
}
