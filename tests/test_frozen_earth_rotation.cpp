#include <gtest/gtest.h>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>
#include "../src/algorithms/fgo_internal.hpp"
#include <iostream>

TEST(FrozenEarthRotation, SeedPerturbationRangeErrorObeysRotationBound) {
    using namespace libgnss;
    const Vector3d receiver(6371000.,0.,0.);
    for(double distance:{1.,10.,100.,1000.}) {
        double maximum=0.;
        for(int az=0;az<24;++az) for(int el=1;el<9;++el) {
            const double a=az*2.*3.141592653589793/24., e=el*.16;
            const Vector3d satellite=receiver+22000000.*Vector3d(
                std::sin(e),std::cos(e)*std::cos(a),std::cos(e)*std::sin(a));
            const Vector3d exact=fgo_internal::earthRotationCorrected(satellite,receiver);
            for(int axis=0;axis<3;++axis) for(double sign:{-1.,1.}) {
                Vector3d seed=receiver; seed[axis]+=sign*distance;
                const Vector3d frozen=fgo_internal::earthRotationCorrected(satellite,seed);
                const double error=std::abs((frozen-receiver).norm()-(exact-receiver).norm());
                maximum=std::max(maximum,error);
                // Reverse triangle inequality bounds range and travel-time
                // differences; a rotation chord is bounded by radius*angle.
                const double bound=satellite.head<2>().norm()*constants::OMEGA_E*
                    distance/constants::SPEED_OF_LIGHT;
                EXPECT_LE(error,bound+1e-8);
            }
        }
        std::cout<<"seed_offset_m="<<distance<<" max_range_error_m="<<maximum<<'\n';
        EXPECT_LT(maximum, distance*1e-5);
    }
}
