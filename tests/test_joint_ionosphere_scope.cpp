#include <gtest/gtest.h>
#include <libgnss++/algorithms/fgo.hpp>
#include <limits>

TEST(JointIonosphereScope, DefaultOffAndExplicitParametersRequired) {
    libgnss::FGOProcessor::FGOConfig c;
    EXPECT_FALSE(c.use_native_joint_ionosphere);
    EXPECT_DOUBLE_EQ(c.native_joint_ionosphere_anchor_sigma_m,0.);
    EXPECT_DOUBLE_EQ(c.native_joint_ionosphere_density_m_sqrt_s,0.);
    EXPECT_DOUBLE_EQ(c.native_joint_ionosphere_max_gap_s,0.);
}

TEST(JointIonosphereScope, UnsupportedModesFailAtDedicatedGuard) {
    using Processor=libgnss::FGOProcessor;
    Processor::FGOConfig supported;
    supported.use_native_joint_ionosphere=true;
    supported.backend=libgnss::FGOBackend::GTSAM;
    supported.use_imu=supported.use_pose3_state=true;
    supported.use_native_phase171_raw_p_no_doppler_imu_main=true;
    supported.use_native_source_clock_c0d_gnss_first_meter_state_handoff=true;
    supported.native_joint_ionosphere_anchor_sigma_m=10.;
    supported.native_joint_ionosphere_density_m_sqrt_s=1.;
    supported.native_joint_ionosphere_max_gap_s=2.;
    for(int variant=0;variant<12;++variant) {
        auto c=supported;
        Processor::FGOProblem p; p.imu.valid=true;
        if(variant==0) c.backend=libgnss::FGOBackend::Eigen;
        if(variant==1) c.use_imu=false;
        if(variant==2) c.use_pose3_state=false;
        if(variant==3) p.imu.valid=false;
        if(variant==4) c.use_fixed_lag_smoother=true;
        if(variant==5) c.use_native_phase171_raw_p_no_doppler_imu_main=false;
        if(variant==6) c.use_native_source_clock_c0d_gnss_first_meter_state_handoff=false;
        if(variant==7) c.use_residual_ionosphere_states=true;
        if(variant==8) c.use_native_tdcp_frequency_residual_states=true;
        if(variant==9) c.native_joint_ionosphere_anchor_sigma_m=0.;
        if(variant==10) c.native_joint_ionosphere_density_m_sqrt_s=-1.;
        if(variant==11) c.native_joint_ionosphere_max_gap_s=std::numeric_limits<double>::quiet_NaN();
        try {
            Processor(c).optimizeProblem(p);
            FAIL()<<"accepted variant "<<variant;
        } catch(const std::invalid_argument& e) {
            EXPECT_NE(std::string(e.what()).find("Joint ionosphere requires"),std::string::npos);
        }
    }
}
