#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace {
// Restricted linear position/clock control, NOT the full native C7/V/D
// graph. Neighbor displacement/clock increments are assumed observed.
Eigen::MatrixXd system(bool links, bool geometry) {
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(links ? 16 : 8, 12);
    Eigen::Matrix4d h;
    h << 1,0,0,1, 0,1,0,1, 0,0,1,1, -1,0,0,1;
    if (!geometry) for (int i=1;i<4;++i) h.row(i)=h.row(0);
    a.block<4,4>(0,0)=h;
    a.block<4,4>(4,8)=h;
    if (links) for (int i=0;i<2;++i) {
        a.block<4,4>(8+4*i,4*i)=-Eigen::Matrix4d::Identity();
        a.block<4,4>(8+4*i,4*i+4)=Eigen::Matrix4d::Identity();
    }
    return a;
}
}

TEST(SparseEpochRankControl, ConnectedEmptyMiddleEpochCanBeObservable) {
    const auto a=system(true,true);
    EXPECT_EQ(a.fullPivLu().rank(),12);
    Eigen::VectorXd truth=Eigen::VectorXd::LinSpaced(12,-2,3);
    const Eigen::VectorXd recovered=a.colPivHouseholderQr().solve(a*truth);
    EXPECT_LT((recovered-truth).norm(),1e-12);
}
TEST(SparseEpochRankControl, DisconnectedEmptyEpochIsUnobservable) {
    EXPECT_EQ(system(false,true).fullPivLu().rank(),8);
}
TEST(SparseEpochRankControl, FourRowsDoNotGuaranteeGeometryRank) {
    EXPECT_LT(system(true,false).fullPivLu().rank(),12);
}
