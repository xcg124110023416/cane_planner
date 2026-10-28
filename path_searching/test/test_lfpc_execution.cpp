#include <gtest/gtest.h>
#include <path_searching/lfpc.h>
using namespace cane_planner;
TEST(LfpcExecution, SamplingPreservesRootAndMatchesCanonicalStepEndpoint) {
    ros::NodeHandle nh("/lfpc_execution");nh.setParam("lfpc/t_sup",.35);nh.setParam("lfpc/delta_t",.07);
    LFPC root;root.initializeModel(nh);root.reset({.8,-.2,.3},{1.,2.,0.},LEFT_LEG,4);
    const auto initial=root.getState();const auto position=root.getCOMPos();const auto leg=root.getSupportFeet();
    Eigen::Vector3d control(.4,.03,.1);LFPC end;end.copyState(root);
    end.SetCtrlParams(control);end.updateOneStep();const auto duration=end.getStepCOMTimes().back();end.prepareNextStep();
    EXPECT_DOUBLE_EQ(duration,root.stepDuration());
    LFPC sample;ASSERT_TRUE(root.sampleControlledStep(control,duration*.5,sample));
    EXPECT_NE(position,sample.getCOMPos());EXPECT_EQ(leg,sample.getSupportFeet());EXPECT_EQ(4,sample.getStepNum());
    EXPECT_DOUBLE_EQ(.5*duration,sample.getPhaseTime());
    ASSERT_TRUE(root.sampleControlledStep(control,duration,sample));
    EXPECT_TRUE(end.getCOMPos().isApprox(sample.getCOMPos(),1e-12));
    EXPECT_TRUE(end.getState().isApprox(sample.getState(),1e-12));
    EXPECT_EQ(end.getSupportFeet(),sample.getSupportFeet());EXPECT_EQ(end.getStepNum(),sample.getStepNum());
    EXPECT_DOUBLE_EQ(0.,sample.getPhaseTime());EXPECT_EQ(initial,root.getState());EXPECT_EQ(position,root.getCOMPos());
}
TEST(LfpcExecution, BadTimeAndMidPhaseRootsCannotCreateAnExecutedState) {
    ros::NodeHandle nh("/lfpc_execution");LFPC root;root.initializeModel(nh);root.reset({.5,0,0},{0,0,0},LEFT_LEG,0);
    LFPC sample;const Eigen::Vector3d control(.4,.03,0);
    EXPECT_FALSE(root.sampleControlledStep(control,-.01,sample));
    EXPECT_FALSE(root.sampleControlledStep(control,root.stepDuration()+.01,sample));
    EXPECT_FALSE(root.sampleControlledStep(control,std::numeric_limits<double>::quiet_NaN(),sample));
    ASSERT_TRUE(root.sampleControlledStep(control,.02,sample));LFPC invalid;
    EXPECT_FALSE(sample.sampleControlledStep(control,.02,invalid));
}
int main(int argc,char** argv){ros::init(argc,argv,"test_lfpc_execution");testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
