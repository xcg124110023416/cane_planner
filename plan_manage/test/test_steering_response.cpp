#include <gtest/gtest.h>
#include <plan_manage/steering_response.h>
#include <limits>

using cane_planner::SteeringResponse;

TEST(SteeringResponseTest, ZeroDelayPreservesRequestedTurnAcrossAngleWrap) {
    SteeringResponse r;ASSERT_TRUE(r.setDelay(0.));r.reset(3.1);double turn;
    ASSERT_TRUE(r.command(10.,3.1,.2,turn));EXPECT_NEAR(.2,turn,1e-12);
    EXPECT_EQ(1u,r.selected().sequence);EXPECT_EQ(0u,r.pending());
    EXPECT_EQ(0u,r.applied().sequence); // not applied merely by selection
    r.acknowledge(10.02);EXPECT_NEAR(.02,r.appliedAt()-r.applied().issued,1e-12);
}
TEST(SteeringResponseTest, HoldsHeadingUntilDueAtABoundaryThenDoesNotAccumulateTurn) {
    SteeringResponse r;r.reset(.4);double turn;
    ASSERT_TRUE(r.command(10.,.4,.3,turn));EXPECT_DOUBLE_EQ(0.,turn);
    ASSERT_TRUE(r.command(10.28,.4,.3,turn));EXPECT_DOUBLE_EQ(0.,turn);
    ASSERT_TRUE(r.command(10.56,.4,.3,turn));EXPECT_NEAR(.3,turn,1e-12);
    EXPECT_EQ(1u,r.selected().sequence);r.acknowledge(10.58);
    EXPECT_NEAR(.58,r.appliedAt()-r.applied().issued,1e-12);
    ASSERT_TRUE(r.command(10.60,.7,0.,turn));EXPECT_NEAR(0.,turn,1e-12);
    EXPECT_NEAR(.7,r.selected().yaw,1e-12); // previous absolute cue, not another +.3
}
TEST(SteeringResponseTest, ConsumesLatestDueCueWithoutWaitingForNewerPendingCue) {
    SteeringResponse r;r.reset(0.);double turn;
    ASSERT_TRUE(r.command(10.,0.,.1,turn));
    ASSERT_TRUE(r.command(10.1,0.,.2,turn));
    ASSERT_TRUE(r.command(10.2,0.,.3,turn));
    ASSERT_TRUE(r.command(10.45,0.,.4,turn));
    EXPECT_NEAR(.2,turn,1e-12);EXPECT_EQ(2u,r.selected().sequence);EXPECT_EQ(2u,r.pending());
}
TEST(SteeringResponseTest, ResetClearsOldGoalCuesAndApplicationHistory) {
    SteeringResponse r;r.reset(0.);double turn;
    ASSERT_TRUE(r.command(10.,0.,.8,turn));r.reset(-.6);
    ASSERT_TRUE(r.command(20.,-.6,.2,turn));EXPECT_NEAR(0.,turn,1e-12);
    EXPECT_EQ(0u,r.selected().sequence);EXPECT_EQ(0u,r.applied().sequence);
    EXPECT_EQ(1u,r.pending());EXPECT_EQ(1u,r.desired().sequence);
}
TEST(SteeringResponseTest, InvalidInputsCannotQueueOrApplyACue) {
    SteeringResponse r;const auto nan=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(r.setDelay(nan));EXPECT_FALSE(r.setDelay(-.1));r.reset(0.);double turn=9.;
    EXPECT_FALSE(r.command(0.,0.,0.,turn));EXPECT_FALSE(r.command(10.,0.,nan,turn));
    EXPECT_EQ(0u,r.pending());EXPECT_DOUBLE_EQ(9.,turn);
    ASSERT_TRUE(r.command(10.,0.,.2,turn));EXPECT_FALSE(r.command(9.,0.,.3,turn));
    EXPECT_EQ(1u,r.pending());EXPECT_EQ(1u,r.desired().sequence);
}

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
