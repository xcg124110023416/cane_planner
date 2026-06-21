#include <gtest/gtest.h>

#include <path_searching/timed_walking_corridor.h>
#include <path_searching/trajectory_feasibility.h>

using cane_planner::TimedTrajectorySource;
using cane_planner::TimedWalkingCorridor;
using cane_planner::TimedWalkingCorridorSegment;
using cane_planner::TrajectoryFeasibility;

TEST(TimedWalkingCorridor, TracksTimeRangeAndSegmentLookup)
{
    TimedWalkingCorridor corridor;
    corridor.source = TimedTrajectorySource::PREVIOUS_MPPI;

    TimedWalkingCorridorSegment first;
    first.t_start = 0.0;
    first.t_end = 0.5;
    first.half_width = 0.4;
    first.centerline = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0)};

    TimedWalkingCorridorSegment second;
    second.t_start = 0.5;
    second.t_end = 1.0;
    second.half_width = 0.4;
    second.centerline = {Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(1.0, 1.0)};

    corridor.segments = {first, second};

    ASSERT_TRUE(corridor.valid());
    EXPECT_NEAR(0.0, corridor.tStart(), 1e-9);
    EXPECT_NEAR(1.0, corridor.tEnd(), 1e-9);
    ASSERT_NE(nullptr, corridor.segmentAtTime(0.25));
    EXPECT_NEAR(1.0, corridor.segmentAtTime(0.75)->centerline.back().y(), 1e-9);
    EXPECT_EQ(nullptr, corridor.segmentAtTime(1.5));
}

TEST(TimedWalkingCorridor, ComputesOutsideDistanceAtSegmentTime)
{
    TimedWalkingCorridor corridor;
    TimedWalkingCorridorSegment first;
    first.t_start = 0.0;
    first.t_end = 0.5;
    first.start = Eigen::Vector2d(0.0, 0.0);
    first.end = Eigen::Vector2d(1.0, 0.0);
    first.forward = Eigen::Vector2d::UnitX();
    first.left = Eigen::Vector2d::UnitY();
    first.half_width = 0.4;

    TimedWalkingCorridorSegment second;
    second.t_start = 0.5;
    second.t_end = 1.0;
    second.start = Eigen::Vector2d(1.0, 0.0);
    second.end = Eigen::Vector2d(1.0, 1.0);
    second.forward = Eigen::Vector2d::UnitY();
    second.left = Eigen::Vector2d(-1.0, 0.0);
    second.half_width = 0.4;
    corridor.segments = {first, second};

    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              0.25, Eigen::Vector2d(0.5, 0.2)));
    EXPECT_NEAR(0.2, corridor.outsideDistanceAtTime(
                         0.25, Eigen::Vector2d(0.5, 0.6)), 1e-9);
    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              0.75, Eigen::Vector2d(0.8, 0.5)));
    EXPECT_NEAR(0.2, corridor.outsideDistanceAtTime(
                         0.75, Eigen::Vector2d(0.4, 0.5)), 1e-9);
    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              1.5, Eigen::Vector2d(5.0, 5.0)));
}

TEST(TrajectoryFeasibility, EncodesGoStopFromMppiFeasibleTrajectory)
{
    TrajectoryFeasibility feasible;
    feasible.valid_trajectory_count = 3;
    feasible.inside_corridor_ratio = 0.82;

    EXPECT_TRUE(feasible.hasFeasibleTrajectory());
    EXPECT_FALSE(feasible.shouldStop());

    TrajectoryFeasibility infeasible;
    infeasible.valid_trajectory_count = 0;
    infeasible.inside_corridor_ratio = 0.0;

    EXPECT_FALSE(infeasible.hasFeasibleTrajectory());
    EXPECT_TRUE(infeasible.shouldStop());
}

TEST(TrajectoryFeasibility, CorridorEvaluationOverridesGenericValidity)
{
    TrajectoryFeasibility feasibility;
    feasibility.valid_trajectory_count = 12;
    feasibility.corridor_feasible_trajectory_count = 0;
    feasibility.corridor_evaluated = true;
    feasibility.inside_corridor_ratio = 0.25;

    EXPECT_FALSE(feasibility.hasFeasibleTrajectory());
    EXPECT_TRUE(feasibility.shouldStop());

    feasibility.corridor_feasible_trajectory_count = 1;
    EXPECT_TRUE(feasibility.hasFeasibleTrajectory());
    EXPECT_FALSE(feasibility.shouldStop());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
