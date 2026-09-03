#include <gtest/gtest.h>

#include <path_searching/timed_walking_corridor.h>
#include <path_searching/trajectory_feasibility.h>

using cane_planner::TimedTrajectorySource;
using cane_planner::TimedWalkingCorridor;
using cane_planner::TimedWalkingCorridorSegment;
using cane_planner::TrajectoryFeasibility;
using cane_planner::corridorDeviationFeasible;
using cane_planner::selectBestTrajectoryIndex;

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

TEST(TimedWalkingCorridor, HalfPlanesOverrideCenterlineBandDistance)
{
    TimedWalkingCorridor corridor;
    TimedWalkingCorridorSegment segment;
    segment.t_start = 0.0;
    segment.t_end = 1.0;
    segment.start = Eigen::Vector2d(0.0, 0.0);
    segment.end = Eigen::Vector2d(1.0, 0.0);
    segment.forward = Eigen::Vector2d::UnitX();
    segment.left = Eigen::Vector2d::UnitY();
    segment.half_width = 0.1;
    using HalfPlane2D = TimedWalkingCorridorSegment::HalfPlane2D;
    segment.half_planes = {
        HalfPlane2D{Eigen::Vector2d(1.0, 0.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(-1.0, 0.0), 0.0},
        HalfPlane2D{Eigen::Vector2d(0.0, 1.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(0.0, -1.0), 0.0},
    };
    corridor.segments = {segment};

    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              0.5, Eigen::Vector2d(0.5, 0.8)));
    EXPECT_NEAR(0.2, corridor.outsideDistanceAtTime(
                         0.5, Eigen::Vector2d(1.2, 0.5)), 1e-9);
}

TEST(TimedWalkingCorridor, AllowsAdjacentConvexCellContinuityAtTurns)
{
    using HalfPlane2D = TimedWalkingCorridorSegment::HalfPlane2D;
    TimedWalkingCorridor corridor;

    TimedWalkingCorridorSegment first;
    first.t_start = 0.0;
    first.t_end = 0.5;
    first.half_planes = {
        HalfPlane2D{Eigen::Vector2d(1.0, 0.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(-1.0, 0.0), 0.0},
        HalfPlane2D{Eigen::Vector2d(0.0, 1.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(0.0, -1.0), 0.0},
    };

    TimedWalkingCorridorSegment second;
    second.t_start = 0.5;
    second.t_end = 1.0;
    second.half_planes = {
        HalfPlane2D{Eigen::Vector2d(1.0, 0.0), -2.0},
        HalfPlane2D{Eigen::Vector2d(-1.0, 0.0), 1.0},
        HalfPlane2D{Eigen::Vector2d(0.0, 1.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(0.0, -1.0), 0.0},
    };
    corridor.segments = {first, second};

    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              0.25, Eigen::Vector2d(1.5, 0.5)));
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

TEST(TrajectoryFeasibility, ReportsNoFeasibleTrajectoryStopReason)
{
    TrajectoryFeasibility infeasible;
    infeasible.valid_trajectory_count = 8;
    infeasible.corridor_feasible_trajectory_count = 0;
    infeasible.corridor_evaluated = true;

    EXPECT_TRUE(infeasible.shouldStop());
    EXPECT_EQ("NO_FEASIBLE_TRAJECTORY", infeasible.stopReason());

    TrajectoryFeasibility feasible;
    feasible.valid_trajectory_count = 8;
    feasible.corridor_feasible_trajectory_count = 2;
    feasible.corridor_evaluated = true;

    EXPECT_FALSE(feasible.shouldStop());
    EXPECT_EQ("OK", feasible.stopReason());
}

TEST(TrajectoryFeasibility, SelectsBestInsideCorridorWhenAvailable)
{
    const std::vector<double> costs = {4.0, 1.0, 2.0};
    const std::vector<bool> corridor_feasible = {true, false, true};

    EXPECT_EQ(2, selectBestTrajectoryIndex(costs, corridor_feasible, true));
    EXPECT_EQ(1, selectBestTrajectoryIndex(costs, corridor_feasible, false));
}

TEST(TrajectoryFeasibility, AllowsBoundedCorridorDeviation)
{
    EXPECT_TRUE(corridorDeviationFeasible(0.0, 0.3));
    EXPECT_TRUE(corridorDeviationFeasible(0.2, 0.3));
    EXPECT_FALSE(corridorDeviationFeasible(0.31, 0.3));
}

TEST(TrajectoryFeasibility, FallsBackToBestFiniteCostWhenNoCorridorFeasibleSample)
{
    const std::vector<double> costs = {
        std::numeric_limits<double>::infinity(),
        3.0,
        2.0,
    };
    const std::vector<bool> corridor_feasible = {false, false, false};

    EXPECT_EQ(2, selectBestTrajectoryIndex(costs, corridor_feasible, true));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(TimedWalkingCorridor, SignedMarginGradesTheInsideOfTheCell)
{
    TimedWalkingCorridor corridor;
    TimedWalkingCorridorSegment segment;
    segment.t_start = 0.0;
    segment.t_end = 1.0;
    segment.start = Eigen::Vector2d(0.0, 0.0);
    segment.end = Eigen::Vector2d(2.0, 0.0);
    segment.forward = Eigen::Vector2d::UnitX();
    segment.left = Eigen::Vector2d::UnitY();
    segment.half_width = 0.4;
    // Lateral band |y| <= 0.4 expressed as half-planes, normals not unit length
    // so the normalisation is exercised too.
    TimedWalkingCorridorSegment::HalfPlane2D upper;
    upper.normal = Eigen::Vector2d(0.0, 2.0);
    upper.offset = -0.8;
    TimedWalkingCorridorSegment::HalfPlane2D lower;
    lower.normal = Eigen::Vector2d(0.0, -2.0);
    lower.offset = -0.8;
    TimedWalkingCorridorSegment::HalfPlane2D back;
    back.normal = Eigen::Vector2d(-1.0, 0.0);
    back.offset = 0.0;
    TimedWalkingCorridorSegment::HalfPlane2D front;
    front.normal = Eigen::Vector2d(1.0, 0.0);
    front.offset = -2.0;
    segment.half_planes = {upper, lower, back, front};
    corridor.segments = {segment};

    // Inside: the margin is the distance to the nearest boundary, and it grades
    // instead of saturating at zero the way -outsideDistance did.
    const double centre = corridor.signedMarginAtTimeWithNeighbours(
        0.5, Eigen::Vector2d(1.0, 0.0));
    const double near_edge = corridor.signedMarginAtTimeWithNeighbours(
        0.5, Eigen::Vector2d(1.0, 0.3));
    EXPECT_NEAR(0.4, centre, 1e-9);
    EXPECT_NEAR(0.1, near_edge, 1e-9);
    EXPECT_GT(centre, near_edge);
    EXPECT_NEAR(0.0, corridor.outsideDistanceAtTime(0.5, Eigen::Vector2d(1.0, 0.3)), 1e-9);

    // Outside: it must stay the exact negation of outsideDistanceAtTime so the
    // existing safety threshold keeps its current results.
    for (double y : {0.45, 0.6, 1.2})
    {
        const Eigen::Vector2d p(1.0, y);
        EXPECT_NEAR(-corridor.outsideDistanceAtTime(0.5, p),
                    corridor.signedMarginAtTimeWithNeighbours(0.5, p), 1e-9);
    }

    // Uncovered times have no margin at all.
    EXPECT_FALSE(std::isfinite(corridor.signedMarginAtTimeWithNeighbours(
        2.0, Eigen::Vector2d(1.0, 0.0))));
}

TEST(TimedWalkingCorridor, SignedMarginFoldsInDynamicPenetration)
{
    TimedWalkingCorridor corridor;
    TimedWalkingCorridorSegment segment;
    segment.t_start = 0.0;
    segment.t_end = 1.0;
    segment.start = Eigen::Vector2d(0.0, 0.0);
    segment.end = Eigen::Vector2d(2.0, 0.0);
    segment.forward = Eigen::Vector2d::UnitX();
    segment.left = Eigen::Vector2d::UnitY();
    segment.half_width = 0.5;

    TimedWalkingCorridorSegment::DynamicEllipseObstacle obstacle;
    obstacle.center = Eigen::Vector2d(1.0, 0.0);
    obstacle.forward = Eigen::Vector2d::UnitX();
    obstacle.left = Eigen::Vector2d::UnitY();
    obstacle.longitudinal_radius = 0.3;
    obstacle.lateral_radius = 0.3;
    segment.dynamic_obstacles = {obstacle};
    corridor.segments = {segment};

    // Geometrically inside the cell, but inside the pedestrian keep-out: the
    // margin must go negative even though the cell itself is clear.
    const double inside_obstacle = corridor.signedMarginAtTimeWithNeighbours(
        0.5, Eigen::Vector2d(1.0, 0.0));
    EXPECT_LT(inside_obstacle, 0.0);
    EXPECT_NEAR(-corridor.dynamicObstacleViolationAtTime(0.5, Eigen::Vector2d(1.0, 0.0)),
                inside_obstacle, 1e-9);

    // Clear of the pedestrian, still inside the cell: positive and graded.
    EXPECT_GT(corridor.signedMarginAtTimeWithNeighbours(0.5, Eigen::Vector2d(1.9, 0.0)), 0.0);
}

// Reproduces why the 2-D margin was useless in the pilot bags: an eroded cell
// only just contains its own endpoints, so any trajectory that starts or ends
// at a cell boundary pins the 2-D minimum at ~0 regardless of the human's
// lateral position. The lateral margin has to ignore those faces.
TEST(TimedWalkingCorridor, LateralMarginIgnoresEntryAndExitFaces)
{
    TimedWalkingCorridor corridor;
    TimedWalkingCorridorSegment segment;
    segment.t_start = 0.0;
    segment.t_end = 1.0;
    segment.start = Eigen::Vector2d(0.0, 0.0);
    segment.end = Eigen::Vector2d(2.0, 0.0);
    segment.forward = Eigen::Vector2d::UnitX();
    segment.left = Eigen::Vector2d::UnitY();
    segment.half_width = 0.4;

    TimedWalkingCorridorSegment::HalfPlane2D upper;   // y <= 0.4
    upper.normal = Eigen::Vector2d(0.0, 1.0);
    upper.offset = -0.4;
    TimedWalkingCorridorSegment::HalfPlane2D lower;   // y >= -0.4
    lower.normal = Eigen::Vector2d(0.0, -1.0);
    lower.offset = -0.4;
    TimedWalkingCorridorSegment::HalfPlane2D back;    // x >= 0, touches the start
    back.normal = Eigen::Vector2d(-1.0, 0.0);
    back.offset = 0.0;
    TimedWalkingCorridorSegment::HalfPlane2D front;   // x <= 2, touches the end
    front.normal = Eigen::Vector2d(1.0, 0.0);
    front.offset = -2.0;
    segment.half_planes = {upper, lower, back, front};
    corridor.segments = {segment};

    const Eigen::Vector2d at_start(0.0, 0.0);
    const Eigen::Vector2d at_start_off(0.0, 0.3);

    // 2-D margin: pinned at zero by the entry face, and it cannot tell the two
    // lateral positions apart. This is exactly what the bags showed.
    EXPECT_NEAR(0.0, corridor.signedMarginAtTimeWithNeighbours(0.5, at_start), 1e-9);
    EXPECT_NEAR(0.0, corridor.signedMarginAtTimeWithNeighbours(0.5, at_start_off), 1e-9);

    // Lateral margin: grades the width even at the cell entry.
    EXPECT_NEAR(0.4, corridor.lateralMarginAtTime(0.5, at_start), 1e-9);
    EXPECT_NEAR(0.1, corridor.lateralMarginAtTime(0.5, at_start_off), 1e-9);
    EXPECT_NEAR(-0.2, corridor.lateralMarginAtTime(0.5, Eigen::Vector2d(1.0, 0.6)), 1e-9);

    // No half-planes: fall back to the centreline band.
    TimedWalkingCorridor band = corridor;
    band.segments[0].half_planes.clear();
    EXPECT_NEAR(0.25, band.lateralMarginAtTime(0.5, Eigen::Vector2d(1.0, 0.15)), 1e-9);

    // Uncovered time still has no margin.
    EXPECT_FALSE(std::isfinite(corridor.lateralMarginAtTime(2.0, at_start)));
}

// Regression guard for the clamp that pinned both margins at zero: the helpers
// already fold dynamic penetration in, so a caller must not additionally clamp
// them against -dynamicObstacleViolationAtTime(), which is 0 whenever no
// pedestrian is penetrated and would therefore cap every positive margin.
TEST(TimedWalkingCorridor, MarginsStayPositiveWithNonPenetratedObstacles)
{
    TimedWalkingCorridor corridor;
    TimedWalkingCorridorSegment segment;
    segment.t_start = 0.0;
    segment.t_end = 1.0;
    segment.start = Eigen::Vector2d(0.0, 0.0);
    segment.end = Eigen::Vector2d(2.0, 0.0);
    segment.forward = Eigen::Vector2d::UnitX();
    segment.left = Eigen::Vector2d::UnitY();
    segment.half_width = 0.5;

    TimedWalkingCorridorSegment::DynamicEllipseObstacle far_obstacle;
    far_obstacle.center = Eigen::Vector2d(1.0, 5.0);   // nowhere near the cell
    far_obstacle.forward = Eigen::Vector2d::UnitX();
    far_obstacle.left = Eigen::Vector2d::UnitY();
    far_obstacle.longitudinal_radius = 0.3;
    far_obstacle.lateral_radius = 0.3;
    segment.dynamic_obstacles = {far_obstacle};
    corridor.segments = {segment};

    const Eigen::Vector2d centre(1.0, 0.0);
    EXPECT_NEAR(0.0, corridor.dynamicObstacleViolationAtTime(0.5, centre), 1e-9);
    EXPECT_GT(corridor.signedMarginAtTimeWithNeighbours(0.5, centre), 0.4);
    EXPECT_GT(corridor.lateralMarginAtTime(0.5, centre), 0.4);
}
