#include <gtest/gtest.h>

#include <path_searching/timed_trajectory_builder.h>

#include <cmath>

using cane_planner::TimedTrajectoryBuilder;
using cane_planner::TimedTrajectorySource;

TEST(TimedTrajectoryBuilder, PrefersPreviousMppiBestPath)
{
    std::vector<Eigen::Vector3d> previous_mppi = {
        Eigen::Vector3d(1.0, 2.0, 0.1),
        Eigen::Vector3d(1.2, 2.1, 0.2),
        Eigen::Vector3d(1.5, 2.2, 0.3),
    };
    std::vector<Eigen::Vector2d> global_reference = {
        Eigen::Vector2d(1.0, 2.0),
        Eigen::Vector2d(2.0, 2.3),
    };

    auto timed = TimedTrajectoryBuilder::buildNominal(previous_mppi, global_reference, 0.2, 1.0, 5.0);

    ASSERT_TRUE(timed.valid());
    EXPECT_EQ(TimedTrajectorySource::PREVIOUS_MPPI, timed.source);
    ASSERT_EQ(previous_mppi.size(), timed.points.size());
    EXPECT_NEAR(1.0, timed.points[0].position.x(), 1e-9);
    EXPECT_NEAR(2.1, timed.points[1].position.y(), 1e-9);
    EXPECT_NEAR(0.3, timed.points[2].yaw, 1e-9);
    EXPECT_NEAR(0.0, timed.points[0].t_from_now, 1e-9);
    EXPECT_NEAR(0.2, timed.points[1].t_from_now, 1e-9);
    EXPECT_NEAR(0.4, timed.points[2].t_from_now, 1e-9);
}

TEST(TimedTrajectoryBuilder, UsesPreviousMppiEvenWhenItLeavesGlobalReference)
{
    std::vector<Eigen::Vector3d> previous_mppi = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.5, 2.8, 1.0),
        Eigen::Vector3d(1.0, 4.0, 1.0),
    };
    std::vector<Eigen::Vector2d> global_reference = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(4.0, 0.0),
    };

    auto timed = TimedTrajectoryBuilder::buildNominal(previous_mppi, global_reference, 0.2, 1.0, 5.0);

    ASSERT_TRUE(timed.valid());
    EXPECT_EQ(TimedTrajectorySource::PREVIOUS_MPPI, timed.source);
    EXPECT_NEAR(0.0, timed.points[0].position.x(), 1e-9);
    EXPECT_NEAR(0.0, timed.points[0].position.y(), 1e-9);
    EXPECT_NEAR(0.5, timed.points[1].position.x(), 1e-9);
    EXPECT_NEAR(2.8, timed.points[1].position.y(), 1e-9);
}

TEST(TimedTrajectoryBuilder, FallsBackToAstarBootstrapWithDistanceTiming)
{
    std::vector<Eigen::Vector3d> previous_mppi;
    std::vector<Eigen::Vector2d> global_reference = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(1.0, 1.0),
    };

    auto timed = TimedTrajectoryBuilder::buildNominal(previous_mppi, global_reference, 0.2, 0.5, 5.0);

    ASSERT_TRUE(timed.valid());
    EXPECT_EQ(TimedTrajectorySource::ASTAR_BOOTSTRAP, timed.source);
    ASSERT_EQ(global_reference.size(), timed.points.size());
    EXPECT_NEAR(0.0, timed.points[0].t_from_now, 1e-9);
    EXPECT_NEAR(2.0, timed.points[1].t_from_now, 1e-9);
    EXPECT_NEAR(4.0, timed.points[2].t_from_now, 1e-9);
    EXPECT_NEAR(0.0, timed.points[1].yaw, 1e-9);
    EXPECT_NEAR(M_PI / 2.0, timed.points[2].yaw, 1e-9);
}

// The corridor built around the previous prediction moves with the robot, so
// lateral drift never shrinks its margin. Preferring the route keeps the wall
// still. Same inputs as UsesPreviousMppiEvenWhenItLeavesGlobalReference above.
TEST(TimedTrajectoryBuilder, PrefersGlobalReferenceWhenAsked)
{
    std::vector<Eigen::Vector3d> previous_mppi = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.5, 2.8, 1.0),
        Eigen::Vector3d(1.0, 4.0, 1.0),
    };
    std::vector<Eigen::Vector2d> global_reference = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(4.0, 0.0),
    };

    auto timed = TimedTrajectoryBuilder::buildNominal(previous_mppi, global_reference, 0.2, 1.0, 5.0,
                                                     true);

    ASSERT_TRUE(timed.valid());
    EXPECT_EQ(TimedTrajectorySource::ASTAR_BOOTSTRAP, timed.source);
    EXPECT_NEAR(2.0, timed.points[1].position.x(), 1e-9);
    EXPECT_NEAR(0.0, timed.points[1].position.y(), 1e-9);
    // Distance-based timing, not the MPPI step index.
    EXPECT_NEAR(2.0, timed.points[1].t_from_now, 1e-9);
}

// The route is unusable on the final approach, where fewer than two points are
// left; the prediction has to remain the fallback or the corridor disappears.
TEST(TimedTrajectoryBuilder, FallsBackToPreviousMppiWhenGlobalReferenceIsUnusable)
{
    std::vector<Eigen::Vector3d> previous_mppi = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.4, 0.0, 0.0),
        Eigen::Vector3d(0.8, 0.0, 0.0),
    };
    std::vector<Eigen::Vector2d> global_reference = {Eigen::Vector2d(0.0, 0.0)};

    auto timed = TimedTrajectoryBuilder::buildNominal(previous_mppi, global_reference, 0.2, 1.0, 5.0,
                                                     true);

    ASSERT_TRUE(timed.valid());
    EXPECT_EQ(TimedTrajectorySource::PREVIOUS_MPPI, timed.source);
    EXPECT_NEAR(0.4, timed.points[1].position.x(), 1e-9);
    // Step-index timing (1 * mppi_dt), not the distance-based timing the global
    // reference would have produced.
    EXPECT_NEAR(0.2, timed.points[1].t_from_now, 1e-9);
}

// A prediction that still describes the route is the better centreline: it is
// the only one the robot can hold through a corner.
TEST(TimedTrajectoryBuilder, KeepsPreviousMppiWhileItStillDescribesTheRoute)
{
    std::vector<Eigen::Vector3d> previous_mppi = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.2, 0.0),
        Eigen::Vector3d(2.0, 0.5, 0.0),
    };
    std::vector<Eigen::Vector2d> global_reference = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(4.0, 0.0),
    };

    EXPECT_NEAR(0.5,
                TimedTrajectoryBuilder::maxDeviationFromRoute(previous_mppi, global_reference, 5.0),
                1e-9);

    auto timed = TimedTrajectoryBuilder::buildNominal(previous_mppi, global_reference, 0.2, 1.0, 5.0,
                                                     false, 0.6);

    ASSERT_TRUE(timed.valid());
    EXPECT_EQ(TimedTrajectorySource::PREVIOUS_MPPI, timed.source);
}

// Once it has wandered, a corridor around it is a corridor around the
// wandering, and drifting further never shrinks the margin.
TEST(TimedTrajectoryBuilder, SwitchesToRouteWhenThePredictionHasWandered)
{
    std::vector<Eigen::Vector3d> previous_mppi = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.5, 2.8, 1.0),
        Eigen::Vector3d(1.0, 4.0, 1.0),
    };
    std::vector<Eigen::Vector2d> global_reference = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(4.0, 0.0),
    };

    auto timed = TimedTrajectoryBuilder::buildNominal(previous_mppi, global_reference, 0.2, 1.0, 5.0,
                                                     false, 0.6);

    ASSERT_TRUE(timed.valid());
    EXPECT_EQ(TimedTrajectorySource::ASTAR_BOOTSTRAP, timed.source);
}

// Past the end of the route every point projects onto its last vertex, so an
// unclipped measurement would reject a prediction that never left the route.
TEST(TimedTrajectoryBuilder, DeviationIsMeasuredOnlyWhereTheRouteReaches)
{
    std::vector<Eigen::Vector3d> previous_mppi = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(1.5, 0.0, 0.0),
        Eigen::Vector3d(3.0, 0.0, 0.0),
    };
    std::vector<Eigen::Vector2d> global_reference = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
    };

    EXPECT_NEAR(0.0,
                TimedTrajectoryBuilder::maxDeviationFromRoute(previous_mppi, global_reference, 5.0),
                1e-9);

    auto timed = TimedTrajectoryBuilder::buildNominal(previous_mppi, global_reference, 0.2, 1.0, 5.0,
                                                     false, 0.6);

    ASSERT_TRUE(timed.valid());
    EXPECT_EQ(TimedTrajectorySource::PREVIOUS_MPPI, timed.source);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
