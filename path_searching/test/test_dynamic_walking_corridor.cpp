#include <gtest/gtest.h>

#include <path_searching/dynamic_walking_corridor.h>
#include <path_searching/timed_trajectory.h>

using cane_planner::DynamicWalkingCorridor;
using cane_planner::TimedTrajectory;
using cane_planner::TimedTrajectoryPoint;
using cane_planner::TimedTrajectorySource;

TEST(DynamicWalkingCorridor, SelectsBehindSideForCrossingPedestrian)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.35;
    cfg.lateral_shift = 1.0;
    cfg.prediction_horizon = 3.0;
    cfg.prediction_dt = 0.25;
    cfg.robot_speed = 1.0;
    cfg.robot_radius = 0.25;
    cfg.dynamic_safety_margin = 0.10;
    cfg.front_pass_weight = 10.0;
    cfg.lateral_offset_weight = 0.2;
    cfg.lateral_candidates_enable = true;
    cfg.dynamic_blocking_enable = true;
    corridor.setConfig(cfg);

    Eigen::Vector2d robot(0.0, 0.0);
    Eigen::Vector2d forward(1.0, 0.0);
    std::vector<Eigen::Vector3d> obs_pos = {Eigen::Vector3d(2.0, 0.0, 0.0)};
    std::vector<Eigen::Vector3d> obs_vel = {Eigen::Vector3d(0.0, 0.5, 0.0)};
    std::vector<Eigen::Vector3d> obs_size = {Eigen::Vector3d(0.5, 0.5, 1.7)};

    auto result = corridor.plan(robot, forward, obs_pos, obs_vel, obs_size);

    ASSERT_TRUE(result.has_feasible);
    // For a pedestrian moving upward, the negative-y side is behind the pedestrian.
    EXPECT_LT(result.selected.lateral_offset, -0.5);
    EXPECT_FALSE(result.selected.blocked_dynamic);
}

TEST(DynamicWalkingCorridor, ReportsInfeasibleWhenAllCandidateCorridorsBlocked)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 3.0;
    cfg.half_width = 0.35;
    cfg.lateral_shift = 0.7;
    cfg.prediction_horizon = 2.0;
    cfg.prediction_dt = 0.25;
    cfg.robot_speed = 1.0;
    cfg.robot_radius = 0.25;
    cfg.dynamic_safety_margin = 0.20;
    cfg.lateral_candidates_enable = true;
    cfg.dynamic_blocking_enable = true;
    corridor.setConfig(cfg);

    Eigen::Vector2d robot(0.0, 0.0);
    Eigen::Vector2d forward(1.0, 0.0);
    std::vector<Eigen::Vector3d> obs_pos = {
        Eigen::Vector3d(1.5, -0.7, 0.0),
        Eigen::Vector3d(1.5, 0.0, 0.0),
        Eigen::Vector3d(1.5, 0.7, 0.0),
    };
    std::vector<Eigen::Vector3d> obs_vel(obs_pos.size(), Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> obs_size(obs_pos.size(), Eigen::Vector3d(0.5, 0.5, 1.7));

    auto result = corridor.plan(robot, forward, obs_pos, obs_vel, obs_size);

    EXPECT_FALSE(result.has_feasible);
}

TEST(DynamicWalkingCorridor, ComputesOutsideDistanceForCandidateRectangle)
{
    DynamicWalkingCorridor::Candidate candidate;
    candidate.start = Eigen::Vector2d(1.0, 2.0);
    candidate.forward = Eigen::Vector2d::UnitX();
    candidate.left = Eigen::Vector2d::UnitY();
    candidate.length = 4.0;
    candidate.half_width = 0.5;

    EXPECT_DOUBLE_EQ(0.0, DynamicWalkingCorridor::outsideDistance(
                              candidate, Eigen::Vector2d(3.0, 2.2)));
    EXPECT_NEAR(0.2, DynamicWalkingCorridor::outsideDistance(
                         candidate, Eigen::Vector2d(3.0, 2.7)), 1e-9);
    EXPECT_NEAR(0.3, DynamicWalkingCorridor::outsideDistance(
                         candidate, Eigen::Vector2d(5.3, 2.0)), 1e-9);
    EXPECT_NEAR(std::sqrt(0.3 * 0.3 + 0.2 * 0.2),
                DynamicWalkingCorridor::outsideDistance(
                    candidate, Eigen::Vector2d(5.3, 2.7)), 1e-9);
}

TEST(DynamicWalkingCorridor, ComputesOutsideDistanceForPolylineCorridor)
{
    DynamicWalkingCorridor::Candidate candidate;
    candidate.half_width = 0.5;
    candidate.centerline = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(2.0, 2.0),
    };

    EXPECT_DOUBLE_EQ(0.0, DynamicWalkingCorridor::outsideDistance(
                              candidate, Eigen::Vector2d(1.0, 0.3)));
    EXPECT_DOUBLE_EQ(0.0, DynamicWalkingCorridor::outsideDistance(
                              candidate, Eigen::Vector2d(2.3, 1.0)));
    EXPECT_NEAR(0.2, DynamicWalkingCorridor::outsideDistance(
                         candidate, Eigen::Vector2d(1.0, 0.7)), 1e-9);
    EXPECT_NEAR(0.3, DynamicWalkingCorridor::outsideDistance(
                             candidate, Eigen::Vector2d(2.8, 1.0)), 1e-9);
}

TEST(DynamicWalkingCorridor, TruncatedPolylineCorridorIgnoresForwardOverrun)
{
    DynamicWalkingCorridor::Candidate candidate;
    candidate.half_width = 0.5;
    candidate.truncated_static = true;
    candidate.centerline = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
    };

    EXPECT_DOUBLE_EQ(0.0, DynamicWalkingCorridor::outsideDistance(
                              candidate, Eigen::Vector2d(3.0, 0.2)));
    EXPECT_NEAR(0.3, DynamicWalkingCorridor::outsideDistance(
                         candidate, Eigen::Vector2d(3.0, 0.8)), 1e-9);

    candidate.truncated_static = false;
    EXPECT_NEAR(std::sqrt(1.0 * 1.0 + 0.2 * 0.2) - 0.5,
                DynamicWalkingCorridor::outsideDistance(
                    candidate, Eigen::Vector2d(3.0, 0.2)), 1e-9);
}

TEST(DynamicWalkingCorridor, GeneratesCandidatesAlongReferencePolyline)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.4;
    cfg.lateral_shift = 0.8;
    cfg.prediction_horizon = 0.5;
    cfg.prediction_dt = 0.25;
    corridor.setConfig(cfg);

    std::vector<Eigen::Vector2d> reference = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(2.0, 2.0),
    };
    auto result = corridor.plan(reference, {}, {}, {});

    ASSERT_TRUE(result.has_feasible);
    EXPECT_EQ(1u, result.candidates.size());
    ASSERT_GE(result.selected.centerline.size(), 3u);
    EXPECT_NEAR(0.0, result.selected.centerline.front().x(), 1e-9);
    EXPECT_NEAR(0.0, result.selected.centerline.front().y(), 1e-9);
    EXPECT_NEAR(2.0, result.selected.centerline[1].x(), 1e-9);
    EXPECT_NEAR(0.0, result.selected.centerline[1].y(), 1e-9);
    EXPECT_NEAR(2.0, result.selected.centerline.back().x(), 1e-9);
    EXPECT_NEAR(2.0, result.selected.centerline.back().y(), 1e-9);
}

TEST(DynamicWalkingCorridor, GeneratesCandidatesAlongTimedNominalTrajectory)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.4;
    cfg.lateral_shift = 0.8;
    cfg.prediction_horizon = 0.5;
    cfg.prediction_dt = 0.25;
    corridor.setConfig(cfg);

    TimedTrajectory nominal;
    nominal.source = TimedTrajectorySource::PREVIOUS_MPPI;
    TimedTrajectoryPoint p0;
    p0.position = Eigen::Vector2d(0.0, 0.0);
    p0.t_from_now = 0.0;
    TimedTrajectoryPoint p1;
    p1.position = Eigen::Vector2d(1.0, 0.0);
    p1.t_from_now = 0.2;
    TimedTrajectoryPoint p2;
    p2.position = Eigen::Vector2d(1.0, 1.0);
    p2.t_from_now = 0.4;
    nominal.points = {p0, p1, p2};

    auto result = corridor.plan(nominal, {}, {}, {});

    ASSERT_TRUE(result.has_feasible);
    ASSERT_GE(result.selected.centerline.size(), 3u);
    EXPECT_NEAR(0.0, result.selected.centerline.front().x(), 1e-9);
    EXPECT_NEAR(0.0, result.selected.centerline.front().y(), 1e-9);
    EXPECT_NEAR(1.0, result.selected.centerline[1].x(), 1e-9);
    EXPECT_NEAR(0.0, result.selected.centerline[1].y(), 1e-9);
    EXPECT_NEAR(1.0, result.selected.centerline.back().x(), 1e-9);
    EXPECT_NEAR(1.0, result.selected.centerline.back().y(), 1e-9);
}

TEST(DynamicWalkingCorridor, UsesNominalTrajectoryTimeForDynamicBlocking)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.35;
    cfg.lateral_shift = 0.8;
    cfg.prediction_horizon = 6.0;
    cfg.prediction_dt = 1.0;
    cfg.robot_speed = 1.0;
    cfg.robot_radius = 0.25;
    cfg.dynamic_min_radius = 0.20;
    cfg.dynamic_safety_margin = 0.10;
    cfg.dynamic_progress_margin = 0.0;
    cfg.dynamic_blocking_enable = true;
    cfg.static_centerline_opt_enable = false;
    corridor.setConfig(cfg);

    std::vector<Eigen::Vector2d> spatial_path = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
    };

    TimedTrajectory nominal;
    nominal.source = TimedTrajectorySource::PREVIOUS_MPPI;
    TimedTrajectoryPoint p0;
    p0.position = spatial_path[0];
    p0.t_from_now = 0.0;
    TimedTrajectoryPoint p1;
    p1.position = spatial_path[1];
    p1.t_from_now = 10.0;
    nominal.points = {p0, p1};

    std::vector<Eigen::Vector3d> obs_pos = {Eigen::Vector3d(1.0, 0.0, 0.0)};
    std::vector<Eigen::Vector3d> obs_vel = {Eigen::Vector3d(0.0, 0.4, 0.0)};
    std::vector<Eigen::Vector3d> obs_size = {Eigen::Vector3d(0.4, 0.4, 1.7)};

    auto spatial_result = corridor.plan(spatial_path, obs_pos, obs_vel, obs_size);
    EXPECT_FALSE(spatial_result.has_feasible);

    auto timed_result = corridor.plan(nominal, obs_pos, obs_vel, obs_size);
    ASSERT_TRUE(timed_result.has_feasible);
    EXPECT_FALSE(timed_result.selected.blocked_dynamic);
}

TEST(DynamicWalkingCorridor, KeepsCenterCorridorAsSingleDefaultCandidate)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.4;
    cfg.lateral_shift = 0.8;
    corridor.setConfig(cfg);

    auto result = corridor.plan(Eigen::Vector2d::Zero(),
                                Eigen::Vector2d::UnitX(),
                                {}, {}, {});

    ASSERT_TRUE(result.has_feasible);
    ASSERT_EQ(1u, result.candidates.size());
    EXPECT_NEAR(0.0, result.selected.lateral_offset, 1e-9);
}

TEST(DynamicWalkingCorridor, DoesNotBlockForFarFutureSpatialOccupancy)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.35;
    cfg.lateral_shift = 0.8;
    cfg.prediction_horizon = 1.0;
    cfg.prediction_dt = 0.25;
    cfg.robot_speed = 1.0;
    cfg.robot_radius = 0.25;
    cfg.dynamic_safety_margin = 0.10;
    corridor.setConfig(cfg);

    Eigen::Vector2d robot(0.0, 0.0);
    Eigen::Vector2d forward(1.0, 0.0);
    std::vector<Eigen::Vector3d> obs_pos = {Eigen::Vector3d(3.0, 0.0, 0.0)};
    std::vector<Eigen::Vector3d> obs_vel = {Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> obs_size = {Eigen::Vector3d(0.5, 0.5, 1.7)};

    auto result = corridor.plan(robot, forward, obs_pos, obs_vel, obs_size);

    ASSERT_TRUE(result.has_feasible);
    EXPECT_FALSE(result.selected.blocked_dynamic);
}

TEST(DynamicWalkingCorridor, OptimizesCenterlineIntoNearbyStaticFreeBand)
{
    DynamicWalkingCorridor::Config cfg;
    cfg.half_width = 0.35;
    cfg.min_half_width = 0.20;
    cfg.static_sample_dl = 0.05;
    cfg.static_opt_lateral_range = 1.0;
    cfg.static_opt_lateral_step = 0.1;
    cfg.static_opt_smooth_weight = 1.0;

    std::vector<Eigen::Vector2d> reference = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(3.0, 0.0),
    };

    auto traversable = [](double, double y) {
        return y >= 0.40 && y <= 1.00;
    };

    std::vector<Eigen::Vector2d> optimized;
    double half_width = 0.0;
    const bool ok = DynamicWalkingCorridor::optimizeCenterlineForStaticMap(
        reference, cfg, traversable, optimized, half_width);

    ASSERT_TRUE(ok);
    ASSERT_EQ(reference.size(), optimized.size());
    EXPECT_GE(half_width, cfg.min_half_width);
    for (const auto &p : optimized)
    {
        EXPECT_GE(p.y(), 0.40);
        EXPECT_LE(p.y(), 1.00);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
