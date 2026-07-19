#include <gtest/gtest.h>

#include <path_searching/dynamic_walking_corridor.h>
#include <path_searching/timed_trajectory.h>

#include <algorithm>

using cane_planner::DynamicWalkingCorridor;
using cane_planner::TimedTrajectory;
using cane_planner::TimedTrajectoryPoint;
using cane_planner::TimedTrajectorySource;
using cane_planner::TimedWalkingCorridorSegment;

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

TEST(DynamicWalkingCorridor, ExposesSelectedTimedCorridorFromNominalTrajectory)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.4;
    cfg.static_centerline_opt_enable = false;
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
    ASSERT_TRUE(result.timed_corridor.valid());
    EXPECT_EQ(TimedTrajectorySource::PREVIOUS_MPPI, result.timed_corridor.source);
    ASSERT_EQ(2u, result.timed_corridor.segments.size());
    EXPECT_NEAR(0.0, result.timed_corridor.tStart(), 1e-9);
    EXPECT_NEAR(0.4, result.timed_corridor.tEnd(), 1e-9);
    ASSERT_NE(nullptr, result.timed_corridor.segmentAtTime(0.1));
    EXPECT_NEAR(1.0, result.timed_corridor.segmentAtTime(0.3)->end.y(), 1e-9);
}

TEST(DynamicWalkingCorridor, BuildsSparseOverlappingTimedCorridorCover)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.4;
    cfg.static_min_feasible_length = 0.8;
    cfg.static_centerline_opt_enable = false;
    corridor.setConfig(cfg);

    TimedTrajectory nominal;
    nominal.source = TimedTrajectorySource::PREVIOUS_MPPI;
    for (int i = 0; i <= 40; ++i)
    {
        TimedTrajectoryPoint p;
        p.position = Eigen::Vector2d(0.1 * static_cast<double>(i), 0.0);
        p.t_from_now = 0.1 * static_cast<double>(i);
        nominal.points.push_back(p);
    }

    auto result = corridor.plan(nominal, {}, {}, {});

    ASSERT_TRUE(result.has_feasible);
    ASSERT_TRUE(result.timed_corridor.valid());
    ASSERT_LE(result.timed_corridor.segments.size(), 5u);
    ASSERT_GE(result.timed_corridor.segments.size(), 2u);
    for (const auto &segment : result.timed_corridor.segments)
    {
        EXPECT_GE((segment.end - segment.start).norm(), 0.8);
    }
    for (size_t i = 1; i < result.timed_corridor.segments.size(); ++i)
    {
        const auto &prev = result.timed_corridor.segments[i - 1];
        const auto &curr = result.timed_corridor.segments[i];
        EXPECT_LT(curr.t_start, prev.t_end);
        EXPECT_LT((curr.start - prev.end).norm(), (prev.end - prev.start).norm());
    }
}

TEST(DynamicWalkingCorridor, TimedCorridorSegmentsExposeConvexCells)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.4;
    cfg.static_centerline_opt_enable = false;
    corridor.setConfig(cfg);

    TimedTrajectory nominal;
    nominal.source = TimedTrajectorySource::PREVIOUS_MPPI;
    TimedTrajectoryPoint p0;
    p0.position = Eigen::Vector2d(0.0, 0.0);
    p0.t_from_now = 0.0;
    TimedTrajectoryPoint p1;
    p1.position = Eigen::Vector2d(1.0, 0.0);
    p1.t_from_now = 0.2;
    nominal.points = {p0, p1};

    auto result = corridor.plan(nominal, {}, {}, {});

    ASSERT_TRUE(result.has_feasible);
    ASSERT_TRUE(result.timed_corridor.valid());
    ASSERT_EQ(1u, result.timed_corridor.segments.size());
    const auto &segment = result.timed_corridor.segments.front();
    EXPECT_GE(segment.half_planes.size(), 4u);
    EXPECT_GE(segment.polygon.size(), 4u);
    EXPECT_DOUBLE_EQ(0.0, result.timed_corridor.outsideDistanceAtTime(
                              0.1, Eigen::Vector2d(0.5, 0.0)));
}

TEST(DynamicWalkingCorridor, SplitsDynamicallyBlockedTimedSegments)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 8.0;
    cfg.half_width = 0.4;
    cfg.static_centerline_opt_enable = false;
    cfg.dynamic_safety_margin = 0.10;
    cfg.dynamic_split_max_depth = 1;
    cfg.dynamic_split_min_length = 0.4;
    corridor.setConfig(cfg);

    TimedTrajectory nominal;
    nominal.source = TimedTrajectorySource::PREVIOUS_MPPI;
    TimedTrajectoryPoint p0;
    p0.position = Eigen::Vector2d(0.0, 0.0);
    p0.t_from_now = 0.0;
    TimedTrajectoryPoint p1;
    p1.position = Eigen::Vector2d(2.0, 0.0);
    p1.t_from_now = 1.0;
    nominal.points = {p0, p1};

    std::vector<Eigen::Vector3d> obs_pos = {Eigen::Vector3d(1.0, 0.0, 0.0)};
    std::vector<Eigen::Vector3d> obs_vel = {Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> obs_size = {Eigen::Vector3d(0.4, 0.4, 1.7)};

    auto result = corridor.plan(nominal, obs_pos, obs_vel, obs_size);

    ASSERT_TRUE(result.has_feasible);
    ASSERT_TRUE(result.timed_corridor.valid());
    ASSERT_EQ(2u, result.timed_corridor.segments.size());
    EXPECT_NEAR(0.0, result.timed_corridor.segments.front().t_start, 1e-9);
    EXPECT_NEAR(0.5, result.timed_corridor.segments.front().t_end, 1e-9);
    EXPECT_NEAR(0.5, result.timed_corridor.segments.back().t_start, 1e-9);
    EXPECT_NEAR(1.0, result.timed_corridor.segments.back().t_end, 1e-9);
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

TEST(DynamicWalkingCorridor, ExposesDynamicPedestriansAsTimedConvexFootprints)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 0.6;
    cfg.prediction_dt = 0.25;
    cfg.robot_radius = 0.25;
    cfg.dynamic_min_radius = 0.20;
    cfg.dynamic_safety_margin = 0.10;
    cfg.static_centerline_opt_enable = false;
    corridor.setConfig(cfg);

    TimedTrajectory nominal;
    nominal.source = TimedTrajectorySource::PREVIOUS_MPPI;
    TimedTrajectoryPoint p0;
    p0.position = Eigen::Vector2d(0.0, 0.0);
    p0.t_from_now = 0.0;
    TimedTrajectoryPoint p1;
    p1.position = Eigen::Vector2d(2.0, 0.0);
    p1.t_from_now = 1.0;
    nominal.points = {p0, p1};

    std::vector<Eigen::Vector3d> obs_pos = {Eigen::Vector3d(1.0, 0.0, 0.0)};
    std::vector<Eigen::Vector3d> obs_vel = {Eigen::Vector3d(0.0, 0.0, 0.0)};
    std::vector<Eigen::Vector3d> obs_size = {Eigen::Vector3d(0.4, 0.4, 1.7)};

    auto result = corridor.plan(nominal, obs_pos, obs_vel, obs_size);

    ASSERT_TRUE(result.has_feasible);
    ASSERT_TRUE(result.timed_corridor.valid());
    ASSERT_FALSE(result.timed_corridor.segments.empty());
    const auto segment_with_obstacle = std::find_if(
        result.timed_corridor.segments.begin(),
        result.timed_corridor.segments.end(),
        [](const TimedWalkingCorridorSegment &segment)
        {
            return !segment.dynamic_obstacles.empty();
        });
    ASSERT_NE(result.timed_corridor.segments.end(), segment_with_obstacle);
    ASSERT_EQ(1u, segment_with_obstacle->dynamic_obstacles.size());
    EXPECT_GE(segment_with_obstacle->dynamic_obstacles.front().vertices.size(), 3u);
    EXPECT_GT(result.timed_corridor.dynamicObstacleViolationAtTime(
                  0.5, Eigen::Vector2d(1.0, 0.0)),
              0.0);
    EXPECT_DOUBLE_EQ(0.0, result.timed_corridor.dynamicObstacleViolationAtTime(
                              0.5, Eigen::Vector2d(1.0, 1.5)));
}

TEST(DynamicWalkingCorridor, IterativeFiriCellExpandsTowardDynamicObstacleBoundary)
{
    DynamicWalkingCorridor corridor;
    DynamicWalkingCorridor::Config cfg;
    cfg.length = 4.0;
    cfg.half_width = 1.0;
    cfg.min_half_width = 0.25;
    cfg.prediction_dt = 0.25;
    cfg.robot_radius = 0.25;
    cfg.dynamic_min_radius = 0.20;
    cfg.dynamic_safety_margin = 0.10;
    cfg.static_centerline_opt_enable = false;
    cfg.human_cane_footprint_enable = false;
    corridor.setConfig(cfg);

    TimedTrajectory nominal;
    nominal.source = TimedTrajectorySource::PREVIOUS_MPPI;
    TimedTrajectoryPoint p0;
    p0.position = Eigen::Vector2d(0.0, 0.0);
    p0.t_from_now = 0.0;
    TimedTrajectoryPoint p1;
    p1.position = Eigen::Vector2d(2.0, 0.0);
    p1.t_from_now = 1.0;
    nominal.points = {p0, p1};

    std::vector<Eigen::Vector3d> obs_pos = {Eigen::Vector3d(1.0, 1.0, 0.0)};
    std::vector<Eigen::Vector3d> obs_vel = {Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> obs_size = {Eigen::Vector3d(0.4, 0.4, 1.7)};

    auto result = corridor.plan(nominal, obs_pos, obs_vel, obs_size);

    ASSERT_TRUE(result.has_feasible);
    ASSERT_TRUE(result.timed_corridor.valid());
    EXPECT_DOUBLE_EQ(0.0, result.timed_corridor.outsideDistanceAtTime(
                              0.5, Eigen::Vector2d(1.0, 0.35)));
    EXPECT_GT(result.timed_corridor.outsideDistanceAtTime(
                  0.5, Eigen::Vector2d(1.0, 1.0)),
              0.0);
}

TEST(DynamicWalkingCorridor, HumanCaneFootprintErodesConvexCell)
{
    TimedTrajectory nominal;
    nominal.source = TimedTrajectorySource::PREVIOUS_MPPI;
    TimedTrajectoryPoint p0;
    p0.position = Eigen::Vector2d(0.0, 0.0);
    p0.t_from_now = 0.0;
    TimedTrajectoryPoint p1;
    p1.position = Eigen::Vector2d(2.0, 0.0);
    p1.t_from_now = 1.0;
    nominal.points = {p0, p1};

    DynamicWalkingCorridor::Config point_cfg;
    point_cfg.length = 4.0;
    point_cfg.half_width = 1.0;
    point_cfg.static_opt_lateral_range = 1.0;
    point_cfg.static_centerline_opt_enable = false;
    point_cfg.human_cane_footprint_enable = false;

    DynamicWalkingCorridor point_corridor;
    point_corridor.setConfig(point_cfg);
    const auto point_result = point_corridor.plan(nominal, {}, {}, {});

    DynamicWalkingCorridor::Config footprint_cfg = point_cfg;
    footprint_cfg.human_cane_footprint_enable = true;
    footprint_cfg.human_cane_body_radius = 0.25;
    footprint_cfg.human_cane_cane_length = 0.65;
    footprint_cfg.human_cane_front_radius = 0.12;
    footprint_cfg.human_cane_safety_margin = 0.03;

    DynamicWalkingCorridor footprint_corridor;
    footprint_corridor.setConfig(footprint_cfg);
    const auto footprint_result = footprint_corridor.plan(nominal, {}, {}, {});

    ASSERT_TRUE(point_result.has_feasible);
    ASSERT_TRUE(footprint_result.has_feasible);
    ASSERT_TRUE(point_result.timed_corridor.valid());
    ASSERT_TRUE(footprint_result.timed_corridor.valid());
    EXPECT_DOUBLE_EQ(0.0, point_result.timed_corridor.outsideDistanceAtTime(
                              0.5, Eigen::Vector2d(1.0, 0.85)));
    EXPECT_GT(footprint_result.timed_corridor.outsideDistanceAtTime(
                  0.5, Eigen::Vector2d(1.0, 0.85)),
              0.0);
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
