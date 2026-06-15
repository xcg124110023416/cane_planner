#include <gtest/gtest.h>

#include <path_searching/convex_corridor.h>

#include <cmath>

using cane_planner::ConvexCorridor;

TEST(ConvexCorridor, ContainsPointInsideRectangle)
{
    ConvexCorridor::Segment seg;
    seg.halfspaces = {
        {Eigen::Vector2d(1.0, 0.0), 1.0},
        {Eigen::Vector2d(-1.0, 0.0), 1.0},
        {Eigen::Vector2d(0.0, 1.0), 0.5},
        {Eigen::Vector2d(0.0, -1.0), 0.5},
    };

    EXPECT_TRUE(ConvexCorridor::contains(seg, Eigen::Vector2d(0.0, 0.0)));
    EXPECT_TRUE(ConvexCorridor::contains(seg, Eigen::Vector2d(0.8, 0.3)));
    EXPECT_FALSE(ConvexCorridor::contains(seg, Eigen::Vector2d(1.2, 0.0)));
    EXPECT_NEAR(0.2, ConvexCorridor::violation(seg, Eigen::Vector2d(1.2, 0.0)), 1e-9);
}

TEST(ConvexCorridor, BuildsStaticSegmentsInsideTraversableBand)
{
    ConvexCorridor cc;
    ConvexCorridor::Config cfg;
    cfg.segment_length = 1.0;
    cfg.half_width = 0.5;
    cfg.min_half_width = 0.25;
    cfg.static_sample_dl = 0.05;
    cc.setConfig(cfg);

    std::vector<Eigen::Vector2d> path = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(3.0, 0.0),
    };
    auto traversable = [](double, double y) { return std::abs(y) <= 0.35; };

    auto result = cc.buildStatic(path, traversable);

    ASSERT_TRUE(result.feasible);
    ASSERT_GE(result.segments.size(), 3u);
    EXPECT_GE(result.min_width, cfg.min_half_width);
    for (const auto &seg : result.segments)
        EXPECT_TRUE(ConvexCorridor::contains(seg, seg.center));
}

TEST(ConvexCorridor, ClipsSegmentAwayFromPredictedPedestrian)
{
    ConvexCorridor cc;
    ConvexCorridor::Config cfg;
    cfg.segment_length = 1.0;
    cfg.half_width = 0.6;
    cfg.min_half_width = 0.2;
    cfg.pedestrian_radius = 0.3;
    cc.setConfig(cfg);

    std::vector<Eigen::Vector2d> path = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
    };
    auto traversable = [](double, double) { return true; };
    std::vector<ConvexCorridor::PedestrianPrediction> peds = {
        {Eigen::Vector2d(1.0, 0.35), Eigen::Vector2d::Zero(), 0.3},
    };

    auto result = cc.buildSpatioTemporal(path, traversable, peds);

    ASSERT_TRUE(result.feasible);
    ASSERT_FALSE(result.segments.empty());
    EXPECT_GT(result.dynamic_block_count, 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
