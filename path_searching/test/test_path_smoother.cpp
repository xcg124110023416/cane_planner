#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <path_searching/path_smoother.h>

using cane_planner::PathSmoother;

namespace
{

double maxTurnDeg(const std::vector<Eigen::Vector2d>& path)
{
    double max_turn = 0.0;
    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d a = path[i] - path[i - 1];
        const Eigen::Vector2d b = path[i + 1] - path[i];
        if (a.norm() < 1e-6 || b.norm() < 1e-6)
            continue;
        double dot = a.dot(b) / (a.norm() * b.norm());
        dot = std::max(-1.0, std::min(1.0, dot));
        max_turn = std::max(max_turn, std::acos(dot) * 180.0 / M_PI);
    }
    return max_turn;
}

}  // namespace

TEST(PathSmoother, RoundsSharpCornerWhenTraversable)
{
    std::vector<Eigen::Vector2d> path = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(1.0, 1.0),
    };

    PathSmoother::Config cfg;
    cfg.enable = true;
    cfg.corner_radius = 0.3;
    cfg.min_turn_angle = 30.0 * M_PI / 180.0;
    cfg.samples_per_corner = 4;

    auto smooth = PathSmoother::smoothCorners(path, cfg, [](double, double) {
        return true;
    });

    EXPECT_GT(smooth.size(), path.size());
    EXPECT_NEAR(smooth.front().x(), path.front().x(), 1e-9);
    EXPECT_NEAR(smooth.front().y(), path.front().y(), 1e-9);
    EXPECT_NEAR(smooth.back().x(), path.back().x(), 1e-9);
    EXPECT_NEAR(smooth.back().y(), path.back().y(), 1e-9);
    EXPECT_LT(maxTurnDeg(smooth), 90.0);
}

TEST(PathSmoother, KeepsOriginalCornerWhenRoundedSamplesAreBlocked)
{
    std::vector<Eigen::Vector2d> path = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(1.0, 1.0),
    };

    PathSmoother::Config cfg;
    cfg.enable = true;
    cfg.corner_radius = 0.3;
    cfg.min_turn_angle = 30.0 * M_PI / 180.0;
    cfg.samples_per_corner = 4;

    auto smooth = PathSmoother::smoothCorners(path, cfg, [](double x, double y) {
        return !(x > 0.75 && y > 0.05);
    });

    ASSERT_EQ(path.size(), smooth.size());
    for (size_t i = 0; i < path.size(); ++i)
    {
        EXPECT_NEAR(path[i].x(), smooth[i].x(), 1e-9);
        EXPECT_NEAR(path[i].y(), smooth[i].y(), 1e-9);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
