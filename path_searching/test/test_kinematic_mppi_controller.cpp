#include <gtest/gtest.h>

#include <path_searching/kinematic_mppi_controller.h>

using cane_planner::KinematicMppiController;

TEST(KinematicMppiController, DrivesTowardForwardGoalWithoutObstacles)
{
    KinematicMppiController controller;
    controller.init();

    Eigen::Vector3d start(0.0, 0.0, 0.0);
    Eigen::Vector3d goal(2.0, 0.0, 0.0);
    std::vector<Eigen::Vector3d> obs_pos;
    std::vector<Eigen::Vector3d> obs_vel;
    std::vector<Eigen::Vector3d> obs_size;

    Eigen::Vector2d cmd = controller.plan(start, goal, obs_pos, obs_vel, obs_size);

    EXPECT_TRUE(controller.lastPlanValid());
    EXPECT_GT(cmd(0), 0.0);
    EXPECT_NEAR(cmd(1), 0.0, 0.35);
    EXPECT_FALSE(controller.getBestPath().empty());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
