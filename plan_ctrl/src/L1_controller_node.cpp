#include "plan_ctrl/L1_controller_v2.h"

int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "L1Controller_v2");
    L1Controller controller;
    ros::spin();
    return 0;
}
