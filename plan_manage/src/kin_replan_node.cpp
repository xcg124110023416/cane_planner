#include <plan_manager.h>
#include <path_searching/astar.h>

using namespace cane_planner;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kin_replan_node");
    ros::NodeHandle nh("~");

    int planner;
    nh.param("planner_node/planner", planner, 2);

    PlannerManager plan_manage(planner);
    plan_manage.init(nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
