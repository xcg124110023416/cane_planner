#include <rviz/default_plugin/tools/goal_tool.h>
#include <rviz/properties/property.h>
#include <pluginlib/class_list_macros.h>

namespace cane_planner {

// Reuse RViz's pose-selection gesture and publisher, with an independent topic.
class PedestrianTool : public rviz::GoalTool {
 public:
  PedestrianTool() { shortcut_key_ = 'p'; }

  void onInitialize() override {
    rviz::GoalTool::onInitialize();
    setName(QString::fromUtf8("放置行人"));
    getPropertyContainer()->subProp("Topic")->setValue("/pedestrian_sim/initial_pose");
  }
};

}  // namespace cane_planner

PLUGINLIB_EXPORT_CLASS(cane_planner::PedestrianTool, rviz::Tool)
