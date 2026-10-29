#include <rviz/default_plugin/tools/goal_tool.h>
#include <rviz/default_plugin/tools/point_tool.h>
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

class SocialStartTool : public rviz::GoalTool {
 public:
  SocialStartTool() { shortcut_key_ = 'p'; }
  void onInitialize() override {
    rviz::GoalTool::onInitialize();
    setName(QString::fromUtf8("行人起点"));
    getPropertyContainer()->subProp("Topic")->setValue("/pedsim_scene/start_pose");
  }
};

class SocialGoalTool : public rviz::GoalTool {
 public:
  SocialGoalTool() { shortcut_key_ = 'k'; }
  void onInitialize() override {
    rviz::GoalTool::onInitialize();
    setName(QString::fromUtf8("行人终点"));
    getPropertyContainer()->subProp("Topic")->setValue("/pedsim_scene/goal_pose");
  }
};

class SocialSelectTool : public rviz::PointTool {
 public:
  SocialSelectTool() { shortcut_key_ = 'u'; }
  void onInitialize() override {
    rviz::PointTool::onInitialize();
    setName(QString::fromUtf8("选择行人"));
    getPropertyContainer()->subProp("Topic")->setValue("/pedsim_scene/select_point");
  }
};

}  // namespace cane_planner

PLUGINLIB_EXPORT_CLASS(cane_planner::PedestrianTool, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(cane_planner::SocialStartTool, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(cane_planner::SocialGoalTool, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(cane_planner::SocialSelectTool, rviz::Tool)
