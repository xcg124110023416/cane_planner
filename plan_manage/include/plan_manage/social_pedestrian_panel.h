#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <plan_manage/SocialSceneState.h>
#include <plan_manage/ControlSocialScene.h>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace cane_planner {

class SocialPedestrianPanel : public rviz::Panel {
  Q_OBJECT
 public:
  explicit SocialPedestrianPanel(QWidget* parent = nullptr);

 private:
  void poll();
  void receive(const plan_manage::SocialSceneState::ConstPtr& state);
  bool send(plan_manage::ControlSocialScene& service);
  void control(const std::string& command);
  void saveScene();
  void loadScene();
  void enabled(bool connected);

  ros::CallbackQueue queue_;
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  ros::ServiceClient client_;
  ros::WallTime received_;
  plan_manage::SocialSceneState state_;
  bool have_state_ = false;
  bool syncing_ = false;
  bool refresh_ = true;
  QComboBox* people_;
  QDoubleSpinBox *x_, *y_, *yaw_, *gx_, *gy_, *speed_, *time_;
  QComboBox* behavior_;
  QCheckBox* on_goal_;
  QLabel *status_, *feedback_, *memory_;
  QPushButton *add_, *remove_, *clear_, *apply_, *calculate_, *start_, *pause_, *reset_, *save_, *load_;
};

}  // namespace cane_planner
