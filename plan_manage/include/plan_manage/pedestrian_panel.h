#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <plan_manage/PedestrianState.h>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace cane_planner {

class PedestrianPanel : public rviz::Panel {
  Q_OBJECT
 public:
  explicit PedestrianPanel(QWidget* parent = nullptr);

 private:
  void poll();
  void receiveState(const plan_manage::PedestrianState::ConstPtr& state);
  void control(const std::string& command);
  void saveScenario();
  void loadScenario();
  void updateEnabled(bool connected);

  // Pump a private ROS queue on the Qt thread; widgets never cross threads.
  ros::CallbackQueue queue_;
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  ros::ServiceClient client_;
  ros::WallTime last_received_;
  plan_manage::PedestrianState state_;
  bool have_state_ = false;
  bool sync_controls_ = false;
  QDoubleSpinBox* speed_;
  QDoubleSpinBox* duration_;
  QComboBox* behavior_;
  QCheckBox* on_goal_;
  QLabel* pose_;
  QLabel* status_;
  QLabel* feedback_;
  QLabel* remember_;
  QPushButton* apply_;
  QPushButton* start_;
  QPushButton* pause_;
  QPushButton* reset_;
  QPushButton* clear_;
  QPushButton* save_;
  QPushButton* load_;
};

}  // namespace cane_planner
