#include <plan_manage/pedestrian_panel.h>
#include <plan_manage/ControlPedestrian.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSaveFile>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace cane_planner {

PedestrianPanel::PedestrianPanel(QWidget* parent) : rviz::Panel(parent) {
  auto* layout = new QVBoxLayout(this);
  auto* help = new QLabel(tr("用工具栏“放置行人”点击起点，拖拽设置行走方向。"));
  help->setWordWrap(true);
  layout->addWidget(help);
  pose_ = new QLabel(tr("尚未收到行人状态"));
  pose_->setWordWrap(true);
  layout->addWidget(pose_);
  auto* form = new QFormLayout;
  speed_ = new QDoubleSpinBox;
  speed_->setObjectName("pedestrian_speed");
  speed_->setRange(0.0, 10.0);
  speed_->setDecimals(3);
  speed_->setSingleStep(0.1);
  speed_->setValue(0.4);
  duration_ = new QDoubleSpinBox;
  duration_->setObjectName("pedestrian_duration");
  duration_->setRange(0.1, 3600.0);
  duration_->setDecimals(2);
  duration_->setSingleStep(0.5);
  duration_->setValue(5.0);
  behavior_ = new QComboBox;
  behavior_->setObjectName("pedestrian_behavior");
  behavior_->addItem(tr("终点停止"), "stop");
  behavior_->addItem(tr("终点消失"), "disappear");
  behavior_->addItem(tr("返回起点后停止"), "return");
  behavior_->addItem(tr("持续往返"), "repeat");
  on_goal_ = new QCheckBox(tr("收到机器人导航目标后启动"));
  on_goal_->setObjectName("pedestrian_on_goal");
  on_goal_->setChecked(true);
  form->addRow(tr("速度 (m/s)"), speed_);
  form->addRow(tr("单程时间 (s)"), duration_);
  form->addRow(tr("结束行为"), behavior_);
  form->addRow(on_goal_);
  layout->addLayout(form);
  auto* timing = new QLabel(tr("返回耗时与去程相同；不勾选自动启动时，点击“开始／继续”。"));
  timing->setWordWrap(true);
  layout->addWidget(timing);
  auto* buttons = new QGridLayout;
  apply_ = new QPushButton(tr("应用并复位"));
  apply_->setObjectName("pedestrian_apply");
  start_ = new QPushButton(tr("开始／继续"));
  start_->setObjectName("pedestrian_start");
  pause_ = new QPushButton(tr("暂停"));
  pause_->setObjectName("pedestrian_pause");
  reset_ = new QPushButton(tr("复位"));
  reset_->setObjectName("pedestrian_reset");
  clear_ = new QPushButton(tr("清除"));
  clear_->setObjectName("pedestrian_clear");
  save_ = new QPushButton(tr("保存场景…"));
  save_->setObjectName("pedestrian_save");
  load_ = new QPushButton(tr("加载场景…"));
  load_->setObjectName("pedestrian_load");
  buttons->addWidget(apply_, 0, 0, 1, 2);
  buttons->addWidget(start_, 1, 0);
  buttons->addWidget(pause_, 1, 1);
  buttons->addWidget(reset_, 2, 0);
  buttons->addWidget(clear_, 2, 1);
  buttons->addWidget(load_, 3, 0);
  buttons->addWidget(save_, 3, 1);
  layout->addLayout(buttons);
  remember_ = new QLabel;
  remember_->setObjectName("pedestrian_remember");
  remember_->setWordWrap(true);
  layout->addWidget(remember_);
  status_ = new QLabel;
  status_->setWordWrap(true);
  layout->addWidget(status_);
  feedback_ = new QLabel;
  feedback_->setObjectName("pedestrian_feedback");
  feedback_->setWordWrap(true);
  layout->addWidget(feedback_);
  layout->addStretch();
  connect(apply_, &QPushButton::clicked, this, [this] { control("apply"); });
  connect(start_, &QPushButton::clicked, this, [this] { control("start"); });
  connect(pause_, &QPushButton::clicked, this, [this] { control("pause"); });
  connect(reset_, &QPushButton::clicked, this, [this] { control("reset"); });
  connect(clear_, &QPushButton::clicked, this, [this] { control("clear"); });
  connect(save_, &QPushButton::clicked, this, &PedestrianPanel::saveScenario);
  connect(load_, &QPushButton::clicked, this, &PedestrianPanel::loadScenario);
  nh_.setCallbackQueue(&queue_);
  subscriber_ = nh_.subscribe("/pedestrian_sim/state", 1, &PedestrianPanel::receiveState, this);
  client_ = nh_.serviceClient<plan_manage::ControlPedestrian>("/pedestrian_sim/control");
  auto* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &PedestrianPanel::poll);
  timer->start(100);
  updateEnabled(false);
}

void PedestrianPanel::poll() {
  queue_.callAvailable(ros::WallDuration(0));
  const bool connected = have_state_ && (ros::WallTime::now() - last_received_).toSec() < 1.0;
  updateEnabled(connected);
  if (!connected) {
    status_->setText(tr("等待确定性行人节点（use_pedestrians:=true，pedestrian_source:=deterministic）"));
  }
}

void PedestrianPanel::receiveState(const plan_manage::PedestrianState::ConstPtr& msg) {
  const bool changed = !have_state_ || sync_controls_ ||
      msg->speed != state_.speed || msg->duration != state_.duration ||
      msg->end_behavior != state_.end_behavior || msg->start_on_goal != state_.start_on_goal;
  state_ = *msg;
  have_state_ = true;
  sync_controls_ = false;
  last_received_ = ros::WallTime::now();
  remember_->setText(!msg->persistence_error.empty()
      ? tr("自动记忆失败，请使用“保存场景”导出配置。")
      : msg->remember_last ? tr("已应用的设置会自动记住，下次启动恢复。")
                           : tr("本次未启用自动记忆。"));
  if (changed) {
    speed_->setValue(msg->speed);
    duration_->setValue(msg->duration);
    behavior_->setCurrentIndex(behavior_->findData(QString::fromStdString(msg->end_behavior)));
    on_goal_->setChecked(msg->start_on_goal);
  }
  const auto& p = msg->start_pose.position;
  pose_->setText(tr("坐标系：%1\n起点：(%2, %3)　朝向：%4°")
      .arg(QString::fromStdString(msg->header.frame_id))
      .arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2)
      .arg(tf::getYaw(msg->start_pose.orientation) * 180.0 / M_PI, 0, 'f', 1));
  if (!msg->editable) {
    status_->setText(tr("当前为多行人场景。请选择 dwcg_crossing 等单行人场景使用编辑器。"));
    return;
  }
  QString phase = QString::fromStdString(msg->phase);
  if (phase == "waiting") phase = tr("已放置，等待启动");
  else if (phase == "walking") phase = tr("行走中");
  else if (phase == "paused") phase = tr("已暂停（仍发布静止行人）");
  else if (phase == "finished") phase = msg->visible ? tr("已结束，停留原地") : tr("已结束，行人已消失");
  else if (phase == "cleared") phase = tr("已清除，点击地图重新放置");
  else phase = tr("原 YAML 场景；放置或应用后进入交互模式");
  status_->setText(phase);
}

void PedestrianPanel::updateEnabled(bool connected) {
  const bool editable = connected && state_.editable;
  const bool placed = editable && state_.placed;
  speed_->setEnabled(editable);
  duration_->setEnabled(editable);
  behavior_->setEnabled(editable);
  on_goal_->setEnabled(editable);
  apply_->setEnabled(placed);
  start_->setEnabled(placed);
  pause_->setEnabled(placed && state_.phase == "walking");
  reset_->setEnabled(placed);
  clear_->setEnabled(placed);
  save_->setEnabled(placed);
  load_->setEnabled(editable);
}

void PedestrianPanel::control(const std::string& command) {
  plan_manage::ControlPedestrian srv;
  srv.request.command = command;
  srv.request.speed = speed_->value();
  srv.request.duration = duration_->value();
  srv.request.end_behavior = behavior_->currentData().toString().toStdString();
  srv.request.start_on_goal = on_goal_->isChecked();
  if (!client_.call(srv)) {
    feedback_->setText(tr("控制失败：行人服务不可用。"));
  } else if (!srv.response.success) {
    feedback_->setText(tr("控制失败：%1").arg(QString::fromStdString(srv.response.message)));
  } else {
    feedback_->setText(command == "apply" ? tr("设置已应用，行人已复位到起点。") : QString());
    sync_controls_ = command == "apply";
  }
}

void PedestrianPanel::saveScenario() {
  plan_manage::ControlPedestrian srv;
  srv.request.command = "export";
  if (!client_.call(srv) || !srv.response.success) {
    feedback_->setText(tr("保存失败：无法读取当前已应用的配置。"));
    return;
  }
  const QString filename = QFileDialog::getSaveFileName(this, tr("保存已应用的行人场景"),
                                                       "pedestrian_custom.yaml", tr("YAML (*.yaml)"));
  if (filename.isEmpty()) return;
  QSaveFile file(filename);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    feedback_->setText(tr("保存失败：%1").arg(file.errorString()));
    return;
  }
  const auto contents = QByteArray::fromStdString(srv.response.scenario_yaml);
  if (file.write(contents) != contents.size() || !file.commit()) feedback_->setText(tr("保存失败：%1").arg(file.errorString()));
  else feedback_->setText(tr("已保存当前已应用的设置：%1").arg(filename));
}

void PedestrianPanel::loadScenario() {
  const QString filename = QFileDialog::getOpenFileName(this, tr("加载行人场景"),
                                                       QString(), tr("YAML (*.yaml *.yml)"));
  if (filename.isEmpty()) return;
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    feedback_->setText(tr("加载失败：%1").arg(file.errorString()));
    return;
  }
  const auto contents = file.readAll();
  if (file.error() != QFileDevice::NoError) {
    feedback_->setText(tr("加载失败：%1").arg(file.errorString()));
    return;
  }
  plan_manage::ControlPedestrian srv;
  srv.request.command = "load";
  srv.request.scenario_yaml = contents.toStdString();
  if (!client_.call(srv)) {
    feedback_->setText(tr("加载失败：行人服务不可用。"));
  } else if (!srv.response.success) {
    feedback_->setText(tr("加载失败：%1").arg(QString::fromStdString(srv.response.message)));
  } else {
    sync_controls_ = true;
    feedback_->setText(tr("已加载并应用场景：%1").arg(filename));
  }
}

}  // namespace cane_planner

PLUGINLIB_EXPORT_CLASS(cane_planner::PedestrianPanel, rviz::Panel)
