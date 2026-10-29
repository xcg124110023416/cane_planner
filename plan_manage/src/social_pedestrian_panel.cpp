#include <plan_manage/social_pedestrian_panel.h>
#include <pluginlib/class_list_macros.h>

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
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace cane_planner {
namespace {
constexpr double kDegrees = 180.0 / 3.14159265358979323846;
bool sameConfig(const plan_manage::SocialPedestrian& a, const plan_manage::SocialPedestrian& b) {
  return a.id == b.id && a.start.x == b.start.x && a.start.y == b.start.y &&
         a.start.theta == b.start.theta && a.goal.x == b.goal.x && a.goal.y == b.goal.y &&
         a.desired_speed == b.desired_speed && a.end_behavior == b.end_behavior;
}
const plan_manage::SocialPedestrian* selected(const plan_manage::SocialSceneState& s) {
  for (const auto& p : s.pedestrians) if (p.id == s.selected_id) return &p;
  return nullptr;
}
}  // namespace

SocialPedestrianPanel::SocialPedestrianPanel(QWidget* parent) : rviz::Panel(parent) {
  auto* outer = new QVBoxLayout(this);
  auto* scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  auto* contents = new QWidget;
  scroll->setWidget(contents);
  outer->addWidget(scroll);
  auto* layout = new QVBoxLayout(contents);
  auto label = [&](const QString& text) {
    auto* value = new QLabel(text); value->setWordWrap(true); layout->addWidget(value); return value;
  };
  label(tr("社会力场景（最多 5 人）。新增后用“行人起点 P”点击并拖拽；“行人终点 K”设置目标；“选择行人 U”点选。"));
  people_ = new QComboBox;
  people_->setObjectName("social_people");
  layout->addWidget(people_);
  auto* row = new QGridLayout;
  auto button = [&](const QString& text, const char* name) {
    auto* b = new QPushButton(text); b->setObjectName(name); return b;
  };
  add_ = button(tr("新增（地图放置）"), "social_add");
  remove_ = button(tr("删除选中"), "social_delete");
  clear_ = button(tr("清空场景"), "social_clear");
  row->addWidget(add_, 0, 0); row->addWidget(remove_, 0, 1); row->addWidget(clear_, 1, 0, 1, 2);
  layout->addLayout(row);
  auto* form = new QFormLayout;
  auto spin = [&](const char* name, double lo, double hi, double step) {
    auto* s = new QDoubleSpinBox; s->setObjectName(name); s->setDecimals(3);
    s->setRange(lo, hi); s->setSingleStep(step); return s;
  };
  x_ = spin("social_x", -14.499, 14.499, .1);
  y_ = spin("social_y", -14.499, 14.499, .1);
  yaw_ = spin("social_yaw", -180., 180., 5.);
  gx_ = spin("social_goal_x", -14.499, 14.499, .1);
  gy_ = spin("social_goal_y", -14.499, 14.499, .1);
  speed_ = spin("social_speed", 0., 3., .1);
  time_ = spin("social_time", .1, 3600., 1.); time_->setValue(6.);
  behavior_ = new QComboBox; behavior_->setObjectName("social_behavior");
  behavior_->addItem(tr("终点停止"), "stop");
  behavior_->addItem(tr("返回起点后停止"), "return");
  behavior_->addItem(tr("持续往返"), "repeat");
  form->addRow(tr("起点 X (m)"), x_); form->addRow(tr("起点 Y (m)"), y_);
  form->addRow(tr("初始朝向 (°)"), yaw_);
  form->addRow(tr("终点 X (m)"), gx_); form->addRow(tr("终点 Y (m)"), gy_);
  form->addRow(tr("期望速度 (m/s)"), speed_); form->addRow(tr("参考单程时间 (s)"), time_);
  form->addRow(tr("结束行为"), behavior_); layout->addLayout(form);
  calculate_ = button(tr("按朝向和参考时间计算终点"), "social_calculate"); layout->addWidget(calculate_);
  label(tr("参考时间只用于计算终点；实际速度、路线与到达时间由社会力决定。速度为 0 时原地静止。"));
  on_goal_ = new QCheckBox(tr("新机器人目标时复位并启动全部行人"));
  on_goal_->setObjectName("social_on_goal"); layout->addWidget(on_goal_);
  label(tr("修改起终点、应用、增删或加载会将全部行人复位等待；机器人不复位。"));
  auto* actions = new QGridLayout;
  apply_ = button(tr("应用并全部复位"), "social_apply");
  start_ = button(tr("全部开始／继续"), "social_start"); pause_ = button(tr("暂停行人"), "social_pause");
  reset_ = button(tr("全部复位"), "social_reset");
  load_ = button(tr("加载场景…"), "social_load"); save_ = button(tr("保存场景…"), "social_save");
  actions->addWidget(apply_, 0, 0, 1, 2); actions->addWidget(start_, 1, 0); actions->addWidget(pause_, 1, 1);
  actions->addWidget(reset_, 2, 0, 1, 2); actions->addWidget(load_, 3, 0); actions->addWidget(save_, 3, 1);
  layout->addLayout(actions);
  memory_ = label(QString()); memory_->setObjectName("social_memory");
  status_ = label(tr("等待 pedsim 编辑器…")); status_->setObjectName("social_status");
  feedback_ = label(QString()); feedback_->setObjectName("social_feedback"); layout->addStretch();

  connect(people_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i) {
    if (syncing_ || i < 0) return;
    plan_manage::ControlSocialScene s; s.request.command = "select";
    s.request.selected_id = people_->itemData(i).toUInt(); send(s);
  });
  connect(add_, &QPushButton::clicked, this, [this] {
    plan_manage::ControlSocialScene s; s.request.command = "select"; s.request.selected_id = 0;
    if (send(s)) feedback_->setText(tr("请用“行人起点 P”在地图点击并拖拽放置新行人。"));
  });
  for (const auto& action : {std::make_pair(remove_, "delete"), {clear_, "clear"}, {apply_, "apply"},
                            {start_, "start"}, {pause_, "pause"}, {reset_, "reset"}}) {
    const std::string command = action.second;
    connect(action.first, &QPushButton::clicked, this, [this, command] { control(command); });
  }
  connect(calculate_, &QPushButton::clicked, this, [this] {
    const double distance = speed_->value() * time_->value();
    const double angle = yaw_->value() / kDegrees;
    const double gx = x_->value() + distance * std::cos(angle), gy = y_->value() + distance * std::sin(angle);
    if (std::abs(gx) >= 14.5 || std::abs(gy) >= 14.5) {
      feedback_->setText(tr("计算的终点超出场地，请缩短参考时间。")); return;
    }
    gx_->setValue(gx); gy_->setValue(gy); feedback_->setText(tr("终点已计算；点击“应用并全部复位”生效。"));
  });
  connect(save_, &QPushButton::clicked, this, &SocialPedestrianPanel::saveScene);
  connect(load_, &QPushButton::clicked, this, &SocialPedestrianPanel::loadScene);
  nh_.setCallbackQueue(&queue_);
  subscriber_ = nh_.subscribe("/pedsim_scene/state", 1, &SocialPedestrianPanel::receive, this);
  client_ = nh_.serviceClient<plan_manage::ControlSocialScene>("/pedsim_scene/control");
  auto* timer = new QTimer(this); connect(timer, &QTimer::timeout, this, &SocialPedestrianPanel::poll); timer->start(100);
  enabled(false);
}

void SocialPedestrianPanel::poll() {
  queue_.callAvailable(ros::WallDuration(0));
  const bool connected = have_state_ && (ros::WallTime::now() - received_).toSec() < 1.0 && state_.ready;
  enabled(connected);
  if (have_state_ && (ros::WallTime::now() - received_).toSec() >= 1.0) status_->setText(tr("编辑器断联，请检查仿真进程。"));
}

void SocialPedestrianPanel::receive(const plan_manage::SocialSceneState::ConstPtr& msg) {
  if (have_state_ && msg->revision < state_.revision) return;
  const auto* old = selected(state_); const auto* current = selected(*msg);
  const bool changed = refresh_ || !have_state_ || state_.selected_id != msg->selected_id ||
      (old && current && !sameConfig(*old, *current));
  const bool goal_changed = refresh_ || !have_state_ || state_.start_on_goal != msg->start_on_goal;
  state_ = *msg; have_state_ = true; received_ = ros::WallTime::now(); syncing_ = true;
  people_->clear();
  people_->addItem(tr("新增：等待地图放置"), 0);
  for (const auto& p : state_.pedestrians) people_->addItem(tr("行人 #%1").arg(p.id), p.id);
  people_->setCurrentIndex(people_->findData(state_.selected_id));
  if (changed && current) {
    x_->setValue(current->start.x); y_->setValue(current->start.y); yaw_->setValue(current->start.theta * kDegrees);
    gx_->setValue(current->goal.x); gy_->setValue(current->goal.y); speed_->setValue(current->desired_speed);
    behavior_->setCurrentIndex(behavior_->findData(QString::fromStdString(current->end_behavior)));
    if (current->desired_speed > 0.) time_->setValue(std::hypot(current->goal.x-current->start.x, current->goal.y-current->start.y) / current->desired_speed);
  }
  if (goal_changed) on_goal_->setChecked(state_.start_on_goal);
  refresh_ = false; syncing_ = false;
  QString text = tr("%1 人 · 版本 %2 · %3").arg(state_.pedestrians.size()).arg(state_.revision)
      .arg(state_.running ? tr("运行中") : tr("暂停／等待"));
  for (size_t i = 0; i < state_.pedestrians.size() && i < state_.phases.size(); ++i) {
    const auto& phase = state_.phases[i];
    QString name = QString::fromStdString(phase);
    if (phase == "waiting") name = tr("等待"); else if (phase == "paused") name = tr("暂停");
    else if (phase == "walking") name = tr("去程"); else if (phase == "returning") name = tr("返程");
    else if (phase == "finished") name = tr("已结束");
    text += tr("\n#%1 %2").arg(state_.pedestrians[i].id).arg(name);
  }
  if (!state_.message.empty()) text += "\n" + QString::fromStdString(state_.message);
  status_->setText(text);
  memory_->setText(state_.remember_last ? tr("已开启：下次自动恢复已应用的场景（复位等待）。") : tr("本轮不读取或保存自动记忆。"));
  if (!state_.persistence_error.empty()) memory_->setText(tr("自动记忆失败：%1").arg(QString::fromStdString(state_.persistence_error)));
}

void SocialPedestrianPanel::enabled(bool connected) {
  const bool person = connected && selected(state_);
  for (auto* w : {x_, y_, yaw_, gx_, gy_, speed_, time_}) w->setEnabled(person);
  behavior_->setEnabled(person); calculate_->setEnabled(person); remove_->setEnabled(person);
  people_->setEnabled(connected); add_->setEnabled(connected && state_.pedestrians.size() < 5);
  apply_->setEnabled(connected); on_goal_->setEnabled(connected);
  for (auto* b : {clear_, start_, pause_, reset_, save_, load_}) b->setEnabled(connected);
}

bool SocialPedestrianPanel::send(plan_manage::ControlSocialScene& s) {
  s.request.expected_revision = state_.revision;
  if (!client_.call(s)) { feedback_->setText(tr("编辑服务不可用。")); return false; }
  if (!s.response.success) { feedback_->setText(tr("操作失败：%1").arg(QString::fromStdString(s.response.message))); return false; }
  state_.revision = s.response.revision;
  feedback_->clear(); return true;
}

void SocialPedestrianPanel::control(const std::string& command) {
  plan_manage::ControlSocialScene s; s.request.command = command;
  s.request.start_on_goal = on_goal_->isChecked();
  s.request.pedestrian.id = state_.selected_id;
  s.request.pedestrian.start.x = x_->value(); s.request.pedestrian.start.y = y_->value();
  s.request.pedestrian.start.theta = yaw_->value() / kDegrees;
  s.request.pedestrian.goal.x = gx_->value(); s.request.pedestrian.goal.y = gy_->value();
  s.request.pedestrian.desired_speed = speed_->value();
  s.request.pedestrian.end_behavior = behavior_->currentData().toString().toStdString();
  if (send(s) && command == "apply") { refresh_ = true; feedback_->setText(tr("设置已应用，全部行人已复位等待。")); }
}

void SocialPedestrianPanel::saveScene() {
  plan_manage::ControlSocialScene s; s.request.command = "export";
  if (!send(s)) return;
  const QString name = QFileDialog::getSaveFileName(this, tr("保存已应用的社会力场景"), "social_scene.yaml", tr("YAML (*.yaml)"));
  if (name.isEmpty()) return;
  QSaveFile file(name); const auto data = QByteArray::fromStdString(s.response.scenario_yaml);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text) || file.write(data) != data.size() || !file.commit())
    feedback_->setText(tr("保存失败：%1").arg(file.errorString()));
  else feedback_->setText(tr("场景已保存：%1").arg(name));
}

void SocialPedestrianPanel::loadScene() {
  const QString name = QFileDialog::getOpenFileName(this, tr("加载社会力场景"), QString(), tr("YAML (*.yaml *.yml)"));
  if (name.isEmpty()) return;
  QFile file(name);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text) || file.size() > 65536) {
    feedback_->setText(tr("无法读取场景，或文件超过 64 KiB。")); return;
  }
  plan_manage::ControlSocialScene s; s.request.command = "load"; s.request.scenario_yaml = file.readAll().toStdString();
  if (file.error() != QFileDevice::NoError) { feedback_->setText(tr("场景读取失败。")); return; }
  if (send(s)) { refresh_ = true; feedback_->setText(tr("已加载，全部行人复位等待。")); }
}
}  // namespace cane_planner

PLUGINLIB_EXPORT_CLASS(cane_planner::SocialPedestrianPanel, rviz::Panel)
