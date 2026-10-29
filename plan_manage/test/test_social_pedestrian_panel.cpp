#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/tool.h>
#include <geometry_msgs/PoseStamped.h>
#include <plan_manage/SocialSceneState.h>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFontMetrics>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <functional>

namespace {
bool waitFor(const std::function<bool()>& condition) {
  const auto deadline = ros::WallTime::now() + ros::WallDuration(6);
  while (ros::WallTime::now() < deadline) {
    QApplication::processEvents(); ros::spinOnce();
    if (condition()) return true;
    QThread::msleep(10);
  }
  return false;
}
void chooseFile(const QString& path) {
  QTimer::singleShot(100, [path] {
    auto* dialog = qobject_cast<QFileDialog*>(QApplication::activeModalWidget()); ASSERT_TRUE(dialog);
    auto* input = dialog->findChild<QLineEdit*>("fileNameEdit"); ASSERT_TRUE(input);
    input->setText(path); QMetaObject::invokeMethod(dialog, "accept");
    QTimer::singleShot(3000, dialog, &QDialog::reject);
  });
}

// Run inside pedsim_scene_smoke.py's owned master with real simulator services.
TEST(SocialPanel, RealEditorAndPortableFiles) {
  pluginlib::ClassLoader<rviz::Tool> tools("rviz", "rviz::Tool");
  EXPECT_EQ(tools.createInstance("plan_manage/SocialStart")->getShortcutKey(), 'p');
  EXPECT_EQ(tools.createInstance("plan_manage/SocialGoal")->getShortcutKey(), 'k');
  EXPECT_EQ(tools.createInstance("plan_manage/SocialSelect")->getShortcutKey(), 'u');
  pluginlib::ClassLoader<rviz::Panel> panels("rviz", "rviz::Panel");
  auto panel = panels.createInstance("plan_manage/SocialPedestrianPanel");
  panel->resize(460, 1050); panel->show();
  auto spin = [&](const char* name) { return panel->findChild<QDoubleSpinBox*>(name); };
  auto button = [&](const char* name) { return panel->findChild<QPushButton*>(name); };
  auto* speed = spin("social_speed"); auto* time = spin("social_time");
  auto* apply = button("social_apply"); auto* add = button("social_add");
  ASSERT_TRUE(speed && time && apply && add);
  ros::NodeHandle nh; plan_manage::SocialSceneState state;
  auto sub = nh.subscribe<plan_manage::SocialSceneState>("/pedsim_scene/state", 1,
      [&state](const plan_manage::SocialSceneState::ConstPtr& msg) { state = *msg; });
  auto pub = nh.advertise<geometry_msgs::PoseStamped>("/pedsim_scene/start_pose", 1);
  ASSERT_TRUE(waitFor([&] { return apply->isEnabled() && pub.getNumSubscribers(); }));
  add->click(); ASSERT_TRUE(waitFor([&] { return state.selected_id == 0; }));
  geometry_msgs::PoseStamped pose; pose.header.frame_id = "world";
  pose.pose.position.x = -3.; pose.pose.position.y = 5.; pose.pose.orientation.w = 1.; pub.publish(pose);
  ASSERT_TRUE(waitFor([&] {
    return state.pedestrians.size() == 3 && speed->isEnabled() && spin("social_x")->value() == -3. &&
        panel->findChild<QComboBox*>("social_people")->currentData().toUInt() == state.selected_id;
  }));
  speed->setValue(.7); time->setValue(2.);
  button("social_calculate")->click();
  EXPECT_NEAR(spin("social_goal_x")->value(), -1.6, .001);
  auto* on_goal = panel->findChild<QCheckBox*>("social_on_goal"); ASSERT_TRUE(on_goal); on_goal->setChecked(false);
  const auto before = state.header.stamp;
  ASSERT_TRUE(waitFor([&] { return (state.header.stamp-before).toSec() > .4; }));
  EXPECT_DOUBLE_EQ(speed->value(), .7);  // heartbeat preserves pending edits
  apply->click();
  ASSERT_TRUE(waitFor([&] { return !state.start_on_goal && state.pedestrians.back().desired_speed == .7; }));
  button("social_start")->click(); ASSERT_TRUE(waitFor([&] { return state.running; }))
      << panel->findChild<QLabel*>("social_feedback")->text().toStdString();
  button("social_pause")->click(); ASSERT_TRUE(waitFor([&] { return !state.running; }));
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  const auto path = directory.filePath("social.yaml");
  chooseFile(path); button("social_save")->click();
  QFile file(path); ASSERT_TRUE(file.open(QIODevice::ReadOnly)); const auto yaml = file.readAll();
  EXPECT_TRUE(yaml.contains("cane_pedsim_scene_v1")); EXPECT_TRUE(yaml.contains("desired_speed: 0.7"));
  button("social_clear")->click(); ASSERT_TRUE(waitFor([&] { return state.pedestrians.empty(); }));
  chooseFile(path); button("social_load")->click();
  ASSERT_TRUE(waitFor([&] { return state.pedestrians.size() == 3 && !state.running; }));
  EXPECT_TRUE(panel->grab().save("/tmp/cane_social_pedestrian_panel.png"));
  const QFontMetrics metrics(QApplication::font());
  for (QChar c : QString::fromUtf8("社会力行人起点终点暂停保存")) EXPECT_TRUE(metrics.inFontUcs4(c.unicode()));
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_social_pedestrian_panel"); testing::InitGoogleTest(&argc, argv);
  QApplication::setAttribute(Qt::AA_DontUseNativeDialogs); QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
