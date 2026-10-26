#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/tool.h>
#include <geometry_msgs/PoseStamped.h>
#include <plan_manage/PedestrianState.h>
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

bool waitFor(const std::function<bool()>& predicate) {
  const auto deadline = ros::WallTime::now() + ros::WallDuration(7);
  while (ros::WallTime::now() < deadline) {
    QApplication::processEvents();
    ros::spinOnce();
    if (predicate()) return true;
    QThread::msleep(10);
  }
  return false;
}

void chooseDialogFile(const QString& filename) {
  QTimer::singleShot(100, [filename] {
    auto* dialog = qobject_cast<QFileDialog*>(QApplication::activeModalWidget());
    ASSERT_TRUE(dialog);
    auto* input = dialog->findChild<QLineEdit*>("fileNameEdit");
    ASSERT_TRUE(input);
    // An absolute typed name avoids racing QFileSystemModel's asynchronous selection.
    input->setText(filename);
    QMetaObject::invokeMethod(dialog, "accept");
    QTimer::singleShot(3000, dialog, &QDialog::reject);
  });
}

TEST(PedestrianPanel, LoadsPluginsAndControlsTheSimulator) {
  pluginlib::ClassLoader<rviz::Tool> tool_loader("rviz", "rviz::Tool");
  auto tool = tool_loader.createInstance("plan_manage/PlacePedestrian");
  ASSERT_TRUE(tool);
  EXPECT_EQ(tool->getShortcutKey(), 'p');
  pluginlib::ClassLoader<rviz::Panel> loader("rviz", "rviz::Panel");
  auto panel = loader.createInstance("plan_manage/PedestrianPanel");
  ASSERT_TRUE(panel);
  panel->resize(360, 550);
  panel->show();
  auto* speed = panel->findChild<QDoubleSpinBox*>("pedestrian_speed");
  auto* duration = panel->findChild<QDoubleSpinBox*>("pedestrian_duration");
  auto* behavior = panel->findChild<QComboBox*>("pedestrian_behavior");
  auto* goal_trigger = panel->findChild<QCheckBox*>("pedestrian_on_goal");
  auto* apply = panel->findChild<QPushButton*>("pedestrian_apply");
  auto* start = panel->findChild<QPushButton*>("pedestrian_start");
  auto* pause = panel->findChild<QPushButton*>("pedestrian_pause");
  auto* reset = panel->findChild<QPushButton*>("pedestrian_reset");
  auto* clear = panel->findChild<QPushButton*>("pedestrian_clear");
  ASSERT_TRUE(speed && duration && behavior && goal_trigger && apply && start && pause && reset && clear);
  ros::NodeHandle nh;
  plan_manage::PedestrianState state;
  auto sub = nh.subscribe<plan_manage::PedestrianState>("/pedestrian_sim/state", 1,
      [&state](const plan_manage::PedestrianState::ConstPtr& msg) { state = *msg; });
  auto pub = nh.advertise<geometry_msgs::PoseStamped>("/pedestrian_sim/initial_pose", 1);
  ASSERT_TRUE(waitFor([&] { return apply->isEnabled() && pub.getNumSubscribers() > 0; }));
  EXPECT_DOUBLE_EQ(speed->value(), 0.4);
  EXPECT_DOUBLE_EQ(duration->value(), 10.0);
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 2;
  pose.pose.position.y = 1;
  pose.pose.orientation.w = 1;
  pub.publish(pose);
  ASSERT_TRUE(waitFor([&] { return state.phase == "waiting" && state.start_pose.position.y == 1; }));
  speed->setValue(0.6);
  duration->setValue(2.0);
  behavior->setCurrentIndex(behavior->findData("return"));
  goal_trigger->setChecked(false);
  // Fresh status heartbeats must not overwrite un-applied edits.
  const auto stamp = state.header.stamp;
  ASSERT_TRUE(waitFor([&] { return (state.header.stamp - stamp).toSec() > 0.3; }));
  EXPECT_DOUBLE_EQ(speed->value(), 0.6);
  apply->click();
  ASSERT_TRUE(waitFor([&] { return state.speed == 0.6 && state.duration == 2.0 && !state.start_on_goal; }));
  EXPECT_EQ(state.end_behavior, "return");
  start->click();
  ASSERT_TRUE(waitFor([&] { return state.phase == "walking" && pause->isEnabled(); }));
  pause->click();
  ASSERT_TRUE(waitFor([&] { return state.phase == "paused"; }));
  EXPECT_TRUE(state.visible);
  start->click();
  ASSERT_TRUE(waitFor([&] { return state.phase == "finished"; }));
  reset->click();
  ASSERT_TRUE(waitFor([&] { return state.phase == "waiting"; }));
  EXPECT_TRUE(panel->grab().save("/tmp/cane_pedestrian_panel.png"));
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const auto filename = dir.filePath("pedestrian_custom.yaml");
  chooseDialogFile(filename);
  auto* save = panel->findChild<QPushButton*>("pedestrian_save");
  ASSERT_TRUE(save->isEnabled());
  save->click();
  QFile exported(filename);
  ASSERT_TRUE(exported.open(QIODevice::ReadOnly))
      << panel->findChild<QLabel*>("pedestrian_feedback")->text().toStdString();
  const auto yaml = exported.readAll();
  EXPECT_TRUE(yaml.contains("custom_pedestrian:"));
  EXPECT_TRUE(yaml.contains("mode: controlled"));
  EXPECT_TRUE(yaml.contains("end_behavior: return"));
  EXPECT_TRUE(yaml.contains("start_on_goal: false"));
  // Leave the actual exported artifact available for an independent reload smoke check.
  QFile::remove("/tmp/cane_pedestrian_export.yaml");
  EXPECT_TRUE(exported.copy("/tmp/cane_pedestrian_export.yaml"));
  clear->click();
  ASSERT_TRUE(waitFor([&] { return !state.placed && !apply->isEnabled(); }));
  auto* load = panel->findChild<QPushButton*>("pedestrian_load");
  ASSERT_TRUE(load && load->isEnabled());
  speed->setValue(1.3);  // Loading must also discard pending edits with equal backend settings.
  chooseDialogFile(filename);
  load->click();
  ASSERT_TRUE(waitFor([&] {
    return state.placed && state.phase == "waiting" && speed->value() == 0.6;
  })) << panel->findChild<QLabel*>("pedestrian_feedback")->text().toStdString();
  EXPECT_DOUBLE_EQ(duration->value(), 2.0);
  EXPECT_EQ(behavior->currentData().toString(), "return");
  EXPECT_TRUE(panel->grab().save("/tmp/cane_pedestrian_panel.png"));
}

TEST(PedestrianPanel, ChineseLabelsHaveFontCoverage) {
  const QFontMetrics metrics(QApplication::font());
  for (const QChar character : QString::fromUtf8("行人设置速度暂停")) {
    EXPECT_TRUE(metrics.inFontUcs4(character.unicode()))
        << "Install the fonts-wqy-microhei runtime dependency for Chinese labels";
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_pedestrian_panel");
  testing::InitGoogleTest(&argc, argv);
  QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
