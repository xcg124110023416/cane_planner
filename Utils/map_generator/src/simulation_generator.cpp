#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// [ADDED] 引入 TF2 相关的头文件
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
string file_name;
ros::Publisher odom_pub, pose_pub;
nav_msgs::Odometry odom;

void testCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start)
{
  odom.header.frame_id = "world";
  // 这里接收到了 RViz 2DPose 的设定，包含了位置和朝向
  odom.pose = start->pose; 
  // odom.pose.pose.position.z = 1.0;
  ROS_INFO("get start");
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
  static ros::Time current_time = ros::Time::now();
  ros::Time now_time = ros::Time::now();
  double T = now_time.toSec() - current_time.toSec();
  // ROS_INFO("time change is %lf", T); // 避免刷屏，可以注释掉

  // odom
  // TODO:odom change (简单的运动学模拟可以在这里做)
  // 例如：
  // double yaw = tf2::getYaw(odom.pose.pose.orientation);
  // odom.pose.pose.position.x += twist->linear.x * cos(yaw) * T;
  // odom.pose.pose.position.y += twist->linear.x * sin(yaw) * T;
  // 更新 current_time...
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulation_generator");
  ros::NodeHandle node;

  ros::Publisher cloud_pub =
      node.advertise<sensor_msgs::PointCloud2>("/simulation_generator/global_cloud", 10, true);
  pose_pub =
      node.advertise<geometry_msgs::PoseStamped>("/simulation_generator/pose", 10, true);
  odom_pub =
      node.advertise<nav_msgs::Odometry>("/simulation_generator/odom", 10, true);

  ros::Subscriber start_sub = node.subscribe("/initialpose", 10, testCallback);
  ros::Subscriber cmd_sub = node.subscribe("/cmd_vel", 10, cmdCallback);
  
  // [ADDED] 创建 TF 广播器
  static tf2_ros::TransformBroadcaster br;

  if(argc > 1) {
      file_name = argv[1];
  } else {
      // 防止未输入参数报错
      ROS_WARN("No pcd file provided, running without map or use default.");
  }

  ros::Duration(1.0).sleep();

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // 简单的容错检查
  if (!file_name.empty()) {
      int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud);
      if (status == -1)
      {
        cout << "can't read file." << endl;
        // return -1; // 建议不要直接退出，以免仅仅为了测试TF而必须加载地图
      }
  }

  // init odom
  odom.header.frame_id = "world";
  odom.child_frame_id = "cane_base"; // 建议加上 child_frame_id
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 1.0; // 注意：通常地面机器人的Z是0，除非你的雷达或base有高度
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";

  int count = 0;
  while (ros::ok())
  {
    // [MODIFIED] 为了保证TF流畅，Loop频率最好高一点，或者将TF发布独立出去。
    // 当前是 0.2s sleep (5Hz)，对于测试勉强够用，但建议 10Hz-50Hz。
    ros::Duration(0.02).sleep(); // 改为 50Hz，让 TF 更顺滑

    // 1. 发布 Odom 消息
    odom.header.stamp = ros::Time::now();
    odom_pub.publish(odom);
    
    // [ADDED] 2. 发布 TF 变换 (/world -> /cane_base)
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";       // 父坐标系
    transformStamped.child_frame_id = "cane_base";    // 子坐标系 (你的机器人base)
    
    // 将 odom 中的位置赋值给 TF
    transformStamped.transform.translation.x = odom.pose.pose.position.x;
    transformStamped.transform.translation.y = odom.pose.pose.position.y;
    transformStamped.transform.translation.z = odom.pose.pose.position.z;
    
    // 将 odom 中的朝向(四元数)赋值给 TF (这是解决你问题的关键！)
    transformStamped.transform.rotation = odom.pose.pose.orientation;

    // 发送变换
    br.sendTransform(transformStamped);

    // 降低点云发布频率 (没必要 50Hz 发一次大点云)
    if (count % 50 == 0) { 
        cloud_pub.publish(msg);
    }

    count++;
    ros::spinOnce();
  }

  return 0;
}