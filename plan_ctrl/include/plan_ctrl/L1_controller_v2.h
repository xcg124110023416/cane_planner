#ifndef L1_CONTROLLER_V2_H
#define L1_CONTROLLER_V2_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <serial/serial.h>

#include <Eigen/Eigen>

#define PI 3.14159265358979

// 帧头43->大写C
#define CMD_FRAME_HEADER 0x43
// cmd
#define CMD_VEL 0x01
#define CMD_POS 0x02
#define CMD_STOP 0x03
#define CMD_UPDATE 0x04
#define CMD_VAREPSILON 0X05
#define CMD_A 0x06
#define CMD_SET_POS_ZERO 0x07
#define CMD_ZERO 0x08

#define GKF_DATA_LEN 4

class L1Controller
{
public:
    L1Controller();
    void initMarker();
    bool isForwardWayPt(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose);
    bool isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos);
    double getYawFromPose(const geometry_msgs::Pose &carPose);
    double getEta(const geometry_msgs::Pose &carPose);
    double getCar2GoalDist();
    double getL1Distance(const double &_Vcmd);
    double getSteeringAngle(double eta);
    double getGasInput(const float &current_v);
    void Set(uint8_t cmd, int16_t data);
    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose);
    static double CalculateEta(const geometry_msgs::Pose &carPose, const geometry_msgs::Point &targetPos);

    // Public for access if needed, or keep private and add getters
    // For now keeping as in original file (private)
    // But if kinodynamic_astar needs to access members, we might need to change this.
    // Assuming it only calls public methods.

private:
    ros::NodeHandle n_;
    ros::Subscriber odom_sub, path_sub, goal_sub, way_sub;
    ros::Publisher pub_, marker_pub, goal_marker_pub;
    ros::Timer timer1, timer2;
    tf::TransformListener tf_listener;

    visualization_msgs::Marker points, line_strip, goal_circle, text_marker;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Point odom_goal_pos;
    nav_msgs::Odometry odom;
    nav_msgs::Path map_path, odom_path;

    // serial
    serial::Serial ser_;
    bool use_ser_flag_;
    int plan_;

    double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
    double Gas_gain, baseAngle, Angle_gain, goalRadius, link_length, wheel_radius;
    int controller_freq, baseSpeed;
    bool foundForwardPt, goal_received, goal_reached, stop_sent_flag_;
    bool have_odom;

    void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
    void waypointCB(const nav_msgs::PathConstPtr &msg);
    void goalReachingCB(const ros::TimerEvent &);
    void controlLoopCB(const ros::TimerEvent &);

};

#endif // L1_CONTROLLER_V2_H
