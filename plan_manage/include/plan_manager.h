#ifndef __PLAN_MANAGE__
#define __PLAN_MANAGE__

#include <iostream>
#include <vector>
#include <mutex>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <onboard_detector/DynamicObstacles.h>  // 动态障碍物话题消息
#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>


// #include <nav_msgs/OccupancyGrid.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/lfpc.h>
#include <plan_env/collision_detection.h>
#include <plan_container.hpp>

namespace cane_planner
{
    class PlannerManager
    {
    private:
        /* data */
        enum FSM_STATE
        {
            INIT,
            WAIT_TARGET,
            GEN_NEW_TRAJ,
            EXEC_TRAJ,
            REPLAN_TRAJ
        };

        enum TRAJECTORY_PLANNING_ID {
            GOAL = 1,
            PATH = 200,
            BSPLINE = 300,
            BSPLINE_CTRL_PT = 400,
            POLY_TRAJ = 500
        };
        /*---------- data -----------*/
        fast_planner::SDFMap::Ptr sdf_map_;
        CollisionDetection::Ptr collision_;
        LFPC::Ptr lfpc_model_;

        
        unique_ptr<Astar> astar_finder_;
        unique_ptr<KinodynamicAstar> kin_finder_;
        BsplineOptimizer::Ptr bspline_optimizers_;
        NonUniformBspline::Ptr bspline_init_;


        FSM_STATE exec_state_;
        bool have_odom_, have_target_;
        bool simulation_;
        // planner:Astar == 1 or kinplan == 2
        int planner_;
        double no_replan_thresh_, replan_thresh_;

        Eigen::Vector3d odom_pos_, odom_vel_;
        Eigen::Quaterniond odom_ori_;

        Eigen::Vector2d start_pt_; // start pos
        Eigen::Vector2d end_pt_;   // target pos

        Eigen::Vector3d start_state_; // start state
        Eigen::Vector3d end_state_;   // end state

        // 动态障碍物缓存（从话题订阅获取）
        std::vector<Eigen::Vector3d> dynObsPos_;
        std::vector<Eigen::Vector3d> dynObsVel_;
        std::vector<Eigen::Vector3d> dynObsSize_;
        std::mutex dynObsMutex_;  // 线程安全锁

        /*---------- Ros utils -----------*/
        ros::Timer exec_timer_;
        ros::Timer replan_timer_;
        ros::Subscriber odom_sub_, goal_sub, waypoint_sub_;
        ros::Subscriber goal_sub_, start_sub_;
        ros::Subscriber dyn_obs_sub_;  // 动态障碍物订阅者
        ros::Publisher astar_pub_, kin_vis_pub_, kin_foot_pub_;
        ros::Publisher kin_path_pub_, a_path_pub_;
        ros::Publisher traj_pub_;
        tf::TransformListener tf_listener_;

        /*---------- helper function -----------*/
        bool callAstarPlan();
        bool callKinodynamicAstarPlan();
        void displayAstar();
        void displayKinastar();
        double getPathLen(vector<Eigen::Vector2d> list);
        double getPathLen(vector<Eigen::Vector3d> list);

        // publish a star path;
        void publishKinodynamicAstarPath();
        void publishAstarPath();

        void changeFSMExecState(FSM_STATE new_state);
        double QuatenionToYaw(geometry_msgs::Quaternion ori);
        double QuatenionToYaw(Eigen::Quaterniond ori);

        /*---------- ROS function -----------*/
        // timer
        void execFSMCallback(const ros::TimerEvent &e);
        void checkCollisionCallback(const ros::TimerEvent &e);
        // sub callback
        void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void waypointCallback(const nav_msgs::PathConstPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        // sim sub callback
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);
        void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start);
        // 动态障碍物话题回调
        void dynamicObstaclesCallback(const onboard_detector::DynamicObstacles::ConstPtr &msg);

    public:
        PlannerManager(bool simulation, int planner)
        {
            simulation_ = simulation;
            planner_ = planner;
        }
        ~PlannerManager();
        void init(ros::NodeHandle &nh);
        void simInit(ros::NodeHandle &nh);
        void callPath();
        void Param_init(ros::NodeHandle &nh);

        void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);
        
        void displaySphereList(const vector<Eigen::Vector3d>& list);

        PlanParameters pp_;
    };

} // namespace cane_planner

#endif