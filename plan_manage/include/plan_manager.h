#ifndef __PLAN_MANAGE__
#define __PLAN_MANAGE__

#include <iostream>
#include <limits>
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
#include <onboard_detector/DynamicObstacles.h>
#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/mpc_controller.h>
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
            REPLAN_TRAJ,
            MPC_STEP
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
        unique_ptr<MpcController> mpc_controller_;
        BsplineOptimizer::Ptr bspline_optimizers_;
        NonUniformBspline::Ptr bspline_init_;

        FSM_STATE exec_state_;
        ros::NodeHandle nh_;             // 节点私有句柄 (用于读param)
        bool have_odom_, have_target_;
        bool simulation_;                 // 里程计来源：true=仿真odom, false=真实TF
        int planner_;                     // 1=A*, 2=kinodynamic, 3=MPC
        bool gazebo_sim_;                 // true=Gazebo mode (publish cmd_vel)
        double no_replan_thresh_, replan_thresh_;

        Eigen::Vector3d odom_pos_, odom_vel_;
        Eigen::Quaterniond odom_ori_;

        Eigen::Vector2d start_pt_;
        Eigen::Vector2d end_pt_;

        Eigen::Vector3d start_state_;
        Eigen::Vector3d end_state_;

        // 动态障碍物缓存
        std::vector<Eigen::Vector3d> dynObsPos_;
        std::vector<Eigen::Vector3d> dynObsVel_;
        std::vector<Eigen::Vector3d> dynObsSize_;
        std::mutex dynObsMutex_;

        // MPC 规划结果缓存 (用于可视化)
        std::vector<Eigen::Vector3d> mpc_com_path_;
        std::vector<Eigen::Vector3d> mpc_feet_path_;
        std::vector<Eigen::Vector3d> mpc_step_path_;

        // 全局路径层 waypoints (A* → MPC 追踪)
        std::vector<Eigen::Vector2d> global_waypoints_;
        size_t global_wp_idx_;
        double global_wp_spacing_ = 1.0;
        double global_wp_arrival_radius_ = 0.1;
        double lookahead_dist_ = 1.0;
        Eigen::Vector2d last_goal_cmd_ = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
        double last_goal_yaw_ = std::numeric_limits<double>::quiet_NaN();
        ros::Time last_goal_cmd_time_;
        double duplicate_goal_xy_thresh_ = 0.05;
        double duplicate_goal_yaw_thresh_ = 0.05;
        double duplicate_goal_time_window_ = 1.5;
        double mpc_fov_range_ = 5.0;
        bool mpc_debug_enable_ = true;
        bool mpc_stop_advice_enable_ = true;
        bool mpc_stop_advice_enforce_ = true;
        double mpc_stop_valid_ratio_thresh_ = 0.08;
        double mpc_stop_clearance_thresh_ = 0.15;
        double mpc_stop_ttc_thresh_ = 0.5;
        double mpc_stop_best_traj_min_front_dist_ = 0.7;
        double mpc_stop_hold_time_ = 0.8;
        double mpc_stop_release_clear_time_ = 0.5;
        bool mpc_yield_enable_ = true;
        double mpc_yield_front_dist_ = 2.0;
        double mpc_yield_lateral_dist_ = 0.9;
        double mpc_yield_cross_speed_ = 0.15;
        double mpc_yield_time_gap_ = 0.8;
        double mpc_yield_min_front_dist_ = 0.7;
        double mpc_yield_release_front_margin_ = 0.4;
        double mpc_yield_release_lateral_margin_ = 0.2;
        double mpc_yield_release_time_margin_ = 0.2;
        bool mpc_yield_occupancy_enable_ = true;
        double mpc_yield_occupancy_lateral_dist_ = 1.25;
        double mpc_yield_occupancy_front_dist_ = 3.0;
        double mpc_yield_occupancy_time_gap_ = 0.6;
        bool mpc_yield_prediction_enable_ = true;
        double mpc_yield_prediction_time_ = 3.5;
        double mpc_yield_prediction_step_ = 0.2;
        double mpc_yield_prediction_longitudinal_dist_ = 0.8;
        bool mpc_behavior_state_enable_ = true;
        bool mpc_behavior_commit_enable_ = false;
        double mpc_behavior_commit_front_dist_ = 0.9;
        double mpc_behavior_commit_back_dist_ = 0.6;
        double mpc_behavior_commit_speed_ = 0.25;
        double mpc_behavior_commit_yaw_rate_ = 0.6;
        double mpc_behavior_yield_hold_time_ = 0.8;
        double mpc_behavior_yield_clear_time_ = 0.7;
        double mpc_behavior_commit_hold_time_ = 0.5;
        double mpc_behavior_commit_clear_time_ = 0.3;
        bool mpc_stop_state_active_ = false;
        ros::Time mpc_stop_enter_time_;
        ros::Time mpc_stop_clear_since_;
        std::string mpc_latched_stop_reason_ = "OK";

        // 仿真路径推进 (planner=1,2 沿规划路径移动 odom)
        std::vector<Eigen::Vector2d> sim_path_;
        size_t sim_path_idx_;
        double sim_speed_;    // 仿真行走速度 m/s

        // MPC 步进状态
        enum MpcSimState { MPC_IDLE, MPC_ACTIVE, MPC_DONE };
        enum MpcBehaviorState { BEHAVIOR_NORMAL, BEHAVIOR_YIELD_BEFORE_CROSSING, BEHAVIOR_COMMIT_TO_PASS };
        MpcSimState mpc_sim_state_;
        MpcBehaviorState mpc_behavior_state_ = BEHAVIOR_NORMAL;
        MpcBehaviorState mpc_last_logged_behavior_state_ = BEHAVIOR_NORMAL;
        ros::Time mpc_behavior_enter_time_;
        ros::Time mpc_behavior_clear_since_;
        int mpc_step_count_;             // 总步数计数 (仅用于日志)
        int mpc_stuck_steps_;            // waypoint 未推进的连续步数
        Eigen::Vector2d last_com_pos_;   // 上一帧CoM位置，用于检测是否实际移动
        double last_theta_;              // 上一帧航向角，用于计算cmd_vel角速度
        static constexpr int STUCK_THRESHOLD = 30;
        bool mpc_reached_goal_;

        Eigen::Vector3d mpc_sim_goal_;

        /*---------- Ros utils -----------*/
        ros::Timer exec_timer_;
        ros::Timer replan_timer_;
        ros::Subscriber odom_sub_, goal_sub, waypoint_sub_;
        ros::Subscriber start_sub_;
        ros::Subscriber dyn_obs_sub_;
        ros::Publisher astar_pub_, kin_vis_pub_, kin_foot_pub_;
        ros::Publisher kin_path_pub_, a_path_pub_;
        ros::Publisher traj_pub_;
        ros::Publisher mpc_vis_pub_, mpc_foot_pub_, mpc_path_pub_;
        ros::Publisher mpc_fov_pub_, mpc_wp_pub_, mpc_wps_pub_;
        ros::Publisher mpc_best_traj_pub_; // MPC predicted optimal CoM path (LINE_STRIP)
        ros::Publisher mpc_debug_metrics_pub_; // std_msgs/Float64MultiArray
        ros::Publisher mpc_stop_advice_pub_;   // std_msgs/Bool
        ros::Publisher mpc_stop_reason_pub_;   // std_msgs/String
        ros::Publisher mpc_behavior_state_pub_; // std_msgs/String
        ros::Publisher mpc_behavior_debug_pub_; // std_msgs/Float64MultiArray
        ros::Publisher risk_field_pub_;   // sensor_msgs::PointCloud2 (risk > hard_threshold)
        ros::Publisher risk_halo_pub_;    // sensor_msgs::PointCloud2 (halo component only)
        ros::Publisher cmd_vel_pub_;      // geometry_msgs::Twist for Gazebo
        ros::Publisher steer_pub_;        // std_msgs::Float64 steering joint angle
        ros::Publisher sim_odom_pub_;
        tf::TransformListener tf_listener_;

        /*---------- helper function -----------*/
        bool callAstarPlan();
        bool callKinodynamicAstarPlan();
        void loadSimPath();     // 从当前planner提取路径到sim_path_
        void stepSimMotion();   // 沿sim_path_推进odom (所有planner通用)
        void mpcSimInit();
        bool mpcSimStep();
        void publishSimOdom();
        void generateGlobalWaypoints();
        void reanchorWaypoint(const Eigen::Vector2d& robot_pos);
        void publishFovRange();
        void publishCurrentWaypoint();
        void publishWaypointsList();
        void publishRiskField(const std::vector<Eigen::Vector3d>& obs_pos,
                              const std::vector<Eigen::Vector3d>& obs_vel);
        void displayAstar();
        void displayKinastar();
        void displayMpcPlan();
        void publishMpcPath();
        double getPathLen(vector<Eigen::Vector2d> list);
        double getPathLen(vector<Eigen::Vector3d> list);
        void publishKinodynamicAstarPath();
        void publishAstarPath();

        void changeFSMExecState(FSM_STATE new_state);
        double QuatenionToYaw(geometry_msgs::Quaternion ori);
        double QuatenionToYaw(Eigen::Quaterniond ori);
        bool shouldIgnoreDuplicateGoal(const Eigen::Vector2d& goal, double yaw, const char* source);

        /*---------- ROS function -----------*/
        void execFSMCallback(const ros::TimerEvent &e);
        void checkCollisionCallback(const ros::TimerEvent &e);
        void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void waypointCallback(const nav_msgs::PathConstPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start);
        void dynamicObstaclesCallback(const onboard_detector::DynamicObstacles::ConstPtr &msg);

    public:
        PlannerManager(int planner = 2)
        {
            planner_ = planner;
            simulation_ = false;
            mpc_sim_state_ = MPC_IDLE;
            sim_path_idx_ = 0;
            sim_speed_ = 0.5;
        }
        ~PlannerManager();
        void init(ros::NodeHandle &nh);
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
