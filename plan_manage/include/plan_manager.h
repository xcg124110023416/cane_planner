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
        bool have_odom_, have_target_;
        bool simulation_;                 // 里程计来源：true=仿真odom, false=真实TF
        int planner_;                     // 1=A*, 2=kinodynamic, 3=MPC
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
        double global_wp_arrival_radius_ = 0.5;
        double lookahead_dist_ = 1.0;
        double mpc_fov_range_ = 5.0;

        // 仿真路径推进 (planner=1,2 沿规划路径移动 odom)
        std::vector<Eigen::Vector2d> sim_path_;
        size_t sim_path_idx_;
        double sim_speed_;    // 仿真行走速度 m/s

        // MPC 步进状态
        enum MpcSimState { MPC_IDLE, MPC_ACTIVE, MPC_DONE };
        MpcSimState mpc_sim_state_;
        int mpc_step_count_;
        int mpc_max_steps_;
        int mpc_stuck_steps_;            // waypoint 未推进的连续步数
        int mpc_blocked_count_;          // 阻塞迟滞计数 (防闪烁)
        Eigen::Vector2d last_com_pos_;   // 上一帧CoM位置，用于检测是否实际移动
        static constexpr int STUCK_THRESHOLD = 30;
        int unblock_cooldown_;           // 解封冷却帧数
        bool mpc_reached_goal_;

        // 停等/恢复参数 (从launch读取)
        double narrow_clearance_;        // 窄通道判定: clearance低于此值算"窄"
        double narrow_risk_ratio_;       // 窄通道判定: risk > safe×ratio 算"有人在挤"
        double unblock_risk_ratio_;      // 恢复期门槛: 行人附近时 safe×ratio
        double ped_nearby_range_;        // 行人"附近"的距离范围
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
        ros::Publisher risk_field_pub_;   // sensor_msgs::PointCloud2
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
