#include <plan_manager.h>
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cane_planner
{

    PlannerManager::~PlannerManager()
    {
    }
    //------------------- real experience ---------------------
    void PlannerManager::Param_init(ros::NodeHandle &nh)
    {
        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_acc_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
        nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
        nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
        nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

        nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
        nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

        nh.param("planner_node/simulation", simulation_, false);
        nh.param("manager/sim_speed", sim_speed_, 0.5);  // 仿真行走速度 m/s
        nh.param("manager/global_wp_spacing", global_wp_spacing_, 1.0);
        nh.param("manager/global_wp_arrival_radius", global_wp_arrival_radius_, 0.5);
        nh.param("manager/lookahead_dist", lookahead_dist_, 1.0);
        nh.param("mpc/fov_range", mpc_fov_range_, 5.0);
    }


    void PlannerManager::init(ros::NodeHandle &nh)
    {
        nh_ = nh;
        // init FSM
        exec_state_ = FSM_STATE::INIT;
        have_odom_ = false;
        have_target_ = false;
        Param_init(nh);
        // init detector
        // ROS_WARN(" onboard detector start");
        // detector_.reset(new onboardDetector::dynamicDetector(nh));
        // init esdf_map and collision
        ROS_WARN(" sdf_map and collision start");
        sdf_map_.reset(new fast_planner::SDFMap);
        // sdf_map_->setDetector(detector_);
        sdf_map_->initMap(nh);
        collision_.reset(new CollisionDetection);
        collision_->init(nh);
        collision_->setMap(sdf_map_);
        // init kin planner
        ROS_WARN(" Astar planer start");
        astar_finder_.reset(new Astar);
        astar_finder_->setParam(nh);
        astar_finder_->setCollision(collision_);
        astar_finder_->init();
        // init lfpc model
        ROS_WARN(" LFPC model start");
        lfpc_model_.reset(new LFPC);
        lfpc_model_->initializeModel(nh);
        lfpc_model_->setCollisionDetection(collision_);
        // init kin planner
        ROS_WARN(" kinodynamic planer start");
        kin_finder_.reset(new KinodynamicAstar);
        kin_finder_->setParam(nh);
        kin_finder_->setCollision(collision_);
        kin_finder_->setModel(lfpc_model_);
        kin_finder_->init();
        // init MPC controller (planner=3)
        if (planner_ == 3)
        {
            ROS_WARN(" MPC controller start");
            mpc_controller_.reset(new MpcController);
            mpc_controller_->setParam(nh);
            mpc_controller_->setModel(lfpc_model_);
            mpc_controller_->setCollision(collision_);
            mpc_controller_->init();
        }
        //init bspline
        ROS_WARN(" Bspline start");
        bspline_init_.reset(new NonUniformBspline);
        // init bspline optimizer
        ROS_WARN(" Bspline optimizer start");
        bspline_optimizers_.reset(new BsplineOptimizer);
        bspline_optimizers_->setParam(nh);
        // replan
        goal_sub =
            nh.subscribe("/move_base_simple/goal", 1, &PlannerManager::GoalCallback, this);
        waypoint_sub_ =
            nh.subscribe("/waypoint_generator/waypoints", 1, &PlannerManager::waypointCallback, this);
        odom_sub_ =
            nh.subscribe("/odom_world", 1, &PlannerManager::odometryCallback, this);
        // 仿真模式下订阅初始位姿
        start_sub_ =
            nh.subscribe("/initialpose", 1, &PlannerManager::startCallback, this);
        // 订阅动态障碍物信息话题
        dyn_obs_sub_ = nh.subscribe("/onboard_detector/dynamic_obstacles_info", 10, 
                                     &PlannerManager::dynamicObstaclesCallback, this);
        // Timer
        exec_timer_ =
            nh.createTimer(ros::Duration(0.1), &PlannerManager::execFSMCallback, this);
        // replan_timer_ =
            // nh.createTimer(ros::Duration(0.1), &PlannerManager::checkCollisionCallback, this);
        // Visial
        astar_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/kinpath_sample", 20);
        kin_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/kin_astar", 20);
        kin_foot_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/kin_foot", 20);
        // Path
        kin_path_pub_ = nh.advertise<nav_msgs::Path>("/kin_astar/path", 20);
        a_path_pub_ = nh.advertise<nav_msgs::Path>("/astar/path", 20);
        traj_pub_ = nh.advertise<nav_msgs::Path>("/planning_vis/trajectory", 20);
        // MPC vis
        mpc_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/mpc_rollout", 20);
        mpc_foot_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/mpc_foot", 20);
        mpc_path_pub_ = nh.advertise<nav_msgs::Path>("/mpc/path", 20);
        sim_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/sim_odom", 20);
        mpc_fov_pub_ = nh.advertise<visualization_msgs::Marker>("/mpc/fov_range", 10);
        mpc_wp_pub_ = nh.advertise<visualization_msgs::Marker>("/mpc/current_waypoint", 10);
        mpc_wps_pub_ = nh.advertise<visualization_msgs::Marker>("/mpc/waypoints", 10);
        risk_field_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/mpc/risk_field", 1);
        risk_halo_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/mpc/risk_halo", 1);
        mpc_best_traj_pub_ = nh.advertise<visualization_msgs::Marker>("/mpc/best_traj", 10);
    }
    // real experience callback waypoint or goal
    void PlannerManager::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (msg->pose.position.z < -0.1)
            return;
        end_pt_ << msg->pose.position.x, msg->pose.position.y;
        end_state_(0) = msg->pose.position.x;
        end_state_(1) = msg->pose.position.y;
        double yaw = QuatenionToYaw(msg->pose.orientation);
        end_state_(2) = yaw;
        // ROS_INFO("set end pos is: %lf and %lf", end_pt_(0), end_pt_(1));
        // ROS_INFO("end yaw is: %lf", yaw);
        have_target_ = true;
    }
    void PlannerManager::waypointCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (msg->poses[0].pose.position.z < -0.1)
            return;
        end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y;
        end_state_(0) = msg->poses[0].pose.position.x;
        end_state_(1) = msg->poses[0].pose.position.y;
        double yaw = QuatenionToYaw(msg->poses[0].pose.orientation);
        end_state_(2) = yaw;
        // ROS_INFO("set end pos is: %lf and %lf", end_pt_(0), end_pt_(1));
        // ROS_INFO("end yaw is: %lf", yaw);
        have_target_ = true;
    }
    // odomtry
    void PlannerManager::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start)
    {
        double px = start->pose.pose.position.x;
        double py = start->pose.pose.position.y;
        double yaw = QuatenionToYaw(start->pose.pose.orientation);

        start_pt_(0) = px;
        start_pt_(1) = py;
        start_state_(0) = px;
        start_state_(1) = py;
        start_state_(2) = yaw;

        odom_pos_(0) = px;
        odom_pos_(1) = py;
        odom_pos_(2) = 0.0;

        have_odom_ = true;
    }

    void PlannerManager::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        // 仿真模式下odom由自身控制(MPC步进或沿路径推进)，不依赖外部里程计
        if (simulation_)
            return;

        // transform cam to world
        geometry_msgs::PoseStamped pose_cam;
        pose_cam.header = msg->header;
        pose_cam.pose = msg->pose.pose;
        geometry_msgs::PoseStamped pose_world;
        tf_listener_.transformPose("world", pose_cam, pose_world);
        // position
        odom_pos_(0) = pose_world.pose.position.x;
        odom_pos_(1) = pose_world.pose.position.y;
        odom_pos_(2) = pose_world.pose.position.z;
        // ori
        if (!simulation_) // using faster-lio
        {
            tf::StampedTransform trans;
            try
            {
                tf_listener_.lookupTransform("/world", "/cane_base", ros::Time(), trans);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                return;
            }

            auto cane_Q = trans.getRotation();

            odom_ori_.x() = cane_Q.getX();
            odom_ori_.y() = cane_Q.getY();
            odom_ori_.z() = cane_Q.getZ();
            odom_ori_.w() = cane_Q.getW();
        }
        else
        {
            odom_vel_(0) = msg->twist.twist.linear.x;
            odom_vel_(1) = msg->twist.twist.linear.y;
            odom_vel_(2) = msg->twist.twist.linear.z;
            odom_ori_.x() = pose_world.pose.orientation.x;
            odom_ori_.y() = pose_world.pose.orientation.y;
            odom_ori_.z() = pose_world.pose.orientation.z;
            odom_ori_.w() = pose_world.pose.orientation.w;
        }

        // odom and start set
        // start_pt_(0) = odom_pos_(0);
        // start_pt_(1) = odom_pos_(1);
        // start_state_(0) = odom_pos_(0);
        // start_state_(1) = odom_pos_(1);

        // yaw = QuatenionToYaw(odom_ori_);
        // yaw = QuatenionToYaw(msg->pose.pose.orientation);
        // ROS_WARN("start_pt_ is %f and %f", start_pt_(0), start_pt_(1));
        // ROS_WARN("odom_yaw is %f,change is %f", yaw_test, yaw);

        double yaw = 0.0;
        Eigen::Vector3d rot_x = odom_ori_.toRotationMatrix().block(0, 0, 3, 1);
        yaw = atan2(rot_x(1), rot_x(0));
        start_state_(2) = yaw;
        // ROS_WARN("odom_yaw is %f", yaw);

        have_odom_ = true;
    }

    // 动态障碍物话题回调函数
    void PlannerManager::dynamicObstaclesCallback(const onboard_detector::DynamicObstacles::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(dynObsMutex_);
        
        // 清空旧数据
        dynObsPos_.clear();
        dynObsVel_.clear();
        dynObsSize_.clear();
        
        // 机器人尺寸用于膨胀障碍物
        Eigen::Vector3d robotSize(0.5, 0.5, 0);
        
        // 从消息中提取动态障碍物信息
        for (size_t i = 0; i < msg->num; ++i) {
            Eigen::Vector3d pos(msg->position[i].x, msg->position[i].y, msg->position[i].z);
            Eigen::Vector3d vel(msg->velocity[i].x, msg->velocity[i].y, msg->velocity[i].z);
            // 尺寸膨胀机器人大小
            Eigen::Vector3d size(msg->size[i].x + robotSize(0), 
                                 msg->size[i].y + robotSize(1), 
                                 msg->size[i].z + robotSize(2));
            
            dynObsPos_.push_back(pos);
            dynObsVel_.push_back(vel);
            dynObsSize_.push_back(size);
            // cout<<"Dynamic Obstacle " << i << ": Pos(" << pos.transpose() << "), Vel(" << vel.transpose() << "), Size(" << size.transpose() << ")" << endl; 
        }
    }

    // ------------------------ FSM Callback --------------------------------
    void PlannerManager::execFSMCallback(const ros::TimerEvent &e)
    {
        static int fsm_num = 0;
        static bool success1 = false;
        static bool success2 = false;
        fsm_num++;
        if (fsm_num == 100)
        {
            fsm_num = 0;
            if (!have_odom_)
                ROS_WARN("no odom.");
            if (!have_target_)
                ROS_WARN("wait for goal.");
        }
        // Risk field visualization (always on for planner=3, regardless of FSM state)
        if (planner_ == 3 && mpc_controller_)
        {
            std::vector<Eigen::Vector3d> obs_pos, obs_vel;
            {
                std::lock_guard<std::mutex> lock(dynObsMutex_);
                obs_pos = dynObsPos_;
                obs_vel = dynObsVel_;
            }
            publishRiskField(obs_pos, obs_vel);
        }

        // FSM loop
        switch (exec_state_)
        {
            case INIT:
            {
                if (!have_odom_)
                    return;
                changeFSMExecState(WAIT_TARGET);
                break;
            }
            case WAIT_TARGET:
            {
                if (!have_target_)
                    return;
                changeFSMExecState(GEN_NEW_TRAJ);
                break;
            }
            case GEN_NEW_TRAJ:
            {
                if (planner_ == 1)
                {
                    success1 = callAstarPlan();
                    if (success1)
                    {
                        loadSimPath();
                        changeFSMExecState(EXEC_TRAJ);
                    }
                    else
                        changeFSMExecState(REPLAN_TRAJ);
                }
                else if (planner_ == 2)
                {
                    success2 = callKinodynamicAstarPlan();
                    if (success2)
                    {
                        loadSimPath();
                        changeFSMExecState(EXEC_TRAJ);
                    }
                    else
                        changeFSMExecState(REPLAN_TRAJ);
                }
                else if (planner_ == 3)
                {
                    // A* 全局规划 → 生成 waypoints，MPC 局部追踪
                    if (callAstarPlan())
                        generateGlobalWaypoints();
                    mpcSimInit();
                    changeFSMExecState(MPC_STEP);
                }

                break;
            }
            case MPC_STEP:
            {
                bool done = mpcSimStep();
                if (done)
                {
                    if (mpc_reached_goal_)
                    {
                        displayMpcPlan();
                        publishMpcPath();
                        success2 = true;
                        changeFSMExecState(EXEC_TRAJ);
                    }
                    else
                    {
                        ROS_WARN("MPC: max steps, replanning...");
                        success2 = false;
                        changeFSMExecState(REPLAN_TRAJ);
                    }
                }
                break;
            }
            case REPLAN_TRAJ:
            {
                if (planner_ == 1)
                {
                    success1 = callAstarPlan();
                    if (success1)
                    {
                        loadSimPath();
                        changeFSMExecState(EXEC_TRAJ);
                    }
                    else
                        changeFSMExecState(REPLAN_TRAJ);
                }
                else if (planner_ == 2)
                {
                    success2 = callKinodynamicAstarPlan();
                    if (success2)
                    {
                        loadSimPath();
                        changeFSMExecState(EXEC_TRAJ);
                    }
                    else
                        changeFSMExecState(REPLAN_TRAJ);
                }
                else if (planner_ == 3)
                {
                    // 重规划：从当前位置重新跑 A* + 生成 waypoints
                    if (callAstarPlan())
                        generateGlobalWaypoints();
                    mpcSimInit();
                    changeFSMExecState(MPC_STEP);
                }
                break;
            }
            case EXEC_TRAJ:
            {
                // if (success1) // a star success
                // {
                //     displayAstar();
                //     publishAstarPath();
                // }
                // else if (success2) // kin star success
                // {
                //     drawBspline(*bspline_init_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                //                 Eigen::Vector4d(1, 0, 0, 1));   //发布拟合的b样条
                //     // displayKinastar(); //发布离散点和足迹
                //     // publishKinodynamicAstarPath();  //发布路径
                // }
                // 仿真模式下沿路径推进odom (planner=3由MPC_STEP自行处理)
                if (simulation_ && planner_ != 3)
                    stepSimMotion();

                Eigen::Vector2d odom_pt(odom_pos_(0), odom_pos_(1));

                double dis2end = (odom_pt - end_pt_).norm();
                double dis2start = (odom_pt - start_pt_).norm();
                if (dis2end <= 0.5)
                {
                    have_target_ = false;
                    sim_path_.clear();
                    ROS_WARN("Reach the destination");
                    changeFSMExecState(WAIT_TARGET);
                }
                else if (dis2end < no_replan_thresh_)
                {
                    return;
                }
                else if (dis2start < replan_thresh_)
                {
                    return;
                }
                else
                {
                    changeFSMExecState(REPLAN_TRAJ);
                }
                break;
            }
        }
        return;
    }
    // --------------------------- Collision replan ----------------------------
    void PlannerManager::checkCollisionCallback(const ros::TimerEvent &e)
    {
        // end pos is in Collision,change end pos in 0.5 range
        if (have_target_)
        {
            double dist = collision_->getCollisionDistance(end_pt_);
            if (dist <= 0.2)
            {
                /* try to find a max distance goal around */
                const double dr = 0.5, dtheta = 30;
                double new_x, new_y, new_z, max_dist = -1.0;
                Eigen::Vector3d goal(-1, -1, -1);
                for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
                {
                    for (double theta = -90; theta <= 270; theta += dtheta)
                    {
                        new_x = end_pt_(0) + r * cos(theta / 57.3);
                        new_y = end_pt_(1) + r * sin(theta / 57.3);
                        new_z = 1.0;
                        Eigen::Vector2d new_pt(new_x, new_y);
                        dist = collision_->getCollisionDistance(new_pt);
                        if (dist > max_dist)
                        {
                            /* reset end_pt_ */
                            goal(0) = new_x;
                            goal(1) = new_y;
                            goal(2) = new_z;
                            max_dist = dist;
                        }
                    }
                }
                if (max_dist > 0.2)
                {
                    end_pt_ << goal(0), goal(1);
                    end_state_(0) = goal(0);
                    end_state_(1) = goal(1);
                    have_target_ = true;
                    if (exec_state_ == EXEC_TRAJ)
                    {
                        ROS_WARN("goal near collision,change end");
                        changeFSMExecState(REPLAN_TRAJ);
                    }
                }
                else
                {
                    have_target_ = false;
                    cout << "Goal near collision, stop." << endl;
                    changeFSMExecState(WAIT_TARGET);
                }
            }
        }
        // Collision replan
        if (exec_state_ == EXEC_TRAJ)
        {
            vector<Eigen::Vector3d> list;
            list = kin_finder_->getPath();
            for (size_t i = 0; i < list.size(); i++)
            {
                // Eigen::Vector2d temp(list[i](0), list[i](1));
                // double dist = collision_->getCollisionDistance(temp);
                // if (dist < 0.1)
                Eigen::Vector3d pro_pos = list[i];
                if (collision_->sdf_map_->getInflateOccupancy(pro_pos) == 1)
                {
                    ROS_WARN("current traj in collision.");
                    changeFSMExecState(REPLAN_TRAJ);
                }
            }
        }
        return;
    }

    // ------------------------- helper function -------------------------------------
    void PlannerManager::changeFSMExecState(FSM_STATE new_state)
    {
        string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ", "MPC_STEP"};
        // int pre_s = int(exec_state_);
        exec_state_ = new_state;
        // cout << "[now]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    }
    bool PlannerManager::callAstarPlan()
    {
        static int num = 0;
        astar_finder_->reset();
        start_pt_(0) = odom_pos_(0);
        start_pt_(1) = odom_pos_(1);
        start_state_(0) = odom_pos_(0);
        start_state_(1) = odom_pos_(1);
        num++;
        std::cout << "astar"
                  << "," << num << ",";
        ros::Time time_1 = ros::Time::now();
        bool plan_success = astar_finder_->search(start_pt_, end_pt_);
        ros::Time time_2 = ros::Time::now();
        if (plan_success)
        {
            std::cout << (time_2 - time_1).toSec() << ",";
            // vector<Eigen::Vector2d> list;
            // list = astar_finder_->getPath();
            // double len = getPathLen(list);
            // std::cout << len << ",1" << std::endl;
            publishAstarPath();
        }

        return plan_success;
    }
    bool PlannerManager::callKinodynamicAstarPlan()
    {
        static int num = 0;

        kin_finder_->reset();
        num++;       
        // ==================== 从缓存获取动态障碍物信息（通过话题订阅更新） ====================
        {
            std::lock_guard<std::mutex> lock(dynObsMutex_);
            // 将缓存的动态障碍物信息传给kinodynamic_astar
            kin_finder_->setDynamicObstacles(dynObsPos_, dynObsVel_, dynObsSize_);
            // cout<<"Set " << dynObsPos_.size() << " dynamic obstacles to kinodynamic A*." << endl;
        }
        // =========================================================================
        
        start_pt_(0) = odom_pos_(0);
        start_pt_(1) = odom_pos_(1);
        start_state_(0) = odom_pos_(0);
        start_state_(1) = odom_pos_(1);
        Eigen::Vector3d input;
        // double vx, vy;
        // vx = 0.5 * sin(start_state_(2));
        // vy = 0.5 * cos(start_state_(2));
        // input << 0.0, vx, 0.0, vy;
        input << 0.0, 0.0, start_state_(2);//vx,vy,theta
        //
        ros::Time time_1 = ros::Time::now();
        cout<<"end_state_: "<<end_state_.transpose()<<endl;
        bool plan_success = kin_finder_->search(start_state_, input, end_state_);

        if (!plan_success) {
            ROS_WARN("[Planner Manager]: Kinodynamic A* failed to find a path!");
            return false; // 直接退出函数
        }

        // B-spline fitting disabled (not needed for front-end validation)
        displayKinastar();
        publishKinodynamicAstarPath();

        ros::Time time_2 = ros::Time::now();
        if (plan_success)
        {
            std::cout << "kin：" << num << "，usedtime：" <<(time_2 - time_1).toSec() << endl;
            // vector<Eigen::Vector3d> list;
            // list = kin_finder_->getPath();  //多个com_pos组成的路径点
            // double len = getPathLen(list);  //路径长度
            // std::cout << len << ",1" << std::endl;
        }
        return plan_success;
    }
    // ==================== 通用仿真路径推进 ====================

    void PlannerManager::loadSimPath()
    {
        sim_path_.clear();
        sim_path_idx_ = 0;

        if (planner_ == 1)
        {
            auto path = astar_finder_->getPath();
            for (const auto& p : path)
                sim_path_.push_back(p);
        }
        else if (planner_ == 2)
        {
            auto path = kin_finder_->getPath();  // vector<Eigen::Vector3d>
            for (const auto& p : path)
                sim_path_.push_back(Eigen::Vector2d(p(0), p(1)));
        }
        // planner=3: MPC handles its own odom stepping, no load needed
    }

    void PlannerManager::stepSimMotion()
    {
        if (sim_path_.empty() || sim_path_idx_ >= sim_path_.size())
            return;

        double step = sim_speed_ * 0.1;  // 0.1s timer interval
        Eigen::Vector2d cur(odom_pos_(0), odom_pos_(1));
        Eigen::Vector2d prev = cur;

        // Advance along path
        while (step > 0 && sim_path_idx_ < sim_path_.size())
        {
            Eigen::Vector2d wp = sim_path_[sim_path_idx_];
            double dist = (wp - cur).norm();
            if (dist <= step)
            {
                cur = wp;
                step -= dist;
                sim_path_idx_++;
            }
            else
            {
                cur += (wp - cur).normalized() * step;
                step = 0;
            }
        }

        odom_pos_(0) = cur(0);
        odom_pos_(1) = cur(1);

        // Publish generic sim odom for rviz
        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = cur(0);
        odom.pose.pose.position.y = cur(1);
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.w = 1.0;
        double vx = (cur(0) - prev(0)) / 0.1;
        double vy = (cur(1) - prev(1)) / 0.1;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        sim_odom_pub_.publish(odom);
    }

    // ==================== 全局路径层 (A* → waypoints) ====================

    void PlannerManager::generateGlobalWaypoints()
    {
        global_waypoints_.clear();
        global_wp_idx_ = 0;

        auto path = astar_finder_->getPath();  // vector<Eigen::Vector2d>
        if (path.size() < 2)
        {
            global_waypoints_.push_back(end_pt_);
            ROS_WARN("[MPC global] A* path too short, using direct goal as only waypoint");
            return;
        }

        // Downsample: walk the A* path, pick points at ~global_wp_spacing_ intervals
        Eigen::Vector2d last = path[0];
        double accum = 0.0;
        for (size_t i = 1; i < path.size(); ++i)
        {
            Eigen::Vector2d seg = path[i] - path[i - 1];
            double seg_len = seg.norm();
            accum += seg_len;
            if (accum >= global_wp_spacing_)
            {
                global_waypoints_.push_back(path[i]);
                accum = 0.0;
                last = path[i];
            }
        }

        // Ensure the final point equals end_pt_
        if (global_waypoints_.empty())
        {
            global_waypoints_.push_back(end_pt_);
        }
        else
        {
            double dist_last_to_end = (end_pt_ - global_waypoints_.back()).norm();
            if (dist_last_to_end > global_wp_spacing_ * 0.3)
                global_waypoints_.push_back(end_pt_);
            else
                global_waypoints_.back() = end_pt_;  // snap last wp to exact goal
        }

        ROS_INFO("[MPC global] Generated %zu waypoints (spacing=%.1fm) for %.1fm path",
                 global_waypoints_.size(), global_wp_spacing_,
                 (path.back() - path.front()).norm());

        publishWaypointsList();
    }

    // ==================== 步进式MPC仿真 ====================

    void PlannerManager::mpcSimInit()
    {
        // 重置MPC warm-start
        mpc_controller_->reset();

        // 从里程计设置起点 (startCallback 已写入 start_state_)
        start_pt_(0) = odom_pos_(0);
        start_pt_(1) = odom_pos_(1);
        start_state_(0) = odom_pos_(0);
        start_state_(1) = odom_pos_(1);

        // 初始化LFPC状态
        Eigen::Vector3d init_v_state(0.0, 0.0, start_state_(2)); // vx, vy, theta
        Eigen::Vector3d com_init_pos(odom_pos_(0), odom_pos_(1), 0.0);
        lfpc_model_->reset(init_v_state, com_init_pos, LEFT_LEG, 0);

        // 缓存目标：优先使用全局 waypoint，A* 失败则直接面向终点
        global_wp_idx_ = 0;
        if (!global_waypoints_.empty())
            mpc_sim_goal_ << global_waypoints_[0](0), global_waypoints_[0](1), 0.0;
        else
            mpc_sim_goal_ << end_state_(0), end_state_(1), 0.0;

        // 清空路径缓存
        mpc_com_path_.clear();
        mpc_feet_path_.clear();
        mpc_step_path_.clear();

        // 初始化计数器
        mpc_step_count_ = 0;
        mpc_stuck_steps_ = 0;
        mpc_reached_goal_ = false;

        // 更新odom初始位置
        odom_pos_ = com_init_pos;
    }

    // 正交投影 + 动态截断：找到路径上最近投影点，重锚定索引，前向截断 L 距离
    void PlannerManager::reanchorWaypoint(const Eigen::Vector2d& robot_pos)
    {
        if (global_waypoints_.size() < 2) return;

        size_t old_idx = global_wp_idx_;

        // Step 1: 遍历所有线段，找到全局最近投影点
        double best_dist = std::numeric_limits<double>::max();
        size_t best_seg = 0;
        Eigen::Vector2d best_proj = global_waypoints_[0];

        for (size_t i = 0; i + 1 < global_waypoints_.size(); ++i)
        {
            const Eigen::Vector2d& a = global_waypoints_[i];
            const Eigen::Vector2d& b = global_waypoints_[i + 1];
            Eigen::Vector2d ab = b - a;
            double seg_len_sq = ab.squaredNorm();
            if (seg_len_sq < 1e-9) continue;

            double t = (robot_pos - a).dot(ab) / seg_len_sq;
            t = std::max(0.0, std::min(1.0, t));  // clamp to segment
            Eigen::Vector2d proj = a + t * ab;
            double dist = (robot_pos - proj).squaredNorm();
            if (dist < best_dist)
            {
                best_dist = dist;
                best_seg = i;
                best_proj = proj;
            }
        }

        // Step 2: 重锚定索引，只进不退
        global_wp_idx_ = std::max(global_wp_idx_, best_seg);

        // Step 3: 从投影点出发，沿路径向前截断 L 距离
        double accum = 0.0;
        Eigen::Vector2d target = best_proj;

        // 第一条线段从投影点到 W_{idx+1}，而非从 W_{idx} 开始
        {
            Eigen::Vector2d first_seg = global_waypoints_[global_wp_idx_ + 1] - best_proj;
            double first_len = first_seg.norm();
            if (first_len >= lookahead_dist_)
            {
                target = best_proj + (lookahead_dist_ / first_len) * first_seg;
                accum = lookahead_dist_;
            }
            else
            {
                accum = first_len;
                target = global_waypoints_[global_wp_idx_ + 1];
            }
        }

        // 后续完整线段
        for (size_t i = global_wp_idx_ + 1; i + 1 < global_waypoints_.size() && accum < lookahead_dist_; ++i)
        {
            Eigen::Vector2d seg = global_waypoints_[i + 1] - global_waypoints_[i];
            double seg_len = seg.norm();
            if (seg_len < 1e-6) continue;

            if (accum + seg_len >= lookahead_dist_)
            {
                double remain = lookahead_dist_ - accum;
                target = global_waypoints_[i] + (remain / seg_len) * seg;
                accum = lookahead_dist_;
                break;
            }
            accum += seg_len;
            target = global_waypoints_[i + 1];
        }

        // 剩余路径不足 lookahead，直接瞄准终点
        if (accum < lookahead_dist_)
            target = end_pt_;

        mpc_sim_goal_ << target(0), target(1), 0.0;

        if (global_wp_idx_ != old_idx)
        {
            mpc_stuck_steps_ = 0;
            ROS_INFO("[MPC] Reanchored wp_idx %zu -> %zu, target (%.2f, %.2f)",
                     old_idx, global_wp_idx_, target(0), target(1));
        }
    }

    bool PlannerManager::mpcSimStep()
    {
        // 从缓存获取动态障碍物
        std::vector<Eigen::Vector3d> obs_pos, obs_vel;
        {
            std::lock_guard<std::mutex> lock(dynObsMutex_);
            obs_pos = dynObsPos_;
            obs_vel = dynObsVel_;
        }

        Eigen::Vector3d current_com = lfpc_model_->getCOMPos();
        Eigen::Vector2d foot_pos = lfpc_model_->getFootPosition();

        // 记录当前位置
        mpc_com_path_.push_back(current_com);
        mpc_feet_path_.push_back(Eigen::Vector3d(foot_pos(0), foot_pos(1), 0.0));

        // 正交投影重锚定 + 前向截断
        reanchorWaypoint(current_com.head(2));

        // 检查是否到达最终目标
        double dist_to_final = (current_com.head(2) - end_pt_).norm();
        // 只在物理上靠近最终目标时才宣布到达
        if (dist_to_final < global_wp_arrival_radius_)
        {
            mpc_reached_goal_ = true;
            ROS_INFO("[MPC] Reached final goal at (%.2f, %.2f), %d steps",
                     current_com(0), current_com(1), mpc_step_count_);
            return true;
        }

        // 卡住检测：基于实际位移而非 waypoint 索引切换
        // (waypoint 间距可能较大，索引不变但机器人仍在前进)
        double moved = (current_com.head(2) - last_com_pos_).norm();
        if (moved > 0.03)  // 本帧移动超过 3cm → 在前进
            mpc_stuck_steps_ = 0;
        else
            mpc_stuck_steps_++;
        last_com_pos_ = current_com.head(2);

        if (mpc_stuck_steps_ > STUCK_THRESHOLD)
        {
            ROS_WARN("[MPC] Stuck for %d steps, triggering replan", STUCK_THRESHOLD);
            mpc_reached_goal_ = false;
            return true;
        }

        // MPC规划一步
        Eigen::Vector3d control = mpc_controller_->plan(
            lfpc_model_, mpc_sim_goal_, obs_pos, obs_vel);

        // 无路可走 → 停止本帧
        if (!mpc_controller_->lastPlanValid())
        {
            if (obs_pos.empty())
                ROS_WARN("[MPC] STOP reason=NO_VALID_PLAN type=static");
            else
            {
                mpc_stuck_steps_ = 0;  // pedestrian-related: waiting is correct
                ROS_WARN("[MPC] STOP reason=NO_VALID_PLAN type=dynamic obs=%zu", obs_pos.size());
            }
            publishFovRange();
            publishCurrentWaypoint();
            mpc_step_count_++;
            return false;
        }

        // 发布MPC最优预测轨迹
        {
            const auto& best_path = mpc_controller_->getBestPath();
            if (!best_path.empty())
            {
                visualization_msgs::Marker traj_mk;
                traj_mk.header.frame_id = "world";
                traj_mk.header.stamp = ros::Time::now();
                traj_mk.ns = "mpc_best";
                traj_mk.id = 0;
                traj_mk.type = visualization_msgs::Marker::LINE_STRIP;
                traj_mk.action = visualization_msgs::Marker::ADD;
                traj_mk.pose.orientation.w = 1.0;
                traj_mk.scale.x = 0.04;
                traj_mk.color.a = 0.9;
                traj_mk.color.r = 0.0;
                traj_mk.color.g = 1.0;
                traj_mk.color.b = 1.0;
                for (const auto& pt : best_path)
                {
                    geometry_msgs::Point p;
                    p.x = pt(0); p.y = pt(1); p.z = 0.2;
                    traj_mk.points.push_back(p);
                }
                mpc_best_traj_pub_.publish(traj_mk);
            }
        }

        // 正常步进
        lfpc_model_->SetCtrlParams(control);
        lfpc_model_->updateOneStep();

        // 收集子步路径
        std::vector<Eigen::Vector3d> step_path = lfpc_model_->getStepCOMPath();
        for (const auto& pt : step_path)
            mpc_step_path_.push_back(pt);

        lfpc_model_->prepareNextStep();

        // 更新里程计位置
        Eigen::Vector3d new_com = lfpc_model_->getCOMPos();
        odom_pos_(0) = new_com(0);
        odom_pos_(1) = new_com(1);
        odom_pos_(2) = new_com(2);

        // 发布模拟里程计
        publishSimOdom();

        // 发布 FOV 范围和当前 waypoint 可视化
        publishFovRange();
        publishCurrentWaypoint();

        // 增量发布可视化（每步更新）
        displayMpcPlan();
        publishMpcPath();

        mpc_step_count_++;
        return false;  // 继续
    }

    void PlannerManager::publishSimOdom()
    {
        Eigen::Vector3d com = lfpc_model_->getCOMPos();
        Eigen::Vector3d vel = lfpc_model_->getNextIterState(); // vx_t, vy_t, theta_
        double theta = vel(2);

        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = com(0);
        odom.pose.pose.position.y = com(1);
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        odom.twist.twist.linear.x = vel(0);
        odom.twist.twist.linear.y = vel(1);
        odom.twist.twist.angular.z = 0.0;

        sim_odom_pub_.publish(odom);
    }

    void PlannerManager::publishFovRange()
    {
        // 绿色半透明 FOV 圆圈
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.ns = "mpc_fov";
        mk.id = 0;
        mk.type = visualization_msgs::Marker::LINE_STRIP;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = 0.05;  // 线宽
        mk.color.a = 0.4;
        mk.color.r = 0.2;
        mk.color.g = 0.8;
        mk.color.b = 0.2;

        double cx = odom_pos_(0);
        double cy = odom_pos_(1);
        int n_segments = 48;
        for (int i = 0; i <= n_segments; ++i)
        {
            double angle = 2.0 * M_PI * i / n_segments;
            geometry_msgs::Point pt;
            pt.x = cx + mpc_fov_range_ * std::cos(angle);
            pt.y = cy + mpc_fov_range_ * std::sin(angle);
            pt.z = 0.05;
            mk.points.push_back(pt);
        }
        mpc_fov_pub_.publish(mk);
    }

    void PlannerManager::publishCurrentWaypoint()
    {
        // 亮黄色当前追踪 waypoint 球
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.ns = "mpc_curr_wp";
        mk.id = 0;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = mpc_sim_goal_(0);
        mk.pose.position.y = mpc_sim_goal_(1);
        mk.pose.position.z = 0.3;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = 0.25;
        mk.scale.y = 0.25;
        mk.scale.z = 0.25;
        mk.color.a = 0.9;
        mk.color.r = 1.0;
        mk.color.g = 0.9;
        mk.color.b = 0.0;
        mpc_wp_pub_.publish(mk);
    }

    void PlannerManager::publishWaypointsList()
    {
        // 全部 waypoints 小球列表
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.ns = "mpc_waypoints";
        mk.id = 0;
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = 0.12;
        mk.scale.y = 0.12;
        mk.scale.z = 0.12;
        mk.color.a = 0.7;
        mk.color.r = 1.0;
        mk.color.g = 0.6;
        mk.color.b = 0.0;

        for (size_t i = 0; i < global_waypoints_.size(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = global_waypoints_[i](0);
            pt.y = global_waypoints_[i](1);
            pt.z = 0.15;
            mk.points.push_back(pt);
        }
        mpc_wps_pub_.publish(mk);
    }

    void PlannerManager::publishRiskField(const std::vector<Eigen::Vector3d>& obs_pos,
                                          const std::vector<Eigen::Vector3d>& obs_vel)
    {
        if (!mpc_controller_ || obs_pos.empty()) return;

        // Hard threshold = A * ratio (points at or above this risk = INF in MPC)
        double A, ratio;
        nh_.param("mpc/risk_A", A, 5.0);
        nh_.param("mpc/risk_hard_threshold_ratio", ratio, 0.25);
        double hard_thresh = A * ratio;

        pcl::PointCloud<pcl::PointXYZI> cloud_hard, cloud_halo;
        cloud_hard.header.frame_id = "world";
        cloud_hard.header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud_halo.header = cloud_hard.header;

        const auto& rf = mpc_controller_->getRiskField();
        double cx = odom_pos_(0);
        double cy = odom_pos_(1);
        double range = mpc_fov_range_ + 2.0;
        double res = 0.2;

        for (double x = cx - range; x <= cx + range; x += res)
        {
            for (double y = cy - range; y <= cy + range; y += res)
            {
                double dx = x - cx;
                double dy = y - cy;
                if (dx*dx + dy*dy > range*range) continue;

                double risk_sum = 0.0, halo_sum = 0.0;
                for (size_t oi = 0; oi < obs_pos.size(); ++oi)
                {
                    risk_sum += rf.getIndividualCostFast(
                        x, y,
                        obs_pos[oi](0), obs_pos[oi](1),
                        obs_vel[oi](0), obs_vel[oi](1));
                    halo_sum += rf.getHaloCostFast(
                        x, y,
                        obs_pos[oi](0), obs_pos[oi](1),
                        obs_vel[oi](0), obs_vel[oi](1));
                }

                // Hard zone: risk above hard threshold (red in Rviz)
                if (risk_sum >= hard_thresh)
                {
                    pcl::PointXYZI pt;
                    pt.x = x; pt.y = y; pt.z = 0.12;
                    pt.intensity = std::min(risk_sum, 15.0);
                    cloud_hard.points.push_back(pt);
                }
                // Halo zone: halo component only, for soft gradient visualization (green in Rviz)
                if (halo_sum > 0.1)
                {
                    pcl::PointXYZI pt;
                    pt.x = x; pt.y = y; pt.z = 0.08;
                    pt.intensity = std::min(halo_sum, 10.0);
                    cloud_halo.points.push_back(pt);
                }
            }
        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud_hard, output);
        risk_field_pub_.publish(output);
        pcl::toROSMsg(cloud_halo, output);
        risk_halo_pub_.publish(output);
    }

    void PlannerManager::displayMpcPlan()
    {
        // 发布MPC CoM路径点 (蓝色)
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::DELETE;
        mk.id = 0;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.r = 0.0;
        mk.color.g = 0.0;
        mk.color.b = 1.0;
        mk.color.a = 0.5;
        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;

        geometry_msgs::Point pt;
        for (size_t i = 0; i < mpc_com_path_.size(); i++)
        {
            pt.x = mpc_com_path_[i](0);
            pt.y = mpc_com_path_[i](1);
            pt.z = collision_->getSliceHeight();
            mk.points.push_back(pt);
        }
        mpc_vis_pub_.publish(mk);

        // 发布MPC足迹 (绿色)
        mk.points.clear();
        mk.color.r = 0.0;
        mk.color.g = 1.0;
        mk.color.b = 0.0;
        mk.color.a = 0.8;
        mk.scale.x = 0.2;
        mk.scale.y = 0.2;
        mk.scale.z = 0.2;
        for (size_t i = 0; i < mpc_feet_path_.size(); i++)
        {
            pt.x = mpc_feet_path_[i](0);
            pt.y = mpc_feet_path_[i](1);
            pt.z = collision_->getSliceHeight();
            mk.points.push_back(pt);
        }
        mpc_foot_pub_.publish(mk);

        ros::Duration(0.001).sleep();
    }

    void PlannerManager::publishMpcPath()
    {
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        for (size_t i = 0; i < mpc_step_path_.size(); i++)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = mpc_step_path_[i](0);
            this_pose_stamped.pose.position.y = mpc_step_path_[i](1);
            this_pose_stamped.pose.position.z = collision_->getSliceHeight();
            this_pose_stamped.pose.orientation.x = 0.0;
            this_pose_stamped.pose.orientation.y = 0.0;
            this_pose_stamped.pose.orientation.z = 0.0;
            this_pose_stamped.pose.orientation.w = 1.0;
            this_pose_stamped.header.frame_id = "world";
            this_pose_stamped.header.stamp = ros::Time::now();
            path.poses.push_back(this_pose_stamped);
        }
        mpc_path_pub_.publish(path);
    }

    // publish traj to L1-control
    void PlannerManager::publishKinodynamicAstarPath()
    {
        vector<Eigen::Vector3d> list;
        list = kin_finder_->getPath();
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        for (size_t i = 0; i < list.size(); i++)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = list[i](0);
            this_pose_stamped.pose.position.y = list[i](1);
            this_pose_stamped.pose.position.z = collision_->getSliceHeight();
            this_pose_stamped.pose.orientation.x = 0.0;
            this_pose_stamped.pose.orientation.y = 0.0;
            this_pose_stamped.pose.orientation.z = 0.0;
            this_pose_stamped.pose.orientation.w = 1.0;
            this_pose_stamped.header.frame_id = "world";
            this_pose_stamped.header.stamp = ros::Time::now();
            path.poses.push_back(this_pose_stamped);
        }
        kin_path_pub_.publish(path);
    }
    // publish astar traj to L1-control
    void PlannerManager::publishAstarPath()
    {
        vector<Eigen::Vector2d> list;
        list = astar_finder_->getPath();
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        for (size_t i = 0; i < list.size(); i++)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = list[i](0);
            this_pose_stamped.pose.position.y = list[i](1);
            this_pose_stamped.pose.position.z = collision_->getSliceHeight();
            this_pose_stamped.pose.orientation.x = 0.0;
            this_pose_stamped.pose.orientation.y = 0.0;
            this_pose_stamped.pose.orientation.z = 0.0;
            this_pose_stamped.pose.orientation.w = 1.0;
            this_pose_stamped.header.frame_id = "world";
            this_pose_stamped.header.stamp = ros::Time::now();
            path.poses.push_back(this_pose_stamped);
        }
        a_path_pub_.publish(path);
    }

    // visial
    void PlannerManager::displayAstar()
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::DELETE;
        mk.id = 0;

        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.r = 1.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;
        mk.color.a = 1;
        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;

        geometry_msgs::Point pt;
        vector<Eigen::Vector2d> list;
        list = astar_finder_->getPath();
        for (int i = 0; i < int(list.size()); i++)
        {
            pt.x = list[i](0);
            pt.y = list[i](1);
            pt.z = 0.0;
            mk.points.push_back(pt);
        }

        // astar_pub_.publish(mk);
        ros::Duration(0.001).sleep();
    }

    void PlannerManager::displayKinastar()
    {
        // marker set
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::DELETE;
        mk.id = 0;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.r = 0.0;
        mk.color.g = 0.0;
        mk.color.b = 1.0;
        mk.color.a = 0.5;
        mk.scale.x = 1.0;
        mk.scale.y = 1.0;
        mk.scale.z = 1.0;
        // give point
        geometry_msgs::Point pt;
        vector<Eigen::Vector3d> list;
        list = kin_finder_->getComPos();
        for (int i = 0; i < int(list.size()); i++)
        {
            pt.x = list[i](0);
            pt.y = list[i](1);
            pt.z = collision_->getSliceHeight();
            mk.points.push_back(pt);
        }
        // publish traj
        kin_vis_pub_.publish(mk);

        mk.color.r = 0.0;
        mk.color.g = 0.0;
        mk.color.b = 1.0;
        mk.color.a = 1;
        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;
        astar_pub_.publish(mk);

        // set feet pos publisher
        mk.points.clear();
        mk.color.r = 0.0;
        mk.color.g = 1.0;
        mk.color.b = 0.0;
        mk.color.a = 0.8;
        mk.scale.x = 0.2;
        mk.scale.y = 0.2;
        mk.scale.z = 0.2;
        list.clear();
        list = kin_finder_->getFeetPos();
        for (int i = 0; i < int(list.size()); i++)
        {
            pt.x = list[i](0);
            pt.y = list[i](1);
            pt.z = collision_->getSliceHeight();
            mk.points.push_back(pt);
        }
        // publish feet
        kin_foot_pub_.publish(mk);

        // ros::Duration(0.001).sleep();
    }
    // calculate YAW by different function
    double PlannerManager::QuatenionToYaw(geometry_msgs::Quaternion ori)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(ori, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }
    // yaw
    double PlannerManager::QuatenionToYaw(Eigen::Quaterniond ori)
    {
        Eigen::Matrix3d oRx = ori.toRotationMatrix();
        // roll world to body is
        double yaw = 0, pitch = -M_PI / 2, roll = M_PI / 2;
        Eigen::Matrix3d Rx;
        Rx = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        oRx = oRx * Rx;
        Eigen::Vector3d ea = oRx.eulerAngles(2, 1, 0);
        // ZYX ,yaw is ea(0)
        return ea(0);
    }
    //  calculate path len
    double PlannerManager::getPathLen(vector<Eigen::Vector2d> list)
    {
        Eigen::Vector2d cur;
        Eigen::Vector2d last;
        double len = 0.0;
        last << list[0](0), list[0](1);
        for (size_t i = 0; i < list.size(); i++)
        {
            cur << list[i](0), list[i](1);
            len = len + (cur - last).norm();
            last = cur;
        }
        return len;
    }
    double PlannerManager::getPathLen(vector<Eigen::Vector3d> list)
    {
        Eigen::Vector3d cur;
        Eigen::Vector3d last;
        double len = 0.0;
        last << list[0](0), list[0](1), list[0](2);
        for (size_t i = 0; i < list.size(); i++)
        {
            cur << list[i](0), list[i](1), list[i](2);
            len = len + (cur - last).norm();
            last = cur;
        }
        return len;
    }


    void PlannerManager::drawBspline(NonUniformBspline& bspline, double size,
                                        const Eigen::Vector4d& color, bool show_ctrl_pts, double size2,
                                        const Eigen::Vector4d& color2, int id1, int id2) {
        if (bspline.getControlPoint().size() == 0) return;

        vector<Eigen::Vector3d> traj_pts;
        double                  tm, tmp;
        bspline.getTimeSpan(tm, tmp);

        for (double t = tm; t <= tmp; t += 0.35) {
            Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
            traj_pts.push_back(pt);
        }
        displaySphereList(traj_pts);
        // displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);

        // draw the control point
        // if (!show_ctrl_pts) return;

        // Eigen::MatrixXd         ctrl_pts = bspline.getControlPoint();
        // vector<Eigen::Vector3d> ctp;

        // for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
        //     Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
        //     ctp.push_back(pt);
        // }

        // displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
    }
    
    
    void PlannerManager::displaySphereList(const vector<Eigen::Vector3d>& list) {
 
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        for (size_t i = 0; i < list.size(); i++)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = list[i](0);
            this_pose_stamped.pose.position.y = list[i](1);
            this_pose_stamped.pose.position.z = collision_->getSliceHeight();
            this_pose_stamped.pose.orientation.x = 0.0;
            this_pose_stamped.pose.orientation.y = 0.0;
            this_pose_stamped.pose.orientation.z = 0.0;
            this_pose_stamped.pose.orientation.w = 1.0;
            this_pose_stamped.header.frame_id = "world";
            this_pose_stamped.header.stamp = ros::Time::now();
            path.poses.push_back(this_pose_stamped);
        }
        traj_pub_.publish(path);
        ros::Duration(0.001).sleep();


        // visualization_msgs::Marker mk;
        // mk.header.frame_id = "world";
        // mk.header.stamp    = ros::Time::now();
        // mk.type            = visualization_msgs::Marker::SPHERE_LIST;
        // mk.action          = visualization_msgs::Marker::DELETE;
        // mk.id              = id;
        // traj_pub_.publish(mk);

        // mk.action             = visualization_msgs::Marker::ADD;
        // mk.pose.orientation.x = 0.0;
        // mk.pose.orientation.y = 0.0;
        // mk.pose.orientation.z = 0.0;
        // mk.pose.orientation.w = 1.0;

        // mk.color.r = color(0);
        // mk.color.g = color(1);
        // mk.color.b = color(2);
        // mk.color.a = color(3);

        // mk.scale.x = resolution;
        // mk.scale.y = resolution;
        // mk.scale.z = resolution;

        // geometry_msgs::Point pt;
        // for (int i = 0; i < int(list.size()); i++) {
        //     pt.x = list[i](0);
        //     pt.y = list[i](1);
        //     pt.z = list[i](2);
        //     mk.points.push_back(pt);
        // }
        // traj_pub_.publish(mk);
        // ros::Duration(0.001).sleep();
    }

} // namespace cane_planner
