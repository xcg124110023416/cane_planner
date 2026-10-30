#include <gtest/gtest.h>
#include <plan_manager.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <atomic>
#include <thread>
#include <future>

namespace cane_planner
{
// Reuse LFPC's existing test access to observe gait time and control state.
class StaticMpcTestAccess
{
public:
    static CollisionDetection::Ptr map()
    {
        auto c=std::make_shared<CollisionDetection>();
        c->static_global_map_ready_=true; c->static_global_esdf_ready_=false;
        c->static_origin_=Eigen::Vector2d(-10,-10); c->static_size_=Eigen::Vector2i(200,200);
        c->static_map_resolution_=.1; c->static_map_inv_resolution_=10.; c->static_require_known_region_=false;
        c->static_inflated_.assign(40000,0); c->static_distance_.assign(40000,10.);
        return c;
    }
    static void narrow(CollisionDetection& c) {
        for(int x=0;x<200;++x)for(int y=0;y<200;++y) {
            const double py=-10+(y+.5)*.1;
            if(std::abs(py)>.45)c.static_inflated_[c.staticToAddress(Eigen::Vector2i(x,y))]=1;
        }
    }
    static void block(CollisionDetection& c, const Eigen::Vector2d& p) {
        const Eigen::Vector2i cell=((p-c.static_origin_)*c.static_map_inv_resolution_).array().floor().cast<int>();
        c.static_inflated_[c.staticToAddress(cell)]=1;
        ++c.map_generation_; // model a selected-map update, as init/setMap do in production
    }
    static void unblock(CollisionDetection& c, const Eigen::Vector2d& p) {
        const Eigen::Vector2i cell=((p-c.static_origin_)*c.static_map_inv_resolution_).array().floor().cast<int>();
        c.static_inflated_[c.staticToAddress(cell)]=0;
        ++c.map_generation_;
    }
    static std::vector<double> state(const LFPC& m)
    {
        return {double(m.support_leg_), m.t_sup_, m.delta_t_, m.h_, m.t_c_,
                double(m.step_num_), m.x_0_, m.vx_0_, m.y_0_, m.vy_0_, m.t_,
                m.x_t_, m.vx_t_, m.y_t_, m.vy_t_, m.al_, m.aw_, m.theta_, m.b_};
    }
};

class AstarEdgeTestAccess {
public:
    static void initMap(Astar& a) {
        a.inv_resolution_=1./a.resolution_;a.inv_time_resolution_=0.;
        a.origin_=Eigen::Vector2d(-10,-10);a.map_max_2d_=Eigen::Vector2d(10,10);
        a.path_node_pool_.resize(a.allocate_num_);for(auto& p:a.path_node_pool_)p=new Node;
        a.use_node_num_=a.iter_num_=0;
    }
};
class SimOdomTestAccess
{
public:
    static void setup(PlannerManager& m, ros::NodeHandle& nh)
    {
        nh.setParam("human_cane/enabled",false); // historical point/circle transaction fixtures
        nh.setParam("mpc/num_samples",256);nh.setParam("mpc/horizon_steps",6);
        nh.setParam("mpc/max_api",.5236);
        nh.setParam("mpc/timed_collision_enable",true);
        m.initial_route_active_ = true; // existing step fixtures model an already activated route
        m.simulation_ = true;
        m.gazebo_sim_ = false;
        m.mpc_debug_enable_ = false;
        m.transaction_diag_pub_=nh.advertise<std_msgs::Float64MultiArray>("transaction",20);
        m.lfpc_model_.reset(new LFPC);
        m.lfpc_model_->initializeModel(nh);
        m.lfpc_model_->reset(Eigen::Vector3d(0.4, -0.2, 0.7),
                             Eigen::Vector3d(5.4, 2.6, 0.0), LEFT_LEG, 5);
        m.lfpc_model_->SetCtrlParams(Eigen::Vector3d(0.25, 0.03, 0.1));
        // A real moving state, including nonzero phase and output velocity.
        m.lfpc_model_->updateOneStep();
        m.odom_pos_ = m.lfpc_model_->getCOMPos();
        m.odom_vel_ = Eigen::Vector3d(0.6, -0.4, 0.0);
        m.start_state_ = Eigen::Vector3d(m.odom_pos_.x(), m.odom_pos_.y(), 0.8);
        m.end_pt_ = Eigen::Vector2d(20.0, 20.0);
        m.mpc_sim_goal_ = Eigen::Vector3d(20.0, 20.0, 0.0);
        m.global_wp_idx_ = 0;
        m.last_com_pos_ = m.odom_pos_.head(2);
        m.last_theta_ = 0.8;
        m.mpc_stuck_steps_ = 0;
        m.mpc_step_count_ = 0;
        m.mpc_reached_goal_ = false;
        m.mpc_controller_.reset(new MpcController);
        // Missing corridor exercises the real fail-closed plan and STOP branch.
        m.mpc_controller_->init();
        m.sim_odom_pub_ = nh.advertise<nav_msgs::Odometry>("sim_odom", 20);
        m.mpc_stop_advice_pub_ = nh.advertise<std_msgs::Bool>("stop", 20);
        m.mpc_stop_reason_pub_ = nh.advertise<std_msgs::String>("reason", 20);
        m.mpc_timed_collision_pub_ = nh.advertise<visualization_msgs::MarkerArray>("timed_collision",20);
        m.mpc_timed_plan_pub_ = nh.advertise<visualization_msgs::MarkerArray>("timed_plan",20);
        m.mpc_timed_path_pub_ = nh.advertise<nav_msgs::Path>("timed_path",20);
        m.mpc_best_traj_pub_ = nh.advertise<visualization_msgs::Marker>("best", 20);
        m.mpc_fov_pub_ = nh.advertise<visualization_msgs::Marker>("fov", 20);
        m.mpc_wp_pub_ = nh.advertise<visualization_msgs::Marker>("wp", 20);
        m.mpc_wps_pub_ = nh.advertise<visualization_msgs::Marker>("wps", 20);
        m.mpc_vis_pub_ = nh.advertise<visualization_msgs::Marker>("com", 20);
        m.mpc_foot_pub_ = nh.advertise<visualization_msgs::Marker>("feet", 20);
        m.mpc_path_pub_ = nh.advertise<nav_msgs::Path>("path", 20);
        m.mpc_convex_corridor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("corridor", 20);
        m.cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    }

    static void dynamicSetup(PlannerManager& m)
    {
        m.pedestrians_enabled_=true;
        m.convex_corridor_.reset(new ConvexCorridor);
        m.collision_=StaticMpcTestAccess::map(); m.mpc_controller_->setCollision(m.collision_);
        const Eigen::Vector2d p=m.lfpc_model_->getCOMPos().head<2>();
        m.global_waypoints_={p,p+Eigen::Vector2d(1,0),p+Eigen::Vector2d(2,0)};
        m.global_wp_idx_=0;
    }
    static void recoverySetup(PlannerManager&m,ros::NodeHandle&nh)
    {
        m.lfpc_model_->reset(Eigen::Vector3d(.4,0,0),Eigen::Vector3d(0,0,0),LEFT_LEG,5);
        m.odom_pos_=m.lfpc_model_->getCOMPos();m.last_com_pos_=m.odom_pos_.head<2>();
        m.start_state_=Eigen::Vector3d::Zero();m.mpc_sim_goal_=Eigen::Vector3d(5,0,0);
        m.end_pt_=Eigen::Vector2d(5,0);m.global_waypoints_={Eigen::Vector2d(0,0),Eigen::Vector2d(1,0),Eigen::Vector2d(2,0),Eigen::Vector2d(3,0),Eigen::Vector2d(4,0),Eigen::Vector2d(5,0)};
        m.global_wp_idx_=0;m.mpc_controller_->setParam(nh);m.mpc_controller_->init();
    }
    static void astarSetup(PlannerManager&m,ros::NodeHandle&nh,bool narrow=false) {
        if(narrow)StaticMpcTestAccess::narrow(*m.collision_);
        nh.setParam("astar/resolution_astar",.1);nh.setParam("astar/lambda_heu",1.);
        nh.setParam("astar/horizon",200.);nh.setParam("astar/allocate_num",100000);
        nh.setParam("astar/w_clearance",1.);nh.setParam("astar/clearance_sigma",.8);
        m.astar_finder_.reset(new Astar);m.astar_finder_->setParam(nh);m.astar_finder_->setCollision(m.collision_);
        m.astar_finder_->setCorridorEdgeClearance(m.convex_corridor_->getConfig().clearance);AstarEdgeTestAccess::initMap(*m.astar_finder_);
        m.a_path_pub_=nh.advertise<nav_msgs::Path>("astar",20);
    }
    static void invalidateRebuild(PlannerManager&m) {auto c=m.convex_corridor_->getConfig();c.iterations=0;m.convex_corridor_->setConfig(c);}
    // Candidate acceptance tests begin with a completed real background job;
    // concurrency/cadence tests below exercise the pending state separately.
    static bool primeGlobal(PlannerManager& m) {
        {std::lock_guard<std::mutex> lock(m.dynObsMutex_);std::vector<Eigen::Vector2d> pending;
         auto pose=m.lfpc_model_->getCOMPos();pose.z()=m.lfpc_model_->getNextIterState().z();
         m.updateAndPublishConvexCorridorLocked(pose,nullptr,&pending,ros::Time::now());}
        return workerReady(m);
    }
    static void timedSetup(PlannerManager& m,ros::NodeHandle& nh) {
        fullGeometrySetup(m,nh);nh.setParam("lfpc/delta_t",.07);m.lfpc_model_->initializeModel(nh);
        m.lfpc_model_->reset(Eigen::Vector3d(.4,0,0),Eigen::Vector3d::Zero(),LEFT_LEG,0);
        m.odom_pos_=m.lfpc_model_->getCOMPos();m.odom_vel_=Eigen::Vector3d(.4,0,0);m.last_com_pos_=V2(m.odom_pos_);
        m.execution_active_=false;m.execution_phase_=0.;
        m.timed_sim_execution_=true;m.have_target_=m.have_odom_=true;m.exec_state_=PlannerManager::MPC_STEP;
        m.execution_diag_pub_=nh.advertise<std_msgs::Float64MultiArray>("execution",20);
    }
    static Eigen::Vector2d V2(const Eigen::Vector3d& v) {return v.head<2>();}
    static auto executionHull(PlannerManager& m) {return m.pedestrian_geometry_.front().hull;}
    static auto allGeometry(PlannerManager& m) {return m.pedestrian_geometry_;}
    static bool reached(PlannerManager& m) {return m.mpc_reached_goal_;}
    static bool executing(PlannerManager& m) {return m.execution_active_;}
    static double phase(PlannerManager& m) {return m.execution_phase_;}
    static bool execute(PlannerManager& m) {std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.advanceExecutionLocked(ros::Time::now());}
    static void timer(PlannerManager& m) {m.executionCallback(ros::TimerEvent());}
    static void startAt(PlannerManager& m,double yaw) {
        geometry_msgs::PoseWithCovarianceStamped::Ptr start(new geometry_msgs::PoseWithCovarianceStamped);
        start->pose.pose.orientation=tf::createQuaternionMsgFromYaw(yaw);m.startCallback(start);
    }
    static void initializePlant(PlannerManager& m) {m.mpcSimInit();m.publishSimOdom();}
    static LFPC expectedSample(PlannerManager& m,double phase) {LFPC value;m.execution_root_.sampleControlledStep(m.execution_control_,phase,value);return value;}
    static void clipExecutionCorridor(PlannerManager& m,double xmax) {
        for(auto& region:m.execution_corridor_.segments)region.halfspaces.push_back({Eigen::Vector2d(1,0),xmax});
    }
    static void clipExecutionFoot(PlannerManager& m,double xmin) {
        for(auto& region:m.execution_corridor_.segments)region.halfspaces.push_back({Eigen::Vector2d(-1,0),-xmin});
    }
    static auto executionRevision(PlannerManager& m) {return m.execution_map_revision_;}
    static void unblock(PlannerManager& m,const Eigen::Vector2d& p) {StaticMpcTestAccess::unblock(*m.collision_,p);}
    static bool executionContains(PlannerManager& m,const Eigen::Vector2d& p) {
        return std::any_of(m.execution_corridor_.segments.begin(),m.execution_corridor_.segments.end(),
            [&](const ConvexCorridor::Segment& r){return ConvexCorridor::contains(r,p,1e-6);});
    }
    static void workerHook(PlannerManager& m,std::function<void()> hook) {m.global_worker_test_hook_=std::move(hook);}
    static bool workerReady(PlannerManager& m) {
        return m.global_route_job_.valid() && m.global_route_job_.wait_for(std::chrono::seconds(2))==std::future_status::ready;
    }
    static bool pollRoute(PlannerManager& m) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);std::vector<Eigen::Vector2d> pending;
        auto pose=m.lfpc_model_->getCOMPos();pose.z()=m.lfpc_model_->getNextIterState()(2);
        return m.refreshGlobalRouteLocked(pose,pending,ros::Time::now());
    }
    static void afterSearch(PlannerManager&m,std::function<void()> hook) {m.after_global_search_test_hook_=std::move(hook);}
    static void refreshPeriod(PlannerManager&m,double period) {m.global_replan_period_=period;}
    static uint64_t attempts(PlannerManager&m) {return m.global_replan_attempts_;}
    static std::string recoveryReason(PlannerManager&m) {return m.global_replan_reason_;}
    static std::vector<Eigen::Vector2d> route(PlannerManager&m) {return m.global_waypoints_;}
    static void reference(PlannerManager& m, const std::vector<Eigen::Vector2d>& path, size_t index=0) {
        m.global_waypoints_=path;m.global_wp_idx_=index;
    }
    static size_t reanchor(PlannerManager& m) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);
        m.reanchorWaypoint(m.lfpc_model_->getCOMPos().head<2>());
        return m.global_wp_idx_;
    }
    static void block(PlannerManager& m,const Eigen::Vector2d& p) {StaticMpcTestAccess::block(*m.collision_,p);}
    static void root(PlannerManager& m,const Eigen::Vector2d& p) {
        m.lfpc_model_->reset(Eigen::Vector3d(.4,0,0),Eigen::Vector3d(p.x(),p.y(),0),LEFT_LEG,5);
    }
    static void smallCorridor(PlannerManager& m) {
        auto c=m.convex_corridor_->getConfig();c.local_radius=.3;c.max_segment_length=.3;
        m.convex_corridor_->setConfig(c);
    }
    static void adaptiveCorridor(PlannerManager& m) {
        auto c=m.convex_corridor_->getConfig();c.adaptive_sampling=true;
        m.convex_corridor_->setConfig(c);m.corridor_planning_length_=8.;
    }
    static void candidate(PlannerManager& m,const std::vector<Eigen::Vector2d>& path) {
        m.global_route_ready_.reset(new PlannerManager::GlobalRouteResult);
        auto& r=*m.global_route_ready_;r.revision=m.global_route_revision_;r.dynamic=m.pedestrians_enabled_;
        r.stamp=m.pedestrian_header_.stamp;r.receipt=m.pedestrian_receipt_;r.requested=ros::Time::now();
        r.search.path=path;r.search.status=Astar::RecoveryStatus::FOUND;r.search.seconds=.001;
    }
    static void finalStepSetup(PlannerManager&m) {
        m.have_odom_=m.have_target_=true;m.exec_state_=PlannerManager::MPC_STEP;
        m.end_pt_=Eigen::Vector2d(.30,0);m.global_wp_arrival_radius_=.2;
        m.global_waypoints_={Eigen::Vector2d(0,0),Eigen::Vector2d(.30,0)};m.global_wp_idx_=0;
    }
    static void arriveSetup(PlannerManager&m) {m.have_odom_=m.have_target_=true;m.exec_state_=PlannerManager::MPC_STEP;m.end_pt_=m.lfpc_model_->getCOMPos().head<2>();}
    static void idleSetup(PlannerManager&m,bool initialized) {m.have_odom_=initialized;m.have_target_=false;m.mpc_reached_goal_=initialized;m.exec_state_=PlannerManager::WAIT_TARGET;}
    static void initialSetup(PlannerManager&m) {m.have_odom_=m.have_target_=true;m.exec_state_=PlannerManager::GEN_NEW_TRAJ;}
    static void fsm(PlannerManager&m) {m.execFSMCallback(ros::TimerEvent());}
    static bool active(PlannerManager&m) {return m.exec_state_==PlannerManager::MPC_STEP && m.initial_route_active_;}
    static void initialHook(PlannerManager&m,std::function<void()> h) {m.after_initial_search_test_hook_=std::move(h);}
    static void goalEntry(PlannerManager&m,std::function<void()> h) {m.goal_entry_test_hook_=std::move(h);}
    static void goalB(PlannerManager&m) {geometry_msgs::PoseStamped::Ptr g(new geometry_msgs::PoseStamped);g->pose.position.x=4.;g->pose.position.y=1.;g->pose.orientation.w=1.;m.GoalCallback(g);}
    static MpcController& controller(PlannerManager&m) {return *m.mpc_controller_;}
    static void smallPedestrian(PlannerManager&m) {m.pedestrian_config_.safety_radius=.1;m.pedestrian_config_.alpha=0.;}
    static bool planValid(PlannerManager&m) {return m.mpc_controller_->lastPlanValid();}
    static void fullGeometrySetup(PlannerManager& m,ros::NodeHandle& nh) {
        dynamicSetup(m);recoverySetup(m,nh);m.pedestrians_enabled_=false;
        nh.setParam("human_cane/enabled",true);m.mpc_controller_->setParam(nh);m.mpc_controller_->init();
        auto config=m.convex_corridor_->getConfig();config.adaptive_sampling=true;m.convex_corridor_->setConfig(config);
        m.corridor_planning_length_=2.;m.corridor_preview_length_=8.;m.corridor_preview_enabled_=true;
        m.global_waypoints_={Eigen::Vector2d(0,0),Eigen::Vector2d(2,0),Eigen::Vector2d(4,0),Eigen::Vector2d(8,0)};
        m.end_pt_=Eigen::Vector2d(8,0);m.mpc_sim_goal_=Eigen::Vector3d(3,0,0);
        m.corridor_preview_pub_=nh.advertise<visualization_msgs::MarkerArray>("preview",20);
        m.human_cane_pub_=nh.advertise<visualization_msgs::MarkerArray>("human_cane",20);
    }
    static void previewSearchEpoch(PlannerManager& m,const ros::Time& epoch) {
        m.last_global_replan_=epoch;++m.corridor_preview_revision_;m.corridor_preview_period_=.2;
    }
    static bool previewReady(PlannerManager& m) {
        return m.corridor_preview_job_.valid() &&
            m.corridor_preview_job_.wait_for(std::chrono::milliseconds(500))==std::future_status::ready;
    }
    static ConvexCorridor::Result takePreview(PlannerManager& m) {return m.corridor_preview_job_.get().result;}
    static void publishCompletedPreview(PlannerManager& m) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);
        const auto result=m.corridor_preview_job_.get();m.publishCorridorPreviewLocked(&result);
    }
    static void preview(PlannerManager& m,bool valid=true,const ros::Time& schedule_epoch=ros::Time()) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);
        auto pose=m.lfpc_model_->getCOMPos();pose.z()=m.lfpc_model_->getNextIterState()(2);
        m.updateCorridorPreviewLocked(pose,valid,schedule_epoch);
    }
    static auto humanShape(PlannerManager& m) {
        return m.mpc_controller_->robotFootprintAt(m.lfpc_model_->getCOMPos().head<2>(),m.lfpc_model_->getNextIterState()(2));
    }
    static void receive(PlannerManager& m,const onboard_detector::DynamicObstacles::Ptr& msg) {m.dynamicObstaclesCallback(msg);}
    static ConvexCorridor::Result corridor(PlannerManager&m) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);
        return m.updateAndPublishConvexCorridorLocked(m.lfpc_model_->getCOMPos());
    }
    static void callbackEntry(PlannerManager&m,std::function<void()> hook) {m.pedestrian_callback_entry_test_hook_=std::move(hook);}
    static ros::Time receipt(PlannerManager&m) {std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.pedestrian_receipt_;}
    static void enabled(PlannerManager&m,bool enabled) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);
        ++m.pedestrian_generation_;m.pedestrians_enabled_=enabled;
    }
    static void afterMppi(PlannerManager&m,std::function<void()> hook) {m.after_mppi_test_hook_=std::move(hook);}
    static void duringGeometry(PlannerManager&m,std::function<void()> hook) {m.geometry_locked_test_hook_=std::move(hook);}
    static void goalReset(PlannerManager&m) {
        m.have_odom_=false; // no FSM transition; exercise both real goal callbacks
        geometry_msgs::PoseStamped::Ptr goal(new geometry_msgs::PoseStamped);
        goal->pose.position.x=10.;goal->pose.orientation.w=1.;
        m.duplicate_goal_time_window_=0.;
        m.GoalCallback(goal);
        nav_msgs::Path::Ptr path(new nav_msgs::Path);
        goal->pose.position.x=11.;path->poses.push_back(*goal);
        m.waypointCallback(path);
    }
    static std::string reason(PlannerManager&m) {std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.pedestrian_data_reason_;}
    static std::vector<PedestrianPolygon::Observation> observations(PlannerManager&m) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.pedestrian_observations_;
    }
    static void transform(PlannerManager&m,const tf::StampedTransform&t) {ASSERT_TRUE(m.tf_listener_.setTransform(t,"test"));}
    static bool geometryEmpty(PlannerManager&m) {std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.pedestrian_geometry_.empty();}
    static bool step(PlannerManager& m) { return m.mpcSimStep(); }
    static void publishMoving(PlannerManager& m) { m.publishSimOdom(); }
    static LFPC& model(PlannerManager& m) { return *m.lfpc_model_; }
    static void mode(PlannerManager& m, bool simulation, bool gazebo)
    {
        m.simulation_ = simulation;
        m.gazebo_sim_ = gazebo;
    }
    static void expectHeld(const PlannerManager& m, const Eigen::Vector3d& pos)
    {
        EXPECT_EQ(pos, m.odom_pos_);
        EXPECT_EQ(Eigen::Vector3d(0.6, -0.4, 0.0), m.odom_vel_);
        EXPECT_EQ(Eigen::Vector3d(pos.x(), pos.y(), 0.8), m.start_state_);
        EXPECT_DOUBLE_EQ(0.8, m.last_theta_);
        EXPECT_TRUE(m.mpc_step_path_.empty());
        EXPECT_FALSE(m.mpc_controller_->lastPlanValid());
        EXPECT_EQ(0, m.mpc_stuck_steps_);
    }
};

class SimOdomTest : public ::testing::Test
{
protected:
    ros::NodeHandle nh_{"~"};
    PlannerManager manager_{3};
    std::vector<nav_msgs::Odometry> odom_;
    std::vector<bool> stops_;
    std::vector<int> best_actions_;
    ros::Subscriber odom_sub_, stop_sub_, best_sub_;

    void SetUp() override
    {
        SimOdomTestAccess::setup(manager_, nh_);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("sim_odom", 20,
            [this](const nav_msgs::Odometry::ConstPtr& msg) { odom_.push_back(*msg); });
        stop_sub_ = nh_.subscribe<std_msgs::Bool>("stop", 20,
            [this](const std_msgs::Bool::ConstPtr& msg) { stops_.push_back(msg->data); });
        best_sub_ = nh_.subscribe<visualization_msgs::Marker>("best", 20,
            [this](const visualization_msgs::Marker::ConstPtr& msg) {
                best_actions_.push_back(msg->action);
            });
        const auto deadline = ros::WallTime::now() + ros::WallDuration(3.0);
        while ((odom_sub_.getNumPublishers() == 0 || stop_sub_.getNumPublishers() == 0 ||
                best_sub_.getNumPublishers() == 0) && ros::WallTime::now() < deadline)
        {
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
        ASSERT_EQ(1u, odom_sub_.getNumPublishers());
        ASSERT_EQ(1u, stop_sub_.getNumPublishers());
        ASSERT_EQ(1u, best_sub_.getNumPublishers());
    }

    // Force callback entry during real MPPI's transaction, but never join a
    // callback while holding that transaction. The pre-lock entry barrier proves
    // actual contention; timeout must fail on the old geometry/MPPI lock gap.
    void queuedFrameStep(const onboard_detector::DynamicObstacles::Ptr& msg, bool expire=false)
    {
        std::promise<void> entered;
        auto entry=entered.get_future();
        std::future<void> receiving;
        std::atomic<bool> done{false};std::thread clock;
        SimOdomTestAccess::callbackEntry(manager_,[&]{entered.set_value();});
        SimOdomTestAccess::afterMppi(manager_,[&] {
            EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
            receiving=std::async(std::launch::async,[&]{SimOdomTestAccess::receive(manager_,msg);});
            EXPECT_EQ(std::future_status::ready,entry.wait_for(std::chrono::seconds(2)));
            EXPECT_EQ(std::future_status::timeout,receiving.wait_for(std::chrono::milliseconds(20)));
            if(expire)ros::Time::setNow(ros::Time::now()+ros::Duration(1.));
            else {
                // Each transaction needs a distinct simulated tick: the plant
                // deliberately coalesces odometry at an identical timestamp.
                // Do not rely on the clock thread winning its scheduling race.
                const double epoch=ros::Time::now().toSec()+.001;
                ros::Time::setNow(ros::Time(epoch));
                clock=std::thread([&,epoch]{double t=epoch;while(!done){t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
            }
        });
        EXPECT_FALSE(SimOdomTestAccess::step(manager_));
        done=true;if(clock.joinable())clock.join();
        receiving.get(); // now outside execution lock, genuinely accepted after commit
        SimOdomTestAccess::afterMppi(manager_,{});SimOdomTestAccess::callbackEntry(manager_,{});
    }
    void waitForDecision(size_t count,bool stop)
    {
        const auto end=ros::WallTime::now()+ros::WallDuration(2.);
        while(stops_.size()<count && ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.005).sleep();}
        ASSERT_EQ(count,stops_.size());EXPECT_EQ(stop,stops_.back());
        waitForOdom(count);
    }
    void waitForStops(size_t count)
    {
        const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
        while ((stops_.size() < count || best_actions_.size() < count) &&
               ros::WallTime::now() < deadline)
        {
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
        ASSERT_EQ(count, stops_.size());
        ASSERT_EQ(count, best_actions_.size());
        EXPECT_TRUE(stops_.back());
        EXPECT_EQ(visualization_msgs::Marker::DELETE, best_actions_.back());
    }

    void waitForOdom(size_t count)
    {
        const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
        while (odom_.size() < count && ros::WallTime::now() < deadline)
        {
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
        ASSERT_EQ(count, odom_.size());
    }
};

TEST_F(SimOdomTest, PreviewCadenceStaysAlignedDespiteDifferentGlobalSearchDurations) {
    ros::Time::setNow(ros::Time(640.08));SimOdomTestAccess::fullGeometrySetup(manager_,nh_);
    std::vector<visualization_msgs::MarkerArray> previews;
    auto sub=nh_.subscribe<visualization_msgs::MarkerArray>("preview",20,
        [&](const visualization_msgs::MarkerArray::ConstPtr& msg){previews.push_back(*msg);});
    const auto end=ros::WallTime::now()+ros::WallDuration(2.);
    while(sub.getNumPublishers()==0 && ros::WallTime::now()<end) {ros::spinOnce();ros::WallDuration(.005).sleep();}
    ASSERT_EQ(1u,sub.getNumPublishers());
    SimOdomTestAccess::previewSearchEpoch(manager_,ros::Time(640.0));
    SimOdomTestAccess::preview(manager_,true,ros::Time(640.0));ASSERT_TRUE(SimOdomTestAccess::previewReady(manager_));
    // Non-refresh MPPI crossed the .2 deadline, but its schedule epoch was .1.
    ros::Time::setNow(ros::Time(640.23));SimOdomTestAccess::preview(manager_,true,ros::Time(640.1));
    ros::WallDuration(.03).sleep();ros::spinOnce();ASSERT_EQ(1u,previews.size());
    ASSERT_GT(previews.back().markers.size(),1u);
    // First search took 80ms; the next one took 30ms. Preview is due on the
    // common .2s phase, rather than .2s after the previous job's late launch.
    ros::Time::setNow(ros::Time(640.25));
    SimOdomTestAccess::previewSearchEpoch(manager_,ros::Time(640.2));
    SimOdomTestAccess::preview(manager_,true,ros::Time(640.2));ASSERT_TRUE(SimOdomTestAccess::previewReady(manager_));
    ros::Time::setNow(ros::Time(640.31));SimOdomTestAccess::preview(manager_,true,ros::Time(640.3));
    ros::WallDuration(.03).sleep();ros::spinOnce();ASSERT_EQ(2u,previews.size());
    EXPECT_GT(previews.back().markers.size(),1u);
}
TEST_F(SimOdomTest, DistantPreviewFailureCannotStopOrReplaceActivePlan) {
    ros::Time::setNow(ros::Time(100.));SimOdomTestAccess::fullGeometrySetup(manager_,nh_);
    SimOdomTestAccess::block(manager_,Eigen::Vector2d(6,0));
    std::vector<visualization_msgs::MarkerArray> previews,humans;
    auto preview=nh_.subscribe<visualization_msgs::MarkerArray>("preview",20,[&](const visualization_msgs::MarkerArray::ConstPtr& m){previews.push_back(*m);});
    auto human=nh_.subscribe<visualization_msgs::MarkerArray>("human_cane",20,[&](const visualization_msgs::MarkerArray::ConstPtr& m){humans.push_back(*m);});
    const auto connected=ros::WallTime::now()+ros::WallDuration(2.);
    while((preview.getNumPublishers()==0 || human.getNumPublishers()==0) && ros::WallTime::now()<connected) {ros::spinOnce();ros::WallDuration(.005).sleep();}
    const auto route=SimOdomTestAccess::route(manager_);
    std::atomic<bool> done{false};
    std::thread clock([&]{double t=100.;while(!done){t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
    const bool finished=SimOdomTestAccess::step(manager_);done=true;clock.join();
    EXPECT_FALSE(finished);waitForDecision(1,false);ASSERT_TRUE(SimOdomTestAccess::planValid(manager_));
    const auto held=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    const auto end=ros::WallTime::now()+ros::WallDuration(2.);bool blocked=false;
    while(!blocked && ros::WallTime::now()<end) {
        SimOdomTestAccess::preview(manager_);ros::spinOnce();ros::WallDuration(.005).sleep();
        for(const auto& frame:previews)for(const auto& marker:frame.markers)
            blocked=blocked || marker.text.find("Preview blocked")!=std::string::npos;
    }
    EXPECT_TRUE(blocked);EXPECT_EQ(route,SimOdomTestAccess::route(manager_));
    EXPECT_EQ(held,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    ASSERT_FALSE(humans.empty());const auto expected=SimOdomTestAccess::humanShape(manager_);bool found=false;
    for(const auto& marker:humans.back().markers)if(marker.ns=="human_cane_current") {
        found=true;ASSERT_EQ(expected.size()+1,marker.points.size());
        for(size_t i=0;i<expected.size();++i){EXPECT_DOUBLE_EQ(expected[i].x(),marker.points[i].x);EXPECT_DOUBLE_EQ(expected[i].y(),marker.points[i].y);}
    }
    EXPECT_TRUE(found);
    // A malformed callback must clear the preview immediately, before the next
    // planning cycle, and invalidate any outstanding copied-input result.
    SimOdomTestAccess::enabled(manager_,true);
    onboard_detector::DynamicObstacles::Ptr malformed(new onboard_detector::DynamicObstacles);
    malformed->header.frame_id="world";malformed->header.stamp=ros::Time::now();malformed->num=2;
    SimOdomTestAccess::receive(manager_,malformed);ros::WallDuration(.03).sleep();ros::spinOnce();
    ASSERT_FALSE(previews.empty());ASSERT_EQ(1u,previews.back().markers.size());
    EXPECT_EQ(visualization_msgs::Marker::DELETEALL,previews.back().markers.front().action);
}

TEST_F(SimOdomTest, RepeatedRealStopPublishesFrozenPoseAndZeroTwist)
{
    LFPC& model = SimOdomTestAccess::model(manager_);
    const auto state = StaticMpcTestAccess::state(model);
    const auto pos = model.getCOMPos();
    const auto foot = model.getFootPosition();
    const auto path = model.getStepCOMPath();
    const auto velocity = model.getNextIterState();
    ASSERT_GT(velocity.head(2).norm(), 0.01);
    ASSERT_GT(state[10], 0.0);  // nonzero LFPC time detects accidental phase resets

    for (size_t i = 0; i < 12; ++i)
    {
        ros::Time::setNow(ros::Time(100.0 + 0.1 * i));
        EXPECT_FALSE(SimOdomTestAccess::step(manager_));
        waitForStops(i + 1);
        waitForOdom(i + 1);
        ASSERT_EQ(i + 1, odom_.size());
        const auto& msg = odom_.back();
        EXPECT_EQ(ros::Time::now(), msg.header.stamp);
        if (i)
        {
            EXPECT_GT(msg.header.stamp, odom_[i - 1].header.stamp);
        }
        EXPECT_EQ("world", msg.header.frame_id);
        EXPECT_TRUE(msg.child_frame_id.empty());
        EXPECT_DOUBLE_EQ(pos.x(), msg.pose.pose.position.x);
        EXPECT_DOUBLE_EQ(pos.y(), msg.pose.pose.position.y);
        EXPECT_DOUBLE_EQ(0.0, msg.pose.pose.position.z);
        EXPECT_EQ(tf::createQuaternionMsgFromYaw(velocity.z()), msg.pose.pose.orientation);
        EXPECT_EQ(geometry_msgs::Twist(), msg.twist.twist);
        EXPECT_EQ(state, StaticMpcTestAccess::state(model));
        EXPECT_EQ(pos, model.getCOMPos());
        EXPECT_EQ(foot, model.getFootPosition());
        EXPECT_EQ(path, model.getStepCOMPath());
        SimOdomTestAccess::expectHeld(manager_, pos);
    }
    // Default/active serialization must still expose the original moving values.
    ros::Time::setNow(ros::Time(102.0));
    SimOdomTestAccess::publishMoving(manager_);
    waitForOdom(13);
    ASSERT_EQ(13u, odom_.size());
    EXPECT_DOUBLE_EQ(velocity.x(), odom_.back().twist.twist.linear.x);
    EXPECT_DOUBLE_EQ(velocity.y(), odom_.back().twist.twist.linear.y);
    EXPECT_EQ(odom_.front().pose, odom_.back().pose);
    EXPECT_EQ(state, StaticMpcTestAccess::state(model));
}

TEST_F(SimOdomTest, HardwareAndGazeboStopsDoNotPublishSimHeartbeat)
{
    size_t count = 0;
    for (const auto& mode : {std::make_pair(false, false), std::make_pair(true, true),
                             std::make_pair(false, true)})
    {
        SimOdomTestAccess::mode(manager_, mode.first, mode.second);
        ros::Time::setNow(ros::Time(110.0 + count));
        EXPECT_FALSE(SimOdomTestAccess::step(manager_));
        waitForStops(++count);
        const auto deadline = ros::WallTime::now() + ros::WallDuration(0.1);
        while (ros::WallTime::now() < deadline)
        {
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
        EXPECT_TRUE(odom_.empty());
    }
}
onboard_detector::DynamicObstacles::Ptr frame(const ros::Time& stamp, const std::string& name="world")
{
    onboard_detector::DynamicObstacles::Ptr msg(new onboard_detector::DynamicObstacles);
    msg->header.stamp=stamp;msg->header.frame_id=name;msg->num=0;return msg;
}
void addPedestrian(onboard_detector::DynamicObstacles& msg,const Eigen::Vector2d& p)
{
    geometry_msgs::Vector3 pos,vel,size;pos.x=p.x();pos.y=p.y();vel.x=1.;size.x=.4;size.y=.6;size.z=1.7;
    msg.position.push_back(pos);msg.velocity.push_back(vel);msg.size.push_back(size);msg.num=msg.position.size();
}
static void expectPedestrianExcluded(const ConvexCorridor::Result& corridor,
                                    const ConvexCorridor::Polygon& hull)
{
    ASSERT_TRUE(corridor.feasible)<<ConvexCorridor::failureReasonName(corridor.failure_reason);
    EXPECT_TRUE(corridor.has_dynamic_input);
    for(const auto& region:corridor.segments) {
        bool separated=false;
        for(const auto& plane:region.halfspaces) {
            double gap=std::numeric_limits<double>::infinity();
            for(const auto& p:hull)gap=std::min(gap,plane.normal.dot(p)-plane.offset);
            separated=separated || gap>=.002-1e-7;
        }
        EXPECT_TRUE(separated)<<"A displayed/active region includes the current pedestrian hull";
    }
}
TEST_F(SimOdomTest, FivePeopleParticipateAndInvalidOrEmptyFramesClearAllMarkers)
{
    ros::Time::setNow(ros::Time(740));SimOdomTestAccess::fullGeometrySetup(manager_,nh_);
    SimOdomTestAccess::enabled(manager_,true);
    std::vector<visualization_msgs::MarkerArray> markers;
    auto sub=nh_.subscribe<visualization_msgs::MarkerArray>("corridor",20,
        [&](const visualization_msgs::MarkerArray::ConstPtr& m){markers.push_back(*m);});
    const auto end=ros::WallTime::now()+ros::WallDuration(2.);
    while(sub.getNumPublishers()==0 && ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.005).sleep();}
    ASSERT_EQ(1u,sub.getNumPublishers());
    auto msg=frame(ros::Time::now());
    for(int i=0;i<5;++i){addPedestrian(*msg,Eigen::Vector2d(.3+.7*i,2.));msg->velocity.back().x=0.;}
    SimOdomTestAccess::receive(manager_,msg);
    ASSERT_EQ(5u,SimOdomTestAccess::observations(manager_).size());
    const auto corridor=SimOdomTestAccess::corridor(manager_);
    const auto geometry=SimOdomTestAccess::allGeometry(manager_);ASSERT_EQ(5u,geometry.size());
    for(const auto& g:geometry)expectPedestrianExcluded(corridor,g.hull);
    // The fifth person is independently capable of blocking the reference.
    msg->position.back().x=1.5;msg->position.back().y=0.;SimOdomTestAccess::receive(manager_,msg);
    const auto blocked=SimOdomTestAccess::corridor(manager_);
    EXPECT_TRUE(blocked.failure_reason==ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED ||
                blocked.truncation_reason==ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED);
    addPedestrian(*msg,Eigen::Vector2d(4,2));SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);EXPECT_TRUE(SimOdomTestAccess::geometryEmpty(manager_));
    ros::WallDuration(.03).sleep();ros::spinOnce();
    bool fifth_deleted=false;
    for(const auto& batch:markers)for(const auto& m:batch.markers)
        fifth_deleted=fifth_deleted || (m.ns=="mpc_pedestrian_peer_force" && m.id==4 && m.action==visualization_msgs::Marker::DELETE);
    EXPECT_TRUE(fifth_deleted);
}

TEST_F(SimOdomTest, FifthNewPedestrianStopsQueuedStepAndFreshEmptyResumes)
{
    ros::Time::setNow(ros::Time(741));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::enabled(manager_,true);SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    const auto initial=SimOdomTestAccess::model(manager_).getCOMPos();
    auto msg=frame(ros::Time::now());
    for(int i=0;i<4;++i)addPedestrian(*msg,Eigen::Vector2d(4+i,4));
    addPedestrian(*msg,Eigen::Vector2d(.5,0));
    for(auto& v:msg->velocity)v=geometry_msgs::Vector3();
    SimOdomTestAccess::receive(manager_,msg);ros::Time::setNow(ros::Time(741.02));
    EXPECT_FALSE(SimOdomTestAccess::execute(manager_));
    EXPECT_EQ(initial,SimOdomTestAccess::model(manager_).getCOMPos());EXPECT_DOUBLE_EQ(0.,SimOdomTestAccess::phase(manager_));
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));ros::Time::setNow(ros::Time(741.04));
    EXPECT_TRUE(SimOdomTestAccess::execute(manager_));EXPECT_NE(initial,SimOdomTestAccess::model(manager_).getCOMPos());
    nh_.deleteParam("lfpc/delta_t"); // timedSetup's slower model must not leak into other fixtures
}
TEST_F(SimOdomTest, TimedChecksStillExcludeCurrentPedestrianFromActiveAndPreviewCorridors)
{
    ros::Time::setNow(ros::Time(490));SimOdomTestAccess::fullGeometrySetup(manager_,nh_);
    SimOdomTestAccess::enabled(manager_,true);
    auto msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(1.5,1.2));
    msg->velocity[0].x=.2;SimOdomTestAccess::receive(manager_,msg);
    const auto active=SimOdomTestAccess::corridor(manager_);
    const auto hull=SimOdomTestAccess::executionHull(manager_);
    expectPedestrianExcluded(active,hull);
    SimOdomTestAccess::preview(manager_);ASSERT_TRUE(SimOdomTestAccess::previewReady(manager_));
    expectPedestrianExcluded(SimOdomTestAccess::takePreview(manager_),hull);
}
TEST_F(SimOdomTest, SpatialHullUsesCurrentEpochWhileFuturePredictionRemainsSeparate)
{
    ros::Time::setNow(ros::Time(491.2));SimOdomTestAccess::fullGeometrySetup(manager_,nh_);
    SimOdomTestAccess::enabled(manager_,true);
    auto msg=frame(ros::Time(491));addPedestrian(*msg,Eigen::Vector2d(1.5,1.2));
    msg->velocity[0].x=0.;msg->velocity[0].y=-.5;SimOdomTestAccess::receive(manager_,msg);
    std::vector<visualization_msgs::MarkerArray> visuals;
    auto sub=nh_.subscribe<visualization_msgs::MarkerArray>("corridor",20,
        [&](const visualization_msgs::MarkerArray::ConstPtr& m){visuals.push_back(*m);});
    ros::WallDuration(.03).sleep();
    const auto corridor=SimOdomTestAccess::corridor(manager_);
    auto hull=SimOdomTestAccess::executionHull(manager_);
    for(auto& p:hull)p+=Eigen::Vector2d(0,-.1);
    expectPedestrianExcluded(corridor,hull);
    // The pedestrian will later reach the reference; that future hull must not
    // be subtracted from this current spatial corridor.
    EXPECT_TRUE(std::any_of(corridor.segments.begin(),corridor.segments.end(),
        [&](const ConvexCorridor::Segment& r){return ConvexCorridor::contains(r,Eigen::Vector2d(1.5,0));}));
    ros::WallDuration(.03).sleep();ros::spinOnce();ASSERT_FALSE(visuals.empty());
    bool found=false;
    for(const auto& marker:visuals.back().markers)if(marker.ns=="mpc_pedestrian_used_hull") {
        found=true;ASSERT_FALSE(marker.points.empty());
        EXPECT_NEAR(hull.front().x(),marker.points.front().x,1e-9);
        EXPECT_NEAR(hull.front().y(),marker.points.front().y,1e-9);
    }
    EXPECT_TRUE(found);
}
TEST_F(SimOdomTest, PreviewCannotDisplayOldFreeSpaceAfterPedestrianMovesInside)
{
    ros::Time::setNow(ros::Time(492));SimOdomTestAccess::fullGeometrySetup(manager_,nh_);
    SimOdomTestAccess::enabled(manager_,true);
    auto msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(1.5,1.2));
    msg->velocity[0].x=0.;SimOdomTestAccess::receive(manager_,msg);
    ASSERT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
    SimOdomTestAccess::preview(manager_);ASSERT_TRUE(SimOdomTestAccess::previewReady(manager_));
    msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(1.5,0));msg->velocity[0].x=0.;
    SimOdomTestAccess::receive(manager_,msg);
    const auto current=SimOdomTestAccess::corridor(manager_);
    ASSERT_TRUE(current.feasible);EXPECT_FALSE(current.complete);
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED,current.truncation_reason);
    expectPedestrianExcluded(current,SimOdomTestAccess::executionHull(manager_));
    std::vector<visualization_msgs::MarkerArray> visuals;
    auto sub=nh_.subscribe<visualization_msgs::MarkerArray>("preview",20,
        [&](const visualization_msgs::MarkerArray::ConstPtr& m){visuals.push_back(*m);});
    ros::WallDuration(.03).sleep();SimOdomTestAccess::publishCompletedPreview(manager_);
    ros::WallDuration(.03).sleep();ros::spinOnce();ASSERT_FALSE(visuals.empty());
    ASSERT_EQ(1u,visuals.back().markers.size());EXPECT_EQ(visualization_msgs::Marker::DELETEALL,visuals.back().markers.front().action);
}
TEST_F(SimOdomTest, TimedSnapshotUsesSourceAgeAndPublishesMatchedGeometry) {
    ros::Time::setNow(ros::Time(500));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto msg=frame(ros::Time(499.9));addPedestrian(*msg,Eigen::Vector2d(1,2));
    msg->velocity[0].x=.2;SimOdomTestAccess::receive(manager_,msg);
    std::vector<visualization_msgs::MarkerArray> visual;
    std::vector<nav_msgs::Path> paths;
    auto vs=nh_.subscribe<visualization_msgs::MarkerArray>("timed_collision",20,
        [&](const visualization_msgs::MarkerArray::ConstPtr& m){visual.push_back(*m);});
    auto ps=nh_.subscribe<nav_msgs::Path>("timed_path",20,
        [&](const nav_msgs::Path::ConstPtr& m){paths.push_back(*m);});
    const auto deadline=ros::WallTime::now()+ros::WallDuration(2.);
    while((!vs.getNumPublishers() || !ps.getNumPublishers()) && ros::WallTime::now()<deadline) {
        ros::spinOnce();ros::WallDuration(.005).sleep();
    }
    std::atomic<bool> done{false};std::thread clock;
    SimOdomTestAccess::afterMppi(manager_,[&]{
        auto& c=SimOdomTestAccess::controller(manager_);
        EXPECT_TRUE(c.lastPlanValid());EXPECT_NEAR(.1,c.timedObservationAge(),1e-9);
        ASSERT_EQ(1u,c.timedCollision().obstacles().size());
        EXPECT_EQ(Eigen::Vector2d(.2,0),c.timedCollision().obstacles()[0].velocity);
        clock=std::thread([&]{double t=500.;while(!done){t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
    });
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));done=true;if(clock.joinable())clock.join();
    SimOdomTestAccess::afterMppi(manager_,{});waitForDecision(1,false);
    ASSERT_FALSE(visual.empty());ASSERT_FALSE(paths.empty());ASSERT_FALSE(paths.back().poses.empty());
    const auto& c=SimOdomTestAccess::controller(manager_);
    EXPECT_TRUE(c.getDebugMetrics().timed_execution_checked);EXPECT_TRUE(c.getDebugMetrics().timed_execution_safe);
    ASSERT_EQ(c.getBestTimes().size(),paths.back().poses.size());
    for(size_t i=0;i<c.getBestTimes().size();++i) {
        EXPECT_NEAR(c.getBestTimes()[i],(paths.back().poses[i].header.stamp-paths.back().header.stamp).toSec(),1e-8);
        EXPECT_DOUBLE_EQ(c.getBestPath()[i].z(),paths.back().poses[i].pose.position.z);
    }
    bool robot=false,pedestrian=false,label=false;
    for(const auto& m:visual.back().markers) {
        if(m.action!=visualization_msgs::Marker::ADD)continue;
        if(m.ns=="timed_robot")robot=true;
        if(m.ns=="timed_labels")label=true;
        if(m.ns=="timed_pedestrian" && m.id==0) {
            pedestrian=true;const auto& obstacle=c.timedCollision().obstacles()[0];
            const auto expected=(obstacle.hull[0]+obstacle.velocity*(c.timedObservationAge()+c.getBestTimes()[0])).eval();
            ASSERT_FALSE(m.points.empty());EXPECT_NEAR(expected.x(),m.points[0].x,1e-8);EXPECT_NEAR(expected.y(),m.points[0].y,1e-8);
        }
    }
    EXPECT_TRUE(robot);EXPECT_TRUE(pedestrian);EXPECT_TRUE(label);
    // Accepting fresh empty input clears both prediction and all visible marker IDs.
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    ros::WallDuration(.05).sleep();ros::spinOnce();
    EXPECT_TRUE(c.timedCollision().empty());ASSERT_FALSE(visual.back().markers.empty());
    EXPECT_EQ(visualization_msgs::Marker::DELETEALL,visual.back().markers[0].action);
    EXPECT_EQ(1u,visual.back().markers.size());EXPECT_TRUE(paths.back().poses.empty());
}

TEST_F(SimOdomTest, CrossingUsesCurrentSpatialDetourDespiteFutureVacatedSpace) {
    ros::Time::setNow(ros::Time(505));SimOdomTestAccess::fullGeometrySetup(manager_,nh_);
    SimOdomTestAccess::enabled(manager_,true);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(1.8,0));
    msg->velocity[0].x=0.;msg->velocity[0].y=2.;
    SimOdomTestAccess::receive(manager_,msg);
    const auto prefix=SimOdomTestAccess::corridor(manager_);
    ASSERT_TRUE(prefix.feasible);EXPECT_FALSE(prefix.complete);
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED,prefix.truncation_reason);
    expectPedestrianExcluded(prefix,SimOdomTestAccess::executionHull(manager_));
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto route=SimOdomTestAccess::route(manager_);
    const auto before=SimOdomTestAccess::model(manager_).getCOMPos();
    queuedFrameStep(msg);waitForDecision(1,false);
    EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-before).norm(),1e-6);
    EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    EXPECT_EQ(route.back(),SimOdomTestAccess::route(manager_).back());
    EXPECT_NE(route,SimOdomTestAccess::route(manager_));
    EXPECT_TRUE(SimOdomTestAccess::controller(manager_).getDebugMetrics().timed_execution_safe);
}

TEST_F(SimOdomTest, TimedExecutionCollisionStopsBeforeAnyIntegration) {
    ros::Time::setNow(ros::Time(510));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::smallPedestrian(manager_);
    auto msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(0,1.5));
    msg->size[0].x=msg->size[0].y=.1;msg->velocity[0].x=0.;msg->velocity[0].y=-10.;
    SimOdomTestAccess::receive(manager_,msg);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    const auto pos=SimOdomTestAccess::model(manager_).getCOMPos();
    SimOdomTestAccess::afterMppi(manager_,[&]{
        EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
        // Still within the existing 0.5-second source/receipt freshness limit.
        ros::Time::setNow(ros::Time(510.15));
    });
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));SimOdomTestAccess::afterMppi(manager_,{});
    waitForStops(1);waitForOdom(1);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(pos,SimOdomTestAccess::model(manager_).getCOMPos());
    EXPECT_EQ(geometry_msgs::Twist(),odom_.back().twist.twist);
    const auto dbg=SimOdomTestAccess::controller(manager_).getDebugMetrics();
    EXPECT_TRUE(dbg.timed_execution_checked);EXPECT_FALSE(dbg.timed_execution_safe);
    EXPECT_FALSE(SimOdomTestAccess::planValid(manager_));
}

TEST_F(SimOdomTest, DynamicValidityTransitionsAndFrozenStop)
{
    ros::Time::setNow(ros::Time(200));SimOdomTestAccess::dynamicSetup(manager_);
    using R=ConvexCorridor::FailureReason;
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForStops(1);waitForOdom(1);
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(geometry_msgs::Twist(),odom_.back().twist.twist);
    auto msg=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,msg);
    EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible); // empty != unavailable
    ros::Time::setNow(ros::Time(201));
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    msg=frame(ros::Time::now());msg->num=1;SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    msg=frame(ros::Time(202));SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    msg=frame(ros::Time::now(),"no_such_frame");SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(2,2));msg->size[0].x=NAN;
    SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    EXPECT_TRUE(SimOdomTestAccess::geometryEmpty(manager_));
    SimOdomTestAccess::enabled(manager_,false);EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
}
TEST_F(SimOdomTest, DynamicRootCollisionInvalidatesCurrentSpatialCorridor)
{
    ros::Time::setNow(ros::Time(210));SimOdomTestAccess::dynamicSetup(manager_);
    const auto p=SimOdomTestAccess::model(manager_).getCOMPos().head<2>().eval();
    auto msg=frame(ros::Time::now());addPedestrian(*msg,p);
    SimOdomTestAccess::receive(manager_,msg);
    auto blocked=SimOdomTestAccess::corridor(manager_);
    EXPECT_FALSE(blocked.feasible);EXPECT_TRUE(blocked.segments.empty());
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED,blocked.failure_reason);
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForStops(1);waitForOdom(1);
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    ros::Time::setNow(ros::Time(210.1));msg=frame(ros::Time::now());
    addPedestrian(*msg,p+Eigen::Vector2d(1,2));SimOdomTestAccess::receive(manager_,msg);
    const auto clear=SimOdomTestAccess::corridor(manager_);EXPECT_TRUE(clear.feasible)<<ConvexCorridor::failureReasonName(clear.failure_reason);
    msg=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,msg);
    EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
    EXPECT_TRUE(SimOdomTestAccess::geometryEmpty(manager_));
}
TEST_F(SimOdomTest, TimedAblationRetainsCurrentHullCorridorExclusion)
{
    ros::Time::setNow(ros::Time(225));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("mpc/timed_collision_enable",false);SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(1.3,0));
    SimOdomTestAccess::receive(manager_,msg);
    const auto corridor=SimOdomTestAccess::corridor(manager_);
    EXPECT_FALSE(corridor.feasible);EXPECT_TRUE(corridor.has_dynamic_input);
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED,corridor.failure_reason);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::step(manager_);waitForDecision(1,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(0u,SimOdomTestAccess::attempts(manager_));
}
TEST_F(SimOdomTest, FreshEmptyAfterDynamicBlockResumesActualIntegration)
{
    ros::Time::setNow(ros::Time(230));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(0,0));
    SimOdomTestAccess::receive(manager_,msg);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForStops(1);waitForOdom(1);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    msg=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,msg);
    // Production visualization uses a ROS-duration sleep; advance only the test
    // clock while exercising the actual successful manager branch.
    std::atomic<bool> done{false};
    std::thread clock([&] {double t=230.;while(!done) {t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
    const bool finished=SimOdomTestAccess::step(manager_);
    done=true;clock.join();
    EXPECT_FALSE(finished);EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
    EXPECT_NE(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    waitForOdom(2);
    // Existing prepareNextStep resets serialized instantaneous twist to zero;
    // actual recovery is certified by successful plan and integrated displacement.
    EXPECT_GT(std::hypot(odom_.back().pose.pose.position.x-odom_.front().pose.pose.position.x,
                         odom_.back().pose.pose.position.y-odom_.front().pose.pose.position.y),1e-6);
}
TEST_F(SimOdomTest, DynamicTransformsPositionVelocityAndCompleteBox)
{
    ros::Time::setNow(ros::Time(220));SimOdomTestAccess::dynamicSetup(manager_);
    tf::Transform t;t.setOrigin(tf::Vector3(3,4,0));t.setRotation(tf::createQuaternionFromYaw(1.5707963267948966));
    SimOdomTestAccess::transform(manager_,tf::StampedTransform(t,ros::Time(220),"world","ped_sensor"));
    auto msg=frame(ros::Time::now(),"ped_sensor");addPedestrian(*msg,Eigen::Vector2d(1,2));
    SimOdomTestAccess::receive(manager_,msg);
    const auto& obs=SimOdomTestAccess::observations(manager_);ASSERT_EQ(1u,obs.size());
    EXPECT_NEAR(1,obs[0].position.x(),1e-9);EXPECT_NEAR(5,obs[0].position.y(),1e-9);
    EXPECT_NEAR(0,obs[0].velocity.x(),1e-9);EXPECT_NEAR(1,obs[0].velocity.y(),1e-9);
    EXPECT_NEAR(.6,obs[0].full_size.x(),1e-9);EXPECT_NEAR(.4,obs[0].full_size.y(),1e-9);
    EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
}
TEST_F(SimOdomTest, DisabledSourceMessageAfterMppiDoesNotStopStaticIntegration)
{
    ros::Time::setNow(ros::Time(350));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::enabled(manager_,false);
    const auto position=SimOdomTestAccess::model(manager_).getCOMPos();
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1,0));
    queuedFrameStep(blocker);waitForDecision(1,false);
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-position).norm(),1e-6);
    EXPECT_TRUE(SimOdomTestAccess::observations(manager_).empty());
}
TEST_F(SimOdomTest, QueuedSameStampBlockerIsAcceptedAfterCommitThenStopsNextCycle)
{
    ros::Time::setNow(ros::Time(300));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto a=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,a);
    auto b=frame(a->header.stamp);addPedestrian(*b,Eigen::Vector2d(.3,0));
    const auto position=SimOdomTestAccess::model(manager_).getCOMPos();
    queuedFrameStep(b);waitForDecision(1,false);
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-position).norm(),1e-6);
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED,SimOdomTestAccess::corridor(manager_).failure_reason);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    // Distinct decisions use distinct simulation ticks: odom is coalesced per tick.
    ros::Time::setNow(ros::Time::now()+ros::Duration(.001));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(2,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    ros::Time::setNow(ros::Time::now()+ros::Duration(.001));
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(3,false);
    EXPECT_NE(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
}
TEST_F(SimOdomTest, QueuedFrameCannotMaskExpiryAndRetainsOriginalReceipt)
{
    ros::Time::setNow(ros::Time(310));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto a=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,a);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    queuedFrameStep(frame(ros::Time::now()),true);waitForDecision(1,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(ros::Time(310),SimOdomTestAccess::receipt(manager_));
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
}
TEST_F(SimOdomTest, UnreplacedPlannedFrameMustStillBeFreshAtExecution)
{
    ros::Time::setNow(ros::Time(320));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto a=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,a);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::afterMppi(manager_,[&] {ros::Time::setNow(ros::Time(321));});
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForStops(1);waitForOdom(1);
    EXPECT_EQ("PLANNED_FRAME_EXPIRED",SimOdomTestAccess::reason(manager_));
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
}
TEST_F(SimOdomTest, AcceptedInvalidFrameAndEnableTransitionRequireFreshValidData)
{
    ros::Time::setNow(ros::Time(330));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    auto invalid=frame(ros::Time::now());invalid->num=1;
    SimOdomTestAccess::receive(manager_,invalid);
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    SimOdomTestAccess::enabled(manager_,false);SimOdomTestAccess::enabled(manager_,true);
    ros::Time::setNow(ros::Time::now()+ros::Duration(.001));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(2,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    ros::Time::setNow(ros::Time::now()+ros::Duration(.001));
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(3,false);
}
TEST_F(SimOdomTest, RepeatedTenHzEmptyUpdatesDoNotStarveExecution)
{
    ros::Time::setNow(ros::Time(400));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto first=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,first);
    const auto origin=SimOdomTestAccess::model(manager_).getCOMPos();
    double total=0.,maximum=0.;
    for(int i=0;i<8;++i) {
        ros::Time::setNow(ros::Time(400.+.1*i));
        const auto start=ros::WallTime::now();
        queuedFrameStep(frame(ros::Time::now()));waitForDecision(i+1,false);
        const double ms=(ros::WallTime::now()-start).toSec()*1000.;total+=ms;maximum=std::max(maximum,ms);
    }
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-origin).norm(),.1);
    std::cout<<"TRAFFIC empty cycles=8 mean_ms="<<total/8<<" max_ms="<<maximum<<std::endl;
}
TEST_F(SimOdomTest, RepeatedTenHzMovingPedestrianUpdatesDoNotStarveExecution)
{
    ros::Time::setNow(ros::Time(410));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto first=frame(ros::Time::now());addPedestrian(*first,Eigen::Vector2d(1,2));SimOdomTestAccess::receive(manager_,first);
    const auto origin=SimOdomTestAccess::model(manager_).getCOMPos();
    double total=0.,maximum=0.;
    for(int i=0;i<8;++i) {
        ros::Time::setNow(ros::Time(410.+.1*i));
        auto next=frame(ros::Time::now());addPedestrian(*next,Eigen::Vector2d(1.+.1*i,2));
        const auto start=ros::WallTime::now();queuedFrameStep(next);waitForDecision(i+1,false);
        const double ms=(ros::WallTime::now()-start).toSec()*1000.;total+=ms;maximum=std::max(maximum,ms);
    }
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-origin).norm(),.1);
    std::cout<<"TRAFFIC moving cycles=8 mean_ms="<<total/8<<" max_ms="<<maximum<<std::endl;
}
TEST_F(SimOdomTest, GoalResetSerializesWithProductionGeometryAndPublication)
{
    ros::Time::setNow(ros::Time(340));SimOdomTestAccess::dynamicSetup(manager_);
    auto msg=frame(ros::Time::now());
    addPedestrian(*msg,SimOdomTestAccess::model(manager_).getCOMPos().head<2>()+Eigen::Vector2d(1,2));
    SimOdomTestAccess::receive(manager_,msg);
    std::promise<void> held, release, reset_entered;
    auto released=release.get_future().share();
    SimOdomTestAccess::duringGeometry(manager_,[&] {held.set_value();released.wait();});
    auto generating=std::async(std::launch::async,[&] {return SimOdomTestAccess::corridor(manager_);});
    held.get_future().wait();
    auto resetting=std::async(std::launch::async,[&] {reset_entered.set_value();SimOdomTestAccess::goalReset(manager_);});
    reset_entered.get_future().wait();
    EXPECT_EQ(std::future_status::timeout,resetting.wait_for(std::chrono::milliseconds(30)));
    release.set_value();
    EXPECT_TRUE(generating.get().feasible);
    resetting.get();
    EXPECT_TRUE(SimOdomTestAccess::geometryEmpty(manager_));
    SimOdomTestAccess::duringGeometry(manager_,{});
    // Exercise repeated resets against the real build + marker iteration path.
    std::atomic<bool> valid{true};
    std::thread builds([&] {for(int i=0;i<30;++i)if(!SimOdomTestAccess::corridor(manager_).feasible)valid=false;});
    std::thread resets([&] {for(int i=0;i<30;++i)SimOdomTestAccess::goalReset(manager_);});
    builds.join();resets.join();EXPECT_TRUE(valid);
}
TEST_F(SimOdomTest, TimedPlantQueuesWithoutTeleportAndSamplesActualLfpcPhase)
{
    ros::Time::setNow(ros::Time(700));SimOdomTestAccess::timedSetup(manager_,nh_);
    const auto initial=SimOdomTestAccess::model(manager_).getCOMPos();
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    EXPECT_EQ(initial,SimOdomTestAccess::model(manager_).getCOMPos());
    auto expected=SimOdomTestAccess::expectedSample(manager_,.02);
    ros::Time::setNow(ros::Time(700.02));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    EXPECT_TRUE(expected.getCOMPos().isApprox(SimOdomTestAccess::model(manager_).getCOMPos(),1e-12));
    EXPECT_NE(initial,SimOdomTestAccess::model(manager_).getCOMPos());EXPECT_NEAR(.02,SimOdomTestAccess::phase(manager_),1e-9);
    ros::Time::setNow(ros::Time(700.8));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    EXPECT_NEAR(.06,SimOdomTestAccess::phase(manager_),1e-9); // delayed callback cannot skip the step
}
TEST_F(SimOdomTest, ExpiredExecutionFreezesTruePhaseAndFreshClearResumes)
{
    ros::Time::setNow(ros::Time(701));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::enabled(manager_,true);SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    ros::Time::setNow(ros::Time(701.02));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));const auto position=SimOdomTestAccess::model(manager_).getCOMPos();
    ros::Time::setNow(ros::Time(701.7));EXPECT_FALSE(SimOdomTestAccess::execute(manager_));
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));EXPECT_EQ(position,SimOdomTestAccess::model(manager_).getCOMPos());
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    ros::Time::setNow(ros::Time(701.72));EXPECT_TRUE(SimOdomTestAccess::execute(manager_));EXPECT_NEAR(.04,SimOdomTestAccess::phase(manager_),1e-9);
}
TEST_F(SimOdomTest, ExecutionUsesFutureHullForPedestriansButOnlyComAndFootForContainment)
{
    ros::Time::setNow(ros::Time(709));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    auto future=SimOdomTestAccess::expectedSample(manager_,.28);
    auto& controller=SimOdomTestAccess::controller(manager_);
    const auto root=SimOdomTestAccess::humanShape(manager_);
    const auto hull=controller.robotFootprintAt(future.getCOMPos().head<2>(),future.getNextIterState().z());
    auto max_x=[](const Eigen::Vector2d& a,const Eigen::Vector2d& b){return a.x()<b.x();};
    const auto tip=*std::max_element(hull.begin(),hull.end(),max_x);
    const double limit=.5*(std::max_element(root.begin(),root.end(),max_x)->x()+tip.x());
    ASSERT_LT(future.getCOMPos().x(),limit);
    SimOdomTestAccess::clipExecutionCorridor(manager_,limit);
    // This square touches only the future cane, away from the actual root and COM.
    const Eigen::Vector2d cell=(tip.array()/.1).floor().matrix()*.1;
    const TimedPedestrianCollision::Polygon square{cell,cell+Eigen::Vector2d(.1,0),cell+Eigen::Vector2d(.1,.1),cell+Eigen::Vector2d(0,.1)};
    TimedPedestrianCollision oracle;ASSERT_TRUE(oracle.bind({{square,Eigen::Vector2d::Zero()}},controller.robotFootprintAt(Eigen::Vector2d::Zero(),0.)));
    EXPECT_FALSE(oracle.intersects(Eigen::Vector2d::Zero(),0.));
    EXPECT_TRUE(oracle.intersects(future.getCOMPos().head<2>(),0.,future.getNextIterState().z()));
    ros::Time::setNow(ros::Time(709.02));EXPECT_TRUE(SimOdomTestAccess::execute(manager_));
    EXPECT_NEAR(.02,SimOdomTestAccess::phase(manager_),1e-9);
}

TEST_F(SimOdomTest, UnchangedMapExecutesWithoutRebuildingStaticCorridor)
{
    ros::Time::setNow(ros::Time(710));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    const auto revision=SimOdomTestAccess::executionRevision(manager_);
    SimOdomTestAccess::invalidateRebuild(manager_); // any unexpected FIRI rebuild would fail
    ros::Time::setNow(ros::Time(710.02));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    EXPECT_EQ(revision,SimOdomTestAccess::executionRevision(manager_));
    EXPECT_NEAR(.02,SimOdomTestAccess::phase(manager_),1e-9);
}

TEST_F(SimOdomTest, MapUpdateRebuildsCorridorBeforeContinuingQueuedStep)
{
    ros::Time::setNow(ros::Time(711));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    const auto revision=SimOdomTestAccess::executionRevision(manager_);
    const Eigen::Vector2d obstacle(.85,.75);
    ASSERT_TRUE(SimOdomTestAccess::executionContains(manager_,obstacle));
    SimOdomTestAccess::block(manager_,obstacle);
    ros::Time::setNow(ros::Time(711.02));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    EXPECT_NE(revision,SimOdomTestAccess::executionRevision(manager_));
    EXPECT_FALSE(SimOdomTestAccess::executionContains(manager_,obstacle));
}

TEST_F(SimOdomTest, BlockedReferenceCannotFreezeSafeRemainingStepOrNextRoute)
{
    ros::Time::setNow(ros::Time(714));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    ros::Time::setNow(ros::Time(714.04));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    ros::Time::setNow(ros::Time(714.08));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    const auto actual=SimOdomTestAccess::model(manager_).getCOMPos();
    const auto foot=SimOdomTestAccess::model(manager_).getFootPosition();
    const auto support=SimOdomTestAccess::model(manager_).getSupportFeet();
    const auto step=SimOdomTestAccess::model(manager_).getStepNum();
    const auto revision=SimOdomTestAccess::executionRevision(manager_);
    // A newly occupied retained reference is unrelated to the already admitted
    // straight support phase. Its full first seed cannot be certified.
    const std::vector<Eigen::Vector2d> retained{actual.head<2>(),actual.head<2>()+Eigen::Vector2d(0,4),Eigen::Vector2d(8,0)};
    SimOdomTestAccess::reference(manager_,retained);
    const Eigen::Vector2d obstacle=actual.head<2>()+Eigen::Vector2d(0,.9);
    SimOdomTestAccess::block(manager_,obstacle);
    ASSERT_FALSE(SimOdomTestAccess::corridor(manager_).feasible);
    ros::Time::setNow(ros::Time(714.10));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    EXPECT_NE(revision,SimOdomTestAccess::executionRevision(manager_));
    EXPECT_NEAR(.10,SimOdomTestAccess::phase(manager_),1e-9);
    EXPECT_EQ(foot,SimOdomTestAccess::model(manager_).getFootPosition());
    EXPECT_EQ(support,SimOdomTestAccess::model(manager_).getSupportFeet());
    EXPECT_EQ(step,SimOdomTestAccess::model(manager_).getStepNum());
    auto expected=SimOdomTestAccess::expectedSample(manager_,.10);
    EXPECT_EQ(expected.getCOMPos(),SimOdomTestAccess::model(manager_).getCOMPos());
    EXPECT_FALSE(SimOdomTestAccess::executionContains(manager_,obstacle));
    for(int i=6;i<=14;++i) {
        ros::Time::setNow(ros::Time(714.+.02*i));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    }
    ASSERT_FALSE(SimOdomTestAccess::executing(manager_));
    EXPECT_EQ(step+1,SimOdomTestAccess::model(manager_).getStepNum());
    EXPECT_NE(support,SimOdomTestAccess::model(manager_).getSupportFeet());
    // The support boundary can now adopt a freshly searched route and run MPPI.
    SimOdomTestAccess::astarSetup(manager_,nh_);ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    SimOdomTestAccess::step(manager_);
    EXPECT_TRUE(SimOdomTestAccess::executing(manager_));
    EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
    EXPECT_NE(retained,SimOdomTestAccess::route(manager_));
}

TEST_F(SimOdomTest, ActualRemainingStepBlockerPreservesPhaseAndResumesAfterClear)
{
    ros::Time::setNow(ros::Time(715));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    ros::Time::setNow(ros::Time(715.04));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    ros::Time::setNow(ros::Time(715.08));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    const auto actual=SimOdomTestAccess::model(manager_).getCOMPos();
    const auto foot=SimOdomTestAccess::model(manager_).getFootPosition();
    const auto phase=SimOdomTestAccess::phase(manager_);
    auto endpoint=SimOdomTestAccess::expectedSample(manager_,.28);
    const Eigen::Vector2d obstacle=endpoint.getCOMPos().head<2>();
    SimOdomTestAccess::block(manager_,obstacle);
    ros::Time::setNow(ros::Time(715.10));EXPECT_FALSE(SimOdomTestAccess::execute(manager_));
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(actual,SimOdomTestAccess::model(manager_).getCOMPos());
    EXPECT_EQ(foot,SimOdomTestAccess::model(manager_).getFootPosition());
    EXPECT_DOUBLE_EQ(phase,SimOdomTestAccess::phase(manager_));
    SimOdomTestAccess::unblock(manager_,obstacle);
    ros::Time::setNow(ros::Time(715.50));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    EXPECT_NEAR(phase+.04,SimOdomTestAccess::phase(manager_),1e-9);
    auto expected=SimOdomTestAccess::expectedSample(manager_,phase+.04);
    EXPECT_EQ(expected.getCOMPos(),SimOdomTestAccess::model(manager_).getCOMPos());
    EXPECT_EQ(foot,SimOdomTestAccess::model(manager_).getFootPosition());
}

TEST_F(SimOdomTest, ExecutionRejectsFootOutsideUnionWithFreeMap)
{
    ros::Time::setNow(ros::Time(712));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    const auto initial=SimOdomTestAccess::model(manager_).getCOMPos();
    auto future=SimOdomTestAccess::expectedSample(manager_,.28);
    ASSERT_LT(future.getFootPosition().x(),initial.x());
    SimOdomTestAccess::clipExecutionFoot(manager_,.5*(initial.x()+future.getFootPosition().x()));
    ros::Time::setNow(ros::Time(712.02));EXPECT_FALSE(SimOdomTestAccess::execute(manager_));
    EXPECT_EQ(initial,SimOdomTestAccess::model(manager_).getCOMPos());
    EXPECT_DOUBLE_EQ(0.,SimOdomTestAccess::phase(manager_));
}

TEST_F(SimOdomTest, MapChangeAfterOptimizationCannotAuthorizeOldCorridor)
{
    ros::Time::setNow(ros::Time(713));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::afterMppi(manager_,[&] {SimOdomTestAccess::block(manager_,Eigen::Vector2d(.15,0.));});
    const auto initial=SimOdomTestAccess::model(manager_).getCOMPos();
    SimOdomTestAccess::step(manager_);
    EXPECT_FALSE(SimOdomTestAccess::executing(manager_));
    EXPECT_EQ(initial,SimOdomTestAccess::model(manager_).getCOMPos());
}
TEST_F(SimOdomTest, NewStaticCellAndPedestrianStopAnAlreadyQueuedStep)
{
    ros::Time::setNow(ros::Time(702));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    const auto initial=SimOdomTestAccess::model(manager_).getCOMPos();
    auto endpoint=SimOdomTestAccess::expectedSample(manager_,.28);
    SimOdomTestAccess::block(manager_,endpoint.getCOMPos().head<2>());ros::Time::setNow(ros::Time(702.02));
    EXPECT_FALSE(SimOdomTestAccess::execute(manager_));EXPECT_EQ(initial,SimOdomTestAccess::model(manager_).getCOMPos());EXPECT_DOUBLE_EQ(0.,SimOdomTestAccess::phase(manager_));
    // Restore a clear map; independently inject a new accepted pedestrian frame.
    SimOdomTestAccess::timedSetup(manager_,nh_);SimOdomTestAccess::enabled(manager_,true);
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));SimOdomTestAccess::step(manager_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(.5,0));blocker->velocity[0].x=blocker->velocity[0].y=0.;
    SimOdomTestAccess::receive(manager_,blocker);ros::Time::setNow(ros::Time(702.04));
    EXPECT_FALSE(SimOdomTestAccess::execute(manager_));EXPECT_DOUBLE_EQ(0.,SimOdomTestAccess::phase(manager_));
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));ros::Time::setNow(ros::Time(702.06));EXPECT_TRUE(SimOdomTestAccess::execute(manager_));
}
TEST_F(SimOdomTest, TimedPlantGoalResetAndModeGuardsPreventQueuedMotion)
{
    ros::Time::setNow(ros::Time(703));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    const auto initial=SimOdomTestAccess::model(manager_).getCOMPos();
    SimOdomTestAccess::mode(manager_,true,true);ros::Time::setNow(ros::Time(703.02));SimOdomTestAccess::timer(manager_);EXPECT_EQ(initial,SimOdomTestAccess::model(manager_).getCOMPos());
    SimOdomTestAccess::mode(manager_,false,false);SimOdomTestAccess::timer(manager_);EXPECT_EQ(initial,SimOdomTestAccess::model(manager_).getCOMPos());
    SimOdomTestAccess::mode(manager_,true,false);SimOdomTestAccess::goalB(manager_);
    EXPECT_FALSE(SimOdomTestAccess::executing(manager_));SimOdomTestAccess::timer(manager_);EXPECT_EQ(initial,SimOdomTestAccess::model(manager_).getCOMPos());
}
TEST_F(SimOdomTest, TimedPlantStartResetPublishesRequestedYawAndClearsPriorVelocity)
{
    ros::Time::setNow(ros::Time(708));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    ros::Time::setNow(ros::Time(708.02));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    ros::Time::setNow(ros::Time(708.04));
    SimOdomTestAccess::startAt(manager_,1.2);waitForOdom(3);
    EXPECT_NEAR(1.2,tf::getYaw(odom_.back().pose.pose.orientation),1e-12);
    EXPECT_DOUBLE_EQ(0.,odom_.back().twist.twist.linear.x);
    EXPECT_FALSE(SimOdomTestAccess::executing(manager_));
    ros::Time::setNow(ros::Time(708.06));
    SimOdomTestAccess::initializePlant(manager_);waitForOdom(4);
    EXPECT_NEAR(1.2,tf::getYaw(odom_.back().pose.pose.orientation),1e-12);
    EXPECT_DOUBLE_EQ(0.,odom_.back().twist.twist.linear.x);
    EXPECT_DOUBLE_EQ(0.,odom_.back().twist.twist.linear.y);
}
TEST_F(SimOdomTest, TimedPlantReachesActualGoalOnlyAfterElapsedStepThenHolds)
{
    ros::Time::setNow(ros::Time(704));SimOdomTestAccess::timedSetup(manager_,nh_);SimOdomTestAccess::finalStepSetup(manager_);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    EXPECT_FALSE(SimOdomTestAccess::reached(manager_));
    for(int i=1;i<=16;++i){ros::Time::setNow(ros::Time(704.+i*.02));SimOdomTestAccess::timer(manager_);}
    EXPECT_TRUE(SimOdomTestAccess::reached(manager_));EXPECT_FALSE(SimOdomTestAccess::executing(manager_));
    const auto final=SimOdomTestAccess::model(manager_).getCOMPos();ros::Time::setNow(ros::Time(704.5));SimOdomTestAccess::fsm(manager_);EXPECT_EQ(final,SimOdomTestAccess::model(manager_).getCOMPos());
}
TEST_F(SimOdomTest, ExecutionDisplayUsesCurrentSourceAgeAndOnlyCheckedRemainingTimes)
{
    ros::Time::setNow(ros::Time(707));SimOdomTestAccess::timedSetup(manager_,nh_);SimOdomTestAccess::enabled(manager_,true);
    auto distant=frame(ros::Time(706.9));addPedestrian(*distant,Eigen::Vector2d(4,2));distant->velocity[0].x=.2;
    SimOdomTestAccess::receive(manager_,distant);
    std::vector<nav_msgs::Path> paths;std::vector<visualization_msgs::MarkerArray> visuals;
    auto ps=nh_.subscribe<nav_msgs::Path>("timed_path",30,[&](const nav_msgs::Path::ConstPtr& m){paths.push_back(*m);});
    auto vs=nh_.subscribe<visualization_msgs::MarkerArray>("timed_collision",30,[&](const visualization_msgs::MarkerArray::ConstPtr& m){visuals.push_back(*m);});
    const auto end=ros::WallTime::now()+ros::WallDuration(2.);
    while((!ps.getNumPublishers() || !vs.getNumPublishers()) && ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.005).sleep();}
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));
    ros::Time::setNow(ros::Time(707.02));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));
    for(int i=0;i<10;++i){ros::spinOnce();ros::WallDuration(.005).sleep();}
    ASSERT_FALSE(paths.empty());ASSERT_FALSE(visuals.empty());const auto path=paths.back();
    EXPECT_EQ(ros::Time::now(),path.header.stamp);ASSERT_GT(path.poses.size(),4u);
    const auto hull=SimOdomTestAccess::executionHull(manager_);
    for(const auto& m:visuals.back().markers)if(m.ns=="timed_pedestrian") {
        ASSERT_EQ(0,m.id%32); // single pedestrian in the multi-person marker schema
        const size_t sample=m.id/32;
        ASSERT_GT(sample,0u);ASSERT_LE(sample,path.poses.size());
        const double tau=(path.poses[sample-1].header.stamp-path.header.stamp).toSec();
        EXPECT_NEAR(hull[0].x()+.2*(.12+tau),m.points[0].x,1e-8);
        EXPECT_NEAR(hull[0].y(),m.points[0].y,1e-8);
    }
    for(const auto& p:path.poses) {
        const double tau=(p.header.stamp-path.header.stamp).toSec();EXPECT_GE(tau,0.);EXPECT_LE(tau,.26+1e-9);
        auto expected=SimOdomTestAccess::expectedSample(manager_,.02+tau);
        EXPECT_NEAR(expected.getCOMPos().x(),p.pose.position.x,1e-10);EXPECT_NEAR(expected.getCOMPos().y(),p.pose.position.y,1e-10);
    }
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,SimOdomTestAccess::model(manager_).getCOMPos().head<2>());blocker->velocity[0].x=0.;SimOdomTestAccess::receive(manager_,blocker);
    ros::Time::setNow(ros::Time(707.04));EXPECT_FALSE(SimOdomTestAccess::execute(manager_));
    for(int i=0;i<10;++i){ros::spinOnce();ros::WallDuration(.005).sleep();}
    EXPECT_TRUE(paths.back().poses.empty());
    EXPECT_TRUE(std::any_of(visuals.back().markers.begin(),visuals.back().markers.end(),[](const visualization_msgs::Marker& m){return m.ns=="timed_rejected" && m.color.r==1.;}));
}
TEST_F(SimOdomTest, FullTimedPredictionRemainsVisibleAfterExecutionDisplayUpdates)
{
    ros::Time::setNow(ros::Time(770));
    SimOdomTestAccess::timedSetup(manager_,nh_);SimOdomTestAccess::enabled(manager_,true);
    auto distant=frame(ros::Time(769.9));addPedestrian(*distant,Eigen::Vector2d(4,2));
    distant->velocity[0].x=.5;SimOdomTestAccess::receive(manager_,distant);
    visualization_msgs::MarkerArray execution,plan;
    auto es=nh_.subscribe<visualization_msgs::MarkerArray>("timed_collision",30,
        [&](const visualization_msgs::MarkerArray::ConstPtr& m){execution=*m;});
    auto ps=nh_.subscribe<visualization_msgs::MarkerArray>("timed_plan",30,
        [&](const visualization_msgs::MarkerArray::ConstPtr& m){plan=*m;});
    const auto ready=ros::WallTime::now()+ros::WallDuration(2.);
    while(!es.getNumPublishers() && ros::WallTime::now()<ready) {
        ros::spinOnce();ros::WallDuration(.005).sleep();
    }
    auto drain=[] {for(int i=0;i<10;++i){ros::spinOnce();ros::WallDuration(.005).sleep();}};
    auto horizon=[](const visualization_msgs::MarkerArray& message) {
        double maximum=0.;
        for(const auto& marker:message.markers)
            if(marker.action==visualization_msgs::Marker::ADD && marker.type==visualization_msgs::Marker::TEXT_VIEW_FACING) {
                const auto plus=marker.text.find('+');
                if(plus!=std::string::npos)maximum=std::max(maximum,std::stod(marker.text.substr(plus+1)));
            }
        return maximum;
    };
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));drain();
    const double planned_horizon=horizon(execution);ASSERT_GT(planned_horizon,1.);
    ros::Time::setNow(ros::Time(770.02));ASSERT_TRUE(SimOdomTestAccess::execute(manager_));drain();
    std::cout<<"TIMED_DISPLAY full_plan_seconds="<<planned_horizon
             <<" execution_seconds="<<horizon(execution)<<" retained_plan_seconds="<<horizon(plan)<<std::endl;
    EXPECT_LE(horizon(execution),.26+1e-9);
    EXPECT_GE(std::max(horizon(execution),horizon(plan)),planned_horizon-1e-9)
        <<"The 20ms execution display must not erase the full pedestrian prediction from RViz";
    if(!plan.markers.empty()) {
        EXPECT_DOUBLE_EQ(planned_horizon,horizon(plan));
        EXPECT_EQ(ros::Time(770),plan.markers.front().header.stamp);
    }
    // Invalid source/STOP must also clear the retained planning snapshot.
    ros::Time::setNow(ros::Time(770.6));EXPECT_FALSE(SimOdomTestAccess::execute(manager_));drain();
    EXPECT_DOUBLE_EQ(0.,horizon(plan));
    // A new successful plan can restore the snapshot; a fresh empty execution
    // frame must clear it immediately, even before the next planning boundary.
    ros::Time::setNow(ros::Time(771));SimOdomTestAccess::timedSetup(manager_,nh_);SimOdomTestAccess::enabled(manager_,true);
    distant->header.stamp=ros::Time::now();SimOdomTestAccess::receive(manager_,distant);
    SimOdomTestAccess::step(manager_);ASSERT_TRUE(SimOdomTestAccess::executing(manager_));drain();
    ASSERT_GT(horizon(plan),1.);
    ros::Time::setNow(ros::Time(771.02));SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    ASSERT_TRUE(SimOdomTestAccess::execute(manager_));drain();
    EXPECT_DOUBLE_EQ(0.,horizon(plan));
}
TEST_F(SimOdomTest, TimedPlantExecutesAndChecksNewFramesWhileGlobalWorkerIsDelayed)
{
    ros::Time::setNow(ros::Time(705));SimOdomTestAccess::timedSetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    SimOdomTestAccess::enabled(manager_,true);SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    std::promise<void> entered,release;auto entry=entered.get_future();auto gate=release.get_future().share();
    SimOdomTestAccess::workerHook(manager_,[&]{entered.set_value();gate.wait_for(std::chrono::seconds(1));});
    SimOdomTestAccess::step(manager_);EXPECT_EQ(std::future_status::ready,entry.wait_for(std::chrono::seconds(1)));
    const auto root=SimOdomTestAccess::model(manager_).getCOMPos();
    ros::Time::setNow(ros::Time(705.02));SimOdomTestAccess::timer(manager_);EXPECT_NE(root,SimOdomTestAccess::model(manager_).getCOMPos());
    const auto actual=SimOdomTestAccess::model(manager_).getCOMPos();const auto phase=SimOdomTestAccess::phase(manager_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,actual.head<2>());blocker->velocity[0].x=blocker->velocity[0].y=0.;SimOdomTestAccess::receive(manager_,blocker);
    ros::Time::setNow(ros::Time(705.04));SimOdomTestAccess::timer(manager_);
    EXPECT_EQ(actual,SimOdomTestAccess::model(manager_).getCOMPos());EXPECT_DOUBLE_EQ(phase,SimOdomTestAccess::phase(manager_));
    release.set_value();EXPECT_TRUE(SimOdomTestAccess::workerReady(manager_));SimOdomTestAccess::workerHook(manager_,{});
}
TEST_F(SimOdomTest, BackgroundGlobalSearchDoesNotBlockPlantOrAcceptedObservations)
{
    ros::Time::setNow(ros::Time(590));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    std::promise<void> entered,release;auto entered_future=entered.get_future();
    auto gate=release.get_future().share();
    SimOdomTestAccess::workerHook(manager_,[&]{entered.set_value();gate.wait_for(std::chrono::seconds(1));});
    const auto old_route=SimOdomTestAccess::route(manager_);
    const auto root=SimOdomTestAccess::model(manager_).getCOMPos();
    const auto start=std::chrono::steady_clock::now();SimOdomTestAccess::step(manager_);
    EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),.1);
    EXPECT_EQ(std::future_status::ready,entered_future.wait_for(std::chrono::seconds(1)));
    EXPECT_NE(root,SimOdomTestAccess::model(manager_).getCOMPos());
    auto receipt=std::async(std::launch::async,[&]{SimOdomTestAccess::receive(manager_,clear);});
    EXPECT_EQ(std::future_status::ready,receipt.wait_for(std::chrono::milliseconds(50)));receipt.get();
    ros::Time::setNow(ros::Time(590.2));clear->header.stamp=ros::Time::now();SimOdomTestAccess::receive(manager_,clear);
    SimOdomTestAccess::step(manager_);
    EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_)); // busy slot is coalesced
    EXPECT_EQ(old_route,SimOdomTestAccess::route(manager_));
    release.set_value();EXPECT_TRUE(SimOdomTestAccess::workerReady(manager_));
    SimOdomTestAccess::workerHook(manager_,{});
    SimOdomTestAccess::step(manager_);
    EXPECT_EQ("SUCCESS",SimOdomTestAccess::recoveryReason(manager_));
    EXPECT_EQ(Eigen::Vector2d(5,0),SimOdomTestAccess::route(manager_).back());
}
TEST_F(SimOdomTest, BackgroundResultFromPreviousGoalIsDiscarded)
{
    ros::Time::setNow(ros::Time(591));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    SimOdomTestAccess::step(manager_);EXPECT_TRUE(SimOdomTestAccess::workerReady(manager_));
    SimOdomTestAccess::goalB(manager_);const auto old=SimOdomTestAccess::route(manager_);
    EXPECT_FALSE(SimOdomTestAccess::pollRoute(manager_));
    EXPECT_EQ("SUPERSEDED",SimOdomTestAccess::recoveryReason(manager_));
    EXPECT_EQ(old,SimOdomTestAccess::route(manager_));
}
TEST_F(SimOdomTest, FreshObservationDoesNotRestampAnExpiredBackgroundRequest)
{
    ros::Time::setNow(ros::Time(592));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    SimOdomTestAccess::step(manager_);EXPECT_TRUE(SimOdomTestAccess::workerReady(manager_));
    const auto old=SimOdomTestAccess::route(manager_);
    ros::Time::setNow(ros::Time(592.6));SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    SimOdomTestAccess::step(manager_);
    EXPECT_EQ("EXPIRED",SimOdomTestAccess::recoveryReason(manager_));EXPECT_EQ(old,SimOdomTestAccess::route(manager_));
    EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
}
TEST_F(SimOdomTest, GlobalRefreshRunsProactivelyAtHalfSecondCadence)
{
    ros::Time::setNow(ros::Time(600));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    SimOdomTestAccess::refreshPeriod(manager_,.5); // slower cadence remains configurable
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    const auto root=SimOdomTestAccess::model(manager_).getCOMPos().head<2>().eval();
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    queuedFrameStep(clear);waitForDecision(1,false);
    EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    EXPECT_EQ(root,SimOdomTestAccess::route(manager_).front());
    EXPECT_EQ(Eigen::Vector2d(5,0),SimOdomTestAccess::route(manager_).back());
    ros::Time::setNow(ros::Time(600.49));clear->header.stamp=ros::Time::now();
    SimOdomTestAccess::receive(manager_,clear);queuedFrameStep(clear);waitForDecision(2,false);
    EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    ros::Time::setNow(ros::Time(600.5));clear->header.stamp=ros::Time::now();
    SimOdomTestAccess::receive(manager_,clear);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto next_root=SimOdomTestAccess::model(manager_).getCOMPos().head<2>().eval();
    queuedFrameStep(clear);waitForDecision(3,false);
    EXPECT_EQ(2u,SimOdomTestAccess::attempts(manager_));
    EXPECT_EQ(next_root,SimOdomTestAccess::route(manager_).front());
    EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
}
TEST_F(SimOdomTest, DefaultGlobalRefreshRunsAtFiveHertzWithoutCatchUpBurst)
{
    ros::Time::setNow(ros::Time(625));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(1,false);
    ros::Time::setNow(ros::Time(625.2));clear->header.stamp=ros::Time::now();
    SimOdomTestAccess::receive(manager_,clear);queuedFrameStep(clear);waitForDecision(2,false);
    EXPECT_EQ(2u,SimOdomTestAccess::attempts(manager_));
    // A delayed cycle skips missed deadlines; it cannot run several searches.
    ros::Time::setNow(ros::Time(626.01));clear->header.stamp=ros::Time::now();
    SimOdomTestAccess::receive(manager_,clear);queuedFrameStep(clear);waitForDecision(3,false);
    EXPECT_EQ(3u,SimOdomTestAccess::attempts(manager_));
    ros::Time::setNow(ros::Time(626.11));clear->header.stamp=ros::Time::now();
    SimOdomTestAccess::receive(manager_,clear);queuedFrameStep(clear);waitForDecision(4,false);
    EXPECT_EQ(3u,SimOdomTestAccess::attempts(manager_));
}
TEST_F(SimOdomTest, FastGlobalRefreshKeepsPhaseWhenControlTicksHaveSmallJitter)
{
    ros::Time::setNow(ros::Time(620.017));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    SimOdomTestAccess::refreshPeriod(manager_,.1); // also support explicit 10Hz requests
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(1,false);
    EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    ros::Time::setNow(ros::Time(620.118));clear->header.stamp=ros::Time::now();
    SimOdomTestAccess::receive(manager_,clear);queuedFrameStep(clear);waitForDecision(2,false);
    EXPECT_EQ(2u,SimOdomTestAccess::attempts(manager_));
    // The previous search started 1ms late. The next deadline must remain .217,
    // not drift to .218 and skip an entire 100ms control tick.
    ros::Time::setNow(ros::Time(620.217));clear->header.stamp=ros::Time::now();
    SimOdomTestAccess::receive(manager_,clear);queuedFrameStep(clear);waitForDecision(3,false);
    EXPECT_EQ(3u,SimOdomTestAccess::attempts(manager_));
    EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
}
TEST_F(SimOdomTest, NewlyObservedStaticObstacleRefreshesWholeRoute)
{
    ros::Time::setNow(ros::Time(610));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    queuedFrameStep(clear);waitForDecision(1,false);
    const auto original=SimOdomTestAccess::route(manager_);
    for(double y=-.6;y<=.6;y+=.1)SimOdomTestAccess::block(manager_,Eigen::Vector2d(2.,y));
    ros::Time::setNow(ros::Time(610.5));clear->header.stamp=ros::Time::now();
    SimOdomTestAccess::receive(manager_,clear);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto root=SimOdomTestAccess::model(manager_).getCOMPos().head<2>().eval();
    queuedFrameStep(clear);waitForDecision(2,false);
    const auto detour=SimOdomTestAccess::route(manager_);
    EXPECT_EQ(2u,SimOdomTestAccess::attempts(manager_));EXPECT_NE(original,detour);
    EXPECT_EQ(root,detour.front());EXPECT_EQ(Eigen::Vector2d(5,0),detour.back());
    double deviation=0.;for(const auto& p:detour)deviation=std::max(deviation,std::abs(p.y()));
    EXPECT_GT(deviation,.6);EXPECT_EQ("SUCCESS",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, FarPedestrianDoesNotDiscardSafeCorridorPrefix)
{
    ros::Time::setNow(ros::Time(495));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::adaptiveCorridor(manager_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(4,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;SimOdomTestAccess::receive(manager_,blocker);
    const auto before=SimOdomTestAccess::model(manager_).getCOMPos().eval();
    queuedFrameStep(blocker);waitForDecision(1,false);
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-before).norm(),.01);
    EXPECT_TRUE(SimOdomTestAccess::controller(manager_).getDebugMetrics().timed_execution_safe);
}
TEST_F(SimOdomTest, RejectedGlobalCandidateRevalidatesRetainedRoute)
{
    ros::Time::setNow(ros::Time(496));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::adaptiveCorridor(manager_);
    SimOdomTestAccess::astarSetup(manager_,nh_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_)); // candidate predates this new static cell
    SimOdomTestAccess::block(manager_,Eigen::Vector2d(2,0));
    const std::vector<Eigen::Vector2d> retained{{0,0},{1,-.5},{3,-.5},{5,0}};
    SimOdomTestAccess::reference(manager_,retained);
    const auto before=SimOdomTestAccess::model(manager_).getCOMPos().eval();
    queuedFrameStep(clear);waitForDecision(1,false);
    EXPECT_EQ(retained,SimOdomTestAccess::route(manager_));
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-before).norm(),.01);
    EXPECT_EQ("RETAINED_AFTER_REBUILD_FAILED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, StaleCandidateCannotCommitOnlyBecauseItsPrefixIsUsable)
{
    ros::Time::setNow(ros::Time(496.5));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::adaptiveCorridor(manager_);
    SimOdomTestAccess::astarSetup(manager_,nh_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(4,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;SimOdomTestAccess::receive(manager_,blocker);
    const auto retained=SimOdomTestAccess::route(manager_);
    const auto before=SimOdomTestAccess::model(manager_).getCOMPos().eval();
    queuedFrameStep(blocker);waitForDecision(1,false);
    EXPECT_EQ(retained,SimOdomTestAccess::route(manager_));
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-before).norm(),.01);
    EXPECT_EQ("RETAINED_AFTER_REBUILD_FAILED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, MppiRejectedCandidateCanUseFreshRetainedRoute)
{
    ros::Time::setNow(ros::Time(497));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("mpc/num_samples",1);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::adaptiveCorridor(manager_);
    SimOdomTestAccess::astarSetup(manager_,nh_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    // A spatially free north-going route turns too sharply for this single
    // LFPC proposal; the retained east-going route is free below the wall.
    for(double x=.4;x<1.;x+=.1)for(double y=.2;y<3.;y+=.1)
        SimOdomTestAccess::block(manager_,Eigen::Vector2d(x,y));
    SimOdomTestAccess::candidate(manager_,{{0,0},{0,3.5},{2,3.5},{5,0}});
    const auto retained=SimOdomTestAccess::route(manager_);
    const auto before=SimOdomTestAccess::model(manager_).getCOMPos().eval();
    queuedFrameStep(clear);waitForDecision(1,false);
    EXPECT_EQ(retained,SimOdomTestAccess::route(manager_));
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-before).norm(),.01);
    EXPECT_EQ("RETAINED_AFTER_MPPI_REJECTED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, PeriodicGlobalReferenceAvoidsStationaryBlocker)
{
    ros::Time::setNow(ros::Time(499));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1.3,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;SimOdomTestAccess::receive(manager_,blocker);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto route=SimOdomTestAccess::route(manager_);
    queuedFrameStep(blocker);waitForDecision(1,false);
    EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));EXPECT_NE(route,SimOdomTestAccess::route(manager_));
    EXPECT_TRUE(SimOdomTestAccess::controller(manager_).getDebugMetrics().timed_execution_safe);
    // Keep the blocker present until the actual CoM has passed it. Eventual
    // progress after deleting a pedestrian would not demonstrate a bypass.
    for(int i=0;i<15 && SimOdomTestAccess::model(manager_).getCOMPos().x()<2.3;++i) {
        blocker->header.stamp=ros::Time::now();SimOdomTestAccess::receive(manager_,blocker);
        queuedFrameStep(blocker);waitForDecision(i+2,false);
        EXPECT_TRUE(SimOdomTestAccess::controller(manager_).getDebugMetrics().timed_execution_safe);
    }
    EXPECT_GT(SimOdomTestAccess::model(manager_).getCOMPos().x(),2.3);
}
TEST_F(SimOdomTest, PointDetourCannotExecuteWhenSingleMppiProposalHitsPedestrian)
{
    ros::Time::setNow(ros::Time(500));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);nh_.setParam("mpc/num_samples",1);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1.3,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;
    SimOdomTestAccess::receive(manager_,blocker);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto original=SimOdomTestAccess::route(manager_);
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    // A spatial point detour alone no longer certifies the future cane. With no
    // random exploration, the colliding proposal must not commit or move LFPC.
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ("MPPI_REJECTED",SimOdomTestAccess::recoveryReason(manager_));
    EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    EXPECT_EQ(original,SimOdomTestAccess::route(manager_));
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    // The final debug metrics describe the retained-route retry. The candidate
    // rejection is preserved by MPPI_REJECTED above; neither attempt may move.
    EXPECT_FALSE(SimOdomTestAccess::planValid(manager_));
}
TEST_F(SimOdomTest, FullyBlockedGlobalSearchStopsAndFreshClearResumes)
{
    ros::Time::setNow(ros::Time(510));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_,true);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1.3,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;
    SimOdomTestAccess::receive(manager_,blocker);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    const auto route=SimOdomTestAccess::route(manager_);const auto start=ros::WallTime::now();
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    std::cout<<"MANAGER no_path_cycle_ms="<<(ros::WallTime::now()-start).toSec()*1000<<std::endl;
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(2,false);
}
TEST_F(SimOdomTest, GlobalCandidateExpiryCannotCommitOrExecute)
{
    ros::Time::setNow(ros::Time(520));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);nh_.setParam("mpc/num_samples",1);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1.3,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;
    SimOdomTestAccess::receive(manager_,blocker);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto route=SimOdomTestAccess::route(manager_);const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::afterSearch(manager_,[]{ros::Time::setNow(ros::Time(521));});
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ("EXPIRED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, GlobalRebuildFailureCannotCommitOrExecute)
{
    ros::Time::setNow(ros::Time(530));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);nh_.setParam("mpc/num_samples",1);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1.3,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;
    SimOdomTestAccess::receive(manager_,blocker);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto route=SimOdomTestAccess::route(manager_);const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::afterSearch(manager_,[&]{SimOdomTestAccess::invalidateRebuild(manager_);});
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ("REBUILD_FAILED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, RefreshedTrajectoryExpiryAtExecutionCannotCommitRoute)
{
    ros::Time::setNow(ros::Time(535));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true); // retain normal MPPI exploration for point guidance
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1.3,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;SimOdomTestAccess::receive(manager_,blocker);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto route=SimOdomTestAccess::route(manager_);const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::afterMppi(manager_,[&]{
        EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));ros::Time::setNow(ros::Time(536));
    });
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ("EXPIRED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, GeometricDetourCannotCommitWhenItsTimedMppiFails)
{
    ros::Time::setNow(ros::Time(537));SimOdomTestAccess::dynamicSetup(manager_);
    nh_.setParam("human_cane/enabled",true);nh_.setParam("mpc/num_samples",1);nh_.setParam("mpc/max_api",0.);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1.3,0));
    blocker->velocity[0].x=blocker->velocity[0].y=0.;SimOdomTestAccess::receive(manager_,blocker);
    ASSERT_TRUE(SimOdomTestAccess::primeGlobal(manager_));
    const auto route=SimOdomTestAccess::route(manager_);const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ("MPPI_REJECTED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, InitialSearchActivationQueuesNewGoalAndRequiresItsOwnSearch)
{
    ros::Time::setNow(ros::Time(540));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    SimOdomTestAccess::initialSetup(manager_);
    std::promise<void> entered;auto entry=entered.get_future();std::future<void> goal;
    SimOdomTestAccess::goalEntry(manager_,[&]{entered.set_value();});
    SimOdomTestAccess::initialHook(manager_,[&]{
        goal=std::async(std::launch::async,[&]{SimOdomTestAccess::goalB(manager_);});
        EXPECT_EQ(std::future_status::ready,entry.wait_for(std::chrono::seconds(2)));
        EXPECT_EQ(std::future_status::timeout,goal.wait_for(std::chrono::milliseconds(20)));
    });
    const auto pos=SimOdomTestAccess::model(manager_).getCOMPos();
    SimOdomTestAccess::fsm(manager_);goal.get();
    EXPECT_FALSE(SimOdomTestAccess::active(manager_));
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(pos,SimOdomTestAccess::model(manager_).getCOMPos());
    SimOdomTestAccess::initialHook(manager_,{});SimOdomTestAccess::goalEntry(manager_,{});
    SimOdomTestAccess::fsm(manager_);
    EXPECT_TRUE(SimOdomTestAccess::active(manager_));
    ASSERT_FALSE(SimOdomTestAccess::route(manager_).empty());
    EXPECT_EQ(Eigen::Vector2d(4,1),SimOdomTestAccess::route(manager_).back());
    EXPECT_EQ(pos,SimOdomTestAccess::model(manager_).getCOMPos());
}
TEST_F(SimOdomTest, TransactionDiagnosticIdentityClockAndSkippedStageReset)
{
    std::vector<std_msgs::Float64MultiArray> records;
    auto sub=nh_.subscribe<std_msgs::Float64MultiArray>("transaction",20,
        [&](const std_msgs::Float64MultiArray::ConstPtr& m){records.push_back(*m);});
    const auto connected=ros::WallTime::now()+ros::WallDuration(2.);
    while(sub.getNumPublishers()==0 && ros::WallTime::now()<connected){ros::spinOnce();ros::WallDuration(.005).sleep();}
    ASSERT_EQ(1u,sub.getNumPublishers());
    auto wait=[&](size_t n){const auto end=ros::WallTime::now()+ros::WallDuration(2.);
        while(records.size()<n && ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.005).sleep();}};
    ros::Time::setNow(ros::Time(600));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(1,false);wait(1);
    ASSERT_EQ(1u,records.size());ASSERT_EQ(33u,records[0].data.size());
    const auto a=records[0].data;
    EXPECT_EQ(1.,a[0]);EXPECT_EQ(600.,a[1]);EXPECT_TRUE(std::isnan(a[3]));EXPECT_TRUE(std::isnan(a[4]));
    EXPECT_EQ(600.,a[7]);EXPECT_EQ(600.,a[8]);EXPECT_EQ(0.,a[9]);EXPECT_EQ(0.,a[10]);
    EXPECT_NE(std::string::npos,records[0].layout.dim[0].label.find("source_frame=world"));
    EXPECT_GE(a[13],0.);EXPECT_TRUE(std::isnan(a[14]));EXPECT_TRUE(std::isnan(a[15]));
    EXPECT_GE(a[16],0.);EXPECT_GE(a[17],0.);EXPECT_GE(a[32],0.);EXPECT_EQ(0.,a[21]);
    EXPECT_GE(a[19],a[18]);EXPECT_EQ(0.,a[29]);EXPECT_EQ(1.,a[30]);
    // Large ROS jump, small wall interval: diagnose without asserting its cause.
    ros::Time::setNow(ros::Time(603));auto malformed=frame(ros::Time::now());malformed->num=1;
    SimOdomTestAccess::receive(manager_,malformed);
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(2,true);wait(2);
    ASSERT_EQ(2u,records.size());const auto b=records[1].data;
    EXPECT_EQ(3.,b[3]);EXPECT_GE(b[4],0.);EXPECT_EQ(603.,b[7]);EXPECT_EQ(603.,b[8]);EXPECT_GT(b[6],a[6]);
    EXPECT_TRUE(std::isnan(b[13]));EXPECT_TRUE(std::isnan(b[14]));EXPECT_TRUE(std::isnan(b[15]));
    EXPECT_TRUE(std::isnan(b[22]));EXPECT_TRUE(std::isnan(b[28]));EXPECT_TRUE(std::isnan(b[32]));
    EXPECT_EQ(1.,b[21]);EXPECT_EQ(0.,b[30]);
    EXPECT_EQ(static_cast<double>(ConvexCorridor::FailureReason::DYNAMIC_DATA_INVALID),b[31]);
    ros::Time::setNow(ros::Time(603.001));
    SimOdomTestAccess::enabled(manager_,false);queuedFrameStep(clear);waitForDecision(3,false);wait(3);
    ASSERT_EQ(3u,records.size());EXPECT_EQ(0.,records[2].data[5]);EXPECT_TRUE(std::isnan(records[2].data[7]));
    EXPECT_NE(std::string::npos,records[2].layout.dim[0].label.find("source_frame=DISABLED"));
}
TEST_F(SimOdomTest, ActualArrivalIdleHeartbeatAndNewGoalKeepPlantState)
{
    ros::Time::setNow(ros::Time(700));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    SimOdomTestAccess::arriveSetup(manager_);
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    const auto pos=SimOdomTestAccess::model(manager_).getCOMPos();
    std::atomic<bool> done{false};
    std::thread clock([&]{double t=700.;while(!done){t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
    SimOdomTestAccess::fsm(manager_);done=true;clock.join();waitForOdom(1);
    for(int i=0;i<12;++i) {
        ros::Time::setNow(ros::Time(700.1+.1*i));SimOdomTestAccess::fsm(manager_);waitForOdom(i+2);
        EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
        EXPECT_EQ(pos,SimOdomTestAccess::model(manager_).getCOMPos());
        EXPECT_EQ(geometry_msgs::Twist(),odom_.back().twist.twist);
        EXPECT_EQ(ros::Time::now(),odom_.back().header.stamp);
    }
    SimOdomTestAccess::goalB(manager_);SimOdomTestAccess::fsm(manager_); // WAIT -> GEN
    EXPECT_EQ(13u,odom_.size());EXPECT_FALSE(SimOdomTestAccess::active(manager_));
    SimOdomTestAccess::fsm(manager_);EXPECT_TRUE(SimOdomTestAccess::active(manager_));
    EXPECT_EQ(Eigen::Vector2d(4,1),SimOdomTestAccess::route(manager_).back());
    EXPECT_EQ(pos,SimOdomTestAccess::model(manager_).getCOMPos());
}
TEST_F(SimOdomTest, LastSafeFinalStepThenActualArrivalAndIdle)
{
    ros::Time::setNow(ros::Time(720));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::finalStepSetup(manager_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    std::atomic<bool> done{false};
    std::thread clock([&]{double t=720.;while(!done){t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
    SimOdomTestAccess::fsm(manager_);
    EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
    EXPECT_LT((SimOdomTestAccess::model(manager_).getCOMPos().head<2>()-Eigen::Vector2d(.30,0)).norm(),.2);
    // Arrival is a distinct plant publication on the next simulation tick.
    done=true;clock.join();ros::Time::setNow(ros::Time::now()+ros::Duration(.001));
    SimOdomTestAccess::fsm(manager_);waitForOdom(2);
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    for(int i=0;i<8;++i){ros::Time::setNow(ros::Time(720.1+.1*i));SimOdomTestAccess::fsm(manager_);}
    waitForOdom(10);EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(geometry_msgs::Twist(),odom_.back().twist.twist);
}
TEST_F(SimOdomTest, AdvancedCornerReferenceCoversActualPreviousLegalStep)
{
    ros::Time::setNow(ros::Time(740));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    const auto before=SimOdomTestAccess::model(manager_).getCOMPos().eval();
    std::atomic<bool> done{false};
    std::thread clock([&]{double t=740.;while(!done){t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
    SimOdomTestAccess::step(manager_);done=true;clock.join();
    ASSERT_TRUE(SimOdomTestAccess::planValid(manager_));
    const Eigen::Vector2d p=SimOdomTestAccess::model(manager_).getCOMPos().head<2>();
    ASSERT_GT((p-before.head<2>()).norm(),.01);
    SimOdomTestAccess::smallCorridor(manager_);
    const std::vector<Eigen::Vector2d> route{p+Eigen::Vector2d(-.7,.3372),
        p+Eigen::Vector2d(-.2,.3372),p+Eigen::Vector2d(.5,.3372),p+Eigen::Vector2d(.5,1)};
    SimOdomTestAccess::reference(manager_,route);
    ASSERT_EQ(1u,SimOdomTestAccess::reanchor(manager_));
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    const auto result=SimOdomTestAccess::corridor(manager_);
    ASSERT_TRUE(result.feasible)<<ConvexCorridor::failureReasonName(result.failure_reason);
    EXPECT_TRUE(std::any_of(result.segments.begin(),result.segments.end(),[&](const ConvexCorridor::Segment& s){return ConvexCorridor::contains(s,p,1e-6);}));
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));
    for(const auto& q:{route[1],route[2],route[3]})
        EXPECT_TRUE(std::any_of(result.segments.begin(),result.segments.end(),[&](const ConvexCorridor::Segment& s){return ConvexCorridor::contains(s,q,1e-6);}));
}

TEST_F(SimOdomTest, RecordedOffsetAndNearZeroConnectorsKeepExactRoot)
{
    ros::Time::setNow(ros::Time(750));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::enabled(manager_,false);
    const Eigen::Vector2d recorded(-2.5058499314422193,-2.157256244571377);
    const Eigen::Vector2d projection(-2.2674125774,-2.3956935986);
    const Eigen::Vector2d direction=Eigen::Vector2d(1,1).normalized();
    SimOdomTestAccess::smallCorridor(manager_);
    for(double offset:{1.,1e-6,1e-10,0.}) {
        const Eigen::Vector2d p=projection+offset*(recorded-projection);
        SimOdomTestAccess::root(manager_,p);
        SimOdomTestAccess::reference(manager_,{projection-direction,projection,projection+.5*direction,projection+Eigen::Vector2d(1,0)},1);
        const auto result=SimOdomTestAccess::corridor(manager_);
        ASSERT_TRUE(result.feasible)<<offset<<" "<<ConvexCorridor::failureReasonName(result.failure_reason);
        EXPECT_TRUE(std::any_of(result.segments.begin(),result.segments.end(),[&](const ConvexCorridor::Segment& s){return ConvexCorridor::contains(s,p,1e-6);}));
    }
}

TEST_F(SimOdomTest, StaticConnectorCannotCutBlockedCellOrAdvanceState)
{
    ros::Time::setNow(ros::Time(760));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::enabled(manager_,false);
    SimOdomTestAccess::root(manager_,Eigen::Vector2d(0,0));
    const std::vector<Eigen::Vector2d> route{{0,2},{1,2},{2,2}};
    SimOdomTestAccess::reference(manager_,route);
    ASSERT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
    SimOdomTestAccess::block(manager_,Eigen::Vector2d(.01,1.01));
    const auto blocked=SimOdomTestAccess::corridor(manager_);
    EXPECT_FALSE(blocked.feasible);EXPECT_TRUE(blocked.segments.empty());
    EXPECT_EQ(ConvexCorridor::FailureReason::REFERENCE_OCCUPIED,blocked.failure_reason);
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::step(manager_);waitForStops(1);
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));
    EXPECT_FALSE(SimOdomTestAccess::planValid(manager_));
}

TEST_F(SimOdomTest, PedestrianRootBlocksCurrentCorridorWithoutChangingStaticRoute)
{
    ros::Time::setNow(ros::Time(770));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    SimOdomTestAccess::root(manager_,Eigen::Vector2d(0,0));
    const std::vector<Eigen::Vector2d> route{{0,2},{1,2},{2,2}};
    SimOdomTestAccess::reference(manager_,route);
    SimOdomTestAccess::receive(manager_,frame(ros::Time::now()));
    ASSERT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
    auto msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(0,0));
    SimOdomTestAccess::receive(manager_,msg);
    const auto blocked=SimOdomTestAccess::corridor(manager_);
    EXPECT_FALSE(blocked.feasible);EXPECT_TRUE(blocked.segments.empty());
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED,blocked.failure_reason);
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::step(manager_);waitForStops(1);
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));
    EXPECT_FALSE(SimOdomTestAccess::planValid(manager_));
}

TEST_F(SimOdomTest, IdleHeartbeatModeAndInitializationGuards)
{
    for(int mode=0;mode<3;++mode) {
        SimOdomTestAccess::idleSetup(manager_,mode!=0);
        SimOdomTestAccess::mode(manager_,mode!=1,mode==2);
        SimOdomTestAccess::fsm(manager_);
    }
    ros::WallDuration(.1).sleep();ros::spinOnce();EXPECT_TRUE(odom_.empty());
}

TEST_F(SimOdomTest, PoseResetWhileWaitingForNextGoalKeepsStationaryHeartbeat)
{
    ros::Time::setNow(ros::Time(760));SimOdomTestAccess::timedSetup(manager_,nh_);
    SimOdomTestAccess::idleSetup(manager_,true); // previous goal has completed
    SimOdomTestAccess::startAt(manager_,.65);waitForOdom(1);
    EXPECT_FALSE(SimOdomTestAccess::reached(manager_));
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    const auto pose=odom_.back().pose.pose;
    for(int i=0;i<8;++i) {
        ros::Time::setNow(ros::Time(760.1+.1*i));
        SimOdomTestAccess::timer(manager_);SimOdomTestAccess::fsm(manager_);
        waitForOdom(i+2);ASSERT_EQ(size_t(i+2),odom_.size());
        EXPECT_EQ(ros::Time::now(),odom_.back().header.stamp);
        EXPECT_EQ(pose,odom_.back().pose.pose);
        EXPECT_EQ(geometry_msgs::Twist(),odom_.back().twist.twist);
        EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
        EXPECT_FALSE(SimOdomTestAccess::active(manager_));
    }
}
}  // namespace cane_planner

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_sim_odom_stop");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
