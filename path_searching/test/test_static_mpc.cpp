#include <gtest/gtest.h>
#include <chrono>
#include <array>
#include <iostream>
#include <path_searching/mpc_controller.h>

namespace fast_planner {
class SdfProducerTestAccess { public: static void update(SDFMap& m) { m.clearAndInflateLocalMap(); m.updateESDF3d(); } };
}

namespace cane_planner
{
// Fixture access initializes domain state without a ROS parameter server. All
// decisions below run the production plan(), sampling, LFPC and map queries.
class StaticMpcTestAccess
{
public:
    static LFPC::Ptr model()
    {
        auto m = std::make_shared<LFPC>();
        m->t_sup_ = 0.35;
        m->delta_t_ = 0.07;
        m->h_ = 1.0;
        m->t_c_ = std::sqrt(m->h_ / 10.0);
        m->b_ = m->t_c_ / std::tanh(m->t_sup_ / m->t_c_);
        m->reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), LEFT_LEG, 0);
        return m;
    }

    static CollisionDetection::Ptr map(bool block_foot = false, bool block_com = false)
    {
        auto c = std::make_shared<CollisionDetection>();
        c->static_global_map_ready_ = true;
        c->static_map_resolution_ = 0.005;
        c->static_map_inv_resolution_ = 200.0;
        c->static_origin_ = Eigen::Vector2d(-2.0, -2.0);
        c->static_size_ = Eigen::Vector2i(800, 800);
        c->static_require_known_region_ = false;
        c->static_inflated_.assign(800 * 800, 0);
        for (int x = 0; x < 800; ++x)
            for (int y = 0; y < 800; ++y)
            {
                const Eigen::Vector2d p = c->static_origin_ +
                    0.005 * Eigen::Vector2d(x + 0.5, y + 0.5);
                if ((block_foot && (p - Eigen::Vector2d(-0.25, 0.0)).norm() < 0.045) ||
                    (block_com && p.x() > 0.01))
                    c->static_inflated_[c->staticToAddress(Eigen::Vector2i(x, y))] = 1;
            }
        return c;
    }

    static void configure(MpcController &c)
    {
        c.cfg_.horizon_steps = 1;
        c.cfg_.nominal_al = 0.25;
        c.cfg_.max_api = 3.14159265358979323846/6.;
        c.cfg_.num_samples = 64;c.cfg_.mpc_iters = 2;
        c.rng_.seed(17);c.normal_dist_.reset();
        c.cfg_.w_goal = c.cfg_.w_steer = c.cfg_.w_dapi = 0.0;
    }

    static void horizon(MpcController& c,int horizon) {c.cfg_.horizon_steps=horizon;c.cfg_.w_goal=10.;c.cfg_.goal_arrival_threshold=0.;}
    static void sampling(MpcController& c,int count,int iterations,double sigma=.15) {c.cfg_.num_samples=count;c.cfg_.mpc_iters=iterations;c.cfg_.sigma_api=sigma;}
    static void seed(MpcController& c,int value) {c.rng_.seed(value);c.normal_dist_.reset();}
    static const std::vector<double>& warm(MpcController& c) {return c.warm_start_;}
    static std::vector<double> weights(MpcController& c,const std::vector<double>& costs) {return c.computeWeights(costs);}
    static double rolloutCost(MpcController& c,const LFPC& base,const std::vector<double>& u,const Eigen::Vector3d& goal) {return c.rolloutSequence(base,u,goal,nullptr,0.).cost;}
    static auto samples(MpcController& c,const std::vector<double>& mean,const std::vector<double>& incumbent) {return c.sampleSequences(mean,incumbent);}
    static void temperature(MpcController& c,double value) {c.cfg_.temperature=value;}
    static void coldStartBudget(MpcController& c) {
        c.cfg_.horizon_steps=10;c.cfg_.num_samples=256;c.cfg_.mpc_iters=2;
        c.cfg_.nominal_al=.4;c.cfg_.w_goal=25.;c.cfg_.w_steer=.5;c.cfg_.w_dapi=.2;
        c.cfg_.goal_arrival_threshold=.2;seed(c,0);
    }
    static void timedEnable(MpcController& c,bool value) {c.cfg_.timed_collision_enable=value;}
    static void footprint(MpcController& c,bool value) {c.cfg_.footprint.enabled=value;}
    static void angleLimit(MpcController& c,double limit) {c.cfg_.max_api=limit;}
    static CollisionDetection::Ptr wideMap() {auto c=map();c->static_origin_=Eigen::Vector2d(-10,-10);c->static_map_resolution_=.025;c->static_map_inv_resolution_=40.;return c;}
    static void suffixCost(MpcController& c) {c.cfg_.horizon_steps=10;c.cfg_.w_steer=.5;c.cfg_.w_dapi=.7;c.cfg_.w_goal=10.;c.cfg_.goal_arrival_threshold=.2;}
    static void steeringCost(MpcController &c) { c.cfg_.w_steer = 0.5; }

    static void corridorGrid(CollisionDetection& c)
    {
        c.static_global_map_ready_ = true;
        c.static_global_esdf_ready_ = false;
        c.static_origin_ = Eigen::Vector2d::Zero();
        c.static_size_ = Eigen::Vector2i(4,4);
        c.static_map_resolution_ = .25;
        c.static_map_inv_resolution_ = 4.;
        c.static_inflated_.assign(16,0);
        c.static_known_inflated_.assign(16,1);
        c.static_inflated_[c.staticToAddress(Eigen::Vector2i(1,1))] = 1;
        c.static_known_inflated_[c.staticToAddress(Eigen::Vector2i(2,1))] = 0;
        c.static_require_known_region_ = false; // Preserve the backend's optional known-region policy.
        Eigen::Vector2d origin; Eigen::Vector2i size; double res;
        std::vector<uint8_t> blocked; std::string reason;
        ASSERT_TRUE(c.getStaticCorridorGrid(Eigen::Vector2d(-1,-1),Eigen::Vector2d(2,2),origin,res,size,blocked,reason));
        EXPECT_TRUE(origin.isZero()); EXPECT_EQ(size,Eigen::Vector2i(4,4));
        EXPECT_DOUBLE_EQ(res,.25); EXPECT_EQ(blocked[1*4+1],1); EXPECT_EQ(blocked[1*4+2],0); EXPECT_EQ(blocked[0],0);
        c.static_require_known_region_ = true;
        ASSERT_TRUE(c.getStaticCorridorGrid(Eigen::Vector2d(0,0),Eigen::Vector2d(1,1),origin,res,size,blocked,reason));
        EXPECT_EQ(blocked[1*4+2],1);
        c.static_global_esdf_ready_ = true;
        c.global_esdf_origin_ = Eigen::Vector3d(0,0,-1);
        c.global_esdf_size_ = Eigen::Vector3i(4,4,12);
        c.global_esdf_resolution_ = .25; c.global_esdf_inv_resolution_ = 4.;
        c.global_esdf_distance_.assign(4*4*12,2.);
        for(int z=0;z<12;++z)c.global_esdf_distance_[c.globalToAddress(Eigen::Vector3i(3,2,z))]=0.;
        ASSERT_TRUE(c.getStaticCorridorGrid(Eigen::Vector2d(0,0),Eigen::Vector2d(1,1),origin,res,size,blocked,reason));
        for(int y=0;y<4;++y)for(int x=0;x<4;++x)
            EXPECT_EQ(blocked[y*4+x],!c.isStaticTraversable((x+.5)*.25,(y+.5)*.25));
        EXPECT_EQ(blocked[2*4+3],1); EXPECT_EQ(blocked[1*4+1],0); // ESDF precedence
        c.static_global_esdf_ready_ = c.static_global_map_ready_ = false;
        EXPECT_FALSE(c.getStaticCorridorGrid(Eigen::Vector2d(0,0),Eigen::Vector2d(1,1),origin,res,size,blocked,reason));
        EXPECT_EQ(reason,"MAP_UNAVAILABLE");
    }

    static void localProducer(CollisionDetection& c)
    {
        ros::NodeHandle nh("/firi_local_producer_test");
        nh.setParam("sdf_map/resolution", .1);
        nh.setParam("sdf_map/map_size_x", 6.); nh.setParam("sdf_map/map_size_y", 6.); nh.setParam("sdf_map/map_size_z", 4.);
        nh.setParam("sdf_map/ground_height", -1.);
        nh.setParam("sdf_map/local_update_range_x", 5.5); nh.setParam("sdf_map/local_update_range_y", 5.5); nh.setParam("sdf_map/local_update_range_z", 4.5);
        nh.setParam("sdf_map/max_ray_length", 5.); nh.setParam("sdf_map/input_min_height", .15); nh.setParam("sdf_map/input_max_height", 2.6);
        nh.setParam("sdf_map/obstacles_inflation", .1); nh.setParam("map_ros/is_simulation", false);
        nh.setParam("map_ros/skip_pixel", 2);
        c.sdf_map_.reset(new SDFMap); c.sdf_map_->initMap(nh);
        c.static_global_map_ready_=c.static_global_esdf_ready_=false;
        c.slice_height_=0.; c.margin_=.1;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.push_back(pcl::PointXYZ(1.05,.05,.45));
        cloud.push_back(pcl::PointXYZ(1.05,1.05,.05)); // filtered before ray fusion
        for(int i=0;i<3;++i)c.sdf_map_->inputPointCloud(cloud,cloud.size(),Eigen::Vector3d(.05,.05,.45));
        fast_planner::SdfProducerTestAccess::update(*c.sdf_map_);
        EXPECT_EQ(c.sdf_map_->getOccupancy(Eigen::Vector3d(1.05,1.05,.05)),SDFMap::UNKNOWN);
        EXPECT_EQ(c.sdf_map_->getOccupancy(Eigen::Vector3d(1.05,.05,.45)),SDFMap::OCCUPIED);
        EXPECT_FALSE(c.isTraversable(1.05,.05));
        EXPECT_TRUE(c.isTraversable(-1.05,-1.05)); // existing optimistic unknown semantics
        Eigen::Vector2d origin; Eigen::Vector2i size; double res;std::vector<uint8_t> blocked;std::string reason;
        ASSERT_TRUE(c.getStaticCorridorGrid(Eigen::Vector2d(-2,-2),Eigen::Vector2d(2,2),origin,res,size,blocked,reason));
        int free_count=0;
        for(int y=0;y<size.y();++y)for(int x=0;x<size.x();++x){
            Eigen::Vector2d p=origin+res*Eigen::Vector2d(x+.5,y+.5);
            EXPECT_EQ(blocked[y*size.x()+x],!c.isTraversable(p.x(),p.y()));
            if(!blocked[y*size.x()+x])++free_count;
        }
        EXPECT_GT(free_count,0);
        EXPECT_FALSE(c.getStaticCorridorGrid(Eigen::Vector2d(20,20),Eigen::Vector2d(21,21),origin,res,size,blocked,reason));
    }

    static void resolutionBackends(CollisionDetection &c)
    {
        c.static_global_map_ready_ = true;
        c.static_map_resolution_ = 0.08;
        EXPECT_DOUBLE_EQ(c.getStaticQueryResolution(), 0.08);
        c.static_global_esdf_ready_ = true;
        c.global_esdf_resolution_ = 0.12;
        EXPECT_DOUBLE_EQ(c.getStaticQueryResolution(), 0.12);
    }
};
}

using namespace cane_planner;

static ConvexCorridor::Segment box(double xmin = -1.0, double xmax = 1.0,
                                   double ymin = -1.0, double ymax = 1.0)
{
    ConvexCorridor::Segment s;
    s.center = Eigen::Vector2d((xmin + xmax) / 2.0, (ymin + ymax) / 2.0);
    s.static_feasible = true;
    s.halfspaces = {{Eigen::Vector2d(1, 0), xmax}, {Eigen::Vector2d(-1, 0), -xmin},
                    {Eigen::Vector2d(0, 1), ymax}, {Eigen::Vector2d(0, -1), -ymin}};
    return s;
}
TEST(FootprintMpcRuntime, CaneOutsideCorridorRejectsAlthoughCenterFits) {
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::footprint(c,true);
    c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box(-1,.6)});
    auto model=StaticMpcTestAccess::model();c.plan(model,Eigen::Vector3d(2,0,0));
    EXPECT_FALSE(c.lastPlanValid());EXPECT_TRUE(c.getBestPath().empty());
    c.setConvexCorridor({box()});c.plan(model,Eigen::Vector3d(2,0,0));EXPECT_TRUE(c.lastPlanValid());
    EXPECT_EQ(c.getBestPath().size(),c.getBestYaws().size());EXPECT_EQ(c.getBestTimes().size(),c.getBestYaws().size());
}
TEST(FootprintMpcRuntime, CaneOnlyTimedCollisionAndExecutionAge) {
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::footprint(c,true);
    StaticMpcTestAccess::angleLimit(c,0.);c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box()});
    const TimedPedestrianCollision::Polygon obstacle{{.7,.49},{.74,.49},{.74,.53},{.7,.53}};
    ASSERT_TRUE(c.setTimedPedestrians({{obstacle,Eigen::Vector2d(0,-1)}},0.));
    c.plan(StaticMpcTestAccess::model(),Eigen::Vector3d(2,0,0));ASSERT_TRUE(c.lastPlanValid());
    EXPECT_FALSE(c.revalidateTimedPath(.5));EXPECT_FALSE(c.lastPlanValid());
    EXPECT_TRUE(c.collisionWitness().valid);EXPECT_NEAR(c.collisionWitness().com.norm(),0.,1e-12);
}

static void expectExecutedPath(MpcController &c, const LFPC::Ptr &m,
                               const Eigen::Vector3d &command)
{
    LFPC replay;
    replay.copyState(*m);
    replay.SetCtrlParams(command);
    replay.updateOneStep();
    const auto expected = replay.getStepCOMPath();
    const auto actual = c.getBestPath();
    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_FALSE(actual.empty());
    for (size_t i = 0; i < actual.size(); ++i)
        EXPECT_NEAR((actual[i] - expected[i]).norm(), 0.0, 1e-12);
}

TEST(StaticMpcRuntime, MissingInvalidAndClearedCorridorStop)
{
    MpcController c;
    StaticMpcTestAccess::configure(c);
    c.setCollision(StaticMpcTestAccess::map());
    const auto m = StaticMpcTestAccess::model();
    const Eigen::Vector3d goal(2, 0, 0);
    EXPECT_TRUE(c.plan(m, goal).isZero());
    EXPECT_FALSE(c.lastPlanValid());
    auto invalid = box();
    invalid.static_feasible = false;
    c.setConvexCorridor({invalid});
    EXPECT_TRUE(c.plan(m, goal).isZero());
    EXPECT_FALSE(c.lastPlanValid());
    c.setConvexCorridor({box()});
    c.plan(m, goal);
    ASSERT_TRUE(c.lastPlanValid());
    c.clearConvexCorridor();
    EXPECT_TRUE(c.plan(m, goal).isZero());
    EXPECT_FALSE(c.lastPlanValid());
    EXPECT_TRUE(c.getBestPath().empty());
}

TEST(StaticMpcRuntime, WeightedCommandMatchesExecutedAction)
{
    MpcController c;StaticMpcTestAccess::configure(c);
    c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box()});
    auto m=StaticMpcTestAccess::model();const auto command=c.plan(m,Eigen::Vector3d(2,0,0));
    ASSERT_TRUE(c.lastPlanValid());const auto d=c.getDebugMetrics();
    EXPECT_TRUE(d.weighted_checked);EXPECT_FALSE(d.weighted_fallback);
    EXPECT_EQ(128,d.valid_trajectory_count);EXPECT_EQ(128,d.num_samples);
    EXPECT_DOUBLE_EQ(1.,d.valid_sample_ratio);EXPECT_DOUBLE_EQ(0.,d.executed_total_cost);
    expectExecutedPath(c,m,command);
}

TEST(StaticMpcRuntime, WeightedFootCollisionFallsBackToVerifiedSampleReproducibly)
{
    MpcController a,b;auto m=StaticMpcTestAccess::model();auto collision=StaticMpcTestAccess::map(true);
    for(auto* c:{&a,&b}) {
        StaticMpcTestAccess::configure(*c);StaticMpcTestAccess::sampling(*c,64,1,.35);
        c->setCollision(collision);c->setConvexCorridor({box()});
    }
    const auto expected=a.plan(m,Eigen::Vector3d(2,0,0));const auto actual=b.plan(m,Eigen::Vector3d(2,0,0));
    ASSERT_TRUE(a.lastPlanValid());ASSERT_TRUE(b.lastPlanValid());EXPECT_EQ(expected,actual);
    EXPECT_TRUE(a.getDebugMetrics().weighted_checked);EXPECT_TRUE(a.getDebugMetrics().weighted_fallback);
    EXPECT_EQ(1,a.optimizationEvidence().weighted_rejects);
    EXPECT_GT(a.getDebugMetrics().static_reject_count,0);expectExecutedPath(a,m,expected);
}

TEST(StaticMpcRuntime, WeightedMeanAcrossNonconvexUnionIsRechecked)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::sampling(c,64,1,.35);
    c.setCollision(StaticMpcTestAccess::map());
    c.setConvexCorridor({box(-1,.025),box(-1,1,.006,1),box(-1,1,-1,-.006)});
    auto m=StaticMpcTestAccess::model();const auto command=c.plan(m,Eigen::Vector3d(2,0,0));
    ASSERT_TRUE(c.lastPlanValid());EXPECT_TRUE(c.getDebugMetrics().weighted_fallback);
    EXPECT_EQ(1,c.optimizationEvidence().weighted_rejects);
    EXPECT_GT(c.getDebugMetrics().convex_corridor_reject_count,0);
    expectExecutedPath(c,m,command);
}

TEST(StaticMpcRuntime, EverySampleOutsideUnionOrInStaticObstacleStops)
{
    for(bool static_obstacle:{false,true}) {
        MpcController c;StaticMpcTestAccess::configure(c);
        c.setCollision(StaticMpcTestAccess::map(false,static_obstacle));
        c.setConvexCorridor({static_obstacle?box():box(-.001,.001)});
        EXPECT_TRUE(c.plan(StaticMpcTestAccess::model(),Eigen::Vector3d(2,0,0)).isZero());
        EXPECT_FALSE(c.lastPlanValid());EXPECT_FALSE(c.getDebugMetrics().weighted_checked);
        EXPECT_EQ(0,c.getDebugMetrics().valid_trajectory_count);
        EXPECT_TRUE(c.getBestPath().empty());EXPECT_TRUE(StaticMpcTestAccess::warm(c).empty());
    }
}

TEST(StaticMpcRuntime, ContinuousOptimizationImprovesCostAndReplaysFullState)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::horizon(c,5);
    StaticMpcTestAccess::sampling(c,256,3,.25);
    c.setCollision(StaticMpcTestAccess::wideMap());c.setConvexCorridor({box(-9,9,-9,9)});
    auto model=StaticMpcTestAccess::model();const auto state=model->getState();const auto pos=model->getCOMPos();
    const Eigen::Vector3d goal(1,1,0);
    const double initial=StaticMpcTestAccess::rolloutCost(c,*model,std::vector<double>(5,0.),goal);
    const auto command=c.plan(model,goal);ASSERT_TRUE(c.lastPlanValid());
    EXPECT_EQ(state,model->getState());EXPECT_EQ(pos,model->getCOMPos());
    EXPECT_TRUE(c.getDebugMetrics().weighted_checked);EXPECT_FALSE(c.getDebugMetrics().weighted_fallback);
    EXPECT_LT(c.getDebugMetrics().executed_total_cost,initial);
    const auto& e=c.optimizationEvidence();EXPECT_EQ(3,e.iterations);EXPECT_EQ(768,e.evaluated);
    ASSERT_EQ(3u,e.best_verified_costs.size());
    for(size_t i=1;i<e.best_verified_costs.size();++i)EXPECT_LE(e.best_verified_costs[i],e.best_verified_costs[i-1]);
    const auto& controls=c.getBestControls();ASSERT_EQ(5u,controls.size());
    EXPECT_DOUBLE_EQ(command.z(),controls.front());
    EXPECT_GT(std::abs(command.z()/(3.14159265358979323846/12.)-std::round(command.z()/(3.14159265358979323846/12.))),1e-4);
    LFPC replay;replay.copyState(*model);std::vector<Eigen::Vector3d> path;
    for(double action:controls) {
        EXPECT_LE(std::abs(action),3.14159265358979323846/6.);
        replay.SetCtrlParams(Eigen::Vector3d(command.x(),command.y(),action));replay.updateOneStep();
        const auto step=replay.getStepCOMPath();path.insert(path.end(),step.begin(),step.end());replay.prepareNextStep();
    }
    ASSERT_EQ(path.size(),c.getBestPath().size());
    for(size_t i=0;i<path.size();++i)EXPECT_NEAR(0.,(path[i]-c.getBestPath()[i]).norm(),1e-12);
    EXPECT_DOUBLE_EQ(c.getDebugMetrics().executed_total_cost,StaticMpcTestAccess::rolloutCost(c,*model,controls,goal));
    EXPECT_TRUE(e.reroll_valid);
}

TEST(StaticMpcRuntime, StableWeightsExcludeAllUnsafeInputs)
{
    MpcController c;const double inf=std::numeric_limits<double>::infinity();
    const auto w=StaticMpcTestAccess::weights(c,{inf,1e9,1e9+.4,NAN});
    EXPECT_DOUBLE_EQ(0.,w[0]);EXPECT_DOUBLE_EQ(0.,w[3]);
    EXPECT_NEAR(1.,w[1]+w[2],1e-12);EXPECT_NEAR(std::exp(-1.),w[2]/w[1],1e-6);
    EXPECT_EQ(std::vector<double>({0.,0.}),StaticMpcTestAccess::weights(c,{inf,NAN}));
}

TEST(StaticMpcRuntime, ColdStartRightAngleTurnFindsFullFeasibleRollout)
{
    MpcController c;StaticMpcTestAccess::coldStartBudget(c);
    c.setCollision(StaticMpcTestAccess::wideMap());
    // Robot initially faces +X, but the long corridor runs along +Y. A zero
    // proposal is unsafe; default exploration must find a full turning sequence.
    c.setConvexCorridor({box(-.9,.9,-.5,6.)});
    auto model=StaticMpcTestAccess::model();
    const auto command=c.plan(model,Eigen::Vector3d(0,3,0));
    ASSERT_TRUE(c.lastPlanValid());EXPECT_GT(command.z(),0.);
    EXPECT_EQ(10,c.optimizationEvidence().selected_steps);
    EXPECT_EQ(512,c.optimizationEvidence().evaluated);
    EXPECT_TRUE(c.optimizationEvidence().reroll_valid);
    EXPECT_GT(c.getDebugMetrics().convex_corridor_reject_count,0);
    for(const auto& p:c.getBestPath()) {
        EXPECT_GE(p.x(),-.900001);EXPECT_LE(p.x(),.900001);
        EXPECT_GE(p.y(),-.500001);EXPECT_LE(p.y(),6.000001);
    }
}

TEST(StaticMpcRuntime, WarmStartShiftsAndRetainsCurrentCycleCandidate)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::horizon(c,4);
    c.setCollision(StaticMpcTestAccess::wideMap());c.setConvexCorridor({box(-9,9,-9,9)});
    auto model=StaticMpcTestAccess::model();c.plan(model,Eigen::Vector3d(1,1,0));ASSERT_TRUE(c.lastPlanValid());
    const auto controls=c.getBestControls();const auto warm=StaticMpcTestAccess::warm(c);
    ASSERT_EQ(4u,warm.size());for(size_t i=0;i+1<warm.size();++i)EXPECT_DOUBLE_EQ(controls[i+1],warm[i]);
    EXPECT_DOUBLE_EQ(0.,warm.back());
    const auto sampled=StaticMpcTestAccess::samples(c,warm,controls);
    EXPECT_EQ(warm,sampled[0]);EXPECT_EQ(controls,sampled[1]);
    c.resetWarmStart();EXPECT_TRUE(StaticMpcTestAccess::warm(c).empty());
}

TEST(StaticMpcRuntime, InvalidSamplingConfigFailsClosed)
{
    for(int kind=0;kind<5;++kind) {
        MpcController c;StaticMpcTestAccess::configure(c);c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box()});
        if(kind==0)StaticMpcTestAccess::horizon(c,33);
        if(kind==1)StaticMpcTestAccess::sampling(c,0,2);
        if(kind==2)StaticMpcTestAccess::sampling(c,64,0);
        if(kind==3)StaticMpcTestAccess::sampling(c,64,2,NAN);
        if(kind==4)StaticMpcTestAccess::temperature(c,0.);
        EXPECT_TRUE(c.plan(StaticMpcTestAccess::model(),Eigen::Vector3d(2,0,0)).isZero());
        EXPECT_FALSE(c.lastPlanValid());EXPECT_TRUE(StaticMpcTestAccess::warm(c).empty());
    }
}

TEST(StaticMpcRuntime, ActionLimitsAndNoFullContinuation)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::horizon(c,4);StaticMpcTestAccess::angleLimit(c,0.);
    c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box(-.1,.2,-1,1)});
    EXPECT_TRUE(c.plan(StaticMpcTestAccess::model(),Eigen::Vector3d(3,0,0)).isZero());
    EXPECT_FALSE(c.lastPlanValid());EXPECT_EQ(0,c.getDebugMetrics().valid_trajectory_count);
}

TEST(StaticMpcRuntime, FinalArrivalIsSafeTerminalButLocalGoalIsNot)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::horizon(c,10);StaticMpcTestAccess::steeringCost(c);
    c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box(-.5,.7,-.3,.3)});
    auto model=StaticMpcTestAccess::model();const Eigen::Vector2d final(.30,0);
    c.plan(model,Eigen::Vector3d(.30,0,0));EXPECT_FALSE(c.lastPlanValid());
    const auto cmd=c.plan(model,Eigen::Vector3d(.30,0,0),&final,.2);
    ASSERT_TRUE(c.lastPlanValid());EXPECT_TRUE(c.optimizationEvidence().selected_terminal);
    EXPECT_LT(c.optimizationEvidence().selected_steps,10);EXPECT_TRUE(c.optimizationEvidence().reroll_valid);
    LFPC state;state.copyState(*model);state.SetCtrlParams(cmd);state.updateOneStep();
    EXPECT_LT((state.getCOMPos().head<2>()-final).norm(),.2);
    c.setConvexCorridor({box(-.001,.001,-.3,.3)});
    EXPECT_TRUE(c.plan(model,Eigen::Vector3d(.30,0,0),&final,.2).isZero());EXPECT_FALSE(c.lastPlanValid());
}

TEST(StaticMpcRuntime, LocalArrivalStillChargesEveryWalkingSuffixEdge)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::suffixCost(c);
    c.setCollision(StaticMpcTestAccess::wideMap());c.setConvexCorridor({box(-9,9,-9,9)});
    auto model=StaticMpcTestAccess::model();
    const auto command=c.plan(model,Eigen::Vector3d(.30,0,0));ASSERT_TRUE(c.lastPlanValid());
    EXPECT_EQ(10,c.optimizationEvidence().selected_steps);EXPECT_FALSE(c.optimizationEvidence().selected_terminal);
    EXPECT_DOUBLE_EQ(0.,command.z());EXPECT_DOUBLE_EQ(0.,c.getDebugMetrics().executed_total_cost);
    std::vector<double> turns(10,0.);turns[1]=.2;
    EXPECT_NEAR(.5*.2+.7*.4,StaticMpcTestAccess::rolloutCost(c,*model,turns,Eigen::Vector3d(.30,0,0)),1e-12);
    const Eigen::Vector2d final(.30,0);
    c.plan(model,Eigen::Vector3d(.30,0,0),&final,.2);ASSERT_TRUE(c.lastPlanValid());
    EXPECT_TRUE(c.optimizationEvidence().selected_terminal);EXPECT_EQ(1,c.optimizationEvidence().selected_steps);
}
TEST(StaticMpcRuntime, QueryResolutionTracksSelectedStaticBackend)
{
    auto c = StaticMpcTestAccess::map();
    StaticMpcTestAccess::resolutionBackends(*c);
}

TEST(StaticMpcRuntime, LocalProducerMatchesDefaultHeightFilterAndModel)
{
    auto c=StaticMpcTestAccess::map();
    StaticMpcTestAccess::localProducer(*c);
}

TEST(StaticMpcRuntime, TriangularCorridorAccepted)
{
    MpcController c;
    StaticMpcTestAccess::configure(c);
    c.setCollision(StaticMpcTestAccess::map());
    auto triangle = box();
    triangle.halfspaces = {{Eigen::Vector2d(-1,0),1.},
        {Eigen::Vector2d(1,1).normalized(),2.},
        {Eigen::Vector2d(1,-1).normalized(),2.}};
    c.setConvexCorridor({triangle});
    c.plan(StaticMpcTestAccess::model(), Eigen::Vector3d(2,0,0));
    EXPECT_TRUE(c.lastPlanValid());
}

TEST(StaticMpcRuntime, CorridorGridPreservesSelectedModelSemantics)
{
    auto c = StaticMpcTestAccess::map();
    StaticMpcTestAccess::corridorGrid(*c);
}

static TimedPedestrianCollision::Obstacle movingBox(const Eigen::Vector2d& p,const Eigen::Vector2d& v) {
    const double r=.02;
    return {{{p.x()-r,p.y()-r},{p.x()+r,p.y()-r},{p.x()+r,p.y()+r},{p.x()-r,p.y()+r}},v};
}

TEST(TimedMpcRuntime, ActualSampleTimesAcrossStepsAndCopies) {
    auto model=StaticMpcTestAccess::model();
    model->SetCtrlParams(Eigen::Vector3d(.25,0,0));model->updateOneStep();
    const auto phases=model->getStepCOMTimes();
    ASSERT_EQ(model->getStepCOMPath().size(),phases.size());ASSERT_FALSE(phases.empty());
    for(size_t i=0;i<phases.size();++i)EXPECT_NEAR((i+1)*.07,phases[i],1e-12);
    LFPC copy;copy.copyState(*model);EXPECT_EQ(phases,copy.getStepCOMTimes());
    model->prepareNextStep();EXPECT_TRUE(model->getStepCOMTimes().empty());
    model->reset(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),LEFT_LEG,0);
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::horizon(c,3);
    StaticMpcTestAccess::angleLimit(c,0.);c.setCollision(StaticMpcTestAccess::wideMap());
    c.setConvexCorridor({box(-9,9,-9,9)});c.plan(model,Eigen::Vector3d(5,0,0));
    ASSERT_TRUE(c.lastPlanValid());const auto& times=c.getBestTimes();
    ASSERT_EQ(phases.size()*3,times.size());ASSERT_EQ(c.getBestPath().size(),times.size());
    for(size_t i=0;i<times.size();++i)EXPECT_NEAR((i+1)*.07,times[i],1e-12);
    EXPECT_EQ(3,c.optimizationEvidence().selected_steps);
}

TEST(TimedMpcRuntime, CrossingBeyondFirstStepRejectsFullContinuation) {
    auto m=StaticMpcTestAccess::model();MpcController c;
    StaticMpcTestAccess::configure(c);StaticMpcTestAccess::horizon(c,3);StaticMpcTestAccess::angleLimit(c,0.);
    c.setCollision(StaticMpcTestAccess::wideMap());c.setConvexCorridor({box(-9,9,-9,9)});
    const auto command=c.plan(m,Eigen::Vector3d(5,0,0));ASSERT_TRUE(c.lastPlanValid());
    const auto target=c.getBestPath().back().head<2>().eval();const double time=c.getBestTimes().back();
    m->updateOneStep();const double first_duration=m->getStepCOMTimes().back();
    m=StaticMpcTestAccess::model();
    ASSERT_TRUE(c.setTimedPedestrians({movingBox(target+Eigen::Vector2d(0,time),Eigen::Vector2d(0,-1))},0.));
    c.plan(m,Eigen::Vector3d(5,0,0));EXPECT_FALSE(c.lastPlanValid());
    ASSERT_TRUE(c.collisionWitness().valid);EXPECT_GT(c.collisionWitness().future_time,first_duration);
    EXPECT_GT(c.getDebugMetrics().dynamic_reject_count,0);
    EXPECT_GT(c.getDebugMetrics().timed_collision_checks,0);
    EXPECT_TRUE(c.getBestPath().empty());EXPECT_TRUE(c.getBestTimes().empty());
    // Fresh empty data clears prediction; static baseline command is unchanged.
    ASSERT_TRUE(c.setTimedPedestrians({},0.));
    EXPECT_EQ(command,c.plan(m,Eigen::Vector3d(5,0,0)));EXPECT_TRUE(c.lastPlanValid());
    EXPECT_FALSE(c.getDebugMetrics().timed_collision_active);
}

TEST(TimedMpcRuntime, ExecutionEpochRecheckCatchesLatencyWithoutChangingModel) {
    auto m=StaticMpcTestAccess::model();MpcController c;StaticMpcTestAccess::configure(c);
    StaticMpcTestAccess::angleLimit(c,0.);c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box()});
    ASSERT_TRUE(c.setTimedPedestrians({movingBox(Eigen::Vector2d(0,1),Eigen::Vector2d(0,-2))},0.));
    c.plan(m,Eigen::Vector3d(2,0,0));ASSERT_TRUE(c.lastPlanValid());
    const auto pos=m->getCOMPos();const auto state=m->getState();
    EXPECT_FALSE(c.revalidateTimedPath(.2));EXPECT_FALSE(c.lastPlanValid());
    EXPECT_TRUE(c.getDebugMetrics().timed_execution_checked);EXPECT_FALSE(c.getDebugMetrics().timed_execution_safe);
    EXPECT_EQ(pos,m->getCOMPos());EXPECT_EQ(state,m->getState());
    ASSERT_TRUE(c.collisionWitness().valid);EXPECT_DOUBLE_EQ(.2,c.collisionWitness().observation_age);
}

TEST(TimedMpcRuntime, InvalidBindingFailsClosedAndExplicitAblationRestoresBaseline) {
    auto m=StaticMpcTestAccess::model();MpcController c;StaticMpcTestAccess::configure(c);
    c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box()});
    EXPECT_FALSE(c.setTimedPedestrians({movingBox(Eigen::Vector2d(0,0),Eigen::Vector2d::Zero())},-1.));
    c.plan(m,Eigen::Vector3d(2,0,0));EXPECT_FALSE(c.lastPlanValid());
    StaticMpcTestAccess::timedEnable(c,false);
    EXPECT_TRUE(c.setTimedPedestrians({movingBox(Eigen::Vector2d(0,0),Eigen::Vector2d::Zero())},0.));
    c.plan(m,Eigen::Vector3d(2,0,0));EXPECT_TRUE(c.lastPlanValid());
}

TEST(FootprintMpcRuntime, DISABLED_BenchmarkSameBudget) {
    auto m=StaticMpcTestAccess::model();MpcController c;StaticMpcTestAccess::coldStartBudget(c);
    c.setCollision(StaticMpcTestAccess::wideMap());c.setConvexCorridor({box(-9,9,-9,9)});
    std::array<std::vector<double>,2> timings;
    for(int repeat=0;repeat<55;++repeat)for(int mode=0;mode<2;++mode) {
        StaticMpcTestAccess::footprint(c,mode);c.resetWarmStart();StaticMpcTestAccess::seed(c,0);
        ASSERT_TRUE(c.setTimedPedestrians({movingBox(Eigen::Vector2d(6,6),Eigen::Vector2d(.1,0))},.1));
        c.plan(m,Eigen::Vector3d(5,0,0));ASSERT_TRUE(c.lastPlanValid());
        if(repeat>=5)timings[mode].push_back(c.lastPlanTimeMs());
    }
    for(int mode=0;mode<2;++mode) {
        auto& t=timings[mode];std::sort(t.begin(),t.end());
        std::cout<<"FOOTPRINT_BENCH enabled="<<mode<<" median_ms="<<t[t.size()/2]<<" p95_ms="<<t[47]<<std::endl;
    }
}

// Opt-in measured benchmark; same seed and sample budget, no timing assertion.
TEST(TimedMpcRuntime, DISABLED_BenchmarkSameMppiWork) {
    auto m=StaticMpcTestAccess::model();MpcController c;StaticMpcTestAccess::coldStartBudget(c);
    c.setCollision(StaticMpcTestAccess::wideMap());c.setConvexCorridor({box(-9,9,-9,9)});
    std::array<std::vector<double>,2> timings;
    for(int repeat=0;repeat<105;++repeat)for(int mode=0;mode<2;++mode) {
        c.resetWarmStart();StaticMpcTestAccess::seed(c,0);
        if(mode)c.setTimedPedestrians({movingBox(Eigen::Vector2d(6,6),Eigen::Vector2d(.1,0))},.1);
        else c.clearTimedPedestrians();
        c.plan(m,Eigen::Vector3d(5,0,0));ASSERT_TRUE(c.lastPlanValid());
        if(repeat>=5)timings[mode].push_back(c.lastPlanTimeMs());
    }
    for(int mode=0;mode<2;++mode) {
        auto& t=timings[mode];std::sort(t.begin(),t.end());
        std::cout << "TIMED_BENCH mode="<<mode<<" median_ms="<<t[t.size()/2]<<" p95_ms="<<t[95]<<std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_static_mpc", ros::init_options::AnonymousName |
              ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
