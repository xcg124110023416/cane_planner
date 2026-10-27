#include <gtest/gtest.h>
#include <chrono>
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

    static void configure(MpcController &c, bool = false)
    {
        c.cfg_.horizon_steps = 1;
        c.cfg_.nominal_al = 0.25;
        c.cfg_.max_api = 0.7854;
        c.cfg_.w_goal = c.cfg_.w_steer = c.cfg_.w_dapi = 0.0;
    }

    static void beam(MpcController& c,int horizon,int width) {c.cfg_.horizon_steps=horizon;c.cfg_.beam_width=width;c.cfg_.w_goal=10.;c.cfg_.goal_arrival_threshold=0.;}
    static void timedEnable(MpcController& c,bool value) {c.cfg_.timed_collision_enable=value;}
    static void angleLimit(MpcController& c,double limit) {c.cfg_.max_api=limit;}
    static CollisionDetection::Ptr wideMap() {auto c=map();c->static_origin_=Eigen::Vector2d(-10,-10);c->static_map_resolution_=.025;c->static_map_inv_resolution_=40.;return c;}
    static void suffixCost(MpcController& c) {c.cfg_.horizon_steps=10;c.cfg_.beam_width=8;c.cfg_.w_steer=.5;c.cfg_.w_dapi=.7;c.cfg_.w_goal=10.;c.cfg_.goal_arrival_threshold=.2;}
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

TEST(StaticMpcRuntime, DiscreteSelectedPathMatchesExecutedAction)
{
    MpcController c;
    StaticMpcTestAccess::configure(c);
    c.setCollision(StaticMpcTestAccess::map());
    c.setConvexCorridor({box()});
    auto m = StaticMpcTestAccess::model();
    const auto command = c.plan(m, Eigen::Vector3d(2, 0, 0));
    ASSERT_TRUE(c.lastPlanValid());
    const auto d = c.getDebugMetrics();
    EXPECT_FALSE(d.weighted_checked);
    EXPECT_FALSE(d.weighted_fallback);
    EXPECT_EQ(d.valid_trajectory_count, 5);
    EXPECT_EQ(d.num_samples, 5);
    EXPECT_DOUBLE_EQ(d.executed_total_cost, 0.0);
    expectExecutedPath(c, m, command);
}

TEST(StaticMpcRuntime, DiscreteFootCollisionRejectsAndSelectionIsDeterministic)
{
    MpcController weighted, best;
    StaticMpcTestAccess::configure(weighted);
    StaticMpcTestAccess::configure(best, true);
    auto collision = StaticMpcTestAccess::map(true);
    auto m = StaticMpcTestAccess::model();
    for (auto *c : {&weighted, &best})
    {
        c->setCollision(collision);
        c->setConvexCorridor({box()});
    }
    const Eigen::Vector3d goal(2, 0, 0);
    const auto expected = best.plan(m, goal);
    const auto actual = weighted.plan(m, goal);
    ASSERT_TRUE(best.lastPlanValid());
    ASSERT_TRUE(weighted.lastPlanValid());
    EXPECT_FALSE(weighted.getDebugMetrics().weighted_checked);
    EXPECT_FALSE(weighted.getDebugMetrics().weighted_fallback);
    EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-12);
    EXPECT_GT(weighted.getDebugMetrics().static_reject_count, 0);
    EXPECT_EQ(weighted.getDebugMetrics().static_reject_count,
              best.getDebugMetrics().static_reject_count);
    expectExecutedPath(weighted, m, actual);
}

TEST(StaticMpcRuntime, DiscreteSelectedCostMatchesExecutedCost)
{
    MpcController weighted, best;
    auto m = StaticMpcTestAccess::model();
    for (auto *c : {&weighted, &best})
    {
        StaticMpcTestAccess::configure(*c, c == &best);
        StaticMpcTestAccess::steeringCost(*c);
        c->setCollision(StaticMpcTestAccess::map());
        c->setConvexCorridor({box()});
        c->plan(m, Eigen::Vector3d(2, 0, 0));
        ASSERT_TRUE(c->lastPlanValid());
    }
    EXPECT_DOUBLE_EQ(weighted.getDebugMetrics().best_total_cost,
                     best.getDebugMetrics().best_total_cost);
    EXPECT_DOUBLE_EQ(weighted.getDebugMetrics().executed_total_cost,
              weighted.getDebugMetrics().best_total_cost);
}

TEST(StaticMpcRuntime, DiscreteUnionChecksPreservedWithoutAveraging)
{
    MpcController weighted, best;
    auto m = StaticMpcTestAccess::model();
    // Two connected branches with a narrow throat at time zero. Their weighted
    // mean goes through the gap even though each retained branch is feasible.
    const std::vector<ConvexCorridor::Segment> corridor = {
        box(-1.0, 0.025), box(-1.0, 1.0, 0.006, 1.0), box(-1.0, 1.0, -1.0, -0.006)};
    for (auto *c : {&weighted, &best})
    {
        StaticMpcTestAccess::configure(*c, c == &best);
        c->setCollision(StaticMpcTestAccess::map());
        c->setConvexCorridor(corridor);
    }
    const auto expected = best.plan(m, Eigen::Vector3d(2, 0, 0));
    const auto actual = weighted.plan(m, Eigen::Vector3d(2, 0, 0));
    ASSERT_TRUE(best.lastPlanValid());
    ASSERT_TRUE(weighted.lastPlanValid());
    EXPECT_FALSE(weighted.getDebugMetrics().weighted_fallback);
    EXPECT_GT(weighted.getDebugMetrics().convex_corridor_reject_count, 0);
    EXPECT_GT(weighted.getDebugMetrics().max_convex_corridor_violation, 0.0);
    EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-12);
    expectExecutedPath(weighted, m, actual);
}

TEST(StaticMpcRuntime, EverySampleOutsideUnionStops)
{
    MpcController c;
    StaticMpcTestAccess::configure(c);
    c.setCollision(StaticMpcTestAccess::map());
    c.setConvexCorridor({box(-0.001, 0.001)});
    EXPECT_TRUE(c.plan(StaticMpcTestAccess::model(), Eigen::Vector3d(2, 0, 0)).isZero());
    EXPECT_FALSE(c.lastPlanValid());
    EXPECT_EQ(c.getDebugMetrics().convex_corridor_reject_count, 5);
    EXPECT_FALSE(c.getDebugMetrics().weighted_checked);
    EXPECT_TRUE(c.getBestPath().empty());
}

TEST(StaticMpcRuntime, EverySampleComCollisionStops)
{
    MpcController c;
    StaticMpcTestAccess::configure(c);
    c.setCollision(StaticMpcTestAccess::map(false, true));
    c.setConvexCorridor({box()});
    EXPECT_TRUE(c.plan(StaticMpcTestAccess::model(), Eigen::Vector3d(2, 0, 0)).isZero());
    EXPECT_FALSE(c.lastPlanValid());
    EXPECT_EQ(c.getDebugMetrics().static_reject_count, 5);
    EXPECT_TRUE(c.getBestPath().empty());
}

TEST(StaticMpcRuntime, BeamFullHorizonCopiesAncestryDiversityAndPruningUnknown)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::beam(c,5,2);
    c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box(-10,10,-10,10)});
    auto model=StaticMpcTestAccess::model();const auto original=model->getState();
    const auto position=model->getCOMPos();const auto control=c.plan(model,Eigen::Vector3d(1,1,0));
    ASSERT_TRUE(c.lastPlanValid());EXPECT_EQ(original,model->getState());EXPECT_EQ(position,model->getCOMPos());
    const auto evidence=c.beamEvidence();ASSERT_GE(evidence.selected_leaf,0);
    EXPECT_LE(c.tree().size(),size_t(1+5+25*2*4));EXPECT_EQ(5,c.tree()[evidence.selected_leaf].depth);
    EXPECT_TRUE(evidence.reroll_valid);int found=0,pruned=0;for(int i=0;i<5;++i){found+=evidence.found[i];pruned+=evidence.pruned[i];}
    EXPECT_EQ(5,found);EXPECT_GT(pruned,0);
    std::vector<int> chain;for(int id=evidence.selected_leaf;id>0;id=c.tree()[id].parent){chain.push_back(id);EXPECT_TRUE(c.tree()[id].continuation_found);}
    std::reverse(chain.begin(),chain.end());EXPECT_DOUBLE_EQ(control.z(),c.tree()[chain.front()].action);
    LFPC replay;replay.copyState(*model);bool changed=false;
    for(size_t i=0;i<chain.size();++i) {
        const auto& n=c.tree()[chain[i]];replay.SetCtrlParams(Eigen::Vector3d(control.x(),control.y(),n.action));replay.updateOneStep();replay.prepareNextStep();
        EXPECT_EQ(replay.getCOMPos(),const_cast<LFPC&>(n.state).getCOMPos());
        EXPECT_EQ(replay.getFootPosition(),const_cast<LFPC&>(n.state).getFootPosition());
        EXPECT_EQ(replay.getState(),const_cast<LFPC&>(n.state).getState());
        if(i && n.action!=c.tree()[chain[i-1]].action)changed=true;
    }
    EXPECT_TRUE(changed); // candidate can change steering at successive nodes
    const auto again=c.plan(model,Eigen::Vector3d(1,1,0));EXPECT_EQ(control,again);
    EXPECT_EQ(evidence.evaluated,c.beamEvidence().evaluated);EXPECT_EQ(evidence.selected_leaf,c.beamEvidence().selected_leaf);
    std::cout<<"BEAM horizon=5 width_per_root=2 evaluated="<<evidence.evaluated<<" ms="<<c.lastPlanTimeMs()<<std::endl;
}
TEST(StaticMpcRuntime, BeamActionLimitsAndNoFullContinuation)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::beam(c,4,2);StaticMpcTestAccess::angleLimit(c,0.);
    c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box(-.1,.2,-1,1)});
    const auto result=c.plan(StaticMpcTestAccess::model(),Eigen::Vector3d(3,0,0));
    EXPECT_FALSE(c.lastPlanValid());EXPECT_TRUE(result.isZero());EXPECT_EQ(0,c.beamEvidence().full_leaves);
    EXPECT_FALSE(c.beamEvidence().available[0]);EXPECT_TRUE(c.beamEvidence().available[2]);
    EXPECT_FALSE(c.beamEvidence().found[2]);
    StaticMpcTestAccess::beam(c,33,2);EXPECT_TRUE(c.plan(StaticMpcTestAccess::model(),Eigen::Vector3d(3,0,0)).isZero());
    EXPECT_TRUE(c.tree().empty());
}
TEST(StaticMpcRuntime, FinalArrivalIsSafeTerminalButLocalGoalIsNot)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::beam(c,10,8);StaticMpcTestAccess::steeringCost(c);
    c.setCollision(StaticMpcTestAccess::map());c.setConvexCorridor({box(-.5,.7,-.3,.3)});
    auto model=StaticMpcTestAccess::model();const Eigen::Vector2d final(.30,0);
    c.plan(model,Eigen::Vector3d(.30,0,0));EXPECT_FALSE(c.lastPlanValid()); // local-only: no invented idle
    const auto cmd=c.plan(model,Eigen::Vector3d(.30,0,0),&final,.2);
    ASSERT_TRUE(c.lastPlanValid());ASSERT_GE(c.beamEvidence().selected_leaf,0);
    EXPECT_TRUE(c.tree()[c.beamEvidence().selected_leaf].terminal);
    EXPECT_LT(c.tree()[c.beamEvidence().selected_leaf].depth,10);EXPECT_TRUE(c.beamEvidence().reroll_valid);
    LFPC state;state.copyState(*model);state.SetCtrlParams(cmd);state.updateOneStep();
    EXPECT_LT((state.getCOMPos().head<2>()-final).norm(),.2);
    c.setConvexCorridor({box(-.001,.001,-.3,.3)});
    EXPECT_TRUE(c.plan(model,Eigen::Vector3d(.30,0,0),&final,.2).isZero());EXPECT_FALSE(c.lastPlanValid());
}
TEST(StaticMpcRuntime, LocalArrivalStillChargesEveryWalkingSuffixEdge)
{
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::suffixCost(c);
    c.setCollision(StaticMpcTestAccess::wideMap());c.setConvexCorridor({box(-10,10,-10,10)});
    auto model=StaticMpcTestAccess::model();
    const auto command=c.plan(model,Eigen::Vector3d(.30,0,0));ASSERT_TRUE(c.lastPlanValid());
    const auto& nodes=c.tree();const auto& leaf=nodes[c.beamEvidence().selected_leaf];
    EXPECT_EQ(10,leaf.depth);EXPECT_FALSE(leaf.terminal);EXPECT_TRUE(leaf.arrived);
    EXPECT_DOUBLE_EQ(0.,command.z());EXPECT_DOUBLE_EQ(0.,leaf.motion_cost);
    int suffix_turns=0;
    for(const auto& n:nodes)if(n.safe && n.parent>=0) {
        const auto& parent=nodes[n.parent];
        const double increment=.5*std::abs(n.action)+(n.depth>1?.7*std::abs(n.action-parent.action):0.);
        EXPECT_DOUBLE_EQ(parent.motion_cost+increment,n.motion_cost);
        if(parent.arrived && n.action!=0.){++suffix_turns;EXPECT_GT(n.motion_cost,parent.motion_cost);}
    }
    EXPECT_GT(suffix_turns,0);
    for(int id=leaf.id;id>0;id=nodes[id].parent)EXPECT_DOUBLE_EQ(0.,nodes[id].action);
    EXPECT_TRUE(c.beamEvidence().reroll_valid);
    EXPECT_DOUBLE_EQ(leaf.objective,c.getDebugMetrics().executed_total_cost);
    const Eigen::Vector2d final(.30,0);
    c.plan(model,Eigen::Vector3d(.30,0,0),&final,.2);ASSERT_TRUE(c.lastPlanValid());
    EXPECT_TRUE(c.tree()[c.beamEvidence().selected_leaf].terminal);
    EXPECT_EQ(1,c.tree()[c.beamEvidence().selected_leaf].depth);
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
    MpcController c;StaticMpcTestAccess::configure(c);StaticMpcTestAccess::beam(c,3,8);
    StaticMpcTestAccess::angleLimit(c,0.);c.setCollision(StaticMpcTestAccess::wideMap());
    c.setConvexCorridor({box(-9,9,-9,9)});c.plan(model,Eigen::Vector3d(5,0,0));
    ASSERT_TRUE(c.lastPlanValid());const auto& times=c.getBestTimes();
    ASSERT_EQ(phases.size()*3,times.size());ASSERT_EQ(c.getBestPath().size(),times.size());
    for(size_t i=0;i<times.size();++i)EXPECT_NEAR((i+1)*.07,times[i],1e-12);
    EXPECT_NEAR(times.back(),c.tree().back().elapsed,1e-12);
}

TEST(TimedMpcRuntime, CrossingBeyondFirstStepRejectsFullContinuation) {
    auto m=StaticMpcTestAccess::model();MpcController c;
    StaticMpcTestAccess::configure(c);StaticMpcTestAccess::beam(c,3,8);StaticMpcTestAccess::angleLimit(c,0.);
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

// Opt-in measured benchmark; same full search, no pruning difference or timing assertion.
TEST(TimedMpcRuntime, DISABLED_BenchmarkSameBeamWork) {
    auto m=StaticMpcTestAccess::model();MpcController c;StaticMpcTestAccess::configure(c);
    StaticMpcTestAccess::suffixCost(c);c.setCollision(StaticMpcTestAccess::wideMap());c.setConvexCorridor({box(-9,9,-9,9)});
    std::array<std::vector<double>,2> timings;
    for(int repeat=0;repeat<105;++repeat)for(int mode=0;mode<2;++mode) {
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
