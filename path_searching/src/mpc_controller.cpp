#include <path_searching/mpc_controller.h>
#include <path_searching/trajectory_feasibility.h>
#include <cmath>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <std_msgs/Float64MultiArray.h>

namespace cane_planner
{

MpcController::MpcController() : last_plan_time_ms_(0.0) {}
MpcController::~MpcController() = default;

void MpcController::setParam(ros::NodeHandle &nh)
{
    nh.param("mpc/sample_vis_enable", sample_vis_enable_, true);
    nh.param("mpc/sample_vis_count", sample_vis_count_, 20);
    sample_vis_count_ = std::max(0, std::min(sample_vis_count_, 50));
    sample_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/mpc/sample_trajectories", 1);
    publishSampleTrajectories({}, Eigen::VectorXd());

    nh.param("mpc/horizon_steps", cfg_.horizon_steps, 6);
    nh.param("mpc/beam_width", cfg_.beam_width, 8);
    beam_pub_=nh.advertise<std_msgs::Float64MultiArray>("/mpc/beam_evidence",1);

    nh.param("mpc/max_al", cfg_.max_al, 0.4);
    nh.param("mpc/min_al", cfg_.min_al, 0.05);
    nh.param("mpc/max_aw", cfg_.max_aw, 0.15);
    nh.param("mpc/max_api", cfg_.max_api, 0.5236);


    nh.param("mpc/w_steer", cfg_.w_steer, 0.5);
    nh.param("mpc/w_goal", cfg_.w_goal, 10.0);
    nh.param("mpc/w_dapi", cfg_.w_dapi, 0.0);
    nh.param("mpc/goal_arrival_threshold", cfg_.goal_arrival_threshold, 0.3);

    nh.param("mpc/nominal_al", cfg_.nominal_al, 0.25);
    nh.param("mpc/nominal_aw", cfg_.nominal_aw, 0.0);
    nh.param("mpc/timed_collision_enable", cfg_.timed_collision_enable, true);
    nh.param("mpc/timed_robot_radius", cfg_.timed_robot_radius, 0.25);

}

void MpcController::init()
{
    reset();
}

void MpcController::reset()
{
    tree_.clear();beam_evidence_=BeamEvidence();
    best_path_.clear(); best_times_.clear();
    clearTimedPedestrians();
    last_plan_valid_ = false;
    last_plan_time_ms_ = 0.0;
    last_debug_metrics_ = DebugMetrics();
    publishSampleTrajectories({}, Eigen::VectorXd());
    publishBeamEvidence();
}

void MpcController::resetWarmStart()
{
    tree_.clear();beam_evidence_=BeamEvidence(); // no stochastic warm start retained
}

void MpcController::publishSampleTrajectories(
    const std::vector<std::vector<Eigen::Vector3d>> &paths,
    const Eigen::VectorXd &costs)
{
    if (!sample_vis_pub_)
        return;

    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // This dedicated topic owns its markers. Clear old IDs even if disabled,
    // reset, or publishing fewer samples than on the previous frame.
    marker.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(marker);
    const int available = std::min(static_cast<int>(paths.size()),
                                   static_cast<int>(costs.size()));
    const int count = sample_vis_enable_ ? std::min(sample_vis_count_, available) : 0;
    marker.ns = "mpc_sample_trajectories";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.015;
    marker.color.a = 0.65;
    marker.lifetime = ros::Duration(0.5);
    for (int i = 0; i < count; ++i)
    {
        // Uniform original indices, not top-cost ranking and no RNG draws.
        const int k = count > 1 ? static_cast<int>(
            static_cast<size_t>(i) * (available - 1) / (count - 1)) : 0;
        marker.id = k;
        const bool finite_cost = std::isfinite(costs(k));
        marker.color.r = finite_cost ? 0.0 : 1.0;
        marker.color.g = finite_cost ? 0.7 : 0.4;
        marker.color.b = finite_cost ? 1.0 : 0.0;
        marker.points.clear();
        // Copy only real computed CoM points. Rejected/early-arrival paths
        // may be shorter than the configured horizon; never extend them.
        for (const auto &pt : paths[k])
        {
            geometry_msgs::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            marker.points.push_back(p);
        }
        if (marker.points.size() >= 2)
            msg.markers.push_back(marker);
    }
    sample_vis_pub_.publish(msg);
}

void MpcController::setCollision(const CollisionDetection::Ptr &col)
{
    collision_ = col;
}

void MpcController::setConvexCorridor(const std::vector<ConvexCorridor::Segment> &segments)
{
    convex_corridor_segments_ = segments;
    has_convex_corridor_ = !segments.empty() && std::all_of(
        segments.begin(), segments.end(), [](const ConvexCorridor::Segment &segment) {
            if (!segment.static_feasible || segment.halfspaces.size() < 3 ||
                !segment.center.allFinite())
                return false;
            return std::all_of(segment.halfspaces.begin(), segment.halfspaces.end(),
                [](const ConvexCorridor::Halfspace &h) {
                    return h.normal.allFinite() && h.normal.norm() > 1e-9 &&
                           std::isfinite(h.offset);
                });
        });
}

void MpcController::clearConvexCorridor()
{
    has_convex_corridor_ = false;
    convex_corridor_segments_.clear();
}

// =========================================================================
// Main API
// =========================================================================

void MpcController::publishBeamEvidence()
{
    if(!beam_pub_)return;
    std_msgs::Float64MultiArray msg;
    std_msgs::MultiArrayDimension d;d.label="lfpc_beam_v1;roots=-30,-15,0,15,30;not_found_is_not_proof";
    d.size=d.stride=24;msg.layout.dim.push_back(d);
    msg.data={1.,double(beam_evidence_.horizon),double(beam_evidence_.width),double(beam_evidence_.evaluated),
        double(beam_evidence_.full_leaves),double(beam_evidence_.selected_root),double(beam_evidence_.selected_leaf),beam_evidence_.reroll_valid?1.:0.};
    for(int i=0;i<5;++i){msg.data.push_back(beam_evidence_.available[i]);msg.data.push_back(beam_evidence_.found[i]);msg.data.push_back(beam_evidence_.pruned[i]);}
    msg.data.push_back(beam_evidence_.selected_leaf>=0 && tree_[beam_evidence_.selected_leaf].terminal?1.:0.);
    beam_pub_.publish(msg);
}

double MpcController::distanceToGoal(LFPC& state,const Eigen::Vector3d& goal) const
{return (state.getCOMPos().head<2>()-goal.head<2>()).norm();}

bool MpcController::setTimedPedestrians(
    const std::vector<TimedPedestrianCollision::Obstacle>& obstacles, double age)
{
    clearTimedPedestrians();
    if (!cfg_.timed_collision_enable || obstacles.empty()) return true;
    timed_observation_age_ = age;
    timed_input_valid_ = timed_collision_.bind(obstacles, cfg_.timed_robot_radius) &&
                         std::isfinite(age) && age >= 0.;
    return timed_input_valid_;
}

void MpcController::clearTimedPedestrians()
{
    timed_collision_.clear(); timed_input_valid_ = true; timed_observation_age_ = 0.;
    collision_witness_ = CollisionWitness();
}

bool MpcController::timedFree(const Eigen::Vector2d& com, double time, double age)
{
    if (timed_collision_.empty() && timed_input_valid_) return true;
    ++last_debug_metrics_.timed_collision_checks;
    if (timed_input_valid_ && timed_collision_.intersects(com, age+time) == false) return true;
    ++last_debug_metrics_.dynamic_reject_count;
    // Keep one bounded witness, not every rejected beam sample.
    if (!collision_witness_.valid) collision_witness_ = {true, com, time, age};
    return false;
}

bool MpcController::revalidateTimedPath(double age)
{
    if (!last_plan_valid_) return false;
    if (timed_collision_.empty() && timed_input_valid_) return true;
    last_debug_metrics_.timed_execution_checked = true;
    bool safe = timed_input_valid_ && std::isfinite(age) && age >= 0. &&
                best_path_.size() == best_times_.size();
    // A latency-induced failure must replace a search witness for visualization.
    collision_witness_ = CollisionWitness();
    if (safe) safe = timedFree(timed_root_, 0., age);
    for (size_t i=0; safe && i<best_path_.size(); ++i)
        safe = timedFree(best_path_[i].head<2>(), best_times_[i], age);
    last_debug_metrics_.timed_execution_safe = safe;
    if (safe) timed_observation_age_ = age;
    else {
        last_plan_valid_ = last_debug_metrics_.plan_valid = false;
        best_path_.clear(); best_times_.clear();
    }
    return safe;
}

bool MpcController::advance(LFPC& state,double action,std::vector<Eigen::Vector3d>& path,
                            double& elapsed,std::vector<double>* times)
{
    const size_t previous_samples = state.getStepCOMPath().size();
    const double phase_start = state.getPhaseTime();
    state.SetCtrlParams(Eigen::Vector3d(cfg_.nominal_al,cfg_.nominal_aw,action));
    state.updateOneStep();
    const auto points=state.getStepCOMPath();
    const auto& phases=state.getStepCOMTimes();
    auto free=[&](const Eigen::Vector2d& p){return p.allFinite() && (collision_->hasStaticGlobalMap()?
        collision_->isStaticTraversable(p.x(),p.y()):collision_->isTraversable(p.x(),p.y()));};
    if(points.size()<=previous_samples || phases.size()!=points.size() ||
       !state.getCOMPos().allFinite() || !free(state.getFootPosition())) {
        ++last_debug_metrics_.static_reject_count;return false;
    }
    for(size_t i=previous_samples;i<points.size();++i) {
        const Eigen::Vector2d p=points[i].head<2>();
        last_debug_metrics_.corridor_evaluated=true;
        const bool inside=p.allFinite() && std::any_of(convex_corridor_segments_.begin(),convex_corridor_segments_.end(),
            [&](const ConvexCorridor::Segment& seg){return ConvexCorridor::contains(seg,p,1e-6);});
        if(!inside) {
            double v=std::numeric_limits<double>::infinity();
            for(const auto& seg:convex_corridor_segments_)v=std::min(v,ConvexCorridor::violation(seg,p));
            last_debug_metrics_.max_convex_corridor_violation=std::max(last_debug_metrics_.max_convex_corridor_violation,v);
            ++last_debug_metrics_.convex_corridor_reject_count;return false;
        }
        if(!free(p)){++last_debug_metrics_.static_reject_count;return false;}
        const double time=elapsed+phases[i]-phase_start;
        if (!std::isfinite(time) || time<=elapsed || !timedFree(p,time,timed_observation_age_)) return false;
    }
    for(size_t i=previous_samples;i<points.size();++i) {
        path.push_back(points[i]);
        if(times) times->push_back(elapsed+phases[i]-phase_start);
    }
    elapsed += phases.back()-phase_start;
    state.prepareNextStep();return true;
}

Eigen::Vector3d MpcController::plan(const LFPC::Ptr& base,const Eigen::Vector3d& goal,const Eigen::Vector2d* final_goal,double final_radius)
{
    const auto started=std::chrono::steady_clock::now();
    tree_.clear();best_path_.clear();best_times_.clear();collision_witness_=CollisionWitness();last_plan_valid_=false;last_debug_metrics_=DebugMetrics();beam_evidence_=BeamEvidence();
    last_debug_metrics_.timed_collision_active = !timed_collision_.empty() || !timed_input_valid_;
    const int N=cfg_.horizon_steps,W=cfg_.beam_width;
    beam_evidence_.horizon=N;beam_evidence_.width=W;
    Eigen::Vector3d command=Eigen::Vector3d::Zero();
    auto finish=[&] {
        last_plan_time_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
        last_debug_metrics_.plan_time_ms=last_plan_time_ms_;last_debug_metrics_.plan_valid=last_plan_valid_;
        publishBeamEvidence();return command;
    };
    bool config=!final_goal || (final_goal->allFinite() && std::isfinite(final_radius) && final_radius>0.);
    for(double x:{cfg_.nominal_al,cfg_.nominal_aw,cfg_.min_al,cfg_.max_al,cfg_.max_aw,cfg_.max_api,
                  cfg_.w_steer,cfg_.w_dapi,cfg_.w_goal,cfg_.goal_arrival_threshold})config=config&&std::isfinite(x);
    config=config && N>=1 && N<=32 && W>=1 && W<=32 && cfg_.min_al>=0 && cfg_.nominal_al>=cfg_.min_al &&
        cfg_.nominal_al<=cfg_.max_al && cfg_.nominal_aw>=0 && cfg_.nominal_aw<=cfg_.max_aw && cfg_.max_api>=0 &&
        cfg_.w_steer>=0 && cfg_.w_dapi>=0 && cfg_.w_goal>=0 && cfg_.goal_arrival_threshold>=0;
    const bool inside=base && base->getCOMPos().allFinite() && std::any_of(convex_corridor_segments_.begin(),convex_corridor_segments_.end(),
        [&](const ConvexCorridor::Segment& seg){return ConvexCorridor::contains(seg,base->getCOMPos().head<2>(),1e-6);});
    if(!config || !timed_input_valid_ || !collision_ || !goal.allFinite() || !has_convex_corridor_ || !inside ||
       !(collision_->hasStaticGlobalMap()?collision_->isStaticTraversable(base->getCOMPos().x(),base->getCOMPos().y()):
         collision_->isTraversable(base->getCOMPos().x(),base->getCOMPos().y()))) {
        publishSampleTrajectories({},Eigen::VectorXd());return finish();
    }
    timed_root_ = base->getCOMPos().head<2>();
    if (!timedFree(timed_root_,0.,timed_observation_age_)) {
        publishSampleTrajectories({},Eigen::VectorXd());return finish();
    }
    constexpr double rad=3.14159265358979323846/180.;
    const std::array<double,5> actions{{-30*rad,-15*rad,0,15*rad,30*rad}};
    for(int i=0;i<5;++i)beam_evidence_.available[i]=std::abs(actions[i])<=cfg_.max_api;
    tree_.reserve(1+5+25*W*(N-1));
    TreeNode root;root.state.copyState(*base);root.safe=true;tree_.push_back(root);
    std::array<std::vector<int>,5> frontier;
    std::vector<int> leaves;
    auto expand=[&](int parent,int ai,int ri) {
        TreeNode node;node.id=tree_.size();node.parent=parent;node.root=ri;node.depth=tree_[parent].depth+1;node.action=actions[ai];
        node.state.copyState(tree_[parent].state);node.motion_cost=tree_[parent].motion_cost;node.arrived=tree_[parent].arrived;
        node.elapsed=tree_[parent].elapsed;
        std::vector<Eigen::Vector3d> ignored;node.safe=advance(node.state,node.action,ignored,node.elapsed);++beam_evidence_.evaluated;
        if(node.safe) {
            node.motion_cost+=cfg_.w_steer*std::abs(node.action)+(node.depth>1?cfg_.w_dapi*std::abs(node.action-tree_[parent].action):0.);
            node.arrived=node.arrived || distanceToGoal(node.state,goal)<cfg_.goal_arrival_threshold;
            node.terminal=final_goal && (node.state.getCOMPos().head<2>()-*final_goal).norm()<final_radius;
            node.objective=node.motion_cost+(node.arrived?0.:(cfg_.w_goal/N)*distanceToGoal(node.state,goal));
            node.safe=std::isfinite(node.objective);
        }
        tree_[parent].children.push_back(node.id);tree_.push_back(std::move(node));return int(tree_.size()-1);
    };
    for(int ri=0;ri<5;++ri)if(beam_evidence_.available[ri]) {
        int id=expand(0,ri,ri);if(tree_[id].safe) {if(tree_[id].terminal)leaves.push_back(id);else frontier[ri].push_back(id);}
    }
    for(int depth=2;depth<=N;++depth)for(int ri=0;ri<5;++ri) {
        std::vector<int> next;
        for(int parent:frontier[ri])for(int ai=0;ai<5;++ai)if(beam_evidence_.available[ai]) {
            int id=expand(parent,ai,ri);if(tree_[id].safe) {if(tree_[id].terminal)leaves.push_back(id);else next.push_back(id);}
        }
        std::stable_sort(next.begin(),next.end(),[&](int a,int b){return tree_[a].objective<tree_[b].objective;});
        if(next.size()>size_t(W)) {
            beam_evidence_.pruned[ri]=true;
            for(size_t i=W;i<next.size();++i)tree_[next[i]].pruned=true;
            next.resize(W);
        }
        frontier[ri]=std::move(next);
    }
    for(int ri=0;ri<5;++ri)for(int id:frontier[ri])if(tree_[id].depth==N) {
        leaves.push_back(id);
    }
    for(int id:leaves) {beam_evidence_.found[tree_[id].root]=true;
        for(int p=id;p>=0;p=tree_[p].parent)tree_[p].continuation_found=true;}
    beam_evidence_.full_leaves=leaves.size();
    last_debug_metrics_.num_samples=beam_evidence_.evaluated;
    last_debug_metrics_.valid_trajectory_count=leaves.size();last_debug_metrics_.corridor_feasible_trajectory_count=leaves.size();
    last_debug_metrics_.convex_corridor_segments=convex_corridor_segments_.size();
    // Legacy ratio unavailable: expanded edges and retained leaves are not Monte Carlo samples.
    last_debug_metrics_.valid_sample_ratio=-1.;last_debug_metrics_.candidate_inside_corridor_ratio=-1.;
    std::stable_sort(leaves.begin(),leaves.end(),[&](int a,int b){return tree_[a].objective<tree_[b].objective;});
    std::vector<std::vector<Eigen::Vector3d>> display_paths;std::vector<double> display_costs;
    if(!leaves.empty()) {
        const int selected=leaves.front();std::vector<int> ancestry;
        for(int id=selected;tree_[id].parent>=0;id=tree_[id].parent)ancestry.push_back(id);
        std::reverse(ancestry.begin(),ancestry.end());
        LFPC checked;checked.copyState(*base);bool valid=true,arrived=false;
        double replay_motion=0.,previous_action=0.,elapsed=0.;
        for(size_t step=0;step<ancestry.size();++step) {
            const double action=tree_[ancestry[step]].action;
            if(!advance(checked,action,best_path_,elapsed,&best_times_)){valid=false;break;}
            replay_motion+=cfg_.w_steer*std::abs(action)+(step>0?cfg_.w_dapi*std::abs(action-previous_action):0.);
            previous_action=action;
            arrived=arrived || distanceToGoal(checked,goal)<cfg_.goal_arrival_threshold;
        }
        const double replay_objective=replay_motion+(arrived?0.:(cfg_.w_goal/N)*distanceToGoal(checked,goal));
        valid=valid && std::isfinite(replay_objective) && replay_objective==tree_[selected].objective;
        if(tree_[selected].terminal)valid=valid && final_goal && (checked.getCOMPos().head<2>()-*final_goal).norm()<final_radius;
        beam_evidence_.selected_root=tree_[selected].root;beam_evidence_.selected_leaf=selected;beam_evidence_.reroll_valid=valid;
        if(valid) {
            last_plan_valid_=true;command=Eigen::Vector3d(cfg_.nominal_al,cfg_.nominal_aw,tree_[ancestry.front()].action);
            last_debug_metrics_.best_total_cost=last_debug_metrics_.executed_total_cost=tree_[selected].objective;
            display_paths.push_back(best_path_);display_costs.push_back(tree_[selected].objective);
        }else {best_path_.clear();best_times_.clear();}
        for(size_t k=1;k<leaves.size() && display_paths.size()<size_t(sample_vis_count_);++k) {
            std::vector<int> chain;for(int id=leaves[k];tree_[id].parent>=0;id=tree_[id].parent)chain.push_back(id);
            std::vector<Eigen::Vector3d> path;
            // Each node retains full LFPC pose, draw actual validated step end points.
            path.push_back(base->getCOMPos());for(auto i=chain.rbegin();i!=chain.rend();++i)path.push_back(tree_[*i].state.getCOMPos());
            display_paths.push_back(path);display_costs.push_back(tree_[leaves[k]].objective);
        }
    }
    Eigen::VectorXd costs(display_costs.size());for(size_t i=0;i<display_costs.size();++i)costs[i]=display_costs[i];
    publishSampleTrajectories(display_paths,costs);return finish();
}

} // namespace cane_planner
