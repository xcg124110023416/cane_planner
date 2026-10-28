#include <path_searching/mpc_controller.h>
#include <cmath>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Geometry>

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
    nh.param("mpc/num_samples", cfg_.num_samples, 256);
    nh.param("mpc/mpc_iters", cfg_.mpc_iters, 2);
    nh.param("mpc/temperature", cfg_.temperature, 0.4);
    nh.param("mpc/sigma_api", cfg_.sigma_api, 0.35);
    nh.param("mpc/random_seed", cfg_.random_seed, 0);
    rng_.seed(cfg_.random_seed); normal_dist_.reset();
    optimization_pub_=nh.advertise<std_msgs::Float64MultiArray>("/mpc/optimization_evidence",1);

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
    nh.param("human_cane/enabled",cfg_.footprint.enabled,true);
    nh.param("human_cane/body_radius",cfg_.footprint.body_radius,.25);
    nh.param("human_cane/cane_length",cfg_.footprint.cane_length,.65);
    nh.param("human_cane/front_radius",cfg_.footprint.front_radius,.12);
    nh.param("human_cane/margin",cfg_.footprint.margin,.03);
    footprint_local_=HumanCaneFootprint::relative(cfg_.footprint,0.);

}

void MpcController::init()
{
    reset();
}

void MpcController::reset()
{
    resetWarmStart(); optimization_evidence_=OptimizationEvidence();
    best_path_.clear(); best_times_.clear(); best_yaws_.clear(); best_controls_.clear();
    clearTimedPedestrians();
    last_plan_valid_ = false;
    last_plan_time_ms_ = 0.0;
    last_debug_metrics_ = DebugMetrics();
    publishSampleTrajectories({}, Eigen::VectorXd());
    publishOptimizationEvidence();
}

void MpcController::resetWarmStart()
{
    warm_start_.clear();
}

bool MpcController::seedReference(const LFPC& base,const std::vector<Eigen::Vector2d>& reference)
{
    if(!warm_start_.empty())return true;
    resetWarmStart();
    if(reference.size()<2 || cfg_.horizon_steps<1 || cfg_.horizon_steps>32)return false;
    for(const auto& p:reference)if(!p.allFinite())return false;
    LFPC state;state.copyState(base);ControlSequence proposal;size_t index=0;
    for(int i=0;i<cfg_.horizon_steps;++i) {
        const Eigen::Vector2d position=state.getCOMPos().head<2>();
        double nearest=std::numeric_limits<double>::infinity();
        // Advance the copied model's projection too. Reusing segment zero after
        // passing its endpoint would steer the remaining proposal backwards.
        for(size_t j=index;j+1<reference.size();++j) {
            const Eigen::Vector2d edge=reference[j+1]-reference[j];
            if(edge.squaredNorm()<1e-18)continue;
            const auto projection=(reference[j]+std::max(0.,std::min(1.,
                (position-reference[j]).dot(edge)/edge.squaredNorm()))*edge).eval();
            const double distance=(position-projection).squaredNorm();
            if(distance<nearest) {nearest=distance;index=j;}
        }
        const auto ahead=ConvexCorridor::referencePath(reference,index,position,
            std::max(.5,2.*cfg_.nominal_al));
        if(ahead.empty())return false;
        const Eigen::Vector2d d=ahead.back()-state.getCOMPos().head<2>();
        const double error=std::atan2(d.y(),d.x())-state.getNextIterState()(2);
        const double action=std::max(-cfg_.max_api,std::min(cfg_.max_api,std::atan2(std::sin(error),std::cos(error))));
        if(!std::isfinite(action))return false;
        proposal.push_back(action);
        state.SetCtrlParams(Eigen::Vector3d(cfg_.nominal_al,cfg_.nominal_aw,action));
        state.updateOneStep();state.prepareNextStep();
        if(!state.getCOMPos().allFinite())return false;
    }
    warm_start_=std::move(proposal);return true;
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

void MpcController::publishOptimizationEvidence()
{
    if(!optimization_pub_) return;
    const auto& e=optimization_evidence_;
    const auto finite=[](double x){return std::isfinite(x)?x:-1.;};
    std_msgs::Float64MultiArray msg;
    msg.data={1.,double(e.horizon),double(e.samples_per_iteration),double(e.iterations),
        double(e.evaluated),double(e.valid),double(e.weighted_checks),double(e.weighted_rejects),
        e.effective_samples,double(e.selected_steps),e.selected_terminal?1.:0.,e.reroll_valid?1.:0.,
        finite(last_debug_metrics_.best_total_cost),finite(last_debug_metrics_.executed_total_cost)};
    for(double cost:e.best_verified_costs)msg.data.push_back(finite(cost));
    for(double cost:e.weighted_costs)msg.data.push_back(finite(cost));
    std_msgs::MultiArrayDimension d;
    d.label="lfpc_mppi_v1;0=version,1=horizon,2=samples_per_iter,3=iters,4=sampled,5=valid,6=weighted_checks,7=weighted_rejects,8=ESS,9=selected_steps,10=terminal,11=reroll_valid,12=best_verified_cost,13=executed_cost;then[iter_best_costs],[iter_weighted_costs];-1=unavailable";
    d.size=d.stride=msg.data.size();msg.layout.dim.push_back(d);
    optimization_pub_.publish(msg);
}

double MpcController::distanceToGoal(LFPC& state,const Eigen::Vector3d& goal) const
{return (state.getCOMPos().head<2>()-goal.head<2>()).norm();}

bool MpcController::setTimedPedestrians(
    const std::vector<TimedPedestrianCollision::Obstacle>& obstacles, double age)
{
    clearTimedPedestrians();
    if (!cfg_.timed_collision_enable || obstacles.empty()) return true;
    timed_observation_age_ = age;
    timed_input_valid_ = (cfg_.footprint.enabled?timed_collision_.bind(obstacles,HumanCaneFootprint::relative(cfg_.footprint,0.)):
                         timed_collision_.bind(obstacles, cfg_.timed_robot_radius)) &&
                         std::isfinite(age) && age >= 0.;
    return timed_input_valid_;
}

void MpcController::clearTimedPedestrians()
{
    timed_collision_.clear(); timed_input_valid_ = true; timed_observation_age_ = 0.;
    collision_witness_ = CollisionWitness();
}

bool MpcController::timedFree(const Eigen::Vector2d& com, double time, double age,double yaw)
{
    if (timed_collision_.empty() && timed_input_valid_) return true;
    ++last_debug_metrics_.timed_collision_checks;
    if (timed_input_valid_ && timed_collision_.intersects(com, age+time,cfg_.footprint.enabled?yaw:0.) == false) return true;
    ++last_debug_metrics_.dynamic_reject_count;
    // Keep one bounded witness, not every rejected rollout sample.
    if (!collision_witness_.valid) collision_witness_ = {true, com, time, age,yaw};
    return false;
}

bool MpcController::revalidateTimedPath(double age)
{
    if (!last_plan_valid_) return false;
    if (timed_collision_.empty() && timed_input_valid_) return true;
    last_debug_metrics_.timed_execution_checked = true;
    bool safe = timed_input_valid_ && std::isfinite(age) && age >= 0. &&
                best_path_.size() == best_times_.size() && best_path_.size()==best_yaws_.size();
    // A latency-induced failure must replace a search witness for visualization.
    collision_witness_ = CollisionWitness();
    if (safe) safe = timedFree(timed_root_, 0., age,timed_root_yaw_);
    for (size_t i=0; safe && i<best_path_.size(); ++i)
        safe = timedFree(best_path_[i].head<2>(), best_times_[i], age,best_yaws_[i]);
    last_debug_metrics_.timed_execution_safe = safe;
    if (safe) timed_observation_age_ = age;
    else {
        last_plan_valid_ = last_debug_metrics_.plan_valid = false;
        best_path_.clear(); best_times_.clear(); best_yaws_.clear(); best_controls_.clear();
    }
    return safe;
}

HumanCaneFootprint::Polygon MpcController::robotFootprintAt(const Eigen::Vector2d& p,double yaw) const {
    if(cfg_.footprint.enabled) {auto shape=relativeFootprint(yaw);for(auto& v:shape)v+=p;return shape;}
    TimedPedestrianCollision checker;if(!checker.bind({},cfg_.timed_robot_radius))return {};
    auto shape=checker.robotFootprint();for(auto& v:shape)v+=p;return shape;
}
HumanCaneFootprint::Polygon MpcController::relativeFootprint(double yaw) const {
    const auto& local=footprint_local_;
    if(local.empty())return HumanCaneFootprint::relative(cfg_.footprint,yaw);
    const Eigen::Rotation2Dd rotation(yaw);HumanCaneFootprint::Polygon shape;shape.reserve(local.size());
    for(const auto& v:local)shape.push_back(rotation*v);
    return shape;
}
bool MpcController::spatialFree(const Eigen::Vector2d& p) {
    if(!p.allFinite())return false;
    return std::any_of(convex_corridor_segments_.begin(),convex_corridor_segments_.end(),
        [&](const ConvexCorridor::Segment& region){return ConvexCorridor::contains(region,p,1e-6);});
}
bool MpcController::advance(LFPC& state,double action,std::vector<Eigen::Vector3d>& path,
                            double& elapsed,std::vector<double>* times,std::vector<double>* yaws)
{
    const size_t previous_samples = state.getStepCOMPath().size();
    const double phase_start = state.getPhaseTime();
    const double initial_yaw=state.getNextIterState()(2);
    const Eigen::Vector3d initial_com=state.getCOMPos();
    if(cfg_.footprint.enabled) {
        // LFPC changes torso yaw at a step boundary. Sample the swept yaw there;
        // these model-time-zero checks are retained for execution-age revalidation.
        const int rotations=std::max(1,int(std::ceil(std::abs(action)/.0872664625997165)));
        for(int i=1;i<=rotations;++i) {
            const double yaw=initial_yaw+action*double(i)/rotations;
            if(!timedFree(initial_com.head<2>(),elapsed,timed_observation_age_,yaw))return false;
            path.push_back(initial_com);if(times)times->push_back(elapsed);if(yaws)yaws->push_back(yaw);
        }
    }
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
        const bool inside=spatialFree(p);
        if(!inside) {
            double v=std::numeric_limits<double>::infinity();
            for(const auto& seg:convex_corridor_segments_)v=std::min(v,ConvexCorridor::violation(seg,p));
            last_debug_metrics_.max_convex_corridor_violation=std::max(last_debug_metrics_.max_convex_corridor_violation,v);
            ++last_debug_metrics_.convex_corridor_reject_count;return false;
        }
        if(!free(p)){++last_debug_metrics_.static_reject_count;return false;}
        const double time=elapsed+phases[i]-phase_start;
        if (!std::isfinite(time) || time<=elapsed || !timedFree(p,time,timed_observation_age_,state.getNextIterState()(2))) return false;
    }
    for(size_t i=previous_samples;i<points.size();++i) {
        path.push_back(points[i]);
        if(times) times->push_back(elapsed+phases[i]-phase_start);
        if(yaws)yaws->push_back(state.getNextIterState()(2));
    }
    elapsed += phases.back()-phase_start;
    state.prepareNextStep();return true;
}

std::vector<MpcController::ControlSequence> MpcController::sampleSequences(
    const ControlSequence& mean,const ControlSequence& incumbent)
{
    std::vector<ControlSequence> samples(cfg_.num_samples,mean);
    if(samples.size()>1) samples[1]=incumbent.size()==mean.size()?incumbent:ControlSequence(mean.size(),0.);
    // Proposal and retained candidate (or straight sequence), then antithetic Gaussian
    // pairs. Paired noise reduces sampling bias without quantizing steering.
    for(size_t k=2;k<samples.size();k+=2)for(size_t n=0;n<mean.size();++n) {
        const double noise=cfg_.sigma_api*normal_dist_(rng_);
        samples[k][n]=std::max(-cfg_.max_api,std::min(cfg_.max_api,mean[n]+noise));
        if(k+1<samples.size())
            samples[k+1][n]=std::max(-cfg_.max_api,std::min(cfg_.max_api,mean[n]-noise));
    }
    return samples;
}

std::vector<double> MpcController::computeWeights(const std::vector<double>& costs) const
{
    std::vector<double> weights(costs.size(),0.);
    double minimum=std::numeric_limits<double>::infinity(),sum=0.;
    for(double cost:costs)if(std::isfinite(cost))minimum=std::min(minimum,cost);
    if(!std::isfinite(minimum))return weights; // no uniform weight for unsafe samples
    for(size_t i=0;i<costs.size();++i)if(std::isfinite(costs[i])) {
        weights[i]=std::exp(-(costs[i]-minimum)/cfg_.temperature);sum+=weights[i];
    }
    for(auto& weight:weights)weight/=sum; // minimum has weight one, sum cannot underflow
    return weights;
}

MpcController::ControlSequence MpcController::weightedUpdate(
    const std::vector<ControlSequence>& samples,const std::vector<double>& weights) const
{
    ControlSequence mean(cfg_.horizon_steps,0.);
    for(size_t k=0;k<samples.size();++k)for(size_t n=0;n<mean.size();++n)
        mean[n]+=weights[k]*samples[k][n];
    for(auto& action:mean)action=std::max(-cfg_.max_api,std::min(cfg_.max_api,action));
    return mean;
}

MpcController::Rollout MpcController::rolloutSequence(const LFPC& base,
    const ControlSequence& controls,const Eigen::Vector3d& goal,
    const Eigen::Vector2d* final_goal,double final_radius)
{
    Rollout result;
    if(controls.size()!=size_t(cfg_.horizon_steps))return result;
    LFPC state;state.copyState(base);
    double elapsed=0.,motion_cost=0.,previous_action=0.;bool arrived=false;
    for(size_t n=0;n<controls.size();++n) {
        const double action=controls[n];
        if(!std::isfinite(action) || std::abs(action)>cfg_.max_api ||
           !advance(state,action,result.path,elapsed,&result.times,&result.yaws))return result;
        motion_cost+=cfg_.w_steer*std::abs(action)+(n>0?cfg_.w_dapi*std::abs(action-previous_action):0.);
        previous_action=action;++result.steps;
        arrived=arrived || distanceToGoal(state,goal)<cfg_.goal_arrival_threshold;
        // Local arrival only suppresses terminal distance cost, never safety or
        // suffix walking cost. Only the actual final goal permits idle termination.
        result.terminal=final_goal && (state.getCOMPos().head<2>()-*final_goal).norm()<final_radius;
        if(result.terminal)break;
    }
    result.cost=motion_cost+(arrived?0.:(cfg_.w_goal/cfg_.horizon_steps)*distanceToGoal(state,goal));
    result.valid=std::isfinite(result.cost);
    return result;
}

Eigen::Vector3d MpcController::plan(const LFPC::Ptr& base,const Eigen::Vector3d& goal,
                                   const Eigen::Vector2d* final_goal,double final_radius)
{
    const auto started=std::chrono::steady_clock::now();
    footprint_local_=HumanCaneFootprint::relative(cfg_.footprint,0.);
    best_path_.clear();best_times_.clear();best_yaws_.clear();best_controls_.clear();collision_witness_=CollisionWitness();
    last_plan_valid_=false;last_debug_metrics_=DebugMetrics();optimization_evidence_=OptimizationEvidence();
    last_debug_metrics_.timed_collision_active=!timed_collision_.empty() || !timed_input_valid_;
    const int N=cfg_.horizon_steps,K=cfg_.num_samples,I=cfg_.mpc_iters;
    auto& evidence=optimization_evidence_;evidence.horizon=N;evidence.samples_per_iteration=K;
    Eigen::Vector3d command=Eigen::Vector3d::Zero();
    auto finish=[&] {
        if(!last_plan_valid_)resetWarmStart();
        last_plan_time_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
        last_debug_metrics_.plan_time_ms=last_plan_time_ms_;last_debug_metrics_.plan_valid=last_plan_valid_;
        publishOptimizationEvidence();return command;
    };
    bool config=HumanCaneFootprint::validConfig(cfg_.footprint) && (!final_goal || (final_goal->allFinite() && std::isfinite(final_radius) && final_radius>0.));
    for(double x:{cfg_.nominal_al,cfg_.nominal_aw,cfg_.min_al,cfg_.max_al,cfg_.max_aw,cfg_.max_api,
                  cfg_.w_steer,cfg_.w_dapi,cfg_.w_goal,cfg_.goal_arrival_threshold,cfg_.sigma_api,cfg_.temperature})
        config=config && std::isfinite(x);
    config=config && N>=1 && N<=32 && K>=1 && K<=4096 && I>=1 && I<=16 && cfg_.random_seed>=0 &&
        cfg_.min_al>=0 && cfg_.nominal_al>=cfg_.min_al && cfg_.nominal_al<=cfg_.max_al &&
        cfg_.nominal_aw>=0 && cfg_.nominal_aw<=cfg_.max_aw && cfg_.max_api>=0 && cfg_.sigma_api>=0 &&
        cfg_.max_api<=3.14159265358979323846 && cfg_.temperature>0 && cfg_.w_steer>=0 && cfg_.w_dapi>=0 && cfg_.w_goal>=0 && cfg_.goal_arrival_threshold>=0;
    const bool inside=base && std::isfinite(base->getNextIterState()(2)) && spatialFree(base->getCOMPos().head<2>());
    if(!config || !timed_input_valid_ || !collision_ || !goal.allFinite() || !has_convex_corridor_ || !inside ||
       !(collision_->hasStaticGlobalMap()?collision_->isStaticTraversable(base->getCOMPos().x(),base->getCOMPos().y()):
         collision_->isTraversable(base->getCOMPos().x(),base->getCOMPos().y()))) {
        publishSampleTrajectories({},Eigen::VectorXd());return finish();
    }
    timed_root_=base->getCOMPos().head<2>();timed_root_yaw_=base->getNextIterState()(2);
    if(!timedFree(timed_root_,0.,timed_observation_age_,timed_root_yaw_)) {
        publishSampleTrajectories({},Eigen::VectorXd());return finish();
    }
    ControlSequence mean=warm_start_.size()==size_t(N)?warm_start_:ControlSequence(N,0.);
    ControlSequence fallback_controls,optimized_controls;
    Rollout fallback,optimized;
    bool has_weighted=false;
    for(int iteration=0;iteration<I;++iteration) {
        const auto samples=sampleSequences(mean,fallback_controls);
        std::vector<double> costs(K,std::numeric_limits<double>::infinity());
        std::vector<std::vector<Eigen::Vector3d>> display_paths;
        if(iteration+1==I)display_paths.reserve(K);
        for(int k=0;k<K;++k) {
            auto rollout=rolloutSequence(*base,samples[k],goal,final_goal,final_radius);
            ++evidence.evaluated;
            if(rollout.valid) {
                costs[k]=rollout.cost;++evidence.valid;
                if(!fallback.valid || rollout.cost<fallback.cost) {fallback=rollout;fallback_controls=samples[k];}
                // Retain the initial feasible proposal so optimization never
                // forces a higher-cost command than a validated warm start.
                if(iteration==0 && k==0) {optimized=rollout;optimized_controls=samples[k];}
            }
            if(iteration+1==I)display_paths.push_back(std::move(rollout.path));
        }
        if(iteration+1==I) {
            Eigen::VectorXd c(K);for(int k=0;k<K;++k)c[k]=costs[k];
            publishSampleTrajectories(display_paths,c);
        }
        const auto weights=computeWeights(costs);
        double sum=0.,squares=0.;for(double w:weights){sum+=w;squares+=w*w;}
        evidence.effective_samples=squares>0.?1./squares:0.;
        double weighted_cost=std::numeric_limits<double>::infinity();
        if(sum>0.) {
            auto proposal=weightedUpdate(samples,weights);
            auto checked=rolloutSequence(*base,proposal,goal,final_goal,final_radius);
            ++evidence.weighted_checks;
            if(checked.valid) {
                has_weighted=true;weighted_cost=checked.cost;mean=proposal;
                if(!optimized.valid || checked.cost<optimized.cost) {optimized=checked;optimized_controls=proposal;}
                if(!fallback.valid || checked.cost<fallback.cost) {fallback=checked;fallback_controls=proposal;}
            } else {
                ++evidence.weighted_rejects;
                if(fallback.valid)mean=fallback_controls;
            }
        } else if(fallback.valid)mean=fallback_controls;
        ++evidence.iterations;
        evidence.best_verified_costs.push_back(fallback.cost);
        evidence.weighted_costs.push_back(weighted_cost);
    }
    last_debug_metrics_.num_samples=evidence.evaluated;
    last_debug_metrics_.valid_trajectory_count=last_debug_metrics_.corridor_feasible_trajectory_count=evidence.valid;
    last_debug_metrics_.valid_sample_ratio=double(evidence.valid)/evidence.evaluated;
    last_debug_metrics_.candidate_inside_corridor_ratio=-1.;
    last_debug_metrics_.convex_corridor_segments=convex_corridor_segments_.size();
    last_debug_metrics_.weighted_checked=evidence.weighted_checks>0;
    last_debug_metrics_.weighted_fallback=evidence.weighted_checks>0 && !has_weighted;
    last_debug_metrics_.best_total_cost=fallback.cost;
    if(fallback.valid) {
        const auto& selected=has_weighted?optimized:fallback;
        const auto& controls=has_weighted?optimized_controls:fallback_controls;
        auto replay=rolloutSequence(*base,controls,goal,final_goal,final_radius);
        evidence.reroll_valid=replay.valid && replay.cost==selected.cost && replay.steps==selected.steps &&
                              replay.terminal==selected.terminal;
        if(evidence.reroll_valid) {
            last_plan_valid_=true;last_debug_metrics_.executed_total_cost=replay.cost;
            best_path_=std::move(replay.path);best_times_=std::move(replay.times);best_yaws_=std::move(replay.yaws);
            best_controls_.assign(controls.begin(),controls.begin()+replay.steps);
            evidence.selected_steps=replay.steps;evidence.selected_terminal=replay.terminal;
            command=Eigen::Vector3d(cfg_.nominal_al,cfg_.nominal_aw,controls.front());
            warm_start_=controls;std::rotate(warm_start_.begin(),warm_start_.begin()+1,warm_start_.end());
            warm_start_.back()=0.;
        }
    }
    return finish();
}

} // namespace cane_planner
