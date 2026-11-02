#include <plan_manager.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace cane_planner {
namespace {
using V=Eigen::Vector2d;
struct PoseSample {Eigen::Vector3d position;double yaw,phase;};
}
void PlannerManager::resetHumanResponseLocked(double yaw) {
    human_response_.reset(yaw);human_response_guard_stopped_=false;
    human_response_guard_interventions_=0;execution_rebuild_corridor_=false;
    if(human_response_vis_pub_) {
        visualization_msgs::MarkerArray markers;visualization_msgs::Marker m;
        m.header.frame_id="world";m.header.stamp=ros::Time::now();
        m.action=visualization_msgs::Marker::DELETEALL;markers.markers.push_back(m);
        human_response_vis_pub_.publish(markers);
    }
}

void PlannerManager::publishHumanResponseLocked(const std::string& reason,bool moved) {
    if(!human_response_enabled_)return;
    const auto now=ros::Time::now();
    const bool stopped=reason!="OK";
    if(stopped && !human_response_guard_stopped_)++human_response_guard_interventions_;
    human_response_guard_stopped_=stopped;
    if(moved)human_response_.acknowledge(now.toSec());
    const auto& desired=human_response_.desired();const auto& selected=human_response_.selected();
    const auto& applied=human_response_.applied();
    const double actual_yaw=lfpc_model_->getNextIterState().z();
    const double applied_delay=applied.sequence?human_response_.appliedAt()-applied.issued:-1.;
    if(human_response_diag_pub_) {
        std_msgs::Float64MultiArray m;std_msgs::MultiArrayDimension d;
        d.label="human_steering_response_v1;0=version,1=epoch,2=enabled,3=configured_delay,4=issued_seq,5=desired_yaw,6=selected_seq,7=selected_issued,8=selected_due,9=selected_yaw,10=applied_seq,11=applied_at,12=applied_delay,13=actual_yaw,14=pending,15=guard_stopped,16=guard_interventions,17=applied_issued,18=applied_yaw";
        d.size=d.stride=19;m.layout.dim.push_back(d);
        m.data={1.,now.toSec(),1.,human_response_.delay(),double(desired.sequence),desired.yaw,
            double(selected.sequence),selected.issued,selected.due,selected.yaw,double(applied.sequence),
            human_response_.appliedAt(),applied_delay,actual_yaw,double(human_response_.pending()),
            stopped?1.:0.,double(human_response_guard_interventions_),applied.issued,applied.yaw};
        human_response_diag_pub_.publish(m);
    }
    if(human_response_vis_pub_) {
        visualization_msgs::MarkerArray markers;visualization_msgs::Marker m;
        m.header.frame_id="world";m.header.stamp=now;m.ns="human_response";
        m.action=visualization_msgs::Marker::ADD;m.type=visualization_msgs::Marker::ARROW;
        m.pose.orientation.w=1.;m.scale.x=.04;m.scale.y=.10;m.scale.z=.10;
        m.lifetime=ros::Duration(.5);m.color.a=1.;
        const auto com=lfpc_model_->getCOMPos();
        for(int id=0;id<2;++id) {
            m.id=id;m.points.clear();m.color.r=id==0?1.:.1;m.color.g=id==0?.8:.6;m.color.b=id==0?0.:1.;
            const double yaw=id==0 && desired.sequence?desired.yaw:actual_yaw;
            geometry_msgs::Point a,b;a.x=com.x();a.y=com.y();a.z=id==0?.95:.8;b=a;
            b.x+=.9*std::cos(yaw);b.y+=.9*std::sin(yaw);m.points={a,b};markers.markers.push_back(m);
        }
        m.id=2;m.type=visualization_msgs::Marker::TEXT_VIEW_FACING;m.points.clear();
        m.pose.position.x=com.x();m.pose.position.y=com.y();m.pose.position.z=1.25;
        m.scale.z=.16;m.color.r=1.;m.color.g=stopped?0.:1.;m.color.b=stopped?0.:1.;
        std::ostringstream label;label<<std::fixed<<std::setprecision(2)<<"Steering delay "<<human_response_.delay()<<" s; applied ";
        if(applied_delay<0.)label<<"pending";else label<<applied_delay<<" s";
        label<<"; guards "<<human_response_guard_interventions_;if(stopped)label<<" ["<<reason<<"]";
        m.text=label.str();markers.markers.push_back(m);human_response_vis_pub_.publish(markers);
    }
}

void PlannerManager::publishExecutionDecisionLocked(const std::string& reason) {
    const bool stopped=reason!="OK";
    if(mpc_stop_advice_pub_){std_msgs::Bool m;m.data=stopped;mpc_stop_advice_pub_.publish(m);}
    if(mpc_stop_reason_pub_){std_msgs::String m;m.data=reason;mpc_stop_reason_pub_.publish(m);}
    if(stopped) {
        ROS_WARN_THROTTLE(1.,"[MPC execution] STOP reason=%s phase=%.3f/%.3f",
            reason.c_str(),execution_phase_,execution_root_.stepDuration());
        publishTimedCollisionLocked(ros::Time::now(),false);
        if(reason=="TIMED_EXECUTION_COLLISION" && execution_witness_.valid && mpc_timed_collision_pub_) {
            visualization_msgs::MarkerArray markers;visualization_msgs::Marker m;
            m.header.frame_id="world";m.header.stamp=ros::Time::now();m.action=visualization_msgs::Marker::ADD;
            m.ns="timed_rejected";m.id=0;m.type=visualization_msgs::Marker::LINE_STRIP;m.pose.orientation.w=1.;
            m.scale.x=.04;m.color.r=1.;m.color.a=1.;m.lifetime=ros::Duration(.15);
            const auto polygon=mpc_controller_->robotFootprintAt(execution_witness_.com,execution_witness_.yaw);
            for(size_t j=0;j<=polygon.size();++j){geometry_msgs::Point p;p.x=polygon[j%polygon.size()].x();p.y=polygon[j%polygon.size()].y();p.z=.3;m.points.push_back(p);}
            markers.markers.push_back(m);m.ns="timed_labels";m.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
            m.scale.z=.18;m.points.clear();m.pose.position.x=execution_witness_.com.x();m.pose.position.y=execution_witness_.com.y();m.pose.position.z=.65;m.text="Rejected";markers.markers.push_back(m);
            mpc_timed_collision_pub_.publish(markers);
        }
        if(mpc_best_traj_pub_) {
            visualization_msgs::Marker marker;marker.header.frame_id="world";marker.header.stamp=ros::Time::now();
            marker.ns="mpc_best";marker.id=0;marker.action=visualization_msgs::Marker::DELETE;mpc_best_traj_pub_.publish(marker);
        }
    }
}

bool PlannerManager::advanceExecutionLocked(const ros::Time& now,bool admission_only) {
    if(!execution_active_)return false;
    const auto began=std::chrono::steady_clock::now();
    execution_witness_=MpcController::CollisionWitness();
    const double previous_phase=execution_phase_;
    const double duration=execution_root_.stepDuration();
    const double elapsed=(now-execution_clock_).toSec();
    // A delayed/paused plant cannot catch up by teleporting through a whole step.
    const double delta=admission_only?0.:std::max(0.,std::min(.04,elapsed));
    const double target=std::min(duration,execution_phase_+delta);
    auto finish=[&](const std::string& reason) {
        execution_stop_reason_=reason;
        if(admission_only)return reason=="OK"; // manager publishes the admission decision once
        execution_clock_=now;
        const bool stopped=reason!="OK";
        if(stopped)publishSimOdom(true);
        publishExecutionDecisionLocked(reason);
        // A successful preflight has not yet moved or applied the selected cue.
        publishHumanResponseLocked(reason,!stopped && execution_phase_>previous_phase);
        auto pose=lfpc_model_->getCOMPos();pose.z()=lfpc_model_->getNextIterState().z();
        publishHumanCaneLocked(pose,false);
        if(execution_diag_pub_) {
            std_msgs::Float64MultiArray m;
            std_msgs::MultiArrayDimension d;d.label="lfpc_execution_v1;0=version,1=epoch,2=phase,3=duration,4=advance,5=stopped,6=generation,7=validation_seconds";d.size=d.stride=8;m.layout.dim.push_back(d);
            m.data={1.,now.toSec(),execution_phase_,duration,stopped?0.:execution_phase_-previous_phase,stopped?1.:0.,double(pedestrian_generation_),std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()};
            execution_diag_pub_.publish(m);
        }
        return !stopped;
    };
    if(now.isZero() || elapsed<0. || duration<=0.)return finish("EXECUTION_CLOCK_INVALID");
    if(pedestrians_enabled_ && !pedestrianDataValid(now))return finish("DYNAMIC_DATA_INVALID");
    const PedestrianFrameToken frame{pedestrian_generation_,pedestrians_enabled_,pedestrian_header_.stamp,pedestrian_receipt_};
    std::vector<PoseSample> samples;
    const auto current=lfpc_model_->getCOMPos();const double current_yaw=lfpc_model_->getNextIterState().z();
    const bool rebuild=execution_rebuild_corridor_ || collision_->getStaticMapRevision()!=execution_map_revision_;
    samples.push_back({current,current_yaw,execution_phase_});
    // Include boundary rotation, every nominal LFPC sample, and extra20ms poses.
    if(execution_phase_==0.) {
        const int n=std::max(1,int(std::ceil(std::abs(execution_control_.z())/.0872664625997165)));
        for(int i=1;i<=n;++i)samples.push_back({current,current_yaw+execution_control_.z()*i/n,0.});
    }
    std::vector<double> phases{target,duration};
    for(double t=execution_phase_+.02;t<duration;t+=.02)phases.push_back(t);
    LFPC canonical;canonical.copyState(execution_root_);canonical.SetCtrlParams(execution_control_);canonical.updateOneStep();
    for(double t:canonical.getStepCOMTimes())if(t>=execution_phase_)phases.push_back(t);
    std::sort(phases.begin(),phases.end());
    phases.erase(std::unique(phases.begin(),phases.end(),[](double a,double b){return std::abs(a-b)<1e-10;}),phases.end());
    LFPC executed;
    if(!execution_root_.sampleControlledStep(execution_control_,target,executed))return finish("EXECUTION_MODEL_INVALID");
    for(double t:phases) {
        LFPC state;
        if(!execution_root_.sampleControlledStep(execution_control_,t,state))return finish("EXECUTION_MODEL_INVALID");
        samples.push_back({state.getCOMPos(),state.getNextIterState().z(),t});
    }
    const auto shape=mpc_controller_->robotFootprintAt(V::Zero(),0.);
    TimedPedestrianCollision dynamic_check;
    // Rebuild geometry from this accepted observation. The grid used for the
    // social force includes the same local planning domain as normal FIRI.
    std::vector<TimedPedestrianCollision::Obstacle> obstacles;
    pedestrian_geometry_.clear();
    ConvexCorridor::Grid social_grid;
    CollisionDetection::MapRevision snapshot_revision;
    if(pedestrians_enabled_ || rebuild) {
        Eigen::Vector2i social_size;
        std::string why;
        V social_lo=current.head<2>(),social_hi=social_lo;
        if(pedestrians_enabled_) {
            const auto reference=ConvexCorridor::referencePath(global_waypoints_,global_wp_idx_,current.head<2>(),corridor_planning_length_);
            for(const auto& p:reference){social_lo=social_lo.cwiseMin(p);social_hi=social_hi.cwiseMax(p);}
        }
        for(const auto& s:samples){social_lo=social_lo.cwiseMin(s.position.head<2>());social_hi=social_hi.cwiseMax(s.position.head<2>());}
        social_lo=social_lo.cwiseMin(canonical.getFootPosition());social_hi=social_hi.cwiseMax(canonical.getFootPosition());
        const auto padding=V::Constant(convex_corridor_->getConfig().local_radius).eval();
        if(!collision_->getStaticCorridorGrid(social_lo-padding,social_hi+padding,social_grid.origin,social_grid.resolution,
            social_size,social_grid.blocked,why,&snapshot_revision))return finish("INVALID_STATIC_CORRIDOR");
        social_grid.width=social_size.x();social_grid.height=social_size.y();
        const Eigen::Vector3d pose(current.x(),current.y(),current_yaw);
        if(pedestrians_enabled_)for(size_t i=0;i<pedestrian_observations_.size();++i) {
            const auto& o=pedestrian_observations_[i];
            std::vector<PedestrianPolygon::Observation> neighbors;
            for(size_t j=0;j<pedestrian_observations_.size();++j)if(j!=i)neighbors.push_back(pedestrian_observations_[j]);
            const auto geometry=PedestrianPolygon::build(o,pose,social_grid,pedestrian_config_,neighbors);
            if(!geometry.valid){pedestrian_geometry_.clear();return finish("DYNAMIC_DATA_INVALID");}
            pedestrian_geometry_.push_back(geometry);
            obstacles.push_back({geometry.hull,mpc_controller_->timedCollisionEnabled()?o.velocity:V::Zero()});
        }
    }
    if(rebuild) {
        // Re-certify the actual remaining support phase, not the remote global
        // reference. A blocked suffix must not prevent reaching the boundary
        // where a new global route and MPPI control can be adopted safely.
        std::vector<V> remainder{current.head<2>()};
        for(const auto& s:samples)
            if((s.position.head<2>()-remainder.back()).norm()>=1e-9)remainder.push_back(s.position.head<2>());
        ConvexCorridor builder;
        auto config=convex_corridor_->getConfig();
        config.footprint=mpc_controller_->footprintConfig();config.root_yaw=current_yaw;
        builder.setConfig(config);
        const auto rebuilt=builder.buildStatic(remainder,social_grid,currentPedestrianPolygonsLocked(ros::Time::now()));
        if(!rebuilt.feasible) {
            ROS_WARN_THROTTLE(1.,"[MPC execution] remaining-step corridor failed: %s",
                ConvexCorridor::failureReasonName(rebuilt.failure_reason));
            return finish("INVALID_STATIC_CORRIDOR");
        }
        execution_corridor_=rebuilt;execution_map_revision_=snapshot_revision;
        execution_rebuild_corridor_=false;
    }
    if(!dynamic_check.bind(obstacles,shape))return finish("DYNAMIC_DATA_INVALID");
    auto in_corridor=[&](const V& p) {
        return p.allFinite() && std::any_of(execution_corridor_.segments.begin(),execution_corridor_.segments.end(),
            [&](const ConvexCorridor::Segment& region){return ConvexCorridor::contains(region,p,1e-6);});
    };
    if(!in_corridor(canonical.getFootPosition()))return finish("INVALID_STATIC_CORRIDOR");
    for(const auto& s:samples) {
        // The spatial corridor constrains COM samples and the support foot. Future human-cane
        // envelopes are used only for time-matched pedestrian collision checks.
        if(!in_corridor(s.position.head<2>()))return finish("INVALID_STATIC_CORRIDOR");
    }
    if(collision_->getStaticMapRevision()!=execution_map_revision_)return finish("INVALID_STATIC_CORRIDOR");
    const auto authorized=ros::Time::now();
    if(!authorizePedestrianFrameLocked(frame,authorized))return finish("DYNAMIC_DATA_INVALID");
    const double age=frame.enabled && mpc_controller_->timedCollisionEnabled()?(authorized-frame.stamp).toSec():0.;
    for(const auto& s:samples)
        if(dynamic_check.intersects(s.position.head<2>(),age+std::max(0.,s.phase-target),s.yaw)) {
            execution_witness_={true,s.position.head<2>(),std::max(0.,s.phase-target),age,s.yaw};
            return finish("TIMED_EXECUTION_COLLISION");
        }
    if(admission_only)return true;
    lfpc_model_->copyState(executed);execution_phase_=target;
    odom_pos_=executed.getCOMPos();odom_vel_=executed.getNextIterState();
    if(target>=duration-1e-12) {
        const auto state=executed.getState();odom_vel_.x()=state(0);odom_vel_.y()=state(1);
        execution_active_=false;
    }
    mpc_step_path_.push_back(odom_pos_);publishSimOdom();
    // During execution the timed display represents only the checked remainder.
    if(mpc_timed_path_pub_ && mpc_timed_collision_pub_) {
        nav_msgs::Path path;path.header.frame_id="world";path.header.stamp=authorized;
        visualization_msgs::MarkerArray markers;visualization_msgs::Marker marker;
        marker.header=path.header;marker.action=visualization_msgs::Marker::DELETEALL;markers.markers.push_back(marker);
        if(mpc_timed_plan_pub_ && (!frame.enabled || obstacles.empty()))
            mpc_timed_plan_pub_.publish(markers);
        if(frame.enabled && !obstacles.empty()) {
            size_t display_count=0;
            for(const auto& s:samples)if(s.phase>=target) {
                const double future=s.phase-target;
                geometry_msgs::PoseStamped p;p.header=path.header;p.header.stamp=authorized+ros::Duration(future);p.pose.position.x=s.position.x();p.pose.position.y=s.position.y();p.pose.orientation=tf::createQuaternionMsgFromYaw(s.yaw);path.poses.push_back(p);
                if(display_count++%3!=0)continue;
                marker.pose=geometry_msgs::Pose();
                marker.action=visualization_msgs::Marker::ADD;marker.type=visualization_msgs::Marker::LINE_STRIP;marker.pose.orientation.w=1.;marker.scale.x=.015;marker.color.r=.1;marker.color.g=1.;marker.color.b=1.;marker.color.a=.6;marker.lifetime=ros::Duration(.15);marker.ns="timed_robot";marker.id=display_count;marker.points.clear();
                auto polygon=mpc_controller_->robotFootprintAt(s.position.head<2>(),s.yaw);
                for(size_t j=0;j<=polygon.size();++j){geometry_msgs::Point q;q.x=polygon[j%polygon.size()].x();q.y=polygon[j%polygon.size()].y();q.z=.25;marker.points.push_back(q);}markers.markers.push_back(marker);
                marker.ns="timed_pedestrian";marker.color.r=1.;marker.color.g=.3;marker.points.clear();
                for(size_t i=0;i<obstacles.size();++i) {
                    marker.id=display_count*32+i;
                    marker.points.clear();const auto& obstacle=obstacles[i];
                    for(size_t j=0;j<=obstacle.hull.size();++j){const auto v=(obstacle.hull[j%obstacle.hull.size()]+obstacle.velocity*(age+future)).eval();geometry_msgs::Point q;q.x=v.x();q.y=v.y();q.z=.25;marker.points.push_back(q);}markers.markers.push_back(marker);
                }
                marker.id=display_count;
                marker.ns="timed_labels";marker.type=visualization_msgs::Marker::TEXT_VIEW_FACING;marker.scale.z=.16;
                marker.points.clear();marker.pose.position.x=s.position.x();marker.pose.position.y=s.position.y();marker.pose.position.z=.6;
                marker.text="+"+std::to_string(future).substr(0,4)+" s";markers.markers.push_back(marker);
            }
        }
        mpc_timed_path_pub_.publish(path);mpc_timed_collision_pub_.publish(markers);
    }
    return finish("OK");
}

void PlannerManager::executionCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> fsm_lock(fsmMutex_);
    bool request_plan=false;
    {
        std::lock_guard<std::mutex> lock(dynObsMutex_);
        if(!timed_sim_execution_ || !simulation_ || gazebo_sim_ || planner_!=3 ||
            exec_state_!=MPC_STEP || !initial_route_active_ || !have_target_)return;
        if(execution_active_)advanceExecutionLocked(ros::Time::now());
        request_plan=!execution_active_;
    }
    if(request_plan) {
        if(mpcSimStep()) {
            if(mpc_reached_goal_) {have_target_=false;changeFSMExecState(WAIT_TARGET);}
            else changeFSMExecState(REPLAN_TRAJ);
        }
    }
}
}
