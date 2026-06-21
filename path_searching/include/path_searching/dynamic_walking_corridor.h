#ifndef _DYNAMIC_WALKING_CORRIDOR_H_
#define _DYNAMIC_WALKING_CORRIDOR_H_

#include <Eigen/Eigen>
#include <functional>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <plan_env/collision_detection.h>
#include <path_searching/timed_trajectory.h>

namespace cane_planner
{

class DynamicWalkingCorridor
{
public:
    struct Config
    {
        bool enable = true;
        bool lateral_candidates_enable = false;
        double length = 4.0;
        double half_width = 0.45;
        double min_half_width = 0.25;
        double lateral_shift = 0.8;
        double prediction_horizon = 3.0;
        double prediction_dt = 0.25;
        double robot_speed = 1.0;
        double robot_radius = 0.25;
        double dynamic_min_radius = 0.20;
        double dynamic_safety_margin = 0.15;
        double dynamic_progress_margin = 0.50;
        bool dynamic_blocking_enable = false;
        bool static_centerline_opt_enable = true;
        bool static_truncate_enable = true;
        double static_opt_lateral_range = 1.0;
        double static_opt_lateral_step = 0.1;
        double static_opt_smooth_weight = 2.0;
        double static_min_feasible_length = 0.8;
        double static_truncate_backoff = 0.2;
        double static_start_grace_length = 0.4;
        double lateral_offset_weight = 0.2;
        double front_pass_weight = 4.0;
        double front_pass_sigma_s = 1.2;
        double front_pass_sigma_l = 1.0;
        double front_pass_length = 3.0;
        double front_pass_lateral = 1.8;
        double static_sample_ds = 0.4;
        double static_sample_dl = 0.3;
    };

    struct Candidate
    {
        int id = 0;
        double lateral_offset = 0.0;
        double total_cost = 0.0;
        double front_pass_cost = 0.0;
        bool feasible = true;
        bool blocked_static = false;
        bool blocked_dynamic = false;
        bool truncated_static = false;
        int dynamic_block_count = 0;
        double length = 0.0;
        double half_width = 0.0;
        bool static_min_width_valid = false;
        double static_min_half_width = 0.0;
        double static_min_s = 0.0;
        Eigen::Vector2d static_min_point = Eigen::Vector2d::Zero();
        Eigen::Vector2d start = Eigen::Vector2d::Zero();
        Eigen::Vector2d end = Eigen::Vector2d::Zero();
        Eigen::Vector2d forward = Eigen::Vector2d::UnitX();
        Eigen::Vector2d left = Eigen::Vector2d::UnitY();
        std::vector<Eigen::Vector2d> centerline;
    };

    struct Result
    {
        bool has_feasible = false;
        Candidate selected;
        std::vector<Candidate> candidates;
    };

    DynamicWalkingCorridor() = default;

    void setParam(ros::NodeHandle &nh);
    void setConfig(const Config &cfg) { cfg_ = cfg; }
    Config getConfig() const { return cfg_; }
    void setCollision(const CollisionDetection::Ptr &col) { collision_ = col; }

    Result plan(const Eigen::Vector2d &robot_pos,
                const Eigen::Vector2d &path_forward,
                const std::vector<Eigen::Vector3d> &obs_pos,
                const std::vector<Eigen::Vector3d> &obs_vel,
                const std::vector<Eigen::Vector3d> &obs_size = {}) const;

    Result plan(const std::vector<Eigen::Vector2d> &reference_path,
                const std::vector<Eigen::Vector3d> &obs_pos,
                const std::vector<Eigen::Vector3d> &obs_vel,
                const std::vector<Eigen::Vector3d> &obs_size = {}) const;

    Result plan(const TimedTrajectory &nominal_trajectory,
                const std::vector<Eigen::Vector3d> &obs_pos,
                const std::vector<Eigen::Vector3d> &obs_vel,
                const std::vector<Eigen::Vector3d> &obs_size = {}) const;

    static double outsideDistance(const Candidate &candidate,
                                  const Eigen::Vector2d &point);
    static bool optimizeCenterlineForStaticMap(
        const std::vector<Eigen::Vector2d> &reference_path,
        const Config &cfg,
        const std::function<bool(double, double)> &is_traversable,
        std::vector<Eigen::Vector2d> &optimized_path,
        double &optimized_half_width);

    typedef std::shared_ptr<DynamicWalkingCorridor> Ptr;

private:
    Config cfg_;
    CollisionDetection::Ptr collision_;

    Candidate evaluateCandidate(int id,
                                double lateral_offset,
                                const Eigen::Vector2d &robot_pos,
                                const Eigen::Vector2d &forward,
                                const Eigen::Vector2d &left,
                                const std::vector<Eigen::Vector3d> &obs_pos,
                                const std::vector<Eigen::Vector3d> &obs_vel,
                                const std::vector<Eigen::Vector3d> &obs_size) const;
    Candidate evaluateCandidate(int id,
                                double lateral_offset,
                                const std::vector<Eigen::Vector2d> &reference_path,
                                const std::vector<Eigen::Vector3d> &obs_pos,
                                const std::vector<Eigen::Vector3d> &obs_vel,
                                const std::vector<Eigen::Vector3d> &obs_size) const;

    bool shrinkStaticWidth(Candidate &candidate) const;
    bool isDynamicallyBlocked(const Candidate &candidate,
                              const std::vector<Eigen::Vector3d> &obs_pos,
                              const std::vector<Eigen::Vector3d> &obs_vel,
                              const std::vector<Eigen::Vector3d> &obs_size,
                              int &block_count) const;
    double computeFrontPassCost(const Candidate &candidate,
                                const std::vector<Eigen::Vector3d> &obs_pos,
                                const std::vector<Eigen::Vector3d> &obs_vel) const;
};

} // namespace cane_planner

#endif // _DYNAMIC_WALKING_CORRIDOR_H_
