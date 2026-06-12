#ifndef _KINEMATIC_MPPI_CONTROLLER_H_
#define _KINEMATIC_MPPI_CONTROLLER_H_

#include <Eigen/Eigen>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include <ros/ros.h>

#include <path_searching/dynamic_risk_field.h>
#include <plan_env/collision_detection.h>

namespace cane_planner
{

class KinematicMppiController
{
public:
    struct Config
    {
        int horizon_steps = 10;
        int num_samples = 200;
        int mppi_iters = 1;
        double dt = 0.35;

        double min_v = 0.0;
        double max_v = 1.4;
        double max_omega = 1.0;
        double sigma_v = 0.12;
        double sigma_omega = 0.30;

        double nominal_v = 1.14;
        double nominal_omega = 0.0;

        double w_goal = 25.0;
        double w_turn = 0.5;
        double w_domega = 1.0;
        double w_risk = 2.0;
        double w_static = 1.0;
        double static_penalty = 500.0;
        double w_prox = 5.0;
        double prox_margin = 0.6;

        bool dynamic_hard_reject_enable = true;
        double risk_hard_threshold = 2.0;
        bool use_dynamic_size = true;
        double dynamic_safety_margin = 0.05;
        double dynamic_min_radius = 0.20;
        double robot_radius = 0.25;
        double goal_arrival_threshold = 0.30;
        double fov_range = 5.0;
    };

    struct DebugMetrics
    {
        double plan_time_ms = 0.0;
        double valid_sample_ratio = 0.0;
        double best_total_cost = std::numeric_limits<double>::infinity();
        double min_dynamic_clearance = std::numeric_limits<double>::infinity();
        double min_cpa_time = std::numeric_limits<double>::infinity();
        double best_min_dynamic_clearance = std::numeric_limits<double>::infinity();
        double best_min_cpa_time = std::numeric_limits<double>::infinity();
        int dynamic_reject_count = 0;
        int static_reject_count = 0;
        int num_samples = 0;
        bool plan_valid = false;
    };

    KinematicMppiController();

    void setParam(ros::NodeHandle &nh);
    void init();
    void reset();
    void resetWarmStart();
    void setCollision(const CollisionDetection::Ptr &col);
    void setRiskField(const DynamicRiskField &rf);

    Eigen::Vector2d plan(const Eigen::Vector3d &start_state,
                         const Eigen::Vector3d &goal_pos,
                         const std::vector<Eigen::Vector3d> &obs_pos,
                         const std::vector<Eigen::Vector3d> &obs_vel,
                         const std::vector<Eigen::Vector3d> &obs_size = {});

    std::vector<Eigen::Vector3d> getBestPath() const { return best_path_; }
    const DynamicRiskField& getRiskField() const { return risk_field_; }
    bool lastPlanValid() const { return last_plan_valid_; }
    double lastPlanTimeMs() const { return last_plan_time_ms_; }
    DebugMetrics getDebugMetrics() const { return last_debug_metrics_; }
    double getDt() const { return cfg_.dt; }

    typedef std::shared_ptr<KinematicMppiController> Ptr;

private:
    Config cfg_;
    CollisionDetection::Ptr collision_;
    DynamicRiskField risk_field_;

    Eigen::MatrixXd warm_start_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;

    std::vector<Eigen::Vector3d> best_path_;
    bool last_plan_valid_ = false;
    double last_plan_time_ms_ = 0.0;
    DebugMetrics last_debug_metrics_;

    Eigen::MatrixXd makeNominalSequence(int N) const;
    std::vector<Eigen::MatrixXd> sampleSequences(const Eigen::MatrixXd &mean,
                                                  int N,
                                                  int K);
    void shiftSequence(Eigen::MatrixXd &seq, int N) const;
    void rolloutBatch(const Eigen::Vector3d &start_state,
                      const std::vector<Eigen::MatrixXd> &samples,
                      const Eigen::Vector3d &goal_pos,
                      const std::vector<Eigen::Vector3d> &obs_pos,
                      const std::vector<Eigen::Vector3d> &obs_vel,
                      const std::vector<Eigen::Vector3d> &obs_size,
                      Eigen::VectorXd &costs,
                      std::vector<std::vector<Eigen::Vector3d>> &paths,
                      std::vector<double> &sample_min_dynamic_clearances,
                      std::vector<double> &sample_min_cpa_times);
};

} // namespace cane_planner

#endif // _KINEMATIC_MPPI_CONTROLLER_H_
