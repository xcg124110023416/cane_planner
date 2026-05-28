#ifndef _MPC_CONTROLLER_H_
#define _MPC_CONTROLLER_H_

#include <Eigen/Eigen>
#include <limits>
#include <random>
#include <vector>

#include <ros/ros.h>

#include <path_searching/lfpc.h>
#include <path_searching/dynamic_risk_field.h>
#include <plan_env/collision_detection.h>

namespace cane_planner
{

class MpcController
{
public:
    struct Config
    {
        // Horizon
        int horizon_steps = 6;       // N: lookahead steps

        // MPPI sampling
        int num_samples = 100;       // K
        double temperature = 0.1;    // lower = more greedy
        int mpc_iters = 1;           // MPPI iterations per plan

        // Control bounds
        double max_al = 0.4;
        double min_al = 0.05;
        double max_aw = 0.15;
        double max_api = 0.5236;     // ~30 deg

        // Exploration noise std
        double sigma_al = 0.06;
        double sigma_aw = 0.04;
        double sigma_api = 0.12;

        // Cost weights
        double w_move = 1.0;
        double w_steer = 0.5;
        double w_risk = 2.0;
        double w_goal = 10.0;
        double w_dapi = 0.0;    // steering rate penalty: |api[n] - api[n-1]|
        double interaction_w_front_pass = 0.0;
        bool interaction_enable_behind_corridor = false;
        double interaction_w_behind_corridor = 0.0;
        double interaction_corridor_width = 0.8;
        double interaction_front_buffer = 0.25;
        double interaction_target_dist_cap = 3.0;

        // Optional adaptive dynamic-risk weight. It only scales soft risk cost;
        // hard safety checks remain unchanged.
        bool adaptive_risk_weight = false;
        double adaptive_risk_max_scale = 2.0;
        double adaptive_risk_clearance = 0.8;
        double adaptive_risk_ttc = 1.2;

        // Hard constraint: risk > threshold → INF cost (dynamic obstacles only)
        bool dynamic_hard_reject_enable = true;
        double risk_hard_threshold = 8.5;

        // Static obstacle: high penalty per colliding point (effectively hard)
        double static_penalty = 500.0;
        double w_static = 1.0;

        // Dynamic obstacle geometry. Sizes are interpreted as obstacle boxes
        // already inflated by the planner manager when available.
        bool use_dynamic_size = true;
        double dynamic_safety_margin = 0.15;
        double dynamic_min_radius = 0.35;
        bool dynamic_uncertainty_enable = true;
        double dynamic_uncertainty_base_rate = 0.08;      // m/s radius growth for prediction time
        double dynamic_uncertainty_crossing_rate = 0.25;  // extra m/s for lateral crossing motion
        double dynamic_uncertainty_slow_speed = 0.20;     // low speed implies weak intent evidence
        double dynamic_uncertainty_path_lateral = 1.0;    // apply crossing growth near planned path corridor

        // SDF proximity cost: repulsive gradient around obstacles
        double w_prox = 5.0;
        double prox_margin = 0.6;  // wider than collision margin (0.3)

        // Use best trajectory instead of weighted average (avoids mode collapse)
        bool use_best = true;

        // Fix step length/width to nominal (human provides, only optimize api)
        bool fix_step_params = false;

        // Early goal arrival: stop rollout and skip terminal cost if within this radius
        double goal_arrival_threshold = 0.3;

        // FOV limitation: points beyond this distance from robot are treated as traversable
        // Set to <= 0 to disable (full map access)
        double fov_range = 5.0;

        // Warm-start nominal
        double nominal_al = 0.25;
        double nominal_aw = 0.0;
        double nominal_api = 0.0;
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
        double risk_weight_scale = 1.0;
        int dynamic_reject_count = 0;
        int static_reject_count = 0;
        int num_samples = 0;
        bool plan_valid = false;
    };

    struct InteractionContext
    {
        bool enabled = false;
        int scene = 0;
        int mode = 0;
        Eigen::Vector2d path_forward = Eigen::Vector2d::UnitX();
        Eigen::Vector2d path_left = Eigen::Vector2d::UnitY();
        Eigen::Vector2d robot_pos = Eigen::Vector2d::Zero();
        Eigen::Vector2d ped_pos = Eigen::Vector2d::Zero();
        Eigen::Vector2d ped_vel = Eigen::Vector2d::Zero();
        Eigen::Vector2d crossing_point = Eigen::Vector2d::Zero();
        Eigen::Vector2d behind_target = Eigen::Vector2d::Zero();
        bool target_valid = false;
    };

    MpcController();
    ~MpcController();

    void setParam(ros::NodeHandle &nh);
    void init();
    void reset();
    void resetWarmStart();
    void setModel(const LFPC::Ptr &model);
    void setCollision(const CollisionDetection::Ptr &col);
    void setRiskField(const DynamicRiskField &rf);

    void setDynamicObstacles(const std::vector<Eigen::Vector3d> &pos,
                             const std::vector<Eigen::Vector3d> &vel);
    void setInteractionContext(const InteractionContext &ctx);
    void clearInteractionContext();

    // Main API: plan one step, returns [al, aw, api]
    Eigen::Vector3d plan(const LFPC::Ptr &lfpc_base,
                         const Eigen::Vector3d &goal_pos,
                         const std::vector<Eigen::Vector3d> &obs_pos,
                         const std::vector<Eigen::Vector3d> &obs_vel,
                         const std::vector<Eigen::Vector3d> &obs_size = {});

    // Return the best predicted CoM path from last plan (for visualization)
    std::vector<Eigen::Vector3d> getBestPath() const { return best_path_; }

    // Access risk field for external visualization
    const DynamicRiskField& getRiskField() const { return risk_field_; }

    double lastPlanTimeMs() const { return last_plan_time_ms_; }

    bool lastPlanValid() const { return last_plan_valid_; }

    DebugMetrics getDebugMetrics() const { return last_debug_metrics_; }

    typedef shared_ptr<MpcController> Ptr;

private:
    Config cfg_;
    LFPC::Ptr lfpc_model_;
    CollisionDetection::Ptr collision_;
    DynamicRiskField risk_field_;
    InteractionContext interaction_ctx_;

    // Pre-allocated LFPC pool for rollout
    std::vector<LFPC::Ptr> lfpc_pool_;
    bool pool_initialized_;

    // Warm-start control sequence: N x 3
    Eigen::MatrixXd warm_start_;

    // RNG
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;

    // Best path from last plan
    std::vector<Eigen::Vector3d> best_path_;

    // Timing
    double last_plan_time_ms_;
    bool last_plan_valid_ = true;
    DebugMetrics last_debug_metrics_;

    // Internal methods
    Eigen::MatrixXd makeNominalSequence(int N);
    std::vector<Eigen::MatrixXd> sampleSequences(const Eigen::MatrixXd &mean,
                                                  int N, int K);
    void rolloutBatch(const LFPC::Ptr &lfpc_base,
                      const std::vector<Eigen::MatrixXd> &samples,
                      const Eigen::Vector3d &goal_pos,
                      const std::vector<Eigen::Vector3d> &obs_pos,
                      const std::vector<Eigen::Vector3d> &obs_vel,
                      const std::vector<Eigen::Vector3d> &obs_size,
                      Eigen::VectorXd &costs,
                      std::vector<std::vector<Eigen::Vector3d>> &paths,
                      std::vector<double> &sample_min_dynamic_clearances,
                      std::vector<double> &sample_min_cpa_times);
    Eigen::VectorXd computeWeights(const Eigen::VectorXd &costs);
    Eigen::MatrixXd weightedUpdate(const std::vector<Eigen::MatrixXd> &samples,
                                    const Eigen::VectorXd &weights,
                                    int N);
    void shiftSequence(Eigen::MatrixXd &mean, int N);
    void ensurePool(int K);
};

} // namespace cane_planner

#endif // _MPC_CONTROLLER_H_
