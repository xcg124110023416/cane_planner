#ifndef _MPC_CONTROLLER_H_
#define _MPC_CONTROLLER_H_

#include <Eigen/Eigen>
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

        // Hard constraint: risk > threshold → INF cost (dynamic obstacles only)
        double risk_hard_threshold = 8.5;

        // Static obstacle: high penalty per colliding point (effectively hard)
        double static_penalty = 500.0;
        double w_static = 1.0;

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

    // Main API: plan one step, returns [al, aw, api]
    Eigen::Vector3d plan(const LFPC::Ptr &lfpc_base,
                         const Eigen::Vector3d &goal_pos,
                         const std::vector<Eigen::Vector3d> &obs_pos,
                         const std::vector<Eigen::Vector3d> &obs_vel);

    // Return the best predicted CoM path from last plan (for visualization)
    std::vector<Eigen::Vector3d> getBestPath() const { return best_path_; }

    // Access risk field for external visualization
    const DynamicRiskField& getRiskField() const { return risk_field_; }

    double lastPlanTimeMs() const { return last_plan_time_ms_; }

    bool lastPlanValid() const { return last_plan_valid_; }

    typedef shared_ptr<MpcController> Ptr;

private:
    Config cfg_;
    LFPC::Ptr lfpc_model_;
    CollisionDetection::Ptr collision_;
    DynamicRiskField risk_field_;

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

    // Internal methods
    Eigen::MatrixXd makeNominalSequence(int N);
    std::vector<Eigen::MatrixXd> sampleSequences(const Eigen::MatrixXd &mean,
                                                  int N, int K);
    void rolloutBatch(const LFPC::Ptr &lfpc_base,
                      const std::vector<Eigen::MatrixXd> &samples,
                      const Eigen::Vector3d &goal_pos,
                      const std::vector<Eigen::Vector3d> &obs_pos,
                      const std::vector<Eigen::Vector3d> &obs_vel,
                      Eigen::VectorXd &costs,
                      std::vector<std::vector<Eigen::Vector3d>> &paths);
    Eigen::VectorXd computeWeights(const Eigen::VectorXd &costs);
    Eigen::MatrixXd weightedUpdate(const std::vector<Eigen::MatrixXd> &samples,
                                    const Eigen::VectorXd &weights,
                                    int N);
    void shiftSequence(Eigen::MatrixXd &mean, int N);
    void ensurePool(int K);
};

} // namespace cane_planner

#endif // _MPC_CONTROLLER_H_
