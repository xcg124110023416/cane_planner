#ifndef _MPC_CONTROLLER_H_
#define _MPC_CONTROLLER_H_

#include <Eigen/Eigen>
#include <limits>
#include <random>
#include <vector>

#include <ros/ros.h>

#include <path_searching/lfpc.h>
#include <path_searching/convex_corridor.h>
#include <path_searching/timed_pedestrian_collision.h>
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

        int num_samples = 256;
        int mpc_iters = 2;
        double temperature = 0.4;
        double sigma_api = 0.35;
        int random_seed = 0; // reproducible runs; does not reseed every cycle

        // Control bounds
        double max_al = 0.4;
        double min_al = 0.05;
        double max_aw = 0.15;
        double max_api = 0.5236;     // ~30 deg

        // Cost weights
        double w_steer = 0.5;
        double w_goal = 10.0;
        double w_dapi = 0.0;    // steering rate penalty: |api[n] - api[n-1]|

        // Local arrival suppresses distance cost; it never truncates safety checks.
        double goal_arrival_threshold = 0.3;

        // Warm-start nominal
        double nominal_al = 0.25;
        double nominal_aw = 0.0;

        bool timed_collision_enable = true;
        double timed_robot_radius = 0.25;
        HumanCaneFootprint::Config footprint;

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
        int timed_collision_checks = 0;
        bool timed_collision_active = false;
        bool timed_execution_checked = false;
        bool timed_execution_safe = false;
        int static_reject_count = 0;
        int corridor_reject_count = 0;
        int convex_corridor_reject_count = 0;
        int convex_corridor_segments = 0;
        double max_convex_corridor_violation = 0.0;
        double candidate_inside_corridor_ratio = 1.0;
        int valid_trajectory_count = 0;
        int corridor_feasible_trajectory_count = 0;
        bool corridor_evaluated = false;
        int num_samples = 0;
        bool plan_valid = false;
        bool weighted_checked = false;
        bool weighted_fallback = false;
        double executed_total_cost = std::numeric_limits<double>::infinity();
    };

    using ControlSequence = std::vector<double>;
    struct OptimizationEvidence {
        int horizon=0, samples_per_iteration=0, iterations=0;
        int evaluated=0, valid=0, weighted_checks=0, weighted_rejects=0, selected_steps=0;
        bool reroll_valid=false, selected_terminal=false;
        double effective_samples=0.;
        std::vector<double> best_verified_costs;
        std::vector<double> weighted_costs;
    };
    const OptimizationEvidence& optimizationEvidence() const { return optimization_evidence_; }
    const ControlSequence& getBestControls() const { return best_controls_; }

    MpcController();
    ~MpcController();

    void setParam(ros::NodeHandle &nh);
    void init();
    void reset();
    void resetWarmStart();
    // Seed a missing proposal from current geometry; preserve a retained sequence.
    // plan() fully revalidates this proposal and never assumes it is safe.
    bool seedReference(const LFPC& base,const std::vector<Eigen::Vector2d>& reference);
    void setCollision(const CollisionDetection::Ptr &col);
    void setConvexCorridor(const std::vector<ConvexCorridor::Segment> &segments);
    void clearConvexCorridor();
    bool setTimedPedestrians(const std::vector<TimedPedestrianCollision::Obstacle>& obstacles,
                             double observation_age);
    void clearTimedPedestrians();
    // Called with the SAME observation at the actual execution epoch, before integration.
    bool revalidateTimedPath(double observation_age);
    const TimedPedestrianCollision& timedCollision() const { return timed_collision_; }
    double timedObservationAge() const { return timed_observation_age_; }
    const std::vector<double>& getBestTimes() const { return best_times_; }
    const std::vector<double>& getBestYaws() const { return best_yaws_; }
    const HumanCaneFootprint::Config& footprintConfig() const { return cfg_.footprint; }
    bool timedCollisionEnabled() const { return cfg_.timed_collision_enable; }
    HumanCaneFootprint::Polygon robotFootprintAt(const Eigen::Vector2d&,double yaw) const;
    struct CollisionWitness {
        bool valid = false;
        Eigen::Vector2d com = Eigen::Vector2d::Zero();
        double future_time = 0., observation_age = 0.;
        double yaw = 0.;
    };
    const CollisionWitness& collisionWitness() const { return collision_witness_; }

    // One LFPC command, with spatial and optional time-synchronized hard checks.
    Eigen::Vector3d plan(const LFPC::Ptr &lfpc_base,
                         const Eigen::Vector3d &goal_pos,
                         const Eigen::Vector2d* final_goal = nullptr, double final_radius = 0.);

    // Return the best predicted CoM path from last plan (for visualization)
    std::vector<Eigen::Vector3d> getBestPath() const { return best_path_; }

    double lastPlanTimeMs() const { return last_plan_time_ms_; }

    bool lastPlanValid() const { return last_plan_valid_; }

    DebugMetrics getDebugMetrics() const { return last_debug_metrics_; }

    typedef shared_ptr<MpcController> Ptr;

private:
    friend class StaticMpcTestAccess;
    Config cfg_;
    CollisionDetection::Ptr collision_;
    bool has_convex_corridor_ = false;
    std::vector<ConvexCorridor::Segment> convex_corridor_segments_;

    struct Rollout {
        bool valid=false, terminal=false;
        int steps=0;
        double cost=std::numeric_limits<double>::infinity();
        std::vector<Eigen::Vector3d> path;
        std::vector<double> times;
        std::vector<double> yaws;
    };
    OptimizationEvidence optimization_evidence_;
    ros::Publisher optimization_pub_;
    void publishOptimizationEvidence();
    ControlSequence warm_start_, best_controls_;
    std::mt19937 rng_{0};
    std::normal_distribution<double> normal_dist_{0.,1.};
    std::vector<ControlSequence> sampleSequences(const ControlSequence& mean,
                                                const ControlSequence& incumbent);
    std::vector<double> computeWeights(const std::vector<double>& costs) const;
    ControlSequence weightedUpdate(const std::vector<ControlSequence>& samples,
                                   const std::vector<double>& weights) const;
    Rollout rolloutSequence(const LFPC& base, const ControlSequence& controls,
                            const Eigen::Vector3d& goal, const Eigen::Vector2d* final_goal,
                            double final_radius);
    bool advance(LFPC& state, double action, std::vector<Eigen::Vector3d>& path,
                 double& elapsed, std::vector<double>* times = nullptr, std::vector<double>* yaws = nullptr);
    bool spatialFree(const Eigen::Vector2d& com,double yaw);
    HumanCaneFootprint::Polygon relativeFootprint(double yaw) const;
    HumanCaneFootprint::Polygon footprint_local_;
    bool timedFree(const Eigen::Vector2d& com, double future_time, double age,double yaw = 0.);
    TimedPedestrianCollision timed_collision_;
    bool timed_input_valid_ = true;
    double timed_observation_age_ = 0.;
    Eigen::Vector2d timed_root_ = Eigen::Vector2d::Zero();
    double timed_root_yaw_ = 0.;
    CollisionWitness collision_witness_;
    double distanceToGoal(LFPC&,const Eigen::Vector3d&) const;
    // Best path from last plan
    std::vector<Eigen::Vector3d> best_path_;
    std::vector<double> best_times_;
    std::vector<double> best_yaws_;

    // Timing
    double last_plan_time_ms_;
    bool last_plan_valid_ = false;
    DebugMetrics last_debug_metrics_;

    // Bounded diagnostic output; never used by planning or sampling.
    ros::Publisher sample_vis_pub_;
    bool sample_vis_enable_ = true;
    int sample_vis_count_ = 20;
    void publishSampleTrajectories(
        const std::vector<std::vector<Eigen::Vector3d>> &paths,
        const Eigen::VectorXd &costs);


};

} // namespace cane_planner

#endif // _MPC_CONTROLLER_H_
