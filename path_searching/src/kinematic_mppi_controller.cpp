#include <path_searching/kinematic_mppi_controller.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace cane_planner
{

namespace
{
double wrapAngle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}
}

KinematicMppiController::KinematicMppiController()
    : normal_dist_(0.0, 1.0)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng_.seed(seed);
}

void KinematicMppiController::setParam(ros::NodeHandle &nh)
{
    nh.param("mpc_kin/horizon_steps", cfg_.horizon_steps, 10);
    nh.param("mpc_kin/num_samples", cfg_.num_samples, 200);
    nh.param("mpc_kin/mppi_iters", cfg_.mppi_iters, 1);
    nh.param("mpc_kin/dt", cfg_.dt, 0.35);

    nh.param("mpc_kin/min_v", cfg_.min_v, 0.0);
    nh.param("mpc_kin/max_v", cfg_.max_v, 1.4);
    nh.param("mpc_kin/max_omega", cfg_.max_omega, 1.0);
    nh.param("mpc_kin/sigma_v", cfg_.sigma_v, 0.12);
    nh.param("mpc_kin/sigma_omega", cfg_.sigma_omega, 0.30);
    nh.param("mpc_kin/nominal_v", cfg_.nominal_v, 1.14);
    nh.param("mpc_kin/nominal_omega", cfg_.nominal_omega, 0.0);

    nh.param("mpc/w_goal", cfg_.w_goal, 25.0);
    nh.param("mpc/w_steer", cfg_.w_turn, 0.5);
    nh.param("mpc/w_dapi", cfg_.w_domega, 1.0);
    nh.param("mpc/w_risk", cfg_.w_risk, 2.0);
    nh.param("mpc/w_static", cfg_.w_static, 1.0);
    nh.param("mpc/static_penalty", cfg_.static_penalty, 500.0);
    nh.param("mpc/w_prox", cfg_.w_prox, 5.0);
    nh.param("mpc/prox_margin", cfg_.prox_margin, 0.6);
    nh.param("mpc/dynamic_hard_reject_enable", cfg_.dynamic_hard_reject_enable, true);
    nh.param("mpc/use_dynamic_size", cfg_.use_dynamic_size, true);
    nh.param("mpc/dynamic_safety_margin", cfg_.dynamic_safety_margin, 0.05);
    nh.param("mpc/dynamic_min_radius", cfg_.dynamic_min_radius, 0.20);
    nh.param("mpc/goal_arrival_threshold", cfg_.goal_arrival_threshold, 0.30);
    nh.param("mpc/fov_range", cfg_.fov_range, 5.0);
    nh.param("mpc_kin/robot_radius", cfg_.robot_radius, 0.25);

    DynamicRiskField::Config rf_cfg;
    nh.param("mpc/risk_tau", rf_cfg.tau, 1.0);
    nh.param("mpc/risk_A", rf_cfg.A_risk, 5.0);
    nh.param("mpc/risk_sigma_x", rf_cfg.sigma_x, 0.4);
    nh.param("mpc/risk_sigma_y", rf_cfg.sigma_y, 0.22);
    nh.param("mpc/risk_cutoff", rf_cfg.cutoff_dist, 3.0);
    nh.param("mpc/risk_halo_scale", rf_cfg.halo_scale, 0.0);
    nh.param("mpc/risk_halo_ratio", rf_cfg.halo_ratio, 0.25);
    nh.param("mpc/risk_cpa_enable", rf_cfg.cpa_enable, false);
    nh.param("mpc/risk_cpa_weight", rf_cfg.cpa_weight, 0.6);
    nh.param("mpc/risk_cpa_sigma_d", rf_cfg.cpa_sigma_d, 0.65);
    nh.param("mpc/risk_cpa_tau", rf_cfg.cpa_tau, 1.0);
    nh.param("mpc/risk_cpa_time_horizon", rf_cfg.cpa_time_horizon, 2.0);
    nh.param("mpc/risk_cpa_cutoff_dist", rf_cfg.cpa_cutoff_dist, 3.0);
    risk_field_.setConfig(rf_cfg);

    double hard_ratio;
    nh.param("mpc/risk_hard_threshold_ratio", hard_ratio, 0.5);
    cfg_.risk_hard_threshold = rf_cfg.A_risk * hard_ratio;
}

void KinematicMppiController::init()
{
    reset();
}

void KinematicMppiController::reset()
{
    warm_start_.resize(0, 0);
    best_path_.clear();
    last_plan_valid_ = false;
    last_plan_time_ms_ = 0.0;
    last_debug_metrics_ = DebugMetrics();
}

void KinematicMppiController::resetWarmStart()
{
    warm_start_.resize(0, 0);
}

void KinematicMppiController::setCollision(const CollisionDetection::Ptr &col)
{
    collision_ = col;
}

void KinematicMppiController::setRiskField(const DynamicRiskField &rf)
{
    risk_field_ = rf;
}

Eigen::MatrixXd KinematicMppiController::makeNominalSequence(int N) const
{
    Eigen::MatrixXd seq(N, 2);
    seq.col(0).setConstant(cfg_.nominal_v);
    seq.col(1).setConstant(cfg_.nominal_omega);
    return seq;
}

std::vector<Eigen::MatrixXd> KinematicMppiController::sampleSequences(
    const Eigen::MatrixXd &mean, int N, int K)
{
    std::vector<Eigen::MatrixXd> samples(K);
    for (int k = 0; k < K; ++k)
    {
        samples[k].resize(N, 2);
        for (int n = 0; n < N; ++n)
        {
            double v = mean(n, 0);
            double omega = mean(n, 1);
            if (k > 0)
            {
                v += cfg_.sigma_v * normal_dist_(rng_);
                omega += cfg_.sigma_omega * normal_dist_(rng_);
            }
            v = std::max(cfg_.min_v, std::min(cfg_.max_v, v));
            omega = std::max(-cfg_.max_omega, std::min(cfg_.max_omega, omega));
            samples[k](n, 0) = v;
            samples[k](n, 1) = omega;
        }
    }
    return samples;
}

void KinematicMppiController::shiftSequence(Eigen::MatrixXd &seq, int N) const
{
    if (seq.rows() != N || N <= 1)
        return;
    for (int i = 0; i < N - 1; ++i)
        seq.row(i) = seq.row(i + 1);
    seq(N - 1, 0) = cfg_.nominal_v;
    seq(N - 1, 1) = cfg_.nominal_omega;
}

Eigen::Vector2d KinematicMppiController::plan(
    const Eigen::Vector3d &start_state,
    const Eigen::Vector3d &goal_pos,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size)
{
    auto t_start = std::chrono::high_resolution_clock::now();

    const int N = cfg_.horizon_steps;
    const int K = cfg_.num_samples;
    last_debug_metrics_ = DebugMetrics();
    last_debug_metrics_.num_samples = K;

    if (warm_start_.rows() != N || warm_start_.cols() != 2)
        warm_start_ = makeNominalSequence(N);

    Eigen::MatrixXd mean_seq = warm_start_;
    Eigen::Vector2d cmd(0.0, 0.0);
    int best_idx_global = -1;

    for (int iter = 0; iter < cfg_.mppi_iters; ++iter)
    {
        auto samples = sampleSequences(mean_seq, N, K);
        Eigen::VectorXd costs(K);
        std::vector<std::vector<Eigen::Vector3d>> paths(K);
        std::vector<double> sample_min_dynamic_clearances;
        std::vector<double> sample_min_cpa_times;
        rolloutBatch(start_state, samples, goal_pos, obs_pos, obs_vel, obs_size,
                     costs, paths, sample_min_dynamic_clearances, sample_min_cpa_times);

        double best_cost = std::numeric_limits<double>::infinity();
        int best_idx = -1;
        int valid_count = 0;
        for (int k = 0; k < K; ++k)
        {
            if (std::isfinite(costs(k)))
            {
                valid_count++;
                if (costs(k) < best_cost)
                {
                    best_cost = costs(k);
                    best_idx = k;
                }
            }
        }

        last_debug_metrics_.valid_sample_ratio = K > 0 ? (double)valid_count / (double)K : 0.0;
        last_debug_metrics_.best_total_cost = best_cost;
        if (best_idx >= 0)
        {
            cmd = samples[best_idx].row(0);
            mean_seq = samples[best_idx];
            best_path_ = paths[best_idx];
            best_idx_global = best_idx;
            if (best_idx < (int)sample_min_dynamic_clearances.size())
                last_debug_metrics_.best_min_dynamic_clearance = sample_min_dynamic_clearances[best_idx];
            if (best_idx < (int)sample_min_cpa_times.size())
                last_debug_metrics_.best_min_cpa_time = sample_min_cpa_times[best_idx];
        }
    }

    shiftSequence(mean_seq, N);
    warm_start_ = mean_seq;
    last_plan_valid_ = best_idx_global >= 0;
    if (!last_plan_valid_)
        cmd.setZero();

    auto t_end = std::chrono::high_resolution_clock::now();
    last_plan_time_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    last_debug_metrics_.plan_time_ms = last_plan_time_ms_;
    last_debug_metrics_.plan_valid = last_plan_valid_;
    return cmd;
}

void KinematicMppiController::rolloutBatch(
    const Eigen::Vector3d &start_state,
    const std::vector<Eigen::MatrixXd> &samples,
    const Eigen::Vector3d &goal_pos,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size,
    Eigen::VectorXd &costs,
    std::vector<std::vector<Eigen::Vector3d>> &paths,
    std::vector<double> &sample_min_dynamic_clearances,
    std::vector<double> &sample_min_cpa_times)
{
    const int K = (int)samples.size();
    const int N = cfg_.horizon_steps;
    costs.setConstant(K, std::numeric_limits<double>::infinity());
    sample_min_dynamic_clearances.assign(K, std::numeric_limits<double>::infinity());
    sample_min_cpa_times.assign(K, std::numeric_limits<double>::infinity());

    last_debug_metrics_.dynamic_reject_count = 0;
    last_debug_metrics_.static_reject_count = 0;
    last_debug_metrics_.min_dynamic_clearance = std::numeric_limits<double>::infinity();
    last_debug_metrics_.min_cpa_time = std::numeric_limits<double>::infinity();

    const double base_x = start_state(0);
    const double base_y = start_state(1);
    const bool limit_fov = cfg_.fov_range > 0.0;
    const double fov_sq = cfg_.fov_range * cfg_.fov_range;
    const double slice_h = collision_ ? collision_->getSliceHeight() : 0.0;

    for (int k = 0; k < K; ++k)
    {
        double x = start_state(0);
        double y = start_state(1);
        double theta = start_state(2);
        double prev_omega = samples[k](0, 1);
        double total_cost = 0.0;
        bool static_collided = false;
        bool dyn_collided = false;
        bool arrived = false;
        double rollout_min_dynamic_clearance = std::numeric_limits<double>::infinity();
        double rollout_min_cpa_time = std::numeric_limits<double>::infinity();

        for (int n = 0; n < N; ++n)
        {
            const double v = samples[k](n, 0);
            const double omega = samples[k](n, 1);
            theta = wrapAngle(theta + omega * cfg_.dt);
            const double prev_x = x;
            const double prev_y = y;
            x += v * std::cos(theta) * cfg_.dt;
            y += v * std::sin(theta) * cfg_.dt;
            const double point_t = (n + 1) * cfg_.dt;
            paths[k].push_back(Eigen::Vector3d(x, y, theta));

            bool in_fov = true;
            if (limit_fov)
            {
                const double dx = x - base_x;
                const double dy = y - base_y;
                in_fov = dx * dx + dy * dy <= fov_sq;
            }

            double prox_cost = 0.0;
            double risk_cost = 0.0;
            int risk_evals = 0;

            if (in_fov)
            {
                if (collision_ && !collision_->isTraversable(x, y))
                {
                    static_collided = true;
                    break;
                }
                if (collision_)
                {
                    const double dist = collision_->sdf_map_->getDistance(Eigen::Vector3d(x, y, slice_h));
                    if (dist < cfg_.prox_margin)
                    {
                        const double d = cfg_.prox_margin - dist;
                        prox_cost += cfg_.w_prox * d * d;
                    }
                }

                for (size_t oi = 0; oi < obs_pos.size(); ++oi)
                {
                    const double vx = obs_vel[oi](0);
                    const double vy = obs_vel[oi](1);
                    const double ox = obs_pos[oi](0) + point_t * vx;
                    const double oy = obs_pos[oi](1) + point_t * vy;
                    double dyn_radius = cfg_.dynamic_min_radius;
                    if (cfg_.use_dynamic_size && oi < obs_size.size())
                        dyn_radius = std::max(dyn_radius, 0.5 * std::max(obs_size[oi](0), obs_size[oi](1)));
                    dyn_radius += cfg_.dynamic_safety_margin + cfg_.robot_radius;

                    const double ddx = x - ox;
                    const double ddy = y - oy;
                    const double dyn_dist = std::sqrt(ddx * ddx + ddy * ddy);
                    const double dyn_clearance = dyn_dist - dyn_radius;
                    rollout_min_dynamic_clearance = std::min(rollout_min_dynamic_clearance, dyn_clearance);
                    last_debug_metrics_.min_dynamic_clearance =
                        std::min(last_debug_metrics_.min_dynamic_clearance, dyn_clearance);

                    if (cfg_.dynamic_hard_reject_enable && dyn_clearance < 0.0)
                    {
                        dyn_collided = true;
                        break;
                    }

                    const double rvx = (x - prev_x) / std::max(1e-3, cfg_.dt);
                    const double rvy = (y - prev_y) / std::max(1e-3, cfg_.dt);
                    const double rel_vx = rvx - vx;
                    const double rel_vy = rvy - vy;
                    const double rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy;
                    if (rel_speed_sq > 1e-6)
                    {
                        const double t_cpa = -(ddx * rel_vx + ddy * rel_vy) / rel_speed_sq;
                        if (t_cpa >= 0.0)
                        {
                            rollout_min_cpa_time = std::min(rollout_min_cpa_time, t_cpa);
                            last_debug_metrics_.min_cpa_time =
                                std::min(last_debug_metrics_.min_cpa_time, t_cpa);
                        }
                    }

                    double r = risk_field_.getIndividualCostFast(x, y, ox, oy, vx, vy);
                    r += risk_field_.getTimeConflictCostFast(
                        x, y,
                        (x - prev_x) / std::max(1e-3, cfg_.dt),
                        (y - prev_y) / std::max(1e-3, cfg_.dt),
                        ox, oy, vx, vy, dyn_radius);
                    if (std::isfinite(r) && cfg_.dynamic_hard_reject_enable &&
                        r > cfg_.risk_hard_threshold)
                    {
                        dyn_collided = true;
                        break;
                    }
                    if (std::isfinite(r))
                    {
                        risk_cost += r;
                        risk_evals++;
                    }
                }
            }

            if (dyn_collided || static_collided)
                break;

            if (risk_evals > 0)
                risk_cost = cfg_.w_risk * risk_cost / risk_evals;

            const double turn_cost = cfg_.w_turn * std::abs(omega);
            const double smooth_cost = n > 0 ? cfg_.w_domega * std::abs(omega - prev_omega) : 0.0;
            total_cost += turn_cost + smooth_cost + risk_cost + prox_cost;
            prev_omega = omega;

            const double goal_dx = x - goal_pos(0);
            const double goal_dy = y - goal_pos(1);
            if (std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy) < cfg_.goal_arrival_threshold)
            {
                arrived = true;
                break;
            }
        }

        if (static_collided || dyn_collided)
        {
            total_cost = std::numeric_limits<double>::infinity();
            if (static_collided)
                last_debug_metrics_.static_reject_count++;
            if (dyn_collided)
                last_debug_metrics_.dynamic_reject_count++;
        }
        else if (!arrived)
        {
            const double dx = x - goal_pos(0);
            const double dy = y - goal_pos(1);
            total_cost += cfg_.w_goal * std::sqrt(dx * dx + dy * dy);
        }

        costs(k) = total_cost;
        sample_min_dynamic_clearances[k] = rollout_min_dynamic_clearance;
        sample_min_cpa_times[k] = rollout_min_cpa_time;
    }
}

} // namespace cane_planner
