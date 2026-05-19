#include <path_searching/mpc_controller.h>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace cane_planner
{

MpcController::MpcController()
    : pool_initialized_(false)
    , normal_dist_(0.0, 1.0)
    , last_plan_time_ms_(0.0)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng_.seed(seed);
}

MpcController::~MpcController()
{
    lfpc_pool_.clear();
}

void MpcController::setParam(ros::NodeHandle &nh)
{
    nh.param("mpc/horizon_steps", cfg_.horizon_steps, 6);
    nh.param("mpc/num_samples", cfg_.num_samples, 100);
    nh.param("mpc/temperature", cfg_.temperature, 0.1);
    nh.param("mpc/mpc_iters", cfg_.mpc_iters, 1);

    nh.param("mpc/max_al", cfg_.max_al, 0.4);
    nh.param("mpc/min_al", cfg_.min_al, 0.05);
    nh.param("mpc/max_aw", cfg_.max_aw, 0.15);
    nh.param("mpc/max_api", cfg_.max_api, 0.5236);

    nh.param("mpc/sigma_al", cfg_.sigma_al, 0.06);
    nh.param("mpc/sigma_aw", cfg_.sigma_aw, 0.04);
    nh.param("mpc/sigma_api", cfg_.sigma_api, 0.12);

    nh.param("mpc/w_move", cfg_.w_move, 1.0);
    nh.param("mpc/w_steer", cfg_.w_steer, 0.5);
    nh.param("mpc/w_risk", cfg_.w_risk, 2.0);
    nh.param("mpc/w_goal", cfg_.w_goal, 10.0);
    nh.param("mpc/w_bias", cfg_.w_bias, 0.3);
    nh.param("mpc/w_dapi", cfg_.w_dapi, 0.0);

    nh.param("mpc/static_penalty", cfg_.static_penalty, 500.0);
    nh.param("mpc/w_static", cfg_.w_static, 1.0);
    nh.param("mpc/w_prox", cfg_.w_prox, 5.0);
    nh.param("mpc/prox_margin", cfg_.prox_margin, 0.6);
    nh.param("mpc/use_best", cfg_.use_best, true);
    nh.param("mpc/fix_step_params", cfg_.fix_step_params, false);
    nh.param("mpc/goal_arrival_threshold", cfg_.goal_arrival_threshold, 0.3);
    nh.param("mpc/fov_range", cfg_.fov_range, 5.0);

    nh.param("mpc/nominal_al", cfg_.nominal_al, 0.25);
    nh.param("mpc/nominal_aw", cfg_.nominal_aw, 0.0);
    nh.param("mpc/nominal_api", cfg_.nominal_api, 0.0);

    // Risk field config
    DynamicRiskField::Config rf_cfg;
    nh.param("mpc/risk_tau", rf_cfg.tau, 1.0);
    nh.param("mpc/risk_A", rf_cfg.A_risk, 5.0);
    nh.param("mpc/risk_sigma_x", rf_cfg.sigma_x, 0.4);
    nh.param("mpc/risk_sigma_y", rf_cfg.sigma_y, 0.22);
    nh.param("mpc/risk_cutoff", rf_cfg.cutoff_dist, 3.0);
    risk_field_.setConfig(rf_cfg);

    // Hard threshold = peak × ratio, auto-scales with risk_A
    double hard_ratio;
    nh.param("mpc/risk_hard_threshold_ratio", hard_ratio, 0.5);
    cfg_.risk_hard_threshold = rf_cfg.A_risk * hard_ratio;
}

void MpcController::init()
{
    warm_start_.resize(0, 0);
    best_path_.clear();
}

void MpcController::reset()
{
    warm_start_.resize(0, 0);
    best_path_.clear();
}

void MpcController::resetWarmStart()
{
    warm_start_.resize(0, 0);  // forces fresh nominal init on next plan()
}

void MpcController::setModel(const LFPC::Ptr &model)
{
    lfpc_model_ = model;
}

void MpcController::setCollision(const CollisionDetection::Ptr &col)
{
    collision_ = col;
}

void MpcController::setRiskField(const DynamicRiskField &rf)
{
    risk_field_ = rf;
}

void MpcController::setDynamicObstacles(const std::vector<Eigen::Vector3d> &pos,
                                        const std::vector<Eigen::Vector3d> &vel)
{
    // No-op: obstacles are passed in plan() directly.
    // This method exists for API compatibility.
}

// =========================================================================
// Main API
// =========================================================================

Eigen::Vector3d MpcController::plan(const LFPC::Ptr &lfpc_base,
                                     const Eigen::Vector3d &goal_pos,
                                     const std::vector<Eigen::Vector3d> &obs_pos,
                                     const std::vector<Eigen::Vector3d> &obs_vel)
{
    auto t_start = std::chrono::high_resolution_clock::now();

    int N = cfg_.horizon_steps;
    int K = cfg_.num_samples;

    ensurePool(K);

    // Initialize or shift warm-start
    if (warm_start_.rows() != N)
    {
        warm_start_ = makeNominalSequence(N);
        // Tiny random bias on first control to break left-right symmetry
        warm_start_(0, 2) += normal_dist_(rng_) * 0.03;
    }

    Eigen::MatrixXd mean_seq = warm_start_;

    Eigen::Vector3d control_cmd;
    int best_idx_global = -1;

    for (int iter = 0; iter < cfg_.mpc_iters; ++iter)
    {
        // 1. Sample K sequences around current mean
        std::vector<Eigen::MatrixXd> samples = sampleSequences(mean_seq, N, K);

        // 2. Rollout and compute costs
        Eigen::VectorXd costs(K);
        std::vector<std::vector<Eigen::Vector3d>> paths(K);
        rolloutBatch(lfpc_base, samples, goal_pos, obs_pos, obs_vel, costs, paths);

        // 3. Find best trajectory
        int best_idx = -1;
        double min_cost = std::numeric_limits<double>::max();
        for (int kk = 0; kk < K; ++kk)
        {
            if (costs(kk) < min_cost)
            {
                min_cost = costs(kk);
                best_idx = kk;
            }
        }

        if (cfg_.use_best && best_idx >= 0)
        {
            // Best-of-K: avoids MPPI mode collapse with multimodal obstacles
            control_cmd = samples[best_idx].row(0);
            mean_seq = samples[best_idx];
            best_path_ = paths[best_idx];
            best_idx_global = best_idx;
        }
        else
        {
            // MPPI weighted average
            Eigen::VectorXd weights = computeWeights(costs);
            mean_seq = weightedUpdate(samples, weights, N);
            control_cmd = mean_seq.row(0);
            if (best_idx >= 0)
            {
                best_path_ = paths[best_idx];
                best_idx_global = best_idx;
            }
        }
    }

    // Shift warm-start for next cycle
    shiftSequence(mean_seq, N);
    warm_start_ = mean_seq;

    // No valid trajectory found (all INF) → full stop, wait for obstacle to pass
    if (best_idx_global < 0)
    {
        control_cmd << 0.0, 0.0, 0.0;
        last_plan_valid_ = false;
    }
    else
    {
        last_plan_valid_ = true;
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    last_plan_time_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    return control_cmd;
}

// =========================================================================
// Pool management
// =========================================================================

void MpcController::ensurePool(int K)
{
    if ((int)lfpc_pool_.size() >= K)
        return;

    lfpc_pool_.clear();
    lfpc_pool_.reserve(K);
    for (int i = 0; i < K; ++i)
    {
        LFPC::Ptr lfpc = std::make_shared<LFPC>();
        lfpc_pool_.push_back(lfpc);
    }
    pool_initialized_ = true;
}

// =========================================================================
// Sampling
// =========================================================================

Eigen::MatrixXd MpcController::makeNominalSequence(int N)
{
    Eigen::MatrixXd seq(N, 3);
    seq.col(0).setConstant(cfg_.nominal_al);
    seq.col(1).setConstant(cfg_.nominal_aw);
    seq.col(2).setConstant(cfg_.nominal_api);
    return seq;
}

std::vector<Eigen::MatrixXd> MpcController::sampleSequences(
    const Eigen::MatrixXd &mean, int N, int K)
{
    std::vector<Eigen::MatrixXd> samples(K);
    double sa = cfg_.sigma_al;
    double sw = cfg_.sigma_aw;
    double sp = cfg_.sigma_api;

    for (int k = 0; k < K; ++k)
    {
        samples[k] = Eigen::MatrixXd(N, 3);
        for (int n = 0; n < N; ++n)
        {
            double al, aw, api;
            if (cfg_.fix_step_params)
            {
                al  = cfg_.nominal_al;
                aw  = cfg_.nominal_aw;
                api = mean(n, 2) + sp * normal_dist_(rng_);
            }
            else
            {
                al  = mean(n, 0) + sa * normal_dist_(rng_);
                aw  = mean(n, 1) + sw * normal_dist_(rng_);
                api = mean(n, 2) + sp * normal_dist_(rng_);
            }

            // Clamp
            al  = std::max(cfg_.min_al, std::min(cfg_.max_al, al));
            aw  = std::max(0.0, std::min(cfg_.max_aw, aw));
            api = std::max(-cfg_.max_api, std::min(cfg_.max_api, api));

            samples[k](n, 0) = al;
            samples[k](n, 1) = aw;
            samples[k](n, 2) = api;
        }
    }
    return samples;
}

// =========================================================================
// Rollout (hot path)
// =========================================================================

void MpcController::rolloutBatch(
    const LFPC::Ptr &lfpc_base,
    const std::vector<Eigen::MatrixXd> &samples,
    const Eigen::Vector3d &goal_pos,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    Eigen::VectorXd &costs,
    std::vector<std::vector<Eigen::Vector3d>> &paths)
{
    int K = (int)samples.size();
    int N = cfg_.horizon_steps;
    costs.resize(K);
    costs.setConstant(std::numeric_limits<double>::infinity());

    int n_obs = (int)obs_pos.size();
    double risk_thresh = cfg_.risk_hard_threshold;
    double w_move = cfg_.w_move;
    double w_steer = cfg_.w_steer;
    double w_dapi = cfg_.w_dapi;
    double w_risk = cfg_.w_risk;
    double w_goal = cfg_.w_goal;
    double w_prox = cfg_.w_prox;
    double prox_margin = cfg_.prox_margin;
    double step_dt = lfpc_base->getTimeUpdate();
    double slice_h = collision_ ? collision_->getSliceHeight() : 0.0;

    // FOV limitation: robot's actual current position (step 0, not rollout)
    double base_x = lfpc_base->getCOMPos()(0);
    double base_y = lfpc_base->getCOMPos()(1);
    bool limit_fov = (cfg_.fov_range > 0.0);
    double fov_sq = cfg_.fov_range * cfg_.fov_range;

    // Oncoming pedestrian: scan from pedestrian's position which side has more space
    // Rationale: the pedestrian is closer to the interaction point, so obstacles
    // near them determine which passing side is actually feasible.
    bool has_oncoming = false;
    double preferred_sign = 0.0;  // +1=robot-left side open, -1=robot-right side open
    double base_theta = lfpc_base->getNextIterState()(2);
    Eigen::Vector2d oncoming_ped_pos(0, 0);
    oncoming_tight_side_ = false;
    if (n_obs > 0)
    {
        Eigen::Vector2d robot_vel = lfpc_base->getNextIterState().head(2);
        double closest_oncoming_dist = std::numeric_limits<double>::max();
        for (int oi = 0; oi < n_obs; ++oi)
        {
            double dot = obs_vel[oi](0) * robot_vel(0) + obs_vel[oi](1) * robot_vel(1);
            if (dot >= 0) continue;
            Eigen::Vector2d d = obs_pos[oi].head(2) - Eigen::Vector2d(base_x, base_y);
            double dist = d.norm();
            if (dist < cfg_.fov_range * 1.6 && dist < closest_oncoming_dist)
            {
                has_oncoming = true;
                closest_oncoming_dist = dist;
                oncoming_ped_pos = obs_pos[oi].head(2);
            }
        }

        if (has_oncoming && collision_)
        {
            // Scan from pedestrian's position, using robot's heading for left/right
            Eigen::Vector2d lateral_dir(-sin(base_theta), cos(base_theta));
            double left_sum = 0, right_sum = 0;
            for (int i = 1; i <= 5; ++i)
            {
                double off = i * 0.6;
                Eigen::Vector3d lp(oncoming_ped_pos(0) + off * lateral_dir(0),
                                   oncoming_ped_pos(1) + off * lateral_dir(1), slice_h);
                Eigen::Vector3d rp(oncoming_ped_pos(0) - off * lateral_dir(0),
                                   oncoming_ped_pos(1) - off * lateral_dir(1), slice_h);
                left_sum  += collision_->sdf_map_->getDistance(lp);
                right_sum += collision_->sdf_map_->getDistance(rp);
            }
            if (left_sum > right_sum + 0.5)       preferred_sign = +1.0;
            else if (right_sum > left_sum + 0.5)  preferred_sign = -1.0;
            // else: roughly equal, no preference

            // Tight-side detection: if robot is on the non-preferred side of the
            // pedestrian, the open path is blocked by the pedestrian themselves.
            // E.g. ped's right is open but robot is left of ped → can't reach it.
            // Suppress bias and signal stop so we wait rather than squeeze through.
            if (preferred_sign != 0.0)
            {
                double robot_lateral_from_ped = (base_x - oncoming_ped_pos(0)) * lateral_dir(0)
                                              + (base_y - oncoming_ped_pos(1)) * lateral_dir(1);
                if (preferred_sign * robot_lateral_from_ped < 0.0)
                {
                    oncoming_tight_side_ = true;
                    preferred_sign = 0.0;
                }
            }
        }
    }

    for (int k = 0; k < K; ++k)
    {
        LFPC::Ptr lfpc = lfpc_pool_[k];
        lfpc->copyState(*lfpc_base);

        double prev_com_x = lfpc->getCOMPos()(0);
        double prev_com_y = lfpc->getCOMPos()(1);
        double prev_api = samples[k](0, 2);  // for steering rate penalty
        double total_cost = 0.0;
        bool static_collided = false;
        bool dyn_collided = false;
        bool arrived_early = false;

        for (int n = 0; n < N; ++n)
        {
            double al  = samples[k](n, 0);
            double aw  = samples[k](n, 1);
            double api = samples[k](n, 2);

            lfpc->SetCtrlParams(Eigen::Vector3d(al, aw, api));
            lfpc->updateOneStep();

            std::vector<Eigen::Vector3d> com_path = lfpc->getStepCOMPath();
            Eigen::Vector3d final_com = lfpc->getCOMPos();
            double fx = final_com(0);
            double fy = final_com(1);

            for (const auto &pt : com_path)
                paths[k].push_back(pt);

            // Move + steer cost
            double dx = fx - prev_com_x;
            double dy = fy - prev_com_y;
            double move_cost = w_move * std::sqrt(dx * dx + dy * dy);
            double steer_cost = w_steer * std::abs(api);

            // Steering rate penalty: penalize large api changes between steps
            double dapi_cost = 0.0;
            if (n > 0)
                dapi_cost = w_dapi * std::abs(api - prev_api);

            // ---- Foot placement: hard check (FOV-gated) ----
            Eigen::Vector2d foot_pos = lfpc->getFootPosition();
            bool foot_in_fov = true;
            if (limit_fov)
            {
                double fdx = foot_pos(0) - base_x;
                double fdy = foot_pos(1) - base_y;
                foot_in_fov = (fdx*fdx + fdy*fdy <= fov_sq);
            }
            if (collision_ && foot_in_fov && !collision_->isTraversable(foot_pos(0), foot_pos(1)))
            {
                static_collided = true;
                break;
            }

            // ---- CoM path: FOV-gated proximity cost + dynamic check ----
            double prox_cost = 0.0;
            double risk_cost = 0.0;
            int risk_evals = 0;
            for (size_t pi = 0; pi < com_path.size(); ++pi)
            {
                double px = com_path[pi](0);
                double py = com_path[pi](1);

                // FOV gate: beyond perception range → treat as traversable
                bool in_fov = true;
                if (limit_fov)
                {
                    double rdx = px - base_x;
                    double rdy = py - base_y;
                    in_fov = (rdx*rdx + rdy*rdy <= fov_sq);
                }

                if (in_fov)
                {
                    // Static obstacle: high penalty per colliding point
                    if (collision_ && !collision_->isTraversable(px, py))
                    {
                        static_collided = true;
                        break;
                    }
                    else if (collision_)
                    {
                        // SDF proximity gradient (only for non-colliding points)
                        Eigen::Vector3d pt(px, py, slice_h);
                        double dist = collision_->sdf_map_->getDistance(pt);
                        if (dist < prox_margin)
                        {
                            double d = prox_margin - dist;
                            prox_cost += w_prox * d * d;
                        }
                    }

                    // Dynamic obstacle: hard threshold + risk accumulation
                    for (int oi = 0; oi < n_obs; ++oi)
                    {
                        double ox = obs_pos[oi](0) + n * step_dt * obs_vel[oi](0);
                        double oy = obs_pos[oi](1) + n * step_dt * obs_vel[oi](1);
                        double vx = obs_vel[oi](0);
                        double vy = obs_vel[oi](1);

                        double r = risk_field_.getIndividualCostFast(px, py, ox, oy, vx, vy);
                        if (r > risk_thresh)
                        {
                            dyn_collided = true;
                            break;
                        }
                        risk_cost += r;
                        risk_evals++;
                    }
                }
                // else: beyond FOV, skip all checks (optimistic)
                if (dyn_collided || static_collided)
                    break;
            }

            // Normalize risk: average per evaluation, then apply weight
            if (risk_evals > 0)
                risk_cost = w_risk * risk_cost / risk_evals;

            if (dyn_collided || static_collided)
            {
                total_cost = std::numeric_limits<double>::infinity();
                break;
            }

            total_cost += move_cost + steer_cost + dapi_cost + risk_cost + prox_cost;

            // Lateral bias: prefer side with more space when oncoming pedestrian
            if (has_oncoming && preferred_sign != 0.0)
            {
                double lateral = cos(base_theta) * dy - sin(base_theta) * dx;
                total_cost += -cfg_.w_bias * preferred_sign * lateral;
            }

            prev_com_x = fx;
            prev_com_y = fy;
            prev_api = api;

            // Early goal arrival: stop rollout if within threshold
            {
                double dxg = fx - goal_pos(0);
                double dyg = fy - goal_pos(1);
                if (std::sqrt(dxg*dxg + dyg*dyg) < cfg_.goal_arrival_threshold)
                {
                    arrived_early = true;
                    break;
                }
            }

            lfpc->prepareNextStep();
        }

        // Hard-inf any trajectory that touched a static or dynamic obstacle,
        // regardless of where the break happened (foot placement or CoM path).
        if (static_collided || dyn_collided)
        {
            total_cost = std::numeric_limits<double>::infinity();
        }
        else if (!arrived_early)
        {
            double gx = goal_pos(0);
            double gy = goal_pos(1);
            Eigen::Vector3d final_com = lfpc->getCOMPos();
            double fx = final_com(0);
            double fy = final_com(1);
            double dx = fx - gx;
            double dy = fy - gy;
            total_cost += (w_goal / N) * std::sqrt(dx * dx + dy * dy);
        }

        costs(k) = total_cost;
    }
}

// =========================================================================
// MPPI weight computation
// =========================================================================

Eigen::VectorXd MpcController::computeWeights(const Eigen::VectorXd &costs)
{
    int K = (int)costs.size();
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(K);

    // Find min finite cost
    double min_cost = std::numeric_limits<double>::max();
    bool any_finite = false;
    for (int i = 0; i < K; ++i)
    {
        if (std::isfinite(costs(i)))
        {
            any_finite = true;
            if (costs(i) < min_cost)
                min_cost = costs(i);
        }
    }

    if (!any_finite)
    {
        weights.setConstant(1.0 / K);
        return weights;
    }

    double w_sum = 0.0;
    for (int i = 0; i < K; ++i)
    {
        if (std::isfinite(costs(i)))
        {
            weights(i) = std::exp(-(costs(i) - min_cost) / cfg_.temperature);
            w_sum += weights(i);
        }
    }

    if (w_sum < 1e-12)
    {
        weights.setConstant(1.0 / K);
        return weights;
    }

    weights /= w_sum;
    return weights;
}

Eigen::MatrixXd MpcController::weightedUpdate(
    const std::vector<Eigen::MatrixXd> &samples,
    const Eigen::VectorXd &weights,
    int N)
{
    Eigen::MatrixXd new_mean = Eigen::MatrixXd::Zero(N, 3);
    int K = (int)samples.size();

    for (int k = 0; k < K; ++k)
    {
        new_mean += weights(k) * samples[k];
    }

    // Clamp
    for (int n = 0; n < N; ++n)
    {
        new_mean(n, 0) = std::max(cfg_.min_al, std::min(cfg_.max_al, new_mean(n, 0)));
        new_mean(n, 1) = std::max(0.0, std::min(cfg_.max_aw, new_mean(n, 1)));
        new_mean(n, 2) = std::max(-cfg_.max_api, std::min(cfg_.max_api, new_mean(n, 2)));
    }

    return new_mean;
}

void MpcController::shiftSequence(Eigen::MatrixXd &mean, int N)
{
    // Roll by -1 (shift up)
    for (int n = 0; n < N - 1; ++n)
    {
        mean.row(n) = mean.row(n + 1);
    }
    // Set last row to nominal
    mean(N - 1, 0) = cfg_.nominal_al;
    mean(N - 1, 1) = cfg_.nominal_aw;
    mean(N - 1, 2) = cfg_.nominal_api;
}

} // namespace cane_planner
