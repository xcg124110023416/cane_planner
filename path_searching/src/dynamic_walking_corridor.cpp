#include <path_searching/dynamic_walking_corridor.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace cane_planner
{

namespace
{
Eigen::Vector2d normalizedOrDefault(const Eigen::Vector2d &v)
{
    if (v.norm() < 1e-6)
        return Eigen::Vector2d::UnitX();
    return v.normalized();
}

double obstacleRadius(size_t idx,
                      const std::vector<Eigen::Vector3d> &obs_size,
                      double min_radius)
{
    double r = min_radius;
    if (idx < obs_size.size())
        r = std::max(r, 0.5 * std::max(obs_size[idx](0), obs_size[idx](1)));
    return r;
}

double polylineLength(const std::vector<Eigen::Vector2d> &path)
{
    double length = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
        length += (path[i + 1] - path[i]).norm();
    return length;
}

std::vector<Eigen::Vector2d> truncatePolylineAtLength(
    const std::vector<Eigen::Vector2d> &path,
    double target_length)
{
    std::vector<Eigen::Vector2d> out;
    if (path.empty())
        return out;

    out.push_back(path.front());
    if (path.size() == 1 || target_length <= 0.0)
        return out;

    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d a = path[i];
        const Eigen::Vector2d b = path[i + 1];
        const Eigen::Vector2d seg = b - a;
        const double seg_len = seg.norm();
        if (seg_len < 1e-6)
            continue;

        if (accum + seg_len >= target_length)
        {
            const double u = std::max(0.0, std::min(1.0, (target_length - accum) / seg_len));
            const Eigen::Vector2d p = a + u * seg;
            if ((p - out.back()).norm() > 1e-6)
                out.push_back(p);
            return out;
        }

        if ((b - out.back()).norm() > 1e-6)
            out.push_back(b);
        accum += seg_len;
    }
    return out;
}

Eigen::Vector2d polylineDirectionAtStart(const std::vector<Eigen::Vector2d> &path)
{
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        Eigen::Vector2d seg = path[i + 1] - path[i];
        if (seg.norm() > 1e-6)
            return seg.normalized();
    }
    return Eigen::Vector2d::UnitX();
}

Eigen::Vector2d interpolatePolyline(const std::vector<Eigen::Vector2d> &path,
                                    double distance)
{
    if (path.empty())
        return Eigen::Vector2d::Zero();
    if (path.size() == 1 || distance <= 0.0)
        return path.front();

    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d seg = path[i + 1] - path[i];
        const double seg_len = seg.norm();
        if (seg_len < 1e-6)
            continue;
        if (accum + seg_len >= distance)
            return path[i] + ((distance - accum) / seg_len) * seg;
        accum += seg_len;
    }
    return path.back();
}

bool hasTimedPolyline(const DynamicWalkingCorridor::Candidate &candidate)
{
    return candidate.centerline.size() >= 2 &&
           candidate.centerline_times.size() == candidate.centerline.size();
}

double polylineDistanceAtTime(const std::vector<Eigen::Vector2d> &path,
                              const std::vector<double> &times,
                              double t)
{
    if (path.size() < 2 || times.size() != path.size())
        return 0.0;
    if (t <= times.front())
        return 0.0;

    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d seg = path[i + 1] - path[i];
        const double seg_len = seg.norm();
        const double t0 = times[i];
        const double t1 = times[i + 1];
        if (t <= t1)
        {
            const double denom = std::max(1e-6, t1 - t0);
            const double u = std::max(0.0, std::min(1.0, (t - t0) / denom));
            return accum + u * seg_len;
        }
        accum += seg_len;
    }
    return accum;
}

bool projectPointToPolyline(const std::vector<Eigen::Vector2d> &path,
                            const Eigen::Vector2d &point,
                            double &along,
                            double &distance)
{
    if (path.size() < 2)
        return false;

    double best_dist = std::numeric_limits<double>::infinity();
    double best_along = 0.0;
    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d a = path[i];
        const Eigen::Vector2d b = path[i + 1];
        const Eigen::Vector2d ab = b - a;
        const double len_sq = ab.squaredNorm();
        const double seg_len = std::sqrt(len_sq);
        if (seg_len < 1e-6)
            continue;
        double u = (point - a).dot(ab) / len_sq;
        u = std::max(0.0, std::min(1.0, u));
        const Eigen::Vector2d proj = a + u * ab;
        const double d = (point - proj).norm();
        if (d < best_dist)
        {
            best_dist = d;
            best_along = accum + u * seg_len;
        }
        accum += seg_len;
    }

    if (!std::isfinite(best_dist))
        return false;
    along = best_along;
    distance = best_dist;
    return true;
}

std::vector<Eigen::Vector2d> shiftPolyline(const std::vector<Eigen::Vector2d> &path,
                                           double lateral_offset)
{
    std::vector<Eigen::Vector2d> shifted;
    shifted.reserve(path.size());
    if (path.empty())
        return shifted;

    for (size_t i = 0; i < path.size(); ++i)
    {
        Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
        if (i > 0)
            tangent += normalizedOrDefault(path[i] - path[i - 1]);
        if (i + 1 < path.size())
            tangent += normalizedOrDefault(path[i + 1] - path[i]);
        tangent = normalizedOrDefault(tangent);
        Eigen::Vector2d left(-tangent.y(), tangent.x());
        shifted.push_back(path[i] + lateral_offset * left);
    }
    return shifted;
}

Eigen::Vector2d pathLeftAt(const std::vector<Eigen::Vector2d> &path, size_t idx)
{
    Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
    if (idx > 0)
        tangent += normalizedOrDefault(path[idx] - path[idx - 1]);
    if (idx + 1 < path.size())
        tangent += normalizedOrDefault(path[idx + 1] - path[idx]);
    tangent = normalizedOrDefault(tangent);
    return Eigen::Vector2d(-tangent.y(), tangent.x());
}

double availableHalfWidthAt(
    const Eigen::Vector2d &center,
    const Eigen::Vector2d &left,
    const DynamicWalkingCorridor::Config &cfg,
    const std::function<bool(double, double)> &is_traversable)
{
    if (!is_traversable(center.x(), center.y()))
        return 0.0;

    const double dl = std::max(
        0.02,
        std::min(cfg.static_sample_dl, 0.5 * std::max(0.02, cfg.min_half_width)));
    const double max_half_width = std::max(0.0, cfg.half_width);

    double left_free = 0.0;
    for (double l = dl; l <= max_half_width + 1e-6; l += dl)
    {
        const Eigen::Vector2d p = center + left * l;
        if (!is_traversable(p.x(), p.y()))
            break;
        left_free = l;
    }

    double right_free = 0.0;
    for (double l = dl; l <= max_half_width + 1e-6; l += dl)
    {
        const Eigen::Vector2d p = center - left * l;
        if (!is_traversable(p.x(), p.y()))
            break;
        right_free = l;
    }

    return std::min(left_free, right_free);
}
} // namespace

void DynamicWalkingCorridor::setParam(ros::NodeHandle &nh)
{
    nh.param("dwc/enable", cfg_.enable, true);
    nh.param("dwc/lateral_candidates_enable", cfg_.lateral_candidates_enable, false);
    nh.param("dwc/length", cfg_.length, 4.0);
    nh.param("dwc/half_width", cfg_.half_width, 0.45);
    nh.param("dwc/min_half_width", cfg_.min_half_width, 0.25);
    nh.param("dwc/lateral_shift", cfg_.lateral_shift, 0.8);
    nh.param("dwc/prediction_horizon", cfg_.prediction_horizon, 3.0);
    nh.param("dwc/prediction_dt", cfg_.prediction_dt, 0.25);
    nh.param("dwc/robot_speed", cfg_.robot_speed, 1.0);
    nh.param("dwc/robot_radius", cfg_.robot_radius, 0.25);
    nh.param("dwc/dynamic_min_radius", cfg_.dynamic_min_radius, 0.20);
    nh.param("dwc/dynamic_safety_margin", cfg_.dynamic_safety_margin, 0.15);
    nh.param("dwc/dynamic_progress_margin", cfg_.dynamic_progress_margin, 0.50);
    nh.param("dwc/dynamic_blocking_enable", cfg_.dynamic_blocking_enable, false);
    nh.param("dwc/static_centerline_opt_enable", cfg_.static_centerline_opt_enable, true);
    nh.param("dwc/static_truncate_enable", cfg_.static_truncate_enable, true);
    nh.param("dwc/static_opt_lateral_range", cfg_.static_opt_lateral_range, 1.0);
    nh.param("dwc/static_opt_lateral_step", cfg_.static_opt_lateral_step, 0.1);
    nh.param("dwc/static_opt_smooth_weight", cfg_.static_opt_smooth_weight, 2.0);
    nh.param("dwc/static_min_feasible_length", cfg_.static_min_feasible_length, 0.8);
    nh.param("dwc/static_truncate_backoff", cfg_.static_truncate_backoff, 0.2);
    nh.param("dwc/static_start_grace_length", cfg_.static_start_grace_length, 0.4);
    nh.param("dwc/lateral_offset_weight", cfg_.lateral_offset_weight, 0.2);
    nh.param("dwc/front_pass_weight", cfg_.front_pass_weight, 4.0);
    nh.param("dwc/front_pass_sigma_s", cfg_.front_pass_sigma_s, 1.2);
    nh.param("dwc/front_pass_sigma_l", cfg_.front_pass_sigma_l, 1.0);
    nh.param("dwc/front_pass_length", cfg_.front_pass_length, 3.0);
    nh.param("dwc/front_pass_lateral", cfg_.front_pass_lateral, 1.8);
    nh.param("dwc/static_sample_ds", cfg_.static_sample_ds, 0.4);
    nh.param("dwc/static_sample_dl", cfg_.static_sample_dl, 0.3);
}

DynamicWalkingCorridor::Result DynamicWalkingCorridor::plan(
    const Eigen::Vector2d &robot_pos,
    const Eigen::Vector2d &path_forward,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    Result result;
    Eigen::Vector2d forward = normalizedOrDefault(path_forward);
    Eigen::Vector2d left(-forward.y(), forward.x());
    std::vector<double> offsets = {0.0};
    if (cfg_.lateral_candidates_enable)
    {
        offsets.push_back(cfg_.lateral_shift);
        offsets.push_back(-cfg_.lateral_shift);
    }

    result.candidates.reserve(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i)
    {
        result.candidates.push_back(evaluateCandidate(
            (int)i, offsets[i], robot_pos, forward, left,
            obs_pos, obs_vel, obs_size));
    }

    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto &candidate : result.candidates)
    {
        if (!candidate.feasible)
            continue;
        if (candidate.total_cost < best_cost)
        {
            best_cost = candidate.total_cost;
            result.selected = candidate;
            result.has_feasible = true;
        }
    }
    return result;
}

DynamicWalkingCorridor::Result DynamicWalkingCorridor::plan(
    const std::vector<Eigen::Vector2d> &reference_path,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    if (reference_path.size() < 2)
    {
        const Eigen::Vector2d robot_pos =
            reference_path.empty() ? Eigen::Vector2d::Zero() : reference_path.front();
        return plan(robot_pos, Eigen::Vector2d::UnitX(), obs_pos, obs_vel, obs_size);
    }

    Result result;
    std::vector<double> offsets = {0.0};
    if (cfg_.lateral_candidates_enable)
    {
        offsets.push_back(cfg_.lateral_shift);
        offsets.push_back(-cfg_.lateral_shift);
    }

    result.candidates.reserve(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i)
    {
        result.candidates.push_back(evaluateCandidate(
            (int)i, offsets[i], reference_path, obs_pos, obs_vel, obs_size));
    }

    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto &candidate : result.candidates)
    {
        if (!candidate.feasible)
            continue;
        if (candidate.total_cost < best_cost)
        {
            best_cost = candidate.total_cost;
            result.selected = candidate;
            result.has_feasible = true;
        }
    }
    return result;
}

DynamicWalkingCorridor::Result DynamicWalkingCorridor::plan(
    const TimedTrajectory &nominal_trajectory,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    if (!nominal_trajectory.valid())
        return plan(std::vector<Eigen::Vector2d>(), obs_pos, obs_vel, obs_size);

    Result result;
    std::vector<double> offsets = {0.0};
    if (cfg_.lateral_candidates_enable)
    {
        offsets.push_back(cfg_.lateral_shift);
        offsets.push_back(-cfg_.lateral_shift);
    }

    result.candidates.reserve(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i)
    {
        result.candidates.push_back(evaluateCandidate(
            (int)i, offsets[i], nominal_trajectory, obs_pos, obs_vel, obs_size));
    }

    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto &candidate : result.candidates)
    {
        if (!candidate.feasible)
            continue;
        if (candidate.total_cost < best_cost)
        {
            best_cost = candidate.total_cost;
            result.selected = candidate;
            result.has_feasible = true;
        }
    }
    return result;
}

DynamicWalkingCorridor::Candidate DynamicWalkingCorridor::evaluateCandidate(
    int id,
    double lateral_offset,
    const Eigen::Vector2d &robot_pos,
    const Eigen::Vector2d &forward,
    const Eigen::Vector2d &left,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    Candidate candidate;
    candidate.id = id;
    candidate.lateral_offset = lateral_offset;
    candidate.forward = forward;
    candidate.left = left;
    candidate.length = cfg_.length;
    candidate.half_width = cfg_.half_width;
    candidate.start = robot_pos + left * lateral_offset;
    candidate.end = candidate.start + forward * cfg_.length;
    if (cfg_.static_centerline_opt_enable && collision_)
    {
        std::vector<Eigen::Vector2d> reference = {candidate.start, candidate.end};
        double optimized_half_width = candidate.half_width;
        if (optimizeCenterlineForStaticMap(
                reference, cfg_,
                [this](double x, double y) { return collision_->isTraversable(x, y); },
                candidate.centerline, optimized_half_width))
        {
            candidate.half_width = std::min(candidate.half_width, optimized_half_width);
            candidate.start = candidate.centerline.front();
            candidate.end = candidate.centerline.back();
            candidate.forward = polylineDirectionAtStart(candidate.centerline);
            candidate.left = Eigen::Vector2d(-candidate.forward.y(), candidate.forward.x());
        }
    }

    candidate.blocked_static = shrinkStaticWidth(candidate);
    candidate.blocked_dynamic = isDynamicallyBlocked(
        candidate, obs_pos, obs_vel, obs_size, candidate.dynamic_block_count);
    candidate.front_pass_cost = computeFrontPassCost(candidate, obs_pos, obs_vel);
    candidate.total_cost =
        cfg_.lateral_offset_weight * std::abs(lateral_offset) +
        candidate.front_pass_cost;
    candidate.feasible =
        !candidate.blocked_static &&
        (cfg_.dynamic_blocking_enable ? !candidate.blocked_dynamic : true);
    if (!candidate.feasible)
        candidate.total_cost = std::numeric_limits<double>::infinity();
    return candidate;
}

DynamicWalkingCorridor::Candidate DynamicWalkingCorridor::evaluateCandidate(
    int id,
    double lateral_offset,
    const std::vector<Eigen::Vector2d> &reference_path,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    Candidate candidate;
    candidate.id = id;
    candidate.lateral_offset = lateral_offset;
    candidate.centerline = shiftPolyline(reference_path, lateral_offset);
    candidate.length = polylineLength(candidate.centerline);
    candidate.half_width = cfg_.half_width;
    if (cfg_.static_centerline_opt_enable && collision_)
    {
        double optimized_half_width = candidate.half_width;
        std::vector<Eigen::Vector2d> optimized;
        if (optimizeCenterlineForStaticMap(
                candidate.centerline, cfg_,
                [this](double x, double y) { return collision_->isTraversable(x, y); },
                optimized, optimized_half_width))
        {
            candidate.centerline = optimized;
            candidate.half_width = std::min(candidate.half_width, optimized_half_width);
        }
    }
    candidate.start = candidate.centerline.empty() ? Eigen::Vector2d::Zero() : candidate.centerline.front();
    candidate.end = candidate.centerline.empty() ? candidate.start : candidate.centerline.back();
    candidate.forward = polylineDirectionAtStart(candidate.centerline);
    candidate.left = Eigen::Vector2d(-candidate.forward.y(), candidate.forward.x());

    candidate.blocked_static = shrinkStaticWidth(candidate);
    candidate.blocked_dynamic = isDynamicallyBlocked(
        candidate, obs_pos, obs_vel, obs_size, candidate.dynamic_block_count);
    candidate.front_pass_cost = computeFrontPassCost(candidate, obs_pos, obs_vel);
    candidate.total_cost =
        cfg_.lateral_offset_weight * std::abs(lateral_offset) +
        candidate.front_pass_cost;
    candidate.feasible =
        !candidate.blocked_static &&
        (cfg_.dynamic_blocking_enable ? !candidate.blocked_dynamic : true);
    if (!candidate.feasible)
        candidate.total_cost = std::numeric_limits<double>::infinity();
    return candidate;
}

DynamicWalkingCorridor::Candidate DynamicWalkingCorridor::evaluateCandidate(
    int id,
    double lateral_offset,
    const TimedTrajectory &nominal_trajectory,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    std::vector<Eigen::Vector2d> reference_path;
    reference_path.reserve(nominal_trajectory.points.size());
    std::vector<double> reference_times;
    reference_times.reserve(nominal_trajectory.points.size());
    for (const auto &point : nominal_trajectory.points)
    {
        reference_path.push_back(point.position);
        reference_times.push_back(point.t_from_now);
    }

    Candidate candidate;
    candidate.id = id;
    candidate.lateral_offset = lateral_offset;
    candidate.centerline = shiftPolyline(reference_path, lateral_offset);
    candidate.centerline_times = reference_times;
    candidate.length = polylineLength(candidate.centerline);
    candidate.half_width = cfg_.half_width;
    if (cfg_.static_centerline_opt_enable && collision_)
    {
        double optimized_half_width = candidate.half_width;
        std::vector<Eigen::Vector2d> optimized;
        if (optimizeCenterlineForStaticMap(
                candidate.centerline, cfg_,
                [this](double x, double y) { return collision_->isTraversable(x, y); },
                optimized, optimized_half_width))
        {
            candidate.centerline = optimized;
            candidate.half_width = std::min(candidate.half_width, optimized_half_width);
            if (candidate.centerline_times.size() != candidate.centerline.size())
                candidate.centerline_times.clear();
        }
    }
    candidate.start = candidate.centerline.empty() ? Eigen::Vector2d::Zero() : candidate.centerline.front();
    candidate.end = candidate.centerline.empty() ? candidate.start : candidate.centerline.back();
    candidate.forward = polylineDirectionAtStart(candidate.centerline);
    candidate.left = Eigen::Vector2d(-candidate.forward.y(), candidate.forward.x());

    candidate.blocked_static = shrinkStaticWidth(candidate);
    candidate.blocked_dynamic = isDynamicallyBlocked(
        candidate, obs_pos, obs_vel, obs_size, candidate.dynamic_block_count);
    candidate.front_pass_cost = computeFrontPassCost(candidate, obs_pos, obs_vel);
    candidate.total_cost =
        cfg_.lateral_offset_weight * std::abs(lateral_offset) +
        candidate.front_pass_cost;
    candidate.feasible =
        !candidate.blocked_static &&
        (cfg_.dynamic_blocking_enable ? !candidate.blocked_dynamic : true);
    if (!candidate.feasible)
        candidate.total_cost = std::numeric_limits<double>::infinity();
    return candidate;
}

bool DynamicWalkingCorridor::shrinkStaticWidth(Candidate &candidate) const
{
    if (!collision_)
        return false;

    const double ds = std::max(0.05, cfg_.static_sample_ds);
    const double dl = std::max(
        0.02,
        std::min(cfg_.static_sample_dl, 0.5 * std::max(0.02, cfg_.min_half_width)));
    const double max_half_width = std::max(0.0, candidate.half_width);
    const double start_grace = std::max(0.0, cfg_.static_start_grace_length);
    double min_available_half_width = max_half_width;
    double prefix_min_available_half_width = max_half_width;
    candidate.static_min_width_valid = false;
    candidate.static_min_half_width = max_half_width;
    candidate.static_min_s = 0.0;
    candidate.static_min_point = candidate.start;
    candidate.truncated_static = false;

    auto updateAvailableAt = [&](const Eigen::Vector2d &center,
                                 const Eigen::Vector2d &left,
                                 double path_s) -> double {
        double available_width = 0.0;
        if (!collision_->isTraversable(center.x(), center.y()))
        {
            available_width = 0.0;
        }
        else
        {
            double left_free = 0.0;
            for (double l = dl; l <= max_half_width + 1e-6; l += dl)
            {
                Eigen::Vector2d p = center + left * l;
                if (!collision_->isTraversable(p.x(), p.y()))
                    break;
                left_free = l;
            }

            double right_free = 0.0;
            for (double l = dl; l <= max_half_width + 1e-6; l += dl)
            {
                Eigen::Vector2d p = center - left * l;
                if (!collision_->isTraversable(p.x(), p.y()))
                    break;
                right_free = l;
            }
            available_width = std::min(left_free, right_free);
        }

        if (!candidate.static_min_width_valid ||
            available_width < candidate.static_min_half_width)
        {
            candidate.static_min_width_valid = true;
            candidate.static_min_half_width = available_width;
            candidate.static_min_s = path_s;
            candidate.static_min_point = center;
        }
        return available_width;
    };

    auto maybeTruncateBeforeBlock = [&](double path_s) -> bool {
        if (!cfg_.static_truncate_enable)
            return false;
        if (path_s <= std::max(0.0, cfg_.static_min_feasible_length))
            return false;
        if (candidate.centerline.size() < 2)
            return false;

        const double cutoff = std::max(
            std::max(0.0, cfg_.static_min_feasible_length),
            path_s - std::max(0.0, cfg_.static_truncate_backoff));
        std::vector<Eigen::Vector2d> truncated =
            truncatePolylineAtLength(candidate.centerline, cutoff);
        if (truncated.size() < 2 || polylineLength(truncated) < cfg_.static_min_feasible_length)
            return false;

        candidate.centerline = truncated;
        candidate.start = candidate.centerline.front();
        candidate.end = candidate.centerline.back();
        candidate.length = polylineLength(candidate.centerline);
        candidate.forward = polylineDirectionAtStart(candidate.centerline);
        candidate.left = Eigen::Vector2d(-candidate.forward.y(), candidate.forward.x());
        candidate.half_width = std::min(candidate.half_width, prefix_min_available_half_width);
        candidate.truncated_static = true;
        return true;
    };

    if (candidate.centerline.size() >= 2)
    {
        double path_s_base = 0.0;
        for (size_t i = 0; i + 1 < candidate.centerline.size(); ++i)
        {
            const Eigen::Vector2d a = candidate.centerline[i];
            const Eigen::Vector2d b = candidate.centerline[i + 1];
            const Eigen::Vector2d seg = b - a;
            const double seg_len = seg.norm();
            if (seg_len < 1e-6)
                continue;
            const Eigen::Vector2d forward = seg / seg_len;
            const Eigen::Vector2d left(-forward.y(), forward.x());
            for (double s = 0.0; s <= seg_len + 1e-6; s += ds)
            {
                Eigen::Vector2d center = a + forward * s;
                const double path_s = path_s_base + s;
                const double available_width = updateAvailableAt(center, left, path_s);
                if (path_s <= start_grace)
                    continue;
                min_available_half_width = std::min(min_available_half_width, available_width);
                if (available_width < cfg_.min_half_width)
                {
                    if (maybeTruncateBeforeBlock(path_s))
                        return false;
                    candidate.half_width = std::min(candidate.half_width, min_available_half_width);
                    return true;
                }
                prefix_min_available_half_width =
                    std::min(prefix_min_available_half_width, available_width);
            }
            path_s_base += seg_len;
        }
        candidate.half_width = std::min(candidate.half_width, min_available_half_width);
        return candidate.half_width < cfg_.min_half_width;
    }

    for (double s = 0.0; s <= cfg_.length + 1e-6; s += ds)
    {
        Eigen::Vector2d center = candidate.start + candidate.forward * s;
        const double available_width = updateAvailableAt(center, candidate.left, s);
        if (s <= start_grace)
            continue;
        min_available_half_width = std::min(min_available_half_width, available_width);
        if (available_width < cfg_.min_half_width)
        {
            candidate.half_width = std::min(candidate.half_width, min_available_half_width);
            return true;
        }
        prefix_min_available_half_width =
            std::min(prefix_min_available_half_width, available_width);
    }
    candidate.half_width = std::min(candidate.half_width, min_available_half_width);
    return candidate.half_width < cfg_.min_half_width;
}

bool DynamicWalkingCorridor::optimizeCenterlineForStaticMap(
    const std::vector<Eigen::Vector2d> &reference_path,
    const Config &cfg,
    const std::function<bool(double, double)> &is_traversable,
    std::vector<Eigen::Vector2d> &optimized_path,
    double &optimized_half_width)
{
    optimized_path.clear();
    optimized_half_width = 0.0;
    if (reference_path.size() < 2 || !is_traversable)
        return false;

    const double range = std::max(0.0, cfg.static_opt_lateral_range);
    const double step = std::max(0.02, cfg.static_opt_lateral_step);
    std::vector<double> offsets;
    for (double off = -range; off <= range + 1e-6; off += step)
        offsets.push_back(off);
    if (offsets.empty())
        offsets.push_back(0.0);

    const size_t n = reference_path.size();
    const size_t m = offsets.size();
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> dp(n, std::vector<double>(m, inf));
    std::vector<std::vector<int>> parent(n, std::vector<int>(m, -1));
    std::vector<std::vector<double>> widths(n, std::vector<double>(m, 0.0));

    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector2d left = pathLeftAt(reference_path, i);
        for (size_t j = 0; j < m; ++j)
        {
            const Eigen::Vector2d p = reference_path[i] + offsets[j] * left;
            const double width = availableHalfWidthAt(p, left, cfg, is_traversable);
            widths[i][j] = width;
            if (width + 1e-6 < cfg.min_half_width)
                continue;

            const double base_cost =
                cfg.lateral_offset_weight * std::abs(offsets[j]) -
                0.05 * std::min(width, cfg.half_width);
            if (i == 0)
            {
                dp[i][j] = base_cost;
                continue;
            }

            for (size_t k = 0; k < m; ++k)
            {
                if (!std::isfinite(dp[i - 1][k]))
                    continue;
                const double d = offsets[j] - offsets[k];
                const double cost =
                    dp[i - 1][k] + base_cost + cfg.static_opt_smooth_weight * d * d;
                if (cost < dp[i][j])
                {
                    dp[i][j] = cost;
                    parent[i][j] = static_cast<int>(k);
                }
            }
        }
    }

    int best = -1;
    double best_cost = inf;
    for (size_t j = 0; j < m; ++j)
    {
        if (dp[n - 1][j] < best_cost)
        {
            best_cost = dp[n - 1][j];
            best = static_cast<int>(j);
        }
    }
    if (best < 0 || !std::isfinite(best_cost))
        return false;

    std::vector<int> choice(n, 0);
    choice[n - 1] = best;
    for (size_t i = n - 1; i > 0; --i)
    {
        const int p = parent[i][choice[i]];
        if (p < 0)
            return false;
        choice[i - 1] = p;
    }

    optimized_path.reserve(n);
    optimized_half_width = cfg.half_width;
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector2d left = pathLeftAt(reference_path, i);
        const int j = choice[i];
        optimized_path.push_back(reference_path[i] + offsets[j] * left);
        optimized_half_width = std::min(optimized_half_width, widths[i][j]);
    }
    return optimized_half_width + 1e-6 >= cfg.min_half_width;
}

bool DynamicWalkingCorridor::isDynamicallyBlocked(
    const Candidate &candidate,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size,
    int &block_count) const
{
    block_count = 0;
    if (obs_pos.empty())
        return false;

    const double dt = std::max(0.05, cfg_.prediction_dt);
    const double robot_speed = std::max(0.0, cfg_.robot_speed);
    for (double t = 0.0; t <= cfg_.prediction_horizon + 1e-6; t += dt)
    {
        const double robot_s = hasTimedPolyline(candidate)
                                   ? polylineDistanceAtTime(candidate.centerline,
                                                            candidate.centerline_times,
                                                            t)
                                   : std::min(candidate.length, robot_speed * t);
        for (size_t i = 0; i < obs_pos.size(); ++i)
        {
            Eigen::Vector2d ped(obs_pos[i](0), obs_pos[i](1));
            if (i < obs_vel.size())
                ped += Eigen::Vector2d(obs_vel[i](0), obs_vel[i](1)) * t;

            Eigen::Vector2d rel = ped - candidate.start;
            const double s = rel.dot(candidate.forward);
            const double l = rel.dot(candidate.left);
            const double r = obstacleRadius(i, obs_size, cfg_.dynamic_min_radius) +
                             cfg_.robot_radius + cfg_.dynamic_safety_margin;
            const double progress_window = r + std::max(0.0, cfg_.dynamic_progress_margin);

            if (candidate.centerline.size() >= 2)
            {
                double ped_s = 0.0;
                double lateral_dist = std::numeric_limits<double>::infinity();
                if (projectPointToPolyline(candidate.centerline, ped, ped_s, lateral_dist) &&
                    lateral_dist <= candidate.half_width + r &&
                    std::abs(ped_s - robot_s) <= progress_window)
                    block_count++;
            }
            else if (s >= -r && s <= cfg_.length + r &&
                     std::abs(l) <= candidate.half_width + r &&
                     std::abs(s - robot_s) <= progress_window)
            {
                block_count++;
            }
        }
    }
    return block_count > 0;
}

double DynamicWalkingCorridor::computeFrontPassCost(
    const Candidate &candidate,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel) const
{
    if (cfg_.front_pass_weight <= 0.0 || obs_pos.empty())
        return 0.0;

    double cost = 0.0;
    const double dt = std::max(0.05, cfg_.prediction_dt);
    const double sigma_s_sq = std::max(1e-4, cfg_.front_pass_sigma_s * cfg_.front_pass_sigma_s);
    const double sigma_l_sq = std::max(1e-4, cfg_.front_pass_sigma_l * cfg_.front_pass_sigma_l);

    for (double t = 0.0; t <= cfg_.prediction_horizon + 1e-6; t += dt)
    {
        const double travel = hasTimedPolyline(candidate)
                                  ? polylineDistanceAtTime(candidate.centerline,
                                                           candidate.centerline_times,
                                                           t)
                                  : std::min(cfg_.length, std::max(0.0, cfg_.robot_speed) * t);
        const Eigen::Vector2d robot =
            candidate.centerline.size() >= 2
                ? interpolatePolyline(candidate.centerline, travel)
                : candidate.start + candidate.forward * travel;
        for (size_t i = 0; i < obs_pos.size(); ++i)
        {
            if (i >= obs_vel.size())
                continue;
            Eigen::Vector2d v(obs_vel[i](0), obs_vel[i](1));
            const double speed = v.norm();
            if (speed < 1e-3)
                continue;

            const Eigen::Vector2d e = v / speed;
            const Eigen::Vector2d e_perp(-e.y(), e.x());
            Eigen::Vector2d ped(obs_pos[i](0), obs_pos[i](1));
            ped += v * t;

            const Eigen::Vector2d rel = robot - ped;
            const double s = rel.dot(e);
            const double l = rel.dot(e_perp);
            if (s <= 0.0 || s > cfg_.front_pass_length ||
                std::abs(l) > cfg_.front_pass_lateral)
                continue;

            const double gs = std::exp(-0.5 * s * s / sigma_s_sq);
            const double gl = std::exp(-0.5 * l * l / sigma_l_sq);
            cost += cfg_.front_pass_weight * gs * gl * dt;
        }
    }
    return cost;
}

double DynamicWalkingCorridor::outsideDistance(const Candidate &candidate,
                                               const Eigen::Vector2d &point)
{
    if (candidate.centerline.size() >= 2)
    {
        double min_centerline_dist = std::numeric_limits<double>::infinity();
        double beyond_end_lateral_dist = std::numeric_limits<double>::infinity();
        bool beyond_truncated_end = false;
        for (size_t i = 0; i + 1 < candidate.centerline.size(); ++i)
        {
            const Eigen::Vector2d a = candidate.centerline[i];
            const Eigen::Vector2d b = candidate.centerline[i + 1];
            const Eigen::Vector2d ab = b - a;
            const double len_sq = ab.squaredNorm();
            if (len_sq < 1e-9)
                continue;
            double t = (point - a).dot(ab) / len_sq;
            t = std::max(0.0, std::min(1.0, t));
            const Eigen::Vector2d proj = a + t * ab;
            min_centerline_dist = std::min(min_centerline_dist, (point - proj).norm());

            if (candidate.truncated_static && i + 2 == candidate.centerline.size())
            {
                const double seg_len = ab.norm();
                if (seg_len > 1e-6)
                {
                    const Eigen::Vector2d forward = ab / seg_len;
                    const Eigen::Vector2d left(-forward.y(), forward.x());
                    const Eigen::Vector2d rel_end = point - b;
                    if (rel_end.dot(forward) > 0.0)
                    {
                        beyond_truncated_end = true;
                        beyond_end_lateral_dist = std::abs(rel_end.dot(left));
                    }
                }
            }
        }
        if (beyond_truncated_end && std::isfinite(beyond_end_lateral_dist))
            return std::max(0.0, beyond_end_lateral_dist - std::max(0.0, candidate.half_width));
        if (std::isfinite(min_centerline_dist))
            return std::max(0.0, min_centerline_dist - std::max(0.0, candidate.half_width));
    }

    const Eigen::Vector2d forward = normalizedOrDefault(candidate.forward);
    Eigen::Vector2d left = candidate.left;
    if (left.norm() < 1e-6)
        left = Eigen::Vector2d(-forward.y(), forward.x());
    else
        left.normalize();

    const double length = std::max(0.0, candidate.length);
    const double half_width = std::max(0.0, candidate.half_width);
    const Eigen::Vector2d rel = point - candidate.start;
    const double s = rel.dot(forward);
    const double l = rel.dot(left);

    const double outside_s = std::max(std::max(-s, s - length), 0.0);
    const double outside_l = std::max(std::abs(l) - half_width, 0.0);
    return std::sqrt(outside_s * outside_s + outside_l * outside_l);
}

} // namespace cane_planner
