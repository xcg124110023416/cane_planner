#include <path_searching/timed_trajectory_builder.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace cane_planner
{
namespace
{
constexpr double kEps = 1e-6;

double yawOf(const Eigen::Vector2d &delta, double fallback)
{
    if (delta.norm() < kEps)
        return fallback;
    return std::atan2(delta.y(), delta.x());
}

double distanceToPolyline(const Eigen::Vector2d &p, const std::vector<Eigen::Vector2d> &route)
{
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < route.size(); ++i)
    {
        const Eigen::Vector2d a = route[i];
        const Eigen::Vector2d ab = route[i + 1] - a;
        const double len_sq = ab.squaredNorm();
        if (len_sq < kEps * kEps)
            continue;
        double u = (p - a).dot(ab) / len_sq;
        u = std::max(0.0, std::min(1.0, u));
        best = std::min(best, (p - (a + u * ab)).norm());
    }
    return best;
}

double polylineLength(const std::vector<Eigen::Vector2d> &route)
{
    double total = 0.0;
    for (std::size_t i = 0; i + 1 < route.size(); ++i)
        total += (route[i + 1] - route[i]).norm();
    return total;
}

} // namespace

double TimedTrajectoryBuilder::maxDeviationFromRoute(
    const std::vector<Eigen::Vector3d> &path,
    const std::vector<Eigen::Vector2d> &route,
    double max_length)
{
    if (path.size() < 2 || route.size() < 2)
        return 0.0;

    // Stop at whichever runs out first. Beyond the route's own end every point
    // projects onto its last vertex, so the "distance" there is mostly along
    // track and would reject a perfectly on-route prediction.
    double budget = polylineLength(route);
    if (max_length > kEps)
        budget = std::min(budget, max_length);

    double worst = distanceToPolyline(path.front().head(2), route);
    double accum = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        accum += (path[i].head(2) - path[i - 1].head(2)).norm();
        if (accum > budget)
            break;
        worst = std::max(worst, distanceToPolyline(path[i].head(2), route));
    }
    return worst;
}

TimedTrajectory TimedTrajectoryBuilder::buildNominal(
    const std::vector<Eigen::Vector3d> &previous_mppi_path,
    const std::vector<Eigen::Vector2d> &global_reference,
    double mppi_dt,
    double reference_speed,
    double max_length,
    bool prefer_global_reference,
    double max_route_deviation)
{
    TimedTrajectory from_route = fromGlobalReference(global_reference, reference_speed, max_length);
    TimedTrajectory from_prediction = fromPreviousMppi(previous_mppi_path, mppi_dt, max_length);

    // Only one usable input: no rule to apply.
    if (!from_prediction.valid())
        return from_route;
    if (!from_route.valid())
        return from_prediction;

    if (prefer_global_reference)
        return from_route;

    if (max_route_deviation > kEps &&
        maxDeviationFromRoute(previous_mppi_path, global_reference, max_length) >
            max_route_deviation)
        return from_route;

    return from_prediction;
}

TimedTrajectory TimedTrajectoryBuilder::fromPreviousMppi(
    const std::vector<Eigen::Vector3d> &path,
    double dt,
    double max_length)
{
    TimedTrajectory timed;
    if (path.size() < 2 || dt <= kEps)
        return timed;

    timed.source = TimedTrajectorySource::PREVIOUS_MPPI;
    double accum = 0.0;
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (i > 0)
        {
            accum += (path[i].head(2) - path[i - 1].head(2)).norm();
            if (max_length > kEps && accum > max_length)
                break;
        }

        TimedTrajectoryPoint point;
        point.position = path[i].head(2);
        point.yaw = path[i](2);
        point.t_from_now = static_cast<double>(i) * dt;
        timed.points.push_back(point);
    }

    if (!timed.valid())
        timed.clear();
    return timed;
}

TimedTrajectory TimedTrajectoryBuilder::fromGlobalReference(
    const std::vector<Eigen::Vector2d> &path,
    double speed,
    double max_length)
{
    TimedTrajectory timed;
    if (path.size() < 2 || speed <= kEps)
        return timed;

    timed.source = TimedTrajectorySource::ASTAR_BOOTSTRAP;
    double accum = 0.0;
    double fallback_yaw = yawOf(path[1] - path[0], 0.0);

    TimedTrajectoryPoint first;
    first.position = path[0];
    first.yaw = fallback_yaw;
    first.t_from_now = 0.0;
    timed.points.push_back(first);

    Eigen::Vector2d prev = path[0];
    for (size_t i = 1; i < path.size(); ++i)
    {
        const Eigen::Vector2d seg = path[i] - prev;
        const double seg_len = seg.norm();
        if (seg_len < kEps)
            continue;

        Eigen::Vector2d next = path[i];
        double next_accum = accum + seg_len;
        if (max_length > kEps && next_accum > max_length)
        {
            const double remain = std::max(0.0, max_length - accum);
            next = prev + (remain / seg_len) * seg;
            next_accum = max_length;
        }

        TimedTrajectoryPoint point;
        point.position = next;
        point.yaw = yawOf(next - prev, fallback_yaw);
        point.t_from_now = next_accum / speed;
        timed.points.push_back(point);

        fallback_yaw = point.yaw;
        accum = next_accum;
        prev = next;
        if (max_length > kEps && accum >= max_length - kEps)
            break;
    }

    if (!timed.valid())
        timed.clear();
    return timed;
}

} // namespace cane_planner
