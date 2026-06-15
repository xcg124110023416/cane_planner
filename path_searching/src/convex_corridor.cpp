#include <path_searching/convex_corridor.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace cane_planner
{
namespace
{

std::vector<double> accumulateArcLengths(const std::vector<Eigen::Vector2d> &path)
{
    std::vector<double> s(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i)
        s[i] = s[i - 1] + (path[i] - path[i - 1]).norm();
    return s;
}

Eigen::Vector2d interpolatePath(const std::vector<Eigen::Vector2d> &path,
                                const std::vector<double> &arc_lengths,
                                double s)
{
    if (path.empty())
        return Eigen::Vector2d::Zero();
    if (s <= 0.0 || path.size() == 1)
        return path.front();
    if (s >= arc_lengths.back())
        return path.back();

    const auto upper = std::upper_bound(arc_lengths.begin(), arc_lengths.end(), s);
    const size_t idx = static_cast<size_t>(std::distance(arc_lengths.begin(), upper));
    const double s0 = arc_lengths[idx - 1];
    const double s1 = arc_lengths[idx];
    const double ratio = (s - s0) / std::max(1e-9, s1 - s0);
    return path[idx - 1] + ratio * (path[idx] - path[idx - 1]);
}

bool rectangleTraversable(const Eigen::Vector2d &start,
                          const Eigen::Vector2d &end,
                          const Eigen::Vector2d &left,
                          double width,
                          double sample_ds,
                          double sample_dl,
                          const ConvexCorridor::TraversableFn &is_traversable)
{
    const Eigen::Vector2d span = end - start;
    const double length = span.norm();
    const int s_steps = std::max(1, static_cast<int>(std::ceil(length / std::max(1e-3, sample_ds))));
    const int l_steps = std::max(1, static_cast<int>(std::ceil((2.0 * width) / std::max(1e-3, sample_dl))));

    for (int si = 0; si <= s_steps; ++si)
    {
        const double u = static_cast<double>(si) / static_cast<double>(s_steps);
        const Eigen::Vector2d center = start + u * span;
        for (int li = 0; li <= l_steps; ++li)
        {
            const double l = -width + (2.0 * width) * static_cast<double>(li) / static_cast<double>(l_steps);
            const Eigen::Vector2d p = center + left * l;
            if (!is_traversable(p.x(), p.y()))
                return false;
        }
    }
    return true;
}

}  // namespace

bool ConvexCorridor::contains(const Segment &segment, const Eigen::Vector2d &point, double tol)
{
    for (const auto &h : segment.halfspaces)
    {
        if (h.normal.dot(point) - h.offset > tol)
            return false;
    }
    return true;
}

double ConvexCorridor::violation(const Segment &segment, const Eigen::Vector2d &point)
{
    double max_v = 0.0;
    for (const auto &h : segment.halfspaces)
        max_v = std::max(max_v, h.normal.dot(point) - h.offset);
    return std::max(0.0, max_v);
}

ConvexCorridor::Result ConvexCorridor::buildStatic(const std::vector<Eigen::Vector2d> &reference_path,
                                                   const TraversableFn &is_traversable) const
{
    Result result;
    if (reference_path.size() < 2 || !is_traversable)
        return result;

    const std::vector<double> arc_lengths = accumulateArcLengths(reference_path);
    const double total_length = arc_lengths.back();
    if (total_length < 1e-6)
        return result;

    result.feasible = true;
    result.min_width = std::numeric_limits<double>::infinity();

    const double segment_length = std::max(1e-3, cfg_.segment_length);
    for (double s0 = 0.0; s0 < total_length - 1e-6; s0 += segment_length)
    {
        const double s1 = std::min(total_length, s0 + segment_length);
        const Eigen::Vector2d start = interpolatePath(reference_path, arc_lengths, s0);
        const Eigen::Vector2d end = interpolatePath(reference_path, arc_lengths, s1);
        Eigen::Vector2d forward = end - start;
        if (forward.norm() < 1e-6)
            continue;
        forward.normalize();
        const Eigen::Vector2d left(-forward.y(), forward.x());

        double width = std::max(0.0, cfg_.half_width);
        while (width > 0.0 &&
               !rectangleTraversable(start, end, left, width, cfg_.static_sample_ds,
                                     cfg_.static_sample_dl, is_traversable))
        {
            width = std::max(0.0, width - std::max(1e-3, cfg_.static_sample_dl));
        }

        Segment segment;
        segment.s0 = s0;
        segment.s1 = s1;
        segment.t0 = s0;
        segment.t1 = s1;
        segment.center = 0.5 * (start + end);
        segment.static_feasible = width >= cfg_.min_half_width || s0 < cfg_.start_grace_length;
        segment.dynamic_feasible = true;
        segment.halfspaces = {
            {forward, forward.dot(end)},
            {-forward, -forward.dot(start)},
            {left, left.dot(segment.center) + width},
            {-left, -left.dot(segment.center) + width},
        };

        if (!segment.static_feasible)
        {
            result.feasible = false;
            ++result.static_block_count;
        }
        result.min_width = std::min(result.min_width, width);
        result.segments.push_back(segment);
    }

    if (result.segments.empty())
    {
        result.feasible = false;
        result.min_width = 0.0;
    }
    else if (!std::isfinite(result.min_width))
    {
        result.min_width = 0.0;
    }

    return result;
}

ConvexCorridor::Result ConvexCorridor::buildSpatioTemporal(
    const std::vector<Eigen::Vector2d> &reference_path,
    const TraversableFn &is_traversable,
    const std::vector<PedestrianPrediction> &pedestrians) const
{
    Result result = buildStatic(reference_path, is_traversable);
    if (result.segments.empty() || pedestrians.empty())
        return result;

    for (auto &segment : result.segments)
    {
        for (const auto &pred : pedestrians)
        {
            const double tm = 0.5 * (segment.t0 + segment.t1) + cfg_.pedestrian_time_margin;
            const Eigen::Vector2d ped = pred.p0 + pred.v * tm;
            const double combined_radius = pred.radius + cfg_.pedestrian_radius;

            if (violation(segment, ped) > combined_radius)
                continue;

            Eigen::Vector2d n = segment.center - ped;
            if (n.norm() < 1e-6)
                n = Eigen::Vector2d::UnitY();
            n.normalize();

            Halfspace h;
            h.normal = -n;
            h.offset = -n.dot(ped + n * combined_radius);
            segment.halfspaces.push_back(h);
            ++result.dynamic_block_count;

            if (!contains(segment, segment.center, 1e-6))
            {
                segment.dynamic_feasible = false;
                result.feasible = false;
            }
        }
    }

    return result;
}

}  // namespace cane_planner
