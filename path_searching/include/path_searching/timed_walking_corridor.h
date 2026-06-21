#ifndef _TIMED_WALKING_CORRIDOR_H_
#define _TIMED_WALKING_CORRIDOR_H_

#include <Eigen/Eigen>

#include <path_searching/timed_trajectory.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace cane_planner
{

struct TimedWalkingCorridorSegment
{
    double t_start = 0.0;
    double t_end = 0.0;
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d end = Eigen::Vector2d::Zero();
    Eigen::Vector2d forward = Eigen::Vector2d::UnitX();
    Eigen::Vector2d left = Eigen::Vector2d::UnitY();
    std::vector<Eigen::Vector2d> centerline;
    double half_width = 0.0;
    bool feasible = true;
    bool blocked_static = false;
    bool blocked_dynamic = false;
};

struct TimedWalkingCorridor
{
    TimedTrajectorySource source = TimedTrajectorySource::EMPTY;
    std::vector<TimedWalkingCorridorSegment> segments;

    bool valid() const { return !segments.empty(); }
    double tStart() const { return valid() ? segments.front().t_start : 0.0; }
    double tEnd() const { return valid() ? segments.back().t_end : 0.0; }
    const TimedWalkingCorridorSegment *segmentAtTime(const double t) const
    {
        for (const auto &segment : segments)
        {
            if (t >= segment.t_start && t < segment.t_end)
            {
                return &segment;
            }
        }
        return nullptr;
    }
    double outsideDistanceAtTime(const double t, const Eigen::Vector2d &point) const
    {
        const auto *segment = segmentAtTime(t);
        if (!segment)
            return 0.0;

        const Eigen::Vector2d delta = segment->end - segment->start;
        const double length = delta.norm();
        if (length < 1e-6)
            return 0.0;

        const Eigen::Vector2d forward =
            segment->forward.norm() > 1e-6 ? segment->forward.normalized() : delta / length;
        const Eigen::Vector2d left =
            segment->left.norm() > 1e-6 ? segment->left.normalized()
                                        : Eigen::Vector2d(-forward.y(), forward.x());
        const Eigen::Vector2d rel = point - segment->start;
        const double s = rel.dot(forward);
        const double l = rel.dot(left);
        const double ds = std::max({0.0, -s, s - length});
        const double dl = std::max(0.0, std::abs(l) - segment->half_width);
        if (ds <= 0.0)
            return dl;
        if (dl <= 0.0)
            return ds;
        return std::sqrt(ds * ds + dl * dl);
    }
    void clear()
    {
        source = TimedTrajectorySource::EMPTY;
        segments.clear();
    }
};

} // namespace cane_planner

#endif // _TIMED_WALKING_CORRIDOR_H_
