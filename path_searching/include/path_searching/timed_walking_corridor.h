#ifndef _TIMED_WALKING_CORRIDOR_H_
#define _TIMED_WALKING_CORRIDOR_H_

#include <Eigen/Eigen>

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
    double half_width = 0.0;
    bool feasible = true;
    bool blocked_static = false;
    bool blocked_dynamic = false;
};

struct TimedWalkingCorridor
{
    std::vector<TimedWalkingCorridorSegment> segments;

    bool valid() const { return !segments.empty(); }
    void clear() { segments.clear(); }
};

} // namespace cane_planner

#endif // _TIMED_WALKING_CORRIDOR_H_
