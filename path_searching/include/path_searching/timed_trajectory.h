#ifndef _TIMED_TRAJECTORY_H_
#define _TIMED_TRAJECTORY_H_

#include <Eigen/Eigen>

#include <vector>

namespace cane_planner
{

enum class TimedTrajectorySource
{
    EMPTY = 0,
    ASTAR_BOOTSTRAP = 1,
    PREVIOUS_MPPI = 2
};

struct TimedTrajectoryPoint
{
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    double yaw = 0.0;
    double t_from_now = 0.0;
};

struct TimedTrajectory
{
    TimedTrajectorySource source = TimedTrajectorySource::EMPTY;
    std::vector<TimedTrajectoryPoint> points;

    bool valid() const { return points.size() >= 2; }
    void clear()
    {
        source = TimedTrajectorySource::EMPTY;
        points.clear();
    }
};

} // namespace cane_planner

#endif // _TIMED_TRAJECTORY_H_
