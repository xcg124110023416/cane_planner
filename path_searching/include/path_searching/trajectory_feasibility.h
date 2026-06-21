#ifndef _TRAJECTORY_FEASIBILITY_H_
#define _TRAJECTORY_FEASIBILITY_H_

namespace cane_planner
{

struct TrajectoryFeasibility
{
    int valid_trajectory_count = 0;
    int corridor_feasible_trajectory_count = 0;
    double inside_corridor_ratio = 0.0;
    bool corridor_evaluated = false;

    bool hasFeasibleTrajectory() const
    {
        return corridor_evaluated
                   ? corridor_feasible_trajectory_count > 0
                   : valid_trajectory_count > 0;
    }
    bool shouldStop() const { return !hasFeasibleTrajectory(); }
};

} // namespace cane_planner

#endif // _TRAJECTORY_FEASIBILITY_H_
