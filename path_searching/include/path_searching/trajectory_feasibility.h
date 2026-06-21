#ifndef _TRAJECTORY_FEASIBILITY_H_
#define _TRAJECTORY_FEASIBILITY_H_

#include <cmath>
#include <limits>
#include <vector>

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

inline int selectBestTrajectoryIndex(const std::vector<double> &costs,
                                     const std::vector<bool> &corridor_feasible,
                                     const bool corridor_evaluated)
{
    auto choose_best = [&](const bool require_corridor_feasible) {
        int best = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < costs.size(); ++i)
        {
            if (!std::isfinite(costs[i]))
                continue;
            if (require_corridor_feasible &&
                (i >= corridor_feasible.size() || !corridor_feasible[i]))
                continue;
            if (costs[i] < best_cost)
            {
                best_cost = costs[i];
                best = static_cast<int>(i);
            }
        }
        return best;
    };

    if (corridor_evaluated)
    {
        const int corridor_best = choose_best(true);
        if (corridor_best >= 0)
            return corridor_best;
    }
    return choose_best(false);
}

} // namespace cane_planner

#endif // _TRAJECTORY_FEASIBILITY_H_
