#ifndef PATH_SEARCHING_PATH_SMOOTHER_H
#define PATH_SEARCHING_PATH_SMOOTHER_H

#include <Eigen/Eigen>

#include <functional>
#include <vector>

namespace cane_planner
{

class PathSmoother
{
public:
    using TraversableFn = std::function<bool(double, double)>;

    struct Config
    {
        bool enable = true;
        double corner_radius = 0.45;
        double min_turn_angle = 0.55;
        double max_trim_ratio = 0.45;
        int samples_per_corner = 4;
    };

    static std::vector<Eigen::Vector2d> smoothCorners(
        const std::vector<Eigen::Vector2d>& path,
        const Config& cfg,
        const TraversableFn& is_traversable = TraversableFn());
};

}  // namespace cane_planner

#endif
