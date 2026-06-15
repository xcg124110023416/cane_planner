#ifndef PATH_SEARCHING_CONVEX_CORRIDOR_H
#define PATH_SEARCHING_CONVEX_CORRIDOR_H

#include <Eigen/Eigen>
#include <functional>
#include <vector>

namespace cane_planner
{

class ConvexCorridor
{
public:
    using TraversableFn = std::function<bool(double, double)>;

    struct Halfspace
    {
        Eigen::Vector2d normal = Eigen::Vector2d::Zero();
        double offset = 0.0;  // normal.dot(p) <= offset
    };

    struct Segment
    {
        double s0 = 0.0;
        double s1 = 0.0;
        double t0 = 0.0;
        double t1 = 0.0;
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        std::vector<Halfspace> halfspaces;
        bool static_feasible = true;
        bool dynamic_feasible = true;
    };

    struct Config
    {
        bool enable = false;
        double segment_length = 0.8;
        double half_width = 0.45;
        double min_half_width = 0.25;
        double static_sample_ds = 0.2;
        double static_sample_dl = 0.1;
        double pedestrian_radius = 0.35;
        double pedestrian_time_margin = 0.4;
        double start_grace_length = 0.4;
    };

    struct PedestrianPrediction
    {
        Eigen::Vector2d p0 = Eigen::Vector2d::Zero();
        Eigen::Vector2d v = Eigen::Vector2d::Zero();
        double radius = 0.35;
    };

    struct Result
    {
        std::vector<Segment> segments;
        bool feasible = false;
        double min_width = 0.0;
        int static_block_count = 0;
        int dynamic_block_count = 0;
    };

    static bool contains(const Segment &segment, const Eigen::Vector2d &point, double tol = 1e-6);
    static double violation(const Segment &segment, const Eigen::Vector2d &point);

    Result buildStatic(const std::vector<Eigen::Vector2d> &reference_path,
                       const TraversableFn &is_traversable) const;

    Result buildSpatioTemporal(const std::vector<Eigen::Vector2d> &reference_path,
                               const TraversableFn &is_traversable,
                               const std::vector<PedestrianPrediction> &pedestrians) const;

    void setConfig(const Config &cfg) { cfg_ = cfg; }
    const Config &getConfig() const { return cfg_; }

private:
    Config cfg_;
};

}  // namespace cane_planner

#endif
