#include <path_searching/path_smoother.h>

#include <algorithm>
#include <cmath>

namespace cane_planner
{
namespace
{

double clamp(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

bool appendIfDistinct(std::vector<Eigen::Vector2d>& out,
                      const Eigen::Vector2d& point,
                      double eps = 1e-6)
{
    if (!out.empty() && (out.back() - point).norm() <= eps)
        return false;
    out.push_back(point);
    return true;
}

bool samplesTraversable(const std::vector<Eigen::Vector2d>& samples,
                        const PathSmoother::TraversableFn& is_traversable)
{
    if (!is_traversable)
        return true;
    for (const auto& p : samples)
    {
        if (!is_traversable(p.x(), p.y()))
            return false;
    }
    return true;
}

}  // namespace

std::vector<Eigen::Vector2d> PathSmoother::smoothCorners(
    const std::vector<Eigen::Vector2d>& path,
    const Config& cfg,
    const TraversableFn& is_traversable)
{
    if (!cfg.enable || path.size() < 3)
        return path;

    std::vector<Eigen::Vector2d> out;
    out.reserve(path.size() + path.size() * std::max(0, cfg.samples_per_corner));
    out.push_back(path.front());

    const int corner_samples = std::max(1, cfg.samples_per_corner);
    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d prev = path[i - 1];
        const Eigen::Vector2d corner = path[i];
        const Eigen::Vector2d next = path[i + 1];
        const Eigen::Vector2d vin = corner - prev;
        const Eigen::Vector2d vout = next - corner;
        const double lin = vin.norm();
        const double lout = vout.norm();
        if (lin < 1e-6 || lout < 1e-6)
        {
            appendIfDistinct(out, corner);
            continue;
        }

        const Eigen::Vector2d din = vin / lin;
        const Eigen::Vector2d dout = vout / lout;
        const double turn = std::acos(clamp(din.dot(dout), -1.0, 1.0));
        const double trim = std::min(cfg.corner_radius,
                                     cfg.max_trim_ratio * std::min(lin, lout));
        if (turn < cfg.min_turn_angle || trim < 1e-6)
        {
            appendIfDistinct(out, corner);
            continue;
        }

        const Eigen::Vector2d p0 = corner - din * trim;
        const Eigen::Vector2d p2 = corner + dout * trim;
        std::vector<Eigen::Vector2d> curve;
        curve.reserve(corner_samples + 1);
        curve.push_back(p0);
        for (int s = 1; s <= corner_samples; ++s)
        {
            const double t = static_cast<double>(s) / static_cast<double>(corner_samples);
            const double omt = 1.0 - t;
            curve.push_back(omt * omt * p0 + 2.0 * omt * t * corner + t * t * p2);
        }

        if (!samplesTraversable(curve, is_traversable))
        {
            appendIfDistinct(out, corner);
            continue;
        }

        for (const auto& p : curve)
            appendIfDistinct(out, p);
    }

    appendIfDistinct(out, path.back());
    return out;
}

}  // namespace cane_planner
