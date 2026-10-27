#ifndef CANE_TIMED_PEDESTRIAN_COLLISION_H
#define CANE_TIMED_PEDESTRIAN_COLLISION_H

#include <Eigen/Core>
#include <vector>

namespace cane_planner {

// Immutable observation geometry, translated by a constant world-frame velocity.
class TimedPedestrianCollision {
public:
    using Polygon = std::vector<Eigen::Vector2d>;
    struct Obstacle { Polygon hull; Eigen::Vector2d velocity = Eigen::Vector2d::Zero(); };

    bool bind(const std::vector<Obstacle>& obstacles, double robot_radius);
    bool bind(const std::vector<Obstacle>& obstacles, const Polygon& relative_robot);
    void clear();
    // Seconds since the source observation, INCLUDING its age at execution.
    // Touching counts as collision; invalid input fails closed.
    bool intersects(const Eigen::Vector2d& com, double observation_dt, double yaw = 0.) const;
    bool valid() const { return valid_; }
    bool empty() const { return obstacles_.empty(); }
    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    const Polygon& robotFootprint() const { return robot_; }

private:
    struct Axis { Eigen::Vector2d normal; double lo, hi, speed; };
    std::vector<Obstacle> obstacles_;
    mutable std::vector<std::vector<Axis>> axes_;
    mutable double cached_yaw_ = 0.;
    void prepareAxes(double yaw) const;
    Polygon robot_;
    bool valid_ = true;
};
}
#endif
