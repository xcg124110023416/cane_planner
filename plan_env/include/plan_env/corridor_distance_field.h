#ifndef CANE_PLANNER_CORRIDOR_DISTANCE_FIELD_H
#define CANE_PLANNER_CORRIDOR_DISTANCE_FIELD_H

#include <Eigen/Core>
#include <cstdint>
#include <vector>

namespace cane_planner {

// Unsigned Euclidean comfort distance to whole blocked cells and the domain
// exterior. Bilinear interpolation is approximate; never a collision certificate.
class CorridorDistanceField {
public:
    bool build(const Eigen::Vector2d& origin, double resolution,
               const Eigen::Vector2i& size, const std::vector<uint8_t>& blocked);
    double distance(const Eigen::Vector2d& position) const;

private:
    Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
    Eigen::Vector2i size_ = Eigen::Vector2i::Zero();
    double spacing_ = 0.;
    std::vector<double> distances_;
};

} // namespace cane_planner
#endif
