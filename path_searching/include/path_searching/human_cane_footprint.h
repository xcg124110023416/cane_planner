#ifndef CANE_HUMAN_CANE_FOOTPRINT_H
#define CANE_HUMAN_CANE_FOOTPRINT_H
#include <Eigen/Core>
#include <vector>
namespace cane_planner {
// Rigid torso/cane envelope. Both disks are circumscribed, including between samples.
class HumanCaneFootprint {
public:
    using Polygon = std::vector<Eigen::Vector2d>;
    struct Config {
        bool enabled = false;
        double body_radius = .25, cane_length = .65, front_radius = .12, margin = .03;
    };
    static bool validConfig(const Config&);
    static Polygon relative(const Config&, double yaw);
    static Polygon world(const Config&, const Eigen::Vector2d& position, double yaw);
    static double support(const Polygon&, const Eigen::Vector2d& normal);
};
}
#endif
