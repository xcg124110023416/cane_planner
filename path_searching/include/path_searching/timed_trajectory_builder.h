#ifndef _TIMED_TRAJECTORY_BUILDER_H_
#define _TIMED_TRAJECTORY_BUILDER_H_

#include <path_searching/timed_trajectory.h>

namespace cane_planner
{

class TimedTrajectoryBuilder
{
public:
    // `prefer_global_reference` chooses which path seeds the corridor when both
    // are usable; the other one stays the fallback either way.
    //
    // The previous MPPI prediction is dynamically feasible, which is why it was
    // preferred, but it is self-referential as a corridor centreline: a corridor
    // built around where the robot was already heading moves with the robot, so
    // lateral drift never shrinks the corridor margin and the intervention layer
    // has nothing left to trigger on. Preferring the global reference pins the
    // corridor to the route, at the cost of a centreline the robot cannot track
    // exactly at sharp corners.
    // `max_route_deviation` is the rule that decides between the two when both
    // are usable: the prediction stays the centreline only while it still
    // describes the route. Once it has wandered further than this, a corridor
    // built around it is a corridor around the wandering, and lateral drift
    // stops shrinking the margin. Non-positive disables the check.
    //
    // `prefer_global_reference` short-circuits that rule and always takes the
    // route. It pins the corridor hardest, at the cost of a centreline the
    // robot cannot track through a sharp corner.
    //
    // Either flag only chooses between usable inputs; whichever path is the
    // only usable one is used regardless.
    static TimedTrajectory buildNominal(const std::vector<Eigen::Vector3d> &previous_mppi_path,
                                        const std::vector<Eigen::Vector2d> &global_reference,
                                        double mppi_dt,
                                        double reference_speed,
                                        double max_length,
                                        bool prefer_global_reference = false,
                                        double max_route_deviation = -1.0);

    // Largest distance from `path` to `route`, measured over the arc length both
    // of them cover. Exposed for the corridor source rule above and its tests.
    static double maxDeviationFromRoute(const std::vector<Eigen::Vector3d> &path,
                                        const std::vector<Eigen::Vector2d> &route,
                                        double max_length);

private:
    static TimedTrajectory fromPreviousMppi(const std::vector<Eigen::Vector3d> &path,
                                            double dt,
                                            double max_length);
    static TimedTrajectory fromGlobalReference(const std::vector<Eigen::Vector2d> &path,
                                               double speed,
                                               double max_length);
};

} // namespace cane_planner

#endif // _TIMED_TRAJECTORY_BUILDER_H_
