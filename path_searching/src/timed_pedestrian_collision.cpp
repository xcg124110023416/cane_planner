#include <path_searching/timed_pedestrian_collision.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <Eigen/Geometry>

namespace cane_planner {
namespace {
constexpr double pi = 3.14159265358979323846;
bool convex(const TimedPedestrianCollision::Polygon& p) {
    if (p.size() < 3 || p.size() > 32) return false;
    double orientation = 0.;
    for (const auto& v : p) if (!v.allFinite()) return false;
    for (size_t i = 0; i < p.size(); ++i) {
        const auto edge = (p[(i+1)%p.size()] - p[i]).eval();
        if (edge.norm() < 1e-9) return false;
        for (const auto& v : p) {
            const auto d = (v-p[i]).eval();
            const double cross = edge.x()*d.y()-edge.y()*d.x();
            if (std::abs(cross) <= 1e-10) continue;
            if (orientation*cross < 0.) return false;
            orientation = cross;
        }
    }
    return orientation != 0.;
}
}
void TimedPedestrianCollision::clear() {
    obstacles_.clear(); axes_.clear(); robot_.clear(); valid_ = true;cached_yaw_=0.;
}
bool TimedPedestrianCollision::bind(const std::vector<Obstacle>& obstacles, double radius) {
    if (!std::isfinite(radius) || radius <= 0.) {clear();valid_=false;return false;}
    Polygon robot;
    // Circumscribed octagon: covers the entire requested circular COM envelope.
    for (int i=0; i<8; ++i) {
        const double a = (i+.5)*pi/4.;
        robot.emplace_back(std::cos(a)*radius/std::cos(pi/8.),
                            std::sin(a)*radius/std::cos(pi/8.));
    }
    return bind(obstacles,robot);
}
bool TimedPedestrianCollision::bind(const std::vector<Obstacle>& obstacles,const Polygon& robot) {
    clear();valid_=false;
    if(!convex(robot))return false;
    for(const auto& obstacle:obstacles)
        if(!convex(obstacle.hull) || !obstacle.velocity.allFinite())return false;
    robot_=robot;obstacles_=obstacles;prepareAxes(0.);valid_=true;return true;
}
void TimedPedestrianCollision::prepareAxes(double yaw) const {
    axes_.clear();cached_yaw_=yaw;
    Polygon robot;
    for(const auto& p:robot_)robot.push_back(Eigen::Rotation2Dd(yaw)*p);
    for (const auto& obstacle : obstacles_) {
        std::vector<Axis> axes;
        for (const auto* polygon : std::array<const Polygon*,2>{{&obstacle.hull, &robot}}) {
            for (size_t i=0; i<polygon->size(); ++i) {
                const Eigen::Vector2d edge = (*polygon)[(i+1)%polygon->size()]-(*polygon)[i];
                const Eigen::Vector2d n = Eigen::Vector2d(-edge.y(),edge.x()).normalized();
                double pmin=std::numeric_limits<double>::infinity(), pmax=-pmin;
                double rmin=pmin, rmax=-pmin;
                for (const auto& p : obstacle.hull) { const double x=n.dot(p); pmin=std::min(pmin,x); pmax=std::max(pmax,x); }
                for (const auto& r : robot) { const double x=n.dot(r); rmin=std::min(rmin,x); rmax=std::max(rmax,x); }
                axes.push_back({n,pmin-rmax,pmax-rmin,n.dot(obstacle.velocity)});
            }
        }
        axes_.push_back(std::move(axes));
    }
}
bool TimedPedestrianCollision::intersects(const Eigen::Vector2d& com, double dt,double yaw) const {
    if (!valid_ || !com.allFinite() || !std::isfinite(dt) || dt < 0. || !std::isfinite(yaw)) return true;
    if(yaw!=cached_yaw_)prepareAxes(yaw);
    for (const auto& axes : axes_) {
        bool overlap = true;
        for (const auto& a : axes) {
            const double projection = a.normal.dot(com)-a.speed*dt;
            if (!std::isfinite(projection)) return true;
            if (projection < a.lo-1e-9 || projection > a.hi+1e-9) { overlap=false; break; }
        }
        if (overlap) return true;
    }
    return false;
}
}
