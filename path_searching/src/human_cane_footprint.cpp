#include <path_searching/human_cane_footprint.h>
#include <path_searching/pedestrian_polygon.h>
#include <cmath>
#include <limits>
namespace cane_planner {
bool HumanCaneFootprint::validConfig(const Config& c) {
    for(double x:{c.body_radius,c.cane_length,c.front_radius,c.margin})
        if(!std::isfinite(x) || x<0 || x>2.)return false;
    return c.body_radius>=.01 && c.front_radius>=.01;
}
HumanCaneFootprint::Polygon HumanCaneFootprint::relative(const Config& c,double yaw) {
    if(!validConfig(c) || !std::isfinite(yaw))return {};
    if(!c.enabled)return {Eigen::Vector2d::Zero()};
    constexpr double pi=3.14159265358979323846;
    Polygon points;
    for(int disk=0;disk<2;++disk) {
        const double radius=((disk?c.front_radius:c.body_radius)+c.margin)/std::cos(pi/8.);
        const Eigen::Vector2d center=disk?Eigen::Vector2d(c.cane_length*std::cos(yaw),c.cane_length*std::sin(yaw)):Eigen::Vector2d::Zero();
        for(int i=0;i<8;++i) {
            const double angle=yaw+(i+.5)*pi/4.;
            points.push_back(center+radius*Eigen::Vector2d(std::cos(angle),std::sin(angle)));
        }
    }
    return PedestrianPolygon::convexHull(std::move(points));
}
HumanCaneFootprint::Polygon HumanCaneFootprint::world(const Config& c,const Eigen::Vector2d& p,double yaw) {
    if(!p.allFinite())return {};
    auto polygon=relative(c,yaw);for(auto& v:polygon)v+=p;return polygon;
}
double HumanCaneFootprint::support(const Polygon& polygon,const Eigen::Vector2d& normal) {
    double value=-std::numeric_limits<double>::infinity();
    for(const auto& p:polygon)value=std::max(value,normal.dot(p));
    return value;
}
}
