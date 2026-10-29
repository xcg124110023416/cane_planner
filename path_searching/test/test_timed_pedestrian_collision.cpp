#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <path_searching/timed_pedestrian_collision.h>
#include <path_searching/human_cane_footprint.h>

namespace cane_planner {
using Checker = TimedPedestrianCollision;
static Checker::Polygon square(double x, double y, double r=.1) {
    return {{x-r,y-r},{x+r,y-r},{x+r,y+r},{x-r,y+r}};
}
TEST(TimedPedestrianCollision, FutureCrossingAndObservationAge) {
    Checker c;
    ASSERT_TRUE(c.bind({{square(0,2),Eigen::Vector2d(0,-1)}},.25));
    EXPECT_FALSE(c.intersects(Eigen::Vector2d::Zero(),0.));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),2.));
    EXPECT_FALSE(c.intersects(Eigen::Vector2d::Zero(),3.));
    // A 0.2-second-old observation queried 1.8 seconds into the future.
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),.2+1.8));
}
TEST(TimedPedestrianCollision, ContactContainmentAndSeparation) {
    Checker c;
    ASSERT_TRUE(c.bind({{square(0,0),Eigen::Vector2d::Zero()}},.25));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),0.));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d(.35,0),0.)); // exactly touching
    EXPECT_FALSE(c.intersects(Eigen::Vector2d(.35001,0),0.));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d(.34,0),100.)); // stationary
}
TEST(TimedPedestrianCollision, RotatedSeparationAndClockwiseHull) {
    Checker c;
    Checker::Polygon diamond={{0,1},{1,0},{0,-1},{-1,0}};
    ASSERT_TRUE(c.bind({{diamond,Eigen::Vector2d::Zero()}},.05));
    EXPECT_FALSE(c.intersects(Eigen::Vector2d(.8,.8),0.)); // AABBs overlap
    EXPECT_TRUE(c.intersects(Eigen::Vector2d(.4,.4),0.));
    std::reverse(diamond.begin(),diamond.end());
    ASSERT_TRUE(c.bind({{diamond,Eigen::Vector2d::Zero()}},.05));
    EXPECT_FALSE(c.intersects(Eigen::Vector2d(.8,.8),0.));
}
TEST(TimedPedestrianCollision, InvalidGeometryAndTimeFailClosed) {
    Checker c;
    EXPECT_FALSE(c.bind({{{{0,0},{1,0},{.2,.2},{0,1}},Eigen::Vector2d::Zero()}},.25));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d(100,100),0.));
    EXPECT_FALSE(c.bind({{square(0,0),Eigen::Vector2d(NAN,0)}},.25));
    EXPECT_FALSE(c.bind({},0.));
    ASSERT_TRUE(c.bind({{square(0,0),Eigen::Vector2d::Zero()}},.25));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d(100,100),-1.));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d(100,100),NAN));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d(NAN,100),0.));
    c.clear(); EXPECT_TRUE(c.valid());EXPECT_TRUE(c.empty());
    EXPECT_FALSE(c.intersects(Eigen::Vector2d::Zero(),0.));
}
TEST(TimedPedestrianCollision, CircumscribedRobotAndMultipleObstacles) {
    Checker c;
    ASSERT_TRUE(c.bind({{square(5,0),Eigen::Vector2d::Zero()},
                        {square(-5,0),Eigen::Vector2d(1,0)}},.25));
    EXPECT_FALSE(c.intersects(Eigen::Vector2d::Zero(),0.));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),5.));
    const auto& robot=c.robotFootprint();ASSERT_EQ(8u,robot.size());
    for(size_t i=0;i<robot.size();++i) {
        Eigen::Vector2d e=robot[(i+1)%robot.size()]-robot[i];
        Eigen::Vector2d n(-e.y(),e.x());n.normalize();
        EXPECT_NEAR(.25,std::abs(n.dot(robot[i])),1e-12);
    }
}
TEST(TimedPedestrianCollision, FifthPedestrianFutureContactCannotBeIgnored) {
    Checker c;std::vector<Checker::Obstacle> people;
    for(int i=0;i<4;++i)people.push_back({square(5+i,5),Eigen::Vector2d::Zero()});
    people.push_back({square(0,2),Eigen::Vector2d(0,-1)});
    ASSERT_TRUE(c.bind(people,.25));EXPECT_FALSE(c.intersects(Eigen::Vector2d::Zero(),0.));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),2.));
    people.pop_back();ASSERT_TRUE(c.bind(people,.25));EXPECT_FALSE(c.intersects(Eigen::Vector2d::Zero(),2.));
}
TEST(TimedPedestrianCollision, CaneTipAndRotatingEnvelopeUseSameTime) {
    HumanCaneFootprint::Config config;config.enabled=true;Checker c;
    const auto footprint=HumanCaneFootprint::relative(config,0.);
    ASSERT_TRUE(c.bind({{square(.72,0,.03),Eigen::Vector2d::Zero()}},footprint));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),0.,0.));
    EXPECT_FALSE(c.intersects(Eigen::Vector2d::Zero(),0.,1.5707963267948966));
    // Repeated yaw changes must rebuild supports, rather than reusing stale axes.
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),0.,0.));
    ASSERT_TRUE(c.bind({{square(.72,1,.03),Eigen::Vector2d(0,-1)}},footprint));
    EXPECT_FALSE(c.intersects(Eigen::Vector2d::Zero(),0.,0.));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),1.,0.));
    EXPECT_TRUE(c.intersects(Eigen::Vector2d::Zero(),1.,NAN));
}
TEST(HumanCaneFootprint, BothCompleteDisksRemainCoveredWhenRotated) {
    HumanCaneFootprint::Config config;config.enabled=true;
    for(double yaw:{0.,.4,1.57,-2.}) {
        const auto hull=HumanCaneFootprint::relative(config,yaw);ASSERT_GE(hull.size(),8u);
        for(int disk=0;disk<2;++disk)for(int j=0;j<720;++j) {
            const double angle=j*3.14159265358979323846/360.;
            const double r=(disk?config.front_radius:config.body_radius)+config.margin;
            const Eigen::Vector2d p=r*Eigen::Vector2d(std::cos(angle),std::sin(angle))+
                (disk?Eigen::Vector2d(config.cane_length*std::cos(yaw),config.cane_length*std::sin(yaw)):Eigen::Vector2d::Zero());
            for(size_t i=0;i<hull.size();++i) {
                const Eigen::Vector2d e=hull[(i+1)%hull.size()]-hull[i],d=p-hull[i];
                EXPECT_GE(e.x()*d.y()-e.y()*d.x(),-1e-10);
            }
        }
    }
    config.cane_length=NAN;EXPECT_TRUE(HumanCaneFootprint::relative(config,0.).empty());
}
}
int main(int argc,char** argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
