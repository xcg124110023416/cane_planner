#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <path_searching/timed_pedestrian_collision.h>

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
}
int main(int argc,char** argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
