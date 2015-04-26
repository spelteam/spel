#include <gtest/gtest.h>
#include <poseHelper.hpp>

TEST(poseHelperTest, angleTest)
{
  Point2f p1 = Point2f(0.0, 0.0);
  Point2f p2 = Point2f(1.0, 1.0);
  Point2f p3 = Point2f(1.0, 0.0);
  Point2f p4 = Point2f(0.0, 1.0);
  Point2f p5 = Point2f(-1.0, 0.0);

  double controlAngle45 = 45;
  double controlAngle0 = 0;
  double controlAngle90 = 90;
  double controlAngle180 = 180;

  double angle45 = PoseHelper::angle2D(p1.x, p1.y, p2.x, p2.y) * (180.0 / M_PI);
  double angle0 = PoseHelper::angle2D(p1.x, p1.y, p3.x, p3.y) * (180.0 / M_PI);
  double angle90 = PoseHelper::angle2D(p1.x, p1.y, p4.x, p4.y) * (180.0 / M_PI);
  double angle180 = PoseHelper::angle2D(p1.x, p1.y, p5.x, p5.y) * (180.0 / M_PI);

  EXPECT_DOUBLE_EQ(controlAngle45, angle45);
  EXPECT_DOUBLE_EQ(controlAngle0, angle0);
  EXPECT_DOUBLE_EQ(controlAngle90, angle90);
  EXPECT_DOUBLE_EQ(controlAngle180, angle180);
}

