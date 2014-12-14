#include <gtest/gtest.h>
#include <poseHelper.hpp>

TEST(poseHelperTest, distsquaredTest)
{
  Point2f p1 = Point2f(0.0, 0.0);
  Point2f p2 = Point2f(1.0, 1.0);
  Point2f p3 = Point2f(1.0, 0.0);
  Point2f p4 = Point2f(0.0, 1.0);

  double controlDist11 = 2.0;
  double controlDist10 = 1.0;
  double controlDist01 = 1.0;

  double dist11 = PoseHelper::distSquared(p1, p2);
  double dist10 = PoseHelper::distSquared(p1, p3);
  double dist01 = PoseHelper::distSquared(p1, p4);

  EXPECT_DOUBLE_EQ(controlDist11, dist11);
  EXPECT_DOUBLE_EQ(controlDist10, dist10);
  EXPECT_DOUBLE_EQ(controlDist01, dist01);
}

