#include <gtest/gtest.h>
#include <poseHelper.hpp>

TEST(poseHelperTest, poserectTest)
{
  Point2f p1 = Point2f(1.0, 2.0);
  Point2f p2 = Point2f(2.0, 2.0);
  Point2f p3 = Point2f(2.0, 1.0);
  Point2f p4 = Point2f(1.0, 1.0);

  POSERECT <Point2f> rect1 = POSERECT <Point2f> (p1, p2, p3, p4);
  EXPECT_EQ(p1, rect1.point1);
  EXPECT_EQ(p2, rect1.point2);
  EXPECT_EQ(p3, rect1.point3);
  EXPECT_EQ(p4, rect1.point4);

  POSERECT <Point2f> rect2 = POSERECT <Point2f> (p1, p2, p3, p4);
  POSERECT <Point2f> rect3 = POSERECT <Point2f> (p4, p3, p2, p1);
  EXPECT_TRUE(rect1 == rect2);
  EXPECT_TRUE(rect1 != rect3);

  Point2f inside = Point2f(1.5, 1.5);
  Point2f outside = Point2f(3.0, 3.0);
  Point2f onTheEdge = Point2f(1.0, 1.5);

  EXPECT_EQ(1, rect1.containsPoint(inside));
  EXPECT_EQ(-1, rect1.containsPoint(outside));
  EXPECT_EQ(0, rect1.containsPoint(onTheEdge));
  
  vector <Point2f> v = rect1.asVector();
  EXPECT_EQ(4, v.size());

  EXPECT_EQ(rect1.point1, v.at(0));
  EXPECT_EQ(rect1.point2, v.at(1));
  EXPECT_EQ(rect1.point3, v.at(2));
  EXPECT_EQ(rect1.point4, v.at(3));
}

