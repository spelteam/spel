#include <gtest/gtest.h>
#include <lockframe.hpp>
TEST(lockframeTest, GetAndSetTest)
{
  int id = 7;
  Mat image = Mat::zeros(100, 100, CV_32F);
  Point pt1, pt2;
  pt1.x = 10;
  pt1.y = 10;
  pt2.x = 90;
  pt2.y = 90;
  int icolor = 0x00FFAA11;
  Scalar color(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
  line(image, pt1, pt2, color, 5, 8);

  Mat mask = Mat::zeros(100, 100, CV_32F);
  pt1.x = 10;
  pt1.y = 90;
  pt2.x = 90;
  pt2.y = 10;
  icolor = 0x00AABBCC;
  Scalar color1(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
  line(mask, pt1, pt2, color1, 9, 8);

  Skeleton skeleton;
  Point2f groundPoint(0.0, 0.0);

  Lockframe lockframe;
  lockframe.setID(id);
  lockframe.setImage(image);
  lockframe.setMask(mask);
  lockframe.setSkeleton(skeleton);
  lockframe.setGroundPoint(groundPoint);

  EXPECT_EQ(id, lockframe.getID());
  EXPECT_EQ(0, countNonZero(image != lockframe.getImage()));
  EXPECT_EQ(0, countNonZero(mask != lockframe.getMask()));
  EXPECT_EQ(skeleton, lockframe.getSkeleton());
  EXPECT_EQ(groundPoint, lockframe.getGroundPoint());

  EXPECT_EQ(LOCKFRAME, lockframe.getFrametype());

}

