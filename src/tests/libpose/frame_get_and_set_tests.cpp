#include <gtest/gtest.h>
#include <frame.hpp>
TEST(frameTest, GetAndSetTest)
{
  int id = 7;
  RNG rng(0xFFFFFFFF);
  Mat image = Mat::zeros(100, 100, CV_32F);
  Point pt1, pt2;
  pt1.x = rng.uniform(0, 100);
  pt1.y = rng.uniform(0, 100);
  pt2.x = rng.uniform(0, 100);
  pt2.y = rng.uniform(0, 100);
  int icolor = (unsigned) rng;
  Scalar color(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
  line(image, pt1, pt2, color, rng.uniform(1, 10), 8);

  Mat mask = Mat::zeros(100, 100, CV_32F);
  pt1.x = rng.uniform(0, 100);
  pt1.y = rng.uniform(0, 100);
  pt2.x = rng.uniform(0, 100);
  pt2.y = rng.uniform(0, 100);
  icolor = (unsigned) rng;
  Scalar color1(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
  line(mask, pt1, pt2, color1, rng.uniform(1, 10), 8);

  Skeleton skeleton;
  Point2f groundPoint(0.0, 0.0);

  Frame frame;
  frame.setID(id);
  frame.setImage(image);
  frame.setMask(mask);
  frame.setSkeleton(skeleton);
  frame.setGroundPoint(groundPoint);

  EXPECT_EQ(id, frame.getID());
  EXPECT_EQ(0, countNonZero(image != frame.getImage()));
  EXPECT_EQ(0, countNonZero(mask != frame.getMask()));
  EXPECT_EQ(skeleton, frame.getSkeleton());
  EXPECT_EQ(groundPoint, frame.getGroundPoint());
}

