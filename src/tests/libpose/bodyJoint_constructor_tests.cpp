#include <gtest/gtest.h>
#include <bodyJoint.hpp>

TEST(bodyJointTest, ConstructorTest)
{
  BodyJoint bj1;
  Point2f f2(0.0, 0.0);
  Point3f f3(0.0, 0.0, 0.0);

  EXPECT_EQ(0, bj1.getLimbID());
  EXPECT_TRUE(bj1.getJointName().empty());
  EXPECT_EQ(f2, bj1.getImageLocation());
  EXPECT_EQ(f3, bj1.getSpaceLocation());
  EXPECT_FALSE(bj1.getDepthSign());

  int limbID = 5;
  string jointName = "Some Name";
  Point2f imageLocation(1.234, -4.321);

  BodyJoint bj2(limbID, jointName, imageLocation);
  EXPECT_EQ(limbID, bj2.getLimbID());
  EXPECT_EQ(jointName, bj2.getJointName());
  EXPECT_EQ(imageLocation, bj2.getImageLocation());
  EXPECT_EQ(f3, bj2.getSpaceLocation());
  EXPECT_FALSE(bj2.getDepthSign());

}

