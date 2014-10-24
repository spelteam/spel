#include <gtest/gtest.h>
#include <bodyJoint.hpp>

TEST(bodyJointTest, OperatorsTest)
{
  BodyJoint bj1, bj2;
  int limbID = 5;
  string jointName = "Some Name";
  Point2f imageLocation(2.534, -1.635);
  Point3f spaceLocation(1.231, -2.0, -1.5);
  bool depthSign = true;  
  bj1.setLimbID(limbID);
  bj1.setJointName(jointName);
  bj1.setImageLocation(imageLocation);
  bj1.setSpaceLocation(spaceLocation);
  bj1.setDepthSign(depthSign);

  bj2.setLimbID(limbID);
  bj2.setJointName(jointName);
  bj2.setImageLocation(imageLocation);
  bj2.setSpaceLocation(spaceLocation);
  bj2.setDepthSign(depthSign);

  EXPECT_TRUE(bj1 == bj2);
  EXPECT_FALSE(bj1 != bj2);
}
