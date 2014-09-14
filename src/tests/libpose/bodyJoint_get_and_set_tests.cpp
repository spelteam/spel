#include <gtest/gtest.h>
#include <bodyJoint.hpp>

TEST(bodyJointTest, GetAndSetTest)
{
  BodyJoint bj1;
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
  
  EXPECT_EQ(limbID, bj1.getLimbID());
  EXPECT_EQ(jointName, bj1.getJointName());
  EXPECT_EQ(imageLocation, bj1.getImageLocation());
  EXPECT_EQ(spaceLocation, bj1.getSpaceLocation());
  EXPECT_EQ(depthSign, bj1.getDepthSign());
}

