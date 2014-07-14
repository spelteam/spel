#include <gtest/gtest.h>
#include <bodyPart.hpp>
TEST(bodyPartTest, GetAndSetTest)
{
  BodyPart bp1;
  int partID = 3;
  string partName = "Part Name";
  BodyJoint *parentJoint = new BodyJoint();
  BodyJoint *childJoint = new BodyJoint();
  bool isOccluded = false;
  float spaceLength = 1.343;

  bp1.setPartID(partID);
  bp1.setPartName(partName);
  bp1.setParentJoint(parentJoint);
  bp1.setChildJoint(childJoint);
  bp1.setIsOccluded(isOccluded);
  bp1.setSpaceLength(spaceLength);

  EXPECT_EQ(partID, bp1.getPartID());
  EXPECT_EQ(partName, bp1.getPartName());
  EXPECT_EQ(parentJoint, bp1.getParentJoint());
  EXPECT_EQ(childJoint, bp1.getChildJoint());
  EXPECT_EQ(isOccluded, bp1.getIsOccluded());
  EXPECT_EQ(spaceLength, bp1.getSpaceLength());
  
  delete parentJoint;
  delete childJoint;
}

