#include <gtest/gtest.h>
#include <bodyPart.hpp>

TEST(bodyPartTest, OperatorsTest)
{
  int partID = 3;
  string partName = "Part Name";
  BodyJoint *parentJoint = new BodyJoint();
  BodyJoint *childJoint = new BodyJoint();
  bool isOccluded = true;
  float spaceLength = 1.343;

  BodyPart bp1(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);
  BodyPart bp2(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);

  EXPECT_TRUE(bp1 == bp2);
  EXPECT_FALSE(bp1 != bp2);

  delete parentJoint;
  delete childJoint;
}

