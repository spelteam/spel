#include <gtest/gtest.h>
#include <bodyPart.hpp>

TEST(bodyPartTest, OperatorsTest)
{
  int partID = 3;
  string partName = "Part Name";
  int parentJoint = 0;
  int childJoint = 0;
  bool isOccluded = true;
  float spaceLength = 1.343;

  BodyPart bp1(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);
  BodyPart bp2(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);

  EXPECT_TRUE(bp1 == bp2);
  EXPECT_FALSE(bp1 != bp2);
}

