#include <gtest/gtest.h>
#include <bodyPart.hpp>

TEST(bodyPartTest, ConstructorTest)
{
  BodyPart bp1;
  EXPECT_EQ(0, bp1.getPartID());
  EXPECT_TRUE(bp1.getPartName().empty());
  EXPECT_EQ(0, bp1.getParentJoint());
  EXPECT_EQ(0, bp1.getChildJoint());
  EXPECT_FALSE(bp1.getIsOccluded());
  EXPECT_EQ(0, bp1.getSpaceLength());

  int partID = 3;
  string partName = "Part Name";
  int parentJoint = 0;
  int childJoint = 0;
  bool isOccluded = true;
  float spaceLength = 1.343;

  BodyPart bp2(partID, partName, parentJoint, childJoint);
  EXPECT_EQ(partID, bp2.getPartID());
  EXPECT_EQ(partName, bp2.getPartName());
  EXPECT_EQ(parentJoint, bp2.getParentJoint());
  EXPECT_EQ(childJoint, bp2.getChildJoint());
  EXPECT_FALSE(bp2.getIsOccluded());
  EXPECT_EQ(0, bp2.getSpaceLength());
  
  BodyPart bp3(partID, partName, parentJoint, childJoint, isOccluded);
  EXPECT_EQ(partID, bp3.getPartID());
  EXPECT_EQ(partName, bp3.getPartName());
  EXPECT_EQ(parentJoint, bp3.getParentJoint());
  EXPECT_EQ(childJoint, bp3.getChildJoint());
  EXPECT_TRUE(bp3.getIsOccluded());
  EXPECT_EQ(0, bp3.getSpaceLength());
  
  BodyPart bp4(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);
  EXPECT_EQ(partID, bp4.getPartID());
  EXPECT_EQ(partName, bp4.getPartName());
  EXPECT_EQ(parentJoint, bp4.getParentJoint());
  EXPECT_EQ(childJoint, bp4.getChildJoint());
  EXPECT_TRUE(bp4.getIsOccluded());
  EXPECT_EQ(spaceLength, bp4.getSpaceLength());
}

