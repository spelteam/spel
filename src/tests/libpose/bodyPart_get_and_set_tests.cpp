#include <gtest/gtest.h>
#include <bodyPart.hpp>
TEST(bodyPartTest, GetAndSetTest)
{
  BodyPart bp1;
  int partID = 3;
  string partName = "Part Name";
  int parentJoint = 0;
  int childJoint = 0;
  bool isOccluded = false;
  float spaceLength = 1.343;
  POSERECT <Point2f> polygon (Point2f(1.0, 2.0), Point2f(2.0, 3.0), Point2f(3.0, 4.0), Point2f(4.0, 5.0));

  bp1.setPartID(partID);
  bp1.setPartName(partName);
  bp1.setParentJoint(parentJoint);
  bp1.setChildJoint(childJoint);
  bp1.setIsOccluded(isOccluded);
  bp1.setSpaceLength(spaceLength);
  bp1.setPartPolygon(polygon);

  EXPECT_EQ(partID, bp1.getPartID());
  EXPECT_EQ(partName, bp1.getPartName());
  EXPECT_EQ(parentJoint, bp1.getParentJoint());
  EXPECT_EQ(childJoint, bp1.getChildJoint());
  EXPECT_EQ(isOccluded, bp1.getIsOccluded());
  EXPECT_EQ(spaceLength, bp1.getSpaceLength());
  EXPECT_EQ(polygon, bp1.getPartPolygon());
}

