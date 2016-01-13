// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#include <fstream>
#include <bodyPart.hpp>

using namespace std;
using namespace cv;

namespace SPEL
{
  TEST(bodyPartTest, OperatorsTest)
  {
    int partID = 3;
    string partName = "Part Name";
    int parentJoint = 0;
    int childJoint = 0;
    bool isOccluded = true;
    float spaceLength = 1.343f;

    float expectedDistance = 3.4f;
    POSERECT <Point2f> partPolygon(Point2f(0.0, 0.0), Point2f(1.0, 1.0), Point2f(2.0, 2.0), Point2f(3.0, 3.0));
    float lwRatio = 1.8f;
    float relativeLength = 0.6f;
    float searchRadius = 0.5f;
    float rotationSearchRange = 3.14f;

    BodyPart bp1(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);
    BodyPart bp2(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);

    EXPECT_TRUE(bp1 == bp2);
    EXPECT_FALSE(bp1 != bp2);

    //Testinting operator "="
    bp2.setExpectedDistance(expectedDistance);
    bp2.setPartPolygon(partPolygon);
    bp2.setLWRatio(lwRatio);
    bp2.setRelativeLength(relativeLength);
    bp2.setSearchRadius(searchRadius);
    bp2.setRotationSearchRange(rotationSearchRange);
    BodyPart bp3 = bp2;
    EXPECT_EQ(partID, bp3.getPartID());
    EXPECT_EQ(partName, bp3.getPartName());
    EXPECT_EQ(parentJoint, bp3.getParentJoint());
    EXPECT_EQ(childJoint, bp3.getChildJoint());
    EXPECT_EQ(isOccluded, bp3.getIsOccluded());
    EXPECT_EQ(expectedDistance, bp3.getExpectedDistance());
    EXPECT_EQ(partPolygon, bp3.getPartPolygon());
    EXPECT_EQ(lwRatio, bp3.getLWRatio());
    EXPECT_EQ(relativeLength, bp3.getRelativeLength());
    EXPECT_EQ(searchRadius, bp3.getSearchRadius());
    EXPECT_EQ(rotationSearchRange, bp3.getRotationSearchRange());
  }

  TEST(bodyPartTest, MoveAssigmentOperator)
  {
    int partID = 3;
    string partName = "Part Name";
    int parentJoint = 0;
    int childJoint = 0;
    bool isOccluded = true;
    float spaceLength = 1.343f;

    float expectedDistance = 3.4f;
    POSERECT <Point2f> partPolygon(Point2f(0.0, 0.0), Point2f(1.0, 1.0), Point2f(2.0, 2.0), Point2f(3.0, 3.0));
    float lwRatio = 1.8f;
    float relativeLength = 0.6f;
    float searchRadius = 0.5f;
    float rotationSearchRange = 3.14f;
    BodyPart bp1(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);
    bp1.setExpectedDistance(expectedDistance);
    bp1.setPartPolygon(partPolygon);
    bp1.setLWRatio(lwRatio);
    bp1.setRelativeLength(relativeLength);
    bp1.setSearchRadius(searchRadius);
    bp1.setRotationSearchRange(rotationSearchRange);

    BodyPart bp2 = static_cast<BodyPart&&>(bp1);
    EXPECT_EQ(partID, bp2.getPartID());
    EXPECT_EQ(partName, bp2.getPartName());
    EXPECT_EQ(parentJoint, bp2.getParentJoint());
    EXPECT_EQ(childJoint, bp2.getChildJoint());
    EXPECT_EQ(isOccluded, bp2.getIsOccluded());
    EXPECT_EQ(expectedDistance, bp2.getExpectedDistance());
    EXPECT_EQ(partPolygon, bp2.getPartPolygon());
    EXPECT_EQ(lwRatio, bp2.getLWRatio());
    EXPECT_EQ(relativeLength, bp2.getRelativeLength());
    EXPECT_EQ(searchRadius, bp2.getSearchRadius());
    EXPECT_EQ(rotationSearchRange, bp2.getRotationSearchRange());
  }
 
  TEST(bodyPartTest, OutputOperator)
  {
    //Create bodyPart
    int partID = 3;
    string partName = "Part Name";
    int parentJoint = 0;
    int childJoint = 0;
    bool isOccluded = true;
    float spaceLength = 1.343f;
    BodyPart bodyPart(partID, partName, parentJoint, childJoint, isOccluded, spaceLength);

    //Testing operator "<<"
    ofstream f1("bodyPartTest_OutputOperator.txt");
    f1 << bodyPart;
    f1.close();
    ifstream f2("bodyPartTest_OutputOperator.txt");
    ASSERT_TRUE(f2.is_open());
    //char S[255];
    //f2.getline(S, 255);
    int i;
    f2 >> i;
    EXPECT_EQ(partID, i);

  }
}
