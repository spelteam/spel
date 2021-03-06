// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#include <bodyJoint.hpp>

using namespace std;
using namespace cv;

namespace SPEL
{
  TEST(bodyJointTest, ConstructorTest)
  {
    BodyJoint bj1;
    Point2f f2(0.0f, 0.0f);
    Point3f f3(0.0f, 0.0f, 0.0f);

    EXPECT_EQ(0, bj1.getLimbID());
    EXPECT_TRUE(bj1.getJointName().empty());
    EXPECT_EQ(f2, bj1.getImageLocation());
    EXPECT_EQ(f3, bj1.getSpaceLocation());
    EXPECT_FALSE(bj1.getDepthSign());

    int limbID = 5;
    bool depthSign = true;
    string jointName = "Some Name";
    Point2f imageLocation(1.234f, -4.321f);
    Point3f spaceLocation(1.234f, -4.321f, 0.0f);

    BodyJoint bj2(limbID, jointName, imageLocation);
    EXPECT_EQ(limbID, bj2.getLimbID());
    EXPECT_EQ(jointName, bj2.getJointName());
    EXPECT_EQ(imageLocation, bj2.getImageLocation());
    EXPECT_EQ(f3, bj2.getSpaceLocation());
    EXPECT_FALSE(bj2.getDepthSign());

    //Testing constructor "BodyJoint(BodyJoint)"
    BodyJoint bj3(bj2);
    EXPECT_EQ(limbID, bj3.getLimbID());
    EXPECT_EQ(jointName, bj3.getJointName());
    EXPECT_EQ(imageLocation, bj3.getImageLocation());
    EXPECT_EQ(f3, bj3.getSpaceLocation());
    EXPECT_FALSE(bj3.getDepthSign());

    //Testing constructor "BodyJoint(limbID, jointName, imageLocation, spaceLocation)"
    BodyJoint bj4(limbID, jointName, imageLocation, spaceLocation);
    EXPECT_EQ(limbID, bj4.getLimbID());
    EXPECT_EQ(jointName, bj4.getJointName());
    EXPECT_EQ(imageLocation, bj4.getImageLocation());
    EXPECT_EQ(spaceLocation, bj4.getSpaceLocation());
    EXPECT_FALSE(bj4.getDepthSign());

    //Testing constructor "BodyJoint(limbID, jointName, imageLocation, spaceLocation, depthSign)"
    BodyJoint bj5(limbID, jointName, imageLocation, spaceLocation, depthSign);
    EXPECT_EQ(limbID, bj5.getLimbID());
    EXPECT_EQ(jointName, bj5.getJointName());
    EXPECT_EQ(imageLocation, bj5.getImageLocation());
    EXPECT_EQ(spaceLocation, bj5.getSpaceLocation());
    EXPECT_TRUE(bj5.getDepthSign());
  }

  TEST(bodyJointTest, MoveConstructor)
  {    
    int limbID = 5;
    string jointName = "Some Name";
    Point2f imageLocation(1.234f, -4.321f);
    BodyJoint bj1(limbID, jointName, imageLocation);

    BodyJoint bj2(static_cast<BodyJoint&&>(bj1));
    EXPECT_EQ(limbID, bj2.getLimbID());
    EXPECT_EQ(jointName, bj2.getJointName());
    EXPECT_EQ(imageLocation, bj2.getImageLocation());
    EXPECT_FALSE(bj2.getDepthSign());
  }

}
