#include <gtest/gtest.h>
#include <skeleton.hpp>
#include "TestsFunctions.hpp"

using namespace std;

namespace SPEL
{
  TEST(skeletonTest, GetAndSetTest)
  {
    Skeleton s;
    string name = "Skeleton Test";
    tree <BodyPart> partTree;
    tree <BodyPart>::iterator pti;
    BodyPart bp;
    pti = partTree.begin();
    partTree.insert(pti, bp);
    tree <BodyJoint> jointTree;
    BodyJoint bj;
    tree <BodyJoint>::iterator jti;
    jti = jointTree.begin();
    jointTree.insert(jti, bj);
    float scale = 0.5;

    s.setName(name);
    s.setPartTree(partTree);
    s.setJointTree(jointTree);
    s.setScale(scale);

    EXPECT_EQ(name, s.getName());
    tree <BodyPart> pt1 = s.getPartTree();
    EXPECT_TRUE(equal(pt1.begin(), pt1.end(), partTree.begin()));
    tree <BodyJoint> jt1 = s.getJointTree();
    EXPECT_TRUE(equal(jt1.begin(), jt1.end(), jointTree.begin()));
    EXPECT_EQ(scale, s.getScale());
    EXPECT_EQ(1, s.getPartTreeCount());
    EXPECT_TRUE(bj == *s.getBodyJoint(bj.getLimbID()));
    EXPECT_TRUE(NULL == s.getBodyJoint(-1));
  }

  TEST(skeletonTest, Constructor)
  {
    //Load the input data
    vector<Frame*> frames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    Skeleton expected = frames[0]->getSkeleton();
    Skeleton actual(expected);
    ASSERT_EQ(expected.getPartTree().size(), actual.getPartTree().size());
    EXPECT_EQ(expected, actual);
  }

  TEST(skeletonTest, getPartTreeCount)
  {
    //Load the input data
    vector<Frame*> frames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    Skeleton skeleton = frames[0]->getSkeleton();

    //Part count in the skeleton from "trijumpSD_50x41.xml"
    int PartCount = 17;

    //Compare
    EXPECT_EQ(PartCount, skeleton.getPartTreeCount());
  }

  TEST(skeletonTest, getBodyPart)
  {
    //Create set of parts
    BodyPart A(0, "Head", 0, 1);
    A.setLWRatio(1.9f);
    BodyPart B(1, "Left Hand", 1, 2);
    B.setLWRatio(3.2f);
    BodyPart C(2, "Right Hand", 1, 4);
    C.setLWRatio(3.2f);
    BodyPart D(3, "Trunk", 1, 3);
    D.setLWRatio(1.454);
    BodyPart E(4, "Left Leg", 3, 5);
    E.setLWRatio(3.4);
    BodyPart F(5, "Right Leg", 3, 6);
    F.setLWRatio(3.4);

    vector<BodyPart> parts;
    parts.push_back(A);
    parts.push_back(B);
    parts.push_back(C);
    parts.push_back(D);
    parts.push_back(E);
    parts.push_back(F);

    //Create part tree
    tree<BodyPart> partTree;
    tree<BodyPart>::iterator p;
    p = partTree.begin();
    p = partTree.append_child(p, A);
    partTree.append_child(p, B);
    partTree.append_child(p, C);
    p = partTree.append_child(p, D);
    partTree.append_child(p, E);
    partTree.append_child(p, F);

    //Create skeleton
    Skeleton skeleton;
    skeleton.setPartTree(partTree);

    //Get part by ID and compare
    for (int i = 0; i < parts.size(); i++)
    {
      BodyPart* actual = skeleton.getBodyPart(i);
      EXPECT_EQ(parts[i], *actual);
      EXPECT_EQ(parts[i].getPartName(), actual->getPartName());
      EXPECT_EQ(parts[i].getParentJoint(), actual->getParentJoint());
      EXPECT_EQ(parts[i].getChildJoint(), actual->getChildJoint());
      EXPECT_EQ(parts[i].getLWRatio(), actual->getLWRatio());
    }
  }

}
