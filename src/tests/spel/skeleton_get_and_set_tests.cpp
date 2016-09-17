// SPEL definitions
#include "predef.hpp"

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
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> frames = project.getFrames();

    Skeleton expected = frames[0]->getSkeleton();
    Skeleton actual(expected);
    ASSERT_EQ(expected.getPartTree().size(), actual.getPartTree().size());
    EXPECT_EQ(expected, actual);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
  }

  TEST(skeletonTest, getPartTreeCount)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> frames = project.getFrames();
    Skeleton skeleton = frames[0]->getSkeleton();

    //Part count in the skeleton from "trijumpSD_50x41.xml"
    int PartCount = 17;

    //Compare
    EXPECT_EQ(PartCount, skeleton.getPartTreeCount());

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
  }

  TEST(skeletonTest, getBodyPart)
  {
    //Create set of parts
    BodyPart A(0, "Head", 0, 1, false, 0.0f);
    A.setLWRatio(1.9f);
    BodyPart B(1, "Left Hand", 1, 2, false, 0.0f);
    B.setLWRatio(3.2f);
    BodyPart C(2, "Right Hand", 1, 4, false, 0.0f);
    C.setLWRatio(3.2f);
    BodyPart D(3, "Trunk", 1, 3, false, 0.0f);
    D.setLWRatio(1.454f);
    BodyPart E(4, "Left Leg", 3, 5, false, 0.0f);
    E.setLWRatio(3.4f);
    BodyPart F(5, "Right Leg", 3, 6, false, 0.0f);
    F.setLWRatio(3.4f);

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
    p = partTree.insert(p, A);
    partTree.append_child(p, B);
    partTree.append_child(p, C);
    p = partTree.append_child(p, D);
    partTree.append_child(p, E);
    partTree.append_child(p, F);

    //Create skeleton
    Skeleton skeleton;
    skeleton.setPartTree(partTree);

    //Get part by ID and compare
    for (unsigned int i = 0; i < parts.size(); i++)
    {
      BodyPart* actual = skeleton.getBodyPart(i);
      EXPECT_EQ(parts[i], *actual);
      EXPECT_EQ(parts[i].getPartName(), actual->getPartName());
      EXPECT_EQ(parts[i].getParentJoint(), actual->getParentJoint());
      EXPECT_EQ(parts[i].getChildJoint(), actual->getChildJoint());
      EXPECT_EQ(parts[i].getLWRatio(), actual->getLWRatio());
    }
    parts.clear();
  }

TEST(skeletonTest, getPartTreePtr)
  {
    //Create set of parts
    BodyPart A(0, "Head", 0, 1, false, 0.0f);
    A.setLWRatio(1.9f);
    BodyPart B(1, "Left Hand", 1, 2, false, 0.0f);
    B.setLWRatio(3.2f);
    BodyPart C(2, "Right Hand", 1, 4, false, 0.0f);
    C.setLWRatio(3.2f);
    BodyPart D(3, "Trunk", 1, 3, false, 0.0f);
    D.setLWRatio(1.454f);
    BodyPart E(4, "Left Leg", 3, 5, false, 0.0f);
    E.setLWRatio(3.4f);
    BodyPart F(5, "Right Leg", 3, 6, false, 0.0f);
    F.setLWRatio(3.4f);

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
    p = partTree.insert(p, A);
    partTree.append_child(p, B);
    partTree.append_child(p, C);
    p = partTree.append_child(p, D);
    partTree.append_child(p, E);
    partTree.append_child(p, F);

    //Create skeleton
    Skeleton skeleton;
    skeleton.setPartTree(partTree);

    //Create actual value
    tree<BodyPart>* partTree_actual = skeleton.getPartTreePtr();

    //Compare
    tree<BodyPart>::iterator k = partTree_actual->begin();
    for (tree<BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
    {
      EXPECT_EQ(i->getPartID(), k->getPartID());
      EXPECT_EQ(i->getPartName(), k->getPartName());
      EXPECT_EQ(i->getParentJoint(), k->getParentJoint());
      EXPECT_EQ(i->getChildJoint(), k->getChildJoint());
      EXPECT_EQ(i->getIsOccluded(), k->getIsOccluded());
      EXPECT_EQ(i->getLWRatio(), k->getLWRatio());
      EXPECT_EQ(i->getPartPolygon(), k->getPartPolygon());
      cout << "i = " << i->getPartID() << " ~ k = " << k->getPartID() << endl;
      k++;
    }
    EXPECT_TRUE(equal(partTree_actual->begin(), partTree_actual->end(), partTree.begin()));

    parts.clear();
    partTree.clear();
  }

  TEST(skeletonTest, GetJointTreePtr)
  {
    //Prepare test data
    Skeleton skeleton;
    tree <BodyJoint> jointTree;
    BodyJoint bj;
    tree <BodyJoint>::iterator i;
    i = jointTree.begin();
    i = jointTree.insert(i, BodyJoint(0, "0", Point2f(0.0f, 0.0f), Point3f(0.0f, 0.0f, 0.0f), false));
    jointTree.insert(i, BodyJoint(1, "1", Point2f(1.0f, 1.0f), Point3f(1.0f, 1.0f, 1.0f), true));
    skeleton.setJointTree(jointTree);

    //Create actual value
    tree <BodyJoint>* joinntTree_actual = skeleton.getJointTreePtr();

    //Compare
    tree<BodyJoint>::iterator k = joinntTree_actual->begin();
    for (tree<BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
    {
      EXPECT_EQ(i->getLimbID(), k->getLimbID());
      EXPECT_EQ(i->getJointName(), k->getJointName());
      EXPECT_EQ(i->getImageLocation(), k->getImageLocation());
      EXPECT_EQ(i->getSpaceLocation(), k->getSpaceLocation());
      EXPECT_EQ(i->getDepthSign(), k->getDepthSign());
      k++;
    }
    EXPECT_TRUE(equal(joinntTree_actual->begin(), joinntTree_actual->end(), jointTree.begin()));

    jointTree.clear();
  }



}
