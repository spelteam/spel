// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif
#include <gtest/gtest.h>
#include <solvlet.hpp>
#include "TestsFunctions.hpp"

#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

namespace SPEL
{
  TEST(solvlet_Tests, Solvlet_0)
  {
    Solvlet s;
    EXPECT_EQ(-1, s.getFrameID());
  }

  TEST(solvlet_Tests, Solvlet_1)
  {
    vector<LimbLabel> Labels;
    for (int i = 0; i < 10; i++)
    {
      float x = static_cast<float>(i);
      vector<Point2f> polygon = { Point2f(x, x) };
      vector<Score> scores = { Score(x, "", 1.0f) };
      LimbLabel label(i, Point2f(x, x), x, polygon, scores);
      Labels.push_back(label);
      polygon.clear();
      scores.clear();
    }

    Solvlet s(0, Labels);
    EXPECT_EQ(0, s.getFrameID());
    EXPECT_EQ(Labels, s.getLabels());

    Labels.clear();
  }

  TEST(solvlet_Tests, SetAndGet)
  {
    vector<LimbLabel> Labels;
    for (int i = 0; i < 10; i++)
      {
        float x = static_cast<float>(i);
        vector<Point2f> polygon = { Point2f(x, x) };
        vector<Score> scores = { Score(x, "", 1.0f) };
        LimbLabel label(i, Point2f(x, x), x, polygon, scores);
        Labels.push_back(label);
        polygon.clear();
        scores.clear();
      }

    Solvlet s;

    s.setFrameID(10);
    EXPECT_EQ(10, s.getFrameID());

    s.setLabels(Labels);
    EXPECT_EQ(Labels, s.getLabels());
    EXPECT_EQ(Labels, *s.getLabelsPtr());

    Labels.clear();
  }

  TEST(solvlet_Tests, Operators)
  {
    Solvlet s0, s1;

    s0.setFrameID(0);
    s1.setFrameID(1);

    vector<LimbLabel> Labels;
    for (int i = 0; i < 10; i++)
    {
      float x = static_cast<float>(i);
      vector<Point2f> polygon = { Point2f(x, x) };
      vector<Score> scores = { Score(x, "", 1.0f) };
      LimbLabel label(i, Point2f(x, x), x, polygon, scores);
      Labels.push_back(label);
      polygon.clear();
      scores.clear();
    }

    Solvlet s(0, Labels);

    EXPECT_TRUE(s0 < s1);
    EXPECT_TRUE(s1 > s0);
    EXPECT_FALSE(s0 > s1);
    EXPECT_FALSE(s1 > s0);
    EXPECT_FALSE(s0 > s0);// ?
    EXPECT_FALSE(s0 < s0);// ?

    s0 = s;
    EXPECT_EQ(s.getFrameID(), s0.getFrameID());
    EXPECT_EQ(s.getLabels(), s0.getLabels());

    Labels.clear();
  }  

  TEST(solvlet_Tests, toSkeleton)
  {    
    //Load the input data
    vector<Frame*> Frames = LoadTestProject( "speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    int frameID = 0;
    Skeleton skeleton = Frames[frameID]->getSkeleton();
    
    // Build Labels from skeleton
    vector<LimbLabel> Labels;
    map<int, POSERECT<Point2f>> PartRects = SkeletonRects(skeleton);
    map<int, pair<Point2f, Point2f>> PartsLocations = getPartLocations(skeleton);

    for (int i = 0; i < PartRects.size(); i++)
    {
      vector<Point2f> polygon = PartRects[i].asVector();	  
      Point2f center = Point2f(0, 0);
      for (int k = 0; k < polygon.size(); k++)
        center += polygon[k];
      center = 0.25*center;
      Point2f p0 = PartsLocations[i].first;
      Point2f p1 = PartsLocations[i].second;
      float angle = spelHelper::angle2D(p0.x, p0.y, p1.x, p1.y)* (180.0 / M_PI);
      vector<Score> scores = { Score(0.0f, "", 1.0f) };
      LimbLabel label(i, center, angle, polygon, scores);
      Labels.push_back(label);
      polygon.clear();
      scores.clear();
    }
    
    PartRects.clear();
    PartsLocations.clear();

    // Create expected value
    Skeleton Expected_Skeleton = skeleton;
    Expected_Skeleton.infer3D();

    // Run "toSkeleton"
    Solvlet s(frameID, Labels);
    Skeleton Actual_Skeleton = s.toSkeleton(skeleton);
    
    // Calculate joints location
    map<int, pair<Point2f, Point2f>> ActualPartsLocations = getPartLocations(Actual_Skeleton);
    map<int, pair<Point2f, Point2f>> ExpectedPartsLocations = getPartLocations(Expected_Skeleton);

    // Temporary debug info
    for (int i = 0; i < ExpectedPartsLocations.size(); i++)
    {
      cout << "Locations(PartID = " << i << "):" << endl;
      cout << "  Expected: {e0 = " << ExpectedPartsLocations[i].first << ", e1 = " << ExpectedPartsLocations[i].second << "}" << endl;
      cout << "    Actual: {a0 = " << ActualPartsLocations[i].first << ", a1 = " << ActualPartsLocations[i].second << "}" << endl;
    }
    //

    // Compare
    EXPECT_EQ(Expected_Skeleton, Actual_Skeleton); // don't work!??? == true 

    // Compare body parts
    tree<BodyPart> ExpectedPartTree = Expected_Skeleton.getPartTree();
    tree<BodyPart> PartTree = Actual_Skeleton.getPartTree();
    tree<BodyPart>::iterator p0 = ExpectedPartTree.begin();
    for (tree<BodyPart>::iterator p1 = PartTree.begin(); p1 != PartTree.end(); p1++)
    {
      EXPECT_EQ(p0->getPartID(), p1->getPartID());
      EXPECT_EQ(p0->getParentJoint(), p1->getParentJoint());
      EXPECT_EQ(p0->getChildJoint(), p1->getChildJoint());
      EXPECT_EQ(p0->getLWRatio(), p1->getLWRatio());
      p0++;
    }

    // Compare part joints
    float error = 5.0f; // pixels
    bool JointsIsNear = true;
    for (int i = 0; i < ExpectedPartsLocations.size(); i++)
    {
      //EXPECT_NEAR(ExpectedPartsLocations[i].first.x, ActualPartsLocations[i].first.x, error);
      //EXPECT_NEAR(ExpectedPartsLocations[i].first.y, ActualPartsLocations[i].first.y, error);
      //EXPECT_NEAR(ExpectedPartsLocations[i].second.x, ActualPartsLocations[i].second.x, error);
      //EXPECT_NEAR(ExpectedPartsLocations[i].second.y, ActualPartsLocations[i].second.y, error);
      if(abs(ExpectedPartsLocations[i].first.x - ActualPartsLocations[i].first.x) > error)
        JointsIsNear = false;
      if (abs(ExpectedPartsLocations[i].first.y - ActualPartsLocations[i].first.y) > error)
        JointsIsNear = false;
      if (abs(ExpectedPartsLocations[i].second.x - ActualPartsLocations[i].second.x) > error)
        JointsIsNear = false;
      if (abs(ExpectedPartsLocations[i].second.y - ActualPartsLocations[i].second.y) > error)
        JointsIsNear = false;
    }
    EXPECT_TRUE(JointsIsNear);

    ActualPartsLocations.clear();
    ExpectedPartsLocations.clear();

    Labels.clear();
  }

}
