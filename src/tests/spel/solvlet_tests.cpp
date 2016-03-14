// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif
#include <gtest/gtest.h>
#include <solvlet.hpp>
#include <lockframe.hpp>
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

    for (unsigned int i = 0; i < PartRects.size(); i++)
    {
      vector<Point2f> polygon = PartRects[i].asVector();	  
      Point2f center = Point2f(0, 0);
      for (unsigned int k = 0; k < polygon.size(); k++)
        center += polygon[k];
      center = 0.25*center;
      Point2f p0 = PartsLocations[i].first;
      Point2f p1 = PartsLocations[i].second;
      float angle = spelHelper::angle2D(p0.x, p0.y, p1.x, p1.y)*static_cast<float>(180.0 / M_PI);
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
    for (unsigned int i = 0; i < ExpectedPartsLocations.size(); i++)
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
    for (unsigned int i = 0; i < ExpectedPartsLocations.size(); i++)
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

  TEST(solvlet_Tests, evaluateSolution)
  {
    // Prepare test data
    Point2f D = Point2f(0.0f, 50.0f);
    float dx = 5.0f;
    Point2f shift(dx, 0.0f);
    vector<Point2f> polygon0 = {Point2f(10.0f, 10.0f), Point2f(30.0f, 10.0f), Point2f(30.0f, 20.0f), Point2f(10.0f, 20.0f) };
    vector<Point2f> polygon1 = { Point2f(10.0f, 10.0f) + D, Point2f(30.0f, 10.0f) + D, Point2f(30.0f, 20.0f) + D, Point2f(10.0f, 20.0f) + D };
    vector<Point2f> shifted_polygon0 = { Point2f(10.0f, 10.0f) + shift, Point2f(30.0f, 10.0f) + shift, Point2f(30.0f, 20.0f) + shift, Point2f(10.0f, 20.0f) + shift };
    vector<Point2f> shifted_polygon1 = { Point2f(10.0f, 10.0f) + D + shift, Point2f(30.0f, 10.0f) + D + shift, Point2f(30.0f, 20.0f) + D + shift, Point2f(10.0f, 20.0f) + D + shift };
    float L = polygon0[2].x - polygon0[0].x;
    Point2f center(0.0f, 0.0f);
    for (unsigned int i = 0; i < polygon0.size(); i++)
      center += polygon0[i];
    center = center * (1.0f / polygon0.size());
    Score score(0.0f, "", 1.0f);
    vector<Score> scores;
    scores.push_back(score);
    LimbLabel label0(0, center, 0.0f, polygon0, scores, false);
    LimbLabel label1(1, center + D, 0.0f, polygon1, scores, false);
    vector<LimbLabel> labels = {label0, label1};

    LimbLabel shifted_label0(0, center + shift , 0.0f, shifted_polygon0, scores, false);
    LimbLabel shifted_label1(1, center + D + shift, 0.0f, shifted_polygon1, scores, false);
    vector<LimbLabel> shifted_labels = { shifted_label0, shifted_label1 };

    Solvlet S;
    S.setLabels(labels);

    int rows = 200, cols = 300;
    Mat image(rows, cols, CV_8UC3, Scalar(0, 0, 0));
    for (int y = polygon0[0].y; y < polygon0[2].y + 1; y++)
      for(int x = polygon0[0].x; x < polygon0[2].x + 1; x++)
      {
        image.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
        image.at<Vec3b>(y + D.y, x + D.x) = Vec3b(255, 255, 255);
      }
    //imwrite("S_evaluateSolution.jpg", image);

    Mat mask;
    cvtColor(image, mask, CV_RGB2GRAY);

    Frame* frame = new Lockframe();
    frame->setID(0);
    frame->setImage(image);
    frame->setMask(mask);


    // Create actual value and compare
    map<string, float> params;
    params.emplace(pair<string, float>("maxFrameHeight", 0.0f));// image.rows));
    double X = S.evaluateSolution(frame, params); // Ideal labels

    EXPECT_EQ(1.0f, X);

    S.setLabels(shifted_labels);
    X = S.evaluateSolution(frame, params); // Shifted labels

    float error = 0.06f;
    EXPECT_NEAR((L-dx)/(L+dx), X, error);

    params.emplace(pair<string, float>("badLabelThresh", 0.99f));
    X = S.evaluateSolution(frame, params); // Shifted labels and higth "badLabelThresh"

    EXPECT_NEAR((L - dx) / (L + dx) -1.0f, X, error);

  }

}
