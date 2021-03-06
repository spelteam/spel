#include "predef.hpp"

#include <gtest/gtest.h>
#include "tlpssolver.hpp"
#include "tlpssolver_old.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "limbLabel.hpp"
#include "bodyJoint.hpp"
#include "bodyPart.hpp"
#include "skeleton.hpp"
#include "spelHelper.hpp"
#include "TestsFunctions.hpp"
#include "imagesimilaritymatrix.hpp"

#include <iostream>

using namespace std;
using namespace cv;

namespace SPEL
{
  TEST(tlpssolverTests, TLPSSolver)
  {
    TLPSSolver solver;
    EXPECT_EQ(0, solver.getId());
    EXPECT_EQ("TLPS", solver.getName());
  }

  TEST(tlpssolverTests, DISABLED_solve_0)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/nskpsolverTestData/", "skier.xml");
    vector<Frame*> Frames = project.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();
    Sequence sequence(0, "colorHistDetector", Frames);

    // Run "solve"
    TLPSSolver solver;
    std::vector<Solvlet> Solves;
    Solves = solver.solve(sequence);

    // Compare
    float AcceptableJointLinearError = 5; // pixels
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    Frames.clear();
  }

  TEST(tlpssolverTests, DISABLED_solve_1)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/nskpsolverTestData/", "skier.xml");
    vector<Frame*> Frames = project.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();
    Sequence sequence(0, "colorHistDetector", Frames);

    // Run "solve"
    std::map<std::string, float>  params;
    TLPSSolver solver;
    std::vector<Solvlet> Solves;
    Solves = solver.solve(sequence, params, Solves);

    // Compute expected value and compare
    for (unsigned int i = 1; i < Solves.size(); i++)
      EXPECT_LE(Solves[i - 1].getFrameID(), Solves[i].getFrameID());

    // Compare
    float AcceptableJointLinearError = 5; // pixels
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    Frames.clear();
  }

  TEST(tlpssolverTests, DISABLED_solve_2)
  {
    //Load the input data
    std::map<std::string, float>  params;
    TestProjectLoader project("speltests_TestData/nskpsolverTestData/", "skier.xml");
    vector<Frame*> Frames = project.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();
    Sequence sequence(0, "colorHistDetector", Frames);

    // Run "solve"
    TLPSSolver solver;
    std::vector<Solvlet> Solves;
    Solves = solver.solve(sequence, params);

    // Compare
    float AcceptableJointLinearError = 5; // pixels
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    Frames.clear();
  }

  TEST(tlpssolverTests, DISABLED_solveGlobal)
  { 
    //Load the input data
    /*std::map<std::string, float>  params;
    TestProjectLoader project("speltests_TestData/nskpsolverTestData/", "trijumpSD_13-22.xml");
    vector<Frame*> Frames = project.getFrames();
    Sequence sequence(0, "colorHistDetector", Frames);*/
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> Frames = project.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();
    
    Frames[1]->setSkeleton(Patterns[1]->getSkeleton()); //interoilation
    /*
    // Creating the joints 3D location
    for (int i = 0; i < Frames.size(); i++)
    {
      Skeleton temp = Frames[i]->getSkeleton();
      tree<BodyJoint> PartJoints = temp.getJointTree();
      vector<float> z = { 2.0f, 2.0f, 1.0f, 3.0f, 2.0f, 0.0f, 4.0f, 0.0f, 3.0f, 2.0f, 0.0f, 4.0f, 1.0f, 3.0f, 0.0f, 4.0f, 1.0f, 3.0f };
      for (tree<BodyJoint>::iterator i = PartJoints.begin(); i != PartJoints.end(); i++)
      {
        int id = i->getLimbID();
        Point2f P = i->getImageLocation();
        i->setSpaceLocation(Point3f(P.x, P.y, z[id]));
      }
      temp.setJointTree(PartJoints);
      Frames[i]->setSkeleton(temp);
    }*/

    Sequence sequence(0, "", Frames);

    // Set Solver parameters
    std::map<std::string, float>  params;
    params.emplace("debugLevel", 1);
    params.emplace("useCSdet", 1.0f); //determine if ColHist detector is used and with what coefficient
    params.emplace("useHoGdet", 0.0f); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useSURFdet", 0.0f); //determine whether SURF detector is used and with what coefficient
    params.emplace("temporalWindowSize", 0.0f);
    params.emplace("maxPartCandidates", 4.0f);

    params.emplace("baseRotationRange", 40.0f); //search angle range of +/- 60 degrees
    float baseRotationRange = params.at("baseRotationRange");
    params.emplace("baseRotationStep", baseRotationRange / 4.0f); //search with angle step of 10 degrees

    params.emplace("baseSearchRadius", Frames[0]->getImageSize().height / 30.0f); //search a radius of 100 pixels
    int baseSearchRadius = params.at("baseSearchRadius");
    params.emplace("baseSearchStep", baseSearchRadius / 10.0f); //search in a grid every 10 pixels

    params.emplace("partShiftCoeff", 1.5f); //search radius multiplier of distance between part in current and prev frames
    params.emplace("partRotationCoeff", 1.5f); //rotation radius multiplier of distance between part in current and prev frames

    params.emplace("scoreIndex", 0); //solver sensitivity parameters
    params.emplace("imageCoeff", 1.0f); //set solver detector infromation sensitivity
    params.emplace("jointCoeff", 0.5f); //set solver body part connectivity sensitivity
    params.emplace("jointLeeway", 0.05f); //set solver lenience for body part disconnectedness, as a percentage of part length
    params.emplace("tempCoeff", 0.1f); //set the temporal link coefficient
    params.emplace("tlpsLockframeThreshold", 0.52f); //set up the lockframe accept threshold by mask coverage

    params.emplace("partDepthRotationCoeff", 1.0f);
    sequence.estimateUniformScale(params);

    // Run "solveGlobal"
    TLPSSolver solver;
    std::vector<Solvlet> Solves;
    Solves = solver.solveGlobal(sequence, params);

    // Compare
    float AcceptableJointLinearError = 5; // pixels
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    Frames.clear();    
  }

  TEST(tlpssolverTests, DISABLED_solveWindowed)
  {
    // "TLPSSolver::solveWindowed" - empty function
  }

  vector<Point2f> shiftPolygon_(vector<Point2f> polygon, float dx, float dy)
  {
    vector<Point2f> X = polygon;
    for (unsigned int i = 0; i < polygon.size(); i++)
      X[i] += Point2f(dx, dy);
    return X;
  }

 TEST(tlpssolverTests, evaluateSolution)
  {
    int id = 0;
    Point2f center = Point2f(10, 10);
    float angle = 0;
    bool isOccluded = false;
    float score1Value = 0.1f, score2Value = 0.3f;
    float scoreCoeff = 1.0f;
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    //float LimbLength = polygon[1].y - polygon[0].y;
    float LimbWidth = polygon[2].x - polygon[1].x;

    vector <Score> scores;
    Score score1(score1Value, "", scoreCoeff);
    Score score2(score2Value, "", scoreCoeff);
    scores.push_back(score1);
    scores.push_back(score2);

    //Create labels
    float dx = 60.0f;
    LimbLabel label1(id, center, angle, polygon, scores, isOccluded);
    LimbLabel label2(id + 1, center, angle, shiftPolygon_(polygon, dx, 0.0f), scores, isOccluded);

    //Create labels vector
    vector<LimbLabel> labels;
    labels.push_back(label1);
    labels.push_back(label2);

    //Create mask
    int rows = 100, cols = 120;
    Mat mask = Mat(Size(cols, rows), CV_8UC1, Scalar(0));

    for (int i = 0; i < rows; i++)
      for (int k = 0; k < cols; k++)
        if (label1.containsPoint(Point2f(float(k), float(i))) || label2.containsPoint(Point2f(float(k), float(i))))
          mask.at<uchar>(i, k) = 255;

    imwrite("mask.jpg", mask);

    //Create frame
    Lockframe* frame = new Lockframe();
    frame->setMask(mask);

    //Tesing function "evaluateSolution"
    map<string, float> params;
    params.emplace("maxFrameHeight", 100);
    TLPSSolver S;

    //All labels pixels in mask
    float ActualValue = S.evaluateSolution(frame, labels, params);
    float ExpectedValue = 1;
    EXPECT_EQ(ExpectedValue, ActualValue);
    cout << ExpectedValue << " ~ " << ActualValue << endl << endl;

    //30% of "labels[1]" not in mask
    float e = 0.3f; // Relative shift
    LimbLabel label3(id + 1, center, angle, shiftPolygon_(polygon, dx - LimbWidth*e, 0), scores, isOccluded);
    labels[1] = label3; // Replace the label

    ActualValue = S.evaluateSolution(frame, labels, params);
    ExpectedValue = (2 - e) / (2 + e); // (2 - e) / ( 2 - e + 2*e);
    float epsilon = 0.04f;
    EXPECT_LE(abs(ActualValue - ExpectedValue), epsilon);
    cout << ExpectedValue << " ~ " << ActualValue << endl << endl;

    //90% of "labels[1]" not in mask
    //"labels[1]" is badly lokalised
    e = 0.9f; // Relative shift of "label[1]"
    LimbLabel label4(id + 1, center, angle, shiftPolygon_(polygon, dx - LimbWidth*e, 0), scores, isOccluded);
    labels[1] = label4; // Replace the label

    ActualValue = S.evaluateSolution(frame, labels, params);
    ExpectedValue = (2 - e) / (2 + e);
    ExpectedValue = ExpectedValue - 1;
    epsilon = 0.04f;
    EXPECT_LE(abs(ActualValue - ExpectedValue), epsilon);
    cout << ExpectedValue << " ~ " << ActualValue << endl;
  }

 TEST(tlpssolverTests, findFrameIndexById)
 {
    TLPSSolver S;
    vector<Frame*> frames;
    for (int id = 0; id < 10; id++)
    {
      frames.push_back(new Lockframe());
      frames[id]->setID(id);
    }
    uint32_t id = 6;
    EXPECT_EQ(id, S.findFrameIndexById(id, frames));   

    // Clear
    for (unsigned int i = 0; i < frames.size(); i++)
      delete frames[i];
    frames.clear();
 }

 TEST(tlpssolverTests, ScoreCostAndJointCost)
 {
   string hogName = "18500";
   string csName = "4409412";
   string surfName = "21316";

   int id = 0;
   float max = 1.0f;
   float dx = 5; // labels linear shift
   float squareDistance = pow(dx, 2.0f); // square distance between normal and shifted label

   //Create the part joints
   Point2f p0(10, 2), p1(10, 18);
   float LimbLength = p1.y - p0.y;
   BodyJoint* j0 = new BodyJoint(id, "", p0, Point3f(p0.x, p0.y, 0), false);
   BodyJoint* j1 = new BodyJoint(id + 1, "", p1, Point3f(p1.x, p0.y, 1), false);

   //Create the joints tree
   tree<BodyJoint> jointsTree;
   tree<BodyJoint>::iterator j = jointsTree.begin();
   j = jointsTree.insert(j, *j0);
   j = jointsTree.insert(j, *j1);

   // Create body part
   Point2f center = 0.5f*(p0 + p1);
   float LWRatio = 0.3f;
   vector<Point2f> polygon = BuildPartRect(j0, j1, LWRatio).asVector();
   bool isOccluded = false;
   BodyPart bodyPart(id, "", j0->getLimbID(), j1->getLimbID(), false, LimbLength);
   tree<BodyPart> partsTree;
   tree<BodyPart>::iterator p = partsTree.begin();
   p = partsTree.insert(p, bodyPart);

   // Create the skeleton
   Skeleton skeleton;
   skeleton.setJointTree(jointsTree);
   skeleton.setPartTree(partsTree);

   // Create frame
   Frame* frame = new Lockframe;
   frame->setSkeleton(skeleton);

   // Create labels
   float score1Value = 0.1f, score2Value = 0.3f;
   float scoreCoeff = 1.0f;
   Score score1(score1Value, csName, scoreCoeff);
   Score score2(score2Value, csName, scoreCoeff);
   vector <Score> scores;
   scores.push_back(score1);
   scores.push_back(score2);
   float angle = 0;

   LimbLabel label1(id, center, angle, polygon, scores, isOccluded);
   LimbLabel label2;
   LimbLabel label3( id, center + Point2f(dx, 0), angle, shiftPolygon_(polygon, dx, 0), scores, isOccluded);

   // Set parameters
   map<string, float> params;
   params.emplace("imageCoeff", 1.0f);
   params.emplace("useCSdet", 1.0f);
   params.emplace("useHoGdet", 0.0f);
   params.emplace("useSURFdet", 0.0f);
   TLPSSolver S;

   // Set acceptable error for "EXPECT_NEAR"
   float error = 1.0e-4f;

   //Testing function "computeScoreCost"
   float expected_scoreCost = score1Value + score2Value;
   EXPECT_EQ(expected_scoreCost, S.computeScoreCost(label1, params)); // scores = {score1Value, score2Value}
   EXPECT_EQ(1.0f, S.computeScoreCost(label2, params));	// scores is empty
     
   //Testing function "computeJointCost"
   EXPECT_EQ(0.0f, S.computeJointCost(label1, label1, params, false));
   EXPECT_EQ(LimbLength, S.computeJointCost(label1, label1, params, true));

   //Testing function "computeNormJointCost"
   params.emplace("jointCoeff", 1.0f);
   EXPECT_EQ(0.0f, S.computeNormJointCost(label1, label1, params, max, false));
   EXPECT_EQ(LimbLength, S.computeNormJointCost(label1, label1, params, max, true));

   //Testing function "computePriorCost"
   EXPECT_NEAR(0.0f, S.computePriorCost(label1, bodyPart, skeleton, params), error);

   //Testing function "computeNormPriorCost"
   EXPECT_EQ(0.0f, S.computeNormPriorCost(label1, bodyPart, skeleton, params, max));
   params.emplace("priorCoeff", 1.0f);
   EXPECT_NEAR(2.0f*squareDistance, S.computeNormPriorCost(label3, bodyPart, skeleton, params, max), error);

   //Testing function "computePastTempCost"
   EXPECT_EQ(0.0f, S.computePastTempCost(label1, label1, params));
   EXPECT_EQ(2.0f*squareDistance, S.computePastTempCost(label1, label3, params));

   //Testing function "computeNormPastTempCost"
   EXPECT_EQ(0.0f, S.computeNormPastTempCost(label1, label1, params, max));
   params.emplace("tempCoeff", 1.0f);
   EXPECT_EQ(2.0f*squareDistance, S.computeNormPastTempCost(label1, label3, params, max));

   //Testing function "computeFutureTempCost"
   EXPECT_EQ(0.0f, S.computeFutureTempCost(label1, label1, params));
   EXPECT_EQ(2.0f*squareDistance, S.computeFutureTempCost(label1, label3, params));

   //Testing function "computeNormFutureTempCost"
   float x = 2.0f;
   EXPECT_EQ(0.0f, S.computeNormFutureTempCost(label1, label1, params, x));
   EXPECT_EQ(squareDistance, S.computeNormFutureTempCost(label1, label3, params, x));

   //Testing function "computeAnchorCost"
   EXPECT_EQ(0.0f, S.computeAnchorCost(label1, frame, params));
   EXPECT_EQ(4.0f*squareDistance, S.computeAnchorCost(label3, frame, params));

   //Testing function "computeNormAnchorCost"
   EXPECT_EQ(0.0f, S.computeNormAnchorCost(label1, frame, params, max));
   EXPECT_EQ(4.0f*squareDistance, S.computeNormAnchorCost(label3, frame, params, max));

   //Clear
   delete frame;
 }

 TEST(tlpssolver_Tests, slice_)
 {
   //Load the input data
    TestProjectLoader project("speltests_TestData/nskpsolverTestData/", "trijumpSD_13-22.xml");
    vector<Frame*> Frames = project.getFrames();

   //Create actual value
   TLPSSolver S;
   vector<vector<Frame*>> Actual_Slice = S.slice(Frames);

   //Checking the result
   int ExpectedSliceSize = keyFramesCount(Frames) - 1;
   ASSERT_EQ(Actual_Slice.size(), ExpectedSliceSize);
   int SliceFramesCount = 0;
   for (unsigned int i = 0; i < Actual_Slice.size(); i++)
   {
     SliceFramesCount += Actual_Slice[i].size();
     EXPECT_GT(Actual_Slice[i].size(), 2);
     bool KeyframeAtBegin = false;
     bool KeyframeAtEnd = false;
     if((Actual_Slice[i][0]->getFrametype() == KEYFRAME) || (Actual_Slice[i][0]->getFrametype() == LOCKFRAME))
       KeyframeAtBegin = true;
     int n = Actual_Slice[i].size() - 1;
     if ((Actual_Slice[i][n]->getFrametype() == KEYFRAME) || (Actual_Slice[i][n]->getFrametype() == LOCKFRAME))
       KeyframeAtEnd = true;
     EXPECT_TRUE(KeyframeAtBegin);
     EXPECT_TRUE(KeyframeAtEnd);
   }
   int ExpectedFramesCount = Frames.size() + keyFramesCount(Frames) - 2;
   EXPECT_EQ(ExpectedFramesCount, SliceFramesCount);

   // Clear
   //project.TestProjectLoader::~TestProjectLoader();
   Frames.clear();
   for (unsigned int i = 0; i < Actual_Slice.size(); i++)
     Actual_Slice[i].clear();
   Actual_Slice.clear();
 }

  TEST(tlpssolverTests_old, solve)
  { 
    //Load the input data
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> Frames = project.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();
    
    //Frames[1]->setSkeleton(Patterns[1]->getSkeleton()); //interoilation

    // Set Solver parameters
    std::map<std::string, float>  params;
    //global settings
    params.emplace("debugLevel", 0);
    params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
    params.emplace("jointCoeff", 0.6); //set solver body part connectivity sensitivity
    params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity
    params.emplace("tempCoeff", 0.1);
    //detectors settings
    params.emplace("uniqueLocationCandidates", 360); //unique location candidates count limit 
    //params.emplace("maxFrameHeight", 288); //scale to 288p - same size as trijump video seq, for detection
    params.emplace("maxFrameHeight", Frames[0]->getMask().size().height); // disable image scaling
    params.emplace("grayImages", 1); // use grayscale images for HoG?
    //solver settings
    params.emplace("useCSdet", 1.0f); //determine if ColHist detector is used and with what coefficient
    params.emplace("useHoGdet", 1.0f); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useSURFdet", 0.0f); //determine whether SURF detector is used and with what coefficient

    params.emplace("badLabelThresh", 0.45); //set bad label threshold, which will force solution discard at 0.45
    params.emplace("partDepthRotationCoeff", 1.25); //search radius increase for each depth level in the part tree

    params.emplace("anchorBindDistance", 0); //restrict search regions if within bind distance of existing keyframe or lockframe (like a temporal link
    params.emplace("anchorBindCoeff", 0.3); //multiplier for narrowing the search range if close to an anchor (lockframe/keyframe)

    params.emplace("bindToLockframes", 0); //should binds be also used on lockframes?
    params.emplace("maxPartCandidates", 0.1); // set 1.0 for using the max number of part candidates to allow into the solver

    params.emplace("tlpsLockframeThreshold", 0.0); // 0.0 avoiding the rejection of some TLPS solutions

    //Create Sequence
    Sequence sequence(0, "test", Frames);
    sequence.computeInterpolation(params);
    sequence.estimateUniformScale(params);

    // Run "solveGlobal"
    TLPSSolver_old solver;
    std::vector<Solvlet> Solves;	
    Solves = solver.solve(sequence, params);
    cout << "Solves.size() = " << Solves.size() << endl;

    // Compare
    float AcceptableJointLinearError = 5; // pixels
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    Frames.clear();    
  }

}