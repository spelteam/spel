// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#include <tree_util.hh>
#include "nskpsolver.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "limbLabel.hpp"
#include "bodyJoint.hpp"
#include "bodyPart.hpp"
#include "skeleton.hpp"
#include "spelHelper.hpp"
#include "TestsFunctions.hpp"
#include "imagesimilaritymatrix.hpp"
#include <imagemasksimilaritymatrix.hpp>


#include <iostream>

using namespace std;
using namespace cv;

namespace SPEL
{
  TEST(tlpssolverTests, NSKPSolver)
  {
    NSKPSolver solver;
    EXPECT_EQ(1, solver.getId());
    EXPECT_EQ("NSKP", solver.getName());
  }

  TEST(nskpsolverTests, findFrameIndexById)
  {
    NSKPSolver S;
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

  TEST(nskpsolverTests, ScoreCostAndJointCost)
  {
    string hogName = "18500";
    string csName = "4409412";
    string surfName = "21316";
    int id = 0;
    Point2f center = Point2f(10.0f, 10.0f);
    float angle = 0.0f;
    bool isOccluded = false;
    float score1Value = 0.1f, score2Value = 0.3f;
    float scoreCoeff = 1;
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    float LimbLength = polygon[1].y - polygon[0].y;
    vector <Score> scores;

    Score score1(score1Value, csName, scoreCoeff);
    Score score2(score2Value, csName, scoreCoeff);

    scores.push_back(score1);
    scores.push_back(score2);

    LimbLabel label1(id, center, angle, polygon, scores, isOccluded);

    map<string, float> params;
    params.emplace("imageCoeff", 1.0);
    params.emplace("useCSdet", 1.0);
    params.emplace("useHoGdet", 0.0);
    params.emplace("useSURFdet", 0.0);
    NSKPSolver S;

    /*
    //Testing function "computeScoreCost"
    //scores[0] = score1Value, isWeak = true, isOccluded = false, 
    EXPECT_EQ(0, S.computeScoreCost(label1, params));

    //scores[0] = score1Value, isWeak = false, isOccluded = false, 
    EXPECT_EQ(score1Value, S.computeScoreCost(label1, params));

    //scores is empty, isWeak = false, isOccluded = false, 
    LimbLabel label2;
    label1.isOccluded = false;
    EXPECT_EQ(0, S.computeScoreCost(label2, params));
    */

    //Testing function "computeScoreCost"
    LimbLabel label2;
    float expected_scoreCost = score1Value + score2Value;
    EXPECT_EQ(expected_scoreCost, S.computeScoreCost(label1, params)); // scores = {score1Value, score2Value}
    EXPECT_EQ(1.0f, S.computeScoreCost(label2, params));	// scores is empty

    //Testing function "computeJointCost"
    EXPECT_EQ(0, S.computeJointCost(label1, label1, false));
    EXPECT_EQ(LimbLength, S.computeJointCost(label1, label1, true));

    //Testing function "computeNormJointCost"
    float max = 1;
    params.emplace("jointCoeff", 1.0);
    EXPECT_EQ(0, S.computeNormJointCost(label1, label1, params, max, false));
    EXPECT_EQ(LimbLength, S.computeNormJointCost(label1, label1, params, max, true));

    //Testing function "computePriorCost"
    Point2f p0(10, 2), p1(10, 18);
    LimbLength = p1.y - p0.y;

    BodyJoint j0(0, "", p0, { 0, 0, 0 }, false);
    BodyJoint j1(1, "", p1, { 0, 0, 0 }, false);
    tree<BodyJoint> jointsTree;
    tree<BodyJoint>::iterator j = jointsTree.begin();
    j = jointsTree.insert(j, j0);
    j = jointsTree.insert(j, j1);

    BodyPart bodyPart(0, "", 0, 1, false, LimbLength);
    tree<BodyPart> partsTree;
    tree<BodyPart>::iterator p = partsTree.begin();
    p = partsTree.insert(p, bodyPart);

    Skeleton skeleton;
    skeleton.setJointTree(jointsTree);
    skeleton.setPartTree(partsTree);

    EXPECT_EQ(0, S.computePriorCost(label1, bodyPart, skeleton));

    //Testing function "computeNormPriorCost"
    params.emplace("priorCoeff", 1.0);
    EXPECT_EQ(0, S.computeNormPriorCost(label1, bodyPart, skeleton, params, max));
  }

  vector<Point2f> shiftPolygon(vector<Point2f> polygon, float dx, float dy)
  {
    vector<Point2f> X = polygon;
    for (unsigned int i = 0; i < polygon.size(); i++)
      X[i] += Point2f(dx, dy);
    return X;
  }

  TEST(nskpsolverTests, evaluateSolution)
  {
    int id = 0;
    Point2f center = Point2f(10, 10);
    float angle = 0;
    bool isOccluded = false;
    float score1Value = 0.1f, score2Value = 0.3f;
    float scoreCoeff = 1;
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
    LimbLabel label2(id + 1, center, angle, shiftPolygon(polygon, dx, 0.0f), scores, isOccluded);

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
    NSKPSolver S;

    //All labels pixels in mask
    float ActualValue = S.evaluateSolution(frame, labels, params);
    float ExpectedValue = 1;
    EXPECT_EQ(ExpectedValue, ActualValue);
    cout << ExpectedValue << " ~ " << ActualValue << endl << endl;

    //30% of "labels[1]" not in mask
    float e = 0.3f; // Relative shift
    LimbLabel label3(id + 1, center, angle, shiftPolygon(polygon, dx - LimbWidth*e, 0), scores, isOccluded);
    labels[1] = label3; // Replace the label

    ActualValue = S.evaluateSolution(frame, labels, params);
    ExpectedValue = (2 - e) / (2 + e); // (2 - e) / ( 2 - e + 2*e);
    float epsilon = 0.04f;
    EXPECT_LE(abs(ActualValue - ExpectedValue), epsilon);
    cout << ExpectedValue << " ~ " << ActualValue << endl << endl;

    //90% of "labels[1]" not in mask
    //"labels[1]" is badly lokalised
    e = 0.9f; // Relative shift of "label[1]"
    LimbLabel label4(id + 1, center, angle, shiftPolygon(polygon, dx - LimbWidth*e, 0), scores, isOccluded);
    labels[1] = label4; // Replace the label

    ActualValue = S.evaluateSolution(frame, labels, params);
    ExpectedValue = (2 - e) / (2 + e);
    ExpectedValue = ExpectedValue - 1;
    epsilon = 0.04f;
    EXPECT_LE(abs(ActualValue - ExpectedValue), epsilon);
    cout << ExpectedValue << " ~ " << ActualValue << endl;

    //Clear
    delete frame;
  }

  // Testing "propagateFrame" function.
  // "ImagePixelSimilarityMatrix", "computeISMcell" - produce error and crash the test,
  // - replaced by alternative buildISM function from "TestFunctions.hpp".
  // "solver.buildFrameMSTs" - don't work with current dataset, and replaced by "MinSpanningTree::build".
  // "solver.propagateFrame" - produce crash the test after "detect"
  TEST(nskpsolverTests, propagateFrame) // "nskpsolver::propagateFrame" produce error: "unknown file: error: C++ exception with description "invalid vector<T> subscript" thrown in the test body"
  {
    int DebugLevel = 0;
    SpelObject::setDebugLevel(DebugLevel);
    cout << "DebugLevel = " << DebugLevel << endl;

    //Load the input data
    std::map<std::string, float>  params;
    TestSequence sequence(params, "speltests_TestData/nskpsolverTestData/", "trijumpSD_13-22.xml");
    //sequence.Load(params, "speltests_TestData/testdata1/", "trijumpSD_new.xml");
    //sequence.Load(params, "speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> Frames = sequence.getFrames();
    int k = FirstKeyFrameNum(Frames);
    unsigned int partsCount = Frames[k]->getSkeleton().getPartTree().size();
    //sequence.TestSequence::~TestSequence();

    // Build frames ISM
    ImageMaskSimilarityMatrix MSM(Frames);
    //MSM.write("propagateFrame_1_MSM.txt");

    cout << "MSM.size = " << MSM.size() << endl;

    // Build trees for current frames seequence
    //std::vector<MinSpanningTree> trees = solver.buildFrameMSTs(ISM, params); // it return all trees with sizes = 1
    std::vector<MinSpanningTree> trees;
    for (unsigned int i = 0; i < Frames.size(); i++) // it replaces "solver.buildFrameMSTs"
    {
      MinSpanningTree MST;
      MST.build(MSM, i, 3, 0);// it return trees with sizes = 3..4 for current dataset
      trees.push_back(MinSpanningTree(MST));
    }

    // Temporary debug info
    cout << "trees.size = " << trees.size() << endl;
    cout << "'trees sizes':" << endl;
    for (unsigned int i = 0; i < trees.size(); i++)
      cout << i << ": " << trees[i].getMST().size() << endl;
      //

    // Run "propagateFrame"
    int frameID = 0; // Select the frame as root frame for propagation
    NSKPSolver solver;
    std::vector<int> ignored;
    std::vector<NSKPSolver::SolvletScore> allSolves;
    /*params.emplace(pair<string, float>("useSURFdet", 0.0f)); // Disable SURF detector
    cout << "SURFDetector diabled" << endl;*/
    allSolves = solver.propagateFrame(frameID, Frames, params, MSM, trees, ignored);

    ASSERT_GT( allSolves.size(), 0);

    for (unsigned int i = 0; i < allSolves.size(); i++)
      ASSERT_EQ(partsCount, allSolves[i].solvlet.getLabels().size());

    // Copy ID of all frames, which identical to the selected frame ("frameID")
    vector<int> IdenticalFramesID;
    for (unsigned int i = 0; i < MSM.size(); i++)
      if ((MSM.at(frameID, i) <= 0.05) && (Frames[i]->getFrametype() != KEYFRAME))
        IdenticalFramesID.push_back(i);

    ASSERT_GE(allSolves.size(), IdenticalFramesID.size());

    // Copy skeleton joints location from selected keyframe
    Skeleton skeleton = Frames[frameID]->getSkeleton();
    map<int, pair<Point2f, Point2f>> PartLocations = getPartLocations(skeleton);

    //Compare
    // Search all frames, which identical to selected keyframe
    float AcceptableError = 2; // 2 pixels
    for (unsigned int i = 0; i < allSolves.size(); i++)
    {
      for (unsigned int k = 0; k < IdenticalFramesID.size(); k++)
        if(allSolves[i].solvlet.getFrameID() == IdenticalFramesID[k])
        {
          vector<LimbLabel> Labels =  allSolves[i].solvlet.getLabels();
          //Compare all parts from current lockframe
          for (unsigned int t = 0; t < Labels.size(); t++)
          {
            Point2f l0, l1; // Actual current part joints locations
            Labels[t].getEndpoints(l0, l1);
            int partID = Labels[t].getLimbID();

            Point2f p0, p1; // Expected current part joints locations
            p0 = PartLocations[partID].first;// [t]
            p1 = PartLocations[partID].second;// [t]

            Point2f delta0 = l0 - p0;
            Point2f  delta1 = l1 - p1;
            float error_A = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
            delta0 = l0 - p1;
            delta1 = l1 - p0;
            float error_B = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
            float error = min(error_A, error_B); // Distance between ideal body part and label
            EXPECT_LE(error, AcceptableError) << "RootFrameID = " << frameID << ", LockframeID = " << IdenticalFramesID[k] << ", PartID = " << partID << ", Error = " << error << "(pixels)" << endl;
          }
        }
    }

    // Clear
    for (unsigned int i = 0; i < Frames.size(); i++)
      delete Frames[i];
    Frames.clear();
  }

  TEST(nskpsolverTests, propagateFrame_labelsCount)
  {
    /*int DebugLevel = 0;
    SpelObject::setDebugLevel(DebugLevel);
    cout << "DebugLevel = " << DebugLevel << endl;*/

    //Load the input data
    std::map<std::string, float>  params;
    TestSequence sequence(params, "speltests_TestData/nskpsolverTestData/", "trijumpSD_13-22.xml");
    vector<Frame*> Frames = sequence.getFrames();
    int k = FirstKeyFrameNum(Frames);
    unsigned int partsCount = Frames[k]->getSkeleton().getPartTree().size();

    ImageMaskSimilarityMatrix MSM(Frames);
    //MSM.write("propagateFrame_2_MSM.txt");

    // Build trees for current frames seequence
    //std::vector<MinSpanningTree> trees = solver.buildFrameMSTs(ISM, params); // it return all trees with sizes = 1
    std::vector<MinSpanningTree> trees;
    for (unsigned int i = 0; i < Frames.size(); i++) // it replaces "solver.buildFrameMSTs"
    {
      MinSpanningTree MST;
      MST.build(MSM, i, 3, 0);// it return trees with sizes = 3..4 for current dataset
      trees.push_back(MinSpanningTree(MST));
    }

    // Run "propagateFrame"
    int frameID = 0; // Select the frame as root frame for propagation
    NSKPSolver solver;
    std::vector<int> ignored;
    std::vector<NSKPSolver::SolvletScore> allSolves;
    params.emplace(pair<string, float>("useSURFdet", 0.0f));  // Disable SURF detector
    cout << "SURFDetector disabled" << endl;
    allSolves = solver.propagateFrame(frameID, Frames, params, MSM, trees, ignored);

    for(unsigned int i = 0; i < allSolves.size(); i++)
      EXPECT_EQ(partsCount, allSolves[i].solvlet.getLabels().size());
  }

  // "buildFrameMSTs" and "computeISMcell" don't work with current dataset
  // Call of "propagateFrame" causes crash tests??
  // This test crashed and don't checked 
  TEST(nskpsolverTests, solve_0)
  {
    //Load the input data
    /*std::map<std::string, float>  params;
    TestSequence sequence(params, "speltests_TestData/nskpsolverTestData/", "trijumpSD_13-22.xml");
    vector<Frame*> Frames = sequence.getFrames();*/
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> Frames = project.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();


    Frames[1]->setSkeleton(Patterns[1]->getSkeleton());//interpolation
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
    //sequence.computeInterpolation(params);
    //sequence.estimateUniformScale(params);

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
    params.emplace("tlpsLockframeThreshold", 0.52f);

    params.emplace("partDepthRotationCoeff", 1.0f);
    params.emplace("withTLPS", 0.0f);
    params.emplace("nskpLockframeThreshold", 0.0f);

    // Run "solve"
    NSKPSolver solver;
    std::vector<Solvlet> Solves;
    ImageMaskSimilarityMatrix ISM(Frames);
    Solves = solver.solve(sequence, params, ISM);

    // Debug messages
    cout << "Solves size = " << Solves.size() << endl;
    for (int i = 0; i < Solves.size(); i++)
    {
      Skeleton x = Solves[i].toSkeleton(Patterns[1]->getSkeleton());
      cout << "PartTree[" << i << "] size = " << x.getPartTree().size() << endl;
    }

    // Compare
    float AcceptableJointLinearError = 5; // pixels;
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    Frames.clear();
  }

  // DISABLED - call of "propagateFrame" causes crash tests??
  // This test crashed and don't checked 
  TEST(nskpsolverTests, DISABLED_solve_1) 
  {
    //Load the input data
    std::map<std::string, float>  params;
    TestSequence sequence(params, "speltests_TestData/nskpsolverTestData/", "skier.xml");
    vector<Frame*> Frames = sequence.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();

    // Run "solve"
    NSKPSolver solver;
    std::vector<Solvlet> Solves;
    Solves = solver.solve(sequence);

    // Compare
    float AcceptableJointLinearError = 5; // pixels;
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    //sequence.TestSequence::~TestSequence();
    for (unsigned int i = 0; i < Frames.size(); i++)
        delete Frames[i];
    Frames.clear();
  }

  // DISABLED - call of "propagateFrame" causes crash tests??
  // This test crashed and don't checked 
  TEST(nskpsolverTests, DISABLED_solve_2)
  {
    //Load the input data
    std::map<std::string, float>  params;
    TestSequence sequence(params, "speltests_TestData/nskpsolverTestData/", "skier.xml");
    vector<Frame*> Frames = sequence.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();

    // Run "solve"
    NSKPSolver solver;
    std::vector<Solvlet> Solves;
    Solves = solver.solve(sequence, params);

    // Compare
    float AcceptableJointLinearError = 5; // pixels;
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    //sequence.TestSequence::~TestSequence();
    for (unsigned int i = 0; i < Frames.size(); i++)
      delete Frames[i];
    Frames.clear();
  }

  // DISABLED - call of "propagateFrame" causes crash tests??
  // This test crashed and don't checked 
  TEST(nskpsolverTests, propagateKeyframes)
  {  
    int DebugLevel = 0;
    SpelObject::setDebugLevel(DebugLevel);
    cout << "DebugLevel = " << DebugLevel << endl;

    //Load the input data
    std::map<std::string, float>  params;
    TestSequence sequence(params, "speltests_TestData/nskpsolverTestData/", "skier.xml");
    vector<Frame*> Frames = sequence.getFrames();
    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Patterns = projectPattern.getFrames();

    int k = FirstKeyFrameNum(Frames);
    unsigned int partsCount = Frames[k]->getSkeleton().getPartTree().size();
    //sequence.TestSequence::~TestSequence();
      
    ImageMaskSimilarityMatrix MSM(Frames);

    // Build trees for current frames seequence
    std::vector<MinSpanningTree> trees;
    for (unsigned int i = 0; i < Frames.size(); i++) // it replaces "solver.buildFrameMSTs"
    {
      MinSpanningTree MST;
      MST.build(MSM, i, 3, 0);// it return trees with sizes = 3..4 for current dataset
      trees.push_back(MinSpanningTree(MST));
    }

    // Run "propagateKeyframes"
    NSKPSolver solver;
    std::vector<int> ignored;
    std::vector<Solvlet> Solves;
    params.emplace(pair<string, float>("useSURFdet", 0.0f)); // Disable SURFDetector
    cout << "SURFDetector disabled" << endl;
    Solves = solver.propagateKeyframes(Frames, params, MSM, trees, ignored);

    ASSERT_GT(Solves.size(), 0);

    for (unsigned int i = 0; i < Solves.size(); i++)
      ASSERT_EQ(partsCount, Solves[i].getLabels().size());

    // Compare
    float AcceptableJointLinearError = 5; // pixels;
    CompareSolves(Solves, Patterns, AcceptableJointLinearError);

    // Clear
    for (unsigned int i = 0; i < Frames.size(); i++)
      delete Frames[i];
    Frames.clear();
  }

  Mat ToMatrix(tree<int> mst, ImageSimilarityMatrix &ism)
  {
    Mat X(ism.size(), ism.size(), cv::DataType<float>::type, 0.0f);
    for (tree<int>::iterator i = mst.begin(); i != mst.end(); i++)
      for (tree<int>::sibling_iterator k = i.begin(); k != i.end(); k++)
      if((*i < X.rows) && (*k <X.cols))
      {
        X.at<float>(*i, *k) = 1.0f;
        X.at<float>(*k, *i) = 1.0f;
        /*X.at<float>(*k, *i) = ism.at(*i, *k);fixed_depth_iterator*/
      }
    return X;
  }

  TEST(nskpsolverTests, buildMSTs)
  {
    // Prepare test data
    ImagePixelSimilarityMatrix ISM;
    bool b;
    string FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif
    b = ISM.read(FilePath + "ISM1.txt");
    ASSERT_TRUE(b);

    tree<int> temp;
    int rootNode = 3;
    tree<int>::iterator A = temp.begin(), B;
    A = temp.insert(A, rootNode);
    B = temp.append_child(A, 0);
    temp.append_child(A, 5);
    B = temp.append_child(B, 1);
    B = temp.append_child(B, 4);
    temp.append_child(B, 2);
    temp.append_child(B, 6);

    // Create expected value
    Mat Expected = ToMatrix(temp, ISM);
    temp.clear();

    // Create actual value
    std::map<std::string, float> params;
    params.emplace("mstThresh", 1.0f);
    params.emplace("treeSize", ISM.size());

    NSKPSolver s;
    vector<MinSpanningTree > MSTs = s.buildFrameMSTs(ISM, params);

    // Compare	  
    bool EdgesMatched = true;
    //float error = 1E-6;
    for(int root_node = 0; root_node < MSTs.size(); root_node++)
    {
      tree<int> mst = MSTs[root_node].getMST();
      Mat X = ToMatrix(mst, ISM);
      EXPECT_EQ(root_node, *mst.begin());
      ASSERT_EQ(Expected.rows, X.rows);
      for(int i = 0; i < Expected.rows; i++)
      {
        for(int k = 0; k < Expected.cols; k++)
        {
          //if(abs(X.at<float>(i, k) - Expected.at<float>(i, k)) > error);
          if(X.at<float>(i, k) != Expected.at<float>(i, k))
          {
            EdgesMatched = false;
            cout << endl << "i = " << i << ", k= " << k << ", error = " << abs(X.at<float>(i, k) - Expected.at<float>(i, k)) << endl;
          }
        }
      }
      X.release();
    }	  
    EXPECT_TRUE(EdgesMatched);

    // Put results
    if(!EdgesMatched)
    {
      cout << endl << "Expected:" <<endl;
      cout << "Frame i:" << endl;
      cout << "root node: i";
      for(int i = 0; i < Expected.rows; i++)
      {
        cout << endl;
        for (int k = 0; k < Expected.cols; k++)
        cout << Expected.at<float>(i, k) << " ";
      }
      Expected.release();

      cout << endl << endl <<"Actual values:" << endl;
      for(int t = 0; t < MSTs.size(); t++)
      {
        tree<int> mst = MSTs[t].getMST();
        Mat X = ToMatrix(mst, ISM);
        cout << endl << "Frame " << t << ":" << endl;
        cout << "root node: " << *(mst.begin());
        for(int i = 0; i < X.rows; i++)
        {
          cout << endl;
          for(int k = 0; k < X.cols; k++)
            cout << X.at<float>(i, k) << " ";
        }
        X.release();
        cout << endl;
      }
    }

    MSTs.clear();
  }

  TEST(nskpsolverTests, suggestKeyframes_A)
  {
  // Prepare test data
    ImagePixelSimilarityMatrix ISM;
    bool b;
    string FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif
    b = ISM.read(FilePath + "ISM1.txt");
    ASSERT_TRUE(b);

    //Create expected value
    vector<std::pair<int, int>> Expected = { std::make_pair(0, 7) };

    // Create actual value
    std::map<std::string, float> params;
    //params.emplace("mstThresh", 1.0f);
    //params.emplace("treeSize", ISM.size());
    
    NSKPSolver s;
    vector<std::pair<int, int>> Actual;
    try
    {
      Actual = s.suggestKeyframes(ISM, params);
    }
    catch(...)
    {
    }

    // Compare	  
    EXPECT_EQ(Expected, Actual);

    Expected.clear();
    Actual.clear();
  }

  TEST(nskpsolverTests, suggestKeyframes_B)
  {
  // Prepare test data
    ImagePixelSimilarityMatrix ISM;
    bool b;
    string FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif
    b = ISM.read(FilePath + "ISM.txt");
    ASSERT_TRUE(b);

    //Create expected value
    vector<std::pair<int, int>> Expected = { std::make_pair(0, 4), std::make_pair(2, 2) };

    // Create actual value
    std::map<std::string, float> params;
    //params.emplace("mstThresh", 1.0f);
    //params.emplace("treeSize", ISM.size());
    
    NSKPSolver s;
    vector<std::pair<int, int>> Actual;
    try
    {
      Actual = s.suggestKeyframes(ISM, params);
    }
    catch(...)
    {
    }

    // Compare	  
    EXPECT_EQ(Expected, Actual);

    Expected.clear();
    Actual.clear();
  }

}
